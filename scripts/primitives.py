import math

# === 物理与环境常量 ===
DT = 1.0 / 30.0
WHEELBASE = 1.6
MAX_STEER = 0.65
M_PI = math.pi
PI2 = 2.0 * math.pi
ALIGN_GOAL_DYAW = 5.0 * M_PI / 180.0

# === RS 启发函数常量 ===
MIN_TURN_RADIUS = WHEELBASE / math.tan(MAX_STEER)  # ≈ 1.97m

# RS 目标位姿中心点（th=0 表示车头朝 -x 方向，即正对托盘）
RS_GOAL_X = 2.10
RS_GOAL_Y = 0.0
RS_GOAL_TH = 0.0

# === 路径质量惩罚常量 ===
GEAR_CHANGE_PENALTY = 0.1  # seconds
STEER_JUMP_PENALTY  = 0.5  # seconds

# === 两阶段规划参数 ===
# 第一阶段宽松目标：完成 K-turn 返程后离墙有余量（x≥2.2），Stage-2 才容易求解
PREAPPROACH_X_MIN  = 2.45  # 确保离墙足够远，Stage-2 有空间机动
PREAPPROACH_X_MAX  = 8.0   # 工作区右边界（放宽至 8.0m 允许大倒库空间）
PREAPPROACH_Y_MAX  = 0.5   # 横向偏移 ≤ 0.5m
PREAPPROACH_TH_MAX = 0.35  # 朝向偏差 ≤ ±20°

# |y| 或 |θ| 超过此阈值时自动启用两阶段规划
# TWO_STAGE_Y_THRESH 应等于 PREAPPROACH_Y_MAX：超出预定位区才需要 K-turn 预定位
TWO_STAGE_Y_THRESH  = 0.5  # m（= PREAPPROACH_Y_MAX；y∈(0.5,0.8m) 走两阶段而非慢速直接A*）
TWO_STAGE_TH_THRESH = 0.7  # rad ≈ 40°

# Python A* 可规划的最大横向偏移（超出后直接返回 IMPOSSIBLE+说明）
MAX_PLANNABLE_Y = 5.0  # 放开限制（足以覆盖 y=2.47m 等极端侧偏）

# === 多圆碰撞模型参数 ===
# 默认值：长 1.5m × 宽 0.5m → 3 个碰撞圆（半径 0.25m）
VEHICLE_LENGTH = 1.5    # m, 车辆总长（含叉齿）
VEHICLE_WIDTH  = 0.5    # m, 车辆总宽
VEHICLE_HALF_WIDTH = 0.25       # = width / 2
VEHICLE_CHECK_OFFSETS = (0.17, -0.33, -0.83)  # 3 个圆的局部 x 偏移
VEHICLE_MAX_RADIUS = 0.83 + 0.25  # 1.08m（最远圆心距 + 半径）

DEFAULT_PRIMITIVE_PROFILE = "default"
SLALOM_PRIMITIVE_PROFILE = "slalom"
_PRIMITIVE_BANK_CACHE = {}

_BASE_TURN_DURATIONS = (0.33, 0.50, 0.67)
_BASE_STRAIGHT_EXTRA_DURATIONS = (1.0, 1.5)
_SLALOM_STRAIGHT_DURATIONS = (0.17, 0.25, 0.33, 0.50, 0.67, 1.0, 1.5)
_SLALOM_HALF_TURN_DURATIONS = (0.33, 0.50, 0.67, 0.85)
_SLALOM_FULL_TURN_DURATIONS = (0.33, 0.50, 0.67, 0.85, 1.0, 1.2)


def configure_vehicle(length=1.5, width=0.5):
    """
    根据用户提供的长宽参数，自动计算多圆碰撞模型。

    参数:
        length: 车辆总长（含叉齿），单位 m，默认 1.5
        width:  车辆总宽，单位 m，默认 0.5

    算法:
        - half_width = width / 2（每个碰撞圆的半径）
        - span = length - width（最前与最后圆心之间的距离）
        - 圆的数量 = ceil(span / width) + 1（确保相邻圆心距 ≤ width，无缝覆盖）
        - 后方偏移 ≈ span 的 1/6（参考点接近后轴）
    """
    global VEHICLE_LENGTH, VEHICLE_WIDTH, VEHICLE_HALF_WIDTH
    global VEHICLE_CHECK_OFFSETS, VEHICLE_MAX_RADIUS

    VEHICLE_LENGTH = length
    VEHICLE_WIDTH = width
    VEHICLE_HALF_WIDTH = width / 2.0

    span = length - width  # 圆心之间的总跨度
    if span <= 0:
        # 极端情况：长 ≤ 宽，用单个圆
        VEHICLE_CHECK_OFFSETS = (0.0,)
        VEHICLE_MAX_RADIUS = VEHICLE_HALF_WIDTH
        return

    max_gap = width  # = 2 * half_width，保证无缝
    # 加 1e-9 容差防止浮点误差（如 1.2/0.6=2.0000000000000004 → ceil=3）
    n_circles = max(2, math.ceil(span / max_gap - 1e-9) + 1)

    # 后方偏移 ≈ span/6，参考点略偏向车尾
    rear_offset = round(span / 6.0, 2)
    if rear_offset < 0.05:
        rear_offset = 0.05

    if n_circles == 2:
        VEHICLE_CHECK_OFFSETS = (
            round(rear_offset, 2),
            round(-(span - rear_offset), 2),
        )
    else:
        step = span / (n_circles - 1)
        VEHICLE_CHECK_OFFSETS = tuple(
            round(rear_offset - i * step, 2) for i in range(n_circles)
        )

    front_offset = min(VEHICLE_CHECK_OFFSETS)
    VEHICLE_MAX_RADIUS = abs(front_offset) + VEHICLE_HALF_WIDTH

def _primitive_durations_for(profile, steer):
    if profile == DEFAULT_PRIMITIVE_PROFILE:
        durations = list(_BASE_TURN_DURATIONS)
        if steer == 0.0:
            durations.extend(_BASE_STRAIGHT_EXTRA_DURATIONS)
        return durations

    if profile == SLALOM_PRIMITIVE_PROFILE:
        if steer == 0.0:
            return list(_SLALOM_STRAIGHT_DURATIONS)
        if abs(steer) >= 0.99:
            return list(_SLALOM_FULL_TURN_DURATIONS)
        return list(_SLALOM_HALF_TURN_DURATIONS)

    raise ValueError(f"Unknown primitive profile: {profile}")


def init_primitives(profile=DEFAULT_PRIMITIVE_PROFILE):
    """
    前向积分预推演运动基元，并将相对坐标增量与相对角度的正余弦值进行缓存。

    profile="default" 保持现有 34 个基元不变。
    profile="slalom" 追加更长的弧线和更短的直行校正，专门给交错障碍场景使用。
    """
    profile = (profile or DEFAULT_PRIMITIVE_PROFILE).lower()
    gears = [('F', 0.25), ('R', -0.20)]
    steers = [-1.0, -0.5, 0.0, 0.5, 1.0]
    
    precomp_prim = []
    for g_name, v in gears:
        for s in steers:
            durations = _primitive_durations_for(profile, s)
            for d in durations:
                delta = s * MAX_STEER
                # 依题意强制截断纠正浮点飘移 (例 0.33 / 1/30 ≈ 9.9 -> 10)
                N = math.ceil(d / DT - 1e-9)
                
                traj = []
                x, y, th = 0.0, 0.0, 0.0
                for _ in range(N):
                    # 向托盘前进会让 dist_front (x) 减小
                    x -= v * math.cos(th) * DT
                    y -= v * math.sin(th) * DT
                    th += (v / WHEELBASE) * math.tan(delta) * DT
                    
                    # 相对偏航角包装限定至 (-pi, pi]
                    if th > M_PI: th -= PI2
                    elif th <= -M_PI: th += PI2
                    
                    # 同时缓存微小增量的三角函数，后续通过恒等式替代内层庞大的 math.sin() 调用
                    traj.append((x, y, th, math.cos(th), math.sin(th)))
                    
                precomp_prim.append(( (g_name, s, d), N, traj ))
                
    return precomp_prim


def get_cached_primitives(profile=DEFAULT_PRIMITIVE_PROFILE):
    """Return a cached primitive bank for the requested profile."""
    profile = (profile or DEFAULT_PRIMITIVE_PROFILE).lower()
    prims = _PRIMITIVE_BANK_CACHE.get(profile)
    if prims is None:
        prims = init_primitives(profile=profile)
        _PRIMITIVE_BANK_CACHE[profile] = prims
    return prims


def resolve_replay_primitives(actions, preferred_profile=DEFAULT_PRIMITIVE_PROFILE):
    """
    Choose a primitive bank that can exactly replay the provided action list.

    We try the preferred profile first, then fall back to the slalom superset.
    Raise ValueError if no known bank can represent every action.
    """
    actions = list(actions or [])
    profiles = []
    for profile in (preferred_profile, SLALOM_PRIMITIVE_PROFILE):
        profile = (profile or DEFAULT_PRIMITIVE_PROFILE).lower()
        if profile not in profiles:
            profiles.append(profile)

    missing_by_profile = {}
    for profile in profiles:
        prims = get_cached_primitives(profile)
        known = {p[0] for p in prims}
        missing = [act for act in actions if act not in known]
        if not missing:
            return profile, prims
        missing_by_profile[profile] = sorted(set(missing))

    raise ValueError(f"No primitive bank can replay actions; missing={missing_by_profile}")

def _replay_to_end(x0, y0, theta0, acts, precomp_prim):
    """用动作序列计算最终位姿（取每段 primitive 的最后一步）"""
    cx, cy, cth = x0, y0, theta0
    pmap = {p[0]: p[2] for p in precomp_prim}
    for act in (acts or []):
        seg = pmap.get(act)
        if seg is None:
            continue
        cos_t, sin_t = math.cos(cth), math.sin(cth)
        ddx, ddy, ddth = seg[-1][0], seg[-1][1], seg[-1][2]
        nx = cx + ddx * cos_t - ddy * sin_t
        ny = cy + ddx * sin_t + ddy * cos_t
        nth = cth + ddth
        if nth > M_PI:  nth -= PI2
        elif nth <= -M_PI: nth += PI2
        cx, cy, cth = nx, ny, nth
    return cx, cy, cth

def simulate_path(x0, y0, theta0, actions, precomp_prim, final_step_limit=None):
    """
    根据动作序列回放路径，返回完整的轨迹点
    用于可视化
    """
    trajectory = [(x0, y0, theta0)]
    cx, cy, cth = x0, y0, theta0
    
    # 建立动作映射以便快速查找
    prim_map = {}
    for p in precomp_prim:
        # p[0] is (g_name, s, d)
        prim_map[p[0]] = p[2] # p[2] is traj
        
    actions = list(actions or [])
    for idx, act in enumerate(actions):
        # act is (g_name, s, d)
        if act not in prim_map:
            continue

        traj = prim_map[act]
        if final_step_limit is not None and idx == len(actions) - 1:
            traj = traj[:max(0, min(int(final_step_limit), len(traj)))]
        cos_th = math.cos(cth)
        sin_th = math.sin(cth)
        
        for dx, dy, dth, _, _ in traj:
            nx = cx + dx * cos_th - dy * sin_th
            ny = cy + dx * sin_th + dy * cos_th
            nth = cth + dth
            
            if nth > M_PI: nth -= PI2
            elif nth <= -M_PI: nth += PI2
            
            trajectory.append((nx, ny, nth))
            
            # 检查是否到达目标区域（模拟截断）
            if nx <= 2.25 and -0.18 <= ny <= 0.18 and -ALIGN_GOAL_DYAW <= nth <= ALIGN_GOAL_DYAW:
                return trajectory
                
        # 更新当前点为这一段的终点
        cx, cy, cth = trajectory[-1]
        
    return trajectory


def simulate_path_strict(x0, y0, theta0, actions, precomp_prim,
                         stop_at_goal=True, final_step_limit=None):
    """
    Replay a path exactly. Unlike simulate_path(), this fails loudly if any
    action is missing from the supplied primitive bank.
    """
    trajectory = [(x0, y0, theta0)]
    cx, cy, cth = x0, y0, theta0

    prim_map = {p[0]: p[2] for p in precomp_prim}
    missing = [act for act in (actions or []) if act not in prim_map]
    if missing:
        raise KeyError(f"Unknown actions during strict replay: {sorted(set(missing))}")

    actions = list(actions or [])
    for idx, act in enumerate(actions):
        traj = prim_map[act]
        if final_step_limit is not None and idx == len(actions) - 1:
            traj = traj[:max(0, min(int(final_step_limit), len(traj)))]
        cos_th = math.cos(cth)
        sin_th = math.sin(cth)

        for dx, dy, dth, _, _ in traj:
            nx = cx + dx * cos_th - dy * sin_th
            ny = cy + dx * sin_th + dy * cos_th
            nth = cth + dth

            if nth > M_PI:
                nth -= PI2
            elif nth <= -M_PI:
                nth += PI2

            trajectory.append((nx, ny, nth))

            if (stop_at_goal and nx <= 2.25 and -0.18 <= ny <= 0.18
                    and -ALIGN_GOAL_DYAW <= nth <= ALIGN_GOAL_DYAW):
                return trajectory

        cx, cy, cth = trajectory[-1]

    return trajectory
