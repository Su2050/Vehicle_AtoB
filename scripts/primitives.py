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

def init_primitives():
    """前向积分预推演 30 种运动基元，并将相对坐标增量与相对角度的正余弦值进行缓存"""
    gears = [('F', 0.25), ('R', -0.20)]
    steers = [-1.0, -0.5, 0.0, 0.5, 1.0]
    
    precomp_prim = []
    for g_name, v in gears:
        for s in steers:
            durations = [0.33, 0.50, 0.67]
            if s == 0.0:
                durations.extend([1.0, 1.5])
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

def simulate_path(x0, y0, theta0, actions, precomp_prim):
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
        
    for act in actions:
        # act is (g_name, s, d)
        if act not in prim_map:
            continue
            
        traj = prim_map[act]
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
