import sys
import math
import heapq
import time
import rs

# === 物理与环境常量 ===
DT = 1.0 / 30.0
WHEELBASE = 1.6
MAX_STEER = 0.65
M_PI = math.pi
PI2 = 2.0 * math.pi
ALIGN_GOAL_DYAW = 5.0 * M_PI / 180.0

# === RS 启发函数常量 ===
# 最小转弯半径：由轴距和最大舵角决定
MIN_TURN_RADIUS = WHEELBASE / math.tan(MAX_STEER)  # ≈ 1.97m

# RS 目标位姿中心点（th=0 表示车头朝 -x 方向，即正对托盘）
RS_GOAL_X = 2.10
RS_GOAL_Y = 0.0
RS_GOAL_TH = 0.0

# RS 解析终点扩展：离目标的 RS 距离小于此值时，尝试直接用 RS 曲线一杆进洞
RS_EXPANSION_RADIUS = 0.8  # meters

# === 路径质量惩罚常量 ===
GEAR_CHANGE_PENALTY = 0.1  # seconds
STEER_JUMP_PENALTY  = 0.5  # seconds

# === 两阶段规划参数 ===
# 第一阶段宽松目标：完成 K-turn 返程后离墙有余量（x≥2.2），Stage-2 才容易求解
PREAPPROACH_X_MIN  = 2.2   # 确保离墙足够远，Stage-2 启发函数不会过估计
PREAPPROACH_X_MAX  = 8.0   # 工作区右边界（放宽至 8.0m 允许大倒库空间）
PREAPPROACH_Y_MAX  = 0.5   # 横向偏移 ≤ 0.5m
PREAPPROACH_TH_MAX = 0.8   # 朝向偏差 ≤ ±46°

# |y| 或 |θ| 超过此阈值时自动启用两阶段规划
# TWO_STAGE_Y_THRESH 应等于 PREAPPROACH_Y_MAX：超出预定位区才需要 K-turn 预定位
TWO_STAGE_Y_THRESH  = 0.5  # m（= PREAPPROACH_Y_MAX；y∈(0.5,0.8m) 走两阶段而非慢速直接A*）
TWO_STAGE_TH_THRESH = 0.7  # rad ≈ 40°

# Python A* 可规划的最大横向偏移（超出后直接返回 IMPOSSIBLE+说明）
# 原因：|y| > 0.8m 时需要多次 K-turn，Python A* 需要 20s+ 才能求解或直接失败。
# 说明：本规划器仅设计用于"最终对准"阶段（final approach）。
#       全局仓库导航（|y| 大偏移）应由 Isaac Lab 或 ROS Nav2 处理。
MAX_PLANNABLE_Y = 5.0  # 放开限制（足以覆盖 y=2.47m 等极端侧偏）

def init_primitives():
    """前向积分预推演 30 种运动基元，并将相对坐标增量与相对角度的正余弦值进行缓存"""
    gears = [('F', 0.25), ('R', -0.20)]
    steers = [-1.0, -0.5, 0.0, 0.5, 1.0]
    durations = [0.33, 0.50, 0.67]
    
    precomp_prim = []
    for g_name, v in gears:
        for s in steers:
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

def check_collision(nx, ny, nth, sin_nth=None, no_corridor=False):
    """
    检查状态是否合法
    返回: (is_valid, reason)
    reason: 'OK', 'CORRIDOR'

    no_corridor=True 时跳过安全走廊约束。
    """
    # 安全走廊门控 (safe_corridor) —— 可选关闭
    if not no_corridor and nx <= 2.05:
        if sin_nth is None:
            sin_nth = math.sin(nth)
        tip_lat = ny - 1.87 * sin_nth
        sc = (0.15 + (nx - 1.87) * 0.8) if nx > 1.87 else 0.15
        if tip_lat > sc or tip_lat < -sc:
            return False, 'CORRIDOR'

    return True, 'OK'

def _check_traj_collision(traj, no_corridor=False):
    """对一组轨迹点 [(x,y,th),...] 逐点做碰撞检测，全通过返回 True"""
    for x, y, th in traj:
        is_valid, _ = check_collision(x, y, th, no_corridor=no_corridor)
        if not is_valid:
            return False
    return True


def plan_path(x0, y0, theta0, precomp_prim,
              use_rs=False, stats=None, no_corridor=False,
              # 可选目标区覆写（供两阶段规划第一阶段使用）
              _goal_xmin=1.92, _goal_xmax=2.25,
              _goal_yhalf=0.18, _goal_thmax=ALIGN_GOAL_DYAW,
              _step_limit=1079, _expand_limit=150000, _prim_limit=30,
              _rs_expand=True,
              # Stage-1 专用简单启发函数（仅惩罚横向偏移）
              _heuristic_preapproach=False):
    """
    执行 A* 搜索

    参数:
        use_rs       : 若为 True，使用 Reeds-Shepp 距离作为启发函数
        stats        : 可选字典，填充 expanded / elapsed_ms / use_rs / no_corridor 等
        no_corridor  : 若为 True，跳过走廊约束，仅保留硬墙边界
        _goal_x*/_goal_y*/_goal_th* : 目标区参数（两阶段规划 Stage-1 时覆写）
        _step_limit  : 最大 DT 步数（默认 1079）
        _expand_limit: 最大扩展节点数（默认 150000）
        _prim_limit  : 最大基元深度（默认 30）
        _rs_expand   : 是否尝试 RS 解析扩展（Stage-1 时关闭）

    返回: (success, actions, rs_traj)
    """
    t_start = time.perf_counter()
    visited = {}
    
    # 将浮点转为正整数键用于极速哈希去重
    ix0 = int((x0 - 1.9) * 50.0 + 0.5)
    if ix0 < 0: ix0 = 0
    iy0 = int((y0 + 3.1) * 50.0 + 0.5)
    ith0 = int((theta0 + M_PI) * 50.0 + 0.5)
    start_key = (ix0, iy0, ith0, 'N')
    
    visited[start_key] = 0.0
    
    tiebreaker = 0
    # 优先队列 Tuple:
    # (f_score, tiebreaker, cost, x, y, th, prev_gear, prev_s, steps, path_len, path_node)
    q = [(0.0, tiebreaker, 0.0, x0, y0, theta0, 'N', 0.0, 0, 0, None)]
    
    expanded = 0
    
    while q:
        f, _, cost, cx, cy, cth, prev_gear, prev_s, steps, path_len, path_node = heapq.heappop(q)
        
        c_ix = int((cx - 1.9) * 50.0 + 0.5)
        if c_ix < 0: c_ix = 0
        c_iy = int((cy + 3.1) * 50.0 + 0.5)
        c_ith = int((cth + M_PI) * 50.0 + 0.5)
        ckey = (c_ix, c_iy, c_ith, prev_gear)
        
        # 延后状态甄别 (Outdated node pruning)
        if visited.get(ckey, float('inf')) < cost - 1e-5:
            continue

        # ── RS 解析终点扩展（一杆进洞）──────────────────────────────────
        # Stage-1 宽松目标时禁用（_rs_expand=False），避免误触发
        if _rs_expand:
            rs_dist_to_goal = rs.rs_distance_pose(
                cx, cy, cth,
                RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                MIN_TURN_RADIUS
            )
            if rs_dist_to_goal < RS_EXPANSION_RADIUS:
                import sys as _sys
                print('[RS_EXPAND] node=({:.3f},{:.3f},{:.2f}deg) '
                      'rs_dist={:.3f}m < {:.1f}m → try'.format(
                          cx, cy, math.degrees(cth),
                          rs_dist_to_goal, RS_EXPANSION_RADIUS),
                      file=_sys.stderr)
                rs_traj = rs.rs_sample_path(
                    cx, cy, cth,
                    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                    MIN_TURN_RADIUS, step=DT * 0.5
                )
                if rs_traj:
                    ex, ey, eth = rs_traj[-1]
                    goal_reached = (ex <= 2.25
                                    and abs(ey) <= 0.18
                                    and abs(eth) <= ALIGN_GOAL_DYAW)
                    collision_ok = _check_traj_collision(rs_traj, no_corridor)
                    print('[RS_EXPAND] endpoint=({:.3f},{:.3f},{:.2f}deg) '
                          'goal_ok={} collision_ok={}'.format(
                              ex, ey, math.degrees(eth),
                              goal_reached, collision_ok),
                          file=_sys.stderr)
                    if goal_reached and collision_ok:
                        final_path = []
                        curr = path_node
                        while curr is not None:
                            final_path.append(curr[1])
                            curr = curr[0]
                        final_path.reverse()
                        if stats is not None:
                            stats['expanded'] = expanded
                            stats['elapsed_ms'] = round(
                                (time.perf_counter() - t_start) * 1000.0, 1)
                            stats['use_rs'] = use_rs
                            stats['no_corridor'] = no_corridor
                            stats['rs_expansion'] = True
                        return True, final_path, rs_traj
                else:
                    import sys as _sys
                    print('[RS_EXPAND] rs_sample_path returned None', file=_sys.stderr)
        # ────────────────────────────────────────────────────────────

        # Primitive 数量上限硬约束
        if path_len >= _prim_limit:
            continue

        expanded += 1
        if expanded % 20000 == 0:
            print(f"[A* 规划中] 已展开 {expanded} 个节点，耗时: {(time.perf_counter() - t_start)*1000:.0f}ms...")
            
        if expanded > _expand_limit:
            break
            
        cos_th = math.cos(cth)
        sin_th = math.sin(cth)
        
        for act, N, traj in precomp_prim:
            ok = True
            hit_goal = False
            step_hit = 0
            
            nx = ny = nth = 0.0
            
            # C 语言层级的隐式解包循环比传统索引遍历快很多
            for i, (dx, dy, dth, cdth, sdth) in enumerate(traj):
                # 采用相对变换矩阵将缓存微小增量直接投射为绝对坐标
                nx = cx + dx * cos_th - dy * sin_th
                ny = cy + dx * sin_th + dy * cos_th
                nth = cth + dth
                
                if nth > M_PI: nth -= PI2
                elif nth <= -M_PI: nth += PI2
                
                # 碰撞检测优化：利用已有的三角函数值
                # sin(nth) = sin(th + dth) = sin_th * cdth + cos_th * sdth
                sin_nth = sin_th * cdth + cos_th * sdth
                
                is_valid, _ = check_collision(nx, ny, nth, sin_nth,
                                              no_corridor=no_corridor)
                if not is_valid:
                    ok = False
                    break
                        
                # 3. 对齐命中终止判别 (Handover Region / pre-approach zone)
                if _goal_xmin <= nx <= _goal_xmax and abs(ny) <= _goal_yhalf and abs(nth) <= _goal_thmax:
                    hit_goal = True
                    step_hit = i + 1
                    break
                    
            if not ok:
                continue
                
            new_steps = steps + (step_hit if hit_goal else N)
            if new_steps > _step_limit:
                continue
                
            # 命中目标: 触发重构链回溯
            if hit_goal:
                final_path = [act]
                curr = path_node
                while curr is not None:
                    final_path.append(curr[1])
                    curr = curr[0]
                final_path.reverse()
                if stats is not None:
                    stats['expanded'] = expanded
                    stats['elapsed_ms'] = round((time.perf_counter() - t_start) * 1000.0, 1)
                    stats['use_rs'] = use_rs
                    stats['no_corridor'] = no_corridor
                    stats['rs_expansion'] = False
                    # 记录 A* 实际命中目标时的位姿（mid-primitive 截断点）
                    # 供两阶段规划的 Stage-2 使用，避免 primitive 末端越界问题
                    stats['goal_pos'] = (nx, ny, nth)
                    # 命中目标时位于当前 primitive 的第 step_hit 步（1-based）
                    # 用于测试/回放时截断最后一段，避免“末段过冲”假碰撞。
                    stats['goal_step_hit'] = step_hit
                return True, final_path, None
                
            # 计算 Cost 计分
            # 状态键已包含 prev_gear，此处换挡惩罚由 B-spline 后处理和平滑代价共同承担。
            # 为了平滑长线倒车，此处增加明显的档位切换惩罚 (GEAR_CHANGE_PENALTY)。
            step_time = N * DT
            step_cost = step_time
            if act[0] == 'R':
                step_cost += 1.5 * step_time
            
            # 对换挡施加惩罚，鼓励顺滑的连续曲线
            if prev_gear != 'N' and act[0] != prev_gear:
                step_cost += 5.0
                
            new_cost = cost + step_cost
            
            n_ix = int((nx - 1.9) * 50.0 + 0.5)
            if n_ix < 0: n_ix = 0
            n_iy = int((ny + 3.1) * 50.0 + 0.5)
            n_ith = int((nth + M_PI) * 50.0 + 0.5)
            nkey = (n_ix, n_iy, n_ith, act[0])
            
            # 新代际更优状态则覆盖扩展
            if visited.get(nkey, float('inf')) > new_cost:
                visited[nkey] = new_cost
                
                # ==== Heuristic ====
                if _heuristic_preapproach:
                    # Stage-1 专用启发：同时惩罚横向偏移、x 越界、角度偏离
                    abs_ny = ny if ny >= 0 else -ny
                    h_y = max(0.0, abs_ny - _goal_yhalf) * 5.0
                    h_x = max(0.0, _goal_xmin - nx) * 3.0  # 鼓励从墙边退回
                    abs_nth = nth if nth >= 0 else -nth
                    h_th = max(0.0, abs_nth - _goal_thmax) * 3.0
                    h = h_y + h_x + h_th
                    h_weight = 1.0
                elif use_rs:
                    # RS 距离（米）除以最大速度（0.25 m/s）换算为时间下界（秒）
                    rs_dist = rs.rs_distance_pose(
                        nx, ny, nth,
                        RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                        MIN_TURN_RADIUS
                    )
                    h = rs_dist / 0.25  # max forward speed = 0.25 m/s
                    h_weight = 1.0
                else:
                    # 几何启发式：各系数经实测校准使 h ≈ 实际代价
                    dx_h = nx - 2.25 if nx > 2.25 else 0.0
                    abs_ny = ny if ny > 0 else -ny
                    dy_h = abs_ny - 0.18 if abs_ny > 0.18 else 0.0
                    abs_nth = nth if nth > 0 else -nth
                    dth_h = abs_nth - ALIGN_GOAL_DYAW if abs_nth > ALIGN_GOAL_DYAW else 0.0

                    # room_needed：完成侧向对齐至少需要的 x-空间
                    room_needed = 2.10 + dy_h * 3.0 + dth_h * 2.0
                    if room_needed > 2.95:
                        room_needed = 2.95

                    back_up_penalty = 0.0
                    if nx < room_needed and (dy_h > 0.05 or dth_h > 0.05):
                        # 系数由 2.0→5.0（≈1/v_reverse），cap 由 2.0→6.0
                        # 令 h 更接近"需要倒退距离/速度"的真实代价
                        back_up_penalty = min((room_needed - nx) * 5.0, 6.0)

                    # dy 系数 8→16，dth 系数 4→6，使 h 更逼近实际代价
                    h = dx_h * 4.0 + dy_h * 16.0 + dth_h * 6.0 + back_up_penalty
                    h_weight = 1.0  # 已足够准确，不需再加权放大
                # ===================
                tiebreaker += 1
                heapq.heappush(q, (
                    new_cost + h * h_weight,
                    tiebreaker,
                    new_cost,
                    nx, ny, nth,
                    act[0],   # prev_gear
                    act[1],   # prev_s
                    new_steps,
                    path_len + 1,
                    (path_node, act)
                ))
                
    if stats is not None:
        stats['expanded'] = expanded
        stats['elapsed_ms'] = round((time.perf_counter() - t_start) * 1000.0, 1)
        stats['use_rs'] = use_rs
        stats['no_corridor'] = no_corridor
        stats['rs_expansion'] = False

    return False, None, None

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


def _k_turn_preposition(x0, y0, theta0, precomp_prim, no_corridor=False):
    """
    确定性 K-turn 预定位（三阶段）：
      Phase-1   安全减 y：两步前瞻贪心，x 硬下限 X_FLOOR_SAFE=2.05m。
      Phase-1b  紧急减 y：一步贪心，x 硬下限 X_FLOOR_EMG=1.92m（仅在 y 仍大时触发）。
      Phase-2   x 恢复  ：倒退使 x >= PREAPPROACH_X_MIN=2.2m。
    不依赖 A*，总是在毫秒内完成。
    返回: (ok, actions, final_x, final_y, final_theta)
    """
    cx, cy, cth = x0, y0, theta0
    acts = []
    X_FLOOR_SAFE = 2.05   # 安全区：Stage-2 在 x>=2.05 时可解
    X_FLOOR_EMG  = 1.92   # 紧急区：最低不触墙

    def _apply(px, py, pth, traj, x_floor=None):
        cos_t, sin_t = math.cos(pth), math.sin(pth)
        ex = ey = eth = 0.0
        for dx, dy, dth, cdth, sdth in traj:
            ex = px + dx * cos_t - dy * sin_t
            ey = py + dx * sin_t + dy * cos_t
            eth = pth + dth
            if eth > M_PI:   eth -= PI2
            elif eth <= -M_PI: eth += PI2
            if x_floor is not None and ex < x_floor:
                return False, px, py, pth
            sin_nth = sin_t * cdth + cos_t * sdth
            valid, _ = check_collision(ex, ey, eth, sin_nth, no_corridor=no_corridor)
            if not valid:
                return False, px, py, pth
        return True, ex, ey, eth

    def _score_y(ex, ey, eth, gear=None, prev_gear=None, w_x=1.0):
        y_over = max(0.0, abs(ey) - PREAPPROACH_Y_MAX) * 10.0
        y_raw  = abs(ey) * 1.5  # 提升：更渴望靠近 y=0，缩短路径
        x_pen  = max(0.0, PREAPPROACH_X_MIN - ex) * w_x * 5.0
        # 对 x 的远端惩罚改为温和渐进式，促使它在能够倒进去的前提下尽量选择紧凑弧线
        x_over = max(0.0, ex - 3.2) * 2.0  # 提升：增强远端惩罚
        th_pen = max(0.0, abs(eth) - PREAPPROACH_TH_MAX) * 0.05
        # 换挡惩罚下调至2.0，足以防止原地锯齿揉库，又不会不敢换挡
        gear_pen = 0.0 if prev_gear is None or gear == prev_gear else 1.0  # 降低：允许果断换挡
        return y_over + y_raw + x_pen + x_over + th_pen + gear_pen

    def _check_goal():
        return (abs(cy) <= PREAPPROACH_Y_MAX
                and PREAPPROACH_X_MIN <= cx <= PREAPPROACH_X_MAX
                and abs(cth) <= PREAPPROACH_TH_MAX)

    # ── Phase-0：如果横向偏差极大且距离墙太近，强制倒车拉开空间并打方向 ──
    needed_x_init = 2.2 + abs(y0) * 1.5
    needed_x_init = min(needed_x_init, PREAPPROACH_X_MAX - 1.0)
    if cx < needed_x_init and abs(cy) > 1.0:
        for _ in range(40):
            if cx >= needed_x_init: break
            best_p0 = None
            best_s0 = float('inf')
            for act0, _n0, traj0 in precomp_prim:
                if act0[0] != 'R': continue
                ok0, ex0, ey0, eth0 = _apply(cx, cy, cth, traj0)
                if not ok0: continue
                # 倒退时需要配合打死方向盘：在右边(cy<0)期待负向，左边期待正向
                target_th = -1.5 if cy < 0 else 1.5
                th_pen = abs(eth0 - target_th) * 5.0
                if th_pen < best_s0:
                    best_s0 = th_pen
                    best_p0 = act0
                    best_st0 = (ex0, ey0, eth0)
            if best_p0 is None: break
            cx, cy, cth = best_st0
            acts.append(best_p0)

    # ── Phase-1：安全减 y（两步前瞻，x >= X_FLOOR_SAFE） ─────────────
    best_abs_y  = abs(cy)
    stagnate_p1 = 0
    for _ in range(150):
        if _check_goal():
            return True, acts, cx, cy, cth
        lookahead_2 = abs(cy) > PREAPPROACH_Y_MAX + 0.06
        best_first = best_state = None
        prev_gear = acts[-1][0] if acts else None
        best_score = float('inf')
        for act1, _n1, traj1 in precomp_prim:
            ok1, ex1, ey1, eth1 = _apply(cx, cy, cth, traj1, X_FLOOR_SAFE)
            if not ok1:
                continue
            if lookahead_2:
                local = float('inf')
                for _a2, _n2, traj2 in precomp_prim:
                    ok2, ex2, ey2, eth2 = _apply(ex1, ey1, eth1, traj2, X_FLOOR_SAFE)
                    if ok2:
                        s2 = _score_y(ex2, ey2, eth2, gear=_a2[0], prev_gear=act1[0])
                        if s2 < local: local = s2
                
                # 若两步前瞻可行，需将“当前步的换挡惩罚”加上
                if local < float('inf'):
                    cand = local + (0.0 if prev_gear is None or act1[0] == prev_gear else 1.0)
                else:
                    cand = _score_y(ex1, ey1, eth1, gear=act1[0], prev_gear=prev_gear)
            else:
                cand = _score_y(ex1, ey1, eth1, gear=act1[0], prev_gear=prev_gear)
            if cand < best_score:
                best_score = cand; best_first = act1; best_state = (ex1, ey1, eth1)
        if best_first is None:
            break
        cx, cy, cth = best_state; acts.append(best_first)
        if abs(cy) < best_abs_y - 1e-3:
            best_abs_y = abs(cy); stagnate_p1 = 0
        else:
            stagnate_p1 += 1
            if stagnate_p1 >= 15:
                break

    # ── Phase-1b：紧急减 y（一步贪心，x >= X_FLOOR_EMG，仅当 y 仍大时） ─
    if abs(cy) > PREAPPROACH_Y_MAX + 0.05:
        best_abs_y = abs(cy); stagnate_emg = 0
        for _ in range(60):
            if _check_goal():
                return True, acts, cx, cy, cth
            best_first = best_state = None
            prev_gear = acts[-1][0] if acts else None
            best_score = float('inf')
            for act1, _n1, traj1 in precomp_prim:
                ok1, ex1, ey1, eth1 = _apply(cx, cy, cth, traj1, X_FLOOR_EMG)
                if not ok1: continue
                cand = _score_y(ex1, ey1, eth1, gear=act1[0], prev_gear=prev_gear, w_x=0.5)   # x 惩罚减半，放宽向墙推进
                if cand < best_score:
                    best_score = cand; best_first = act1; best_state = (ex1, ey1, eth1)
            if best_first is None:
                break
            cx, cy, cth = best_state; acts.append(best_first)
            if abs(cy) < best_abs_y - 1e-3:
                best_abs_y = abs(cy); stagnate_emg = 0
            else:
                stagnate_emg += 1
                if stagnate_emg >= 15:
                    break

    # ── Phase-2：x 恢复——倒退使 x >= PREAPPROACH_X_MIN ──────────────────
    for _ in range(50):
        if cx >= PREAPPROACH_X_MIN:
            break
        best_rev = best_rev_st = None
        prev_gear = acts[-1][0] if acts else None
        best_rv = float('inf')
        for act, _n, traj in precomp_prim:
            # 允许在极度靠近墙时前进，否则优先倒退
            if act[0] != 'R' and cx >= X_FLOOR_EMG + 0.5:
                continue
            ok_r, ex_r, ey_r, eth_r = _apply(cx, cy, cth, traj)
            if not ok_r: continue
            
            x_lack = max(0.0, PREAPPROACH_X_MIN - ex_r) * 3.0
            dy_pen = abs(abs(ey_r) - abs(cy)) * 2.0
            th_pen = max(0.0, abs(eth_r) - PREAPPROACH_TH_MAX) * 0.05
            gear_pen = 0.0 if prev_gear is None or act[0] == prev_gear else 50.0
            
            rv = x_lack + dy_pen + th_pen + gear_pen
            if rv < best_rv:
                best_rv = rv; best_rev = act; best_rev_st = (ex_r, ey_r, eth_r)
        if best_rev is None: break
        cx, cy, cth = best_rev_st; acts.append(best_rev)

    # ── 最终检查 ─────────────────────────────────────────────────────────
    if _check_goal():
        return True, acts, cx, cy, cth
    return False, acts, cx, cy, cth

def plan_path_robust(x0, y0, theta0, precomp_prim,
                     use_rs=False, stats=None, no_corridor=False):
    """
    鲁棒两阶段规划器：自动处理大侧偏 / 大偏航起点。

    逻辑：
      • 若 |y0| ≤ TWO_STAGE_Y_THRESH 且 |θ0| ≤ TWO_STAGE_TH_THRESH
        → 直接调用 plan_path（单阶段，与原来行为完全相同）
      • 否则启动两阶段：
          Stage-1  确定性 K-turn 预定位（贪心选择基元，毫秒级完成）
          Stage-2  从 Stage-1 终点用标准 plan_path 精准对准托盘

    返回: (success, actions, rs_traj)   — 与 plan_path 接口完全一致
    """
    t_robust = time.perf_counter()

    # 超出 Python A* 可处理范围：立即返回（避免 20s+ 阻塞）
    if abs(y0) > MAX_PLANNABLE_Y:
        if stats is not None:
            stats['expanded'] = 0
            stats['elapsed_ms'] = 0.0
            stats['two_stage'] = False
            stats['out_of_range'] = True
        return False, None, None

    needs_two_stage = (abs(y0) > TWO_STAGE_Y_THRESH
                       or abs(theta0) > TWO_STAGE_TH_THRESH)

    if not needs_two_stage:
        return plan_path(x0, y0, theta0, precomp_prim,
                         use_rs=use_rs, stats=stats,
                         no_corridor=no_corridor)

    # ── Stage-1：预定位（y 大偏差优先用 A*，θ 大偏差用贪心）────────────────
    import time as _time
    t1 = _time.perf_counter()
    ok1 = False
    acts1 = []
    mx, my, mth = x0, y0, theta0
    stage1_mode = 'none'
    stage1_goal_step_hit = None

    # y 偏差超出预接近区时，优先使用 Stage-1 A* 进入 pre-approach zone。
    # 这能避免贪心失败后回退到慢速单阶段 A*（7~12s）。
    if abs(y0) > PREAPPROACH_Y_MAX:
        st1 = {}
        ok1, acts1, _ = plan_path(
            x0, y0, theta0, precomp_prim,
            use_rs=False,
            stats=st1,
            no_corridor=no_corridor,
            _goal_xmin=PREAPPROACH_X_MIN,
            _goal_xmax=PREAPPROACH_X_MAX,
            _goal_yhalf=PREAPPROACH_Y_MAX,
            _goal_thmax=PREAPPROACH_TH_MAX,
            # 仅做“快速预接近”尝试；超时后立刻走贪心兜底，避免 Stage-1 自身变慢。
            _step_limit=700,
            _expand_limit=12000,
            _prim_limit=18,
            _rs_expand=False,
            _heuristic_preapproach=True
        )
        if ok1:
            goal_pos = st1.get('goal_pos')
            stage1_goal_step_hit = st1.get('goal_step_hit')
            if goal_pos is not None:
                mx, my, mth = goal_pos
            else:
                mx, my, mth = _replay_to_end(x0, y0, theta0, acts1, precomp_prim)
            stage1_mode = 'astar_preapproach'

    # A* 预接近失败或 θ-only 大偏差时，使用贪心 K-turn 兜底。
    if not ok1:
        ok1, acts1, mx, my, mth = _k_turn_preposition(
            x0, y0, theta0, precomp_prim, no_corridor=no_corridor)
        if ok1:
            stage1_mode = 'greedy_kturn'
        else:
            # 贪心已显著改善但未达硬阈值时，允许进入 Stage-2 继续精对准，
            # 避免回退到慢速单阶段 Direct A*。
            # 阈值再次松绑：只要把 y 压进 1.2m，Stage-2 的 A* 也能算出来，且比从头算快
            improved_enough = (abs(my) <= PREAPPROACH_Y_MAX + 0.8 and
                               abs(my) <= abs(y0) - 0.08)
            if improved_enough and acts1:
                ok1 = True
                stage1_mode = 'greedy_kturn_partial'

    t1_ms = round((_time.perf_counter() - t1) * 1000.0, 1)

    if not ok1:
        # Stage-1 全部失败：回退单阶段（保留成功机会）
        return plan_path(x0, y0, theta0, precomp_prim,
                         use_rs=use_rs, stats=stats,
                         no_corridor=no_corridor)

    # ── Stage-2：精准对准托盘 ────────────────────────────────────────
    st2 = {}
    ok2, acts2, rs_traj = plan_path(
        mx, my, mth, precomp_prim,
        use_rs=use_rs,
        stats=st2,
        no_corridor=no_corridor,
    )

    total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)

    if ok2:
        if stats is not None:
            stats['expanded']    = st2.get('expanded', 0)
            stats['elapsed_ms']  = total_ms
            stats['use_rs']      = use_rs
            stats['no_corridor'] = no_corridor
            stats['rs_expansion'] = st2.get('rs_expansion', False)
            stats['two_stage']   = True
            stats['stage1_acts'] = len(acts1 or [])
            stats['stage2_acts'] = len(acts2 or [])
            stats['stage1_ms']   = t1_ms
            stats['stage2_ms']   = st2.get('elapsed_ms', 0)
            stats['stage1_mode'] = stage1_mode
            stats['stage1_end']  = (mx, my, mth)
            stats['stage1_goal_step_hit'] = stage1_goal_step_hit
            # Stage-2 精确命中目标的位姿（A* mid-primitive 截断点，比完整 replay 更准确）
            stats['goal_pos']    = st2.get('goal_pos')
            stats['goal_step_hit'] = st2.get('goal_step_hit')
        return True, (acts1 or []) + (acts2 or []), rs_traj

    # Stage-2 失败 → 直接单阶段兜底
    return plan_path(x0, y0, theta0, precomp_prim,
                     use_rs=use_rs, stats=stats,
                     no_corridor=no_corridor)


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

def solve():
    # 采用批量读取高速解析，极大地避免 Special Judge 逐行 I/O 阻塞
    input_data = sys.stdin.read().split()
    if not input_data:
        return
        
    precomp_prim = init_primitives()
    T = int(input_data[0])
    idx = 1
    
    out = []
    
    for _ in range(T):
        if idx >= len(input_data):
            break
        x0 = float(input_data[idx])
        y0 = float(input_data[idx+1])
        theta0 = float(input_data[idx+2])
        idx += 3
        
        success, result, _ = plan_path(x0, y0, theta0, precomp_prim)
        
        if success:
            final_path = result
            out.append(str(len(final_path)))
            for a in final_path:
                out.append(f"{a[0]} {a[1]:.1f} {a[2]:.2f}")
        else:
            out.append("IMPOSSIBLE")

    # 全量结果一次性缓冲输出彻底释放阻塞锁
    sys.stdout.write("\n".join(out) + "\n")

if __name__ == '__main__':
    solve()
