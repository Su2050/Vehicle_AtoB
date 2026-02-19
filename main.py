import sys
import math
import heapq

# === 物理与环境常量 ===
DT = 1.0 / 30.0
WHEELBASE = 1.6
MAX_STEER = 0.65
M_PI = math.pi
PI2 = 2.0 * math.pi
ALIGN_GOAL_DYAW = 5.0 * M_PI / 180.0

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

def check_collision(nx, ny, nth, sin_nth=None):
    """
    检查状态是否合法
    返回: (is_valid, reason)
    reason: 'OK', 'WALL', 'CORRIDOR'
    """
    # 1. 越界 & hard_wall 硬墙阻挡监控 (1.92 内阻断了出界 < 0)
    if nx > 3.0 or nx < 1.92 or ny > 3.0 or ny < -3.0:
        return False, 'WALL'
        
    # 2. 安全走廊门控 (safe_corridor)
    if nx <= 2.05:
        if sin_nth is None:
            sin_nth = math.sin(nth)
        tip_lat = ny - 1.87 * sin_nth
        sc = (0.15 + (nx - 1.87) * 0.8) if nx > 1.87 else 0.15
        if tip_lat > sc or tip_lat < -sc:
            return False, 'CORRIDOR'
            
    return True, 'OK'

def plan_path(x0, y0, theta0, precomp_prim):
    """
    执行 A* 搜索
    返回: (success, result)
    success: True/False
    result: actions list if success, else None
    """
    visited = {}
    
    # 将浮点转为正整数键用于极速哈希去重
    ix0 = int((x0 - 1.9) * 50.0 + 0.5)
    if ix0 < 0: ix0 = 0
    iy0 = int((y0 + 3.1) * 50.0 + 0.5)
    ith0 = int((theta0 + M_PI) * 50.0 + 0.5)
    start_key = (ix0, iy0, ith0)
    
    visited[start_key] = 0.0
    
    tiebreaker = 0
    # 优先队列 Tuple : (f_score, tiebreaker, cost, x, y, th, prev_s, steps, path_len, path_node)
    q = [(0.0, tiebreaker, 0.0, x0, y0, theta0, 0.0, 0, 0, None)]
    
    expanded = 0
    
    while q:
        f, _, cost, cx, cy, cth, prev_s, steps, path_len, path_node = heapq.heappop(q)
        
        c_ix = int((cx - 1.9) * 50.0 + 0.5)
        if c_ix < 0: c_ix = 0
        c_iy = int((cy + 3.1) * 50.0 + 0.5)
        c_ith = int((cth + M_PI) * 50.0 + 0.5)
        ckey = (c_ix, c_iy, c_ith)
        
        # 延后状态甄别 (Outdated node pruning)
        if visited.get(ckey, float('inf')) < cost - 1e-5:
            continue
            
        # Primitive 数量上限硬约束
        if path_len >= 30:
            continue
            
        expanded += 1
        if expanded > 150000: # 触碰极限深度截断兜底
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
                
                is_valid, _ = check_collision(nx, ny, nth, sin_nth)
                if not is_valid:
                    ok = False
                    break
                        
                # 3. 对齐命中终止判别 (Handover Region)
                if nx <= 2.25 and -0.18 <= ny <= 0.18 and -ALIGN_GOAL_DYAW <= nth <= ALIGN_GOAL_DYAW:
                    hit_goal = True
                    step_hit = i + 1
                    break
                    
            if not ok:
                continue
                
            new_steps = steps + (step_hit if hit_goal else N)
            if new_steps > 1079: # 超过执行步数限额 1079
                continue
                
            # 命中目标: 触发重构链回溯
            if hit_goal:
                final_path = [act]
                curr = path_node
                while curr is not None:
                    final_path.append(curr[1])
                    curr = curr[0]
                final_path.reverse()
                return True, final_path
                
            # 计算 Cost 计分 (严格依照题意记分板规则)
            step_time = N * DT
            step_cost = step_time
            if act[0] == 'R':
                step_cost += 1.5 * step_time
            if path_len > 0:
                diff = act[1] - prev_s
                step_cost += 0.3 * (diff if diff >= 0 else -diff)
                
            new_cost = cost + step_cost
            
            n_ix = int((nx - 1.9) * 50.0 + 0.5)
            if n_ix < 0: n_ix = 0
            n_iy = int((ny + 3.1) * 50.0 + 0.5)
            n_ith = int((nth + M_PI) * 50.0 + 0.5)
            nkey = (n_ix, n_iy, n_ith)
            
            # 新代际更优状态则覆盖扩展
            if visited.get(nkey, float('inf')) > new_cost:
                visited[nkey] = new_cost
                
                # ==== Heuristic 启发式漏洞评估 ====
                # 拆解逻辑手动优化 max 和 abs 以避开大量的函数调用栈开销
                dx_h = nx - 2.25 if nx > 2.25 else 0.0
                abs_ny = ny if ny > 0 else -ny
                dy_h = abs_ny - 0.18 if abs_ny > 0.18 else 0.0
                abs_nth = nth if nth > 0 else -nth
                dth_h = abs_nth - ALIGN_GOAL_DYAW if abs_nth > ALIGN_GOAL_DYAW else 0.0
                
                room_needed = 2.10 + dy_h * 2.5 + dth_h * 1.5
                if room_needed > 2.95: 
                    room_needed = 2.95
                    
                # 退让漏斗 (Back-up Penalty)：如发觉车辆太靠墙但角度横向未打正，严厉施加惩罚以逼迫其换入 R 倒挡退让避险！
                back_up_penalty = 0.0
                if nx < room_needed and (dy_h > 0.05 or dth_h > 0.05):
                    back_up_penalty = (room_needed - nx) * 8.0
                    
                h = dx_h * 4.0 + dy_h * 8.0 + dth_h * 4.0 + back_up_penalty
                # ==================================
                
                tiebreaker += 1
                heapq.heappush(q, (
                    new_cost + h * 1.5,
                    tiebreaker,
                    new_cost,
                    nx, ny, nth,
                    act[1],
                    new_steps,
                    path_len + 1,
                    (path_node, act)
                ))
                
    return False, None

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
        
        success, result = plan_path(x0, y0, theta0, precomp_prim)
        
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
