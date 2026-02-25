import math
import heapq
import time
from primitives import M_PI, PI2, DT, ALIGN_GOAL_DYAW


def plan_path(x0, y0, theta0, precomp_prim,
              collision_fn,
              heuristic_fn,
              rs_expand_fn=None,
              stats=None,
              _goal_xmin=1.92, _goal_xmax=2.25,
              _goal_ymin=-0.18, _goal_ymax=0.18, _goal_thmax=ALIGN_GOAL_DYAW,
              _step_limit=1079, _expand_limit=150000, _prim_limit=30,
              _rs_expand=True,
              rs_expansion_radius=2.5,
              deadline=None):
    """
    纯 A* 搜索引擎。

    通过回调注入碰撞检测和启发式，引擎内部不直接依赖外部环境参数。

    参数:
        collision_fn(nx, ny, nth, sin_nth) -> (is_valid, reason)
        heuristic_fn(nx, ny, nth) -> (h, h_weight)
        rs_expand_fn(cx, cy, cth, cost, path_node, expanded, t_start, stats) -> (success, actions, rs_traj) or None
        stats: 可选字典
        _goal_*: 目标区参数
        _step_limit, _expand_limit, _prim_limit: 搜索限制
        _rs_expand: 是否启用 RS 扩展
        rs_expansion_radius: RS 扩展距离判定范围

    返回: (success, actions, rs_traj)
    """
    t_start = time.perf_counter()
    visited = {}

    ix0 = int((x0 - 1.9) * 50.0 + 0.5)
    if ix0 < 0: ix0 = 0
    iy0 = int((y0 + 3.1) * 50.0 + 0.5)
    ith0 = int((theta0 + M_PI) * 50.0 + 0.5)
    start_key = (ix0, iy0, ith0, 'N')

    visited[start_key] = 0.0

    tiebreaker = 0
    q = [(0.0, tiebreaker, 0.0, x0, y0, theta0, 'N', 0.0, 0, 0, None)]

    expanded = 0

    while q:
        f, _, cost, cx, cy, cth, prev_gear, prev_s, steps, path_len, path_node = heapq.heappop(q)

        c_ix = int((cx - 1.9) * 50.0 + 0.5)
        if c_ix < 0: c_ix = 0
        c_iy = int((cy + 3.1) * 50.0 + 0.5)
        c_ith = int((cth + M_PI) * 50.0 + 0.5)
        ckey = (c_ix, c_iy, c_ith, prev_gear)

        if visited.get(ckey, float('inf')) < cost - 1e-5:
            continue

        # RS 解析终点扩展（一杆进洞）
        if _rs_expand and rs_expand_fn is not None:
            result = rs_expand_fn(cx, cy, cth, cost, path_node, expanded, t_start, stats)
            if result is not None:
                return result

        if path_len >= _prim_limit:
            continue

        expanded += 1
        if expanded % 5000 == 0:
            elapsed_so_far = (time.perf_counter() - t_start) * 1000
            print(f"[A*] expanded={expanded}, f={f:.1f}, cx={cx:.2f}, cy={cy:.2f}, elapsed={elapsed_so_far:.0f}ms")

        if expanded > _expand_limit:
            break
        # Check deadline more frequently (every 500 instead of 2000)
        if expanded % 500 == 0:
            now = time.perf_counter()
            if deadline is not None and now > deadline:
                break
            if deadline is None and (now - t_start) > 20.0:
                break

        cos_th = math.cos(cth)
        sin_th = math.sin(cth)

        for act, N, traj in precomp_prim:
            ok = True
            hit_goal = False
            step_hit = 0

            nx = ny = nth = 0.0

            for i, (dx, dy, dth, cdth, sdth) in enumerate(traj):
                nx = cx + dx * cos_th - dy * sin_th
                ny = cy + dx * sin_th + dy * cos_th
                nth = cth + dth

                if nth > M_PI: nth -= PI2
                elif nth <= -M_PI: nth += PI2

                sin_nth = sin_th * cdth + cos_th * sdth

                is_valid, _ = collision_fn(nx, ny, nth, sin_nth)
                if not is_valid:
                    ok = False
                    break

                if _goal_xmin <= nx <= _goal_xmax and _goal_ymin <= ny <= _goal_ymax and abs(nth) <= _goal_thmax:
                    hit_goal = True
                    step_hit = i + 1
                    break

            if not ok:
                continue

            new_steps = steps + (step_hit if hit_goal else N)
            if new_steps > _step_limit:
                continue

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
                    stats['rs_expansion'] = False
                    stats['goal_pos'] = (nx, ny, nth)
                    stats['goal_step_hit'] = step_hit
                return True, final_path, None

            # Cost calculation
            step_time = N * DT
            step_cost = step_time
            if act[0] == 'R':
                step_cost += 1.5 * step_time

            if prev_gear != 'N' and act[0] != prev_gear:
                step_cost += 5.0

                gc_count = 0
                curr_node = path_node
                curr_gear = prev_gear
                lookback_depth = 0
                while curr_node is not None and curr_node[0] is not None and lookback_depth < 12:
                    p_node = curr_node[0]
                    if p_node[1] is not None:
                        p_gear = p_node[1][0]
                        if p_gear != curr_gear:
                            gc_count += 1
                            curr_gear = p_gear
                    curr_node = p_node
                    lookback_depth += 1
                    if gc_count >= 4:
                        break

                if gc_count >= 4:
                    step_cost += 15.0

            new_cost = cost + step_cost

            n_ix = int((nx - 1.9) * 50.0 + 0.5)
            if n_ix < 0: n_ix = 0
            n_iy = int((ny + 3.1) * 50.0 + 0.5)
            n_ith = int((nth + M_PI) * 50.0 + 0.5)
            nkey = (n_ix, n_iy, n_ith, act[0])

            if visited.get(nkey, float('inf')) > new_cost:
                visited[nkey] = new_cost

                h, h_weight = heuristic_fn(nx, ny, nth)

                tiebreaker += 1
                heapq.heappush(q, (
                    new_cost + h * h_weight,
                    tiebreaker,
                    new_cost,
                    nx, ny, nth,
                    act[0],
                    act[1],
                    new_steps,
                    path_len + 1,
                    (path_node, act)
                ))

    if stats is not None:
        stats['expanded'] = expanded
        stats['elapsed_ms'] = round((time.perf_counter() - t_start) * 1000.0, 1)
        stats['rs_expansion'] = False

    return False, None, None
