import math
import time
import rs
import primitives
from primitives import (
    DT, M_PI, PI2, ALIGN_GOAL_DYAW, MIN_TURN_RADIUS,
    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
    PREAPPROACH_X_MIN, PREAPPROACH_X_MAX, PREAPPROACH_Y_MAX, PREAPPROACH_TH_MAX,
    TWO_STAGE_Y_THRESH, TWO_STAGE_TH_THRESH, MAX_PLANNABLE_Y,
    _replay_to_end
)
from collision import check_collision
from rs_utils import plan_path_pure_rs
from heuristic import (DijkstraGrid, geometric_heuristic, rs_heuristic,
                       rs_grid_heuristic, preapproach_heuristic)
import astar_core
from fallback_2d import plan_2d_fallback


# ── Wall-rejection constants ──
_WALL_X = 1.92
_WALL_CORRIDOR_Y = 0.5
_MAX_BEHIND_WALL_PTS = 5


def _path_goes_behind_wall(traj):
    """Reject paths that route behind the pallet wall (x < 1.92, |y| > 0.5)."""
    if not traj:
        return False
    count = 0
    for pt in traj:
        if pt[0] < _WALL_X and abs(pt[1]) > _WALL_CORRIDOR_Y:
            count += 1
            if count > _MAX_BEHIND_WALL_PTS:
                return True
    return False


def _count_gear_shifts(acts):
    if not acts:
        return 0
    s = 0
    last_g = acts[0][0]
    for a in acts[1:]:
        if a[0] != last_g:
            s += 1
            last_g = a[0]
    return s


def _preprocess_obstacles(obstacles):
    if not obstacles:
        return None
    fast = []
    for obs in obstacles:
        ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
        fast.append((min(ox, ox + ow), max(ox, ox + ow),
                     min(oy, oy + oh), max(oy, oy + oh)))
    return fast


def _make_collision_fn(no_corridor, fast_obstacles):
    def collision_fn(nx, ny, nth, sin_nth=None, cos_nth=None):
        return check_collision(nx, ny, nth, sin_nth, cos_nth,
                               no_corridor=no_corridor,
                               obstacles=fast_obstacles)
    return collision_fn


def _make_rs_expand_fn(collision_fn, rs_expansion_radius):
    def rs_expand_fn(cx, cy, cth, cost, path_node, expanded, t_start, stats):
        euclidean_goal = math.hypot(cx - RS_GOAL_X, cy - RS_GOAL_Y)
        if euclidean_goal < rs_expansion_radius + 1.0:
            rs_dist = rs.rs_distance_pose(
                cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
            if rs_dist < rs_expansion_radius:
                rs_traj = rs.rs_sample_path(
                    cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                    MIN_TURN_RADIUS, step=DT * 0.5)
                if rs_traj:
                    ex, ey, eth = rs_traj[-1]
                    goal_reached = (ex <= 2.25 and abs(ey) <= 0.18 and abs(eth) <= ALIGN_GOAL_DYAW)
                    traj_ok = True
                    for pt in rs_traj:
                        valid, _ = collision_fn(pt[0], pt[1], pt[2])
                        if not valid:
                            traj_ok = False; break
                    if goal_reached and traj_ok:
                        final_path = []
                        curr = path_node
                        while curr is not None:
                            final_path.append(curr[1]); curr = curr[0]
                        final_path.reverse()
                        if stats is not None:
                            stats['expanded'] = expanded
                            stats['elapsed_ms'] = round((time.perf_counter() - t_start) * 1000.0, 1)
                            stats['rs_expansion'] = True
                        return True, final_path, rs_traj
        return None
    return rs_expand_fn


def _make_heuristic_fn(use_rs, dijkstra_grid):
    if use_rs:
        if dijkstra_grid is not None:
            def h_fn(nx, ny, nth):
                return rs_grid_heuristic(nx, ny, nth, dijkstra_grid)
            return h_fn
        else:
            return rs_heuristic
    else:
        return geometric_heuristic


def _k_turn_preposition_obs(x0, y0, theta0, precomp_prim, no_corridor, fast_obstacles,
                            dijkstra_grid=None, ignore_obs_for_y_range=False,
                            target_y_max=None, x_ceil=None):
    """确定性 K-turn 预定位（有障碍版），完整移植自 main.py"""
    cx, cy, cth = x0, y0, theta0
    acts = []
    X_FLOOR_SAFE = 2.05
    X_FLOOR_EMG = 1.92
    _eff_y_max = target_y_max if target_y_max is not None else PREAPPROACH_Y_MAX
    _x_ceil = x_ceil if x_ceil is not None else PREAPPROACH_X_MAX

    def _apply(px, py, pth, traj, x_floor=None):
        cos_t, sin_t = math.cos(pth), math.sin(pth)
        ex = ey = eth = 0.0
        for dx, dy, dth, cdth, sdth in traj:
            ex = px + dx * cos_t - dy * sin_t
            ey = py + dx * sin_t + dy * cos_t
            eth = pth + dth
            if eth > M_PI: eth -= PI2
            elif eth <= -M_PI: eth += PI2
            if x_floor is not None and ex < x_floor: return False, px, py, pth
            if ex > _x_ceil: return False, px, py, pth
            sin_nth = sin_t * cdth + cos_t * sdth
            valid, _ = check_collision(ex, ey, eth, sin_nth, no_corridor=no_corridor, obstacles=fast_obstacles)
            if not valid: return False, px, py, pth
        return True, ex, ey, eth

    def _get_safe_y_range():
        y_min, y_max = -_eff_y_max, _eff_y_max
        if fast_obstacles and not ignore_obs_for_y_range:
            for obs in fast_obstacles:
                min_x, max_x, min_y, max_y = obs
                if max_x >= 2.1 and min_x <= max(cx, 4.0):
                    min_y_obs = min_y - 0.52
                    max_y_obs = max_y + 0.52
                    if min_y_obs < y_max and max_y_obs > y_min:
                        r1_ok = y_min < min_y_obs
                        r2_ok = max_y_obs < y_max
                        if r1_ok and not r2_ok: y_max = min_y_obs
                        elif r2_ok and not r1_ok: y_min = max_y_obs
                        elif r1_ok and r2_ok:
                            if abs(cy - min_y_obs) < abs(cy - max_y_obs): y_max = min_y_obs
                            else: y_min = max_y_obs
                        else:
                            if abs(cy - min_y_obs) < abs(cy - max_y_obs):
                                y_max = min_y_obs; y_min = min_y_obs - 0.5
                            else:
                                y_min = max_y_obs; y_max = max_y_obs + 0.5
        return y_min, y_max

    def _y_dist(y_val):
        y_min, y_max = _get_safe_y_range()
        if y_val < y_min: return y_min - y_val
        elif y_val > y_max: return y_val - y_max
        return 0.0

    def _score_y(ex, ey, eth, gear=None, prev_gear=None, w_x=1.0):
        y_min, y_max = _get_safe_y_range()
        if ey > y_max: y_over = (ey - y_max) * 10.0
        elif ey < y_min: y_over = (y_min - ey) * 10.0
        else: y_over = 0.0
        y_raw = 0.0
        if not (y_min <= ey <= y_max):
            y_raw = min(abs(ey - y_max), abs(ey - y_min)) * 2.0
        y_center_pen = abs(ey) * 3.0
        x_pen = max(0.0, PREAPPROACH_X_MIN - ex) * w_x * 5.0
        if abs(eth) > 0.4 and (abs(eth) < 1.57 or gear == 'F'):
            x_pen += max(0.0, 3.2 - ex) * w_x * 3.0
        ref_x = max(3.2, cx + 0.5)
        x_over = max(0.0, ex - ref_x) * 2.0
        align_weight = 10.0 if (y_min <= ey <= y_max) else 0.5
        th_pen = max(0.0, abs(eth) - PREAPPROACH_TH_MAX) * align_weight
        if gear == 'R':
            if ey - y_min < 0.3 and eth < 0: th_pen += 20.0 * abs(eth)
            if y_max - ey < 0.3 and eth > 0: th_pen += 20.0 * abs(eth)
            if y_min + 0.1 <= ey <= y_max - 0.1: th_pen += 15.0 * abs(eth)
        gear_pen = 0.0 if prev_gear is None or gear == prev_gear else 1.0
        return y_over + y_raw + y_center_pen + x_pen + x_over + th_pen + gear_pen

    def _check_goal():
        if cx <= 2.25 and abs(cy) <= 0.2 and abs(cth) <= 0.2:
            return True
        y_min, y_max = _get_safe_y_range()
        target_y = y_min if abs(cy - y_min) < abs(cy - y_max) else y_max
        required_x_min = max(PREAPPROACH_X_MIN, 2.3 + 2.0 * abs(cy - target_y))
        return (y_min <= cy <= y_max
                and required_x_min <= cx <= PREAPPROACH_X_MAX
                and abs(cth) <= PREAPPROACH_TH_MAX)

    # Phase-0
    needed_x_init = min(2.2 + abs(y0) * 1.5, PREAPPROACH_X_MAX - 1.0)
    if cx < needed_x_init and abs(cy) > 1.0:
        pull_away_gear = 'R' if abs(cth) > 1.57 else 'F'
        for _ in range(40):
            if cx >= needed_x_init: break
            best_p0 = None; best_s0 = float('inf')
            for act0, _n0, traj0 in precomp_prim:
                if act0[0] != pull_away_gear: continue
                ok0, ex0, ey0, eth0 = _apply(cx, cy, cth, traj0)
                if not ok0: continue
                target_th = (-1.5 if cy < 0 else 1.5) if abs(cy) > 1.2 else 0.0
                th_pen = abs(eth0 - target_th) * 5.0
                if th_pen < best_s0:
                    best_s0 = th_pen; best_p0 = act0; best_st0 = (ex0, ey0, eth0)
            if best_p0 is None: break
            cx, cy, cth = best_st0; acts.append(best_p0)

    # Phase-1
    best_abs_y = _y_dist(cy); stagnate_p1 = 0
    for _ in range(150):
        if _check_goal(): return True, acts, cx, cy, cth
        lookahead_2 = _y_dist(cy) > 0.06
        best_first = best_state = None; prev_gear = acts[-1][0] if acts else None; best_score = float('inf')
        for act1, _n1, traj1 in precomp_prim:
            ok1, ex1, ey1, eth1 = _apply(cx, cy, cth, traj1, X_FLOOR_SAFE)
            if not ok1: continue
            if lookahead_2:
                local = float('inf')
                for _a2, _n2, traj2 in precomp_prim:
                    ok2, ex2, ey2, eth2 = _apply(ex1, ey1, eth1, traj2, X_FLOOR_SAFE)
                    if ok2:
                        s2 = _score_y(ex2, ey2, eth2, gear=_a2[0], prev_gear=act1[0])
                        if s2 < local: local = s2
                if local < float('inf'):
                    cand = local + (0.0 if prev_gear is None or act1[0] == prev_gear else 1.0)
                else:
                    cand = _score_y(ex1, ey1, eth1, gear=act1[0], prev_gear=prev_gear)
            else:
                cand = _score_y(ex1, ey1, eth1, gear=act1[0], prev_gear=prev_gear)
            if cand < best_score:
                best_score = cand; best_first = act1; best_state = (ex1, ey1, eth1)
        if best_first is None: break
        cx, cy, cth = best_state; acts.append(best_first)
        curr_y_dist = _y_dist(cy)
        if curr_y_dist < best_abs_y - 1e-3: best_abs_y = curr_y_dist; stagnate_p1 = 0
        else:
            stagnate_p1 += 1
            if stagnate_p1 >= 15: break

    # Phase-1b
    if _y_dist(cy) > 0.02:
        best_abs_y = _y_dist(cy); stagnate_emg = 0
        for _ in range(60):
            if _check_goal(): return True, acts, cx, cy, cth
            best_first = best_state = None; prev_gear = acts[-1][0] if acts else None; best_score = float('inf')
            for act1, _n1, traj1 in precomp_prim:
                ok1, ex1, ey1, eth1 = _apply(cx, cy, cth, traj1, X_FLOOR_EMG)
                if not ok1: continue
                cand = _score_y(ex1, ey1, eth1, gear=act1[0], prev_gear=prev_gear, w_x=0.5)
                if cand < best_score:
                    best_score = cand; best_first = act1; best_state = (ex1, ey1, eth1)
            if best_first is None: break
            cx, cy, cth = best_state; acts.append(best_first)
            curr_y_dist = _y_dist(cy)
            if curr_y_dist < best_abs_y - 1e-3: best_abs_y = curr_y_dist; stagnate_emg = 0
            else:
                stagnate_emg += 1
                if stagnate_emg >= 15: break

    # Phase-2
    for _ in range(250):
        if _check_goal(): break
        best_rev = best_rev_st = None; prev_gear = acts[-1][0] if acts else None; best_rv = float('inf')
        for act, _n, traj in precomp_prim:
            if act[0] != 'R':
                if X_FLOOR_EMG + 0.5 <= cx < PREAPPROACH_X_MAX - 1.5: continue
            ok_r, ex_r, ey_r, eth_r = _apply(cx, cy, cth, traj, X_FLOOR_EMG)
            if not ok_r: continue
            if ex_r >= _x_ceil: continue
            x_lack = max(0.0, PREAPPROACH_X_MIN - ex_r) * 5.0
            y_min, y_max = _get_safe_y_range()
            if ey_r > y_max: y_over = (ey_r - y_max) * 10.0
            elif ey_r < y_min: y_over = (y_min - ey_r) * 10.0
            else: y_over = 0.0
            y_raw = 0.0
            if not (y_min <= ey_r <= y_max):
                y_raw = min(abs(ey_r - y_max), abs(ey_r - y_min)) * 2.0
            th_pen = max(0.0, abs(eth_r) - PREAPPROACH_TH_MAX) * 2.0
            if ey_r > 0 and eth_r > 0 and act[0] == 'R': th_pen += 2.0 * abs(eth_r)
            if ey_r < 0 and eth_r < 0 and act[0] == 'R': th_pen += 2.0 * abs(eth_r)
            gear_pen = 0.0 if prev_gear is None or act[0] == prev_gear else 2.0
            rv = x_lack + y_over + y_raw + th_pen + gear_pen
            if rv < best_rv:
                best_rv = rv; best_rev = act; best_rev_st = (ex_r, ey_r, eth_r)
        if best_rev is None: break
        cx, cy, cth = best_rev_st; acts.append(best_rev)

    if _check_goal(): return True, acts, cx, cy, cth
    return False, acts, cx, cy, cth


def plan_path_robust_obs(x0, y0, theta0, precomp_prim,
                         use_rs=False, stats=None, no_corridor=False,
                         rs_expansion_radius=2.5,
                         obstacles=None, dijkstra_grid=None):
    """
    鲁棒三级瀑布规划器（有障碍版）。

    Level-1: Pure RS 直达（仅 use_rs=True 时）
    Level-2: K-turn + A*（两阶段 + 逐障碍物穿越）
    Level-3: 单阶段 A* 兜底

    返回: (success, actions, rs_traj)
    """
    t_robust = time.perf_counter()
    planner_deadline = t_robust + 28.0  # leave 2s margin for the 30s test timeout

    if abs(y0) > MAX_PLANNABLE_Y:
        if stats is not None:
            stats['expanded'] = 0; stats['elapsed_ms'] = 0.0
            stats['two_stage'] = False; stats['out_of_range'] = True
        return False, None, None

    fast_obstacles = _preprocess_obstacles(obstacles)
    coll_fn = _make_collision_fn(no_corridor, fast_obstacles)

    # ── Goal reachability pre-check ──
    goal_valid, _ = coll_fn(RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH)
    if not goal_valid:
        if stats is not None:
            stats.update(expanded=0, elapsed_ms=0.0, two_stage=False,
                         level='FAILED_goal_blocked')
        return False, None, None

    if dijkstra_grid is None and use_rs and obstacles:
        dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.25)
        dijkstra_grid.build_map(obstacles, start_x=x0, start_y=y0)

    # ── Level-1: Pure RS ──
    if use_rs:
        rs_stats = {}
        ok, acts, traj = plan_path_pure_rs(x0, y0, theta0, collision_fn=coll_fn, stats=rs_stats)
        if ok:
            if stats is not None:
                stats.update(rs_stats); stats['level'] = 'L1_pure_rs'
            return True, acts, traj

    # Force two-stage when obstacles present
    needs_two_stage = True

    heuristic_fn = _make_heuristic_fn(use_rs, dijkstra_grid)
    rs_expand_fn = _make_rs_expand_fn(coll_fn, rs_expansion_radius) if use_rs else None

    if not needs_two_stage:
        st = {}
        ok, acts, traj = astar_core.plan_path(
            x0, y0, theta0, precomp_prim,
            collision_fn=coll_fn, heuristic_fn=heuristic_fn,
            rs_expand_fn=rs_expand_fn, stats=st,
            _rs_expand=use_rs, rs_expansion_radius=rs_expansion_radius)
        if stats is not None:
            stats.update(st); stats['two_stage'] = False
            stats['use_rs'] = use_rs; stats['no_corridor'] = no_corridor
        return ok, acts, traj

    # ── Level-2: Two-stage planning ──
    t1 = time.perf_counter()
    if stats is not None: stats['two_stage'] = True

    mx, my, mth = x0, y0, theta0
    phase0_acts = []

    # Phase-0: turnaround
    if abs(mth) > 2.6 or (abs(mth) > 1.57 and mth * my < 0):
        for _ in range(40):
            if abs(mth) <= M_PI / 2: break
            best_act = None; best_st = None; best_score = float('inf')
            restrict_F = (mx > 7.5)
            for act, _n, traj in precomp_prim:
                if restrict_F and act[0] == 'F': continue
                if abs(act[1]) < 0.5: continue
                cos_t, sin_t = math.cos(mth), math.sin(mth)
                ok_p = True; ex = ey = eth = 0.0
                for dx, dy, dth, cdth, sdth in traj:
                    ex = mx + dx * cos_t - dy * sin_t
                    ey = my + dx * sin_t + dy * cos_t
                    eth = mth + dth
                    if eth > M_PI: eth -= PI2
                    elif eth <= -M_PI: eth += PI2
                    sin_nth = sin_t * cdth + cos_t * sdth
                    valid, _ = check_collision(ex, ey, eth, sin_nth, no_corridor=no_corridor, obstacles=fast_obstacles)
                    if not valid: ok_p = False; break
                if not ok_p: continue
                score = abs(eth)
                if act[0] == 'F': score += 2.0 * ey * eth
                else: score -= 2.0 * ey * eth; score += 1.0
                if ex > 6.5: score += (ex - 6.5) * 5.0
                score += abs(ey) * 0.5
                if abs(act[1]) < 0.9: score += 10.0
                if act[0] == 'F': score -= ex * 1.5
                else: score += 2.0
                score += abs(ey - my) * 2.0
                score += (1.0 - act[2]) * 2.0
                score += abs(eth - mth) * 0.5
                if abs(ey) > 2.8: score += 20.0
                if eth > 0:
                    if act[0] == 'F' and act[1] > 0: score += 5.0
                    if act[0] == 'R' and act[1] < 0: score += 5.0
                else:
                    if act[0] == 'F' and act[1] < 0: score += 5.0
                    if act[0] == 'R' and act[1] > 0: score += 5.0
                if score < best_score:
                    best_score = score; best_act = act; best_st = (ex, ey, eth)
            if best_act is None: break
            phase0_acts.append(best_act); mx, my, mth = best_st

    # Stage-1
    ok1 = False; acts1 = []; stage1_mode = 'none'; stage1_goal_step_hit = None
    mx2, my2, mth2 = mx, my, mth; acts1_greedy = []
    obs_tight_y = 0.2 if fast_obstacles else None

    if phase0_acts or abs(my) > PREAPPROACH_Y_MAX or abs(mth) > 0.5 or fast_obstacles is not None:
        ok1_greedy, acts1_greedy, mx2, my2, mth2 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, target_y_max=obs_tight_y)
        if ok1_greedy and len(acts1_greedy) > 0:
            stage1_mode = 'greedy_kturn + astar'
            acts1.extend(acts1_greedy); mx, my, mth = mx2, my2, mth2

        # Stage-1 gets a tight budget so L1.8 (2D skeleton) has time to run
        stage1_deadline = min(time.perf_counter() + 8.0, planner_deadline)

        # Sequential obstacle clearance
        if fast_obstacles and (mx > 2.5 or abs(my) > PREAPPROACH_Y_MAX):
            blocking_obs = []
            for obs in fast_obstacles:
                if obs[0] < mx + 0.5 and obs[1] >= PREAPPROACH_X_MIN:
                    blocking_obs.append(obs)
            blocking_obs.sort(key=lambda o: o[0], reverse=True)

            sub_ok = True
            for sub_idx, (obs_min_x, obs_max_x, obs_min_y, obs_max_y) in enumerate(blocking_obs):
                sub_xmax = max(PREAPPROACH_X_MIN, obs_min_x - 0.3)
                if mx <= sub_xmax + 0.1: continue
                sub_ymin, sub_ymax = -3.5, 3.5
                sub_thmax = M_PI * 0.5
                dist_sub = max(0.0, mx - sub_xmax)
                expand_sub = 120000 + int(dist_sub * 30000)
                prim_sub = 80 + int(dist_sub / 0.05)
                step_sub = 3000 + int(dist_sub * 300)

                sub_dg = dijkstra_grid
                if sub_dg is None:
                    sub_dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.15, inflate_radius=0.7)
                    sub_dg.build_map(obstacles, start_x=mx, start_y=my)

                def _sub_preapproach_h(nx, ny, nth):
                    return preapproach_heuristic(nx, ny, nth,
                                                PREAPPROACH_X_MIN, sub_xmax,
                                                sub_ymin, sub_ymax, sub_thmax,
                                                dijkstra_grid=sub_dg)

                st_sub = {}
                ok_sub, acts_sub, _ = astar_core.plan_path(
                    mx, my, mth, precomp_prim,
                    collision_fn=coll_fn, heuristic_fn=_sub_preapproach_h,
                    stats=st_sub,
                    _goal_xmin=PREAPPROACH_X_MIN, _goal_xmax=sub_xmax,
                    _goal_ymin=sub_ymin, _goal_ymax=sub_ymax, _goal_thmax=sub_thmax,
                    _step_limit=step_sub, _expand_limit=expand_sub, _prim_limit=prim_sub,
                    _rs_expand=False, deadline=stage1_deadline)
                if ok_sub:
                    acts1.extend(acts_sub or [])
                    gp = st_sub.get('goal_pos')
                    if gp: mx, my, mth = gp
                    else: mx, my, mth = _replay_to_end(mx, my, mth, acts_sub, precomp_prim)
                    stage1_goal_step_hit = st_sub.get('goal_step_hit')
                else:
                    sub_ok = False; break

            if sub_ok:
                ok1 = True; stage1_mode = 'sequential_obs_clear'

        if not ok1 and (mx > 2.5 or abs(my) > PREAPPROACH_Y_MAX or fast_obstacles):
            st1 = {}
            eff_y_bound = obs_tight_y if obs_tight_y is not None else PREAPPROACH_Y_MAX
            safe_y_min, safe_y_max = -eff_y_bound, eff_y_bound
            dynamic_xmax = PREAPPROACH_X_MAX
            if fast_obstacles:
                for min_x, max_x, min_y, max_y in fast_obstacles:
                    if max_x >= PREAPPROACH_X_MIN and min_x <= max(mx, 4.0):
                        dynamic_xmax = min(dynamic_xmax, max(PREAPPROACH_X_MIN, min_x - 0.3))
            dist_x_st1 = max(0.0, mx - dynamic_xmax)
            prim_limit_st1 = 50 + int(dist_x_st1 / 0.08)
            base_expand = 80000 if phase0_acts or abs(mth) > 0.5 else 40000
            expand_limit_st1 = base_expand + int(dist_x_st1 * 15000)
            step_limit_st1 = 2000 + int(dist_x_st1 * 200)

            def _st1_h(nx, ny, nth):
                return preapproach_heuristic(nx, ny, nth,
                                            PREAPPROACH_X_MIN, dynamic_xmax,
                                            safe_y_min, safe_y_max,
                                            PREAPPROACH_TH_MAX,
                                            dijkstra_grid=dijkstra_grid)

            ok1, acts1_astar, _ = astar_core.plan_path(
                mx, my, mth, precomp_prim,
                collision_fn=coll_fn, heuristic_fn=_st1_h,
                stats=st1,
                _goal_xmin=PREAPPROACH_X_MIN, _goal_xmax=dynamic_xmax,
                _goal_ymin=safe_y_min, _goal_ymax=safe_y_max,
                _goal_thmax=PREAPPROACH_TH_MAX,
                _step_limit=step_limit_st1, _expand_limit=expand_limit_st1,
                _prim_limit=prim_limit_st1, _rs_expand=False,
                deadline=stage1_deadline)
            if ok1:
                acts1.extend(acts1_astar or [])
                goal_pos = st1.get('goal_pos')
                stage1_goal_step_hit = st1.get('goal_step_hit')
                if goal_pos is not None: mx, my, mth = goal_pos
                else: mx, my, mth = _replay_to_end(mx, my, mth, acts1_astar, precomp_prim)
                stage1_mode = 'astar_preapproach'
            else:
                improved_enough = (abs(my2) <= PREAPPROACH_Y_MAX + 0.8 and
                                   abs(my2) <= abs(y0) - 0.08 and abs(mth2) <= 1.2)
                if improved_enough and acts1_greedy:
                    ok1 = True; stage1_mode = 'greedy_kturn_partial'
                    acts1 = acts1_greedy; mx, my, mth = mx2, my2, mth2

    t1_ms = round((time.perf_counter() - t1) * 1000.0, 1)

    # Stage-1.5
    t15_start = time.perf_counter(); stage15_acts = []
    s15_x_ceil = None
    if fast_obstacles:
        for obs_min_x, obs_max_x, obs_min_y, obs_max_y in fast_obstacles:
            if mx < obs_min_x:
                cand = obs_min_x - 0.1
                if s15_x_ceil is None or cand < s15_x_ceil: s15_x_ceil = cand
    if abs(my) > 0.15:
        ok15, acts15, mx15, my15, mth15 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, ignore_obs_for_y_range=True,
            target_y_max=obs_tight_y, x_ceil=s15_x_ceil)
        if ok15 and acts15:
            stage15_acts = acts15; mx, my, mth = mx15, my15, mth15
    t15_ms = round((time.perf_counter() - t15_start) * 1000.0, 1)
    if stats is not None: stats['stage15_ms'] = t15_ms

    # ── Level-1.8: 2D skeleton + RS stitching (same as v2) ──
    prefix_acts = phase0_acts + acts1 + stage15_acts
    prefix_too_many_shifts = _count_gear_shifts(prefix_acts) > 8

    if bool(obstacles) and dijkstra_grid is not None:
        l18_deadline = min(time.perf_counter() + 15.0, planner_deadline)
        if prefix_too_many_shifts:
            starts_2d = [(x0, y0, theta0, [])]
        else:
            starts_2d = [(mx, my, mth, prefix_acts)]
            if (mx, my) != (x0, y0):
                starts_2d.append((x0, y0, theta0, []))
        for sx2, sy2, sth2, pre_acts in starts_2d:
            if time.perf_counter() > l18_deadline:
                break
            for spc in [2.5, 1.5, 1.0]:
                if time.perf_counter() > l18_deadline:
                    break
                fb2d_ok, fb2d_traj = plan_2d_fallback(
                    dijkstra_grid, sx2, sy2, sth2, coll_fn,
                    spacing=spc, max_rs_paths=12, obstacles=obstacles,
                    deadline=l18_deadline)
                if fb2d_ok and not _path_goes_behind_wall(fb2d_traj):
                    total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
                    if stats is not None:
                        stats.update(
                            expanded=0, elapsed_ms=total_ms,
                            level=f'L1_8_2d_skeleton_sp{spc}',
                            use_rs=use_rs, no_corridor=no_corridor,
                            two_stage=True, stage1_ms=t1_ms,
                        )
                    return True, pre_acts, fb2d_traj

    if not ok1:
        st_fb = {}
        ok_fb, acts_fb, traj_fb = astar_core.plan_path(
            x0, y0, theta0, precomp_prim,
            collision_fn=coll_fn, heuristic_fn=heuristic_fn,
            rs_expand_fn=rs_expand_fn, stats=st_fb,
            _rs_expand=use_rs, rs_expansion_radius=rs_expansion_radius,
            deadline=planner_deadline)
        if stats is not None:
            stats.update(st_fb); stats['two_stage'] = True; stats['stage1_ms'] = t1_ms
            stats['fail_reason'] = 'Stage1-Fail-Fallback'
            stats['elapsed_ms'] = round((time.perf_counter() - t_robust) * 1000.0, 1)
            stats['use_rs'] = use_rs; stats['no_corridor'] = no_corridor; stats['level'] = 'L3_fallback'
        return ok_fb, acts_fb, traj_fb

    # Stage-2
    st2 = {}
    dist_x = max(0.0, mx - 2.1)
    dynamic_prim_limit = max(150, 30 + int(dist_x / 0.1))
    stage2_deadline = min(time.perf_counter() + 10.0, planner_deadline)
    ok2, acts2, rs_traj = astar_core.plan_path(
        mx, my, mth, precomp_prim,
        collision_fn=coll_fn, heuristic_fn=heuristic_fn,
        rs_expand_fn=rs_expand_fn, stats=st2,
        _goal_xmin=0.5, _goal_xmax=8.0,
        _prim_limit=dynamic_prim_limit,
        _rs_expand=use_rs, rs_expansion_radius=max(rs_expansion_radius, 4.0),
        deadline=stage2_deadline)

    total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)

    if ok2:
        final_acts = phase0_acts + (acts1 or []) + stage15_acts + (acts2 or [])
        if stats is not None:
            stats['expanded'] = st2.get('expanded', 0); stats['elapsed_ms'] = total_ms
            stats['use_rs'] = use_rs; stats['no_corridor'] = no_corridor
            stats['rs_expansion'] = st2.get('rs_expansion', False); stats['two_stage'] = True
            stats['stage1_acts'] = len(acts1 or []) + len(phase0_acts)
            stats['stage2_acts'] = len(acts2 or []); stats['stage1_ms'] = t1_ms
            stats['stage2_ms'] = st2.get('elapsed_ms', 0)
            stats['stage1_mode'] = stage1_mode if not phase0_acts else 'phase0+' + stage1_mode
            stats['stage1_end'] = (mx, my, mth)
            stats['stage1_goal_step_hit'] = stage1_goal_step_hit
            stats['goal_pos'] = st2.get('goal_pos'); stats['goal_step_hit'] = st2.get('goal_step_hit')
            stats['level'] = 'L2_two_stage'
        return True, final_acts, rs_traj

    # Level-3 fallback
    st_fb2 = {}
    ok_fb, acts_fb, traj_fb = astar_core.plan_path(
        x0, y0, theta0, precomp_prim,
        collision_fn=coll_fn, heuristic_fn=heuristic_fn,
        rs_expand_fn=rs_expand_fn, stats=st_fb2,
        _expand_limit=150000, _prim_limit=max(150, dynamic_prim_limit),
        _rs_expand=use_rs, rs_expansion_radius=rs_expansion_radius,
        deadline=planner_deadline)
    if stats is not None:
        stats.update(st_fb2); stats['two_stage'] = True; stats['stage1_ms'] = t1_ms
        stats['stage2_ms'] = st2.get('elapsed_ms', 0); stats['fail_reason'] = 'Stage2-Fail-Fallback'
        stats['elapsed_ms'] = round((time.perf_counter() - t_robust) * 1000.0, 1)
        stats['use_rs'] = use_rs; stats['no_corridor'] = no_corridor; stats['level'] = 'L3_fallback'
    return ok_fb, acts_fb, traj_fb
