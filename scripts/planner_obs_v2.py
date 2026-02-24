"""
planner_obs_v2.py — Enhanced obstacle planner v2 (no modifications to original files).

Key improvements over v1 (planner_obs.py):
  1. RS multi-candidate expansion: tries up to N RS paths when shortest collides
  2. Progress-aware heuristic: soft D_goal gate reduces wasted node expansion
  3. Multi-stage recovery: escalates parameters across fast → robust → aggressive
"""

import math
import time
import rs
from primitives import (
    DT, M_PI, PI2, ALIGN_GOAL_DYAW, MIN_TURN_RADIUS,
    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
    PREAPPROACH_Y_MAX, MAX_PLANNABLE_Y,
)
from collision import check_collision
from rs_utils import plan_path_pure_rs
from planner_obs import _preprocess_obstacles, _make_collision_fn, _k_turn_preposition_obs
from heuristic import DijkstraGrid
from fallback_2d import plan_2d_fallback
import astar_core


# ═══════════════════════════════════════════════════════════════════════
# Improvement 1: RS multi-candidate expansion
# ═══════════════════════════════════════════════════════════════════════

def _make_rs_expand_fn_multi(collision_fn, rs_expansion_radius, max_paths=8):
    """
    RS expansion that tries multiple candidate paths (sorted by length).
    Rejects paths that go behind the pallet wall or take unreasonable detours.
    """
    def rs_expand_fn(cx, cy, cth, cost, path_node, expanded, t_start, stats):
        euclidean_goal = math.hypot(cx - RS_GOAL_X, cy - RS_GOAL_Y)
        if euclidean_goal >= rs_expansion_radius + 1.0:
            return None

        trajs = rs.rs_sample_path_multi(
            cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
            MIN_TURN_RADIUS, step=DT * 0.5, max_paths=max_paths)

        for traj in trajs:
            if not traj:
                continue
            ex, ey, eth = traj[-1]
            if not (ex <= 2.25 and abs(ey) <= 0.18 and abs(eth) <= ALIGN_GOAL_DYAW):
                continue
            if _path_goes_behind_wall(traj):
                continue
            if _path_is_unreasonable(traj, cx, cy):
                continue
            traj_ok = True
            for pt in traj:
                valid, _ = collision_fn(pt[0], pt[1], pt[2])
                if not valid:
                    traj_ok = False
                    break
            if traj_ok:
                final_path = []
                curr = path_node
                while curr is not None:
                    final_path.append(curr[1])
                    curr = curr[0]
                final_path.reverse()
                if stats is not None:
                    stats['expanded'] = expanded
                    stats['elapsed_ms'] = round((time.perf_counter() - t_start) * 1000.0, 1)
                    stats['rs_expansion'] = True
                return True, final_path, traj
        return None
    return rs_expand_fn


# ═══════════════════════════════════════════════════════════════════════
# Improvement 1b: Milestone-based RS bypass (inspired by obc_alg)
# ═══════════════════════════════════════════════════════════════════════

def _generate_bypass_milestones(sx, sy, sth, obstacles, clearance=1.0):
    """
    Generate a dense grid of candidate waypoint poses around each obstacle.
    Focuses on lateral offsets large enough for RS turning radius (~2.1m).
    """
    milestones = []
    if not obstacles:
        return milestones

    for obs in obstacles:
        if isinstance(obs, dict):
            ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
            min_x, max_x = min(ox, ox + ow), max(ox, ox + ow)
            min_y, max_y = min(oy, oy + oh), max(oy, oy + oh)
        elif isinstance(obs, tuple) and len(obs) == 4:
            min_x, max_x, min_y, max_y = obs
        else:
            continue

        mid_x = (min_x + max_x) * 0.5
        x_vals = [
            max_x - 0.5, max_x, max_x + 0.5, max_x + 1.0,
            mid_x, min_x + 0.5, min_x,
        ]
        x_vals = [x for x in x_vals if 0.5 < x < 8.5]

        for y_sign in [1.0, -1.0]:
            edge_y = max_y if y_sign > 0 else min_y
            y_offsets = [1.0, 1.4, 1.8, 2.2]
            for y_off in y_offsets:
                wp_y = edge_y + y_sign * y_off
                if abs(wp_y) > 3.0:
                    continue
                for wp_x in x_vals:
                    milestones.append((wp_x, wp_y, 0.0, 'grid'))

    return milestones


def _check_traj_collision(traj, collision_fn):
    """Check if an RS trajectory is collision-free."""
    for pt in traj:
        valid, _ = collision_fn(pt[0], pt[1], pt[2])
        if not valid:
            return False
    return True


_WALL_X = 1.92
_WALL_CORRIDOR_Y = 0.5
_MAX_BEHIND_WALL_PTS = 5


def _path_goes_behind_wall(traj):
    """Reject paths that route behind the pallet wall (x < 1.92, |y| > 0.5).
    Brief dips (fewer than _MAX_BEHIND_WALL_PTS points) are tolerated
    for near-goal RS curve arcs.
    """
    if not traj:
        return False
    count = 0
    for pt in traj:
        if pt[0] < _WALL_X and abs(pt[1]) > _WALL_CORRIDOR_Y:
            count += 1
            if count > _MAX_BEHIND_WALL_PTS:
                return True
    return False


_PATH_LENGTH_RATIO_MAX = 3.0
_PATH_LATERAL_EXTRA_MAX = 2.0


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


def _path_is_unreasonable(traj, sx, sy):
    """Reject paths with excessive length or lateral deviation.
    Catches L1/L1.5 arcs that technically avoid the wall but detour wildly.
    """
    if not traj or len(traj) < 3:
        return False
    direct = math.hypot(sx - RS_GOAL_X, sy - RS_GOAL_Y)
    # 改进：如果起终点极近（如 0.1m），直接距离很小会导致分母爆炸，引入 2.0 的下限保护
    eff_direct = max(direct, 2.0)
    traj_len = 0.0
    max_abs_y = abs(sy)
    for i in range(1, len(traj)):
        traj_len += math.hypot(traj[i][0] - traj[i - 1][0],
                               traj[i][1] - traj[i - 1][1])
        ay = abs(traj[i][1])
        if ay > max_abs_y:
            max_abs_y = ay
    if traj_len > eff_direct * _PATH_LENGTH_RATIO_MAX:
        return True
    y_bound = max(abs(sy), abs(RS_GOAL_Y)) + _PATH_LATERAL_EXTRA_MAX
    if max_abs_y > y_bound:
        return True
    return False


def _try_milestone_rs_bypass(sx, sy, sth, obstacles, collision_fn,
                              fast_obstacles=None, no_corridor=False,
                              max_paths=8, max_milestones=20):
    """
    Try multi-segment RS paths through milestone waypoints to bypass obstacles.
    Uses relaxed collision (no corridor) for mid-segments, strict for final segment.
    Returns (success, trajectory) or (False, None).
    """
    milestones = _generate_bypass_milestones(sx, sy, sth, obstacles)
    if not milestones:
        return False, None

    coll_relaxed = collision_fn

    step = DT * 0.5
    best_traj = None
    best_len = float('inf')

    # --- Try 2-segment: start → wp → goal ---
    for mx, my, mth, _tag in milestones[:max_milestones]:
        seg1_list = rs.rs_sample_path_multi(
            sx, sy, sth, mx, my, mth,
            MIN_TURN_RADIUS, step=step, max_paths=max_paths)

        for seg1 in seg1_list:
            if not seg1 or not _check_traj_collision(seg1, coll_relaxed):
                continue

            seg2_list = rs.rs_sample_path_multi(
                mx, my, mth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                MIN_TURN_RADIUS, step=step, max_paths=max_paths)

            for seg2 in seg2_list:
                if not seg2 or not _check_traj_collision(seg2, collision_fn):
                    continue
                ex, ey, eth = seg2[-1]
                if not (ex <= 2.25 and abs(ey) <= 0.18
                        and abs(eth) <= ALIGN_GOAL_DYAW):
                    continue
                total_len = len(seg1) + len(seg2)
                if total_len < best_len:
                    best_len = total_len
                    best_traj = seg1 + seg2

    if best_traj is not None:
        return True, best_traj

    # --- Try 3-segment: start → wp_bypass → wp_return → goal ---
    return_wps = [
        (2.8, 0.0, 0.0), (3.0, 0.0, 0.0), (3.2, 0.0, 0.0),
        (2.5, 0.0, 0.0), (2.8, 0.15, 0.0), (2.8, -0.15, 0.0),
    ]
    for mx, my, mth, _tag in milestones[:max_milestones]:
        seg1_list = rs.rs_sample_path_multi(
            sx, sy, sth, mx, my, mth,
            MIN_TURN_RADIUS, step=step, max_paths=6)
        seg1_ok = [s for s in seg1_list
                   if s and _check_traj_collision(s, coll_relaxed)]
        if not seg1_ok:
            continue
        for rwx, rwy, rwth in return_wps:
            seg2_list = rs.rs_sample_path_multi(
                mx, my, mth, rwx, rwy, rwth,
                MIN_TURN_RADIUS, step=step, max_paths=12)
            for seg2 in seg2_list:
                if not seg2 or not _check_traj_collision(seg2, coll_relaxed):
                    continue
                seg3_list = rs.rs_sample_path_multi(
                    rwx, rwy, rwth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                    MIN_TURN_RADIUS, step=step, max_paths=5)
                for seg3 in seg3_list:
                    if not seg3 or not _check_traj_collision(seg3, collision_fn):
                        continue
                    ex, ey, eth = seg3[-1]
                    if not (ex <= 2.25 and abs(ey) <= 0.18
                            and abs(eth) <= ALIGN_GOAL_DYAW):
                        continue
                    total_len = len(seg1_ok[0]) + len(seg2) + len(seg3)
                    if total_len < best_len:
                        best_len = total_len
                        best_traj = seg1_ok[0] + seg2 + seg3

    if best_traj is not None:
        return True, best_traj
    return False, None


# ═══════════════════════════════════════════════════════════════════════
# Improvement 2: Progress-aware heuristic
# ═══════════════════════════════════════════════════════════════════════

def _make_heuristic_fn_v2(dijkstra_grid, start_d_goal,
                          allow_uphill=2.0, h_weight=3.0):
    """
    RS + Dijkstra heuristic with soft D_goal progress gate.
    Nodes farther from goal than (start + allow_uphill) get prohibitive h.
    h_weight controls greediness: higher = more greedy, faster but less optimal.
    """
    def h_fn(nx, ny, nth):
        h_grid_dist, pure_dist = dijkstra_grid.get_heuristic(nx, ny, nth)

        if (pure_dist != float('inf') and start_d_goal != float('inf')
                and allow_uphill < 50.0):
            if pure_dist > start_d_goal + allow_uphill:
                return 1e6, 1.0

        h_grid = h_grid_dist / 0.25

        euclidean = math.hypot(nx - RS_GOAL_X, ny - RS_GOAL_Y)
        if euclidean < 4.0 and dijkstra_grid.line_of_sight(nx, ny, RS_GOAL_X, RS_GOAL_Y):
            rs_dist = rs.rs_distance_pose(
                nx, ny, nth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
            h_rs = rs_dist / 0.25
            h = max(h_rs, h_grid * 1.2)
        else:
            h = h_grid * 1.5

        dy_err = abs(ny - RS_GOAL_Y)
        dx_err = nx - max(RS_GOAL_X, 2.0)
        th_err = abs(nth - RS_GOAL_TH)
        if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
            needed_x = dy_err * 2.0 + th_err * 1.5
            if dx_err < needed_x:
                h += min(50.0, (needed_x - dx_err) * 10.0)

        return h, h_weight
    return h_fn


# ═══════════════════════════════════════════════════════════════════════
# Improvement 3: Multi-stage recovery configuration
# ═══════════════════════════════════════════════════════════════════════

RECOVERY_STAGES = [
    {
        'name': 'fast',
        'expand_limit': 8000,
        'prim_limit': 80,
        'rs_radius': 4.0,
        'allow_uphill': 2.0,
        'max_rs_paths': 6,
        'h_weight': 4.0,
    },
    {
        'name': 'robust',
        'expand_limit': 25000,
        'prim_limit': 100,
        'rs_radius': 6.0,
        'allow_uphill': 6.0,
        'max_rs_paths': 10,
        'h_weight': 3.0,
    },
    {
        'name': 'aggressive',
        'expand_limit': 80000,
        'prim_limit': 120,
        'rs_radius': 10.0,
        'allow_uphill': 100.0,
        'max_rs_paths': 15,
        'h_weight': 2.0,
    },
]


# ═══════════════════════════════════════════════════════════════════════
# Phase-0 turnaround (copied from planner_obs.py — ~30 lines)
# ═══════════════════════════════════════════════════════════════════════

def _phase0_turnaround(mx, my, mth, precomp_prim, no_corridor, fast_obstacles):
    """Turn vehicle roughly toward goal when starting nearly backwards."""
    acts = []
    for _ in range(40):
        if abs(mth) <= M_PI / 2:
            break
        best_act = best_st = None
        best_score = float('inf')
        restrict_F = (mx > 7.5)
        for act, _n, traj in precomp_prim:
            if restrict_F and act[0] == 'F':
                continue
            if abs(act[1]) < 0.5:
                continue
            cos_t, sin_t = math.cos(mth), math.sin(mth)
            ok_p = True
            ex = ey = eth = 0.0
            for dx, dy, dth, cdth, sdth in traj:
                ex = mx + dx * cos_t - dy * sin_t
                ey = my + dx * sin_t + dy * cos_t
                eth = mth + dth
                if eth > M_PI: eth -= PI2
                elif eth <= -M_PI: eth += PI2
                sin_nth = sin_t * cdth + cos_t * sdth
                valid, _ = check_collision(ex, ey, eth, sin_nth,
                                           no_corridor=no_corridor,
                                           obstacles=fast_obstacles)
                if not valid:
                    ok_p = False
                    break
            if not ok_p:
                continue
            score = abs(eth)
            if act[0] == 'F':
                score += 2.0 * ey * eth
            else:
                score -= 2.0 * ey * eth
                score += 1.0
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
                best_score = score
                best_act = act
                best_st = (ex, ey, eth)
        if best_act is None:
            break
        acts.append(best_act)
        mx, my, mth = best_st
    return acts, mx, my, mth


# ═══════════════════════════════════════════════════════════════════════
# Main planner v2
# ═══════════════════════════════════════════════════════════════════════

def plan_path_robust_obs_v2(x0, y0, theta0, precomp_prim,
                             use_rs=True, stats=None, no_corridor=False,
                             rs_expansion_radius=2.5, obstacles=None):
    """
    Enhanced obstacle planner v2 with multi-stage recovery.
    Drop-in replacement for plan_path_robust_obs.
    """
    t_robust = time.perf_counter()
    if stats is None:
        stats = {}

    if abs(y0) > MAX_PLANNABLE_Y:
        stats.update(expanded=0, elapsed_ms=0.0, two_stage=False, out_of_range=True)
        return False, None, None

    fast_obstacles = _preprocess_obstacles(obstacles)
    coll_fn = _make_collision_fn(no_corridor, fast_obstacles)

    goal_valid, _ = coll_fn(RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH)
    if not goal_valid:
        stats.update(expanded=0, elapsed_ms=0.0, two_stage=False,
                     level='FAILED_goal_blocked')
        return False, None, None

    dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
    dijkstra_grid.build_map(obstacles, start_x=x0, start_y=y0)
    _, start_d_goal = dijkstra_grid.get_heuristic(x0, y0)

    # ── Level-1: Pure RS ──
    if use_rs:
        rs_stats = {}
        ok, acts, traj = plan_path_pure_rs(x0, y0, theta0,
                                           collision_fn=coll_fn, stats=rs_stats)
        if ok and not _path_goes_behind_wall(traj) \
                and not _path_is_unreasonable(traj, x0, y0):
            stats.update(rs_stats)
            stats['level'] = 'L1_pure_rs'
            return True, acts, traj

    # ── Level-1.5: Milestone RS bypass ──
    if use_rs and obstacles:
        bypass_ok, bypass_traj = _try_milestone_rs_bypass(
            x0, y0, theta0, obstacles, coll_fn,
            fast_obstacles=fast_obstacles, no_corridor=no_corridor,
            max_paths=10, max_milestones=20)
        if bypass_ok and not _path_goes_behind_wall(bypass_traj) \
                and not _path_is_unreasonable(bypass_traj, x0, y0):
            total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
            stats.update(
                expanded=0, elapsed_ms=total_ms,
                level='L1_5_milestone_rs', use_rs=True,
                no_corridor=no_corridor,
            )
            return True, [], bypass_traj

    # ── Preprocessing: Phase-0 turnaround + K-turn ──
    stats['two_stage'] = True
    t1 = time.perf_counter()

    mx, my, mth = x0, y0, theta0
    phase0_acts = []

    if abs(mth) > 2.6 or (abs(mth) > 1.57 and mth * my < 0):
        phase0_acts, mx, my, mth = _phase0_turnaround(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles)

    obs_tight_y = 0.2 if fast_obstacles else None
    acts1 = []
    if phase0_acts or abs(my) > PREAPPROACH_Y_MAX or abs(mth) > 0.5 or fast_obstacles is not None:
        ok1_greedy, acts1_greedy, mx2, my2, mth2 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, target_y_max=obs_tight_y)
        if ok1_greedy and acts1_greedy:
            acts1.extend(acts1_greedy)
            mx, my, mth = mx2, my2, mth2

    stage15_acts = []
    if abs(my) > 0.15:
        s15_x_ceil = None
        if fast_obstacles:
            for obs_min_x, *_ in fast_obstacles:
                if mx < obs_min_x:
                    cand = obs_min_x - 0.1
                    if s15_x_ceil is None or cand < s15_x_ceil:
                        s15_x_ceil = cand
        ok15, acts15, mx15, my15, mth15 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, ignore_obs_for_y_range=True,
            target_y_max=obs_tight_y, x_ceil=s15_x_ceil)
        if ok15 and acts15:
            stage15_acts = acts15
            mx, my, mth = mx15, my15, mth15

    t1_ms = round((time.perf_counter() - t1) * 1000.0, 1)
    stats['stage1_ms'] = t1_ms

    prefix_acts = phase0_acts + acts1 + stage15_acts
    prefix_too_many_shifts = _count_gear_shifts(prefix_acts) > 8

    # ── Level-1.8: 2D skeleton + RS stitching (fast, before A*) ──
    if obstacles and not prefix_too_many_shifts:
        coll_relaxed = coll_fn
        starts_2d = [(mx, my, mth, prefix_acts)]
        if (mx, my) != (x0, y0):
            starts_2d.append((x0, y0, theta0, []))
        for sx2, sy2, sth2, prefix_acts in starts_2d:
            for spc in [2.5, 1.5]:
                fb2d_ok, fb2d_traj = plan_2d_fallback(
                    dijkstra_grid, sx2, sy2, sth2, coll_relaxed,
                    spacing=spc, max_rs_paths=12, obstacles=obstacles)
                if fb2d_ok and not _path_goes_behind_wall(fb2d_traj) and not _path_is_unreasonable(fb2d_traj, sx2, sy2):
                    total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
                    stats.update(
                        expanded=0, elapsed_ms=total_ms,
                        level=f'L1_8_2d_skeleton_sp{spc}',
                        use_rs=use_rs, no_corridor=no_corridor,
                        two_stage=True, stage1_ms=t1_ms,
                    )
                    return True, prefix_acts, fb2d_traj

    # ── Level-2: Multi-stage A* from K-turn position ──
    _, mid_d_goal = dijkstra_grid.get_heuristic(mx, my)

    if not prefix_too_many_shifts:
        for stage_idx, stage in enumerate(RECOVERY_STAGES):
            st = {}
            rs_expand = _make_rs_expand_fn_multi(
                coll_fn, max(rs_expansion_radius, stage['rs_radius']),
                max_paths=stage['max_rs_paths']) if use_rs else None

            heuristic = _make_heuristic_fn_v2(
                dijkstra_grid, mid_d_goal,
                allow_uphill=stage['allow_uphill'], h_weight=stage['h_weight'])

            dist_x = max(0.0, mx - 2.1)
            dyn_prim = max(stage['prim_limit'], 30 + int(dist_x / 0.1))

            ok2, acts2, rs_traj = astar_core.plan_path(
                mx, my, mth, precomp_prim,
                collision_fn=coll_fn, heuristic_fn=heuristic,
                rs_expand_fn=rs_expand, stats=st,
                _goal_xmin=0.5, _goal_xmax=8.0,
                _prim_limit=dyn_prim,
                _expand_limit=stage['expand_limit'],
                _rs_expand=use_rs,
                rs_expansion_radius=max(rs_expansion_radius, stage['rs_radius']))

            if ok2:
                if rs_traj is None and not st.get('rs_expansion', False):
                    gp = st.get('goal_pos')
                    if gp and obstacles:
                        gx, gy, gth = gp
                        in_narrow_goal = (gx <= 2.25 and abs(gy) <= 0.18
                                          and abs(gth) <= ALIGN_GOAL_DYAW)
                        if not in_narrow_goal:
                            verify = rs.rs_sample_path_multi(
                                gx, gy, gth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                                MIN_TURN_RADIUS, step=DT * 0.5, max_paths=10)
                            verified = False
                            for vt in verify:
                                if vt and _check_traj_collision(vt, coll_fn):
                                    ex, ey, eth = vt[-1]
                                    if (ex <= 2.25 and abs(ey) <= 0.18
                                            and abs(eth) <= ALIGN_GOAL_DYAW):
                                        rs_traj = vt
                                        verified = True
                                        break
                            if not verified:
                                continue

                final_acts = phase0_acts + acts1 + stage15_acts + (acts2 or [])
                total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
                stats.update(
                    expanded=st.get('expanded', 0), elapsed_ms=total_ms,
                    use_rs=use_rs, no_corridor=no_corridor,
                    rs_expansion=st.get('rs_expansion', False),
                    two_stage=True, stage1_ms=t1_ms,
                    stage1_end=(mx, my, mth),
                    goal_pos=st.get('goal_pos'),
                    level=f'L2_{stage["name"]}',
                    recovery_stage=stage_idx,
                )
                return True, final_acts, rs_traj

    # ── Level-3: Fallback from original start (robust + aggressive only) ──
    for stage_idx, stage in enumerate(RECOVERY_STAGES[1:], 1):
        st_fb = {}
        rs_expand = _make_rs_expand_fn_multi(
            coll_fn, max(rs_expansion_radius, stage['rs_radius']),
            max_paths=stage['max_rs_paths']) if use_rs else None

        heuristic = _make_heuristic_fn_v2(
            dijkstra_grid, start_d_goal,
            allow_uphill=stage['allow_uphill'], h_weight=stage['h_weight'])

        ok_fb, acts_fb, traj_fb = astar_core.plan_path(
            x0, y0, theta0, precomp_prim,
            collision_fn=coll_fn, heuristic_fn=heuristic,
            rs_expand_fn=rs_expand, stats=st_fb,
            _expand_limit=stage['expand_limit'],
            _prim_limit=stage['prim_limit'],
            _rs_expand=use_rs,
            rs_expansion_radius=max(rs_expansion_radius, stage['rs_radius']))

        if ok_fb:
            if traj_fb is None and not st_fb.get('rs_expansion', False):
                gp = st_fb.get('goal_pos')
                if gp and obstacles:
                    gx, gy, gth = gp
                    in_narrow_goal = (gx <= 2.25 and abs(gy) <= 0.18
                                      and abs(gth) <= ALIGN_GOAL_DYAW)
                    if not in_narrow_goal:
                        verify = rs.rs_sample_path_multi(
                            gx, gy, gth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                            MIN_TURN_RADIUS, step=DT * 0.5, max_paths=10)
                        verified = False
                        for vt in verify:
                            if vt and _check_traj_collision(vt, coll_fn):
                                ex, ey, eth = vt[-1]
                                if (ex <= 2.25 and abs(ey) <= 0.18
                                        and abs(eth) <= ALIGN_GOAL_DYAW):
                                    traj_fb = vt
                                    verified = True
                                    break
                        if not verified:
                            continue

            total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
            stats.update(st_fb)
            stats.update(
                two_stage=True, stage1_ms=t1_ms, elapsed_ms=total_ms,
                use_rs=use_rs, no_corridor=no_corridor,
                level=f'L3_fb_{stage["name"]}',
                recovery_stage=stage_idx,
            )
            return True, acts_fb, traj_fb

    # All stages exhausted
    total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
    stats.update(
        expanded=0, elapsed_ms=total_ms, two_stage=True, stage1_ms=t1_ms,
        use_rs=use_rs, no_corridor=no_corridor, level='FAILED_all_stages',
    )
    return False, None, None
