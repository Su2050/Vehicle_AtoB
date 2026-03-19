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
import primitives
from primitives import (
    DT, M_PI, PI2, ALIGN_GOAL_DYAW, MIN_TURN_RADIUS,
    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
    PREAPPROACH_Y_MAX, MAX_PLANNABLE_Y, simulate_path,
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

def _make_rs_expand_fn_multi(collision_fn, rs_expansion_radius, max_paths=8,
                             dijkstra_grid=None):
    """
    RS expansion that tries multiple candidate paths (sorted by length).
    Rejects paths that go behind the pallet wall or take unreasonable detours.
    Uses Dijkstra line-of-sight to skip expensive RS computation when blocked.
    """
    def rs_expand_fn(cx, cy, cth, cost, path_node, expanded, t_start, stats):
        euclidean_goal = math.hypot(cx - RS_GOAL_X, cy - RS_GOAL_Y)
        if euclidean_goal >= rs_expansion_radius + 1.0:
            return None

        # Fast pre-check: if obstacle blocks line-of-sight to goal, skip RS
        # (RS paths will almost certainly collide with the obstacle)
        if dijkstra_grid is not None and euclidean_goal > 1.5:
            if not dijkstra_grid.line_of_sight(cx, cy, RS_GOAL_X, RS_GOAL_Y):
                return None

        trajs = rs.rs_sample_path_multi(
            cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
            MIN_TURN_RADIUS, step=0.1, max_paths=max_paths, collision_fn=collision_fn)

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
    Generate candidate waypoint poses around each obstacle for L1.5 bypass.

    Exp-6B enhancements over original:
      - Goal-side x positions (min_x - 0.5, min_x - 1.0) for S03/S11 scenarios
        where the vehicle must route around the obstacle toward the goal.
      - Heading variants pointing toward goal for each milestone position,
        enabling RS connections that the default th=0 cannot achieve.
      - Wider y offsets (1.0, 1.5, 2.0, 2.5) and raised y limit (4.5)
        to cover more detour geometries.
    """
    milestones_primary = []   # th=0 milestones (most likely to succeed)
    milestones_heading = []   # heading-variant milestones (backup)
    if not obstacles:
        return milestones_primary

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
        # Start-side x positions (past obstacle from start direction)
        x_vals = [max_x + 0.5, max_x + 1.0, mid_x]
        # Goal-side x positions (between obstacle and goal)
        x_vals += [min_x - 0.5, min_x - 1.0]
        x_vals = [x for x in x_vals if 0.5 < x < 8.5]

        for y_sign in [1.0, -1.0]:
            edge_y = max_y if y_sign > 0 else min_y
            y_offsets = [1.0, 1.5, 2.0, 2.5]
            for y_off in y_offsets:
                wp_y = edge_y + y_sign * y_off
                if abs(wp_y) > 4.5:
                    continue
                for wp_x in x_vals:
                    # Heading 0 first (parallel to goal direction, most useful)
                    milestones_primary.append((wp_x, wp_y, 0.0, 'grid'))
                    # Heading variant pointing toward goal (backup, tried later)
                    th_to_goal = math.atan2(RS_GOAL_Y - wp_y, RS_GOAL_X - wp_x)
                    if abs(th_to_goal) > 0.2:
                        milestones_heading.append((wp_x, wp_y, th_to_goal, 'grid_h'))

    # Exp-7: th=0 milestones first — they're simpler RS paths and more
    # likely to succeed.  Heading variants are appended as backup.
    return milestones_primary + milestones_heading


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
_QUALITY_EPS = 1e-3
_QM1_DETOUR_MAX = 5.0
_QM2_LATERAL_EXTRA = 3.0
_QM3_X_BACKTRACK_EXTRA = 3.0
_QM4_BEHIND_WALL_PTS_MAX = 5
_QM5_RS_DETOUR_MAX = 4.0
_QM6_GEAR_SHIFT_MAX = 8
_RESCUE_SEED_MAX_ACTS = 96
_RESCUE_SEED_MAX_LEN = 8.0
_RESCUE_SEED_MAX_RATIO = 2.8
_RESCUE_SEED_MAX_EXTRA_Y = 2.5
_RESCUE_SEED_MAX_X_EXTRA = 2.5
_RESCUE_SEED_ALT_Y = 0.35


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


def _trajectory_length(traj):
    if not traj or len(traj) < 2:
        return 0.0
    return sum(
        math.hypot(traj[i][0] - traj[i - 1][0], traj[i][1] - traj[i - 1][1])
        for i in range(1, len(traj))
    )


def _check_rescue_seed_quality(start_x, start_y, start_th, acts, precomp_prim):
    """
    Rescue seed is only meant to smooth small geometric discontinuities.
    If the seed itself is long or highly wandering, it should fall back to A*
    instead of being treated as a cheap shortcut.
    """
    if not acts:
        return False, "empty", {}

    traj = simulate_path(start_x, start_y, start_th, acts, precomp_prim)
    geom_len = _trajectory_length(traj)
    direct_goal = math.hypot(start_x - RS_GOAL_X, start_y - RS_GOAL_Y)
    shifts = _count_gear_shifts(acts)
    max_abs_y = max(abs(p[1]) for p in traj)
    max_x = max(p[0] for p in traj)

    allowed_len = max(_RESCUE_SEED_MAX_LEN, direct_goal * _RESCUE_SEED_MAX_RATIO)
    allowed_y = max(abs(start_y), abs(RS_GOAL_Y)) + _RESCUE_SEED_MAX_EXTRA_Y
    allowed_x = start_x + _RESCUE_SEED_MAX_X_EXTRA
    metrics = {
        'acts': len(acts),
        'shifts': shifts,
        'geom_len': round(geom_len, 3),
        'max_abs_y': round(max_abs_y, 3),
        'max_x': round(max_x, 3),
        'allowed_len': round(allowed_len, 3),
        'allowed_y': round(allowed_y, 3),
        'allowed_x': round(allowed_x, 3),
    }

    if len(acts) > _RESCUE_SEED_MAX_ACTS:
        return False, "act_count", metrics
    if geom_len > allowed_len + _QUALITY_EPS:
        return False, "geom_len", metrics
    if max_abs_y > allowed_y + _QUALITY_EPS:
        return False, "max_abs_y", metrics
    if max_x > allowed_x + _QUALITY_EPS:
        return False, "max_x", metrics
    return True, "ok", metrics


def _score_rescue_seed_candidate(seed_d_goal, metrics, my, mth):
    """
    Lower score is better. Prefer seeds that materially reduce Dijkstra distance
    while staying short, compact, and roughly goal-aligned.
    """
    return (
        seed_d_goal * 3.0
        + metrics.get('geom_len', 0.0) * 0.25
        + metrics.get('acts', 0) * 0.02
        + abs(my) * 0.6
        + abs(mth) * 0.4
    )


def _build_kturn_seed_candidates(x0, y0, theta0, precomp_prim, no_corridor,
                                 fast_obstacles, dijkstra_grid, target_y_max=None):
    """
    Enumerate a few bounded rescue-seed variants and rank the valid ones.

    The failure band near simple obstacles is sensitive to how aggressively the
    K-turn stage tries to compress lateral error. Trying a tiny candidate set of
    target_y_max values is enough to recover boundary cases without opening a
    combinatorial search.
    """
    target_opts = []
    for ty in [target_y_max, _RESCUE_SEED_ALT_Y, PREAPPROACH_Y_MAX]:
        if ty is None:
            continue
        if all(abs(ty - existing) > 1e-9 for existing in target_opts):
            target_opts.append(ty)

    candidates = []
    rejected = []
    seen = set()
    for ty in target_opts:
        acts, mx, my, mth = _build_kturn_seed(
            x0, y0, theta0, precomp_prim, no_corridor,
            fast_obstacles, dijkstra_grid, target_y_max=ty)
        if not acts:
            rejected.append({
                'target_y_max': round(ty, 3),
                'reason': 'empty',
            })
            continue

        # Small rounding is enough to collapse equivalent variants.
        key = (len(acts), round(mx, 2), round(my, 2), round(mth, 2))
        if key in seen:
            continue
        seen.add(key)

        rescue_ok, rescue_reason, rescue_metrics = _check_rescue_seed_quality(
            x0, y0, theta0, acts, precomp_prim)
        seed_d_goal, _ = dijkstra_grid.get_heuristic(mx, my)
        candidate = {
            'acts': acts,
            'seed': (mx, my, mth),
            'target_y_max': round(ty, 3),
            'seed_d_goal': round(seed_d_goal, 3),
            'reason': rescue_reason,
            'metrics': rescue_metrics,
        }
        if rescue_ok:
            candidate['score'] = round(
                _score_rescue_seed_candidate(seed_d_goal, rescue_metrics, my, mth), 3)
            candidates.append(candidate)
        else:
            rejected.append({
                'target_y_max': round(ty, 3),
                'reason': rescue_reason,
                'metrics': rescue_metrics,
                'seed_d_goal': round(seed_d_goal, 3),
            })

    candidates.sort(key=lambda c: (c['score'], c['seed_d_goal'], c['metrics'].get('acts', 0)))
    return candidates, rejected


def _check_l18_quality(start_x, start_y, start_th, acts, rs_traj, precomp_prim):
    """
    Apply the same QM thresholds as test_path_quality_v2 before accepting L1.8.
    """
    traj_astar = simulate_path(start_x, start_y, start_th, acts, precomp_prim) if acts else [(start_x, start_y, start_th)]
    full_traj = list(traj_astar)
    if rs_traj:
        full_traj.extend(rs_traj[1:] if full_traj else rs_traj)
    if not full_traj:
        return False, "empty_traj"

    length = _trajectory_length(full_traj)
    euclidean = math.hypot(start_x - RS_GOAL_X, start_y - RS_GOAL_Y)
    detour_ratio = length / max(euclidean, 2.0)
    max_abs_y = max(abs(p[1]) for p in full_traj)
    max_x = max(p[0] for p in full_traj)
    behind_wall_cnt = sum(1 for p in full_traj if p[0] < _WALL_X and abs(p[1]) > _WALL_CORRIDOR_Y)

    rs_detour = 0.0
    if rs_traj and len(rs_traj) > 1:
        rs_len = _trajectory_length(rs_traj)
        rs_euclidean = math.hypot(rs_traj[0][0] - rs_traj[-1][0], rs_traj[0][1] - rs_traj[-1][1])
        rs_detour = rs_len / max(rs_euclidean, 1.0)

    shifts = _count_gear_shifts(acts)
    allowed_y = max(abs(start_y), abs(RS_GOAL_Y)) + _QM2_LATERAL_EXTRA
    allowed_x = start_x + _QM3_X_BACKTRACK_EXTRA

    if detour_ratio > _QM1_DETOUR_MAX + _QUALITY_EPS:
        return False, "qm1_detour"
    if max_abs_y > allowed_y + _QUALITY_EPS:
        return False, "qm2_lateral"
    if max_x > allowed_x + _QUALITY_EPS:
        return False, "qm3_max_x"
    if behind_wall_cnt > _QM4_BEHIND_WALL_PTS_MAX:
        return False, "qm4_behind_wall"
    if rs_detour > _QM5_RS_DETOUR_MAX + _QUALITY_EPS:
        return False, "qm5_rs_detour"
    if shifts > _QM6_GEAR_SHIFT_MAX:
        return False, "qm6_shifts"
    return True, "ok"


def _path_is_unreasonable(traj, sx, sy,
                          max_ratio=_PATH_LENGTH_RATIO_MAX,
                          max_lat_extra=_PATH_LATERAL_EXTRA_MAX):
    """Reject paths with excessive length or lateral deviation.
    Catches L1/L1.5 arcs that technically avoid the wall but detour wildly.

    Parameters
    ----------
    max_ratio : float
        Maximum allowed trajectory-length / direct-distance ratio.
        Default 3.0 for L1/RS-expansion; use 5.0 for L1.5 detour paths.
    max_lat_extra : float
        Maximum extra lateral deviation beyond start/goal y.
        Default 2.0 for L1; use 3.0 for L1.5 where lateral detour is expected.
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
    if traj_len > eff_direct * max_ratio:
        return True
    y_bound = max(abs(sy), abs(RS_GOAL_Y)) + max_lat_extra
    if max_abs_y > y_bound:
        return True
    return False


def _try_milestone_rs_bypass(sx, sy, sth, obstacles, collision_fn,
                              fast_obstacles=None, no_corridor=False,
                              max_paths=8, max_milestones=20, deadline=None):
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
        if deadline is not None and time.perf_counter() > deadline:
            return False, None  # Hard exit on deadline
        seg1_list = rs.rs_sample_path_multi(
            sx, sy, sth, mx, my, mth,
            MIN_TURN_RADIUS, step=step, max_paths=max_paths)

        for seg1 in seg1_list:
            if deadline is not None and time.perf_counter() > deadline:
                return False, None
            if not seg1 or not _check_traj_collision(seg1, coll_relaxed):
                continue

            seg2_list = rs.rs_sample_path_multi(
                mx, my, mth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                MIN_TURN_RADIUS, step=step, max_paths=max_paths)

            for seg2 in seg2_list:
                if deadline is not None and time.perf_counter() > deadline:
                    return False, None
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
                    # Exp-7: Return first viable 2-seg path immediately.
                    # Critical fix: without early exit, the exhaustive search
                    # across 30+ milestones causes deadline timeout, which
                    # then DISCARDS best_traj via `return False, None`.
                    return True, best_traj

    if best_traj is not None:
        return True, best_traj

    # --- Try 3-segment: start → wp_bypass → wp_return → goal ---
    # Reduced return waypoints for speed
    return_wps = [
        (2.8, 0.0, 0.0), (3.0, 0.0, 0.0), (2.8, 0.15, 0.0),
    ]
    for mx, my, mth, _tag in milestones[:max_milestones]:
        if deadline is not None and time.perf_counter() > deadline:
            return False, None
        seg1_list = rs.rs_sample_path_multi(
            sx, sy, sth, mx, my, mth,
            MIN_TURN_RADIUS, step=step, max_paths=4)  # Reduced from 6
        seg1_ok = [s for s in seg1_list
                   if s and _check_traj_collision(s, coll_relaxed)]
        if not seg1_ok:
            continue
        for rwx, rwy, rwth in return_wps:
            if deadline is not None and time.perf_counter() > deadline:
                return False, None
            seg2_list = rs.rs_sample_path_multi(
                mx, my, mth, rwx, rwy, rwth,
                MIN_TURN_RADIUS, step=step, max_paths=6)  # Reduced from 12
            for seg2 in seg2_list:
                if deadline is not None and time.perf_counter() > deadline:
                    return False, None
                if not seg2 or not _check_traj_collision(seg2, coll_relaxed):
                    continue
                seg3_list = rs.rs_sample_path_multi(
                    rwx, rwy, rwth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                    MIN_TURN_RADIUS, step=step, max_paths=3)  # Reduced from 5
                for seg3 in seg3_list:
                    if deadline is not None and time.perf_counter() > deadline:
                        return False, None
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
                        # Exp-7: Early exit for 3-seg path too
                        return True, best_traj

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

        # Removed heading penalty

        return h, h_weight
    return h_fn


# ═══════════════════════════════════════════════════════════════════════
# Improvement 3: Multi-stage recovery configuration
# ═══════════════════════════════════════════════════════════════════════

RECOVERY_STAGES = [
    {
        'name': 'fast',
        'expand_limit': 12000,  # Increased from 8000
        'prim_limit': 80,
        'rs_radius': 4.0,
        'allow_uphill': 2.0,
        'max_rs_paths': 6,
        'h_weight': 4.0,
    },
    {
        'name': 'robust',
        'expand_limit': 40000,  # Increased from 25000
        'prim_limit': 100,
        'rs_radius': 6.0,
        'allow_uphill': 6.0,
        'max_rs_paths': 10,
        'h_weight': 3.0,
    },
    {
        'name': 'aggressive',
        'expand_limit': 100000,  # Increased from 80000
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


def _build_kturn_seed(x0, y0, theta0, precomp_prim, no_corridor,
                      fast_obstacles, dijkstra_grid, target_y_max=None):
    """
    Build a prepositioned seed even for cases that normally skip K-turn.

    Some simple single-obstacle scenes are geometrically brittle: a tiny lateral
    perturbation flips the raw-start L1.8 stitching from success to failure.
    Returning one rescued seed smooths that discontinuity without forcing every
    easy case through K-turn first.
    """
    mx, my, mth = x0, y0, theta0
    seed_acts = []

    ok1_greedy, acts1_greedy, mx2, my2, mth2 = _k_turn_preposition_obs(
        mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
        dijkstra_grid=dijkstra_grid, target_y_max=target_y_max)
    if ok1_greedy and acts1_greedy:
        seed_acts.extend(acts1_greedy)
        mx, my, mth = mx2, my2, mth2

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
            target_y_max=target_y_max, x_ceil=s15_x_ceil)
        if ok15 and acts15:
            seed_acts.extend(acts15)
            mx, my, mth = mx15, my15, mth15

    return seed_acts, mx, my, mth


# ═══════════════════════════════════════════════════════════════════════
# Main planner v2
# ═══════════════════════════════════════════════════════════════════════

def plan_path_robust_obs_v2(x0, y0, theta0, precomp_prim,
                             use_rs=True, stats=None, no_corridor=False,
                             rs_expansion_radius=2.5, obstacles=None,
                             _time_budget=28.0):
    """
    Enhanced obstacle planner v2 with multi-stage recovery.
    Drop-in replacement for plan_path_robust_obs.

    _time_budget: total wall-clock seconds before the planner gives up.
                  Default 28s leaves a 2s margin under a typical 30s test timeout.
                  Exp-8: increased from 25s to give A* more time, since L1.5/L1.8
                  RS-based stages are provably infeasible for multi-circle model
                  near tight obstacles.
    """
    t_robust = time.perf_counter()
    planner_deadline = t_robust + _time_budget
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

    # Exp-10: Reduced inflate_radius from 0.50 → 0.30.
    # With grid_res=0.15, inflate=0.50 gives inf_cells=4 (0.60m hard-blocked zone),
    # which over-inflates obstacles in the heuristic — diagnostic shows A* gets
    # stuck near the start because the soft-cost zone (1.0m) creates heuristic
    # bottlenecks.  inflate=0.30 gives inf_cells=2 (0.30m), still > VEHICLE_HALF_WIDTH
    # (0.25m), but the Dijkstra detour_ratio for S11 drops 1.73→1.41 (-18%).
    # The actual multi-circle collision check still ensures physical safety.
    dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=primitives.VEHICLE_HALF_WIDTH + 0.05)
    dijkstra_grid.build_map(obstacles, start_x=x0, start_y=y0)
    _, start_d_goal = dijkstra_grid.get_heuristic(x0, y0)

    # ── Exp-3c: Dijkstra unreachability early-exit ──
    # If Dijkstra says start is unreachable (inf distance), fail fast
    if start_d_goal == float('inf'):
        stats.update(expanded=0, elapsed_ms=0.0, two_stage=False,
                     level='FAILED_unreachable')
        return False, None, None

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
    # Enable for ALL obstacle cases — single obstacles (S02/S03) also benefit
    # from milestone-based RS detour, which is fast (~3s budget) and avoids
    # expensive A* search entirely.
    # Exp-8: Reduced L1.5 budget from 3s→1s.  Diagnostic shows seg2
    #         (milestone→goal) has 0 collision-free RS paths for all 11 timeout
    #         cases — the multi-circle tail sweep makes milestone→goal RS
    #         geometrically infeasible near obstacles.  L1.5 fails fast (<1.5s),
    #         so 1s is sufficient while preserving budget for A*.
    run_milestone_bypass = use_rs and bool(obstacles)
    if run_milestone_bypass:
        l15_deadline = min(time.perf_counter() + 1.0, planner_deadline)
        bypass_ok, bypass_traj = _try_milestone_rs_bypass(
            x0, y0, theta0, obstacles, coll_fn,
            fast_obstacles=fast_obstacles, no_corridor=no_corridor,
            max_paths=8, max_milestones=30, deadline=l15_deadline)
        if bypass_ok and not _path_goes_behind_wall(bypass_traj) \
                and not _path_is_unreasonable(bypass_traj, x0, y0,
                                              max_ratio=5.0, max_lat_extra=3.0):
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

    # Phase-0 turnaround only when facing away from goal
    if abs(mth) > 2.6 or (abs(mth) > 1.57 and mth * my < 0):
        phase0_acts, mx, my, mth = _phase0_turnaround(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles)

    # For simple obstacles (1-2), skip expensive K-turn and go straight to A*
    # K-turn is only beneficial for complex scenarios or large y-offsets
    skip_kturn = fast_obstacles is not None and len(fast_obstacles) <= 2 and abs(my) < 2.0

    obs_tight_y = 0.2 if fast_obstacles else None
    acts1 = []
    if not skip_kturn and (phase0_acts or abs(my) > PREAPPROACH_Y_MAX or abs(mth) > 0.5 or fast_obstacles is not None):
        ok1_greedy, acts1_greedy, mx2, my2, mth2 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, target_y_max=obs_tight_y)
        if ok1_greedy and acts1_greedy:
            acts1.extend(acts1_greedy)
            mx, my, mth = mx2, my2, mth2

    stage15_acts = []
    if not skip_kturn and abs(my) > 0.15:
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

    # ── Level-1.8: 2D skeleton + RS stitching ──
    # Enable for ALL obstacle cases: 2D Dijkstra trace + RS stitching is the most
    # effective approach for navigating around obstacles (even single ones).
    # A* with fine-grained primitives (~0.007m lateral/step) cannot efficiently
    # detour around obstacles > 0.5m.
    run_2d_skeleton = bool(obstacles)
    simple_obs_fast_path = bool(
        skip_kturn and fast_obstacles is not None and len(fast_obstacles) <= 2
    )
    rescue_acts = None
    rescue_seed = None
    # Track best L1.8 candidates separately: QM-passing vs any collision-free
    _l18_qm_traj = None
    _l18_qm_acts = None
    _l18_qm_spc = None
    _l18_any_traj = None
    _l18_any_acts = None
    _l18_any_spc = None
    _l18_any_len = float('inf')
    if run_2d_skeleton:
        def _try_l18_seed(sx2, sy2, sth2, seed_acts):
            nonlocal _l18_qm_traj, _l18_qm_acts, _l18_qm_spc
            nonlocal _l18_any_traj, _l18_any_acts, _l18_any_spc, _l18_any_len
            if time.perf_counter() > l18_deadline:
                return False
            for spc in [2.5, 1.5, 1.0]:
                if time.perf_counter() > l18_deadline:
                    break
                fb2d_ok, fb2d_traj = plan_2d_fallback(
                    dijkstra_grid, sx2, sy2, sth2, coll_relaxed,
                    spacing=spc, max_rs_paths=12, obstacles=obstacles,
                    deadline=l18_deadline)
                if fb2d_ok and not _path_goes_behind_wall(fb2d_traj):
                    qm_ok, qm_reason = _check_l18_quality(
                        x0, y0, theta0, seed_acts, fb2d_traj, precomp_prim)
                    if qm_ok:
                        _l18_qm_traj = fb2d_traj
                        _l18_qm_acts = seed_acts
                        _l18_qm_spc = spc
                        return True
                    stats['l18_reject_reason'] = qm_reason
                    tlen = _trajectory_length(fb2d_traj)
                    if tlen < _l18_any_len:
                        _l18_any_traj = fb2d_traj
                        _l18_any_acts = seed_acts
                        _l18_any_spc = spc
                        _l18_any_len = tlen
            return False

        # Exp-8: Reduced L1.8 budget from 30%/8s → 20%/5s.
        # Diagnostic shows RS stitching fails for 10/11 timeout cases and
        # takes 40-60s when it does succeed — most budget is wasted.
        # Keep min=2s so L1.8 can still solve easy detours quickly.
        _remaining = planner_deadline - time.perf_counter()
        _l18_budget = min(_remaining * 0.2, 5.0)
        l18_deadline = time.perf_counter() + max(_l18_budget, 2.0)
        coll_relaxed = coll_fn
        if prefix_too_many_shifts:
            # K-turn prefix is too long; skip it, only try from original position
            starts_2d = [(x0, y0, theta0, [])]
        else:
            starts_2d = [(mx, my, mth, prefix_acts)]
            if (mx, my) != (x0, y0):
                starts_2d.append((x0, y0, theta0, []))
        for sx2, sy2, sth2, prefix_acts in starts_2d:
            if _try_l18_seed(sx2, sy2, sth2, prefix_acts):
                break
            if _l18_qm_traj is not None:
                break  # outer loop

        # Simple-obstacle fast path is intentionally brittle for speed: it skips
        # K-turn entirely. If the primary L1.8 attempt finds no QM-passing path,
        # build one rescued seed and hand it to L2 instead of forcing a brittle
        # non-QM shortcut to count as success.
        if (_l18_qm_traj is None
                and simple_obs_fast_path
                and (abs(mth) > 0.9 or abs(my) > PREAPPROACH_Y_MAX)):
            rescue_t0 = time.perf_counter()
            rescue_candidates, rescue_rejected = _build_kturn_seed_candidates(
                x0, y0, theta0, precomp_prim, no_corridor,
                fast_obstacles, dijkstra_grid, target_y_max=obs_tight_y)
            rescue_ms = round((time.perf_counter() - rescue_t0) * 1000.0, 1)
            if rescue_candidates:
                best = rescue_candidates[0]
                rescue_acts = best['acts']
                rescue_seed = best['seed']
                stats['simple_obs_kturn_rescue'] = True
                stats['simple_obs_kturn_rescue_ms'] = rescue_ms
                stats['simple_obs_kturn_rescue_seed'] = rescue_seed
                stats['simple_obs_kturn_rescue_shifts'] = _count_gear_shifts(rescue_acts)
                stats['simple_obs_kturn_rescue_target_y_max'] = best['target_y_max']
                stats['simple_obs_kturn_rescue_seed_d_goal'] = best['seed_d_goal']
                stats['simple_obs_kturn_rescue_score'] = best['score']
                stats['simple_obs_kturn_rescue_deferred_to_l2'] = True
                stats['simple_obs_kturn_rescue_alternatives'] = max(0, len(rescue_candidates) - 1)
            else:
                rescue_acts = None
                if rescue_rejected:
                    best_rejected = min(
                        rescue_rejected,
                        key=lambda r: (r.get('seed_d_goal', float('inf')),
                                       r.get('metrics', {}).get('acts', float('inf'))))
                    stats['simple_obs_kturn_rescue_rejected'] = best_rejected.get('reason')
                    stats['simple_obs_kturn_rescue_candidate'] = best_rejected

        # Return the best L1.8 path: prefer QM-passing, fallback to collision-free
        if _l18_qm_traj is not None:
            total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
            level = f'L1_8_2d_skeleton_sp{_l18_qm_spc}'
            if stats.get('simple_obs_kturn_rescue'):
                level += '_rescue'
            stats.update(
                expanded=0, elapsed_ms=total_ms,
                level=level,
                use_rs=use_rs, no_corridor=no_corridor,
                two_stage=True, stage1_ms=t1_ms,
            )
            return True, _l18_qm_acts, _l18_qm_traj
        if _l18_any_traj is not None:
            stats['l18_non_qm_candidate'] = True
            stats['l18_non_qm_spc'] = _l18_any_spc
            stats['l18_non_qm_path_len'] = round(_l18_any_len, 3)
            if not simple_obs_fast_path:
                total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
                level = f'L1_8_2d_skeleton_sp{_l18_any_spc}'
                if stats.get('simple_obs_kturn_rescue'):
                    level += '_rescue'
                stats.update(
                    expanded=0, elapsed_ms=total_ms,
                    level=level,
                    use_rs=use_rs, no_corridor=no_corridor,
                    two_stage=True, stage1_ms=t1_ms,
                )
                return True, _l18_any_acts, _l18_any_traj
            stats['l18_non_qm_deferred'] = True

    # ── Level-2: Multi-stage A* from selected seed ──
    l2_mx, l2_my, l2_mth = mx, my, mth
    l2_prefix_acts = prefix_acts
    l2_start_kind = 'primary'
    l2_prefix_too_many_shifts = prefix_too_many_shifts
    if rescue_acts and rescue_seed is not None:
        l2_prefix_acts = rescue_acts
        l2_mx, l2_my, l2_mth = rescue_seed
        l2_start_kind = 'rescue'
        # Rescue seed is explicitly built to hand off to downstream search,
        # so don't skip L2 solely because the prefix itself shifts often.
        l2_prefix_too_many_shifts = False
    stats['l2_start_kind'] = l2_start_kind

    _, mid_d_goal = dijkstra_grid.get_heuristic(l2_mx, l2_my)
    total_expanded = 0  # accumulate across all A* stages for diagnostics

    # Exp-8: Increased A* budgets from [5,8,15] → [6,10,18].
    # Total A* budget now 34s (vs prev 28s).  Combined with reduced L1.5 (1s)
    # and L1.8 (≤5s), A* gets ~22-25s of real wall-clock time.
    STAGE_TIME_BUDGETS = [6.0, 10.0, 18.0]  # fast, robust, aggressive

    if not l2_prefix_too_many_shifts:
        for stage_idx, stage in enumerate(RECOVERY_STAGES):
            stage_start = time.perf_counter()
            # Calculate per-stage deadline
            stage_budget = STAGE_TIME_BUDGETS[stage_idx] if stage_idx < len(STAGE_TIME_BUDGETS) else 10.0
            stage_deadline = min(stage_start + stage_budget, planner_deadline)

            if stage_start > planner_deadline:
                break
            st = {}
            rs_expand = _make_rs_expand_fn_multi(
                coll_fn, max(rs_expansion_radius, stage['rs_radius']),
                max_paths=stage['max_rs_paths'],
                dijkstra_grid=dijkstra_grid) if use_rs else None

            heuristic = _make_heuristic_fn_v2(
                dijkstra_grid, mid_d_goal,
                allow_uphill=stage['allow_uphill'], h_weight=stage['h_weight'])

            dist_x = max(0.0, l2_mx - 2.1)
            dyn_prim = max(stage['prim_limit'], 30 + int(dist_x / 0.1))

            ok2, acts2, rs_traj = astar_core.plan_path(
                l2_mx, l2_my, l2_mth, precomp_prim,
                collision_fn=coll_fn, heuristic_fn=heuristic,
                rs_expand_fn=rs_expand, stats=st,
                _goal_xmin=0.5, _goal_xmax=8.0,
                _prim_limit=dyn_prim,
                _expand_limit=stage['expand_limit'],
                _rs_expand=use_rs,
                rs_expansion_radius=max(rs_expansion_radius, stage['rs_radius']),
                deadline=stage_deadline)  # Use per-stage deadline
            total_expanded += st.get('expanded', 0)

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

                final_acts = l2_prefix_acts + (acts2 or [])
                total_ms = round((time.perf_counter() - t_robust) * 1000.0, 1)
                stats.update(
                    expanded=st.get('expanded', 0), elapsed_ms=total_ms,
                    use_rs=use_rs, no_corridor=no_corridor,
                    rs_expansion=st.get('rs_expansion', False),
                    two_stage=True, stage1_ms=t1_ms,
                    stage1_end=(l2_mx, l2_my, l2_mth),
                    goal_pos=st.get('goal_pos'),
                    l2_start_kind=l2_start_kind,
                    level=f'L2_{stage["name"]}',
                    recovery_stage=stage_idx,
                )
                return True, final_acts, rs_traj

    # ── Level-3: Fallback from original start (robust + aggressive only) ──
    # Exp-3e: Per-stage time allocation for fallback (robust=7s, aggressive=remaining)
    FALLBACK_STAGE_BUDGETS = [7.0, 15.0]  # for robust, aggressive

    for stage_idx, stage in enumerate(RECOVERY_STAGES[1:], 1):
        fb_stage_start = time.perf_counter()
        fb_budget = FALLBACK_STAGE_BUDGETS[stage_idx-1] if (stage_idx-1) < len(FALLBACK_STAGE_BUDGETS) else 10.0
        fb_stage_deadline = min(fb_stage_start + fb_budget, planner_deadline)

        if fb_stage_start > planner_deadline:
            break
        st_fb = {}
        rs_expand = _make_rs_expand_fn_multi(
            coll_fn, max(rs_expansion_radius, stage['rs_radius']),
            max_paths=stage['max_rs_paths'],
            dijkstra_grid=dijkstra_grid) if use_rs else None

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
            rs_expansion_radius=max(rs_expansion_radius, stage['rs_radius']),
            deadline=fb_stage_deadline)  # Use per-stage deadline
        total_expanded += st_fb.get('expanded', 0)

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
        expanded=total_expanded, elapsed_ms=total_ms, two_stage=True,
        stage1_ms=t1_ms, use_rs=use_rs, no_corridor=no_corridor,
        level='FAILED_all_stages',
    )
    return False, None, None
