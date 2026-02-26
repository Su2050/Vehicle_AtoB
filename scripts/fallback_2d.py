"""
fallback_2d.py — 2D Dijkstra skeleton path + RS segment stitching.

When the fine-resolution A* times out on complex obstacle layouts, this module
traces a coarse 2D path on the DijkstraGrid (0.15m resolution) and attempts to
stitch the path with collision-free Reeds-Shepp segments.
"""

import math
import heapq
import time
import rs
from primitives import (
    DT, MIN_TURN_RADIUS, ALIGN_GOAL_DYAW,
    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
)

_QUALITY_EPS = 1e-3
_QM1_DETOUR_MAX = 5.0
_QM2_LATERAL_EXTRA = 3.0
_QM3_X_BACKTRACK_EXTRA = 3.0
_QM5_RS_DETOUR_MAX = 4.0


def _trace_gradient_path(dg, start_wx, start_wy):
    """
    Follow the Dijkstra distance-field gradient from start to goal.
    Returns list of world-coordinate points [(x, y), ...].
    """
    gx, gy = dg._world_to_grid(start_wx, start_wy)
    if not (0 <= gx < dg.nx and 0 <= gy < dg.ny):
        return []
    if dg.dist_map[gx][gy] == float('inf'):
        return []

    path = []
    visited = set()
    dirs = [
        (1, 0), (0, 1), (-1, 0), (0, -1),
        (1, 1), (-1, 1), (1, -1), (-1, -1),
    ]

    max_steps = dg.nx * dg.ny
    for _ in range(max_steps):
        if (gx, gy) in visited:
            break
        visited.add((gx, gy))
        wx, wy = dg._grid_to_world(gx, gy)
        path.append((wx, wy))

        cur_d = dg.dist_map[gx][gy]
        if cur_d < dg.res * 1.5:
            wx_goal, wy_goal = dg.goal_x, dg.goal_y
            path.append((wx_goal, wy_goal))
            break

        best_d = cur_d
        best_gx, best_gy = gx, gy
        for dx, dy in dirs:
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < dg.nx and 0 <= ny < dg.ny:
                if not dg.obs_map[nx][ny]:
                    d = dg.dist_map[nx][ny]
                    if d < best_d:
                        best_d = d
                        best_gx, best_gy = nx, ny
        if (best_gx, best_gy) == (gx, gy):
            break
        gx, gy = best_gx, best_gy

    return path


def _dijkstra_2d_path(start_wx, start_wy, goal_wx, goal_wy,
                       obstacles, inflate_radius=0.7, soft_radius=None,
                       grid_res=0.15):
    """
    Standalone 2D Dijkstra path search with hard inflation + soft cost penalty.
    The soft penalty discourages routing through narrow gaps that RS curves
    can't navigate, pushing the path toward wider openings.
    Returns list of world-coordinate points [(x, y), ...] or [].
    """
    if soft_radius is None:
        soft_radius = MIN_TURN_RADIUS * 1.2
    min_x, max_x = -1.0, 10.0
    min_y, max_y = -6.0, 6.0
    nx = int((max_x - min_x) / grid_res)
    ny = int((max_y - min_y) / grid_res)

    def w2g(wx, wy):
        return int((wx - min_x) / grid_res), int((wy - min_y) / grid_res)

    def g2w(gx, gy):
        return min_x + (gx + 0.5) * grid_res, min_y + (gy + 0.5) * grid_res

    obs_map = [[False] * ny for _ in range(nx)]
    cost_map = [[1.0] * ny for _ in range(nx)]
    inf_cells = int(math.ceil(inflate_radius / grid_res))
    soft_cells = int(math.ceil(soft_radius / grid_res))

    wall_gx_max = int((1.9 - min_x) / grid_res)
    for x in range(min(nx, wall_gx_max)):
        for y in range(ny):
            obs_map[x][y] = True

    obs_rects = []
    if obstacles:
        for obs in obstacles:
            if isinstance(obs, tuple):
                owx1, owx2, owy1, owy2 = obs
            else:
                ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
                owx1, owx2 = min(ox, ox + ow), max(ox, ox + ow)
                owy1, owy2 = min(oy, oy + oh), max(oy, oy + oh)
            obs_rects.append((owx1, owx2, owy1, owy2))
            gx1, gy1 = w2g(owx1, owy1)
            gx2, gy2 = w2g(owx2, owy2)
            for x in range(max(0, gx1 - inf_cells), min(nx, gx2 + inf_cells + 1)):
                for y in range(max(0, gy1 - inf_cells), min(ny, gy2 + inf_cells + 1)):
                    obs_map[x][y] = True
            for x in range(max(0, gx1 - soft_cells), min(nx, gx2 + soft_cells + 1)):
                for y in range(max(0, gy1 - soft_cells), min(ny, gy2 + soft_cells + 1)):
                    if 0 <= x < nx and 0 <= y < ny and not obs_map[x][y]:
                        wx, wy = g2w(x, y)
                        d_obs = max(
                            max(owx1 - wx, 0.0, wx - owx2),
                            max(owy1 - wy, 0.0, wy - owy2))
                        if d_obs < soft_radius:
                            penalty = ((soft_radius - d_obs) / soft_radius) * 5.0
                            cost_map[x][y] = max(cost_map[x][y], 1.0 + penalty)

    sx, sy = w2g(start_wx, start_wy)
    gx_goal, gy_goal = w2g(goal_wx, goal_wy)

    clear_r = int(math.ceil(0.6 / grid_res))
    for cx in range(max(0, sx - clear_r), min(nx, sx + clear_r + 1)):
        for cy in range(max(0, sy - clear_r), min(ny, sy + clear_r + 1)):
            obs_map[cx][cy] = False
    for cx in range(max(0, gx_goal - clear_r), min(nx, gx_goal + clear_r + 1)):
        for cy in range(max(0, gy_goal - clear_r), min(ny, gy_goal + clear_r + 1)):
            if cx >= wall_gx_max:
                obs_map[cx][cy] = False

    if obs_map[sx][sy] or obs_map[gx_goal][gy_goal]:
        return []

    dist_arr = [[float('inf')] * ny for _ in range(nx)]
    parent = [[None] * ny for _ in range(nx)]
    dist_arr[sx][sy] = 0.0
    q = [(0.0, sx, sy)]
    dirs = [
        (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414),
    ]

    while q:
        d, cx, cy = heapq.heappop(q)
        if cx == gx_goal and cy == gy_goal:
            break
        if d > dist_arr[cx][cy]:
            continue
        for dx, dy, mc in dirs:
            nx2, ny2 = cx + dx, cy + dy
            if 0 <= nx2 < nx and 0 <= ny2 < ny and not obs_map[nx2][ny2]:
                nd = d + mc * grid_res * cost_map[nx2][ny2]
                if nd < dist_arr[nx2][ny2]:
                    dist_arr[nx2][ny2] = nd
                    parent[nx2][ny2] = (cx, cy)
                    heapq.heappush(q, (nd, nx2, ny2))

    if dist_arr[gx_goal][gy_goal] == float('inf'):
        return []

    path = []
    cx, cy = gx_goal, gy_goal
    while (cx, cy) != (sx, sy):
        path.append(g2w(cx, cy))
        p = parent[cx][cy]
        if p is None:
            break
        cx, cy = p
    path.append(g2w(sx, sy))
    path.reverse()
    return path


def _simplify_path(path, min_spacing=0.5):
    """
    Downsample path keeping points at least min_spacing apart,
    plus always keep first/last.
    """
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    for pt in path[1:-1]:
        dx = pt[0] - result[-1][0]
        dy = pt[1] - result[-1][1]
        if math.hypot(dx, dy) >= min_spacing:
            result.append(pt)
    result.append(path[-1])
    return result


def _heading_between(p1, p2):
    """Compute heading angle from p1 to p2."""
    return math.atan2(p2[1] - p1[1], p2[0] - p1[0])


def _assign_headings(waypoints):
    """
    Assign a heading to each waypoint based on the direction to the next one.
    Last waypoint gets RS_GOAL_TH.
    """
    n = len(waypoints)
    headings = [0.0] * n
    for i in range(n - 1):
        headings[i] = _heading_between(waypoints[i], waypoints[i + 1])
    headings[-1] = RS_GOAL_TH
    return headings


def _check_traj_collision(traj, collision_fn):
    for pt in traj:
        valid, _ = collision_fn(pt[0], pt[1], pt[2])
        if not valid:
            return False
    return True


def _try_rs_connect(x1, y1, th1, x2, y2, th2, collision_fn,
                    max_paths=10, step=None):
    """
    Try RS connections from (x1,y1,th1) to (x2,y2,th2).
    Returns the shortest collision-free trajectory, or None.
    """
    if step is None:
        step = DT * 0.5
    trajs = rs.rs_sample_path_multi(
        x1, y1, th1, x2, y2, th2,
        MIN_TURN_RADIUS, step=step, max_paths=max_paths)
    best = None
    for traj in trajs:
        if not traj:
            continue
        if _check_traj_collision(traj, collision_fn):
            if best is None or len(traj) < len(best):
                best = traj
    return best


def _try_connect_with_heading_variants(x1, y1, th1, x2, y2, th2_nom,
                                        collision_fn, max_paths=10, deadline=None):
    """
    Try RS connections with many heading variants at the destination.
    Returns (trajectory, actual_th2) or (None, None).
    """
    offsets = [0.0]
    for step in [0.3, 0.6, 1.0, 1.4, 1.8, 2.5, math.pi]:
        offsets.extend([step, -step])
    best_traj = None
    best_th2 = th2_nom
    for off in offsets:
        if deadline is not None and time.perf_counter() > deadline:
            break
        th2 = th2_nom + off
        traj = _try_rs_connect(x1, y1, th1, x2, y2, th2, collision_fn,
                               max_paths=max_paths)
        if traj is not None:
            if best_traj is None or len(traj) < len(best_traj):
                best_traj = traj
                best_th2 = th2
    return best_traj, best_th2


def _stitch_waypoints(waypoints, start_th, collision_fn, max_rs_paths, deadline=None):
    """
    Stitch waypoints with RS segments. Last segment targets exact goal pose.
    If the last segment fails, backtrack to earlier waypoints and scan for
    heading combinations that enable goal connection.
    Returns (success, trajectory).
    """
    nominal_headings = _assign_headings(waypoints)
    full_traj = []
    cur_th = start_th
    # Track (traj_prefix_length, heading) at each waypoint for backtracking
    wp_states = [(0, cur_th)]

    for i in range(len(waypoints) - 1):
        if deadline is not None and time.perf_counter() > deadline:
            return False, None
        wx1, wy1 = waypoints[i]
        wx2, wy2 = waypoints[i + 1]
        target_th = nominal_headings[i + 1]

        is_last = (i == len(waypoints) - 2)
        if is_last:
            wx2, wy2 = RS_GOAL_X, RS_GOAL_Y
            target_th = RS_GOAL_TH

        if is_last:
            seg = None
            seg_best_len = float('inf')
            seg_best_lat = float('inf')
            for off in [0.0, 0.04, -0.04, 0.08, -0.08]:
                cand = _try_rs_connect(
                    wx1, wy1, cur_th, wx2, wy2, target_th + off,
                    collision_fn, max_paths=max_rs_paths)
                if cand is None:
                    continue
                cand_len = _trajectory_length(cand)
                cand_lat = max(abs(p[1]) for p in cand)
                # Prefer smaller lateral excursion, then shorter final segment.
                if (cand_lat < seg_best_lat - 1e-9) or (
                        abs(cand_lat - seg_best_lat) <= 1e-9 and cand_len < seg_best_len):
                    seg = cand
                    seg_best_lat = cand_lat
                    seg_best_len = cand_len

            # Backtrack fallback: redo last k segments with goal-viable headings
            if seg is None:
                bt = _try_backtrack_to_goal(
                    waypoints, wp_states, i, collision_fn,
                    max_rs_paths, deadline)
                if bt is not None:
                    trunc_len, bt_segs = bt
                    full_traj = full_traj[:trunc_len]
                    for s in bt_segs:
                        full_traj.extend(s)
                    if full_traj:
                        ex, ey, eth = full_traj[-1]
                        if ex <= 2.25 and abs(ey) <= 0.18 and abs(eth) <= ALIGN_GOAL_DYAW:
                            return True, full_traj
                    return False, None

            actual_th = target_th
        else:
            seg, actual_th = _try_connect_with_heading_variants(
                wx1, wy1, cur_th, wx2, wy2, target_th,
                collision_fn, max_paths=max_rs_paths, deadline=deadline)

        if seg is None:
            return False, None

        full_traj.extend(seg)
        cur_th = actual_th
        wp_states.append((len(full_traj), cur_th))

    if full_traj:
        ex, ey, eth = full_traj[-1]
        if ex <= 2.25 and abs(ey) <= 0.18 and abs(eth) <= ALIGN_GOAL_DYAW:
            return True, full_traj

    return False, None


def _try_backtrack_to_goal(waypoints, wp_states, last_seg_idx,
                           collision_fn, max_rs_paths, deadline):
    """
    When the last RS segment to goal fails, backtrack through earlier waypoints
    and scan for heading combinations that enable goal connection.

    For each backtrack level k:
      1. Try direct RS from the pivot waypoint to goal.
      2. Try pivot → intermediate (with scanned heading) → goal.

    Returns (traj_truncate_point, [segment_list]) or None.
    """
    max_back = min(last_seg_idx, 3)

    for k in range(1, max_back + 1):
        if deadline is not None and time.perf_counter() > deadline:
            return None

        pivot_idx = last_seg_idx - k
        pivot_traj_len, pivot_th = wp_states[pivot_idx]
        px, py = waypoints[pivot_idx]

        # Strategy 1: Direct RS from pivot to goal (with small offsets)
        for off in [0.0, 0.04, -0.04, 0.08, -0.08]:
            seg = _try_rs_connect(px, py, pivot_th,
                                  RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH + off,
                                  collision_fn, max_paths=max_rs_paths)
            if seg is not None:
                return (pivot_traj_len, [seg])

        # Strategy 2: pivot → intermediate waypoint → goal
        for mid_idx in range(pivot_idx + 1, last_seg_idx + 1):
            if deadline is not None and time.perf_counter() > deadline:
                return None

            mx, my = waypoints[mid_idx]

            # Scan headings at the intermediate that enable goal connection
            for deg in range(-180, 180, 15):
                if deadline is not None and time.perf_counter() > deadline:
                    return None
                test_th = math.radians(deg)

                # Can we reach goal from (mx, my, test_th)?
                goal_seg = _try_rs_connect(mx, my, test_th,
                                           RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                                           collision_fn, max_paths=max_rs_paths)
                if goal_seg is None:
                    continue

                # Can we reach (mx, my, test_th) from pivot?
                mid_seg = _try_rs_connect(px, py, pivot_th,
                                          mx, my, test_th,
                                          collision_fn, max_paths=max_rs_paths)
                if mid_seg is not None:
                    return (pivot_traj_len, [mid_seg, goal_seg])

    return None


def _trajectory_length(traj):
    """Compute total geometric length of a trajectory [(x,y,th), ...]."""
    total = 0.0
    for i in range(1, len(traj)):
        dx = traj[i][0] - traj[i - 1][0]
        dy = traj[i][1] - traj[i - 1][1]
        total += math.hypot(dx, dy)
    return total


def _evaluate_traj_quality(start_x, start_y, traj):
    """
    Compute L1.8 quality metrics aligned with test_path_quality_v2 thresholds.
    Returns (ok, score, metrics_dict).
    """
    if not traj or len(traj) < 2:
        return False, float('inf'), {}

    tlen = _trajectory_length(traj)
    direct = math.hypot(start_x - RS_GOAL_X, start_y - RS_GOAL_Y)
    detour = tlen / max(direct, 2.0)
    max_abs_y = max(abs(p[1]) for p in traj)
    max_x = max(p[0] for p in traj)
    rs_direct = math.hypot(traj[0][0] - traj[-1][0], traj[0][1] - traj[-1][1])
    rs_detour = tlen / max(rs_direct, 1.0)

    allowed_y = max(abs(start_y), abs(RS_GOAL_Y)) + _QM2_LATERAL_EXTRA
    allowed_x = start_x + _QM3_X_BACKTRACK_EXTRA

    ok = (
        detour <= (_QM1_DETOUR_MAX + _QUALITY_EPS)
        and max_abs_y <= (allowed_y + _QUALITY_EPS)
        and max_x <= (allowed_x + _QUALITY_EPS)
        and rs_detour <= (_QM5_RS_DETOUR_MAX + _QUALITY_EPS)
    )
    # Quality-first score: prioritize smaller detour/lateral excursions,
    # then use geometric length as tie breaker.
    score = (
        detour * 10.0
        + (max_abs_y / max(allowed_y, 1.0)) * 8.0
        + (max_x / max(allowed_x, 1.0)) * 4.0
        + rs_detour * 8.0
        + tlen * 0.2
    )
    return ok, score, {
        'detour': detour,
        'max_abs_y': max_abs_y,
        'max_x': max_x,
        'rs_detour': rs_detour,
        'traj_len': tlen,
    }


_HEADING_PULL_DIST = 1.5


def plan_2d_fallback(dijkstra_grid, start_x, start_y, start_th,
                     collision_fn, spacing=1.5, max_rs_paths=12,
                     obstacles=None, deadline=None):
    """
    2D skeleton fallback planner with heading-aware candidate selection.
    Optimized with aggressive deadline checks for speed.
    """
    # Fast deadline check at start
    if deadline is not None and time.perf_counter() > deadline:
        return False, None

    raw_path = _trace_gradient_path(dijkstra_grid, start_x, start_y)

    candidates = []
    if len(raw_path) >= 2:
        candidates.append(raw_path)

    # Early exit if no path found
    if not candidates and not obstacles:
        return False, None

    if obstacles:
        candidate_cfgs = [
            # Prefer narrower hard inflation to avoid excessive detours.
            (0.25, MIN_TURN_RADIUS * 0.9),
            (0.22, MIN_TURN_RADIUS * 0.8),
            (0.30, MIN_TURN_RADIUS * 1.0),
        ]
        for inf_r, soft_r in candidate_cfgs:
            if deadline is not None and time.perf_counter() > deadline:
                break
            alt = _dijkstra_2d_path(
                start_x, start_y, RS_GOAL_X, RS_GOAL_Y, obstacles,
                inflate_radius=inf_r, soft_radius=soft_r)
            if len(alt) >= 2:
                candidates.append(alt)

    # Skip heading-biased variants if deadline is tight
    if deadline is None or time.perf_counter() + 1.0 < deadline:
        for sign in (1.0, -1.0):
            if deadline is not None and time.perf_counter() > deadline:
                break
            for pull_dist in (_HEADING_PULL_DIST, 1.0):
                px = start_x - sign * pull_dist * math.cos(start_th)
                py = start_y - sign * pull_dist * math.sin(start_th)
                if px < 0.0 or px > 9.5 or py < -5.5 or py > 5.5:
                    continue
                pull_path = _dijkstra_2d_path(
                    px, py, RS_GOAL_X, RS_GOAL_Y, obstacles,
                    inflate_radius=0.22, soft_radius=MIN_TURN_RADIUS * 0.8)
                if len(pull_path) >= 2:
                    candidates.append([(start_x, start_y)] + pull_path)

    # Try lower spacing first to reduce lateral drift in narrow passages.
    spacings = []
    for sp in [spacing * 0.6, spacing * 0.8, spacing, spacing * 1.5]:
        sp = max(0.7, min(3.0, sp))
        if all(abs(sp - existing) > 1e-6 for existing in spacings):
            spacings.append(sp)

    best_traj = None
    best_score = float('inf')
    best_len = float('inf')
    for raw in candidates:
        if deadline is not None and time.perf_counter() > deadline:
            break
        for sp in spacings:
            if deadline is not None and time.perf_counter() > deadline:
                break
            wps = _simplify_path(raw, min_spacing=sp)
            if len(wps) < 2:
                continue
            ok, traj = _stitch_waypoints(
                wps, start_th, collision_fn, max_rs_paths, deadline=deadline)
            if ok:
                pass_qm, score, qm = _evaluate_traj_quality(start_x, start_y, traj)
                if not pass_qm:
                    continue
                tlen = qm['traj_len']
                if (score < best_score - 1e-9) or (
                        abs(score - best_score) <= 1e-9 and tlen < best_len):
                    best_traj = traj
                    best_score = score
                    best_len = tlen

    if best_traj is not None:
        return True, best_traj

    # Retry with reduced inflation only if we have time
    if obstacles:
        if deadline is not None and time.perf_counter() > deadline - 1.0:
            return False, None  # Skip retry if low on time

        candidates2 = []
        alt2 = _dijkstra_2d_path(
            start_x, start_y, RS_GOAL_X, RS_GOAL_Y, obstacles,
            inflate_radius=0.18, soft_radius=MIN_TURN_RADIUS * 0.7)
        if len(alt2) >= 2:
            candidates2.append(alt2)

        fine_spacings = [max(0.7, spacing * 0.5), max(0.8, spacing * 0.7), spacing]
        for raw in candidates2:
            if deadline is not None and time.perf_counter() > deadline:
                break
            for sp in fine_spacings:
                if deadline is not None and time.perf_counter() > deadline:
                    break
                wps = _simplify_path(raw, min_spacing=sp)
                if len(wps) < 2:
                    continue
                ok, traj = _stitch_waypoints(
                    wps, start_th, collision_fn, max_rs_paths, deadline=deadline)
                if ok:
                    pass_qm, score, qm = _evaluate_traj_quality(start_x, start_y, traj)
                    if not pass_qm:
                        continue
                    tlen = qm['traj_len']
                    if (score < best_score - 1e-9) or (
                            abs(score - best_score) <= 1e-9 and tlen < best_len):
                        best_traj = traj
                        best_score = score
                        best_len = tlen

        if best_traj is not None:
            return True, best_traj

    return False, None


def _try_with_finer_waypoints(dijkstra_grid, coarse_wps, fail_idx, cur_th,
                               prefix_traj, collision_fn, max_rs_paths):
    """
    When a coarse segment fails, re-trace with finer spacing and retry.
    """
    raw_path = _trace_gradient_path(
        dijkstra_grid, coarse_wps[fail_idx][0], coarse_wps[fail_idx][1])
    if len(raw_path) < 2:
        return False, None

    fine_wps = _simplify_path(raw_path, min_spacing=0.6)
    if len(fine_wps) < 2:
        return False, None

    fine_headings = _assign_headings(fine_wps)
    full_traj = list(prefix_traj)

    for j in range(len(fine_wps) - 1):
        wx1, wy1 = fine_wps[j]
        wx2, wy2 = fine_wps[j + 1]
        target_th = fine_headings[j + 1]

        is_last = (j == len(fine_wps) - 2)
        if is_last:
            wx2, wy2 = RS_GOAL_X, RS_GOAL_Y
            target_th = RS_GOAL_TH

        if is_last:
            seg = _try_rs_connect(wx1, wy1, cur_th, wx2, wy2, target_th,
                                  collision_fn, max_paths=max_rs_paths)
            if seg is None:
                for small_off in [0.04, -0.04, 0.08, -0.08]:
                    seg = _try_rs_connect(wx1, wy1, cur_th,
                                          wx2, wy2, target_th + small_off,
                                          collision_fn, max_paths=max_rs_paths)
                    if seg is not None:
                        break
            actual_th = target_th
        else:
            seg, actual_th = _try_connect_with_heading_variants(
                wx1, wy1, cur_th, wx2, wy2, target_th,
                collision_fn, max_paths=max_rs_paths)

        if seg is None:
            return False, None

        full_traj.extend(seg)
        cur_th = actual_th

    if full_traj:
        ex, ey, eth = full_traj[-1]
        if ex <= 2.25 and abs(ey) <= 0.18 and abs(eth) <= ALIGN_GOAL_DYAW:
            return True, full_traj

    return False, None
