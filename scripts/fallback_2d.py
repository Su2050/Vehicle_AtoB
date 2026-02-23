"""
fallback_2d.py — 2D Dijkstra skeleton path + RS segment stitching.

When the fine-resolution A* times out on complex obstacle layouts, this module
traces a coarse 2D path on the DijkstraGrid (0.15m resolution) and attempts to
stitch the path with collision-free Reeds-Shepp segments.
"""

import math
import heapq
import rs
from primitives import (
    DT, MIN_TURN_RADIUS, ALIGN_GOAL_DYAW,
    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
)


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
                                        collision_fn, max_paths=10):
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
        th2 = th2_nom + off
        traj = _try_rs_connect(x1, y1, th1, x2, y2, th2, collision_fn,
                               max_paths=max_paths)
        if traj is not None:
            if best_traj is None or len(traj) < len(best_traj):
                best_traj = traj
                best_th2 = th2
    return best_traj, best_th2


def _stitch_waypoints(waypoints, start_th, collision_fn, max_rs_paths):
    """
    Stitch waypoints with RS segments. Last segment targets exact goal pose.
    Returns (success, trajectory).
    """
    nominal_headings = _assign_headings(waypoints)
    full_traj = []
    cur_th = start_th

    for i in range(len(waypoints) - 1):
        wx1, wy1 = waypoints[i]
        wx2, wy2 = waypoints[i + 1]
        target_th = nominal_headings[i + 1]

        is_last = (i == len(waypoints) - 2)
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


def plan_2d_fallback(dijkstra_grid, start_x, start_y, start_th,
                     collision_fn, spacing=1.5, max_rs_paths=12,
                     obstacles=None):
    """
    2D skeleton fallback planner.

    1. Trace a 2D path on the Dijkstra grid from start toward goal
    2. Simplify into waypoints
    3. Stitch consecutive waypoints with RS segments
    4. If RS stitching fails, retry with wider-inflation Dijkstra path

    Returns (success, trajectory) where trajectory is [(x,y,th), ...].
    """
    raw_path = _trace_gradient_path(dijkstra_grid, start_x, start_y)

    candidates = []
    if len(raw_path) >= 2:
        candidates.append(raw_path)

    if obstacles:
        alt = _dijkstra_2d_path(
            start_x, start_y, RS_GOAL_X, RS_GOAL_Y, obstacles)
        if len(alt) >= 2:
            candidates.append(alt)

    spacings = [spacing, spacing * 1.5, spacing * 0.6, spacing * 2.0]
    for raw in candidates:
        for sp in spacings:
            wps = _simplify_path(raw, min_spacing=sp)
            if len(wps) < 2:
                continue
            ok, traj = _stitch_waypoints(
                wps, start_th, collision_fn, max_rs_paths)
            if ok:
                return True, traj

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
