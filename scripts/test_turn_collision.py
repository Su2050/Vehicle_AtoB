from collision import check_collision
from planner_obs import _preprocess_obstacles
from primitives import init_primitives
import math
import json

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

obs = cases[0]['obstacles']
fast_obs = _preprocess_obstacles(obs)
precomp_prim = init_primitives()

cx, cy, cth = 2.16, -0.97, -1.64
cos_th = math.cos(cth)
sin_th = math.sin(cth)

for act, N, traj_base in precomp_prim:
    if act[0] == 'F' and act[1] > 0: # Turn left
        ok = True
        for dx, dy, dth, cdth, sdth in traj_base:
            nx = cx + dx * cos_th - dy * sin_th
            ny = cy + dx * sin_th + dy * cos_th
            nth = cth + dth
            sin_nth = math.sin(nth)
            cos_nth = math.cos(nth)
            valid, reason = check_collision(nx, ny, nth, sin_nth, cos_nth, no_corridor=False, obstacles=fast_obs)
            if not valid:
                ok = False
                print(f"Action {act} rejected: {reason} at {nx:.2f}, {ny:.2f}")
                break
        if ok:
            print(f"Action {act} OK")

act = ('F', 1.0, 0.67)
for n, N, traj_base in precomp_prim:
    if n == act:
        dx, dy, dth, _, _ = traj_base[-1]
        nx = cx + dx * cos_th - dy * sin_th
        ny = cy + dx * sin_th + dy * cos_th
        nth = cth + dth
        print(f"End state: {nx:.2f}, {ny:.2f}, {nth:.2f}")
        
        # Check heuristic
        from test_fast_obs import grid
        grid.build_map(obs)
        grid.build_3d_map()
        h_3d = grid.get_3d_heuristic(nx, ny, nth)
        import rs
        RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
        MIN_TURN_RADIUS = 1.97
        rs_dist = rs.rs_distance_pose(nx, ny, nth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
        h_rs = rs_dist / 0.25
        h = max(h_3d * 1.5, h_rs)
        
        dx_to_goal = abs(nx - RS_GOAL_X)
        dy_to_goal = abs(ny - RS_GOAL_Y)
        th_err = abs(nth - RS_GOAL_TH)
        while th_err > math.pi: th_err -= 2 * math.pi
        th_err = abs(th_err)
        if dx_to_goal < 2.0 and dy_to_goal < 1.0:
            if th_err > 0.2:
                penalty = th_err * (2.0 - dx_to_goal) * (1.0 - dy_to_goal) * 10.0
                h += penalty
                print(f"Heading penalty: {penalty:.2f}")
        print(f"h_3d={h_3d:.2f}, h_rs={h_rs:.2f}, h={h:.2f}")
