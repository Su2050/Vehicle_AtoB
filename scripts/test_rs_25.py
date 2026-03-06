from collision import check_collision
from planner_obs import _preprocess_obstacles
import json
import rs

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

fast_obs = _preprocess_obstacles(cases[0]['obstacles'])

cx, cy, cth = 2.5, -0.5, 0.0
RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
MIN_TURN_RADIUS = 1.97

trajs = rs.rs_sample_path_multi(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS, step=0.05)
for i, traj in enumerate(trajs):
    if not traj: continue
    ok = True
    for pt in traj:
        valid, reason = check_collision(pt[0], pt[1], pt[2], no_corridor=False, obstacles=fast_obs)
        if not valid:
            ok = False
            print(f"Traj {i} rejected: {reason} at {pt[0]:.2f}, {pt[1]:.2f}")
            break
    if ok:
        print(f"Traj {i} OK!")
