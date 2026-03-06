import math
from primitives import init_primitives
from collision import check_collision
from planner_obs import _preprocess_obstacles
import json

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

obs = cases[0]['obstacles']
fast_obs = _preprocess_obstacles(obs)
precomp_prim = init_primitives()

cx, cy, cth = 2.05, -3.24, -1.50
cos_th = math.cos(cth)
sin_th = math.sin(cth)

for act, N, traj_base in precomp_prim:
    if act[0] == 'R':
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
