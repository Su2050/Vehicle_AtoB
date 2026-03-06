import math
from primitives import init_primitives
from collision import check_collision
from planner_obs import _preprocess_obstacles
import json
import primitives

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

obs = cases[0]['obstacles']
fast_obs = _preprocess_obstacles(obs)
precomp_prim = init_primitives()

cx, cy, cth = 2.05, -3.24, -1.50
cos_th = math.cos(cth)
sin_th = math.sin(cth)

act = ('R', 0.0, 0.33)
for n, N, traj_base in precomp_prim:
    if n == act:
        for dx, dy, dth, cdth, sdth in traj_base:
            nx = cx + dx * cos_th - dy * sin_th
            ny = cy + dx * sin_th + dy * cos_th
            nth = cth + dth
            sin_nth = math.sin(nth)
            cos_nth = math.cos(nth)
            
            front_offset = min(primitives.VEHICLE_CHECK_OFFSETS)
            tip_lat = ny - front_offset * sin_nth
            sc = (0.15 + (nx - 1.87) * 0.8) if nx > 1.87 else 0.15
            print(f"nx={nx:.2f}, ny={ny:.2f}, nth={nth:.2f}, tip_lat={tip_lat:.2f}, sc={sc:.2f}")
            
            valid, reason = check_collision(nx, ny, nth, sin_nth, cos_nth, no_corridor=False, obstacles=fast_obs)
            print(f"Valid: {valid}, Reason: {reason}")
print(f"nx <= 2.05: {nx <= 2.05}")
