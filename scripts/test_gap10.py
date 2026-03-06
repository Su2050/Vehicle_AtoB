import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y
import math
import primitives

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.25)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

primitives.init_primitives()
is_obs = False
for offset in primitives.VEHICLE_CHECK_OFFSETS:
    cx = 4.5 - offset * math.cos(3.14)
    cy = -0.47 - offset * math.sin(3.14)
    cgx = int((cx - grid.min_x) / grid.res)
    cgy = int((cy - grid.min_y) / grid.res)
    print(f"Circle at {cx:.2f}, {cy:.2f} -> obs_map: {grid.obs_map[cgx][cgy]}")
    if grid.obs_map[cgx][cgy]:
        is_obs = True

print(f"is_obs: {is_obs}")
