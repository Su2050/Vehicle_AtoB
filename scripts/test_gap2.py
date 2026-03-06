import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

gx = int((4.0 - grid.min_x) / grid.res)
gy = int((-0.47 - grid.min_y) / grid.res)

print(f"obs_map at gap center (4.0, -0.47): {grid.obs_map[gx][gy]}")

# Check circles for th=0.0
import math
import primitives
primitives.init_primitives()
is_obs = False
for offset in primitives.VEHICLE_CHECK_OFFSETS:
    cx = 4.0 - offset * math.cos(0.0)
    cy = -0.47 - offset * math.sin(0.0)
    cgx = int((cx - grid.min_x) / grid.res)
    cgy = int((cy - grid.min_y) / grid.res)
    print(f"Circle at {cx}, {cy} -> obs_map: {grid.obs_map[cgx][cgy]}")
    if grid.obs_map[cgx][cgy]:
        is_obs = True

print(f"is_obs: {is_obs}")
