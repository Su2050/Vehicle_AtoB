import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

gx_2d = int((2.00 - grid.min_x) / grid.res)
gy_2d = int((-2.50 - grid.min_y) / grid.res)

print(f"2.00, -2.50 -> gx_2d={gx_2d}, gy_2d={gy_2d}")
print(f"Is obs: {grid.obs_map[gx_2d][gy_2d]}")
