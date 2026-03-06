import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.25)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

gx = int((4.5 - grid.min_x) / grid.res)
gy = int((-0.50 - grid.min_y) / grid.res)

print(f"obs_map at (4.5, -0.50): {grid.obs_map[gx][gy]}")
