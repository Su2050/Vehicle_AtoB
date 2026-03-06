import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

cgx = int((2.00 - grid.min_x) / grid.res)
cgy = int((-2.33 - grid.min_y) / grid.res)
print(f"2.00, -2.33 -> cgx={cgx}, cgy={cgy}")
print(f"Is obs: {grid.obs_map[cgx][cgy]}")
