import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

gx = int((4.0 - grid.min_x) / grid.res)

print(f"gx = {gx}")
for y in range(50, 62):
    wy = grid.min_y + y * grid.res
    print(f"y={y} (wy={wy:.2f}): obs_map={grid.obs_map[gx][y]}")

