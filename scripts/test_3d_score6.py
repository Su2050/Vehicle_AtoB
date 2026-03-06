import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])
grid.build_3d_map()

print("h_3d at 2.21, -2.98, -1.56:", grid.get_3d_heuristic(2.21, -2.98, -1.56))
