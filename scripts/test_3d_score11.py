import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.20)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])
grid.build_3d_map()

print("h_3d at 4.85, -1.81, -2.94:", grid.get_3d_heuristic(4.85, -1.81, -2.94))
