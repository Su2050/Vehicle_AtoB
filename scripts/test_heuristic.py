import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y
from planner_obs_v2 import _make_heuristic_fn_v2

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

start_h, start_pure = grid.get_heuristic(case['start']['x'], case['start']['y'], case['start']['th'])
h_fn = _make_heuristic_fn_v2(grid, start_pure, allow_uphill=100.0, h_weight=1.5)

print("h at 2.5, -1.92, th=1.5:", h_fn(2.5, -1.92, 1.5))
