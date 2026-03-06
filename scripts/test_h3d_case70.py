import json
from planner_obs_v2 import DijkstraGrid
from planner_obs import _preprocess_obstacles
import math

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 70)

dijkstra_grid = DijkstraGrid(2.25, 0.0, grid_res=0.15, inflate_radius=0.30)
dijkstra_grid.build_map(case['obstacles'], start_x=case['start']['x'], start_y=case['start']['y'])

print(f"Start: x={case['start']['x']}, y={case['start']['y']}, th={case['start']['th']}")
h_grid, pure_dist = dijkstra_grid.get_heuristic(case['start']['x'], case['start']['y'], case['start']['th'])
print(f"h_grid={h_grid}, pure_dist={pure_dist}")

