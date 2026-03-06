import json
from planner_obs_v2 import DijkstraGrid
from planner_obs import _preprocess_obstacles

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 2670)

dijkstra_grid = DijkstraGrid(2.25, 0.0, grid_res=0.10, inflate_radius=0.20)
dijkstra_grid.build_map(case['obstacles'], start_x=case['start']['x'], start_y=case['start']['y'])

print(f"Grid size: {dijkstra_grid.nx} x {dijkstra_grid.ny}")
print(f"Goal: {dijkstra_grid.goal_ix}, {dijkstra_grid.goal_iy}")
print(f"Start: {dijkstra_grid.start_ix}, {dijkstra_grid.start_iy}")

