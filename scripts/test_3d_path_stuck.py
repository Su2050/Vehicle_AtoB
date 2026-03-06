import json
from planner_obs_v2 import DijkstraGrid
from planner_obs import _preprocess_obstacles
import math

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 9192)

dijkstra_grid = DijkstraGrid(2.25, 0.0, grid_res=0.10, inflate_radius=0.20)
dijkstra_grid.build_map(case['obstacles'], start_x=case['start']['x'], start_y=case['start']['y'])
dijkstra_grid.build_3d_map()

x, y, th = 5.92, 3.16, -2.63
gx = int((x - dijkstra_grid.min_x) / dijkstra_grid.res_3d)
gy = int((y - dijkstra_grid.min_y) / dijkstra_grid.res_3d)
th_norm = th
while th_norm < -math.pi: th_norm += 2 * math.pi
while th_norm >= math.pi: th_norm -= 2 * math.pi
gth = int((th_norm + math.pi) / dijkstra_grid.res_th) % dijkstra_grid.nth

print(f"Stuck state: gx={gx}, gy={gy}, gth={gth}")
print(f"Heuristics around stuck state:")
for dx in [-1, 0, 1]:
    for dy in [-1, 0, 1]:
        for dth in [-1, 0, 1]:
            nx, ny, nth = gx + dx, gy + dy, (gth + dth) % dijkstra_grid.nth
            if 0 <= nx < dijkstra_grid.nx_3d and 0 <= ny < dijkstra_grid.ny_3d:
                h0 = dijkstra_grid.dist_3d[nx][ny][nth][0]
                h1 = dijkstra_grid.dist_3d[nx][ny][nth][1]
                if h0 != float('inf') or h1 != float('inf'):
                    print(f"  dx={dx}, dy={dy}, dth={dth}: h0={h0:.2f}, h1={h1:.2f}")

