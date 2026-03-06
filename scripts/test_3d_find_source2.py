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

gx_A, gy_A, gth_A, gdir_A = 15, 29, 1, 0
h_A = dijkstra_grid.dist_3d[gx_A][gy_A][gth_A][gdir_A]
print(f"Target A: gx={gx_A}, gy={gy_A}, gth={gth_A}, gear={gdir_A}, h={h_A:.2f}")

gx_B, gy_B, gth_B, gdir_B = 13, 28, 2, 0
h_B = dijkstra_grid.dist_3d[gx_B][gy_B][gth_B][gdir_B]
print(f"Source B: gx={gx_B}, gy={gy_B}, gth={gth_B}, gear={gdir_B}, h={h_B:.2f}")

wx_B = dijkstra_grid.min_x + gx_B * dijkstra_grid.res_3d + dijkstra_grid.res_3d/2
wy_B = dijkstra_grid.min_y + gy_B * dijkstra_grid.res_3d + dijkstra_grid.res_3d/2
wth_B = -math.pi + gth_B * dijkstra_grid.res_th

d_step = 0.5
steer = 0.25

new_wth = wth_B - steer * d_step
new_wx = wx_B - d_step * math.cos(new_wth)
new_wy = wy_B - d_step * math.sin(new_wth)

print(f"B state: wx={wx_B:.2f}, wy={wy_B:.2f}, wth={wth_B:.2f}")
print(f"A state (calculated): wx={new_wx:.2f}, wy={new_wy:.2f}, wth={new_wth:.2f}")

wx_A = dijkstra_grid.min_x + gx_A * dijkstra_grid.res_3d + dijkstra_grid.res_3d/2
wy_A = dijkstra_grid.min_y + gy_A * dijkstra_grid.res_3d + dijkstra_grid.res_3d/2
wth_A = -math.pi + gth_A * dijkstra_grid.res_th
print(f"A state (center): wx={wx_A:.2f}, wy={wy_A:.2f}, wth={wth_A:.2f}")

