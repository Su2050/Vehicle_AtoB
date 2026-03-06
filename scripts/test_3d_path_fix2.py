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

x, y, th = 4.61, 2.43, -2.63
gx = int((x - dijkstra_grid.min_x) / dijkstra_grid.res_3d)
gy = int((y - dijkstra_grid.min_y) / dijkstra_grid.res_3d)
th_norm = th
while th_norm < -math.pi: th_norm += 2 * math.pi
while th_norm >= math.pi: th_norm -= 2 * math.pi
gth = int((th_norm + math.pi) / dijkstra_grid.res_th) % dijkstra_grid.nth
gear = 0

print(f"Stuck state: gx={gx}, gy={gy}, gth={gth}, gear={gear}, h={dijkstra_grid.dist_3d[gx][gy][gth][gear]:.2f}")

for d_step in [0.5, -0.5]:
    for steer in [0.0, 0.25, -0.25]:
        new_dir = 1 if d_step > 0 else 0
        cost_transition = 1.25 if d_step > 0 else 0.5
        if steer != 0.0: cost_transition += 0.1
        if new_dir != gear: cost_transition += 3.0
        
        new_wth = th - steer * d_step
        new_wx = x - d_step * math.cos(new_wth)
        new_wy = y - d_step * math.sin(new_wth)
        
        new_wth_norm = new_wth
        while new_wth_norm < -math.pi: new_wth_norm += 2 * math.pi
        while new_wth_norm >= math.pi: new_wth_norm -= 2 * math.pi
        nth_g = int((new_wth_norm + math.pi) / dijkstra_grid.res_th) % dijkstra_grid.nth
        nx_g = int((new_wx - dijkstra_grid.min_x) / dijkstra_grid.res_3d)
        ny_g = int((new_wy - dijkstra_grid.min_y) / dijkstra_grid.res_3d)
        
        if 0 <= nx_g < dijkstra_grid.nx_3d and 0 <= ny_g < dijkstra_grid.ny_3d:
            h = dijkstra_grid.dist_3d[nx_g][ny_g][nth_g][new_dir]
            total_cost = h + cost_transition
            print(f"Action d_step={d_step}, steer={steer}: nx={nx_g}, ny={ny_g}, nth={nth_g}, dir={new_dir}, h={h:.2f}, total={total_cost:.2f}")

