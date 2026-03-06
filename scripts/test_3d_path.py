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

print("Start heuristic:", dijkstra_grid.get_3d_heuristic(case['start']['x'], case['start']['y'], case['start']['th']))

x, y, th = case['start']['x'], case['start']['y'], case['start']['th']
gear = 0

for _ in range(20):
    gx = int((x - dijkstra_grid.min_x) / dijkstra_grid.res_3d)
    gy = int((y - dijkstra_grid.min_y) / dijkstra_grid.res_3d)
    
    th_norm = th
    while th_norm < -math.pi: th_norm += 2 * math.pi
    while th_norm >= math.pi: th_norm -= 2 * math.pi
    gth = int((th_norm + math.pi) / dijkstra_grid.res_th) % dijkstra_grid.nth
    
    print(f"Current: x={x:.2f}, y={y:.2f}, th={th:.2f}, gx={gx}, gy={gy}, gth={gth}, gear={gear}, h={dijkstra_grid.dist_3d[gx][gy][gth][gear]:.2f}")
    
    best_h = float('inf')
    best_n = None
    
    for d_step in [0.25, -0.25]:
        for steer in [0.0, 0.15, -0.15]:
            new_dir = 0 if d_step > 0 else 1
            new_wth = th - steer * d_step
            new_wx = x - d_step * math.cos(new_wth)
            new_wy = y - d_step * math.sin(new_wth)
            
            new_wth_norm = new_wth % (2 * math.pi)
            nth_g = int(new_wth_norm / dijkstra_grid.res_th) % dijkstra_grid.nth
            nx_g = int((new_wx - dijkstra_grid.min_x) / dijkstra_grid.res_3d)
            ny_g = int((new_wy - dijkstra_grid.min_y) / dijkstra_grid.res_3d)
            
            if 0 <= nx_g < dijkstra_grid.nx_3d and 0 <= ny_g < dijkstra_grid.ny_3d:
                h = dijkstra_grid.dist_3d[nx_g][ny_g][nth_g][new_dir]
                if h < best_h:
                    best_h = h
                    best_n = (new_wx, new_wy, new_wth, new_dir)
                    
    if best_n is None or best_h >= dijkstra_grid.dist_3d[gx][gy][gth][gear]:
        print("Stuck!")
        break
        
    x, y, th, gear = best_n
