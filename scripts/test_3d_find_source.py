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

# Target state A
gx_A, gy_A, gth_A, gdir_A = 15, 29, 1, 0
h_A = dijkstra_grid.dist_3d[gx_A][gy_A][gth_A][gdir_A]
print(f"Target A: gx={gx_A}, gy={gy_A}, gth={gth_A}, gear={gdir_A}, h={h_A:.2f}")

# Find which B generated A
found = False
for gx_B in range(max(0, gx_A - 5), min(dijkstra_grid.nx_3d, gx_A + 6)):
    for gy_B in range(max(0, gy_A - 5), min(dijkstra_grid.ny_3d, gy_A + 6)):
        for gth_B in range(dijkstra_grid.nth):
            for gdir_B in [0, 1]:
                h_B = dijkstra_grid.dist_3d[gx_B][gy_B][gth_B][gdir_B]
                if h_B >= h_A: continue
                
                # Try to expand B to see if it generates A
                wx_B = dijkstra_grid.min_x + gx_B * dijkstra_grid.res_3d + dijkstra_grid.res_3d/2
                wy_B = dijkstra_grid.min_y + gy_B * dijkstra_grid.res_3d + dijkstra_grid.res_3d/2
                wth_B = -math.pi + gth_B * dijkstra_grid.res_th
                
                for d_step in [0.5, -0.5]:
                    for steer in [0.0, 0.25, -0.25]:
                        new_dir = 0 if d_step > 0 else 1
                        cost = 0.5 if d_step > 0 else 1.25
                        if steer != 0.0: cost += 0.1
                        if new_dir != gdir_B: cost += 3.0
                        
                        new_wth = wth_B - steer * d_step
                        new_wx = wx_B - d_step * math.cos(new_wth)
                        new_wy = wy_B - d_step * math.sin(new_wth)
                        
                        new_wth_norm = new_wth
                        while new_wth_norm < -math.pi: new_wth_norm += 2 * math.pi
                        while new_wth_norm >= math.pi: new_wth_norm -= 2 * math.pi
                        nth_g = int((new_wth_norm + math.pi) / dijkstra_grid.res_th) % dijkstra_grid.nth
                        nx_g = int((new_wx - dijkstra_grid.min_x) / dijkstra_grid.res_3d)
                        ny_g = int((new_wy - dijkstra_grid.min_y) / dijkstra_grid.res_3d)
                        
                        if nx_g == gx_A and ny_g == gy_A and nth_g == gth_A and new_dir == gdir_A:
                            print(f"Found source B: gx={gx_B}, gy={gy_B}, gth={gth_B}, gear={gdir_B}, h={h_B:.2f}")
                            print(f"  Transition: d_step={d_step}, steer={steer}, cost={cost:.2f}")
                            print(f"  Calculated h_A: {h_B + cost:.2f}")
                            found = True

if not found:
    print("Source not found within neighborhood!")
