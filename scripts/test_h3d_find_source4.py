from test_fast_obs import grid
import math

target_gx, target_gy, target_gth, target_dir = 13, 22, 12, 1
target_d = grid.dist_3d[target_gx][target_gy][target_gth][target_dir]
print(f"Target: gx={target_gx}, gy={target_gy}, gth={target_gth}, dir={target_dir}, d={target_d}")

found = False
for gx in range(grid.nx_3d):
    for gy in range(grid.ny_3d):
        for gth in range(grid.nth):
            for gdir in [0, 1]:
                d = grid.dist_3d[gx][gy][gth][gdir]
                if d == float('inf'): continue
                
                wx = grid.min_x + gx * grid.res_3d + grid.res_3d/2
                wy = grid.min_y + gy * grid.res_3d + grid.res_3d/2
                wth = gth * grid.res_th
                
                for d_step in [0.25, -0.25]:
                    for steer in [0.0, 0.15, -0.15]:
                        new_dir = 0 if d_step > 0 else 1
                        cost = 0.25 if d_step > 0 else 0.625
                        if steer != 0.0: cost += 0.1
                        if new_dir != gdir: cost += 3.0
                        
                        new_wth = wth - steer * d_step
                        new_wx = wx - d_step * math.cos(new_wth)
                        new_wy = wy - d_step * math.sin(new_wth)
                        
                        new_wth_norm = new_wth % (2 * math.pi)
                        nth_g = int(new_wth_norm / grid.res_th) % grid.nth
                        nx_g = int((new_wx - grid.min_x) / grid.res_3d)
                        ny_g = int((new_wy - grid.min_y) / grid.res_3d)
                        
                        if nx_g == target_gx and ny_g == target_gy and nth_g == target_gth and new_dir == target_dir:
                            if abs(d + cost - target_d) < 1e-5:
                                print(f"Found source: gx={gx}, gy={gy}, gth={gth}, dir={gdir}, d={d:.2f}, cost={cost:.2f}")
                                found = True
if not found:
    print("No source found!")
