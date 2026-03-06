from test_fast_obs import grid
import math

gx, gy, gth = 12, 22, 11

print(f"Start: gx={gx}, gy={gy}, gth={gth}, h_3d={min(grid.dist_3d[gx][gy][gth])}")

curr_gx, curr_gy, curr_gth = gx, gy, gth
for i in range(20):
    best_d = float('inf')
    best_next = None
    
    wx = grid.min_x + curr_gx * grid.res_3d + grid.res_3d/2
    wy = grid.min_y + curr_gy * grid.res_3d + grid.res_3d/2
    wth = curr_gth * grid.res_th
    
    for d_step in [0.25, -0.25]:
        for steer in [0.0, 0.15, -0.15]:
            new_wth = wth - steer * d_step
            new_wx = wx + d_step * math.cos(new_wth)
            new_wy = wy + d_step * math.sin(new_wth)
            
            nx_g = int((new_wx - grid.min_x) / grid.res_3d)
            ny_g = int((new_wy - grid.min_y) / grid.res_3d)
            
            if 0 <= nx_g < grid.nx_3d and 0 <= ny_g < grid.ny_3d:
                th_norm = new_wth % (2 * math.pi)
                if th_norm < 0: th_norm += 2 * math.pi
                nth_g = int(th_norm / grid.res_th) % grid.nth
                
                d = min(grid.dist_3d[nx_g][ny_g][nth_g])
                if d < best_d:
                    best_d = d
                    best_next = (nx_g, ny_g, nth_g, d_step, steer)
                    
    if best_next:
        curr_gx, curr_gy, curr_gth, d_step, steer = best_next
        print(f"Step {i+1}: gx={curr_gx}, gy={curr_gy}, gth={curr_gth}, d_step={d_step}, steer={steer}, h_3d={best_d:.2f}")
    else:
        print("No path found")
        break
