from test_fast_obs import grid
import math

gx, gy, gth = 13, 22, 12
print(f"dist_3d[{gx}][{gy}][{gth}]: {grid.dist_3d[gx][gy][gth]}")

wx = grid.min_x + gx * grid.res_3d + grid.res_3d/2
wy = grid.min_y + gy * grid.res_3d + grid.res_3d/2
wth = gth * grid.res_th

for d_step in [0.25, -0.25]:
    for steer in [0.0, 0.15, -0.15]:
        new_wth = wth - steer * d_step
        new_wx = wx - d_step * math.cos(new_wth)
        new_wy = wy - d_step * math.sin(new_wth)
        
        nx_g = int((new_wx - grid.min_x) / grid.res_3d)
        ny_g = int((new_wy - grid.min_y) / grid.res_3d)
        
        if 0 <= nx_g < grid.nx_3d and 0 <= ny_g < grid.ny_3d:
            th_norm = new_wth % (2 * math.pi)
            if th_norm < 0: th_norm += 2 * math.pi
            nth_g = int(th_norm / grid.res_th) % grid.nth
            
            d = grid.dist_3d[nx_g][ny_g][nth_g]
            print(f"Neighbor d_step={d_step}, steer={steer}: gx={nx_g}, gy={ny_g}, gth={nth_g}, d={d}")
