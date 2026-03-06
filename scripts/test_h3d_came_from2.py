from test_fast_obs import grid
import math

gx, gy, gth, gdir = 28, 43, 12, 1
d = grid.dist_3d[gx][gy][gth][gdir]
print(f"Target: gx={gx}, gy={gy}, gth={gth}, dir={gdir}, d={d}")

wx = grid.min_x + gx * grid.res_3d + grid.res_3d/2
wy = grid.min_y + gy * grid.res_3d + grid.res_3d/2
wth = gth * grid.res_th

for d_step in [0.25, -0.25]:
    for steer in [0.0, 0.4, -0.4]:
        new_dir = 0 if d_step > 0 else 1
        cost = 0.25 if d_step == -0.25 else 0.375
        if steer != 0.0: cost += 0.1
        if new_dir != gdir: cost += 3.0
        
        new_wth = wth - steer * d_step
        new_wx = wx - d_step * math.cos(new_wth)
        new_wy = wy - d_step * math.sin(new_wth)
        
        new_wth_norm = new_wth % (2 * math.pi)
        nth_g = int(new_wth_norm / grid.res_th) % grid.nth
        nx_g = int((new_wx - grid.min_x) / grid.res_3d)
        ny_g = int((new_wy - grid.min_y) / grid.res_3d)
        
        if 0 <= nx_g < grid.nx_3d and 0 <= ny_g < grid.ny_3d:
            prev_d = grid.dist_3d[nx_g][ny_g][nth_g][new_dir]
            print(f"Neighbor gx={nx_g}, gy={ny_g}, gth={nth_g}, dir={new_dir}, d_step={d_step}, steer={steer}, prev_d={prev_d:.2f}, cost={cost:.2f}, total={prev_d+cost:.2f}")
