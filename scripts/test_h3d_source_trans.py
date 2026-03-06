from test_fast_obs import grid
import math

gx, gy, gth, gdir = 28, 44, 13, 1
wx = grid.min_x + gx * grid.res_3d + grid.res_3d/2
wy = grid.min_y + gy * grid.res_3d + grid.res_3d/2
wth = gth * grid.res_th

for d_step in [0.25, -0.25]:
    for steer in [0.0, 0.1, -0.1]:
        new_dir = 0 if d_step > 0 else 1
        new_wth = wth - steer * d_step
        new_wx = wx - d_step * math.cos(new_wth)
        new_wy = wy - d_step * math.sin(new_wth)
        
        new_wth_norm = new_wth % (2 * math.pi)
        nth_g = int(new_wth_norm / grid.res_th) % grid.nth
        nx_g = int((new_wx - grid.min_x) / grid.res_3d)
        ny_g = int((new_wy - grid.min_y) / grid.res_3d)
        
        if nx_g == 28 and ny_g == 43 and nth_g == 12:
            print(f"Transition: d_step={d_step}, steer={steer}, new_wth={new_wth:.3f}, nth_g={nth_g}")
