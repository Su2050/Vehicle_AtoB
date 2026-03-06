import math
def trace_path(grid, start_x, start_y, start_th):
    gx = int((start_x - grid.min_x) / grid.res_3d)
    gy = int((start_y - grid.min_y) / grid.res_3d)
    gth = int((start_th % (2*math.pi)) / grid.res_th) % grid.nth
    
    path = []
    curr = (gx, gy, gth, 0)
    if grid.dist_3d[gx][gy][gth][1] < grid.dist_3d[gx][gy][gth][0]:
        curr = (gx, gy, gth, 1)
        
    while True:
        path.append(curr)
        if grid.dist_3d[curr[0]][curr[1]][curr[2]][curr[3]] == 0:
            break
        
        best_next = None
        best_d = grid.dist_3d[curr[0]][curr[1]][curr[2]][curr[3]]
        
        wx = grid.min_x + curr[0] * grid.res_3d + grid.res_3d/2
        wy = grid.min_y + curr[1] * grid.res_3d + grid.res_3d/2
        wth = curr[2] * grid.res_th
        
        # Forward kinematics
        for d_step in [0.25, -0.25]:
            for steer in [0.0, 0.4, -0.4]:
                next_wth = wth + d_step * steer
                next_wth_norm = next_wth % (2 * math.pi)
                next_wx = wx + d_step * math.cos(next_wth_norm)
                next_wy = wy + d_step * math.sin(next_wth_norm)
                
                nx_g = int((next_wx - grid.min_x) / grid.res_3d)
                ny_g = int((next_wy - grid.min_y) / grid.res_3d)
                nth_g = int(next_wth_norm / grid.res_th) % grid.nth
                next_dir = 0 if d_step > 0 else 1
                
                if 0 <= nx_g < grid.nx_3d and 0 <= ny_g < grid.ny_3d:
                    if grid.dist_3d[nx_g][ny_g][nth_g][next_dir] < best_d - 1e-5:
                        best_d = grid.dist_3d[nx_g][ny_g][nth_g][next_dir]
                        best_next = (nx_g, ny_g, nth_g, next_dir)
        
        if best_next is None or best_next in path:
            break
        curr = best_next
        
    print(f"Path length: {len(path)}")
    for p in path[:20]:
        print(f"  {grid.min_x + p[0]*grid.res_3d:.2f}, {grid.min_y + p[1]*grid.res_3d:.2f}, {p[2]*grid.res_th:.2f}, dir={p[3]}, dist={grid.dist_3d[p[0]][p[1]][p[2]][p[3]]}")

from test_fast_obs import grid
trace_path(grid, 6.14, 4.54, -1.55)
