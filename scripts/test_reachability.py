from test_fast_obs import grid

print("Checking Reachability from Goal")
# Print reachable states at y=4.54
reachable = []
for gx in range(grid.nx_3d):
    for gth in range(grid.nth):
        for dir in [0, 1]:
            if grid.dist_3d[gx][42][gth][dir] < float('inf'): # gy=42 is y=4.54
                wx = grid.min_x + gx * grid.res_3d + grid.res_3d/2
                wth = gth * grid.res_th
                reachable.append((wx, wth, dir, grid.dist_3d[gx][42][gth][dir]))

print(f"Found {len(reachable)} reachable states at y=4.54")
for r in reachable[:10]:
    print(f"  x={r[0]:.2f}, th={r[1]:.2f}, dir={r[2]}, dist={r[3]:.2f}")

max_y = -float('inf')
for gx in range(grid.nx_3d):
    for gy in range(grid.ny_3d):
        for gth in range(grid.nth):
            for dir in [0, 1]:
                if grid.dist_3d[gx][gy][gth][dir] < float('inf'):
                    wy = grid.min_y + gy * grid.res_3d + grid.res_3d/2
                    if wy > max_y:
                        max_y = wy

print(f"Max reachable y: {max_y}")

print("Checking obs_map at x=1.0, y=0.5")
cgx = int((1.0 - grid.min_x) / grid.res)
cgy = int((0.5 - grid.min_y) / grid.res)
print(f"obs_map[{cgx}][{cgy}] = {grid.obs_map[cgx][cgy]}")

print("Finding obstacle at (20, 65)")
for o in all_obs:
    xmin, xmax, ymin, ymax = o
    gx_min = int((xmin - grid.min_x) / grid.res)
    gy_min = int((ymin - grid.min_y) / grid.res)
    gx_max = int((xmax - grid.min_x) / grid.res)
    gy_max = int((ymax - grid.min_y) / grid.res)
    
    inf_cells = int(math.ceil(grid.inflate_radius / grid.res))
    hx_min = max(0, gx_min - inf_cells)
    hy_min = max(0, gy_min - inf_cells)
    hx_max = min(grid.nx - 1, gx_max + inf_cells)
    hy_max = min(grid.ny - 1, gy_max + inf_cells)
    
    if hx_min <= 20 <= hx_max and hy_min <= 65 <= hy_max:
        print(f"  Caused by obstacle: {o}")
