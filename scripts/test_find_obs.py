from heuristic import DijkstraGrid
import math

obs = [
    (6.03, 6.66, 1.65, 2.06),
    (3.28, 4.21, -1.4, 0.43),
    (6.79, 7.15, 2.57, 4.25),
    (3.87, 4.39, -2.43, -1.61)
]
boundary_obs = [
    (-1.0, 0.0, -6.0, 6.0),
    (11.0, 12.0, -6.0, 6.0)
]
all_obs = obs + boundary_obs

grid = DijkstraGrid(2.10, 0.0, grid_res=0.10, inflate_radius=0.20)
grid.build_map(obs, start_x=5.05, start_y=2.67)

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
