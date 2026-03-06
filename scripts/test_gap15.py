import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.25)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

gx = int((4.89 - grid.min_x) / grid.res)
gy = int((-0.44 - grid.min_y) / grid.res)

print(f"obs_map at (4.89, -0.44): {grid.obs_map[gx][gy]}")

# Find which obstacle causes it
inf_cells = 3
from primitives import MAX_PLANNABLE_Y
boundary_obs = [
    (-1.0, 12.0, MAX_PLANNABLE_Y, 6.0),
    (-1.0, 12.0, -6.0, -MAX_PLANNABLE_Y),
    (-1.0, 0.0, -6.0, 6.0),
    (11.0, 12.0, -6.0, 6.0)
]
all_obstacles = list(case['obstacles']) + boundary_obs

for obs in all_obstacles:
    if isinstance(obs, tuple):
        min_wx, max_wx, min_wy, max_wy = obs
    else:
        ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
        min_wx = min(ox, ox + ow)
        max_wx = max(ox, ox + ow)
        min_wy = min(oy, oy + oh)
        max_wy = max(oy, oy + oh)
    gx_min, gy_min = grid._world_to_grid(min_wx, min_wy)
    gx_max, gy_max = grid._world_to_grid(max_wx, max_wy)
    
    hx_min = max(0, gx_min - inf_cells)
    hy_min = max(0, gy_min - inf_cells)
    hx_max = min(grid.nx - 1, gx_max + inf_cells)
    hy_max = min(grid.ny - 1, gy_max + inf_cells)
    
    if hx_min <= gx <= hx_max and hy_min <= gy <= hy_max:
        print(f"Caused by: {obs}")

