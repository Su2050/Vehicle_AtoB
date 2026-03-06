import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

gx_2d = int((2.75 - grid.min_x) / grid.res)
gy_2d = int((0.00 - grid.min_y) / grid.res)

print(f"2.75, 0.00 -> gx_2d={gx_2d}, gy_2d={gy_2d}")
print(f"Is obs: {grid.obs_map[gx_2d][gy_2d]}")

# Find which obstacle covers it
import math
inf_cells = int(math.ceil(0.30 / grid.res))

for i, obs in enumerate(case['obstacles']):
    ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
    min_wx = min(ox, ox + ow)
    max_wx = max(ox, ox + ow)
    min_wy = min(oy, oy + oh)
    max_wy = max(oy, oy + oh)
    gx_min, gy_min = grid._world_to_grid(min_wx, min_wy)
    gx_max, gy_max = grid._world_to_grid(max_wx, max_wy)
    hx_min = max(0, gx_min - inf_cells)
    hx_max = min(grid.nx - 1, gx_max + inf_cells)
    hy_min = max(0, gy_min - inf_cells)
    hy_max = min(grid.ny - 1, gy_max + inf_cells)
    
    if hx_min <= gx_2d <= hx_max and hy_min <= gy_2d <= hy_max:
        print(f"Obstacle {i} covers it! {obs}")
