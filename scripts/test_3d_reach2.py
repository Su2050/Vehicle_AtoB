import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.25)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])
grid.build_3d_map()

gx = int((4.5 - grid.min_x) / grid.res_3d)

for y in range(grid.ny_3d):
    for th in range(grid.nth):
        if grid.dist_3d[gx][y][th] != float('inf'):
            wy = grid.min_y + y * grid.res_3d + grid.res_3d/2
            wth = th * grid.res_th
            print(f"Reachable at x=4.5: y={wy:.2f}, th={wth:.2f}")
