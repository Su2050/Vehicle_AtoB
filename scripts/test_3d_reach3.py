import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.25)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])
grid.build_3d_map()

gx = int((4.0 - grid.min_x) / grid.res_3d)
gy = int((-0.47 - grid.min_y) / grid.res_3d)

reachable = False
for th in range(grid.nth):
    if grid.dist_3d[gx][gy][th] != float('inf'):
        print(f"Reachable at x=4.0, y=-0.47 with th={th * grid.res_th:.2f}")
        reachable = True

if not reachable:
    print("NOT reachable at x=4.0, y=-0.47 with ANY heading!")
