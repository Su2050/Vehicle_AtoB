import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.25)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])
grid.build_3d_map()

reachable = 0
for x in range(grid.nx_3d):
    for y in range(grid.ny_3d):
        for th in range(grid.nth):
            if grid.dist_3d[x][y][th] != float('inf'):
                reachable += 1

print(f"Total reachable states: {reachable} out of {grid.nx_3d * grid.ny_3d * grid.nth}")
