import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

print("Real obstacles in map:")
for x in range(grid.nx):
    for y in range(grid.ny):
        if grid.real_obs_map[x][y]:
            wx = grid.min_x + x * grid.res
            wy = grid.min_y + y * grid.res
            if 1.5 < wx < 2.5 and -2.0 < wy < -1.0:
                print(f"Real obs at grid ({x}, {y}), world ({wx:.2f}, {wy:.2f})")

