import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

print(f"obs_map[20][30] = {grid.obs_map[20][30]}")
print(f"real_obs_map[20][30] = {grid.real_obs_map[20][30]}")

# Find the nearest real obstacle to (20, 30)
min_dist = float('inf')
nearest_real = None
for x in range(grid.nx):
    for y in range(grid.ny):
        if grid.real_obs_map[x][y]:
            dist = (x - 20)**2 + (y - 30)**2
            if dist < min_dist:
                min_dist = dist
                nearest_real = (x, y)

print(f"Nearest real obstacle to (20, 30) is at {nearest_real}, dist^2 = {min_dist}")
print(f"World coords: ({grid.min_x + nearest_real[0]*grid.res}, {grid.min_y + nearest_real[1]*grid.res})")

