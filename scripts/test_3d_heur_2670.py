import json
from planner_obs_v2 import DijkstraGrid
from planner_obs import _preprocess_obstacles
import matplotlib.pyplot as plt
import numpy as np
import math

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 2670)

dijkstra_grid = DijkstraGrid(2.25, 0.0, grid_res=0.10, inflate_radius=0.20)
dijkstra_grid.build_map(case['obstacles'], start_x=case['start']['x'], start_y=case['start']['y'])
dijkstra_grid.build_3d_map()

print("Start heuristic:", dijkstra_grid.get_3d_heuristic(case['start']['x'], case['start']['y'], case['start']['th']))

grid = np.zeros((dijkstra_grid.ny, dijkstra_grid.nx))
for y in range(dijkstra_grid.ny):
    for x in range(dijkstra_grid.nx):
        grid[y, x] = dijkstra_grid.get_3d_heuristic(dijkstra_grid.min_x + x * 0.1, dijkstra_grid.min_y + y * 0.1, 1.5)

plt.imshow(grid, origin='lower', extent=[dijkstra_grid.min_x, dijkstra_grid.max_x, dijkstra_grid.min_y, dijkstra_grid.max_y], vmax=50)
plt.colorbar()
plt.plot(case['start']['x'], case['start']['y'], 'go')
plt.plot(2.25, 0.0, 'bo')
plt.savefig('h3d_2670.png')
print("Saved to h3d_2670.png")
