import json
from planner_obs_v2 import DijkstraGrid
from planner_obs import _preprocess_obstacles
import matplotlib.pyplot as plt
import numpy as np

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 9192)

dijkstra_grid = DijkstraGrid(2.25, 0.0, grid_res=0.10, inflate_radius=0.20)
dijkstra_grid.build_map(case['obstacles'], start_x=case['start']['x'], start_y=case['start']['y'])

grid = np.zeros((dijkstra_grid.height, dijkstra_grid.width))
for y in range(dijkstra_grid.height):
    for x in range(dijkstra_grid.width):
        grid[y, x] = dijkstra_grid.grid[y][x]

plt.imshow(grid, origin='lower', extent=[dijkstra_grid.min_x, dijkstra_grid.max_x, dijkstra_grid.min_y, dijkstra_grid.max_y])
plt.colorbar()
plt.plot(case['start']['x'], case['start']['y'], 'go')
plt.plot(2.25, 0.0, 'bo')
plt.savefig('obs_map_9192.png')
print("Saved to obs_map_9192.png")
