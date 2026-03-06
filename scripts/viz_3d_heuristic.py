import json
import matplotlib.pyplot as plt
import numpy as np
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])
grid.build_3d_map()

# Visualize h_3d for th=0.0
h_map = np.zeros((grid.ny_3d, grid.nx_3d))
for x in range(grid.nx_3d):
    for y in range(grid.ny_3d):
        h_map[y, x] = grid.dist_3d[x][y][0] * 1.5

plt.figure(figsize=(10, 8))
plt.imshow(h_map, origin='lower', extent=[grid.min_x, grid.max_x, grid.min_y, grid.max_y])
plt.colorbar(label='h_3d (th=0.0)')

# Plot obstacles
for obs in case['obstacles']:
    rect = plt.Rectangle((obs['x'], obs['y']), obs['w'], obs['h'], color='red', alpha=0.5)
    plt.gca().add_patch(rect)

plt.plot(case['start']['x'], case['start']['y'], 'go', label='Start')
plt.plot(RS_GOAL_X, RS_GOAL_Y, 'r*', markersize=15, label='Goal')
plt.plot(6.87, -0.20, 'bo', label='K-turn end')

plt.title('3D Heuristic (th=0.0)')
plt.legend()
plt.savefig('h3d_th0.png')

# Visualize h_3d for th=3.14
h_map = np.zeros((grid.ny_3d, grid.nx_3d))
for x in range(grid.nx_3d):
    for y in range(grid.ny_3d):
        h_map[y, x] = grid.dist_3d[x][y][16] * 1.5 # 16 is th=pi

plt.figure(figsize=(10, 8))
plt.imshow(h_map, origin='lower', extent=[grid.min_x, grid.max_x, grid.min_y, grid.max_y])
plt.colorbar(label='h_3d (th=3.14)')

# Plot obstacles
for obs in case['obstacles']:
    rect = plt.Rectangle((obs['x'], obs['y']), obs['w'], obs['h'], color='red', alpha=0.5)
    plt.gca().add_patch(rect)

plt.plot(case['start']['x'], case['start']['y'], 'go', label='Start')
plt.plot(RS_GOAL_X, RS_GOAL_Y, 'r*', markersize=15, label='Goal')
plt.plot(6.87, -0.20, 'bo', label='K-turn end')

plt.title('3D Heuristic (th=3.14)')
plt.legend()
plt.savefig('h3d_thpi.png')
