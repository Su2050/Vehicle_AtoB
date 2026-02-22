from main import DijkstraGrid, RS_GOAL_X, RS_GOAL_Y

obstacles = [
    {'x': 3.0, 'y': -2.0, 'w': 0.8, 'h': 1.8}
]
dgrid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
dgrid.build_map(obstacles, start_x=5.05, start_y=-1.25)
gx, gy = dgrid._world_to_grid(5.05, -1.25)
print(f"Goal cell: {dgrid._world_to_grid(dgrid.goal_x, dgrid.goal_y)}")
gx_g, gy_g = dgrid._world_to_grid(dgrid.goal_x, dgrid.goal_y)
print(f"obs_map at goal: {dgrid.obs_map[gx_g][gy_g]}")
print(f"Start cell: {gx}, {gy}")
print(f"obs_map: {dgrid.obs_map[gx][gy]}")
print(f"dist: {dgrid.dist_map[gx][gy]}")

import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize, linewidth=200)

arr = np.array(dgrid.dist_map)
print("dist_map:")
print(np.round(arr[25:50, 50:75], 1))
arr2 = np.array(dgrid.cost_map)
print("cost_map:")
print(np.round(arr2[25:50, 50:75], 1))
