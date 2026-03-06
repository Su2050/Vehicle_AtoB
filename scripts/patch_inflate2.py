import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_grid = "dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=primitives.VEHICLE_HALF_WIDTH)"
new_grid = "dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.20)"

if old_grid in content:
    content = content.replace(old_grid, new_grid)
    print("Patched successfully!")
else:
    print("Could not find old_grid!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
