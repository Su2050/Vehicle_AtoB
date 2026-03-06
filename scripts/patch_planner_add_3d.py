import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_build = "dijkstra_grid.build_map(obstacles, start_x=x0, start_y=y0)"
new_build = "dijkstra_grid.build_map(obstacles, start_x=x0, start_y=y0)\n    dijkstra_grid.build_3d_map()"

content = content.replace(old_build, new_build)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
