import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_check = """    def _check_goal():
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(cx, cy, cth)
            if h_3d < 15.0:
                return True"""

new_check = """    def _check_goal():
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(cx, cy, cth)
            if h_3d < 5.0:
                return True"""

content = content.replace(old_check, new_check)

with open('planner_obs.py', 'w') as f:
    f.write(content)
