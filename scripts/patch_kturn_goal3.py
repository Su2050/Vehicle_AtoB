import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_check = """    def _check_goal():
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(cx, cy, cth)
            if h_3d < 5.0:
                return True
                
        y_min, y_max = _get_safe_y_range()
        target_y = y_min if abs(cy - y_min) < abs(cy - y_max) else y_max
        required_x_min = max(PREAPPROACH_X_MIN, 2.3 + 2.0 * abs(cy - target_y))
        return (y_min <= cy <= y_max
                and required_x_min <= cx <= PREAPPROACH_X_MAX
                and abs(cth) <= PREAPPROACH_TH_MAX)"""

new_check = """    def _check_goal():
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(cx, cy, cth)
            if h_3d < 15.0:
                return True
            return False
                
        y_min, y_max = _get_safe_y_range()
        target_y = y_min if abs(cy - y_min) < abs(cy - y_max) else y_max
        required_x_min = max(PREAPPROACH_X_MIN, 2.3 + 2.0 * abs(cy - target_y))
        return (y_min <= cy <= y_max
                and required_x_min <= cx <= PREAPPROACH_X_MAX
                and abs(cth) <= PREAPPROACH_TH_MAX)"""

if old_check in content:
    content = content.replace(old_check, new_check)
    print("Patched successfully!")
else:
    print("Could not find old_check!")

with open('planner_obs.py', 'w') as f:
    f.write(content)
