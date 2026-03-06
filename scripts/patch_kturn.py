import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs.py', 'r') as f:
    content = f.read()

old_kturn = """    def _score_y(ex, ey, eth, gear=None, prev_gear=None, w_x=1.0):
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(ex, ey, eth)
            if h_3d != float('inf'):
                gear_pen = 0.0 if prev_gear is None or gear == prev_gear else 1.0
                score = h_3d + gear_pen * 2.0
                
                return score
                
        # Fallback to old scoring
        y_min, y_max = _get_safe_y_range()"""

new_kturn = """    def _score_y(ex, ey, eth, gear=None, prev_gear=None, w_x=1.0):
        # Fallback to old scoring
        y_min, y_max = _get_safe_y_range()"""

content = content.replace(old_kturn, new_kturn)

old_kturn2 = """    def _check_goal():
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(cx, cy, cth)
            if h_3d < 15.0:
                return True
            return False
                
        y_min, y_max = _get_safe_y_range()"""

new_kturn2 = """    def _check_goal():
        y_min, y_max = _get_safe_y_range()"""

content = content.replace(old_kturn2, new_kturn2)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs.py', 'w') as f:
    f.write(content)
