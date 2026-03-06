import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_score = """    def _score_y(ex, ey, eth, gear=None, prev_gear=None, w_x=1.0):
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(ex, ey, eth)
            if h_3d != float('inf'):
                gear_pen = 0.0 if prev_gear is None or gear == prev_gear else 1.0
                return h_3d + gear_pen * 2.0"""

new_score = """    def _score_y(ex, ey, eth, gear=None, prev_gear=None, w_x=1.0):
        if dijkstra_grid is not None and hasattr(dijkstra_grid, 'get_3d_heuristic'):
            h_3d = dijkstra_grid.get_3d_heuristic(ex, ey, eth)
            if h_3d != float('inf'):
                gear_pen = 0.0 if prev_gear is None or gear == prev_gear else 1.0
                score = h_3d + gear_pen * 2.0
                print(f"DEBUG: score={score} (h_3d={h_3d}) at x={ex}, y={ey}, th={eth}")
                return score"""

if old_score in content:
    content = content.replace(old_score, new_score)
    print("Patched successfully!")
else:
    print("Could not find old_score!")

with open('planner_obs.py', 'w') as f:
    f.write(content)
