import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'r') as f:
    content = f.read()

old_rs_skip = """        # If h_3d is significantly larger than euclidean distance, it means there are obstacles
        # in the way or a complex maneuver is required. Skip RS expansion to save time.
        if dijkstra_grid is not None:
            h_3d = dijkstra_grid.get_3d_heuristic(cx, cy, cth)
            # Relaxed threshold to allow RS expansion when heading is wrong (which causes h_3d to be ~15-20)
            if h_3d > euclidean_goal * 1.5 + 15.0:
                return None"""

new_rs_skip = """        # If grid heuristic is significantly larger than euclidean distance, it means there are obstacles
        # in the way. Skip RS expansion to save time.
        if dijkstra_grid is not None:
            h_grid_dist, _ = dijkstra_grid.get_heuristic(cx, cy, cth)
            if h_grid_dist > euclidean_goal + 2.0:
                return None"""

content = content.replace(old_rs_skip, new_rs_skip)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'w') as f:
    f.write(content)
