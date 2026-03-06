import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_rs_check = """        # If RS distance is much larger than euclidean, it's a complex maneuver that likely hits obstacles
        rs_dist = rs.rs_distance_pose(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
        if rs_dist > euclidean_goal * 2.5 + 2.0:
            return None"""

new_rs_check = """        # If h_grid is significantly larger than euclidean distance, it means there are obstacles
        # in the way, so RS path will likely hit them. Skip RS expansion to save time.
        if dijkstra_grid is not None:
            h_grid, _ = dijkstra_grid.get_heuristic(cx, cy, cth)
            if h_grid > euclidean_goal * 1.3 + 0.5:
                return None"""

content = content.replace(old_rs_check, new_rs_check)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
