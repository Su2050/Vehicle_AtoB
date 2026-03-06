import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'r') as f:
    content = f.read()

# Let's completely remove the RS heuristic as well!
# And let's increase the inflate_radius back to 0.50.
# The RS heuristic might be causing the A* to get stuck in local minima because it ignores obstacles.

old_h_fn = """        euclidean = math.hypot(nx - RS_GOAL_X, ny - RS_GOAL_Y)
        if euclidean < 4.0 and dijkstra_grid.line_of_sight(nx, ny, RS_GOAL_X, RS_GOAL_Y):
            rs_dist = rs.rs_distance_pose(
                nx, ny, nth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
            h_rs = rs_dist / 0.25
            h = max(h_rs, h_grid * 1.2)
        else:
            h = h_grid * 1.5"""

new_h_fn = """        h = h_grid * 1.5"""

content = content.replace(old_h_fn, new_h_fn)

old_inflate = """dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.20)"""
new_inflate = """dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.15, inflate_radius=0.30)"""

content = content.replace(old_inflate, new_inflate)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'w') as f:
    f.write(content)
