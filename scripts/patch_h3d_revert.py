import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'r') as f:
    content = f.read()

# Let's revert the 3D heuristic completely from planner_obs_v2.py.
# It seems the 2D heuristic + RS heuristic + heading penalty was actually working better!
# The only issue was the inflate_radius.
# In the original planner_obs_v2.py, inflate_radius was 0.30.
# Let's check what it is now.

old_h_fn = """    def h_fn(nx, ny, nth):
        h_grid_dist, pure_dist = dijkstra_grid.get_heuristic(nx, ny, nth)

        if (pure_dist != float('inf') and start_d_goal != float('inf')
                and allow_uphill < 50.0):
            if pure_dist > start_d_goal + allow_uphill:
                return 1e6, 1.0

        h_grid = h_grid_dist / 0.25

        euclidean = math.hypot(nx - RS_GOAL_X, ny - RS_GOAL_Y)
        if euclidean < 4.0 and dijkstra_grid.line_of_sight(nx, ny, RS_GOAL_X, RS_GOAL_Y):
            rs_dist = rs.rs_distance_pose(
                nx, ny, nth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
            h_rs = rs_dist / 0.25
            h = max(h_rs, h_grid * 1.2)
        else:
            h = h_grid * 1.5

        # Use 3D heuristic if available and valid
        h_3d = dijkstra_grid.get_3d_heuristic(nx, ny, nth)
        if h_3d != float('inf'):
            h = max(h, h_3d * 1.5)

        dy_err = abs(ny - RS_GOAL_Y)
        dx_err = nx - max(RS_GOAL_X, 2.0)
        th_err = abs(nth - RS_GOAL_TH)
        if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
            needed_x = dy_err * 2.0 + th_err * 1.5
            if dx_err < needed_x:
                h += min(50.0, (needed_x - dx_err) * 10.0)

        return h, h_weight
    return h_fn"""

new_h_fn = """    def h_fn(nx, ny, nth):
        h_grid_dist, pure_dist = dijkstra_grid.get_heuristic(nx, ny, nth)

        if (pure_dist != float('inf') and start_d_goal != float('inf')
                and allow_uphill < 50.0):
            if pure_dist > start_d_goal + allow_uphill:
                return 1e6, 1.0

        h_grid = h_grid_dist / 0.25

        euclidean = math.hypot(nx - RS_GOAL_X, ny - RS_GOAL_Y)
        if euclidean < 4.0 and dijkstra_grid.line_of_sight(nx, ny, RS_GOAL_X, RS_GOAL_Y):
            rs_dist = rs.rs_distance_pose(
                nx, ny, nth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
            h_rs = rs_dist / 0.25
            h = max(h_rs, h_grid * 1.2)
        else:
            h = h_grid * 1.5

        dy_err = abs(ny - RS_GOAL_Y)
        dx_err = nx - max(RS_GOAL_X, 2.0)
        th_err = abs(nth - RS_GOAL_TH)
        if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
            needed_x = dy_err * 2.0 + th_err * 1.5
            if dx_err < needed_x:
                h += min(50.0, (needed_x - dx_err) * 10.0)

        return h, h_weight
    return h_fn"""

content = content.replace(old_h_fn, new_h_fn)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'w') as f:
    f.write(content)
