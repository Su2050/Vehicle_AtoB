import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

# Call build_3d_map
old_build = """    dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=primitives.VEHICLE_HALF_WIDTH + 0.05)
    dijkstra_grid.build_map(fast_obstacles, x0, y0)"""

new_build = """    dijkstra_grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=primitives.VEHICLE_HALF_WIDTH + 0.05)
    dijkstra_grid.build_map(fast_obstacles, x0, y0)
    dijkstra_grid.build_3d_map()"""

content = content.replace(old_build, new_build)

# Use get_3d_heuristic
old_heur = """        euclidean = math.hypot(nx - RS_GOAL_X, ny - RS_GOAL_Y)
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
                h += min(15.0, (needed_x - dx_err) * 3.0)"""

new_heur = """        h_3d = dijkstra_grid.get_3d_heuristic(nx, ny, nth)
        if h_3d != float('inf'):
            h = max(h_grid * 1.2, h_3d)
        else:
            h = h_grid * 1.5
            
        euclidean = math.hypot(nx - RS_GOAL_X, ny - RS_GOAL_Y)
        if euclidean < 4.0 and dijkstra_grid.line_of_sight(nx, ny, RS_GOAL_X, RS_GOAL_Y):
            rs_dist = rs.rs_distance_pose(
                nx, ny, nth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
            h_rs = rs_dist / 0.25
            h = max(h, h_rs)"""

content = content.replace(old_heur, new_heur)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
