import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_rs_check = """        # If pure_dist is significantly larger than euclidean distance, it means there are obstacles
        # in the way, so RS path will likely hit them. Skip RS expansion to save time.
        if dijkstra_grid is not None:
            _, pure_dist = dijkstra_grid.get_heuristic(cx, cy, cth)
            if pure_dist > euclidean_goal * 1.5 + 0.5:
                return None"""

new_rs_check = """        # If dist_map is significantly larger than euclidean distance, it means there are obstacles
        # in the way, so RS path will likely hit them. Skip RS expansion to save time.
        if dijkstra_grid is not None:
            gx, gy = dijkstra_grid._world_to_grid(cx, cy)
            if 0 <= gx < dijkstra_grid.nx and 0 <= gy < dijkstra_grid.ny:
                raw_dist = dijkstra_grid.dist_map[gx][gy]
                if raw_dist > euclidean_goal * 1.5 + 0.5:
                    return None"""

content = content.replace(old_rs_check, new_rs_check)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
