import math
from test_fast_obs import grid

cx, cy, cth = 2.15, -1.14, -1.68
RS_GOAL_X, RS_GOAL_Y = 2.25, 0.0

euclidean_goal = math.hypot(cx - RS_GOAL_X, cy - RS_GOAL_Y)
h_3d = grid.get_3d_heuristic(cx, cy, cth)

print(f"h_3d = {h_3d}")
print(f"threshold = {euclidean_goal * 1.5 + 15.0}")
print(f"Skip? {h_3d > euclidean_goal * 1.5 + 15.0}")
