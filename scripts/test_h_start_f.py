import rs
import math
from test_fast_obs import grid

cx, cy, cth = 2.05, -3.13, -1.61
RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
MIN_TURN_RADIUS = 1.97

rs_dist = rs.rs_distance_pose(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
h_rs = rs_dist / 0.25

h_3d = grid.get_3d_heuristic(cx, cy, cth)
h = max(h_3d * 1.5, h_rs)

print(f"h_rs = {h_rs}, h_3d_scaled = {h_3d * 1.5}, h = {h}")
