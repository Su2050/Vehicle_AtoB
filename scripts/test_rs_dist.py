import rs
import math

cx, cy, cth = 2.06, -3.30, -1.47
RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
MIN_TURN_RADIUS = 1.97

rs_dist = rs.rs_distance_pose(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
print(f"rs_dist = {rs_dist}")
print(f"h_rs = {rs_dist / 0.25}")
