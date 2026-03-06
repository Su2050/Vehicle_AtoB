import rs
import math

cx, cy, cth = 2.05, -0.50, -1.32
RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
MIN_TURN_RADIUS = 1.97

rs_dist = rs.rs_distance_pose(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
h_rs = rs_dist / 0.25

dx_to_goal = abs(cx - RS_GOAL_X)
dy_to_goal = abs(cy - RS_GOAL_Y)

th_err = abs(cth - RS_GOAL_TH)
while th_err > math.pi: th_err -= 2 * math.pi
th_err = abs(th_err)

h = h_rs
if dx_to_goal < 2.0 and dy_to_goal < 1.0:
    if th_err > 0.2:
        penalty = th_err * (2.0 - dx_to_goal) * (1.0 - dy_to_goal) * 10.0
        h += penalty
        print(f"Heading penalty = {penalty}")

print(f"h_rs = {h_rs}, h = {h}")
