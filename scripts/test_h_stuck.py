import rs
import math

cx, cy, cth = 2.05, -1.03, -1.32
RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
MIN_TURN_RADIUS = 1.97

rs_dist = rs.rs_distance_pose(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
print(f"rs_dist = {rs_dist}")
print(f"h_rs = {rs_dist / 0.25}")

# Heading penalty
dx_to_goal = abs(cx - RS_GOAL_X)
dy_to_goal = abs(cy - RS_GOAL_Y)
print(f"dx_to_goal={dx_to_goal}, dy_to_goal={dy_to_goal}")

th_err = abs(cth - RS_GOAL_TH)
while th_err > math.pi: th_err -= 2 * math.pi
th_err = abs(th_err)
print(f"th_err={th_err}")

if dx_to_goal < 2.0 and dy_to_goal < 1.0:
    if th_err > 0.2:
        penalty = th_err * (2.0 - dx_to_goal) * (1.0 - dy_to_goal) * 10.0
        print(f"Heading penalty = {penalty}")
else:
    print("No heading penalty")
