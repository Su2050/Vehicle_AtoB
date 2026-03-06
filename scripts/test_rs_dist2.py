import rs
from primitives import MIN_TURN_RADIUS, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH

dist = rs.rs_distance_pose(4.85, -1.81, -2.94, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
print(f"rs_dist: {dist}")
print(f"h_rs: {dist / 0.25}")
