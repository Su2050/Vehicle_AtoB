import rs
import math
from test_fast_obs import grid

RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
MIN_TURN_RADIUS = 1.97

def get_h(cx, cy, cth):
    rs_dist = rs.rs_distance_pose(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
    h_rs = rs_dist / 0.25
    h_3d = grid.get_3d_heuristic(cx, cy, cth)
    return max(h_3d * 1.5, h_rs)

# Straight (move up)
cx1, cy1, cth1 = 2.20, -0.47, -1.64
h1 = get_h(cx1, cy1, cth1)
print(f"Straight to {cy1}: h = {h1}")

# Turn (move left and up)
cx2, cy2, cth2 = 2.25, -0.47, -1.50
h2 = get_h(cx2, cy2, cth2)
print(f"Turn to {cy2}, {cth2}: h = {h2}")
