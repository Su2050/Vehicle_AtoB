from test_fast_obs import grid
import math

cx, cy, cth = 2.25, -0.47, -1.50
h_3d = grid.get_3d_heuristic(cx, cy, cth)
print(f"h_3d = {h_3d}")
