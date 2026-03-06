from test_fast_obs import grid
import math

obs = [{'x': 3.85, 'y': -1.49, 'w': 0.51, 'h': 0.63}, {'x': 3.1, 'y': 0.22, 'w': 1.6, 'h': 0.61}, {'x': 3.58, 'y': -0.09, 'w': 1.53, 'h': 1.96}, {'x': 5.12, 'y': -1.3, 'w': 0.47, 'h': 0.63}]
grid.build_map(obs)
grid.build_3d_map()

cx, cy = 2.25, -0.50
for cth in [0.0, -1.57, 1.57, 3.14]:
    h_3d = grid.get_3d_heuristic(cx, cy, cth)
    print(f"h_3d for ({cx}, {cy}, {cth}) = {h_3d}")
cx, cy = 2.25, 0.0
for cth in [0.0, -1.57, 1.57, 3.14]:
    h_3d = grid.get_3d_heuristic(cx, cy, cth)
    print(f"h_3d for ({cx}, {cy}, {cth}) = {h_3d}")
