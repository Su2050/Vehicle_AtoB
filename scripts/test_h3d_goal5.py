from test_fast_obs import grid
import math

obs = [{'x': 3.85, 'y': -1.49, 'w': 0.51, 'h': 0.63}, {'x': 3.1, 'y': 0.22, 'w': 1.6, 'h': 0.61}, {'x': 3.58, 'y': -0.09, 'w': 1.53, 'h': 1.96}, {'x': 5.12, 'y': -1.3, 'w': 0.47, 'h': 0.63}]
grid.build_map(obs)
grid.build_3d_map()

cx, cy, cth = 2.10, 0.0, 0.0
h_3d = grid.get_3d_heuristic(cx, cy, cth)
print(f"h_3d for ({cx}, {cy}, {cth}) = {h_3d}")
cx, cy, cth = 2.00, 0.0, 0.0
h_3d = grid.get_3d_heuristic(cx, cy, cth)
print(f"h_3d for ({cx}, {cy}, {cth}) = {h_3d}")

cx, cy, cth = 1.75, 0.0, 0.0
h_3d = grid.get_3d_heuristic(cx, cy, cth)
print(f"h_3d for ({cx}, {cy}, {cth}) = {h_3d}")
