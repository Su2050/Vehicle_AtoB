import time
from heuristic import DijkstraGrid
import math

obs = [
    (6.03-0.315, 6.03+0.315, 1.65-0.205, 1.65+0.205),
    (3.28-0.465, 3.28+0.465, -1.4-0.915, -1.4+0.915),
    (6.79-0.18, 6.79+0.18, 2.57-0.84, 2.57+0.84),
    (3.87-0.26, 3.87+0.26, -2.43-0.41, -2.43+0.41)
]
grid = DijkstraGrid(2.10, 0.0, grid_res=0.10, inflate_radius=0.20)
grid.build_map(obs, start_x=5.05, start_y=2.67)

t0 = time.perf_counter()
grid.build_3d_map()
t1 = time.perf_counter()

print(f"build_3d_map took {t1-t0:.2f}s")
