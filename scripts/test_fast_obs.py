from heuristic import DijkstraGrid
import math

obs = [
    (6.03, 6.66, 1.65, 2.06),
    (3.28, 4.21, -1.4, 0.43),
    (6.79, 7.15, 2.57, 4.25),
    (3.87, 4.39, -2.43, -1.61)
]

grid = DijkstraGrid(2.10, 0.0, grid_res=0.10, inflate_radius=0.20)
grid.build_map(obs, start_x=5.05, start_y=2.67)
grid.build_3d_map()

h = grid.get_3d_heuristic(6.14, 4.54, -1.55)
print(f"h_3d for (6.14, 4.54, -1.55) = {h}")

print("Checking (6.14, 4.54, -1.55)")
for offset in [-0.83, -0.33, 0.17]:
    cx = 6.14 - offset * math.cos(-1.55)
    cy = 4.54 - offset * math.sin(-1.55)
    cgx = int((cx - grid.min_x) / grid.res)
    cgy = int((cy - grid.min_y) / grid.res)
    print(f"offset={offset}, cx={cx:.2f}, cy={cy:.2f}, cgx={cgx}, cgy={cgy}, is_obs={grid.obs_map[cgx][cgy]}")
    
    if grid.obs_map[cgx][cgy]:
        print("  Inside obstacle!")
