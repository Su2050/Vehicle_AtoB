from test_fast_obs import grid
import math

cx, cy, cth = 6.14, 4.54, -1.55

print(f"Start: {cx:.2f}, {cy:.2f}, {cth:.2f} -> h_3d={grid.get_3d_heuristic(cx, cy, cth):.2f}")
for i in range(10):
    dth = -0.1
    cth += dth
    cx += 0.25 * math.cos(cth)
    cy += 0.25 * math.sin(cth)
    h = grid.get_3d_heuristic(cx, cy, cth)
    print(f"Step {i+1}: {cx:.2f}, {cy:.2f}, {cth:.2f} -> h_3d={h:.2f}")

print("\nChecking obs_map for Obstacle 1:")
for x in range(27, 30):
    for y in range(30, 33):
        wx = grid.min_x + x * grid.res_3d
        wy = grid.min_y + y * grid.res_3d
        gx_2d = int((wx - grid.min_x) / grid.res)
        gy_2d = int((wy - grid.min_y) / grid.res)
        print(f"gx_3d={x} (wx={wx:.2f}), gy_3d={y} (wy={wy:.2f}) -> obs_map={grid.obs_map[gx_2d][gy_2d]}")

print("\nChecking obs_map for Step 3:")
wx, wy = 6.125, 3.875
gx_2d = int((wx - grid.min_x) / grid.res)
gy_2d = int((wy - grid.min_y) / grid.res)
print(f"wx={wx:.3f}, wy={wy:.3f} -> obs_map={grid.obs_map[gx_2d][gy_2d]}")

wth = -1.85
cos_t = math.cos(wth)
sin_t = math.sin(wth)
import primitives
for offset in primitives.VEHICLE_CHECK_OFFSETS:
    cx = wx - offset * cos_t
    cy = wy - offset * sin_t
    cgx = int((cx - grid.min_x) / grid.res)
    cgy = int((cy - grid.min_y) / grid.res)
    is_obs = grid.obs_map[cgx][cgy]
    print(f"offset={offset}, cx={cx:.3f}, cy={cy:.3f} -> obs_map={is_obs}")
