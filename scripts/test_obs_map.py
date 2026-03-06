from test_fast_obs import grid

wx = 5.75
gx_2d = int((wx - grid.min_x) / grid.res)
print(f"wx={wx}, gx_2d={gx_2d}")
for y in range(60, 80):
    wy = grid.min_y + y * grid.res
    print(f"wy={wy:.2f}, gy_2d={y}, obs_map={grid.obs_map[gx_2d][y]}")
    
wx = 6.00
gx_2d = int((wx - grid.min_x) / grid.res)
print(f"wx={wx}, gx_2d={gx_2d}")
for y in range(60, 80):
    wy = grid.min_y + y * grid.res
    print(f"wy={wy:.2f}, gy_2d={y}, obs_map={grid.obs_map[gx_2d][y]}")
