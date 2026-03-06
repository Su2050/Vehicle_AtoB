from test_fast_obs import grid
import math
import primitives

wx, wy = 2.25, 0.0
cos_t = 1.0
sin_t = 0.0

is_obs = False
if wx <= 2.05:
    front_offset = min(primitives.VEHICLE_CHECK_OFFSETS)
    tip_lat = wy - front_offset * sin_t
    sc = (0.15 + (wx - 1.87) * 0.8) if wx > 1.87 else 0.15
    if tip_lat > sc or tip_lat < -sc:
        is_obs = True
        print(f"Corridor constraint hit: tip_lat={tip_lat}, sc={sc}")

if not is_obs:
    for offset in primitives.VEHICLE_CHECK_OFFSETS:
        cx = wx - offset * cos_t
        cy = wy - offset * sin_t
        cgx = int((cx - grid.min_x) / grid.res)
        cgy = int((cy - grid.min_y) / grid.res)
        if 0 <= cgx < grid.nx and 0 <= cgy < grid.ny:
            if grid.obs_map[cgx][cgy]:
                is_obs = True
                print(f"Obstacle hit at offset={offset}: cx={cx}, cy={cy}, cgx={cgx}, cgy={cgy}")
                break
        else:
            is_obs = True
            print(f"Out of bounds at offset={offset}: cx={cx}, cy={cy}")
            break

print(f"is_obs: {is_obs}")
