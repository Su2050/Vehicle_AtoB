from test_fast_obs import grid
import math
import primitives

wx, wy, wth = 1.75, 0.0, -1.57
cos_t = math.cos(wth)
sin_t = math.sin(wth)

is_obs = False
if wx <= 2.05:
    front_offset = min(primitives.VEHICLE_CHECK_OFFSETS)
    tip_lat = wy - front_offset * sin_t
    sc = (0.15 + (wx - 1.87) * 0.8) if wx > 1.87 else 0.15
    if tip_lat > sc or tip_lat < -sc:
        is_obs = True
        print(f"Corridor constraint hit: tip_lat={tip_lat}, sc={sc}")

print(f"is_obs: {is_obs}")
