from collision import check_collision
from planner_obs import _preprocess_obstacles
import json
import math
import primitives

obs = [{'x': 3.85, 'y': -1.49, 'w': 0.51, 'h': 0.63}, {'x': 3.1, 'y': 0.22, 'w': 1.6, 'h': 0.61}, {'x': 3.58, 'y': -0.09, 'w': 1.53, 'h': 1.96}, {'x': 5.12, 'y': -1.3, 'w': 0.47, 'h': 0.63}]
fast_obs = _preprocess_obstacles(obs)

nx, ny, nth = 2.25, 0.0, 0.0
sin_nth, cos_nth = math.sin(nth), math.cos(nth)

half_w_sq = primitives.VEHICLE_HALF_WIDTH * primitives.VEHICLE_HALF_WIDTH
offsets = primitives.VEHICLE_CHECK_OFFSETS

for obs in fast_obs:
    min_x, max_x, min_y, max_y = obs
    for offset in offsets:
        px = nx - offset * cos_nth
        py = ny - offset * sin_nth
        
        if px > max_x: dx = px - max_x
        elif px < min_x: dx = min_x - px
        else: dx = 0.0
        
        if py > max_y: dy = py - max_y
        elif py < min_y: dy = min_y - py
        else: dy = 0.0
        
        dist_sq = dx * dx + dy * dy
        if dist_sq < half_w_sq:
            print(f"Collision with obstacle {obs} at offset {offset}: px={px:.2f}, py={py:.2f}, dx={dx:.2f}, dy={dy:.2f}, dist_sq={dist_sq:.4f}")

