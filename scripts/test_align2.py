from collision import check_collision
from planner_obs import _preprocess_obstacles
import json
import primitives
import math

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

fast_obs = _preprocess_obstacles(cases[0]['obstacles'])
nx, ny, nth = 2.5, 0.0, 0.0
sin_nth = 0.0
cos_nth = 1.0

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
        
        if dx * dx + dy * dy < half_w_sq:
            print(f"Collision with obs {obs} at offset {offset}: px={px}, py={py}")

