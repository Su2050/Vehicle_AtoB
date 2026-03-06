import json
from planner_obs import _preprocess_obstacles
import math
import primitives

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

fast_obs = _preprocess_obstacles(case['obstacles'])

x, y, th = 2.99, -0.80, 1.54
cos_t = math.cos(th)
sin_t = math.sin(th)

print("Obstacles:")
for obs in fast_obs:
    print(obs)

print("Circles:")
for offset in primitives.VEHICLE_CHECK_OFFSETS:
    cx = x - offset * cos_t
    cy = y - offset * sin_t
    print(f"cx={cx:.2f}, cy={cy:.2f}")
    for obs in fast_obs:
        min_x, max_x, min_y, max_y = obs
        dx = max(min_x - cx, 0, cx - max_x)
        dy = max(min_y - cy, 0, cy - max_y)
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < 0.25:
            print(f"  Collides with {obs} (dist={dist:.3f})")
