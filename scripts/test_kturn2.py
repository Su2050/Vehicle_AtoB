import json
from planner_obs import _preprocess_obstacles
import math

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

fast_obs = _preprocess_obstacles(case['obstacles'])

cx, cy = 3.86, -1.92
_eff_y_max = 0.2

y_min, y_max = -_eff_y_max, _eff_y_max
print(f"Initial: {y_min}, {y_max}")

for obs in fast_obs:
    min_x, max_x, min_y, max_y = obs
    if max_x >= 2.1 and min_x <= max(cx, 4.0):
        min_y_obs = min_y - 0.52
        max_y_obs = max_y + 0.52
        print(f"Obs {obs} blocks {min_y_obs} to {max_y_obs}")
        if min_y_obs < y_max and max_y_obs > y_min:
            r1_ok = y_min < min_y_obs
            r2_ok = max_y_obs < y_max
            if r1_ok and not r2_ok: y_max = min_y_obs
            elif r2_ok and not r1_ok: y_min = max_y_obs
            elif r1_ok and r2_ok:
                if abs(cy - min_y_obs) < abs(cy - max_y_obs): y_max = min_y_obs
                else: y_min = max_y_obs
            else:
                if abs(cy - min_y_obs) < abs(cy - max_y_obs):
                    y_max = min_y_obs; y_min = min_y_obs - 0.5
                else:
                    y_min = max_y_obs; y_max = max_y_obs + 0.5
            print(f"  -> Updated to {y_min}, {y_max}")
