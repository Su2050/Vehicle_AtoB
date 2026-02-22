import math
from main import check_collision

cx = 7.65
cy = -0.42
obstacles = [{'x': 3.0, 'y': -2.0, 'w': 0.8, 'h': 1.8}]
PREAPPROACH_Y_MAX = 1.25

y_min = -PREAPPROACH_Y_MAX
y_max = PREAPPROACH_Y_MAX
if obstacles:
    for obs in obstacles:
        ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
        min_x = min(ox, ox + ow)
        max_x = max(ox, ox + ow)
        if max_x >= 2.1 and min_x <= cx:
            min_y = min(oy, oy + oh) - 0.52
            max_y = max(oy, oy + oh) + 0.52
            if min_y < y_max and max_y > y_min:
                r1_ok = y_min < min_y
                r2_ok = max_y < y_max
                if r1_ok and not r2_ok:
                    y_max = min_y
                elif r2_ok and not r1_ok:
                    y_min = max_y
                elif r1_ok and r2_ok:
                    if abs(cy - min_y) < abs(cy - max_y):
                        y_max = min_y
                    else:
                        y_min = max_y
                else:
                    if abs(cy - min_y) < abs(cy - max_y):
                        y_max = min_y
                        y_min = min_y - 0.5
                    else:
                        y_min = max_y
                        y_max = max_y + 0.5
print(f"y_min: {y_min}, y_max: {y_max}")
