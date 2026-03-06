import json
from planner_obs import _preprocess_obstacles
import math

cx, cy = 3.86, -1.92

def _score_y(ex, ey, eth, gear=None, prev_gear=None, w_x=1.0):
    y_min, y_max = -0.8, -0.61
    if ey > y_max: y_over = (ey - y_max) * 10.0
    elif ey < y_min: y_over = (y_min - ey) * 10.0
    else: y_over = 0.0
    y_raw = 0.0
    if not (y_min <= ey <= y_max):
        y_raw = min(abs(ey - y_max), abs(ey - y_min)) * 2.0
    y_center_pen = abs(ey) * 3.0
    x_pen = max(0.0, 2.0 - ex) * w_x * 5.0
    if abs(eth) > 0.4 and (abs(eth) < 1.57 or gear == 'F'):
        x_pen += max(0.0, 3.2 - ex) * w_x * 3.0
    ref_x = max(3.2, cx + 0.5)
    x_over = max(0.0, ex - ref_x) * 2.0
    align_weight = 10.0 if (y_min <= ey <= y_max) else 0.5
    th_pen = max(0.0, abs(eth) - 0.5) * align_weight
    if gear == 'R':
        if ey - y_min < 0.3 and eth < 0: th_pen += 20.0 * abs(eth)
        if y_max - ey < 0.3 and eth > 0: th_pen += 20.0 * abs(eth)
        if y_min + 0.1 <= ey <= y_max - 0.1: th_pen += 15.0 * abs(eth)
    gear_pen = 0.0 if prev_gear is None or gear == prev_gear else 1.0
    
    print(f"y_over={y_over:.2f}, y_raw={y_raw:.2f}, y_center_pen={y_center_pen:.2f}, x_pen={x_pen:.2f}, x_over={x_over:.2f}, th_pen={th_pen:.2f}, gear_pen={gear_pen:.2f}")
    return y_over + y_raw + y_center_pen + x_pen + x_over + th_pen + gear_pen

print("Score for 5.0, -1.92, 0.0 (R):")
print(_score_y(5.0, -1.92, 0.0, 'R', 'F'))
