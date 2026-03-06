import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y
import math
import primitives

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

new_wx = 2.00
new_wy = -1.50
new_wth_norm = 4.71

cos_t = math.cos(new_wth_norm)
sin_t = math.sin(new_wth_norm)
for offset in primitives.VEHICLE_CHECK_OFFSETS:
    cx = new_wx - offset * cos_t
    cy = new_wy - offset * sin_t
    
    cgx = int((cx - grid.min_x) / grid.res)
    cgy = int((cy - grid.min_y) / grid.res)
    
    is_obs = False
    if 0 <= cgx < grid.nx and 0 <= cgy < grid.ny:
        is_obs = grid.obs_map[cgx][cgy]
    print(f"offset {offset}: cx={cx:.2f}, cy={cy:.2f} -> cgx={cgx}, cgy={cgy} -> is_obs={is_obs}")
