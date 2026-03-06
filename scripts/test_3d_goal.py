import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y
import math

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

wx, wy, wth = RS_GOAL_X, RS_GOAL_Y, 0.0
print(f"Goal: {wx}, {wy}, {wth}")

cos_t = math.cos(wth)
sin_t = math.sin(wth)
for offset in [0.375, 0.0, -0.375]:
    cx = wx + offset * cos_t
    cy = wy + offset * sin_t
    cgx = int((cx - grid.min_x) / grid.res)
    cgy = int((cy - grid.min_y) / grid.res)
    is_obs = grid.obs_map[cgx][cgy]
    print(f"offset {offset}: cx={cx:.2f}, cy={cy:.2f} -> cgx={cgx}, cgy={cgy} -> is_obs={is_obs}")
