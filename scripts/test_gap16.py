import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y
import math
import primitives

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, grid_res=0.10, inflate_radius=0.25)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

primitives.init_primitives()

blocked_ths = []
for th_idx in range(32):
    th = th_idx * 2 * math.pi / 32
    is_obs = False
    for offset in primitives.VEHICLE_CHECK_OFFSETS:
        cx = 4.0625 - offset * math.cos(th)
        cy = -0.4375 - offset * math.sin(th)
        cgx = int((cx - grid.min_x) / grid.res)
        cgy = int((cy - grid.min_y) / grid.res)
        if 0 <= cgx < grid.nx and 0 <= cgy < grid.ny:
            if grid.obs_map[cgx][cgy]:
                is_obs = True
                break
        else:
            is_obs = True
            break
    if is_obs:
        blocked_ths.append(th_idx)

print(f"Blocked headings: {blocked_ths}")
print(f"Open headings: {[i for i in range(32) if i not in blocked_ths]}")
