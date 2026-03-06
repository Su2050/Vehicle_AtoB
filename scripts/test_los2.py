import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

x0, y0 = 2.21, -2.98
x1, y1 = RS_GOAL_X, RS_GOAL_Y

gx0 = int((x0 - grid.min_x) / grid.res)
gy0 = int((y0 - grid.min_y) / grid.res)
gx1 = int((x1 - grid.min_x) / grid.res)
gy1 = int((y1 - grid.min_y) / grid.res)

print(f"gx0={gx0}, gy0={gy0}, gx1={gx1}, gy1={gy1}")

dx = abs(gx1 - gx0)
dy = abs(gy1 - gy0)
x, y = gx0, gy0
n = 1 + dx + dy
x_inc = 1 if gx1 > gx0 else -1
y_inc = 1 if gy1 > gy0 else -1
error = dx - dy
dx *= 2
dy *= 2

for _ in range(n):
    if grid.obs_map[x][y]:
        print(f"Hit obstacle at grid ({x}, {y}), world ({grid.min_x + x*grid.res}, {grid.min_y + y*grid.res})")
        break
    if error > 0:
        x += x_inc
        error -= dy
    else:
        y += y_inc
        error += dx
