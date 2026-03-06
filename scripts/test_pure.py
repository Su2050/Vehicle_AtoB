import json
from heuristic import DijkstraGrid
from primitives import RS_GOAL_X, RS_GOAL_Y

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y, inflate_radius=0.30)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])

gx, gy = grid._world_to_grid(2.5, -2.0)
print(f"Start: {gx}, {gy} (dist={grid.dist_map[gx][gy]})")

curr_x, curr_y = gx, gy
path = []
while grid.dist_map[curr_x][curr_y] > 0:
    best_next = None
    best_dist = grid.dist_map[curr_x][curr_y]
    
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            nx = curr_x + dx
            ny = curr_y + dy
            if 0 <= nx < grid.nx and 0 <= ny < grid.ny:
                if grid.dist_map[nx][ny] < best_dist:
                    best_dist = grid.dist_map[nx][ny]
                    best_next = (nx, ny)
    
    if best_next:
        curr_x, curr_y = best_next
        path.append(best_next)
    else:
        break

for p in path:
    wx = grid.min_x + p[0] * grid.res
    wy = grid.min_y + p[1] * grid.res
    print(f" -> {wx:.2f}, {wy:.2f} (dist={grid.dist_map[p[0]][p[1]]})")
