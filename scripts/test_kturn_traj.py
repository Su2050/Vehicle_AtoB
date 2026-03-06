import json
import primitives
from planner_obs import _k_turn_preposition_obs
from heuristic import DijkstraGrid

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

grid = DijkstraGrid(2.1, 0.0, grid_res=0.10, inflate_radius=0.20)
grid.build_map(case['obstacles'], case['start']['x'], case['start']['y'])
grid.build_3d_map()

precomp_prim = primitives.init_primitives()

fast_obstacles = []
for obs in case['obstacles']:
    ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
    fast_obstacles.append((min(ox, ox + ow), max(ox, ox + ow),
                 min(oy, oy + oh), max(oy, oy + oh)))

kturn_x_floor = 5.61

ok, acts, cx, cy, cth = _k_turn_preposition_obs(
    case['start']['x'], case['start']['y'], case['start']['th'],
    precomp_prim, False, fast_obstacles,
    dijkstra_grid=grid, target_y_max=0.2, x_floor=kturn_x_floor)

print(f"ok={ok}, final state: {cx}, {cy}, {cth}")
for act in acts:
    print(act)
