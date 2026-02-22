import math
import sys
from main import _k_turn_preposition, init_primitives, PREAPPROACH_Y_MAX, PREAPPROACH_X_MIN

x, y, th = 5.05, -1.25, math.radians(-14.6)
prims = init_primitives()
obstacles = [
    {'x': 3.0, 'y': -2.0, 'w': 0.8, 'h': 1.8}
]

from main import DijkstraGrid, RS_GOAL_X, RS_GOAL_Y

dgrid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
dgrid.build_map(obstacles)

print("Running greedy K-turn...")
ok1, acts1_greedy, mx2, my2, mth2 = _k_turn_preposition(x, y, th, prims, no_corridor=False, obstacles=obstacles, dijkstra_grid=dgrid)

print(f"ok1={ok1}, length={len(acts1_greedy)}")
print(f"End state: x={mx2:.2f}, y={my2:.2f}, th={math.degrees(mth2):.1f}°")
if acts1_greedy:
    print(f"Acts: {acts1_greedy}")

