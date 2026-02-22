import math
import time
from main import plan_path, init_primitives
from main import DijkstraGrid, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS
import rs

x, y, th = 7.34, 0.33, math.radians(-6.9)
prims = init_primitives()

obstacles = [
    {'x': 3.0, 'y': -2.0, 'w': 0.8, 'h': 1.8}
]

dgrid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
dgrid.build_map(obstacles, start_x=x, start_y=y)

print("Running pure Stage-2 A* from K-turn end state...")
dist_start, pure_start = dgrid.get_heuristic(x, y, th)
print(f"Start dist: {dist_start}, pure: {pure_start}")
st = {}
t0 = time.time()
ok, acts, traj = plan_path(
    x, y, th, prims, use_rs=True, stats=st, obstacles=obstacles, dijkstra_grid=dgrid
)
print(f"ok={ok}, stats={st}, time={time.time()-t0:.3f}s")
