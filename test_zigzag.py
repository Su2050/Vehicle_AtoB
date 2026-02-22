import math
import time
from main import plan_path_robust, init_primitives
from main import DijkstraGrid, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS
import rs

x, y, th = 5.05, -1.25, math.radians(-14.6)
prims = init_primitives()

obstacles = [
    {'x': 3.0, 'y': -2.0, 'w': 0.8, 'h': 1.8}
]

dgrid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
dgrid.build_map(obstacles, start_x=x, start_y=y)
h_g, _ = dgrid.get_heuristic(x, y)
h_rs = rs.rs_distance_pose(x, y, th, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
print(f"Start pos: {x}, {y}, th={th}")
print(f"h_grid={h_g:.3f}m -> time {h_g/0.25:.3f}s")
print(f"h_rs={h_rs:.3f}m -> time {h_rs/0.25:.3f}s")

print("--- RS Heuristic with Obstacle ---")
st2 = {}
t0 = time.time()
ok2, acts2, rs_traj2 = plan_path_robust(x, y, th, prims, use_rs=True, no_corridor=False, stats=st2, obstacles=obstacles)
print(f"ok={ok2}, stats={st2}, time={time.time()-t0:.3f}s")
if acts2:
    print(f"acts len: {len(acts2)}")
    
    # Check gear changes
    gear_changes = 0
    prev_g = 'N'
    for a in acts2:
        if prev_g != 'N' and a[0] != prev_g:
            gear_changes += 1
        prev_g = a[0]
    print(f"gear changes: {gear_changes}")

# 顺便打一下起点的启发式看看
from main import DijkstraGrid, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS
import rs
dgrid = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
dgrid.build_map(obstacles, start_x=x, start_y=y)
h_g, _ = dgrid.get_heuristic(x, y)
h_rs = rs.rs_distance_pose(x, y, th, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
print(f"Start pos: {x}, {y}, th={th}")
print(f"h_grid={h_g:.3f}m -> time {h_g/0.25:.3f}s")
print(f"h_rs={h_rs:.3f}m -> time {h_rs/0.25:.3f}s")


