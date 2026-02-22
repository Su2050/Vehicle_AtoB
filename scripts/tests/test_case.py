import math
from main import plan_path_robust, plan_path_pure_rs, init_primitives
import time

x, y, th = 4.23, -1.55, 0.0
prims = init_primitives()

print("--- Pure RS ---")
st1 = {}
t0 = time.time()
ok1, acts1, rs_traj1 = plan_path_pure_rs(x, y, th, no_corridor=False, stats=st1)
print(f"ok={ok1}, stats={st1}, time={time.time()-t0:.3f}s")

print("\n--- RS Heuristic ---")
st2 = {}
t0 = time.time()
ok2, acts2, rs_traj2 = plan_path_robust(x, y, th, prims, use_rs=True, no_corridor=False, stats=st2)
print(f"ok={ok2}, stats={st2}, time={time.time()-t0:.3f}s")
if acts2:
    print(f"acts len: {len(acts2)}")
