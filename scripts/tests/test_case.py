import math, sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from planner import plan_path_robust
from rs_utils import plan_path_pure_rs
from primitives import init_primitives

x, y, th = 4.23, -1.55, 0.0
prims = init_primitives()

print("--- Pure RS ---")
st1 = {}
t0 = time.time()
ok1, acts1, rs_traj1 = plan_path_pure_rs(x, y, th, stats=st1)
print(f"ok={ok1}, stats={st1}, time={time.time()-t0:.3f}s")

print("\n--- RS Heuristic ---")
st2 = {}
t0 = time.time()
ok2, acts2, rs_traj2 = plan_path_robust(x, y, th, prims, use_rs=True, no_corridor=False, stats=st2)
print(f"ok={ok2}, stats={st2}, time={time.time()-t0:.3f}s")
if acts2:
    print(f"acts len: {len(acts2)}")
