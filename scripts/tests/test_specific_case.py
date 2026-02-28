import time, math, sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from primitives import init_primitives
from planner_obs import plan_path_with_obs

x, y, deg = 7.5, -1.5, -135
th = deg * math.pi / 180.0
obstacles = [{'x': 3.0, 'y': -1.5, 'w': 0.6, 'h': 1.2}]

prims = init_primitives()
st = {}
t0 = time.perf_counter()
ok, acts, rs_traj = plan_path_with_obs(x, y, th, prims, use_rs=True, stats=st, obstacles=obstacles)
t1 = time.perf_counter()

print(f"Success: {ok}")
print(f"Elapsed: {t1 - t0:.2f}s")
for k, v in st.items():
    print(f"  {k}: {v}")
