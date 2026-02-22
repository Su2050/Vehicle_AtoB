import time
import main
import math

x, y, deg = 7.5, -1.5, -135
th = deg * math.pi / 180.0
obstacles = [{'x': 3.0, 'y': -1.5, 'w': 0.6, 'h': 1.2}]

prims = main.init_primitives()
st = {}
t0 = time.perf_counter()
ok, acts, rs_traj = main.plan_path_robust(x, y, th, prims, use_rs=True, stats=st, obstacles=obstacles)
t1 = time.perf_counter()

print(f"Success: {ok}")
print(f"Elapsed: {t1 - t0:.2f}s")
for k, v in st.items():
    print(f"  {k}: {v}")
