import math
import time
import sys
import threading
import traceback
from main import plan_path_robust, init_primitives

def dump_trace():
    while True:
        time.sleep(5)
        print("--- traceback ---")
        for thread_id, frame in sys._current_frames().items():
            traceback.print_stack(frame)
        print("-----------------")

threading.Thread(target=dump_trace, daemon=True).start()

x, y, th = 5.05, -1.25, math.radians(-14.6)
prims = init_primitives()

obstacles = [
    {'x': 3.0, 'y': -2.0, 'w': 0.8, 'h': 1.8}
]

print("Starting robust planning...")
st2 = {}
t0 = time.time()
ok2, acts2, rs_traj2 = plan_path_robust(x, y, th, prims, use_rs=True, no_corridor=False, stats=st2, obstacles=obstacles)
print(f"ok={ok2}, stats={st2}, time={time.time()-t0:.3f}s")
