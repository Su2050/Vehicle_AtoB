import json
from planner_obs_v2 import plan_path_robust_obs_v2
from primitives import init_primitives

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

case = cases[0]
obs = case['obstacles']
start = case['start']

precomp_prim = init_primitives()

# We need to hack _make_heuristic_fn_v2 to print
import planner_obs_v2
old_make = planner_obs_v2._make_heuristic_fn_v2

def new_make(*args, **kwargs):
    h_fn = old_make(*args, **kwargs)
    def wrapped_h_fn(nx, ny, nth):
        h, w = h_fn(nx, ny, nth)
        if abs(nx - 2.05) < 0.05 and abs(ny - -3.30) < 0.05:
            print(f"H_FN: nx={nx:.2f}, ny={ny:.2f}, nth={nth:.2f}, h={h:.2f}, w={w}")
        return h, w
    return wrapped_h_fn

planner_obs_v2._make_heuristic_fn_v2 = new_make

success, traj, info = plan_path_robust_obs_v2(
    start['x'], start['y'], start['th'],
    precomp_prim,
    obstacles=obs,
    _time_budget=10.0
)
