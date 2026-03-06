import json
from planner_obs_v2 import plan_path_robust_obs_v2
import primitives
import time

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 2670)

precomp_prim = primitives.init_primitives()
stats = {}
t0 = time.perf_counter()
success, acts, traj = plan_path_robust_obs_v2(case['start']['x'], case['start']['y'], case['start']['th'], precomp_prim, obstacles=case['obstacles'], stats=stats)
t1 = time.perf_counter()

print(f"Success: {success}, Time: {(t1-t0)*1000:.1f}ms")
print(f"Stats: {stats}")
