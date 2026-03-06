import json
from planner_obs_v2 import plan_path_robust_obs_v2
import primitives

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 70)

precomp_prim = primitives.init_primitives()
stats = {}
success, acts, traj = plan_path_robust_obs_v2(case['start']['x'], case['start']['y'], case['start']['th'], precomp_prim, obstacles=case['obstacles'], stats=stats)
print(f"Success: {success}, Stats: {stats}")
