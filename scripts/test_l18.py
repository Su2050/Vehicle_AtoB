import json
from planner_obs_v2 import plan_path_robust_obs_v2
import primitives

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

precomp_prim = primitives.init_primitives()

# I will modify planner_obs_v2.py to print L1.8 result
