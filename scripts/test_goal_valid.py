from planner_obs_v2 import plan_path_robust_obs_v2
from planner_obs import _preprocess_obstacles, _make_collision_fn
import primitives
import json

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

fast_obstacles = _preprocess_obstacles(cases[0]['obstacles'])
coll_fn = _make_collision_fn(False, fast_obstacles)
print(coll_fn(2.25, 0.0, 0.0))
