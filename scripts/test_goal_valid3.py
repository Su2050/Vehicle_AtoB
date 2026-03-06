import json
from planner_obs_v2 import plan_path_robust_obs_v2
from planner_obs import _preprocess_obstacles, _make_collision_fn
import primitives

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

case = cases[0]
fast_obstacles = _preprocess_obstacles(case['obstacles'])
coll_fn = _make_collision_fn(False, fast_obstacles)
print("Before init:", coll_fn(2.25, 0.0, 0.0))

primitives.init_primitives()
print("After init:", coll_fn(2.25, 0.0, 0.0))

print("Offsets:", primitives.VEHICLE_CHECK_OFFSETS)
stats = {}
res = plan_path_robust_obs_v2(case['start']['x'], case['start']['y'], case['start']['th'], primitives.init_primitives(), obstacles=case['obstacles'], stats=stats)
print("Planner result:", res[0], "stats:", stats)
