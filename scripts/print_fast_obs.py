import json
from planner_obs import _preprocess_obstacles

with open("logs/stress_timeouts_20260302_184337.json", "r") as f:
    data = json.load(f)
case = next(c for c in data if c['case_id'] == 9192)
fast_obs = _preprocess_obstacles(case['obstacles'])
for o in fast_obs:
    print(o)
