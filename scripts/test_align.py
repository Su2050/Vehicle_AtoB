from collision import check_collision
from planner_obs import _preprocess_obstacles
import json

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

fast_obs = _preprocess_obstacles(cases[0]['obstacles'])
print(check_collision(2.5, 0.0, 0.0, obstacles=fast_obs))
print(check_collision(3.0, 0.0, 0.0, obstacles=fast_obs))
print(check_collision(3.5, 0.0, 0.0, obstacles=fast_obs))
