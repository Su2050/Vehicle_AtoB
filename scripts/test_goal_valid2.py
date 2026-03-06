import json
from collision import check_collision
from planner_obs import _preprocess_obstacles
from primitives import init_primitives
import math

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

case = cases[0]
obs = case['obstacles']
fast_obs = _preprocess_obstacles(obs)
print(f"fast_obs: {fast_obs}")

init_primitives() # to set VEHICLE_CHECK_OFFSETS
import primitives
print(f"VEHICLE_CHECK_OFFSETS: {primitives.VEHICLE_CHECK_OFFSETS}")

valid, _ = check_collision(2.25, 0.0, 0.0, 0.0, 1.0, no_corridor=False, obstacles=fast_obs)
print(f"Goal valid: {valid}")
