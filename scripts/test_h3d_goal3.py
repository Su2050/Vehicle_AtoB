from collision import check_collision
from planner_obs import _preprocess_obstacles
import json
import math

obs = [{'x': 3.85, 'y': -1.49, 'w': 0.51, 'h': 0.63}, {'x': 3.1, 'y': 0.22, 'w': 1.6, 'h': 0.61}, {'x': 3.58, 'y': -0.09, 'w': 1.53, 'h': 1.96}, {'x': 5.12, 'y': -1.3, 'w': 0.47, 'h': 0.63}]
fast_obs = _preprocess_obstacles(obs)

cx, cy, cth = 2.25, 0.0, 0.0
valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=False, obstacles=fast_obs)
print(f"Valid: {valid}, Reason: {reason}")
