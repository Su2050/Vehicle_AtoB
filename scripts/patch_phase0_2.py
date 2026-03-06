import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_if = "    if abs(mth) > M_PI / 2 and (fast_obstacles is None or len(fast_obstacles) < 3):"
new_if = "    if abs(mth) > M_PI / 2:"

if old_if in content:
    content = content.replace(old_if, new_if)
    print("Patched successfully!")
else:
    print("Could not find old_if!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
