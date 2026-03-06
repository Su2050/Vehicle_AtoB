import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_skip = "skip_kturn = fast_obstacles is not None and len(fast_obstacles) <= 2 and abs(my) < 2.0"
new_skip = "skip_kturn = True  # Disabled K-turn for obstacle cases"

content = content.replace(old_skip, new_skip)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
