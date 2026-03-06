import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_skip = "skip_kturn = True  # Disabled K-turn for obstacle cases"
new_skip = "skip_kturn = False"

content = content.replace(old_skip, new_skip)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
