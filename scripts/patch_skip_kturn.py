import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_skip = "    skip_kturn = False"
new_skip = "    skip_kturn = True"

if old_skip in content:
    content = content.replace(old_skip, new_skip)
    print("Patched successfully!")
else:
    print("Could not find old_skip!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
