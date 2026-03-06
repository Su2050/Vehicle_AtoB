import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_cond = """            if abs(act[1]) < 0.5:
                continue"""
new_cond = """            # Allow straight actions
            # if abs(act[1]) < 0.5:
            #     continue"""

if old_cond in content:
    content = content.replace(old_cond, new_cond)
    print("Patched successfully!")
else:
    print("Could not find old_cond!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
