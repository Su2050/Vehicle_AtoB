import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_score = """            if act[0] == 'F': score -= ex * 1.5
            else: score += 2.0"""
new_score = """            if act[0] == 'R': score -= ex * 1.5
            else: score += 2.0"""

if old_score in content:
    content = content.replace(old_score, new_score)
    print("Patched successfully!")
else:
    print("Could not find old_score!")

# Also enable _phase0_turnaround
old_if = """    if False:
        print(f"DEBUG: calling _phase0_turnaround with mx={mx}, my={my}, mth={mth}")"""
new_if = """    if facing_away:
        print(f"DEBUG: calling _phase0_turnaround with mx={mx}, my={my}, mth={mth}")"""

if old_if in content:
    content = content.replace(old_if, new_if)
    print("Patched if successfully!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
