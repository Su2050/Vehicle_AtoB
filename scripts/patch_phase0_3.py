import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_if = """    facing_away = False
    if mx < 2.10:
        if abs(mth) > 2.6 or (abs(mth) > 1.57 and mth * my < 0):
            facing_away = True
    else:
        # If mx >= 2.10, pointing right (abs(mth) < pi/2) is facing away from goal
        if abs(mth) < 0.5:
            facing_away = True"""

new_if = """    facing_away = False
    if mx < 2.10:
        if abs(mth) > 2.6 or (abs(mth) > 1.57 and mth * my < 0):
            facing_away = True
    else:
        # If mx >= 2.10, pointing right (abs(mth) < pi/2) is facing away from goal
        if abs(mth) < 0.5:
            facing_away = True
        # Also, if we are pointing left but the gap is behind us, we might need to turn around
        elif abs(mth) > 1.57:
            facing_away = True"""

if old_if in content:
    content = content.replace(old_if, new_if)
    print("Patched successfully!")
else:
    print("Could not find old_if!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
