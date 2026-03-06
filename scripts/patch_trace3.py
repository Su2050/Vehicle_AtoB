import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_return = "    if _check_goal(): return True, acts, cx, cy, cth\n    return False, acts, cx, cy, cth"
new_return = "    print(f'DEBUG: _k_turn_preposition_obs returning {len(acts)} acts, final state: {cx}, {cy}, {cth}')\n    if _check_goal(): return True, acts, cx, cy, cth\n    return False, acts, cx, cy, cth"

if old_return in content:
    content = content.replace(old_return, new_return)
    print("Patched successfully!")
else:
    print("Could not find old_return!")

with open('planner_obs.py', 'w') as f:
    f.write(content)
