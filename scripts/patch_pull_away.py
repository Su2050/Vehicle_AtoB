import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_pull = """    if cx < X_FLOOR_SAFE:
        needed_x_init = X_FLOOR_SAFE
        pull_away_gear = 'F' if abs(cth) < M_PI/2 else 'R'"""

new_pull = """    if cx < X_FLOOR_SAFE and x_floor is None:
        needed_x_init = X_FLOOR_SAFE
        pull_away_gear = 'F' if abs(cth) < M_PI/2 else 'R'"""

if old_pull in content:
    content = content.replace(old_pull, new_pull)
    print("Patched successfully!")
else:
    print("Could not find old_pull!")

with open('planner_obs.py', 'w') as f:
    f.write(content)
