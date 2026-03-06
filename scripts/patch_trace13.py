import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_print = "    print(f\"DEBUG: _k_turn_preposition_obs called with x0={x0}, y0={y0}, theta0={theta0}\")"
new_print = "    print(f\"DEBUG: _k_turn_preposition_obs called with x0={x0}, y0={y0}, theta0={theta0}, x_floor={x_floor}\")"

if old_print in content:
    content = content.replace(old_print, new_print)
    print("Patched successfully!")
else:
    print("Could not find old_print!")

with open('planner_obs.py', 'w') as f:
    f.write(content)
