import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_if = """    if facing_away:
        print(f"DEBUG: calling _phase0_turnaround with mx={mx}, my={my}, mth={mth}")
        phase0_acts, mx, my, mth = _phase0_turnaround(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles)
        print(f"DEBUG: _phase0_turnaround returned {len(phase0_acts)} acts, mx={mx}, my={my}, mth={mth}")"""

new_if = """    if False:
        print(f"DEBUG: calling _phase0_turnaround with mx={mx}, my={my}, mth={mth}")
        phase0_acts, mx, my, mth = _phase0_turnaround(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles)
        print(f"DEBUG: _phase0_turnaround returned {len(phase0_acts)} acts, mx={mx}, my={my}, mth={mth}")"""

if old_if in content:
    content = content.replace(old_if, new_if)
    print("Patched successfully!")
else:
    print("Could not find old_if!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
