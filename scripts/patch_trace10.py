import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_phase0 = "    phase0_acts = []"
new_phase0 = "    phase0_acts = []\n    print(f\"DEBUG: _phase0_turnaround called with mx={mx}, my={my}, mth={mth}\")"

if old_phase0 in content:
    content = content.replace(old_phase0, new_phase0)
    print("Patched successfully!")
else:
    print("Could not find old_phase0!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
