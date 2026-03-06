import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_phase0 = """        if best_act is None:
            break"""
new_phase0 = """        if best_act is None:
            print(f"DEBUG: _phase0_turnaround broke because best_act is None! valid_count={sum(1 for act, _n, traj in precomp_prim if not (restrict_F and act[0] == 'F') and abs(act[1]) >= 0.5)}")
            break"""

if old_phase0 in content:
    content = content.replace(old_phase0, new_phase0)
    print("Patched successfully!")
else:
    print("Could not find old_phase0!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
