import sys

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_code = """        for act, _n, traj in precomp_prim:
            if restrict_F and act[0] == 'F': continue
            if abs(act[1]) < 0.5: continue
            ex, ey, eth = _apply_traj(mx, my, mth, traj)
            if collision_fn(ex, ey, eth): continue
            score = 0.0
            if act[0] == 'R': score -= ex * 1.5
            else: score += 2.0
            score += abs(ey - my) * 2.0"""

new_code = """        for act, _n, traj in precomp_prim:
            if restrict_F and act[0] == 'F': continue
            if abs(act[1]) < 0.5: continue
            ex, ey, eth = _apply_traj(mx, my, mth, traj)
            if collision_fn(ex, ey, eth): continue
            score = 0.0
            if act[0] == 'R': score += ex * 1.5  # Reward negative ex
            else: score += 2.0
            score += abs(ey - my) * 2.0"""

if old_code in content:
    content = content.replace(old_code, new_code)
    with open('planner_obs_v2.py', 'w') as f:
        f.write(content)
    print("Patched phase0 turnaround scoring successfully!")
else:
    print("Could not find the target code block.")
