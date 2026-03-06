import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_pull = """    if cx < needed_x_init and abs(cy) > 1.0:
        pull_away_gear = 'R' if abs(cth) > 1.57 else 'F'
        for _ in range(40):
            if cx >= needed_x_init: break
            best_p0 = None; best_s0 = float('inf')
            for act0, _n0, traj0 in precomp_prim:
                if act0[0] != pull_away_gear: continue
                ok0, ex0, ey0, eth0 = _apply(cx, cy, cth, traj0)"""

new_pull = """    if cx < needed_x_init and abs(cy) > 1.0:
        pull_away_gear = 'R' if abs(cth) > 1.57 else 'F'
        for _ in range(40):
            if cx >= needed_x_init: break
            best_p0 = None; best_s0 = float('inf')
            for act0, _n0, traj0 in precomp_prim:
                if act0[0] != pull_away_gear: continue
                ok0, ex0, ey0, eth0 = _apply(cx, cy, cth, traj0, x_floor)"""

if old_pull in content:
    content = content.replace(old_pull, new_pull)
    print("Patched successfully!")
else:
    print("Could not find old_pull!")

with open('planner_obs.py', 'w') as f:
    f.write(content)
