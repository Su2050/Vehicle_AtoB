import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_call = """        trajs = rs.rs_sample_path_multi(
            cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
            MIN_TURN_RADIUS, step=DT * 0.5, max_paths=max_paths)"""

new_call = """        trajs = rs.rs_sample_path_multi(
            cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
            MIN_TURN_RADIUS, step=DT * 0.5, max_paths=max_paths, collision_fn=collision_fn)"""

content = content.replace(old_call, new_call)

# Also remove the collision check loop from planner_obs_v2.py
old_check = """            traj_ok = True
            for pt in traj:
                valid, _ = collision_fn(pt[0], pt[1], pt[2])
                if not valid:
                    traj_ok = False
                    break
            if traj_ok:"""

new_check = """            if True:"""

content = content.replace(old_check, new_check)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
