import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'r') as f:
    content = f.read()

old_call = """        trajs = rs.rs_sample_path_multi(
            cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
            MIN_TURN_RADIUS, step=DT * 0.5, max_paths=max_paths, collision_fn=collision_fn)"""

new_call = """        trajs = rs.rs_sample_path_multi(
            cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
            MIN_TURN_RADIUS, step=0.1, max_paths=max_paths, collision_fn=collision_fn)"""

content = content.replace(old_call, new_call)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'w') as f:
    f.write(content)
