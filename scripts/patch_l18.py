import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_l18 = """        if ok_2d and acts_2d:
            _l18_any_acts = acts_2d
            _l18_any_traj = traj_2d
            _l18_any_len = len(acts_2d)"""

new_l18 = """        if ok_2d and acts_2d:
            print("L1.8 SUCCEEDED!")
            _l18_any_acts = acts_2d
            _l18_any_traj = traj_2d
            _l18_any_len = len(acts_2d)
        else:
            print("L1.8 FAILED!")"""

content = content.replace(old_l18, new_l18)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
