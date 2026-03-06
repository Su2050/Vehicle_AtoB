import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_penalty = """        if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
            needed_x = dy_err * 2.0 + th_err * 1.5
            if dx_err < needed_x:
                h += min(50.0, (needed_x - dx_err) * 10.0)"""

new_penalty = """        if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
            needed_x = dy_err * 2.0 + th_err * 1.5
            if dx_err < needed_x:
                h += min(15.0, (needed_x - dx_err) * 3.0)"""

content = content.replace(old_penalty, new_penalty)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
