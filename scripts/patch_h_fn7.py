import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'r') as f:
    content = f.read()

# Let's fix the heading penalty in the original planner_obs_v2.py.
# The original heading penalty in planner_obs_v2.py:
# dy_err = abs(ny - RS_GOAL_Y)
# dx_err = nx - max(RS_GOAL_X, 2.0)
# th_err = abs(nth - RS_GOAL_TH)
# if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
#     needed_x = dy_err * 2.0 + th_err * 1.5
#     if dx_err < needed_x:
#         h += min(50.0, (needed_x - dx_err) * 10.0)

# The issue is that the vehicle needs to be able to align itself.
# If it's close to the goal, it must be aligned.
# But if it's not aligned, it shouldn't be penalized so much that it gets stuck.
# Let's just remove it.

old_penalty = """        dy_err = abs(ny - RS_GOAL_Y)
        dx_err = nx - max(RS_GOAL_X, 2.0)
        th_err = abs(nth - RS_GOAL_TH)
        if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
            needed_x = dy_err * 2.0 + th_err * 1.5
            if dx_err < needed_x:
                h += min(50.0, (needed_x - dx_err) * 10.0)"""

new_penalty = """        # Removed heading penalty"""

content = content.replace(old_penalty, new_penalty)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'w') as f:
    f.write(content)
