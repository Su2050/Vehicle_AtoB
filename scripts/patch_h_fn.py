import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'r') as f:
    content = f.read()

# Let's fix the heading penalty.
# The original heading penalty in planner_obs_v2.py:
# dy_err = abs(ny - RS_GOAL_Y)
# dx_err = nx - max(RS_GOAL_X, 2.0)
# th_err = abs(nth - RS_GOAL_TH)
# if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
#     needed_x = dy_err * 2.0 + th_err * 1.5
#     if dx_err < needed_x:
#         h += min(50.0, (needed_x - dx_err) * 10.0)

# This penalty is what causes the local minimum!
# If the vehicle is close to the goal in X (dx_err is small), but has a large heading error (th_err > 0.2),
# it adds a huge penalty.
# But sometimes the vehicle NEEDS to be close to the goal in X to maneuver!
# Let's remove this penalty and rely on the corridor constraint to guide the vehicle.

old_penalty = """        dy_err = abs(ny - RS_GOAL_Y)
        dx_err = nx - max(RS_GOAL_X, 2.0)
        th_err = abs(nth - RS_GOAL_TH)
        if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
            needed_x = dy_err * 2.0 + th_err * 1.5
            if dx_err < needed_x:
                h += min(50.0, (needed_x - dx_err) * 10.0)"""

new_penalty = """        # Removed heading penalty to avoid local minima near the goal
        pass"""

content = content.replace(old_penalty, new_penalty)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs_v2.py', 'w') as f:
    f.write(content)
