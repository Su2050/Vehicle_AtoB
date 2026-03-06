import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs.py', 'r') as f:
    content = f.read()

# Make K-turn check goal correctly.
# If we are close to the goal, we don't need to K-turn.
# The original check goal:
# y_min, y_max = _get_safe_y_range()
# target_y = y_min if abs(cy - y_min) < abs(cy - y_max) else y_max
# required_x_min = max(PREAPPROACH_X_MIN, 2.3 + 2.0 * abs(cy - target_y))
# if cx >= required_x_min and abs(cy) <= _eff_y_max and abs(cth) <= PREAPPROACH_TH_MAX:
#     return True

# But if we are close to the goal, we don't need to do this.
# Let's add a check for the goal.

old_check = """    def _check_goal():
        y_min, y_max = _get_safe_y_range()"""

new_check = """    def _check_goal():
        if cx <= 2.25 and abs(cy) <= 0.2 and abs(cth) <= 0.2:
            return True
        y_min, y_max = _get_safe_y_range()"""

content = content.replace(old_check, new_check)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/planner_obs.py', 'w') as f:
    f.write(content)
