import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_penalty = """        # Heading penalty: if close to goal in X, MUST be aligned!
        # Because there is no space to turn near the goal.
        dx_to_goal = abs(nx - RS_GOAL_X)
        if dx_to_goal < 2.0:
            th_err = abs(nth - RS_GOAL_TH)
            while th_err > math.pi: th_err -= 2 * math.pi
            th_err = abs(th_err)
            if th_err > 0.2:
                # Penalize unaligned poses near the goal
                # The closer to the goal, the higher the penalty
                h += th_err * (2.0 - dx_to_goal) * 20.0"""

new_penalty = """        # Heading penalty: if close to goal in X, MUST be aligned!
        # Because there is no space to turn near the goal.
        # REDUCED PENALTY: Only apply if y is also close to goal, meaning we are actually in the narrow spot
        dx_to_goal = abs(nx - RS_GOAL_X)
        dy_to_goal = abs(ny - RS_GOAL_Y)
        if dx_to_goal < 2.0 and dy_to_goal < 1.0:
            th_err = abs(nth - RS_GOAL_TH)
            while th_err > math.pi: th_err -= 2 * math.pi
            th_err = abs(th_err)
            if th_err > 0.2:
                # Penalize unaligned poses near the goal
                # The closer to the goal, the higher the penalty
                h += th_err * (2.0 - dx_to_goal) * (1.0 - dy_to_goal) * 10.0"""

if old_penalty in content:
    content = content.replace(old_penalty, new_penalty)
    print("Patched successfully!")
else:
    print("Could not find old_penalty!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
