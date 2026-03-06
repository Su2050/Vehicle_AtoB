import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# Let's revert the cost logic to the original one that was working, but keep the step size 0.5.
# The original logic was:
# new_dir = 0 if d_step > 0 else 1
# cost = 0.25 if d_step > 0 else 0.625

old_code = """                    # B is current node, A is neighbor.
                    # A = B - d_step * cos(...) => B = A + d_step * cos(...)
                    # So A -> B is a movement of +d_step.
                    # If d_step > 0, A -> B is FORWARD.
                    # If d_step < 0, A -> B is BACKWARD.
                    new_dir = 0 if d_step > 0 else 1
                    cost = 0.5 if d_step > 0 else 1.25"""

new_code = """                    new_dir = 0 if d_step > 0 else 1
                    cost = 0.5 if d_step > 0 else 1.25"""

content = content.replace(old_code, new_code)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'w') as f:
    f.write(content)
