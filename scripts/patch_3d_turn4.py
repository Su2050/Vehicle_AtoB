import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# The grid resolution is 0.25m.
# The step size is 0.25m.
# If we move diagonally, we might stay in the same cell.
# Let's increase the step size to 0.5m for the heuristic search.
# This will make the search faster and prevent getting stuck in the same cell.

old_code = """            for d_step in [0.25, -0.25]:
                for steer in [0.0, 0.25, -0.25]:
                    # d_step is B -> A. So A -> B is -d_step.
                    # If d_step > 0, A -> B is negative (BACKWARD).
                    # If d_step < 0, A -> B is positive (FORWARD).
                    new_dir = 1 if d_step > 0 else 0
                    cost = 0.625 if d_step > 0 else 0.25"""

new_code = """            for d_step in [0.5, -0.5]:
                for steer in [0.0, 0.25, -0.25]:
                    # d_step is B -> A. So A -> B is -d_step.
                    # If d_step > 0, A -> B is negative (BACKWARD).
                    # If d_step < 0, A -> B is positive (FORWARD).
                    new_dir = 1 if d_step > 0 else 0
                    cost = 1.25 if d_step > 0 else 0.5"""

content = content.replace(old_code, new_code)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'w') as f:
    f.write(content)
