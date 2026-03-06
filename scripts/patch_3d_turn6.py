import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# The cost is added to the neighbor's cost to get the new cost.
# new_d = d + cost
# So if we are at B (cost d), and we expand to A (new_d).
# A -> B is the actual movement.
# If d_step > 0, B is in front of A. Wait.
# new_wx = wx - d_step * math.cos(new_wth)
# A = B - d_step * cos(...)
# So B = A + d_step * cos(...)
# If d_step > 0, B is in front of A.
# So A -> B is a FORWARD move!
# Let's re-evaluate:
# If d_step > 0, A -> B is FORWARD.
# So cost should be 0.5.
# If d_step < 0, A -> B is BACKWARD.
# So cost should be 1.25.
# Let's check my previous comment:
# "If d_step > 0, A is in front of B. Vehicle moves from A to B, which is a BACKWARD move."
# Wait, B is the current node (wx, wy). A is the neighbor (new_wx, new_wy).
# new_wx = wx - d_step * math.cos(...)
# A = B - d_step * cos(...)
# B = A + d_step * cos(...)
# So if d_step > 0, B is in the direction of cos(...) from A.
# This means A -> B is a FORWARD move (+d_step).
# Yes! My previous logic was backwards!
# If d_step > 0, A -> B is FORWARD.
# If d_step < 0, A -> B is BACKWARD.

old_code = """                    # d_step is B -> A. So A -> B is -d_step.
                    # If d_step > 0, A -> B is negative (BACKWARD).
                    # If d_step < 0, A -> B is positive (FORWARD).
                    new_dir = 1 if d_step > 0 else 0
                    cost = 1.25 if d_step > 0 else 0.5"""

new_code = """                    # B is current node, A is neighbor.
                    # A = B - d_step * cos(...) => B = A + d_step * cos(...)
                    # So A -> B is a movement of +d_step.
                    # If d_step > 0, A -> B is FORWARD.
                    # If d_step < 0, A -> B is BACKWARD.
                    new_dir = 0 if d_step > 0 else 1
                    cost = 0.5 if d_step > 0 else 1.25"""

content = content.replace(old_code, new_code)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'w') as f:
    f.write(content)
