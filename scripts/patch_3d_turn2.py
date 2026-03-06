import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# The issue is that the vehicle is stuck in a loop.
# It moves forward, then backward, then forward, etc.
# Why? Because moving backward is penalized (cost 0.625) and changing direction is penalized (cost 3.0).
# But wait, if it's searching backwards from the goal, then:
# d_step > 0 means the vehicle moved FORWARD to reach the goal.
# d_step < 0 means the vehicle moved BACKWARD to reach the goal.
# So if the vehicle is currently at state A, and it wants to reach state B (which is closer to goal),
# it needs to know the cost of moving from A to B.
# In the Dijkstra search, we expand from B to A.
# B is the current node popped from queue. A is the neighbor.
# We are calculating the cost to reach A from B? No, cost to reach goal from A, given cost to reach goal from B.
# So cost(A) = cost(B) + transition_cost(A -> B).
# If A -> B is a FORWARD move, then transition_cost is 0.25.
# If A -> B is a BACKWARD move, then transition_cost is 0.625.
# How do we know if A -> B is forward or backward?
# Let's say d_step is the displacement from B to A.
# So x_A = x_B + d_step * cos(...)
# This means A is reached from B by moving d_step.
# But actually, the vehicle moves from A to B!
# So x_B = x_A - d_step * cos(...)
# This means the vehicle moves -d_step from A to B.
# So if d_step > 0, the vehicle moves -0.25 (BACKWARD) from A to B.
# If d_step < 0, the vehicle moves +0.25 (FORWARD) from A to B.
# Let's check the code:
# new_dir = 0 if d_step > 0 else 1
# cost = 0.25 if d_step > 0 else 0.625
# Wait! If d_step > 0, it means A is in front of B.
# So to go from A to B, the vehicle must move BACKWARD.
# But the code says: cost = 0.25 if d_step > 0 else 0.625.
# This means it assigns cost 0.25 (FORWARD cost) when the vehicle moves BACKWARD!
# And it assigns cost 0.625 (BACKWARD cost) when the vehicle moves FORWARD!
# This is a huge bug!

# Let's fix this.
# If d_step > 0, A is in front of B. Vehicle moves from A to B, which is a BACKWARD move.
# So d_step > 0 => BACKWARD move (dir 1, cost 0.625)
# d_step < 0 => FORWARD move (dir 0, cost 0.25)

old_code = """                    new_dir = 0 if d_step > 0 else 1
                    # d_step > 0 means vehicle moves FORWARDS (cost 0.25)
                    # d_step < 0 means vehicle moves BACKWARDS (cost 0.25 * 2.5 = 0.625)
                    cost = 0.25 if d_step > 0 else 0.625"""

new_code = """                    # d_step is B -> A. So A -> B is -d_step.
                    # If d_step > 0, A -> B is negative (BACKWARD).
                    # If d_step < 0, A -> B is positive (FORWARD).
                    new_dir = 1 if d_step > 0 else 0
                    cost = 0.625 if d_step > 0 else 0.25"""

content = content.replace(old_code, new_code)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'w') as f:
    f.write(content)
