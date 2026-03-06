import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# The cost of moving backward is 1.25, forward is 0.5.
# If we are at B, and d_step = 0.5, A is in front of B.
# So A -> B is a BACKWARD move (-0.5).
# The cost should be 0.5 * 2.5 = 1.25.
# If d_step = -0.5, A is behind B.
# So A -> B is a FORWARD move (+0.5).
# The cost should be 0.5.
# This seems correct.

# Wait, the problem is that the state is stuck.
# Current: x=5.92, y=3.16, th=-2.63, gx=27, gy=36, gth=1, gear=0, h=23.76
# Action d_step=0.5, steer=0.0: nx=29, ny=37, nth=1, dir=1, h=20.26
# Action d_step=-0.5, steer=0.0: nx=25, ny=35, nth=1, dir=0, h=22.51
# The best neighbor is nx=29, ny=37, nth=1, dir=1 with h=20.26.
# But the current gear is 0 (FORWARD).
# If we choose d_step=0.5, the new dir is 1 (BACKWARD).
# So we are changing gear!
# The cost of changing gear is 3.0.
# So the total cost from nx=29, ny=37, nth=1, dir=1 to current state (dir=0)
# is h=20.26 + 1.25 (backward move) + 3.0 (gear change) = 24.51.
# But the current state has h=23.76!
# So 24.51 > 23.76, which means this neighbor is NOT better.
# Let's check the other neighbor:
# Action d_step=-0.5, steer=0.0: nx=25, ny=35, nth=1, dir=0, h=22.51
# The new dir is 0 (FORWARD).
# The cost is 0.5 (forward move).
# Total cost = 22.51 + 0.5 = 23.01.
# But wait, 23.01 < 23.76!
# So why didn't it choose this neighbor?
# Because in test_3d_path.py, we are not adding the transition cost!
# We are just looking for the neighbor with the absolute lowest h!
# Ah! That's a bug in test_3d_path.py, not in the heuristic itself!
# The heuristic might be perfectly fine!

