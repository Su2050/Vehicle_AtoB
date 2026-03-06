import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# Fix how new_wth is calculated from wth
# In Dijkstra, we are searching BACKWARDS from the goal.
# If a vehicle moves forward (d_step > 0) from state A to state B,
# then state A is reached from state B by moving backward (-d_step).
# However, the heading changes: th_B = th_A + steer * d_step
# So th_A = th_B - steer * d_step
# AND the position changes: x_B = x_A + d_step * cos(th_A)
# So x_A = x_B - d_step * cos(th_A) => This is tricky because we only know th_B.
# For small d_step, x_A ~= x_B - d_step * cos(th_B)
# Let's check the current implementation:
# new_wth = wth - steer * d_step
# new_wx = wx - d_step * math.cos(new_wth)
# new_wy = wy - d_step * math.sin(new_wth)
# This is correct for backward search!

# BUT, the steer should be larger to allow turning in place? No, it's a car.
# Let's increase the steer options to match the actual vehicle capabilities.
# The primitives use steers: [-1.0, -0.5, 0.0, 0.5, 1.0]
# The heuristic uses steers: [0.0, 0.15, -0.15]
# Let's change the heuristic steers to match the primitives.
# Wait, 0.15 is the angle change per step?
# If d_step = 0.25, and steer = 1.0, then angle change is d_step / L * tan(steer)
# L = 1.5, steer = 1.0 rad? No, steer is steering angle.
# In primitives: dth = v * math.tan(s) / primitives.VEHICLE_LENGTH * dt
# If v=0.25, dt=1.0, s=1.0, dth = 0.25 * math.tan(1.0) / 1.5 = 0.25 * 1.557 / 1.5 = 0.26 rad
# So for d_step = 0.25, angle change is around 0.26 rad.
# The heuristic uses 0.15, which is a bit small. Let's change it to 0.25.

content = content.replace("for steer in [0.0, 0.15, -0.15]:", "for steer in [0.0, 0.25, -0.25]:")

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'w') as f:
    f.write(content)
