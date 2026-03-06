import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# Wait, if d_step > 0, A -> B is FORWARD.
# This means the vehicle moves FORWARD from A to B.
# So the gear at A should be FORWARD (0).
# In the code: new_dir = 0 if d_step > 0 else 1.
# This is correct.
# But what about the gear change penalty?
# if new_dir != gdir: cost += 3.0
# gdir is the gear at B. new_dir is the gear at A.
# If the vehicle moves FORWARD from A to B (new_dir=0), and then at B it moves BACKWARD (gdir=1),
# then there is a gear change at B.
# So the cost of A should include the gear change penalty.
# This is correct.

# So why is the heuristic value increasing as we move towards the start?
# At the start state: gx=24, gy=34, gth=1, gear=0, h=21.04
# Action d_step=0.5, steer=0.0: nx=25, ny=35, nth=1, dir=0, h=21.54, total=22.04
# Action d_step=-0.5, steer=0.0: nx=22, ny=33, nth=1, dir=1, h=25.29, total=29.54
# Wait, if we are at A (gx=24, gy=34), and we want to find the best neighbor B to move to.
# We should look at the neighbors and find the one that minimizes:
# h(B) + transition_cost(A -> B).
# In test_3d_path_fix3.py, I am generating neighbors B from A.
# new_wx = x - d_step * math.cos(new_wth)
# This means B = A - d_step * cos(...) => A = B + d_step * cos(...)
# Wait! In test_3d_path_fix3.py, I used the SAME formula as in the Dijkstra search!
# But in Dijkstra, we are expanding from B to A!
# So if I am at A, and I want to find B, I should use:
# new_wx = x + d_step * math.cos(th)
# new_wy = y + d_step * math.sin(th)
# new_wth = th + steer * d_step
# This is the FORWARD kinematics!
