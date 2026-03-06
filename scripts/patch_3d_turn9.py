import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# Let's check the original heuristic.py logic before my changes:
# new_dir = 0 if d_step > 0 else 1
# cost = 0.25 if d_step > 0 else 0.625
# new_wth = wth - steer * d_step
# new_wx = wx - d_step * math.cos(new_wth)
# new_wy = wy - d_step * math.sin(new_wth)

# Wait, if I am at B (wx, wy) and I want to find A (new_wx, new_wy) such that A -> B is a valid move.
# If A -> B is a FORWARD move (+d_step), then:
# B = A + d_step * cos(th_A)
# th_B = th_A + steer * d_step
# So th_A = th_B - steer * d_step
# A = B - d_step * cos(th_A)
# This means:
# new_wth = wth - steer * d_step
# new_wx = wx - d_step * math.cos(new_wth)
# new_wy = wy - d_step * math.sin(new_wth)
# This perfectly matches the original code!
# And if d_step > 0, it means A -> B is a FORWARD move.
# So the gear at A should be FORWARD (0).
# And the cost should be the FORWARD cost (0.25).
# So:
# new_dir = 0 if d_step > 0 else 1
# cost = 0.25 if d_step > 0 else 0.625
# This is EXACTLY the original code!

# My previous patch patch_3d_turn2.py changed it to:
# new_dir = 1 if d_step > 0 else 0
# cost = 0.625 if d_step > 0 else 0.25
# This was WRONG!

# My patch patch_3d_turn6.py changed it back to:
# new_dir = 0 if d_step > 0 else 1
# cost = 0.5 if d_step > 0 else 1.25
# This is CORRECT!

# So why did the path loop in test_3d_path_fix6.py?
# Because in test_3d_path_fix6.py, I am tracing the path FORWARD from start to goal.
# So I am at A, and I want to find B.
# B = A + d_step * cos(th_A)
# th_B = th_A + steer * d_step
# But in test_3d_path_fix6.py, I used:
# new_wth = th + steer * d_step
# new_wx = x + d_step * math.cos(th)
# new_wy = y + d_step * math.sin(th)
# This is correct for FORWARD tracing.
# And I checked: h(B) + transition_cost(A -> B) < best_cost.
# But wait, in Dijkstra, h(A) = h(B) + transition_cost(A -> B).
# So h(B) = h(A) - transition_cost(A -> B).
# So we should look for a neighbor B such that h(B) is approximately h(A) - transition_cost(A -> B).
# Or simply, the neighbor B that minimizes h(B) + transition_cost(A -> B)?
# No! If we want to reach the goal, we want to minimize h(B).
# But we also want to minimize the cost to reach B.
# So we should choose B that minimizes: h(B) + transition_cost(A -> B).
# But wait, if h(A) is the optimal cost to reach the goal from A,
# then there MUST exist a neighbor B such that h(A) = h(B) + transition_cost(A -> B).
# So we should just find the neighbor B that minimizes h(B) + transition_cost(A -> B).
# And this minimum value should be exactly h(A)!

# Let's check the output of test_3d_path_fix6.py:
# Start state: gx=24, gy=34, gth=1, gear=0, h=21.04
# Action d_step=0.5, steer=0.0: nx=22, ny=33, nth=1, dir=0, h=20.54, total=21.04
# This matches perfectly! h(A) = 21.04, h(B) = 20.54, cost = 0.5. total = 21.04.
# So the best neighbor is nx=22, ny=33, nth=1, dir=0.
# Then at nx=22, ny=33, nth=1, gear=0, h=20.54:
# Action d_step=0.5, steer=0.0: nx=20, ny=32, nth=1, dir=0, h=20.04, total=20.54
# This also matches!
# But then it goes to nx=17, ny=30, nth=1, gear=0, h=19.54.
# Then nx=15, ny=29, nth=1, gear=0, h=19.28.
# Then from nx=15, ny=29, nth=1, gear=0, h=19.28:
# It goes to nx=17, ny=30, nth=1, gear=1, h=24.03.
# Wait! Why did it go to h=24.03?
# Because from nx=15, ny=29, nth=1, gear=0, there was NO valid neighbor B that had total_cost <= 19.28!
# Why? Because nx=15, ny=29 is close to an obstacle, and maybe the next step is blocked!
# Let's check the obstacle map!

