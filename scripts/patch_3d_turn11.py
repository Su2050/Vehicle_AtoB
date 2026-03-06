import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# The problem is that in Dijkstra, we expand from B to A.
# B is the center of the cell: wx_B, wy_B, wth_B.
# We calculate A: new_wx, new_wy, new_wth.
# Then we find the cell of A: gx_A, gy_A, gth_A.
# We update the cost of cell A using the cost of cell B.
# BUT, when we query the heuristic at some continuous state (x, y, th) that falls into cell A,
# we are assuming that the optimal path from (x, y, th) to the goal is similar to the path from the center of cell A.
# However, if we trace the path FORWARD from (x, y, th), we might not be able to reach the center of cell B!
# Because the kinematics are non-holonomic.
# If we are at (x, y, th) in cell A, and we try all actions, we might end up in some cell C, not B.
# And cell C might have a higher heuristic value.
# This means the heuristic is not perfectly admissible/consistent for the continuous state space.
# But this is normal for a grid-based heuristic!
# The A* search should still be able to find a path, it just might need to expand more nodes.
# The problem in test_3d_path_fix8.py is that it's a greedy search!
# It only looks at the immediate neighbors and picks the best one.
# It doesn't use a priority queue like A*!
# So if it gets stuck in a local minimum, it fails.
# But A* would just put the neighbors in the open list and eventually find a way around!
# So the heuristic is actually FINE!
# The "stuck" behavior is just an artifact of my greedy test script!

# Let's verify this by running the actual A* search with the new heuristic!
