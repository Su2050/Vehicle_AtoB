import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# The heuristic is missing a penalty for moving backward?
# Wait, if d_step > 0, cost is 0.625. If d_step < 0, cost is 0.25.
# And if new_dir != gdir, cost += 3.0.
# The problem is that the state space is too coarse.
# res_3d = 0.25, res_th = 22.5 deg.
# d_step = 0.25. So moving 1 d_step might not change the grid cell!
# If nx_g == gx and ny_g == gy and nth_g == gth and new_dir == gdir:
# It will just loop!
# We need to ensure that d_step is large enough to change the grid cell,
# or we use a smaller grid res, or we don't allow self-loops.
# Let's check:
# if nx_g == gx and ny_g == gy and nth_g == gth and new_dir == gdir: continue

old_code = """                    if 0 <= nx_g < self.nx_3d and 0 <= ny_g < self.ny_3d:
                        is_obs = False"""

new_code = """                    if nx_g == gx and ny_g == gy and nth_g == gth and new_dir == gdir:
                        continue
                    if 0 <= nx_g < self.nx_3d and 0 <= ny_g < self.ny_3d:
                        is_obs = False"""

content = content.replace(old_code, new_code)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'w') as f:
    f.write(content)
