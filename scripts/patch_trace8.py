import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_h = "        return h, h_weight"
new_h = "        if nx == 2.21 and ny == -2.98:\n            print(f\"DEBUG H: nx={nx}, ny={ny}, nth={nth}, h_3d={h_3d}, h_rs={h_rs if 'h_rs' in locals() else -1}, th_err={th_err if 'th_err' in locals() else -1}, h={h}\")\n        return h, h_weight"

content = content.replace(old_h, new_h)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
