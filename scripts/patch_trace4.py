import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_print = "    print(f\"Start A* from: mx={mx:.2f}, my={my:.2f}, mth={mth:.2f}\")"
new_print = "    print(f\"Start A* from: mx={mx:.2f}, my={my:.2f}, mth={mth:.2f}, ok1_greedy={ok1_greedy if 'ok1_greedy' in locals() else 'None'}, ok15={ok15 if 'ok15' in locals() else 'None'}\")"

content = content.replace(old_print, new_print)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
