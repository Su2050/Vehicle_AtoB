import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_print = "    t1_ms = round((time.perf_counter() - t1) * 1000.0, 1)"
new_print = """    print(f"Start A* from: mx={mx:.2f}, my={my:.2f}, mth={mth:.2f}")
    t1_ms = round((time.perf_counter() - t1) * 1000.0, 1)"""

content = content.replace(old_print, new_print)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
