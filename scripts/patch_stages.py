import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_stage3 = """    {
        'name': 'aggressive',
        'expand_limit': 100000,  # Increased from 80000
        'prim_limit': 120,
        'rs_radius': 10.0,
        'allow_uphill': 100.0,
        'max_rs_paths': 15,
        'h_weight': 2.0,
    },"""

new_stage3 = """    {
        'name': 'aggressive',
        'expand_limit': 100000,
        'prim_limit': 120,
        'rs_radius': 6.0,
        'allow_uphill': 100.0,
        'max_rs_paths': 8,
        'h_weight': 2.0,
    },"""

content = content.replace(old_stage3, new_stage3)

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
