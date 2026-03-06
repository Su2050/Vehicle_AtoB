import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_kturn = """def _k_turn_preposition_obs(x0, y0, theta0, precomp_prim, no_corridor, fast_obstacles,
                            dijkstra_grid=None, ignore_obs_for_y_range=False,
                            target_y_max=None, x_ceil=None):"""
new_kturn = """def _k_turn_preposition_obs(x0, y0, theta0, precomp_prim, no_corridor, fast_obstacles,
                            dijkstra_grid=None, ignore_obs_for_y_range=False,
                            target_y_max=None, x_ceil=None):
    print(f"DEBUG: _k_turn_preposition_obs called with x0={x0}, y0={y0}, theta0={theta0}")"""

if old_kturn in content:
    content = content.replace(old_kturn, new_kturn)
    print("Patched successfully!")
else:
    print("Could not find old_kturn!")

with open('planner_obs.py', 'w') as f:
    f.write(content)
