import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_call2 = """        ok15, acts15, mx15, my15, mth15 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, ignore_obs_for_y_range=True,
            target_y_max=obs_tight_y, x_ceil=s15_x_ceil)"""
new_call2 = """        ok15, acts15, mx15, my15, mth15 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, ignore_obs_for_y_range=True,
            target_y_max=obs_tight_y, x_ceil=s15_x_ceil, x_floor=kturn_x_floor if 'kturn_x_floor' in locals() else None)"""

if old_call2 in content:
    content = content.replace(old_call2, new_call2)
    print("Patched successfully!")
else:
    print("Could not find old_call2!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
