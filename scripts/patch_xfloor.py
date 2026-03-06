import re

with open('planner_obs.py', 'r') as f:
    content = f.read()

old_def = """def _k_turn_preposition_obs(x0, y0, theta0, precomp_prim, no_corridor, fast_obstacles,
                            dijkstra_grid=None, ignore_obs_for_y_range=False,
                            target_y_max=None, x_ceil=None):"""
new_def = """def _k_turn_preposition_obs(x0, y0, theta0, precomp_prim, no_corridor, fast_obstacles,
                            dijkstra_grid=None, ignore_obs_for_y_range=False,
                            target_y_max=None, x_ceil=None, x_floor=None):"""

old_floor = """    X_FLOOR_SAFE = 2.05
    X_FLOOR_EMG = 1.92"""
new_floor = """    X_FLOOR_SAFE = x_floor if x_floor is not None else 2.05
    X_FLOOR_EMG = x_floor if x_floor is not None else 1.92"""

if old_def in content:
    content = content.replace(old_def, new_def)
    content = content.replace(old_floor, new_floor)
    print("Patched planner_obs.py successfully!")
else:
    print("Could not find old_def!")

with open('planner_obs.py', 'w') as f:
    f.write(content)

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_call = """        ok1_greedy, acts1_greedy, mx2, my2, mth2 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, target_y_max=obs_tight_y)"""
new_call = """        # Determine x_floor based on obstacles
        kturn_x_floor = None
        if fast_obstacles:
            for obs in fast_obstacles:
                if obs[0] > mx:  # Obstacle is in front of us
                    continue
                # Find the rightmost edge of obstacles that are to our left
                if kturn_x_floor is None or obs[1] > kturn_x_floor:
                    kturn_x_floor = obs[1]
            if kturn_x_floor is not None:
                kturn_x_floor += 0.5  # Add some margin
                
        ok1_greedy, acts1_greedy, mx2, my2, mth2 = _k_turn_preposition_obs(
            mx, my, mth, precomp_prim, no_corridor, fast_obstacles,
            dijkstra_grid=dijkstra_grid, target_y_max=obs_tight_y, x_floor=kturn_x_floor)"""

old_skip = "    skip_kturn = True"
new_skip = "    skip_kturn = False"

if old_call in content:
    content = content.replace(old_call, new_call)
    content = content.replace(old_skip, new_skip)
    print("Patched planner_obs_v2.py successfully!")
else:
    print("Could not find old_call!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
