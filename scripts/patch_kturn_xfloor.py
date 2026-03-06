import sys

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_code = """        kturn_x_floor = None
        if fast_obstacles:
            for obs in fast_obstacles:
                if obs[0] > mx:  # Obstacle is in front of us
                    continue
                # Find the rightmost edge of obstacles that are to our left
                if kturn_x_floor is None or obs[1] > kturn_x_floor:
                    kturn_x_floor = obs[1]
            if kturn_x_floor is not None:
                kturn_x_floor += 0.5  # Add some margin"""

new_code = """        kturn_x_floor = None
        if fast_obstacles:
            for obs in fast_obstacles:
                if obs[1] >= mx:  # Obstacle is in front of us or overlaps us
                    continue
                # Find the rightmost edge of obstacles that are strictly behind us
                if kturn_x_floor is None or obs[1] > kturn_x_floor:
                    kturn_x_floor = obs[1]
            if kturn_x_floor is not None:
                kturn_x_floor += 0.5  # Add some margin"""

if old_code in content:
    content = content.replace(old_code, new_code)
    with open('planner_obs_v2.py', 'w') as f:
        f.write(content)
    print("Patched kturn_x_floor successfully!")
else:
    print("Could not find the target code block.")
