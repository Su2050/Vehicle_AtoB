import re

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_call = """        # Determine x_floor based on obstacles
        kturn_x_floor = None
        if fast_obstacles:
            for obs in fast_obstacles:
                if obs[0] > mx:  # Obstacle is in front of us
                    continue
                # Find the rightmost edge of obstacles that are to our left
                if kturn_x_floor is None or obs[1] > kturn_x_floor:
                    kturn_x_floor = obs[1]
            if kturn_x_floor is not None:
                kturn_x_floor += 0.5  # Add some margin"""
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
        print(f"DEBUG: kturn_x_floor={kturn_x_floor}")"""

if old_call in content:
    content = content.replace(old_call, new_call)
    print("Patched successfully!")
else:
    print("Could not find old_call!")

with open('planner_obs_v2.py', 'w') as f:
    f.write(content)
