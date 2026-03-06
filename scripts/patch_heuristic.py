import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_penalty = """            # Apply heading alignment penalty near goal
            if wth is not None and dist < 5.0:
                goal_th = 0.0 
                diff_goal = abs(wth - goal_th)
                while diff_goal > math.pi: diff_goal -= 2 * math.pi
                diff_goal = abs(diff_goal)
                align_goal_diff = min(diff_goal, math.pi - diff_goal)
                
                weight = (5.0 - dist) * 2.0
                dist += align_goal_diff * weight"""

new_penalty = """            # Apply heading alignment penalty near goal
            if wth is not None and dist < 5.0:
                goal_th = 0.0 
                diff_goal = abs(wth - goal_th)
                while diff_goal > math.pi: diff_goal -= 2 * math.pi
                diff_goal = abs(diff_goal)
                align_goal_diff = min(diff_goal, math.pi - diff_goal)
                
                # Constant weight prevents local minima where vehicle stays away from goal
                # just to reduce the heading penalty.
                weight = 8.0
                dist += align_goal_diff * weight"""

content = content.replace(old_penalty, new_penalty)

with open('heuristic.py', 'w') as f:
    f.write(content)
