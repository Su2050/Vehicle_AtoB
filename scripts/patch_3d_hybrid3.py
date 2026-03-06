import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_loop = """            for d_step in [1.0, -1.0]:
                for steer in [0.0, 0.4, -0.4]:
                    cost = 1.0 if d_step == -1.0 else 1.5
                    if steer != 0.0: cost += 0.2"""

new_loop = """            for d_step in [0.5, -0.5]:
                for steer in [0.0, 0.2, -0.2]:
                    cost = 0.5 if d_step == -0.5 else 0.75
                    if steer != 0.0: cost += 0.1"""

content = content.replace(old_loop, new_loop)

with open('heuristic.py', 'w') as f:
    f.write(content)
