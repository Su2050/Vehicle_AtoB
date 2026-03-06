import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_init = """    def __init__(self, goal_x, goal_y, inflate_radius=0.50):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.inflate_radius = inflate_radius
        self.res = 0.15
        self.min_x = -1.0"""

new_init = """    def __init__(self, goal_x, goal_y, inflate_radius=0.50):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.inflate_radius = inflate_radius
        self.res = 0.10
        self.min_x = -1.0"""

if old_init in content:
    content = content.replace(old_init, new_init)
    print("Patched successfully!")
else:
    print("Could not find old_init!")

with open('heuristic.py', 'w') as f:
    f.write(content)
