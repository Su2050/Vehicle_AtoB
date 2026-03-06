import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_nth = """        self.res_3d = 0.5
        self.nth = 16"""

new_nth = """        self.res_3d = 0.5
        self.nth = 32"""

content = content.replace(old_nth, new_nth)

with open('heuristic.py', 'w') as f:
    f.write(content)
