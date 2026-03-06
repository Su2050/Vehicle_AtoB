import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_res = """        self.res_3d = 0.5
        self.res_th = math.pi / 4
        self.nth = 8"""

new_res = """        self.res_3d = 0.25
        self.res_th = math.pi / 4
        self.nth = 8"""

content = content.replace(old_res, new_res)

old_step = """                step_dist = self.res_3d * 1.5"""

new_step = """                step_dist = self.res_3d * 1.5"""

content = content.replace(old_step, new_step)

with open('heuristic.py', 'w') as f:
    f.write(content)
