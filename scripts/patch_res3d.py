import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_res3d = "        self.res_3d = 0.5"
new_res3d = "        self.res_3d = 0.25"

if old_res3d in content:
    content = content.replace(old_res3d, new_res3d)
    print("Patched successfully!")
else:
    print("Could not find old_res3d!")

with open('heuristic.py', 'w') as f:
    f.write(content)
