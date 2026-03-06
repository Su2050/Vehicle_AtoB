import re

with open('astar_core.py', 'r') as f:
    content = f.read()

old_print = "if expanded % 5000 == 0:"
new_print = "if expanded < 20 or expanded % 5000 == 0:"

content = content.replace(old_print, new_print)

with open('astar_core.py', 'w') as f:
    f.write(content)
