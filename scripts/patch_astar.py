import re

with open('astar_core.py', 'r') as f:
    content = f.read()

old_print = "print(f\"[A*] expanded={expanded}, f={f:.1f}, cx={cx:.2f}, cy={cy:.2f}, elapsed={elapsed_so_far:.0f}ms\")"
new_print = "print(f\"[A*] expanded={expanded}, f={f:.1f}, cx={cx:.2f}, cy={cy:.2f}, th={cth:.2f}, elapsed={elapsed_so_far:.0f}ms\")"

content = content.replace(old_print, new_print)

with open('astar_core.py', 'w') as f:
    f.write(content)
