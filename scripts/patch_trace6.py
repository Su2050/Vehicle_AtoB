import re

with open('astar_core.py', 'r') as f:
    content = f.read()

old_print = "print(f\"[A*] expanded={expanded}, f={f:.1f}, cx={nx:.2f}, cy={ny:.2f}, th={nth:.2f}, elapsed={elapsed_ms}ms\")"
new_print = "print(f\"[A*] expanded={expanded}, f={f:.1f}, g={new_cost:.1f}, h={h:.1f}, cx={nx:.2f}, cy={ny:.2f}, th={nth:.2f}, elapsed={elapsed_ms}ms\")"

content = content.replace(old_print, new_print)

with open('astar_core.py', 'w') as f:
    f.write(content)
