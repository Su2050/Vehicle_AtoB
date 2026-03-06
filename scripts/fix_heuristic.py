import re

with open('heuristic.py', 'r') as f:
    lines = f.readlines()

# Find where DijkstraGrid ends (before geometric_heuristic)
insert_idx = 0
for i, line in enumerate(lines):
    if line.startswith("def geometric_heuristic"):
        insert_idx = i
        break

# Find where build_3d_map starts
start_3d = 0
for i, line in enumerate(lines):
    if "def build_3d_map" in line:
        start_3d = i
        break

if start_3d > insert_idx:
    new_lines = lines[:insert_idx] + lines[start_3d:] + lines[insert_idx:start_3d]
    with open('heuristic.py', 'w') as f:
        f.writelines(new_lines)
