import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_loop = """        while q:
            d, x, y, th = heapq.heappop(q)"""

new_loop = """        expanded = 0
        while q:
            d, x, y, th = heapq.heappop(q)
            expanded += 1"""

content = content.replace(old_loop, new_loop)

old_end = """    def get_3d_heuristic(self, wx, wy, wth):"""

new_end = """        print(f"3D Heuristic built, expanded {expanded} nodes")
    def get_3d_heuristic(self, wx, wy, wth):"""

content = content.replace(old_end, new_end)

with open('heuristic.py', 'w') as f:
    f.write(content)
