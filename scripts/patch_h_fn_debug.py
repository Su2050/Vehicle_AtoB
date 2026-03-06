import sys

with open('planner_obs_v2.py', 'r') as f:
    content = f.read()

old_code = """    def h_fn(nx, ny, nth):
        h_3d = dijkstra_grid.get_3d_heuristic(nx, ny, nth)
        if h_3d == float('inf'):
            return 1e6, 1.0"""

new_code = """    def h_fn(nx, ny, nth):
        h_3d = dijkstra_grid.get_3d_heuristic(nx, ny, nth)
        if h_3d == float('inf'):
            print(f"DEBUG H_FN: h_3d is inf at ({nx:.2f}, {ny:.2f}, {nth:.2f})")
            return 1e6, 1.0"""

if old_code in content:
    content = content.replace(old_code, new_code)
    with open('planner_obs_v2.py', 'w') as f:
        f.write(content)
    print("Patched h_fn successfully!")
else:
    print("Could not find the target code block.")
