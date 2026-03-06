import sys

with open('heuristic.py', 'r') as f:
    content = f.read()

old_code = """        from primitives import MAX_PLANNABLE_Y
        boundary_obs = [
            (-1.0, 12.0, MAX_PLANNABLE_Y, 6.0),
            (-1.0, 12.0, -6.0, -MAX_PLANNABLE_Y),
            (-1.0, 0.0, -6.0, 6.0),
            (11.0, 12.0, -6.0, 6.0)
        ]"""

new_code = """        from primitives import MAX_PLANNABLE_Y
        boundary_obs = [
            (-1.0, 0.0, -6.0, 6.0),
            (11.0, 12.0, -6.0, 6.0)
        ]"""

if old_code in content:
    content = content.replace(old_code, new_code)
    with open('heuristic.py', 'w') as f:
        f.write(content)
    print("Patched boundary_obs successfully!")
else:
    print("Could not find the target code block.")
