from test_fast_obs import grid
import math

path = [
    (6.14, 4.54, -1.55),
    (6.05, 4.05, -1.75),
    (5.87, 3.58, -1.95),
    (5.59, 3.17, -2.15),
    (5.24, 2.81, -2.35),
    (4.83, 2.53, -2.55),
    (4.36, 2.34, -2.75),
    (3.87, 2.24, -2.95),
    (3.41, 2.05, -2.75),
    (2.99, 1.77, -2.55),
    (2.64, 1.41, -2.35),
    (2.37, 1.00, -2.15),
    (2.18, 0.53, -1.95),
    (2.09, 0.04, -1.75)
]

for i, (cx, cy, cth) in enumerate(path):
    h = grid.get_3d_heuristic(cx, cy, cth)
    print(f"Step {i}: {cx:.2f}, {cy:.2f}, {cth:.2f} -> h_3d={h:.2f}")

print("\nGoing left (steer right):")
cx, cy, cth = 6.14, 4.54, -1.55
for i in range(5):
    dth = 0.2
    cth += dth
    cx += 0.5 * math.cos(cth)
    cy += 0.5 * math.sin(cth)
    h = grid.get_3d_heuristic(cx, cy, cth)
    print(f"Step {i+1}: {cx:.2f}, {cy:.2f}, {cth:.2f} -> h_3d={h:.2f}")

print("\nGoing backward:")
cx, cy, cth = 6.14, 4.54, -1.55
for i in range(5):
    dth = 0.0
    cth += dth
    cx -= 0.5 * math.cos(cth)
    cy -= 0.5 * math.sin(cth)
    h = grid.get_3d_heuristic(cx, cy, cth)
    print(f"Step {i+1}: {cx:.2f}, {cy:.2f}, {cth:.2f} -> h_3d={h:.2f}")
