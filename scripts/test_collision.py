from collision import check_collision
import math

fast_obstacles = [
    (6.02, 6.66, 0.44, 2.01),
    (6.96, 7.58, 0.45, 2.01),
    (7.88, 8.5, 0.45, 2.01),
    (6.05, 6.66, -2.01, -0.45),
    (6.97, 7.58, -2.01, -0.45),
    (7.89, 8.5, -2.01, -0.45)
]

cx, cy, cth = 6.01, 3.80, -1.85
is_valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=True, obstacles=fast_obstacles)
print(f"Collision check for {cx}, {cy}, {cth}: valid={is_valid}, reason={reason}")

cx, cy, cth = 6.14, 4.54, -1.55
is_valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=True, obstacles=fast_obstacles)
print(f"Collision check for {cx}, {cy}, {cth}: valid={is_valid}, reason={reason}")

cx, cy, cth = 5.50, 3.80, -1.57
is_valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=True, obstacles=fast_obstacles)
print(f"Collision check for {cx}, {cy}, {cth}: valid={is_valid}, reason={reason}")


cx, cy, cth = 6.00, 2.00, -1.57
is_valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=True, obstacles=fast_obstacles)
print(f"Collision check for {cx}, {cy}, {cth}: valid={is_valid}, reason={reason}")

cx, cy, cth = 5.53, 2.00, -1.57
is_valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=True, obstacles=fast_obstacles)
print(f"Collision check for {cx}, {cy}, {cth}: valid={is_valid}, reason={reason}")
