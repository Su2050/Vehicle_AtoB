from collision import check_collision
import math

obs = [
    (6.03, 6.66, 1.65, 2.06),
    (3.28, 4.21, -1.4, 0.43),
    (6.79, 7.15, 2.57, 4.25),
    (3.87, 4.39, -2.43, -1.61)
]

cx, cy, cth = 6.14, 4.54, -1.55
print(f"Start: {cx:.2f}, {cy:.2f}, {cth:.2f}")

# Steer right and go forward
d_step = 0.5
steer = -0.4 # right steer? Wait, steer is dth.
# In astar_core.py, precomp_prim has act[1] as steer.
# If act[1] < 0, it's right steer.
# Let's just apply dth = -0.2 rad per 0.5m.

for i in range(10):
    dth = -0.2
    cth += dth
    cx += d_step * math.cos(cth)
    cy += d_step * math.sin(cth)
    valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=True, obstacles=obs)
    print(f"Step {i+1}: {cx:.2f}, {cy:.2f}, {cth:.2f}, valid={valid}, reason={reason}")


print("\nNow turn left to go down:")
cx, cy, cth = 3.87, 2.24, -2.95
for i in range(10):
    dth = 0.2
    cth += dth
    cx += d_step * math.cos(cth)
    cy += d_step * math.sin(cth)
    valid, reason = check_collision(cx, cy, cth, math.sin(cth), math.cos(cth), no_corridor=True, obstacles=obs)
    print(f"Step {i+1}: {cx:.2f}, {cy:.2f}, {cth:.2f}, valid={valid}, reason={reason}")
