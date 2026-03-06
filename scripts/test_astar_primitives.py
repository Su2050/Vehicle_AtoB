import math
from primitives import init_primitives

precomp_prim = init_primitives()
cx, cy, cth = 6.01, 3.80, -1.85

print(f"State: {cx}, {cy}, {cth}")
for act, N, traj_base in precomp_prim:
    if act[0] == 'R':
        cos_th = math.cos(cth)
        sin_th = math.sin(cth)
        traj = []
        for dx, dy, dth, _, _ in traj_base:
            nx = cx + dx * cos_th - dy * sin_th
            ny = cy + dx * sin_th + dy * cos_th
            nth = cth + dth
            traj.append((nx, ny, nth))
        ex, ey, eth = traj[-1]
        print(f"Action: {act}, End: {ex:.2f}, {ey:.2f}, {eth:.2f}")
