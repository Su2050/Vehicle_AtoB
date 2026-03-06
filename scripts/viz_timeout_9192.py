import json
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from planner_obs_v2 import plan_path_robust_obs_v2
import primitives
import math

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)

case = next(c for c in cases if c['case_id'] == 9192)
start = case['start']
obstacles = case['obstacles']

precomp_prim = primitives.init_primitives()
stats = {}
success, acts, traj = plan_path_robust_obs_v2(start['x'], start['y'], start['th'], precomp_prim, obstacles=obstacles, stats=stats)

print(f"Success: {success}, Stats: {stats}")

fig, ax = plt.subplots(figsize=(10, 10))
ax.set_aspect('equal')
ax.set_xlim(0, 10)
ax.set_ylim(-5, 5)

# Draw obstacles
for obs in obstacles:
    rect = patches.Rectangle((obs['x'], obs['y']), obs['w'], obs['h'], linewidth=1, edgecolor='r', facecolor='red', alpha=0.5)
    ax.add_patch(rect)

# Draw start and goal
ax.plot(start['x'], start['y'], 'go', markersize=10)
ax.arrow(start['x'], start['y'], math.cos(start['th'])*0.5, math.sin(start['th'])*0.5, head_width=0.1, head_length=0.1, fc='g', ec='g')

ax.plot(2.25, 0.0, 'bo', markersize=10)
ax.arrow(2.25, 0.0, 0.5, 0.0, head_width=0.1, head_length=0.1, fc='b', ec='b')

# Draw trajectory
if traj:
    xs = [p[0] for p in traj]
    ys = [p[1] for p in traj]
    ax.plot(xs, ys, 'b-', linewidth=2)

plt.grid(True)
plt.savefig('timeout_9192.png')
print("Saved to timeout_9192.png")
