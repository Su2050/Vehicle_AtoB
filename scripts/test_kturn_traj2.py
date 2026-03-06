import json
import primitives
from collision import check_collision

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

fast_obstacles = []
for obs in case['obstacles']:
    ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
    fast_obstacles.append((min(ox, ox + ow), max(ox, ox + ow),
                 min(oy, oy + oh), max(oy, oy + oh)))

primitives.init_primitives()
precomp_prim = primitives.get_precomputed_primitives() if hasattr(primitives, 'get_precomputed_primitives') else primitives.init_primitives()

cx, cy, cth = 2.0679625078700687, -3.2891446039196035, -1.9221959562886552

valid_count = 0
for act, _n, traj in precomp_prim:
    if act[0] != 'R': continue
    
    import math
    cos_t, sin_t = math.cos(cth), math.sin(cth)
    ex = ey = eth = 0.0
    ok = True
    for dx, dy, dth, cdth, sdth in traj:
        ex = cx + dx * cos_t - dy * sin_t
        ey = cy + dx * sin_t + dy * cos_t
        eth = cth + dth
        if eth > math.pi: eth -= 2*math.pi
        elif eth <= -math.pi: eth += 2*math.pi
        sin_nth = sin_t * cdth + cos_t * sdth
        valid, reason = check_collision(ex, ey, eth, sin_nth, no_corridor=False, obstacles=fast_obstacles)
        if not valid:
            print(f"Action {act} failed at {ex:.2f}, {ey:.2f}, {eth:.2f}: {reason}")
            ok = False
            break
    if ok:
        valid_count += 1

print(f"Valid actions: {valid_count}")
