import json
import primitives
from collision import check_collision
import math

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

mx, my, mth = case['start']['x'], case['start']['y'], case['start']['th']

valid_count = 0
for act, _n, traj in precomp_prim:
    if abs(act[1]) < 0.5: continue
    
    cos_t, sin_t = math.cos(mth), math.sin(mth)
    ex = ey = eth = 0.0
    ok = True
    for dx, dy, dth, cdth, sdth in traj:
        ex = mx + dx * cos_t - dy * sin_t
        ey = my + dx * sin_t + dy * cos_t
        eth = mth + dth
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
        print(f"Action {act} is VALID! Ends at {ex:.2f}, {ey:.2f}, {eth:.2f}")

print(f"Valid actions: {valid_count}")
