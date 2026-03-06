import json
from planner_obs_v2 import _generate_bypass_milestones, _try_milestone_rs_bypass
from planner_obs import _make_collision_fn, _preprocess_obstacles
import primitives
import rs

with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]

fast_obs = _preprocess_obstacles(case['obstacles'])
coll_fn = _make_collision_fn(False, fast_obs)

milestones = _generate_bypass_milestones(
    case['start']['x'], case['start']['y'], case['start']['th'],
    fast_obs)

print(f"Generated {len(milestones)} milestones")

for mx, my, mth, _tag in milestones:
    seg2_list = rs.rs_sample_path_multi(
        mx, my, mth, primitives.RS_GOAL_X, primitives.RS_GOAL_Y, primitives.RS_GOAL_TH,
        primitives.MIN_TURN_RADIUS, step=0.2, max_paths=3)
    
    for seg2 in seg2_list:
        if not seg2: continue
        
        ok = True
        for pt in seg2:
            valid, _ = coll_fn(pt[0], pt[1], pt[2])
            if not valid:
                ok = False
                break
        if ok:
            print(f"Milestone {mx:.2f}, {my:.2f}, {mth:.2f} works for seg2!")
            break
