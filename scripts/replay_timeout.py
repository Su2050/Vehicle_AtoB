import json
import time
import sys
from planner_obs_v2 import plan_path_robust_obs_v2
import primitives

def replay(case_id, json_path="logs/stress_timeouts_20260302_184337.json"):
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    case = next((c for c in data if c['case_id'] == case_id), None)
    if not case:
        print(f"Case {case_id} not found.")
        return
    
    start = case['start']
    obstacles = case['obstacles']
    
    print(f"Replaying Case {case_id}: start={start}, n_obs={len(obstacles)}")
    
    precomp_prim = primitives.init_primitives()
    
    t0 = time.perf_counter()
    stats = {}
    res = plan_path_robust_obs_v2(start['x'], start['y'], start['th'], precomp_prim, obstacles=obstacles, stats=stats)
    t1 = time.perf_counter()
    
    print(f"Result: {res[0]}, Time: {t1-t0:.2f}s, Stats: {stats}")
    
if __name__ == "__main__":
    if len(sys.argv) > 1:
        replay(int(sys.argv[1]))
    else:
        with open("logs/stress_timeouts_20260302_184337.json", 'r') as f:
            data = json.load(f)
        replay(data[0]['case_id'])
