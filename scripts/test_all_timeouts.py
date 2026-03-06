import json
import time
from planner_obs_v2 import plan_path_robust_obs_v2
import primitives

def replay_all():
    with open("logs/stress_timeouts_20260302_184337.json", 'r') as f:
        data = json.load(f)
    
    precomp_prim = primitives.init_primitives()
    
    success_count = 0
    total_time = 0
    
    for case in data:
        start = case['start']
        obstacles = case['obstacles']
        
        print(f"Replaying Case {case['case_id']}...")
        t0 = time.perf_counter()
        stats = {}
        res = plan_path_robust_obs_v2(start['x'], start['y'], start['th'], precomp_prim, obstacles=obstacles, stats=stats)
        t1 = time.perf_counter()
        
        if res[0]:
            success_count += 1
            print(f"  Success in {t1-t0:.2f}s (level: {stats.get('level')})")
        else:
            print(f"  Failed in {t1-t0:.2f}s")
            
        total_time += (t1 - t0)
        
    print(f"Solved {success_count}/{len(data)} timeout cases in {total_time:.2f}s total.")

if __name__ == "__main__":
    replay_all()
