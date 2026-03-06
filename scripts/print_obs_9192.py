import json
with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
case = next(c for c in cases if c['case_id'] == 9192)
for obs in case['obstacles']:
    print(f"x: {obs['x']:.2f}, y: {obs['y']:.2f}, w: {obs['w']:.2f}, h: {obs['h']:.2f}")
