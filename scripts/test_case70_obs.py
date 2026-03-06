import json
with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
c = cases[0]
print(f"Case {c['case_id']}: start x={c['start']['x']:.2f}, y={c['start']['y']:.2f}, th={c['start']['th']:.2f}")
for obs in c['obstacles']:
    print(obs)
