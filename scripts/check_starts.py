import json
with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/logs/stress_timeouts_20260302_184337.json', 'r') as f:
    cases = json.load(f)
for c in cases:
    print(f"Case {c['case_id']}: x={c['start']['x']:.2f}, y={c['start']['y']:.2f}, th={c['start']['th']:.2f}")
