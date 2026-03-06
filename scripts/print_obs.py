import json
with open("logs/stress_timeouts_20260302_184337.json", "r") as f:
    data = json.load(f)
case = next(c for c in data if c['case_id'] == 2670)
print(case['obstacles'])
