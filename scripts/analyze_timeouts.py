import json

with open("logs/stress_timeouts_20260302_184337.json", "r") as f:
    data = json.load(f)

print(f"Total timeout cases: {len(data)}")
for i, case in enumerate(data[:32]):
    print(f"Case {i}: ID={case['case_id']}, Start={case['start']}, Obstacles={len(case['obstacles'])}")
