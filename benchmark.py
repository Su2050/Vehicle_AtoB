"""
启发函数对比基准脚本

4 种组合：
  Geometric + Corridor    原始方案（基准）
  RS        + Corridor    RS 启发 + 走廊约束
  Geometric + NoCorridor  几何启发 + 无走廊
  RS        + NoCorridor  RS 启发 + 无走廊（验证目标）

运行方式：
    conda run -n forklift_sim python benchmark.py
"""
import main

prims = main.init_primitives()

cases = [
    (2.8,  0.0,  0.0, "simple straight"),
    (2.8,  1.5,  0.5, "offset + angled"),
    (2.5, -1.0, -0.3, "lateral offset"),
]

combos = [
    (False, False, "Geometric+Corridor"),
    (True,  False, "RS      +Corridor"),
    (False, True,  "Geometric+NoCorr  "),
    (True,  True,  "RS      +NoCorr   "),
]

HDR = "{:<24} {:<20} {:>10} {:>10} {:>7} {:>5}"
ROW = "{:<24} {:<20} {:>10} {:>10} {:>7} {:>5}"
SEP = "-" * 80

print(HDR.format("Case", "Mode", "Expanded", "Time(ms)", "Steps", "OK"))
print(SEP)

for x0, y0, th0, label in cases:
    print(f"  {label}  ({x0}, {y0:.1f}, {th0:.1f})")
    for use_rs, no_corr, mode_label in combos:
        st = {}
        ok, acts = main.plan_path(x0, y0, th0, prims,
                                  use_rs=use_rs,
                                  no_corridor=no_corr,
                                  stats=st)
        steps = str(len(acts)) if ok else "N/A"
        print(ROW.format(
            "",
            mode_label,
            st['expanded'],
            st['elapsed_ms'],
            steps,
            "YES" if ok else "NO"
        ))
    print()
