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
import sys, io
import main

prims = main.init_primitives()

cases = [
    (2.8,  0.0,  0.0, "直线接近"),
    (2.7,  0.4,  0.3, "侧偏+角偏"),
    (2.7, -0.4, -0.3, "侧偏-角偏"),
    (2.8,  0.0,  0.75, "大偏航 43°"),
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

print(HDR.format("Case", "Mode", "Expanded", "Time(ms)", "Acts", "OK"))
print(SEP)

for x0, y0, th0, label in cases:
    print(f"  {label}  ({x0}, {y0:.1f}, {th0:.2f})")
    for use_rs, no_corr, mode_label in combos:
        st = {}
        # 捕获 stderr（RS_EXPAND 调试日志）
        buf = io.StringIO(); old = sys.stderr; sys.stderr = buf
        ok, acts, _rs_traj = main.plan_path_robust(
            x0, y0, th0, prims,
            use_rs=use_rs,
            no_corridor=no_corr,
            stats=st
        )
        sys.stderr = old
        steps = str(len(acts)) if ok and acts else ("RS" if _rs_traj else "N/A")
        mode_tag = mode_label
        if st.get('two_stage'):
            mode_tag += "(2S)"
        elif st.get('out_of_range'):
            mode_tag += "(OOR)"
        print(ROW.format(
            "",
            mode_tag,
            st.get('expanded', 0),
            f"{st.get('elapsed_ms', 0):.1f}",
            steps,
            "YES" if ok else "NO"
        ))
    print()
