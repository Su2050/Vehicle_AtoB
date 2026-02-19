"""
路径质量自动验收测试脚本

验收标准（AC）：
  AC-1  rs_sample_path 终点位置误差  ≤ 0.05 m
  AC-2  rs_sample_path 终点角度误差  ≤ 3°
  AC-3  plan_path 返回 True 时终点满足目标条件（100%）
  AC-4  8 个测试用例规划成功率  ≥ 6/8
  AC-5  平均规划时间  ≤ 500 ms

运行方式：
    python test_path_quality.py            # 正常运行
    python test_path_quality.py --verbose  # 显示 RS_SAMPLE / RS_EXPAND 详细日志
"""

import sys
import math
import io
import time
import contextlib
import argparse

# ── 解析命令行 ────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--verbose', action='store_true',
                    help='打印 RS_SAMPLE/RS_EXPAND 详细日志')
args = parser.parse_args()

import main
import rs as rs_mod

# ── 验收阈值 ──────────────────────────────────────────────────────────────────
AC1_POS_TOL  = 0.05   # m
AC2_ANG_TOL  = 3.0    # degrees
AC4_MIN_PASS = 6      # 8 个用例至少通过 6 个
AC5_TIME_TOL = 500.0  # ms（平均）

GOAL_X_MAX   = 2.25
GOAL_Y_HALF  = 0.18
GOAL_TH_TOL  = 5.0 * math.pi / 180.0   # = ALIGN_GOAL_DYAW

# ── 辅助 ──────────────────────────────────────────────────────────────────────
def angle_diff(a, b):
    d = a - b
    while d > math.pi:  d -= 2 * math.pi
    while d <= -math.pi: d += 2 * math.pi
    return d

def satisfies_goal(x, y, th):
    return (x <= GOAL_X_MAX
            and abs(y) <= GOAL_Y_HALF
            and abs(th) <= GOAL_TH_TOL)

def capture_stderr(fn):
    """执行 fn()，捕获 stderr 输出并返回 (result, stderr_text)"""
    buf = io.StringIO()
    old = sys.stderr
    sys.stderr = buf
    try:
        result = fn()
    finally:
        sys.stderr = old
    return result, buf.getvalue()

# ── AC-1 / AC-2：rs_sample_path 直接单元测试 ─────────────────────────────────
SAMPLE_CASES = [
    # (x1, y1, th1,  x2, y2, th2,  label)
    (2.8,  0.0,  0.0,  2.10, 0.0, 0.0,   '正前方直线'),
    (2.5,  0.5,  0.3,  2.10, 0.0, 0.0,   '侧偏+偏角'),
    (2.6, -0.8,  0.3,  2.10, 0.0, 0.0,   '负侧偏+偏角'),
    (2.8, -0.8, -0.6,  2.10, 0.0, 0.0,   '远角'),
    (2.4,  0.1, -0.2,  2.10, 0.0, 0.0,   '近距小偏'),
]

print('=' * 70)
print('AC-1 / AC-2  rs_sample_path 终点精度测试')
print('=' * 70)

ac1_results = []
ac2_results = []

for x1, y1, th1, x2, y2, th2, label in SAMPLE_CASES:
    def _run():
        return rs_mod.rs_sample_path(
            x1, y1, th1, x2, y2, th2,
            main.MIN_TURN_RADIUS, step=0.02,
            verbose=args.verbose
        )
    traj, log_txt = capture_stderr(_run)

    if traj is None:
        pos_err = float('inf')
        ang_err = float('inf')
        status = 'FAIL(no path)'
    else:
        ep = traj[-1]
        pos_err = math.hypot(ep[0] - x2, ep[1] - y2)
        ang_err = abs(math.degrees(angle_diff(ep[2], th2)))
        ok1 = pos_err <= AC1_POS_TOL
        ok2 = ang_err <= AC2_ANG_TOL
        status = 'PASS' if (ok1 and ok2) else 'FAIL'
        if not ok1:
            status += '(pos_err={:.4f}m)'.format(pos_err)
        if not ok2:
            status += '(ang_err={:.2f}deg)'.format(ang_err)

    ac1_results.append(pos_err <= AC1_POS_TOL)
    ac2_results.append(ang_err <= AC2_ANG_TOL)
    print('  [{:4s}] {:12s}  pos_err={:.4f}m  ang_err={:.2f}deg'.format(
        status[:4], label, pos_err, ang_err))
    if args.verbose and log_txt.strip():
        for line in log_txt.strip().split('\n'):
            print('         ' + line)

ac1_pass = sum(ac1_results)
ac2_pass = sum(ac2_results)
print()
print('AC-1 位置误差 ≤{:.2f}m : {}/{} 通过'.format(
    AC1_POS_TOL, ac1_pass, len(SAMPLE_CASES)))
print('AC-2 角度误差 ≤{:.0f}° : {}/{} 通过'.format(
    AC2_ANG_TOL, ac2_pass, len(SAMPLE_CASES)))

# ── AC-3 / AC-4 / AC-5：plan_path 集成测试 ────────────────────────────────────
PLAN_CASES = [
    # (x0, y0, th0, use_rs, no_corridor, label)
    # 4 个默认起点（RS 展开瞬间完成）
    (2.8,  0.0,  0.0,   False, False, 'Geo+Corr  默认起点'),
    (2.8,  0.0,  0.0,   True,  False, 'RS +Corr  默认起点'),
    (2.8,  0.0,  0.0,   False, True,  'Geo+NoCorr默认起点'),
    (2.8,  0.0,  0.0,   True,  True,  'RS +NoCorr默认起点'),
    # 4 个轻度侧偏起点（A* 在 <500ms 内求解）
    (2.7,  0.3,  0.3,   False, True,  'Geo+NoCorr轻侧偏+'),
    (2.7,  0.3,  0.3,   True,  True,  'RS +NoCorr轻侧偏+'),
    (2.7, -0.3, -0.3,   False, True,  'Geo+NoCorr轻侧偏-'),
    (2.7, -0.3, -0.3,   True,  True,  'RS +NoCorr轻侧偏-'),
]

print()
print('=' * 70)
print('AC-3 / AC-4 / AC-5  plan_path 集成测试')
print('=' * 70)

prims = main.init_primitives()

ac3_fail_cases  = []
elapsed_list    = []
plan_pass_count = 0

for x0, y0, th0, use_rs, no_corridor, label in PLAN_CASES:
    st = {}

    def _plan():
        return main.plan_path(x0, y0, th0, prims,
                              use_rs=use_rs,
                              no_corridor=no_corridor,
                              stats=st)

    (ok, acts, rs_traj), log_txt = capture_stderr(_plan)

    elapsed = st.get('elapsed_ms', 0)
    elapsed_list.append(elapsed)

    if not ok:
        result_tag = 'FAIL(no path)'
        plan_pass_count += 0
        end_pos = None
    else:
        plan_pass_count += 1
        # 重建完整轨迹终点
        # ddx/ddy 是相对 primitive 起点的【累计】位移，每个 primitive 只需取最后一步
        cx, cy, cth = x0, y0, th0
        pmap = {p[0]: p[2] for p in prims}
        for act in (acts or []):
            seg = pmap.get(act)
            if seg is None:
                continue
            cos_t, sin_t = math.cos(cth), math.sin(cth)
            # 取该 primitive 最后一个 DT 步的累计位移
            ddx, ddy, ddth = seg[-1][0], seg[-1][1], seg[-1][2]
            nx = cx + ddx * cos_t - ddy * sin_t
            ny = cy + ddx * sin_t + ddy * cos_t
            nth = cth + ddth
            if nth > math.pi:   nth -= 2 * math.pi
            elif nth <= -math.pi: nth += 2 * math.pi
            cx, cy, cth = nx, ny, nth
        if rs_traj:
            cx, cy, cth = rs_traj[-1]
        end_pos = (cx, cy, cth)

        goal_ok = satisfies_goal(cx, cy, cth)
        rs_tag = '[RSexp]' if st.get('rs_expansion') else '       '
        if goal_ok:
            result_tag = 'PASS {}'.format(rs_tag)
        else:
            result_tag = 'FAIL(AC3) {}'.format(rs_tag)
            ac3_fail_cases.append((label, cx, cy, cth))

    expand_info = ''
    if st.get('rs_expansion'):
        expand_info = ' rs_expand=YES'

    print('  [{:16s}] {:20s} ok={} exp={:5d} {:.0f}ms{}'.format(
        result_tag[:16], label, ok,
        st.get('expanded', 0), elapsed, expand_info))

    if args.verbose and log_txt.strip():
        for line in log_txt.strip().split('\n'):
            print('         ' + line)

    if end_pos and args.verbose:
        print('         终点=({:.3f},{:.3f},{:.2f}deg) goal_ok={}'.format(
            end_pos[0], end_pos[1], math.degrees(end_pos[2]),
            satisfies_goal(*end_pos)))

# ── 汇总 ──────────────────────────────────────────────────────────────────────
avg_time = sum(elapsed_list) / len(elapsed_list) if elapsed_list else 0

print()
print('=' * 70)
print('验收结果汇总')
print('=' * 70)

ac1_ok = (ac1_pass == len(SAMPLE_CASES))
ac2_ok = (ac2_pass == len(SAMPLE_CASES))
ac3_ok = (len(ac3_fail_cases) == 0)
ac4_ok = (plan_pass_count >= AC4_MIN_PASS)
ac5_ok = (avg_time <= AC5_TIME_TOL)

def _tag(ok): return 'PASS' if ok else 'FAIL'

print('AC-1  rs_sample 位置误差 ≤{:.2f}m  : {} ({}/{})'.format(
    AC1_POS_TOL, _tag(ac1_ok), ac1_pass, len(SAMPLE_CASES)))
print('AC-2  rs_sample 角度误差 ≤{:.0f}°   : {} ({}/{})'.format(
    AC2_ANG_TOL, _tag(ac2_ok), ac2_pass, len(SAMPLE_CASES)))
print('AC-3  plan 返回True终点达标        : {} ({} 失败)'.format(
    _tag(ac3_ok), len(ac3_fail_cases)))
print('AC-4  规划成功率 ≥{}/{}            : {} ({}/{})'.format(
    AC4_MIN_PASS, len(PLAN_CASES), _tag(ac4_ok),
    plan_pass_count, len(PLAN_CASES)))
print('AC-5  平均规划时间 ≤{:.0f}ms         : {} ({:.1f}ms)'.format(
    AC5_TIME_TOL, _tag(ac5_ok), avg_time))

if ac3_fail_cases:
    print()
    print('AC-3 失败详情：')
    for lbl, ex, ey, eth in ac3_fail_cases:
        print('  {} → 终点=({:.3f},{:.3f},{:.2f}deg) '
              'x≤2.25={} |y|≤0.18={} |th|≤5°={}'.format(
                  lbl, ex, ey, math.degrees(eth),
                  ex <= GOAL_X_MAX,
                  abs(ey) <= GOAL_Y_HALF,
                  abs(eth) <= GOAL_TH_TOL))

all_pass = ac1_ok and ac2_ok and ac3_ok and ac4_ok and ac5_ok
print()
print('总体结果: {}'.format('ALL PASS ✓' if all_pass else 'SOME FAIL ✗'))
sys.exit(0 if all_pass else 1)
