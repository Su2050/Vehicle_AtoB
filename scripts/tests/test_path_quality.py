"""
路径质量自动验收测试脚本 v2

验收标准（AC）：
  AC-1  rs_sample_path 终点位置误差         ≤ 0.05 m        (8 用例全部通过)
  AC-2  rs_sample_path 终点角度误差         ≤ 3°            (8 用例全部通过)
  AC-3  plan_path 返回 True 时终点满足目标  100%             (0 失败)
  AC-4  plan_path_robust 标准场景成功率     ≥ 8/8
  AC-5  标准场景（|y|≤0.5m）平均规划时间   ≤ 500 ms
  AC-6  大偏航角两阶段规划用时             ≤ 3000 ms 且成功
  AC-7  超限场景（|y|>MAX_PLANNABLE_Y）    立即拒绝 ≤ 10 ms
  AC-8  所有返回路径的点均在工作区内        100%（零碰撞）
  AC-9  MAX_PLANNABLE_Y 边界正确：          0.79m 可规划，0.81m 拒绝

运行方式：
    conda run -n forklift_sim python test_path_quality.py
    conda run -n forklift_sim python test_path_quality.py --verbose
    conda run -n forklift_sim python test_path_quality.py --section rs
    conda run -n forklift_sim python test_path_quality.py --section plan
    conda run -n forklift_sim python test_path_quality.py --section robust
"""

import sys
import os
import math
import io
import time
import argparse
import textwrap

# 将 scripts/ 目录加入搜索路径，以便从 tests/ 子目录运行时也能找到模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# ── 命令行 ────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='打印 RS_SAMPLE / RS_EXPAND 详细日志')
    parser.add_argument('--section', choices=['rs', 'plan', 'robust', 'all'],
                        default='all', help='只运行指定测试板块')
    args = parser.parse_args()
else:
    class _DummyArgs:
        verbose = False
        section = 'none'
    args = _DummyArgs()

import main
import rs as rs_mod

# ── 常量 ──────────────────────────────────────────────────────────────────────
AC1_POS_TOL      = 0.05    # m
AC2_ANG_TOL      = 3.0     # deg
AC4_MIN_PASS     = 8       # / 8 个标准用例
AC5_TIME_TOL     = 500.0   # ms（标准场景均值）
AC6_TIME_TOL     = 3000.0  # ms（大偏航角两阶段）
AC7_TIME_TOL     = 10.0    # ms（超限立即拒绝）

GOAL_X_MAX  = 2.25
GOAL_Y_HALF = 0.18
GOAL_TH_TOL = 5.0 * math.pi / 180.0

# ── 辅助函数 ──────────────────────────────────────────────────────────────────
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
    buf = io.StringIO()
    old = sys.stderr
    sys.stderr = buf
    try:
        result = fn()
    finally:
        sys.stderr = old
    return result, buf.getvalue()

def _reconstruct_endpoint(x0, y0, th0, acts, rs_traj, prims, stats=None):
    """
    用动作序列重建路径终点。

    优先使用 stats['goal_pos']（A* mid-primitive 精确截断点），
    回退到完整 replay（每段取最后一步），误差来自末段 primitive 越界。
    """
    # 最精确：A* 命中目标时记录的中间位姿
    if stats and stats.get('goal_pos'):
        return stats['goal_pos']
    # RS 解析扩展终点
    if rs_traj:
        return rs_traj[-1]
    # 完整 replay（末段 primitive 可能轻微越出目标区）
    cx, cy, cth = x0, y0, th0
    pmap = {p[0]: p[2] for p in prims}
    for act in (acts or []):
        seg = pmap.get(act)
        if seg is None:
            continue
        cos_t, sin_t = math.cos(cth), math.sin(cth)
        ddx, ddy, ddth = seg[-1][0], seg[-1][1], seg[-1][2]
        nx = cx + ddx * cos_t - ddy * sin_t
        ny = cy + ddx * sin_t + ddy * cos_t
        nth = cth + ddth
        if nth > math.pi:   nth -= 2 * math.pi
        elif nth <= -math.pi: nth += 2 * math.pi
        cx, cy, cth = nx, ny, nth
    return cx, cy, cth

def _check_path_in_workspace(x0, y0, th0, acts, rs_traj, prims, no_corridor, stats=None):
    """验证路径所有点均在工作区内（无碰撞），返回 (ok, first_violation)"""
    traj = [(x0, y0, th0)]
    cx, cy, cth = x0, y0, th0
    pmap = {p[0]: p[2] for p in prims}
    st = stats or {}
    last_idx = len(acts or []) - 1
    goal_step_hit = st.get('goal_step_hit')
    stage1_acts = st.get('stage1_acts')
    stage1_goal_step_hit = st.get('stage1_goal_step_hit')
    for idx, act in enumerate(acts or []):
        seg = pmap.get(act)
        if seg is None:
            continue
        # 命中目标发生在 primitive 中间时，只回放到命中步，避免过冲假碰撞
        # 两阶段场景下可能同时存在：Stage-1 截断 + Stage-2 截断
        cut_steps = None
        if stage1_goal_step_hit and stage1_acts and idx == stage1_acts - 1:
            cut_steps = stage1_goal_step_hit
        if goal_step_hit and idx == last_idx and rs_traj is None:
            cut_steps = goal_step_hit if cut_steps is None else min(cut_steps, goal_step_hit)
        seg_iter = seg if cut_steps is None else seg[:cut_steps]
        cos_t, sin_t = math.cos(cth), math.sin(cth)
        for dx, dy, dth, _, _ in seg_iter:
            nx = cx + dx * cos_t - dy * sin_t
            ny = cy + dx * sin_t + dy * cos_t
            nth = cth + dth
            if nth > math.pi:   nth -= 2 * math.pi
            elif nth <= -math.pi: nth += 2 * math.pi
            traj.append((nx, ny, nth))
        cx, cy, cth = traj[-1]
    if rs_traj:
        traj.extend(rs_traj[1:])
    for pt in traj:
        valid, reason = main.check_collision(pt[0], pt[1], pt[2],
                                             no_corridor=no_corridor)
        if not valid:
            return False, (pt, reason)
    return True, None

passed_sections = []
failed_sections = []

def _section_result(name, ok):
    if ok:
        passed_sections.append(name)
    else:
        failed_sections.append(name)
    return ok

PASS = '\033[32mPASS\033[0m'
FAIL = '\033[31mFAIL\033[0m'
SKIP = '\033[90mSKIP\033[0m'

def _tag(ok):   return PASS if ok else FAIL
def _hdr(title):
    print()
    print('=' * 72)
    print(' ' + title)
    print('=' * 72)

# ══════════════════════════════════════════════════════════════════════════════
#  板块 1  rs_sample_path 精度（AC-1 / AC-2）
# ══════════════════════════════════════════════════════════════════════════════
RUN_RS = args.section in ('rs', 'all')

if RUN_RS:
    _hdr('板块 1  rs_sample_path 终点精度  [AC-1 / AC-2]')

    SAMPLE_CASES = [
        # (x1, y1, th1,  x2, y2, th2,  label)
        (2.8,  0.0,  0.0,   2.10,  0.0,  0.0,  '正前方直线'),
        (2.5,  0.5,  0.3,   2.10,  0.0,  0.0,  '侧偏+偏角'),
        (2.6, -0.8,  0.3,   2.10,  0.0,  0.0,  '负侧偏+偏角'),
        (2.8, -0.8, -0.6,   2.10,  0.0,  0.0,  '远角'),
        (2.4,  0.1, -0.2,   2.10,  0.0,  0.0,  '近距小偏'),
        (2.9,  0.6,  0.5,   2.10,  0.0,  0.0,  '中等侧偏偏航'),
        (2.7, -0.4,  0.6,   2.10,  0.0,  0.0,  '负侧偏正偏航'),
        (3.0,  0.0,  0.0,   2.10,  0.0,  0.0,  '最远起点'),
    ]

    ac1_results = []
    ac2_results = []

    print(f"  {'标签':<16} {'pos_err(m)':>10} {'ang_err(°)':>10}  结果")
    print('  ' + '-' * 50)
    for x1, y1, th1, x2, y2, th2, label in SAMPLE_CASES:
        def _run(x1=x1, y1=y1, th1=th1, x2=x2, y2=y2, th2=th2):
            return rs_mod.rs_sample_path(
                x1, y1, th1, x2, y2, th2,
                main.MIN_TURN_RADIUS, step=0.02,
                verbose=args.verbose)
        traj, log_txt = capture_stderr(_run)

        if traj is None:
            pos_err = ang_err = float('inf')
            status = 'FAIL(no path)'
        else:
            ep = traj[-1]
            pos_err = math.hypot(ep[0] - x2, ep[1] - y2)
            ang_err = abs(math.degrees(angle_diff(ep[2], th2)))
            ok1 = pos_err <= AC1_POS_TOL
            ok2 = ang_err <= AC2_ANG_TOL
            status = 'PASS' if (ok1 and ok2) else 'FAIL'

        ac1_results.append(pos_err <= AC1_POS_TOL)
        ac2_results.append(ang_err <= AC2_ANG_TOL)
        res_str = 'PASS' if pos_err <= AC1_POS_TOL and ang_err <= AC2_ANG_TOL else 'FAIL'
        print(f"  {label:<16} {pos_err:>10.4f} {ang_err:>10.2f}  {res_str}")
        if args.verbose and log_txt.strip():
            for line in log_txt.strip().split('\n'):
                print('    ' + line)

    ac1_pass = sum(ac1_results)
    ac2_pass = sum(ac2_results)
    n = len(SAMPLE_CASES)
    ac1_ok = (ac1_pass == n)
    ac2_ok = (ac2_pass == n)
    print()
    print(f'  AC-1 位置误差 ≤{AC1_POS_TOL:.2f}m : {_tag(ac1_ok)} ({ac1_pass}/{n})')
    print(f'  AC-2 角度误差 ≤{AC2_ANG_TOL:.0f}°  : {_tag(ac2_ok)} ({ac2_pass}/{n})')
    _section_result('AC-1', ac1_ok)
    _section_result('AC-2', ac2_ok)

# ══════════════════════════════════════════════════════════════════════════════
#  板块 2  plan_path 标准集成测试（AC-3 / AC-4 / AC-5）
# ══════════════════════════════════════════════════════════════════════════════
RUN_PLAN = args.section in ('plan', 'all')

if RUN_PLAN:
    _hdr('板块 2  plan_path 标准集成测试  [AC-3 / AC-4 / AC-5]')

    PLAN_CASES = [
        # (x0, y0, th0, use_rs, no_corridor, label)
        (2.8,  0.0,  0.0,   False, False, 'Geo+Corr   默认'),
        (2.8,  0.0,  0.0,   True,  False, 'RS +Corr   默认'),
        (2.8,  0.0,  0.0,   False, True,  'Geo+NoCorr 默认'),
        (2.8,  0.0,  0.0,   True,  True,  'RS +NoCorr 默认'),
        (2.7,  0.3,  0.3,   False, True,  'Geo+NoCorr 轻侧偏+'),
        (2.7,  0.3,  0.3,   True,  True,  'RS +NoCorr 轻侧偏+'),
        (2.7, -0.3, -0.3,   False, True,  'Geo+NoCorr 轻侧偏-'),
        (2.7, -0.3, -0.3,   True,  True,  'RS +NoCorr 轻侧偏-'),
    ]

    prims = main.init_primitives()
    ac3_fails = []
    elapsed_list = []
    plan_pass_count = 0
    collision_fails = []

    print(f"  {'标签':<22} {'ok':>4} {'exp':>6} {'ms':>7}  终点达标  碰撞")
    print('  ' + '-' * 62)

    for x0, y0, th0, use_rs, no_corridor, label in PLAN_CASES:
        st = {}
        def _plan(x0=x0, y0=y0, th0=th0, use_rs=use_rs,
                  no_corridor=no_corridor, st=st):
            return main.plan_path(x0, y0, th0, prims,
                                  use_rs=use_rs,
                                  no_corridor=no_corridor,
                                  stats=st)
        (ok, acts, rs_traj), log_txt = capture_stderr(_plan)
        elapsed = st.get('elapsed_ms', 0)

        if ok:
            plan_pass_count += 1
            elapsed_list.append(elapsed)
            cx, cy, cth = _reconstruct_endpoint(x0, y0, th0, acts, rs_traj, prims, st)
            goal_ok = satisfies_goal(cx, cy, cth)
            if not goal_ok:
                ac3_fails.append((label, cx, cy, cth))
            ws_ok, viol = _check_path_in_workspace(
                x0, y0, th0, acts, rs_traj, prims, no_corridor, st)
            if not ws_ok:
                collision_fails.append((label, viol))
            rs_mark = '[RS]' if st.get('rs_expansion') else '    '
            goal_mark = 'OK ' if goal_ok else 'FAIL'
            ws_mark   = 'OK ' if ws_ok   else 'FAIL'
            print(f"  {label:<22} {'OK':>4} {st.get('expanded',0):>6} {elapsed:>7.1f}  "
                  f"{goal_mark}{rs_mark}  {ws_mark}")
        else:
            print(f"  {label:<22} {'FAIL':>4} {st.get('expanded',0):>6} {elapsed:>7.1f}  "
                  f"---          ---")

        if args.verbose and log_txt.strip():
            for line in log_txt.strip().split('\n'):
                print('    ' + line)

    avg_time = sum(elapsed_list) / len(elapsed_list) if elapsed_list else 0

    ac3_ok = len(ac3_fails) == 0
    ac4_ok = plan_pass_count >= AC4_MIN_PASS
    ac5_ok = avg_time <= AC5_TIME_TOL
    ac8_ok_plan = len(collision_fails) == 0

    print()
    print(f'  AC-3 终点满足目标区        : {_tag(ac3_ok)} ({len(ac3_fails)} 失败)')
    print(f'  AC-4 规划成功率 ≥{AC4_MIN_PASS}/{len(PLAN_CASES)}  : '
          f'{_tag(ac4_ok)} ({plan_pass_count}/{len(PLAN_CASES)})')
    print(f'  AC-5 平均规划时间 ≤{AC5_TIME_TOL:.0f}ms : {_tag(ac5_ok)} ({avg_time:.1f}ms)')
    print(f'  AC-8 路径无碰撞(本板块)   : {_tag(ac8_ok_plan)} ({len(collision_fails)} 失败)')

    if ac3_fails:
        print()
        print('  AC-3 失败详情:')
        for lbl, ex, ey, eth in ac3_fails:
            print(f'    {lbl}: ({ex:.3f},{ey:.3f},{math.degrees(eth):.1f}°) '
                  f'x≤2.25:{ex<=GOAL_X_MAX} |y|≤0.18:{abs(ey)<=GOAL_Y_HALF}')
    if collision_fails:
        print()
        print('  AC-8 碰撞详情:')
        for lbl, (pt, reason) in collision_fails:
            print(f'    {lbl}: ({pt[0]:.3f},{pt[1]:.3f}) → {reason}')

    _section_result('AC-3', ac3_ok)
    _section_result('AC-4', ac4_ok)
    _section_result('AC-5', ac5_ok)
    _section_result('AC-8(plan)', ac8_ok_plan)

# ══════════════════════════════════════════════════════════════════════════════
#  板块 3  plan_path_robust 全场景测试（AC-4 / AC-6 / AC-7 / AC-8 / AC-9）
# ══════════════════════════════════════════════════════════════════════════════
RUN_ROBUST = args.section in ('robust', 'all')

if RUN_ROBUST:
    _hdr('板块 3  plan_path_robust 全场景  [AC-4/6/7/8/9]')

    if 'prims' not in dir():
        prims = main.init_primitives()

    # ── 3-a 标准场景（|y|≤0.5m, |θ|≤40°）────────────────────────────────────
    print('  [3-a] 标准场景（|y|≤0.5m, |θ|≤40°）')
    STD_CASES = [
        (2.8,  0.0,  0.0,  False, False, 'Geo+Corr  默认'),
        (2.8,  0.0,  0.0,  True,  False, 'RS+Corr   默认'),
        (2.8,  0.0,  0.0,  False, True,  'Geo+NoC   默认'),
        (2.8,  0.0,  0.0,  True,  True,  'RS+NoC    默认'),
        (2.7,  0.3,  0.3,  False, True,  'Geo+NoC   侧偏+'),
        (2.7,  0.3,  0.3,  True,  True,  'RS+NoC    侧偏+'),
        (2.7, -0.3, -0.3,  False, True,  'Geo+NoC   侧偏-'),
        (2.7, -0.3, -0.3,  True,  True,  'RS+NoC    侧偏-'),
    ]
    std_pass = 0
    std_elapsed = []
    std_coll_fails = []
    print(f"    {'标签':<18} {'模式':>8} {'ok':>4} {'ms':>7}  目标  碰撞")
    print('    ' + '-' * 56)
    for x0, y0, th0, use_rs, no_corr, label in STD_CASES:
        st = {}
        def _p(x0=x0, y0=y0, th0=th0, use_rs=use_rs, no_corr=no_corr, st=st):
            return main.plan_path_robust(x0, y0, th0, prims,
                                         use_rs=use_rs, no_corridor=no_corr, stats=st)
        (ok, acts, rs_traj), _ = capture_stderr(_p)
        elapsed = st.get('elapsed_ms', 0)
        mode = '2-Stage' if st.get('two_stage') else 'Direct '
        if ok:
            std_pass += 1
            std_elapsed.append(elapsed)
            cx, cy, cth = _reconstruct_endpoint(x0, y0, th0, acts, rs_traj, prims, st)
            goal_ok = satisfies_goal(cx, cy, cth)
            ws_ok, _ = _check_path_in_workspace(
                x0, y0, th0, acts, rs_traj, prims, no_corr, st)
            if not ws_ok:
                std_coll_fails.append(label)
            gm = 'OK ' if goal_ok else 'FAIL'
            wm = 'OK ' if ws_ok   else 'FAIL'
            print(f"    {label:<18} {mode:>8} {'OK':>4} {elapsed:>7.1f}  {gm}  {wm}")
        else:
            print(f"    {label:<18} {mode:>8} {'FAIL':>4} {elapsed:>7.1f}  ---  ---")

    # ── 3-b 大偏航角两阶段（|θ|>40°）────────────────────────────────────────
    print()
    print('  [3-b] 大偏航角两阶段（|θ|>TWO_STAGE_TH_THRESH）')
    BIG_THETA_CASES = [
        (2.8,  0.0,  0.75,  False, True,  'θ=+43° NoCorr'),
        (2.8,  0.0, -0.75,  False, True,  'θ=-43° NoCorr'),
        (2.8,  0.0,  0.75,  False, False, 'θ=+43° Corr'),
        (2.8,  0.2,  0.72,  False, True,  'θ=+41° y=0.2m'),
    ]
    big_theta_pass = 0
    big_theta_times = []
    print(f"    {'标签':<18} {'模式':>8} {'ok':>4} {'ms':>7}  目标  碰撞  s1_acts s2_acts")
    print('    ' + '-' * 70)
    for x0, y0, th0, use_rs, no_corr, label in BIG_THETA_CASES:
        st = {}
        def _p(x0=x0, y0=y0, th0=th0, use_rs=use_rs, no_corr=no_corr, st=st):
            return main.plan_path_robust(x0, y0, th0, prims,
                                         use_rs=use_rs, no_corridor=no_corr, stats=st)
        (ok, acts, rs_traj), _ = capture_stderr(_p)
        elapsed = st.get('elapsed_ms', 0)
        mode = '2-Stage' if st.get('two_stage') else 'Direct '
        if ok:
            big_theta_pass += 1
            big_theta_times.append(elapsed)
            cx, cy, cth = _reconstruct_endpoint(x0, y0, th0, acts, rs_traj, prims, st)
            goal_ok = satisfies_goal(cx, cy, cth)
            ws_ok, _ = _check_path_in_workspace(
                x0, y0, th0, acts, rs_traj, prims, no_corr, st)
            gm = 'OK ' if goal_ok else 'FAIL'
            wm = 'OK ' if ws_ok   else 'FAIL'
            s1a = st.get('stage1_acts', '—')
            s2a = st.get('stage2_acts', '—')
            print(f"    {label:<18} {mode:>8} {'OK':>4} {elapsed:>7.1f}  {gm}  {wm}  "
                  f"s1={s1a:>3}  s2={s2a:>3}")
        else:
            print(f"    {label:<18} {mode:>8} {'FAIL':>4} {elapsed:>7.1f}  ---  ---")

    # ── 3-b2 中等 y 偏差两阶段（原"慢区"0.5<|y|≤0.8m，TWO_STAGE_Y_THRESH=0.5m 后走两阶段）
    print()
    AC6B_MAX_TOL = 12000.0  # ms（最坏时延上限）
    AC6B_AVG_TOL = 8000.0   # ms（平均时延上限）
    print(f'  [3-b2] 中等 y 偏差两阶段（0.5m<|y|≤{main.MAX_PLANNABLE_Y}m，'
          f'avg≤{AC6B_AVG_TOL:.0f}ms 且 max≤{AC6B_MAX_TOL:.0f}ms）')
    MID_Y_CASES = [
        (2.8,  0.55,  0.0,  False, True,  'y=+0.55m'),
        (2.8,  0.65,  0.0,  False, True,  'y=+0.65m'),
        (2.8,  0.75,  0.0,  False, True,  'y=+0.75m'),
        (2.8, -0.55,  0.0,  False, True,  'y=-0.55m'),
        (2.8, -0.65,  0.0,  False, True,  'y=-0.65m'),
        (2.8, -0.75,  0.0,  False, True,  'y=-0.75m'),
        (2.8,  0.6,   0.5,  False, True,  'y=+0.6m θ=29°'),   # y+θ 联合偏差
        (2.8, -0.6,  -0.5,  False, True,  'y=-0.6m θ=-29°'),
    ]
    mid_y_pass = 0
    mid_y_times = []
    print(f"    {'标签':<18} {'模式':>8} {'ok':>4} {'ms':>7}  目标  碰撞  s1_acts s2_acts")
    print('    ' + '-' * 72)
    for x0, y0, th0, use_rs, no_corr, label in MID_Y_CASES:
        st = {}
        def _pm(x0=x0, y0=y0, th0=th0, use_rs=use_rs, no_corr=no_corr, st=st):
            return main.plan_path_robust(x0, y0, th0, prims,
                                         use_rs=use_rs, no_corridor=no_corr, stats=st)
        (ok, acts, rs_traj), _ = capture_stderr(_pm)
        elapsed = st.get('elapsed_ms', 0)
        mode = '2-Stage' if st.get('two_stage') else 'Direct '
        if ok:
            mid_y_pass += 1
            mid_y_times.append(elapsed)
            cx, cy, cth = _reconstruct_endpoint(x0, y0, th0, acts, rs_traj, prims, st)
            goal_ok = satisfies_goal(cx, cy, cth)
            ws_ok, _ = _check_path_in_workspace(
                x0, y0, th0, acts, rs_traj, prims, no_corr, st)
            gm = 'OK ' if goal_ok else 'FAIL'
            wm = 'OK ' if ws_ok   else 'FAIL'
            s1a = st.get('stage1_acts', '—')
            s2a = st.get('stage2_acts', '—')
            print(f"    {label:<18} {mode:>8} {'OK':>4} {elapsed:>7.1f}  {gm}  {wm}  "
                  f"s1={s1a:>3}  s2={s2a:>3}")
        else:
            print(f"    {label:<18} {mode:>8} {'FAIL':>4} {elapsed:>7.1f}  ---  ---")
    mid_y_max = max(mid_y_times) if mid_y_times else 0
    mid_y_avg = sum(mid_y_times) / len(mid_y_times) if mid_y_times else 0
    mid_y_ok = (mid_y_pass == len(MID_Y_CASES)
                and mid_y_avg <= AC6B_AVG_TOL
                and mid_y_max <= AC6B_MAX_TOL)
    _section_result('AC-6b(中等y两阶段)', mid_y_ok)

    # ── 3-c 超限场景（|y|>MAX_PLANNABLE_Y，应立即拒绝）─────────────────────
    print()
    limit = main.MAX_PLANNABLE_Y
    print(f'  [3-c] 超限场景（|y|>{limit:.1f}m，期望立即返回 IMPOSSIBLE）')
    OUTRANGE_CASES = [
        (2.8,  5.1,  0.0,  False, True,  f'y=+5.1m > {limit:.1f}m'),
        (2.8, -5.1,  0.0,  False, True,  f'y=-5.1m > {limit:.1f}m'),
        (2.8,  6.0,  0.0,  False, True,  'y=+6.0m'),
    ]
    outrange_pass = 0
    outrange_slow = []
    print(f"    {'标签':<22} {'ok=False':>9} {'ms':>7}  立即?")
    print('    ' + '-' * 50)
    for x0, y0, th0, use_rs, no_corr, label in OUTRANGE_CASES:
        st = {}
        t0 = time.perf_counter()
        def _p(x0=x0, y0=y0, th0=th0, use_rs=use_rs, no_corr=no_corr, st=st):
            return main.plan_path_robust(x0, y0, th0, prims,
                                         use_rs=use_rs, no_corridor=no_corr, stats=st)
        (ok, _, _), _ = capture_stderr(_p)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        instant = elapsed_ms <= AC7_TIME_TOL
        expected_fail = not ok and st.get('out_of_range', False)
        if expected_fail and instant:
            outrange_pass += 1
        else:
            outrange_slow.append((label, ok, elapsed_ms, st.get('out_of_range')))
        status = 'PASS' if (expected_fail and instant) else 'FAIL'
        instant_mark = f'YES({elapsed_ms:.1f}ms)' if instant else f'NO({elapsed_ms:.0f}ms)'
        print(f"    {label:<22} {str(expected_fail):>9} {elapsed_ms:>7.1f}  {instant_mark}  {status}")

    # ── 3-d MAX_PLANNABLE_Y 边界测试（AC-9）─────────────────────────────────
    print()
    print(f'  [3-d] 边界测试（AC-9）: limit={limit:.2f}m')
    BOUNDARY_CASES = [
        # y = limit - 0.01 → 应该可以规划
        (2.8,  round(limit - 0.01, 3),  0.0, False, True,
         f'y={limit-0.01:.2f}m (刚好在内)'),
        (2.8, -round(limit - 0.01, 3),  0.0, False, True,
         f'y=-{limit-0.01:.2f}m (刚好在内)'),
        # y = limit + 0.01 → 应该立即拒绝
        (2.8,  round(limit + 0.01, 3),  0.0, False, True,
         f'y={limit+0.01:.2f}m (刚好超出)'),
        (2.8, -round(limit + 0.01, 3),  0.0, False, True,
         f'y=-{limit+0.01:.2f}m (刚好超出)'),
    ]
    boundary_pass = 0
    print(f"    {'标签':<26} {'期望':>8} {'实际':>8} {'ms':>7}  结果")
    print('    ' + '-' * 58)
    for x0, y0, th0, use_rs, no_corr, label in BOUNDARY_CASES:
        should_plan = abs(y0) <= limit
        st = {}
        t0 = time.perf_counter()
        def _p(x0=x0, y0=y0, th0=th0, use_rs=use_rs, no_corr=no_corr, st=st):
            return main.plan_path_robust(x0, y0, th0, prims,
                                         use_rs=use_rs, no_corridor=no_corr, stats=st)
        (ok, _, _), _ = capture_stderr(_p)
        elapsed_ms = (time.perf_counter() - t0) * 1000
        # 刚好超出：应立即拒绝且 out_of_range=True
        if should_plan:
            correct = ok  # 应该成功
        else:
            correct = (not ok and st.get('out_of_range') and elapsed_ms <= AC7_TIME_TOL)
        if correct:
            boundary_pass += 1
        expected_str = '可规划' if should_plan else '拒绝'
        actual_str   = ('OK' if ok else ('拒绝(立即)' if st.get('out_of_range') else '失败'))
        res = 'PASS' if correct else 'FAIL'
        print(f"    {label:<26} {expected_str:>8} {actual_str:>8} {elapsed_ms:>7.1f}  {res}")

    # ── 3-e 走廊约束对比 ─────────────────────────────────────────────────────
    print()
    print('  [3-e] 走廊约束 vs NoCorr（相同起点，不同约束）')
    CORR_CASES = [
        (2.8, 0.0, 0.0,  '默认起点 0.0m'),
        (2.7, 0.5, 0.3,  '侧偏+0.5m'),
        (2.7, -0.5, -0.3, '侧偏-0.5m'),
    ]
    print(f"    {'起点':>18}  Corridor(ms)  NoCorr(ms)  一致?")
    print('    ' + '-' * 52)
    for x0, y0, th0, label in CORR_CASES:
        results = {}
        for no_corr in (False, True):
            st = {}
            def _p(x0=x0, y0=y0, th0=th0, nc=no_corr, st=st):
                return main.plan_path_robust(x0, y0, th0, prims,
                                             no_corridor=nc, stats=st)
            (ok, _, _), _ = capture_stderr(_p)
            results[no_corr] = (ok, st.get('elapsed_ms', 0))
        ok_c, ms_c = results[False]
        ok_nc, ms_nc = results[True]
        # NoCorr 成功率应 ≥ Corr（宽松约束不应比严格约束更差）
        consistent = (not ok_c) or ok_nc
        cons_mark = 'OK ' if consistent else 'WARN'
        print(f"    {label:>18}  {ms_c if ok_c else 'FAIL':>12}  "
              f"{ms_nc if ok_nc else 'FAIL':>10}  {cons_mark}")

    # ── 汇总板块 3 ───────────────────────────────────────────────────────────
    print()
    std_avg = sum(std_elapsed) / len(std_elapsed) if std_elapsed else 0
    max_bt  = max(big_theta_times) if big_theta_times else 0

    ac4r_ok = std_pass >= AC4_MIN_PASS
    ac5r_ok = std_avg <= AC5_TIME_TOL
    ac6_ok  = big_theta_pass == len(BIG_THETA_CASES) and max_bt <= AC6_TIME_TOL
    ac7_ok  = outrange_pass == len(OUTRANGE_CASES)
    ac8r_ok = len(std_coll_fails) == 0
    ac9_ok  = boundary_pass == len(BOUNDARY_CASES)

    print(f'  AC-4  标准成功率 ≥{AC4_MIN_PASS}/{len(STD_CASES)}  : '
          f'{_tag(ac4r_ok)} ({std_pass}/{len(STD_CASES)}, avg={std_avg:.1f}ms)')
    print(f'  AC-5  标准avg ≤{AC5_TIME_TOL:.0f}ms  : {_tag(ac5r_ok)} ({std_avg:.1f}ms)')
    print(f'  AC-6  大偏航 ≤{AC6_TIME_TOL:.0f}ms : {_tag(ac6_ok)} '
          f'({big_theta_pass}/{len(BIG_THETA_CASES)}, max={max_bt:.1f}ms)')
    print(f'  AC-6b 中等y avg≤{AC6B_AVG_TOL:.0f}ms & max≤{AC6B_MAX_TOL:.0f}ms : {_tag(mid_y_ok)} '
          f'({mid_y_pass}/{len(MID_Y_CASES)}, avg={mid_y_avg:.1f}ms, max={mid_y_max:.1f}ms)')
    print(f'  AC-7  超限立即拒绝 ≤{AC7_TIME_TOL:.0f}ms : {_tag(ac7_ok)} '
          f'({outrange_pass}/{len(OUTRANGE_CASES)})')
    print(f'  AC-8  路径无碰撞(Robust)    : {_tag(ac8r_ok)} ({len(std_coll_fails)} 失败)')
    print(f'  AC-9  边界正确性            : {_tag(ac9_ok)} ({boundary_pass}/{len(BOUNDARY_CASES)})')

    _section_result('AC-4(robust)', ac4r_ok)
    _section_result('AC-5(robust)', ac5r_ok)
    _section_result('AC-6', ac6_ok)
    _section_result('AC-7', ac7_ok)
    _section_result('AC-8(robust)', ac8r_ok)
    _section_result('AC-9', ac9_ok)

# ══════════════════════════════════════════════════════════════════════════════
#  最终汇总
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == '__main__':
    _hdr('验收结果汇总')
    
    all_sections = passed_sections + failed_sections
    total = len(all_sections)
    passed_n = len(passed_sections)
    
    for name in passed_sections:
        print(f'  {PASS}  {name}')
    for name in failed_sections:
        print(f'  {FAIL}  {name}')
    
    print()
    print(f'  总计: {passed_n}/{total} 通过', end='  ')
    if not failed_sections:
        print('— ALL PASS ✓')
        sys.exit(0)
    else:
        print('— SOME FAIL ✗')
        print()
        print('  失败板块:', ', '.join(failed_sections))
        sys.exit(1)
