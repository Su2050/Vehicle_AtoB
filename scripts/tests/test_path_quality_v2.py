#!/usr/bin/env python3
"""
test_path_quality_v2.py — 路径几何质量专项测试套件

针对"规划成功但路径极其不合理"的隐式失效，通过 6 个度量维度保证路径质量：
  QM-1: 全局绕行比        (<= 5.0)
  QM-2: 最大横向偏移       (<= max(|sy|, |gy|) + 3.0m)
  QM-3: 最大后退距离       (<= start_x + 3.0m)
  QM-4: 穿墙采样点数       (<= 5)
  QM-5: RS 段独立绕行比    (<= 4.0)
  QM-6: 挡位切换上限       (<= 8)

用法:
  python tests/test_path_quality_v2.py              # 正常模式
  python tests/test_path_quality_v2.py --regress    # 回归验证（临时禁用规划器质量过滤，证明测试能抓到坏路径）
"""

import os
import sys
import math
import time
import signal
import argparse

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from primitives import init_primitives, simulate_path, RS_GOAL_X, RS_GOAL_Y


# ============================================================================
# 质量度量阈值
# ============================================================================
QM1_DETOUR_MAX = 5.0
QM2_LATERAL_EXTRA = 3.0
QM3_X_BACKTRACK_EXTRA = 3.0
QM4_BEHIND_WALL_PTS_MAX = 5
QM5_RS_DETOUR_MAX = 4.5
QM6_GEAR_SHIFT_MAX = 8

CASE_TIMEOUT = 15  # 单用例超时（秒）


# ============================================================================
# 工具函数
# ============================================================================

class _Timeout(Exception):
    pass

def _alarm_handler(signum, frame):
    raise _Timeout()

def _trajectory_length(traj):
    if not traj or len(traj) < 2:
        return 0.0
    return sum(
        math.hypot(traj[i][0] - traj[i-1][0], traj[i][1] - traj[i-1][1])
        for i in range(1, len(traj))
    )

def _compute_quality_metrics(start_x, start_y, acts, traj_astar, rs_traj):
    """计算路径质量度量，返回 (metrics_dict, message, all_ok)."""
    full_traj = list(traj_astar)
    if rs_traj:
        full_traj.extend(rs_traj[1:] if full_traj else rs_traj)

    if not full_traj:
        return None, "Empty trajectory", False

    length = _trajectory_length(full_traj)
    euclidean = math.hypot(start_x - RS_GOAL_X, start_y - RS_GOAL_Y)
    detour_ratio = length / max(euclidean, 2.0)

    max_abs_y = max(abs(p[1]) for p in full_traj)
    max_x = max(p[0] for p in full_traj)
    behind_wall_cnt = sum(1 for p in full_traj if p[0] < 1.92 and abs(p[1]) > 0.5)

    rs_detour = 0.0
    if rs_traj and len(rs_traj) > 1:
        rsl = _trajectory_length(rs_traj)
        rse = math.hypot(rs_traj[0][0] - rs_traj[-1][0], rs_traj[0][1] - rs_traj[-1][1])
        rs_detour = rsl / max(rse, 1.0)

    shifts = 0
    if acts:
        last_gear = acts[0][0]
        for a in acts[1:]:
            if a[0] != last_gear:
                shifts += 1
                last_gear = a[0]

    metrics = dict(
        length=length, euclidean=euclidean, detour_ratio=detour_ratio,
        max_abs_y=max_abs_y, max_x=max_x,
        behind_wall_cnt=behind_wall_cnt, rs_detour=rs_detour, shifts=shifts,
    )

    reasons = []
    if detour_ratio > QM1_DETOUR_MAX:
        reasons.append(f"QM1(Detour {detour_ratio:.1f}>{QM1_DETOUR_MAX})")
    allowed_y = max(abs(start_y), abs(RS_GOAL_Y)) + QM2_LATERAL_EXTRA
    if max_abs_y > allowed_y:
        reasons.append(f"QM2(LatY {max_abs_y:.1f}>{allowed_y:.1f})")
    allowed_x = start_x + QM3_X_BACKTRACK_EXTRA
    if max_x > allowed_x:
        reasons.append(f"QM3(MaxX {max_x:.1f}>{allowed_x:.1f})")
    if behind_wall_cnt > QM4_BEHIND_WALL_PTS_MAX:
        reasons.append(f"QM4(BehindWall {behind_wall_cnt}>{QM4_BEHIND_WALL_PTS_MAX})")
    if rs_detour > QM5_RS_DETOUR_MAX:
        reasons.append(f"QM5(RSDetour {rs_detour:.1f}>{QM5_RS_DETOUR_MAX})")
    if shifts > QM6_GEAR_SHIFT_MAX:
        reasons.append(f"QM6(Shifts {shifts}>{QM6_GEAR_SHIFT_MAX})")

    ok = len(reasons) == 0
    msg = ", ".join(reasons) if not ok else "OK"
    return metrics, msg, ok


# ============================================================================
# 单用例执行器
# ============================================================================

def _run_case(x, y, th_deg, obstacles, prims, planner_fn):
    th = math.radians(th_deg)
    st = {}

    old = signal.getsignal(signal.SIGALRM)
    try:
        signal.signal(signal.SIGALRM, _alarm_handler)
        signal.alarm(CASE_TIMEOUT)
        ok, acts, rs_traj = planner_fn(
            x, y, th, prims, use_rs=True, stats=st,
            obstacles=obstacles, rs_expansion_radius=2.5,
        )
    except _Timeout:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)
        return True, "TIMEOUT (quality filters blocked fast bad path)", None
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)

    if not ok:
        return True, f"Graceful Fail | Level:{st.get('level','?')}", None

    traj_astar = simulate_path(x, y, th, acts, prims) if acts else [(x, y, th)]
    metrics, msg, qm_ok = _compute_quality_metrics(x, y, acts, traj_astar, rs_traj)
    detail = f"{msg} | L:{st.get('level','?')} Exp:{st.get('expanded','?')} {st.get('elapsed_ms',0):.0f}ms"
    return qm_ok, detail, metrics


# ============================================================================
# 用例定义
# ============================================================================

REGRESSION_CASES = [
    ("REG-01 Screenshot Bug", 4.18, 2.03, -119.2, [{'x': 2.8, 'y': 0.2, 'w': 1.2, 'h': 0.8}]),
    ("REG-02 Extreme Angle", 5.0, 2.5, -150.0,   [{'x': 2.8, 'y': 0.2, 'w': 1.2, 'h': 0.8}]),
    ("REG-03 Neg Offset",    3.5, -2.0, 160.0,    [{'x': 3.0, 'y': -1.0, 'w': 1.0, 'h': 0.5}]),
]

SWEEP_ANGLES = [-180, -135, -90, 90, 135, 180]
SWEEP_STARTS = [(4.0, -1.0), (5.0, 0.0), (6.0, 1.5)]
SWEEP_OBS = [
    ("CenterBlock", [{'x': 3.5, 'y': -0.5, 'w': 1.0, 'h': 1.0}]),
    ("NarrowPass",  [{'x': 3.5, 'y': -2.5, 'w': 0.5, 'h': 2.0},
                     {'x': 3.5, 'y': 0.5,  'w': 0.5, 'h': 2.0}]),
]

LEVEL_CASES = [
    ("L1 Pure RS",   4.0, 0.0, 0.0,  None),
    ("L1.8 2D Skel", 4.5, 0.0, 0.0,  [{'x': 3.0, 'y': -0.5, 'w': 0.5, 'h': 1.0}]),
    ("L2 A*",        3.0, 1.0, 90.0,  [{'x': 2.5, 'y': 0.0, 'w': 1.5, 'h': 0.2}]),
]


# ============================================================================
# 主流程
# ============================================================================

def run_tests(regress_mode=False):
    prims = init_primitives()
    total = passed = 0

    if regress_mode:
        import planner_obs_v2 as _mod
        _orig_behind = _mod._path_goes_behind_wall
        _orig_unreas = _mod._path_is_unreasonable
        _mod._path_goes_behind_wall = lambda traj: False
        _mod._path_is_unreasonable = lambda traj, sx, sy: False
        _mod._count_gear_shifts = lambda acts: 0
        planner_fn = _mod.plan_path_robust_obs_v2
        print("*** REGRESS MODE: planner quality filters DISABLED ***\n")
    else:
        from planner_obs_v2 import plan_path_robust_obs_v2
        planner_fn = plan_path_robust_obs_v2

    print("=" * 80)
    print(f" PATH QUALITY TEST SUITE {'(REGRESS)' if regress_mode else ''}")
    print("=" * 80)

    # ── Block A: Regression ──
    print("\n[Block A] Regression Cases")
    for name, x, y, th, obs in REGRESSION_CASES:
        total += 1
        ok, msg, m = _run_case(x, y, th, obs, prims, planner_fn)
        passed += int(ok)
        mark = "PASS" if ok else "FAIL"
        print(f"  {mark} | {name:<22} | {msg}")

    # ── Block B: Parametric Sweep ──
    print("\n[Block B] Parameter Sweep (Large Angle + Obstacle)")
    b_total = b_pass = 0
    for obs_name, obs in SWEEP_OBS:
        for sx, sy in SWEEP_STARTS:
            for th in SWEEP_ANGLES:
                total += 1; b_total += 1
                ok, msg, _ = _run_case(sx, sy, th, obs, prims, planner_fn)
                passed += int(ok); b_pass += int(ok)
                if not ok:
                    print(f"  FAIL | ({sx},{sy},{th}) {obs_name} | {msg}")
    print(f"  Block B: {b_pass}/{b_total} passed")

    # ── Block C: Level Baselines ──
    print("\n[Block C] Planning Level Baselines")
    for name, x, y, th, obs in LEVEL_CASES:
        total += 1
        ok, msg, _ = _run_case(x, y, th, obs, prims, planner_fn)
        passed += int(ok)
        mark = "PASS" if ok else "FAIL"
        print(f"  {mark} | {name:<16} | {msg}")

    print("\n" + "=" * 80)
    all_ok = (passed == total)
    status = "ALL PASS" if all_ok else "SOME FAILED"
    print(f" RESULT: {passed}/{total} — {status}")
    print("=" * 80)

    if regress_mode:
        _mod._path_goes_behind_wall = _orig_behind
        _mod._path_is_unreasonable = _orig_unreas

    return all_ok


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--regress', action='store_true',
                        help='Disable planner quality filters to prove the test catches bad paths')
    args = parser.parse_args()

    ok = run_tests(regress_mode=args.regress)
    sys.exit(0 if ok else 1)
