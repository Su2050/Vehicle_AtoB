#!/usr/bin/env python3
"""
viz_quality_failures.py — 批量可视化 test_path_quality_v2 中 7 个失败 case
生成 matplotlib 静态图，保存到 docs/quality_failures/ 目录
"""

import os
import sys
import math
import time
import signal
import matplotlib
matplotlib.use("Agg")  # 无需 GUI
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from primitives import init_primitives, simulate_path, RS_GOAL_X, RS_GOAL_Y

# ── 质量阈值（与 test_path_quality_v2.py 完全一致）──
QM1_DETOUR_MAX = 5.0
QM2_LATERAL_EXTRA = 3.0
QM3_X_BACKTRACK_EXTRA = 3.0
QM4_BEHIND_WALL_PTS_MAX = 5
QM5_RS_DETOUR_MAX = 4.0
QM6_GEAR_SHIFT_MAX = 8

CASE_TIMEOUT = 20  # 单用例超时

WALL_X_MIN, WALL_X_MAX = 1.92, 3.0
WALL_Y_MIN, WALL_Y_MAX = -3.0, 3.0

# ── 7 个失败 case 定义 ──
FAILED_CASES = [
    {
        "name": "REG-02 Extreme Angle",
        "x": 5.0, "y": 2.5, "th_deg": -150.0,
        "obstacles": [{"x": 2.8, "y": 0.2, "w": 1.2, "h": 0.8}],
    },
    {
        "name": "B-(4,-1,90) CenterBlock",
        "x": 4.0, "y": -1.0, "th_deg": 90.0,
        "obstacles": [{"x": 3.5, "y": -0.5, "w": 1.0, "h": 1.0}],
    },
    {
        "name": "B-(5,0,-90) CenterBlock",
        "x": 5.0, "y": 0.0, "th_deg": -90.0,
        "obstacles": [{"x": 3.5, "y": -0.5, "w": 1.0, "h": 1.0}],
    },
    {
        "name": "B-(5,0,-180) NarrowPass",
        "x": 5.0, "y": 0.0, "th_deg": -180.0,
        "obstacles": [
            {"x": 3.5, "y": -2.5, "w": 0.5, "h": 2.0},
            {"x": 3.5, "y": 0.5, "w": 0.5, "h": 2.0},
        ],
    },
    {
        "name": "B-(5,0,-135) NarrowPass",
        "x": 5.0, "y": 0.0, "th_deg": -135.0,
        "obstacles": [
            {"x": 3.5, "y": -2.5, "w": 0.5, "h": 2.0},
            {"x": 3.5, "y": 0.5, "w": 0.5, "h": 2.0},
        ],
    },
    {
        "name": "B-(5,0,180) NarrowPass",
        "x": 5.0, "y": 0.0, "th_deg": 180.0,
        "obstacles": [
            {"x": 3.5, "y": -2.5, "w": 0.5, "h": 2.0},
            {"x": 3.5, "y": 0.5, "w": 0.5, "h": 2.0},
        ],
    },
    {
        "name": "L1.8 2D Skel baseline",
        "x": 4.5, "y": 0.0, "th_deg": 0.0,
        "obstacles": [{"x": 3.0, "y": -0.5, "w": 0.5, "h": 1.0}],
    },
]


# ── 质量计算 ──
def _traj_length(traj):
    if not traj or len(traj) < 2:
        return 0.0
    return sum(
        math.hypot(traj[i][0] - traj[i - 1][0], traj[i][1] - traj[i - 1][1])
        for i in range(1, len(traj))
    )


def compute_metrics(sx, sy, acts, traj_astar, rs_traj):
    full_traj = list(traj_astar)
    if rs_traj:
        full_traj.extend(rs_traj[1:] if full_traj else rs_traj)
    if not full_traj:
        return None, [], full_traj

    length = _traj_length(full_traj)
    euclidean = math.hypot(sx - RS_GOAL_X, sy - RS_GOAL_Y)
    detour_ratio = length / max(euclidean, 2.0)
    max_abs_y = max(abs(p[1]) for p in full_traj)
    max_x = max(p[0] for p in full_traj)
    behind_wall_cnt = sum(1 for p in full_traj if p[0] < 1.92 and abs(p[1]) > 0.5)

    rs_detour = 0.0
    if rs_traj and len(rs_traj) > 1:
        rsl = _traj_length(rs_traj)
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
    allowed_y = max(abs(sy), abs(RS_GOAL_Y)) + QM2_LATERAL_EXTRA
    allowed_x = sx + QM3_X_BACKTRACK_EXTRA
    if detour_ratio > QM1_DETOUR_MAX:
        reasons.append(f"QM1 Detour {detour_ratio:.2f} > {QM1_DETOUR_MAX}")
    if max_abs_y > allowed_y:
        reasons.append(f"QM2 LatY {max_abs_y:.2f} > {allowed_y:.1f}")
    if max_x > allowed_x:
        reasons.append(f"QM3 MaxX {max_x:.2f} > {allowed_x:.1f}")
    if behind_wall_cnt > QM4_BEHIND_WALL_PTS_MAX:
        reasons.append(f"QM4 BehindWall {behind_wall_cnt} > {QM4_BEHIND_WALL_PTS_MAX}")
    if rs_detour > QM5_RS_DETOUR_MAX:
        reasons.append(f"QM5 RSDetour {rs_detour:.2f} > {QM5_RS_DETOUR_MAX}")
    if shifts > QM6_GEAR_SHIFT_MAX:
        reasons.append(f"QM6 Shifts {shifts} > {QM6_GEAR_SHIFT_MAX}")

    return metrics, reasons, full_traj


# ── 超时处理 ──
class _Timeout(Exception):
    pass

def _alarm_handler(signum, frame):
    raise _Timeout()


# ── 可视化单个 case ──
def plot_case(idx, case, prims, planner_fn, out_dir):
    name = case["name"]
    sx, sy, th_deg = case["x"], case["y"], case["th_deg"]
    obs = case["obstacles"]
    th = math.radians(th_deg)
    st = {}

    print(f"  [{idx+1}/7] {name} ... ", end="", flush=True)

    # 运行规划
    old = signal.getsignal(signal.SIGALRM)
    timed_out = False
    try:
        signal.signal(signal.SIGALRM, _alarm_handler)
        signal.alarm(CASE_TIMEOUT)
        ok, acts, rs_traj = planner_fn(
            sx, sy, th, prims, use_rs=True, stats=st,
            obstacles=obs, rs_expansion_radius=2.5,
        )
    except _Timeout:
        ok, acts, rs_traj = False, [], None
        timed_out = True
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old)

    if timed_out:
        print("TIMEOUT")
        _plot_empty(idx, case, obs, out_dir, "TIMEOUT")
        return
    if not ok:
        print(f"Plan FAILED (level={st.get('level','?')})")
        _plot_empty(idx, case, obs, out_dir, f"PLAN FAILED\nLevel: {st.get('level','?')}")
        return

    traj_astar = simulate_path(sx, sy, th, acts, prims) if acts else [(sx, sy, th)]
    metrics, reasons, full_traj = compute_metrics(sx, sy, acts, traj_astar, rs_traj)
    level = st.get("level", "?")
    elapsed = st.get("elapsed_ms", 0)
    print(f"OK  Level={level}  {elapsed:.0f}ms  violations={len(reasons)}")

    # ── 绘图 ──
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    # 墙壁
    wall_left = mpatches.Rectangle(
        (WALL_Y_MIN, WALL_X_MIN), WALL_Y_MAX - WALL_Y_MIN, WALL_X_MAX - WALL_X_MIN,
        linewidth=1.5, edgecolor="#333", facecolor="#ddd", alpha=0.4, label="Wall zone"
    )
    ax.add_patch(wall_left)

    # 障碍物
    for i_obs, ob in enumerate(obs):
        rect = mpatches.Rectangle(
            (ob["y"], ob["x"]), ob["h"], ob["w"],
            linewidth=2, edgecolor="red", facecolor="#ff000030",
            label="Obstacle" if i_obs == 0 else None,
        )
        ax.add_patch(rect)

    # 完整轨迹
    xs_full = [p[0] for p in full_traj]
    ys_full = [p[1] for p in full_traj]
    ax.plot(ys_full, xs_full, "-", color="#2080DD", linewidth=1.8, alpha=0.85, label="Full trajectory")

    # A* 段
    if traj_astar and len(traj_astar) > 1:
        xs_a = [p[0] for p in traj_astar]
        ys_a = [p[1] for p in traj_astar]
        ax.plot(ys_a, xs_a, ".", color="#2080DD", markersize=2, alpha=0.5)

    # RS 段
    if rs_traj and len(rs_traj) > 1:
        xs_r = [p[0] for p in rs_traj]
        ys_r = [p[1] for p in rs_traj]
        ax.plot(ys_r, xs_r, "-", color="#CC44AA", linewidth=2.2, alpha=0.8, label="RS segment")

    # 起点箭头
    arrow_len = 0.3
    dx_arrow = -arrow_len * math.cos(th)  # -x forward
    dy_arrow = -arrow_len * math.sin(th)
    ax.annotate("", xy=(sy + dy_arrow, sx + dx_arrow), xytext=(sy, sx),
                arrowprops=dict(arrowstyle="->", color="green", lw=2.5))
    ax.plot(sy, sx, "o", color="green", markersize=10, zorder=10, label=f"Start ({sx},{sy},{th_deg}°)")

    # 终点
    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold", markersize=12, markeredgecolor="orange",
            markeredgewidth=2, zorder=10, label=f"Goal ({RS_GOAL_X},{RS_GOAL_Y})")

    # QM-2 横向偏移限制线
    allowed_y = max(abs(sy), abs(RS_GOAL_Y)) + QM2_LATERAL_EXTRA
    ax.axvline(x=allowed_y, color="orange", linestyle="--", alpha=0.6, label=f"QM2 limit ±{allowed_y:.1f}m")
    ax.axvline(x=-allowed_y, color="orange", linestyle="--", alpha=0.6)

    # QM-3 后退距离限制线
    allowed_x = sx + QM3_X_BACKTRACK_EXTRA
    ax.axhline(y=allowed_x, color="purple", linestyle="--", alpha=0.6, label=f"QM3 limit X={allowed_x:.1f}m")

    # 设置范围
    all_ys = ys_full + [sy, RS_GOAL_Y, allowed_y, -allowed_y]
    all_xs = xs_full + [sx, RS_GOAL_X, allowed_x]
    y_margin = 0.8
    x_margin = 0.8
    ax.set_xlim(min(all_ys) - y_margin, max(all_ys) + y_margin)
    ax.set_ylim(min(min(all_xs) - x_margin, 1.4), max(all_xs) + x_margin)

    ax.set_xlabel("Y (lateral, m)", fontsize=11)
    ax.set_ylabel("X (distance to pallet, m)", fontsize=11)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    # Title + violation info
    title_lines = [f"Case {idx+1}: {name}"]
    title_lines.append(f"Level: {level}  |  Time: {elapsed:.0f}ms  |  Path: {metrics['length']:.2f}m  |  Euclid: {metrics['euclidean']:.2f}m")
    if reasons:
        title_lines.append("FAIL: " + "  |  ".join(reasons))
    else:
        title_lines.append("ALL PASS")

    ax.set_title("\n".join(title_lines), fontsize=10, color="red" if reasons else "green", fontweight="bold")

    # Annotate key metrics on chart
    txt_parts = [
        f"QM1 Detour:  {metrics['detour_ratio']:.2f}  (limit {QM1_DETOUR_MAX})",
        f"QM2 MaxAbsY: {metrics['max_abs_y']:.2f}  (limit {allowed_y:.1f})",
        f"QM3 MaxX:    {metrics['max_x']:.2f}  (limit {allowed_x:.1f})",
        f"QM5 RSDetour:{metrics['rs_detour']:.2f}  (limit {QM5_RS_DETOUR_MAX})",
        f"QM6 Shifts:  {metrics['shifts']}     (limit {QM6_GEAR_SHIFT_MAX})",
    ]
    ax.text(0.02, 0.02, "\n".join(txt_parts), transform=ax.transAxes,
            fontsize=8, verticalalignment="bottom", fontfamily="monospace",
            bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.85))

    fig.tight_layout()
    fname = os.path.join(out_dir, f"fail_{idx+1:02d}_{_safe_name(name)}.png")
    fig.savefig(fname, dpi=150)
    plt.close(fig)
    print(f"    → saved: {fname}")


def _plot_empty(idx, case, obs, out_dir, reason_text):
    """规划失败/超时时画一个仅显示场景的图"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    sx, sy, th_deg = case["x"], case["y"], case["th_deg"]

    wall = mpatches.Rectangle(
        (WALL_Y_MIN, WALL_X_MIN), WALL_Y_MAX - WALL_Y_MIN, WALL_X_MAX - WALL_X_MIN,
        linewidth=1.5, edgecolor="#333", facecolor="#ddd", alpha=0.4,
    )
    ax.add_patch(wall)
    for ob in obs:
        rect = mpatches.Rectangle(
            (ob["y"], ob["x"]), ob["h"], ob["w"],
            linewidth=2, edgecolor="red", facecolor="#ff000030",
        )
        ax.add_patch(rect)

    ax.plot(sy, sx, "o", color="green", markersize=10, zorder=10)
    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold", markersize=12, markeredgecolor="orange", markeredgewidth=2, zorder=10)

    ax.set_xlim(-5.5, 5.5)
    ax.set_ylim(1.4, max(sx + 1.5, 6.5))
    ax.set_xlabel("Y (lateral, m)")
    ax.set_ylabel("X (distance to pallet, m)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title(f"Case {idx+1}: {case['name']}\n{reason_text}", fontsize=11, color="red", fontweight="bold")

    fig.tight_layout()
    fname = os.path.join(out_dir, f"fail_{idx+1:02d}_{_safe_name(case['name'])}.png")
    fig.savefig(fname, dpi=150)
    plt.close(fig)
    print(f"    → saved: {fname}")


def _safe_name(s):
    return s.replace(" ", "_").replace("(", "").replace(")", "").replace(",", "_").replace("/", "_")


# ── main ──
def main():
    out_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                           "docs", "quality_failures")
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 60)
    print(" Visualizing 7 failed cases from test_path_quality_v2.py")
    print(f" Output: {out_dir}")
    print("=" * 60)

    prims = init_primitives()
    from planner_obs_v2 import plan_path_robust_obs_v2
    planner_fn = plan_path_robust_obs_v2

    for idx, case in enumerate(FAILED_CASES):
        plot_case(idx, case, prims, planner_fn, out_dir)

    print("\n" + "=" * 60)
    print(f" Done! {len(FAILED_CASES)} plots saved to {out_dir}")
    print("=" * 60)


if __name__ == "__main__":
    main()
