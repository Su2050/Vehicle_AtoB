#!/usr/bin/env python3
"""
viz_stress_failures.py — 可视化压力测试中所有失败 case（超时 + 碰撞）

读取 logs/stress_timeouts_*.json 和 logs/stress_collisions_*.json，
绘制每个失败 case 的场景布局（使用多圆碰撞模型），生成总览大图 + 逐个详图。

用法:
  cd scripts && python viz_stress_failures.py [--timeout-json PATH] [--collision-json PATH] [--out-dir PATH]
"""

import os
import sys
import json
import math
import glob
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# ── 场景常量 ──
RS_GOAL_X = 2.10
RS_GOAL_Y = 0.0
WALL_X_MIN, WALL_X_MAX = 1.92, 3.0
WALL_Y_MIN, WALL_Y_MAX = -3.0, 3.0

# ── 多圆碰撞模型参数（与 primitives.py 一致）──
VEHICLE_HALF_WIDTH = 0.25
VEHICLE_CHECK_OFFSETS = (0.5, 0.0, -0.5, -1.0, -1.45, -1.87)
VEHICLE_MAX_RADIUS = 1.87 + VEHICLE_HALF_WIDTH  # 2.12m


def _draw_multi_circle_vehicle(ax, cx, cy, cth, color="#00aa00", alpha=0.25,
                                label=None, zorder=8):
    """
    在 (cx, cy, cth) 处绘制多圆碰撞模型的车身轮廓。
    注意坐标系：图中 X轴=Y(lateral), Y轴=X(forward)
    """
    cos_th = math.cos(cth)
    sin_th = math.sin(cth)

    for i, offset in enumerate(VEHICLE_CHECK_OFFSETS):
        # 局部坐标系下：offset 沿车身 x 方向（正=后方，负=前方/叉齿方向）
        # 车辆前方 = (-cos(θ), -sin(θ))
        # 所以 offset>0 → 远离前方（后方），offset<0 → 靠近前方
        world_x = cx + offset * cos_th
        world_y = cy + offset * sin_th

        circle = mpatches.Circle(
            (world_y, world_x), VEHICLE_HALF_WIDTH,
            linewidth=0.8, edgecolor=color, facecolor=color,
            alpha=alpha, zorder=zorder,
            label=label if i == 0 else None,
        )
        ax.add_patch(circle)

    # 绘制车身中心线（连接最前和最后的圆心）
    front_offset = min(VEHICLE_CHECK_OFFSETS)
    rear_offset = max(VEHICLE_CHECK_OFFSETS)
    front_x = cx + front_offset * cos_th
    front_y = cy + front_offset * sin_th
    rear_x = cx + rear_offset * cos_th
    rear_y = cy + rear_offset * sin_th
    ax.plot([front_y, rear_y], [front_x, rear_x],
            "-", color=color, linewidth=1.0, alpha=0.5, zorder=zorder - 1)


def _draw_scene(ax, case, fail_type="TIMEOUT", show_title=True, compact=False):
    """在给定 ax 上绘制一个失败 case 的场景"""
    start = case.get("start", {})
    sx = start.get("x", 5.0)
    sy = start.get("y", 0.0)
    sth = start.get("th", 0.0)
    obstacles = case.get("obstacles", [])
    case_id = case.get("case_id", "?")
    n_obs = case.get("n_obs", len(obstacles))
    elapsed = case.get("elapsed_ms", 0)

    # ── 墙壁区域 ──
    wall = mpatches.Rectangle(
        (WALL_Y_MIN, WALL_X_MIN),
        WALL_Y_MAX - WALL_Y_MIN,
        WALL_X_MAX - WALL_X_MIN,
        linewidth=1.0, edgecolor="#555", facecolor="#ddd", alpha=0.35,
    )
    ax.add_patch(wall)
    if not compact:
        ax.text((WALL_Y_MIN + WALL_Y_MAX) / 2, (WALL_X_MIN + WALL_X_MAX) / 2,
                "Wall", fontsize=7, ha="center", va="center", color="#888")

    # ── 障碍物 ──
    for i, ob in enumerate(obstacles):
        ox, oy, ow, oh = ob["x"], ob["y"], ob["w"], ob["h"]
        rect = mpatches.Rectangle(
            (oy, ox), oh, ow,
            linewidth=1.5, edgecolor="red", facecolor="#ff000020",
            label="Obstacle" if i == 0 and not compact else None,
        )
        ax.add_patch(rect)
        if not compact:
            cx_ob, cy_ob = ox + ow / 2, oy + oh / 2
            ax.text(cy_ob, cx_ob, f"{ow:.1f}×{oh:.1f}", fontsize=6,
                    ha="center", va="center", color="#aa0000", fontweight="bold")

    # ── 目标点 ──
    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold",
            markersize=10 if not compact else 7,
            markeredgecolor="orange", markeredgewidth=1.5, zorder=10,
            label="Goal" if not compact else None)

    # ── 起点：多圆碰撞模型 ──
    _draw_multi_circle_vehicle(
        ax, sx, sy, sth,
        color="#00aa00", alpha=0.25,
        label="Vehicle (6-circle)" if not compact else None,
        zorder=8,
    )

    # ── 起点圆心 + 朝向箭头 ──
    arrow_len = 0.5 if not compact else 0.35
    # 车辆前方 = (-cos(θ), -sin(θ))
    dx_arrow = -arrow_len * math.cos(sth)
    dy_arrow = -arrow_len * math.sin(sth)
    ax.annotate("", xy=(sy + dy_arrow, sx + dx_arrow), xytext=(sy, sx),
                arrowprops=dict(arrowstyle="-|>", color="#006600", lw=2.5))
    ax.plot(sy, sx, "o", color="#00cc00", markersize=8 if not compact else 5,
            markeredgecolor="#006600", markeredgewidth=1.0, zorder=10,
            label="Start (ref pt)" if not compact else None)

    # ── 碰撞点 (仅碰撞 case) ──
    collision_pt = case.get("collision_pt")
    if collision_pt:
        cpx, cpy, cpth = collision_pt
        ax.plot(cpy, cpx, "X", color="#ff0000", markersize=14 if not compact else 10,
                markeredgecolor="#800000", markeredgewidth=2, zorder=15,
                label="Collision pt" if not compact else None)
        # 碰撞位置也画车身
        _draw_multi_circle_vehicle(
            ax, cpx, cpy, cpth,
            color="#ff0000", alpha=0.3,
            label=None, zorder=7,
        )

    # ── 从起点到目标的直线 (辅助线) ──
    ax.plot([sy, RS_GOAL_Y], [sx, RS_GOAL_X], "--", color="#aaaaaa",
            linewidth=0.8, alpha=0.5)

    # ── 范围设定 ──
    all_xs = [sx, RS_GOAL_X, WALL_X_MIN, WALL_X_MAX]
    all_ys = [sy, RS_GOAL_Y, WALL_Y_MIN, WALL_Y_MAX]
    for ob in obstacles:
        all_xs.extend([ob["x"], ob["x"] + ob["w"]])
        all_ys.extend([ob["y"], ob["y"] + ob["h"]])
    if collision_pt:
        all_xs.append(collision_pt[0])
        all_ys.append(collision_pt[1])

    margin = 0.8 if not compact else 0.5
    ax.set_xlim(min(all_ys) - margin, max(all_ys) + margin)
    ax.set_ylim(min(min(all_xs) - margin, 1.0), max(all_xs) + margin)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, linewidth=0.5)

    if show_title:
        th_deg = math.degrees(sth)
        status_str = f"TIMEOUT {elapsed:.0f}ms" if fail_type == "TIMEOUT" else "COLLISION!"
        title = (f"#{case_id}  {case.get('type', '')}  |  "
                 f"({sx:.1f},{sy:.1f},{th_deg:.0f}°)  |  "
                 f"{n_obs} obs  |  {status_str}")
        color = "#cc0000" if fail_type == "COLLISION" else "#cc6600"
        ax.set_title(title, fontsize=9 if not compact else 7,
                     color=color, fontweight="bold")

    if not compact:
        ax.set_xlabel("Y (lateral, m)", fontsize=9)
        ax.set_ylabel("X (forward, m)", fontsize=9)


def _analyze_case(case, fail_type):
    """分析失败 case 的根因"""
    start = case.get("start", {})
    sx = start.get("x", 5.0)
    sy = start.get("y", 0.0)
    sth = start.get("th", 0.0)
    obstacles = case.get("obstacles", [])

    gx, gy = RS_GOAL_X, RS_GOAL_Y
    euclid = math.hypot(sx - gx, sy - gy)

    lines = [f"── Analysis ({fail_type}) ──"]
    lines.append(f"Euclidean: {euclid:.2f}m")

    # 起始朝向 (θ=0 时车头朝 -x)
    forward_deg = math.degrees(sth) + 180
    heading_to_goal = math.degrees(math.atan2(gy - sy, gx - sx))
    heading_diff = abs(((forward_deg - heading_to_goal + 180) % 360) - 180)
    lines.append(f"Heading diff: {heading_diff:.0f}°")
    if heading_diff > 120:
        lines.append(">> Facing away from goal!")
    elif heading_diff > 60:
        lines.append(">> Large heading offset")

    # 障碍物分析
    blocking, critical = [], []
    for i, ob in enumerate(obstacles):
        ox, ow = ob["x"], ob["w"]
        oy, oh = ob["y"], ob["h"]
        # 阻塞路径
        if ox < max(sx, 5.0) and ox + ow > gx:
            blocking.append(i + 1)
        # 在关键区域 (x: 2.5~5.0)
        if 2.5 < ox < 5.0 or 2.5 < ox + ow < 5.0:
            critical.append(i + 1)
        # 距起点很近
        dist_to_start = math.hypot(ox + ow / 2 - sx, oy + oh / 2 - sy)
        if dist_to_start < 1.5:
            lines.append(f"Obs-{i+1} very close ({dist_to_start:.1f}m)")

    if blocking:
        lines.append(f"Blocking: obs {blocking}")
    if critical:
        lines.append(f"Critical zone: obs {critical}")

    # 碰撞信息
    if fail_type == "COLLISION":
        reason = case.get("collision_reason", "?")
        source = case.get("collision_source", "?")
        lines.append(f"Collision: {reason}")
        lines.append(f"Source: {source}")
        cp = case.get("collision_pt")
        if cp:
            lines.append(f"At: ({cp[0]:.2f},{cp[1]:.2f},{math.degrees(cp[2]):.0f}°)")

    # 起始位置特殊性
    if sx < 4.0:
        lines.append(">> Start close to wall")
    if abs(sy) > 2.0:
        lines.append(f">> Large y offset ({sy:.1f}m)")

    return "\n".join(lines)


def plot_overview(cases, fail_types, out_dir):
    """所有失败 case 的总览大图"""
    n = len(cases)
    cols = min(4, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5.5 * cols, 5.0 * rows))
    if n == 1:
        axes = np.array([[axes]])
    elif rows == 1:
        axes = np.atleast_2d(axes)
    else:
        axes = np.atleast_2d(axes)

    for idx, (case, ft) in enumerate(zip(cases, fail_types)):
        r, c = divmod(idx, cols)
        _draw_scene(axes[r][c], case, fail_type=ft, show_title=True, compact=True)

    for idx in range(n, rows * cols):
        r, c = divmod(idx, cols)
        axes[r][c].set_visible(False)

    n_to = sum(1 for ft in fail_types if ft == "TIMEOUT")
    n_col = sum(1 for ft in fail_types if ft == "COLLISION")
    fig.suptitle(
        f"Stress Test Failures  ({n} cases: {n_to} timeouts + {n_col} collisions)",
        fontsize=14, fontweight="bold", color="#cc0000", y=1.0)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    fpath = os.path.join(out_dir, "failures_overview.png")
    fig.savefig(fpath, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Overview saved: {fpath}")


def plot_individual(cases, fail_types, out_dir):
    """每个失败 case 单独的详细图"""
    for idx, (case, ft) in enumerate(zip(cases, fail_types)):
        fig, ax = plt.subplots(1, 1, figsize=(11, 9))
        _draw_scene(ax, case, fail_type=ft, show_title=True, compact=False)

        # 信息面板
        start = case.get("start", {})
        sx = start.get("x", 5.0)
        sy = start.get("y", 0.0)
        sth = start.get("th", 0.0)
        obstacles = case.get("obstacles", [])
        elapsed = case.get("elapsed_ms", 0)
        expanded = case.get("expanded", 0)
        level = case.get("level", "")

        info_lines = [
            f"Case ID:     {case.get('case_id', '?')}",
            f"Type:        {case.get('type', '?')}",
            f"Fail Type:   {ft}",
            f"Start:       ({sx:.2f}, {sy:.2f}, {math.degrees(sth):.1f}°)",
            f"Goal:        ({RS_GOAL_X}, {RS_GOAL_Y})",
            f"Obstacles:   {len(obstacles)}",
            f"Elapsed:     {elapsed:.0f} ms",
            f"A* expanded: {expanded}",
            f"Level:       {level if level else 'N/A'}",
        ]
        for i, ob in enumerate(obstacles):
            info_lines.append(
                f"  Obs-{i+1}: ({ob['x']:.2f},{ob['y']:.2f}) "
                f"{ob['w']:.2f}×{ob['h']:.2f}")

        ax.text(0.02, 0.02, "\n".join(info_lines), transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom", fontfamily="monospace",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.9))

        # 分析面板
        analysis = _analyze_case(case, ft)
        ax.text(0.98, 0.02, analysis, transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom", horizontalalignment="right",
                fontfamily="monospace", color="#0055aa",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="#eef5ff", alpha=0.9))

        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()

        tag = "collision" if ft == "COLLISION" else "timeout"
        fpath = os.path.join(out_dir, f"fail_{idx+1:02d}_{tag}_case{case.get('case_id','?')}.png")
        fig.savefig(fpath, dpi=150)
        plt.close(fig)
        print(f"  [{idx+1}/{len(cases)}] saved: {fpath}")


def find_latest_json(log_dir, prefix):
    """查找 logs 目录下最新的指定前缀的 JSON"""
    patterns = [
        os.path.join(log_dir, f"{prefix}_*.json"),
        os.path.join(log_dir, "..", "logs", f"{prefix}_*.json"),
    ]
    all_files = []
    for pat in patterns:
        all_files.extend(glob.glob(pat))
    if not all_files:
        return None
    return max(all_files, key=os.path.getmtime)


def main():
    parser = argparse.ArgumentParser(
        description="Visualize stress test failure cases (timeouts + collisions)")
    parser.add_argument("--timeout-json", type=str, default=None,
                        help="Path to stress_timeouts_*.json")
    parser.add_argument("--collision-json", type=str, default=None,
                        help="Path to stress_collisions_*.json")
    parser.add_argument("--out-dir", type=str, default=None,
                        help="Output directory for plots")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(script_dir, "logs")

    # 查找 JSON 文件
    timeout_json = args.timeout_json or find_latest_json(log_dir, "stress_timeouts")
    collision_json = args.collision_json or find_latest_json(log_dir, "stress_collisions")

    all_cases = []
    fail_types = []

    # 加载超时 cases
    if timeout_json and os.path.exists(timeout_json):
        print(f"Loading timeout cases from: {timeout_json}")
        with open(timeout_json, "r", encoding="utf-8") as f:
            timeout_cases = json.load(f)
        print(f"  Loaded {len(timeout_cases)} timeout cases")
        for c in timeout_cases:
            all_cases.append(c)
            fail_types.append("TIMEOUT")
    else:
        print("No timeout JSON found.")

    # 加载碰撞 cases
    if collision_json and os.path.exists(collision_json):
        print(f"Loading collision cases from: {collision_json}")
        with open(collision_json, "r", encoding="utf-8") as f:
            collision_cases = json.load(f)
        print(f"  Loaded {len(collision_cases)} collision cases")
        for c in collision_cases:
            c.setdefault("n_obs", len(c.get("obstacles", [])))
            c.setdefault("elapsed_ms", 0)
            c.setdefault("expanded", 0)
            c.setdefault("level", "")
            all_cases.append(c)
            fail_types.append("COLLISION")
    else:
        print("No collision JSON found.")

    if not all_cases:
        print("No failure cases to visualize. Exiting.")
        return

    # 输出目录
    out_dir = args.out_dir
    if not out_dir:
        out_dir = os.path.join(os.path.dirname(script_dir), "docs", "stress_failures")
    os.makedirs(out_dir, exist_ok=True)

    # 统计摘要
    print(f"\n{'=' * 60}")
    print(f" Stress Test Failures Visualization ({len(all_cases)} cases)")
    print(f" Output: {out_dir}")
    print(f"{'=' * 60}")

    n_to = sum(1 for ft in fail_types if ft == "TIMEOUT")
    n_col = sum(1 for ft in fail_types if ft == "COLLISION")
    print(f"  Timeouts:    {n_to}")
    print(f"  Collisions:  {n_col}")

    from collections import Counter
    type_counts = Counter(c.get("type", "?") for c in all_cases)
    nobs_counts = Counter(c.get("n_obs", 0) for c in all_cases)
    print(f"  By type:     {dict(type_counts)}")
    print(f"  By #obs:     {dict(sorted(nobs_counts.items()))}")

    # 绘制总览
    print(f"\nGenerating overview plot...")
    plot_overview(all_cases, fail_types, out_dir)

    # 绘制逐个详图
    print(f"\nGenerating individual plots...")
    plot_individual(all_cases, fail_types, out_dir)

    print(f"\n{'=' * 60}")
    print(f" Done! {len(all_cases) + 1} plots saved to {out_dir}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    main()
