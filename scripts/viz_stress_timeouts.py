#!/usr/bin/env python3
"""
viz_stress_timeouts.py — 批量可视化压力测试中超时的 case
读取 logs/stress_timeouts_*.json，绘制每个超时 case 的场景布局，
并生成一个总览大图 + 逐个详图。

用法:
  cd scripts && python viz_stress_timeouts.py [--json PATH] [--out-dir PATH]

如未指定 --json，则自动选取 scripts/logs/ 下最新的 stress_timeouts_*.json
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


def _draw_scene(ax, case, show_title=True, compact=False):
    """在给定 ax 上绘制一个超时 case 的场景"""
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

    # ── 障碍物 ──
    for i, ob in enumerate(obstacles):
        ox, oy, ow, oh = ob["x"], ob["y"], ob["w"], ob["h"]
        rect = mpatches.Rectangle(
            (oy, ox), oh, ow,
            linewidth=1.5, edgecolor="red", facecolor="#ff000025",
            label="Obstacle" if i == 0 and not compact else None,
        )
        ax.add_patch(rect)
        # 障碍物中心标注
        cx, cy = ox + ow / 2, oy + oh / 2
        if not compact:
            ax.text(cy, cx, f"{ow:.1f}x{oh:.1f}", fontsize=6,
                    ha="center", va="center", color="#aa0000", fontweight="bold")

    # ── 目标点 ──
    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold", markersize=10 if not compact else 7,
            markeredgecolor="orange", markeredgewidth=1.5, zorder=10,
            label="Goal" if not compact else None)

    # ── 起点 + 朝向箭头 ──
    arrow_len = 0.4 if not compact else 0.3
    # 叉车前方 = -x 方向
    dx_arrow = -arrow_len * math.cos(sth)
    dy_arrow = -arrow_len * math.sin(sth)
    ax.annotate("", xy=(sy + dy_arrow, sx + dx_arrow), xytext=(sy, sx),
                arrowprops=dict(arrowstyle="-|>", color="#00aa00", lw=2.0))
    ax.plot(sy, sx, "o", color="#00cc00", markersize=9 if not compact else 6,
            markeredgecolor="#006600", markeredgewidth=1.0, zorder=10,
            label=f"Start" if not compact else None)

    # ── 叉车粗略轮廓 (参考用, 简化为矩形) ──
    # 叉车大致尺寸: 长~2.0m, 宽~0.6m, 中心在起点
    veh_l, veh_w = 1.8, 0.5
    corners_local = np.array([
        [-veh_l / 2, -veh_w / 2],
        [-veh_l / 2,  veh_w / 2],
        [ veh_l / 2,  veh_w / 2],
        [ veh_l / 2, -veh_w / 2],
        [-veh_l / 2, -veh_w / 2],
    ])
    cos_th, sin_th = math.cos(sth), math.sin(sth)
    R = np.array([[cos_th, -sin_th], [sin_th, cos_th]])
    corners_world = (R @ corners_local.T).T + np.array([sx, sy])
    ax.plot(corners_world[:, 1], corners_world[:, 0], "-", color="#00aa00",
            linewidth=1.0, alpha=0.5)

    # ── 从起点到目标的直线 (辅助线) ──
    ax.plot([sy, RS_GOAL_Y], [sx, RS_GOAL_X], "--", color="#aaaaaa",
            linewidth=0.8, alpha=0.5)

    # ── 范围设定 ──
    all_xs = [sx, RS_GOAL_X, WALL_X_MIN, WALL_X_MAX]
    all_ys = [sy, RS_GOAL_Y, WALL_Y_MIN, WALL_Y_MAX]
    for ob in obstacles:
        all_xs.extend([ob["x"], ob["x"] + ob["w"]])
        all_ys.extend([ob["y"], ob["y"] + ob["h"]])

    margin = 0.6 if not compact else 0.4
    ax.set_xlim(min(all_ys) - margin, max(all_ys) + margin)
    ax.set_ylim(min(min(all_xs) - margin, 1.2), max(all_xs) + margin)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, linewidth=0.5)

    if show_title:
        th_deg = math.degrees(sth)
        title = (f"Case #{case_id}  |  {n_obs} obs  |  "
                 f"Start=({sx:.1f}, {sy:.1f}, {th_deg:.0f}deg)  |  "
                 f"{elapsed:.0f}ms TIMEOUT")
        ax.set_title(title, fontsize=9 if not compact else 7,
                     color="red", fontweight="bold")

    if not compact:
        ax.set_xlabel("Y (lateral, m)", fontsize=9)
        ax.set_ylabel("X (forward, m)", fontsize=9)


def plot_overview(cases, out_dir):
    """所有超时 case 的总览大图"""
    n = len(cases)
    cols = min(4, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4.5 * rows))
    if rows == 1 and cols == 1:
        axes = np.array([axes])
    axes = np.atleast_2d(axes)

    for idx, case in enumerate(cases):
        r, c = divmod(idx, cols)
        ax = axes[r][c]
        _draw_scene(ax, case, show_title=True, compact=True)

    # 隐藏空白子图
    for idx in range(n, rows * cols):
        r, c = divmod(idx, cols)
        axes[r][c].set_visible(False)

    fig.suptitle(f"Stress Test Timeouts Overview  ({n} cases)",
                 fontsize=14, fontweight="bold", color="#cc0000", y=1.0)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    fpath = os.path.join(out_dir, "timeout_overview.png")
    fig.savefig(fpath, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Overview saved: {fpath}")


def plot_individual(cases, out_dir):
    """每个超时 case 单独的详细图"""
    for idx, case in enumerate(cases):
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        _draw_scene(ax, case, show_title=True, compact=False)

        # 额外标注
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
            f"Start:       ({sx:.2f}, {sy:.2f}, {math.degrees(sth):.1f} deg)",
            f"Goal:        ({RS_GOAL_X}, {RS_GOAL_Y})",
            f"Obstacles:   {len(obstacles)}",
            f"Elapsed:     {elapsed:.0f} ms (TIMEOUT)",
            f"A* expanded: {expanded}",
            f"Level:       {level if level else 'N/A (killed before report)'}",
        ]
        for i, ob in enumerate(obstacles):
            info_lines.append(
                f"  Obs-{i+1}: pos=({ob['x']:.2f},{ob['y']:.2f}) "
                f"size={ob['w']:.2f}x{ob['h']:.2f}")

        ax.text(0.02, 0.02, "\n".join(info_lines), transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom", fontfamily="monospace",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.9))

        # 标注分析：检查障碍物是否阻断起点-目标直线路径
        analysis = _analyze_blocking(sx, sy, sth, obstacles)
        ax.text(0.98, 0.02, analysis, transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom", horizontalalignment="right",
                fontfamily="monospace", color="#0055aa",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="#eef5ff", alpha=0.9))

        if not compact_legend_exists(ax):
            ax.legend(loc="upper right", fontsize=8)

        fig.tight_layout()
        fpath = os.path.join(out_dir, f"timeout_{idx+1:02d}_case{case.get('case_id','?')}.png")
        fig.savefig(fpath, dpi=150)
        plt.close(fig)
        print(f"  [{idx+1}/{len(cases)}] saved: {fpath}")


def compact_legend_exists(ax):
    return ax.get_legend() is not None


def _analyze_blocking(sx, sy, sth, obstacles):
    """简单分析障碍物与起点->目标路径的关系"""
    gx, gy = RS_GOAL_X, RS_GOAL_Y
    euclid = math.hypot(sx - gx, sy - gy)

    lines = [
        "-- Path Analysis --",
        f"Euclidean dist: {euclid:.2f}m",
    ]

    # 检查障碍物是否落在起点与目标之间的通道
    blocking_obs = []
    for i, ob in enumerate(obstacles):
        ox, oy, ow, oh = ob["x"], ob["y"], ob["w"], ob["h"]
        # 障碍物 X 范围
        ob_x_min, ob_x_max = ox, ox + ow
        ob_y_min, ob_y_max = oy, oy + oh
        # 检查是否在起点和目标之间的 x 范围
        path_x_min = min(sx, gx)
        path_x_max = max(sx, gx)
        path_y_min = min(sy, gy)
        path_y_max = max(sy, gy)

        x_overlap = ob_x_min < path_x_max and ob_x_max > path_x_min
        y_near = abs((oy + oh / 2) - (sy + gy) / 2) < (oh / 2 + 1.5)

        if x_overlap and y_near:
            blocking_obs.append(i + 1)

    if blocking_obs:
        lines.append(f"Blocking obs: {blocking_obs}")
    else:
        lines.append("No direct blocking")

    # 检查是否在关键 x 段（墙壁前通道）有障碍物
    critical_obs = []
    for i, ob in enumerate(obstacles):
        if ob["x"] < 5.0 and ob["x"] + ob["w"] > 2.5:
            critical_obs.append(i + 1)
    if critical_obs:
        lines.append(f"Critical zone (x 2.5~5): obs {critical_obs}")

    # 起始朝向分析
    th_deg = math.degrees(sth)
    heading_to_goal = math.degrees(math.atan2(gy - sy, gx - sx))
    heading_diff = abs(((th_deg - heading_to_goal + 180) % 360) - 180)
    lines.append(f"Heading to goal: {heading_to_goal:.0f} deg")
    lines.append(f"Heading diff: {heading_diff:.0f} deg")
    if heading_diff > 90:
        lines.append("=> Facing away from goal!")

    return "\n".join(lines)


def find_latest_json(log_dir):
    """查找 logs 目录下最新的 stress_timeouts JSON"""
    patterns = [
        os.path.join(log_dir, "stress_timeouts_*.json"),
        os.path.join(log_dir, "..", "logs", "stress_timeouts_*.json"),
    ]
    all_files = []
    for pat in patterns:
        all_files.extend(glob.glob(pat))
    if not all_files:
        return None
    return max(all_files, key=os.path.getmtime)


def main():
    parser = argparse.ArgumentParser(description="Visualize stress test timeout cases")
    parser.add_argument("--json", type=str, default=None,
                        help="Path to stress_timeouts_*.json")
    parser.add_argument("--out-dir", type=str, default=None,
                        help="Output directory for plots")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    log_dir = os.path.join(script_dir, "logs")

    # 查找 JSON
    json_path = args.json
    if not json_path:
        json_path = find_latest_json(log_dir)
    if not json_path or not os.path.exists(json_path):
        print(f"ERROR: No timeout JSON found. Run test_stress.py first.")
        sys.exit(1)

    print(f"Loading timeout cases from: {json_path}")
    with open(json_path, "r", encoding="utf-8") as f:
        cases = json.load(f)
    print(f"Loaded {len(cases)} timeout cases.\n")

    if not cases:
        print("No timeout cases to visualize.")
        return

    # 输出目录
    out_dir = args.out_dir
    if not out_dir:
        out_dir = os.path.join(os.path.dirname(script_dir), "docs", "stress_timeouts")
    os.makedirs(out_dir, exist_ok=True)

    # 统计摘要
    print("=" * 60)
    print(f" Timeout Cases Visualization ({len(cases)} cases)")
    print(f" Output: {out_dir}")
    print("=" * 60)
    from collections import Counter
    type_counts = Counter(c.get("type", "?") for c in cases)
    nobs_counts = Counter(c.get("n_obs", 0) for c in cases)
    print(f"\n  By type:     {dict(type_counts)}")
    print(f"  By #obs:     {dict(sorted(nobs_counts.items()))}")
    elapsed_vals = [c.get("elapsed_ms", 0) for c in cases]
    if elapsed_vals:
        print(f"  Elapsed:     min={min(elapsed_vals):.0f}ms  "
              f"max={max(elapsed_vals):.0f}ms  "
              f"avg={sum(elapsed_vals)/len(elapsed_vals):.0f}ms\n")

    # 绘制总览
    print("Generating overview plot...")
    plot_overview(cases, out_dir)

    # 绘制逐个详图
    print("\nGenerating individual plots...")
    plot_individual(cases, out_dir)

    print(f"\n{'='*60}")
    print(f" Done! {len(cases)+1} plots saved to {out_dir}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
