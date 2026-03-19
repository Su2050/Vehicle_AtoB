#!/usr/bin/env python3
"""
viz_stress_unexpected_fails.py

Visualize UNEXPECTED_FAIL cases from stress_timing_*.json by regenerating
the original cases from test_stress.generate_cases(profile, seed).

This fills the gap left by viz_stress_failures.py, which only covers timeout
and collision exports.
"""

import argparse
import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
TESTS_DIR = os.path.join(SCRIPT_DIR, "tests")
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, TESTS_DIR)

import primitives  # noqa: E402
from primitives import RS_GOAL_X, RS_GOAL_Y  # noqa: E402
from test_stress import generate_cases  # noqa: E402

WALL_X_MIN, WALL_X_MAX = 1.92, 3.0
WALL_Y_MIN, WALL_Y_MAX = -3.0, 3.0


def _find_latest_timing_json(log_dir):
    patterns = [
        os.path.join(log_dir, "stress_timing_*.json"),
        os.path.join(log_dir, "..", "logs", "stress_timing_*.json"),
    ]
    files = []
    for pattern in patterns:
        files.extend(glob.glob(pattern))
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def _clear_previous_outputs(out_dir):
    for pattern in ("unexpected_fail_*.png", "unexpected_fail_overview.png"):
        for path in glob.glob(os.path.join(out_dir, pattern)):
            try:
                os.remove(path)
            except OSError:
                pass


def _draw_multi_circle_vehicle(ax, cx, cy, cth, color="#1f77b4", alpha=0.22,
                               label=None, zorder=8):
    cos_th = math.cos(cth)
    sin_th = math.sin(cth)
    offsets = primitives.VEHICLE_CHECK_OFFSETS
    half_w = primitives.VEHICLE_HALF_WIDTH

    for i, offset in enumerate(offsets):
        world_x = cx + offset * cos_th
        world_y = cy + offset * sin_th
        circle = mpatches.Circle(
            (world_y, world_x), half_w,
            linewidth=1.0, edgecolor=color, facecolor=color,
            alpha=alpha, zorder=zorder,
            label=label if i == 0 else None,
        )
        ax.add_patch(circle)

    front_offset = min(offsets)
    rear_offset = max(offsets)
    front_x = cx + front_offset * cos_th
    front_y = cy + front_offset * sin_th
    rear_x = cx + rear_offset * cos_th
    rear_y = cy + rear_offset * sin_th
    ax.plot([front_y, rear_y], [front_x, rear_x],
            "-", color=color, linewidth=1.0, alpha=0.8, zorder=zorder)


def _heading_diff_deg(sx, sy, sth):
    heading_to_goal = math.atan2(RS_GOAL_Y - sy, RS_GOAL_X - sx)
    diff = abs(((sth - heading_to_goal + math.pi) % (2 * math.pi)) - math.pi)
    return math.degrees(diff)


def _draw_scene(ax, case, rec, compact=False):
    sx, sy, sth = case["x"], case["y"], case["th"]
    obstacles = case["obstacles"]

    wall = mpatches.Rectangle(
        (WALL_Y_MIN, WALL_X_MIN),
        WALL_Y_MAX - WALL_Y_MIN,
        WALL_X_MAX - WALL_X_MIN,
        linewidth=1.0, edgecolor="#777", facecolor="#d9d9d9", alpha=0.35,
    )
    ax.add_patch(wall)

    for i, ob in enumerate(obstacles):
        ox, oy, ow, oh = ob["x"], ob["y"], ob["w"], ob["h"]
        rect = mpatches.Rectangle(
            (oy, ox), oh, ow,
            linewidth=1.4, edgecolor="#c62828", facecolor="#ff000020",
            label="Obstacle" if i == 0 and not compact else None,
        )
        ax.add_patch(rect)

    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold",
            markersize=10 if not compact else 7,
            markeredgecolor="orange", markeredgewidth=1.4,
            label="Goal" if not compact else None, zorder=10)

    _draw_multi_circle_vehicle(
        ax, sx, sy, sth,
        color="#1976d2",
        alpha=0.22,
        label=f"Vehicle ({len(primitives.VEHICLE_CHECK_OFFSETS)}-circle)" if not compact else None,
        zorder=8,
    )
    ax.plot(sy, sx, "o", color="#1976d2", markersize=7 if not compact else 5, zorder=11)

    arrow_len = 0.42 if compact else 0.55
    dx = -arrow_len * math.sin(sth)
    dy = -arrow_len * math.cos(sth)
    ax.annotate("", xy=(sy + dx, sx + dy), xytext=(sy, sx),
                arrowprops=dict(arrowstyle="-|>", color="#0d47a1", lw=2.0))

    gate_hint = rec.get("late_merge_gate_hint")
    if gate_hint:
        ghx, ghy, _ = gate_hint
        ax.plot(ghy, ghx, "D", color="#8e24aa", markersize=8 if not compact else 6,
                markeredgecolor="#4a148c", markeredgewidth=1.0,
                label="Gate hint" if not compact else None, zorder=12)

    gate_pose = rec.get("late_merge_gate_stage_pose")
    if gate_pose:
        gpx, gpy, gpth = gate_pose
        _draw_multi_circle_vehicle(
            ax, gpx, gpy, gpth,
            color="#00acc1", alpha=0.18,
            label="Gate-stage pose" if not compact else None, zorder=7,
        )
        ax.plot(gpy, gpx, "X", color="#00acc1", markersize=8 if not compact else 6, zorder=12)

    ax.plot([sy, RS_GOAL_Y], [sx, RS_GOAL_X], "--", color="#999", linewidth=0.8, alpha=0.5)

    all_xs = [sx, RS_GOAL_X, WALL_X_MIN, WALL_X_MAX]
    all_ys = [sy, RS_GOAL_Y, WALL_Y_MIN, WALL_Y_MAX]
    for ob in obstacles:
        all_xs.extend([ob["x"], ob["x"] + ob["w"]])
        all_ys.extend([ob["y"], ob["y"] + ob["h"]])
    if gate_hint:
        all_xs.append(gate_hint[0])
        all_ys.append(gate_hint[1])
    if gate_pose:
        all_xs.append(gate_pose[0])
        all_ys.append(gate_pose[1])

    margin = 0.75 if not compact else 0.5
    ax.set_xlim(min(all_ys) - margin, max(all_ys) + margin)
    ax.set_ylim(min(min(all_xs) - margin, 1.0), max(all_xs) + margin)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, linewidth=0.5)

    title = (
        f"#{case['id']} {case['type']} | {rec.get('level','')} | "
        f"{rec.get('elapsed_ms', 0):.0f}ms | exp={rec.get('expanded', 0)}"
    )
    ax.set_title(title, fontsize=9 if not compact else 7,
                 color="#ad1457", fontweight="bold")

    if not compact:
        ax.set_xlabel("Y (lateral, m)")
        ax.set_ylabel("X (forward, m)")


def _build_notes(case, rec):
    sx, sy, sth = case["x"], case["y"], case["th"]
    lines = []
    lines.append(f"type={case['type']}")
    lines.append(f"start=({sx:.2f}, {sy:.2f}, {math.degrees(sth):.1f}deg)")
    lines.append(f"dist_to_goal={math.hypot(sx - RS_GOAL_X, sy - RS_GOAL_Y):.2f}m")
    lines.append(f"heading_diff={_heading_diff_deg(sx, sy, sth):.1f}deg")
    lines.append(f"n_obs={len(case['obstacles'])}")
    lines.append(f"elapsed={rec.get('elapsed_ms', 0):.1f}ms")
    lines.append(f"expanded={rec.get('expanded', 0)}")
    lines.append(f"level={rec.get('level', '')}")
    lines.append(f"late_merge={bool(rec.get('l18_late_merge_detected'))}")
    lines.append(
        f"gate_attempted={bool(rec.get('late_merge_gate_attempted'))} "
        f"(count={rec.get('late_merge_gate_attempt_count', 0)})"
    )
    lines.append(f"gate_stage_pose={rec.get('late_merge_gate_stage_pose') is not None}")
    lines.append(f"deep_attempted={bool(rec.get('late_merge_deep_attempted'))}")
    if rec.get("late_merge_deep_attempted"):
        lines.append(f"deep_start={rec.get('late_merge_deep_start_kind')}")
        lines.append(f"deep_expanded={rec.get('late_merge_deep_expanded', 0)}")
    if rec.get("late_merge_gate_hint") is not None:
        hint = rec["late_merge_gate_hint"]
        lines.append(f"gate_hint=({hint[0]:.2f}, {hint[1]:.2f}, {hint[2]:.2f})")
    if rec.get("late_merge_gate_stage_pose") is not None:
        pose = rec["late_merge_gate_stage_pose"]
        lines.append(f"gate_pose=({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f})")
    return "\n".join(lines)


def _plot_overview(items, out_dir):
    n = len(items)
    cols = min(4, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5.4 * cols, 4.8 * rows))
    axes = np.array(axes, ndmin=2)

    for idx, item in enumerate(items):
        r, c = divmod(idx, cols)
        ax = axes[r, c]
        _draw_scene(ax, item["case"], item["record"], compact=True)

    for idx in range(n, rows * cols):
        r, c = divmod(idx, cols)
        axes[r, c].axis("off")

    fig.suptitle("Stress UNEXPECTED_FAIL Overview", fontsize=16, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(out_dir, "unexpected_fail_overview.png")
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return path


def _plot_detail(item, out_dir, rank):
    case = item["case"]
    rec = item["record"]

    fig = plt.figure(figsize=(12.5, 6.8))
    ax = fig.add_subplot(1, 2, 1)
    _draw_scene(ax, case, rec, compact=False)

    ax2 = fig.add_subplot(1, 2, 2)
    ax2.axis("off")
    ax2.text(
        0.02, 0.98, _build_notes(case, rec),
        va="top", ha="left", family="monospace", fontsize=11,
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#f7f7f7", edgecolor="#cccccc"),
    )

    fig.suptitle(
        f"Unexpected Fail #{case['id']} ({case['type']})",
        fontsize=16, fontweight="bold",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    path = os.path.join(out_dir, f"unexpected_fail_{rank:02d}_case{case['id']}.png")
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return path


def main():
    parser = argparse.ArgumentParser(description="Visualize stress UNEXPECTED_FAIL cases")
    parser.add_argument("--json", type=str, default=None,
                        help="Path to stress_timing_*.json")
    parser.add_argument("--seed", type=int, default=42,
                        help="Seed used by test_stress.py (default: 42)")
    parser.add_argument("--type", type=str, default=None,
                        help="Only include a specific case type, e.g. SlalomStaggered")
    parser.add_argument("--top", type=int, default=20,
                        help="How many failing cases to visualize")
    parser.add_argument("--sort-by", choices=["elapsed", "expanded", "case_id"], default="elapsed",
                        help="Sort metric for selected failures")
    parser.add_argument("--out-dir", type=str, default=None,
                        help="Output directory")
    args = parser.parse_args()

    log_dir = os.path.join(ROOT_DIR, "logs")
    json_path = args.json or _find_latest_timing_json(log_dir)
    if not json_path or not os.path.exists(json_path):
        print("ERROR: No stress_timing JSON found. Run test_stress.py first.")
        raise SystemExit(1)

    print(f"Loading timing data from: {json_path}")
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    profile = data.get("profile", "standard")
    records = data.get("records", [])
    fail_records = [r for r in records if r.get("category") == "UNEXPECTED_FAIL"]
    if args.type:
        fail_records = [r for r in fail_records if r.get("type") == args.type]
    if not fail_records:
        print("No UNEXPECTED_FAIL cases matched the filter.")
        return

    sort_key = {
        "elapsed": lambda r: (-r.get("elapsed_ms", 0.0), r.get("case_id", 0)),
        "expanded": lambda r: (-r.get("expanded", 0), r.get("case_id", 0)),
        "case_id": lambda r: (r.get("case_id", 0),),
    }[args.sort_by]
    fail_records = sorted(fail_records, key=sort_key)
    selected_records = fail_records[:max(1, args.top)]

    print(f"Regenerating cases with profile={profile}, seed={args.seed} ...")
    cases = generate_cases(profile, seed=args.seed)
    case_map = {c["id"]: c for c in cases}

    merged = []
    missing = []
    for rec in selected_records:
        case = case_map.get(rec["case_id"])
        if case is None:
            missing.append(rec["case_id"])
            continue
        merged.append({"record": rec, "case": case})

    if missing:
        print(f"WARNING: Missing case ids after regeneration: {missing[:10]}")
    if not merged:
        print("No cases available for visualization after regeneration.")
        return

    out_dir = args.out_dir or os.path.join(ROOT_DIR, "docs", "stress_unexpected_fails")
    if args.type:
        out_dir = os.path.join(out_dir, args.type)
    os.makedirs(out_dir, exist_ok=True)
    _clear_previous_outputs(out_dir)

    print(f"Visualizing {len(merged)} UNEXPECTED_FAIL cases -> {out_dir}")
    overview_path = _plot_overview(merged, out_dir)
    print(f"Saved overview: {overview_path}")
    for idx, item in enumerate(merged, start=1):
        path = _plot_detail(item, out_dir, idx)
        print(f"  [{idx:02d}/{len(merged)}] {path}")


if __name__ == "__main__":
    main()
