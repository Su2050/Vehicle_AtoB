#!/usr/bin/env python3
import argparse
import json
import math
import os
import re
import textwrap

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt

import primitives
from primitives import RS_GOAL_X, RS_GOAL_Y
from solvability_oracle import (
    _load_case_from_generated_pool,
    _load_obstacles_from_json,
    analyze_problem_solvability,
)

WALL_X_MIN, WALL_X_MAX = 1.92, 3.0
WALL_Y_MIN, WALL_Y_MAX = -3.0, 3.0

_STATUS_STYLE = {
    "pose_blocked": {"color": "#d32f2f", "label": "Pose blocked", "alpha": 0.35},
    "box_blocked": {"color": "#fb8c00", "label": "Box blocked", "alpha": 0.45},
    "insert_blocked": {"color": "#757575", "label": "Insert blocked", "alpha": 0.55},
    "certified": {"color": "#2e7d32", "label": "Certified", "alpha": 0.8},
}
_STATUS_RANK = {
    "pose_blocked": 0,
    "box_blocked": 1,
    "insert_blocked": 2,
    "certified": 3,
}


def _draw_multi_circle_vehicle(ax, cx, cy, cth, color="#1976d2", alpha=0.22,
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
            linewidth=1.0, edgecolor=color, facecolor=color, alpha=alpha,
            zorder=zorder, label=label if i == 0 else None,
        )
        ax.add_patch(circle)

    front_offset = min(offsets)
    rear_offset = max(offsets)
    front_x = cx + front_offset * cos_th
    front_y = cy + front_offset * sin_th
    rear_x = cx + rear_offset * cos_th
    rear_y = cy + rear_offset * sin_th
    ax.plot([front_y, rear_y], [front_x, rear_x], "-", color=color,
            linewidth=1.1, alpha=0.9, zorder=zorder)


def _aggregate_samples(samples):
    cells = {}
    for sample in samples:
        key = (sample["x"], sample["y"])
        prev = cells.get(key)
        if prev is None or _STATUS_RANK[sample["status"]] > _STATUS_RANK[prev["status"]]:
            cells[key] = sample
    return list(cells.values())


def _draw_stage_scan(ax, analysis):
    samples = _aggregate_samples(analysis["stage_scan"]["samples"])
    drawn = set()
    for status in ("pose_blocked", "box_blocked", "insert_blocked", "certified"):
        pts = [s for s in samples if s["status"] == status]
        if not pts:
            continue
        style = _STATUS_STYLE[status]
        ax.scatter(
            [p["y"] for p in pts],
            [p["x"] for p in pts],
            s=36 if status == "certified" else 24,
            c=style["color"],
            alpha=style["alpha"],
            marker="s",
            label=style["label"] if status not in drawn else None,
            zorder=4 if status != "certified" else 6,
        )
        drawn.add(status)


def _apply_scene_axes(ax, all_xs, all_ys, title=None):
    margin = 0.7
    ax.set_xlim(min(all_ys) - margin, max(all_ys) + margin)
    ax.set_ylim(min(min(all_xs) - margin, 1.0), max(all_xs) + margin)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, linewidth=0.5)
    ax.set_xlabel("Y (lateral, m)")
    ax.set_ylabel("X (forward, m)")
    if title:
        ax.set_title(title, fontsize=12, fontweight="bold")


def _collect_scene_bounds(case, analysis=None):
    all_xs = [case["x"], RS_GOAL_X, WALL_X_MIN, WALL_X_MAX]
    all_ys = [case["y"], RS_GOAL_Y, WALL_Y_MIN, WALL_Y_MAX]
    for ob in case["obstacles"]:
        all_xs.extend([ob["x"], ob["x"] + ob["w"]])
        all_ys.extend([ob["y"], ob["y"] + ob["h"]])
    if analysis:
        best = analysis["terminal"]["best_certificate"]
        if best:
            all_xs.extend([best["stage_box"]["min_x"], best["stage_box"]["max_x"]])
            all_ys.extend([best["stage_box"]["min_y"], best["stage_box"]["max_y"]])
    return all_xs, all_ys


def _draw_base_scene(ax, case, *, show_title=True):
    sx, sy, sth = case["x"], case["y"], case["th"]
    obstacles = case["obstacles"]

    wall = mpatches.Rectangle(
        (WALL_Y_MIN, WALL_X_MIN),
        WALL_Y_MAX - WALL_Y_MIN,
        WALL_X_MAX - WALL_X_MIN,
        linewidth=1.0, edgecolor="#888", facecolor="#d9d9d9", alpha=0.32,
        label="Wall/corridor",
    )
    ax.add_patch(wall)

    for i, ob in enumerate(obstacles):
        rect = mpatches.Rectangle(
            (ob["y"], ob["x"]), ob["h"], ob["w"],
            linewidth=1.4, edgecolor="#c62828", facecolor="#ff000020",
            label="Obstacle" if i == 0 else None,
            zorder=3,
        )
        ax.add_patch(rect)

    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold",
            markersize=11, markeredgecolor="orange", markeredgewidth=1.5,
            label="Goal", zorder=10)

    _draw_multi_circle_vehicle(
        ax, sx, sy, sth, color="#1976d2", alpha=0.22,
        label=f"Start ({len(primitives.VEHICLE_CHECK_OFFSETS)}-circle)", zorder=8,
    )
    ax.plot(sy, sx, "o", color="#1976d2", markersize=7, zorder=10)
    arrow_len = 0.52
    dx = -arrow_len * math.sin(sth)
    dy = -arrow_len * math.cos(sth)
    ax.annotate("", xy=(sy + dx, sx + dy), xytext=(sy, sx),
                arrowprops=dict(arrowstyle="-|>", color="#0d47a1", lw=2.0))

    title = None
    if show_title:
        title = f"{case['label']} | Problem Setup"
    all_xs, all_ys = _collect_scene_bounds(case)
    _apply_scene_axes(ax, all_xs, all_ys, title=title)


def _draw_scene(ax, case, analysis):
    sx, sy = case["x"], case["y"]
    _draw_base_scene(ax, case, show_title=False)

    bounds = analysis["stage_scan"]["bounds"]
    scan_rect = mpatches.Rectangle(
        (-bounds["stage_y_max"], bounds["stage_x_min"]),
        2.0 * bounds["stage_y_max"],
        bounds["stage_x_max"] - bounds["stage_x_min"],
        linewidth=1.2, linestyle="--", edgecolor="#1565c0",
        facecolor="#1565c008", label="Stage scan box", zorder=2,
    )
    ax.add_patch(scan_rect)

    _draw_stage_scan(ax, analysis)

    best = analysis["terminal"]["best_certificate"]
    best_traj = analysis["stage_scan"].get("best_insert_traj")
    if best:
        bx, by, bdeg = best["stage_pose"]
        bth = math.radians(bdeg)
        box = best["stage_box"]
        box_rect = mpatches.Rectangle(
            (box["min_y"], box["min_x"]),
            box["max_y"] - box["min_y"],
            box["max_x"] - box["min_x"],
            linewidth=1.5, edgecolor="#2e7d32", facecolor="#2e7d3218",
            label="Best staging box", zorder=5,
        )
        ax.add_patch(box_rect)
        _draw_multi_circle_vehicle(ax, bx, by, bth, color="#2e7d32", alpha=0.18, zorder=7)
        ax.plot(by, bx, "*", color="#2e7d32", markersize=14, zorder=11, label="Best stage pose")
        ax.plot([sy, by], [sx, bx], "--", color="#1976d2", linewidth=1.0, alpha=0.65,
                label="Start to stage (2D hint)", zorder=4)
    if best_traj:
        ax.plot(
            [pt[1] for pt in best_traj],
            [pt[0] for pt in best_traj],
            "-", color="#8e24aa", linewidth=2.2, label="Certified insert path", zorder=9,
        )

    title = (
        f"{case['label']} | {analysis['verdict']} | "
        f"cert={analysis['terminal']['certificate_count']}/{analysis['terminal']['candidate_count']}"
    )
    all_xs, all_ys = _collect_scene_bounds(case, analysis)
    _apply_scene_axes(ax, all_xs, all_ys, title=title)


def _legend_entries(ax):
    handles, labels = ax.get_legend_handles_labels()
    uniq_handles = []
    uniq_labels = []
    seen = set()
    for handle, label in zip(handles, labels):
        if not label or label in seen:
            continue
        seen.add(label)
        uniq_handles.append(handle)
        uniq_labels.append(label)
    return uniq_handles, uniq_labels


def _draw_legend_panel(ax, source_ax):
    handles, labels = _legend_entries(source_ax)
    ax.axis("off")
    if not handles:
        return
    ax.legend(
        handles,
        labels,
        loc="upper left",
        fontsize=10,
        frameon=True,
        borderaxespad=0.0,
    )


def _format_notes(case, analysis):
    term = analysis["terminal"]
    best = term["best_certificate"]
    rationale = textwrap.fill(
        analysis["rationale"],
        width=46,
        initial_indent="Rationale: ",
        subsequent_indent="           ",
    )
    lines = [
        f"Start: ({case['x']:.2f}, {case['y']:.2f}, {math.degrees(case['th']):.1f} deg)",
        f"Verdict: {analysis['verdict']}",
        f"Solvable: {analysis['solvable']}",
        rationale,
        "",
        f"Goal pose valid: {term['goal_pose_valid']} ({term['goal_pose_reason']})",
        f"Candidates: {term['candidate_count']}",
        f"Aligned free poses: {term['aligned_free_pose_count']}",
        f"Local box clear: {term['local_box_clear_count']}",
        f"Certified inserts: {term['certificate_count']}",
    ]
    if best:
        lines.extend([
            "",
            f"Best stage pose: {best['stage_pose']}",
            f"Insert length: {best['insert_path_len']} m",
            f"2D reachable: {best.get('start_reachable_2d')}",
            f"2D dist: {best.get('start_to_stage_2d_dist')}",
        ])
    reason_counts = term["reason_counts"]
    if reason_counts:
        lines.append("")
        lines.append("Reason counts:")
        for key, value in sorted(reason_counts.items(), key=lambda kv: (-kv[1], kv[0]))[:8]:
            lines.append(f"  {key}: {value}")
    return "\n".join(lines)


def _load_demo_case(name):
    if name == "terminal_blocked":
        return {
            "label": "demo_terminal_blocked",
            "x": 4.40,
            "y": 0.25,
            "th": math.radians(-20.0),
            "obstacles": [
                {"x": 1.85, "y": -0.35, "w": 0.90, "h": 0.90},
            ],
        }
    raise ValueError(f"Unknown demo case: {name}")


def _resolve_case(args):
    if args.demo:
        return _load_demo_case(args.demo)
    if args.case_id is not None:
        case = _load_case_from_generated_pool(args.profile, args.seed, args.case_id)
        return {
            "label": f"case{case['id']}_{case['type']}",
            "x": case["x"],
            "y": case["y"],
            "th": case["th"],
            "obstacles": case["obstacles"],
        }
    if args.x is None or args.y is None or args.deg is None or not args.obs_json:
        raise SystemExit("Provide --demo, --case-id, or all of --x --y --deg --obs-json.")
    return {
        "label": args.name or "custom_case",
        "x": args.x,
        "y": args.y,
        "th": math.radians(args.deg),
        "obstacles": _load_obstacles_from_json(args.obs_json),
    }


def _build_parser():
    p = argparse.ArgumentParser(description="Visualize solvability oracle stage scan.")
    p.add_argument("--case-id", type=int)
    p.add_argument("--profile", default="standard")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--demo", choices=["terminal_blocked"])
    p.add_argument("--x", type=float)
    p.add_argument("--y", type=float)
    p.add_argument("--deg", type=float)
    p.add_argument("--obs-json", type=str)
    p.add_argument("--name", type=str)
    p.add_argument("--out-dir", default="docs/solvability_oracle_viz")
    return p


def _safe_stem(label):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", label).strip("_") or "oracle_case"


def _save_raw_scene(case, out_dir, stem):
    path = os.path.join(out_dir, f"{stem}_raw.png")
    fig = plt.figure(figsize=(8.6, 7.0))
    ax = fig.add_subplot(1, 1, 1)
    _draw_base_scene(ax, case, show_title=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return path


def _save_annotated_scene(case, analysis, out_dir, stem):
    path = os.path.join(out_dir, f"{stem}_annotated.png")
    legacy_path = os.path.join(out_dir, f"{stem}.png")
    fig = plt.figure(figsize=(16.8, 7.0))
    gs = fig.add_gridspec(1, 3, width_ratios=[1.28, 1.08, 0.72])
    ax = fig.add_subplot(gs[0, 0])
    _draw_scene(ax, case, analysis)

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.axis("off")
    ax2.text(
        0.02, 0.98, _format_notes(case, analysis),
        va="top", ha="left", family="monospace", fontsize=10.5,
        bbox=dict(boxstyle="round,pad=0.5", facecolor="#f8f8f8", edgecolor="#cccccc"),
    )
    ax3 = fig.add_subplot(gs[0, 2])
    _draw_legend_panel(ax3, ax)

    fig.suptitle("Solvability Oracle Visualization", fontsize=16, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=160, bbox_inches="tight")
    fig.savefig(legacy_path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return path, legacy_path


def main(argv=None):
    args = _build_parser().parse_args(argv)
    case = _resolve_case(args)
    analysis = analyze_problem_solvability(case["x"], case["y"], case["th"], case["obstacles"])

    os.makedirs(args.out_dir, exist_ok=True)
    stem = _safe_stem(case["label"])
    json_path = os.path.join(args.out_dir, f"{stem}.json")
    raw_path = _save_raw_scene(case, args.out_dir, stem)
    annotated_path, legacy_path = _save_annotated_scene(case, analysis, args.out_dir, stem)

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(analysis, f, indent=2, ensure_ascii=False)

    print(raw_path)
    print(annotated_path)
    print(legacy_path)
    print(json_path)


if __name__ == "__main__":
    main()
