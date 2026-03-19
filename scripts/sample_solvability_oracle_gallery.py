#!/usr/bin/env python3
import argparse
import json
import math
import multiprocessing as mp
import os
import random
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TESTS_DIR = os.path.join(SCRIPT_DIR, "tests")
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)
if TESTS_DIR not in sys.path:
    sys.path.insert(0, TESTS_DIR)

from solvability_oracle import analyze_problem_solvability, assess_problem_solvability
from test_stress import generate_cases
from viz_solvability_oracle import (
    _draw_base_scene,
    _draw_legend_panel,
    _draw_scene,
    _format_notes,
    _safe_stem,
)


_WORKER_CASES = None


def _worker_init(cases):
    global _WORKER_CASES
    _WORKER_CASES = cases


def _worker_assess(case_id):
    case = _WORKER_CASES[case_id]
    analysis = assess_problem_solvability(case["x"], case["y"], case["th"], case["obstacles"])
    return {
        "id": case_id,
        "type": case["type"],
        "verdict": analysis["verdict"],
        "solvable": analysis["solvable"],
        "certificate_count": analysis["terminal"]["certificate_count"],
    }


def _balanced_sample_ids(by_verdict, total_count, rng):
    verdicts = [v for v in sorted(by_verdict.keys()) if by_verdict[v]]
    for verdict in verdicts:
        rng.shuffle(by_verdict[verdict])

    selected = []
    for verdict in verdicts:
        if len(selected) >= total_count:
            break
        selected.append(by_verdict[verdict].pop())

    round_order = list(verdicts)
    while len(selected) < total_count and any(by_verdict[v] for v in verdicts):
        rng.shuffle(round_order)
        advanced = False
        for verdict in round_order:
            if not by_verdict[verdict]:
                continue
            selected.append(by_verdict[verdict].pop())
            advanced = True
            if len(selected) >= total_count:
                break
        if not advanced:
            break
    return selected


def _clear_previous_outputs(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    for name in os.listdir(out_dir):
        if name.startswith("oracle_sample_") and name.endswith(".png"):
            os.remove(os.path.join(out_dir, name))
        elif name in ("oracle_sample_overview.png", "selection_summary.json"):
            os.remove(os.path.join(out_dir, name))


def _plot_overview(items, out_dir):
    n = len(items)
    cols = min(4, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5.0 * cols, 4.6 * rows))
    axes = np.array(axes, ndmin=2)

    for idx, item in enumerate(items):
        r, c = divmod(idx, cols)
        ax = axes[r, c]
        _draw_scene(ax, item["case"], item["analysis"])
        ax.set_title(
            f"#{item['case']['id']} {item['analysis']['verdict']}",
            fontsize=9, fontweight="bold",
        )
        leg = ax.get_legend()
        if leg is not None:
            leg.remove()

    for idx in range(n, rows * cols):
        r, c = divmod(idx, cols)
        axes[r, c].axis("off")

    fig.suptitle("Random Solvability Oracle Gallery", fontsize=16, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    path = os.path.join(out_dir, "oracle_sample_overview.png")
    fig.savefig(path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return path


def _plot_detail(rank, item, out_dir):
    case = item["case"]
    analysis = item["analysis"]
    stem = _safe_stem(f"oracle_sample_{rank:02d}_case{case['id']}_{case['type']}")

    raw_path = os.path.join(out_dir, f"{stem}_raw.png")
    fig_raw = plt.figure(figsize=(8.6, 7.0))
    ax_raw = fig_raw.add_subplot(1, 1, 1)
    _draw_base_scene(ax_raw, case, show_title=True)
    fig_raw.tight_layout()
    fig_raw.savefig(raw_path, dpi=160, bbox_inches="tight")
    plt.close(fig_raw)

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
    fig.suptitle(
        f"Oracle Sample #{rank:02d} | case {case['id']} | {analysis['verdict']}",
        fontsize=16, fontweight="bold",
    )
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    annotated_path = os.path.join(out_dir, f"{stem}_annotated.png")
    legacy_path = os.path.join(out_dir, f"{stem}.png")
    fig.savefig(annotated_path, dpi=160, bbox_inches="tight")
    fig.savefig(legacy_path, dpi=160, bbox_inches="tight")
    plt.close(fig)
    return {
        "raw": raw_path,
        "annotated": annotated_path,
        "legacy": legacy_path,
    }


def _build_parser():
    p = argparse.ArgumentParser(description="Random stratified gallery for solvability oracle.")
    p.add_argument("--profile", default="standard")
    p.add_argument("--seed", type=int, default=42, help="Case generation seed.")
    p.add_argument("--sample-seed", type=int, default=20260319, help="Sampling seed.")
    p.add_argument("--count", type=int, default=20)
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--out-dir", default="docs/solvability_oracle_gallery_random20")
    return p


def main(argv=None):
    args = _build_parser().parse_args(argv)
    rng = random.Random(args.sample_seed)

    cases = generate_cases(args.profile, seed=args.seed)
    case_map = {case["id"]: case for case in cases}

    with mp.Pool(processes=args.workers, initializer=_worker_init, initargs=(case_map,)) as pool:
        compact = list(pool.imap_unordered(_worker_assess, case_map.keys()))

    counts = {}
    by_verdict = {}
    for item in compact:
        counts[item["verdict"]] = counts.get(item["verdict"], 0) + 1
        by_verdict.setdefault(item["verdict"], []).append(item["id"])

    chosen_ids = _balanced_sample_ids(by_verdict, args.count, rng)
    selected = []
    for cid in chosen_ids:
        case = dict(case_map[cid])
        case["label"] = f"case{case['id']}_{case['type']}"
        analysis = analyze_problem_solvability(case["x"], case["y"], case["th"], case["obstacles"])
        selected.append({"case": case, "analysis": analysis})

    _clear_previous_outputs(args.out_dir)
    overview_path = _plot_overview(selected, args.out_dir)
    detail_paths = []
    for idx, item in enumerate(selected, start=1):
        detail_paths.append(_plot_detail(idx, item, args.out_dir))

    summary = {
        "profile": args.profile,
        "seed": args.seed,
        "sample_seed": args.sample_seed,
        "total_cases": len(cases),
        "oracle_verdict_counts": counts,
        "selected_case_ids": chosen_ids,
        "selected": [
            {
                "id": item["case"]["id"],
                "type": item["case"]["type"],
                "verdict": item["analysis"]["verdict"],
                "solvable": item["analysis"]["solvable"],
                "certificate_count": item["analysis"]["terminal"]["certificate_count"],
                "raw_png": os.path.basename(path["raw"]),
                "annotated_png": os.path.basename(path["annotated"]),
                "png": os.path.basename(path["legacy"]),
            }
            for item, path in zip(selected, detail_paths)
        ],
        "overview_png": os.path.basename(overview_path),
    }
    with open(os.path.join(args.out_dir, "selection_summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(json.dumps(summary["oracle_verdict_counts"], ensure_ascii=False, indent=2))
    print(overview_path)
    print(os.path.join(args.out_dir, "selection_summary.json"))


if __name__ == "__main__":
    main()
