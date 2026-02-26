#!/usr/bin/env python3
"""
viz_stress_hard_cases.py — 找出压力测试中最困难但成功的 case，可视化其完整路径

Phase 1: 生成压力测试 case（复用 test_stress 的 generate_cases）
Phase 2: 并行运行规划器，收集成功 case 的统计指标
Phase 3: 按复合难度评分选出 Top-N 最难成功 case
Phase 4: 重跑选中 case，获取完整轨迹
Phase 5: matplotlib 可视化：场景 + A*/RS 路径 + 指标面板

用法:
  cd scripts && python viz_stress_hard_cases.py [--profile quick] [--top 15] [--workers 8]
"""

import os
import sys
import math
import time
import json
import signal
import argparse
import multiprocessing as mp

# ── Path setup ──
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(SCRIPT_DIR, "tests"))

from test_stress import generate_cases  # noqa: E402
from primitives import (init_primitives, simulate_path,  # noqa: E402
                        RS_GOAL_X, RS_GOAL_Y)
from planner_obs import _preprocess_obstacles  # noqa: E402
from planner_obs_v2 import plan_path_robust_obs_v2  # noqa: E402

import matplotlib  # noqa: E402
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import matplotlib.patches as mpatches  # noqa: E402
import numpy as np  # noqa: E402

# ── 场景常量 ──
WALL_X_MIN, WALL_X_MAX = 1.92, 3.0
WALL_Y_MIN, WALL_Y_MAX = -3.0, 3.0


# ============================================================================
# Worker
# ============================================================================
g_prims = None


def _worker_init():
    global g_prims
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    g_prims = init_primitives()


class _Timeout(Exception):
    pass


def _timeout_handler(signum, frame):
    raise _Timeout()


def _stats_worker(case):
    """运行规划器并收集统计数据（不返回完整轨迹以减少 IPC 开销）"""
    global g_prims
    x, y, th = case["x"], case["y"], case["th"]
    obstacles = case["obstacles"]
    expect_fail = case.get("expect_fail", False)

    stats = {}
    ok = False
    timed_out = False
    acts = None
    rs_traj = None

    old_handler = signal.getsignal(signal.SIGALRM)
    t0 = time.perf_counter()
    try:
        signal.signal(signal.SIGALRM, _timeout_handler)
        signal.alarm(20)
        ok, acts, rs_traj = plan_path_robust_obs_v2(
            x, y, th, g_prims,
            use_rs=True, stats=stats, obstacles=obstacles,
            _time_budget=18.0,
        )
    except _Timeout:
        timed_out = True
        ok = False
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)

    elapsed_ms = (time.perf_counter() - t0) * 1000.0

    result = {
        "id": case["id"],
        "type": case["type"],
        "x": x, "y": y, "th": th,
        "obstacles": obstacles,
        "ok": ok,
        "timed_out": timed_out,
        "expect_fail": expect_fail,
        "elapsed_ms": elapsed_ms,
        "expanded": stats.get("expanded", 0),
        "level": stats.get("level", ""),
    }

    if ok and not expect_fail:
        # 换挡次数
        shifts = 0
        if acts:
            last_g = acts[0][0]
            for a in acts[1:]:
                if a[0] != last_g:
                    shifts += 1
                    last_g = a[0]
        result["gear_shifts"] = shifts
        result["n_acts"] = len(acts) if acts else 0

        # 路径长度
        try:
            traj = simulate_path(x, y, th, acts or [], g_prims)
            if rs_traj:
                traj.extend(rs_traj[1:] if traj else rs_traj)
            path_len = sum(
                math.hypot(traj[i][0] - traj[i - 1][0],
                           traj[i][1] - traj[i - 1][1])
                for i in range(1, len(traj))
            )
            result["path_length"] = path_len
            result["n_traj_pts"] = len(traj)
        except Exception:
            result["path_length"] = 0.0
            result["n_traj_pts"] = 0

    return result


# ============================================================================
# 难度评分
# ============================================================================

def _difficulty_score(r):
    """复合难度评分（值越大越难）"""
    score = 0.0

    # 1) 耗时（权重 40%）— 越接近 18s 越难
    score += min(r.get("elapsed_ms", 0) / 18000.0, 1.0) * 40

    # 2) 障碍物数量（权重 15%）
    n_obs = len(r.get("obstacles", []))
    score += min(n_obs / 4.0, 1.0) * 15

    # 3) 换挡次数（权重 15%）
    score += min(r.get("gear_shifts", 0) / 8.0, 1.0) * 15

    # 4) 绕路比（权重 15%）
    euclidean = math.hypot(r["x"] - RS_GOAL_X, r["y"] - RS_GOAL_Y)
    if euclidean > 0.5:
        detour = r.get("path_length", 0) / euclidean
        score += min(detour / 5.0, 1.0) * 15

    # 5) 起始朝向偏离（权重 15%）
    heading_to_goal = math.atan2(RS_GOAL_Y - r["y"], RS_GOAL_X - r["x"])
    heading_diff = abs(((r["th"] - heading_to_goal + math.pi)
                        % (2 * math.pi)) - math.pi)
    score += (heading_diff / math.pi) * 15

    return score


# ============================================================================
# 重跑 case 获取完整轨迹
# ============================================================================

def _rerun_case(case_info, prims):
    """在主进程重跑一个 case，获取完整轨迹"""
    x, y, th = case_info["x"], case_info["y"], case_info["th"]
    obstacles = case_info["obstacles"]
    stats = {}

    ok, acts, rs_traj = plan_path_robust_obs_v2(
        x, y, th, prims,
        use_rs=True, stats=stats, obstacles=obstacles,
        _time_budget=25.0,  # 重跑时给充裕时间
    )

    if ok:
        traj_astar = (simulate_path(x, y, th, acts or [], prims)
                      if acts else [(x, y, th)])
        full_traj = list(traj_astar)
        if rs_traj:
            full_traj.extend(rs_traj[1:] if full_traj else rs_traj)
        return {
            "ok": True,
            "traj_astar": traj_astar,
            "rs_traj": rs_traj,
            "full_traj": full_traj,
            "level": stats.get("level", ""),
            "expanded": stats.get("expanded", 0),
        }
    return {"ok": False}


# ============================================================================
# 绘图
# ============================================================================

def _draw_hard_case(ax, case_info, traj_data, show_title=True, compact=False):
    """在 ax 上绘制一个困难 case 的场景 + 路径"""
    sx, sy, sth = case_info["x"], case_info["y"], case_info["th"]
    obstacles = case_info.get("obstacles", [])
    case_id = case_info.get("id", "?")
    elapsed = case_info.get("elapsed_ms", 0)

    # ── 墙壁 ──
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
        if not compact:
            cx, cy = ox + ow / 2, oy + oh / 2
            ax.text(cy, cx, f"{ow:.1f}x{oh:.1f}", fontsize=6,
                    ha="center", va="center", color="#aa0000", fontweight="bold")

    # ── 目标点 ──
    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold",
            markersize=10 if not compact else 7,
            markeredgecolor="orange", markeredgewidth=1.5, zorder=10,
            label="Goal" if not compact else None)

    # ── 完整轨迹 ──
    if traj_data and traj_data.get("ok"):
        # A* 段 (蓝色)
        traj_astar = traj_data.get("traj_astar", [])
        if len(traj_astar) > 1:
            ax.plot([p[1] for p in traj_astar],
                    [p[0] for p in traj_astar],
                    "-", color="#2196F3", linewidth=2.0, alpha=0.8, zorder=5,
                    label="A* path" if not compact else None)

        # RS 段 (紫色)
        rs_traj = traj_data.get("rs_traj", [])
        if rs_traj and len(rs_traj) > 1:
            ax.plot([p[1] for p in rs_traj],
                    [p[0] for p in rs_traj],
                    "-", color="#9C27B0", linewidth=2.0, alpha=0.8, zorder=5,
                    label="RS path" if not compact else None)

        # 朝向箭头（沿轨迹每 N 个点画一个）
        full_traj = traj_data.get("full_traj", [])
        n_arrows = 8 if compact else 15
        step = max(1, len(full_traj) // n_arrows)
        al = 0.15 if compact else 0.25
        for idx in range(0, len(full_traj), step):
            px, py, pth = full_traj[idx]
            dx = -al * math.cos(pth)
            dy = -al * math.sin(pth)
            ax.annotate(
                "", xy=(py + dy, px + dx), xytext=(py, px),
                arrowprops=dict(arrowstyle="-|>", color="#666",
                                lw=0.8, mutation_scale=8),
            )

    # ── 起点 + 朝向箭头 ──
    arrow_len = 0.4 if not compact else 0.3
    dx_a = -arrow_len * math.cos(sth)
    dy_a = -arrow_len * math.sin(sth)
    ax.annotate(
        "", xy=(sy + dy_a, sx + dx_a), xytext=(sy, sx),
        arrowprops=dict(arrowstyle="-|>", color="#00aa00", lw=2.0),
    )
    ax.plot(sy, sx, "o", color="#00cc00",
            markersize=9 if not compact else 6,
            markeredgecolor="#006600", markeredgewidth=1.0, zorder=10,
            label="Start" if not compact else None)

    # ── 碰撞半径圆 (与 collision.py VEHICLE_RADIUS=0.1 一致) ──
    collision_circle = mpatches.Circle(
        (sy, sx), 0.1,
        linewidth=1.2, edgecolor="#00aa00", facecolor="#00cc0030",
        zorder=9, label="Collision R=0.1m" if not compact else None,
    )
    ax.add_patch(collision_circle)

    # ── 范围 ──
    all_xs = [sx, RS_GOAL_X, WALL_X_MIN, WALL_X_MAX]
    all_ys = [sy, RS_GOAL_Y, WALL_Y_MIN, WALL_Y_MAX]
    for ob in obstacles:
        all_xs.extend([ob["x"], ob["x"] + ob["w"]])
        all_ys.extend([ob["y"], ob["y"] + ob["h"]])
    if traj_data and traj_data.get("full_traj"):
        for p in traj_data["full_traj"]:
            all_xs.append(p[0])
            all_ys.append(p[1])

    margin = 0.6 if not compact else 0.4
    ax.set_xlim(min(all_ys) - margin, max(all_ys) + margin)
    ax.set_ylim(min(min(all_xs) - margin, 1.2), max(all_xs) + margin)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.2, linewidth=0.5)

    if show_title:
        th_deg = math.degrees(sth)
        title = (f"#{case_id}  {case_info.get('type', '')}  |  "
                 f"({sx:.1f},{sy:.1f},{th_deg:.0f}deg)  |  "
                 f"{elapsed:.0f}ms  L:{case_info.get('level', '?')}")
        ax.set_title(title, fontsize=9 if not compact else 7,
                     fontweight="bold", color="#0055aa")

    if not compact:
        ax.set_xlabel("Y (lateral, m)", fontsize=9)
        ax.set_ylabel("X (forward, m)", fontsize=9)


def _analyze_difficulty(case):
    """分析难度来源"""
    sx, sy, sth = case["x"], case["y"], case["th"]
    obstacles = case.get("obstacles", [])

    lines = ["-- Difficulty Analysis --"]

    # 距离
    euclidean = math.hypot(sx - RS_GOAL_X, sy - RS_GOAL_Y)
    lines.append(f"Euclidean: {euclidean:.2f}m")

    # 朝向 (注意: 本系统中 th=0 表示车头朝 -x, 前进方向 = θ+π)
    heading_goal = math.degrees(math.atan2(RS_GOAL_Y - sy, RS_GOAL_X - sx))
    forward_deg = math.degrees(sth) + 180  # 前进方向 = θ + π
    h_diff = abs(((forward_deg - heading_goal + 180) % 360) - 180)
    lines.append(f"Heading diff: {h_diff:.0f} deg")
    if h_diff > 120:
        lines.append("=> Facing away!")
    elif h_diff > 60:
        lines.append("=> Large heading offset")

    # 障碍物分析
    n_blocking, n_critical = 0, 0
    for ob in obstacles:
        ox, ow = ob["x"], ob["w"]
        if ox < max(sx, 5.0) and ox + ow > RS_GOAL_X:
            n_blocking += 1
        if 2.5 < ox < 5.0 or 2.5 < ox + ow < 5.0:
            n_critical += 1
    if n_blocking:
        lines.append(f"Blocking obs: {n_blocking}")
    if n_critical:
        lines.append(f"Critical zone obs: {n_critical}")

    # 绕路比 & 换挡
    detour = case.get("path_length", 0) / max(euclidean, 0.1)
    if detour > 3.0:
        lines.append(f"High detour: {detour:.1f}x")
    shifts = case.get("gear_shifts", 0)
    if shifts > 4:
        lines.append(f"Many shifts: {shifts}")

    lines.append(f"Score: {case.get('difficulty_score', 0):.1f}")
    return "\n".join(lines)


# ── 总览大图 ──

def _plot_overview(cases, traj_list, out_dir):
    n = len(cases)
    cols = min(5, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4.5 * rows))
    if rows == 1 and cols == 1:
        axes = np.array([axes])
    axes = np.atleast_2d(axes)

    for idx, (c, td) in enumerate(zip(cases, traj_list)):
        r, co = divmod(idx, cols)
        _draw_hard_case(axes[r][co], c, td, show_title=True, compact=True)

    for idx in range(n, rows * cols):
        r, co = divmod(idx, cols)
        axes[r][co].set_visible(False)

    fig.suptitle(f"Stress Test — Top {n} Hardest Successful Cases",
                 fontsize=14, fontweight="bold", color="#0055aa", y=1.0)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    fpath = os.path.join(out_dir, "hard_cases_overview.png")
    fig.savefig(fpath, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  Overview saved: {fpath}")


# ── 逐个详图 ──

def _plot_individual(cases, traj_list, out_dir):
    for idx, (c, td) in enumerate(zip(cases, traj_list)):
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        _draw_hard_case(ax, c, td, show_title=True, compact=False)

        # 指标面板
        euclidean = math.hypot(c["x"] - RS_GOAL_X, c["y"] - RS_GOAL_Y)
        plen = c.get("path_length", 0)
        info = [
            f"Case ID:       {c.get('id', '?')}",
            f"Type:          {c.get('type', '?')}",
            f"Start:         ({c['x']:.2f}, {c['y']:.2f}, "
            f"{math.degrees(c['th']):.1f} deg)",
            f"Obstacles:     {len(c.get('obstacles', []))}",
            f"Elapsed:       {c.get('elapsed_ms', 0):.0f} ms",
            f"Level:         {c.get('level', '?')}",
            f"A* expanded:   {c.get('expanded', 0)}",
            f"Gear shifts:   {c.get('gear_shifts', 0)}",
            f"Path length:   {plen:.2f} m",
            f"Euclidean:     {euclidean:.2f} m",
            f"Detour ratio:  {plen / max(euclidean, 0.1):.2f}x",
            f"Difficulty:    {c.get('difficulty_score', 0):.1f}",
        ]
        ax.text(0.02, 0.02, "\n".join(info), transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom", fontfamily="monospace",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                          alpha=0.9))

        # 难度分析面板
        analysis = _analyze_difficulty(c)
        ax.text(0.98, 0.02, analysis, transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom",
                horizontalalignment="right", fontfamily="monospace",
                color="#0055aa",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="#eef5ff",
                          alpha=0.9))

        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()

        fpath = os.path.join(
            out_dir, f"hard_{idx + 1:02d}_case{c.get('id', '?')}.png")
        fig.savefig(fpath, dpi=150)
        plt.close(fig)
        print(f"  [{idx + 1}/{len(cases)}] saved: {fpath}")


# ============================================================================
# 主流程
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Find & visualize hardest successful stress test cases")
    parser.add_argument("--profile", default="standard",
                        choices=["quick", "standard", "thorough"])
    parser.add_argument("--top", type=int, default=15,
                        help="Number of hard cases to visualize")
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out-dir", type=str, default=None)
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = (args.out_dir
               or os.path.join(os.path.dirname(script_dir),
                               "docs", "stress_hard_cases"))
    os.makedirs(out_dir, exist_ok=True)

    # ── Phase 1: 生成 case ──
    print("=" * 60)
    print(f"  Phase 1: Generating cases (profile={args.profile})")
    print("=" * 60)
    cases = generate_cases(profile=args.profile, seed=args.seed)
    # 过滤掉 expect_fail
    runnable = [c for c in cases if not c.get("expect_fail")]
    print(f"  Total generated: {len(cases)}, runnable: {len(runnable)}\n")

    # ── Phase 2: 并行运行收集统计 ──
    print("=" * 60)
    print("  Phase 2: Running all cases (multiprocessing)")
    print("=" * 60)
    workers = args.workers or mp.cpu_count()
    print(f"  Workers: {workers}")

    pool = mp.Pool(processes=workers, initializer=_worker_init)

    success_results = []
    n_ok = n_fail = n_timeout = 0

    try:
        try:
            from tqdm import tqdm
            pbar = tqdm(total=len(runnable), desc="Running",
                        unit="case", dynamic_ncols=True)
            use_tqdm = True
        except ImportError:
            use_tqdm = False
            print("  Progress: ", end="", flush=True)

        for res in pool.imap_unordered(_stats_worker, runnable):
            if res["ok"] and not res.get("expect_fail"):
                success_results.append(res)
                n_ok += 1
            elif res["timed_out"]:
                n_timeout += 1
            else:
                n_fail += 1

            if use_tqdm:
                pbar.set_postfix(ok=n_ok, fail=n_fail, to=n_timeout)
                pbar.update(1)
            elif (n_ok + n_fail + n_timeout) % max(1, len(runnable) // 50) == 0:
                print(".", end="", flush=True)

        if use_tqdm:
            pbar.close()
        else:
            print("\n  Done.")
    except KeyboardInterrupt:
        print("\n  Interrupted!")
        pool.terminate()
        pool.join()
        sys.exit(1)

    pool.close()
    pool.join()

    print(f"\n  Results: {n_ok} success, {n_fail} fail, {n_timeout} timeout")
    print(f"  Successful with stats: {len(success_results)}\n")

    if not success_results:
        print("  No successful cases found. Exiting.")
        return

    # ── Phase 3: 选出 Top-N ──
    print("=" * 60)
    print(f"  Phase 3: Selecting top {args.top} by difficulty score")
    print("=" * 60)

    for r in success_results:
        r["difficulty_score"] = _difficulty_score(r)

    success_results.sort(key=lambda r: r["difficulty_score"], reverse=True)
    top_cases = success_results[:args.top]

    print()
    for i, c in enumerate(top_cases):
        print(f"  {i + 1:2d}. ID={c['id']:5d} | {c['type']:<25s} | "
              f"{c['elapsed_ms']:7.0f}ms | shifts={c.get('gear_shifts', 0)} | "
              f"len={c.get('path_length', 0):.1f}m | "
              f"score={c['difficulty_score']:.1f}")

    # ── Phase 4: 重跑获取轨迹 ──
    print(f"\n{'=' * 60}")
    print(f"  Phase 4: Re-running {len(top_cases)} cases for trajectories")
    print("=" * 60)

    prims = init_primitives()
    traj_data_list = []

    for i, case in enumerate(top_cases):
        print(f"  [{i + 1}/{len(top_cases)}] "
              f"Re-running case #{case['id']}...", end=" ", flush=True)
        t0 = time.perf_counter()
        td = _rerun_case(case, prims)
        el = (time.perf_counter() - t0) * 1000
        status = "OK" if td["ok"] else "FAIL"
        n_pts = len(td.get("full_traj", [])) if td["ok"] else 0
        print(f"{status} ({el:.0f}ms, {n_pts} pts)")
        traj_data_list.append(td)

    # ── Phase 5: 可视化 ──
    print(f"\n{'=' * 60}")
    print(f"  Phase 5: Generating visualizations")
    print(f"  Output: {out_dir}")
    print("=" * 60)

    print("\n  Generating overview...")
    _plot_overview(top_cases, traj_data_list, out_dir)

    print("\n  Generating individual plots...")
    _plot_individual(top_cases, traj_data_list, out_dir)

    # 保存 JSON
    json_path = os.path.join(out_dir, "hard_cases_data.json")
    export = []
    for c in top_cases:
        export.append({
            "id": c["id"], "type": c["type"],
            "x": c["x"], "y": c["y"], "th": c["th"],
            "obstacles": c["obstacles"],
            "elapsed_ms": round(c["elapsed_ms"], 1),
            "gear_shifts": c.get("gear_shifts", 0),
            "path_length": round(c.get("path_length", 0), 2),
            "difficulty_score": round(c["difficulty_score"], 1),
            "level": c.get("level", ""),
            "expanded": c.get("expanded", 0),
        })
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(export, f, indent=2)
    print(f"\n  Case data saved: {json_path}")

    print(f"\n{'=' * 60}")
    total_plots = len(top_cases) + 1
    print(f"  Done! {total_plots} plots saved to {out_dir}")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    mp.freeze_support()
    main()
