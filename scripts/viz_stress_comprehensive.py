#!/usr/bin/env python3
"""
viz_stress_comprehensive.py — 综合可视化压力测试结果

从压力测试中选出三类代表性 case 进行可视化：
  1) 成功但困难的 case（Top-10，按难度评分排序）
  2) 超时 case（从 JSON 加载，最多 10 个）
  3) 意外失败 case（规划器返回失败但起点/终点均可达，最多 10 个）

用法:
  cd scripts && python viz_stress_comprehensive.py [--profile standard] [--workers 4]
"""

import os
import sys
import math
import time
import json
import signal
import glob
import argparse
import multiprocessing as mp

# ── Path setup ──
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(SCRIPT_DIR, "tests"))

from test_stress import generate_cases  # noqa: E402
import primitives  # noqa: E402
from primitives import (init_primitives, simulate_path,  # noqa: E402
                        RS_GOAL_X, RS_GOAL_Y)
from planner_obs import _preprocess_obstacles  # noqa: E402
from planner_obs_v2 import plan_path_robust_obs_v2  # noqa: E402
from collision import check_collision  # noqa: E402

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


def _run_worker(case):
    """运行单个 case 并返回完整结果（含轨迹）"""
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

    # 成功 case 计算额外指标
    if ok and not expect_fail:
        shifts = 0
        if acts:
            last_g = acts[0][0]
            for a in acts[1:]:
                if a[0] != last_g:
                    shifts += 1
                    last_g = a[0]
        result["gear_shifts"] = shifts
        result["n_acts"] = len(acts) if acts else 0

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

        # 保存轨迹数据（用于可视化）
        if acts:
            try:
                traj_astar = simulate_path(x, y, th, acts, g_prims)
            except Exception:
                traj_astar = [(x, y, th)]
        else:
            traj_astar = [(x, y, th)]

        full_traj = list(traj_astar)
        if rs_traj:
            full_traj.extend(rs_traj[1:] if full_traj else rs_traj)

        result["traj_astar"] = traj_astar
        result["rs_traj"] = rs_traj or []
        result["full_traj"] = full_traj

    return result


# ============================================================================
# 难度评分
# ============================================================================

def _difficulty_score(r):
    """复合难度评分（值越大越难）"""
    score = 0.0
    score += min(r.get("elapsed_ms", 0) / 18000.0, 1.0) * 40
    n_obs = len(r.get("obstacles", []))
    score += min(n_obs / 4.0, 1.0) * 15
    score += min(r.get("gear_shifts", 0) / 8.0, 1.0) * 15
    euclidean = math.hypot(r["x"] - RS_GOAL_X, r["y"] - RS_GOAL_Y)
    if euclidean > 0.5:
        detour = r.get("path_length", 0) / euclidean
        score += min(detour / 5.0, 1.0) * 15
    heading_to_goal = math.atan2(RS_GOAL_Y - r["y"], RS_GOAL_X - r["x"])
    heading_diff = abs(((r["th"] - heading_to_goal + math.pi)
                        % (2 * math.pi)) - math.pi)
    score += (heading_diff / math.pi) * 15
    return score


# ============================================================================
# 绘图工具
# ============================================================================

def _draw_multi_circle_vehicle(ax, cx, cy, cth, color="#00aa00", alpha=0.25,
                                label=None, zorder=8):
    """绘制多圆碰撞模型车身"""
    cos_th = math.cos(cth)
    sin_th = math.sin(cth)
    offsets = primitives.VEHICLE_CHECK_OFFSETS
    half_w = primitives.VEHICLE_HALF_WIDTH

    for i, offset in enumerate(offsets):
        wx = cx + offset * cos_th
        wy = cy + offset * sin_th
        circle = mpatches.Circle(
            (wy, wx), half_w,
            linewidth=0.8, edgecolor=color, facecolor=color,
            alpha=alpha, zorder=zorder,
            label=label if i == 0 else None,
        )
        ax.add_patch(circle)

    front_off = min(offsets)
    rear_off = max(offsets)
    ax.plot([cy + rear_off * sin_th, cy + front_off * sin_th],
            [cx + rear_off * cos_th, cx + front_off * cos_th],
            "-", color=color, linewidth=1.0, alpha=0.5, zorder=zorder - 1)


def _draw_scene(ax, case_info, traj_data=None, fail_type=None,
                show_title=True, compact=False):
    """通用场景绘制函数"""
    if isinstance(case_info.get("start"), dict):
        sx = case_info["start"]["x"]
        sy = case_info["start"]["y"]
        sth = case_info["start"]["th"]
    else:
        sx, sy, sth = case_info["x"], case_info["y"], case_info["th"]

    obstacles = case_info.get("obstacles", [])
    case_id = case_info.get("id", case_info.get("case_id", "?"))
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
            cx_ob = ox + ow / 2
            cy_ob = oy + oh / 2
            ax.text(cy_ob, cx_ob, f"{ow:.1f}×{oh:.1f}", fontsize=6,
                    ha="center", va="center", color="#aa0000", fontweight="bold")

    # ── 目标点 ──
    ax.plot(RS_GOAL_Y, RS_GOAL_X, "s", color="gold",
            markersize=10 if not compact else 7,
            markeredgecolor="orange", markeredgewidth=1.5, zorder=10,
            label="Goal" if not compact else None)

    # ── 轨迹 ──
    if traj_data:
        traj_astar = traj_data.get("traj_astar", [])
        if len(traj_astar) > 1:
            ax.plot([p[1] for p in traj_astar],
                    [p[0] for p in traj_astar],
                    "-", color="#2196F3", linewidth=2.0, alpha=0.8, zorder=5,
                    label="A* path" if not compact else None)

        rs_traj = traj_data.get("rs_traj", [])
        if rs_traj and len(rs_traj) > 1:
            ax.plot([p[1] for p in rs_traj],
                    [p[0] for p in rs_traj],
                    "-", color="#9C27B0", linewidth=2.0, alpha=0.8, zorder=5,
                    label="RS path" if not compact else None)

        # 朝向箭头
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

    # ── 起点 + 多圆车身 ──
    _draw_multi_circle_vehicle(
        ax, sx, sy, sth, color="#00aa00", alpha=0.25,
        label=f"Vehicle ({len(primitives.VEHICLE_CHECK_OFFSETS)}-circle)"
              if not compact else None,
    )

    arrow_len = 0.4 if not compact else 0.3
    dx_a = -arrow_len * math.cos(sth)
    dy_a = -arrow_len * math.sin(sth)
    ax.annotate(
        "", xy=(sy + dy_a, sx + dx_a), xytext=(sy, sx),
        arrowprops=dict(arrowstyle="-|>", color="#006600", lw=2.0),
    )
    ax.plot(sy, sx, "o", color="#00cc00",
            markersize=9 if not compact else 6,
            markeredgecolor="#006600", markeredgewidth=1.0, zorder=10,
            label="Start" if not compact else None)

    # ── 到目标的辅助直线 ──
    if fail_type:
        ax.plot([sy, RS_GOAL_Y], [sx, RS_GOAL_X], "--", color="#aaaaaa",
                linewidth=0.8, alpha=0.5)

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
        if fail_type == "TIMEOUT":
            status = f"⏱ TIMEOUT {elapsed:.0f}ms"
            title_color = "#cc6600"
        elif fail_type == "UNEXPECTED_FAIL":
            status = f"⚠ UNEXPECTED FAIL {elapsed:.0f}ms"
            title_color = "#cc0000"
        else:
            status = f"✅ {elapsed:.0f}ms  L:{case_info.get('level', '?')}"
            title_color = "#0055aa"
        n_obs = len(obstacles)
        title = (f"#{case_id}  {case_info.get('type', '')}  |  "
                 f"({sx:.1f},{sy:.1f},{th_deg:.0f}°)  |  "
                 f"{n_obs}obs  |  {status}")
        ax.set_title(title, fontsize=9 if not compact else 7,
                     fontweight="bold", color=title_color)

    if not compact:
        ax.set_xlabel("Y (lateral, m)", fontsize=9)
        ax.set_ylabel("X (forward, m)", fontsize=9)


def _analyze_case(case, category):
    """分析 case 的特征"""
    if isinstance(case.get("start"), dict):
        sx, sy, sth = case["start"]["x"], case["start"]["y"], case["start"]["th"]
    else:
        sx, sy, sth = case["x"], case["y"], case["th"]
    obstacles = case.get("obstacles", [])

    lines = [f"── Analysis ({category}) ──"]
    euclid = math.hypot(sx - RS_GOAL_X, sy - RS_GOAL_Y)
    lines.append(f"Euclidean: {euclid:.2f}m")

    forward_deg = math.degrees(sth) + 180
    heading_to_goal = math.degrees(math.atan2(RS_GOAL_Y - sy, RS_GOAL_X - sx))
    h_diff = abs(((forward_deg - heading_to_goal + 180) % 360) - 180)
    lines.append(f"Heading diff: {h_diff:.0f}°")
    if h_diff > 120:
        lines.append(">> Facing away!")
    elif h_diff > 60:
        lines.append(">> Large heading offset")

    blocking, critical = 0, 0
    for ob in obstacles:
        ox, ow = ob["x"], ob["w"]
        if ox < max(sx, 5.0) and ox + ow > RS_GOAL_X:
            blocking += 1
        if 2.5 < ox < 5.0 or 2.5 < ox + ow < 5.0:
            critical += 1
    if blocking:
        lines.append(f"Blocking obs: {blocking}")
    if critical:
        lines.append(f"Critical zone: {critical}")

    if category == "SUCCESS":
        plen = case.get("path_length", 0)
        detour = plen / max(euclid, 0.1)
        if detour > 3.0:
            lines.append(f"High detour: {detour:.1f}x")
        shifts = case.get("gear_shifts", 0)
        if shifts > 4:
            lines.append(f"Many shifts: {shifts}")
        lines.append(f"Score: {case.get('difficulty_score', 0):.1f}")

    if sx < 4.0:
        lines.append(">> Start close to wall")
    if abs(sy) > 2.0:
        lines.append(f">> Large y offset ({sy:.1f}m)")

    return "\n".join(lines)


# ============================================================================
# 可视化生成
# ============================================================================

def _plot_category_overview(cases, traj_list, fail_types, category_name,
                            out_dir, color):
    """生成某一类 case 的总览大图"""
    n = len(cases)
    if n == 0:
        return
    cols = min(5, n)
    rows = math.ceil(n / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4.5 * rows))
    if rows == 1 and cols == 1:
        axes = np.array([axes])
    axes = np.atleast_2d(axes)

    for idx in range(n):
        r, co = divmod(idx, cols)
        ft = fail_types[idx] if fail_types else None
        td = traj_list[idx] if traj_list else None
        _draw_scene(axes[r][co], cases[idx], traj_data=td,
                    fail_type=ft, show_title=True, compact=True)

    for idx in range(n, rows * cols):
        r, co = divmod(idx, cols)
        axes[r][co].set_visible(False)

    fig.suptitle(f"{category_name} ({n} cases)",
                 fontsize=14, fontweight="bold", color=color, y=1.0)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    safe_name = category_name.lower().replace(" ", "_").replace("/", "_")
    fpath = os.path.join(out_dir, f"{safe_name}_overview.png")
    fig.savefig(fpath, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"    Overview saved: {fpath}")


def _plot_category_individual(cases, traj_list, fail_types, category_name,
                              out_dir, prefix):
    """生成某一类 case 的逐个详图"""
    for idx in range(len(cases)):
        c = cases[idx]
        ft = fail_types[idx] if fail_types else None
        td = traj_list[idx] if traj_list else None

        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
        _draw_scene(ax, c, traj_data=td, fail_type=ft,
                    show_title=True, compact=False)

        # 信息面板
        if isinstance(c.get("start"), dict):
            sx, sy, sth = c["start"]["x"], c["start"]["y"], c["start"]["th"]
        else:
            sx, sy, sth = c["x"], c["y"], c["th"]

        case_id = c.get("id", c.get("case_id", "?"))
        obstacles = c.get("obstacles", [])
        elapsed = c.get("elapsed_ms", 0)
        euclid = math.hypot(sx - RS_GOAL_X, sy - RS_GOAL_Y)

        info = [
            f"Case ID:     {case_id}",
            f"Type:        {c.get('type', '?')}",
            f"Category:    {category_name}",
            f"Start:       ({sx:.2f}, {sy:.2f}, {math.degrees(sth):.1f}°)",
            f"Goal:        ({RS_GOAL_X}, {RS_GOAL_Y})",
            f"Obstacles:   {len(obstacles)}",
            f"Elapsed:     {elapsed:.0f} ms",
            f"A* expanded: {c.get('expanded', 0)}",
            f"Level:       {c.get('level', '') or 'N/A'}",
            f"Euclidean:   {euclid:.2f} m",
        ]
        if ft is None:  # success
            plen = c.get("path_length", 0)
            info.extend([
                f"Path length: {plen:.2f} m",
                f"Detour:      {plen / max(euclid, 0.1):.2f}x",
                f"Gear shifts: {c.get('gear_shifts', 0)}",
                f"Difficulty:  {c.get('difficulty_score', 0):.1f}",
            ])

        for i, ob in enumerate(obstacles):
            info.append(f"  Obs-{i+1}: ({ob['x']:.2f},{ob['y']:.2f}) "
                        f"{ob['w']:.1f}×{ob['h']:.1f}")

        ax.text(0.02, 0.02, "\n".join(info), transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom", fontfamily="monospace",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="white",
                          alpha=0.9))

        # 分析面板
        cat = "SUCCESS" if ft is None else ft
        analysis = _analyze_case(c, cat)
        ax.text(0.98, 0.02, analysis, transform=ax.transAxes,
                fontsize=8, verticalalignment="bottom",
                horizontalalignment="right", fontfamily="monospace",
                color="#0055aa",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="#eef5ff",
                          alpha=0.9))

        ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout()

        fpath = os.path.join(
            out_dir, f"{prefix}_{idx + 1:02d}_case{case_id}.png")
        fig.savefig(fpath, dpi=150)
        plt.close(fig)
        print(f"    [{idx + 1}/{len(cases)}] saved: {fpath}")


# ============================================================================
# 主流程
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Comprehensive stress test visualization")
    parser.add_argument("--profile", default="standard",
                        choices=["quick", "standard", "thorough"])
    parser.add_argument("--workers", type=int, default=None)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--top-success", type=int, default=10,
                        help="Number of hard successful cases")
    parser.add_argument("--top-timeout", type=int, default=10,
                        help="Max timeout cases to show")
    parser.add_argument("--top-unexpected", type=int, default=10,
                        help="Max unexpected failure cases to show")
    parser.add_argument("--out-dir", type=str, default=None)
    parser.add_argument("--vehicle-length", type=float, default=1.5)
    parser.add_argument("--vehicle-width", type=float, default=0.5)
    args = parser.parse_args()

    primitives.configure_vehicle(args.vehicle_length, args.vehicle_width)
    print(f"  Vehicle: {args.vehicle_length}m × {args.vehicle_width}m "
          f"→ {len(primitives.VEHICLE_CHECK_OFFSETS)} circles, "
          f"r={primitives.VEHICLE_HALF_WIDTH:.2f}m")

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = (args.out_dir
               or os.path.join(os.path.dirname(script_dir),
                               "docs", "stress_analysis"))
    os.makedirs(out_dir, exist_ok=True)

    # ── Phase 1: 生成 case ──
    print("\n" + "=" * 60)
    print(f"  Phase 1: Generating cases (profile={args.profile})")
    print("=" * 60)
    cases = generate_cases(profile=args.profile, seed=args.seed)
    runnable = [c for c in cases if not c.get("expect_fail")]
    print(f"  Total: {len(cases)}, Runnable: {len(runnable)}")

    # ── Phase 2: 并行运行 ──
    print("\n" + "=" * 60)
    print("  Phase 2: Running all cases (collecting stats)")
    print("=" * 60)
    workers = args.workers or mp.cpu_count()
    print(f"  Workers: {workers}")

    pool = mp.Pool(processes=workers, initializer=_worker_init)

    success_results = []
    unexpected_results = []
    timeout_results = []
    n_ok = n_fail = n_timeout = n_expected = 0

    try:
        total = len(runnable)
        t_start = time.perf_counter()

        for i, res in enumerate(pool.imap_unordered(_run_worker, runnable)):
            if res["timed_out"]:
                n_timeout += 1
                timeout_results.append(res)
            elif res["ok"]:
                n_ok += 1
                success_results.append(res)
            elif res["expect_fail"]:
                n_expected += 1
            else:
                n_fail += 1
                unexpected_results.append(res)

            done = n_ok + n_fail + n_timeout + n_expected
            if done % max(1, total // 20) == 0 or done == total:
                elapsed = time.perf_counter() - t_start
                rate = done / max(elapsed, 0.1)
                eta = (total - done) / max(rate, 0.01)
                print(f"\r  [{done}/{total}] "
                      f"ok={n_ok} fail={n_fail} timeout={n_timeout} "
                      f"expected={n_expected} "
                      f"({elapsed:.0f}s elapsed, ETA {eta:.0f}s)",
                      end="", flush=True)

        print()  # newline after progress
    except KeyboardInterrupt:
        print("\n  Interrupted!")
        pool.terminate()
        pool.join()
        sys.exit(1)

    pool.close()
    pool.join()

    print(f"\n  ✅ Success:     {n_ok}")
    print(f"  ⚠️  Unexpected:  {n_fail}")
    print(f"  ⏱️  Timeout:     {n_timeout}")
    print(f"  🎯 Expected:    {n_expected}")

    # ── Phase 3: 选择代表性 case ──
    print("\n" + "=" * 60)
    print("  Phase 3: Selecting representative cases")
    print("=" * 60)

    # 3a. 成功但困难的 case
    for r in success_results:
        r["difficulty_score"] = _difficulty_score(r)
    success_results.sort(key=lambda r: r["difficulty_score"], reverse=True)
    top_success = success_results[:args.top_success]

    print(f"\n  Top-{len(top_success)} hardest successful cases:")
    for i, c in enumerate(top_success):
        print(f"    {i+1:2d}. #{c['id']:5d} {c['type']:<20s} "
              f"{c['elapsed_ms']:7.0f}ms shifts={c.get('gear_shifts',0)} "
              f"score={c['difficulty_score']:.1f}")

    # 3b. 超时 case
    timeout_results.sort(key=lambda r: len(r.get("obstacles", [])),
                         reverse=True)
    top_timeout = timeout_results[:args.top_timeout]

    print(f"\n  Timeout cases (showing {len(top_timeout)}/{len(timeout_results)}):")
    for i, c in enumerate(top_timeout):
        print(f"    {i+1:2d}. #{c['id']:5d} {c['type']:<20s} "
              f"{len(c.get('obstacles',[]))} obs  {c['elapsed_ms']:.0f}ms")

    # 3c. 意外失败 case — 多样化选取
    # 按障碍物数量分组，每组取最多3个
    unexpected_by_nobs = {}
    for r in unexpected_results:
        n = len(r.get("obstacles", []))
        unexpected_by_nobs.setdefault(n, []).append(r)
    top_unexpected = []
    for n_obs in sorted(unexpected_by_nobs.keys()):
        group = unexpected_by_nobs[n_obs]
        # 每组按耗时降序排
        group.sort(key=lambda r: r.get("elapsed_ms", 0), reverse=True)
        top_unexpected.extend(group[:3])
    top_unexpected = top_unexpected[:args.top_unexpected]

    print(f"\n  Unexpected failures (showing {len(top_unexpected)}/{len(unexpected_results)}):")
    for i, c in enumerate(top_unexpected):
        print(f"    {i+1:2d}. #{c['id']:5d} {c['type']:<20s} "
              f"{len(c.get('obstacles',[]))} obs  {c['elapsed_ms']:.0f}ms")

    # ── Phase 4: 可视化 ──
    print("\n" + "=" * 60)
    print(f"  Phase 4: Generating visualizations → {out_dir}")
    print("=" * 60)

    # 4a. 成功 case
    if top_success:
        print(f"\n  [SUCCESS] {len(top_success)} hard cases...")
        traj_list = []
        for c in top_success:
            traj_list.append({
                "traj_astar": c.get("traj_astar", []),
                "rs_traj": c.get("rs_traj", []),
                "full_traj": c.get("full_traj", []),
            })
        _plot_category_overview(
            top_success, traj_list, [None] * len(top_success),
            "Hard Successful Cases", out_dir, "#0055aa")
        _plot_category_individual(
            top_success, traj_list, [None] * len(top_success),
            "Hard Successful Cases", out_dir, "success")

    # 4b. 超时 case
    if top_timeout:
        print(f"\n  [TIMEOUT] {len(top_timeout)} cases...")
        _plot_category_overview(
            top_timeout, [None] * len(top_timeout),
            ["TIMEOUT"] * len(top_timeout),
            "Timeout Cases", out_dir, "#cc6600")
        _plot_category_individual(
            top_timeout, [None] * len(top_timeout),
            ["TIMEOUT"] * len(top_timeout),
            "Timeout Cases", out_dir, "timeout")

    # 4c. 意外失败 case
    if top_unexpected:
        print(f"\n  [UNEXPECTED] {len(top_unexpected)} cases...")
        _plot_category_overview(
            top_unexpected, [None] * len(top_unexpected),
            ["UNEXPECTED_FAIL"] * len(top_unexpected),
            "Unexpected Failure Cases", out_dir, "#cc0000")
        _plot_category_individual(
            top_unexpected, [None] * len(top_unexpected),
            ["UNEXPECTED_FAIL"] * len(top_unexpected),
            "Unexpected Failure Cases", out_dir, "unexpected")

    # ── 保存 JSON 数据 ──
    export = {
        "profile": args.profile,
        "total_cases": len(runnable),
        "n_success": n_ok,
        "n_unexpected": n_fail,
        "n_timeout": n_timeout,
        "n_expected": n_expected,
        "success_cases": [{
            "id": c["id"], "type": c["type"],
            "x": c["x"], "y": c["y"], "th": c["th"],
            "obstacles": c["obstacles"],
            "elapsed_ms": round(c["elapsed_ms"], 1),
            "gear_shifts": c.get("gear_shifts", 0),
            "path_length": round(c.get("path_length", 0), 2),
            "difficulty_score": round(c["difficulty_score"], 1),
            "level": c.get("level", ""),
        } for c in top_success],
        "timeout_cases": [{
            "id": c["id"], "type": c["type"],
            "x": c["x"], "y": c["y"], "th": c["th"],
            "obstacles": c["obstacles"],
            "elapsed_ms": round(c["elapsed_ms"], 1),
        } for c in top_timeout],
        "unexpected_cases": [{
            "id": c["id"], "type": c["type"],
            "x": c["x"], "y": c["y"], "th": c["th"],
            "obstacles": c["obstacles"],
            "elapsed_ms": round(c["elapsed_ms"], 1),
        } for c in top_unexpected],
    }
    json_path = os.path.join(out_dir, "analysis_data.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(export, f, indent=2)
    print(f"\n  Data saved: {json_path}")

    # ── 汇总 ──
    total_plots = (
        (len(top_success) + 1 if top_success else 0) +
        (len(top_timeout) + 1 if top_timeout else 0) +
        (len(top_unexpected) + 1 if top_unexpected else 0)
    )
    print(f"\n{'=' * 60}")
    print(f"  Done! {total_plots} plots saved to {out_dir}")
    print(f"  - Success:    {len(top_success)} + 1 overview")
    print(f"  - Timeout:    {len(top_timeout)} + 1 overview")
    print(f"  - Unexpected: {len(top_unexpected)} + 1 overview")
    print(f"{'=' * 60}")


if __name__ == "__main__":
    mp.freeze_support()
    main()
