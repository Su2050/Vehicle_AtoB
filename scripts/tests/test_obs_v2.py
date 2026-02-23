#!/usr/bin/env python
"""
Comprehensive obstacle planning test suite — 12 scenarios.

Covers: open field, single block, long wall, narrow passage, U-trap,
        unreachable, random (3 seeds), start==goal, dead-end reverse,
        corridor straight, L-turn, force fallback.

Run from scripts/ directory:
    python tests/test_obs_v2.py [--timeout SECS] [--scenario NAME]
"""

import math
import time
import os
import sys
import signal
import random as _random
from datetime import datetime

from primitives import (
    init_primitives, M_PI, PI2, DT, ALIGN_GOAL_DYAW, MIN_TURN_RADIUS,
    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, _replay_to_end,
)
from planner_obs import plan_path_robust_obs, _preprocess_obstacles
try:
    from planner_obs_v2 import plan_path_robust_obs_v2
except ImportError:
    plan_path_robust_obs_v2 = None
from collision import check_collision, _check_traj_collision
import astar_core


class _Timeout(Exception):
    pass

def _alarm_handler(signum, frame):
    raise _Timeout()


# ═══════════════════════════════════════════════════════════════════════
# 12 Scenario Definitions
# ═══════════════════════════════════════════════════════════════════════

def _deg(d):
    return math.radians(d)

def _generate_random_obstacles(seed, n=8, corridor_y=0.0, corridor_w=1.8):
    """Generate random obstacles with a carved corridor guaranteeing connectivity."""
    rng = _random.Random(seed)
    obstacles = []
    corr_lo = corridor_y - corridor_w / 2
    corr_hi = corridor_y + corridor_w / 2
    for _ in range(n):
        x = rng.uniform(3.0, 7.5)
        y = rng.uniform(-3.0, 3.0)
        w = rng.uniform(0.3, 1.0)
        h = rng.uniform(0.3, 1.2)
        obs_y_lo, obs_y_hi = y, y + h
        if obs_y_lo < corr_hi + 0.6 and obs_y_hi > corr_lo - 0.6:
            continue
        obstacles.append({'x': x, 'y': y, 'w': w, 'h': h})
    return obstacles


SCENARIOS = {
    # ── 1. Open field: no obstacles, RS direct ──
    "S01_open_field": {
        "obstacles": [],
        "cases": [
            (2.5, 0.0, 0, "near_goal_straight"),
            (4.0, 0.5, 17, "slight_offset"),
            (5.0, -1.0, -29, "mid_dist_offset"),
            (7.0, 0.0, 0, "far_straight"),
        ],
        "expect_success": True,
        "note": "No obstacles — Level-1 Pure RS should handle most",
    },
    # ── 2. Single block blocking direct path ──
    "S02_single_block": {
        "obstacles": [
            {'x': 3.5, 'y': -0.6, 'w': 0.8, 'h': 1.2},
        ],
        "cases": [
            (5.0, 0.0, 0, "blocked_direct"),
            (5.0, 1.5, -17, "offset_upper"),
            (5.0, -1.5, 17, "offset_lower"),
            (6.0, 0.0, 180, "facing_away"),
        ],
        "expect_success": True,
        "note": "One block on direct line — must detour",
    },
    # ── 3. Long wall with gap at one end ──
    "S03_long_wall": {
        "obstacles": [
            {'x': 3.5, 'y': -3.5, 'w': 0.3, 'h': 5.0},
        ],
        "cases": [
            (5.0, 0.0, 0, "behind_wall_center"),
            (5.0, -2.0, 0, "behind_wall_lower"),
            (7.0, 2.5, 0, "near_gap_side"),
            (5.0, -2.5, 90, "wall_parallel_heading"),
        ],
        "expect_success": True,
        "note": "Wall y∈[-3.5,1.5], gap at y>1.5 — includes 90° heading parallel to wall",
    },
    # ── 4. Narrow passage between two obstacles ──
    "S04_narrow_passage": {
        "obstacles": [
            {'x': 3.5, 'y': -3.5, 'w': 0.6, 'h': 2.9},
            {'x': 3.5, 'y': 0.6, 'w': 0.6, 'h': 2.9},
        ],
        "cases": [
            (5.0, 0.0, 0, "passage_center"),
            (6.0, 0.3, -11, "passage_offset_upper"),
            (6.0, -0.3, 11, "passage_offset_lower"),
        ],
        "expect_success": True,
        "note": "Gap y∈[-0.6,0.6]=1.2m, vehicle radius 0.1m — tight passage",
    },
    # ── 5. U-shaped trap (open toward +x) ──
    "S05_u_trap": {
        "obstacles": [
            {'x': 5.0, 'y': -1.5, 'w': 0.3, 'h': 3.0},
            {'x': 5.3, 'y': -1.8, 'w': 2.7, 'h': 0.3},
            {'x': 5.3, 'y': 1.5, 'w': 2.7, 'h': 0.3},
        ],
        "cases": [
            (6.5, 0.0, 0, "u_center_facing_wall"),
            (7.0, 0.0, 180, "u_center_facing_opening"),
            (6.5, 1.0, -90, "u_upper"),
        ],
        "expect_success": True,
        "note": "U-trap x∈[5.0,8.0], y∈[-1.5,1.5], opening at x>8.0 — must escape +x then route to goal",
    },
    # ── 6. Unreachable goal ──
    "S06_unreachable": {
        "obstacles": [
            {'x': 2.4, 'y': -3.5, 'w': 0.8, 'h': 7.0},
        ],
        "cases": [
            (5.0, 0.0, 0, "direct_approach"),
            (5.0, 2.0, -29, "angled_approach"),
        ],
        "expect_success": False,
        "note": "Full-height wall x∈[2.4,3.2] blocks all access to goal — must return failure",
    },
    # ── 7a-c. Random obstacles with carved corridor (3 seeds) ──
    "S07a_random_seed42": {
        "obstacles": _generate_random_obstacles(42),
        "cases": [
            (6.0, 0.0, 0, "corridor_center"),
            (7.0, 0.3, -11, "corridor_offset"),
        ],
        "expect_success": True,
        "note": "Random (seed=42) with y=0 corridor width 1.8m — guaranteed passable",
    },
    "S07b_random_seed123": {
        "obstacles": _generate_random_obstacles(123),
        "cases": [
            (6.0, 0.0, 0, "corridor_center"),
            (7.0, -0.3, 11, "corridor_offset"),
        ],
        "expect_success": True,
        "note": "Random (seed=123) with y=0 corridor width 1.8m",
    },
    "S07c_random_seed2026": {
        "obstacles": _generate_random_obstacles(2026),
        "cases": [
            (6.0, 0.0, 0, "corridor_center"),
            (5.5, 0.5, -17, "corridor_offset"),
        ],
        "expect_success": True,
        "note": "Random (seed=2026) with y=0 corridor width 1.8m",
    },
    # ── 8. Start == Goal (degenerate) ──
    "S08_start_equals_goal": {
        "obstacles": [],
        "cases": [
            (2.10, 0.0, 0, "exact_goal"),
            (2.15, 0.05, 1, "near_goal_micro_offset"),
            (2.20, -0.10, -3, "goal_boundary"),
            (2.50, 0.0, 180, "close_reverse_180"),
        ],
        "expect_success": True,
        "note": "Start inside/near goal region — includes extreme 180° reversal at close range",
    },
    # ── 9. Dead-end corridor (must reverse out) ──
    "S09_dead_end_reverse": {
        "obstacles": [
            {'x': 5.0, 'y': -1.3, 'w': 3.5, 'h': 0.3},
            {'x': 5.0, 'y': 1.0, 'w': 3.5, 'h': 0.3},
            {'x': 8.2, 'y': -1.3, 'w': 0.3, 'h': 2.6},
        ],
        "cases": [
            (7.0, 0.0, 0, "deep_facing_dead_end"),
            (7.0, 0.0, 180, "deep_facing_opening"),
            (6.0, 0.2, 29, "mid_corridor_angled"),
        ],
        "expect_success": True,
        "note": "Corridor x∈[5.0,8.2], y∈[-1.0,1.0], dead-end at x=8.2 — must reverse to escape",
    },
    # ── 10. Corridor straight (shelf aisle) ──
    "S10_corridor_straight": {
        "obstacles": [
            {'x': 3.0, 'y': -3.5, 'w': 0.3, 'h': 2.5},
            {'x': 3.0, 'y': 1.0, 'w': 0.3, 'h': 2.5},
        ],
        "cases": [
            (5.0, 0.0, 0, "corridor_center"),
            (5.0, 0.5, -11, "corridor_upper"),
            (5.0, -0.5, 11, "corridor_lower"),
        ],
        "expect_success": True,
        "note": "Parallel shelves forming corridor y∈[-1.0,1.0] — simple straight-through",
    },
    # ── 11. L-shaped barrier (direction change required) ──
    "S11_l_barrier": {
        "obstacles": [
            {'x': 3.5, 'y': -3.5, 'w': 0.3, 'h': 4.5},
            {'x': 3.5, 'y': 2.5, 'w': 5.5, 'h': 0.3},
        ],
        "cases": [
            (6.0, 0.0, 0, "must_route_around_L"),
            (7.0, -1.0, 45, "far_lower_angled"),
        ],
        "expect_success": True,
        "note": "Vertical wall y∈[-3.5,1.0] + horizontal ceiling y=2.5 — must navigate through gap y∈[1.0,2.5]",
    },
    # ── 12. Goal covered by obstacle (instant fail) ──
    "S12_goal_covered": {
        "obstacles": [
            {'x': 1.9, 'y': -0.3, 'w': 0.5, 'h': 0.6},
        ],
        "cases": [
            (5.0, 0.0, 0, "goal_blocked"),
            (3.0, 1.0, -30, "goal_blocked_angled"),
        ],
        "expect_success": False,
        "note": "Obstacle covers goal (2.1, 0) — planner should fail instantly (<100ms)",
    },
    # ── 13. Force fallback (low expand limit) ──
    "S13_force_fallback": {
        "obstacles": [
            {'x': 4.0, 'y': -0.5, 'w': 0.6, 'h': 1.0},
        ],
        "cases": [
            (5.5, 0.0, 0, "simple_block_low_budget"),
        ],
        "expect_success": True,
        "note": "Simple obstacle but tested with very low _expand_limit to verify fallback kicks in",
        "_force_low_budget": True,
    },
}


# ═══════════════════════════════════════════════════════════════════════
# Path safety validation helpers
# ═══════════════════════════════════════════════════════════════════════

def _validate_rs_traj(rs_traj, obstacles):
    """Check RS trajectory doesn't collide with obstacles. Returns (ok, detail)."""
    if not rs_traj:
        return True, ""
    fast_obs = _preprocess_obstacles(obstacles) if obstacles else None
    for i, pt in enumerate(rs_traj):
        valid, reason = check_collision(pt[0], pt[1], pt[2],
                                        no_corridor=True, obstacles=fast_obs)
        if not valid:
            return False, f"RS traj collision at step {i}: ({pt[0]:.2f},{pt[1]:.2f},{pt[2]:.2f}) reason={reason}"
    return True, ""


def _validate_endpoint(x0, y0, th0, acts, rs_traj, prims, stats):
    """Check if the endpoint is in the goal region. Returns (ok, endpoint)."""
    if rs_traj:
        ex, ey, eth = rs_traj[-1]
    elif acts:
        ex, ey, eth = _replay_to_end(x0, y0, th0, acts, prims)
    else:
        gp = stats.get('goal_pos')
        if gp:
            ex, ey, eth = gp
        else:
            ex, ey, eth = x0, y0, th0
    goal_ok = (ex <= 2.30 and abs(ey) <= 0.25 and abs(eth) <= ALIGN_GOAL_DYAW + 0.05)
    return goal_ok, (ex, ey, eth)


# ═══════════════════════════════════════════════════════════════════════
# Scenario runner
# ═══════════════════════════════════════════════════════════════════════

def _is_start_valid(x, y, th, obstacles):
    """Check start position is not inside an obstacle."""
    fast = _preprocess_obstacles(obstacles) if obstacles else None
    ok, _ = check_collision(x, y, th, no_corridor=True, obstacles=fast)
    return ok


def run_single_case(x, y, th, prims, obstacles, timeout_s,
                    force_low_budget=False, planner_fn=None):
    """Run a single planning case. Returns dict with results."""
    if planner_fn is None:
        planner_fn = plan_path_robust_obs
    stats = {}
    ok = False
    timed_out = False
    t0 = time.perf_counter()

    def _plan():
        return planner_fn(
            x, y, th, prims,
            use_rs=True, stats=stats, obstacles=obstacles,
        )

    old_handler = signal.getsignal(signal.SIGALRM)
    try:
        signal.signal(signal.SIGALRM, _alarm_handler)
        signal.alarm(timeout_s)
        ok, acts, rs_traj = _plan()
    except _Timeout:
        timed_out = True
        ok = False
        acts, rs_traj = None, None
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)

    elapsed = stats.get('elapsed_ms', (time.perf_counter() - t0) * 1000.0)

    result = {
        'ok': ok,
        'timed_out': timed_out,
        'elapsed_ms': elapsed,
        'expanded': stats.get('expanded', 0),
        'level': stats.get('level', ''),
        'acts': acts,
        'rs_traj': rs_traj,
        'stats': stats,
    }

    if force_low_budget and not timed_out:
        from collision import check_collision as _cc
        from heuristic import DijkstraGrid, rs_grid_heuristic
        fast_obs = _preprocess_obstacles(obstacles) if obstacles else None

        def coll_fn(nx, ny, nth, sin_nth=None):
            return _cc(nx, ny, nth, sin_nth, no_corridor=False, obstacles=fast_obs)

        dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
        dg.build_map(obstacles, start_x=x, start_y=y)

        def h_fn(nx, ny, nth):
            return rs_grid_heuristic(nx, ny, nth, dg)

        fb_stats = {}
        fb_ok, _, _ = astar_core.plan_path(
            x, y, th, prims,
            collision_fn=coll_fn, heuristic_fn=h_fn,
            stats=fb_stats, _expand_limit=100, _rs_expand=False,
        )
        result['low_budget_ok'] = fb_ok
        result['low_budget_expanded'] = fb_stats.get('expanded', 0)

    return result


def run_all_scenarios(prims, timeout_s=30, scenario_filter=None, planner_fn=None):
    """Run all (or filtered) scenarios. Returns structured results."""
    all_results = {}
    scenario_list = [(n, s) for n, s in SCENARIOS.items()
                     if not scenario_filter or n == scenario_filter]
    total_scenarios = len(scenario_list)

    planner_name = getattr(planner_fn, '__name__', 'v1') if planner_fn else 'v1'
    print(f"\nPlanner: {planner_name}", flush=True)

    for si, (name, spec) in enumerate(scenario_list, 1):
        obstacles = spec["obstacles"]
        cases = spec["cases"]
        expect = spec["expect_success"]
        force_low = spec.get("_force_low_budget", False)

        print(f"\n[{si}/{total_scenarios}] {name} ({len(cases)} cases, expect={'PASS' if expect else 'FAIL'})...", flush=True)

        scenario_results = []
        for ci, (cx, cy, deg, label) in enumerate(cases, 1):
            th = _deg(deg)
            if not _is_start_valid(cx, cy, th, obstacles):
                print(f"  [{ci}/{len(cases)}] {label}: SKIP (start in obstacle)", flush=True)
                scenario_results.append({
                    'label': label, 'x': cx, 'y': cy, 'deg': deg,
                    'skipped': True, 'reason': 'start_in_obstacle',
                })
                continue

            t_case = time.perf_counter()
            res = run_single_case(cx, cy, th, prims, obstacles, timeout_s,
                                  force_low_budget=force_low, planner_fn=planner_fn)
            wall_ms = (time.perf_counter() - t_case) * 1000
            res['label'] = label
            res['x'] = cx
            res['y'] = cy
            res['deg'] = deg
            res['expect'] = expect

            if res['ok'] and expect:
                traj_ok, traj_detail = _validate_rs_traj(res['rs_traj'], obstacles)
                goal_ok, endpoint = _validate_endpoint(
                    cx, cy, th, res['acts'], res['rs_traj'], prims, res['stats'])
                res['traj_collision_free'] = traj_ok
                res['goal_reached'] = goal_ok
                res['endpoint'] = endpoint
                if not traj_ok:
                    res['traj_detail'] = traj_detail

            match = (res['ok'] == expect)
            res['match'] = match
            scenario_results.append(res)

            status = "OK" if match else "MISMATCH"
            to_str = " TIMEOUT" if res.get('timed_out') else ""
            print(f"  [{ci}/{len(cases)}] {label}: ok={res['ok']} {status}{to_str} ({wall_ms:.0f}ms, exp={res['expanded']})", flush=True)

        all_results[name] = {
            'spec': spec,
            'results': scenario_results,
        }
    return all_results


# ═══════════════════════════════════════════════════════════════════════
# Report formatting
# ═══════════════════════════════════════════════════════════════════════

def print_report(all_results, log_fn=None):
    def out(msg=""):
        print(msg, flush=True)
        if log_fn:
            log_fn(msg)

    total_cases = 0
    total_pass = 0
    total_fail = 0

    out("=" * 100)
    out("  COMPREHENSIVE OBSTACLE PLANNING TEST REPORT")
    out(f"  Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    out("=" * 100)

    for name, data in all_results.items():
        spec = data['spec']
        results = data['results']
        expect = spec['expect_success']
        note = spec.get('note', '')

        out(f"\n{'─' * 100}")
        out(f"  Scenario: {name}")
        out(f"  Note: {note}")
        out(f"  Obstacles: {len(spec['obstacles'])}, Expected: {'PASS' if expect else 'FAIL'}")
        out(f"{'─' * 100}")
        out(f"  {'Label':<30} | {'Pos':<22} | {'OK?':<5} | {'Match':<5} | {'ms':<9} | {'Exp':<7} | {'Level':<16} | Detail")
        out(f"  {'-'*30}-+-{'-'*22}-+-{'-'*5}-+-{'-'*5}-+-{'-'*9}-+-{'-'*7}-+-{'-'*16}-+-{'-'*20}")

        s_pass = 0
        s_fail = 0
        s_time = 0.0

        for r in results:
            if r.get('skipped'):
                out(f"  {r['label']:<30} | ({r['x']:.1f},{r['y']:5.1f},{r['deg']:4d}°) | SKIP  |       |           |         |                  | {r.get('reason','')}")
                continue

            total_cases += 1
            ok_str = "YES" if r['ok'] else "NO"
            match_str = "OK" if r['match'] else "FAIL"
            ms_str = f"{r['elapsed_ms']:.0f}"
            exp_str = str(r['expanded'])
            level = r.get('level', '')

            detail_parts = []
            if r.get('timed_out'):
                detail_parts.append("TIMEOUT")
            if r['ok'] and expect:
                if not r.get('traj_collision_free', True):
                    detail_parts.append("TRAJ_COLLIDE")
                if not r.get('goal_reached', True):
                    ep = r.get('endpoint', (0, 0, 0))
                    detail_parts.append(f"GOAL_MISS({ep[0]:.2f},{ep[1]:.2f},{math.degrees(ep[2]):.0f}°)")
            if r.get('low_budget_ok') is not None:
                lb = "passed" if r['low_budget_ok'] else "failed"
                detail_parts.append(f"low_budget={lb}(exp={r.get('low_budget_expanded',0)})")
            detail = " | ".join(detail_parts) if detail_parts else ""

            if r['match']:
                s_pass += 1
                total_pass += 1
            else:
                s_fail += 1
                total_fail += 1

            s_time += r['elapsed_ms']

            out(f"  {r['label']:<30} | ({r['x']:.1f},{r['y']:5.1f},{r['deg']:4d}°) | {ok_str:<5} | {match_str:<5} | {ms_str:>7}ms | {exp_str:>7} | {level:<16} | {detail}")

        tested = s_pass + s_fail
        avg_ms = s_time / tested if tested > 0 else 0
        out(f"  Summary: {s_pass}/{tested} match expected, avg {avg_ms:.0f}ms")

    out(f"\n{'=' * 100}")
    out(f"  GLOBAL SUMMARY")
    out(f"  Total cases: {total_cases}")
    out(f"  Passed (match expected): {total_pass}")
    out(f"  Failed (mismatch): {total_fail}")
    rate = total_pass / total_cases * 100 if total_cases > 0 else 0
    out(f"  Match rate: {rate:.1f}%")
    out(f"{'=' * 100}")

    return total_fail == 0


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description='Comprehensive obstacle planning test suite')
    p.add_argument('--timeout', type=int, default=30, help='Per-case timeout in seconds (default 30)')
    p.add_argument('--scenario', type=str, default=None, help='Run only this scenario (e.g. S01_open_field)')
    p.add_argument('--planner', type=str, default='v1', choices=['v1', 'v2', 'both'],
                   help='Which planner to test: v1, v2, or both (default v1)')
    args = p.parse_args()

    prims = init_primitives()

    planners = {}
    if args.planner in ('v1', 'both'):
        planners['v1'] = plan_path_robust_obs
    if args.planner in ('v2', 'both'):
        if plan_path_robust_obs_v2 is None:
            print("ERROR: planner_obs_v2 not available (import failed)", flush=True)
            sys.exit(1)
        planners['v2'] = plan_path_robust_obs_v2

    os.makedirs('logs', exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')

    all_pass = True
    for pname, pfn in planners.items():
        print(f"\n{'#' * 100}")
        print(f"#  Running with planner: {pname}")
        print(f"{'#' * 100}", flush=True)

        log_path = os.path.join('logs', f'obs_v2_test_{pname}_{ts}.log')
        with open(log_path, 'w', encoding='utf-8') as lf:
            def log_fn(msg, _lf=lf):
                _lf.write(msg + '\n')
                _lf.flush()

            results = run_all_scenarios(prims, timeout_s=args.timeout,
                                         scenario_filter=args.scenario,
                                         planner_fn=pfn)
            pass_ok = print_report(results, log_fn=log_fn)
            if not pass_ok:
                all_pass = False

        print(f"\nLog saved to: {log_path}", flush=True)

    sys.exit(0 if all_pass else 1)
