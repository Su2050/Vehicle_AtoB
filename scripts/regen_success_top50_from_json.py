#!/usr/bin/env python3
import json
import math
import multiprocessing as mp
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(SCRIPT_DIR, "tests"))

from test_stress import generate_cases  # noqa: E402
from planner_obs_v2 import plan_path_robust_obs_v2  # noqa: E402
from primitives import (  # noqa: E402
    RS_GOAL_X,
    RS_GOAL_Y,
    init_primitives,
    resolve_replay_primitives,
    simulate_path_strict,
)
from viz_stress_hard_cases import (  # noqa: E402
    _clear_previous_outputs,
    _difficulty_score,
    _plot_individual,
    _plot_overview,
)


LOG = os.environ.get("STRESS_JSON", "logs/stress_timing_standard_20260319_122316.json")
OUT = os.environ.get("OUT_DIR", "docs/stress_success_cases_top50")
PROFILE = os.environ.get("STRESS_PROFILE", "standard")
SEED = int(os.environ.get("STRESS_SEED", "42"))
TARGET = int(os.environ.get("TARGET_COUNT", "50"))
CAND_LIMIT = int(os.environ.get("CAND_LIMIT", "220"))
KEEP_VALID = int(os.environ.get("KEEP_VALID", "80"))
RERUN_BUDGET = float(os.environ.get("RERUN_BUDGET", "25.0"))
WORKERS = int(os.environ.get("VALIDATION_WORKERS", "8"))

_WORKER_PRIMS = None


def _worker_init():
    global _WORKER_PRIMS
    _WORKER_PRIMS = init_primitives()


def _validate_candidate(case):
    global _WORKER_PRIMS
    x, y, th = case["x"], case["y"], case["th"]
    stats = {}
    ok, acts, rs_traj = plan_path_robust_obs_v2(
        x,
        y,
        th,
        _WORKER_PRIMS,
        use_rs=True,
        stats=stats,
        obstacles=case["obstacles"],
        _time_budget=RERUN_BUDGET,
    )
    if not ok:
        return {"ok": False, "id": case["id"], "reason": "rerun_failed"}

    if acts:
        replay_profile, replay_prims = resolve_replay_primitives(acts)
        traj_astar = simulate_path_strict(
            x, y, th, acts, replay_prims,
            final_step_limit=stats.get("goal_step_hit"))
    else:
        replay_profile = "none"
        traj_astar = [(x, y, th)]

    join_error = 0.0
    if traj_astar and rs_traj:
        join_error = math.hypot(
            traj_astar[-1][0] - rs_traj[0][0],
            traj_astar[-1][1] - rs_traj[0][1],
        )
        if join_error > 1e-6:
            return {
                "ok": False,
                "id": case["id"],
                "reason": f"join_mismatch:{join_error:.6f}",
            }

    full_traj = list(traj_astar)
    if rs_traj:
        full_traj.extend(rs_traj[1:] if full_traj else rs_traj)

    path_len = sum(
        math.hypot(
            full_traj[i][0] - full_traj[i - 1][0],
            full_traj[i][1] - full_traj[i - 1][1],
        )
        for i in range(1, len(full_traj))
    )

    shifts = 0
    if acts:
        last_g = acts[0][0]
        for act in acts[1:]:
            if act[0] != last_g:
                shifts += 1
                last_g = act[0]

    merged = dict(case)
    merged["gear_shifts"] = shifts
    merged["path_length"] = path_len
    merged["level"] = stats.get("level", case.get("level", ""))
    merged["expanded"] = stats.get("expanded", case.get("expanded", 0))
    merged["difficulty_score"] = _difficulty_score(merged)

    td = {
        "ok": True,
        "traj_astar": traj_astar,
        "rs_traj": rs_traj,
        "full_traj": full_traj,
        "join_error": join_error,
        "replay_primitive_profile": replay_profile,
        "level": merged["level"],
        "expanded": merged["expanded"],
    }
    return {"ok": True, "case": merged, "traj_data": td}


def main():
    with open(LOG, "r", encoding="utf-8") as f:
        data = json.load(f)

    print(f"Loading cases ({PROFILE}, seed={SEED})...")
    cases = generate_cases(PROFILE, seed=SEED)
    case_by_id = {c["id"]: c for c in cases}

    candidates = []
    for rec in data["records"]:
        if rec.get("category") != "SUCCESS":
            continue
        cid = rec["case_id"]
        case = case_by_id.get(cid)
        if case is None:
            continue
        item = {
            "id": cid,
            "type": case["type"],
            "x": case["x"],
            "y": case["y"],
            "th": case["th"],
            "obstacles": case["obstacles"],
            "elapsed_ms": rec.get("elapsed_ms", 0.0),
            "expanded": rec.get("expanded", 0),
            "level": rec.get("level", ""),
        }
        heading_to_goal = math.atan2(RS_GOAL_Y - item["y"], RS_GOAL_X - item["x"])
        heading_diff = abs(((item["th"] - heading_to_goal + math.pi) % (2 * math.pi)) - math.pi)
        item["_pre_score"] = (
            min(item["elapsed_ms"] / 18000.0, 1.0) * 55.0
            + min(item["expanded"] / 10000.0, 1.0) * 20.0
            + min(len(item["obstacles"]) / 4.0, 1.0) * 10.0
            + (heading_diff / math.pi) * 15.0
        )
        candidates.append(item)

    candidates.sort(key=lambda r: (r["_pre_score"], r["elapsed_ms"], r["expanded"]), reverse=True)
    candidates = candidates[:CAND_LIMIT]
    print(f"Candidate pool: {len(candidates)}")

    os.makedirs(OUT, exist_ok=True)
    _clear_previous_outputs(OUT)

    valid = []
    traj_data_list = []
    skipped = []
    with mp.Pool(processes=WORKERS, initializer=_worker_init) as pool:
        for res in pool.imap_unordered(_validate_candidate, candidates):
            if res["ok"]:
                merged = res["case"]
                td = res["traj_data"]
                valid.append(merged)
                traj_data_list.append(td)
                print(
                    f"[{len(valid):02d}] keep case {merged['id']} "
                    f"level={merged['level']} score={merged['difficulty_score']:.1f}",
                    flush=True,
                )
                if len(valid) >= KEEP_VALID:
                    pool.terminate()
                    break
            else:
                skipped.append({"id": res["id"], "reason": res["reason"]})

    pairs = sorted(
        zip(valid, traj_data_list),
        key=lambda p: p[0]["difficulty_score"],
        reverse=True,
    )[:TARGET]

    selected_cases = [p[0] for p in pairs]
    selected_traj = [p[1] for p in pairs]

    _plot_overview(selected_cases, selected_traj, OUT)
    _plot_individual(selected_cases, selected_traj, OUT)

    with open(os.path.join(OUT, "hard_cases_data.json"), "w", encoding="utf-8") as f:
        json.dump(
            [
                {
                    "id": c["id"],
                    "type": c["type"],
                    "x": c["x"],
                    "y": c["y"],
                    "th": c["th"],
                    "obstacles": c["obstacles"],
                    "elapsed_ms": round(c["elapsed_ms"], 1),
                    "gear_shifts": c.get("gear_shifts", 0),
                    "path_length": round(c.get("path_length", 0), 2),
                    "difficulty_score": round(c["difficulty_score"], 1),
                    "level": c.get("level", ""),
                    "expanded": c.get("expanded", 0),
                    "replay_primitive_profile": td.get("replay_primitive_profile", ""),
                    "join_error": round(td.get("join_error", 0.0), 9),
                }
                for c, td in zip(selected_cases, selected_traj)
            ],
            f,
            indent=2,
        )

    with open(os.path.join(OUT, "hard_cases_skipped.json"), "w", encoding="utf-8") as f:
        json.dump(skipped, f, indent=2)

    print(f"Generated {len(selected_cases)} plots in {OUT}", flush=True)
    print(f"Skipped {len(skipped)} candidates", flush=True)


if __name__ == "__main__":
    main()
