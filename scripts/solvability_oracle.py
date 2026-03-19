#!/usr/bin/env python3
import argparse
import json
import math
import os
import sys
from collections import Counter

import rs
from collision import check_collision
from heuristic import DijkstraGrid
from primitives import (
    ALIGN_GOAL_DYAW,
    DT,
    MIN_TURN_RADIUS,
    RS_GOAL_TH,
    RS_GOAL_X,
    RS_GOAL_Y,
    VEHICLE_HALF_WIDTH,
    VEHICLE_LENGTH,
    VEHICLE_WIDTH,
)


_BOX_DX = max(0.18, VEHICLE_LENGTH * 0.12)
_BOX_DY = max(0.08, VEHICLE_WIDTH * 0.20)
_BOX_DTH = math.radians(3.0)
_DEFAULT_STAGE_X_MIN = RS_GOAL_X + 0.35
_DEFAULT_STAGE_X_MAX = min(4.4, RS_GOAL_X + 2.30)
_DEFAULT_STAGE_Y_MAX = 0.35


def _normalize_obstacles(obstacles):
    if not obstacles:
        return []
    fast = []
    for obs in obstacles:
        if isinstance(obs, tuple):
            min_x, max_x, min_y, max_y = obs
        else:
            ox, oy, ow, oh = obs["x"], obs["y"], obs["w"], obs["h"]
            min_x = min(ox, ox + ow)
            max_x = max(ox, ox + ow)
            min_y = min(oy, oy + oh)
            max_y = max(oy, oy + oh)
        fast.append((float(min_x), float(max_x), float(min_y), float(max_y)))
    return fast


def _path_length(traj):
    if not traj or len(traj) < 2:
        return 0.0
    return sum(
        math.hypot(traj[i][0] - traj[i - 1][0], traj[i][1] - traj[i - 1][1])
        for i in range(1, len(traj))
    )


def _check_traj_detail(traj, fast_obstacles, no_corridor=False):
    for idx, pt in enumerate(traj):
        ok, reason = check_collision(
            pt[0], pt[1], pt[2], no_corridor=no_corridor, obstacles=fast_obstacles
        )
        if not ok:
            return {
                "ok": False,
                "reason": reason,
                "idx": idx,
                "pt": (round(pt[0], 3), round(pt[1], 3), round(math.degrees(pt[2]), 1)),
            }
    return {"ok": True, "reason": "OK", "idx": None, "pt": None}


def _sample_values(vmin, vmax, step):
    vals = []
    cur = vmin
    while cur <= vmax + 1e-9:
        vals.append(round(cur, 6))
        cur += step
    return vals


def _staging_box_bounds(x, y):
    front_reach = 0.17 + VEHICLE_HALF_WIDTH + _BOX_DX
    rear_reach = 0.83 + VEHICLE_HALF_WIDTH + _BOX_DX
    lat_reach = VEHICLE_HALF_WIDTH + _BOX_DY
    return {
        "min_x": round(x - front_reach, 3),
        "max_x": round(x + rear_reach, 3),
        "min_y": round(y - lat_reach, 3),
        "max_y": round(y + lat_reach, 3),
    }


def _check_staging_box_clear(x, y, th, fast_obstacles):
    for dx in (-_BOX_DX, 0.0, _BOX_DX):
        for dy in (-_BOX_DY, 0.0, _BOX_DY):
            for dth in (-_BOX_DTH, 0.0, _BOX_DTH):
                ok, reason = check_collision(
                    x + dx, y + dy, th + dth, no_corridor=False, obstacles=fast_obstacles
                )
                if not ok:
                    return {"ok": False, "reason": reason}
    return {"ok": True, "reason": "OK"}


def _build_insert_traj(x, y, th):
    return rs.rs_sample_path(
        x, y, th,
        RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
        MIN_TURN_RADIUS,
        step=max(0.05, DT * 0.5),
    )


def _scan_goal_staging_region(
    obstacles,
    *,
    stage_x_min=_DEFAULT_STAGE_X_MIN,
    stage_x_max=_DEFAULT_STAGE_X_MAX,
    stage_y_max=_DEFAULT_STAGE_Y_MAX,
):
    fast_obstacles = _normalize_obstacles(obstacles)
    goal_valid, goal_reason = check_collision(
        RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, no_corridor=False, obstacles=fast_obstacles
    )

    x_vals = _sample_values(stage_x_min, stage_x_max, 0.20)
    y_vals = _sample_values(-stage_y_max, stage_y_max, 0.10)
    yaw_vals = [math.radians(v) for v in (-8.0, -4.0, 0.0, 4.0, 8.0)]

    reason_counts = Counter()
    free_pose_count = 0
    box_clear_count = 0
    certificates = []
    samples = []

    for x in x_vals:
        for y in y_vals:
            for th in yaw_vals:
                sample = {
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "theta_deg": round(math.degrees(th), 1),
                    "status": None,
                    "reason": None,
                }
                pose_ok, pose_reason = check_collision(
                    x, y, th, no_corridor=False, obstacles=fast_obstacles
                )
                if not pose_ok:
                    sample["status"] = "pose_blocked"
                    sample["reason"] = pose_reason
                    reason_counts[f"pose:{pose_reason}"] += 1
                    samples.append(sample)
                    continue
                free_pose_count += 1

                box_res = _check_staging_box_clear(x, y, th, fast_obstacles)
                if not box_res["ok"]:
                    sample["status"] = "box_blocked"
                    sample["reason"] = box_res["reason"]
                    reason_counts[f"box:{box_res['reason']}"] += 1
                    samples.append(sample)
                    continue
                box_clear_count += 1

                insert_traj = _build_insert_traj(x, y, th)
                if not insert_traj:
                    sample["status"] = "insert_blocked"
                    sample["reason"] = "NO_RS"
                    reason_counts["insert:NO_RS"] += 1
                    samples.append(sample)
                    continue

                goal_hit = insert_traj[-1][0] <= 2.25 and abs(insert_traj[-1][1]) <= 0.18
                if not goal_hit:
                    sample["status"] = "insert_blocked"
                    sample["reason"] = "NO_GOAL_HIT"
                    reason_counts["insert:NO_GOAL_HIT"] += 1
                    samples.append(sample)
                    continue

                traj_chk = _check_traj_detail(insert_traj, fast_obstacles, no_corridor=False)
                if not traj_chk["ok"]:
                    sample["status"] = "insert_blocked"
                    sample["reason"] = traj_chk["reason"]
                    reason_counts[f"insert:{traj_chk['reason']}"] += 1
                    samples.append(sample)
                    continue

                sample["status"] = "certified"
                sample["reason"] = "OK"
                cert = {
                    "stage_pose": (
                        round(x, 3),
                        round(y, 3),
                        round(math.degrees(th), 1),
                    ),
                    "stage_box": _staging_box_bounds(x, y),
                    "insert_path_len": round(_path_length(insert_traj), 3),
                    "insert_steps": len(insert_traj),
                    "insert_traj": insert_traj,
                }
                certificates.append(cert)
                samples.append(sample)

    certificates.sort(
        key=lambda item: (
            item["insert_path_len"],
            abs(item["stage_pose"][1]),
            abs(item["stage_pose"][2]),
            item["stage_pose"][0],
        )
    )

    return {
        "goal_pose_valid": goal_valid,
        "goal_pose_reason": goal_reason,
        "candidate_count": len(x_vals) * len(y_vals) * len(yaw_vals),
        "aligned_free_pose_count": free_pose_count,
        "local_box_clear_count": box_clear_count,
        "certificate_count": len(certificates),
        "certificates": certificates,
        "reason_counts": dict(reason_counts),
        "samples": samples,
        "bounds": {
            "stage_x_min": stage_x_min,
            "stage_x_max": stage_x_max,
            "stage_y_max": stage_y_max,
        },
    }


def find_goal_staging_certificates(
    obstacles,
    *,
    stage_x_min=_DEFAULT_STAGE_X_MIN,
    stage_x_max=_DEFAULT_STAGE_X_MAX,
    stage_y_max=_DEFAULT_STAGE_Y_MAX,
):
    scan = _scan_goal_staging_region(
        obstacles,
        stage_x_min=stage_x_min,
        stage_x_max=stage_x_max,
        stage_y_max=stage_y_max,
    )
    return {
        "goal_pose_valid": scan["goal_pose_valid"],
        "goal_pose_reason": scan["goal_pose_reason"],
        "candidate_count": scan["candidate_count"],
        "aligned_free_pose_count": scan["aligned_free_pose_count"],
        "local_box_clear_count": scan["local_box_clear_count"],
        "certificate_count": scan["certificate_count"],
        "certificates": [
            {
                "stage_pose": cert["stage_pose"],
                "stage_box": cert["stage_box"],
                "insert_path_len": cert["insert_path_len"],
                "insert_steps": cert["insert_steps"],
            }
            for cert in scan["certificates"]
        ],
        "reason_counts": scan["reason_counts"],
    }


def _assess_start_to_stage_reachability(x0, y0, obstacles, certificates):
    if not certificates:
        return []
    reachability = []
    fast_obstacles = _normalize_obstacles(obstacles)
    inflate = max(0.35, VEHICLE_HALF_WIDTH + 0.10)

    for cert in certificates[:6]:
        sx, sy, _deg = cert["stage_pose"]
        dg = DijkstraGrid(sx, sy, inflate_radius=inflate)
        dg.build_map(fast_obstacles, start_x=x0, start_y=y0)
        _dist, pure_dist = dg.get_heuristic(x0, y0, None)
        reachable = pure_dist != float("inf") and pure_dist < 90.0
        item = dict(cert)
        item["start_reachable_2d"] = reachable
        item["start_to_stage_2d_dist"] = round(pure_dist, 3) if reachable else None
        reachability.append(item)

    reachability.sort(
        key=lambda item: (
            0 if item["start_reachable_2d"] else 1,
            item["start_to_stage_2d_dist"] if item["start_to_stage_2d_dist"] is not None else 1e9,
            item["insert_path_len"],
        )
    )
    return reachability


def assess_problem_solvability(
    x0,
    y0,
    theta0,
    obstacles,
    *,
    stage_x_min=_DEFAULT_STAGE_X_MIN,
    stage_x_max=_DEFAULT_STAGE_X_MAX,
    stage_y_max=_DEFAULT_STAGE_Y_MAX,
):
    terminal = find_goal_staging_certificates(
        obstacles,
        stage_x_min=stage_x_min,
        stage_x_max=stage_x_max,
        stage_y_max=stage_y_max,
    )
    reachable = _assess_start_to_stage_reachability(x0, y0, obstacles, terminal["certificates"])
    best = reachable[0] if reachable else (terminal["certificates"][0] if terminal["certificates"] else None)

    verdict = "UNKNOWN"
    solvable = None
    rationale = "No terminal certificate and no terminal blockade proof."

    if best and best.get("start_reachable_2d"):
        verdict = "LIKELY_SOLVABLE"
        solvable = True
        rationale = "Found a goal-front staging certificate and a finite 2D route to it."
    elif terminal["certificate_count"] > 0:
        verdict = "TERMINAL_INSERT_CERTIFIED"
        solvable = None
        rationale = "Final insertion is certified from a staging pose, but global reachability is not proven."
    elif not terminal["goal_pose_valid"]:
        verdict = "LIKELY_UNSOLVABLE_TERMINAL"
        solvable = False
        rationale = "Goal pose itself is in collision."
    elif terminal["local_box_clear_count"] == 0:
        verdict = "LIKELY_UNSOLVABLE_TERMINAL"
        solvable = False
        rationale = "No collision-free aligned staging box was found in front of goal."
    elif terminal["aligned_free_pose_count"] > 0 and terminal["certificate_count"] == 0:
        verdict = "LIKELY_UNSOLVABLE_TERMINAL"
        solvable = False
        rationale = "Aligned poses exist, but none can insert into the goal corridor without collision."

    return {
        "verdict": verdict,
        "solvable": solvable,
        "rationale": rationale,
        "start": {
            "x": round(x0, 3),
            "y": round(y0, 3),
            "theta_deg": round(math.degrees(theta0), 1),
        },
        "goal": {
            "x": RS_GOAL_X,
            "y": RS_GOAL_Y,
            "theta_deg": round(math.degrees(RS_GOAL_TH), 1),
        },
        "terminal": {
            "goal_pose_valid": terminal["goal_pose_valid"],
            "goal_pose_reason": terminal["goal_pose_reason"],
            "candidate_count": terminal["candidate_count"],
            "aligned_free_pose_count": terminal["aligned_free_pose_count"],
            "local_box_clear_count": terminal["local_box_clear_count"],
            "certificate_count": terminal["certificate_count"],
            "reason_counts": terminal["reason_counts"],
            "best_certificate": best,
        },
    }


def analyze_problem_solvability(
    x0,
    y0,
    theta0,
    obstacles,
    *,
    stage_x_min=_DEFAULT_STAGE_X_MIN,
    stage_x_max=_DEFAULT_STAGE_X_MAX,
    stage_y_max=_DEFAULT_STAGE_Y_MAX,
):
    scan = _scan_goal_staging_region(
        obstacles,
        stage_x_min=stage_x_min,
        stage_x_max=stage_x_max,
        stage_y_max=stage_y_max,
    )
    reachable = _assess_start_to_stage_reachability(x0, y0, obstacles, scan["certificates"])
    best = reachable[0] if reachable else (scan["certificates"][0] if scan["certificates"] else None)

    summary = assess_problem_solvability(
        x0,
        y0,
        theta0,
        obstacles,
        stage_x_min=stage_x_min,
        stage_x_max=stage_x_max,
        stage_y_max=stage_y_max,
    )
    summary["terminal"]["best_certificate"] = best
    summary["stage_scan"] = {
        "bounds": scan["bounds"],
        "samples": scan["samples"],
        "reachable_certificates": reachable,
    }
    if best is not None:
        summary["stage_scan"]["best_insert_traj"] = best.get("insert_traj")
    else:
        summary["stage_scan"]["best_insert_traj"] = None
    return summary


def _load_obstacles_from_json(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, dict) and "obstacles" in data:
        return data["obstacles"]
    if isinstance(data, list):
        return data
    raise ValueError("Obstacle JSON must be a list or an object with 'obstacles'.")


def _load_case_from_generated_pool(profile, seed, case_id):
    tests_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tests")
    if tests_dir not in sys.path:
        sys.path.insert(0, tests_dir)
    from test_stress import generate_cases

    for case in generate_cases(profile=profile, seed=seed):
        if case["id"] == case_id:
            return case
    raise ValueError(f"Case id={case_id} not found in profile={profile}, seed={seed}.")


def _build_arg_parser():
    p = argparse.ArgumentParser(
        description="Independent solvability oracle based on goal-front staging certificates."
    )
    p.add_argument("--x", type=float, help="Start x.")
    p.add_argument("--y", type=float, help="Start y.")
    p.add_argument("--deg", type=float, help="Start heading in degrees.")
    p.add_argument("--obs-json", type=str, help="Obstacle JSON path.")
    p.add_argument("--case-id", type=int, help="Load a case from stress generator.")
    p.add_argument("--profile", type=str, default="standard", help="Stress profile for --case-id.")
    p.add_argument("--seed", type=int, default=42, help="Stress seed for --case-id.")
    p.add_argument("--stage-x-max", type=float, default=_DEFAULT_STAGE_X_MAX)
    p.add_argument("--stage-y-max", type=float, default=_DEFAULT_STAGE_Y_MAX)
    p.add_argument("--json", action="store_true", help="Print full JSON result.")
    return p


def main(argv=None):
    args = _build_arg_parser().parse_args(argv)

    if args.case_id is not None:
        case = _load_case_from_generated_pool(args.profile, args.seed, args.case_id)
        x0, y0, theta0 = case["x"], case["y"], case["th"]
        obstacles = case["obstacles"]
    else:
        if args.x is None or args.y is None or args.deg is None or not args.obs_json:
            raise SystemExit("Provide either --case-id or all of --x --y --deg --obs-json.")
        x0, y0, theta0 = args.x, args.y, math.radians(args.deg)
        obstacles = _load_obstacles_from_json(args.obs_json)

    result = assess_problem_solvability(
        x0, y0, theta0, obstacles,
        stage_x_max=args.stage_x_max,
        stage_y_max=args.stage_y_max,
    )

    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return

    print(f"Verdict: {result['verdict']}")
    print(f"Solvable: {result['solvable']}")
    print(f"Rationale: {result['rationale']}")
    term = result["terminal"]
    print(
        "Terminal scan:"
        f" candidates={term['candidate_count']}"
        f", free={term['aligned_free_pose_count']}"
        f", box_clear={term['local_box_clear_count']}"
        f", certified={term['certificate_count']}"
    )
    if term["best_certificate"]:
        best = term["best_certificate"]
        print(
            "Best cert:"
            f" stage={best['stage_pose']}"
            f", insert_len={best['insert_path_len']}m"
            f", 2d_reachable={best.get('start_reachable_2d')}"
            f", 2d_dist={best.get('start_to_stage_2d_dist')}"
        )
    if term["reason_counts"]:
        print(f"Reason counts: {term['reason_counts']}")


if __name__ == "__main__":
    main()
