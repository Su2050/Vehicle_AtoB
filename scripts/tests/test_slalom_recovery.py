import math
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fallback_2d import (
    _build_multi_homotopy_gate_candidates,
    plan_2d_fallback,
)
from heuristic import DijkstraGrid
from planner_obs import _make_collision_fn, _preprocess_obstacles


SLALOM_OBSTACLES = [
    {'x': 3.34, 'y': -2.60, 'w': 0.38, 'h': 0.78},
    {'x': 3.79, 'y': -1.05, 'w': 0.40, 'h': 1.24},
    {'x': 3.55, 'y': 0.62, 'w': 0.46, 'h': 1.18},
]

SLALOM_BUCKET = [
    {
        'name': 'target_case',
        'start': (4.63, 2.87, math.radians(82.2)),
        'expect_late_merge': True,
    },
    {
        'name': 'slightly_shallower',
        'start': (4.55, 2.60, math.radians(78.0)),
        'expect_late_merge': False,
    },
    {
        'name': 'slightly_farther',
        'start': (4.85, 3.05, math.radians(84.0)),
        'expect_late_merge': False,
    },
]


def _run_l18_diag(case):
    sx, sy, sth = case['start']
    fast = _preprocess_obstacles(SLALOM_OBSTACLES)
    coll = _make_collision_fn(no_corridor=False, fast_obstacles=fast)
    dijkstra = DijkstraGrid(2.25, 0.0, inflate_radius=0.30)
    dijkstra.build_map(SLALOM_OBSTACLES, start_x=sx, start_y=sy)
    diag = {}
    ok, traj = plan_2d_fallback(
        dijkstra, sx, sy, sth, coll,
        spacing=2.5, max_rs_paths=12, obstacles=SLALOM_OBSTACLES,
        deadline=None, diag_out=diag)
    return ok, traj, diag


def test_slalom_bucket_gate_candidates_exist():
    """slalom 回归桶里的场景都应触发多同伦 gate 候选生成。"""
    for case in SLALOM_BUCKET:
        sx, sy, _ = case['start']
        candidates = _build_multi_homotopy_gate_candidates(
            sx, sy, SLALOM_OBSTACLES)
        assert candidates, case['name']
        assert candidates[0].get('gate') is not None, (case['name'], candidates)


def test_slalom_bucket_reports_late_merge_signal():
    """目标 case 应显式报告 late-merge，其他点至少保留 gate 诊断。"""
    detected = []
    for case in SLALOM_BUCKET:
        ok, traj, diag = _run_l18_diag(case)
        assert diag.get('gate_candidate_count', 0) > 0, (case['name'], diag)
        assert diag.get('suggested_gate') is not None, (case['name'], diag)
        if case['expect_late_merge']:
            assert diag.get('late_merge_detected') is True, (case['name'], diag)
            assert ok is False and traj is None, (case['name'], ok, traj, diag)
            detected.append(case['name'])
        elif diag.get('late_merge_detected'):
            detected.append(case['name'])
    assert 'target_case' in detected, detected


if __name__ == "__main__":
    test_slalom_bucket_gate_candidates_exist()
    test_slalom_bucket_reports_late_merge_signal()
    print("All slalom recovery tests passed!")
