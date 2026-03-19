import math
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from primitives import init_primitives
from planner_obs import plan_path_robust_obs, _preprocess_obstacles, _make_collision_fn
from heuristic import DijkstraGrid
try:
    from planner_obs_v2 import (
        _build_kturn_seed_candidates,
        plan_path_robust_obs_v2,
        _check_l18_quality,
        _check_rescue_seed_quality,
    )
except ImportError:
    _build_kturn_seed_candidates = None
    plan_path_robust_obs_v2 = None
    _check_l18_quality = None
    _check_rescue_seed_quality = None


def test_planner_obs_interface():
    """plan_path_robust_obs 应接受 obstacles 参数并返回三元组"""
    prims = init_primitives()
    obs = [{'x': 5.0, 'y': -1.0, 'w': 0.6, 'h': 1.0}]
    st = {}
    ok, acts, traj = plan_path_robust_obs(5.0, 0.0, 0.0, prims,
                                          use_rs=True, stats=st, obstacles=obs)
    assert isinstance(ok, bool)
    assert isinstance(acts, (list, type(None)))
    assert isinstance(traj, (list, type(None)))
    assert 'elapsed_ms' in st

def test_preprocess_obstacles():
    """_preprocess_obstacles 应将 dict 格式转为 tuple 格式"""
    obs = [{'x': 3.0, 'y': -1.0, 'w': 1.0, 'h': 2.0}]
    fast = _preprocess_obstacles(obs)
    assert len(fast) == 1
    min_x, max_x, min_y, max_y = fast[0]
    assert min_x == 3.0 and max_x == 4.0
    assert min_y == -1.0 and max_y == 1.0

def test_preprocess_obstacles_none():
    """无障碍时应返回 None"""
    assert _preprocess_obstacles(None) is None
    assert _preprocess_obstacles([]) is None

def test_make_collision_fn_no_obs():
    """无障碍的 collision_fn 应正常工作"""
    coll_fn = _make_collision_fn(no_corridor=False, fast_obstacles=None)
    ok, reason = coll_fn(3.0, 0.0, 0.0)
    assert ok is True

def test_make_collision_fn_with_obs():
    """有障碍的 collision_fn 应检测碰撞"""
    fast_obs = [(3.0, 4.0, -0.5, 0.5)]
    coll_fn = _make_collision_fn(no_corridor=True, fast_obstacles=fast_obs)
    ok, reason = coll_fn(3.5, 0.0, 0.0)
    assert ok is False, "Should collide with obstacle center"
    ok2, _ = coll_fn(6.0, 0.0, 0.0)
    assert ok2 is True, "Should be clear far from obstacle"

def test_planner_obs_pure_rs_level1():
    """简单无遮挡位姿 + use_rs=True 应走 Level-1 Pure RS"""
    prims = init_primitives()
    obs = [{'x': 6.0, 'y': 2.5, 'w': 0.4, 'h': 0.4}]
    st = {}
    ok, acts, traj = plan_path_robust_obs(2.5, 0.0, 0.0, prims,
                                          use_rs=True, stats=st, obstacles=obs)
    assert ok is True
    assert st.get('level') == 'L1_pure_rs', f"Expected level L1_pure_rs, got {st}"

def test_planner_obs_out_of_range():
    """y > MAX_PLANNABLE_Y 应直接返回 False"""
    prims = init_primitives()
    st = {}
    ok, acts, traj = plan_path_robust_obs(4.0, 10.0, 0.0, prims, stats=st, obstacles=[])
    assert ok is False
    assert st.get('out_of_range') is True

def test_planner_obs_stats_keys():
    """stats 应包含核心字段"""
    prims = init_primitives()
    obs = [{'x': 6.0, 'y': 2.5, 'w': 0.4, 'h': 0.4}]
    st = {}
    plan_path_robust_obs(2.5, 0.0, 0.0, prims, use_rs=True, stats=st, obstacles=obs)
    assert 'elapsed_ms' in st

def test_planner_obs_v2_simple_obstacle_kturn_rescue_handoff_to_l2():
    """简单单障碍的 rescue 应作为 L2 seed，而不是直接返回 L1.8 非QM轨迹。"""
    if plan_path_robust_obs_v2 is None:
        return
    prims = init_primitives()
    obs = [{'x': 3.78, 'y': -1.42, 'w': 0.50, 'h': 1.94}]
    st = {}
    ok, acts, traj = plan_path_robust_obs_v2(
        4.79, -0.70, math.radians(84.7), prims,
        use_rs=True, stats=st, obstacles=obs,
        rs_expansion_radius=0.8, _time_budget=5.0,
    )
    assert st.get('simple_obs_kturn_rescue') is True, st
    assert st.get('simple_obs_kturn_rescue_deferred_to_l2') is True, st
    assert st.get('l2_start_kind') == 'rescue', st
    assert not str(st.get('level', '')).startswith('L1_8_2d_skeleton'), st
    assert ok is False or str(st.get('level', '')).startswith('L2_'), st


def test_planner_obs_v2_multi_rescue_candidates_pick_relaxed_band():
    """多候选 rescue 应能在边界点选到较松的 target_y_max 候选。"""
    if _build_kturn_seed_candidates is None:
        return
    prims = init_primitives()
    obs = [{'x': 3.78, 'y': -1.42, 'w': 0.50, 'h': 1.94}]
    fast = _preprocess_obstacles(obs)
    dg = DijkstraGrid(2.10, 0.0, inflate_radius=0.30)
    dg.build_map(obs, start_x=4.55, start_y=-0.70)
    candidates, rejected = _build_kturn_seed_candidates(
        4.55, -0.70, math.radians(84.7), prims,
        False, fast, dg, target_y_max=0.2)
    assert candidates, (candidates, rejected)
    assert candidates[0].get('target_y_max') == 0.35, candidates
    assert candidates[0].get('reason') == 'ok', candidates


def test_planner_obs_v2_l18_success_must_pass_qm():
    """任何直接返回的 L1.8 成功都必须通过 QM。"""
    if plan_path_robust_obs_v2 is None or _check_l18_quality is None:
        return
    prims = init_primitives()
    obs = [{'x': 3.78, 'y': -1.42, 'w': 0.50, 'h': 1.94}]
    st = {}
    ok, acts, traj = plan_path_robust_obs_v2(
        4.85, -0.70, math.radians(84.7), prims,
        use_rs=True, stats=st, obstacles=obs,
        rs_expansion_radius=0.8, _time_budget=5.0,
    )
    assert ok is True, st
    assert str(st.get('level', '')).startswith('L1_8_2d_skeleton'), st
    qm_ok, qm_reason = _check_l18_quality(
        4.85, -0.70, math.radians(84.7), acts or [], traj or [], prims)
    assert qm_ok is True, (qm_reason, st)
    assert qm_reason == 'ok'


def test_planner_obs_v2_rescue_seed_quality_rejects_overlong_seed():
    """Rescue seed 只能用于短小补救，超长前缀应被拒绝。"""
    if _check_rescue_seed_quality is None:
        return
    prims = init_primitives()
    acts = [('F', 0.0, 1.5)] * 120
    ok, reason, metrics = _check_rescue_seed_quality(
        4.79, -0.70, math.radians(84.7), acts, prims)
    assert ok is False
    assert reason in ('act_count', 'geom_len')
    assert metrics.get('acts', 0) == 120


if __name__ == "__main__":
    test_planner_obs_interface()
    test_preprocess_obstacles()
    test_preprocess_obstacles_none()
    test_make_collision_fn_no_obs()
    test_make_collision_fn_with_obs()
    test_planner_obs_pure_rs_level1()
    test_planner_obs_out_of_range()
    test_planner_obs_stats_keys()
    test_planner_obs_v2_simple_obstacle_kturn_rescue_handoff_to_l2()
    test_planner_obs_v2_multi_rescue_candidates_pick_relaxed_band()
    test_planner_obs_v2_l18_success_must_pass_qm()
    test_planner_obs_v2_rescue_seed_quality_rejects_overlong_seed()
    print("All planner_obs tests passed!")
