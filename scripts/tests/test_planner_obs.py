import math
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from primitives import init_primitives
from planner_obs import plan_path_robust_obs, _preprocess_obstacles, _make_collision_fn
from heuristic import DijkstraGrid
from fallback_2d import plan_2d_fallback, _build_multi_homotopy_gate_candidates
try:
    import planner_obs_v2 as planner_obs_v2_mod
    from planner_obs_v2 import (
        _build_kturn_seed_candidates,
        _select_l15_relevant_obstacles,
        plan_path_robust_obs_v2,
        _check_l18_quality,
        _check_rescue_seed_quality,
    )
except ImportError:
    planner_obs_v2_mod = None
    _build_kturn_seed_candidates = None
    _select_l15_relevant_obstacles = None
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


def test_planner_obs_v2_l15_locality_filter_skips_far_obstacles_for_near_goal_case():
    """near-goal case 不应让远处障碍劫持 L1.5 milestone 生成。"""
    if _select_l15_relevant_obstacles is None:
        return
    obs = [
        {'x': 7.14, 'y': -1.20, 'w': 1.26, 'h': 1.26},
        {'x': 6.75, 'y': 1.06, 'w': 1.22, 'h': 0.95},
        {'x': 6.20, 'y': 1.74, 'w': 1.47, 'h': 0.43},
        {'x': 6.23, 'y': -0.92, 'w': 0.47, 'h': 0.69},
    ]
    relevant, diag = _select_l15_relevant_obstacles(2.96, 0.63, obs)
    assert relevant == [], (relevant, diag)
    assert diag.get('l15_locality_mode') == 'near_goal_local', diag
    assert diag.get('l15_relevant_obstacle_count') == 0, diag
    assert diag.get('l15_total_obstacle_count') == 4, diag


def test_planner_obs_v2_l15_locality_filter_keeps_local_obstacle():
    """near-goal case 若局部障碍确实挡路，L1.5 仍应保留该障碍。"""
    if _select_l15_relevant_obstacles is None:
        return
    obs = [
        {'x': 2.45, 'y': -0.35, 'w': 0.55, 'h': 0.90},
        {'x': 6.40, 'y': 1.20, 'w': 1.10, 'h': 1.00},
    ]
    relevant, diag = _select_l15_relevant_obstacles(3.00, 0.55, obs)
    assert len(relevant) == 1, (relevant, diag)
    assert relevant[0]['x'] == 2.45, (relevant, diag)
    assert diag.get('l15_relevant_obstacle_indices') == [0], diag


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


def test_multi_homotopy_gate_candidates_exist_for_three_obstacle_slalom():
    """三障碍交错场景应生成显式 gate 候选，但简单障碍不应启用。"""
    obs3 = [
        {'x': 3.34, 'y': -2.60, 'w': 0.38, 'h': 0.78},
        {'x': 3.79, 'y': -1.05, 'w': 0.40, 'h': 1.24},
        {'x': 3.55, 'y': 0.62, 'w': 0.46, 'h': 1.18},
    ]
    candidates = _build_multi_homotopy_gate_candidates(4.63, 2.87, obs3)
    assert candidates, candidates
    assert candidates[0].get('gate') is not None, candidates

    simple = [{'x': 3.78, 'y': -1.42, 'w': 0.50, 'h': 1.94}]
    assert _build_multi_homotopy_gate_candidates(4.79, -0.70, simple) == []


def test_plan_2d_fallback_reports_late_merge_gate_hint():
    """晚并线失败时，L1.8 应报告 gate hint 而不是静默超时。"""
    obs3 = [
        {'x': 3.34, 'y': -2.60, 'w': 0.38, 'h': 0.78},
        {'x': 3.79, 'y': -1.05, 'w': 0.40, 'h': 1.24},
        {'x': 3.55, 'y': 0.62, 'w': 0.46, 'h': 1.18},
    ]
    fast = _preprocess_obstacles(obs3)
    coll = _make_collision_fn(no_corridor=False, fast_obstacles=fast)
    dg = DijkstraGrid(2.25, 0.0, inflate_radius=0.30)
    dg.build_map(obs3, start_x=4.63, start_y=2.87)
    diag = {}
    ok, traj = plan_2d_fallback(
        dg, 4.63, 2.87, math.radians(82.2), coll,
        spacing=2.5, max_rs_paths=12, obstacles=obs3,
        deadline=None, diag_out=diag)
    assert ok is False
    assert traj is None
    assert diag.get('late_merge_detected') is True, diag
    assert (diag.get('late_merge_count', 0) >= 2
            or diag.get('late_merge_geometry_count', 0) >= 1), diag
    assert diag.get('gate_candidate_count', 0) > 0, diag
    assert diag.get('suggested_gate') is not None, diag


def test_planner_obs_v2_late_merge_gate_attempt_is_wired():
    """诊断出晚并线后，planner 应尝试 gate-biased A* 分支。"""
    if planner_obs_v2_mod is None or plan_path_robust_obs_v2 is None:
        return
    prims = init_primitives()
    obs3 = [
        {'x': 3.34, 'y': -2.60, 'w': 0.38, 'h': 0.78},
        {'x': 3.79, 'y': -1.05, 'w': 0.40, 'h': 1.24},
        {'x': 3.55, 'y': 0.62, 'w': 0.46, 'h': 1.18},
    ]

    orig_fb = planner_obs_v2_mod.plan_2d_fallback
    orig_astar = planner_obs_v2_mod.astar_core.plan_path
    astar_limits = []
    goal_boxes = []

    def fake_fb(*args, **kwargs):
        diag = kwargs.get('diag_out')
        if diag is not None:
            diag.update({
                'late_merge_detected': True,
                'late_merge_count': 3,
                'gate_candidate_count': 2,
                'suggested_gate': (4.05, 0.40, 0.0),
            })
        return False, None

    def fake_astar(*args, **kwargs):
        stats = kwargs.get('stats')
        if stats is not None:
            stats['expanded'] = 0
            stats['elapsed_ms'] = 0.0
            stats['rs_expansion'] = False
        astar_limits.append(kwargs.get('_expand_limit'))
        goal_boxes.append((
            kwargs.get('_goal_xmin'),
            kwargs.get('_goal_xmax'),
            kwargs.get('_goal_ymin'),
            kwargs.get('_goal_ymax'),
        ))
        return False, None, None

    planner_obs_v2_mod.plan_2d_fallback = fake_fb
    planner_obs_v2_mod.astar_core.plan_path = fake_astar
    try:
        st = {}
        ok, acts, traj = plan_path_robust_obs_v2(
            4.63, 2.87, math.radians(82.2), prims,
            use_rs=True, stats=st, obstacles=obs3,
            rs_expansion_radius=0.8, _time_budget=6.0,
        )
    finally:
        planner_obs_v2_mod.plan_2d_fallback = orig_fb
        planner_obs_v2_mod.astar_core.plan_path = orig_astar

    assert ok is False
    assert acts is None and traj is None
    assert st.get('late_merge_gate_hint') == (4.05, 0.40, 0.0), st
    assert st.get('late_merge_gate_attempted') is True, st
    assert 25000 in astar_limits, astar_limits
    assert any(
        box[0] is not None and abs(box[0] - 3.85) < 1e-6 and abs(box[2] - 0.15) < 1e-6
        for box in goal_boxes
    ), goal_boxes


def test_planner_obs_v2_late_merge_deep_stage_uses_slalom_bank():
    """late-merge deep stage 应切换到 slalom primitive bank。"""
    if planner_obs_v2_mod is None or plan_path_robust_obs_v2 is None:
        return
    prims = init_primitives()
    obs3 = [
        {'x': 3.34, 'y': -2.60, 'w': 0.38, 'h': 0.78},
        {'x': 3.79, 'y': -1.05, 'w': 0.40, 'h': 1.24},
        {'x': 3.55, 'y': 0.62, 'w': 0.46, 'h': 1.18},
    ]

    orig_fb = planner_obs_v2_mod.plan_2d_fallback
    orig_gate_stage = planner_obs_v2_mod._plan_to_gate_region
    orig_astar = planner_obs_v2_mod.astar_core.plan_path
    astar_calls = []

    def fake_fb(*args, **kwargs):
        diag = kwargs.get('diag_out')
        if diag is not None:
            diag.update({
                'late_merge_detected': True,
                'late_merge_count': 3,
                'gate_candidate_count': 2,
                'suggested_gate': (4.05, 0.40, 0.0),
            })
        return False, None

    def fake_gate_stage(*args, **kwargs):
        return True, [('F', 0.0, 0.33)], (4.10, 0.35, 0.0), {
            'expanded': 0,
            'elapsed_ms': 0.0,
            'gate_pure': 1.0,
        }

    def fake_astar(*args, **kwargs):
        precomp_prim = args[3]
        stats = kwargs.get('stats')
        if stats is not None:
            stats['expanded'] = 0
            stats['elapsed_ms'] = 0.0
            stats['rs_expansion'] = False
        astar_calls.append({
            'expand_limit': kwargs.get('_expand_limit'),
            'prim_limit': kwargs.get('_prim_limit'),
            'step_limit': kwargs.get('_step_limit'),
            'primitive_count': len(precomp_prim),
        })
        return False, None, None

    planner_obs_v2_mod.plan_2d_fallback = fake_fb
    planner_obs_v2_mod._plan_to_gate_region = fake_gate_stage
    planner_obs_v2_mod.astar_core.plan_path = fake_astar
    try:
        st = {}
        ok, acts, traj = plan_path_robust_obs_v2(
            4.63, 2.87, math.radians(82.2), prims,
            use_rs=True, stats=st, obstacles=obs3,
            rs_expansion_radius=0.8, _time_budget=6.0,
        )
    finally:
        planner_obs_v2_mod.plan_2d_fallback = orig_fb
        planner_obs_v2_mod._plan_to_gate_region = orig_gate_stage
        planner_obs_v2_mod.astar_core.plan_path = orig_astar

    assert ok is False
    assert acts is None and traj is None
    assert st.get('late_merge_deep_attempted') is True, st
    assert st.get('late_merge_deep_start_kind') == 'gate_stage', st
    assert st.get('late_merge_deep_primitive_count', 0) > len(prims), st
    assert any(
        call['expand_limit'] == 180000 and call['primitive_count'] > len(prims)
        and call['prim_limit'] == 180 and call['step_limit'] == 1800
        for call in astar_calls
    ), astar_calls


def test_planner_obs_v2_late_merge_gate_stage_can_promote_alternate_hint():
    """主 gate 失败时，应尝试备选 gate，并让 deep stage 从 gate-stage pose 起跑。"""
    if planner_obs_v2_mod is None or plan_path_robust_obs_v2 is None:
        return
    prims = init_primitives()
    obs3 = [
        {'x': 3.34, 'y': -2.60, 'w': 0.38, 'h': 0.78},
        {'x': 3.79, 'y': -1.05, 'w': 0.40, 'h': 1.24},
        {'x': 3.55, 'y': 0.62, 'w': 0.46, 'h': 1.18},
    ]

    orig_fb = planner_obs_v2_mod.plan_2d_fallback
    orig_gate_stage = planner_obs_v2_mod._plan_to_gate_region
    orig_astar = planner_obs_v2_mod.astar_core.plan_path
    gate_calls = []
    astar_calls = []

    def fake_fb(*args, **kwargs):
        diag = kwargs.get('diag_out')
        if diag is not None:
            diag.update({
                'late_merge_detected': True,
                'late_merge_count': 3,
                'gate_candidate_count': 2,
                'suggested_gate': (3.49, -1.435, 0.0),
            })
        return False, None

    def fake_gate_stage(start_pose, gate_hint, precomp_prim, coll_fn, obstacles, gate_budget):
        gate_calls.append({
            'gate_hint': gate_hint,
            'primitive_count': len(precomp_prim),
        })
        if abs(gate_hint[1] + 1.435) < 1e-6:
            return False, None, None, {'expanded': 100, 'elapsed_ms': 1.0, 'gate_pure': 5.0}
        return True, [('F', 0.0, 0.33)], (4.70, 0.65, 1.30), {
            'expanded': 200,
            'elapsed_ms': 2.0,
            'gate_pure': 3.0,
        }

    def fake_astar(*args, **kwargs):
        stats = kwargs.get('stats')
        if stats is not None:
            stats['expanded'] = 0
            stats['elapsed_ms'] = 0.0
            stats['rs_expansion'] = False
        astar_calls.append({
            'start_pose': args[:3],
            'expand_limit': kwargs.get('_expand_limit'),
        })
        return False, None, None

    planner_obs_v2_mod.plan_2d_fallback = fake_fb
    planner_obs_v2_mod._plan_to_gate_region = fake_gate_stage
    planner_obs_v2_mod.astar_core.plan_path = fake_astar
    try:
        st = {}
        ok, acts, traj = plan_path_robust_obs_v2(
            4.63, 2.87, math.radians(82.2), prims,
            use_rs=True, stats=st, obstacles=obs3,
            rs_expansion_radius=0.8, _time_budget=6.0,
        )
    finally:
        planner_obs_v2_mod.plan_2d_fallback = orig_fb
        planner_obs_v2_mod._plan_to_gate_region = orig_gate_stage
        planner_obs_v2_mod.astar_core.plan_path = orig_astar

    assert ok is False
    assert acts is None and traj is None
    assert len(gate_calls) >= 1, gate_calls
    assert gate_calls[0]['gate_hint'] == (4.54, 0.405, 0.0), gate_calls
    assert gate_calls[0]['primitive_count'] > len(prims), gate_calls
    assert st.get('late_merge_gate_used_hint') == (4.54, 0.405, 0.0), st
    assert st.get('late_merge_deep_start_kind') == 'gate_stage', st
    assert any(
        call['expand_limit'] == 180000 and abs(call['start_pose'][0] - 4.70) < 1e-6
        for call in astar_calls
    ), astar_calls


def test_planner_obs_v2_deep_stage_not_used_without_late_merge_signal():
    """没有 late-merge 诊断时，不应启用 slalom deep stage。"""
    if planner_obs_v2_mod is None or plan_path_robust_obs_v2 is None:
        return
    prims = init_primitives()
    obs3 = [
        {'x': 3.34, 'y': -2.60, 'w': 0.38, 'h': 0.78},
        {'x': 3.79, 'y': -1.05, 'w': 0.40, 'h': 1.24},
        {'x': 3.55, 'y': 0.62, 'w': 0.46, 'h': 1.18},
    ]

    orig_fb = planner_obs_v2_mod.plan_2d_fallback
    orig_astar = planner_obs_v2_mod.astar_core.plan_path
    astar_calls = []

    def fake_fb(*args, **kwargs):
        diag = kwargs.get('diag_out')
        if diag is not None:
            diag.update({
                'late_merge_detected': False,
                'gate_candidate_count': 0,
            })
        return False, None

    def fake_astar(*args, **kwargs):
        precomp_prim = args[3]
        stats = kwargs.get('stats')
        if stats is not None:
            stats['expanded'] = 0
            stats['elapsed_ms'] = 0.0
            stats['rs_expansion'] = False
        astar_calls.append({
            'expand_limit': kwargs.get('_expand_limit'),
            'primitive_count': len(precomp_prim),
        })
        return False, None, None

    planner_obs_v2_mod.plan_2d_fallback = fake_fb
    planner_obs_v2_mod.astar_core.plan_path = fake_astar
    try:
        st = {}
        ok, acts, traj = plan_path_robust_obs_v2(
            4.63, 2.87, math.radians(82.2), prims,
            use_rs=True, stats=st, obstacles=obs3,
            rs_expansion_radius=0.8, _time_budget=6.0,
        )
    finally:
        planner_obs_v2_mod.plan_2d_fallback = orig_fb
        planner_obs_v2_mod.astar_core.plan_path = orig_astar

    assert ok is False
    assert acts is None and traj is None
    assert st.get('late_merge_deep_attempted') is not True, st
    assert not any(call['expand_limit'] == 180000 for call in astar_calls), astar_calls


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
    test_multi_homotopy_gate_candidates_exist_for_three_obstacle_slalom()
    test_plan_2d_fallback_reports_late_merge_gate_hint()
    test_planner_obs_v2_late_merge_gate_attempt_is_wired()
    test_planner_obs_v2_late_merge_deep_stage_uses_slalom_bank()
    test_planner_obs_v2_late_merge_gate_stage_can_promote_alternate_hint()
    test_planner_obs_v2_deep_stage_not_used_without_late_merge_signal()
    print("All planner_obs tests passed!")
