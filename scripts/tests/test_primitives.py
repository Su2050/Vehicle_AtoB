import math
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from primitives import (
    DEFAULT_PRIMITIVE_PROFILE,
    SLALOM_PRIMITIVE_PROFILE,
    init_primitives,
    resolve_replay_primitives,
    _replay_to_end,
    simulate_path,
    simulate_path_strict,
    DT,
    WHEELBASE,
    MAX_STEER,
    M_PI,
    PI2,
    ALIGN_GOAL_DYAW,
)

def _approx(a, b, tol=1e-4):
    return abs(a - b) <= tol

def test_init_primitives_count():
    """init_primitives() 应返回 34 个运动基元"""
    prims = init_primitives()
    assert len(prims) == 34, f"Expected 34 primitives, got {len(prims)}"

def test_init_primitives_slalom_profile_expands_bank():
    """slalom profile 应保留默认基元，并追加更长弧线/更短直行。"""
    default_prims = init_primitives(profile=DEFAULT_PRIMITIVE_PROFILE)
    slalom_prims = init_primitives(profile=SLALOM_PRIMITIVE_PROFILE)
    default_keys = {key for key, _, _ in default_prims}
    slalom_keys = {key for key, _, _ in slalom_prims}

    assert len(default_prims) == 34
    assert len(slalom_prims) > len(default_prims)
    assert default_keys.issubset(slalom_keys)

    for key in [
        ('F', 0.0, 0.17),
        ('R', 0.0, 0.25),
        ('F', 1.0, 1.2),
        ('R', -1.0, 1.2),
        ('F', 0.5, 0.85),
    ]:
        assert key in slalom_keys, f"Expected slalom-only primitive {key}"

def test_primitive_structure():
    """每个基元为 ((gear, steer, duration), N, traj)，traj 每点 5 元组"""
    prims = init_primitives()
    prim = prims[0]
    
    assert len(prim) == 3
    key, N, traj = prim
    assert len(key) == 3
    assert isinstance(key[0], str)
    assert isinstance(key[1], float)
    assert isinstance(key[2], float)
    assert isinstance(N, int)
    assert len(traj) == N
    assert len(traj[0]) == 5
    
def test_straight_primitive_endpoint():
    """steer=0, gear=F, duration=1.0 的基元终点 x 应约 -0.25m（前进 0.25*1.0=0.25m）"""
    prims = init_primitives()
    found = False
    for prim in prims:
        key, N, traj = prim
        if key == ('F', 0.0, 1.0):
            found = True
            last_pt = traj[-1]
            assert _approx(last_pt[0], -0.25, tol=1e-2), f"Expected x to be ~-0.25, got {last_pt[0]}"
            assert _approx(last_pt[1], 0.0, tol=1e-2), f"Expected y to be ~0, got {last_pt[1]}"
            assert _approx(last_pt[2], 0.0, tol=1e-2), f"Expected th to be ~0, got {last_pt[2]}"
            break
    assert found, "Expected primitive ('F', 0.0, 1.0) not found"

def test_turning_primitive_angle():
    """满舵 steer=1.0 转弯基元的终点角度应非零"""
    prims = init_primitives()
    checked = 0
    for prim in prims:
        key, N, traj = prim
        if key[1] == 1.0:
            checked += 1
            last_pt = traj[-1]
            assert abs(last_pt[2]) > 0.03, f"Expected non-zero angle, got {last_pt[2]}"
    assert checked > 0, "No turning primitives with steer=1.0 were found"

def test_slalom_profile_increases_turning_reach():
    """slalom profile 的长弧应带来更大的单步姿态变化。"""
    default_prims = init_primitives()
    slalom_prims = init_primitives(profile=SLALOM_PRIMITIVE_PROFILE)

    def _max_heading(prims, gear, steer):
        headings = []
        for key, _, traj in prims:
            if key[0] == gear and key[1] == steer:
                headings.append(abs(traj[-1][2]))
        return max(headings)

    assert _max_heading(slalom_prims, 'F', 1.0) > _max_heading(default_prims, 'F', 1.0)
    assert _max_heading(slalom_prims, 'R', -1.0) > _max_heading(default_prims, 'R', -1.0)

def test_replay_to_end_consistency():
    """_replay_to_end 结果应与 simulate_path 最终点一致"""
    prims = init_primitives()
    x0, y0, th0 = 2.5, 0.5, 0.1
    acts = [('F', 1.0, 1.0), ('R', -1.0, 0.5)]
    
    rx, ry, rth = _replay_to_end(x0, y0, th0, acts, prims)
    traj = simulate_path(x0, y0, th0, acts, prims)
    sx, sy, sth = traj[-1]
    
    assert _approx(rx, sx), f"x mismatch: {rx} vs {sx}"
    assert _approx(ry, sy), f"y mismatch: {ry} vs {sy}"
    assert _approx(rth, sth), f"th mismatch: {rth} vs {sth}"

def test_simulate_path_goal_truncation():
    """simulate_path 到达目标区域时应截断"""
    prims = init_primitives()
    x0, y0, th0 = 2.45, 0.0, 0.0
    acts = [('F', 0.0, 1.5)]  # Move forward by 0.375m, x should reach 2.075m which is <= 2.25m
    
    traj = simulate_path(x0, y0, th0, acts, prims)
    # The trajectory should be truncated before reaching the end of the primitive
    expected_full_len = 1 + math.ceil(1.5 / DT - 1e-9)
    assert len(traj) < expected_full_len, f"Trajectory was not truncated: len {len(traj)} >= {expected_full_len}"
    
    last_pt = traj[-1]
    assert last_pt[0] <= 2.25
    assert abs(last_pt[1]) <= 0.18
    assert abs(last_pt[2]) <= ALIGN_GOAL_DYAW

def test_resolve_replay_primitives_picks_slalom_for_long_arc():
    """包含 1.2s 满舵长弧时，应自动选择 slalom bank。"""
    acts = [('R', -1.0, 1.2), ('F', 1.0, 1.2), ('F', 0.0, 0.5)]
    profile, prims = resolve_replay_primitives(acts)
    keys = {key for key, _, _ in prims}
    assert profile == SLALOM_PRIMITIVE_PROFILE
    assert set(acts).issubset(keys)

def test_simulate_path_strict_rejects_unknown_actions():
    """严格回放不允许静默跳过未知动作。"""
    prims = init_primitives(profile=DEFAULT_PRIMITIVE_PROFILE)
    try:
        simulate_path_strict(4.0, 2.0, 0.5, [('F', 1.0, 1.2)], prims)
    except KeyError as exc:
        assert "Unknown actions" in str(exc)
    else:
        raise AssertionError("simulate_path_strict should reject unknown actions")

def test_simulate_path_strict_final_step_limit_truncates_last_action():
    """严格回放应支持按 goal_step_hit 截断最后一个 primitive。"""
    prims = init_primitives(profile=DEFAULT_PRIMITIVE_PROFILE)
    act = ('F', 0.0, 1.0)
    full = simulate_path_strict(4.0, 0.0, 0.0, [act], prims, stop_at_goal=False)
    limited = simulate_path_strict(
        4.0, 0.0, 0.0, [act], prims,
        stop_at_goal=False, final_step_limit=3)

    assert len(full) > len(limited)
    assert len(limited) == 4  # start + 3 replay steps
    assert limited[-1][0] > full[-1][0]

def test_constants_exported():
    """DT, WHEELBASE, MAX_STEER, M_PI, PI2, ALIGN_GOAL_DYAW 均应可访问"""
    assert DT > 0
    assert WHEELBASE > 0
    assert MAX_STEER > 0
    assert M_PI > 0
    assert PI2 > 0
    assert ALIGN_GOAL_DYAW > 0

if __name__ == "__main__":
    test_init_primitives_count()
    test_init_primitives_slalom_profile_expands_bank()
    test_primitive_structure()
    test_straight_primitive_endpoint()
    test_turning_primitive_angle()
    test_slalom_profile_increases_turning_reach()
    test_replay_to_end_consistency()
    test_simulate_path_goal_truncation()
    test_resolve_replay_primitives_picks_slalom_for_long_arc()
    test_simulate_path_strict_rejects_unknown_actions()
    test_simulate_path_strict_final_step_limit_truncates_last_action()
    test_constants_exported()
    print("All tests passed!")
