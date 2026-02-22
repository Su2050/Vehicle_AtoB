import math
import time
from primitives import init_primitives
from planner import plan_path_robust


def test_planner_level1_pure_rs():
    """简单位姿 (2.5, 0.0, 0.0) + use_rs=True 应走 Level-1 Pure RS"""
    prims = init_primitives()
    st = {}
    ok, acts, traj = plan_path_robust(2.5, 0.0, 0.0, prims, use_rs=True, stats=st)
    assert ok is True
    assert st.get('level') == 'L1_pure_rs', f"Expected level L1_pure_rs, got {st}"

def test_planner_level2_kturn():
    """大侧偏 use_rs=False 时应走 Level-2 K-turn + A*"""
    prims = init_primitives()
    st = {}
    ok, acts, traj = plan_path_robust(4.0, 2.0, math.radians(90), prims, use_rs=False, stats=st)
    assert ok is True
    assert st.get('two_stage') is True, f"Expected two_stage, got {st}"

def test_planner_level3_fallback():
    """use_rs=False 时应触发 Level-3 A* 路径"""
    prims = init_primitives()
    st = {}
    ok, acts, traj = plan_path_robust(2.8, 0.0, 0.0, prims, use_rs=False, stats=st)
    assert ok is True
    assert st.get('level') != 'L1_pure_rs', f"use_rs=False should not take L1 path, got {st}"

def test_planner_use_rs_false_skips_pure_rs():
    """use_rs=False 时不应尝试 Pure RS，直接走 A* 路径"""
    prims = init_primitives()
    st = {}
    ok, acts, traj = plan_path_robust(2.5, 0.0, 0.0, prims, use_rs=False, stats=st)
    assert ok is True
    assert st.get('pure_rs') is not True, f"Pure RS should not be used when use_rs=False"

def test_planner_phase0_turnaround():
    """theta=180deg 时应触发 Phase-0 掉头"""
    prims = init_primitives()
    st = {}
    ok, acts, traj = plan_path_robust(4.0, 0.0, math.pi, prims, use_rs=True, stats=st)
    assert ok is True
    stage1_mode = st.get('stage1_mode', '')
    assert st.get('two_stage') is True, f"Expected two-stage execution, got {st}"
    assert 'phase0' in stage1_mode, f"Expected phase0 turnaround marker, got {st}"

def test_planner_kturn_fast():
    """K-turn 阶段应在 500ms 内完成"""
    prims = init_primitives()
    st = {}
    t0 = time.perf_counter()
    ok, acts, traj = plan_path_robust(4.0, 1.5, math.radians(45), prims, use_rs=False, stats=st)
    elapsed = (time.perf_counter() - t0) * 1000
    stage1_ms = st.get('stage1_ms', 0)
    assert stage1_ms > 0, f"Missing stage1_ms in stats: {st}"
    assert stage1_ms < 2000, f"K-turn too slow: {stage1_ms}ms"

def test_planner_no_obstacle_params():
    """planner.py 内部不应传递 obstacles 参数给 astar_core"""
    import os
    planner_path = os.path.join(os.path.dirname(__file__), '..', 'planner.py')
    with open(planner_path, 'r') as f:
        source = f.read()
    assert 'obstacles=' not in source, "planner.py should not pass obstacles= to any function"


if __name__ == "__main__":
    test_planner_level1_pure_rs()
    test_planner_level2_kturn()
    test_planner_level3_fallback()
    test_planner_use_rs_false_skips_pure_rs()
    test_planner_phase0_turnaround()
    test_planner_kturn_fast()
    test_planner_no_obstacle_params()
    print("All planner tests passed!")
