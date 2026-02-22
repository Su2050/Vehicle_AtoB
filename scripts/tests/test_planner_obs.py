import math
from primitives import init_primitives
from planner_obs import plan_path_robust_obs, _preprocess_obstacles, _make_collision_fn


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


if __name__ == "__main__":
    test_planner_obs_interface()
    test_preprocess_obstacles()
    test_preprocess_obstacles_none()
    test_make_collision_fn_no_obs()
    test_make_collision_fn_with_obs()
    test_planner_obs_pure_rs_level1()
    test_planner_obs_out_of_range()
    test_planner_obs_stats_keys()
    print("All planner_obs tests passed!")
