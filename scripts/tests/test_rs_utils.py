from rs_utils import plan_path_pure_rs, RS_EXPANSION_RADIUS
from primitives import MIN_TURN_RADIUS, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH

def test_pure_rs_straight_approach():
    """从 (2.8, 0.0, 0.0) 直行到目标，无碰撞，应返回 True"""
    calls = {"n": 0}
    def dummy_collision_fn(x, y, th):
        calls["n"] += 1
        return True, "OK"
    
    ok, acts, traj = plan_path_pure_rs(2.8, 0.0, 0.0, collision_fn=dummy_collision_fn)
    assert ok is True
    assert traj is not None
    assert len(traj) > 0
    assert calls["n"] > 0, "collision_fn should be called when provided"

def test_pure_rs_collision_reject():
    """从 (2.5, 1.0, 0.5) 出发，RS 曲线经过走廊壁时应返回 False"""
    # dummy collision_fn that always returns False
    def reject_collision_fn(x, y, th):
        return False, "COLLISION"
    
    ok, acts, traj = plan_path_pure_rs(2.5, 1.0, 0.5, collision_fn=reject_collision_fn)
    assert ok is False
    assert traj is not None

def test_pure_rs_collision_fn_closure():
    """传入自定义 collision_fn（始终拒绝），应返回 False"""
    ok, acts, traj = plan_path_pure_rs(2.8, 0.0, 0.0, collision_fn=lambda x,y,th: (False, "NO"))
    assert ok is False

def test_pure_rs_no_collision_fn():
    """collision_fn=None 时只检查 RS 是否有解，不做碰撞检测"""
    ok, acts, traj = plan_path_pure_rs(2.8, 0.0, 0.0, collision_fn=None)
    assert ok is True

def test_pure_rs_stats():
    """stats dict 应包含 use_rs, expanded, pure_rs, elapsed_ms"""
    stats = {}
    plan_path_pure_rs(2.8, 0.0, 0.0, stats=stats)
    assert 'use_rs' in stats
    assert 'expanded' in stats
    assert 'pure_rs' in stats
    assert 'elapsed_ms' in stats

def test_rs_constants():
    """MIN_TURN_RADIUS, RS_GOAL_X/Y/TH 应与 primitives 中的物理参数一致"""
    assert MIN_TURN_RADIUS > 0
    assert RS_GOAL_X == 2.10
    assert RS_GOAL_Y == 0.0
    assert RS_GOAL_TH == 0.0
    assert RS_EXPANSION_RADIUS == 0.8

if __name__ == "__main__":
    test_pure_rs_straight_approach()
    test_pure_rs_collision_reject()
    test_pure_rs_collision_fn_closure()
    test_pure_rs_no_collision_fn()
    test_pure_rs_stats()
    test_rs_constants()
    print("All rs_utils tests passed!")
