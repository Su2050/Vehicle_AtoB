import math
from collision import check_collision, _check_traj_collision

def test_valid_position_no_obstacle():
    """(3.0, 0.0, 0.0) 无障碍物，应返回 (True, 'OK')"""
    ok, reason = check_collision(3.0, 0.0, 0.0)
    assert ok is True
    assert reason == 'OK'

def test_corridor_y_boundary():
    """y=4.0 超出 [-3.5, 3.5]，应返回 (False, 'OBSTACLE') (全局边界)"""
    ok, reason = check_collision(3.0, 4.0, 0.0)
    assert ok is False
    assert reason == 'OBSTACLE'

def test_global_boundary_hard_gate():
    """ny=-4.0 即使 no_corridor=True，也必须返回 False
    这验证全局物理边界在 no_corridor 条件之外"""
    ok, reason = check_collision(3.0, -4.0, 0.0, no_corridor=True)
    assert ok is False
    assert reason == 'OBSTACLE'

def test_obstacle_collision():
    """叉车在障碍物半径 0.5m 内，应返回 (False, 'OBSTACLE')"""
    obs = [{'x': 3.0, 'y': 0.0, 'w': 1.0, 'h': 1.0}] # min_x=3, max_x=4, min_y=0, max_y=1
    # 叉车在 2.6, 0.5，到 (3.0, 0.5) 距离 0.4 < 0.5
    ok, reason = check_collision(2.6, 0.5, 0.0, obstacles=obs)
    assert ok is False
    assert reason == 'OBSTACLE'

def test_obstacle_edge_safe():
    """叉车在障碍物 0.6m 外（>0.5m 半径），应返回 (True, 'OK')"""
    obs = [{'x': 3.0, 'y': 0.0, 'w': 1.0, 'h': 1.0}]
    ok, reason = check_collision(2.4, 0.5, 0.0, obstacles=obs) # dist = 0.6
    assert ok is True
    assert reason == 'OK'

def test_corridor_narrow_pass():
    """nx=1.9, ny=0.1, nth=0.0 应在走廊安全范围内"""
    ok, reason = check_collision(1.9, 0.1, 0.0)
    assert ok is True
    assert reason == 'OK'

def test_corridor_wall_violation():
    """nx=1.5, ny 偏大且 sin_nth 导致 tip_lat 越界，应返回 False"""
    # sc is 0.15 for nx <= 1.87
    # tip_lat = ny - 1.87 * sin_nth. If ny=0.5, nth=0, tip_lat=0.5 > 0.15 -> False
    ok, reason = check_collision(1.5, 0.5, 0.0)
    assert ok is False
    assert reason == 'CORRIDOR'

def test_check_traj_collision_all_pass():
    """全部轨迹点合法时返回 True"""
    traj = [(2.5, 0.0, 0.0), (3.0, 0.0, 0.0), (3.5, 0.0, 0.0)]
    assert _check_traj_collision(traj) is True

def test_check_traj_collision_mid_collision():
    """轨迹中间一点碰撞时返回 False"""
    obs = [{'x': 3.0, 'y': -0.5, 'w': 1.0, 'h': 1.0}]
    traj = [(2.0, 0.0, 0.0), (3.0, -0.2, 0.0), (4.0, 0.0, 0.0)]
    assert _check_traj_collision(traj, obstacles=obs) is False

def test_multiple_obstacles():
    """多个障碍物场景，靠近任一均应碰撞"""
    obs = [
        {'x': 3.0, 'y': 0.0, 'w': 1.0, 'h': 1.0},
        {'x': 5.0, 'y': 2.0, 'w': 1.0, 'h': 1.0}
    ]
    # collide with first
    ok, _ = check_collision(2.6, 0.5, 0.0, obstacles=obs)
    assert ok is False
    # collide with second
    ok, _ = check_collision(4.6, 2.5, 0.0, obstacles=obs)
    assert ok is False
    # safe
    ok, _ = check_collision(4.6, 1.0, 0.0, obstacles=obs)
    assert ok is True

if __name__ == "__main__":
    test_valid_position_no_obstacle()
    test_corridor_y_boundary()
    test_global_boundary_hard_gate()
    test_obstacle_collision()
    test_obstacle_edge_safe()
    test_corridor_narrow_pass()
    test_corridor_wall_violation()
    test_check_traj_collision_all_pass()
    test_check_traj_collision_mid_collision()
    test_multiple_obstacles()
    print("All collision tests passed!")
