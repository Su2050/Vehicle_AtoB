import math
from collision import check_collision, _check_traj_collision

def test_valid_position_no_obstacle():
    """(3.0, 0.0, 0.0) 无障碍物，应返回 (True, 'OK')"""
    ok, reason = check_collision(3.0, 0.0, 0.0)
    assert ok is True
    assert reason == 'OK'

def test_corridor_y_boundary():
    """y=6.0 超出 MAX_PLANNABLE_Y(5.0)，应返回 (False, 'OBSTACLE') (全局边界)"""
    ok, reason = check_collision(3.0, 6.0, 0.0)
    assert ok is False
    assert reason == 'OBSTACLE'

def test_global_boundary_hard_gate():
    """ny=-6.0 即使 no_corridor=True，也必须返回 False
    这验证全局物理边界在 no_corridor 条件之外"""
    ok, reason = check_collision(3.0, -6.0, 0.0, no_corridor=True)
    assert ok is False
    assert reason == 'OBSTACLE'

def test_obstacle_collision():
    """多圆模型：车身后端圆覆盖到障碍物内部，应返回 (False, 'OBSTACLE')
    障碍物: x∈[3,4], y∈[0,1]
    叉车在 (2.6, 0.5, 0.0)，th=0 → rear circle at (3.1, 0.5) 在障碍物内部"""
    obs = [{'x': 3.0, 'y': 0.0, 'w': 1.0, 'h': 1.0}]
    ok, reason = check_collision(2.6, 0.5, 0.0, obstacles=obs)
    assert ok is False
    assert reason == 'OBSTACLE'

def test_obstacle_edge_safe():
    """多圆模型：所有碰撞圆与障碍物距离均 > HALF_WIDTH(0.25m)，应返回 (True, 'OK')
    障碍物: x∈[5,6], y∈[0,1]
    叉车在 (2.2, 0.5, 0.0)，th=0 → 前端最远圆 offset=-1.87 at (4.07, 0.5)
    距离: dx=5.0-4.07=0.93 > 0.25 → safe
    (使用 x=2.2 避开走廊安全约束 nx<=2.05)"""
    obs = [{'x': 5.0, 'y': 0.0, 'w': 1.0, 'h': 1.0}]
    ok, reason = check_collision(2.2, 0.5, 0.0, obstacles=obs)
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
    """多个障碍物场景，多圆模型碰撞检测"""
    obs = [
        {'x': 3.0, 'y': 0.0, 'w': 1.0, 'h': 1.0},
        {'x': 5.0, 'y': 2.0, 'w': 1.0, 'h': 1.0}
    ]
    # 车身后端圆 (3.1, 0.5) 在 obs1 内部 → collision
    ok, _ = check_collision(2.6, 0.5, 0.0, obstacles=obs)
    assert ok is False
    # 车身后端圆 (5.1, 2.5) 在 obs2 内部 → collision
    ok, _ = check_collision(4.6, 2.5, 0.0, obstacles=obs)
    assert ok is False
    # 所有碰撞圆均远离两个障碍物 → safe
    ok, _ = check_collision(4.6, 1.5, 0.0, obstacles=obs)
    assert ok is True

def test_multi_circle_front_collision():
    """多圆模型：前端碰撞圆（叉齿方向）检测碰撞
    障碍物: x∈[5.0,6.0], y∈[-0.5,0.5]
    叉车在 (3.5, 0.0, 0.0)，th=0 → 前端圆 offset=-1.87 at (5.37, 0.0)
    5.37 在 [5.0, 6.0] 内，距离=0 < 0.25 → collision"""
    obs = [{'x': 5.0, 'y': -0.5, 'w': 1.0, 'h': 1.0}]
    ok, reason = check_collision(3.5, 0.0, 0.0, obstacles=obs)
    assert ok is False
    assert reason == 'OBSTACLE'

def test_multi_circle_angled():
    """多圆模型：斜向角度时碰撞检测
    叉车在 (4.0, 0.0, pi/2)，th=pi/2 → cos=0, sin=1
    车身沿 y 方向展开：px=4.0, py=-offset for each offset
    前端圆 offset=-1.87 → py=1.87, 后端圆 offset=0.5 → py=-0.5
    障碍物: x∈[3.5,4.5], y∈[2.5,3.5]
    前端圆 at (4.0, 1.87)，dy=2.5-1.87=0.63 > 0.25 → safe"""
    obs = [{'x': 3.5, 'y': 2.5, 'w': 1.0, 'h': 1.0}]
    ok, reason = check_collision(4.0, 0.0, math.pi / 2, obstacles=obs)
    assert ok is True
    assert reason == 'OK'

def test_quick_rejection():
    """快速排斥：远离障碍物时不需要逐圆检查
    叉车在 (5.0, 0.0)，障碍物在 (8.0, 3.0)
    距离 > VEHICLE_MAX_RADIUS(2.12) → 快速跳过"""
    obs = [{'x': 8.0, 'y': 3.0, 'w': 1.0, 'h': 1.0}]
    ok, reason = check_collision(5.0, 0.0, 0.0, obstacles=obs)
    assert ok is True
    assert reason == 'OK'


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
    test_multi_circle_front_collision()
    test_multi_circle_angled()
    test_quick_rejection()
    print("All collision tests passed!")
