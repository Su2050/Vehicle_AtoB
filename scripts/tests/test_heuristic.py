import math
from heuristic import (DijkstraGrid, geometric_heuristic, rs_heuristic,
                       rs_grid_heuristic, preapproach_heuristic)
from primitives import RS_GOAL_X, RS_GOAL_Y, ALIGN_GOAL_DYAW


def test_geometric_at_goal():
    """位于目标区 (2.1, 0.0, 0.0) 时，geometric_heuristic 应返回 ~0"""
    h, w = geometric_heuristic(2.1, 0.0, 0.0)
    assert h < 0.01, f"Expected ~0 at goal, got {h}"

def test_geometric_monotonic_x():
    """x 增大（远离目标）时，geometric_heuristic 应单调增加"""
    h1, _ = geometric_heuristic(3.0, 0.0, 0.0)
    h2, _ = geometric_heuristic(4.0, 0.0, 0.0)
    h3, _ = geometric_heuristic(5.0, 0.0, 0.0)
    assert h1 < h2 < h3, f"Not monotonic: {h1}, {h2}, {h3}"

def test_rs_heuristic_nonnegative():
    """随机位姿的 rs_heuristic 应 >= 0"""
    import random
    random.seed(42)
    for _ in range(50):
        x = random.uniform(2.0, 6.0)
        y = random.uniform(-3.0, 3.0)
        th = random.uniform(-math.pi, math.pi)
        h, w = rs_heuristic(x, y, th)
        assert h >= 0, f"Negative h at ({x:.2f},{y:.2f},{th:.2f}): {h}"

def test_rs_heuristic_admissible():
    """对于无障碍场景，rs_heuristic 应 >= 等效欧几里得距离下界"""
    h, _ = rs_heuristic(2.8, 0.0, 0.0)
    euc_dist = abs(2.8 - RS_GOAL_X)
    euc_time = euc_dist / 0.25
    assert h >= euc_time - 0.1, f"h_rs {h} below euclidean time bound {euc_time}"

def test_dijkstra_grid_build():
    """build_map 后 dist_map 应非空，目标点距离为 0"""
    dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
    dg.build_map(None)
    assert dg.dist_map is not None
    gx, gy = dg._world_to_grid(RS_GOAL_X, RS_GOAL_Y)
    assert dg.dist_map[gx][gy] == 0.0

def test_dijkstra_obstacle_inflation():
    """障碍物 0.7m 内的 grid 点应标记为 obs"""
    dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
    obs = [{'x': 4.0, 'y': 0.0, 'w': 1.0, 'h': 1.0}]
    dg.build_map(obs)
    gx, gy = dg._world_to_grid(4.5, 0.5)
    assert dg.obs_map[gx][gy] is True, "Center of obstacle should be inflated"

def test_dijkstra_out_of_bounds():
    """grid 外节点的 get_heuristic 应返回足够大的值（>=100）"""
    dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
    dg.build_map(None)
    h, _ = dg.get_heuristic(20.0, 20.0)
    assert h >= 100.0, f"Out of bounds heuristic too small: {h}"

def test_rs_grid_heuristic_kturn_estimate():
    """大侧偏 + 小纵向空间时，kturn_est 应 > 0"""
    dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
    obs = [{'x': 3.5, 'y': 0.5, 'w': 0.8, 'h': 1.5}]
    dg.build_map(obs, start_x=4.0, start_y=2.0)
    h_far, _ = rs_grid_heuristic(2.5, 2.0, 0.0, dg)
    h_close, _ = rs_grid_heuristic(2.5, 0.0, 0.0, dg)
    assert h_far > h_close, f"Large y offset should have bigger h: {h_far} vs {h_close}"

def test_preapproach_heuristic_y_penalty():
    """y 超出目标区时惩罚应 > 0，y 在目标区内惩罚应 = 0"""
    h_in, _ = preapproach_heuristic(3.0, 0.0, 0.0, 2.45, 8.0, -0.5, 0.5, 0.35)
    h_out, _ = preapproach_heuristic(3.0, 1.5, 0.0, 2.45, 8.0, -0.5, 0.5, 0.35)
    assert h_in < h_out, f"y in-range should have smaller h: {h_in} vs {h_out}"

def test_line_of_sight_clear():
    """无障碍物时两点间应有直线视线"""
    dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
    dg.build_map(None)
    assert dg.line_of_sight(3.0, 0.0, RS_GOAL_X, RS_GOAL_Y) is True

def test_line_of_sight_blocked():
    """障碍物挡在两点间时应返回 False"""
    dg = DijkstraGrid(RS_GOAL_X, RS_GOAL_Y)
    obs = [{'x': 2.5, 'y': -0.5, 'w': 1.0, 'h': 1.0}]
    dg.build_map(obs)
    assert dg.line_of_sight(4.0, 0.0, RS_GOAL_X, RS_GOAL_Y) is False


if __name__ == "__main__":
    test_geometric_at_goal()
    test_geometric_monotonic_x()
    test_rs_heuristic_nonnegative()
    test_rs_heuristic_admissible()
    test_dijkstra_grid_build()
    test_dijkstra_obstacle_inflation()
    test_dijkstra_out_of_bounds()
    test_rs_grid_heuristic_kturn_estimate()
    test_preapproach_heuristic_y_penalty()
    test_line_of_sight_clear()
    test_line_of_sight_blocked()
    print("All heuristic tests passed!")
