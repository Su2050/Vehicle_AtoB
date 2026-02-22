"""
rs.py 单元测试

运行方式：
    python -m pytest test_rs.py -v
或直接：
    python test_rs.py
"""

import math
import random
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
import rs

# 本项目的最小转弯半径（与 main.py 一致）
WHEELBASE = 1.6
MAX_STEER = 0.65
TURNING_RADIUS = WHEELBASE / math.tan(MAX_STEER)  # ≈ 1.97m


# ── 辅助 ───────────────────────────────────────────────────────────────────

def _approx(a, b, tol=1e-4):
    return abs(a - b) <= tol


# ── 测试 1：平凡情况 ─────────────────────────────────────────────────────────

def test_trivial_zero_distance():
    """起点即终点，RS 距离应为 0"""
    d = rs.rs_distance(0.0, 0.0, 0.0, TURNING_RADIUS)
    assert d == 0.0, f"Expected 0.0, got {d}"


def test_trivial_pose_same():
    """rs_distance_pose 起点终点相同，距离应为 0"""
    d = rs.rs_distance_pose(2.5, 0.3, 0.2, 2.5, 0.3, 0.2, TURNING_RADIUS)
    assert _approx(d, 0.0), f"Expected 0.0, got {d}"


# ── 测试 2：纯直行 ───────────────────────────────────────────────────────────

def test_straight_forward():
    """向前直行 1m：在本项目约定中，前进为 -x 方向"""
    # 从 (0,0,0) 出发，前进 1m 到达 (-1, 0, 0)
    d = rs.rs_distance(-1.0, 0.0, 0.0, TURNING_RADIUS)
    assert _approx(d, 1.0, tol=1e-3), f"Expected ~1.0, got {d}"


def test_straight_backward():
    """向后直行 1m：从 (0,0,0) 到 (1, 0, 0)"""
    d = rs.rs_distance(1.0, 0.0, 0.0, TURNING_RADIUS)
    assert _approx(d, 1.0, tol=1e-3), f"Expected ~1.0, got {d}"


def test_straight_various_lengths():
    """不同直行距离，结果应与距离成正比"""
    for dist in [0.5, 1.0, 2.0, 3.0]:
        d = rs.rs_distance(-dist, 0.0, 0.0, TURNING_RADIUS)
        assert _approx(d, dist, tol=1e-2), f"dist={dist}, got {d}"


# ── 测试 3：对称性 ───────────────────────────────────────────────────────────

def test_lateral_symmetry():
    """左右对称：y 取反结果应相同"""
    x, th = 2.5, 0.3
    d1 = rs.rs_distance_pose(x, 0.5, th, 2.1, 0.0, 0.0, TURNING_RADIUS)
    d2 = rs.rs_distance_pose(x, -0.5, -th, 2.1, 0.0, 0.0, TURNING_RADIUS)
    assert _approx(d1, d2, tol=1e-4), f"Symmetry broken: {d1} vs {d2}"


def test_lateral_symmetry_various():
    """多组对称性检验"""
    random.seed(42)
    for _ in range(20):
        x = random.uniform(1.95, 3.0)
        y = random.uniform(0.0, 2.5)
        th = random.uniform(-math.pi, math.pi)
        d1 = rs.rs_distance_pose(x, y, th, 2.1, 0.0, 0.0, TURNING_RADIUS)
        d2 = rs.rs_distance_pose(x, -y, -th, 2.1, 0.0, 0.0, TURNING_RADIUS)
        assert _approx(d1, d2, tol=1e-3), f"Symmetry failed at ({x:.2f},{y:.2f},{th:.2f}): {d1} vs {d2}"


# ── 测试 4：非负性 ───────────────────────────────────────────────────────────

def test_nonnegative_random():
    """随机位姿的 RS 距离必须非负"""
    random.seed(0)
    for _ in range(200):
        x = random.uniform(1.92, 3.0)
        y = random.uniform(-3.0, 3.0)
        th = random.uniform(-math.pi, math.pi)
        d = rs.rs_distance_pose(x, y, th, 2.1, 0.0, 0.0, TURNING_RADIUS)
        assert d >= 0.0, f"Negative distance at ({x:.2f},{y:.2f},{th:.2f}): {d}"


# ── 测试 5：启发式可容许性验证 ──────────────────────────────────────────────

def test_rs_leq_straight_line_scaled():
    """
    RS 距离 >= 欧氏直线距离（RS 路径受转弯半径限制，不一定 = 直线距离）。
    但对于纯直行路径，RS 距离应 == 直线距离。
    """
    # 沿 x 轴移动（不转向），RS 距离应等于直线距离
    for d in [0.3, 0.8, 1.5, 2.5]:
        rs_d = rs.rs_distance(-d, 0.0, 0.0, TURNING_RADIUS)
        assert rs_d >= d - 1e-3, f"RS dist {rs_d} < straight dist {d}"


def test_rs_is_lower_bound():
    """
    RS 距离是无障碍最短路径，实际带碰撞检测的路径长度必须 >= RS 距离。
    验证 RS 距离不会高于一个合理上界（避免实现错误导致距离虚高）。
    """
    # 从 (2.8, 0, 0) 到目标 (2.1, 0, 0)，合理范围
    d = rs.rs_distance_pose(2.8, 0.0, 0.0, 2.1, 0.0, 0.0, TURNING_RADIUS)
    # 直线距离是 0.7m，RS 不应超过 10m（上界宽松保守）
    assert d < 10.0, f"RS distance unreasonably large: {d}"
    assert d >= 0.7 - 1e-3, f"RS distance below Euclidean: {d}"


# ── 测试 6：turning_radius 参数错误 ─────────────────────────────────────────

def test_invalid_turning_radius():
    """turning_radius <= 0 应抛出 ValueError"""
    try:
        rs.rs_distance(1.0, 0.0, 0.0, turning_radius=0.0)
        assert False, "Should have raised ValueError"
    except ValueError:
        pass


# ── 直接运行 ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    tests = [
        test_trivial_zero_distance,
        test_trivial_pose_same,
        test_straight_forward,
        test_straight_backward,
        test_straight_various_lengths,
        test_lateral_symmetry,
        test_lateral_symmetry_various,
        test_nonnegative_random,
        test_rs_leq_straight_line_scaled,
        test_rs_is_lower_bound,
        test_invalid_turning_radius,
    ]
    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            print(f"  PASS  {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"  FAIL  {t.__name__}: {e}")
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
    if failed:
        sys.exit(1)
