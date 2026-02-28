import math
from primitives import init_primitives
from collision import check_collision
from heuristic import geometric_heuristic, rs_heuristic
from astar_core import plan_path


def _make_collision_fn(no_corridor=False):
    def collision_fn(nx, ny, nth, sin_nth=None, cos_nth=None):
        return check_collision(nx, ny, nth, sin_nth, cos_nth=cos_nth, no_corridor=no_corridor)
    return collision_fn


def test_astar_simple_straight():
    """从 (2.8, 0, 0) 出发，应在少量节点内找到目标"""
    prims = init_primitives()
    st = {}
    ok, acts, _ = plan_path(2.8, 0.0, 0.0, prims,
                            collision_fn=_make_collision_fn(),
                            heuristic_fn=geometric_heuristic,
                            stats=st)
    assert ok is True, f"Should find path, stats={st}"
    assert st['expanded'] < 500, f"Too many expansions: {st['expanded']}"

def test_astar_collision_fn_injection():
    """传入始终返回 False 的 collision_fn，应返回 (False, None, None)"""
    prims = init_primitives()
    def reject_all(nx, ny, nth, sin_nth=None, cos_nth=None):
        return False, 'REJECT'
    ok, acts, _ = plan_path(2.8, 0.0, 0.0, prims,
                            collision_fn=reject_all,
                            heuristic_fn=geometric_heuristic,
                            _expand_limit=100)
    assert ok is False

def test_astar_rs_expand_fn():
    """传入 rs_expand_fn，在接近目标时应触发 RS 一杆进洞"""
    prims = init_primitives()
    rs_called = [False]
    def dummy_rs_expand(cx, cy, cth, cost, path_node, expanded, t_start, stats):
        if abs(cx - 2.1) < 0.5 and abs(cy) < 0.5:
            rs_called[0] = True
            final_path = []
            curr = path_node
            while curr is not None:
                final_path.append(curr[1])
                curr = curr[0]
            final_path.reverse()
            return True, final_path, [(2.1, 0.0, 0.0)]
        return None

    ok, acts, traj = plan_path(2.5, 0.0, 0.0, prims,
                               collision_fn=_make_collision_fn(),
                               heuristic_fn=geometric_heuristic,
                               rs_expand_fn=dummy_rs_expand,
                               _rs_expand=True)
    assert rs_called[0], "RS expand should have been called"
    assert ok is True

def test_astar_expand_limit():
    """expand_limit=10 时，极难用例应返回 False"""
    prims = init_primitives()
    ok, acts, _ = plan_path(6.0, 2.0, math.pi, prims,
                            collision_fn=_make_collision_fn(),
                            heuristic_fn=geometric_heuristic,
                            _expand_limit=10)
    assert ok is False

def test_astar_step_limit():
    """step_limit=5 时应提前终止"""
    prims = init_primitives()
    ok, acts, _ = plan_path(4.0, 0.5, 0.3, prims,
                            collision_fn=_make_collision_fn(),
                            heuristic_fn=geometric_heuristic,
                            _step_limit=5)
    assert ok is False

def test_astar_goal_override():
    """覆写目标区为 Stage-1 宽松目标时应命中"""
    prims = init_primitives()
    ok, acts, _ = plan_path(3.0, 0.0, 0.0, prims,
                            collision_fn=_make_collision_fn(),
                            heuristic_fn=geometric_heuristic,
                            _goal_xmin=2.45, _goal_xmax=8.0,
                            _goal_ymin=-0.5, _goal_ymax=0.5,
                            _goal_thmax=0.35,
                            _rs_expand=False)
    assert ok is True

def test_astar_anti_zigzag():
    """A* 应有防揉搓惩罚逻辑（隐含在 cost 里）"""
    prims = init_primitives()
    st = {}
    ok, acts, _ = plan_path(2.8, 0.0, 0.0, prims,
                            collision_fn=_make_collision_fn(),
                            heuristic_fn=geometric_heuristic,
                            stats=st)
    assert ok is True and acts, f"Expected a valid non-empty plan, got ok={ok}, acts={acts}"
    gear_changes = 0
    prev_g = 'N'
    for a in acts:
        if a[0] != prev_g and prev_g != 'N':
            gear_changes += 1
        prev_g = a[0]
    assert gear_changes < 8, f"Too many gear changes: {gear_changes}"

def test_astar_stats_completeness():
    """stats 应包含 expanded, elapsed_ms, rs_expansion"""
    prims = init_primitives()
    st = {}
    plan_path(2.8, 0.0, 0.0, prims,
              collision_fn=_make_collision_fn(),
              heuristic_fn=geometric_heuristic,
              stats=st)
    assert 'expanded' in st
    assert 'elapsed_ms' in st
    assert 'rs_expansion' in st

def test_astar_rejects_legacy_env_kwargs():
    """astar_core.plan_path 不应接受旧接口里的 obstacles/dijkstra_grid 参数"""
    prims = init_primitives()
    try:
        plan_path(2.8, 0.0, 0.0, prims,
                  collision_fn=_make_collision_fn(),
                  heuristic_fn=geometric_heuristic,
                  obstacles=None)
        assert False, "plan_path should reject unexpected keyword 'obstacles'"
    except TypeError:
        pass

    try:
        plan_path(2.8, 0.0, 0.0, prims,
                  collision_fn=_make_collision_fn(),
                  heuristic_fn=geometric_heuristic,
                  dijkstra_grid=None)
        assert False, "plan_path should reject unexpected keyword 'dijkstra_grid'"
    except TypeError:
        pass


if __name__ == "__main__":
    test_astar_simple_straight()
    test_astar_collision_fn_injection()
    test_astar_rs_expand_fn()
    test_astar_expand_limit()
    test_astar_step_limit()
    test_astar_goal_override()
    test_astar_anti_zigzag()
    test_astar_stats_completeness()
    test_astar_rejects_legacy_env_kwargs()
    print("All astar_core tests passed!")
