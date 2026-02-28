"""
main.py — 薄入口层 (re-export)

重构后的代码已按职责拆分为独立模块：
  primitives.py   — 物理常量 & 运动基元
  collision.py    — 碰撞检测
  rs_utils.py     — Reeds-Shepp 工具
  heuristic.py    — 启发式函数 & DijkstraGrid
  astar_core.py   — 通用 A* 引擎（回调注入）
  planner.py      — 无障碍鲁棒规划器
  planner_obs.py  — 有障碍鲁棒规划器

本文件仅做向后兼容的 re-export，使 test_grid_search / viz / benchmark 等
现有脚本无需修改即可通过 `import main` 正常工作。
"""

# ── 物理常量 & 基元 ──────────────────────────────────────────────────
from primitives import (
    DT, WHEELBASE, MAX_STEER, M_PI, PI2,
    ALIGN_GOAL_DYAW, MIN_TURN_RADIUS,
    RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
    GEAR_CHANGE_PENALTY, STEER_JUMP_PENALTY,
    PREAPPROACH_X_MIN, PREAPPROACH_X_MAX, PREAPPROACH_Y_MAX, PREAPPROACH_TH_MAX,
    TWO_STAGE_Y_THRESH, TWO_STAGE_TH_THRESH, MAX_PLANNABLE_Y,
    init_primitives, _replay_to_end, simulate_path,
)

# ── 碰撞检测 ────────────────────────────────────────────────────────
from collision import check_collision, _check_traj_collision

# ── RS 工具 ──────────────────────────────────────────────────────────
from rs_utils import plan_path_pure_rs, RS_EXPANSION_RADIUS

# ── 启发式 & DijkstraGrid ───────────────────────────────────────────
from heuristic import DijkstraGrid

# ── 规划器 ───────────────────────────────────────────────────────────
from planner import _k_turn_preposition

# 向后兼容: plan_path 和 plan_path_robust 需要保持原始签名
# 原始 main.py 的 plan_path 接受 obstacles / dijkstra_grid 等参数
# 我们通过包装函数在 collision_fn / heuristic_fn 闭包中转发这些参数
import math
import time
import rs
import astar_core as _astar
from heuristic import (
    geometric_heuristic as _geom_h,
    rs_heuristic as _rs_h,
    rs_grid_heuristic as _rs_grid_h,
    preapproach_heuristic as _preapproach_h,
)
from planner_obs import (
    _preprocess_obstacles as _prep_obs,
    _k_turn_preposition_obs,
)


def plan_path(x0, y0, theta0, precomp_prim,
              use_rs=False, stats=None, no_corridor=False,
              rs_expansion_radius=2.5,
              _goal_xmin=1.92, _goal_xmax=2.25,
              _goal_ymin=-0.18, _goal_ymax=0.18, _goal_thmax=ALIGN_GOAL_DYAW,
              _step_limit=1079, _expand_limit=150000, _prim_limit=30,
              _rs_expand=True,
              _heuristic_preapproach=False,
              obstacles=None,
              dijkstra_grid=None):
    """向后兼容的 plan_path — 桥接到新 astar_core.plan_path"""

    fast_obs = _prep_obs(obstacles) if obstacles else None

    def _collision_fn(nx, ny, nth, sin_nth=None, cos_nth=None):
        return check_collision(nx, ny, nth, sin_nth, cos_nth=cos_nth,
                               no_corridor=no_corridor, obstacles=fast_obs)

    if _heuristic_preapproach:
        def _h_fn(nx, ny, nth):
            return _preapproach_h(nx, ny, nth,
                                  _goal_xmin, _goal_xmax,
                                  _goal_ymin, _goal_ymax, _goal_thmax,
                                  dijkstra_grid=dijkstra_grid)
    elif use_rs:
        if dijkstra_grid is not None:
            def _h_fn(nx, ny, nth):
                return _rs_grid_h(nx, ny, nth, dijkstra_grid)
        else:
            _h_fn = _rs_h
    else:
        _h_fn = _geom_h

    rs_expand_fn = None
    if _rs_expand and use_rs:
        def _rs_expand(cx, cy, cth, cost, path_node, expanded, t_start, st):
            euclidean_goal = math.hypot(cx - RS_GOAL_X, cy - RS_GOAL_Y)
            if euclidean_goal < rs_expansion_radius + 1.0:
                rs_dist = rs.rs_distance_pose(
                    cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS)
                if rs_dist < rs_expansion_radius:
                    rs_traj = rs.rs_sample_path(
                        cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                        MIN_TURN_RADIUS, step=DT * 0.5)
                    if rs_traj:
                        ex, ey, eth = rs_traj[-1]
                        goal_ok = (ex <= 2.25 and abs(ey) <= 0.18 and abs(eth) <= ALIGN_GOAL_DYAW)
                        traj_ok = _check_traj_collision(rs_traj, no_corridor, obstacles=fast_obs)
                        if goal_ok and traj_ok:
                            final = []
                            c = path_node
                            while c is not None:
                                final.append(c[1]); c = c[0]
                            final.reverse()
                            if st is not None:
                                st['expanded'] = expanded
                                st['elapsed_ms'] = round((time.perf_counter() - t_start) * 1000.0, 1)
                                st['use_rs'] = use_rs
                                st['no_corridor'] = no_corridor
                                st['rs_expansion'] = True
                            return True, final, rs_traj
            return None
        rs_expand_fn = _rs_expand

    st_inner = {} if stats is not None else None
    ok, acts, traj = _astar.plan_path(
        x0, y0, theta0, precomp_prim,
        collision_fn=_collision_fn,
        heuristic_fn=_h_fn,
        rs_expand_fn=rs_expand_fn,
        stats=st_inner,
        _goal_xmin=_goal_xmin, _goal_xmax=_goal_xmax,
        _goal_ymin=_goal_ymin, _goal_ymax=_goal_ymax, _goal_thmax=_goal_thmax,
        _step_limit=_step_limit, _expand_limit=_expand_limit, _prim_limit=_prim_limit,
        _rs_expand=(_rs_expand and use_rs),
        rs_expansion_radius=rs_expansion_radius)

    if stats is not None and st_inner is not None:
        stats.update(st_inner)
        stats.setdefault('use_rs', use_rs)
        stats.setdefault('no_corridor', no_corridor)

    return ok, acts, traj


def plan_path_robust(x0, y0, theta0, precomp_prim,
                     use_rs=False, stats=None, no_corridor=False,
                     rs_expansion_radius=2.5,
                     obstacles=None, dijkstra_grid=None):
    """向后兼容的 plan_path_robust — 桥接到新 planner / planner_obs"""
    if obstacles:
        from planner_obs import plan_path_robust_obs
        return plan_path_robust_obs(
            x0, y0, theta0, precomp_prim,
            use_rs=use_rs, stats=stats, no_corridor=no_corridor,
            rs_expansion_radius=rs_expansion_radius,
            obstacles=obstacles, dijkstra_grid=dijkstra_grid)
    else:
        from planner import plan_path_robust as _plan_robust
        return _plan_robust(
            x0, y0, theta0, precomp_prim,
            use_rs=use_rs, stats=stats, no_corridor=no_corridor,
            rs_expansion_radius=rs_expansion_radius)
