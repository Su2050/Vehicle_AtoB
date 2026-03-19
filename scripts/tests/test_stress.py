#!/usr/bin/env python3
"""
大规模并行压力测试脚本 - test_stress.py

特点：
1. 连续随机参数空间生成数千/数万测试用例
2. 使用 multiprocessing.Pool 满载 CPU 并行测试
3. 严格验证完整动作轨迹（action trajectory），暴露假阳性碰撞 Bug
4. 将轨迹碰撞的用例导出为 JSON 供 GUI 复现
5. 包含 staggered-slalom 专项批量集，用于压测 late-merge / gate / deep-stage 策略
"""

import os
import sys
import math
import time
import json
import random
import signal
import argparse
from datetime import datetime
import multiprocessing as mp

# 确保能 import 上层目录的模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import primitives
from primitives import (init_primitives, simulate_path, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
                        ALIGN_GOAL_DYAW)
from collision import check_collision
from planner_obs import _preprocess_obstacles
from planner_obs_v2 import plan_path_robust_obs_v2

# ============================================================================
# Worker 进程全局状态
# ============================================================================
g_prims = None

def worker_init():
    """Pool worker 初始化函数，预计算运动基元，避免每个 case 重复计算"""
    global g_prims
    # 忽略键盘中断，交由主进程处理
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    g_prims = init_primitives()

class TimeoutError(Exception):
    pass

def _timeout_handler(signum, frame):
    raise TimeoutError("Planning timed out")

# ============================================================================
# 测试用例生成器
# ============================================================================

def _is_start_valid(x, y, th, obstacles):
    """预过滤：起点必须不在障碍物/走廊内（与轨迹验证使用相同的严格模式）"""
    fast_obs = _preprocess_obstacles(obstacles) if obstacles else None
    valid, _ = check_collision(x, y, th, no_corridor=False, obstacles=fast_obs)
    return valid

def _is_goal_blocked(obstacles):
    """启发式：检查目标区域是否被完全覆盖，若是则预期必定失败"""
    fast_obs = _preprocess_obstacles(obstacles) if obstacles else None
    valid, _ = check_collision(RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, no_corridor=True, obstacles=fast_obs)
    return not valid


def _build_slalom_staggered_scene(rng):
    """
    构造一组 3 障碍交错的 slalom 场景。

    几何目标：
      - 保留两个显式 gate，让 L1.8 的同伦选择更敏感
      - 让起点通常位于障碍簇同侧，且带明显偏航误差
      - 避免大面积生成显然 goal-blocked 的死局
    """
    x_base = rng.uniform(3.25, 4.10)
    x_offsets = [
        rng.uniform(-0.18, 0.06),
        rng.uniform(0.15, 0.48),
        rng.uniform(-0.05, 0.22),
    ]
    widths = [
        rng.uniform(0.32, 0.50),
        rng.uniform(0.38, 0.62),
        rng.uniform(0.35, 0.56),
    ]

    low_y = rng.uniform(-2.80, -2.20)
    low_h = rng.uniform(0.72, 1.00)
    gap0 = rng.uniform(0.78, 1.12)
    mid_y = low_y + low_h + gap0
    mid_h = rng.uniform(0.95, 1.28)
    gap1 = rng.uniform(0.72, 1.05)
    high_y = mid_y + mid_h + gap1
    high_h = rng.uniform(0.75, 1.05)

    top = high_y + high_h
    if top > 2.70:
        shift = top - 2.70
        low_y -= shift
        mid_y -= shift
        high_y -= shift
    if low_y < -2.95:
        shift = -2.95 - low_y
        low_y += shift
        mid_y += shift
        high_y += shift

    obstacles = [
        {'x': round(x_base + x_offsets[0], 2), 'y': round(low_y, 2), 'w': round(widths[0], 2), 'h': round(low_h, 2)},
        {'x': round(x_base + x_offsets[1], 2), 'y': round(mid_y, 2), 'w': round(widths[1], 2), 'h': round(mid_h, 2)},
        {'x': round(x_base + x_offsets[2], 2), 'y': round(high_y, 2), 'w': round(widths[2], 2), 'h': round(high_h, 2)},
    ]

    cluster_x_max = max(ob['x'] + ob['w'] for ob in obstacles)
    upper_clear_y = min(2.95, high_y + high_h + rng.uniform(0.55, 1.00))
    lower_clear_y = max(-2.95, low_y - rng.uniform(0.55, 1.00))
    start_x_anchor = cluster_x_max + rng.uniform(0.70, 2.10)

    starts = [
        (
            start_x_anchor + rng.uniform(0.00, 0.90),
            upper_clear_y,
            math.radians(rng.uniform(60.0, 105.0)),
        ),
        (
            start_x_anchor + rng.uniform(-0.20, 0.70),
            lower_clear_y,
            math.radians(rng.uniform(-105.0, -60.0)),
        ),
    ]
    return obstacles, starts

def generate_cases(profile="quick", seed=42):
    """
    根据档位生成测试用例集合
    """
    rng = random.Random(seed)
    cases = []
    
    profiles = {
        "quick": {"n_random": 100, "n_passage": 50, "n_blocked": 20, "n_slalom": 30, "grid_density": 1.0},
        "standard": {"n_random": 1000, "n_passage": 300, "n_blocked": 100, "n_slalom": 120, "grid_density": 0.5},
        "thorough": {"n_random": 4000, "n_passage": 1000, "n_blocked": 200, "n_slalom": 400, "grid_density": 0.25},
    }
    p = profiles.get(profile, profiles["quick"])
    
    case_id = 0
    
    def add_case(type_name, x, y, th, obstacles, expect_fail=False):
        nonlocal case_id
        # 先取整再做预过滤，确保预过滤和轨迹验证使用完全相同的坐标值
        rx, ry, rth = round(x, 2), round(y, 2), round(th, 3)
        if not _is_start_valid(rx, ry, rth, obstacles):
            return # 过滤掉非法的起点
            
        if not expect_fail and _is_goal_blocked(obstacles):
            expect_fail = True
            
        case_id += 1
        cases.append({
            'id': case_id,
            'type': type_name,
            'x': rx,
            'y': ry,
            'th': rth,
            'obstacles': obstacles,
            'expect_fail': expect_fail
        })

    print(f"Generating cases (profile={profile}, seed={seed})...")
    
    # 1. 随机单/多障碍物
    for _ in range(p["n_random"]):
        n_obs = rng.randint(1, 4)
        obstacles = []
        for _ in range(n_obs):
            obstacles.append({
                'x': round(rng.uniform(2.5, 7.5), 2),
                'y': round(rng.uniform(-3.0, 3.0), 2),
                'w': round(rng.uniform(0.3, 2.0), 2),
                'h': round(rng.uniform(0.3, 2.0), 2)
            })
        
        # 随机几个起点
        for _ in range(3):
            sx = rng.uniform(2.5, 8.5)
            sy = rng.uniform(-3.0, 3.0)
            sth = rng.uniform(-math.pi, math.pi)
            add_case("RandomMulti", sx, sy, sth, obstacles)

    # 2. 参数化窄通道 (夹缝)
    for _ in range(p["n_passage"]):
        gap_w = rng.uniform(0.8, 2.5) # 0.8非常窄, 2.5比较宽
        gap_y = rng.uniform(-1.5, 1.5)
        wall_x = rng.uniform(3.5, 6.0)
        wall_thick = rng.uniform(0.3, 1.0)
        
        obstacles = [
            {'x': round(wall_x, 2), 'y': round(gap_y + gap_w/2, 2), 'w': round(wall_thick, 2), 'h': 4.0},
            {'x': round(wall_x, 2), 'y': round(gap_y - gap_w/2 - 4.0, 2), 'w': round(wall_thick, 2), 'h': 4.0}
        ]
        
        sx = rng.uniform(wall_x + wall_thick + 0.5, 8.5)
        sy = rng.uniform(-3.0, 3.0)
        sth = rng.uniform(-math.pi, math.pi)
        add_case(f"Passage(gap={gap_w:.1f})", sx, sy, sth, obstacles)

    # 3. 目标被阻塞 (Expected Fail)
    for _ in range(p["n_blocked"]):
        obstacles = [
            {'x': round(rng.uniform(1.8, 2.2), 2), 
             'y': round(rng.uniform(-0.5, 0.0), 2), 
             'w': round(rng.uniform(0.5, 1.0), 2), 
             'h': round(rng.uniform(0.5, 1.5), 2)}
        ]
        sx = rng.uniform(4.0, 8.0)
        sy = rng.uniform(-2.0, 2.0)
        sth = rng.uniform(-math.pi, math.pi)
        add_case("GoalBlocked", sx, sy, sth, obstacles, expect_fail=True)

    # 4. 交错 slalom 专项集
    for _ in range(p["n_slalom"]):
        obstacles, starts = _build_slalom_staggered_scene(rng)
        for sx, sy, sth in starts:
            add_case("SlalomStaggered", sx, sy, sth, obstacles)

    print(f"Generated {len(cases)} valid test cases after filtering.")
    return cases

# ============================================================================
# Worker 执行逻辑
# ============================================================================

def run_case_worker(case):
    """
    在 Worker 进程中执行一个测试用例
    包含：规划 -> 轨迹重建 -> 逐点碰撞检测
    """
    global g_prims
    
    x, y, th = case['x'], case['y'], case['th']
    obstacles = case['obstacles']
    expect_fail = case.get('expect_fail', False)
    
    timeout_s = 32 # 压力测试单用例超时（SIGALRM），留 4s 余量给 cleanup
    
    stats = {}
    ok = False
    timed_out = False
    t_start = time.perf_counter()
    
    old_handler = signal.getsignal(signal.SIGALRM)
    try:
        signal.signal(signal.SIGALRM, _timeout_handler)
        signal.alarm(timeout_s)
        # 调用规划器 v2
        # Pass _time_budget=28.0 so the planner has ~28s real planning time
        # (SIGALRM fires at 32s; 4s margin for cleanup)
        ok, acts, rs_traj = plan_path_robust_obs_v2(
            x, y, th, g_prims,
            use_rs=True, stats=stats, obstacles=obstacles,
            _time_budget=28.0
        )
    except TimeoutError:
        timed_out = True
        ok = False
        acts, rs_traj = None, None
    finally:
        signal.alarm(0)
        signal.signal(signal.SIGALRM, old_handler)
        
    elapsed_ms = (time.perf_counter() - t_start) * 1000.0
    
    result = {
        'id': case['id'],
        'type': case['type'],
        'ok': ok,
        'elapsed_ms': elapsed_ms,
        'expanded': stats.get('expanded', 0),
        'level': stats.get('level', ''),
        'timed_out': timed_out,
        'expect_fail': expect_fail,
        'collision': False,
        'dangerous_margin': False,
        'min_margin': 999.0
    }
    
    # Attach full case info for timeout analysis
    if timed_out:
        result['full_case'] = case
    
    # ── 核心：完整轨迹碰撞验证 ──
    if ok and not expect_fail:
        full_traj = []
        if acts:
            try:
                full_traj = simulate_path(x, y, th, acts, g_prims)
            except Exception as e:
                result['collision'] = True
                result['collision_reason'] = f"simulate_path error: {str(e)}"
        else:
            full_traj = [(x, y, th)]
            
        if rs_traj:
            full_traj.extend(rs_traj)
            
        fast_obs = _preprocess_obstacles(obstacles) if obstacles else None
        
        has_collision = False
        min_margin_sq = 999.0
        
        for pt in full_traj:
            nx, ny, nth = pt[0], pt[1], pt[2]
            
            # 1. 严格碰撞检测 (包含走廊)
            valid, reason = check_collision(nx, ny, nth, no_corridor=False, obstacles=fast_obs)
            if not valid:
                has_collision = True
                result['collision_reason'] = reason
                result['collision_pt'] = (nx, ny, nth)
                if rs_traj and (nx, ny, nth) in rs_traj:
                    result['collision_source'] = 'rs_traj'
                else:
                    result['collision_source'] = 'acts'
                break
                
            # 2. 计算危险边距 (多圆模型 - 车身表面到障碍物最小距离)
            if obstacles:
                sin_nth = math.sin(nth)
                cos_nth = math.cos(nth)
                for obs in obstacles:
                    ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
                    min_x, max_x = min(ox, ox+ow), max(ox, ox+ow)
                    min_y, max_y = min(oy, oy+oh), max(oy, oy+oh)
                    
                    for offset in primitives.VEHICLE_CHECK_OFFSETS:
                        px = nx + offset * cos_nth
                        py = ny + offset * sin_nth
                        dx = px - max_x if px > max_x else (min_x - px if px < min_x else 0.0)
                        dy = py - max_y if py > max_y else (min_y - py if py < min_y else 0.0)
                        dist_sq = dx*dx + dy*dy
                        if dist_sq < min_margin_sq:
                            min_margin_sq = dist_sq

        result['collision'] = has_collision
        # min_margin = circle center to AABB edge distance - half_width = surface clearance
        if min_margin_sq < 999.0:
            min_margin = math.sqrt(min_margin_sq) - primitives.VEHICLE_HALF_WIDTH
            if min_margin < 0.0:
                min_margin = 0.0  # collision already caught above
        else:
            min_margin = 999.0
        result['min_margin'] = min_margin
        
        if min_margin < 0.15 and not has_collision:
            result['dangerous_margin'] = True
            
        # 如果发生碰撞，附带完整参数以便 JSON 导出
        if has_collision:
            result['full_case'] = case
            result['acts'] = acts
            result['rs_traj'] = rs_traj

    return result

# ============================================================================
# 主控逻辑
# ============================================================================

def _percentile(sorted_vals, p):
    """计算百分位数（线性插值）"""
    if not sorted_vals:
        return 0.0
    idx = (len(sorted_vals) - 1) * p / 100.0
    lo = int(idx)
    hi = min(lo + 1, len(sorted_vals) - 1)
    frac = idx - lo
    return sorted_vals[lo] * (1 - frac) + sorted_vals[hi] * frac


def _print_timing_analysis(records, profile):
    """打印全量计时分布分析"""
    print("\n" + "=" * 60)
    print(" 📊 TIMING DISTRIBUTION ANALYSIS (for production threshold)")
    print("=" * 60)

    # 成功 case 的计时分析（最有价值：生产只关心能规划出来的场景）
    succ_ms = sorted(r['elapsed_ms'] for r in records if r['category'] == 'SUCCESS')
    timeout_ms = sorted(r['elapsed_ms'] for r in records if r['category'] == 'TIMEOUT')
    unexpfail_ms = sorted(r['elapsed_ms'] for r in records if r['category'] == 'UNEXPECTED_FAIL')
    all_ms = sorted(r['elapsed_ms'] for r in records)

    if succ_ms:
        print(f"\n  ✅ SUCCESS cases ({len(succ_ms)} total):")
        print(f"     Min   : {succ_ms[0]:>10.1f} ms")
        print(f"     P50   : {_percentile(succ_ms, 50):>10.1f} ms")
        print(f"     P75   : {_percentile(succ_ms, 75):>10.1f} ms")
        print(f"     P90   : {_percentile(succ_ms, 90):>10.1f} ms")
        print(f"     P95   : {_percentile(succ_ms, 95):>10.1f} ms")
        print(f"     P99   : {_percentile(succ_ms, 99):>10.1f} ms")
        print(f"     Max   : {succ_ms[-1]:>10.1f} ms")
        print(f"     Mean  : {sum(succ_ms)/len(succ_ms):>10.1f} ms")

    if timeout_ms:
        print(f"\n  ⏱️  TIMEOUT cases ({len(timeout_ms)} total):")
        print(f"     Min   : {timeout_ms[0]:>10.1f} ms")
        print(f"     Max   : {timeout_ms[-1]:>10.1f} ms")
        print(f"     Mean  : {sum(timeout_ms)/len(timeout_ms):>10.1f} ms")

    if unexpfail_ms:
        print(f"\n  ⚠️  UNEXPECTED_FAIL cases ({len(unexpfail_ms)} total):")
        print(f"     Min   : {unexpfail_ms[0]:>10.1f} ms")
        print(f"     Max   : {unexpfail_ms[-1]:>10.1f} ms")
        print(f"     Mean  : {sum(unexpfail_ms)/len(unexpfail_ms):>10.1f} ms")

    # 按规划级别细分成功 case 的耗时
    level_times = {}
    for r in records:
        if r['category'] == 'SUCCESS':
            lv = r['level'] or 'unknown'
            level_times.setdefault(lv, []).append(r['elapsed_ms'])
    if level_times:
        print(f"\n  📈 Timing by planner level (SUCCESS only):")
        for lv in sorted(level_times.keys()):
            vals = sorted(level_times[lv])
            print(f"     {lv:30s} : n={len(vals):>4d}, "
                  f"P50={_percentile(vals, 50):>8.1f}ms, "
                  f"P95={_percentile(vals, 95):>8.1f}ms, "
                  f"max={vals[-1]:>8.1f}ms")

    # 生产超时阈值建议
    if succ_ms:
        p99 = _percentile(succ_ms, 99)
        p95 = _percentile(succ_ms, 95)
        print(f"\n  💡 Production Timeout Recommendation:")
        print(f"     Conservative (P99 × 1.5) : {p99 * 1.5 / 1000.0:>6.1f}s")
        print(f"     Moderate    (P95 × 2.0)  : {p95 * 2.0 / 1000.0:>6.1f}s")
        print(f"     Aggressive  (P99 × 1.2)  : {p99 * 1.2 / 1000.0:>6.1f}s")
    print("=" * 60)


def _export_timing_data(records, profile):
    """导出全量计时数据到 JSON，供后续分析"""
    os.makedirs("logs", exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    json_path = f"logs/stress_timing_{profile}_{ts}.json"

    export = {
        'profile': profile,
        'timestamp': ts,
        'total_cases': len(records),
        'summary': {},
        'records': records,
    }

    # 汇总统计
    succ = [r['elapsed_ms'] for r in records if r['category'] == 'SUCCESS']
    if succ:
        succ_sorted = sorted(succ)
        export['summary']['success'] = {
            'count': len(succ),
            'min_ms': succ_sorted[0],
            'p50_ms': round(_percentile(succ_sorted, 50), 1),
            'p75_ms': round(_percentile(succ_sorted, 75), 1),
            'p90_ms': round(_percentile(succ_sorted, 90), 1),
            'p95_ms': round(_percentile(succ_sorted, 95), 1),
            'p99_ms': round(_percentile(succ_sorted, 99), 1),
            'max_ms': succ_sorted[-1],
            'mean_ms': round(sum(succ) / len(succ), 1),
        }
    timeout = [r['elapsed_ms'] for r in records if r['category'] == 'TIMEOUT']
    if timeout:
        export['summary']['timeout'] = {
            'count': len(timeout),
            'min_ms': min(timeout),
            'max_ms': max(timeout),
            'mean_ms': round(sum(timeout) / len(timeout), 1),
        }
    unexpfail = [r['elapsed_ms'] for r in records if r['category'] == 'UNEXPECTED_FAIL']
    if unexpfail:
        export['summary']['unexpected_fail'] = {
            'count': len(unexpfail),
            'min_ms': min(unexpfail),
            'max_ms': max(unexpfail),
            'mean_ms': round(sum(unexpfail) / len(unexpfail), 1),
        }

    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(export, f, indent=2)
    print(f"\n=> Exported full timing data ({len(records)} records) to {json_path}")
    print(f"   Use this data to determine production timeout threshold.")


def main():
    parser = argparse.ArgumentParser(description="大规模并行压力测试脚本")
    parser.add_argument('--profile', choices=['quick', 'standard', 'thorough'], default='quick',
                        help='测试强度：quick(~360) / standard(~3240) / thorough(~10800)，含 slalom 专项集')
    parser.add_argument('--workers', type=int, default=None,
                        help='并发 Worker 数量，默认等于 CPU 核心数')
    parser.add_argument('--seed', type=int, default=42, help='随机种子')
    parser.add_argument('--vehicle-length', type=float, default=1.5,
                        help='车辆总长(m)，默认 1.5')
    parser.add_argument('--vehicle-width', type=float, default=0.5,
                        help='车辆总宽(m)，默认 0.5')
    args = parser.parse_args()

    # 配置车辆碰撞模型
    primitives.configure_vehicle(args.vehicle_length, args.vehicle_width)
    print(f"  Vehicle: {args.vehicle_length}m x {args.vehicle_width}m "
          f"-> {len(primitives.VEHICLE_CHECK_OFFSETS)} circles, "
          f"r={primitives.VEHICLE_HALF_WIDTH:.2f}m")

    cases = generate_cases(profile=args.profile, seed=args.seed)
    total_cases = len(cases)
    
    if total_cases == 0:
        print("No valid cases generated.")
        return

    workers = args.workers or mp.cpu_count()
    print(f"\nStarting pool with {workers} workers for {total_cases} cases...\n")
    
    pool = mp.Pool(processes=workers, initializer=worker_init)
    
    counts = {
        'SUCCESS': 0,
        'TRAJ_COLLISION': 0,
        'DANGEROUS_MARGIN': 0,
        'EXPECTED_FAIL': 0,
        'UNEXPECTED_FAIL': 0,
        'TIMEOUT': 0
    }
    
    collision_cases_to_export = []
    timeout_cases_to_export = []
    # ── 全量计时统计（用于确定生产超时阈值）──
    all_timing_records = []  # 收集每个 case 的计时数据
    
    try:
        try:
            from tqdm import tqdm
            pbar = tqdm(total=total_cases, desc="Testing", unit="case", dynamic_ncols=True)
            use_tqdm = True
        except ImportError:
            use_tqdm = False
            print("Progress: ", end="", flush=True)

        for i, res in enumerate(pool.imap_unordered(run_case_worker, cases)):
            # 分类统计
            if res['ok']:
                if res['collision']:
                    counts['TRAJ_COLLISION'] += 1
                    collision_cases_to_export.append(res)
                else:
                    counts['SUCCESS'] += 1
                    if res.get('dangerous_margin'):
                        counts['DANGEROUS_MARGIN'] += 1
            else:
                if res['timed_out']:
                    counts['TIMEOUT'] += 1
                    timeout_cases_to_export.append(res)
                elif res['expect_fail']:
                    counts['EXPECTED_FAIL'] += 1
                else:
                    counts['UNEXPECTED_FAIL'] += 1
            
            # ── 收集每个 case 的计时记录 ──
            category = 'SUCCESS'
            if res['timed_out']:
                category = 'TIMEOUT'
            elif not res['ok'] and res.get('expect_fail'):
                category = 'EXPECTED_FAIL'
            elif not res['ok']:
                category = 'UNEXPECTED_FAIL'
            elif res.get('collision'):
                category = 'TRAJ_COLLISION'
            all_timing_records.append({
                'case_id': res['id'],
                'type': res['type'],
                'category': category,
                'elapsed_ms': round(res['elapsed_ms'], 1),
                'expanded': res.get('expanded', 0),
                'level': res.get('level', ''),
            })

            if use_tqdm:
                pbar.set_postfix({
                    'Succ': counts['SUCCESS'], 
                    'COLLISION': counts['TRAJ_COLLISION'],
                    'UnexpFail': counts['UNEXPECTED_FAIL'],
                    'Timeout': counts['TIMEOUT']
                })
                pbar.update(1)
            else:
                if i % max(1, total_cases // 100) == 0:
                    print(".", end="", flush=True)

        if use_tqdm:
            pbar.close()
        else:
            print("\nDone.")
            
    except KeyboardInterrupt:
        print("\nTest interrupted by user. Shutting down workers...")
        pool.terminate()
        pool.join()
        sys.exit(1)
        
    pool.close()
    pool.join()

    print("\n" + "=" * 60)
    print(" 🚨 STRESS TEST REPORT 🚨")
    print("=" * 60)
    print(f"Total Cases Tested : {total_cases}")
    print(f"Profile            : {args.profile}")
    print("-" * 60)
    print(f"✅ SUCCESS (Safe)   : {counts['SUCCESS']} (Contains {counts['DANGEROUS_MARGIN']} dangerous margins)")
    print(f"🎯 EXPECTED FAIL    : {counts['EXPECTED_FAIL']} (Goal blocked, handled correctly)")
    print(f"⚠️ UNEXPECTED FAIL  : {counts['UNEXPECTED_FAIL']} (False Negatives, normal in complex mazes)")
    print(f"⏱️ TIMEOUT          : {counts['TIMEOUT']}")
    print("-" * 60)
    
    col_str = f"{counts['TRAJ_COLLISION']}"
    if counts['TRAJ_COLLISION'] > 0:
        print(f"🔥 TRAJ_COLLISION   : {col_str} (FALSE POSITIVES! PLAN CLAIMED OK BUT CRASHED!)")
    else:
        print(f"✨ TRAJ_COLLISION   : 0 (No false positives found!)")
    print("=" * 60)

    if counts['TRAJ_COLLISION'] > 0:
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_path = f"logs/stress_collisions_{ts}.json"
        
        export_data = []
        for c in collision_cases_to_export:
            export_data.append({
                'case_id': c['id'],
                'type': c['type'],
                'start': {'x': c['full_case']['x'], 'y': c['full_case']['y'], 'th': c['full_case']['th']},
                'obstacles': c['full_case']['obstacles'],
                'collision_pt': c.get('collision_pt'),
                'collision_reason': c.get('collision_reason'),
                'collision_source': c.get('collision_source')
            })
            
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(export_data, f, indent=2)
            
        print(f"\n=> Exported {len(export_data)} collision cases to {json_path}")
        print("   Use these parameters to reproduce the physical collisions in GUI.")

    # ── 导出超时用例详情 ──
    if timeout_cases_to_export:
        os.makedirs("logs", exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        json_path_to = f"logs/stress_timeouts_{ts}.json"

        to_data = []
        for c in timeout_cases_to_export:
            fc = c.get('full_case', {})
            to_data.append({
                'case_id': c['id'],
                'type': c['type'],
                'elapsed_ms': round(c['elapsed_ms'], 1),
                'expanded': c['expanded'],
                'level': c.get('level', ''),
                'start': {'x': fc.get('x'), 'y': fc.get('y'), 'th': fc.get('th')},
                'obstacles': fc.get('obstacles', []),
                'n_obs': len(fc.get('obstacles', [])),
            })

        with open(json_path_to, 'w', encoding='utf-8') as f:
            json.dump(to_data, f, indent=2)

        # ── 打印超时分析摘要 ──
        print(f"\n=> Exported {len(to_data)} timeout cases to {json_path_to}")
        from collections import Counter
        type_counts = Counter(c['type'] for c in to_data)
        nobs_counts = Counter(c['n_obs'] for c in to_data)
        print(f"\n  Timeout by case type:")
        for t, cnt in type_counts.most_common():
            print(f"    {t}: {cnt}")
        print(f"  Timeout by obstacle count:")
        for n, cnt in sorted(nobs_counts.items()):
            print(f"    {n} obstacles: {cnt}")
        elapsed_vals = [c['elapsed_ms'] for c in to_data]
        if elapsed_vals:
            print(f"  Elapsed: min={min(elapsed_vals):.0f}ms, max={max(elapsed_vals):.0f}ms, "
                  f"avg={sum(elapsed_vals)/len(elapsed_vals):.0f}ms")

    # ══════════════════════════════════════════════════════════════════════
    # 📊 全量计时分析（用于确定生产超时阈值）
    # ══════════════════════════════════════════════════════════════════════
    _print_timing_analysis(all_timing_records, args.profile)
    _export_timing_data(all_timing_records, args.profile)

if __name__ == "__main__":
    mp.freeze_support()
    main()
