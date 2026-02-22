import os
import sys
import math
import time
import json
import io
import re
import signal
from datetime import datetime

import main
from test_path_quality import satisfies_goal, _reconstruct_endpoint, capture_stderr

class TimeoutError(Exception):
    pass

def _timeout_handler(signum, frame):
    raise TimeoutError("Planning timed out")

# 预定义的多种障碍物场景（不同位置 / 大小 / 组合）
SCENARIOS = {
    # ── 单障碍物：不同 x 位置 ──
    "A_Near_Goal": [
        {'x': 3.0, 'y': -1.5, 'w': 0.6, 'h': 1.2}   # 靠近目标
    ],
    "B_Mid_Path": [
        {'x': 4.0, 'y': -1.8, 'w': 0.8, 'h': 1.5}   # 中段路径
    ],
    "C_Far_Side": [
        {'x': 5.5, 'y': 0.3, 'w': 0.7, 'h': 1.0}    # 远端正上方
    ],
    # ── 单障碍物：不同 y 位置 ──
    "D_Upper_Block": [
        {'x': 3.5, 'y': 0.5, 'w': 0.8, 'h': 1.5}    # 上方挡路
    ],
    "E_Lower_Block": [
        {'x': 3.5, 'y': -2.5, 'w': 0.8, 'h': 2.0}   # 下方大障碍物
    ],
    # ── 双障碍物（间距足够宽，叉车可通过）──
    "F_Dual_Offset": [
        {'x': 3.5, 'y': -2.0, 'w': 0.6, 'h': 1.0},
        {'x': 5.5, 'y': 0.8, 'w': 0.6, 'h': 1.0}    # 交错分布，y 间距 > 2m
    ],
    # ── 大障碍物 ──
    "G_Wide_Block": [
        {'x': 4.0, 'y': -1.0, 'w': 1.5, 'h': 0.8}   # 宽矮障碍物
    ],
}

CASE_PROFILES = {
    "quick": {
        "x_vals": [3.5, 4.5, 5.5, 6.5, 7.5],
        "y_vals": [-1.5, -0.5, 0.5, 1.5],
        "angles_deg": [0, 90, 180, -90, -135],
    },
    "full": {
        "x_vals": [3.0, 3.5, 4.5, 5.5, 6.5, 7.5],
        "y_vals": [-2.5, -1.5, -0.5, 0.5, 1.5, 2.5],
        "angles_deg": [0, 45, 90, 135, 180, -45, -90, -135],
    },
}

def generate_test_cases(obstacles, profile="quick"):
    """
    生成测试用例，自动过滤掉在障碍物内部（或碰撞）的起点
    """
    p = CASE_PROFILES.get(profile, CASE_PROFILES["quick"])
    x_vals = p["x_vals"]
    y_vals = p["y_vals"]
    angles_deg = p["angles_deg"]
    
    cases = []
    for x in x_vals:
        for y in y_vals:
            for deg in angles_deg:
                th = math.radians(deg)
                # 使用实际的碰撞检测来严格剔除不合法的起点
                valid, _ = main.check_collision(x, y, th, no_corridor=False, obstacles=obstacles)
                if valid:
                    cases.append((x, y, th, deg))
    return cases

def _format_actions(acts):
    if not acts:
        return "[]"
    return " ".join(f"{g}/{s:.1f}/{d:.2f}" for g, s, d in acts)

def _mirror_obstacles_y(obstacles):
    mirrored = []
    for obs in obstacles:
        x, y, w, h = obs["x"], obs["y"], obs["w"], obs["h"]
        mirrored.append({"x": x, "y": -(y + h), "w": w, "h": h})
    return mirrored

def build_scenarios(include_mirror=False):
    scenarios = dict(SCENARIOS)
    if include_mirror:
        for name, obs in list(SCENARIOS.items()):
            scenarios[f"{name}_MirrorY"] = _mirror_obstacles_y(obs)
    return scenarios

def parse_resume_state(log_path, scenario_names):
    """
    解析已有日志，返回续跑状态。
    返回: dict 或 None
      - skip_scenarios: 已完成的场景名列表
      - resume_scenario: 未完成场景名（若有）
      - resume_from_case: 该场景从第几个 case 开始跑（1-based）
      - prev_global_total, prev_global_success: 已完成部分的统计（含未完成场景的已跑部分）
    """
    if not os.path.isfile(log_path):
        return None
    with open(log_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    skip_scenarios = []
    resume_scenario = None
    resume_from_case = 1
    prev_global_total = 0
    prev_global_success = 0
    in_partial = False
    partial_success = 0
    last_case_num = 0
    prev_partial_time = 0.0
    prev_partial_max = 0.0
    for line in lines:
        m = re.match(r'>>> 测试场景: (\w+) <<<', line)
        if m:
            if in_partial:
                in_partial = False
            current = m.group(1)
            if current not in skip_scenarios:
                in_partial = True
                partial_success = 0
                prev_partial_time = 0.0
                prev_partial_max = 0.0
        m = re.match(r'场景 (\w+) 汇总:', line)
        if m:
            skip_scenarios.append(m.group(1))
            in_partial = False
        m = re.match(r'  总用例数: (\d+), 成功: (\d+)', line)
        if m:
            t, s = int(m.group(1)), int(m.group(2))
            prev_global_total += t
            prev_global_success += s
        m = re.match(r'#(\d+)\s+\|.*\|\s+(YES|NO)\s+\|\s+([\d.]+)\s+\|', line)
        if m and in_partial:
            last_case_num = int(m.group(1))
            if m.group(2) == 'YES':
                partial_success += 1
            t = float(m.group(3))
            prev_partial_time += t
            if t > prev_partial_max:
                prev_partial_max = t
    scenario_list = list(scenario_names)
    for s in skip_scenarios:
        if s in scenario_list:
            scenario_list.remove(s)
    if scenario_list and last_case_num > 0:
        resume_scenario = scenario_list[0]
        resume_from_case = last_case_num + 1
        prev_global_total += last_case_num
        prev_global_success += partial_success
        prev_partial_success = partial_success
    else:
        prev_partial_success = 0
    return {
        'skip_scenarios': skip_scenarios,
        'resume_scenario': resume_scenario,
        'resume_from_case': resume_from_case,
        'prev_global_total': prev_global_total,
        'prev_global_success': prev_global_success,
        'prev_partial_success': prev_partial_success,
        'prev_partial_time': prev_partial_time,
        'prev_partial_max': prev_partial_max,
        'log_path': log_path,
    }

def run_obstacle_tests(
    resume_log=None,
    profile="quick",
    include_mirror=False,
    case_timeout=45,
    max_cases_per_scene=0,
):
    prims = main.init_primitives()
    os.makedirs('logs', exist_ok=True)
    scenarios = build_scenarios(include_mirror=include_mirror)
    resume = parse_resume_state(resume_log, scenarios.keys()) if resume_log else None
    if resume:
        log_path = resume['log_path']
        log_mode = 'a'
        skip_scenarios = set(resume['skip_scenarios'])
        resume_scenario = resume.get('resume_scenario')
        resume_from_case = resume.get('resume_from_case', 1)
        prev_global_total = resume['prev_global_total']
        prev_global_success = resume['prev_global_success']
        prev_partial_success = resume.get('prev_partial_success', 0)
        prev_partial_time = resume.get('prev_partial_time', 0.0)
        prev_partial_max = resume.get('prev_partial_max', 0.0)
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_path = os.path.join('logs', f'obs_test_{timestamp}.log')
        log_mode = 'w'
        skip_scenarios = set()
        resume_scenario = None
        resume_from_case = 1
        prev_global_total = 0
        prev_global_success = 0
        prev_partial_success = 0
        prev_partial_time = 0.0
        prev_partial_max = 0.0

    with open(log_path, log_mode, encoding='utf-8') as log_file:
        def log(msg=""):
            print(msg, flush=True)
            log_file.write(msg + '\n')
            log_file.flush()

        if resume:
            log("\n" + "=" * 96)
            log("=== 续跑（跳过已完成场景）===")
            log(f"续跑时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            log(f"跳过场景: {sorted(skip_scenarios)}")
            if resume_scenario:
                log(f"从场景 {resume_scenario} 第 {resume_from_case} 个 case 继续")
            log("=" * 96 + "\n")
        else:
            log("\n" + "=" * 96)
            log("=== 新测试任务 ===")
            log(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            log(f"测试档位: {profile}")
            log(f"镜像场景: {'ON' if include_mirror else 'OFF'}")
            log(f"单用例超时: {case_timeout}s")
            log(f"每场景case上限: {max_cases_per_scene if max_cases_per_scene > 0 else '不限制'}")
            log(f"场景总数: {len(scenarios)}")
            log("=" * 96 + "\n")

        global_total = prev_global_total
        global_success = prev_global_success

        for scene_name, obstacles in scenarios.items():
            if scene_name in skip_scenarios:
                continue
            is_resume_into = (scene_name == resume_scenario)
            if not is_resume_into:
                log(f"\n>>> 测试场景: {scene_name} <<<")
                log(f"障碍物: {obstacles}")
                header = f"{'Case':<8} | {'Pos(x, y, th)':<20} | {'Success':<7} | {'Time(ms)':<8} | {'Exp':<6} | {'Strategy':<10}"
                log("-" * 80)
                log(header)
                log("-" * 80)

            cases = generate_test_cases(obstacles, profile=profile)
            if max_cases_per_scene and max_cases_per_scene > 0:
                cases = cases[:max_cases_per_scene]
            total = len(cases)
            success_count = prev_partial_success if is_resume_into else 0
            fail_cases = []
            total_time = prev_partial_time if is_resume_into else 0.0
            max_time = prev_partial_max if is_resume_into else 0.0
            if is_resume_into:
                prev_partial_success = 0
                prev_partial_time = 0.0
                prev_partial_max = 0.0

            for idx, (x, y, th, deg) in enumerate(cases, 1):
                if is_resume_into and idx < resume_from_case:
                    continue
                st = {}
                ok = False
                timed_out = False
                
                def _plan():
                    return main.plan_path_robust(x, y, th, prims, use_rs=True, stats=st, obstacles=obstacles)
                
                t_case_start = time.perf_counter()
                old_handler = signal.getsignal(signal.SIGALRM)
                try:
                    signal.signal(signal.SIGALRM, _timeout_handler)
                    signal.alarm(case_timeout)
                    (ok, acts, rs_traj), _ = capture_stderr(_plan)
                except TimeoutError:
                    timed_out = True
                    ok = False
                finally:
                    signal.alarm(0)
                    signal.signal(signal.SIGALRM, old_handler)
                
                elapsed = st.get('elapsed_ms', (time.perf_counter() - t_case_start) * 1000.0)
                expanded = st.get('expanded', 0)
                strategy = '2-Stage' if st.get('two_stage') else 'Direct'
                if st.get('out_of_range'):
                    strategy = 'OOR'
                if timed_out:
                    strategy += '+TIMEOUT'
                
                total_time += elapsed
                if elapsed > max_time:
                    max_time = elapsed
                    
                case_id = f"#{idx:03d}"
                pos_str = f"({x:.1f}, {y:4.1f}, {deg:4d}°)"
                
                if ok:
                    success_count += 1
                    log(f"{case_id:<8} | {pos_str:<20} | {'YES':<7} | {elapsed:<8.1f} | {expanded:<6} | {strategy:<10}")
                else:
                    reason = "Timeout" if timed_out else ("Out of range" if st.get('out_of_range') else "No path found")
                    if st.get('two_stage'):
                        p0, s1, s15, s2 = st.get('phase0_ms', 0), st.get('stage1_ms', 0), st.get('stage15_ms', 0), st.get('stage2_ms', 0)
                        s1_end = st.get('stage1_end', None)
                        if s1_end:
                            s1_end_str = f"S1End:({s1_end[0]:.1f},{s1_end[1]:.1f},{s1_end[2]:.1f})"
                        else:
                            s1_end_str = "S1End:None"
                        fr = st.get('fail_reason') or reason
                        detail = f"{fr} (P0:{p0:.0f}ms, S1:{s1:.0f}ms, S1.5:{s15:.0f}ms, S2:{s2:.0f}ms, {s1_end_str})"
                    else:
                        detail = reason
                    log(f"{case_id:<8} | {pos_str:<20} | {'NO':<7} | {elapsed:<8.1f} | {expanded:<6} | {strategy:<10} | {detail}")
                    fail_cases.append((idx, x, y, deg, detail))
                    
            log("-" * 80)
            log(f"场景 {scene_name} 汇总:")
            log(f"  总用例数: {total}, 成功: {success_count}, 失败: {total - success_count}")
            log(f"  成功率: {(success_count/total*100) if total > 0 else 0:.1f}%")
            log(f"  平均耗时: {(total_time/total) if total > 0 else 0:.1f} ms, 最大耗时: {max_time:.1f} ms")
            if fail_cases:
                log("  失败用例:")
                for idx, x, y, deg, reason in fail_cases:
                    log(f"    - #{idx:03d}: ({x:.1f}, {y:.1f}, {deg}°) -> {reason}")
                    
            global_total += total
            global_success += success_count
            
        log("\n" + "=" * 96)
        log("=== 最终全局汇总 ===")
        log(f"总计用例: {global_total}")
        log(f"总计成功: {global_success}")
        success_rate = (global_success/global_total*100) if global_total > 0 else 0
        log(f"全局成功率: {success_rate:.1f}%")
        log("=" * 96)

        # ----------------------------------------------------
        # 回归门 (Regression Gates)
        # ----------------------------------------------------
        failed_count = global_total - global_success
        baseline_rate = 95.0
        baseline_failed = 27

        # 基线是按 quick 档位维护的；更全面档位只输出统计，不做强制失败
        enforce_regression_gate = (
            profile == "quick" and not include_mirror and max_cases_per_scene == 0
        )

        passed_regression = True
        log("\n=== 回归门检查 ===")
        if not enforce_regression_gate:
            log("[提示] 当前为扩展测试配置，跳过固定基线门控（仅输出统计结果）。")
            log(f"[统计] 全局成功率: {success_rate:.1f}%")
            log(f"[统计] 失败用例数: {failed_count}")
        else:
            if success_rate < baseline_rate:
                log(f"[失败] 全局成功率 {success_rate:.1f}% 低于基线 {baseline_rate}%")
                passed_regression = False
            else:
                log(f"[通过] 全局成功率 {success_rate:.1f}% >= {baseline_rate}%")

            if failed_count > baseline_failed:
                log(f"[失败] 失败用例数 {failed_count} 高于基线 {baseline_failed}")
                passed_regression = False
            else:
                log(f"[通过] 失败用例数 {failed_count} <= {baseline_failed}")

            if passed_regression:
                log("\n结论: [回归门通过] ✅ 当前修改符合要求！")
            else:
                log("\n结论: [回归门未通过] ❌ 当前修改存在倒退风险，建议回退！")
                import sys
                sys.exit(1)

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description='带障碍物的路径规划批量测试')
    p.add_argument('--resume', '-r', metavar='LOG', help='从指定日志续跑，跳过已完成的场景和用例')
    p.add_argument('--profile', choices=['quick', 'full'], default='quick',
                   help='测试强度：quick(默认) / full(更全面)')
    p.add_argument('--mirror', action='store_true',
                   help='增加 y 轴镜像场景（覆盖左右对称性）')
    p.add_argument('--case-timeout', type=int, default=45,
                   help='单用例超时秒数（默认45）')
    p.add_argument('--max-cases-per-scene', type=int, default=0,
                   help='每个场景最多跑多少 case（0 表示不限制）')
    args = p.parse_args()
    run_obstacle_tests(
        resume_log=args.resume,
        profile=args.profile,
        include_mirror=args.mirror,
        case_timeout=args.case_timeout,
        max_cases_per_scene=args.max_cases_per_scene,
    )
