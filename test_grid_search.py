import os
import sys
import math
import time
import json
import io
from datetime import datetime

import main
from test_path_quality import satisfies_goal, _reconstruct_endpoint, _check_path_in_workspace, capture_stderr

def generate_test_cases():
    """
    位置生成 (20个)：在合理的初始工作区内进行均匀网格采样
    X轴取 5 个点，Y轴取 4 个点，组合出 20 个不同位置。
    角度生成 (8个)：0°, 45°, 90°, 135°, 180°, -45°, -90°, -135°
    """
    x_vals = [2.5, 3.5, 4.5, 5.5, 6.5]
    y_vals = [-2.0, -0.6, 0.6, 2.0]
    angles_deg = [0, 45, 90, 135, 180, -45, -90, -135]
    
    cases = []
    for x in x_vals:
        for y in y_vals:
            for deg in angles_deg:
                th = math.radians(deg)
                cases.append((x, y, th, deg))
    return cases

def _format_actions(acts):
    """将动作列表转为可读字符串，如 F/0.5/0.33 R/-1.0/0.50 ..."""
    if not acts:
        return "[]"
    return " ".join(f"{g}/{s:.1f}/{d:.2f}" for g, s, d in acts)

def run_grid_tests():
    cases = generate_test_cases()
    prims = main.init_primitives()
    
    os.makedirs('logs', exist_ok=True)
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_path = os.path.join('logs', f'grid_test_{timestamp}.log')
    traj_path = os.path.join('logs', f'grid_test_{timestamp}_trajectories.json')
    
    total = len(cases)
    success_count = 0
    fail_cases = []
    total_time = 0.0
    max_time = 0.0
    all_trajectories = []
    
    with open(log_path, 'w', encoding='utf-8') as log_file:
        def log(msg=""):
            print(msg, flush=True)
            log_file.write(msg + '\n')
            log_file.flush()
            
        log("=" * 96)
        log(f"开始执行网格化自动化测试 | 用例总数: {total}")
        log(f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        log(f"轨迹文件: {traj_path}")
        log("=" * 96 + "\n")
        
        header = f"{'Case':<8} | {'Pos(x, y, th)':<20} | {'Success':<7} | {'Time(ms)':<8} | {'Exp':<6} | {'Strategy':<10} | {'Goal':<4} | {'Coll':<4}"
        log(header)
        log("-" * 88)
        
        for idx, (x, y, th, deg) in enumerate(cases, 1):
            st = {}
            
            def _plan():
                return main.plan_path_robust(x, y, th, prims, use_rs=True, stats=st)
            
            (ok, acts, rs_traj), _ = capture_stderr(_plan)
            
            elapsed = st.get('elapsed_ms', 0)
            expanded = st.get('expanded', 0)
            strategy = '2-Stage' if st.get('two_stage') else 'Direct'
            if st.get('out_of_range'):
                strategy = 'OOR'
            
            total_time += elapsed
            if elapsed > max_time:
                max_time = elapsed
                
            case_id = f"#{idx:03d}"
            pos_str = f"({x:.1f}, {y:4.1f}, {deg:4d}°)"
            
            traj_record = {
                "case_id": idx,
                "start": {"x": x, "y": y, "theta_rad": round(th, 6), "theta_deg": deg},
                "success": ok,
                "elapsed_ms": round(elapsed, 1),
                "expanded": expanded,
                "strategy": strategy,
            }
            
            if ok:
                success_count += 1
                cx, cy, cth = _reconstruct_endpoint(x, y, th, acts, rs_traj, prims, st)
                goal_ok = satisfies_goal(cx, cy, cth)
                ws_ok, viol = _check_path_in_workspace(x, y, th, acts, rs_traj, prims, False, st)
                
                gm = 'OK' if goal_ok else 'FAIL'
                wm = 'OK' if ws_ok else 'FAIL'
                
                log(f"{case_id:<8} | {pos_str:<20} | {'YES':<7} | {elapsed:<8.1f} | {expanded:<6} | {strategy:<10} | {gm:<4} | {wm:<4}")
                
                # 动作序列写入日志
                log(f"         actions({len(acts)}): {_format_actions(acts)}")
                
                # 用 simulate_path 回放完整轨迹
                trajectory = main.simulate_path(x, y, th, acts, prims)
                if rs_traj:
                    trajectory.extend(rs_traj[1:])
                
                traj_record["goal_ok"] = goal_ok
                traj_record["collision_ok"] = ws_ok
                traj_record["endpoint"] = {"x": round(cx, 4), "y": round(cy, 4), "theta_rad": round(cth, 6)}
                traj_record["actions"] = [{"gear": g, "steer": s, "duration": d} for g, s, d in acts]
                traj_record["trajectory"] = [{"x": round(pt[0], 4), "y": round(pt[1], 4), "th": round(pt[2], 6)} for pt in trajectory]
                
                if not goal_ok or not ws_ok:
                    reason = "Goal missed" if not goal_ok else f"Collision: {viol}"
                    fail_cases.append((idx, x, y, deg, reason))
            else:
                reason = "Out of range" if st.get('out_of_range') else "No path found"
                log(f"{case_id:<8} | {pos_str:<20} | {'NO':<7} | {elapsed:<8.1f} | {expanded:<6} | {strategy:<10} | {'---':<4} | {'---':<4}")
                traj_record["fail_reason"] = reason
                fail_cases.append((idx, x, y, deg, reason))
            
            all_trajectories.append(traj_record)
                
        # 全局汇总
        log("\n" + "=" * 96)
        log("测试结果全局汇总")
        log("=" * 96)
        log(f"测试用例总数: {total}")
        log(f"成功用例数量: {success_count}")
        log(f"失败用例数量: {total - success_count}")
        log(f"整体成功率  : {(success_count / total * 100):.1f}%")
        log(f"平均规划耗时: {(total_time / total):.1f} ms")
        log(f"最大规划耗时: {max_time:.1f} ms")
        
        if fail_cases:
            log("\n失败用例明细:")
            for idx, x, y, deg, reason in fail_cases:
                log(f"  - Case #{idx:03d} | 位置: ({x:.1f}, {y:4.1f}, {deg:4d}°) | 原因: {reason}")
        else:
            log("\n所有用例均测试通过！")
        
        log(f"\n轨迹详情已保存至: {traj_path}")
    
    with open(traj_path, 'w', encoding='utf-8') as f:
        json.dump(all_trajectories, f, ensure_ascii=False, indent=2)
            
if __name__ == '__main__':
    run_grid_tests()
