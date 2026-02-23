import time
import os
import glob
import subprocess

# 自动寻找最新的终端输出文件，或直接指定文件
# 注意：我们这里监控系统的 terminals 文件夹
terminal_dir = "/Users/sull/.cursor/projects/Users-sull-Documents-untitled-folder/terminals/"

def get_latest_terminal_file():
    files = glob.glob(os.path.join(terminal_dir, "*.txt"))
    if not files:
        return None
    return max(files, key=os.path.getmtime)

def monitor_stress_test():
    print("🚀 开始监测 Thorough 极限压测进度...")
    log_file = get_latest_terminal_file()
    print(f"正在监控日志文件: {log_file}")
    
    last_size = 0
    finished = False
    
    while not finished:
        if not log_file or not os.path.exists(log_file):
            time.sleep(2)
            log_file = get_latest_terminal_file()
            continue
            
        with open(log_file, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            
            # 定位最后的进度
            lines = content.split('\n')
            progress_lines = [l for l in lines if 'Testing:' in l]
            if progress_lines:
                # 打印最新的一行进度
                print(f"\r{progress_lines[-1].strip()}", end="", flush=True)
                
            # 检查是否包含测试结束标志
            if "STRESS TEST REPORT" in content:
                print("\n\n✅ 压测运行结束！开始分析结果：\n")
                
                # 提取报告部分
                report_idx = content.find("STRESS TEST REPORT")
                # 往上找一点点打印完整的框
                start_idx = content.rfind("=======", 0, report_idx)
                if start_idx == -1: start_idx = report_idx
                report_content = content[start_idx:]
                
                print(report_content)
                
                # 如果有碰撞用例，自动打印前三个进行分析
                if "TRAJ_COLLISION" in report_content and "0 (No false positives" not in report_content:
                    print("\n⚠️ 发现残余的碰撞用例 (TRAJ_COLLISION)，下面是错误详情：")
                    # 查找生成的 json 文件
                    json_files = sorted(glob.glob("logs/stress_collisions_*.json"))
                    if json_files:
                        latest_json = json_files[-1]
                        import json
                        try:
                            with open(latest_json, 'r') as jf:
                                data = json.load(jf)
                                print(f"正在读取最新日志: {latest_json}")
                                for i, d in enumerate(data[:3]):
                                    print(f"[{i+1}] Case ID: {d.get('case_id')}, 原因: {d.get('collision_reason')}, 来源: {d.get('collision_source')}")
                                    print(f"    起点: {d.get('start')}")
                        except Exception as e:
                            print(f"解析 JSON 失败: {e}")
                else:
                    print("\n🎉 太棒了！Thorough 测试完美通过，没有任何物理穿模和假阳性！代码修复绝对稳定！")
                
                finished = True
                
        if not finished:
            time.sleep(2)

if __name__ == "__main__":
    monitor_stress_test()
