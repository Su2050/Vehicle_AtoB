import math
import time
from main import plan_path_robust, plan_path_pure_rs, init_primitives
from viz import _check_traj_collision

# ==========================================
# 调试脚本模板：修改这里的 x, y, th 进行测试
# ==========================================
x, y, th = 4.23, -1.55, 0.0  # 示例坐标

prims = init_primitives()

print("="*40)
print(f"测试起点: x={x:.2f}, y={y:.2f}, th={math.degrees(th):.1f}°")
print("="*40)

# 测试 1: Pure RS 模式
print("\n[模式 1: Pure RS]")
st1 = {}
t0 = time.time()
ok1, acts1, rs_traj1 = plan_path_pure_rs(x, y, th, no_corridor=False, stats=st1)
print(f"成功: {ok1}")
print(f"耗时: {time.time()-t0:.3f}s")
if rs_traj1:
    print(f"轨迹点数: {len(rs_traj1)}")
print(f"统计信息: {st1}")

# 测试 2: RS Heuristic 模式 (带两阶段兜底)
print("\n[模式 2: RS Heuristic (Robust)]")
st2 = {}
t0 = time.time()
ok2, acts2, rs_traj2 = plan_path_robust(x, y, th, prims, use_rs=True, no_corridor=False, stats=st2)
print(f"成功: {ok2}")
print(f"耗时: {time.time()-t0:.3f}s")
if acts2:
    print(f"动作步数: {len(acts2)}")
print(f"统计信息: {st2}")
