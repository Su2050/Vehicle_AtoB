"""
B 样条轨迹平滑模块

将 A* 搜索产生的离散折线轨迹，用 B 样条曲线拟合为曲率连续的密集轨迹，
消除离散转向比跳变带来的曲率不连续问题，使输出轨迹能直接喂给
Isaac Lab 等物理引擎的 Pure Pursuit / MPC 控制器。

使用方式：
    from smooth import smooth_path
    dense_traj = smooth_path(traj_points)   # [(x,y,th), ...]
"""

import math
import numpy as np
from scipy.interpolate import splprep, splev


def smooth_path(traj_points, smoothing=0.02, density_factor=3):
    """
    对轨迹点列表做 B 样条平滑。

    参数:
        traj_points    : [(x, y, th), ...] 原始离散轨迹点
        smoothing      : 样条拟合松弛度
                         0   = 精确插值（过所有点）
                         >0  = 允许偏离，值越大曲线越平滑（推荐 0.01~0.05）
        density_factor : 输出点数 = len(traj_points) * density_factor

    返回:
        [(x, y, th), ...] 平滑后的密集轨迹点
        若输入点数不足（<4），原样返回。
    """
    if len(traj_points) < 4:
        return list(traj_points)

    xs = np.array([p[0] for p in traj_points])
    ys = np.array([p[1] for p in traj_points])

    try:
        tck, _ = splprep([xs, ys], s=smoothing, k=3)
    except Exception:
        # 退化情况（所有点共线等）返回原始轨迹
        return list(traj_points)

    n = max(len(traj_points) * density_factor, 50)
    u_fine = np.linspace(0, 1, n)
    xs_s, ys_s = splev(u_fine, tck)

    # 朝向角采用参数插值（而非运动方向差分），避免 forward=-x 约定导致的 180° 偏移。
    # 差分方式 atan2(dy,dx) 给出的是"运动方向"而非"车头朝向"，
    # 在前进方向=-x 的系统中会整体偏 π，且无法正确处理倒退段。
    original_ths = np.array([p[2] for p in traj_points])
    # 原始轨迹点对应的归一化参数（等距）
    u_orig = np.linspace(0, 1, len(traj_points))
    # unwrap 以消除 ±π 跳变对线性插值的影响
    original_ths_unwrapped = np.unwrap(original_ths)
    ths_interp = np.interp(u_fine, u_orig, original_ths_unwrapped)
    # 重新包装到 (-π, π]
    ths = []
    for th in ths_interp:
        while th > math.pi:
            th -= 2 * math.pi
        while th <= -math.pi:
            th += 2 * math.pi
        ths.append(th)

    return list(zip(xs_s.tolist(), ys_s.tolist(), ths))
