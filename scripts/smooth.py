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

    # 从平滑后的 (x,y) 序列重新推算朝向角
    # 使用相邻两点差分近似切线方向
    ths = []
    for i in range(n - 1):
        dx = xs_s[i + 1] - xs_s[i]
        dy = ys_s[i + 1] - ys_s[i]
        ths.append(math.atan2(dy, dx))
    ths.append(traj_points[-1][2])  # 终点朝向沿用原始值

    return list(zip(xs_s.tolist(), ys_s.tolist(), ths))
