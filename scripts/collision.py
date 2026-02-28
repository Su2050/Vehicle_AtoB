import math
from primitives import (MAX_PLANNABLE_Y, VEHICLE_HALF_WIDTH,
                        VEHICLE_CHECK_OFFSETS, VEHICLE_MAX_RADIUS)

_HALF_W_SQ = VEHICLE_HALF_WIDTH * VEHICLE_HALF_WIDTH
_MAX_R_SQ = VEHICLE_MAX_RADIUS * VEHICLE_MAX_RADIUS


def check_collision(nx, ny, nth, sin_nth=None, cos_nth=None,
                    no_corridor=False, obstacles=None):
    """
    检查状态是否合法（多圆碰撞模型）
    返回: (is_valid, reason)
    reason: 'OK', 'CORRIDOR', 'OBSTACLE'

    沿车身中心线放置 6 个半径为 VEHICLE_HALF_WIDTH(0.25m) 的碰撞圆，
    对每个障碍物先做快速排斥（参考点距离 > VEHICLE_MAX_RADIUS），
    通过后再逐圆检测距离。

    no_corridor=True 时跳过安全走廊约束。
    obstacles: 可选障碍物列表，预处理过则为 [(min_x, max_x, min_y, max_y), ...]
    """
    if abs(ny) > MAX_PLANNABLE_Y:
        return False, 'OBSTACLE'
    if nx > 11.0 or nx < 0.0:
        return False, 'OBSTACLE'

    if obstacles:
        if sin_nth is None:
            sin_nth = math.sin(nth)
        if cos_nth is None:
            cos_nth = math.cos(nth)

        for obs in obstacles:
            if isinstance(obs, tuple):
                min_x, max_x, min_y, max_y = obs
            else:
                ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
                min_x, max_x = min(ox, ox + ow), max(ox, ox + ow)
                min_y, max_y = min(oy, oy + oh), max(oy, oy + oh)

            # 快速排斥：参考点到障碍物边界距离 > 最大包络半径则跳过
            dx0 = nx - max_x if nx > max_x else (min_x - nx if nx < min_x else 0.0)
            dy0 = ny - max_y if ny > max_y else (min_y - ny if ny < min_y else 0.0)
            if dx0 * dx0 + dy0 * dy0 >= _MAX_R_SQ:
                continue

            # 多圆碰撞检测：6 个圆沿车身中心线
            for offset in VEHICLE_CHECK_OFFSETS:
                # Forward is -x in this coordinate system, so subtract offset
                px = nx - offset * cos_nth
                py = ny - offset * sin_nth
                dx = px - max_x if px > max_x else (min_x - px if px < min_x else 0.0)
                dy = py - max_y if py > max_y else (min_y - py if py < min_y else 0.0)
                if dx * dx + dy * dy < _HALF_W_SQ:
                    return False, 'OBSTACLE'

    # 安全走廊门控 (safe_corridor) —— 可选关闭
    if not no_corridor:
        if nx <= 2.05:
            if sin_nth is None:
                sin_nth = math.sin(nth)
            # Check the front of the vehicle (offset = 0.5)
            tip_lat = ny - 0.5 * sin_nth
            sc = (0.15 + (nx - 1.87) * 0.8) if nx > 1.87 else 0.15
            if tip_lat > sc or tip_lat < -sc:
                return False, 'CORRIDOR'

    return True, 'OK'


def _check_traj_collision(traj, no_corridor=False, obstacles=None):
    """对一组轨迹点 [(x,y,th),...] 逐点做碰撞检测，全通过返回 True"""
    for pt in traj:
        x, y, th = pt[0], pt[1], pt[2]
        is_valid, _ = check_collision(x, y, th, no_corridor=no_corridor, obstacles=obstacles)
        if not is_valid:
            return False
    return True
