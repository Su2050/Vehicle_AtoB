import math
from primitives import MAX_PLANNABLE_Y

VEHICLE_RADIUS = 0.1
_VEHICLE_R_SQ = VEHICLE_RADIUS * VEHICLE_RADIUS


def set_vehicle_radius(r):
    """Runtime-configurable vehicle collision radius (meters)."""
    global VEHICLE_RADIUS, _VEHICLE_R_SQ
    VEHICLE_RADIUS = r
    _VEHICLE_R_SQ = r * r


def check_collision(nx, ny, nth, sin_nth=None, no_corridor=False, obstacles=None):
    """
    检查状态是否合法
    返回: (is_valid, reason)
    reason: 'OK', 'CORRIDOR', 'OBSTACLE'

    no_corridor=True 时跳过安全走廊约束。
    obstacles: 可选障碍物列表，如果是预处理过的则为 [(min_x, max_x, min_y, max_y), ...]，否则为原格式
    """
    if abs(ny) > MAX_PLANNABLE_Y:
        return False, 'OBSTACLE'
    if nx > 9.0 or nx < 1.0:
        return False, 'OBSTACLE'

    if obstacles:
        for obs in obstacles:
            if isinstance(obs, tuple):
                min_x, max_x, min_y, max_y = obs
            else:
                ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
                min_x, max_x = min(ox, ox + ow), max(ox, ox + ow)
                min_y, max_y = min(oy, oy + oh), max(oy, oy + oh)

            dx = nx - max_x if nx > max_x else (min_x - nx if nx < min_x else 0.0)
            dy = ny - max_y if ny > max_y else (min_y - ny if ny < min_y else 0.0)

            if dx * dx + dy * dy < _VEHICLE_R_SQ:
                return False, 'OBSTACLE'

    # 安全走廊门控 (safe_corridor) —— 可选关闭
    if not no_corridor:
        if nx <= 2.05:
            if sin_nth is None:
                sin_nth = math.sin(nth)
            tip_lat = ny - 1.87 * sin_nth
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
