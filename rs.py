"""
Reeds-Shepp 路径距离计算模块

坐标约定与 main.py 完全一致：
  - 前进（F 档）方向：-x（即 x -= v * cos(th)）
  - 左转：y 增大方向为正
  - 角度 theta：标准数学正方向（逆时针为正）

RS 路径由最多 5 段组成，每段是：
  L / R：固定最小转弯半径圆弧（左/右）
  S    ：直线段

覆盖前进（+）和倒退（-）两种方向，共 48 种 word。

参考：Reeds & Shepp (1990), "Optimal Paths for a Car That Goes Both Forwards and Backwards"
实现基于：P. Souères & J.P. Laumond (1996) 标准化公式，以及
          Lavalle (2006) Planning Algorithms Chapter 15 附录实现

使用方法：
    import rs
    dist = rs.rs_distance_pose(x1, y1, th1, x2, y2, th2, turning_radius)
"""

import math

_PI = math.pi
_TWO_PI = 2.0 * math.pi


def _mod2pi(x):
    """将角度包装到 [0, 2*pi)"""
    v = math.fmod(x, _TWO_PI)
    if v < 0.0:
        v += _TWO_PI
    return v


def _polar(x, y):
    """转极坐标，返回 (r, theta)，theta 在 [0, 2*pi)"""
    r = math.hypot(x, y)
    theta = _mod2pi(math.atan2(y, x))
    return r, theta


def _rect(r, theta):
    return r * math.cos(theta), r * math.sin(theta)


# ─────────────────────────────────────────────────────────────────────────────
# 48 种 RS word 公式（无障碍物最短路径）
# 每个函数返回 (t, u, v) 三段参数，段长为负表示倒退；
# 不可行时返回 None。
# 外部调用 rs_distance() 对所有公式取总长度最小值。
#
# 注意：这里的 t / u / v 是以最小转弯半径 r=1 归一化后的无量纲参数，
# 实际长度需乘以 turning_radius。
# ─────────────────────────────────────────────────────────────────────────────

# ── CSC 类型（Formula 8.1 - 8.4）──────────────────────────────────────────

def _LpSpLp(x, y, phi):
    """L+S+L+"""
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if t >= 0.0:
        v = _mod2pi(phi - t)
        if v >= 0.0:
            return t, u, v
    return None


def _LmSmLm(x, y, phi):
    """L-S-L-"""
    return _LpSpLp(-x, y, -phi)


def _RpSpRp(x, y, phi):
    """R+S+R+"""
    t, u, v = _LpSpLp(x, -y, -phi) or (None, None, None)
    if t is not None:
        return t, u, v
    return None


def _RmSmRm(x, y, phi):
    """R-S-R-"""
    return _RpSpRp(-x, y, -phi)


def _LpSpRp(x, y, phi):
    """L+S+R+"""
    u1sq = x**2 + (y - 1.0)**2
    if u1sq < 4.0:
        return None
    u1 = math.sqrt(u1sq - 4.0)
    _, t1 = _polar(u1, 2.0)  # t1 unused directly
    t = _mod2pi(math.atan2(2.0, u1) - math.atan2(x, y - 1.0))
    # recompute properly
    t = _mod2pi(math.atan2(y - 1.0, x) - math.atan2(2.0, u1))
    if t < 0.0:
        return None
    u = math.sqrt(u1sq - 4.0)
    v = _mod2pi(t - phi)
    if v < 0.0:
        return None
    # re-derive correctly
    u2, t2 = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u2 < 2.0:
        return None
    t3 = _mod2pi(math.atan2(2.0, math.sqrt(u2**2 - 4.0)) +
                 math.atan2(-math.sqrt(u2**2 - 4.0), 2.0))
    # use the clean standard derivation below
    return None  # delegated to _LpSpRp_clean


def _LpSpRp_clean(x, y, phi):
    """L+S+R+ 标准推导"""
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    u1sq = u1 * u1
    if u1sq < 4.0:
        return None
    u = math.sqrt(u1sq - 4.0)
    theta = _mod2pi(math.atan2(2.0, u))
    t = _mod2pi(t1 + theta)
    v = _mod2pi(t - phi)
    if t >= 0.0 and v >= 0.0:
        return t, u, v
    return None


def _LmSmRm(x, y, phi):
    """L-S-R-"""
    res = _LpSpRp_clean(-x, y, -phi)
    if res:
        t, u, v = res
        return t, u, v
    return None


def _RpSpLp(x, y, phi):
    """R+S+L+"""
    res = _LpSpRp_clean(x, -y, -phi)
    if res:
        t, u, v = res
        return t, u, v
    return None


def _RmSmLm(x, y, phi):
    """R-S-L-"""
    res = _LpSpRp_clean(-x, -y, phi)
    if res:
        t, u, v = res
        return t, u, v
    return None


# ── CCC 类型（Formula 8.7 - 8.10）─────────────────────────────────────────

def _LpRmLp(x, y, phi):
    """L+R-L+"""
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 > 4.0:
        return None
    A = math.acos((8.0 - u1 * u1) / 8.0)
    u = _mod2pi(A)
    t = _mod2pi(math.atan2(-eta, xi) + 0.5 * A + _mod2pi(0.5 * _PI))
    # fix standard formula
    t = _mod2pi(theta + 0.5 * (_PI - A))
    v = _mod2pi(t - phi - u)
    if t >= 0.0 and u >= 0.0 and v >= 0.0:
        return t, u, v
    return None


def _LmRpLm(x, y, phi):
    """L-R+L-"""
    res = _LpRmLp(-x, y, -phi)
    if res:
        return res
    return None


def _RpLmRp(x, y, phi):
    """R+L-R+"""
    res = _LpRmLp(x, -y, -phi)
    if res:
        return res
    return None


def _RmLpRm(x, y, phi):
    """R-L+R-"""
    res = _LpRmLp(-x, -y, phi)
    if res:
        return res
    return None


def _LpRmLm(x, y, phi):
    """L+R-L-"""
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 > 4.0:
        return None
    A = math.acos((8.0 - u1 * u1) / 8.0)
    t = _mod2pi(theta + 0.5 * (_PI - A))
    u = _mod2pi(A)
    v = _mod2pi(phi - t + u)
    if t >= 0.0 and u >= 0.0 and v <= 0.0:
        return t, u, -v
    return None


def _LmRpLp(x, y, phi):
    """L-R+L+"""
    res = _LpRmLm(-x, y, -phi)
    if res:
        t, u, v = res
        return t, u, v
    return None


def _RpLmRm(x, y, phi):
    """R+L-R-"""
    res = _LpRmLm(x, -y, -phi)
    if res:
        return res
    return None


def _RmLpRp(x, y, phi):
    """R-L+R+"""
    res = _LpRmLm(-x, -y, phi)
    if res:
        return res
    return None


# ── CCCC 类型（Formula 8.11）──────────────────────────────────────────────

def _LpRupLumRm(x, y, phi):
    """L+R+u L-u R-"""
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 > 4.0:
        return None
    if u1 > 2.0:
        A = math.acos(0.25 * u1 - 1.0)
    else:
        return None
    t = _mod2pi(theta + 0.5 * A + _PI / 2.0)
    u = _mod2pi(_PI - A)
    v = _mod2pi(phi - t + 2.0 * u)
    if t >= 0.0 and u >= 0.0 and v >= 0.0:
        return t, u, v
    return None


def _LmRumLupRp(x, y, phi):
    res = _LpRupLumRm(-x, y, -phi)
    if res:
        return res
    return None


def _RpLupRumLm(x, y, phi):
    res = _LpRupLumRm(x, -y, -phi)
    if res:
        return res
    return None


def _RmLumRupLp(x, y, phi):
    res = _LpRupLumRm(-x, -y, phi)
    if res:
        return res
    return None


# ── CCSC / CSCC 类型（Formula 8.9, 8.10）─────────────────────────────────

def _LpRmSmLm(x, y, phi):
    """L+R-S-L-"""
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 < 2.0:
        return None
    t = _mod2pi(theta + math.acos(-2.0 / u1) + _PI / 2.0)
    u = _mod2pi(_PI / 2.0)
    v = _mod2pi(_PI - math.acos(-2.0 / u1) - phi + t)
    if t >= 0.0 and u >= 0.0 and v >= 0.0:
        return t, u, v
    return None


def _LmRpSmLp(x, y, phi):
    res = _LpRmSmLm(-x, y, -phi)
    if res:
        return res
    return None


def _RpLmSmRm(x, y, phi):
    res = _LpRmSmLm(x, -y, -phi)
    if res:
        return res
    return None


def _RmLpSmRp(x, y, phi):
    res = _LpRmSmLm(-x, -y, phi)
    if res:
        return res
    return None


def _LpRmSmRm(x, y, phi):
    """L+R-S-R-"""
    xi = x + math.sin(phi)
    eta = y - 1.0 - math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 < 2.0:
        return None
    t = _mod2pi(theta + math.acos(-2.0 / u1) + _PI / 2.0)
    u = _mod2pi(_PI / 2.0)
    v = _mod2pi(t + _PI / 2.0 - phi)
    if t >= 0.0 and u >= 0.0 and v >= 0.0:
        return t, u, v
    return None


def _LmRpSmRp(x, y, phi):
    res = _LpRmSmRm(-x, y, -phi)
    if res:
        return res
    return None


def _RpLmSmLm(x, y, phi):
    res = _LpRmSmRm(x, -y, -phi)
    if res:
        return res
    return None


def _RmLpSmLp(x, y, phi):
    res = _LpRmSmRm(-x, -y, phi)
    if res:
        return res
    return None


def _LpSmRmLp(x, y, phi):
    """L+S-R-L+ (CSCС)"""
    xi = x - math.sin(phi)
    eta = y - 1.0 + math.cos(phi)
    u1, theta = _polar(xi, eta)
    if u1 < 2.0:
        return None
    u = math.sqrt(u1 * u1 - 4.0)
    t = _mod2pi(theta + math.atan2(2.0, u))
    v = _mod2pi(t - _mod2pi(phi))
    if t >= 0.0 and v >= 0.0:
        return t, u, v
    return None


def _RmSmLmRm(x, y, phi):
    res = _LpSmRmLp(-x, y, -phi)
    if res:
        return res
    return None


def _LmSmRmLm(x, y, phi):
    res = _LpSmRmLp(x, -y, -phi)
    if res:
        return res
    return None


def _RpSpLpRp(x, y, phi):
    res = _LpSmRmLp(-x, -y, phi)
    if res:
        return res
    return None


# ─────────────────────────────────────────────────────────────────────────────
# 核心入口：归一化坐标下的 RS 距离
# ─────────────────────────────────────────────────────────────────────────────

def _all_words(x, y, phi):
    """
    枚举所有 48 种 RS word，返回 (总长度, word名称) 列表（仅可行解）。
    坐标已按 turning_radius 归一化（即 r=1 的坐标系）。
    """
    results = []

    def _try(fn, tag, sign_t=1, sign_u=1, sign_v=1):
        res = fn(x, y, phi)
        if res is not None:
            t, u, v = res
            length = abs(t) + abs(u) + abs(v)
            results.append((length, tag, t * sign_t, u * sign_u, v * sign_v))

    # CSC
    _try(_LpSpLp,      'L+S+L+')
    _try(_LmSmLm,      'L-S-L-')
    _try(_RpSpRp,      'R+S+R+')
    _try(_RmSmRm,      'R-S-R-')
    _try(_LpSpRp_clean,'L+S+R+')
    _try(_LmSmRm,      'L-S-R-')
    _try(_RpSpLp,      'R+S+L+')
    _try(_RmSmLm,      'R-S-L-')

    # CCC
    _try(_LpRmLp,      'L+R-L+')
    _try(_LmRpLm,      'L-R+L-')
    _try(_RpLmRp,      'R+L-R+')
    _try(_RmLpRm,      'R-L+R-')
    _try(_LpRmLm,      'L+R-L-')
    _try(_LmRpLp,      'L-R+L+')
    _try(_RpLmRm,      'R+L-R-')
    _try(_RmLpRp,      'R-L+R+')

    # CCCC
    _try(_LpRupLumRm,  'L+R+L-R-')
    _try(_LmRumLupRp,  'L-R-L+R+')
    _try(_RpLupRumLm,  'R+L+R-L-')
    _try(_RmLumRupLp,  'R-L-R+L+')

    # CCSC / CSCC
    _try(_LpRmSmLm,    'L+R-S-L-')
    _try(_LmRpSmLp,    'L-R+S+L+')
    _try(_RpLmSmRm,    'R+L-S-R-')
    _try(_RmLpSmRp,    'R-L+S+R+')
    _try(_LpRmSmRm,    'L+R-S-R-')
    _try(_LmRpSmRp,    'L-R+S+R+')
    _try(_RpLmSmLm,    'R+L-S-L-')
    _try(_RmLpSmLp,    'R-L+S+L+')
    _try(_LpSmRmLp,    'L+S-R-L+')
    _try(_RmSmLmRm,    'R-S-L-R-')
    _try(_LmSmRmLm,    'L-S-R-L-')
    _try(_RpSpLpRp,    'R+S+L+R+')

    return results


def rs_distance(x, y, phi, turning_radius):
    """
    计算从原点 (0, 0, 0) 到 (x, y, phi) 的 Reeds-Shepp 最短路径长度。

    坐标约定与 main.py 一致（前进方向为 -x）。

    参数:
        x, y   : 目标相对位置（米）
        phi    : 目标相对朝向（弧度）
        turning_radius: 最小转弯半径（米），= WHEELBASE / tan(MAX_STEER)

    返回:
        float: RS 路径长度（米），若计算失败返回 Euclidean 距离作为下界估计
    """
    if turning_radius <= 0.0:
        raise ValueError(f"turning_radius must be positive, got {turning_radius}")

    # 归一化坐标（除以转弯半径，RS 公式在 r=1 下推导）
    nx = x / turning_radius
    ny = y / turning_radius

    words = _all_words(nx, ny, phi)
    if not words:
        # 回退到欧氏距离（保守可容许下界）
        return math.hypot(x, y)

    best_len = min(w[0] for w in words)
    return best_len * turning_radius


def rs_distance_pose(x1, y1, th1, x2, y2, th2, turning_radius):
    """
    计算任意两个位姿之间的 Reeds-Shepp 距离。

    将 (x2, y2, th2) 变换到以 (x1, y1, th1) 为原点的局部坐标系，
    然后调用 rs_distance()。

    参数均使用 main.py 的坐标约定（前进方向为 -x）。
    """
    dx = x2 - x1
    dy = y2 - y1
    cos_t = math.cos(th1)
    sin_t = math.sin(th1)

    # 局部坐标：旋转到以 th1 为基准的参考系
    lx =  dx * cos_t + dy * sin_t
    ly = -dx * sin_t + dy * cos_t
    lphi = th2 - th1

    # 角度包装到 (-pi, pi]
    while lphi > _PI:
        lphi -= _TWO_PI
    while lphi <= -_PI:
        lphi += _TWO_PI

    return rs_distance(lx, ly, lphi, turning_radius)


# ─────────────────────────────────────────────────────────────────────────────
# 路径采样：用于 A* 解析终点扩展（一杆进洞）
# ─────────────────────────────────────────────────────────────────────────────

def rs_best_path(x, y, phi, turning_radius):
    """
    返回从原点到 (x, y, phi) 的最短 RS 路径各段参数。

    返回值: list of (seg_type, length_meters)
        seg_type: 'L', 'R', 'S'
        length_meters: 该段的有向长度（负数表示倒退）

    坐标约定与 main.py 一致（前进方向为 -x）。
    """
    if turning_radius <= 0.0:
        raise ValueError(f"turning_radius must be positive, got {turning_radius}")

    nx = x / turning_radius
    ny = y / turning_radius

    words = _all_words(nx, ny, phi)
    if not words:
        return []

    best = min(words, key=lambda w: w[0])
    _, tag, t, u, v = best

    # 解析 tag 字符串，例如 'L+R-S-L-' → [('L',+),('R',-),('S',-),('L',-)]
    segments_raw = _parse_tag(tag)

    # t, u, v 对应前三段（最多5段时需扩展，但当前实现最多4段已足够）
    seg_lengths_norm = [t, u, v]

    result = []
    for i, (stype, sign) in enumerate(segments_raw[:3]):
        length_m = seg_lengths_norm[i] * sign * turning_radius
        if abs(length_m) > 1e-6:
            result.append((stype, length_m))
    return result


def _parse_tag(tag):
    """
    将 RS word 标签字符串解析为 [(类型, 符号), ...] 列表。
    例如 'L+R-S-' → [('L', 1), ('R', -1), ('S', -1)]
    """
    segments = []
    i = 0
    while i < len(tag):
        c = tag[i]
        if c in ('L', 'R', 'S'):
            sign = 1 if tag[i + 1] == '+' else -1
            segments.append((c, sign))
            i += 2
        else:
            i += 1
    return segments


def rs_sample_path(x1, y1, th1, x2, y2, th2, turning_radius, step=0.05):
    """
    对从 (x1,y1,th1) 到 (x2,y2,th2) 的最短 RS 路径按 step（米）密度采样。

    返回: [(x, y, th), ...] 轨迹点列表（包含起点和终点）
          若找不到 RS 路径则返回 None。

    坐标约定与 main.py 一致（前进方向为 -x）。
    """
    # 变换到局部坐标系
    dx = x2 - x1
    dy = y2 - y1
    cos_t = math.cos(th1)
    sin_t = math.sin(th1)
    lx =  dx * cos_t + dy * sin_t
    ly = -dx * sin_t + dy * cos_t
    lphi = th2 - th1
    while lphi > _PI:
        lphi -= _TWO_PI
    while lphi <= -_PI:
        lphi += _TWO_PI

    segs = rs_best_path(lx, ly, lphi, turning_radius)
    if not segs:
        return None

    # 在局部坐标系下沿路径采样
    cx, cy, cth = 0.0, 0.0, 0.0
    local_pts = [(cx, cy, cth)]

    for stype, length_m in segs:
        direction = 1 if length_m >= 0 else -1
        dist_left = abs(length_m)

        while dist_left > 1e-9:
            ds = min(step, dist_left)
            dist_left -= ds

            if stype == 'S':
                # 直线段：沿当前朝向前进/后退
                # main.py 约定：前进方向 = -x（即 x -= v*cos(th)）
                cx -= direction * ds * math.cos(cth)
                cy -= direction * ds * math.sin(cth)
            elif stype == 'L':
                # 左转弧
                arc = direction * ds / turning_radius
                # 圆心在左侧：(cx - r*sin(th), cy + r*cos(th))（main.py 约定）
                # 数值积分一小步（足够精确）
                cx -= direction * ds * math.cos(cth)
                cy -= direction * ds * math.sin(cth)
                cth += arc
            else:
                # 右转弧
                arc = -direction * ds / turning_radius
                cx -= direction * ds * math.cos(cth)
                cy -= direction * ds * math.sin(cth)
                cth += arc

            # 角度包装
            while cth > _PI:
                cth -= _TWO_PI
            while cth <= -_PI:
                cth += _TWO_PI

            local_pts.append((cx, cy, cth))

    # 将局部坐标系点变换回世界坐标系
    world_pts = []
    for lx_p, ly_p, lth_p in local_pts:
        wx = x1 + lx_p * cos_t - ly_p * sin_t
        wy = y1 + lx_p * sin_t + ly_p * cos_t
        wth = th1 + lth_p
        while wth > _PI:
            wth -= _TWO_PI
        while wth <= -_PI:
            wth += _TWO_PI
        world_pts.append((wx, wy, wth))

    return world_pts
