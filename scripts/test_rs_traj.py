import rs
import math

cx, cy, cth = 2.05, -1.03, -1.32
RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH = 2.25, 0.0, 0.0
MIN_TURN_RADIUS = 1.97

trajs = rs.rs_sample_path_multi(cx, cy, cth, RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS, step=0.05)
for i, traj in enumerate(trajs):
    if not traj: continue
    min_x = min(pt[0] for pt in traj)
    max_y = max(abs(pt[1]) for pt in traj)
    print(f"Traj {i}: min_x={min_x:.2f}, max_y={max_y:.2f}")

_WALL_X = 1.92
_WALL_CORRIDOR_Y = 0.5
_MAX_BEHIND_WALL_PTS = 5

def _path_goes_behind_wall(traj):
    if not traj: return False
    count = 0
    for pt in traj:
        if pt[0] < _WALL_X and abs(pt[1]) > _WALL_CORRIDOR_Y:
            count += 1
    return count > _MAX_BEHIND_WALL_PTS

for i, traj in enumerate(trajs):
    if not traj: continue
    print(f"Traj {i} rejected by wall: {_path_goes_behind_wall(traj)}")

def _path_is_unreasonable(traj, sx, sy, max_ratio=3.0, max_lat_extra=2.0):
    if not traj: return True
    ex, ey = traj[-1][0], traj[-1][1]
    direct_dist = math.hypot(ex - sx, ey - sy)
    if direct_dist < 0.1: return False
    
    path_len = 0.0
    for i in range(1, len(traj)):
        path_len += math.hypot(traj[i][0] - traj[i-1][0], traj[i][1] - traj[i-1][1])
        
    if path_len > direct_dist * max_ratio and path_len > 3.0:
        return True
        
    min_y = min(sy, ey) - max_lat_extra
    max_y = max(sy, ey) + max_lat_extra
    for pt in traj:
        if pt[1] < min_y or pt[1] > max_y:
            return True
            
    return False

for i, traj in enumerate(trajs):
    if not traj: continue
    print(f"Traj {i} unreasonable: {_path_is_unreasonable(traj, cx, cy)}")

from collision import check_collision
from planner_obs import _preprocess_obstacles

obs = [{'x': 3.85, 'y': -1.49, 'w': 0.51, 'h': 0.63}, {'x': 3.1, 'y': 0.22, 'w': 1.6, 'h': 0.61}, {'x': 3.58, 'y': -0.09, 'w': 1.53, 'h': 1.96}, {'x': 5.12, 'y': -1.3, 'w': 0.47, 'h': 0.63}]
fast_obs = _preprocess_obstacles(obs)

def _check_traj_collision(traj, no_corridor=False, obstacles=None):
    for pt in traj:
        x, y, th = pt[0], pt[1], pt[2]
        is_valid, _ = check_collision(x, y, th, no_corridor=no_corridor, obstacles=obstacles)
        if not is_valid:
            return False
    return True

for i, traj in enumerate(trajs):
    if not traj: continue
    print(f"Traj {i} collision: {not _check_traj_collision(traj, False, fast_obs)}")
