import math
import heapq
import rs
from primitives import RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH, MIN_TURN_RADIUS, ALIGN_GOAL_DYAW


class DijkstraGrid:
    def __init__(self, goal_x, goal_y, grid_res=0.15, inflate_radius=0.7):
        self.res = grid_res
        self.inflate_radius = inflate_radius
        self.goal_x = goal_x
        self.goal_y = goal_y
        
        # Grid boundaries based on workspace limits
        self.min_x, self.max_x = -1.0, 10.0
        self.min_y, self.max_y = -6.0, 6.0
        
        self.nx = int((self.max_x - self.min_x) / self.res)
        self.ny = int((self.max_y - self.min_y) / self.res)
        
        self.dist_map = None
        self.obs_map = None
        self.angle_map = None

    def _world_to_grid(self, wx, wy):
        gx = int((wx - self.min_x) / self.res)
        gy = int((wy - self.min_y) / self.res)
        return gx, gy

    def _grid_to_world(self, gx, gy):
        wx = self.min_x + gx * self.res
        wy = self.min_y + gy * self.res
        return wx, wy

    def build_map(self, obstacles, start_x=None, start_y=None):
        # Initialize grid
        self.obs_map = [[False] * self.ny for _ in range(self.nx)]
        self.dist_map = [[float('inf')] * self.ny for _ in range(self.nx)]
        self.cost_map = [[1.0] * self.ny for _ in range(self.nx)]
        
        inf_cells = int(math.ceil(0.7 / self.res))
        soft_inf_cells = int(math.ceil(1.2 / self.res))
        
        if obstacles:
            for obs in obstacles:
                if isinstance(obs, tuple):
                    min_wx, max_wx, min_wy, max_wy = obs
                else:
                    ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
                    min_wx = min(ox, ox + ow)
                    max_wx = max(ox, ox + ow)
                    min_wy = min(oy, oy + oh)
                    max_wy = max(oy, oy + oh)
                
                gx_min, gy_min = self._world_to_grid(min_wx, min_wy)
                gx_max, gy_max = self._world_to_grid(max_wx, max_wy)
                
                # Apply hard inflation
                hx_min = max(0, gx_min - inf_cells)
                hy_min = max(0, gy_min - inf_cells)
                hx_max = min(self.nx - 1, gx_max + inf_cells)
                hy_max = min(self.ny - 1, gy_max + inf_cells)
                
                for x in range(hx_min, hx_max + 1):
                    for y in range(hy_min, hy_max + 1):
                        if 0 <= x < self.nx and 0 <= y < self.ny:
                            self.obs_map[x][y] = True

                # Apply soft inflation (cost map)
                sx_min = max(0, gx_min - soft_inf_cells)
                sy_min = max(0, gy_min - soft_inf_cells)
                sx_max = min(self.nx - 1, gx_max + soft_inf_cells)
                sy_max = min(self.ny - 1, gy_max + soft_inf_cells)
                for x in range(sx_min, sx_max + 1):
                    for y in range(sy_min, sy_max + 1):
                        if 0 <= x < self.nx and 0 <= y < self.ny and not self.obs_map[x][y]:
                            wx, wy = self._grid_to_world(x, y)
                            dx = max(min_wx - wx, 0.0, wx - max_wx)
                            dy = max(min_wy - wy, 0, wy - max_wy)
                            dist = math.sqrt(dx*dx + dy*dy)
                            if dist < 1.0:
                                self.cost_map[x][y] += (1.0 - dist) * 2.0
        
        # Mark hard walls (x < 1.9)
        wall_gx_max, _ = self._world_to_grid(1.9, 0)
        for x in range(0, min(self.nx, wall_gx_max)):
            for y in range(self.ny):
                self.obs_map[x][y] = True
                
        # Clear obstacles near the start position
        if start_x is not None and start_y is not None:
            gx_start, gy_start = self._world_to_grid(start_x, start_y)
            clear_radius = int(math.ceil(0.6 / self.res))
            for x in range(max(0, gx_start - clear_radius), min(self.nx, gx_start + clear_radius + 1)):
                for y in range(max(0, gy_start - clear_radius), min(self.ny, gy_start + clear_radius + 1)):
                    if self.obs_map[x][y]:
                        self.obs_map[x][y] = False
                        self.cost_map[x][y] += 20.0

        # Dijkstra starting from goal
        gx_goal, gy_goal = self._world_to_grid(self.goal_x, self.goal_y)
        
        # Clear obstacles near the goal position
        goal_clear_radius = int(math.ceil(0.5 / self.res))
        for x in range(max(0, gx_goal - goal_clear_radius), min(self.nx, gx_goal + goal_clear_radius + 1)):
            for y in range(max(0, gy_goal - goal_clear_radius), min(self.ny, gy_goal + goal_clear_radius + 1)):
                if 0 <= x < self.nx and 0 <= y < self.ny:
                    if x >= wall_gx_max:
                        if self.obs_map[x][y]:
                            self.obs_map[x][y] = False
                            self.cost_map[x][y] += 20.0
        
        q = [(0.0, gx_goal, gy_goal)]
        if 0 <= gx_goal < self.nx and 0 <= gy_goal < self.ny:
            self.dist_map[gx_goal][gy_goal] = 0.0
            
        # 8-connected neighbors
        dirs = [
            (1, 0, 1.0), (0, 1, 1.0), (-1, 0, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (-1, -1, 1.414)
        ]
        
        while q:
            d, gx, gy = heapq.heappop(q)
            
            if d > self.dist_map[gx][gy]:
                continue
                
            for dx, dy, move_cost in dirs:
                nx, ny = gx + dx, gy + dy
                
                if 0 <= nx < self.nx and 0 <= ny < self.ny:
                    if not self.obs_map[nx][ny]:
                        cell_cost = move_cost * self.res * self.cost_map[nx][ny]
                        new_d = d + cell_cost
                        if new_d < self.dist_map[nx][ny]:
                            self.dist_map[nx][ny] = new_d
                            heapq.heappush(q, (new_d, nx, ny))
                            
        # Precompute gradient angle map
        self.angle_map = [[None for _ in range(self.ny)] for _ in range(self.nx)]
        for gx in range(self.nx):
            for gy in range(self.ny):
                dist = self.dist_map[gx][gy]
                if dist != float('inf') and dist > 0.1 and not self.obs_map[gx][gy]:
                    min_d = dist
                    best_dx, best_dy = 0, 0
                    for dx, dy, _cost in dirs:
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < self.nx and 0 <= ny < self.ny:
                            d = self.dist_map[nx][ny]
                            if d < min_d:
                                min_d = d
                                best_dx, best_dy = dx, dy
                    if best_dx != 0 or best_dy != 0:
                        self.angle_map[gx][gy] = math.atan2(best_dy * self.res, best_dx * self.res)

    def line_of_sight(self, wx1, wy1, wx2, wy2):
        x0, y0 = self._world_to_grid(wx1, wy1)
        x1, y1 = self._world_to_grid(wx2, wy2)
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            if x0 == x1 and y0 == y1: return True
            if 0 <= x0 < self.nx and 0 <= y0 < self.ny:
                if self.obs_map[x0][y0]: return False
            else:
                return False
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
        return True

    def get_heuristic(self, wx, wy, wth=None):
        gx, gy = self._world_to_grid(wx, wy)
        if 0 <= gx < self.nx and 0 <= gy < self.ny:
            if self.obs_map[gx][gy]:
                return 1000.0, 1000.0
            dist = self.dist_map[gx][gy]
            pure_dist = dist
            if dist == float('inf'):
                euc = math.hypot(wx - self.goal_x, wy - self.goal_y)
                big = max(euc * 3.0, 50.0)
                return big, big
            if dist != float('inf'):
                if wth is not None and dist < 5.0:
                    goal_th = 0.0 
                    diff_goal = abs(wth - goal_th)
                    while diff_goal > math.pi: diff_goal -= 2 * math.pi
                    diff_goal = abs(diff_goal)
                    align_goal_diff = min(diff_goal, math.pi - diff_goal)
                    
                    weight = (5.0 - dist) * 2.0
                    dist += align_goal_diff * weight

                return dist, pure_dist
        
        euc = math.hypot(wx - self.goal_x, wy - self.goal_y)
        big = max(euc * 5.0, 100.0)
        return big, big


# ── 4 种独立启发式函数 ──────────────────────────────────────────────

def geometric_heuristic(nx, ny, nth):
    """几何启发式：无 RS 模式，各系数经实测校准使 h ≈ 实际代价"""
    dx_h = nx - 2.25 if nx > 2.25 else 0.0
    abs_ny = ny if ny > 0 else -ny
    dy_h = abs_ny - 0.18 if abs_ny > 0.18 else 0.0
    abs_nth = nth if nth > 0 else -nth
    dth_h = abs_nth - ALIGN_GOAL_DYAW if abs_nth > ALIGN_GOAL_DYAW else 0.0

    room_needed = 2.10 + dy_h * 3.0 + dth_h * 2.0
    
    back_up_penalty = 0.0
    if nx < room_needed and (dy_h > 0.05 or dth_h > 0.05):
        back_up_penalty = (room_needed - nx) * 8.0

    h = dx_h * 4.0 + dy_h * 16.0 + dth_h * 6.0 + back_up_penalty
    h_weight = 1.0
    return h, h_weight


def rs_heuristic(nx, ny, nth):
    """纯 RS 启发式：无障碍模式，直接用 RS 距离"""
    rs_dist = rs.rs_distance_pose(
        nx, ny, nth,
        RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
        MIN_TURN_RADIUS
    )
    h_rs = rs_dist / 0.25
    return h_rs, 1.0


def rs_grid_heuristic(nx, ny, nth, dijkstra_grid):
    """RS + Dijkstra 启发式：有障碍模式"""
    h_grid_dist, _ = dijkstra_grid.get_heuristic(nx, ny, nth)
    h_grid = h_grid_dist / 0.25
    
    rs_dist = rs.rs_distance_pose(
        nx, ny, nth,
        RS_GOAL_X, RS_GOAL_Y, RS_GOAL_TH,
        MIN_TURN_RADIUS
    )
    h_rs = rs_dist / 0.25
    
    if dijkstra_grid.line_of_sight(nx, ny, RS_GOAL_X, RS_GOAL_Y):
        h = max(h_rs, h_grid * 1.2)
    else:
        h = max(h_rs, h_grid * 1.5)
        
    # 补充非完整约束揉库预估惩罚
    dy_err = abs(ny - RS_GOAL_Y)
    dx_err = nx - max(RS_GOAL_X, 2.0)
    th_err = abs(nth - RS_GOAL_TH)
    if (dy_err > 0.5 or th_err > 0.2) and dx_err > 0.0:
        needed_x = dy_err * 2.0 + th_err * 1.5
        if dx_err < needed_x:
            kturn_est = min(50.0, (needed_x - dx_err) * 10.0)
            h += kturn_est
    
    h_weight = 2.5
    return h, h_weight


def preapproach_heuristic(nx, ny, nth, goal_xmin, goal_xmax, goal_ymin, goal_ymax, goal_thmax, dijkstra_grid=None):
    """Stage-1 专用启发式：同时惩罚横向偏移、x 越界、角度偏离"""
    abs_nth = nth if nth >= 0 else -nth
    
    if goal_ymin <= ny <= goal_ymax:
        h_y = 0.0
    else:
        h_y = min(abs(ny - goal_ymin), abs(ny - goal_ymax)) * 5.0
        if abs_nth > goal_thmax + 0.2:
            h_y *= 0.2
    
    dy_err = h_y / 5.0
    dx_err = nx - goal_xmin
    kturn_est = 0.0
    needed_x = 0.0
    if (dy_err > 0.5 or abs_nth > goal_thmax + 0.2) and dx_err > 0.0:
        needed_x = dy_err * 2.0 + abs_nth * 1.5
        if dx_err < needed_x:
            kturn_est = (needed_x - dx_err) * 10.0
            
    dyn_xmax = max(goal_xmax if abs_nth <= goal_thmax + 0.2 else max(goal_xmax, 4.0), goal_xmin + needed_x)
    h_x = max(0.0, goal_xmin - nx) * 3.0
    if nx > dyn_xmax:
        h_x += (nx - dyn_xmax) * 1.5
        
    h_th = max(0.0, abs_nth - goal_thmax) * 3.0
    h = h_y + h_x + h_th + kturn_est
    
    if dijkstra_grid is not None:
        h_grid_dist, _ = dijkstra_grid.get_heuristic(nx, ny, nth)
        h_grid = h_grid_dist / 0.25
        h = max(h, h_grid * 1.5)
    h_weight = 2.5
    return h, h_weight
