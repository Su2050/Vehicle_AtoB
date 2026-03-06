import re

with open('heuristic.py', 'r') as f:
    lines = f.readlines()

insert_idx = 0
for i, line in enumerate(lines):
    if line.startswith("def geometric_heuristic"):
        insert_idx = i
        break

new_code = """
    def build_3d_map(self):
        # Build a coarse 3D non-holonomic heuristic map
        self.res_3d = 0.5
        self.nth = 8
        self.res_th = 2 * math.pi / self.nth
        self.nx_3d = int((self.max_x - self.min_x) / self.res_3d)
        self.ny_3d = int((self.max_y - self.min_y) / self.res_3d)
        
        self.dist_3d = [[[float('inf')] * self.nth for _ in range(self.ny_3d)] for _ in range(self.nx_3d)]
        
        gx_goal = int((self.goal_x - self.min_x) / self.res_3d)
        gy_goal = int((self.goal_y - self.min_y) / self.res_3d)
        gth_goal = 0
        
        if not (0 <= gx_goal < self.nx_3d and 0 <= gy_goal < self.ny_3d):
            return
            
        import heapq
        q = [(0.0, gx_goal, gy_goal, gth_goal)]
        self.dist_3d[gx_goal][gy_goal][gth_goal] = 0.0
        
        while q:
            d, gx, gy, gth = heapq.heappop(q)
            if d > self.dist_3d[gx][gy][gth]: continue
            
            wx = self.min_x + gx * self.res_3d + self.res_3d/2
            wy = self.min_y + gy * self.res_3d + self.res_3d/2
            wth = gth * self.res_th
            
            for d_step in [1.0, -1.0]:
                for steer in [0.0, 0.4, -0.4]:
                    cost = 1.0 if d_step == -1.0 else 1.5
                    if steer != 0.0: cost += 0.2
                    
                    new_wth = wth - steer * d_step
                    new_wx = wx - d_step * math.cos(new_wth)
                    new_wy = wy - d_step * math.sin(new_wth)
                    
                    nx_g = int((new_wx - self.min_x) / self.res_3d)
                    ny_g = int((new_wy - self.min_y) / self.res_3d)
                    
                    new_wth_norm = new_wth
                    while new_wth_norm < 0: new_wth_norm += 2 * math.pi
                    while new_wth_norm >= 2 * math.pi: new_wth_norm -= 2 * math.pi
                    nth_g = int(new_wth_norm / self.res_th) % self.nth
                    
                    if 0 <= nx_g < self.nx_3d and 0 <= ny_g < self.ny_3d:
                        gx_2d = int((new_wx - self.min_x) / self.res)
                        gy_2d = int((new_wy - self.min_y) / self.res)
                        
                        is_obs = False
                        if 0 <= gx_2d < self.nx and 0 <= gy_2d < self.ny:
                            is_obs = self.obs_map[gx_2d][gy_2d]
                        else:
                            is_obs = True
                            
                        if not is_obs:
                            new_d = d + cost
                            if new_d < self.dist_3d[nx_g][ny_g][nth_g]:
                                self.dist_3d[nx_g][ny_g][nth_g] = new_d
                                heapq.heappush(q, (new_d, nx_g, ny_g, nth_g))

    def get_3d_heuristic(self, wx, wy, wth):
        if not hasattr(self, 'dist_3d'):
            return float('inf')
        gx = int((wx - self.min_x) / self.res_3d)
        gy = int((wy - self.min_y) / self.res_3d)
        
        import math
        while wth < 0: wth += 2 * math.pi
        while wth >= 2 * math.pi: wth -= 2 * math.pi
        gth = int(wth / self.res_th) % self.nth
        
        if 0 <= gx < self.nx_3d and 0 <= gy < self.ny_3d:
            return self.dist_3d[gx][gy][gth] * 1.5
        return float('inf')

"""

lines.insert(insert_idx, new_code)

with open('heuristic.py', 'w') as f:
    f.writelines(lines)
