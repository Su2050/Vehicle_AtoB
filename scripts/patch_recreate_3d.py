import sys

with open('heuristic.py', 'r') as f:
    content = f.read()

new_methods = """
    def build_3d_map(self):
        self.res_3d = 0.125
        self.nth = 32
        import math
        self.res_th = 2 * math.pi / self.nth
        self.nx_3d = int((self.max_x - self.min_x) / self.res_3d)
        self.ny_3d = int((self.max_y - self.min_y) / self.res_3d)
        
        self.dist_3d = [[[[float('inf')] * 2 for _ in range(self.nth)] for _ in range(self.ny_3d)] for _ in range(self.nx_3d)]
        
        import heapq
        q = []
        for gx in range(self.nx_3d):
            wx = self.min_x + gx * self.res_3d + self.res_3d/2
            if 0.5 <= wx <= 2.25:
                for gy in range(self.ny_3d):
                    wy = self.min_y + gy * self.res_3d + self.res_3d/2
                    if abs(wy) <= 0.2:
                        gth_goal = 0
                        is_obs = False
                        cos_t = 1.0
                        sin_t = 0.0
                        import primitives
                        for offset in primitives.VEHICLE_CHECK_OFFSETS:
                            cx = wx - offset * cos_t
                            cy = wy - offset * sin_t
                            cgx = int((cx - self.min_x) / self.res)
                            cgy = int((cy - self.min_y) / self.res)
                            if 0 <= cgx < self.nx and 0 <= cgy < self.ny:
                                if self.obs_map[cgx][cgy]:
                                    is_obs = True
                                    break
                            else:
                                is_obs = True
                                break
                        if not is_obs:
                            dist_to_ideal = math.hypot(wx - 2.10, wy - 0.0)
                            q.append((dist_to_ideal, gx, gy, gth_goal, 0))
                            q.append((dist_to_ideal, gx, gy, gth_goal, 1))
                            self.dist_3d[gx][gy][gth_goal][0] = dist_to_ideal
                            self.dist_3d[gx][gy][gth_goal][1] = dist_to_ideal
        
        if not q:
            gx_goal = int((self.goal_x - self.min_x) / self.res_3d)
            gy_goal = int((self.goal_y - self.min_y) / self.res_3d)
            gth_goal = 0
            if 0 <= gx_goal < self.nx_3d and 0 <= gy_goal < self.ny_3d:
                q = [(0.0, gx_goal, gy_goal, gth_goal, 0), (0.0, gx_goal, gy_goal, gth_goal, 1)]
                self.dist_3d[gx_goal][gy_goal][gth_goal][0] = 0.0
                self.dist_3d[gx_goal][gy_goal][gth_goal][1] = 0.0
            else:
                return
        
        heapq.heapify(q)
        
        while q:
            d, gx, gy, gth, gdir = heapq.heappop(q)
            if d > self.dist_3d[gx][gy][gth][gdir]: continue
            
            wx = self.min_x + gx * self.res_3d + self.res_3d/2
            wy = self.min_y + gy * self.res_3d + self.res_3d/2
            wth = gth * self.res_th
            
            for d_step in [0.25, -0.25]:
                for steer in [0.0, 0.1, -0.1]:
                    new_dir = 0 if d_step < 0 else 1
                    cost = 0.25 if d_step == -0.25 else 0.375
                    if steer != 0.0: cost += 0.1
                    if new_dir != gdir: cost += 1.0
                    
                    new_wth = wth - steer * d_step
                    new_wx = wx - d_step * math.cos(new_wth)
                    new_wy = wy - d_step * math.sin(new_wth)
                    
                    new_wth_norm = new_wth % (2 * math.pi)
                    nth_g = int(new_wth_norm / self.res_th) % self.nth
                    nx_g = int((new_wx - self.min_x) / self.res_3d)
                    ny_g = int((new_wy - self.min_y) / self.res_3d)
                    
                    if 0 <= nx_g < self.nx_3d and 0 <= ny_g < self.ny_3d:
                        is_obs = False
                        new_cos_t = math.cos(new_wth_norm)
                        new_sin_t = math.sin(new_wth_norm)
                        for offset in primitives.VEHICLE_CHECK_OFFSETS:
                            cx = new_wx - offset * new_cos_t
                            cy = new_wy - offset * new_sin_t
                            cgx = int((cx - self.min_x) / self.res)
                            cgy = int((cy - self.min_y) / self.res)
                            if 0 <= cgx < self.nx and 0 <= cgy < self.ny:
                                if self.obs_map[cgx][cgy]:
                                    is_obs = True
                                    break
                            else:
                                is_obs = True
                                break
                                
                        if not is_obs:
                            new_d = d + cost
                            if new_d < self.dist_3d[nx_g][ny_g][nth_g][new_dir]:
                                self.dist_3d[nx_g][ny_g][nth_g][new_dir] = new_d
                                heapq.heappush(q, (new_d, nx_g, ny_g, nth_g, new_dir))

    def get_3d_heuristic(self, wx, wy, wth):
        if not hasattr(self, 'dist_3d'):
            return float('inf')
        gx = int((wx - self.min_x) / self.res_3d)
        gy = int((wy - self.min_y) / self.res_3d)
        
        import math
        wth_norm = wth % (2 * math.pi)
        gth = int(wth_norm / self.res_th) % self.nth
        
        if 0 <= gx < self.nx_3d and 0 <= gy < self.ny_3d:
            return min(self.dist_3d[gx][gy][gth][0], self.dist_3d[gx][gy][gth][1]) * 1.5
        return float('inf')

def geometric_heuristic(nx, ny, nth):"""

if "def build_3d_map" not in content:
    content = content.replace("def geometric_heuristic(nx, ny, nth):", new_methods)
    with open('heuristic.py', 'w') as f:
        f.write(content)
    print("Recreated build_3d_map successfully!")
else:
    print("build_3d_map already exists.")
