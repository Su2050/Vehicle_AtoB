import sys

with open('heuristic.py', 'r') as f:
    content = f.read()

old_init = """        self.dist_3d = [[[float('inf')] * self.nth for _ in range(self.ny_3d)] for _ in range(self.nx_3d)]
        
        gx_goal = int((self.goal_x - self.min_x) / self.res_3d)
        gy_goal = int((self.goal_y - self.min_y) / self.res_3d)
        gth_goal = 0
        
        if not (0 <= gx_goal < self.nx_3d and 0 <= gy_goal < self.ny_3d):
            return
            
        import heapq
        q = [(0.0, gx_goal, gy_goal, gth_goal)]
        self.dist_3d[gx_goal][gy_goal][gth_goal] = 0.0"""

new_init = """        self.dist_3d = [[[[float('inf')] * 2 for _ in range(self.nth)] for _ in range(self.ny_3d)] for _ in range(self.nx_3d)]
        
        gx_goal = int((self.goal_x - self.min_x) / self.res_3d)
        gy_goal = int((self.goal_y - self.min_y) / self.res_3d)
        gth_goal = 0
        
        if not (0 <= gx_goal < self.nx_3d and 0 <= gy_goal < self.ny_3d):
            return
            
        import heapq
        # 0 for forward, 1 for backward
        q = [(0.0, gx_goal, gy_goal, gth_goal, 0), (0.0, gx_goal, gy_goal, gth_goal, 1)]
        self.dist_3d[gx_goal][gy_goal][gth_goal][0] = 0.0
        self.dist_3d[gx_goal][gy_goal][gth_goal][1] = 0.0"""

old_loop = """        while q:
            d, gx, gy, gth = heapq.heappop(q)
            if d > self.dist_3d[gx][gy][gth]: continue
            
            wx = self.min_x + gx * self.res_3d + self.res_3d/2
            wy = self.min_y + gy * self.res_3d + self.res_3d/2
            wth = gth * self.res_th
            
            for d_step in [0.5, -0.5]:
                for steer in [0.0, 0.2, -0.2]:
                    cost = 0.5 if d_step == -0.5 else 0.75
                    if steer != 0.0: cost += 0.1"""

new_loop = """        while q:
            d, gx, gy, gth, gdir = heapq.heappop(q)
            if d > self.dist_3d[gx][gy][gth][gdir]: continue
            
            wx = self.min_x + gx * self.res_3d + self.res_3d/2
            wy = self.min_y + gy * self.res_3d + self.res_3d/2
            wth = gth * self.res_th
            
            for d_step in [0.5, -0.5]:
                for steer in [0.0, 0.2, -0.2]:
                    # d_step in build_3d_map is reverse kinematics.
                    # d_step = -0.5 means the actual path moves FORWARD to reach this state.
                    new_dir = 0 if d_step < 0 else 1
                    cost = 0.5 if d_step == -0.5 else 0.75
                    if steer != 0.0: cost += 0.1
                    if new_dir != gdir: cost += 3.0  # Gear shift penalty"""

old_push = """                        if not is_obs:
                            new_d = d + cost
                            if new_d < self.dist_3d[nx_g][ny_g][nth_g]:
                                self.dist_3d[nx_g][ny_g][nth_g] = new_d
                                heapq.heappush(q, (new_d, nx_g, ny_g, nth_g))"""

new_push = """                        if not is_obs:
                            new_d = d + cost
                            if new_d < self.dist_3d[nx_g][ny_g][nth_g][new_dir]:
                                self.dist_3d[nx_g][ny_g][nth_g][new_dir] = new_d
                                heapq.heappush(q, (new_d, nx_g, ny_g, nth_g, new_dir))"""

old_get = """    def get_3d_heuristic(self, wx, wy, wth):
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
        return float('inf')"""

new_get = """    def get_3d_heuristic(self, wx, wy, wth):
        if not hasattr(self, 'dist_3d'):
            return float('inf')
        gx = int((wx - self.min_x) / self.res_3d)
        gy = int((wy - self.min_y) / self.res_3d)
        
        import math
        while wth < 0: wth += 2 * math.pi
        while wth >= 2 * math.pi: wth -= 2 * math.pi
        gth = int(wth / self.res_th) % self.nth
        
        if 0 <= gx < self.nx_3d and 0 <= gy < self.ny_3d:
            return min(self.dist_3d[gx][gy][gth][0], self.dist_3d[gx][gy][gth][1]) * 1.5
        return float('inf')"""

content = content.replace(old_init, new_init)
content = content.replace(old_loop, new_loop)
content = content.replace(old_push, new_push)
content = content.replace(old_get, new_get)

with open('heuristic.py', 'w') as f:
    f.write(content)
print("Patched heuristic.py successfully!")
