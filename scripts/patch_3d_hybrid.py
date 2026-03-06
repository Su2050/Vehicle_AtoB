import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_3d = """    def build_3d_map(self):"""

# I will replace the entire build_3d_map and get_3d_heuristic
import re
match = re.search(r'    def build_3d_map\(self\):.*', content, re.DOTALL)
if match:
    old_code = match.group(0)

new_code = """    def build_3d_map(self):
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
            
        q = [(0.0, gx_goal, gy_goal, gth_goal)]
        self.dist_3d[gx_goal][gy_goal][gth_goal] = 0.0
        
        # Precompute transitions (reverse motions from goal to start)
        # R = 2.5m. Step = 1.0m.
        # dth = step / R = 1.0 / 2.5 = 0.4 rad.
        # 0.4 rad is about 1/2 of pi/4 (0.78 rad).
        # So we can transition th by 0.4 rad.
        # To map to grid, we just add continuous th and discretize.
        
        while q:
            d, gx, gy, gth = heapq.heappop(q)
            if d > self.dist_3d[gx][gy][gth]: continue
            
            wx = self.min_x + gx * self.res_3d + self.res_3d/2
            wy = self.min_y + gy * self.res_3d + self.res_3d/2
            wth = gth * self.res_th
            
            # Try reverse primitives (since we search backwards from goal)
            # Forward motion in real life = backward motion in search
            for d_step in [1.0, -1.0]:  # 1.0 is real-life backward, -1.0 is real-life forward
                for steer in [0.0, 0.4, -0.4]:  # straight, left, right
                    cost = 1.0 if d_step == -1.0 else 1.5  # backward is more expensive
                    if steer != 0.0: cost += 0.2
                    
                    # Reverse kinematics
                    # In real life: x' = x + cos(th)*d, th' = th + steer
                    # In reverse: x = x' - cos(th')*d, th = th' - steer
                    # Wait, it's easier:
                    new_wth = wth - steer * d_step
                    new_wx = wx - d_step * math.cos(new_wth)
                    new_wy = wy - d_step * math.sin(new_wth)
                    
                    nx_g = int((new_wx - self.min_x) / self.res_3d)
                    ny_g = int((new_wy - self.min_y) / self.res_3d)
                    
                    while new_wth < 0: new_wth += 2 * math.pi
                    while new_wth >= 2 * math.pi: new_wth -= 2 * math.pi
                    nth_g = int(new_wth / self.res_th) % self.nth
                    
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
        
        while wth < 0: wth += 2 * math.pi
        while wth >= 2 * math.pi: wth -= 2 * math.pi
        gth = int(wth / self.res_th) % self.nth
        
        if 0 <= gx < self.nx_3d and 0 <= gy < self.ny_3d:
            return self.dist_3d[gx][gy][gth] * 1.5  # Scale
        return float('inf')"""

content = content.replace(old_code, new_code)

with open('heuristic.py', 'w') as f:
    f.write(content)
