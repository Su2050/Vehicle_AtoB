import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_3d = """        transitions = []
        for d_step in [1, -1]:
            for d_th in [0, 1, -1]:
                cost = 1.0 if d_step == 1 else 1.5
                if d_th != 0: cost += 0.5
                transitions.append((d_step, d_th, cost))
                
        while q:
            d, x, y, th = heapq.heappop(q)
            if d > self.dist_3d[x][y][th]: continue
            
            th_rad = th * self.res_th
            cos_t = math.cos(th_rad)
            sin_t = math.sin(th_rad)
            
            for d_step, d_th, cost in transitions:
                step_dist = self.res_3d * 1.5
                nx_f = self.min_x + x * self.res_3d + d_step * step_dist * cos_t
                ny_f = self.min_y + y * self.res_3d + d_step * step_dist * sin_t
                
                nx_g = int((nx_f - self.min_x) / self.res_3d)
                ny_g = int((ny_f - self.min_y) / self.res_3d)
                nth_g = (th + d_th) % self.nth
                
                if 0 <= nx_g < self.nx_3d and 0 <= ny_g < self.ny_3d:
                    # Check 2D obs map at this location
                    # We can use the 2D obs map we already built
                    gx_2d = int((nx_f - self.min_x) / self.res)
                    gy_2d = int((ny_f - self.min_y) / self.res)
                    
                    is_obs = False
                    if 0 <= gx_2d < self.nx and 0 <= gy_2d < self.ny:
                        is_obs = self.obs_map[gx_2d][gy_2d]
                    else:
                        is_obs = True
                        
                    if not is_obs:
                        new_d = d + cost
                        if new_d < self.dist_3d[nx_g][ny_g][nth_g]:
                            self.dist_3d[nx_g][ny_g][nth_g] = new_d
                            heapq.heappush(q, (new_d, nx_g, ny_g, nth_g))"""

new_3d = """        while q:
            d, x, y, th = heapq.heappop(q)
            if d > self.dist_3d[x][y][th]: continue
            
            # Forward and backward transitions
            for d_step in [1, -1]:
                for d_th in [0, 1, -1]:
                    cost = 1.0 if d_step == 1 else 1.5
                    if d_th != 0: cost += 0.5
                    
                    # The heading AFTER the turn
                    nth_g = (th + d_th) % self.nth
                    
                    # The movement direction depends on the CURRENT heading
                    th_rad = th * self.res_th
                    dx = int(round(math.cos(th_rad))) * d_step
                    dy = int(round(math.sin(th_rad))) * d_step
                    
                    nx_g = x + dx
                    ny_g = y + dy
                    
                    if 0 <= nx_g < self.nx_3d and 0 <= ny_g < self.ny_3d:
                        # Check 2D obs map at this location
                        wx = self.min_x + nx_g * self.res_3d
                        wy = self.min_y + ny_g * self.res_3d
                        gx_2d = int((wx - self.min_x) / self.res)
                        gy_2d = int((wy - self.min_y) / self.res)
                        
                        is_obs = False
                        if 0 <= gx_2d < self.nx and 0 <= gy_2d < self.ny:
                            is_obs = self.obs_map[gx_2d][gy_2d]
                        else:
                            is_obs = True
                            
                        if not is_obs:
                            new_d = d + cost
                            if new_d < self.dist_3d[nx_g][ny_g][nth_g]:
                                self.dist_3d[nx_g][ny_g][nth_g] = new_d
                                heapq.heappush(q, (new_d, nx_g, ny_g, nth_g))"""

content = content.replace(old_3d, new_3d)

with open('heuristic.py', 'w') as f:
    f.write(content)
