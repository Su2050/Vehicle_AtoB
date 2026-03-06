import sys

with open('heuristic.py', 'r') as f:
    content = f.read()

old_init = """        gx_goal = int((self.goal_x - self.min_x) / self.res_3d)
        gy_goal = int((self.goal_y - self.min_y) / self.res_3d)
        gth_goal = 0
        
        if not (0 <= gx_goal < self.nx_3d and 0 <= gy_goal < self.ny_3d):
            return
            
        import heapq
        # 0 for forward, 1 for backward
        q = [(0.0, gx_goal, gy_goal, gth_goal, 0), (0.0, gx_goal, gy_goal, gth_goal, 1)]
        self.dist_3d[gx_goal][gy_goal][gth_goal][0] = 0.0
        self.dist_3d[gx_goal][gy_goal][gth_goal][1] = 0.0"""

new_init = """        import heapq
        q = []
        # Initialize all valid states in the goal region x in [0.5, 2.25], y in [-0.2, 0.2], th=0
        for gx in range(self.nx_3d):
            wx = self.min_x + gx * self.res_3d + self.res_3d/2
            if 0.5 <= wx <= 2.25:
                for gy in range(self.ny_3d):
                    wy = self.min_y + gy * self.res_3d + self.res_3d/2
                    if abs(wy) <= 0.2:
                        gth_goal = 0
                        # Check if this state is valid
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
                            # Distance is based on how far it is from the ideal goal (2.10, 0)
                            dist_to_ideal = math.hypot(wx - 2.10, wy - 0.0)
                            q.append((dist_to_ideal, gx, gy, gth_goal, 0))
                            q.append((dist_to_ideal, gx, gy, gth_goal, 1))
                            self.dist_3d[gx][gy][gth_goal][0] = dist_to_ideal
                            self.dist_3d[gx][gy][gth_goal][1] = dist_to_ideal
        
        if not q:
            # Fallback to original behavior if no valid states found
            gx_goal = int((self.goal_x - self.min_x) / self.res_3d)
            gy_goal = int((self.goal_y - self.min_y) / self.res_3d)
            gth_goal = 0
            if 0 <= gx_goal < self.nx_3d and 0 <= gy_goal < self.ny_3d:
                q = [(0.0, gx_goal, gy_goal, gth_goal, 0), (0.0, gx_goal, gy_goal, gth_goal, 1)]
                self.dist_3d[gx_goal][gy_goal][gth_goal][0] = 0.0
                self.dist_3d[gx_goal][gy_goal][gth_goal][1] = 0.0
            else:
                return
        
        heapq.heapify(q)"""

if old_init in content:
    content = content.replace(old_init, new_init)
    with open('heuristic.py', 'w') as f:
        f.write(content)
    print("Patched heuristic.py successfully!")
else:
    print("Could not find the target code block.")
