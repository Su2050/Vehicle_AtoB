import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_check = """                        is_obs = False
                        if 0 <= gx_2d < self.nx and 0 <= gy_2d < self.ny:
                            is_obs = self.obs_map[gx_2d][gy_2d]
                        else:
                            is_obs = True"""

new_check = """                        is_obs = False
                        # Check 3 circles for the vehicle
                        cos_t = math.cos(new_wth_norm)
                        sin_t = math.sin(new_wth_norm)
                        for offset in [0.375, 0.0, -0.375]:
                            cx = new_wx + offset * cos_t
                            cy = new_wy + offset * sin_t
                            
                            cgx = int((cx - self.min_x) / self.res)
                            cgy = int((cy - self.min_y) / self.res)
                            
                            if 0 <= cgx < self.nx and 0 <= cgy < self.ny:
                                if self.obs_map[cgx][cgy]:
                                    is_obs = True
                                    break
                            else:
                                is_obs = True
                                break"""

content = content.replace(old_check, new_check)

with open('heuristic.py', 'w') as f:
    f.write(content)
