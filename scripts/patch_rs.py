import re

with open('rs.py', 'r') as f:
    content = f.read()

# Replace rs_sample_path_multi signature
old_sig = "def rs_sample_path_multi(x1, y1, th1, x2, y2, th2, turning_radius, step=0.05, max_paths=5):"
new_sig = "def rs_sample_path_multi(x1, y1, th1, x2, y2, th2, turning_radius, step=0.05, max_paths=5, collision_fn=None):"
content = content.replace(old_sig, new_sig)

# Find the loop where world_pts are appended
old_loop = """        # 逆变换：RS 标准系 → local（180° 旋转逆）→ world
        world_pts = []
        for xr, yr, thr in rs_pts:
            lx_p = -xr
            ly_p = -yr
            wx = x1 + lx_p * cos_t - ly_p * sin_t
            wy = y1 + lx_p * sin_t + ly_p * cos_t
            wth = th1 + thr
            while wth > _PI: wth -= _TWO_PI
            while wth <= -_PI: wth += _TWO_PI
            world_pts.append((wx, wy, wth))
        results.append(world_pts)"""

new_loop = """        # 逆变换：RS 标准系 → local（180° 旋转逆）→ world
        world_pts = []
        traj_ok = True
        for xr, yr, thr in rs_pts:
            lx_p = -xr
            ly_p = -yr
            wx = x1 + lx_p * cos_t - ly_p * sin_t
            wy = y1 + lx_p * sin_t + ly_p * cos_t
            wth = th1 + thr
            while wth > _PI: wth -= _TWO_PI
            while wth <= -_PI: wth += _TWO_PI
            if collision_fn is not None:
                valid, _ = collision_fn(wx, wy, wth)
                if not valid:
                    traj_ok = False
                    break
            world_pts.append((wx, wy, wth))
        if traj_ok:
            results.append(world_pts)
        else:
            results.append([]) # Append empty to maintain index, or just empty list to indicate failure"""

content = content.replace(old_loop, new_loop)

with open('rs.py', 'w') as f:
    f.write(content)
