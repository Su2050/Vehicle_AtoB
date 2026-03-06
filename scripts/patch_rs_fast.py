import re

with open('rs.py', 'r') as f:
    content = f.read()

old_func = """    results = []
    for segs in all_segs[:max_paths]:
        cx_rs, cy_rs, cth_rs = 0.0, 0.0, 0.0
        rs_pts = [(cx_rs, cy_rs, cth_rs)]
        for stype, length_m in segs:
            direction = 1 if length_m >= 0 else -1
            dist_left = abs(length_m)
            while dist_left > 1e-9:
                ds = min(step, dist_left)
                dist_left -= ds
                if stype == 'S':
                    cx_rs += direction * ds * math.cos(cth_rs)
                    cy_rs += direction * ds * math.sin(cth_rs)
                elif stype == 'L':
                    dth = direction * ds / turning_radius
                    cx_rs = cx_rs + turning_radius * (math.sin(cth_rs + dth) - math.sin(cth_rs))
                    cy_rs = cy_rs + turning_radius * (math.cos(cth_rs) - math.cos(cth_rs + dth))
                    cth_rs += dth
                else:
                    dth = -direction * ds / turning_radius
                    cx_rs = cx_rs + turning_radius * (math.sin(cth_rs) - math.sin(cth_rs + dth))
                    cy_rs = cy_rs + turning_radius * (math.cos(cth_rs + dth) - math.cos(cth_rs))
                    cth_rs += dth
                while cth_rs > _PI: cth_rs -= _TWO_PI
                while cth_rs <= -_PI: cth_rs += _TWO_PI
                rs_pts.append((cx_rs, cy_rs, cth_rs))

        # 逆变换：RS 标准系 → local（180° 旋转逆）→ world
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
            results.append([]) # Append empty to maintain index, or just empty list to indicate failure
    return results"""

new_func = """    results = []
    for segs in all_segs[:max_paths]:
        cx_rs, cy_rs, cth_rs = 0.0, 0.0, 0.0
        
        # Add first point
        lx_p = -cx_rs
        ly_p = -cy_rs
        wx = x1 + lx_p * cos_t - ly_p * sin_t
        wy = y1 + lx_p * sin_t + ly_p * cos_t
        wth = th1 + cth_rs
        while wth > _PI: wth -= _TWO_PI
        while wth <= -_PI: wth += _TWO_PI
        
        traj_ok = True
        if collision_fn is not None:
            valid, _ = collision_fn(wx, wy, wth)
            if not valid:
                traj_ok = False
                
        world_pts = [(wx, wy, wth)]
        
        if not traj_ok:
            results.append([])
            continue
            
        for stype, length_m in segs:
            if not traj_ok: break
            direction = 1 if length_m >= 0 else -1
            dist_left = abs(length_m)
            while dist_left > 1e-9:
                ds = min(step, dist_left)
                dist_left -= ds
                if stype == 'S':
                    cx_rs += direction * ds * math.cos(cth_rs)
                    cy_rs += direction * ds * math.sin(cth_rs)
                elif stype == 'L':
                    dth = direction * ds / turning_radius
                    cx_rs = cx_rs + turning_radius * (math.sin(cth_rs + dth) - math.sin(cth_rs))
                    cy_rs = cy_rs + turning_radius * (math.cos(cth_rs) - math.cos(cth_rs + dth))
                    cth_rs += dth
                else:
                    dth = -direction * ds / turning_radius
                    cx_rs = cx_rs + turning_radius * (math.sin(cth_rs) - math.sin(cth_rs + dth))
                    cy_rs = cy_rs + turning_radius * (math.cos(cth_rs + dth) - math.cos(cth_rs))
                    cth_rs += dth
                while cth_rs > _PI: cth_rs -= _TWO_PI
                while cth_rs <= -_PI: cth_rs += _TWO_PI
                
                lx_p = -cx_rs
                ly_p = -cy_rs
                wx = x1 + lx_p * cos_t - ly_p * sin_t
                wy = y1 + lx_p * sin_t + ly_p * cos_t
                wth = th1 + cth_rs
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
            results.append([])
    return results"""

content = content.replace(old_func, new_func)

with open('rs.py', 'w') as f:
    f.write(content)
