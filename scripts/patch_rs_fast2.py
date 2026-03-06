import re

with open('rs.py', 'r') as f:
    content = f.read()

old_loop = """                if collision_fn is not None:
                    valid, _ = collision_fn(wx, wy, wth)
                    if not valid:
                        traj_ok = False
                        break
                world_pts.append((wx, wy, wth))"""

new_loop = """                world_pts.append((wx, wy, wth))
        
        if traj_ok and collision_fn is not None:
            # Fast check with step=0.5
            fast_step = 5
            for i in range(0, len(world_pts), fast_step):
                pt = world_pts[i]
                valid, _ = collision_fn(pt[0], pt[1], pt[2])
                if not valid:
                    traj_ok = False
                    break
            
            # Detailed check if fast check passed
            if traj_ok:
                for i in range(len(world_pts)):
                    if i % fast_step == 0: continue
                    pt = world_pts[i]
                    valid, _ = collision_fn(pt[0], pt[1], pt[2])
                    if not valid:
                        traj_ok = False
                        break"""

content = content.replace(old_loop, new_loop)

with open('rs.py', 'w') as f:
    f.write(content)
