import re

with open('collision.py', 'r') as f:
    content = f.read()

# Optimize check_collision by avoiding function calls and using fast math
old_check = """            dx0 = nx - max_x if nx > max_x else (min_x - nx if nx < min_x else 0.0)
            dy0 = ny - max_y if ny > max_y else (min_y - ny if ny < min_y else 0.0)
            if dx0 * dx0 + dy0 * dy0 >= max_r_sq:
                continue

            # 多圆碰撞检测
            for offset in offsets:
                # Forward is -x in this coordinate system, so subtract offset
                px = nx - offset * cos_nth
                py = ny - offset * sin_nth
                dx = px - max_x if px > max_x else (min_x - px if px < min_x else 0.0)
                dy = py - max_y if py > max_y else (min_y - py if py < min_y else 0.0)
                if dx * dx + dy * dy < half_w_sq:
                    return False, 'OBSTACLE'"""

new_check = """            if nx > max_x: dx0 = nx - max_x
            elif nx < min_x: dx0 = min_x - nx
            else: dx0 = 0.0
            
            if ny > max_y: dy0 = ny - max_y
            elif ny < min_y: dy0 = min_y - ny
            else: dy0 = 0.0
            
            if dx0 * dx0 + dy0 * dy0 >= max_r_sq:
                continue

            # 多圆碰撞检测
            for offset in offsets:
                px = nx - offset * cos_nth
                py = ny - offset * sin_nth
                
                if px > max_x: dx = px - max_x
                elif px < min_x: dx = min_x - px
                else: dx = 0.0
                
                if py > max_y: dy = py - max_y
                elif py < min_y: dy = min_y - py
                else: dy = 0.0
                
                if dx * dx + dy * dy < half_w_sq:
                    return False, 'OBSTACLE'"""

content = content.replace(old_check, new_check)

with open('collision.py', 'w') as f:
    f.write(content)
