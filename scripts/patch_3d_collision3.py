import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_check = """                        import primitives
                        for offset in primitives.VEHICLE_CHECK_OFFSETS:
                            cx = new_wx - offset * cos_t
                            cy = new_wy - offset * sin_t"""

new_check = """                        import primitives
                        new_cos_t = math.cos(new_wth_norm)
                        new_sin_t = math.sin(new_wth_norm)
                        for offset in primitives.VEHICLE_CHECK_OFFSETS:
                            cx = new_wx - offset * new_cos_t
                            cy = new_wy - offset * new_sin_t"""

content = content.replace(old_check, new_check)

with open('heuristic.py', 'w') as f:
    f.write(content)
