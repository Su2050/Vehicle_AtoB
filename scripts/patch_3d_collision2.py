import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_check = """                        for offset in [0.375, 0.0, -0.375]:
                            cx = new_wx + offset * cos_t
                            cy = new_wy + offset * sin_t"""

new_check = """                        import primitives
                        for offset in primitives.VEHICLE_CHECK_OFFSETS:
                            cx = new_wx - offset * cos_t
                            cy = new_wy - offset * sin_t"""

content = content.replace(old_check, new_check)

with open('heuristic.py', 'w') as f:
    f.write(content)
