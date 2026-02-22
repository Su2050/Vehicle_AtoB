import math

def calc_h(x, y, th, target_th):
    diff = abs(th - target_th)
    while diff > math.pi: diff -= 2 * math.pi
    diff = abs(diff)
    align_diff = min(diff, math.pi - diff)
    return align_diff * 4.0

print("Start align penalty:", calc_h(5.05, -1.25, -0.25, 2.2))
