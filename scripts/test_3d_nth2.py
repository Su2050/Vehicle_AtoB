import math

res_th = 2 * math.pi / 16

def to_gth(th):
    th_norm = th
    while th_norm < -math.pi: th_norm += 2 * math.pi
    while th_norm >= math.pi: th_norm -= 2 * math.pi
    return int((th_norm + math.pi) / res_th) % 16

def to_gth2(th):
    th_norm = th % (2 * math.pi)
    return int(th_norm / res_th) % 16

for th in [-3.14, -2.63, -1.5, 0, 1.5, 2.63, 3.14]:
    print(f"th={th}, gth1={to_gth(th)}, gth2={to_gth2(th)}")
