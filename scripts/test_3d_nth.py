import math
import primitives

res_th = 2 * math.pi / 16
th = -2.63
th_norm = th
while th_norm < -math.pi: th_norm += 2 * math.pi
while th_norm >= math.pi: th_norm -= 2 * math.pi
gth = int((th_norm + math.pi) / res_th) % 16

print(f"th={th}, th_norm={th_norm}, gth={gth}")

for d_step in [0.25, -0.25]:
    for steer in [0.0, 0.15, -0.15]:
        new_wth = th - steer * d_step
        new_wth_norm = new_wth % (2 * math.pi)
        nth_g = int(new_wth_norm / res_th) % 16
        print(f"d_step={d_step}, steer={steer}, new_wth={new_wth:.2f}, new_wth_norm={new_wth_norm:.2f}, nth_g={nth_g}")
