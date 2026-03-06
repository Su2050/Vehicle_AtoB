import math
euclidean_goal = math.hypot(6.14 - 2.10, 4.54 - 0.0)
h_3d = 21.77
print(f"euclidean_goal={euclidean_goal}")
print(f"threshold={euclidean_goal * 1.5 + 1.0}")
print(f"skip={h_3d > euclidean_goal * 1.5 + 1.0}")
