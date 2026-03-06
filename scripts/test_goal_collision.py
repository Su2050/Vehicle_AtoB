from collision import check_collision
import math

obs = [(2.345, 4.195, -0.455, 0.015)]
valid, _ = check_collision(2.10, 0.0, 0.0, 0.0, no_corridor=True, obstacles=obs)
print(f"Goal state (2.10, 0.0, 0.0) valid: {valid}")
