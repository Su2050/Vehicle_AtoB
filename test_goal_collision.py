from main import check_collision
import math

obstacles = [{'x': 3.0, 'y': -2.0, 'w': 0.8, 'h': 1.8}]
valid, details = check_collision(2.1, 0.0, 0.0, 0.0, obstacles=obstacles)
print(f"Valid: {valid}")
