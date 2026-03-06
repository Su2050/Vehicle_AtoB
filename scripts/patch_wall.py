import sys

with open('heuristic.py', 'r') as f:
    content = f.read()

old_code = """        # Mark hard walls (x < 1.9)
        wall_gx_max, _ = self._world_to_grid(1.9, 0)
        for x in range(0, min(self.nx, wall_gx_max)):
            for y in range(self.ny):
                self.obs_map[x][y] = True"""

new_code = """        # Mark hard walls (x < 1.9)
        wall_gx_max, _ = self._world_to_grid(1.9, 0)
        for x in range(0, min(self.nx, wall_gx_max)):
            for y in range(self.ny):
                wx, wy = self._grid_to_world(x, y)
                if abs(wy) > 0.5:
                    self.obs_map[x][y] = True
                    self.real_obs_map[x][y] = True"""

if old_code in content:
    content = content.replace(old_code, new_code)
    with open('heuristic.py', 'w') as f:
        f.write(content)
    print("Patched wall successfully!")
else:
    print("Could not find the target code block.")
