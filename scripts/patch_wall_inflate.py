import re

with open('heuristic.py', 'r') as f:
    content = f.read()

old_wall = """        # Mark hard walls (x < 1.9)
        wall_gx_max, _ = self._world_to_grid(1.9, 0)
        for x in range(0, min(self.nx, wall_gx_max)):
            for y in range(self.ny):
                self.obs_map[x][y] = True"""

new_wall = """        # Mark hard walls (x < 1.9)
        # Inflate the wall by inf_cells so the heuristic knows the vehicle can't get too close
        wall_gx_max, _ = self._world_to_grid(1.9, 0)
        for x in range(0, min(self.nx, wall_gx_max + inf_cells)):
            for y in range(self.ny):
                self.obs_map[x][y] = True"""

content = content.replace(old_wall, new_wall)

with open('heuristic.py', 'w') as f:
    f.write(content)
