import re

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'r') as f:
    content = f.read()

# Fix how wth is calculated from gth
content = content.replace("wth = gth * self.res_th", "wth = -math.pi + gth * self.res_th")

# Fix how nth_g is calculated from new_wth
old_calc = "new_wth_norm = new_wth % (2 * math.pi)\n                    nth_g = int(new_wth_norm / self.res_th) % self.nth"
new_calc = """new_wth_norm = new_wth
                    while new_wth_norm < -math.pi: new_wth_norm += 2 * math.pi
                    while new_wth_norm >= math.pi: new_wth_norm -= 2 * math.pi
                    nth_g = int((new_wth_norm + math.pi) / self.res_th) % self.nth"""
content = content.replace(old_calc, new_calc)

# Fix how get_3d_heuristic calculates gth
old_get_calc = "wth_norm = wth % (2 * math.pi)\n        gth = int(wth_norm / self.res_th) % self.nth"
new_get_calc = """wth_norm = wth
        while wth_norm < -math.pi: wth_norm += 2 * math.pi
        while wth_norm >= math.pi: wth_norm -= 2 * math.pi
        gth = int((wth_norm + math.pi) / self.res_th) % self.nth"""
content = content.replace(old_get_calc, new_get_calc)

with open('/Users/sull/Documents/untitled folder/Vehicle_AtoB/scripts/heuristic.py', 'w') as f:
    f.write(content)
