import time
import math
import heapq

def build_3d_heuristic(obstacles, goal_x, goal_y, goal_th):
    t0 = time.perf_counter()
    res_xy = 0.5
    res_th = math.pi / 4
    
    nx = int(12.0 / res_xy)
    ny = int(12.0 / res_xy)
    nth = 8
    
    # 1. Build 2D obs map
    obs_map = [[False] * ny for _ in range(nx)]
    for obs in obstacles:
        if isinstance(obs, tuple):
            min_x, max_x, min_y, max_y = obs
        else:
            ox, oy, ow, oh = obs['x'], obs['y'], obs['w'], obs['h']
            min_x, max_x = min(ox, ox + ow), max(ox, ox + ow)
            min_y, max_y = min(oy, oy + oh), max(oy, oy + oh)
            
        gx_min = max(0, int(min_x / res_xy))
        gx_max = min(nx - 1, int(max_x / res_xy))
        gy_min = max(0, int((min_y + 6.0) / res_xy))
        gy_max = min(ny - 1, int((max_y + 6.0) / res_xy))
        
        for x in range(gx_min, gx_max + 1):
            for y in range(gy_min, gy_max + 1):
                obs_map[x][y] = True
                
    # 2. Dijkstra 3D
    dist = [[[float('inf')] * nth for _ in range(ny)] for _ in range(nx)]
    
    gx_goal = int(goal_x / res_xy)
    gy_goal = int((goal_y + 6.0) / res_xy)
    gth_goal = int((goal_th % (2 * math.pi)) / res_th) % nth
    
    q = [(0.0, gx_goal, gy_goal, gth_goal)]
    dist[gx_goal][gy_goal][gth_goal] = 0.0
    
    # Transitions: forward, backward, turn in place (approximation)
    # Actually, we can use simple dubins-like transitions
    transitions = []
    for d_step in [1, -1]: # forward, backward
        for d_th in [0, 1, -1]: # straight, left, right
            cost = 1.0 if d_step == 1 else 1.5
            if d_th != 0: cost += 0.5
            transitions.append((d_step, d_th, cost))
            
    expanded = 0
    while q:
        d, x, y, th = heapq.heappop(q)
        if d > dist[x][y][th]: continue
        expanded += 1
        
        th_rad = th * res_th
        cos_t = math.cos(th_rad)
        sin_t = math.sin(th_rad)
        
        for d_step, d_th, cost in transitions:
            # Move
            step_dist = res_xy * 1.5
            nx_f = x * res_xy + d_step * step_dist * cos_t
            ny_f = (y * res_xy - 6.0) + d_step * step_dist * sin_t
            
            nx_g = int(nx_f / res_xy)
            ny_g = int((ny_f + 6.0) / res_xy)
            nth_g = (th + d_th) % nth
            
            if 0 <= nx_g < nx and 0 <= ny_g < ny:
                if not obs_map[nx_g][ny_g]:
                    new_d = d + cost
                    if new_d < dist[nx_g][ny_g][nth_g]:
                        dist[nx_g][ny_g][nth_g] = new_d
                        heapq.heappush(q, (new_d, nx_g, ny_g, nth_g))
                        
    t1 = time.perf_counter()
    print(f"3D Heuristic built in {t1-t0:.3f}s, expanded {expanded} nodes")
    return dist

import json
with open('logs/stress_timeouts_20260302_184337.json', 'r') as f:
    data = json.load(f)
case = data[0]
build_3d_heuristic(case['obstacles'], 2.10, 0.0, 0.0)
