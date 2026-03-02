import pygame
import math
import sys
import argparse
import main
import primitives
try:
    from planner_obs_v2 import plan_path_robust_obs_v2 as _plan_v2
    _V2_AVAILABLE = True
except ImportError:
    _V2_AVAILABLE = False
try:
    import smooth as smooth_mod
    _SMOOTH_AVAILABLE = True
except ImportError:
    _SMOOTH_AVAILABLE = False

SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
FPS = 60

# Visible world range (swapped axes: world_y -> horizontal, world_x -> vertical)
# x = distance to pallet (small x = close, at top of screen)
# y = lateral position (left-right on screen)
VIEW_X_MIN = 1.4
VIEW_X_MAX = 6.0
VIEW_Y_MIN = -5.0
VIEW_Y_MAX = 5.0

# Workspace boundaries from main.py
WALL_X_MIN = 1.92
WALL_X_MAX = 3.0
WALL_Y_MIN = -3.0
WALL_Y_MAX = 3.0

# Goal region
GOAL_X_MAX = 2.25
GOAL_Y_HALF = 0.18

# Fork tip distance from reference point (now computed from vehicle config)
# FORK_TIP_DIST is derived from the frontmost collision circle offset

# UI layout
CANVAS_LEFT = 70
CANVAS_TOP = 50
CANVAS_BOTTOM = SCREEN_HEIGHT - 80
CANVAS_RIGHT = SCREEN_WIDTH - 30

# Colors
COL_BG        = (245, 245, 250)
COL_CANVAS_BG = (255, 255, 255)
COL_GRID      = (220, 220, 225)
COL_GRID_MAJOR = (190, 190, 200)
COL_WALL_FILL = (60, 60, 70)
COL_WALL_LINE = (40, 40, 50)
COL_GOAL      = (80, 200, 120, 120)
COL_CORRIDOR  = (255, 220, 80, 80)
COL_PALLET    = (180, 140, 80)
COL_BODY_OK   = (50, 120, 220)
COL_BODY_BAD  = (220, 60, 60)
COL_FORK      = (80, 80, 90)
COL_FORK_TIP  = (220, 50, 50)
COL_PATH        = (30, 100, 255)
COL_PATH_HIST   = (30, 100, 255, 60)
COL_PATH_RAW    = (150, 180, 255)   # raw path (light blue)
COL_PATH_SMOOTH = (255, 150, 50)    # smoothed path (orange)
COL_PATH_RS     = (180, 100, 220)   # RS analytic expansion segment (purple)
COL_TEXT       = (30, 30, 40)
COL_TEXT_DIM   = (120, 120, 130)
COL_LABEL_BG   = (255, 255, 255, 200)


class Transformer:
    """World (meters) <-> Screen (pixels), with axes swapped for better aspect ratio.
    World Y -> Screen horizontal (left-right)
    World X -> Screen vertical  (pallet at top, far end at bottom)
    """
    def __init__(self):
        canvas_w = CANVAS_RIGHT - CANVAS_LEFT
        canvas_h = CANVAS_BOTTOM - CANVAS_TOP

        view_y_range = VIEW_Y_MAX - VIEW_Y_MIN
        view_x_range = VIEW_X_MAX - VIEW_X_MIN

        self.scale = min(canvas_w / view_y_range, canvas_h / view_x_range)

        used_w = view_y_range * self.scale
        used_h = view_x_range * self.scale
        self.ox = CANVAS_LEFT + (canvas_w - used_w) / 2
        self.oy = CANVAS_TOP + (canvas_h - used_h) / 2

    def w2s(self, wx, wy):
        sx = self.ox + (wy - VIEW_Y_MIN) * self.scale
        sy = self.oy + (wx - VIEW_X_MIN) * self.scale
        return int(sx), int(sy)

    def s2w(self, sx, sy):
        wy = (sx - self.ox) / self.scale + VIEW_Y_MIN
        wx = (sy - self.oy) / self.scale + VIEW_X_MIN
        return wx, wy

    def m2px(self, meters):
        return int(meters * self.scale)


class App:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Forklift Sim — Path Planning")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Menlo", 13)
        self.label_font = pygame.font.SysFont("Menlo", 12, bold=True)
        self.title_font = pygame.font.SysFont("Menlo", 18, bold=True)
        self.big_font = pygame.font.SysFont("Menlo", 15)

        self.trans = Transformer()
        self.prims = main.init_primitives()

        # State
        self.sx = 2.8
        self.sy = 0.0
        self.sth = 0.0

        self.path = []          # raw discrete trajectory
        self.smooth_path = []   # B-spline smoothed trajectory
        self.rs_traj = []       # RS analytic expansion segment (tail of path)
        self.stage_split_idx = None  # index in self.path where Stage-2 starts
        self.planning = False
        self.animating = False
        self.anim_i = 0
        self.finished = False
        self.msg = ""
        self.msg_color = COL_TEXT

        # Heuristic / obstacle mode flags
        self.planning_mode = 1    # M to toggle: 0=Geometric, 1=RS Heur, 2=Pure RS
        self.rs_radius = 0.8      # +/- to adjust RS expansion radius
        self.no_corridor = False  # C    to toggle: Corridor / No-Corridor
        self.use_smooth = True    # P    to toggle: show smoothed path（默认开启 B 样条平滑）

        # Last planning stats
        self.last_stats = {}

        self.move_speed = 0.8   # m/s
        self.rot_speed = 2.0    # rad/s
        self.dragging = False
        
        # User defined obstacles
        self.obstacles = [] # [{'x': x, 'y': y, 'w': w, 'h': h}]
        self.drawing_obs = False
        self.obs_start_pt = None
        self.obs_current_pt = None

    # ── main loop ──────────────────────────────────────────────
    def run(self):
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    running = False
                self._event(ev)
            self._update(dt)
            self._draw()
            pygame.display.flip()
        pygame.quit()
        sys.exit()

    # ── input ──────────────────────────────────────────────────
    def _event(self, ev):
        mods = pygame.key.get_mods()
        shift_pressed = (mods & pygame.KMOD_SHIFT) != 0

        if ev.type == pygame.KEYDOWN:
            if ev.key == pygame.K_r:
                self._reset()
                return
            if self.planning or self.animating:
                return
            if ev.key == pygame.K_SPACE or ev.key == pygame.K_RETURN:
                self._plan()
            elif ev.key == pygame.K_m:
                self.planning_mode = (self.planning_mode + 1) % 3
                self.path = []
                self.smooth_path = []
                self.finished = False
                self.last_stats = {}
            elif ev.key == pygame.K_EQUALS or ev.key == pygame.K_PLUS:
                self.rs_radius += 0.5
            elif ev.key == pygame.K_MINUS:
                self.rs_radius = max(0.5, self.rs_radius - 0.5)
            elif ev.key == pygame.K_c:
                self.no_corridor = not self.no_corridor
                self.path = []
                self.smooth_path = []
                self.finished = False
                self.last_stats = {}
            elif ev.key == pygame.K_p:
                if _SMOOTH_AVAILABLE and self.path:
                    self.use_smooth = not self.use_smooth
            elif ev.key == pygame.K_x:
                self.obstacles = []
                self.msg = "Obstacles cleared"
                self.msg_color = COL_TEXT

        if self.planning or self.animating:
            return

        if ev.type == pygame.MOUSEBUTTONDOWN and ev.button == 1:
            wx, wy = self.trans.s2w(*ev.pos)
            if shift_pressed:
                self.drawing_obs = True
                self.obs_start_pt = (wx, wy)
                self.obs_current_pt = (wx, wy)
            elif VIEW_X_MIN <= wx <= VIEW_X_MAX and VIEW_Y_MIN <= wy <= VIEW_Y_MAX:
                self.sx, self.sy = wx, wy
                self.path = []
                self.smooth_path = []
                self.rs_traj = []
                self.finished = False
                self.dragging = True
                self.msg = ""
                self.last_stats = {}
        elif ev.type == pygame.MOUSEBUTTONUP and ev.button == 1:
            if self.drawing_obs:
                self.drawing_obs = False
                wx, wy = self.trans.s2w(*ev.pos)
                x1, y1 = self.obs_start_pt
                # Normalize rect (wx, wy can be any corner relative to start_pt)
                ox = min(x1, wx)
                oy = min(y1, wy)
                ow = abs(wx - x1)
                oh = abs(wy - y1)
                if ow > 0.1 and oh > 0.1: # filter out tiny clicks
                    self.obstacles.append({'x': ox, 'y': oy, 'w': ow, 'h': oh})
            else:
                self.dragging = False
        elif ev.type == pygame.MOUSEMOTION:
            wx, wy = self.trans.s2w(*ev.pos)
            if self.drawing_obs:
                self.obs_current_pt = (wx, wy)
            elif self.dragging:
                self.sx = max(VIEW_X_MIN, min(VIEW_X_MAX, wx))
                self.sy = max(VIEW_Y_MIN, min(VIEW_Y_MAX, wy))
            self.path = []
            self.finished = False

    def _update(self, dt):
        if self.planning:
            return

        keys = pygame.key.get_pressed()

        if not self.animating:
            moved = False
            if keys[pygame.K_w] or keys[pygame.K_UP]:
                self.sx -= self.move_speed * dt
                moved = True
            if keys[pygame.K_s] or keys[pygame.K_DOWN]:
                self.sx += self.move_speed * dt
                moved = True
            if keys[pygame.K_a] or keys[pygame.K_LEFT]:
                self.sy -= self.move_speed * dt
                moved = True
            if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
                self.sy += self.move_speed * dt
                moved = True
            if keys[pygame.K_q]:
                self.sth += self.rot_speed * dt
            if keys[pygame.K_e]:
                self.sth -= self.rot_speed * dt

            if self.sth > math.pi:
                self.sth -= 2 * math.pi
            elif self.sth <= -math.pi:
                self.sth += 2 * math.pi

            if moved:
                self.path = []
                self.finished = False

        if self.animating:
            speed = 2 if keys[pygame.K_f] else 1
            self.anim_i += speed
            if self.anim_i >= len(self.path):
                self.anim_i = len(self.path) - 1
                self.animating = False
                self.finished = True
                self.msg = "Done!"
                self.msg_color = (0, 150, 50)

    # ── planning ───────────────────────────────────────────────
    def _plan(self):
        self.planning = True
        mode_names = ["Geometric", "RS Heur", "Pure RS"]
        heur_label = mode_names[self.planning_mode]
        corr_label = "NoCorr" if self.no_corridor else "Corridor"
        if self.planning_mode < 2:
            self.msg = "Planning... [{} + {}] R={:.1f}m".format(heur_label, corr_label, self.rs_radius)
        else:
            self.msg = "Planning... [{} + {}]".format(heur_label, corr_label)
        self.msg_color = COL_TEXT
        self._draw()
        pygame.display.flip()

        st = {}
        if self.planning_mode == 2:
            ok, acts, rs_traj = main.plan_path_pure_rs(
                self.sx, self.sy, self.sth,
                no_corridor=self.no_corridor,
                stats=st,
                obstacles=self.obstacles
            )
        elif _V2_AVAILABLE:
            ok, acts, rs_traj = _plan_v2(
                self.sx, self.sy, self.sth, self.prims,
                use_rs=(self.planning_mode == 1),
                no_corridor=self.no_corridor,
                stats=st,
                rs_expansion_radius=self.rs_radius,
                obstacles=self.obstacles if self.obstacles else None
            )
        else:
            ok, acts, rs_traj = main.plan_path_robust(
                self.sx, self.sy, self.sth, self.prims,
                use_rs=(self.planning_mode == 1),
                no_corridor=self.no_corridor,
                stats=st,
                rs_expansion_radius=self.rs_radius,
                obstacles=self.obstacles
            )
        self.planning = False
        self.last_stats = st

        if not ok:
            if st.get('out_of_range'):
                self.msg = "IMPOSSIBLE  |y|={:.2f}m 超出可规划范围".format(abs(self.sy))
            elif self.planning_mode == 2:
                # 纯 RS 模式如果有碰撞，给出明确的碰撞提示
                self.msg = "COLLISION in Pure RS mode"
            else:
                self.msg = "IMPOSSIBLE  规划失败（|y|={:.2f}m, |θ|={:.1f}°）".format(
                    abs(self.sy), abs(math.degrees(self.sth)))
            self.msg_color = (200, 50, 50)
            
            # 如果是纯 RS 模式，即使失败也画出轨迹
            if self.planning_mode == 2 and rs_traj:
                self.path = rs_traj
                self.anim_i = 0
                self.animating = True
                self.smooth_path = []
                self.rs_traj = []
            return

        self.stage_split_idx = None

        # ── 重建 A* 基元段轨迹 ──────────────────────────────────────
        traj = [(self.sx, self.sy, self.sth)]
        cx, cy, cth = self.sx, self.sy, self.sth
        pmap = {p[0]: p[2] for p in self.prims}

        for act in (acts or []):
            seg = pmap.get(act)
            if seg is None:
                continue
            cos_t, sin_t = math.cos(cth), math.sin(cth)
            for dx, dy, dth, _, _ in seg:
                nx = cx + dx * cos_t - dy * sin_t
                ny = cy + dx * sin_t + dy * cos_t
                nth = cth + dth
                if nth > math.pi: nth -= 2 * math.pi
                elif nth <= -math.pi: nth += 2 * math.pi
                traj.append((nx, ny, nth))
            cx, cy, cth = traj[-1]

        # ── 如果是两阶段规划，记录 Stage-1/2 分割点 ──────────────
        if st.get('two_stage') and acts:
            # 找出两阶段总 act 数中 stage-1 的长度
            stage1_len = st.get('stage1_acts', 0)
            if stage1_len > 0:
                # 计算 Stage-1 覆盖的轨迹点数
                pts_per_act = len(traj) // len(acts)
                self.stage_split_idx = stage1_len * pts_per_act

        # ── 拼接 RS 解析扩展尾段 ────────────────────────────────────
        self.rs_traj = rs_traj or []
        if self.rs_traj:
            traj.extend(self.rs_traj[1:])

        # ── 如果是纯 RS 模式，重建 RS 轨迹 ────────────────────────────────────
        if self.planning_mode == 2:
            self.path = rs_traj or []
            if self.path:
                self.anim_i = 0
                self.animating = True
                
            # 纯 RS 模式不进行 B 样条平滑
            self.smooth_path = []
            self.rs_traj = [] # 清空以防在 _draw 中被当作额外层绘制
        else:
            self.path = traj
            
            # ── 可选 B 样条平滑 ─────────────────────────────────────────
            if _SMOOTH_AVAILABLE and len(traj) >= 4:
                self.smooth_path = smooth_mod.smooth_path(traj)
            else:
                self.smooth_path = []

            self.anim_i = 0
            self.animating = True

        tags = []
        if self.rs_traj:      tags.append("RS Expand")
        if st.get('two_stage'): tags.append("2-Stage")
        tag_str = "  [{}]".format(", ".join(tags)) if tags else ""
        n_steps = len(acts) if acts else 0
        self.msg = "Found: {} acts  |  {} expanded  |  {}ms{}".format(
            n_steps, st.get('expanded', '?'), st.get('elapsed_ms', '?'), tag_str)
        self.msg_color = (0, 150, 50)

    def _reset(self):
        self.planning = False
        self.animating = False
        self.finished = False
        self.path = []
        self.smooth_path = []
        self.rs_traj = []
        self.stage_split_idx = None
        self.anim_i = 0
        self.last_stats = {}
        self.sx, self.sy, self.sth = 2.8, 0.0, 0.0
        self.msg = "Reset"
        self.msg_color = COL_TEXT

    # ── drawing ────────────────────────────────────────────────
    def _draw(self):
        self.screen.fill(COL_BG)
        self._draw_canvas_bg()
        self._draw_grid()
        self._draw_corridor()
        self._draw_obstacles()
        self._draw_goal()
        self._draw_pallet()
        self._draw_labels()

        # （已移除 plannable zone 虚线限制）

        if len(self.path) > 1:
            self._draw_path()

        if self.animating and self.path:
            active = self.smooth_path if (self.use_smooth and self.smooth_path) else self.path
            end_frac = self.anim_i / max(len(self.path) - 1, 1)
            anim_idx = min(int(end_frac * (len(active) - 1)), len(active) - 1)
            fx, fy, ft = active[anim_idx]
            self._draw_forklift(fx, fy, ft)
        elif self.finished and self.path:
            active = self.smooth_path if (self.use_smooth and self.smooth_path) else self.path
            fx, fy, ft = active[-1]
            self._draw_forklift(fx, fy, ft)
        else:
            self._draw_forklift(self.sx, self.sy, self.sth)

        self._draw_ui()

    def _draw_canvas_bg(self):
        tl = self.trans.w2s(VIEW_X_MIN, VIEW_Y_MIN)
        br = self.trans.w2s(VIEW_X_MAX, VIEW_Y_MAX)
        rect = pygame.Rect(tl[0], tl[1], br[0] - tl[0], br[1] - tl[1])
        pygame.draw.rect(self.screen, COL_CANVAS_BG, rect)

    def _draw_grid(self):
        t = self.trans
        # Vertical lines (world Y values)
        y_start = math.ceil(VIEW_Y_MIN * 2) / 2
        wy = y_start
        while wy <= VIEW_Y_MAX:
            p1 = t.w2s(VIEW_X_MIN, wy)
            p2 = t.w2s(VIEW_X_MAX, wy)
            is_major = abs(wy - round(wy)) < 0.01
            col = COL_GRID_MAJOR if is_major else COL_GRID
            pygame.draw.line(self.screen, col, p1, p2, 1)
            # Tick label (Y axis, horizontal)
            if is_major:
                lbl = self.font.render(f"{wy:.0f}", True, COL_TEXT_DIM)
                self.screen.blit(lbl, (p2[0] - lbl.get_width() // 2, p2[1] + 4))
            wy += 0.5

        # Horizontal lines (world X values)
        x_start = math.ceil(VIEW_X_MIN * 5) / 5
        wx = x_start
        while wx <= VIEW_X_MAX:
            p1 = t.w2s(wx, VIEW_Y_MIN)
            p2 = t.w2s(wx, VIEW_Y_MAX)
            is_major = abs(wx * 10 - round(wx * 10)) < 0.01 and abs(wx * 2 - round(wx * 2)) < 0.01
            col = COL_GRID_MAJOR if is_major else COL_GRID
            pygame.draw.line(self.screen, col, p1, p2, 1)
            if is_major:
                lbl = self.font.render(f"{wx:.1f}", True, COL_TEXT_DIM)
                self.screen.blit(lbl, (p1[0] - lbl.get_width() - 6, p1[1] - lbl.get_height() // 2))
            wx += 0.2

    def _draw_obstacles(self):
        t = self.trans
        # Draw completed obstacles
        for obs in self.obstacles:
            p1 = t.w2s(obs['x'], obs['y'])
            p2 = t.w2s(obs['x'] + obs['w'], obs['y'] + obs['h'])
            rect = pygame.Rect(min(p1[0], p2[0]), min(p1[1], p2[1]),
                               abs(p2[0] - p1[0]), abs(p2[1] - p1[1]))
            # 绘制带有红色填充和粗边框的矩形障碍物
            pygame.draw.rect(self.screen, (200, 50, 50, 180), rect)
            pygame.draw.rect(self.screen, (150, 0, 0), rect, 2)
            
        # Draw in-progress obstacle
        if self.drawing_obs and self.obs_start_pt and self.obs_current_pt:
            p1 = t.w2s(*self.obs_start_pt)
            p2 = t.w2s(*self.obs_current_pt)
            rect = pygame.Rect(min(p1[0], p2[0]), min(p1[1], p2[1]),
                               abs(p2[0] - p1[0]), abs(p2[1] - p1[1]))
            # 半透明的红色表示正在绘制
            s = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)
            s.fill((200, 50, 50, 100))
            self.screen.blit(s, rect)
            pygame.draw.rect(self.screen, (255, 100, 100), rect, 2)

    def _draw_walls(self):
        t = self.trans
        # Fill outside workspace with wall color (semi-transparent overlay)
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)

        # Top wall (x < 1.92): reference can't go here
        tl = t.w2s(VIEW_X_MIN, VIEW_Y_MIN)
        br = t.w2s(WALL_X_MIN, VIEW_Y_MAX)
        pygame.draw.rect(overlay, (60, 60, 70, 50), (tl[0], tl[1], br[0]-tl[0], br[1]-tl[1]))

        # Bottom wall (x > 3.0)
        tl2 = t.w2s(WALL_X_MAX, VIEW_Y_MIN)
        br2 = t.w2s(VIEW_X_MAX, VIEW_Y_MAX)
        pygame.draw.rect(overlay, (60, 60, 70, 50), (tl2[0], tl2[1], br2[0]-tl2[0], br2[1]-tl2[1]))

        # Left wall (y < -3.0)
        tl3 = t.w2s(VIEW_X_MIN, VIEW_Y_MIN)
        br3 = t.w2s(VIEW_X_MAX, WALL_Y_MIN)
        pygame.draw.rect(overlay, (60, 60, 70, 50), (tl3[0], tl3[1], br3[0]-tl3[0], br3[1]-tl3[1]))

        # Right wall (y > 3.0)
        tl4 = t.w2s(VIEW_X_MIN, WALL_Y_MAX)
        br4 = t.w2s(VIEW_X_MAX, VIEW_Y_MAX)
        pygame.draw.rect(overlay, (60, 60, 70, 50), (tl4[0], tl4[1], br4[0]-tl4[0], br4[1]-tl4[1]))

        self.screen.blit(overlay, (0, 0))

        # Workspace boundary lines
        corners = [
            t.w2s(WALL_X_MIN, WALL_Y_MIN),
            t.w2s(WALL_X_MIN, WALL_Y_MAX),
            t.w2s(WALL_X_MAX, WALL_Y_MAX),
            t.w2s(WALL_X_MAX, WALL_Y_MIN),
        ]
        pygame.draw.lines(self.screen, COL_WALL_LINE, True, corners, 2)

    def _draw_corridor(self):
        t = self.trans
        # Safe corridor funnel: when x <= 2.05, tip_lat must be within sc
        # sc = 0.15 + (x - 1.87) * 0.8  if x > 1.87 else 0.15
        # Draw as a filled polygon. Left anchor keeps historical corridor shape.
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        pts_top = []
        pts_bot = []
        N = 30
        for i in range(N + 1):
            wx = WALL_X_MIN + (2.05 - WALL_X_MIN) * (i / N)
            sc = (0.15 + (wx - 1.87) * 0.8) if wx > 1.87 else 0.15
            pts_top.append(t.w2s(wx, sc))
            pts_bot.append(t.w2s(wx, -sc))

        poly = pts_top + list(reversed(pts_bot))
        if len(poly) > 2:
            pygame.draw.polygon(overlay, COL_CORRIDOR, poly)
        self.screen.blit(overlay, (0, 0))

    def _draw_plannable_zone(self):
        """绘制可规划范围的横向边界（±MAX_PLANNABLE_Y 虚线）"""
        t = self.trans
        limit = main.MAX_PLANNABLE_Y
        for sign, label_txt in [(+1, f"+{limit:.1f}m"), (-1, f"-{limit:.1f}m")]:
            y_world = sign * limit
            p1 = t.w2s(VIEW_X_MIN, y_world)
            p2 = t.w2s(VIEW_X_MAX, y_world)
            # 虚线：每 8 像素画 5 像素
            dash, gap = 6, 4
            x = p1[0]
            col = (220, 100, 100, 140)
            while x < p2[0]:
                x1 = int(x)
                x2 = min(int(x + dash), p2[0])
                pygame.draw.line(self.screen, (220, 100, 100), (x1, p1[1]), (x2, p1[1]), 1)
                x += dash + gap
            # 标签
            lbl = self.font.render(f"limit {label_txt}", True, (200, 80, 80))
            self.screen.blit(lbl, (p2[0] - lbl.get_width() - 4, p1[1] - 14))

    def _draw_goal(self):
        t = self.trans
        overlay = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        tl = t.w2s(WALL_X_MIN, -GOAL_Y_HALF)
        br = t.w2s(GOAL_X_MAX, GOAL_Y_HALF)
        w = br[0] - tl[0]
        h = br[1] - tl[1]
        pygame.draw.rect(overlay, COL_GOAL, (tl[0], tl[1], w, h))
        self.screen.blit(overlay, (0, 0))
        pygame.draw.rect(self.screen, (60, 180, 100), (tl[0], tl[1], w, h), 1)

    def _draw_pallet(self):
        t = self.trans
        # Pallet visualization at x ≈ 1.5 to 1.92 near y=0
        p_w = 0.4
        p_h = 0.8
        tl = t.w2s(WALL_X_MIN - p_w, -p_h / 2)
        br = t.w2s(WALL_X_MIN, p_h / 2)
        w = br[0] - tl[0]
        h = br[1] - tl[1]
        pygame.draw.rect(self.screen, COL_PALLET, (tl[0], tl[1], w, h))
        pygame.draw.rect(self.screen, (140, 100, 50), (tl[0], tl[1], w, h), 2)
        # Slats
        for i in range(1, 4):
            frac = i / 4
            sx_line = tl[0] + int(w * frac)
            pygame.draw.line(self.screen, (140, 100, 50), (sx_line, tl[1]), (sx_line, tl[1] + h), 1)

    def _draw_labels(self):
        t = self.trans

        def blit_label(text, wx, wy, color=(60, 60, 70)):
            sx, sy = t.w2s(wx, wy)
            surf = self.label_font.render(text, True, color)
            bg = pygame.Surface((surf.get_width() + 6, surf.get_height() + 2), pygame.SRCALPHA)
            bg.fill((255, 255, 255, 180))
            self.screen.blit(bg, (sx - surf.get_width() // 2 - 3, sy - surf.get_height() // 2 - 1))
            self.screen.blit(surf, (sx - surf.get_width() // 2, sy - surf.get_height() // 2))

        blit_label("PALLET", WALL_X_MIN - 0.2, 0.0, (140, 100, 50))
        blit_label("GOAL", (WALL_X_MIN + GOAL_X_MAX) / 2, GOAL_Y_HALF + 0.15, (40, 150, 80))
        blit_label("CORRIDOR", 1.98, 0.25, (180, 160, 40))

        # Axis labels
        x_lbl = self.font.render("x (dist to pallet) -->", True, COL_TEXT_DIM)
        x_lbl_rot = pygame.transform.rotate(x_lbl, 90)
        self.screen.blit(x_lbl_rot, (8, CANVAS_TOP + 30))

        y_lbl = self.font.render("<-- y (lateral) -->", True, COL_TEXT_DIM)
        bottom_y = CANVAS_BOTTOM + 22
        self.screen.blit(y_lbl, (SCREEN_WIDTH // 2 - y_lbl.get_width() // 2, bottom_y))

    def _draw_path(self):
        tr = self.trans
        if len(self.path) < 2:
            return

        active_path = self.smooth_path if (self.use_smooth and self.smooth_path) else self.path

        # ── 始终绘制原始路径（细蓝色）──────────────────────────────
        raw_pts = [tr.w2s(p[0], p[1]) for p in self.path]
        pygame.draw.lines(self.screen, COL_PATH_RAW, False, raw_pts, 1)

        # ── 两阶段分界点标记（黄色圆圈）────────────────────────────
        if (self.stage_split_idx is not None
                and 0 < self.stage_split_idx < len(self.path)):
            mp = self.path[self.stage_split_idx]
            msx, msy = tr.w2s(mp[0], mp[1])
            pygame.draw.circle(self.screen, (240, 200, 0), (msx, msy), 7, 2)
            lbl = self.font.render("S2", True, (200, 160, 0))
            self.screen.blit(lbl, (msx + 8, msy - 8))

        # ── RS 解析扩展尾段（紫色标注）─────────────────────────────
        if self.rs_traj and len(self.rs_traj) > 1:
            rs_pts = [tr.w2s(p[0], p[1]) for p in self.rs_traj]
            pygame.draw.lines(self.screen, COL_PATH_RS, False, rs_pts, 3)

        # ── 平滑轨迹（橙色，仅在 P 键开启时绘制）──────────────────
        if self.use_smooth and self.smooth_path and len(self.smooth_path) > 1:
            sm_pts = [tr.w2s(p[0], p[1]) for p in self.smooth_path]
            pygame.draw.lines(self.screen, COL_PATH_SMOOTH, False, sm_pts, 3)

        # ── 动画前进段（加粗显示已走过部分）───────────────────────
        if self.animating or self.finished:
            end_frac = self.anim_i / max(len(self.path) - 1, 1)
            end_idx = int(end_frac * (len(active_path) - 1)) + 1 if self.animating else len(active_path)
            if end_idx > 1:
                sub = [tr.w2s(active_path[i][0], active_path[i][1]) for i in range(end_idx)]
                col = COL_PATH_SMOOTH if (self.use_smooth and self.smooth_path) else COL_PATH
                pygame.draw.lines(self.screen, col, False, sub, 3)

    def _draw_forklift(self, x, y, theta):
        t = self.trans
        is_valid, reason = main.check_collision(x, y, theta)
        body_col = COL_BODY_OK if is_valid else COL_BODY_BAD

        ct, st = math.cos(theta), math.sin(theta)

        def local_to_screen(lx, ly):
            wx = x + lx * ct - ly * st
            wy = y + lx * st + ly * ct
            return t.w2s(wx, wy)

        half_w = primitives.VEHICLE_HALF_WIDTH
        offsets = primitives.VEHICLE_CHECK_OFFSETS

        # ── 绘制多圆碰撞模型（与实际碰撞检测一致）──
        for offset in offsets:
            # 圆心世界坐标（forward = -x）
            cx = x - offset * ct
            cy = y - offset * st
            csx, csy = t.w2s(cx, cy)
            r_px = t.m2px(half_w)
            # 填充圆
            surf = pygame.Surface((r_px * 2, r_px * 2), pygame.SRCALPHA)
            col_a = (*body_col, 60)
            pygame.draw.circle(surf, col_a, (r_px, r_px), r_px)
            self.screen.blit(surf, (csx - r_px, csy - r_px))
            # 圆边框
            pygame.draw.circle(self.screen, body_col, (csx, csy), r_px, 2)

        # ── 车身中心线（最后→最前）──
        rear_offset = max(offsets)
        front_offset = min(offsets)
        rear_wx = x - rear_offset * ct
        rear_wy = y - rear_offset * st
        front_wx = x - front_offset * ct
        front_wy = y - front_offset * st
        pygame.draw.line(self.screen, body_col,
                         t.w2s(rear_wx, rear_wy), t.w2s(front_wx, front_wy), 2)

        # ── 叉齿标记（最前方圆心处）──
        fork_sx, fork_sy = t.w2s(front_wx, front_wy)
        pygame.draw.circle(self.screen, COL_FORK_TIP, (fork_sx, fork_sy), 4)

        # Reference point
        ref_sx, ref_sy = t.w2s(x, y)
        pygame.draw.circle(self.screen, (255, 255, 255), (ref_sx, ref_sy), 5)
        pygame.draw.circle(self.screen, (30, 30, 40), (ref_sx, ref_sy), 5, 2)

        # Heading indicator (small arrow in +x direction, i.e. away from pallet)
        arr_end = local_to_screen(0.25, 0)
        pygame.draw.line(self.screen, (30, 30, 40), (ref_sx, ref_sy), arr_end, 2)

        # Invalid warning
        if not is_valid:
            lbl = self.label_font.render(f"INVALID: {reason}", True, COL_BODY_BAD)
            self.screen.blit(lbl, (ref_sx + 12, ref_sy - 8))

    def _draw_ui(self):
        bar_y = SCREEN_HEIGHT - 65
        pygame.draw.rect(self.screen, (235, 235, 240), (0, bar_y, SCREEN_WIDTH, 65))
        pygame.draw.line(self.screen, (200, 200, 210), (0, bar_y), (SCREEN_WIDTH, bar_y), 1)

        is_valid, _ = main.check_collision(self.sx, self.sy, self.sth)
        status_dot = (80, 200, 120) if is_valid else (220, 60, 60)

        # ── Left column: position + status ───────────────────────────
        pygame.draw.circle(self.screen, status_dot, (25, bar_y + 18), 6)
        info = "Ref: ({:.2f}, {:.2f})  Theta: {:.1f} deg".format(
            self.sx, self.sy, math.degrees(self.sth))
        self.screen.blit(self.big_font.render(info, True, COL_TEXT), (40, bar_y + 8))

        if self.msg:
            self.screen.blit(self.title_font.render(self.msg, True, self.msg_color),
                             (40, bar_y + 30))

        # ── Centre: mode badges ───────────────────────────────────────
        mode_names = ["Geometric", "RS Heuristic", "Pure RS"]
        heur_label = mode_names[self.planning_mode]
        if self.planning_mode == 0:
            heur_col = (100, 100, 110)
        elif self.planning_mode == 1:
            heur_col = (30, 120, 200)
        else:
            heur_col = (140, 60, 200)
            
        corr_label = "No Corridor" if self.no_corridor else "Corridor ON"
        corr_col   = (200, 100, 30) if self.no_corridor else (80, 150, 80)

        def badge(text, color, x, y):
            surf = self.label_font.render(text, True, (255, 255, 255))
            pad = 6
            bg = pygame.Surface((surf.get_width() + pad * 2, surf.get_height() + 4),
                                 pygame.SRCALPHA)
            bg.fill((*color, 220))
            self.screen.blit(bg, (x, y))
            self.screen.blit(surf, (x + pad, y + 2))
            return x + bg.get_width() + 6

        smooth_label = "Smooth ON" if self.use_smooth else "Smooth OFF"
        smooth_col   = (200, 120, 30) if self.use_smooth else (140, 140, 150)

        cx_badge = SCREEN_WIDTH // 2 - 220
        cx_badge = badge(heur_label, heur_col, cx_badge, bar_y + 8)
        if self.planning_mode < 2:
            cx_badge = badge(f"R={self.rs_radius:.1f}m", (100, 100, 150), cx_badge, bar_y + 8)
        cx_badge = badge(corr_label, corr_col, cx_badge, bar_y + 8)
        cx_badge = badge(smooth_label, smooth_col, cx_badge, bar_y + 8)

        if self.rs_traj:
            cx_badge = badge("RS Expand", (140, 60, 200), cx_badge, bar_y + 8)
        if self.last_stats.get('two_stage'):
            badge("2-Stage", (30, 150, 130), cx_badge, bar_y + 8)

        # Last stats row
        if self.last_stats:
            st = self.last_stats
            stat_str = "expanded: {}  time: {}ms".format(
                st.get('expanded', '—'), st.get('elapsed_ms', '—'))
            ss = self.font.render(stat_str, True, COL_TEXT_DIM)
            self.screen.blit(ss, (SCREEN_WIDTH // 2 - ss.get_width() // 2, bar_y + 34))

        # ── Right column: key help ────────────────────────────────────
        lines = [
            "W/S/A/D=Move  Q/E=Rotate  SPACE=Plan  R=Reset  X=ClearObs",
            "M=Mode  +/-=Radius  C=Corridor  P=Smooth  Click=Pos  Shift+Drag=Obs",
        ]
        for i, line in enumerate(lines):
            s = self.font.render(line, True, COL_TEXT_DIM)
            self.screen.blit(s, (SCREEN_WIDTH - s.get_width() - 15, bar_y + 8 + i * 18))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Forklift Path Planning Visualizer")
    parser.add_argument("--vehicle-length", type=float, default=1.5,
                        help="Vehicle total length in meters (default: 1.5)")
    parser.add_argument("--vehicle-width", type=float, default=0.5,
                        help="Vehicle total width in meters (default: 0.5)")
    args = parser.parse_args()

    # 根据用户参数配置车辆碰撞模型
    primitives.configure_vehicle(args.vehicle_length, args.vehicle_width)
    print(f"Vehicle config: {args.vehicle_length}m x {args.vehicle_width}m "
          f"-> {len(primitives.VEHICLE_CHECK_OFFSETS)} circles, "
          f"r={primitives.VEHICLE_HALF_WIDTH:.2f}m, "
          f"offsets={primitives.VEHICLE_CHECK_OFFSETS}")

    App().run()
