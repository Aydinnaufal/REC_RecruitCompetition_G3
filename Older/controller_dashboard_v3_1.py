"""
controller_dashboard_v3_1.py - dashboard v3.1: v3 with new controls.

    pip install pygame
    python controller_dashboard_v3_1.py            # real controller
    python controller_dashboard_v3_1.py --demo     # fake input, no controller needed

Controller (Arcade and Arcade + Snap):
    left stick Y     forward / back                  left stick X     turn, ONLY while LT is held
    RT               speed: not pressed = 20 cm/s max, fully pressed = 30 cm/s
    D-pad            slow nudge: up / down = forward / back (5 cm/s), left / right = slight turn
    right stick X    gripper: right = close (how far you push = how far it closes), centre or left = open / release
    right stick Y    tower only: box up / down (how far = how fast)
    LB / RB          snap-turn 90 left / right (Arcade + Snap)       Y / B / A / X   face up / right / down / left of the zero
    START            make the current heading the new zero
Tank mode: left Y / right Y = left / right wheel; the tower box then moves with LB (down) / RB (up).
Mouse: drag in the picture to look around, scroll to zoom. Keys (all also on-screen): F11 / F fullscreen, M learn map, S save map,
R default map, D demo, V next view, T next drive mode, 0 re-zero, C reset cubes, X reset robot, Z reset all, ESC leaves fullscreen / quits.
"""
import sys, time
import pygame
import xbox_input as xi
import controller_viewer as cv
import controller_dashboard as cd
import dash_ui as ui
from controller_viewer import text, BG, PANEL, TEXT, MUTED, ACCENT
from robot_sim import Camera, MAX_SPEED
import robot_sim3 as r3
import math
from robot_sim3 import RobotSim3, MODES, MODE_LABEL, ROBOTS, ROBOT_LABEL, TEAMS, VIEWS3, MON_VIEWS, VIEW_LABEL, draw_scene3, draw_monitor3

CW, CH = 1360, 800                               # the design canvas
M, G = 12, 12
TOP, BOT = 106, 770
LW, MW, RW = 400, 560, 352
LX0 = M
MX0 = LX0 + LW + G
RX0 = MX0 + MW + G
SCENE_H3 = 410
WARN = (240, 150, 90)
GAME, MONITOR = 'game', 'monitor'


class App:
    """Everything the screen shows: the simulator, which mode, which views. The pad drives the simulator; in monitor mode the same
    simulator stands in for the real robot until a data source is plugged in with set_feed()."""

    def __init__(self, robot='simple', team='blue'):
        self.sim = RobotSim3(robot, team)
        self.mode = GAME
        self.view = {GAME: 'arena', MONITOR: 'iso'}
        self.demo = False
        self.fullscreen = False
        self.look = r3.Look()                    # mouse camera + trail / shadow / arrow settings
        self.feed = None                         # later: a function returning a snapshot() style dict from the real robot

    # ---- mode / view
    def views(self): return VIEWS3 if self.mode == GAME else MON_VIEWS

    def set_mode(self, m):
        if m == self.mode:
            return
        self.mode = m
        self.sim.set_arena(m == GAME)            # monitor mode: no arena, no cubes, no collisions
        self.sim.reset_robot() if m == MONITOR else None

    def cur_view(self): return self.view[self.mode]

    def set_view(self, v): self.view[self.mode] = v

    def scene_rect(self): return (MX0 + 10, TOP + 36, MW - 20, SCENE_H3)

    def in_scene(self, pos):
        x, y, w, h = self.scene_rect()
        return pos is not None and x <= pos[0] < x + w and y <= pos[1] < y + h

    def drag(self, dx, dy):
        """Dragging in the picture gives the free camera (it starts from the default angle, then follows the mouse)."""
        if self.cur_view() != 'free':
            self.set_view('free')
        self.look.drag(dx, dy)

    def zoom(self, n): self.look.wheel(n)

    def reset_cam(self): self.look.reset_cam()

    def next_view(self):
        v = self.views()
        self.set_view(v[(v.index(self.cur_view()) + 1) % len(v)])

    def next_drive_mode(self): self.sim.mode = MODES[(MODES.index(self.sim.mode) + 1) % len(MODES)]

    def set_feed(self, fn): self.feed = fn

    def state(self):
        """The robot picture's data. A real feed (monitor mode) replaces the simulator."""
        if self.mode == MONITOR and self.feed is not None:
            return self.feed()
        return self.sim.snapshot()


def build_ui(app, actions):
    """actions: dict name -> callable for the buttons that need the window (fullscreen, learn, save, default map)."""
    u = ui.UI()
    sim = app.sim
    x = 330
    dd = lambda w, title, opts, get, set_, **k: u.add(ui.Dropdown((x, 36, w, 30), title, opts, get, set_, **k))
    app.dd_mode = dd(150, 'MODE', [(GAME, 'Game (arena)'), (MONITOR, 'Monitor')], lambda: app.mode, app.set_mode); x += 162
    app.dd_robot = dd(190, 'ROBOT', [(k, ROBOT_LABEL[k]) for k in ROBOTS], lambda: sim.robot_type, sim.set_robot); x += 202
    app.dd_drive = dd(160, 'DRIVE', [(k, MODE_LABEL[k]) for k in MODES], lambda: sim.mode, lambda v: setattr(sim, 'mode', v)); x += 172
    app.dd_team = dd(110, 'TEAM', [('blue', 'Blue'), ('red', 'Red')], lambda: sim.team, sim.set_team, enabled=lambda: app.mode == GAME); x += 122
    app.dd_view = dd(170, 'VIEW', [(v, VIEW_LABEL[v]) for v in VIEWS3], app.cur_view, app.set_view); x += 182
    x = M
    btn = lambda w, label, fn, **k: (u.add(ui.Button((x, 74, w, 26), label, fn, **k)), None)[0]
    for w, label, fn, k in ((112, 'Re-zero (START)', sim.zero, {}),
                            (116, 'Reset cubes', sim.reset_cubes, dict(enabled=lambda: app.mode == GAME)),
                            (116, 'Reset robot', sim.reset_robot, {}),
                            (100, 'Reset all', sim.reset, {}),
                            (84, 'Demo', actions['demo'], dict(active=lambda: app.demo)),
                            (96, 'Learn map', actions['learn'], {}),
                            (92, 'Save map', actions['save'], {}),
                            (104, 'Default map', actions['default'], {}),
                            (104, 'Fullscreen', actions['fullscreen'], dict(active=lambda: app.fullscreen)),
                            (96, 'Reset camera', app.reset_cam, {})):
        btn(w, label, fn, **k)
        x += w + 8
    L = app.look
    y0 = BOT - 36
    for i, (lab, on, setn, val, setv) in enumerate((('Trail', lambda: L.trail_on, lambda v: setattr(L, 'trail_on', v), lambda: L.trail_i, lambda v: setattr(L, 'trail_i', v)),
                                                    ('Shadow', lambda: L.shadow_on, lambda v: setattr(L, 'shadow_on', v), lambda: L.shadow_i, lambda v: setattr(L, 'shadow_i', v)),
                                                    ('Arrow', lambda: L.arrow_on, lambda v: setattr(L, 'arrow_on', v), lambda: L.arrow_i, lambda v: setattr(L, 'arrow_i', v)))):
        u.add(ui.Stepper((MX0 + 14 + i * 180, y0, 160, 24), lab, on, setn, val, setv))
    return u


def refresh_view_options(app):
    app.dd_view.options[:] = [(v, VIEW_LABEL[v]) for v in app.views()]


def bar(screen, fonts, x, y, w, frac, label, value, color=ACCENT, centre=False):
    text(screen, fonts['s'], label, (x, y + 1), TEXT)
    bx = x + 78
    pygame.draw.rect(screen, (48, 53, 60), (bx, y + 2, w, 12), border_radius=4)
    if centre:
        mid = bx + w // 2
        pygame.draw.rect(screen, color if frac >= 0 else WARN, (min(mid, mid + int(frac * w / 2)), y + 2, abs(int(frac * w / 2)), 12), border_radius=4)
        pygame.draw.line(screen, (96, 104, 115), (mid, y), (mid, y + 16), 1)
    else:
        pygame.draw.rect(screen, color, (bx, y + 2, int(max(0.0, min(1.0, frac)) * w), 12), border_radius=4)
    text(screen, fonts['s'], value, (bx + w + 8, y + 1), TEXT)


def draw_left(screen, fonts, big, st, learn_prompt, note, log_lines):
    pygame.draw.rect(screen, PANEL, (LX0, TOP, LW, BOT - TOP), border_radius=8)
    text(screen, fonts['m'], 'CONTROLLER', (LX0 + 14, TOP + 10), MUTED)
    img = pygame.Surface((cd.CTRL_W, cd.CTRL_H))
    img.fill(PANEL)
    cv.draw_controller(img, big, 0, 0, st)
    scaled = pygame.transform.smoothscale(img, (LW - 20, int((LW - 20) * cd.CTRL_H / cd.CTRL_W)))
    screen.blit(scaled, (LX0 + 10, TOP + 34))
    y = TOP + 34 + scaled.get_height() + 10
    if learn_prompt:
        pygame.draw.rect(screen, ACCENT, (LX0 + 12, y, LW - 24, 56), border_radius=8)
        text(screen, fonts['m'], learn_prompt[:40], (LX0 + 24, y + 12), (8, 24, 22))
        if len(learn_prompt) > 40: text(screen, fonts['s'], learn_prompt[40:], (LX0 + 24, y + 34), (8, 24, 22))
    elif note:
        text(screen, fonts['m'], note, (LX0 + 14, y + 4), ACCENT)
    else:
        text(screen, fonts['s'], 'Learn map = teach it your controller', (LX0 + 14, y + 4), MUTED)
    ly = y + 64
    cv.draw_log(screen, fonts, LX0 + 14, ly + 12, log_lines, w=LW - 4, h=BOT - ly - 4, n=max(3, (BOT - ly - 50) // 18))


def draw_middle(screen, fonts, cam, scene, app):
    sim, d = app.sim, app.state()
    pygame.draw.rect(screen, PANEL, (MX0, TOP, MW, BOT - TOP), border_radius=8)
    title = 'GAME  -  %s  -  %s team' % (ROBOT_LABEL[d['type']], sim.team) if app.mode == GAME else 'MONITOR  -  %s  (no arena)' % ROBOT_LABEL[d['type']]
    text(screen, fonts['m'], title, (MX0 + 14, TOP + 10), ACCENT if app.mode == MONITOR else MUTED)
    text(screen, fonts['s'], 'view: %s' % VIEW_LABEL[app.cur_view()], (MX0 + MW - 14, TOP + 12), TEXT, 'tr')
    if app.mode == GAME:
        draw_scene3(scene, cam, sim, app.cur_view(), app.look)
    else:
        draw_monitor3(scene, cam, d, app.cur_view(), app.look)
    screen.blit(scene, (MX0 + 10, TOP + 36))
    y = TOP + 36 + scene.get_height() + 8
    x, w = MX0 + 14, 220
    bar(screen, fonts, x, y, w, d['vl'], 'L wheel', '%+4.0f cm/s' % (d['vl'] * MAX_SPEED), centre=True)
    bar(screen, fonts, x, y + 20, w, d['vr'], 'R wheel', '%+4.0f cm/s' % (d['vr'] * MAX_SPEED), centre=True)
    g = sim.grip
    if d['type'] == 'simple':
        state = ('HOLDING cube %d' % g.held.id) if (g.held and app.mode == GAME) else ('holding' if d['holding'] else ('touching' if any(g.contact.values()) and app.mode == GAME else 'free'))
        bar(screen, fonts, x, y + 40, w, d['cmd'], 'R-stk claw', '%3.0f%%  %s' % (d['cmd'] * 100, state), color=(238, 168, 58))
    else:
        import sim_tower as tw
        bar(screen, fonts, x, y + 40, w, (d['lift'] - tw.LIFT_MIN) / (tw.LIFT_MAX - tw.LIFT_MIN), 'Box lift', '%4.1f cm  (R stick Y)' % d['lift'], color=(60, 200, 190))
        state = ('HOLDING cube %d' % g.held.id) if (g.held and app.mode == GAME) else ('holding' if d['holding'] else ('touching' if g.contact and app.mode == GAME else 'free'))
        bar(screen, fonts, x, y + 60, w, d['cmd'], 'R-stk pad', '%3.0f%%  %s' % (d['cmd'] * 100, state), color=(238, 168, 58))
    yy = y + (60 if d['type'] == 'simple' else 80)
    bar(screen, fonts, x, yy, w, sim.rt, 'RT speed', '%3.0f cm/s cap' % (sim.speed_scale * MAX_SPEED), color=(90, 170, 240))
    snap = ('turning to %.0f deg' % ((-(sim.snap_target - sim.ref_th) * 180 / 3.141592653589793) % 360)) if sim.snapping else 'idle'
    text(screen, fonts['s'], 'heading %3.0f deg from zero   zero set %d time%s' % (d['heading'], sim.zero_count, '' if sim.zero_count == 1 else 's'), (MX0 + 14, yy + 28), TEXT)
    text(screen, fonts['s'], 'speed %+5.1f cm/s   x %+6.1f   y %+6.1f   snap: %s' % (d['speed'], d['x'], d['y'], snap if sim.mode == 'arcade_snap' else 'off'),
         (MX0 + 14, yy + 46), MUTED)
    text(screen, fonts['s'], 'Drag the picture to look around, scroll to zoom.  LT + left stick X = turn.', (MX0 + 14, yy + 64), MUTED)


def render_dashboard3(canvas, fonts, big, cam, scene, app, u, st, mapping, log_lines, learn_prompt='', note=''):
    """One full frame onto the fixed-size canvas (separate from the event loop so it can be tested without a window)."""
    canvas.fill(BG)
    text(canvas, fonts['l'], 'Robot control', (M + 4, 10))
    text(canvas, fonts['m'], 'dashboard v3.1', (M + 4, 40), MUTED)
    status = st.name if st.connected else 'NO CONTROLLER FOUND'
    text(canvas, fonts['s'], status[:32], (CW - M - 4, 80), ACCENT if st.connected else (240, 120, 100), 'tr')
    text(canvas, fonts['s'], 'map: %s' % mapping.get('name', '?'), (CW - M - 4, 62), MUTED, 'tr')
    draw_left(canvas, fonts, big, st, learn_prompt, note, log_lines)
    draw_middle(canvas, fonts, cam, scene, app)
    cv.draw_raw(canvas, fonts, RX0 + 14, TOP + 12, st, mapping, w=RW, h=BOT - TOP, cell=60)
    text(canvas, fonts['s'], 'F11 fullscreen   V view   T drive mode   0 re-zero   mouse: drag look, scroll zoom   C cubes   X robot   Z all   M/S/R map   D demo   ESC quit',
         (M + 4, CH - 22), MUTED)
    u.draw(canvas, fonts)


def make_window(fullscreen, size):
    if fullscreen:
        return pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    return pygame.display.set_mode(size, pygame.RESIZABLE)


def main():
    app = App()
    app.demo = '--demo' in sys.argv
    pygame.init()
    win_size = (1360, 800)
    window = make_window(False, win_size)
    pygame.display.set_caption('Robot control dashboard v3.1')
    fonts, big = cd.make_fonts()
    canvas = pygame.Surface((CW, CH))
    pad = xi.XboxController()
    elog, note, note_t = cv.EventLog(), '', 0
    box = {'learner': None}
    drag = {'on': False}
    sw, sh = MW - 20, SCENE_H3
    scene, cam = pygame.Surface((sw, sh)), Camera(sw, sh, 400)

    def say(s):
        nonlocal note, note_t
        note, note_t = s, time.time()

    def toggle_full():
        nonlocal window, win_size
        if not app.fullscreen:
            win_size = window.get_size()
        app.fullscreen = not app.fullscreen
        window = make_window(app.fullscreen, win_size)

    def learn(): box['learner'] = xi.Learner(); say('')
    def save(): xi.save_mapping(pad.mapping); say('Saved to ' + xi.MAPPING_FILE)
    def default(): pad.mapping = xi.default_mapping(); box['learner'] = None; say('Default mapping restored')
    def demo(): app.demo = not app.demo

    u = build_ui(app, dict(demo=demo, learn=learn, save=save, default=default, fullscreen=toggle_full))
    sim = app.sim
    clock, t0 = pygame.time.Clock(), time.time()
    while True:
        dt = min(clock.tick(60) / 1000.0, 0.05)
        for e in pygame.event.get():
            if e.type == pygame.QUIT: return
            if e.type == pygame.VIDEORESIZE and not app.fullscreen:
                win_size = (max(480, e.w), max(300, e.h))
                window = pygame.display.set_mode(win_size, pygame.RESIZABLE)
            elif e.type == pygame.MOUSEMOTION:
                cp = ui.to_canvas(e.pos, window.get_size(), CW, CH)
                u.motion(cp)
                if drag['on']:
                    k = ui.fit_scale(window.get_width(), window.get_height(), CW, CH)[0]
                    app.drag(e.rel[0] / k, e.rel[1] / k)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                cp = ui.to_canvas(e.pos, window.get_size(), CW, CH)
                if not u.click(cp) and app.in_scene(cp):
                    drag['on'] = True
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                drag['on'] = False
            elif e.type == pygame.MOUSEWHEEL:
                if app.in_scene(ui.to_canvas(pygame.mouse.get_pos(), window.get_size(), CW, CH)):
                    app.zoom(e.y)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button in (4, 5):          # old pygame: wheel as buttons
                if app.in_scene(ui.to_canvas(e.pos, window.get_size(), CW, CH)):
                    app.zoom(1 if e.button == 4 else -1)
            elif e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE:
                    if any(d.open for d in u.dropdowns): u.close_all()
                    elif app.fullscreen: toggle_full()
                    else: return
                elif e.key in (pygame.K_F11, pygame.K_f): toggle_full()
                elif e.key == pygame.K_d: demo()
                elif e.key == pygame.K_m: learn()
                elif e.key == pygame.K_SPACE and box['learner'] and not box['learner'].done: box['learner'].skip()
                elif e.key == pygame.K_s: save()
                elif e.key == pygame.K_r: default()
                elif e.key == pygame.K_v: app.next_view()
                elif e.key == pygame.K_t: app.next_drive_mode()
                elif e.key == pygame.K_0: sim.zero()
                elif e.key == pygame.K_z: sim.reset()
                elif e.key == pygame.K_x: sim.reset_robot()
                elif e.key == pygame.K_c and app.mode == GAME: sim.reset_cubes()
        refresh_view_options(app)
        if app.demo:
            axes, buttons, hats = cv.demo_raw(time.time() - t0)
            st = xi.apply_mapping(pad.mapping, axes, buttons, hats, 'DEMO (fake input)')
        else:
            st = pad.poll()
        learner = box['learner']
        if learner:
            done = learner.update(st.raw_axes, st.raw_buttons, st.raw_hats) if st.connected else None
            if done is not None and learner.done:
                pad.mapping = learner.m; box['learner'] = None; say('Learned. Press S to save it.')
        else:
            sim.drive(st, dt)
        elog.update(st)
        render_dashboard3(canvas, fonts, big, cam, scene, app, u, st, pad.mapping, elog.lines,
                          learner.prompt if learner else '', note if (note and time.time() - note_t < 6) else '')
        ui.present(window, canvas)
        pygame.display.flip()


if __name__ == '__main__':
    main()
