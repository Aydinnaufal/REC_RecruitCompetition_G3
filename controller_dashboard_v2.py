"""
controller_dashboard_v2.py - the alternate dashboard: same three panels as controller_dashboard.py, with the new simulator
(cubes on pillars, a claw on LT, speed on RT, and the arcade + snap mode).

    pip install pygame
    python controller_dashboard_v2.py            # real controller
    python controller_dashboard_v2.py --demo     # fake input, no controller needed

Controller (Xbox 360 names):
    left stick     drive and steer            LT   claw, 0 open .. 1 closed (both arms on a cube = grab)
    RT             speed, 0 = 80% .. 1 = full   (only in 'arcade_snap' mode:)
    LB / RB        snap-turn 90 deg left / right, once per press
    Y / B / A / X  face up / right / down / left of the map (start orientation), shortest turn
Keys: M learn mapping   S save mapping   R default mapping   D demo   V view   T mode (arcade / arcade_snap / tank)
      Z reset robot and cubes   C reset the cubes only   F11 / F fullscreen   ESC leave fullscreen, else quit
The window can be resized freely; the dashboard is drawn at W x H and scaled to fit (letterboxed, aspect kept).
The original controller_dashboard.py is untouched and still works.
"""
import sys, time
import pygame
import xbox_input as xi
import controller_viewer as cv
import controller_dashboard as cd
import sim_physics as ph
from controller_viewer import text, BG, PANEL, TEXT, MUTED, ACCENT
from robot_sim import Camera, VIEWS, MAX_SPEED
from robot_sim2 import RobotSim2, MODES, draw_scene2, SPEED_MIN

W, H, M = cd.W, cd.H, cd.M
MW, RW = cd.MW, cd.RW
TOP, BOT = cd.TOP, cd.BOT
MX0, RX0 = cd.MX0, cd.RX0
SCENE_H2 = 430                                   # a little shorter than the original so the new gauges fit under it
WARN = (240, 150, 90)


def bar(screen, fonts, x, y, w, frac, label, value, color=ACCENT, centre=False):
    """A labelled gauge. centre=True: zero in the middle, negative values grow to the left (wheel speeds)."""
    text(screen, fonts['s'], label, (x, y + 1), TEXT)
    bx = x + 68
    pygame.draw.rect(screen, (48, 53, 60), (bx, y + 2, w, 12), border_radius=4)
    if centre:
        mid = bx + w // 2
        pygame.draw.rect(screen, color if frac >= 0 else WARN, (min(mid, mid + int(frac * w / 2)), y + 2, abs(int(frac * w / 2)), 12), border_radius=4)
        pygame.draw.line(screen, (96, 104, 115), (mid, y), (mid, y + 16), 1)
    else:
        pygame.draw.rect(screen, color, (bx, y + 2, int(max(0.0, min(1.0, frac)) * w), 12), border_radius=4)
    text(screen, fonts['s'], value, (bx + w + 8, y + 1), TEXT)


def mode_label(sim):
    return {'arcade': 'arcade', 'arcade_snap': 'arcade + snap', 'tank': 'tank'}[sim.mode]


def draw_sim_panel2(screen, fonts, cam, scene, sim, view):
    pygame.draw.rect(screen, PANEL, (MX0, TOP, MW, BOT - TOP), border_radius=8)
    text(screen, fonts['m'], 'SIMULATOR v2', (MX0 + 14, TOP + 10), MUTED)
    text(screen, fonts['s'], 'view: %s (V)   mode: %s (T)' % (view, mode_label(sim)), (MX0 + MW - 14, TOP + 12), TEXT, 'tr')
    draw_scene2(scene, cam, sim, view)
    screen.blit(scene, (MX0 + 10, TOP + 36))
    y = TOP + 36 + scene.get_height() + 8
    x, w = MX0 + 14, 190
    bar(screen, fonts, x, y, w, sim.vl, 'L wheel', '%+4.0f cm/s' % (sim.vl * MAX_SPEED), centre=True)
    bar(screen, fonts, x, y + 20, w, sim.vr, 'R wheel', '%+4.0f cm/s' % (sim.vr * MAX_SPEED), centre=True)
    cl = sim.claw
    state = ('HOLDING cube %d' % cl.held.id) if cl.held else ('touching' if any(cl.contact.values()) else 'free')
    bar(screen, fonts, x, y + 40, w, cl.cmd, 'LT claw', '%3.0f%%  %s' % (cl.cmd * 100, state), color=(238, 168, 58))
    cap = sim.speed_scale * MAX_SPEED
    bar(screen, fonts, x, y + 60, w, sim.rt, 'RT speed', '%3.0f cm/s cap' % cap, color=(90, 170, 240))
    snap = ('turning to %.0f deg' % ((-(sim.snap_target) * 180 / 3.141592653589793) % 360)) if sim.snapping else 'idle'
    text(screen, fonts['s'], 'speed %+5.1f cm/s  heading %3.0f deg  x %+6.1f  y %+6.1f' % (sim.speed, sim.heading_deg, sim.x, sim.y),
         (MX0 + 14, y + 88), MUTED)
    text(screen, fonts['s'], 'snap: %s   arms %+.0f / %+.0f deg' % (snap if sim.mode == 'arcade_snap' else 'off (T: arcade + snap)', cl.ang[-1], cl.ang[1]),
         (MX0 + 14, y + 106), MUTED)
    text(screen, fonts['s'], 'LB/RB snap 90   Y/B/A/X face up/right/down/left', (MX0 + 14, y + 124), MUTED)


def render_dashboard2(screen, fonts, big, cam, scene, sim, view, st, mapping, log_lines, learn_prompt='', note=''):
    """One full frame, separate from the event loop so it can be tested without a window."""
    screen.fill(BG)
    text(screen, fonts['l'], 'Robot control dashboard', (M + 4, 14))
    status = st.name if st.connected else 'NO CONTROLLER FOUND'
    text(screen, fonts['m'], '%s   mapping: %s' % (status, mapping.get('name', '?')), (RX0 - 40, 24), ACCENT if st.connected else (240, 120, 100))
    cd.draw_controller_panel(screen, fonts, big, st, learn_prompt, note, log_lines)
    draw_sim_panel2(screen, fonts, cam, scene, sim, view)
    cv.draw_raw(screen, fonts, RX0 + 14, TOP + 12, st, mapping, w=RW, h=BOT - TOP, cell=74)
    text(screen, fonts['s'], 'M learn   S save   R default map   D demo   V view   T mode   Z reset all   C reset cubes   F11 fullscreen   ESC quit',
         (M + 4, H - 26), MUTED)


def set_window(fullscreen, size):
    """Fullscreen at the desktop resolution, or a resizable window of the given size."""
    if fullscreen:
        return pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    return pygame.display.set_mode(size, pygame.RESIZABLE)


def fit(win_size):
    """The largest W:H rectangle that fits in the window, centred (the rest is letterbox)."""
    ww, wh = win_size
    s = min(ww / W, wh / H)
    w, h = max(1, int(W * s)), max(1, int(H * s))
    return pygame.Rect((ww - w) // 2, (wh - h) // 2, w, h)


def present(window, canvas):
    """Scale the fixed-size dashboard onto whatever size the window is now."""
    r = fit(window.get_size())
    window.fill(BG)
    window.blit(canvas if r.size == canvas.get_size() else pygame.transform.smoothscale(canvas, r.size), r)


def main():
    demo = '--demo' in sys.argv
    pygame.init()
    fullscreen, win_size = False, (W, H)
    window = set_window(fullscreen, win_size)
    pygame.display.set_caption('Robot control dashboard v2')
    screen = pygame.Surface((W, H)).convert()          # everything is drawn here at the design size, then scaled
    fonts, big = cd.make_fonts()
    pad = xi.XboxController()
    elog, learner, note, note_t = cv.EventLog(), None, '', 0
    sim = RobotSim2()
    view_i = 0
    sw, sh = MW - 20, SCENE_H2
    scene, cam = pygame.Surface((sw, sh)), Camera(sw, sh, 400)
    clock, t0 = pygame.time.Clock(), time.time()
    while True:
        dt = min(clock.tick(60) / 1000.0, 0.05)          # a slow frame must not make the physics jump
        for e in pygame.event.get():
            if e.type == pygame.QUIT: return
            if e.type == pygame.KEYDOWN:
                alt_enter = e.key == pygame.K_RETURN and e.mod & pygame.KMOD_ALT
                if e.key in (pygame.K_F11, pygame.K_f) or alt_enter or (e.key == pygame.K_ESCAPE and fullscreen):
                    if not fullscreen: win_size = window.get_size()       # come back to the same window size
                    fullscreen = not fullscreen
                    window = set_window(fullscreen, win_size)
                elif e.key == pygame.K_ESCAPE: return
                elif e.key == pygame.K_d: demo = not demo
                elif e.key == pygame.K_m: learner = xi.Learner(); note = ''
                elif e.key == pygame.K_SPACE and learner and not learner.done: learner.skip()
                elif e.key == pygame.K_s: xi.save_mapping(pad.mapping); note, note_t = 'Saved to ' + xi.MAPPING_FILE, time.time()
                elif e.key == pygame.K_r: pad.mapping = xi.default_mapping(); learner = None; note, note_t = 'Default mapping restored', time.time()
                elif e.key == pygame.K_v: view_i = (view_i + 1) % len(VIEWS)
                elif e.key == pygame.K_t: sim.mode = MODES[(MODES.index(sim.mode) + 1) % len(MODES)]
                elif e.key == pygame.K_z: sim.reset()
                elif e.key == pygame.K_c: sim.reset_cubes()
        if demo:
            axes, buttons, hats = cv.demo_raw(time.time() - t0)
            st = xi.apply_mapping(pad.mapping, axes, buttons, hats, 'DEMO (fake input)')
        else:
            st = pad.poll()
        if learner:
            done = learner.update(st.raw_axes, st.raw_buttons, st.raw_hats) if st.connected else None
            if done is not None and learner.done:
                pad.mapping = learner.m; learner = None; note, note_t = 'Learned. Press S to save it.', time.time()
        else:
            sim.drive(st, dt)                    # the robot ignores the pad while you are teaching it the mapping
        elog.update(st)
        render_dashboard2(screen, fonts, big, cam, scene, sim, VIEWS[view_i], st, pad.mapping, elog.lines,
                          learner.prompt if learner else '', note if (note and time.time() - note_t < 6) else '')
        present(window, screen)
        pygame.display.flip()


if __name__ == '__main__':
    main()
