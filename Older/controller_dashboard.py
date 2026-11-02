"""
controller_dashboard.py - three panels: [ controller picture ] [ robot simulator ] [ raw inputs + event log ]

    pip install pygame
    python controller_dashboard.py            # real controller
    python controller_dashboard.py --demo     # fake input, no controller needed

Controller keys (same as controller_viewer.py):
    M learn mapping   S save mapping   R default mapping   D demo on/off   SPACE skip a step   ESC quit
Robot simulator keys:
    V change view (arena / chase / top)   T drive mode (arcade / tank)   Z put the robot back at the centre

The robot is moved only by the controller input, with the same maths a real two-wheel robot uses:
  arcade: left stick up/down = forward/back, left/right = turn.   tank: left stick Y = left wheel, right stick Y = right wheel.
"""
import sys, time
import pygame
import xbox_input as xi
import controller_viewer as cv
from controller_viewer import text, BG, PANEL, TEXT, MUTED, ACCENT
from robot_sim import RobotSim, Camera, draw_scene, VIEWS, MAX_SPEED, clamp

W, H = 1360, 720
M, G = 12, 12                                   # outer margin, gap between panels
LW, MW, RW = 460, 440, 400                      # panel widths
TOP, BOT = 58, H - 38                           # panels run from y = TOP to y = BOT
LX0, MX0 = M, M + LW + G
RX0 = MX0 + MW + G
SCENE_H = 480                                   # height of the 3D view
CTRL_W, CTRL_H = 600, 430                       # the controller picture is drawn at this size, then scaled to fit the left panel


def draw_controller_panel(screen, fonts, big, st, learn_prompt, note, log_lines):
    pygame.draw.rect(screen, PANEL, (LX0, TOP, LW, BOT - TOP), border_radius=8)
    text(screen, fonts['m'], 'CONTROLLER', (LX0 + 14, TOP + 10), MUTED)
    img = pygame.Surface((CTRL_W, CTRL_H))
    img.fill(PANEL)
    cv.draw_controller(img, big, 0, 0, st)       # drawn with larger fonts so the text stays readable after scaling
    scaled = pygame.transform.smoothscale(img, (LW - 20, int((LW - 20) * CTRL_H / CTRL_W)))
    screen.blit(scaled, (LX0 + 10, TOP + 36))
    y = TOP + 36 + scaled.get_height() + 14
    if learn_prompt:
        pygame.draw.rect(screen, ACCENT, (LX0 + 12, y, LW - 24, 56), border_radius=8)
        text(screen, fonts['m'], learn_prompt[:46], (LX0 + 24, y + 20), (8, 24, 22))
        if len(learn_prompt) > 46: text(screen, fonts['s'], learn_prompt[46:], (LX0 + 24, y + 40), (8, 24, 22))
    elif note:
        text(screen, fonts['m'], note, (LX0 + 14, y + 4), ACCENT)
    else:
        text(screen, fonts['s'], 'M = learn the mapping for your controller', (LX0 + 14, y + 4), MUTED)
    ly = y + 74
    cv.draw_log(screen, fonts, LX0 + 14, ly + 12, log_lines, w=LW - 4, h=BOT - ly - 4, n=max(3, (BOT - ly - 50) // 18))


def draw_sim_panel(screen, fonts, cam, scene, sim, view):
    pygame.draw.rect(screen, PANEL, (MX0, TOP, MW, BOT - TOP), border_radius=8)
    text(screen, fonts['m'], 'ROBOT SIMULATOR', (MX0 + 14, TOP + 10), MUTED)
    text(screen, fonts['s'], 'view: %s (V)   drive: %s (T)' % (view, sim.mode), (MX0 + MW - 14, TOP + 12), TEXT, 'tr')
    draw_scene(scene, cam, sim, view)
    sh = scene.get_height()
    screen.blit(scene, (MX0 + 10, TOP + 36))
    y = TOP + 36 + sh + 10
    # wheel speed bars: centre = stopped, right = forward
    for label, v, yy in (('L wheel', sim.vl, y), ('R wheel', sim.vr, y + 22)):
        text(screen, fonts['s'], label, (MX0 + 14, yy + 1), TEXT)
        bx, bw = MX0 + 82, 250
        pygame.draw.rect(screen, (48, 53, 60), (bx, yy + 2, bw, 12), border_radius=4)
        mid = bx + bw // 2
        pygame.draw.rect(screen, ACCENT if v >= 0 else (240, 150, 90), (min(mid, mid + int(v * bw / 2)), yy + 2, abs(int(v * bw / 2)), 12), border_radius=4)
        pygame.draw.line(screen, (96, 104, 115), (mid, yy), (mid, yy + 16), 1)
        text(screen, fonts['s'], '%+4.0f cm/s' % (v * MAX_SPEED), (bx + bw + 8, yy + 1), TEXT)
    text(screen, fonts['s'], 'speed %+5.1f cm/s   heading %3.0f deg   x %+6.1f  y %+6.1f cm' % (sim.speed, sim.heading_deg, sim.x, sim.y),
         (MX0 + 14, y + 48), MUTED)
    text(screen, fonts['s'], 'heading: 0 = up the map, turning right makes it bigger', (MX0 + 14, y + 66), MUTED)


def render_dashboard(screen, fonts, big, cam, scene, sim, view, st, mapping, log_lines, learn_prompt='', note=''):
    """One full frame. Separate from the event loop so it can be tested without a window."""
    screen.fill(BG)
    text(screen, fonts['l'], 'Robot control dashboard', (M + 4, 14))
    status = st.name if st.connected else 'NO CONTROLLER FOUND'
    text(screen, fonts['m'], '%s   mapping: %s' % (status, mapping.get('name', '?')), (RX0 - 40, 24), ACCENT if st.connected else (240, 120, 100))
    draw_controller_panel(screen, fonts, big, st, learn_prompt, note, log_lines)
    draw_sim_panel(screen, fonts, cam, scene, sim, view)
    cv.draw_raw(screen, fonts, RX0 + 14, TOP + 12, st, mapping, w=RW, h=BOT - TOP, cell=74)
    text(screen, fonts['s'], 'M learn   S save   R default map   D demo   V view   T drive mode   Z reset robot   ESC quit', (M + 4, H - 26), MUTED)


def make_fonts():
    mono = 'consolas,menlo,dejavusansmono,couriernew'
    f = lambda s, b=False: pygame.font.SysFont(mono, s, bold=b)
    return ({'s': f(13), 'm': f(16, True), 'l': f(24, True)}, {'s': f(17), 'm': f(21, True), 'l': f(30, True)})


def main():
    demo = '--demo' in sys.argv
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption('Robot control dashboard')
    fonts, big = make_fonts()
    pad = xi.XboxController()
    elog, learner, note, note_t = cv.EventLog(), None, '', 0
    sim = RobotSim()
    view_i = 0
    sw, sh = MW - 20, SCENE_H
    scene, cam = pygame.Surface((sw, sh)), Camera(sw, sh, 400)
    clock, t0 = pygame.time.Clock(), time.time()
    while True:
        dt = clock.tick(60) / 1000.0
        for e in pygame.event.get():
            if e.type == pygame.QUIT: return
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE: return
                elif e.key == pygame.K_d: demo = not demo
                elif e.key == pygame.K_m: learner = xi.Learner(); note = ''
                elif e.key == pygame.K_SPACE and learner and not learner.done: learner.skip()
                elif e.key == pygame.K_s: xi.save_mapping(pad.mapping); note, note_t = 'Saved to ' + xi.MAPPING_FILE, time.time()
                elif e.key == pygame.K_r: pad.mapping = xi.default_mapping(); learner = None; note, note_t = 'Default mapping restored', time.time()
                elif e.key == pygame.K_v: view_i = (view_i + 1) % len(VIEWS)
                elif e.key == pygame.K_t: sim.mode = 'tank' if sim.mode == 'arcade' else 'arcade'
                elif e.key == pygame.K_z: sim.reset()
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
        render_dashboard(screen, fonts, big, cam, scene, sim, VIEWS[view_i], st, pad.mapping, elog.lines,
                         learner.prompt if learner else '', note if (note and time.time() - note_t < 6) else '')
        pygame.display.flip()


if __name__ == '__main__':
    main()
