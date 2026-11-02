"""
controller_viewer.py - live picture of an Xbox 360 controller plus the raw numbers behind it.

    pip install pygame
    python controller_viewer.py            # real controller
    python controller_viewer.py --demo     # no controller needed: fake input, to see the screen

Keys:  M = learn mapping (guided, about a minute)   S = save mapping to xbox_mapping.json
       R = go back to the default mapping           D = demo on/off        ESC = quit
The right side always shows the RAW axis, hat and button numbers, so you can see exactly what your
controller sends even for inputs the mapping does not know about.
"""
import math, sys, time
import pygame
import xbox_input as xi

W, H = 1170, 690
BG, PANEL, BODY, EDGE, IDLE = (22, 26, 31), (32, 38, 45), (62, 68, 77), (96, 104, 115), (74, 80, 90)
TEXT, MUTED, ACCENT, ON = (226, 231, 236), (140, 150, 162), (0, 200, 170), (255, 255, 255)
COL = {'A': (70, 190, 80), 'B': (225, 65, 55), 'X': (65, 125, 225), 'Y': (238, 192, 45)}


def dim(c, f=0.45): return tuple(int(v * f) for v in c)


def text(surf, font, s, pos, color=TEXT, anchor='tl'):
    img = font.render(s, True, color)
    w, h = img.get_width(), img.get_height()
    x, y = pos
    if anchor == 'center': x, y = x - w // 2, y - h // 2
    elif anchor == 'tr': x -= w
    elif anchor == 'mid': y -= h // 2
    surf.blit(img, (x, y))


def shape(surf, kind, color, geom, edge=None, grow=3):
    """Filled shape with an optional outline made by drawing a slightly larger shape first."""
    def one(c, g):
        if kind == 'rrect': pygame.draw.rect(surf, c, g[:4], border_radius=g[4])
        elif kind == 'ellipse': pygame.draw.ellipse(surf, c, g)
        elif kind == 'circle': pygame.draw.circle(surf, c, g[0], g[1])
    if edge:
        if kind == 'circle': one(edge, ((geom[0]), geom[1] + grow))
        elif kind == 'ellipse': one(edge, (geom[0] - grow, geom[1] - grow, geom[2] + 2 * grow, geom[3] + 2 * grow))
        else: one(edge, (geom[0] - grow, geom[1] - grow, geom[2] + 2 * grow, geom[3] + 2 * grow, geom[4] + grow))
    one(color, geom)


def draw_controller(surf, fonts, ox, oy, st):
    """The controller picture, 600 x 430, top-left at (ox, oy). st is a ControllerState."""
    P = lambda x, y: (ox + x, oy + y)
    # body: two grips and the centre block share one colour and one outline
    shape(surf, 'ellipse', BODY, (ox + 52, oy + 150, 180, 265), EDGE)
    shape(surf, 'ellipse', BODY, (ox + 368, oy + 150, 180, 265), EDGE)
    shape(surf, 'rrect', BODY, (ox + 100, oy + 98, 400, 200, 80), EDGE)
    shape(surf, 'ellipse', BODY, (ox + 52, oy + 150, 180, 265))
    shape(surf, 'ellipse', BODY, (ox + 368, oy + 150, 180, 265))
    # triggers fill from the bottom as they are pulled
    for name, x, v in (('LT', 118, st.lt), ('RT', 372, st.rt)):
        pygame.draw.rect(surf, EDGE, (ox + x - 2, oy + 6, 114, 60), border_radius=14)
        pygame.draw.rect(surf, (48, 53, 60), (ox + x, oy + 8, 110, 56), border_radius=12)
        if v > 0.01:
            h = int(56 * v)
            pygame.draw.rect(surf, ACCENT, (ox + x, oy + 8 + 56 - h, 110, h), border_radius=12)
        text(surf, fonts['m'], '%s  %.2f' % (name, v), P(x + 55, 36), ON if v > 0.5 else TEXT, 'center')
    # bumpers
    for name, x in (('LB', 96), ('RB', 364)):
        on = st.buttons[name]
        pygame.draw.rect(surf, EDGE, (ox + x - 2, oy + 68, 144, 30), border_radius=11)
        pygame.draw.rect(surf, ACCENT if on else IDLE, (ox + x, oy + 70, 140, 26), border_radius=10)
        text(surf, fonts['m'], name, P(x + 70, 83), ON if on else TEXT, 'center')
    # sticks: the cap moves with the stick, the ring lights when the stick is clicked in
    for name, cx, cy, vx, vy, click in (('LS', 165, 172, st.lx, st.ly, st.buttons['L3']), ('RS', 365, 268, st.rx, st.ry, st.buttons['R3'])):
        pygame.draw.circle(surf, (40, 44, 50), P(cx, cy), 42)
        pygame.draw.circle(surf, EDGE, P(cx, cy), 42, 2)
        pygame.draw.circle(surf, ACCENT if click else IDLE, P(cx + vx * 17, cy - vy * 17), 27)
        pygame.draw.circle(surf, ON if click else EDGE, P(cx + vx * 17, cy - vy * 17), 27, 2)
        text(surf, fonts['s'], name, P(cx, cy + 56), MUTED, 'center')
        text(surf, fonts['s'], '%+.2f %+.2f' % (vx, vy), P(cx, cy + 70), TEXT if (vx or vy) else MUTED, 'center')
    # d-pad
    cx, cy = 235, 268
    pygame.draw.rect(surf, EDGE, (ox + cx - 15, oy + cy - 33, 30, 66), border_radius=6)
    pygame.draw.rect(surf, EDGE, (ox + cx - 33, oy + cy - 15, 66, 30), border_radius=6)
    dx, dy = st.dpad
    arms = {'UP': (cx - 13, cy - 31, 26, 31, dy > 0), 'DOWN': (cx - 13, cy, 26, 31, dy < 0),
            'LEFT': (cx - 31, cy - 13, 31, 26, dx < 0), 'RIGHT': (cx, cy - 13, 31, 26, dx > 0)}
    for n, (x, y, w, h, on) in arms.items():
        pygame.draw.rect(surf, ACCENT if on else IDLE, (ox + x, oy + y, w, h), border_radius=4)
    text(surf, fonts['s'], 'D-PAD', P(cx, cy + 50), MUTED, 'center')
    # face buttons
    for name, x, y in (('Y', 435, 132), ('A', 435, 198), ('X', 402, 165), ('B', 468, 165)):
        on = st.buttons[name]
        pygame.draw.circle(surf, COL[name] if on else dim(COL[name]), P(x, y), 18)
        pygame.draw.circle(surf, ON if on else EDGE, P(x, y), 18, 3 if on else 2)
        text(surf, fonts['m'], name, P(x, y), ON if on else TEXT, 'center')
    # centre buttons
    for name, x, y in (('BACK', 268, 172), ('START', 332, 172)):
        on = st.buttons[name]
        pygame.draw.circle(surf, ACCENT if on else IDLE, P(x, y), 10)
        pygame.draw.circle(surf, EDGE, P(x, y), 10, 2)
        text(surf, fonts['s'], name, P(x, y + 22), ON if on else MUTED, 'center')
    on = st.buttons['GUIDE']
    pygame.draw.circle(surf, ACCENT if on else IDLE, P(300, 128), 21)
    pygame.draw.circle(surf, EDGE, P(300, 128), 21, 2)
    text(surf, fonts['l'], 'X', P(300, 128), ON if on else MUTED, 'center')


def reverse_names(mapping):
    """raw index -> mapping name, so the raw panel can say what each raw input is."""
    ax, bt = {}, {}
    for n, m in mapping['axes'].items(): ax.setdefault(m['axis'], []).append(n)
    for n, m in mapping['triggers'].items(): ax.setdefault(m['axis'], []).append(n)
    for n, i in mapping['buttons'].items():
        if i is not None: bt.setdefault(i, []).append(n)
    for n, i in mapping.get('dpad', {}).get('buttons', {}).items(): bt.setdefault(i, []).append('D-' + n)
    return ax, bt


def draw_raw(surf, fonts, x0, y0, st, mapping, w=440, h=440, cell=80):
    ax_names, bt_names = reverse_names(mapping)
    pygame.draw.rect(surf, PANEL, (x0 - 14, y0 - 12, w, h), border_radius=8)
    y = y0
    text(surf, fonts['m'], 'RAW AXES', (x0, y), MUTED); y += 24
    if not st.raw_axes:
        text(surf, fonts['s'], 'no controller found: plug in the receiver, press Guide', (x0, y), MUTED); y += 20
    for i, v in enumerate(st.raw_axes[:8]):
        text(surf, fonts['s'], 'axis %d' % i, (x0, y + 2), TEXT)
        bx, bw = x0 + 56, max(100, w - 250)
        pygame.draw.rect(surf, (48, 53, 60), (bx, y + 3, bw, 12), border_radius=4)
        mid = bx + bw // 2
        pygame.draw.rect(surf, ACCENT, (min(mid, mid + int(v * bw / 2)), y + 3, abs(int(v * bw / 2)), 12), border_radius=4)
        pygame.draw.line(surf, EDGE, (mid, y), (mid, y + 18), 1)
        text(surf, fonts['s'], '%+.2f' % v, (bx + bw + 8, y + 2), TEXT)
        text(surf, fonts['s'], ' '.join(ax_names.get(i, [])), (bx + bw + 56, y + 2), ACCENT)
        y += 21
    y += 8
    text(surf, fonts['m'], 'RAW HATS', (x0, y), MUTED)
    text(surf, fonts['s'], '   '.join('hat %d: %s' % (i, tuple(h)) for i, h in enumerate(st.raw_hats)) or 'none', (x0 + 90, y + 2), TEXT)
    y += 30
    text(surf, fonts['m'], 'RAW BUTTONS (number + mapped name)', (x0, y), MUTED); y += 24
    for i, v in enumerate(st.raw_buttons[:20]):
        cx, cy = x0 + (i % 5) * cell, y + (i // 5) * 40
        pygame.draw.rect(surf, ACCENT if v else (48, 53, 60), (cx, cy, cell - 6, 34), border_radius=6)
        text(surf, fonts['m'], str(i), (cx + 8, cy + 8), (10, 20, 20) if v else TEXT)
        text(surf, fonts['s'], ','.join(bt_names.get(i, [])) or '?', (cx + 30, cy + 10), (10, 20, 20) if v else MUTED)
    return y


def draw_log(surf, fonts, x0, y0, log, w=440, h=190, n=8):
    pygame.draw.rect(surf, PANEL, (x0 - 14, y0 - 12, w, h), border_radius=8)
    text(surf, fonts['m'], 'EVENTS (newest on top)', (x0, y0), MUTED)
    for k, line in enumerate(log[:n]):
        text(surf, fonts['s'], line, (x0, y0 + 26 + k * 18), TEXT if k == 0 else MUTED)


class EventLog:
    """Writes a line whenever a control is pressed/released or a stick/trigger crosses the halfway mark."""
    def __init__(self):
        self.lines, self.prev = [], {}

    def update(self, st):
        now = {}
        for n, v in st.buttons.items(): now[n] = v
        for n, v in (('LX', st.lx), ('LY', st.ly), ('RX', st.rx), ('RY', st.ry)): now[n + ('+' if v >= 0 else '-')] = abs(v) > 0.5
        now['LT'], now['RT'] = st.lt > 0.5, st.rt > 0.5
        now.update({'D-UP': st.dpad[1] > 0, 'D-DOWN': st.dpad[1] < 0, 'D-LEFT': st.dpad[0] < 0, 'D-RIGHT': st.dpad[0] > 0})
        for n, v in now.items():
            if v != self.prev.get(n, False):
                self.lines.insert(0, '%s  %-8s %s' % (time.strftime('%H:%M:%S'), n, 'pressed' if v else 'released'))
        self.prev = now
        del self.lines[40:]


def demo_raw(t):
    """Fake raw input shaped like a 6-axis, 16-button, 1-hat pad so the screen can be seen without a controller."""
    axes = [math.sin(t * 0.9) * 0.9, math.cos(t * 0.7) * 0.9, math.sin(t * 1.3) * 0.7, math.cos(t * 1.1) * 0.7,
            -1 + (math.sin(t * 1.5) + 1) / 2 * 2 * 0.6, -1 + (math.cos(t * 1.2) + 1) / 2 * 2 * 0.9]
    buttons = [0] * 16
    buttons[int(t * 1.6) % 11] = 1
    hat = [(0, 1), (1, 0), (0, -1), (-1, 0), (0, 0)][int(t * 0.8) % 5]
    return axes, buttons, [hat]


def render_frame(screen, fonts, st, mapping, log_lines, learn_prompt='', note=''):
    """Draws one whole frame. Kept separate from the event loop so it can be tested without a window."""
    screen.fill(BG)
    text(screen, fonts['l'], 'Controller viewer', (30, 20))
    status = st.name if st.connected else 'NO CONTROLLER FOUND'
    text(screen, fonts['m'], '%s    mapping: %s' % (status, mapping.get('name', '?')), (30, 52), ACCENT if st.connected else (240, 120, 100))
    draw_controller(screen, fonts, 40, 100, st)
    if learn_prompt:
        pygame.draw.rect(screen, ACCENT, (30, 560, 620, 56), border_radius=8)
        text(screen, fonts['m'], learn_prompt, (44, 588), (8, 24, 22), 'mid')
    elif note:
        text(screen, fonts['m'], note, (30, 580), ACCENT)
    draw_raw(screen, fonts, 700, 40, st, mapping)
    draw_log(screen, fonts, 700, 490, log_lines)
    text(screen, fonts['s'], 'M learn mapping   S save   R default   D demo   SPACE skip   ESC quit', (30, H - 28), MUTED)


def main():
    demo = '--demo' in sys.argv
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption('Controller viewer')
    mono = 'consolas,menlo,dejavusansmono,couriernew'
    fonts = {'s': pygame.font.SysFont(mono, 13), 'm': pygame.font.SysFont(mono, 16, bold=True), 'l': pygame.font.SysFont(mono, 24, bold=True)}
    pad = xi.XboxController()
    elog, learner, note, note_t = EventLog(), None, '', 0
    clock, t0 = pygame.time.Clock(), time.time()
    while True:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: return
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_ESCAPE: return
                elif e.key == pygame.K_d: demo = not demo
                elif e.key == pygame.K_m: learner = xi.Learner(); note = ''
                elif e.key == pygame.K_SPACE and learner and not learner.done: learner.skip()
                elif e.key == pygame.K_s:
                    xi.save_mapping(pad.mapping); note, note_t = 'Saved to ' + xi.MAPPING_FILE, time.time()
                elif e.key == pygame.K_r:
                    pad.mapping = xi.default_mapping(); learner = None; note, note_t = 'Default mapping restored', time.time()
        if demo:
            axes, buttons, hats = demo_raw(time.time() - t0)
            st = xi.apply_mapping(pad.mapping, axes, buttons, hats, 'DEMO (fake input)')
        else:
            st = pad.poll()
        if learner:
            done = learner.update(st.raw_axes, st.raw_buttons, st.raw_hats) if st.connected else None
            if done is not None and learner.done:
                pad.mapping = learner.m; learner = None; note, note_t = 'Learned. Press S to save it.', time.time()
        elog.update(st)

        render_frame(screen, fonts, st, pad.mapping, elog.lines, learner.prompt if learner else '',
                     note if (note and time.time() - note_t < 6) else '')
        pygame.display.flip()
        clock.tick(60)


if __name__ == '__main__':
    main()
