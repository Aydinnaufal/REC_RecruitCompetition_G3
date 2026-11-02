"""ui_V_7_0_1.py - everything the dashboard draws with: colours, text, widgets (button, dropdown, stepper, tabs, text box, slider, spin box), gauges,
the Tune page pictures (speedometer, servo dial, 3D gyro board, battery),
the controller picture and the fonts. No other file of the dashboard is needed to read this one."""
import math
import pygame

# ---- colours
BG, PANEL, PANEL2 = (22, 26, 31), (32, 38, 45), (40, 47, 56)
TEXT, MUTED, ACCENT = (226, 231, 236), (140, 150, 162), (0, 200, 170)
OK, WARN, BAD = (0, 200, 170), (240, 190, 70), (235, 95, 85)
BTN, BTN_HOT, BTN_ON, BTN_OFF, LINE = (48, 55, 64), (66, 76, 88), (0, 150, 128), (38, 42, 48), (86, 96, 108)
BAR_BG = (48, 53, 60)
DANGER, DANGER_HOT = (150, 44, 40), (190, 58, 52)
DIM_TEXT = (110, 118, 128)


def text(surf, font, s, pos, color=TEXT, anchor='tl'):
    img = font.render(s, True, color)
    w, h = img.get_width(), img.get_height()
    x, y = pos
    if anchor == 'center': x, y = x - w // 2, y - h // 2
    elif anchor == 'tr': x -= w
    elif anchor == 'mid': y -= h // 2
    surf.blit(img, (x, y))


def make_fonts():
    """(small set for the dashboard, big set used to draw the controller picture before it is scaled down)."""
    mono = 'consolas,menlo,dejavusansmono,couriernew'
    f = lambda s, b=False: pygame.font.SysFont(mono, s, bold=b)
    return ({'s': f(13), 'm': f(16, True), 'l': f(24, True)}, {'s': f(17), 'm': f(21, True), 'l': f(30, True)})


# ---- the design canvas is scaled into any window
def fit_scale(win_w, win_h, cw, ch):
    s = min(win_w / float(cw), win_h / float(ch))
    w, h = max(1, int(cw * s)), max(1, int(ch * s))
    return s, ((win_w - w) // 2, (win_h - h) // 2), (w, h)


def to_canvas(pos, win_size, cw, ch):
    """Window pixel -> canvas pixel, or None when the pointer is in the side bars."""
    s, (ox, oy), (w, h) = fit_scale(win_size[0], win_size[1], cw, ch)
    x, y = pos[0] - ox, pos[1] - oy
    if x < 0 or y < 0 or x >= w or y >= h:
        return None
    return int(x / s), int(y / s)


def present(window, canvas):
    s, off, size = fit_scale(window.get_width(), window.get_height(), canvas.get_width(), canvas.get_height())
    window.fill((10, 12, 14))
    window.blit(canvas if size == (canvas.get_width(), canvas.get_height()) else pygame.transform.smoothscale(canvas, size), off)


# ---- widgets (all work in canvas coordinates)
class Widget:
    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.hot = False

    def inside(self, pos): return pos is not None and self.rect.collidepoint(pos)


class Button(Widget):
    def __init__(self, rect, label, action, active=None, enabled=None, danger=False):
        super().__init__(rect)
        self.label, self.action, self.active, self.enabled, self.danger = label, action, active, enabled, danger

    def is_enabled(self): return True if self.enabled is None else bool(self.enabled())

    def click(self, pos):
        if self.inside(pos) and self.is_enabled():
            self.action()
            return True
        return False

    def draw(self, surf, fonts):
        on, ok = bool(self.active()) if self.active else False, self.is_enabled()
        if self.danger:
            col = DANGER_HOT if self.hot else DANGER
        else:
            col = BTN_ON if on else (BTN_HOT if self.hot and ok else (BTN if ok else BTN_OFF))
        pygame.draw.rect(surf, col, self.rect, border_radius=7)
        pygame.draw.rect(surf, LINE if ok else (58, 64, 72), self.rect, 1, border_radius=7)
        lab = self.label() if callable(self.label) else self.label
        text(surf, fonts['s'], lab, self.rect.center, (255, 230, 225) if self.danger else (TEXT if ok else DIM_TEXT), 'center')


class Dropdown(Widget):
    """options: list of (value, label). get() returns the current value, set(value) changes it."""
    ITEM_H = 28

    def __init__(self, rect, title, options, get, set_, enabled=None):
        super().__init__(rect)
        self.title, self.options, self.get, self.set, self.enabled = title, options, get, set_, enabled
        self.open, self.hot_item = False, -1

    def is_enabled(self): return True if self.enabled is None else bool(self.enabled())

    def current_label(self):
        v = self.get()
        for val, lab in self.options:
            if val == v:
                return lab
        return str(v) if v not in (None, '') else '-'

    def item_rect(self, i): return pygame.Rect(self.rect.x, self.rect.bottom + 2 + i * self.ITEM_H, self.rect.w, self.ITEM_H)

    def click(self, pos):
        if self.open:
            for i, (val, _) in enumerate(self.options):
                if pos is not None and self.item_rect(i).collidepoint(pos):
                    self.set(val)
                    break
            self.open = False
            return True
        if self.inside(pos) and self.is_enabled() and self.options:
            self.open = True
            return True
        return False

    def draw(self, surf, fonts):
        ok = self.is_enabled()
        pygame.draw.rect(surf, BTN_HOT if (self.hot and ok) else (BTN if ok else BTN_OFF), self.rect, border_radius=7)
        pygame.draw.rect(surf, ACCENT if self.open else (LINE if ok else (58, 64, 72)), self.rect, 1, border_radius=7)
        text(surf, fonts['s'], self.title, (self.rect.x + 8, self.rect.y - 14), MUTED)
        room = max(4, (self.rect.w - 34) // 8)                                    # characters that fit before the arrow
        text(surf, fonts['s'], self.current_label()[:room], (self.rect.x + 9, self.rect.centery - 7), TEXT if ok else DIM_TEXT)
        cx, cy = self.rect.right - 14, self.rect.centery
        pygame.draw.polygon(surf, MUTED, [(cx - 5, cy - 2), (cx + 5, cy - 2), (cx, cy + 4)])

    def draw_list(self, surf, fonts):
        if not self.open:
            return
        n = len(self.options)
        r = pygame.Rect(self.rect.x, self.rect.bottom + 2, self.rect.w, self.ITEM_H * n)
        pygame.draw.rect(surf, (36, 42, 50), r, border_radius=6)
        pygame.draw.rect(surf, ACCENT, r, 1, border_radius=6)
        cur, room = self.get(), max(4, (self.rect.w - 24) // 8)
        for i, (val, lab) in enumerate(self.options):
            ir = self.item_rect(i)
            if self.hot_item == i:
                pygame.draw.rect(surf, BTN_HOT, ir.inflate(-4, -2), border_radius=4)
            text(surf, fonts['s'], (('> ' if val == cur else '  ') + lab)[:room], (ir.x + 8, ir.centery - 7), ACCENT if val == cur else TEXT)


class Stepper(Widget):
    """[ Label ] [-] 55% [+] : a switch for showing something, and its opacity in 5% steps. Fits in 160 x 24."""

    def __init__(self, rect, label, get_on, set_on, get_val, set_val, step=0.05):
        super().__init__(rect)
        self.label, self.get_on, self.set_on, self.get_val, self.set_val, self.step = label, get_on, set_on, get_val, set_val, step
        r = self.rect
        self.r_tog = pygame.Rect(r.x, r.y, 62, r.h)
        self.r_minus = pygame.Rect(r.x + 66, r.y, 24, r.h)
        self.r_plus = pygame.Rect(r.x + 66 + 24 + 38, r.y, 24, r.h)

    def click(self, pos):
        if pos is None:
            return False
        if self.r_tog.collidepoint(pos): self.set_on(not self.get_on())
        elif self.r_minus.collidepoint(pos): self.set_val(max(0.0, round(self.get_val() - self.step, 2)))
        elif self.r_plus.collidepoint(pos): self.set_val(min(1.0, round(self.get_val() + self.step, 2)))
        else: return False
        return True

    def draw(self, surf, fonts):
        on = self.get_on()
        pygame.draw.rect(surf, BTN_ON if on else BTN, self.r_tog, border_radius=7)
        pygame.draw.rect(surf, LINE, self.r_tog, 1, border_radius=7)
        text(surf, fonts['s'], self.label, self.r_tog.center, TEXT, 'center')
        for r, sym in ((self.r_minus, '-'), (self.r_plus, '+')):
            pygame.draw.rect(surf, BTN if on else BTN_OFF, r, border_radius=6)
            pygame.draw.rect(surf, LINE, r, 1, border_radius=6)
            text(surf, fonts['s'], sym, r.center, TEXT if on else MUTED, 'center')
        text(surf, fonts['s'], '%d%%' % round(self.get_val() * 100), (self.r_minus.right + 19, self.rect.centery), TEXT if on else MUTED, 'center')


class Tabs(Widget):
    def __init__(self, rect, items, get, set_):
        super().__init__(rect)
        self.items, self.get, self.set = items, get, set_
        w = self.rect.w // len(items)
        self.cells = [pygame.Rect(self.rect.x + i * w, self.rect.y, w, self.rect.h) for i in range(len(items))]

    def click(self, pos):
        for (key, _), r in zip(self.items, self.cells):
            if pos is not None and r.collidepoint(pos):
                self.set(key)
                return True
        return False

    def draw(self, surf, fonts):
        for (key, label), r in zip(self.items, self.cells):
            on = key == self.get()
            pygame.draw.rect(surf, BTN_ON if on else BTN, r.inflate(-4, 0), border_radius=7)
            text(surf, fonts['m'], label, r.center, TEXT if on else MUTED, 'center')


class TextInput(Widget):
    """One line of text. Enter calls on_submit(text). Up / Down walk through what was sent before."""

    def __init__(self, rect, on_submit, hint='type a message, Enter to send'):
        super().__init__(rect)
        self.text, self.focus, self.on_submit, self.hint = '', False, on_submit, hint
        self.history, self.hi = [], 0

    def click(self, pos):
        self.focus = bool(pos is not None and self.rect.collidepoint(pos))
        return self.focus

    def key(self, event):
        if not self.focus:
            return False
        k = event.key
        if k == pygame.K_RETURN:
            if self.text:
                self.on_submit(self.text)
                self.history.append(self.text)
                self.hi = len(self.history)
            self.text = ''
        elif k == pygame.K_BACKSPACE: self.text = self.text[:-1]
        elif k == pygame.K_ESCAPE: self.focus = False
        elif k == pygame.K_UP and self.history:
            self.hi = max(0, self.hi - 1); self.text = self.history[self.hi]
        elif k == pygame.K_DOWN and self.history:
            self.hi = min(len(self.history), self.hi + 1); self.text = self.history[self.hi] if self.hi < len(self.history) else ''
        else:
            ch = getattr(event, 'unicode', '')
            if ch and ch.isprintable() and len(self.text) < 120:
                self.text += ch
        return True

    def draw(self, surf, fonts):
        pygame.draw.rect(surf, (24, 28, 33), self.rect, border_radius=6)
        pygame.draw.rect(surf, ACCENT if self.focus else LINE, self.rect, 1, border_radius=6)
        room = max(8, (self.rect.w - 20) // 8)
        if self.text or self.focus:
            text(surf, fonts['s'], '> ' + self.text[-room:] + ('_' if self.focus else ''), (self.rect.x + 8, self.rect.centery - 7), TEXT)
        else:
            text(surf, fonts['s'], self.hint, (self.rect.x + 8, self.rect.centery - 7), MUTED)


class Slider(Widget):
    """Drag along the bar. get() / set(value); value goes from lo to hi in `step`s and sticks to `snap` (for example 0) when it is close."""

    def __init__(self, rect, get, set_, lo, hi, step=1, snap=None, enabled=None, color=ACCENT, fmt='%d'):
        super().__init__(rect)
        self.get, self.set, self.lo, self.hi, self.step, self.snap, self.enabled, self.color, self.fmt = get, set_, lo, hi, step, snap, enabled, color, fmt
        self.grab = False

    def is_enabled(self): return True if self.enabled is None else bool(self.enabled())

    def _from(self, pos):
        if pos is None: return
        f = max(0.0, min(1.0, (pos[0] - self.rect.x - 8) / float(self.rect.w - 16)))
        v = self.lo + f * (self.hi - self.lo)
        v = round(v / self.step) * self.step
        if self.snap is not None and abs(v) <= self.snap * (self.hi - self.lo): v = 0
        self.set(max(self.lo, min(self.hi, v)))

    def click(self, pos):
        if self.inside(pos) and self.is_enabled():
            self.grab = True
            self._from(pos)
            return True
        return False

    def drag(self, pos):
        if self.grab: self._from(pos)

    def release(self): self.grab = False

    def draw(self, surf, fonts):
        ok, r = self.is_enabled(), self.rect
        v = self.get()
        v = self.lo if v is None else v
        f = (v - self.lo) / float(self.hi - self.lo)
        y = r.centery
        pygame.draw.rect(surf, BAR_BG, (r.x + 8, y - 3, r.w - 16, 6), border_radius=3)
        mid = r.x + 8 + (r.w - 16) * (-self.lo / float(self.hi - self.lo)) if self.lo < 0 < self.hi else r.x + 8
        kx = r.x + 8 + (r.w - 16) * f
        if ok: pygame.draw.rect(surf, self.color, (min(mid, kx), y - 3, max(2, abs(kx - mid)), 6), border_radius=3)
        if self.lo < 0 < self.hi: pygame.draw.line(surf, (96, 104, 115), (int(mid), y - 8), (int(mid), y + 8), 1)
        pygame.draw.circle(surf, (235, 240, 244) if ok else (92, 98, 106), (int(kx), y), 8)
        pygame.draw.circle(surf, self.color if ok else LINE, (int(kx), y), 8, 2)


class Spin(Widget):
    """label  [-] value [+]  : one tuning number changed in steps. get() may return None (unknown yet): then it is greyed out."""

    def __init__(self, rect, label, get, set_, step, lo, hi, fmt='%g', enabled=None):
        super().__init__(rect)
        self.label, self.get, self.set, self.step, self.lo, self.hi, self.fmt, self.enabled = label, get, set_, step, lo, hi, fmt, enabled
        r = self.rect
        self.r_minus = pygame.Rect(r.right - 24 - 64 - 24, r.y, 24, r.h)
        self.r_plus = pygame.Rect(r.right - 24, r.y, 24, r.h)

    def is_enabled(self): return self.get() is not None and (True if self.enabled is None else bool(self.enabled()))

    def click(self, pos):
        if pos is None or not self.is_enabled(): return False
        v = self.get()
        if self.r_minus.collidepoint(pos): self.set(max(self.lo, round(v - self.step, 3)))
        elif self.r_plus.collidepoint(pos): self.set(min(self.hi, round(v + self.step, 3)))
        else: return False
        return True

    def draw(self, surf, fonts):
        ok, v = self.is_enabled(), self.get()
        text(surf, fonts['s'], self.label, (self.rect.x, self.rect.centery), TEXT if ok else MUTED, 'mid')
        for r, sym in ((self.r_minus, '-'), (self.r_plus, '+')):
            pygame.draw.rect(surf, BTN_HOT if (self.hot and ok) else (BTN if ok else BTN_OFF), r, border_radius=6)
            pygame.draw.rect(surf, LINE, r, 1, border_radius=6)
            text(surf, fonts['s'], sym, r.center, TEXT if ok else MUTED, 'center')
        text(surf, fonts['s'], '-' if v is None else self.fmt % v, (self.r_minus.right + 32, self.rect.centery), TEXT if ok else MUTED, 'center')


class Group:
    """A set of widgets that are shown together. Feed it canvas-space mouse events."""

    def __init__(self):
        self.items = []

    def add(self, w):
        self.items.append(w)
        return w

    def of(self, kind): return [w for w in self.items if isinstance(w, kind)]

    def motion(self, pos):
        for w in self.items:
            w.hot = w.inside(pos)
            if isinstance(w, Slider): w.drag(pos)
        for d in self.of(Dropdown):
            d.hot_item = -1
            if d.open and pos is not None:
                for i in range(len(d.options)):
                    if d.item_rect(i).collidepoint(pos):
                        d.hot_item = i

    def click(self, pos):
        """True when a widget used the click. An open dropdown list takes the click first."""
        for d in self.of(Dropdown):
            if d.open:
                d.click(pos)
                return True
        used = False
        for t in self.of(TextInput):
            used = t.click(pos) or used
        for w in self.items:
            if not isinstance(w, TextInput) and w.click(pos):
                return True
        return used

    def release(self):
        for w in self.of(Slider): w.release()

    def close_all(self):
        for d in self.of(Dropdown): d.open = False

    def typing(self): return any(t.focus for t in self.of(TextInput))

    def key(self, event): return any(t.key(event) for t in self.of(TextInput))

    def drop_focus(self, pos):
        for t in self.of(TextInput):
            if not t.inside(pos): t.focus = False

    def draw_widgets(self, surf, fonts):
        for w in self.items:
            if not isinstance(w, Dropdown): w.draw(surf, fonts)
        for d in self.of(Dropdown): d.draw(surf, fonts)

    def draw_lists(self, surf, fonts):
        for d in self.of(Dropdown): d.draw_list(surf, fonts)


# ---- drawing helpers
def card(surf, rect, title, fonts):
    pygame.draw.rect(surf, PANEL2, rect, border_radius=8)
    text(surf, fonts['s'], title, (rect[0] + 10, rect[1] + 6), MUTED)


def led(surf, pos, on, color=OK, r=5):
    pygame.draw.circle(surf, color if on else (58, 64, 72), pos, r)
    pygame.draw.circle(surf, (20, 24, 28), pos, r, 1)


def signal_bars(surf, x, y, level, n=4):
    for i in range(n):
        h = 5 + 4 * i
        pygame.draw.rect(surf, OK if i < level else (58, 64, 72), (x + i * 7, y + 17 - h, 5, h), border_radius=1)


def hbar(surf, rect, frac, color=ACCENT, centre=False):
    x, y, w, h = rect
    pygame.draw.rect(surf, BAR_BG, rect, border_radius=4)
    if centre:
        mid, n = x + w // 2, int(max(-1.0, min(1.0, frac)) * w / 2)
        if n:
            pygame.draw.rect(surf, color, (min(mid, mid + n), y, max(2, abs(n)), h), border_radius=4)
        pygame.draw.line(surf, (96, 104, 115), (mid, y - 1), (mid, y + h), 1)
    else:
        n = int(max(0.0, min(1.0, frac)) * w)
        if n:
            pygame.draw.rect(surf, color, (x, y, max(2, n), h), border_radius=4)


def labelled_bar(surf, fonts, x, y, w, frac, label, value, color=ACCENT, centre=False, lw=74, vw=96):
    text(surf, fonts['s'], label, (x, y), TEXT)
    hbar(surf, (x + lw, y + 2, w - lw - vw, 11), frac, color, centre)
    text(surf, fonts['s'], value, (x + w - vw + 8, y), TEXT)


def tile(surf, fonts, x, y, label, on, state='', color=OK, off_color=(30, 34, 40), size=56):
    """A square indicator: lit in `color` with a white outline when ON, in `off_color` with a grey outline when OFF, a short word inside and
    the label below. Anything at all can be told at a glance by its colour."""
    r = pygame.Rect(x, y, size, size)
    pygame.draw.rect(surf, color if on else off_color, r, border_radius=9)
    pygame.draw.rect(surf, (255, 255, 255) if on else (74, 80, 90), r, 2, border_radius=9)
    text(surf, fonts['m'], state, r.center, (8, 24, 22) if on else (205, 210, 216), 'center')
    text(surf, fonts['s'], label, (x + size // 2, y + size + 10), TEXT if on else MUTED, 'center')


def compass(surf, fonts, cx, cy, r, heading, omega_dps=0.0):
    """A rotating compass card under a fixed pointer. heading = degrees from the zero direction, turning right makes it bigger.
    Zero is Y (up), then B right, A down, X left: the same face buttons that turn the robot."""
    pygame.draw.circle(surf, (26, 30, 36), (cx, cy), r)
    pygame.draw.circle(surf, LINE, (cx, cy), r, 2)
    P = lambda a, rad: (cx + rad * math.sin(math.radians(a - heading)), cy - rad * math.cos(math.radians(a - heading)))
    for a in range(0, 360, 10):
        major = a % 30 == 0
        pygame.draw.line(surf, (150, 160, 172) if major else (84, 92, 104), P(a, r - 1), P(a, r - (11 if major else 6)), 2 if a % 90 == 0 else 1)
        if major and a % 90 != 0:
            text(surf, fonts['s'], str(a), P(a, r - 22), MUTED, 'center')
    for a, (col, lab) in {0: ((238, 192, 45), 'Y'), 90: ((225, 65, 55), 'B'), 180: ((70, 190, 80), 'A'), 270: ((65, 125, 225), 'X')}.items():
        p = P(a, r - 22)
        pygame.draw.circle(surf, col, (int(p[0]), int(p[1])), 9)
        text(surf, fonts['s'], lab, p, (10, 14, 18), 'center')
    pygame.draw.polygon(surf, ACCENT, [(cx, cy - 15), (cx - 10, cy + 12), (cx, cy + 6), (cx + 10, cy + 12)])
    pygame.draw.polygon(surf, (255, 120, 100), [(cx, cy - r - 1), (cx - 6, cy - r - 11), (cx + 6, cy - r - 11)])
    text(surf, fonts['m'], '%03.0f deg' % (heading % 360), (cx, cy + r + 16), TEXT, 'center')
    bw = int(r * 1.6)
    hbar(surf, (cx - bw // 2, cy + r + 30, bw, 9), omega_dps / 230.0, (240, 170, 70), centre=True)
    text(surf, fonts['s'], 'gyro %+.0f deg/s' % omega_dps, (cx, cy + r + 52), MUTED, 'center')


def raw_compact(surf, fonts, x, y, w, st):
    """Raw controller numbers, small: axis bars, then the buttons as numbered squares. Returns the y below it."""
    text(surf, fonts['s'], 'RAW INPUT', (x, y), MUTED)
    y += 18
    if not st.raw_axes:
        text(surf, fonts['s'], 'no controller: plug in, press Guide', (x, y), MUTED)
        y += 18
    for i, v in enumerate(st.raw_axes[:8]):
        text(surf, fonts['s'], 'axis %d' % i, (x, y), MUTED)
        hbar(surf, (x + 60, y + 2, w - 60 - 50, 9), v, ACCENT, centre=True)
        text(surf, fonts['s'], '%+.2f' % v, (x + w - 44, y), TEXT)
        y += 16
    y += 4
    nb, cell = len(st.raw_buttons[:20]), (w - 4) // 10
    for i, v in enumerate(st.raw_buttons[:20]):
        cx, cy = x + (i % 10) * cell, y + (i // 10) * 22
        pygame.draw.rect(surf, ACCENT if v else BAR_BG, (cx, cy, cell - 3, 19), border_radius=4)
        text(surf, fonts['s'], str(i), (cx + (cell - 3) // 2, cy + 10), (10, 20, 20) if v else MUTED, 'center')
    y += 22 * ((nb + 9) // 10) + 2
    text(surf, fonts['s'], '  '.join('hat%d %s' % (i, tuple(h)) for i, h in enumerate(st.raw_hats)) or 'no hat', (x, y), MUTED)
    return y + 18


# ---- Tune page pictures
def arc(surf, color, cx, cy, r, a0, a1, width=8):
    """A curved line from angle a0 to a1 (degrees, 0 = straight up, clockwise)."""
    n = max(2, int(abs(a1 - a0) / 2.5))
    pts = [(cx + r * math.sin(math.radians(a0 + (a1 - a0) * i / n)), cy - r * math.cos(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)]
    for p, q in zip(pts, pts[1:]):
        pygame.draw.line(surf, color, p, q, width)
    for p in (pts[0], pts[-1]):
        pygame.draw.circle(surf, color, (int(p[0]), int(p[1])), width // 2)


def speedo(surf, fonts, cx, cy, r, value, vmin, vmax, color, label, big, small, ticks=(-100, -50, 0, 50, 100), live=True):
    """A car speedometer: 240 degree dial, zero at the top when the range crosses zero, coloured sweep from zero to the needle."""
    A = lambda v: -120 + 240.0 * (max(vmin, min(vmax, v)) - vmin) / (vmax - vmin)
    pygame.draw.circle(surf, (26, 30, 36), (cx, cy), r + 12)
    pygame.draw.circle(surf, LINE, (cx, cy), r + 12, 2)
    arc(surf, BAR_BG, cx, cy, r - 2, -120, 120, 10)
    zero = A(0 if vmin < 0 else vmin)
    if live and abs(A(value) - zero) > 0.5: arc(surf, color, cx, cy, r - 2, min(zero, A(value)), max(zero, A(value)), 10)
    for t in ticks:
        a = math.radians(A(t))
        pygame.draw.line(surf, (150, 160, 172), (cx + (r - 18) * math.sin(a), cy - (r - 18) * math.cos(a)), (cx + (r - 9) * math.sin(a), cy - (r - 9) * math.cos(a)), 2)
        text(surf, fonts['s'], '%d' % t, (cx + (r - 32) * math.sin(a), cy - (r - 32) * math.cos(a)), MUTED, 'center')
    a = math.radians(A(value if live else (vmin + vmax) / 2.0 if vmin < 0 else vmin))
    tip = (cx + (r - 14) * math.sin(a), cy - (r - 14) * math.cos(a))
    pygame.draw.line(surf, (255, 120, 100) if live else (90, 96, 104), (cx, cy), tip, 3)
    pygame.draw.circle(surf, (210, 214, 220), (cx, cy), 7)
    text(surf, fonts['m'], label, (cx, cy - r // 2 + 4), TEXT, 'center')
    text(surf, fonts['l'], big, (cx, cy + r // 2 - 2), TEXT if live else MUTED, 'center')
    text(surf, fonts['s'], small, (cx, cy + r // 2 + 22), MUTED, 'center')


def servo_dial(surf, fonts, cx, cy, r, angle, lo, hi, color, label):
    """A half dial, 0 degrees at the left, 90 at the top, 180 at the right, with a servo horn. angle None = limp (no signal, free to turn), False = no data."""
    P = lambda deg, rad: (cx - rad * math.cos(math.radians(deg)), cy - rad * math.sin(math.radians(deg)))
    pygame.draw.circle(surf, (26, 30, 36), (cx, cy), r + 10)
    pygame.draw.rect(surf, PANEL2, (cx - r - 12, cy + 1, 2 * r + 24, r // 2 + 22))
    pygame.draw.circle(surf, LINE, (cx, cy), r + 10, 2)
    prev = None
    for d in range(0, 181, 3):                                                     # the allowed range lit up, the rest dark
        q = P(d, r - 3)
        if prev: pygame.draw.line(surf, color if lo <= d <= hi else BAR_BG, prev, q, 8)
        prev = q
    for d in range(0, 181, 30):
        pygame.draw.line(surf, (150, 160, 172), P(d, r - 14), P(d, r - 24), 2)
        text(surf, fonts['s'], '%d' % d, P(d, r - 38), MUTED, 'center')
    if angle is False:
        text(surf, fonts['m'], 'NO DATA', (cx, cy + 28), MUTED, 'center')
    elif angle is None:
        pygame.draw.line(surf, (96, 102, 110), (cx, cy), P(90, r - 20), 5)
        pygame.draw.circle(surf, (96, 102, 110), (int(P(90, r - 20)[0]), int(P(90, r - 20)[1])), 8)
        text(surf, fonts['m'], 'LIMP', (cx, cy + 28), WARN, 'center')
    else:
        tip = P(angle, r - 20)
        pygame.draw.line(surf, (235, 240, 244), (cx, cy), tip, 6)
        pygame.draw.circle(surf, color, (int(tip[0]), int(tip[1])), 9)
        text(surf, fonts['l'], '%d' % angle + chr(176), (cx, cy + 32), TEXT, 'center')
    pygame.draw.circle(surf, (210, 214, 220), (cx, cy), 9)
    pygame.draw.circle(surf, (40, 44, 50), (cx, cy), 4)
    text(surf, fonts['m'], label, (cx, cy - r - 22), TEXT, 'center')


def _rot(v, yaw, pitch, roll):
    """Body frame -> world: roll about x (forward), pitch about y (left), yaw about z (up). Angles in degrees."""
    x, y, z = v
    c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll)); y, z = y * c - z * s, y * s + z * c
    c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch)); x, z = x * c + z * s, -x * s + z * c
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw)); x, y = x * c - y * s, x * s + y * c
    return x, y, z


def board3d(surf, fonts, cx, cy, k, yaw, pitch, roll, ok=True):
    """The MPU6050 board as a little slab turning with the real sensor, with its X (forward), Y (left), Z (up) axes. k = pixels per unit."""
    proj = lambda p: (cx + k * 0.866 * (p[0] - p[1]), cy - k * (p[2] + 0.5 * (p[0] + p[1])))
    ring = [proj((1.8 * math.cos(math.radians(a)), 1.8 * math.sin(math.radians(a)), -0.8)) for a in range(0, 361, 15)]
    for p, q in zip(ring, ring[1:]): pygame.draw.line(surf, (70, 78, 88), p, q, 1)                      # the ground, fixed
    for a, name in ((0, 'N'),):
        pygame.draw.line(surf, (70, 78, 88), proj((0, 0, -0.8)), proj((1.8, 0, -0.8)), 1)
    hx, hy, hz = 1.5, 1.0, 0.14
    V = [(sx * hx, sy * hy, sz * hz) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    R = [_rot(v, yaw, pitch, roll) for v in V]
    faces = [((0, 1, 3, 2), (60, 66, 76)), ((4, 5, 7, 6), (60, 66, 76)), ((0, 1, 5, 4), (66, 72, 82)), ((2, 3, 7, 6), (66, 72, 82)), ((0, 2, 6, 4), (74, 80, 92)), ((1, 3, 7, 5), (74, 80, 92))]
    cam = (-1.0, -1.0, 1.2)
    order = sorted(faces, key=lambda f: sum(sum(R[i][j] * cam[j] for j in range(3)) for i in f[0]))
    for idx, col in order:
        top = idx == (1, 3, 7, 5)                                                                       # +z face: the chip side
        pygame.draw.polygon(surf, (0, 120, 108) if top and ok else (0, 90, 80) if top else col, [proj(R[i]) for i in idx])
        pygame.draw.polygon(surf, (150, 160, 172), [proj(R[i]) for i in idx], 1)
    for axis, col, name in (((2.4, 0, 0), (235, 80, 70), 'X'), ((0, 2.4, 0), (80, 210, 100), 'Y'), ((0, 0, 2.0), (90, 150, 245), 'Z')):
        e = proj(_rot(axis, yaw, pitch, roll))
        pygame.draw.line(surf, col, proj((0, 0, 0)), e, 3)
        pygame.draw.circle(surf, col, (int(e[0]), int(e[1])), 5)
        text(surf, fonts['m'], name, (e[0] + 10, e[1] - 10), col, 'center')


def battery(surf, fonts, x, y, w, h, volts, vmin=6.0, vmax=8.4, low=6.4, cells=2):
    """A battery picture: the fill is the charge between vmin and vmax, the line marks the low-battery warning level."""
    f = 0.0 if volts is None else max(0.0, min(1.0, (volts - vmin) / (vmax - vmin)))
    bad = volts is not None and volts < low
    col = BAD if bad else (WARN if f < 0.35 else OK)
    pygame.draw.rect(surf, (26, 30, 36), (x + w // 2 - 14, y, 28, 8), border_radius=2)                        # the cap
    body = pygame.Rect(x, y + 8, w, h - 8)
    pygame.draw.rect(surf, (26, 30, 36), body, border_radius=8)
    pygame.draw.rect(surf, LINE, body, 2, border_radius=8)
    if volts is not None and f > 0:
        fh = int((body.h - 8) * f)
        pygame.draw.rect(surf, col, (body.x + 4, body.bottom - 4 - fh, body.w - 8, fh), border_radius=5)
    for i in range(1, cells):                                                                                  # cell dividers
        yy = body.y + body.h * i // cells
        pygame.draw.line(surf, (26, 30, 36), (body.x + 4, yy), (body.right - 4, yy), 3)
    ly = body.bottom - 4 - int((body.h - 8) * (low - vmin) / (vmax - vmin))
    pygame.draw.line(surf, (255, 255, 255), (body.x - 6, ly), (body.right + 6, ly), 2)
    text(surf, fonts['s'], 'low %.1f' % low, (body.right + 10, ly), WARN, 'mid')
    text(surf, fonts['s'], '%.1f V' % vmax, (body.right + 10, body.y + 4), MUTED, 'mid')
    text(surf, fonts['s'], '%.1f V' % vmin, (body.right + 10, body.bottom - 4), MUTED, 'mid')


# ---- the controller picture, drawn 600 x 430 and then scaled into the left panel
CTRL_W, CTRL_H = 600, 430
EDGE, IDLE, BODY_COL, ON = (96, 104, 115), (74, 80, 90), (62, 68, 77), (255, 255, 255)
FACE = {'A': (70, 190, 80), 'B': (225, 65, 55), 'X': (65, 125, 225), 'Y': (238, 192, 45)}


def _dim(c, f=0.45): return tuple(int(v * f) for v in c)


def _shape(surf, kind, color, geom, edge=None, grow=3):
    def one(c, g):
        if kind == 'rrect': pygame.draw.rect(surf, c, g[:4], border_radius=g[4])
        else: pygame.draw.ellipse(surf, c, g)
    if edge:
        one(edge, (geom[0] - grow, geom[1] - grow, geom[2] + 2 * grow, geom[3] + 2 * grow) + ((geom[4] + grow,) if kind == 'rrect' else ()))
    one(color, geom)


def draw_controller(surf, fonts, ox, oy, st):
    P = lambda x, y: (ox + x, oy + y)
    _shape(surf, 'ellipse', BODY_COL, (ox + 52, oy + 150, 180, 265), EDGE)
    _shape(surf, 'ellipse', BODY_COL, (ox + 368, oy + 150, 180, 265), EDGE)
    _shape(surf, 'rrect', BODY_COL, (ox + 100, oy + 98, 400, 200, 80), EDGE)
    _shape(surf, 'ellipse', BODY_COL, (ox + 52, oy + 150, 180, 265))
    _shape(surf, 'ellipse', BODY_COL, (ox + 368, oy + 150, 180, 265))
    for name, x, v in (('LT', 118, st.lt), ('RT', 372, st.rt)):
        pygame.draw.rect(surf, EDGE, (ox + x - 2, oy + 6, 114, 60), border_radius=14)
        pygame.draw.rect(surf, (48, 53, 60), (ox + x, oy + 8, 110, 56), border_radius=12)
        if v > 0.01:
            h = int(56 * v)
            pygame.draw.rect(surf, ACCENT, (ox + x, oy + 8 + 56 - h, 110, h), border_radius=12)
        text(surf, fonts['m'], '%s  %.2f' % (name, v), P(x + 55, 36), ON if v > 0.5 else TEXT, 'center')
    for name, x in (('LB', 96), ('RB', 364)):
        on = st.buttons[name]
        pygame.draw.rect(surf, EDGE, (ox + x - 2, oy + 68, 144, 30), border_radius=11)
        pygame.draw.rect(surf, ACCENT if on else IDLE, (ox + x, oy + 70, 140, 26), border_radius=10)
        text(surf, fonts['m'], name, P(x + 70, 83), ON if on else TEXT, 'center')
    for name, cx, cy, vx, vy, click in (('LS', 165, 172, st.lx, st.ly, st.buttons['L3']), ('RS', 365, 268, st.rx, st.ry, st.buttons['R3'])):
        pygame.draw.circle(surf, (40, 44, 50), P(cx, cy), 42)
        pygame.draw.circle(surf, EDGE, P(cx, cy), 42, 2)
        pygame.draw.circle(surf, ACCENT if click else IDLE, P(cx + vx * 17, cy - vy * 17), 27)
        pygame.draw.circle(surf, ON if click else EDGE, P(cx + vx * 17, cy - vy * 17), 27, 2)
        text(surf, fonts['s'], name, P(cx, cy + 56), MUTED, 'center')
        text(surf, fonts['s'], '%+.2f %+.2f' % (vx, vy), P(cx, cy + 70), TEXT if (vx or vy) else MUTED, 'center')
    cx, cy = 235, 268
    pygame.draw.rect(surf, EDGE, (ox + cx - 15, oy + cy - 33, 30, 66), border_radius=6)
    pygame.draw.rect(surf, EDGE, (ox + cx - 33, oy + cy - 15, 66, 30), border_radius=6)
    dx, dy = st.dpad
    for x, y, w, h, on in ((cx - 13, cy - 31, 26, 31, dy > 0), (cx - 13, cy, 26, 31, dy < 0), (cx - 31, cy - 13, 31, 26, dx < 0), (cx, cy - 13, 31, 26, dx > 0)):
        pygame.draw.rect(surf, ACCENT if on else IDLE, (ox + x, oy + y, w, h), border_radius=4)
    text(surf, fonts['s'], 'D-PAD', P(cx, cy + 50), MUTED, 'center')
    for name, x, y in (('Y', 435, 132), ('A', 435, 198), ('X', 402, 165), ('B', 468, 165)):
        on = st.buttons[name]
        pygame.draw.circle(surf, FACE[name] if on else _dim(FACE[name]), P(x, y), 18)
        pygame.draw.circle(surf, ON if on else EDGE, P(x, y), 18, 3 if on else 2)
        text(surf, fonts['m'], name, P(x, y), ON if on else TEXT, 'center')
    for name, x, y in (('BACK', 268, 172), ('START', 332, 172)):
        on = st.buttons[name]
        pygame.draw.circle(surf, ACCENT if on else IDLE, P(x, y), 10)
        pygame.draw.circle(surf, EDGE, P(x, y), 10, 2)
        text(surf, fonts['s'], name, P(x, y + 22), ON if on else MUTED, 'center')
    on = st.buttons['GUIDE']
    pygame.draw.circle(surf, ACCENT if on else IDLE, P(300, 128), 21)
    pygame.draw.circle(surf, EDGE, P(300, 128), 21, 2)
    text(surf, fonts['l'], 'X', P(300, 128), ON if on else MUTED, 'center')
