"""
dash_ui_V_4_0.py - widgets and small drawing helpers for dashboard V_4_0. Builds on dash_ui.py (Button, Dropdown, Stepper, UI).
Adds: Tabs, TextInput, Led, signal bars, compass, compact raw-input panel, section headers.
"""
import math
import pygame
import dash_ui as ui
from dash_ui import Button, Dropdown, Stepper, Widget, fit_scale, to_canvas, present, BTN, BTN_HOT, BTN_ON, BTN_OFF, LINE
from controller_viewer import text, reverse_names, PANEL, TEXT, MUTED, ACCENT

PANEL2 = (40, 47, 56)             # a card inside a panel
OK, WARN, BAD = (0, 200, 170), (240, 190, 70), (235, 95, 85)
BAR_BG = (48, 53, 60)
DANGER, DANGER_HOT = (150, 44, 40), (190, 58, 52)


class Tabs(Widget):
    """A row of tab buttons. get() / set_() hold the selected key."""

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
    """One line of text for the serial terminal. Enter calls on_submit(text). Up / Down walk through what was sent before."""

    def __init__(self, rect, on_submit, hint='type a command, Enter to send'):
        super().__init__(rect)
        self.text, self.focus, self.on_submit, self.hint = '', False, on_submit, hint
        self.history, self.hi = [], 0

    def click(self, pos):
        self.focus = bool(pos is not None and self.rect.collidepoint(pos))
        return self.focus

    def key(self, event):
        """Returns True when the key was used (so the dashboard shortcuts do not also fire)."""
        if not self.focus:
            return False
        k = event.key
        if k == pygame.K_RETURN:
            if self.text:
                self.on_submit(self.text)
                self.history.append(self.text)
                self.hi = len(self.history)
            self.text = ''
        elif k == pygame.K_BACKSPACE:
            self.text = self.text[:-1]
        elif k == pygame.K_ESCAPE:
            self.focus = False
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
        shown = self.text[-44:]
        if shown or self.focus:
            text(surf, fonts['s'], '> ' + shown + ('_' if self.focus else ''), (self.rect.x + 8, self.rect.centery - 7), TEXT)
        else:
            text(surf, fonts['s'], self.hint, (self.rect.x + 8, self.rect.centery - 7), MUTED)


class UI4(ui.UI):
    """ui.UI plus text inputs. keys_captured() tells the main loop that a text box has the keyboard."""

    def __init__(self):
        super().__init__()
        self.inputs, self.tabs = [], []

    def add(self, w):
        if isinstance(w, TextInput): self.inputs.append(w); return w
        if isinstance(w, Tabs): self.tabs.append(w); return w
        return super().add(w)

    def click(self, pos):
        used = False
        for t in self.inputs:
            used = t.click(pos) or used
        for t in self.tabs:
            if t.click(pos):
                return True
        return super().click(pos) or used

    def keys_captured(self): return any(t.focus for t in self.inputs)

    def key(self, event):
        return any(t.key(event) for t in self.inputs)

    def draw(self, surf, fonts):
        for t in self.tabs: t.draw(surf, fonts)
        for t in self.inputs: t.draw(surf, fonts)
        super().draw(surf, fonts)


# ------------------------------------------------------------------------------------------ drawing helpers
def card(surf, rect, title=None, fonts=None, color=PANEL2):
    pygame.draw.rect(surf, color, rect, border_radius=8)
    if title and fonts:
        text(surf, fonts['s'], title, (rect[0] + 10, rect[1] + 6), MUTED)


def led(surf, pos, on, color=OK, r=5):
    pygame.draw.circle(surf, color if on else (58, 64, 72), pos, r)
    pygame.draw.circle(surf, (20, 24, 28), pos, r, 1)


def signal_bars(surf, x, y, level, n=4, color=OK):
    """Phone style bars, level 0..n."""
    for i in range(n):
        h = 5 + 4 * i
        pygame.draw.rect(surf, color if i < level else (58, 64, 72), (x + i * 7, y + 17 - h, 5, h), border_radius=1)


def hbar(surf, rect, frac, color=ACCENT, centre=False, back=BAR_BG):
    x, y, w, h = rect
    pygame.draw.rect(surf, back, rect, border_radius=4)
    if centre:
        mid = x + w // 2
        n = int(frac * w / 2)
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


def compass(surf, fonts, cx, cy, r, heading, omega_dps=0.0, label_colors=None):
    """A rotating compass card. heading = degrees from the zero direction, turning right makes it bigger. The card turns under a fixed pointer
    at the top, so what is at the top is where the robot faces. Zero direction is Y (up), then B right, A down, X left (the same face buttons that turn the robot)."""
    pygame.draw.circle(surf, (26, 30, 36), (cx, cy), r)
    pygame.draw.circle(surf, LINE, (cx, cy), r, 2)
    P = lambda a, rad: (cx + rad * math.sin(math.radians(a - heading)), cy - rad * math.cos(math.radians(a - heading)))
    for a in range(0, 360, 10):
        major = a % 30 == 0
        pygame.draw.line(surf, (150, 160, 172) if major else (84, 92, 104), P(a, r - 1), P(a, r - (11 if major else 6)), 2 if a % 90 == 0 else 1)
        if a % 30 == 0 and a % 90 != 0:
            text(surf, fonts['s'], str(a), P(a, r - 22), MUTED, 'center')
    cols = label_colors or {0: ((238, 192, 45), 'Y'), 90: ((225, 65, 55), 'B'), 180: ((70, 190, 80), 'A'), 270: ((65, 125, 225), 'X')}
    for a, (col, lab) in cols.items():
        p = P(a, r - 22)
        pygame.draw.circle(surf, col, (int(p[0]), int(p[1])), 9)
        text(surf, fonts['s'], lab, p, (10, 14, 18), 'center')
    # the robot (always pointing up) and the pointer
    pygame.draw.polygon(surf, ACCENT, [(cx, cy - 15), (cx - 10, cy + 12), (cx, cy + 6), (cx + 10, cy + 12)])
    pygame.draw.polygon(surf, (255, 120, 100), [(cx, cy - r - 1), (cx - 6, cy - r - 11), (cx + 6, cy - r - 11)])
    # rotation rate: an arc-like bar under the dial (gyro)
    text(surf, fonts['m'], '%03.0f deg' % (heading % 360), (cx, cy + r + 16), TEXT, 'center')
    bw = int(r * 1.6)
    hbar(surf, (cx - bw // 2, cy + r + 30, bw, 9), max(-1.0, min(1.0, omega_dps / 230.0)), (240, 170, 70), centre=True)
    text(surf, fonts['s'], 'gyro %+.0f deg/s' % omega_dps, (cx, cy + r + 52), MUTED, 'center')


def raw_compact(surf, fonts, x, y, w, st, mapping):
    """Raw controller numbers, small: six axis bars stacked, then the buttons as little numbered squares. Returns the y below it."""
    ax_names, bt_names = reverse_names(mapping)
    text(surf, fonts['s'], 'RAW INPUT', (x, y), MUTED)
    y += 18
    if not st.raw_axes:
        text(surf, fonts['s'], 'no controller: plug in, press Guide', (x, y), MUTED)
        y += 18
    for i, v in enumerate(st.raw_axes[:8]):
        name = ' '.join(ax_names.get(i, [])) or '-'
        text(surf, fonts['s'], 'a%d %s' % (i, name[:6]), (x, y), MUTED)
        hbar(surf, (x + 84, y + 2, w - 84 - 50, 9), v, ACCENT, centre=True)
        text(surf, fonts['s'], '%+.2f' % v, (x + w - 44, y), TEXT)
        y += 16
    y += 4
    nb = len(st.raw_buttons[:20])
    cell = (w - 4) // 10
    for i, v in enumerate(st.raw_buttons[:20]):
        cx, cy = x + (i % 10) * cell, y + (i // 10) * 22
        pygame.draw.rect(surf, ACCENT if v else BAR_BG, (cx, cy, cell - 3, 19), border_radius=4)
        text(surf, fonts['s'], str(i), (cx + (cell - 3) // 2, cy + 10), (10, 20, 20) if v else MUTED, 'center')
    y += 22 * ((nb + 9) // 10) + 2
    hats = '  '.join('hat%d %s' % (i, tuple(h)) for i, h in enumerate(st.raw_hats)) or 'no hat'
    text(surf, fonts['s'], hats, (x, y), MUTED)
    return y + 18
