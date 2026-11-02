"""
dash_ui.py - small widgets for the dashboard (buttons, dropdowns) and the canvas-to-window mapping.
The whole dashboard is drawn on a fixed 'design canvas'; the window can be any size or fullscreen and the canvas is scaled to fit
(aspect ratio kept, bars at the sides). Widgets work in canvas coordinates, so the mouse position is mapped first with to_canvas().
"""
import pygame
from controller_viewer import text, PANEL, TEXT, MUTED, ACCENT

BTN, BTN_HOT, BTN_ON, BTN_OFF = (48, 55, 64), (66, 76, 88), (0, 150, 128), (38, 42, 48)
LINE = (86, 96, 108)


def fit_scale(win_w, win_h, cw, ch):
    """Scale and offset that fit the canvas into the window, keeping the aspect ratio."""
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
    """Scale the canvas into the window (letterboxed)."""
    ww, wh = window.get_width(), window.get_height()
    s, off, size = fit_scale(ww, wh, canvas.get_width(), canvas.get_height())
    window.fill((10, 12, 14))
    if size == (canvas.get_width(), canvas.get_height()):
        window.blit(canvas, off)
    else:
        window.blit(pygame.transform.smoothscale(canvas, size), off)


class Widget:
    def __init__(self, rect):
        self.rect = pygame.Rect(rect)
        self.hot = False

    def inside(self, pos): return pos is not None and self.rect.collidepoint(pos)


class Button(Widget):
    def __init__(self, rect, label, action, active=None, enabled=None, tip=''):
        super().__init__(rect)
        self.label, self.action, self.active, self.enabled, self.tip = label, action, active, enabled, tip

    def is_enabled(self): return True if self.enabled is None else bool(self.enabled())

    def click(self, pos):
        if self.inside(pos) and self.is_enabled():
            self.action()
            return True
        return False

    def draw(self, surf, fonts):
        on = bool(self.active()) if self.active else False
        ok = self.is_enabled()
        col = BTN_ON if on else (BTN_HOT if self.hot and ok else (BTN if ok else BTN_OFF))
        pygame.draw.rect(surf, col, self.rect, border_radius=7)
        pygame.draw.rect(surf, LINE if ok else (58, 64, 72), self.rect, 1, border_radius=7)
        lab = self.label() if callable(self.label) else self.label
        text(surf, fonts['s'], lab, self.rect.center, TEXT if ok else (110, 118, 128), 'center')


class Dropdown(Widget):
    """options: list of (value, label). get() returns the current value, set(value) changes it."""
    ITEM_H = 28

    def __init__(self, rect, title, options, get, set_, enabled=None):
        super().__init__(rect)
        self.title, self.options, self.get, self.set, self.enabled = title, options, get, set_, enabled
        self.open = False

    def is_enabled(self): return True if self.enabled is None else bool(self.enabled())

    def current_label(self):
        v = self.get()
        for val, lab in self.options:
            if val == v:
                return lab
        return str(v)

    def item_rect(self, i):
        return pygame.Rect(self.rect.x, self.rect.bottom + 2 + i * self.ITEM_H, self.rect.w, self.ITEM_H)

    def list_rect(self):
        return pygame.Rect(self.rect.x, self.rect.bottom + 2, self.rect.w, self.ITEM_H * len(self.options))

    def click(self, pos):
        """True when the click was used. An open list swallows every click (and closes)."""
        if self.open:
            for i, (val, _) in enumerate(self.options):
                if self.item_rect(i).collidepoint(pos or (-1, -1)):
                    self.set(val)
                    break
            self.open = False
            return True
        if self.inside(pos) and self.is_enabled():
            self.open = True
            return True
        return False

    def draw(self, surf, fonts):
        ok = self.is_enabled()
        pygame.draw.rect(surf, BTN_HOT if (self.hot and ok) else (BTN if ok else BTN_OFF), self.rect, border_radius=7)
        pygame.draw.rect(surf, ACCENT if self.open else (LINE if ok else (58, 64, 72)), self.rect, 1, border_radius=7)
        text(surf, fonts['s'], self.title, (self.rect.x + 8, self.rect.y - 14), MUTED)
        text(surf, fonts['s'], self.current_label(), (self.rect.x + 9, self.rect.centery - 7), TEXT if ok else (110, 118, 128))
        cx, cy = self.rect.right - 14, self.rect.centery
        pygame.draw.polygon(surf, MUTED, [(cx - 5, cy - 2), (cx + 5, cy - 2), (cx, cy + 4)])

    def draw_list(self, surf, fonts):
        if not self.open:
            return
        r = self.list_rect()
        pygame.draw.rect(surf, (36, 42, 50), r, border_radius=6)
        pygame.draw.rect(surf, ACCENT, r, 1, border_radius=6)
        cur = self.get()
        for i, (val, lab) in enumerate(self.options):
            ir = self.item_rect(i)
            if self.hot_item == i:
                pygame.draw.rect(surf, BTN_HOT, ir.inflate(-4, -2), border_radius=4)
            text(surf, fonts['s'], ('> ' if val == cur else '  ') + lab, (ir.x + 8, ir.centery - 7), ACCENT if val == cur else TEXT)

    hot_item = -1


class Stepper(Widget):
    """[x] Label  [-] 55% [+]  : a switch for showing something, and its strength in 5% steps."""
    def __init__(self, rect, label, get_on, set_on, get_val, set_val, step=0.05):
        super().__init__(rect)
        self.label, self.get_on, self.set_on, self.get_val, self.set_val, self.step = label, get_on, set_on, get_val, set_val, step
        r = self.rect
        self.r_tog = pygame.Rect(r.x, r.y, 66, r.h)
        self.r_minus = pygame.Rect(r.x + 72, r.y, 24, r.h)
        self.r_plus = pygame.Rect(r.x + 72 + 24 + 40, r.y, 24, r.h)

    def click(self, pos):
        if pos is None:
            return False
        if self.r_tog.collidepoint(pos):
            self.set_on(not self.get_on())
        elif self.r_minus.collidepoint(pos):
            self.set_val(max(0.0, round(self.get_val() - self.step, 2)))
        elif self.r_plus.collidepoint(pos):
            self.set_val(min(1.0, round(self.get_val() + self.step, 2)))
        else:
            return False
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
        text(surf, fonts['s'], '%d%%' % round(self.get_val() * 100), (self.r_minus.right + 20, self.rect.centery), TEXT if on else MUTED, 'center')


class UI:
    """A set of buttons and dropdowns. Feed it canvas-space mouse events."""

    def __init__(self):
        self.buttons, self.dropdowns, self.steppers = [], [], []

    def add(self, w):
        (self.dropdowns if isinstance(w, Dropdown) else self.steppers if isinstance(w, Stepper) else self.buttons).append(w)
        return w

    def close_all(self):
        for d in self.dropdowns:
            d.open = False

    def motion(self, pos):
        for b in self.buttons:
            b.hot = b.inside(pos)
        for d in self.dropdowns:
            d.hot = d.inside(pos)
            d.hot_item = -1
            if d.open and pos is not None:
                for i in range(len(d.options)):
                    if d.item_rect(i).collidepoint(pos):
                        d.hot_item = i

    def click(self, pos):
        """Returns True when a widget used the click."""
        for d in self.dropdowns:                       # an open list takes the click first
            if d.open:
                d.click(pos)
                return True
        for d in self.dropdowns:
            if d.inside(pos) and d.click(pos):
                return True
        for b in self.buttons + self.steppers:
            if b.click(pos):
                return True
        return False

    def draw(self, surf, fonts):
        for b in self.buttons + self.steppers:
            b.draw(surf, fonts)
        for d in self.dropdowns:
            d.draw(surf, fonts)
        for d in self.dropdowns:
            d.draw_list(surf, fonts)
