"""
xbox_input.py - read an Xbox 360 controller (wireless receiver or wired) and give back the same named
controls on every operating system.

    from xbox_input import XboxController
    pad = XboxController()
    while True:
        s = pad.poll()            # a ControllerState
        print(s.buttons['A'], s.lx, s.ly, s.lt)

Why a mapping layer? The raw numbers (which axis is the left trigger, which button index is Back) depend on the
operating system and the driver. Instead of hard-coding them, each control name (A, B, LX, LT ...) points to a raw
index in a mapping. There are default mappings per operating system, and a learn mode (controller_viewer.py,
press M) that records YOUR controller and saves it to xbox_mapping.json.

Requires: pip install pygame
"""
import json, os, sys, time
from dataclasses import dataclass, field

BUTTON_NAMES = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', 'GUIDE', 'L3', 'R3']
DPAD_NAMES = ['UP', 'DOWN', 'LEFT', 'RIGHT']
STICK_NAMES = ['LX', 'LY', 'RX', 'RY']
TRIGGER_NAMES = ['LT', 'RT']
MAPPING_FILE = 'xbox_mapping.json'
DEADZONE = 0.12


def _axes(lx, ly, rx, ry):
    # y axes: SDL reports "up" as negative, we want up = positive, hence invert=True
    return {'LX': {'axis': lx, 'invert': False}, 'LY': {'axis': ly, 'invert': True},
            'RX': {'axis': rx, 'invert': False}, 'RY': {'axis': ry, 'invert': True}}


# Default mappings. Windows and Linux are the common cases; macOS needs a third-party driver and varies,
# so run learn mode there. If a default is wrong on your machine, learn mode fixes it in about a minute.
DEFAULTS = {
    'windows': {
        'name': 'windows-default',
        'buttons': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7, 'GUIDE': None, 'L3': 8, 'R3': 9},
        'dpad': {'hat': 0},
        'axes': _axes(0, 1, 2, 3),
        'triggers': {'LT': {'axis': 4, 'rest': -1.0, 'full': 1.0}, 'RT': {'axis': 5, 'rest': -1.0, 'full': 1.0}},
    },
    'linux': {
        'name': 'linux-default',
        'buttons': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7, 'GUIDE': 8, 'L3': 9, 'R3': 10},
        'dpad': {'hat': 0},
        'axes': _axes(0, 1, 3, 4),
        'triggers': {'LT': {'axis': 2, 'rest': -1.0, 'full': 1.0}, 'RT': {'axis': 5, 'rest': -1.0, 'full': 1.0}},
    },
    'mac': {
        'name': 'mac-default (guess)',
        'buttons': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 9, 'START': 8, 'GUIDE': 10, 'L3': 6, 'R3': 7},
        'dpad': {'buttons': {'UP': 11, 'DOWN': 12, 'LEFT': 13, 'RIGHT': 14}},
        'axes': _axes(0, 1, 2, 3),
        'triggers': {'LT': {'axis': 4, 'rest': -1.0, 'full': 1.0}, 'RT': {'axis': 5, 'rest': -1.0, 'full': 1.0}},
    },
}


def default_mapping():
    key = 'windows' if sys.platform.startswith('win') else 'mac' if sys.platform == 'darwin' else 'linux'
    return json.loads(json.dumps(DEFAULTS[key]))


def load_mapping(path=MAPPING_FILE):
    """Saved mapping if there is one, otherwise the default for this operating system."""
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default_mapping()


def save_mapping(mapping, path=MAPPING_FILE):
    with open(path, 'w') as f:
        json.dump(mapping, f, indent=2)


@dataclass
class ControllerState:
    connected: bool = False
    name: str = 'no controller'
    buttons: dict = field(default_factory=lambda: {n: False for n in BUTTON_NAMES})
    dpad: tuple = (0, 0)                     # (x, y): right = +1, up = +1
    lx: float = 0.0; ly: float = 0.0         # left stick, right = +, up = +, deadzone applied
    rx: float = 0.0; ry: float = 0.0
    lt: float = 0.0; rt: float = 0.0         # triggers, 0 (released) .. 1 (fully pressed)
    raw_axes: list = field(default_factory=list)
    raw_buttons: list = field(default_factory=list)
    raw_hats: list = field(default_factory=list)


def _deadzone(v, dz=DEADZONE):
    if abs(v) < dz:
        return 0.0
    return (abs(v) - dz) / (1 - dz) * (1 if v > 0 else -1)


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def apply_mapping(mapping, raw_axes, raw_buttons, raw_hats, name='controller'):
    """Turn raw numbers into named controls. Pure function: no pygame needed, easy to test."""
    s = ControllerState(connected=True, name=name, raw_axes=list(raw_axes), raw_buttons=list(raw_buttons), raw_hats=list(raw_hats))

    def ax(i): return raw_axes[i] if i is not None and 0 <= i < len(raw_axes) else 0.0
    def bt(i): return bool(raw_buttons[i]) if i is not None and 0 <= i < len(raw_buttons) else False

    for n in BUTTON_NAMES:
        s.buttons[n] = bt(mapping['buttons'].get(n))
    d = mapping.get('dpad', {})
    if 'hat' in d and d['hat'] is not None and d['hat'] < len(raw_hats):
        s.dpad = tuple(raw_hats[d['hat']])
    elif 'buttons' in d:
        b = d['buttons']
        s.dpad = (int(bt(b.get('RIGHT'))) - int(bt(b.get('LEFT'))), int(bt(b.get('UP'))) - int(bt(b.get('DOWN'))))
    vals = {}
    for n in STICK_NAMES:
        m = mapping['axes'].get(n, {'axis': None, 'invert': False})
        v = ax(m['axis']) * (-1 if m['invert'] else 1)
        vals[n] = _deadzone(v)
    s.lx, s.ly, s.rx, s.ry = vals['LX'], vals['LY'], vals['RX'], vals['RY']
    tv = {}
    for n in TRIGGER_NAMES:
        m = mapping['triggers'].get(n, {'axis': None, 'rest': -1.0, 'full': 1.0})
        if m['axis'] is None or m['full'] == m['rest']:
            tv[n] = 0.0
        else:
            tv[n] = _clamp((ax(m['axis']) - m['rest']) / (m['full'] - m['rest']), 0.0, 1.0)
    s.lt, s.rt = tv['LT'], tv['RT']
    return s


class XboxController:
    """Finds the first joystick, re-finds it if you unplug and replug, and returns ControllerState from poll()."""

    def __init__(self, mapping=None):
        import pygame
        self.pg = pygame
        if not pygame.get_init():
            pygame.init()
        pygame.joystick.init()
        self.mapping = mapping or load_mapping()
        self.joy = None
        self._last_scan = 0.0

    def _scan(self):
        if time.time() - self._last_scan < 1.0:
            return
        self._last_scan = time.time()
        self.pg.joystick.quit(); self.pg.joystick.init()
        if self.pg.joystick.get_count() > 0:
            self.joy = self.pg.joystick.Joystick(0)
            self.joy.init()

    def poll(self):
        self.pg.event.pump()
        if self.joy is None or self.pg.joystick.get_count() == 0:
            self.joy = None
            self._scan()
            if self.joy is None:
                return ControllerState()
        j = self.joy
        try:
            raw_axes = [j.get_axis(i) for i in range(j.get_numaxes())]
            raw_buttons = [j.get_button(i) for i in range(j.get_numbuttons())]
            raw_hats = [j.get_hat(i) for i in range(j.get_numhats())]
        except Exception:
            self.joy = None
            return ControllerState()
        return apply_mapping(self.mapping, raw_axes, raw_buttons, raw_hats, j.get_name())


class Learner:
    """Learn mode: asks for each control in turn and records which raw input moved. No drawing, no pygame."""

    def __init__(self):
        self.steps = ([('button', n, 'Press %s' % n if n != 'GUIDE' else 'Press the big X (Guide) button, or SPACE to skip') for n in BUTTON_NAMES]
                      + [('dpad', n, 'Press D-pad %s' % n) for n in DPAD_NAMES]
                      + [('axis', 'LX', 'Push the LEFT stick fully RIGHT'), ('axis', 'LY', 'Push the LEFT stick fully UP'),
                         ('axis', 'RX', 'Push the RIGHT stick fully RIGHT'), ('axis', 'RY', 'Push the RIGHT stick fully UP'),
                         ('trigger', 'LT', 'Squeeze the LEFT trigger (LT) fully and hold'),
                         ('trigger', 'RT', 'Squeeze the RIGHT trigger (RT) fully and hold')])
        self.i = 0
        self.m = {'name': 'learned', 'buttons': {n: None for n in BUTTON_NAMES}, 'dpad': {'buttons': {}},
                  'axes': {}, 'triggers': {}}
        self.base = None
        self.t0 = time.time()
        self.wait_release = False
        self.done = False
        self.dpad_mode = None

    @property
    def prompt(self):
        if self.done: return 'Done. Press S to save, or M to learn again.'
        if self.wait_release: return 'Got it. Let go of everything...'
        return '%d/%d  %s   (SPACE = skip)' % (self.i + 1, len(self.steps), self.steps[self.i][2])

    def skip(self):
        self._next()

    def _next(self):
        self.i += 1; self.base = None; self.t0 = time.time(); self.wait_release = False
        if self.i >= len(self.steps):
            self.done = True
        elif self.steps[self.i][0] == 'dpad' and self.dpad_mode == 'hat':
            self._next()                       # hat already covers all four directions

    def _used_axes(self):
        used = {v['axis'] for v in self.m['axes'].values()} | {v['axis'] for v in self.m['triggers'].values()}
        return used

    def update(self, axes, buttons, hats):
        """Call every frame with the raw lists. Returns the finished mapping when done, else None."""
        if self.done:
            return self.m
        if self.wait_release:
            if not any(buttons) and not any(tuple(h) != (0, 0) for h in hats) and self._at_rest(axes):
                self._next()
            return self.m if self.done else None
        if self.base is None:
            self.base = {'axes': list(axes)}
            self.t0 = time.time()
        if time.time() - self.t0 < 0.4:        # settle, so the press that finished the last step is ignored
            return None
        kind, name, _ = self.steps[self.i]
        if kind == 'button':
            used = {v for v in self.m['buttons'].values() if v is not None}
            for idx, v in enumerate(buttons):
                if v and idx not in used:
                    self.m['buttons'][name] = idx; self.wait_release = True; break
        elif kind == 'dpad':
            for idx, h in enumerate(hats):
                if tuple(h) != (0, 0):
                    self.m['dpad'] = {'hat': idx}; self.dpad_mode = 'hat'; self.wait_release = True; break
            else:
                used = {v for v in self.m['buttons'].values() if v is not None} | set(self.m['dpad'].get('buttons', {}).values())
                for idx, v in enumerate(buttons):
                    if v and idx not in used:
                        self.dpad_mode = 'buttons'
                        self.m['dpad'].setdefault('buttons', {})[name] = idx; self.wait_release = True; break
        else:
            used = self._used_axes()
            for idx, v in enumerate(axes):
                if idx in used or idx >= len(self.base['axes']):
                    continue
                delta = v - self.base['axes'][idx]
                if abs(delta) > 0.6:
                    if kind == 'axis':
                        self.m['axes'][name] = {'axis': idx, 'invert': delta < 0}
                    else:
                        self.m['triggers'][name] = {'axis': idx, 'rest': round(self.base['axes'][idx], 2), 'full': 1.0 if delta > 0 else -1.0}
                    self.wait_release = True; self._rest = list(self.base['axes']); break
        if self.wait_release and kind in ('axis', 'trigger'):
            pass
        return None

    def _at_rest(self, axes):
        base = getattr(self, '_rest', None)
        if not base or len(base) != len(axes):
            return True
        return all(abs(a - b) < 0.3 for a, b in zip(axes, base))
