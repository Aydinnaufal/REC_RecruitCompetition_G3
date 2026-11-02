"""input_V_7_0_1.py - reads an Xbox 360 controller (wired or wireless receiver) and gives the same named controls on every system.
If a file xbox_mapping.json sits next to the program (saved by an older version) it is used, otherwise the default for this system."""
import json, os, sys, time
from dataclasses import dataclass, field

BUTTON_NAMES = ['A', 'B', 'X', 'Y', 'LB', 'RB', 'BACK', 'START', 'GUIDE', 'L3', 'R3']
STICK_NAMES = ['LX', 'LY', 'RX', 'RY']
TRIGGER_NAMES = ['LT', 'RT']
MAPPING_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'xbox_mapping.json')
DEADZONE = 0.12


def _axes(lx, ly, rx, ry):             # SDL reports "up" as negative, we want up = positive
    return {'LX': {'axis': lx, 'invert': False}, 'LY': {'axis': ly, 'invert': True}, 'RX': {'axis': rx, 'invert': False}, 'RY': {'axis': ry, 'invert': True}}


DEFAULTS = {
    'windows': {'name': 'windows-default', 'buttons': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7, 'GUIDE': None, 'L3': 8, 'R3': 9},
                'dpad': {'hat': 0}, 'axes': _axes(0, 1, 2, 3),
                'triggers': {'LT': {'axis': 4, 'rest': -1.0, 'full': 1.0}, 'RT': {'axis': 5, 'rest': -1.0, 'full': 1.0}}},
    'linux': {'name': 'linux-default', 'buttons': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 6, 'START': 7, 'GUIDE': 8, 'L3': 9, 'R3': 10},
              'dpad': {'hat': 0}, 'axes': _axes(0, 1, 3, 4),
              'triggers': {'LT': {'axis': 2, 'rest': -1.0, 'full': 1.0}, 'RT': {'axis': 5, 'rest': -1.0, 'full': 1.0}}},
    'mac': {'name': 'mac-default (guess)', 'buttons': {'A': 0, 'B': 1, 'X': 2, 'Y': 3, 'LB': 4, 'RB': 5, 'BACK': 9, 'START': 8, 'GUIDE': 10, 'L3': 6, 'R3': 7},
            'dpad': {'buttons': {'UP': 11, 'DOWN': 12, 'LEFT': 13, 'RIGHT': 14}}, 'axes': _axes(0, 1, 2, 3),
            'triggers': {'LT': {'axis': 4, 'rest': -1.0, 'full': 1.0}, 'RT': {'axis': 5, 'rest': -1.0, 'full': 1.0}}},
}


def default_mapping():
    key = 'windows' if sys.platform.startswith('win') else 'mac' if sys.platform == 'darwin' else 'linux'
    return json.loads(json.dumps(DEFAULTS[key]))


def load_mapping(path=MAPPING_FILE):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return default_mapping()


@dataclass
class ControllerState:
    connected: bool = False
    name: str = 'no controller'
    buttons: dict = field(default_factory=lambda: {n: False for n in BUTTON_NAMES})
    dpad: tuple = (0, 0)                     # (x, y): right = +1, up = +1
    lx: float = 0.0; ly: float = 0.0         # sticks: right = +, up = +, deadzone applied
    rx: float = 0.0; ry: float = 0.0
    lt: float = 0.0; rt: float = 0.0         # triggers 0 (released) .. 1 (fully pressed)
    raw_axes: list = field(default_factory=list)
    raw_buttons: list = field(default_factory=list)
    raw_hats: list = field(default_factory=list)

    def any_input(self):
        """True when anything at all is being touched: a button, the D-pad, a stick or a trigger."""
        return (any(self.buttons.values()) or self.dpad != (0, 0) or any(abs(v) > 0.0 for v in (self.lx, self.ly, self.rx, self.ry))
                or self.lt > 0.05 or self.rt > 0.05)


def _deadzone(v):
    return 0.0 if abs(v) < DEADZONE else (abs(v) - DEADZONE) / (1 - DEADZONE) * (1 if v > 0 else -1)


def apply_mapping(mapping, raw_axes, raw_buttons, raw_hats, name='controller'):
    """Raw numbers -> named controls. A pure function, so it can be tested without a controller."""
    s = ControllerState(connected=True, name=name, raw_axes=list(raw_axes), raw_buttons=list(raw_buttons), raw_hats=list(raw_hats))
    ax = lambda i: raw_axes[i] if i is not None and 0 <= i < len(raw_axes) else 0.0
    bt = lambda i: bool(raw_buttons[i]) if i is not None and 0 <= i < len(raw_buttons) else False
    for n in BUTTON_NAMES:
        s.buttons[n] = bt(mapping['buttons'].get(n))
    d = mapping.get('dpad', {})
    if d.get('hat') is not None and d['hat'] < len(raw_hats):
        s.dpad = tuple(raw_hats[d['hat']])
    elif 'buttons' in d:
        b = d['buttons']
        s.dpad = (int(bt(b.get('RIGHT'))) - int(bt(b.get('LEFT'))), int(bt(b.get('UP'))) - int(bt(b.get('DOWN'))))
    vals = {}
    for n in STICK_NAMES:
        m = mapping['axes'].get(n, {'axis': None, 'invert': False})
        vals[n] = _deadzone(ax(m['axis']) * (-1 if m['invert'] else 1))
    s.lx, s.ly, s.rx, s.ry = vals['LX'], vals['LY'], vals['RX'], vals['RY']
    tv = {}
    for n in TRIGGER_NAMES:
        m = mapping['triggers'].get(n, {'axis': None, 'rest': -1.0, 'full': 1.0})
        tv[n] = 0.0 if m['axis'] is None or m['full'] == m['rest'] else max(0.0, min(1.0, (ax(m['axis']) - m['rest']) / (m['full'] - m['rest'])))
    s.lt, s.rt = tv['LT'], tv['RT']
    return s


class XboxController:
    """Finds the first joystick, finds it again after unplug / replug, and returns a ControllerState from poll()."""

    def __init__(self, mapping=None):
        import pygame
        self.pg = pygame
        if not pygame.get_init():
            pygame.init()
        pygame.joystick.init()
        self.mapping = mapping or load_mapping()
        self.joy, self._last_scan = None, 0.0

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


class EventLog:
    """One line whenever a control is pressed / released or a stick or trigger crosses the halfway mark. Newest first."""

    def __init__(self):
        self.lines, self.prev = [], {}

    def update(self, st):
        now = dict(st.buttons)
        for n, v in (('LX', st.lx), ('LY', st.ly), ('RX', st.rx), ('RY', st.ry)): now[n + ('+' if v >= 0 else '-')] = abs(v) > 0.5
        now['LT'], now['RT'] = st.lt > 0.5, st.rt > 0.5
        now.update({'D-UP': st.dpad[1] > 0, 'D-DOWN': st.dpad[1] < 0, 'D-LEFT': st.dpad[0] < 0, 'D-RIGHT': st.dpad[0] > 0})
        for n, v in now.items():
            if v != self.prev.get(n, False):
                self.lines.insert(0, '%s  %-8s %s' % (time.strftime('%H:%M:%S'), n, 'pressed' if v else 'released'))
        self.prev = now
        del self.lines[40:]
