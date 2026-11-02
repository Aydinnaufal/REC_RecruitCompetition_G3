"""controller_dashboard_V_6_0_1.py - Robot Station V 6.0: the tower robot simulation (Game), a monitor view of the real robot (Monitor), and the link to the ESP32.

    python -m pip install -r requirements_V_6_0.txt        (pygame, pyserial, bleak)
    python controller_dashboard_V_6_0_1.py

Everything it needs is in this folder (sim_, render_, ui_, input_, comms_ files). Space = E-STOP. The robot gets drive commands only after ARM OUTPUT.
Keys: Space E-STOP   F11 / F fullscreen   V next view   T drive mode   L turn on/off   0 re-zero   P pause timer   C cubes   X robot   Z all (reset)   ESC quit"""
import json, math, os, sys, textwrap, time
import pygame
import ui_V_6_0_1 as ui
import input_V_6_0_1 as inp
import sim_V_6_0_1 as sm
import render_V_6_0_1 as rd
import comms_V_6_0_1 as cm
from ui_V_6_0_1 import text, PANEL, TEXT, MUTED, ACCENT, BG

VERSION = 'V 6.0'
SETTINGS_FILE = os.path.join(os.path.expanduser('~'), '.rec_station_v60.json')
CW, CH = 1440, 860                                  # the design canvas, scaled to any window
GAME, MONITOR = 'game', 'monitor'
M, TOP, BOT = 12, 60, 832
LX, LW = M, 300                                     # left column
CX, CWD = LX + LW + M, 720                          # centre column
RX, RW = CX + CWD + M, 372                          # right column
VP = (CX + 12, 112, CWD - 24, 456)                  # the picture
X0, WW = RX + 12, RW - 24                           # inside the right column: cards start at X0 and are WW wide; widgets keep 10 px from the edge
TABS = [('link', 'LINK'), ('settings', 'SETTINGS')]
SOURCES = [('sim', 'Simulation'), ('robot', 'Robot (live)')]
TRANSPORTS = [('ble', 'Bluetooth LE'), ('serial', 'USB cable')]
STATE_COL = {cm.Link.CONNECTED: ui.OK, cm.Link.STALE: ui.WARN, cm.Link.HANDSHAKE: ui.WARN, cm.Link.CONNECTING: ui.WARN, cm.Link.NO_REPLY: ui.WARN, cm.Link.ERROR: ui.BAD}
AMBER = (238, 168, 58)
OFF_RED = (112, 42, 40)


# ================================================================================================ the app
class App:
    """Everything the screen shows and does: the simulator, mode and views, the timer, and the link to the robot."""

    def __init__(self, clock=time.monotonic, settings_path=SETTINGS_FILE):
        self.clock, self.settings_path = clock, settings_path
        self.sim, self.link, self.pose, self.look = sm.Robot(), cm.Link(clock), cm.TelemetryPose(), rd.Look()
        self.mode, self.view, self.tab, self.source = GAME, {GAME: 'arena', MONITOR: 'iso'}, 'link', 'sim'
        self.fullscreen = False
        self.timer_on, self.timer_state, self.timer_t = False, 'armed', 0.0       # states: armed (waits for any input), running, paused
        self.transport_kind, self.port, self.ports = 'ble', '', []
        self.ble_addr, self.ble_name, self.ble, self.ble_msg, self._ble_done = '', '', None, '', True
        self.load_settings()
        self.refresh_ports()

    # ---- remembered settings
    def load_settings(self):
        try:
            with open(self.settings_path) as f:
                d = json.load(f)
        except Exception:
            return
        if d.get('transport') in dict(TRANSPORTS): self.transport_kind = d['transport']
        for k in ('port', 'ble_addr', 'ble_name'):
            if isinstance(d.get(k), str): setattr(self, k, d[k])

    def save_settings(self):
        try:
            with open(self.settings_path, 'w') as f:
                json.dump(dict(transport=self.transport_kind, port=self.port, ble_addr=self.ble_addr, ble_name=self.ble_name), f)
        except Exception:
            pass

    # ---- mode / view
    def views(self): return rd.VIEWS_GAME if self.mode == GAME else rd.VIEWS_MONITOR

    def set_mode(self, m):
        if m != self.mode:
            self.mode = m
            self.sim.set_arena(m == GAME)               # Monitor: no arena, no cubes, no collisions
            if m == MONITOR: self.sim.reset_robot()

    def cur_view(self): return self.view[self.mode]

    def set_view(self, v): self.view[self.mode] = v

    def next_view(self):
        v = self.views()
        self.set_view(v[(v.index(self.cur_view()) + 1) % len(v)])

    def next_drive_mode(self): self.sim.mode = sm.MODES[(sm.MODES.index(self.sim.mode) + 1) % len(sm.MODES)]

    def in_scene(self, pos): return pos is not None and VP[0] <= pos[0] < VP[0] + VP[2] and VP[1] <= pos[1] < VP[1] + VP[3]

    def drag(self, dx, dy):
        if self.cur_view() != 'free': self.set_view('free')
        self.look.drag(dx, dy)

    def toggle_fullscreen(self): self.fullscreen = not self.fullscreen        # main() replaces this with the real window switch

    def zero(self):
        self.sim.zero()
        if self.link.telemetry:
            self.pose.zero(self.link.telemetry)
        self.pose.reset()

    # ---- timer: Timer shows it, it then waits and starts on the first touch of ANY control (button, D-pad, stick or trigger)
    def timer_toggle(self):
        self.timer_on = not self.timer_on
        self.timer_reset()

    def timer_reset(self): self.timer_state, self.timer_t = 'armed', 0.0

    def timer_restart(self): self.timer_state, self.timer_t = 'running', 0.0

    def timer_pause(self):
        if self.timer_on and self.timer_state in ('running', 'paused'):
            self.timer_state = 'paused' if self.timer_state == 'running' else 'running'

    def timer_update(self, dt, st):
        if not self.timer_on:
            return
        if self.timer_state == 'armed' and st.any_input():
            self.timer_state = 'running'
        if self.timer_state == 'running':
            self.timer_t += dt

    def timer_text(self): return '%d:%02d.%d' % (int(self.timer_t // 60), int(self.timer_t % 60), int((self.timer_t * 10) % 10))

    # ---- finding the robot
    def refresh_ports(self):
        self.ports = cm.list_serial_ports()
        names = [p.device for p in self.ports]
        if self.port not in names:
            self.port = names[0] if names else ''

    def scanning(self): return bool(self.ble and self.ble.running)

    def refresh(self):
        """Refresh / Scan: re-list the COM ports, or scan for Bluetooth LE devices."""
        if self.transport_kind == 'serial':
            self.refresh_ports()
        elif not self.scanning() and not self.link.active:
            self.ble = cm.BleScanner(5.0)
            self._ble_done, self.ble_msg = False, ''
            self.ble.start()

    def tick_scan(self):
        b = self.ble
        if not b or b.running or self._ble_done:
            return
        self._ble_done = True
        if b.error:
            self.ble_msg = b.error
            self.link.say('err', 'scan: ' + b.error)
            return
        robot = b.robot()
        if robot:
            self.ble_addr, self.ble_name = robot.address, robot.name
            self.ble_msg = 'found %s: press Connect' % robot.name
        else:
            self.ble_msg = '%d device(s), none named %s...: is the robot on and flashed with BT_MODE = BT_BLE?' % (len(b.devices), cm.ROBOT_NAME_PREFIX)
        self.link.say('sys', 'scan: ' + self.ble_msg)

    def target_options(self):
        if self.transport_kind == 'serial':
            return [(p.device, ('%s  %s' % (p.device, p.description))[:40]) for p in self.ports] or [('', 'no ports found')]
        opts = [(d.address, '%s %s' % (d.name or '?', '' if d.rssi is None else '%d dBm' % d.rssi)) for d in (self.ble.devices if self.ble else [])]
        if self.ble_addr and self.ble_addr not in [a for a, _ in opts]:
            opts.insert(0, (self.ble_addr, self.ble_name or self.ble_addr))      # the robot used last time, no scan needed
        return opts or [('', 'press Scan')]

    def target_value(self): return self.port if self.transport_kind == 'serial' else self.ble_addr

    def set_target(self, v):
        if self.transport_kind == 'serial':
            self.port = v
        else:
            self.ble_addr = v
            self.ble_name = next((d.name for d in (self.ble.devices if self.ble else []) if d.address == v), self.ble_name)

    def set_transport(self, k):
        self.transport_kind = k
        if k == 'serial': self.refresh_ports()

    # ---- the link
    def toggle_connection(self):
        if self.link.active:
            self.link.disconnect()
            return
        if not self.target_value():
            msg = 'no COM port: plug the USB cable in and press Refresh' if self.transport_kind == 'serial' else 'no device selected: press Scan, the robot is called REC-Robot'
            self.link.say('err', msg)
            self.link.last_error = msg
            self.link.state = cm.Link.ERROR
            return
        k = self.transport_kind
        self.link.connect(lambda: cm.BleTransport(self.ble_addr, self.ble_name) if k == 'ble' else cm.SerialTransport(self.port))
        self.pose = cm.TelemetryPose()
        self.save_settings()

    def toggle_arm(self):
        self.link.armed = not self.link.armed
        self.link.say('sys', 'output ARMED: the robot now receives drive commands' if self.link.armed else 'output disarmed: the robot receives zeros')

    def estop(self): self.link.estop()

    def hint(self):
        """One or two short lines telling the person what to do next."""
        l = self.link
        if self.scanning(): return 'scanning for Bluetooth LE devices... %d%%' % round(self.ble.progress() * 100)
        if l.state == cm.Link.CONNECTING: return 'opening the link... Bluetooth can take up to 10 seconds'
        if l.state == cm.Link.HANDSHAKE: return 'link open, asking the robot to say HELLO...'
        if l.state == cm.Link.NO_REPLY: return 'open but the robot is silent: powered on? V_5_0 firmware running?'
        if l.state == cm.Link.ERROR: return l.last_error
        if l.state == cm.Link.STALE: return 'nothing heard for a moment: out of range, or the robot restarted?'
        if l.state == cm.Link.DISCONNECTED:
            if self.transport_kind == 'serial':
                return 'press Connect' if self.port else 'no COM ports found: plug the USB cable in and press Refresh'
            return self.ble_msg or ('press Connect (robot: %s)' % (self.ble_name or self.ble_addr) if self.ble_addr else 'press Scan: the robot shows up as REC-Robot')
        return ''

    def command_from_sim(self):
        s, t = self.sim, self.sim.tower
        return cm.Command(s.vl * 100, s.vr * 100, t.pad_cmd * 100, t.lift_cmd * 100, True)

    def tick_link(self):
        self.link.set_command(self.command_from_sim())
        self.link.update()
        self.tick_scan()

    # ---- picture data
    def using_robot(self): return self.mode == MONITOR and self.source == 'robot'

    def telemetry_ok(self): return bool(self.link.telemetry) and self.link.connected and (self.link.telemetry_age() or 0) < 1.0

    def state(self):
        if not self.using_robot():
            return self.sim.snapshot()
        l = self.link
        tel = dict(l.telemetry) if l.telemetry and l.connected else dict(yaw=0.0, gz=0.0, vl=0, vr=0, grip=0, lift=0)
        age = l.telemetry_age()
        if age is not None and age > 0.5:
            tel['vl'] = tel['vr'] = 0
        return self.pose.snapshot(tel, self.clock())


# ================================================================================================ widgets
class Screen:
    """All widgets, in groups: top (always), mon (Monitor only), link and settings (one tab at a time)."""

    def __init__(self, app):
        self.top, self.mon, self.link, self.settings = ui.Group(), ui.Group(), ui.Group(), ui.Group()
        self.build(app)

    def active(self, app):
        return [self.top] + ([self.mon] if app.mode == MONITOR else []) + [self.link if app.tab == 'link' else self.settings]

    def motion(self, app, pos):
        for g in self.active(app): g.motion(pos)

    def click(self, app, pos):
        gs = self.active(app)
        for g in gs:                                                     # an open dropdown list takes the click first
            if any(d.open for d in g.of(ui.Dropdown)):
                return g.click(pos)
        return any([g.click(pos) for g in gs])

    def draw(self, app, surf, fonts):
        for g in self.active(app): g.draw_widgets(surf, fonts)
        for g in self.active(app): g.draw_lists(surf, fonts)

    def close_all(self, app):
        for g in self.active(app): g.close_all()

    def any_open(self, app): return any(d.open for g in self.active(app) for d in g.of(ui.Dropdown))

    def typing(self, app): return any(g.typing() for g in self.active(app))

    def key(self, app, e): return any([g.key(e) for g in self.active(app)])

    def drop_focus(self, app, pos):
        for g in self.active(app): g.drop_focus(pos)

    def build(self, app):
        sim, link, B = app.sim, app.link, ui.Button
        # ---- top bar
        self.top.add(B((250, 20, 92, 28), 'Game', lambda: app.set_mode(GAME), active=lambda: app.mode == GAME))
        self.top.add(B((346, 20, 92, 28), 'Monitor', lambda: app.set_mode(MONITOR), active=lambda: app.mode == MONITOR))
        self.top.add(B((456, 18, 250, 32), lambda: 'LINK  %s' % link.state, lambda: setattr(app, 'tab', 'link')))
        tx = CW - M - 176
        self.top.add(B((tx, 20, 54, 28), 'Timer', app.timer_toggle, active=lambda: app.timer_on))
        self.top.add(B((tx + 58, 20, 54, 28), 'Reset', app.timer_reset, enabled=lambda: app.timer_on))
        self.top.add(B((tx + 116, 20, 60, 28), 'Restart', app.timer_restart, enabled=lambda: app.timer_on))
        self.top.add(ui.Tabs((RX + 8, TOP + 8, RW - 16, 30), TABS, lambda: app.tab, lambda v: setattr(app, 'tab', v)))
        # ---- picture header
        vx = VP[0] + VP[2]
        app.dd_view = self.top.add(ui.Dropdown((vx - 176, TOP + 22, 176, 28), 'VIEW', [], app.cur_view, app.set_view))
        self.mon.add(ui.Dropdown((vx - 176 - 196, TOP + 22, 184, 28), 'PICTURE FROM', SOURCES, lambda: app.source, lambda v: setattr(app, 'source', v)))
        # ---- LINK tab (every widget keeps 10 px inside its card)
        L, idle = self.link, (lambda: not link.active)
        L.add(ui.Dropdown((X0 + 10, 146, 150, 28), 'CONNECT BY', TRANSPORTS, lambda: app.transport_kind, app.set_transport, enabled=idle))
        app.dd_target = L.add(ui.Dropdown((X0 + 170, 146, WW - 180, 28), 'DEVICE', [], app.target_value, app.set_target, enabled=idle))
        L.add(B((X0 + 10, 184, 156, 28), lambda: 'Disconnect' if link.active else 'Connect', app.toggle_connection, active=lambda: link.active, enabled=lambda: not app.scanning()))
        L.add(B((X0 + 172, 184, WW - 182, 28), lambda: 'Scan' if app.transport_kind == 'ble' else 'Refresh', app.refresh, enabled=lambda: idle() and not app.scanning()))
        L.add(B((X0 + 10, 418, 160, 28), lambda: 'OUTPUT ARMED' if link.armed else 'ARM OUTPUT', app.toggle_arm, active=lambda: link.armed))
        L.add(B((X0 + WW - 170, 418, 160, 28), 'E-STOP  (Space)', app.estop, danger=True))
        term = L.add(ui.TextInput((X0 + 10, 752, WW - 82, 26), lambda t: link.send_raw(t)))
        L.add(B((X0 + WW - 62, 752, 52, 26), 'Send', lambda: (term.text and (link.send_raw(term.text), setattr(term, 'text', ''))), enabled=lambda: link.connected))
        L.add(B((X0 + 10, 788, 82, 22), 'Clear', link.log.clear))
        # ---- SETTINGS tab
        S, look = self.settings, app.look
        S.add(ui.Dropdown((X0 + 10, 146, 160, 28), 'DRIVE MODE', [(k, sm.MODE_LABEL[k]) for k in sm.MODES], lambda: sim.mode, lambda v: setattr(sim, 'mode', v)))
        S.add(B((X0 + 180, 146, WW - 190, 28), lambda: 'Turn (LT): ON' if sim.turn_on else 'Turn (LT): OFF', lambda: setattr(sim, 'turn_on', not sim.turn_on), active=lambda: sim.turn_on))
        S.add(ui.Stepper((X0 + 10, 212, 152, 24), 'Trail', lambda: look.trail_on, lambda v: setattr(look, 'trail_on', v), lambda: look.trail_i, lambda v: setattr(look, 'trail_i', v)))
        S.add(ui.Stepper((X0 + 186, 212, 152, 24), 'Shadow', lambda: look.shadow_on, lambda v: setattr(look, 'shadow_on', v), lambda: look.shadow_i, lambda v: setattr(look, 'shadow_i', v)))
        S.add(ui.Stepper((X0 + 10, 246, 152, 24), 'Arrow', lambda: look.arrow_on, lambda v: setattr(look, 'arrow_on', v), lambda: look.arrow_i, lambda v: setattr(look, 'arrow_i', v)))
        S.add(B((X0 + 186, 246, 152, 24), 'Reset camera', look.reset_cam))
        S.add(B((X0 + 10, 316, 160, 28), 'Re-zero heading', app.zero))
        S.add(B((X0 + 178, 316, 160, 28), 'Reset cubes', sim.reset_cubes, enabled=lambda: app.mode == GAME))
        S.add(B((X0 + 10, 350, 160, 28), 'Reset robot', sim.reset_robot))
        S.add(B((X0 + 178, 350, 160, 28), 'Reset all', sim.reset))
        S.add(B((X0 + 10, 430, 164, 28), 'Fullscreen (F11)', app.toggle_fullscreen, active=lambda: app.fullscreen))


def refresh_dynamic(app):
    app.dd_view.options[:] = [(v, rd.VIEW_LABEL[v]) for v in app.views()]
    app.dd_target.title = 'COM PORT' if app.transport_kind == 'serial' else 'BLE DEVICE'
    app.dd_target.options[:] = app.target_options()


# ================================================================================================ drawing
def mmss(t): return '%d:%02d' % (int(t // 60), int(t % 60))


def wrap_hint(msg, width=44, lines=3):
    """At most `lines` lines; when the text is longer the last line ends in '...' so nothing is cut silently."""
    out = textwrap.wrap(msg or '', width)
    if len(out) > lines:
        out = out[:lines]
        out[-1] = out[-1][:width - 3].rstrip() + '...'
    return out


def draw_topbar(canvas, fonts, app):
    pygame.draw.rect(canvas, PANEL, (M, 6, CW - 2 * M, 48), border_radius=8)
    text(canvas, fonts['l'], 'Robot Station', (24, 10))
    text(canvas, fonts['s'], VERSION, (24, 38), MUTED)
    l = app.link
    ui.led(canvas, (730, 34), l.state != cm.Link.DISCONNECTED, STATE_COL.get(l.state, ui.OK))
    if l.connected and l.stats['rtt_avg'] is not None:
        text(canvas, fonts['s'], '%.0f ms' % l.stats['rtt_avg'], (870, 28), MUTED, 'tr')
    if l.armed:
        pygame.draw.rect(canvas, ui.DANGER, (890, 18, 92, 32), border_radius=8)
        text(canvas, fonts['s'], 'ARMED', (936, 34), (255, 220, 215), 'center')
    if app.timer_on:
        col = {'armed': (240, 200, 90), 'running': ACCENT, 'paused': (240, 150, 90)}[app.timer_state]
        text(canvas, fonts['l'], app.timer_text(), (CW - M - 188, 12), col, 'tr')
        text(canvas, fonts['s'], {'armed': 'move any control', 'running': 'running (P)', 'paused': 'paused (P)'}[app.timer_state], (CW - M - 188, 38), MUTED, 'tr')


def draw_left(canvas, fonts, big, st, log_lines):
    pygame.draw.rect(canvas, PANEL, (LX, TOP, LW, BOT - TOP), border_radius=8)
    text(canvas, fonts['m'], 'CONTROLLER', (LX + 12, TOP + 8), MUTED)
    text(canvas, fonts['s'], (st.name if st.connected else 'NOT FOUND')[:20], (LX + LW - 12, TOP + 10), ACCENT if st.connected else (240, 120, 100), 'tr')
    img = pygame.Surface((ui.CTRL_W, ui.CTRL_H))
    img.fill(PANEL)
    ui.draw_controller(img, big, 0, 0, st)
    w = LW - 20
    scaled = pygame.transform.smoothscale(img, (w, int(w * ui.CTRL_H / ui.CTRL_W)))
    canvas.blit(scaled, (LX + 10, TOP + 30))
    y = ui.raw_compact(canvas, fonts, LX + 12, TOP + 30 + scaled.get_height() + 8, LW - 24, st) + 4
    text(canvas, fonts['s'], 'EVENTS', (LX + 12, y), MUTED)
    for k, line in enumerate(log_lines[:max(2, (BOT - y - 26) // 15)]):
        text(canvas, fonts['s'], line[:40], (LX + 12, y + 18 + k * 15), TEXT if k == 0 else MUTED)


def draw_centre(canvas, fonts, cam, scene, app):
    sim = app.sim
    pygame.draw.rect(canvas, PANEL, (CX, TOP, CWD, BOT - TOP), border_radius=8)
    d = app.state()
    robot_live = d['source'] == 'robot'
    text(canvas, fonts['m'], 'GAME  |  tower robot' if app.mode == GAME else 'MONITOR  |  tower robot  |  %s' % ('robot' if robot_live else 'sim'),
         (CX + 14, TOP + 8), ACCENT if app.mode == MONITOR else MUTED)
    text(canvas, fonts['s'], 'drive: %s' % sm.MODE_LABEL[sim.mode], (CX + 14, TOP + 30), MUTED)
    if app.mode == GAME: rd.draw_scene(scene, cam, sim, app.cur_view(), app.look)
    else: rd.draw_monitor(scene, cam, d, app.cur_view(), app.look)
    canvas.blit(scene, (VP[0], VP[1]))
    if app.using_robot() and not app.telemetry_ok():
        pygame.draw.rect(canvas, (20, 23, 28), (VP[0] + VP[2] // 2 - 150, VP[1] + 14, 300, 28), border_radius=6)
        text(canvas, fonts['s'], 'NO TELEMETRY: connect on the LINK tab', (VP[0] + VP[2] // 2, VP[1] + 28), ui.WARN, 'center')
    y0 = VP[1] + VP[3] + 10
    ui.compass(canvas, fonts, CX + 110, y0 + 92, 78, d['heading'], math.degrees(d['omega']))
    x, w = CX + 232, CWD - 244
    ui.labelled_bar(canvas, fonts, x, y0 + 2, w, d['vl'], 'L wheel', '%+4.0f cm/s' % (d['vl'] * sm.MAX_SPEED), centre=True)
    ui.labelled_bar(canvas, fonts, x, y0 + 22, w, d['vr'], 'R wheel', '%+4.0f cm/s' % (d['vr'] * sm.MAX_SPEED), centre=True)
    ui.labelled_bar(canvas, fonts, x, y0 + 42, w, (d['lift'] - sm.LIFT_MIN) / (sm.LIFT_MAX - sm.LIFT_MIN), 'Box lift', '%4.1f cm' % d['lift'], color=(60, 200, 190))
    state = 'HOLDING cube' if d['holding'] else ('touching' if d['touching'] else 'free')
    ui.labelled_bar(canvas, fonts, x, y0 + 62, w, d['cmd'], 'Pad', '%3.0f%% %s' % (d['cmd'] * 100, state), color=AMBER, vw=150)
    ui.labelled_bar(canvas, fonts, x, y0 + 82, w, d['rt'], 'Speed RT', 'not reported' if robot_live else 'cap %2.0f cm/s' % (d['speed_scale'] * sm.MAX_SPEED), color=(90, 170, 240), vw=150)
    text(canvas, fonts['s'], 'speed %+5.1f cm/s     x %+7.1f   y %+7.1f' % (d['speed'], d['x'], d['y']), (x, y0 + 108), TEXT)
    # ---- square indicators: what the controller has switched, readable at a glance
    ty, gap = y0 + 128, 108
    if robot_live:                                                              # the real robot does not report these
        for i, lab in enumerate(('TURN  (LT)', 'SNAP', 'CUBE')):
            ui.tile(canvas, fonts, x + i * gap, ty, lab, False, '--')
    else:
        ui.tile(canvas, fonts, x, ty, 'TURN  (LT)', d['turn_on'], 'ON' if d['turn_on'] else 'OFF', ui.OK, OFF_RED)
        snap_on = d['mode'] == 'arcade_snap'
        ui.tile(canvas, fonts, x + gap, ty, 'SNAP', snap_on, ('SPIN' if d['snapping'] else 'READY') if snap_on else 'OFF', AMBER if d['snapping'] else ui.OK)
        ui.tile(canvas, fonts, x + 2 * gap, ty, 'CUBE', d['holding'] or d['touching'], 'HELD' if d['holding'] else ('TOUCH' if d['touching'] else 'NONE'), AMBER if d['touching'] and not d['holding'] else ui.OK)
    ui.tile(canvas, fonts, x + 3 * gap, ty, 'OUTPUT', app.link.armed, 'LIVE' if app.link.armed else 'SAFE', ui.BAD)


def draw_right(canvas, fonts, app):
    pygame.draw.rect(canvas, PANEL, (RX, TOP, RW, BOT - TOP), border_radius=8)
    if app.tab == 'link':
        draw_link(canvas, fonts, app)
    else:
        for (y, h, t) in ((104, 74, 'DRIVE'), (184, 96, 'VIEW OVERLAYS   (switch + opacity)'), (288, 98, 'ACTIONS'), (394, 68, 'WINDOW')):
            ui.card(canvas, (X0, y, WW, h), t, fonts)


def draw_link(canvas, fonts, app):
    l, s, x0, w = app.link, app.link.stats, X0, WW
    ui.card(canvas, (x0, 104, w, 116), 'CONNECTION', fonts)
    ui.card(canvas, (x0, 228, w, 156), 'LINK STATUS', fonts)
    ui.card(canvas, (x0, 392, w, 62), 'OUTPUT TO ROBOT', fonts)
    ui.card(canvas, (x0, 462, w, 116), 'ROBOT TELEMETRY', fonts)
    ui.card(canvas, (x0, 586, w, 242), 'MESSAGES  (type a message to the robot below)', fonts)
    col = STATE_COL.get(l.state, MUTED)
    ui.led(canvas, (x0 + 20, 262), l.state != cm.Link.DISCONNECTED, col, 7)
    text(canvas, fonts['m'], l.state, (x0 + 34, 254), col)
    ui.signal_bars(canvas, x0 + w - 40, 250, l.quality())
    text(canvas, fonts['s'], (l.transport.name if l.transport else '-')[:26], (x0 + w - 50, 254), MUTED, 'tr')
    now = l.clock()
    ui.led(canvas, (x0 + 22, 286), l.connected and s['tx_rate'] > 0 and int(now * 8) % 2 == 0, ui.OK)
    text(canvas, fonts['s'], 'TX %4.1f/s' % s['tx_rate'], (x0 + 32, 279), TEXT)
    ui.led(canvas, (x0 + 122, 286), l.connected and s['rx_rate'] > 0 and int(now * 8) % 2 == 0, (90, 170, 240))
    text(canvas, fonts['s'], 'RX %4.1f/s' % s['rx_rate'], (x0 + 132, 279), TEXT)
    text(canvas, fonts['s'], 'bad %d  loss %.1f%%' % (s['bad'], l.loss_percent()), (x0 + 224, 279), ui.BAD if s['bad'] else TEXT)
    if s['rtt'] is None:
        text(canvas, fonts['s'], 'latency: -', (x0 + 10, 298), MUTED)
    else:
        text(canvas, fonts['s'], 'latency %3.0f ms  avg %3.0f  min %3.0f  max %3.0f' % (s['rtt'], s['rtt_avg'], s['rtt_min'], s['rtt_max']), (x0 + 10, 298), TEXT)
    age = s['since_rx']
    text(canvas, fonts['s'], 'last data %s     up %s' % ('-' if age is None else '%.2f s ago' % age, mmss(l.uptime())), (x0 + 10, 316), MUTED)
    hcol = ui.BAD if l.state == cm.Link.ERROR else (ui.WARN if (app.scanning() or l.state in (cm.Link.NO_REPLY, cm.Link.STALE)) else MUTED)
    for i, ln in enumerate(wrap_hint(app.hint())):
        text(canvas, fonts['s'], ln, (x0 + 10, 338 + i * 14), hcol)
    text(canvas, fonts['s'], 'LIVE: robot gets drive commands' if l.armed else 'safe: robot gets zeros', (x0 + w - 10, 398), ui.BAD if l.armed else MUTED, 'tr')
    t, h = l.telemetry, l.hello
    text(canvas, fonts['s'], ('%s  %s  (%s)' % (h.get('fw', '?'), h.get('version', ''), h.get('robot', '?')))[:42] if h else 'no HELLO yet', (x0 + 10, 482), TEXT if h else MUTED)
    if t:
        low = bool(t['flags'] & cm.F_LOWBAT)
        ui.labelled_bar(canvas, fonts, x0 + 10, 502, w - 20, (t['vbat_mv'] - 6000) / 2400.0, 'Battery', '%.2f V' % (t['vbat_mv'] / 1000.0), ui.BAD if low else ui.OK, vw=72)
        text(canvas, fonts['s'], 'yaw %+7.1f deg   gyro %+6.1f deg/s' % (t['yaw'], t['gz']), (x0 + 10, 522), TEXT)
        cx = x0 + 12
        for bit, lab, c in ((cm.F_ARMED, 'ARMED', ui.BAD), (cm.F_FAILSAFE, 'FAILSAFE', ui.WARN), (cm.F_GYRO_OK, 'GYRO', ui.OK), (cm.F_GYRO_CAL, 'CAL', ui.WARN), (cm.F_LOWBAT, 'LOW BAT', ui.BAD)):
            on = bool(t['flags'] & bit)
            ui.led(canvas, (cx, 556), on, c, 5)
            text(canvas, fonts['s'], lab, (cx + 9, 549), TEXT if on else MUTED)
            cx += 20 + len(lab) * 7
    else:
        text(canvas, fonts['s'], 'no telemetry', (x0 + 10, 506), MUTED)
    colmap = {'tx': (120, 190, 255), 'rx': TEXT, 'err': ui.BAD, 'sys': MUTED}
    rows = []                                                    # newest messages, long ones wrapped onto a second line
    for ts, kind, msg in list(l.log)[-8:]:
        for j, part in enumerate(textwrap.wrap('%s %s %s' % (mmss(ts % 3600), {'tx': '>', 'rx': '<', 'err': '!', 'sys': '*'}[kind], msg), 44) or ['']):
            rows.append(((part if j == 0 else '     ' + part)[:46], colmap[kind]))
    for i, (line, c) in enumerate(rows[-8:]):
        text(canvas, fonts['s'], line, (x0 + 10, 608 + i * 16), c)


def render(canvas, fonts, big, cam, scene, app, S, st, log_lines):
    canvas.fill(BG)
    refresh_dynamic(app)
    draw_topbar(canvas, fonts, app)
    draw_left(canvas, fonts, big, st, log_lines)
    draw_centre(canvas, fonts, cam, scene, app)
    draw_right(canvas, fonts, app)
    text(canvas, fonts['s'], 'Space E-STOP   F11 fullscreen   V view   T drive mode   L turn   0 re-zero   P timer   C cubes   X robot   Z all   ESC quit', (M + 4, CH - 22), MUTED)
    S.draw(app, canvas, fonts)


def make_window(fullscreen, size):
    return pygame.display.set_mode((0, 0), pygame.FULLSCREEN) if fullscreen else pygame.display.set_mode(size, pygame.RESIZABLE)


# ================================================================================================ the program
def main():
    app = App()
    pygame.init()
    state = {'size': (1440, 860)}
    window = make_window(False, state['size'])
    pygame.display.set_caption('Robot Station ' + VERSION)
    fonts, big = ui.make_fonts()
    canvas = pygame.Surface((CW, CH))
    pad, elog = inp.XboxController(), inp.EventLog()
    scene, cam, drag = pygame.Surface((VP[2], VP[3])), rd.Camera(VP[2], VP[3], 400), {'on': False}

    def toggle_full():
        nonlocal window
        if not app.fullscreen: state['size'] = window.get_size()
        app.fullscreen = not app.fullscreen
        window = make_window(app.fullscreen, state['size'])

    app.toggle_fullscreen = toggle_full
    S = Screen(app)
    sim, link = app.sim, app.link

    def quit_app():
        link.disconnect()
        app.save_settings()

    clock = pygame.time.Clock()
    while True:
        dt = min(clock.tick(60) / 1000.0, 0.05)
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                quit_app(); return
            if e.type == pygame.VIDEORESIZE and not app.fullscreen:
                state['size'] = (max(480, e.w), max(300, e.h))
                window = pygame.display.set_mode(state['size'], pygame.RESIZABLE)
            elif e.type == pygame.MOUSEMOTION:
                S.motion(app, ui.to_canvas(e.pos, window.get_size(), CW, CH))
                if drag['on']:
                    k = ui.fit_scale(window.get_width(), window.get_height(), CW, CH)[0]
                    app.drag(e.rel[0] / k, e.rel[1] / k)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                cp = ui.to_canvas(e.pos, window.get_size(), CW, CH)
                if not S.click(app, cp) and app.in_scene(cp):
                    drag['on'] = True
                S.drop_focus(app, cp)
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                drag['on'] = False
            elif e.type == pygame.MOUSEWHEEL:
                if app.in_scene(ui.to_canvas(pygame.mouse.get_pos(), window.get_size(), CW, CH)): app.look.wheel(e.y)
            elif e.type == pygame.KEYDOWN:
                if S.typing(app):
                    S.key(app, e)
                    continue
                k = e.key
                if k == pygame.K_ESCAPE:
                    if S.any_open(app): S.close_all(app)
                    elif app.fullscreen: toggle_full()
                    else:
                        quit_app(); return
                elif k == pygame.K_SPACE: app.estop()
                elif k in (pygame.K_F11, pygame.K_f): toggle_full()
                elif k == pygame.K_v: app.next_view()
                elif k == pygame.K_t: app.next_drive_mode()
                elif k == pygame.K_p: app.timer_pause()
                elif k == pygame.K_l: sim.turn_on = not sim.turn_on
                elif k == pygame.K_0: app.zero()
                elif k == pygame.K_z: sim.reset()
                elif k == pygame.K_x: sim.reset_robot()
                elif k == pygame.K_c and app.mode == GAME: sim.reset_cubes()
        st = pad.poll()
        sim.drive(st, dt)
        app.tick_link()
        elog.update(st)
        app.timer_update(dt, st)
        render(canvas, fonts, big, cam, scene, app, S, st, elog.lines)
        ui.present(window, canvas)
        pygame.display.flip()


if __name__ == '__main__':
    main()
