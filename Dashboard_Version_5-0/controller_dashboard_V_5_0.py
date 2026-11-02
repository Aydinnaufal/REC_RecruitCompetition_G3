"""
controller_dashboard_V_5_0.py - robot station V 5.0: V 4.1 with a connection you can diagnose and three ways to reach the ESP32.

    python -m pip install -r requirements_V_5_0.txt        (pygame, pyserial, bleak)
    python controller_dashboard_V_5_0.py            # real controller
    python controller_dashboard_V_5_0.py --demo     # fake controller input

What is new compared with V 4.1 (the communication code is in comms_V_5_0.py):
    * DOCTOR tab: checks pyserial, COM ports, paired Bluetooth devices (with MAC) and bleak, and lists what to do next. Saves a report file.
    * Transport "Bluetooth LE (scan)": press Refresh, the robot (REC-Robot) shows up by name, press Connect. No Windows pairing, no COM port.
    * Transport "Bluetooth Classic (MAC)": connects straight to a paired robot's address, no COM port needed.
    * Transport "USB / Bluetooth COM": the port list now explains an empty list, reads Windows' registry as a second source, marks
      Bluetooth OUTGOING / INCOMING ports (the Incoming one is never used) and accepts a port typed by hand (DOCTOR tab).
    * Folder per version: this whole program lives in Dashboard_Version_5-0 and needs nothing from the older folders.
Everything else (game, monitor, controls, settings, timer) is as in V 4.1. Space = E-STOP. The robot is NEVER sent drive commands until you press ARM OUTPUT.
"""
import json, math, os, sys, textwrap, threading, time
import pygame
import xbox_input as xi
import controller_viewer as cv
import controller_dashboard as cd
import controller_dashboard_v3_2 as d32
import dash_ui as ui
import dash_ui_V_4_0 as u4
import comms_V_5_0 as cm
import robot_sim3 as r3
from controller_viewer import text, BG, PANEL, TEXT, MUTED, ACCENT
from robot_sim import Camera, MAX_SPEED
from robot_sim3 import MODES, MODE_LABEL, ROBOTS, ROBOT_LABEL, VIEW_LABEL, draw_scene3, draw_monitor3

VERSION = 'V 5.0'
SETTINGS_FILE = os.path.join(os.path.expanduser('~'), '.rec_station_v50.json')
CW, CH = 1440, 860                                  # the design canvas, scaled to any window
GAME, MONITOR = d32.GAME, d32.MONITOR
M = 12
TOP, BOT = 60, 832
LX, LW = M, 300                                     # left column
CX, CWD = LX + LW + M, 720                          # centre column
RX, RW = CX + CWD + M, 372                          # right column
VP = (CX + 12, 112, CWD - 24, 456)                  # the viewport
WARN = (240, 150, 90)
TABS = [('link', 'LINK'), ('doctor', 'DOCTOR'), ('settings', 'SETTINGS'), ('help', 'HELP')]
SOURCES = [('sim', 'Simulation (this PC)'), ('robot', 'Robot (telemetry)')]
TRANSPORTS = [('virtual', 'Virtual robot (test)'), ('serial', 'USB / Bluetooth COM'), ('ble', 'Bluetooth LE (scan)'), ('rfcomm', 'Bluetooth Classic (MAC)')]
KINDS = [k for k, _ in TRANSPORTS]
LEVEL_COL = {'ok': u4.OK, 'warn': u4.WARN, 'bad': u4.BAD, 'info': MUTED}
STATE_COL = {cm.Link.CONNECTED: u4.OK, cm.Link.STALE: u4.WARN, cm.Link.HANDSHAKE: u4.WARN, cm.Link.CONNECTING: u4.WARN, cm.Link.NO_REPLY: u4.WARN, cm.Link.ERROR: u4.BAD}
HELP = ['LEFT STICK Y   forward / back', 'LEFT STICK X   turn (when Turn is ON)', 'LT             Turn ON / OFF toggle',
        'RT             speed 20 cm/s, full = 30', 'D-PAD          slow nudge fwd / back / turn', 'RIGHT STICK X  gripper: right = close',
        '               centre / left = open', 'RIGHT STICK Y  tower box up / down', 'LB / RB        snap-turn 90 (Arcade+Snap)',
        'Y B A X        face up/right/down/left', 'START          new zero heading', '',
        'MOUSE          drag = look, wheel = zoom', 'SPACE          E-STOP (disarm + stop)', 'F11 / F        fullscreen',
        'V view  T drive mode  0 re-zero', 'P pause timer  L turn on/off', 'C cubes  X robot  Z all  (reset)',
        'D demo  M learn  S save  R default', '', 'Robot gets commands only after', 'ARM OUTPUT (LINK tab).']


# ================================================================================================ the app
class App50(d32.App):
    def __init__(self, robot='simple', team='blue', clock=time.monotonic, settings_path=SETTINGS_FILE):
        super().__init__(robot, team)
        self.clock = clock
        self.settings_path = settings_path
        self.link = cm.Link(clock)
        self.pose = cm.TelemetryPose(robot)
        self.tab = 'link'
        self.source = 'sim'                            # monitor mode picture: the simulation or the robot's telemetry
        self.transport_kind = 'virtual'
        self.baud = 115200
        self.profile = 'rec1'
        self.port = ''
        self.ports = []
        self.virt_loss = 0.0
        self.scanner = None
        self.scan_msg = ''
        self._scan_done = True
        # V 5.0: Bluetooth LE, Bluetooth Classic by address, the Port Doctor
        self.ble_addr, self.ble_name = '', ''
        self.ble_devices, self.ble = [], None
        self.ble_msg, self._ble_done, self._ble_auto = '', True, False
        self.mac, self.mac_name = '', ''
        self.paired = []
        self.report, self.doctor_running, self.doctor_note = None, False, ''
        self.manual_port = ''
        self.report_dir = os.path.dirname(os.path.abspath(__file__))
        self.load_settings()
        self.refresh_ports()
        self.start_doctor()

    # ---- remembered settings
    def load_settings(self):
        try:
            with open(self.settings_path) as f:
                d = json.load(f)
        except Exception:
            return
        if d.get('transport') in KINDS: self.transport_kind = d['transport']
        if d.get('baud') in cm.BAUDS: self.baud = d['baud']
        if d.get('profile') in cm.PROFILES: self.profile = d['profile']
        for k in ('port', 'manual_port', 'ble_addr', 'ble_name', 'mac', 'mac_name'):
            if isinstance(d.get(k), str): setattr(self, k, d[k])
        self.link.auto_reconnect = bool(d.get('auto_retry', False))

    def save_settings(self):
        try:
            with open(self.settings_path, 'w') as f:
                json.dump(dict(transport=self.transport_kind, baud=self.baud, profile=self.profile, port=self.port, manual_port=self.manual_port, ble_addr=self.ble_addr,
                               ble_name=self.ble_name, mac=self.mac, mac_name=self.mac_name, auto_retry=self.link.auto_reconnect), f)
        except Exception:
            pass

    # ---- viewport
    def scene_rect(self): return VP

    # ---- COM ports
    def refresh_ports(self):
        self.ports = cm.list_serial_ports()
        names = [p.device for p in self.ports]
        if self.port and self.port not in names and self.port == self.manual_port:
            return                                     # a port typed by hand stays even when the system does not list it
        if self.port not in names:
            usable = [p.device for p in self.ports if p.role != 'in']
            self.port = usable[0] if usable else (names[0] if names else '')

    def port_info(self, device=None):
        device = self.port if device is None else device
        for p in self.ports:
            if p.device == device: return p
        return None

    def port_is_bt(self):
        p = self.port_info()
        return bool(p and p.bluetooth)

    def port_label(self, p):
        if p.role == 'out':
            name = next((d.name for d in self.paired if d.mac == p.mac), '')
            return '%s  %s' % (p.device, (name or 'BT out')[:13])
        if p.role == 'in': return '%s  BT in (skip)' % p.device
        return '%s  %s' % (p.device, 'Bluetooth' if p.bluetooth else p.description[:13])

    def target_value(self):
        return {'serial': self.port, 'ble': self.ble_addr, 'rfcomm': self.mac}.get(self.transport_kind, '')

    def set_target(self, v):
        k = self.transport_kind
        if k == 'serial': self.port = v
        elif k == 'ble':
            self.ble_addr = v
            self.ble_name = next((d.name for d in self.ble_devices if d.address == v), self.ble_name)
        elif k == 'rfcomm':
            self.mac = v
            self.mac_name = next((d.name for d in self.paired if d.mac == v), '')

    def use_manual_port(self, txt):
        t = (txt or '').strip().upper()
        if t.isdigit(): t = 'COM' + t
        if not t:
            return
        self.manual_port, self.port, self.transport_kind = t, t, 'serial'
        if not any(p.device.upper() == t for p in self.ports):
            self.ports = self.ports + [cm.PortInfo(t, 'typed by hand', '', False, '', '')]
        self.doctor_note = 'port %s selected (LINK tab > Connect)' % t
        self.save_settings()

    def use_manual_mac(self, txt):
        if not cm.valid_mac(txt):
            self.doctor_note = 'that is not a Bluetooth address: use AA:BB:CC:DD:EE:FF'
            return
        self.mac, self.mac_name, self.transport_kind = txt.strip().upper().replace('-', ':'), '', 'rfcomm'
        self.doctor_note = 'address %s selected (LINK tab > Connect)' % self.mac
        self.save_settings()

    # ---- Port Doctor
    def start_doctor(self):
        if self.doctor_running:
            return
        self.doctor_running = True

        def work():
            try:
                rep = cm.diagnose()
                self.report = rep
                self.paired = [d for d in rep.paired if d.kind == 'classic']
                if self.mac and not self.mac_name:
                    self.mac_name = next((d.name for d in self.paired if d.mac == self.mac), '')
                if not self.mac:
                    r = next((d for d in self.paired if d.name.upper().startswith(cm.ROBOT_NAME_PREFIX)), None)
                    if r: self.mac, self.mac_name = r.mac, r.name
            except Exception as e:
                rep = cm.Report()
                rep.add('bad', 'the check itself failed: %s' % e)
                self.report = rep
            finally:
                self.doctor_running = False
        threading.Thread(target=work, daemon=True).start()

    def save_report(self):
        if not self.report:
            return
        path = os.path.join(self.report_dir, 'port_report_V_5_0.txt')
        try:
            with open(path, 'w', encoding='utf-8') as f:
                f.write(self.report.text())
            self.doctor_note = 'saved: ' + path
        except Exception as e:
            self.doctor_note = 'could not save the report: %s' % e

    # ---- Bluetooth LE scan
    def scanning_ble(self): return bool(self.ble and self.ble.running)

    def start_ble_scan(self, auto=False):
        if self.scanning_ble() or self.link.active:
            return
        self.ble = cm.BleScanner(5.0)
        self._ble_done, self._ble_auto, self.ble_msg = False, auto, ''
        self.ble.start()

    def tick_ble(self):
        b = self.ble
        if not b or b.running or self._ble_done:
            return
        self._ble_done = True
        self.ble_devices = b.devices
        if b.error:
            self.ble_msg = b.error
            self.link.say('err', 'BLE scan: ' + b.error)
            return
        robot = b.robot()
        if robot:
            self.ble_addr, self.ble_name = robot.address, robot.name
        elif b.devices and self.ble_addr not in [d.address for d in b.devices]:
            self.ble_addr, self.ble_name = '', ''
        self.ble_msg = ('found %s' % robot.name) if robot else ('%d device(s), none named %s...: is the robot on and flashed with BT_MODE = BT_BLE?' % (len(b.devices), cm.ROBOT_NAME_PREFIX))
        self.link.say('sys', 'BLE scan: ' + self.ble_msg)
        if robot and self._ble_auto:
            self.transport_kind = 'ble'
            self.toggle_connection()

    def refresh(self):
        k = self.transport_kind
        if k == 'serial': self.refresh_ports()
        elif k == 'ble': self.start_ble_scan()
        elif k == 'rfcomm': self.start_doctor()

    def busy(self): return self.scanning() or self.scanning_ble()

    # ---- link
    def target_ready(self):
        k = self.transport_kind
        if k == 'serial': return bool(self.port)
        if k == 'ble': return bool(self.ble_addr)
        if k == 'rfcomm': return cm.valid_mac(self.mac)
        return True

    def make_transport(self):
        k = self.transport_kind
        if k == 'virtual':
            return cm.VirtualRobotTransport(self.sim.robot_type, self.clock, loss=self.virt_loss)
        if k == 'ble':
            return cm.BleTransport(self.ble_addr, self.ble_name)
        if k == 'rfcomm':
            return cm.RfcommTransport(self.mac, self.mac_name)
        return cm.SerialTransport(self.port, self.baud, bluetooth=self.port_is_bt())

    def toggle_connection(self):
        if self.scanner and self.scanner.running:
            self.scanner.cancel()
        if self.link.active:
            self.link.disconnect()
            return
        if not self.target_ready():
            k = self.transport_kind
            msg = {'serial': 'no COM port selected: open the DOCTOR tab to see why, or press Auto-find',
                   'ble': 'no Bluetooth LE device selected: press Refresh to scan, the robot is called REC-Robot',
                   'rfcomm': 'no Bluetooth address: open the DOCTOR tab and type it, or pair the robot first'}[k]
            self.link.say('err', msg)
            self.link.last_error = msg
            return
        self.scan_msg = ''
        self.link.connect(self.make_transport, self.profile)
        self.pose = cm.TelemetryPose(self.sim.robot_type)
        self.save_settings()

    def scanning(self): return bool(self.scanner and self.scanner.running)

    def toggle_scan(self):
        if self.scanning():
            self.scanner.cancel()
            return
        if self.scanning_ble():
            return
        if self.link.active:
            return
        if self.transport_kind == 'ble':
            self.start_ble_scan(auto=True)
            return
        self.refresh_ports()
        self.scan_msg = ''
        self.scanner = cm.PortScanner(self.ports)
        self._scan_done = False
        self.scanner.start()

    def tick_scan(self):
        self.tick_ble()
        sc = self.scanner
        if not sc or sc.running or self._scan_done:
            return
        self._scan_done = True
        self.scan_msg = sc.message
        self.link.say('sys', 'auto-find: ' + sc.message)
        if sc.found:
            p, info = sc.found
            self.transport_kind, self.port, self.profile, self.baud = 'serial', p.device, 'rec1', 115200
            self.refresh_ports()
            self.toggle_connection()

    def toggle_arm(self):
        self.link.armed = not self.link.armed
        self.link.say('sys', 'output ARMED: the robot now receives drive commands' if self.link.armed else 'output disarmed: the robot receives zeros')

    def estop(self): self.link.estop()

    def virtual(self):
        t = self.link.transport
        return t if isinstance(t, cm.VirtualRobotTransport) else None

    def cut_cable(self):
        v = self.virtual()
        if v: v.cut = not v.cut; self.link.say('sys', 'virtual cable %s' % ('UNPLUGGED' if v.cut else 'plugged in'))

    def lossy(self):
        v = self.virtual()
        if v:
            v.loss = 0.0 if v.loss else 0.25
            self.virt_loss = v.loss
            self.link.say('sys', 'virtual link loss %d%%' % round(v.loss * 100))

    def zero_gyro(self):
        self.link.send_frame('ZERO')
        self.pose.reset()

    def toggle_retry(self):
        self.link.auto_reconnect = not self.link.auto_reconnect
        self.save_settings()

    def hint(self):
        """One or two short lines telling the person what to do next."""
        l = self.link
        k = self.transport_kind
        if self.scanning(): return self.scanner.message
        if self.scanning_ble(): return 'scanning for Bluetooth LE devices... %d%%' % round(self.ble.progress() * 100)
        if l.state == cm.Link.CONNECTING: return 'opening the link... Bluetooth can take up to 10 seconds'
        if l.state == cm.Link.HANDSHAKE: return 'link open, asking the robot to say HELLO...'
        if l.state == cm.Link.NO_REPLY:
            return 'open but the robot is silent: powered on? V_5_0 sketch running? (old sketch: use Legacy)'
        if l.state == cm.Link.ERROR: return l.last_error
        if l.state == cm.Link.STALE: return 'nothing heard for a moment: out of range, or the robot restarted?'
        if l.state == cm.Link.DISCONNECTED:
            if self.scan_msg and k == 'serial': return self.scan_msg
            if k == 'virtual': return 'Virtual robot: a pretend ESP32 in this program, no hardware needed'
            if k == 'serial':
                if not self.ports:
                    sc = cm.LAST_SCAN
                    if any('pyserial' in p for p in sc.problems): return 'pyserial is missing: open the DOCTOR tab, it shows the install command'
                    return 'no COM ports found: plug the USB cable in or pair the robot. DOCTOR tab explains'
                p = self.port_info()
                if p and p.role == 'in': return 'that is the Incoming Bluetooth port and cannot connect: pick the Outgoing one'
                if self.port_is_bt(): return 'Bluetooth port. If Connect fails press Auto-find, or try Bluetooth LE'
                return 'Pick the COM port of the board, or press Auto-find'
            if k == 'ble':
                if self.ble_msg: return self.ble_msg
                return 'press Refresh to scan: the robot shows up as REC-Robot' if not self.ble_addr else 'selected %s: press Connect' % (self.ble_name or self.ble_addr)
            if k == 'rfcomm':
                if not cm.valid_mac(self.mac): return 'no address yet: pair the robot in Windows, press Refresh, or type it in the DOCTOR tab'
                return 'robot %s at %s: press Connect (it must be paired)' % (self.mac_name or '', self.mac)
        return ''

    def command_from_sim(self):
        """What the robot should do now = what the simulated robot is doing (same stick maths, same speed limits)."""
        s, g = self.sim, self.sim.grip
        if g.kind == 'claw':
            return cm.Command(s.vl * 100, s.vr * 100, g.cmd * 100, 0, True)
        return cm.Command(s.vl * 100, s.vr * 100, g.pad_cmd * 100, g.lift_cmd * 100, True)

    def tick_link(self):
        self.link.set_command(self.command_from_sim())
        self.link.update()
        self.tick_scan()

    # ---- picture data
    def using_robot(self): return self.mode == MONITOR and self.source == 'robot'

    def state(self):
        if not self.using_robot():
            return self.sim.snapshot()
        l = self.link
        tel = dict(l.telemetry) if l.telemetry and l.connected else dict(yaw=0.0, gz=0.0, vl=0, vr=0, grip=0, lift=0)
        age = l.telemetry_age()
        if age is not None and age > 0.5:
            tel['vl'] = tel['vr'] = 0
        self.pose.type = self.sim.robot_type
        return self.pose.snapshot(tel, self.clock())

    def telemetry_ok(self): return bool(self.link.telemetry) and self.link.connected and (self.link.telemetry_age() or 0) < 1.0


# ================================================================================================ widgets
class UiSet:
    """Five sets of widgets: the top bar, the viewport header, one for each right-hand tab. Only the visible ones get events."""

    def __init__(self):
        self.top, self.mon, self.link, self.settings, self.doctor = u4.UI4(), u4.UI4(), u4.UI4(), u4.UI4(), u4.UI4()

    def active(self, app):
        out = []
        if app.tab == 'link': out.append(self.link)
        elif app.tab == 'settings': out.append(self.settings)
        elif app.tab == 'doctor': out.append(self.doctor)
        if app.mode == MONITOR: out.append(self.mon)
        out.append(self.top)
        return out

    def click(self, app, pos):
        act = self.active(app)
        for u in act:                                   # an open dropdown list takes the click first
            if any(d.open for d in u.dropdowns):
                u.click(pos)
                return True
        for u in act:
            if u.click(pos):
                return True
        return False

    def motion(self, app, pos):
        for u in self.active(app): u.motion(pos)

    def draw(self, app, surf, fonts):
        for u in self.active(app): u.draw(surf, fonts)

    def close_all(self):
        for u in (self.top, self.mon, self.link, self.settings, self.doctor): u.close_all()

    def any_open(self):
        return any(d.open for u in (self.top, self.mon, self.link, self.settings, self.doctor) for d in u.dropdowns)

    def typing(self, app): return any(u.keys_captured() for u in self.active(app))

    def key(self, app, event):
        for u in self.active(app):
            if u.key(event): return True
        return False

    def drop_focus(self, app):
        """Text boxes on a tab that is not showing must not keep the keyboard."""
        shown = self.active(app)
        for u in (self.link, self.doctor):
            if u not in shown:
                for t in u.inputs: t.focus = False


class OnlyIf(ui.Button):
    """A button that is only there (drawn and clickable) while show() is true."""

    def __init__(self, show, *a, **k):
        super().__init__(*a, **k)
        self.show = show

    def click(self, pos): return self.show() and super().click(pos)

    def draw(self, surf, fonts):
        if self.show(): super().draw(surf, fonts)


def build_ui(app, actions):
    S = UiSet()
    sim, link = app.sim, app.link
    B = ui.Button
    # ---- top bar
    S.top.add(B((250, 20, 92, 28), 'Game', lambda: app.set_mode(GAME), active=lambda: app.mode == GAME))
    S.top.add(B((346, 20, 92, 28), 'Monitor', lambda: app.set_mode(MONITOR), active=lambda: app.mode == MONITOR))
    app.dd_robot = S.top.add(ui.Dropdown((456, 20, 190, 28), 'ROBOT', [(k, ROBOT_LABEL[k]) for k in ROBOTS], lambda: sim.robot_type, sim.set_robot))
    app.dd_team = S.top.add(ui.Dropdown((658, 20, 96, 28), 'TEAM', [('blue', 'Blue'), ('red', 'Red')], lambda: sim.team, sim.set_team, enabled=lambda: app.mode == GAME))
    S.top.add(B((772, 18, 250, 32), lambda: 'LINK  %s' % link.state, lambda: setattr(app, 'tab', 'link'), tip='open the link tab'))
    tx = CW - M - 176
    S.top.add(B((tx, 20, 54, 28), 'Timer', app.timer_toggle, active=lambda: app.timer_on))
    S.top.add(B((tx + 58, 20, 54, 28), 'Reset', app.timer_reset, enabled=lambda: app.timer_on))
    S.top.add(B((tx + 116, 20, 60, 28), 'Restart', app.timer_restart, enabled=lambda: app.timer_on))
    S.top.add(u4.Tabs((RX + 8, TOP + 8, RW - 16, 30), TABS, lambda: app.tab, lambda v: setattr(app, 'tab', v)))
    # ---- viewport header
    vx = VP[0] + VP[2]
    app.dd_view = S.top.add(ui.Dropdown((vx - 176, TOP + 22, 176, 28), 'VIEW', [(v, VIEW_LABEL[v]) for v in r3.VIEWS3], app.cur_view, app.set_view))
    app.dd_source = S.mon.add(ui.Dropdown((vx - 176 - 196, TOP + 22, 184, 28), 'PICTURE FROM', SOURCES, lambda: app.source, lambda v: setattr(app, 'source', v)))
    # ---- LINK tab
    x0, w = RX + 12, RW - 24
    y = 104
    idle = lambda: not link.active                      # settings can only change while there is no connection
    app.dd_transport = S.link.add(ui.Dropdown((x0, y + 42, 170, 28), 'TRANSPORT', TRANSPORTS, lambda: app.transport_kind, lambda v: setattr(app, 'transport_kind', v), enabled=idle))
    app.dd_port = S.link.add(ui.Dropdown((x0 + 182, y + 42, 170, 28), 'PORT', [('', 'no ports found')], lambda: app.target_value(), app.set_target,
                                         enabled=lambda: app.transport_kind in ('serial', 'ble', 'rfcomm') and not link.active))
    app.dd_baud = S.link.add(ui.Dropdown((x0, y + 96, 96, 28), 'BAUD', [(b, str(b)) for b in cm.BAUDS], lambda: app.baud, lambda v: setattr(app, 'baud', v),
                                         enabled=lambda: app.transport_kind == 'serial' and not link.active))
    app.dd_profile = S.link.add(ui.Dropdown((x0 + 108, y + 96, 244, 28), 'PROTOCOL', list(cm.PROFILES.items()), lambda: app.profile, lambda v: setattr(app, 'profile', v), enabled=idle))
    yr = y + 132
    S.link.add(B((x0, yr, 100, 28), lambda: 'Disconnect' if link.active else 'Connect', app.toggle_connection, active=lambda: link.active, enabled=lambda: not app.busy()))
    S.link.add(B((x0 + 108, yr, 70, 28), lambda: 'Scan' if app.transport_kind == 'ble' else 'Refresh', app.refresh,
                 enabled=lambda: idle() and not app.busy() and app.transport_kind != 'virtual'))
    S.link.add(B((x0 + 186, yr, 96, 28), lambda: 'Stop scan' if app.scanning() else 'Auto-find', app.toggle_scan, active=app.busy,
                 enabled=lambda: app.scanning() or (idle() and not app.scanning_ble() and app.transport_kind in ('serial', 'ble')),
                 tip='find the robot and connect to it'))
    S.link.add(B((x0 + 290, yr, 58, 28), 'Retry', app.toggle_retry, active=lambda: link.auto_reconnect, tip='reconnect by itself after an error'))
    yb = 278
    virt = lambda: app.virtual() is not None
    S.link.add(OnlyIf(lambda: app.transport_kind == 'virtual', (x0 + 8, yb + 128, 100, 20), 'Cut cable', app.cut_cable, enabled=virt, active=lambda: bool(app.virtual() and app.virtual().cut)))
    S.link.add(OnlyIf(lambda: app.transport_kind == 'virtual', (x0 + 114, yb + 128, 100, 20), 'Lossy link', app.lossy, enabled=virt, active=lambda: bool(app.virtual() and app.virtual().loss)))
    yc = 438
    S.link.add(B((x0, yc + 26, 170, 28), lambda: 'OUTPUT ARMED' if link.armed else 'ARM OUTPUT', app.toggle_arm, active=lambda: link.armed))
    estop = S.link.add(B((x0 + 182, yc + 26, 170, 28), 'E-STOP  (Space)', app.estop))
    estop.danger = True
    ye = 598
    term_in = S.link.add(u4.TextInput((x0, ye + 172, w - 62, 26), lambda t: link.send_raw(t)))
    S.link.add(B((x0 + w - 56, ye + 172, 56, 26), 'Send', lambda: (term_in.text and (link.send_raw(term_in.text), setattr(term_in, 'text', ''))), enabled=lambda: link.connected))
    qx = x0
    for lab, fn in (('HELLO?', lambda: link.send_frame('HELLO?')), ('PING', lambda: link.send_frame('PING', 0)), ('Zero gyro', app.zero_gyro), ('Clear', link.log.clear)):
        S.link.add(B((qx, ye + 204, 82, 22), lab, fn, enabled=(lambda: link.connected) if lab != 'Clear' else None))
        qx += 90
    # ---- DOCTOR tab
    S.doctor.add(B((x0 + 8, 342, 104, 26), lambda: 'Checking...' if app.doctor_running else 'Run check', app.start_doctor, enabled=lambda: not app.doctor_running))
    S.doctor.add(B((x0 + 120, 342, 120, 26), 'Save report', app.save_report, enabled=lambda: app.report is not None))
    com_in = S.doctor.add(u4.TextInput((x0 + 8, 706, 214, 26), app.use_manual_port, hint='COM port, e.g. COM7'))
    S.doctor.add(B((x0 + 230, 706, 106, 26), 'Use port', lambda: (app.use_manual_port(com_in.text), setattr(com_in, 'text', ''))))
    mac_in = S.doctor.add(u4.TextInput((x0 + 8, 770, 214, 26), app.use_manual_mac, hint='AA:BB:CC:DD:EE:FF'))
    S.doctor.add(B((x0 + 230, 770, 106, 26), 'Use address', lambda: (app.use_manual_mac(mac_in.text), setattr(mac_in, 'text', ''))))
    # ---- SETTINGS tab
    y = 104
    S.settings.add(ui.Dropdown((x0, y + 42, 170, 28), 'DRIVE MODE', [(k, MODE_LABEL[k]) for k in MODES], lambda: sim.mode, lambda v: setattr(sim, 'mode', v)))
    S.settings.add(B((x0 + 182, y + 42, 170, 28), lambda: 'Turn (LT): ON' if sim.turn_on else 'Turn (LT): OFF', lambda: setattr(sim, 'turn_on', not sim.turn_on), active=lambda: sim.turn_on))
    L = app.look
    y = 184
    S.settings.add(ui.Stepper((x0, y + 28, 160, 24), 'Trail', lambda: L.trail_on, lambda v: setattr(L, 'trail_on', v), lambda: L.trail_i, lambda v: setattr(L, 'trail_i', v)))
    S.settings.add(ui.Stepper((x0 + 186, y + 28, 160, 24), 'Shadow', lambda: L.shadow_on, lambda v: setattr(L, 'shadow_on', v), lambda: L.shadow_i, lambda v: setattr(L, 'shadow_i', v)))
    S.settings.add(ui.Stepper((x0, y + 60, 160, 24), 'Arrow', lambda: L.arrow_on, lambda v: setattr(L, 'arrow_on', v), lambda: L.arrow_i, lambda v: setattr(L, 'arrow_i', v)))
    S.settings.add(B((x0 + 186, y + 60, 160, 24), 'Reset camera', app.reset_cam))
    y = 288
    S.settings.add(B((x0, y + 28, 170, 28), 'Re-zero heading', sim.zero))
    S.settings.add(B((x0 + 182, y + 28, 170, 28), 'Reset cubes', sim.reset_cubes, enabled=lambda: app.mode == GAME))
    S.settings.add(B((x0, y + 62, 170, 28), 'Reset robot', sim.reset_robot))
    S.settings.add(B((x0 + 182, y + 62, 170, 28), 'Reset all', sim.reset))
    S.settings.add(B((x0, y + 96, 170, 28), 'Demo input', actions['demo'], active=lambda: app.demo))
    y = 428
    S.settings.add(B((x0, y + 28, 110, 28), 'Learn map', actions['learn']))
    S.settings.add(B((x0 + 118, y + 28, 110, 28), 'Save map', actions['save']))
    S.settings.add(B((x0 + 236, y + 28, 116, 28), 'Default map', actions['default']))
    y = 506
    S.settings.add(B((x0, y + 28, 170, 28), 'Fullscreen (F11)', actions['fullscreen'], active=lambda: app.fullscreen))
    return S


def refresh_dynamic(app):
    app.dd_view.options[:] = [(v, VIEW_LABEL[v]) for v in app.views()]
    k = app.transport_kind
    if k == 'ble':
        app.dd_port.title = 'BLE DEVICE'
        opts = [(d.address, cm.ble_label(d)) for d in app.ble_devices[:8]]
        if app.ble_addr and app.ble_addr not in [a for a, _ in opts]:
            opts.insert(0, (app.ble_addr, (app.ble_name or app.ble_addr)[:20]))
        app.dd_port.options[:] = opts or [('', 'press Scan')]
    elif k == 'rfcomm':
        app.dd_port.title = 'PAIRED ROBOT'
        opts = [(d.mac, '%s %s' % (d.name[:12], d.mac[-5:])) for d in app.paired[:8]]
        if cm.valid_mac(app.mac) and app.mac not in [a for a, _ in opts]:
            opts.insert(0, (app.mac, (app.mac_name or app.mac)[:20]))
        app.dd_port.options[:] = opts or [('', 'none paired (DOCTOR)')]
    else:
        app.dd_port.title = 'PORT'
        app.dd_port.options[:] = [(p.device, app.port_label(p)) for p in app.ports[:8]] or [('', 'no ports (DOCTOR)')]


# ================================================================================================ drawing
def draw_topbar(canvas, fonts, app):
    pygame.draw.rect(canvas, PANEL, (M, 6, CW - 2 * M, 48), border_radius=8)
    text(canvas, fonts['l'], 'Robot Station', (24, 10))
    text(canvas, fonts['s'], VERSION, (24, 38), MUTED)
    l = app.link
    u4.led(canvas, (788, 34), l.state != cm.Link.DISCONNECTED, STATE_COL.get(l.state, u4.OK))
    if l.connected and l.stats['rtt_avg'] is not None:
        text(canvas, fonts['s'], '%.0f ms' % l.stats['rtt_avg'], (1016, 28), MUTED, 'tr')
    if l.armed:
        pygame.draw.rect(canvas, u4.DANGER, (1030, 18, 92, 32), border_radius=8)
        text(canvas, fonts['s'], 'ARMED', (1076, 34), (255, 220, 215), 'center')
    if app.timer_on:
        col = {'armed': (240, 200, 90), 'running': ACCENT, 'paused': (240, 150, 90)}[app.timer_state]
        text(canvas, fonts['l'], app.timer_text(), (CW - M - 188, 12), col, 'tr')
        hint = {'armed': 'press a button', 'running': 'running (P)', 'paused': 'paused (P)'}[app.timer_state]
        text(canvas, fonts['s'], hint, (CW - M - 188, 38), MUTED, 'tr')


def draw_left(canvas, fonts, big, st, learn_prompt, note, log_lines, mapping):
    pygame.draw.rect(canvas, PANEL, (LX, TOP, LW, BOT - TOP), border_radius=8)
    text(canvas, fonts['m'], 'CONTROLLER', (LX + 12, TOP + 8), MUTED)
    text(canvas, fonts['s'], (st.name if st.connected else 'NOT FOUND')[:20], (LX + LW - 12, TOP + 10), ACCENT if st.connected else (240, 120, 100), 'tr')
    img = pygame.Surface((cd.CTRL_W, cd.CTRL_H))
    img.fill(PANEL)
    cv.draw_controller(img, big, 0, 0, st)
    w = LW - 20
    scaled = pygame.transform.smoothscale(img, (w, int(w * cd.CTRL_H / cd.CTRL_W)))
    canvas.blit(scaled, (LX + 10, TOP + 30))
    y = TOP + 30 + scaled.get_height() + 6
    if learn_prompt:
        pygame.draw.rect(canvas, ACCENT, (LX + 10, y, LW - 20, 40), border_radius=8)
        text(canvas, fonts['s'], learn_prompt[:36], (LX + 18, y + 6), (8, 24, 22))
        text(canvas, fonts['s'], learn_prompt[36:72], (LX + 18, y + 22), (8, 24, 22))
        y += 46
    elif note:
        text(canvas, fonts['s'], note[:38], (LX + 12, y), ACCENT)
        y += 18
    y = u4.raw_compact(canvas, fonts, LX + 12, y + 2, LW - 24, st, mapping) + 4
    text(canvas, fonts['s'], 'EVENTS', (LX + 12, y), MUTED)
    n = max(2, (BOT - y - 26) // 15)
    for k, line in enumerate(log_lines[:n]):
        text(canvas, fonts['s'], line[:40], (LX + 12, y + 18 + k * 15), TEXT if k == 0 else MUTED)


def draw_centre(canvas, fonts, cam, scene, app, st=None):
    sim = app.sim
    pygame.draw.rect(canvas, PANEL, (CX, TOP, CWD, BOT - TOP), border_radius=8)
    d = app.state()
    if app.mode == GAME:
        head = 'GAME  |  %s  |  %s team' % (sim.robot_type.title(), sim.team)
    else:
        head = 'MONITOR  |  %s  |  %s' % (sim.robot_type.title(), 'robot' if app.source == 'robot' else 'sim')
    text(canvas, fonts['m'], head, (CX + 14, TOP + 8), ACCENT if app.mode == MONITOR else MUTED)
    text(canvas, fonts['s'], 'drive: %s' % MODE_LABEL[sim.mode], (CX + 14, TOP + 30), MUTED)
    if app.mode == GAME:
        draw_scene3(scene, cam, sim, app.cur_view(), app.look)
    else:
        draw_monitor3(scene, cam, d, app.cur_view(), app.look)
    canvas.blit(scene, (VP[0], VP[1]))
    if app.using_robot() and not app.telemetry_ok():
        pygame.draw.rect(canvas, (20, 23, 28), (VP[0] + VP[2] // 2 - 150, VP[1] + 14, 300, 28), border_radius=6)
        text(canvas, fonts['s'], 'NO TELEMETRY: connect on the LINK tab', (VP[0] + VP[2] // 2, VP[1] + 28), u4.WARN, 'center')
    y0 = VP[1] + VP[3] + 10
    u4.compass(canvas, fonts, CX + 110, y0 + 92, 78, d['heading'], math.degrees(d.get('omega', 0.0)))
    x, w = CX + 232, CWD - 244
    g = sim.grip
    u4.labelled_bar(canvas, fonts, x, y0 + 2, w, d['vl'], 'L wheel', '%+4.0f cm/s' % (d['vl'] * MAX_SPEED), centre=True, color=ACCENT)
    u4.labelled_bar(canvas, fonts, x, y0 + 22, w, d['vr'], 'R wheel', '%+4.0f cm/s' % (d['vr'] * MAX_SPEED), centre=True, color=ACCENT)
    live = app.mode == GAME or not app.using_robot()
    if d['type'] == 'simple':
        state = ('HOLDING cube %d' % g.held.id) if (g.held and app.mode == GAME) else ('touching' if (live and any(g.contact.values()) and app.mode == GAME) else 'free')
        u4.labelled_bar(canvas, fonts, x, y0 + 42, w, d['cmd'], 'Claw', '%3.0f%% %s' % (d['cmd'] * 100, state), color=(238, 168, 58), vw=150)
        yy = y0 + 62
    else:
        import sim_tower as tw
        u4.labelled_bar(canvas, fonts, x, y0 + 42, w, (d['lift'] - tw.LIFT_MIN) / (tw.LIFT_MAX - tw.LIFT_MIN), 'Box lift', '%4.1f cm' % d['lift'], color=(60, 200, 190))
        state = ('HOLDING cube %d' % g.held.id) if (g.held and app.mode == GAME) else ('touching' if (live and g.contact and app.mode == GAME) else 'free')
        u4.labelled_bar(canvas, fonts, x, y0 + 62, w, d['cmd'], 'Pad', '%3.0f%% %s' % (d['cmd'] * 100, state), color=(238, 168, 58), vw=150)
        yy = y0 + 82
    u4.labelled_bar(canvas, fonts, x, yy, w, sim.rt, 'Speed RT', 'cap %2.0f cm/s' % (sim.speed_scale * MAX_SPEED), color=(90, 170, 240), vw=150)
    text(canvas, fonts['s'], 'speed %+5.1f cm/s     x %+7.1f   y %+7.1f' % (d['speed'], d['x'], d['y']), (x, yy + 26), TEXT)
    snap = ('snap turning' if sim.snapping else 'snap idle') if sim.mode == 'arcade_snap' else 'snap off'
    text(canvas, fonts['s'], '%s     zero set %d x     turn %s' % (snap, sim.zero_count, 'ON' if sim.turn_on else 'OFF'), (x, yy + 44), MUTED)
    text(canvas, fonts['s'], 'drag = look around   wheel = zoom', (x, yy + 62), MUTED)


def draw_right(canvas, fonts, app):
    pygame.draw.rect(canvas, PANEL, (RX, TOP, RW, BOT - TOP), border_radius=8)
    if app.tab == 'link':
        draw_link(canvas, fonts, app)
    elif app.tab == 'doctor':
        draw_doctor(canvas, fonts, app)
    elif app.tab == 'settings':
        draw_settings(canvas, fonts, app)
    else:
        draw_help(canvas, fonts)


def draw_settings(canvas, fonts, app):
    x0, w = RX + 12, RW - 24
    for (y, h, t) in ((104, 74, 'DRIVE'), (184, 96, 'VIEW OVERLAYS  (switch + strength)'), (288, 132, 'ACTIONS'),
                      (428, 68, 'CONTROLLER MAP'), (506, 68, 'WINDOW')):
        u4.card(canvas, (x0, y, w, h), t, fonts)


def draw_help(canvas, fonts):
    x0, w = RX + 12, RW - 24
    u4.card(canvas, (x0, 104, w, 440), 'CONTROLS', fonts)
    for i, line in enumerate(HELP):
        text(canvas, fonts['s'], line, (x0 + 12, 130 + i * 20), TEXT if line else MUTED)


def draw_doctor(canvas, fonts, app):
    x0, w = RX + 12, RW - 24
    rep = app.report
    u4.card(canvas, (x0, 104, w, 206), 'WHAT TO DO NEXT', fonts)
    u4.card(canvas, (x0, 318, w, 362), 'WHAT THE CHECK FOUND', fonts)
    u4.card(canvas, (x0, 688, w, 54), 'COM PORT BY HAND', fonts)
    u4.card(canvas, (x0, 752, w, 76), 'BLUETOOTH ADDRESS BY HAND', fonts)
    if app.doctor_running and not rep:
        text(canvas, fonts['s'], 'checking this PC (about 2 seconds)...', (x0 + 12, 130), u4.WARN)
        return
    if app.doctor_note:
        for i, ln in enumerate(textwrap.wrap(app.doctor_note, 46)[:2]):
            text(canvas, fonts['s'], ln, (x0 + 12, 650 + i * 14), u4.OK)
    if not rep:
        return
    y = 128
    for i, step in enumerate(rep.steps[:5]):
        lines = wrap_hint(step, 43, 3)
        if y + 14 * len(lines) > 304:
            text(canvas, fonts['s'], '... more steps in the saved report', (x0 + 12, 290), MUTED)
            break
        for j, ln in enumerate(lines):
            text(canvas, fonts['s'], ('%d. ' % (i + 1) if j == 0 else '    ') + ln, (x0 + 12, y), TEXT)
            y += 14
        y += 3
    y = 378
    shown = [(lv, t) for lv, t in rep.lines if not t.startswith('running:')]
    for lv, t in shown:
        lines = textwrap.wrap(t, 46, subsequent_indent='  ', replace_whitespace=False) or ['']
        for j, ln in enumerate(lines[:2]):
            if y > 632:
                text(canvas, fonts['s'], '... more in the saved report', (x0 + 12, 634), MUTED)
                y = 9999
                break
            if j == 0 and lv != 'info':
                u4.led(canvas, (x0 + 14, y + 6), True, LEVEL_COL[lv], 4)
            text(canvas, fonts['s'], ln, (x0 + 24, y), TEXT if lv != 'info' else MUTED)
            y += 14
        if y == 9999: break


def wrap_hint(msg, width=47, lines=2):
    """Wraps to at most `lines` lines; when the text is longer the last line ends in '...' so nothing is cut silently."""
    out = textwrap.wrap(msg or '', width)
    if len(out) > lines:
        out = out[:lines]
        out[-1] = out[-1][:width - 3].rstrip() + '...'
    return out


def mmss(t):
    return '%d:%02d' % (int(t // 60), int(t % 60))


def draw_link(canvas, fonts, app):
    l = app.link
    x0, w = RX + 12, RW - 24
    s = l.stats
    # --- cards
    u4.card(canvas, (x0, 104, w, 166), 'CONNECTION', fonts)
    u4.card(canvas, (x0, 278, w, 152), 'LINK STATUS', fonts)
    u4.card(canvas, (x0, 438, w, 62), 'OUTPUT TO ROBOT', fonts)
    u4.card(canvas, (x0, 508, w, 108), 'ROBOT TELEMETRY', fonts)
    u4.card(canvas, (x0, 624, w, 204), 'SERIAL TERMINAL', fonts)
    # --- status
    u4.led(canvas, (x0 + 20, 306), l.state != cm.Link.DISCONNECTED, STATE_COL.get(l.state, u4.OK), 7)
    text(canvas, fonts['m'], l.state, (x0 + 34, 298), STATE_COL.get(l.state, MUTED))
    u4.signal_bars(canvas, x0 + w - 40, 294, l.quality())
    tname = l.transport.name if l.transport else '-'
    text(canvas, fonts['s'], tname[:30], (x0 + w - 50, 298), MUTED, 'tr')
    now = l.clock()
    u4.led(canvas, (x0 + 22, 330), l.connected and s['tx_rate'] > 0 and int(now * 8) % 2 == 0, u4.OK)
    text(canvas, fonts['s'], 'TX %4.1f/s' % s['tx_rate'], (x0 + 32, 323), TEXT)
    u4.led(canvas, (x0 + 122, 330), l.connected and s['rx_rate'] > 0 and int(now * 8) % 2 == 0, (90, 170, 240))
    text(canvas, fonts['s'], 'RX %4.1f/s' % s['rx_rate'], (x0 + 132, 323), TEXT)
    text(canvas, fonts['s'], 'bad %d   loss %.1f%%' % (s['bad'], l.loss_percent()), (x0 + 224, 323), u4.BAD if s['bad'] else TEXT)
    if s['rtt'] is None:
        text(canvas, fonts['s'], 'latency: -' + ('  (no telemetry in this profile)' if l.profile != 'rec1' and l.connected else ''), (x0 + 10, 342), MUTED)
    else:
        text(canvas, fonts['s'], 'latency %3.0f ms   avg %3.0f   min %3.0f   max %3.0f' % (s['rtt'], s['rtt_avg'], s['rtt_min'], s['rtt_max']), (x0 + 10, 342), TEXT)
    age = s['since_rx']
    text(canvas, fonts['s'], 'last data %s     up %s' % ('-' if age is None else '%.2f s ago' % age, mmss(l.uptime())), (x0 + 10, 360), MUTED)
    # what to do next (two short lines)
    hint = app.hint()
    hcol = u4.BAD if l.state == cm.Link.ERROR else (u4.WARN if (app.scanning() or l.state in (cm.Link.NO_REPLY, cm.Link.STALE)) else MUTED)
    for i, line in enumerate(wrap_hint(hint)):
        text(canvas, fonts['s'], line, (x0 + 10, 378 + i * 13), hcol)
    # --- output
    text(canvas, fonts['s'], 'LIVE: robot gets drive commands' if l.armed else 'safe: robot gets zeros', (x0 + w - 10, 444), u4.BAD if l.armed else MUTED, 'tr')
    # --- telemetry
    t = l.telemetry
    h = l.hello
    text(canvas, fonts['s'], ('%s  %s  (%s)' % (h.get('fw', '?'), h.get('version', ''), h.get('robot', '?'))) if h else 'no HELLO yet', (x0 + 10, 528), TEXT if h else MUTED)
    if t:
        v = t['vbat_mv'] / 1000.0
        low = bool(t['flags'] & cm.F_LOWBAT)
        u4.labelled_bar(canvas, fonts, x0 + 10, 546, w - 20, (t['vbat_mv'] - 6000) / 2400.0, 'Battery', '%.2f V' % v, u4.BAD if low else u4.OK, vw=72)
        text(canvas, fonts['s'], 'yaw %+7.1f deg    gyro %+6.1f deg/s    age %.2f s' % (t['yaw'], t['gz'], l.telemetry_age() or 0), (x0 + 10, 564), TEXT)
        fl = t['flags']
        chips = ((cm.F_ARMED, 'ARMED', u4.BAD), (cm.F_FAILSAFE, 'FAILSAFE', u4.WARN), (cm.F_GYRO_OK, 'GYRO OK', u4.OK), (cm.F_GYRO_CAL, 'GYRO CAL', u4.WARN), (cm.F_LOWBAT, 'LOW BAT', u4.BAD))
        cx = x0 + 12
        for bit, lab, col in chips:
            on = bool(fl & bit)
            u4.led(canvas, (cx, 592), on, col, 5)
            text(canvas, fonts['s'], lab, (cx + 9, 585), TEXT if on else MUTED)
            cx += 20 + len(lab) * 7
    else:
        text(canvas, fonts['s'], 'no telemetry', (x0 + 10, 550), MUTED)
    # --- terminal
    lines = list(l.log)[-7:]
    colmap = {'tx': (120, 190, 255), 'rx': TEXT, 'err': u4.BAD, 'sys': MUTED}
    for i, (ts, kind, msg) in enumerate(lines):
        prefix = {'tx': '>', 'rx': '<', 'err': '!', 'sys': '*'}[kind]
        text(canvas, fonts['s'], ('%s %s %s' % (mmss(ts % 3600), prefix, msg))[:46], (x0 + 10, 646 + i * 16), colmap[kind])


# ================================================================================================ one frame
def render(canvas, fonts, big, cam, scene, app, S, st, mapping, log_lines, learn_prompt='', note=''):
    canvas.fill(BG)
    refresh_dynamic(app)
    draw_topbar(canvas, fonts, app)
    draw_left(canvas, fonts, big, st, learn_prompt, note, log_lines, mapping)
    draw_centre(canvas, fonts, cam, scene, app, st)
    draw_right(canvas, fonts, app)
    text(canvas, fonts['s'], 'Space E-STOP   F11 fullscreen   V view   T drive mode   0 re-zero   P timer   L turn   M/S/R map   D demo   ESC quit', (M + 4, CH - 22), MUTED)
    S.draw(app, canvas, fonts)
    # the E-STOP button is red
    for b in S.link.buttons:
        if getattr(b, 'danger', False) and app.tab == 'link':
            pygame.draw.rect(canvas, u4.DANGER_HOT if b.hot else u4.DANGER, b.rect, border_radius=7)
            pygame.draw.rect(canvas, ui.LINE, b.rect, 1, border_radius=7)
            text(canvas, fonts['s'], b.label, b.rect.center, (255, 230, 225), 'center')


def make_window(fullscreen, size):
    if fullscreen:
        return pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    return pygame.display.set_mode(size, pygame.RESIZABLE)


def main():
    app = App50()
    app.demo = '--demo' in sys.argv
    pygame.init()
    win_size = (1440, 860)
    window = make_window(False, win_size)
    pygame.display.set_caption('Robot Station ' + VERSION)
    fonts, big = cd.make_fonts()
    canvas = pygame.Surface((CW, CH))
    pad = xi.XboxController()
    elog, note, note_t = cv.EventLog(), '', 0
    box, drag = {'learner': None}, {'on': False}
    scene, cam = pygame.Surface((VP[2], VP[3])), Camera(VP[2], VP[3], 400)

    def say(s):
        nonlocal note, note_t
        note, note_t = s, time.time()

    def toggle_full():
        nonlocal window, win_size
        if not app.fullscreen: win_size = window.get_size()
        app.fullscreen = not app.fullscreen
        window = make_window(app.fullscreen, win_size)

    def learn(): box['learner'] = xi.Learner(); say('')
    def save(): xi.save_mapping(pad.mapping); say('Saved to ' + xi.MAPPING_FILE)
    def default(): pad.mapping = xi.default_mapping(); box['learner'] = None; say('Default mapping restored')
    def demo(): app.demo = not app.demo

    def quit_app():
        if app.scanner: app.scanner.cancel()
        app.link.disconnect()
        app.save_settings()

    S = build_ui(app, dict(demo=demo, learn=learn, save=save, default=default, fullscreen=toggle_full))
    sim, link = app.sim, app.link
    clock, t0 = pygame.time.Clock(), time.time()
    while True:
        dt = min(clock.tick(60) / 1000.0, 0.05)
        for e in pygame.event.get():
            if e.type == pygame.QUIT:
                quit_app(); return
            if e.type == pygame.VIDEORESIZE and not app.fullscreen:
                win_size = (max(480, e.w), max(300, e.h))
                window = pygame.display.set_mode(win_size, pygame.RESIZABLE)
            elif e.type == pygame.MOUSEMOTION:
                cp = ui.to_canvas(e.pos, window.get_size(), CW, CH)
                S.motion(app, cp)
                if drag['on']:
                    k = ui.fit_scale(window.get_width(), window.get_height(), CW, CH)[0]
                    app.drag(e.rel[0] / k, e.rel[1] / k)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button == 1:
                cp = ui.to_canvas(e.pos, window.get_size(), CW, CH)
                if not S.click(app, cp) and app.in_scene(cp):
                    drag['on'] = True
                S.drop_focus(app)
            elif e.type == pygame.MOUSEBUTTONUP and e.button == 1:
                drag['on'] = False
            elif e.type == pygame.MOUSEWHEEL:
                if app.in_scene(ui.to_canvas(pygame.mouse.get_pos(), window.get_size(), CW, CH)): app.zoom(e.y)
            elif e.type == pygame.MOUSEBUTTONDOWN and e.button in (4, 5):
                if app.in_scene(ui.to_canvas(e.pos, window.get_size(), CW, CH)): app.zoom(1 if e.button == 4 else -1)
            elif e.type == pygame.KEYDOWN:
                if S.typing(app):
                    S.key(app, e)
                    continue
                if e.key == pygame.K_ESCAPE:
                    if S.any_open(): S.close_all()
                    elif app.fullscreen: toggle_full()
                    else:
                        quit_app(); return
                elif e.key == pygame.K_SPACE:
                    if box['learner'] and not box['learner'].done: box['learner'].skip()
                    else: app.estop()
                elif e.key in (pygame.K_F11, pygame.K_f): toggle_full()
                elif e.key == pygame.K_d: demo()
                elif e.key == pygame.K_m: learn()
                elif e.key == pygame.K_s: save()
                elif e.key == pygame.K_r: default()
                elif e.key == pygame.K_v: app.next_view()
                elif e.key == pygame.K_t: app.next_drive_mode()
                elif e.key == pygame.K_p: app.timer_pause()
                elif e.key == pygame.K_l: sim.turn_on = not sim.turn_on
                elif e.key == pygame.K_0: sim.zero()
                elif e.key == pygame.K_z: sim.reset()
                elif e.key == pygame.K_x: sim.reset_robot()
                elif e.key == pygame.K_c and app.mode == GAME: sim.reset_cubes()
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
        app.tick_link()
        elog.update(st)
        app.timer_update(dt, st)
        render(canvas, fonts, big, cam, scene, app, S, st, pad.mapping, elog.lines, learner.prompt if learner else '',
               note if (note and time.time() - note_t < 6) else '')
        ui.present(window, canvas)
        pygame.display.flip()


if __name__ == '__main__':
    main()
