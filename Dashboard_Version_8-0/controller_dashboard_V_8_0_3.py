"""controller_dashboard_V_8_0_3.py - Robot Station V 8.0: the tower robot simulation (Game), a monitor view of the real robot (Monitor), the wireless
test and tuning page (Tune: motors, servos, gyro, battery) the PID tab (gyro heading control: live orientation, graphs, tuning, step tests) and the link to the ESP32.
BACK button on the controller arms / disarms the robot; the link reconnects by itself if it drops.
With the robot connected, the compass, the picture and the snap turns follow the robot's own gyro; the sticks go to the wheels with no simulated lag.

    python -m pip install -r requirements_V_8_0.txt        (pygame, pyserial, bleak)
    python controller_dashboard_V_8_0_3.py

Everything it needs is in this folder (sim_, render_, ui_, input_, comms_ files). Space = E-STOP. The robot gets drive commands only after ARM OUTPUT.
Keys: Space E-STOP   F11 / F fullscreen   V next view   T drive mode   L turn on/off   0 re-zero   P pause timer   C cubes   X robot   Z all (reset)   ESC quit"""
import json, math, os, sys, textwrap, time
from collections import deque
import pygame
import ui_V_8_0_1 as ui
import input_V_8_0_1 as inp
import sim_V_8_0_2 as sm
import render_V_8_0_2 as rd
import comms_V_8_0_2 as cm
from ui_V_8_0_1 import text, PANEL, TEXT, MUTED, ACCENT, BG

VERSION = 'V 8.0'
SETTINGS_FILE = os.path.join(os.path.expanduser('~'), '.rec_station_v80.json')
CW, CH = 1440, 860                                  # the design canvas, scaled to any window
GAME, MONITOR, TUNE, PID = 'game', 'monitor', 'tune', 'pid'
M, TOP, BOT = 12, 60, 832
LX, LW = M, 300                                     # left column
CX, CWD = LX + LW + M, 720                          # centre column
RX, RW = CX + CWD + M, 372                          # right column
VP = (CX + 12, 112, CWD - 24, 456)                  # the picture
X0, WW = RX + 12, RW - 24                           # inside the right column: cards start at X0 and are WW wide; widgets keep 10 px from the edge
TABS = [('link', 'LINK'), ('settings', 'SETTINGS')]
PAGE = (LX, TOP, CX + CWD - LX, BOT - TOP)                  # the Tune page covers the controller panel and the picture
TUNE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'tuning_values_V_8_0.txt')
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
        self.tune = dict(vl=0.0, vr=0.0, s1=None, s2=None)       # what the Tune page asks for: wheels in %, servos in degrees (None = limp)
        self.sweep, self.sweep_t, self.ramp = False, 0.0, None
        self.hist, self.hist_pause, self.hist_at = deque(maxlen=400), False, None     # PID page: the last samples of the robot's gyro control
        self.pid_test, self.pid_result = None, ''
        self.auto_reconnect, self.want_link, self.retries, self._retry_at, self._stuck_at, self._back_prev = True, False, 0, None, None, False
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
        if isinstance(d.get('auto_reconnect'), bool): self.auto_reconnect = d['auto_reconnect']
        if isinstance(d.get('slow_cm'), (int, float)): self.sim.slow_cm = max(10.0, min(sm.MAX_SPEED, float(d['slow_cm'])))

    def save_settings(self):
        try:
            with open(self.settings_path, 'w') as f:
                json.dump(dict(transport=self.transport_kind, port=self.port, ble_addr=self.ble_addr, ble_name=self.ble_name, auto_reconnect=self.auto_reconnect, slow_cm=self.sim.slow_cm), f)
        except Exception:
            pass

    # ---- mode / view
    def views(self): return rd.VIEWS_GAME if self.mode == GAME else rd.VIEWS_MONITOR


    def set_mode(self, m):
        if m != self.mode:
            if self.mode == TUNE: self.tune_stop()
            self.pid_cancel()
            self.mode = m
            self.sim.set_arena(m == GAME)               # Monitor: no arena, no cubes, no collisions
            if m == MONITOR: self.sim.reset_robot()

    def cur_view(self): return self.view[self.mode]

    def set_view(self, v): self.view[self.mode] = v

    def next_view(self):
        v = self.views()
        self.set_view(v[(v.index(self.cur_view()) + 1) % len(v)])

    def next_drive_mode(self): self.sim.mode = sm.MODES[(sm.MODES.index(self.sim.mode) + 1) % len(sm.MODES)]

    def in_scene(self, pos): return self.mode not in (TUNE, PID) and pos is not None and VP[0] <= pos[0] < VP[0] + VP[2] and VP[1] <= pos[1] < VP[1] + VP[3]

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

    # ---- Tune page: what the robot reports, and what we ask of it
    def tune_tel(self):
        """The tuning firmware's latest report, or {} when there is none (not connected, silent, or the robot runs another firmware)."""
        l = self.link
        t = l.tune_tel
        return t if t and l.connected and l.clock() - t['at'] < 1.0 else {}

    def tune_status(self):
        """(text, colour) for the page header."""
        l = self.link
        if not l.connected: return 'NOT CONNECTED: open the LINK tab, connect to REC-Robot', ui.WARN
        if l.hello and l.hello.get('robot') != 'tune': return 'this robot runs the normal firmware, not esp32_tune_V_8_0_3: flash the tuning sketch', ui.BAD
        if not self.tune_tel(): return 'connected, waiting for tuning data...', ui.WARN
        return ('OUTPUT LIVE: sliders move the robot' if l.armed else 'connected. Press ARM OUTPUT on the LINK tab to move anything'), (ui.BAD if l.armed else ui.OK)

    def cfg(self, key):
        l = self.link
        return l.tune_cfg.get(key) if l.connected else None

    def set_cfg(self, key, v):
        self.link.send_frame('TCFG', key, v)
        self.link.tune_cfg[key] = float(v)                  # shown at once; the robot's answer confirms it

    def set_motor(self, side, v):
        self.tune['vl' if side == 0 else 'vr'] = float(v)
        if self.ramp and self.ramp['side'] == side: self.ramp = None

    def set_servo(self, i, v):
        self.tune['s%d' % i] = None if v is None else int(v)
        self.sweep = False

    def tune_stop(self):
        self.tune['vl'] = self.tune['vr'] = 0.0
        self.ramp, self.sweep = None, False

    def servo_value(self, i):
        v = self.tune['s%d' % i]
        return 90 if v is None else v

    def ramp_toggle(self, side):
        """Find the lowest drive that turns a wheel: the wheel is brought up slowly from 0, press again the moment it starts to turn."""
        if self.ramp and self.ramp['side'] == side:
            t = self.tune_tel()
            pwm = abs(t.get('pwm_l' if side == 0 else 'pwm_r', 0))
            self.tune_stop()
            if pwm: self.set_cfg('minpwm', pwm); self.link.say('sys', 'wheel started at PWM %d: minpwm saved' % pwm)
            return
        self.tune_stop()
        self.set_cfg('minpwm', 0)                                # with minpwm 0 the percent goes straight to PWM
        self.ramp = dict(side=side, pct=0.0)

    def invert(self, side):
        k = 'invl' if side == 0 else 'invr'
        self.set_cfg(k, 0 if self.cfg(k) else 1)

    def save_values(self):
        c = self.link.tune_cfg
        if not c: return
        lines = ['// Robot Station V 8.0 tuning values: paste into the robot firmware', 'const int MIN_PWM = %d;' % c['minpwm'],
                 'const bool INVERT_LEFT = %s, INVERT_RIGHT = %s;' % ('true' if c['invl'] else 'false', 'true' if c['invr'] else 'false'),
                 'servo 1 limits: %d .. %d degrees' % (c['lo1'], c['hi1']), 'servo 2 limits: %d .. %d degrees' % (c['lo2'], c['hi2']), 'const float VBAT_RATIO = %.4f;' % c['ratio']]
        try:
            with open(TUNE_FILE, 'w') as f: f.write('\n'.join(lines) + '\n')
            self.link.say('sys', 'saved %s' % os.path.basename(TUNE_FILE))
        except Exception as e:
            self.link.say('err', 'could not save the values: %s' % e)

    def tick_tune(self, dt):
        l = self.link
        if self.mode != TUNE:
            l.tune = None
            return
        if not l.armed: self.tune['vl'] = self.tune['vr'] = 0.0      # never leave a slider up: arming must not start the wheels
        if self.ramp:
            self.ramp['pct'] += dt * 6.0                              # 6 % per second
            if self.ramp['pct'] >= 100: self.tune_stop()
            else: self.tune['vl' if self.ramp['side'] == 0 else 'vr'] = self.ramp['pct']
        if self.sweep:
            self.sweep_t += dt
            ph = (self.sweep_t % 4.0) / 2.0
            ph = ph if ph <= 1 else 2 - ph
            for i, (lo, hi) in ((1, (self.cfg('lo1'), self.cfg('hi1'))), (2, (self.cfg('lo2'), self.cfg('hi2')))):
                lo, hi = 0 if lo is None else lo, 180 if hi is None else hi
                self.tune['s%d' % i] = int(lo + (hi - lo) * ph)
        t = self.tune
        l.tune = dict(vl=t['vl'], vr=t['vr'], s1=-1 if t['s1'] is None else t['s1'], s2=-1 if t['s2'] is None else t['s2'])

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
            self.want_link = False                          # the person disconnected: do not reconnect
            self.link.disconnect()
            return
        self.want_link = True
        self.start_link()

    def start_link(self):
        if not self.target_value():
            msg = 'no COM port: plug the USB cable in and press Refresh' if self.transport_kind == 'serial' else 'no device selected: press Scan, the robot is called REC-Robot'
            self.link.say('err', msg)
            self.link.last_error = msg
            self.link.state = cm.Link.ERROR
            self.want_link = False
            return
        k = self.transport_kind
        self.link.connect(lambda: cm.BleTransport(self.ble_addr, self.ble_name) if k == 'ble' else cm.SerialTransport(self.port))
        self.pose = cm.TelemetryPose()
        self.save_settings()

    def tick_reconnect(self):
        """The link dropped (robot reset, out of range, Bluetooth hiccup) and the person did not disconnect: try again every 2 s. A new link is always disarmed."""
        l, now = self.link, self.link.clock()
        if not (self.auto_reconnect and self.want_link): self._retry_at = self._stuck_at = None; return
        if l.state in (cm.Link.STALE, cm.Link.NO_REPLY):                 # stuck without data for 8 s: drop it so the retry below starts clean
            self._stuck_at = now if self._stuck_at is None else self._stuck_at
            if now - self._stuck_at > 8.0: l.say('sys', 'no data for 8 s: restarting the link'); l.disconnect(quiet=True); self._stuck_at = None
            return
        self._stuck_at = None
        if l.state in (cm.Link.DISCONNECTED, cm.Link.ERROR) and not self.scanning():
            if self._retry_at is None: self._retry_at = now + 2.0; self.retries += 1
            if now >= self._retry_at:
                self._retry_at = None
                l.say('sys', 'auto-reconnect, attempt %d' % self.retries)
                self.start_link()
        else:
            self._retry_at = None
            if l.state == cm.Link.CONNECTED and self.retries:
                self.retries = 0
                l.say('sys', 'reconnected: the output is DISARMED, press BACK to arm')

    # ---- controller BACK button: arm / disarm
    def tick_pad(self, st):
        back = bool(st.buttons.get('BACK'))
        if back and not self._back_prev: self.back_pressed(st)
        self._back_prev = back

    def back_pressed(self, st):
        l = self.link
        if l.armed:
            self.toggle_arm()                                     # disarming is always allowed
        elif not (l.connected and self.telemetry_ok()):
            l.say('err', 'BACK: not connected to the robot yet')
        elif abs(st.lx) + abs(st.ly) + abs(st.rx) + abs(st.ry) > 0.2 or st.dpad != (0, 0):
            l.say('err', 'BACK: release the sticks and D-pad first, then press BACK')
        else:
            self.toggle_arm()

    def toggle_arm(self):
        if not self.link.armed: self.tune['vl'] = self.tune['vr'] = 0.0; self.ramp = None
        self.link.armed = not self.link.armed
        self.link.say('sys', 'output ARMED: the robot now receives drive commands' if self.link.armed else 'output disarmed: the robot receives zeros')

    def estop(self):
        self.tune_stop()
        self.pid_cancel()
        self.link.estop()

    def hint(self):
        """One or two short lines telling the person what to do next."""
        l = self.link
        if self.scanning(): return 'scanning for Bluetooth LE devices... %d%%' % round(self.ble.progress() * 100)
        if self.auto_reconnect and self.want_link and l.state in (cm.Link.DISCONNECTED, cm.Link.ERROR) and self._retry_at is not None:
            return 'link lost: reconnecting in %.0f s (attempt %d). Press Disconnect to stop.' % (max(0.0, self._retry_at - l.clock()), self.retries)
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
        """What the sticks ask of the wheels, straight (the simulated robot's lag is NOT in the way). During a snap turn the target heading goes to
        the robot, which turns to it with its own gyro."""
        s, t = self.sim, self.sim.tower
        hdg = math.degrees(s.snap_target) if (s.follow and s.snapping) else None
        cmd = cm.Command(s.raw_l * 100, s.raw_r * 100, t.pad_cmd * 100, t.lift_cmd * 100, True, hdg)
        if self.pid_test is not None:                                                       # a step test from the PID page overrides the wheels
            tt, l = self.pid_test, self.link
            now, yaw, gz = l.clock(), l.telemetry.get('yaw', 0.0), l.telemetry.get('gz', 0.0)
            if abs(s.raw_l) + abs(s.raw_r) > 0.05 or not l.armed or self.mode != PID: self.pid_cancel('stopped')
            else:
                if tt['kind'] == 'turn':
                    if abs(yaw - tt['target']) < 1.5 and abs(gz) < 12 and now - tt['t0'] > 0.8: tt.setdefault('arrive', now)
                    else: tt.pop('arrive', None)
                done = now - tt['t0'] > tt['dur'] or (tt.get('arrive') is not None and now - tt['arrive'] > 0.6)
                if done: self.pid_cancel('done')
                elif tt['kind'] == 'straight': cmd.vl = cmd.vr = 50
                else: cmd.vl = cmd.vr = 0; cmd.hdg = tt['target']
        return cmd

    # ---- PID page: history and step tests
    def sample_hist(self):
        if self.hist_pause or not self.hold_fresh(): return
        h, t = self.link.hold, self.link.telemetry
        if h['at'] == self.hist_at: return
        self.hist_at = h['at']
        self.hist.append(dict(t=h['at'], yaw=t.get('yaw', 0.0), gz=t.get('gz', 0.0), target=h['target'], err=h['err'], corr=h['corr'], bal=h['bal'],
                              vl=t.get('vl', 0), vr=t.get('vr', 0), mode=int(h['mode'])))

    def pid_start(self, kind, deg=0.0):
        l = self.link
        if not (l.armed and self.hold_fresh()):
            l.say('err', 'step test: connect and ARM OUTPUT first')
            return
        yaw = l.telemetry.get('yaw', 0.0)
        self.pid_test = dict(kind=kind, t0=l.clock(), deg=deg, y0=yaw, target=yaw + deg, dur=3.0 if kind == 'straight' else 4.0)
        self.pid_result = 'running ... (move a stick to stop it)'

    def pid_cancel(self, why=''):
        tt = self.pid_test
        if tt is None: return
        self.pid_test = None
        rows = [r for r in self.hist if r['t'] >= tt['t0']]
        if why != 'done' or not rows:
            self.pid_result = 'test stopped'
        elif tt['kind'] == 'straight':
            emax = max([abs(r['err']) for r in rows if r['mode'] == 2] or [0.0])
            self.pid_result = 'straight run: biggest error %.1f deg, ended %+.1f deg from the start heading, balance %+.2f %%' % (emax, rows[-1]['yaw'] - tt['y0'], rows[-1]['bal'])
        else:
            sg = 1 if tt['deg'] > 0 else -1
            over = max((r['yaw'] - tt['y0']) * sg for r in rows) - abs(tt['deg'])
            late = [r['t'] for r in rows if abs(r['yaw'] - tt['target']) >= 2.0]
            settle = (max(late) - tt['t0'] + 0.1) if late else 0.0
            self.pid_result = 'turn %+d deg: overshoot %+.1f deg, settled in %.1f s, final error %+.1f deg' % (tt['deg'], over, settle, rows[-1]['yaw'] - tt['target'])

    def pid_status(self):
        l = self.link
        if not l.connected: return 'NOT CONNECTED: open the LINK tab, connect to REC-Robot', ui.WARN
        if not self.hold_fresh(): return 'connected, but no PID data: flash esp32_robot_V_8_0_3', ui.WARN
        return ('OUTPUT LIVE: the robot can move' if l.armed else 'connected. ARM OUTPUT (LINK tab) to run the tests'), (ui.BAD if l.armed else ui.OK)

    def pre_drive(self):
        """Call before sim.drive each frame: with fresh telemetry the picture and the compass follow the real robot's gyro."""
        t = self.link.telemetry
        if self.telemetry_ok():
            self.sim.follow_robot(True, t.get('yaw', 0.0), t.get('gz', 0.0), t.get('vl', 0), t.get('vr', 0))
        else:
            self.sim.follow_robot(False)

    # ---- gyro hold settings kept in the robot (HLD frame shows them, CFG changes them)
    def hold_fresh(self):
        h = self.link.hold
        return bool(h) and self.link.connected and self.link.clock() - h['at'] < 1.5

    def hcfg(self, key): return self.link.hold.get(key) if self.hold_fresh() else None

    def set_hcfg(self, key, v):
        self.link.send_frame('CFG', key, v)
        self.link.hold[key] = float(v)                       # shown at once; the robot's next HLD confirms it

    def hold_toggle(self): self.set_hcfg('hold', 0 if self.hcfg('hold') else 1)

    def gyro_flip(self): self.set_hcfg('gsign', -(self.hcfg('gsign') or 1))

    def tick_link(self, dt=1 / 60.0):
        self.link.set_command(self.command_from_sim())
        self.tick_tune(dt)
        self.link.update()
        self.tick_reconnect()
        self.sample_hist()
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
    """All widgets, in groups: top (always), scene (Game and Monitor), mon (Monitor only), tune (Tune only), link and settings (one tab at a time)."""

    def __init__(self, app):
        self.top, self.scene, self.mon, self.tune, self.link, self.settings, self.pidg = (ui.Group() for _ in range(7))
        self.build(app)

    def active(self, app):
        page = {TUNE: self.tune, PID: self.pidg}.get(app.mode)
        return [self.top] + ([page] if page else [self.scene]) + ([self.mon] if app.mode == MONITOR else []) + [self.link if app.tab == 'link' else self.settings]

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

    def release(self, app):
        for g in (self.top, self.scene, self.mon, self.tune, self.link, self.settings, self.pidg): g.release()

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
        self.top.add(B((442, 20, 92, 28), 'Tune', lambda: app.set_mode(TUNE), active=lambda: app.mode == TUNE))
        self.top.add(B((538, 20, 62, 28), 'PID', lambda: app.set_mode(PID), active=lambda: app.mode == PID))
        self.top.add(B((606, 18, 150, 32), lambda: 'LINK  %s' % link.state, lambda: setattr(app, 'tab', 'link')))
        tx = CW - M - 176
        self.top.add(B((tx, 20, 54, 28), 'Timer', app.timer_toggle, active=lambda: app.timer_on))
        self.top.add(B((tx + 58, 20, 54, 28), 'Reset', app.timer_reset, enabled=lambda: app.timer_on))
        self.top.add(B((tx + 116, 20, 60, 28), 'Restart', app.timer_restart, enabled=lambda: app.timer_on))
        self.top.add(ui.Tabs((RX + 8, TOP + 8, RW - 16, 30), TABS, lambda: app.tab, lambda v: setattr(app, 'tab', v)))
        # ---- picture header
        vx = VP[0] + VP[2]
        app.dd_view = self.scene.add(ui.Dropdown((vx - 176, TOP + 22, 176, 28), 'VIEW', [], app.cur_view, app.set_view))
        self.mon.add(ui.Dropdown((vx - 176 - 196, TOP + 22, 184, 28), 'PICTURE FROM', SOURCES, lambda: app.source, lambda v: setattr(app, 'source', v)))
        # ---- TUNE page: positions match draw_tune()
        T, armed, up = self.tune, (lambda: link.armed), (lambda: bool(app.tune_tel()))
        for side, cx in ((0, LX + 140), (1, LX + 390)):
            T.add(ui.Slider((cx - 120, 312, 240, 22), lambda s=side: app.tune['vl' if s == 0 else 'vr'], lambda v, s=side: app.set_motor(s, v), -100, 100, 1, 0.03, armed, ACCENT, '%d'))
            T.add(B((cx - 120, 342, 70, 26), 'Stop', lambda s=side: app.set_motor(s, 0)))
            T.add(B((cx - 44, 342, 164, 26), lambda s=side: 'Wheel moves: SAVE' if app.ramp and app.ramp['side'] == s else 'Find start power',
                    lambda s=side: app.ramp_toggle(s), active=lambda s=side: bool(app.ramp and app.ramp['side'] == s), enabled=lambda: link.armed and bool(app.cfg('minpwm') is not None)))
        for i, cx in ((1, 544 + 130), (2, 544 + 370)):
            T.add(ui.Slider((cx - 120, 312, 240, 22), lambda i=i: app.servo_value(i), lambda v, i=i: app.set_servo(i, v), 0, 180, 1, None, armed, (240, 170, 70), '%d'))
            T.add(B((cx - 120, 342, 70, 26), 'Limp', lambda i=i: app.set_servo(i, None), active=lambda i=i: app.tune['s%d' % i] is None, enabled=armed))
            T.add(B((cx - 44, 342, 74, 26), 'Centre', lambda i=i: app.set_servo(i, 90), enabled=armed))
            T.add(B((cx + 36, 342, 84, 26), 'Min / Max', lambda i=i: app.set_servo(i, app.cfg('lo%d' % i) if app.tune['s%d' % i] != app.cfg('lo%d' % i) else app.cfg('hi%d' % i)), enabled=lambda: link.armed and app.cfg('lo1') is not None))
        T.add(B((544 + 500 - 130, 96, 120, 22), lambda: 'Sweep: ON' if app.sweep else 'Sweep both', lambda: setattr(app, 'sweep', not app.sweep), active=lambda: app.sweep, enabled=armed))
        T.add(B((LX + 520 - 190, 388, 180, 22), 'Zero gyro (keep still)', lambda: link.send_frame('CAL'), enabled=up))
        rows = [('Start power (MIN_PWM)', 'minpwm', 5, 0, 200, '%d'), ('Servo 1 min deg', 'lo1', 5, 0, 180, '%d'), ('Servo 1 max deg', 'hi1', 5, 0, 180, '%d'),
                ('Servo 2 min deg', 'lo2', 5, 0, 180, '%d'), ('Servo 2 max deg', 'hi2', 5, 0, 180, '%d'), ('Battery ratio', 'ratio', 0.05, 1.0, 6.0, '%.3f')]
        for k, (lab, key, step, lo, hi, fmt) in enumerate(rows):
            T.add(ui.Spin((706 + 12, 432 + k * 30, 314, 24), lab, lambda key=key: app.cfg(key), lambda v, key=key: app.set_cfg(key, v), step, lo, hi, fmt))
        T.add(B((706 + 12, 618, 152, 26), lambda: 'Left wheel: %s' % ('REVERSED' if app.cfg('invl') else 'normal'), lambda: app.invert(0), active=lambda: bool(app.cfg('invl')), enabled=up))
        T.add(B((706 + 174, 618, 152, 26), lambda: 'Right wheel: %s' % ('REVERSED' if app.cfg('invr') else 'normal'), lambda: app.invert(1), active=lambda: bool(app.cfg('invr')), enabled=up))
        T.add(B((706 + 12, 652, 314, 26), 'Save values to a file', app.save_values, enabled=lambda: bool(link.tune_cfg)))
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
        # ---- PID page: positions match draw_pid()
        H = self.pidg
        rows = [('Hold strength  Kp', 'kp', 0.1, 0.0, 5.0, '%.1f'), ('Hold learning  Ki', 'ki', 0.1, 0.0, 5.0, '%.1f'), ('Hold damping  Kd', 'kd', 0.05, 0.0, 2.0, '%.2f'),
                ('Left wheel gain', 'gl', 0.01, 0.5, 1.5, '%.2f'), ('Right wheel gain', 'gr', 0.01, 0.5, 1.5, '%.2f'),
                ('Left start PWM', 'ml', 5, 0, 200, '%d'), ('Right start PWM', 'mr', 5, 0, 200, '%d'),
                ('Turn strength', 'skp', 0.1, 0.2, 4.0, '%.1f'), ('Turn damping', 'skd', 0.05, 0.0, 1.0, '%.2f')]
        for k, (lab, key, step, lo, hi, fmt) in enumerate(rows):
            H.add(ui.Spin((LX + 12, 428 + k * 28, 306, 24), lab, lambda key=key: app.hcfg(key), lambda v, key=key: app.set_hcfg(key, v), step, lo, hi, fmt))
        H.add(B((LX + 10, 690, 154, 28), lambda: 'Gyro hold: ON' if app.hcfg('hold') else 'Gyro hold: OFF', app.hold_toggle, active=lambda: bool(app.hcfg('hold')), enabled=lambda: app.hcfg('hold') is not None))
        H.add(B((LX + 170, 690, 152, 28), 'Flip gyro direction', app.gyro_flip, enabled=lambda: app.hcfg('gsign') is not None))
        H.add(B((LX + 10, 724, 154, 28), 'Reset balance', lambda: app.set_hcfg('bal', 0), enabled=lambda: app.hcfg('bal') is not None))
        H.add(B((LX + 170, 724, 152, 28), 'Re-zero heading', app.zero))
        can = lambda: link.armed and app.hold_fresh() and app.pid_test is None
        H.add(B((366, 640, 160, 28), 'Straight 3 s (50%)', lambda: app.pid_start('straight'), enabled=can))
        for k, dg in enumerate((45, -45, 90, -90)):
            H.add(B((532 + k * 78, 640, 72, 28), 'Turn %+d' % dg, lambda dg=dg: app.pid_start('turn', dg), enabled=can))
        H.add(B((848, 640, 92, 28), lambda: 'Resume' if app.hist_pause else 'Pause', lambda: setattr(app, 'hist_pause', not app.hist_pause), active=lambda: app.hist_pause))
        H.add(B((946, 640, 86, 28), 'Clear', app.hist.clear))
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
        S.add(B((X0 + 10, 500, WW - 20, 28), lambda: 'Auto-reconnect: ON' if app.auto_reconnect else 'Auto-reconnect: OFF', lambda: (setattr(app, 'auto_reconnect', not app.auto_reconnect), app.save_settings()), active=lambda: app.auto_reconnect))
        S.add(ui.Spin((X0 + 12, 540, WW - 24, 24), 'Speed without RT (cm/s)', lambda: sim.slow_cm, lambda v: (setattr(sim, 'slow_cm', float(v)), app.save_settings()), 1, 10, sm.MAX_SPEED, '%d'))


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
    ui.led(canvas, (762, 34), l.state != cm.Link.DISCONNECTED, STATE_COL.get(l.state, ui.OK))
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
    text(canvas, fonts['s'], 'ROBOT GYRO' if sim.follow else 'SIMULATION  (no robot data)', (CX + 110, y0 + 240), ui.OK if sim.follow else MUTED, 'center')
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
    ui.tile(canvas, fonts, x, ty, 'TURN  (LT)', sim.turn_on, 'ON' if sim.turn_on else 'OFF', ui.OK, OFF_RED)
    snap_on = sim.mode == 'arcade_snap'
    ui.tile(canvas, fonts, x + gap, ty, 'SNAP', snap_on, ('SPIN' if sim.snapping else 'READY') if snap_on else 'OFF', AMBER if sim.snapping else ui.OK)
    hd = app.link.hold if app.hold_fresh() else None                            # straight-line hold of the real robot
    if hd is None: ui.tile(canvas, fonts, x + 2 * gap, ty, 'GYRO HOLD', False, '--')
    else:
        mode = int(hd['mode'])
        word = ('OFF', 'READY', 'LOCK', 'TURN')[mode]
        ui.tile(canvas, fonts, x + 2 * gap, ty, 'GYRO HOLD', mode > 0, word, AMBER if (mode == 3 or (mode == 2 and abs(hd['corr']) > 12)) else ui.OK)
    fl = app.link.telemetry.get('flags', 0) if app.telemetry_ok() else None      # the gyro itself
    if fl is None: ui.tile(canvas, fonts, x + 3 * gap, ty, 'GYRO', False, '--')
    elif fl & cm.F_GYRO_OK: ui.tile(canvas, fonts, x + 3 * gap, ty, 'GYRO', True, 'OK', ui.OK)
    elif fl & cm.F_GYRO_CAL: ui.tile(canvas, fonts, x + 3 * gap, ty, 'GYRO', True, 'CAL', AMBER)
    else: ui.tile(canvas, fonts, x + 3 * gap, ty, 'GYRO', True, 'LOST', ui.BAD)


def draw_tune(canvas, fonts, app):
    """The Tune page. Positions must match the widgets built in Screen.build."""
    pygame.draw.rect(canvas, PANEL, PAGE, border_radius=8)
    text(canvas, fonts['m'], 'TUNE  |  wireless test and tuning', (LX + 14, TOP + 8), ACCENT)
    msg, col = app.tune_status()
    text(canvas, fonts['s'], msg, (LX + PAGE[2] - 14, TOP + 10), col, 'tr')
    t, l = app.tune_tel(), app.link
    live = bool(t)
    ui.card(canvas, (LX, 92, 520, 284), 'MOTORS  (needle = PWM duty the robot is really sending)', fonts)
    ui.card(canvas, (544, 92, 500, 284), 'SERVOS', fonts)
    ui.card(canvas, (LX, 384, 520, 442), 'GYRO + ACCELEROMETER  (MPU6050)', fonts)
    ui.card(canvas, (544, 384, 150, 442), 'BATTERY', fonts)
    ui.card(canvas, (706, 384, 338, 442), 'TUNING VALUES', fonts)
    # ---- motors: two speedometers
    for side, cx, name in ((0, LX + 140 + 12, 'LEFT'), (1, LX + 390 + 12, 'RIGHT')):
        pwm = t.get('pwm_l' if side == 0 else 'pwm_r', 0)
        pct = pwm * 100.0 / 255
        ui.speedo(canvas, fonts, cx, 204, 78, pct, -100, 100, ui.OK if pct >= 0 else AMBER, name, ('%+d%%' % round(pct)) if live else '--',
                  ('PWM %+4d  %s' % (pwm, 'FWD' if pwm > 0 else 'REV' if pwm < 0 else 'STOP')) if live else 'no data', live=live)
        ask = app.tune['vl' if side == 0 else 'vr']
        text(canvas, fonts['s'], 'asked %+d%%' % round(ask), (cx, 298), MUTED if not ask else TEXT, 'center')
    # ---- servos: two dials
    for i, cx in ((1, 544 + 130), (2, 544 + 370)):
        a = t.get('s%d' % i) if live else False
        lo, hi = app.cfg('lo%d' % i), app.cfg('hi%d' % i)
        ui.servo_dial(canvas, fonts, cx, 228, 78, (None if a is not None and a < 0 else a) if live else False, 0 if lo is None else lo, 180 if hi is None else hi,
                      (240, 170, 70), 'SERVO %d  (GPIO %d)' % (i, 18 if i == 1 else 19))
        v = app.tune['s%d' % i]
        text(canvas, fonts['s'], 'asked %s' % ('limp' if v is None else '%d deg' % v), (cx, 298), MUTED if v is None else TEXT, 'center')
    # ---- gyro
    gx0, gy0 = LX, 384
    if live:
        ax, ay, az = t['ax'], t['ay'], t['az']
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        yaw = t['yaw']
    else:
        ax = ay = az = roll = pitch = yaw = 0.0
    ui.board3d(canvas, fonts, gx0 + 170, gy0 + 150, 42, yaw, pitch, roll, live)
    x = gx0 + 332
    for k, (lab, v) in enumerate((('YAW', yaw), ('PITCH', pitch), ('ROLL', roll))):
        text(canvas, fonts['s'], lab, (x, gy0 + 44 + k * 38), MUTED)
        text(canvas, fonts['l'], ('%+6.1f' % v + chr(176)) if live else '   --', (x + 54, gy0 + 38 + k * 38), TEXT if live else MUTED)
    ok = live and bool(t['who'])
    ui.led(canvas, (x + 6, gy0 + 168), ok, ui.OK, 6)
    text(canvas, fonts['s'], ('chip id 0x%02X' % t['who']) if ok else 'chip NOT FOUND' if live else 'chip -', (x + 20, gy0 + 161), TEXT if ok else (ui.BAD if live else MUTED))
    text(canvas, fonts['s'], ('temp %.1f C' % t['temp']) if live else 'temp -', (x, gy0 + 186), TEXT if live else MUTED)
    cal = live and bool(l.telemetry.get('flags', 0) & cm.F_GYRO_CAL)
    text(canvas, fonts['s'], 'zeroing... keep still' if cal else ('yaw zero set at power-up' if live else ''), (x, gy0 + 206), ui.WARN if cal else MUTED)
    bx, bw = gx0 + 14, 520 - 28
    text(canvas, fonts['s'], 'ANGULAR RATE  (deg/s)', (bx, gy0 + 246), MUTED)
    for k, (lab, key, col) in enumerate((('X roll', 'gx', (235, 80, 70)), ('Y pitch', 'gy', (80, 210, 100)), ('Z yaw', 'gz', (90, 150, 245)))):
        ui.labelled_bar(canvas, fonts, bx, gy0 + 266 + k * 21, bw, (t[key] / 250.0) if live else 0, lab, ('%+7.1f' % t[key]) if live else '     --', col, True, 74, 80)
    text(canvas, fonts['s'], 'ACCELERATION  (g)', (bx, gy0 + 338), MUTED)
    for k, (lab, key, col) in enumerate((('X', 'ax', (235, 80, 70)), ('Y', 'ay', (80, 210, 100)), ('Z', 'az', (90, 150, 245)))):
        ui.labelled_bar(canvas, fonts, bx, gy0 + 358 + k * 21, bw, (t[key] / 2.0) if live else 0, lab, ('%+6.2f' % t[key]) if live else '    --', col, True, 74, 80)
    text(canvas, fonts['s'], 'flat on the table: Z about +1.00 g, X and Y about 0', (bx, gy0 + 420), MUTED)
    # ---- battery
    vb = l.telemetry.get('vbat_mv') if live else None
    volts = None if vb is None else vb / 1000.0                    # shown whatever it is: a wrong reading tells you what to check
    bcx = 544 + 75
    ui.battery(canvas, fonts, 544 + 14, gy0 + 34, 56, 270, volts)
    low = bool(live and l.telemetry['flags'] & cm.F_LOWBAT)
    text(canvas, fonts['l'], '--' if volts is None else '%.2f V' % volts, (bcx, gy0 + 330), (ui.BAD if low else TEXT) if volts else MUTED, 'center')
    if volts is not None and volts < 4.0:                          # a 2S pack is never below 6 V: nothing is feeding the sense wire
        ratio = app.cfg('ratio') or 3.0
        text(canvas, fonts['m'], 'NO BATTERY?', (bcx, gy0 + 362), ui.WARN, 'center')
        for k, ln in enumerate(('pin 34 reads %d mV' % round(volts * 1000 / ratio), 'Is the battery on and', 'the divider wired to', 'GPIO 34 and 7.4 V?')):
            text(canvas, fonts['s'], ln, (bcx, gy0 + 384 + k * 16), MUTED, 'center')
    elif volts is not None:
        text(canvas, fonts['m'], '%d %%' % round(max(0, min(1, (volts - 6.0) / 2.4)) * 100), (bcx, gy0 + 358), TEXT, 'center')
        text(canvas, fonts['s'], 'cell %.2f V' % (volts / 2), (bcx, gy0 + 382), MUTED, 'center')
        text(canvas, fonts['m'], 'LOW BATTERY' if low else 'OK', (bcx, gy0 + 408), ui.BAD if low else ui.OK, 'center')
    else:
        text(canvas, fonts['s'], 'no reading', (bcx, gy0 + 358), MUTED, 'center')
    # ---- tuning values: what the robot holds now
    c = l.tune_cfg if l.connected else {}
    y = gy0 + 308
    text(canvas, fonts['s'], 'IN THE ROBOT NOW  (kept after power off)', (706 + 12, y), MUTED)
    lines = ['MIN_PWM = %d;' % c['minpwm'], 'INVERT_LEFT = %s;  INVERT_RIGHT = %s;' % ('true' if c['invl'] else 'false', 'true' if c['invr'] else 'false'),
             'servo 1: %d .. %d deg    servo 2: %d .. %d deg' % (c['lo1'], c['hi1'], c['lo2'], c['hi2']), 'VBAT_RATIO = %.3f;' % c['ratio']] if c else ['-']
    for k, ln in enumerate(lines):
        text(canvas, fonts['s'], ln, (706 + 12, y + 22 + k * 18), TEXT if c else MUTED)
    text(canvas, fonts['s'], 'Find start power: arm, press it, the wheel', (706 + 12, y + 96), MUTED)
    text(canvas, fonts['s'], 'speeds up slowly; press again when it moves.', (706 + 12, y + 112), MUTED)


def draw_right(canvas, fonts, app):
    pygame.draw.rect(canvas, PANEL, (RX, TOP, RW, BOT - TOP), border_radius=8)
    if app.tab == 'link':
        draw_link(canvas, fonts, app)
    else:
        for (y, h, t) in ((104, 74, 'DRIVE'), (184, 96, 'VIEW OVERLAYS   (switch + opacity)'), (288, 98, 'ACTIONS'), (394, 68, 'WINDOW'), (470, 136, 'ROBOT  (BACK button = arm / disarm)')):
            ui.card(canvas, (X0, y, WW, h), t, fonts)
        text(canvas, fonts['s'], 'RT = full speed %d cm/s. Raise this to go faster' % sm.MAX_SPEED, (X0 + 10, 572), MUTED)
        text(canvas, fonts['s'], 'without holding RT (30 = RT not needed).', (X0 + 10, 588), MUTED)


def plot(canvas, fonts, rect, series, now, win, lo=None, hi=None, labels=None, minspan=10.0):
    """A scrolling line graph. series = [(colour, [(t, v), ...])]; the newest sample is at the right edge. lo / hi fixed, or None = automatic."""
    x, y, w, h = rect
    pygame.draw.rect(canvas, (22, 26, 32), rect, border_radius=4)
    pts = [(c, [(t, v) for t, v in data if now - win <= t <= now + 0.05]) for c, data in series]
    vals = [v for _, p in pts for _, v in p]
    if lo is None:
        a, b = (min(vals), max(vals)) if vals else (-1.0, 1.0)
        a, b = min(a, 0.0), max(b, 0.0)
        if b - a < minspan: m = (a + b) / 2; a, b = m - minspan / 2, m + minspan / 2
        pad = (b - a) * 0.08; lo, hi = a - pad, b + pad
    Y = lambda v: y + h - (max(lo, min(hi, v)) - lo) / (hi - lo) * h
    for k in range(5):                                                      # horizontal grid with the value on the left
        gy = y + h * k / 4.0
        pygame.draw.line(canvas, (44, 50, 58), (x, gy), (x + w, gy), 1)
        v = hi - (hi - lo) * k / 4.0
        lab = labels[k] if labels else ('%+.0f' % v if abs(hi - lo) >= 8 else '%+.1f' % v)
        text(canvas, fonts['s'], lab, (x - 6 - fonts['s'].render(lab, True, MUTED).get_width(), gy), MUTED, 'mid')
    for sec in range(0, int(win) + 1, 2):                                   # vertical grid every 2 s
        gx = x + w - sec / win * w
        pygame.draw.line(canvas, (36, 42, 50), (gx, y), (gx, y + h), 1)
    if lo < 0 < hi: pygame.draw.line(canvas, (110, 120, 132), (x, Y(0)), (x + w, Y(0)), 1)
    for c, p in pts:
        xy = [(x + w - (now - t) / win * w, Y(v)) for t, v in p]
        for a, b in zip(xy, xy[1:]): pygame.draw.line(canvas, c, a, b, 2)
    text(canvas, fonts['s'], 'last %d s' % win, (x + w - 4, y + h - 15), MUTED, 'tr')
    if not vals: text(canvas, fonts['s'], 'no data yet', (x + w // 2, y + h // 2), MUTED, 'center')


def draw_orient(canvas, fonts, cx, cy, R, yaw, target, mode, vl, vr, live):
    """Top view of the robot at its real heading. Up = where it faced at power-up (yaw 0), turning LEFT raises yaw. Amber = the heading the PID wants."""
    pygame.draw.circle(canvas, (26, 30, 36), (cx, cy), R)
    pygame.draw.circle(canvas, ui.LINE, (cx, cy), R, 2)
    P = lambda a, r: (cx - r * math.sin(math.radians(a)), cy - r * math.cos(math.radians(a)))
    for a in range(0, 360, 15):
        pygame.draw.line(canvas, (150, 160, 172) if a % 45 == 0 else (84, 92, 104), P(a, R - 1), P(a, R - (11 if a % 45 == 0 else 6)), 2 if a % 90 == 0 else 1)
    for a, lab in ((0, '0'), (90, '+90'), (180, '180'), (270, '-90')):
        text(canvas, fonts['s'], lab, P(a, R - 24), MUTED, 'center')
    if live and mode > 0:
        err = (target - yaw + 180) % 360 - 180
        n = max(2, int(abs(err) / 4))
        wedge = [(cx, cy)] + [P(yaw + err * k / n, R - 30) for k in range(n + 1)]
        pygame.draw.polygon(canvas, (86, 62, 28), wedge)
        pygame.draw.line(canvas, AMBER, (cx, cy), P(target, R - 8), 2)
        pygame.draw.circle(canvas, AMBER, [int(v) for v in P(target, R - 8)], 6)
    a = math.radians(yaw)
    fx, fy, rx, ry = -math.sin(a), -math.cos(a), math.cos(a), -math.sin(a)
    L = lambda lx, ly: (cx + lx * rx + ly * fx, cy + lx * ry + ly * fy)
    for side, v in ((-1, vl), (1, vr)):                                      # the wheels glow with their speed: green forward, amber reverse
        col = (60, 70, 80) if abs(v) < 2 else ((60, 200, 140) if v > 0 else AMBER)
        pygame.draw.polygon(canvas, col, [L(side * 21 - 4, -16), L(side * 21 + 4, -16), L(side * 21 + 4, 16), L(side * 21 - 4, 16)])
    pygame.draw.polygon(canvas, (88, 100, 114) if live else (60, 66, 74), [L(-17, -22), L(17, -22), L(17, 22), L(-17, 22)])
    pygame.draw.polygon(canvas, ACCENT if live else MUTED, [L(0, 36), L(-12, 14), L(12, 14)])
    pygame.draw.circle(canvas, (255, 255, 255), [int(v) for v in P(yaw, R - 4)], 4)


def draw_pid(canvas, fonts, app):
    """The PID page. Positions must match the widgets built in Screen.build."""
    pygame.draw.rect(canvas, PANEL, PAGE, border_radius=8)
    text(canvas, fonts['m'], 'PID  |  gyro heading control', (LX + 14, TOP + 8), ACCENT)
    msg, col = app.pid_status()
    text(canvas, fonts['s'], msg, (LX + PAGE[2] - 14, TOP + 10), col, 'tr')
    l = app.link
    h = l.hold if app.hold_fresh() else None
    t = l.telemetry if app.telemetry_ok() else {}
    live = h is not None and bool(t)
    now = l.clock()
    yaw, gz = t.get('yaw', 0.0), t.get('gz', 0.0)
    ui.card(canvas, (LX, 92, 330, 300), 'ORIENTATION  (the real robot, from its gyro)', fonts)
    ui.card(canvas, (LX, 400, 330, 426), 'PID SETTINGS  (kept in the robot)', fonts)
    ui.card(canvas, (354, 92, 690, 190), 'HEADING  (deg)', fonts)
    ui.card(canvas, (354, 290, 690, 150), 'ERROR and CORRECTION  (full scale: error +-20 deg, correction +-25 %)', fonts)
    ui.card(canvas, (354, 448, 690, 150), 'WHEELS  commanded speed (%)', fonts)
    ui.card(canvas, (354, 606, 690, 220), 'STEP TESTS  (move any stick to stop a test)', fonts)
    mode = int(h['mode']) if h else 0
    tgt = h['target'] if h else yaw
    draw_orient(canvas, fonts, LX + 165, 226, 96, yaw, tgt, mode, t.get('vl', 0), t.get('vr', 0), live)
    word = ('OFF', 'READY', 'HOLDING THE LINE', 'TURNING TO TARGET')[mode] if h else 'no data'
    text(canvas, fonts['m'], word, (LX + 165, 340), (MUTED, ui.OK, ui.OK, AMBER)[mode] if h else MUTED, 'center')
    text(canvas, fonts['s'], 'yaw %+7.1f   target %+7.1f   error %+5.1f deg' % (yaw, tgt, (yaw - tgt) if h else 0.0), (LX + 165, 362), TEXT, 'center')
    text(canvas, fonts['s'], 'turn rate %+6.1f deg/s    balance %s' % (gz, ('%+.2f %%' % h['bal']) if h else '-'), (LX + 165, 380), MUTED, 'center')
    W = 12.0
    H = list(app.hist)
    S = lambda key, k=1.0: [(r['t'], r[key] * k) for r in H]
    CY, WH = (90, 200, 230), (240, 150, 60)
    plot(canvas, fonts, (406, 124, 626, 148), [(AMBER, S('target')), (CY, S('yaw'))], now, W)
    text(canvas, fonts['s'], 'yaw', (940, 98), CY); text(canvas, fonts['s'], 'target', (980, 98), AMBER)
    plot(canvas, fonts, (406, 322, 626, 108), [((235, 235, 240), S('err', 1 / 20.0)), (AMBER, S('corr', 1 / 25.0))], now, W, -1.0, 1.0, ('+20/+25', '', '0', '', '-20/-25'))
    text(canvas, fonts['s'], 'error', (910, 296), (235, 235, 240)); text(canvas, fonts['s'], 'correction', (960, 296), AMBER)
    plot(canvas, fonts, (406, 480, 626, 108), [((60, 200, 140), S('vl')), (WH, S('vr'))], now, W, minspan=40.0)
    text(canvas, fonts['s'], 'left', (940, 454), (60, 200, 140)); text(canvas, fonts['s'], 'right', (985, 454), WH)
    r = [x for x in H if x['t'] >= now - 10]
    text(canvas, fonts['s'], 'biggest error in the last 10 s: %s' % ('%.1f deg' % max(abs(x['err']) for x in r) if r else '-'), (366, 680), TEXT)
    text(canvas, fonts['s'], app.pid_result or 'no test run yet', (366, 700), AMBER if app.pid_test else TEXT)
    tips = ('Straight: the error line should stay inside +-2 deg. A slow lean = raise Ki; a quick wobble = lower Kp or raise Kd.',
            'Turn: overshoot = lower Turn strength or raise Turn damping; stops short = raise Turn strength.',
            'If a turn runs away from its target, yaw has the wrong sign: press Flip gyro direction.')
    y = 728
    for tip in tips:
        for ln in textwrap.wrap(tip, 100):
            text(canvas, fonts['s'], ln, (366, y), MUTED); y += 17


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
    if app.mode == TUNE:
        draw_tune(canvas, fonts, app)
    elif app.mode == PID:
        draw_pid(canvas, fonts, app)
    else:
        draw_left(canvas, fonts, big, st, log_lines)
        draw_centre(canvas, fonts, cam, scene, app)
    draw_right(canvas, fonts, app)
    text(canvas, fonts['s'], 'Space E-STOP   F11 fullscreen   ARM OUTPUT is on the LINK tab   ESC quit' if app.mode in (TUNE, PID) else
         'Space E-STOP   BACK arm/disarm   F11 fullscreen   V view   T drive mode   L turn   0 re-zero   P timer   C cubes   ESC quit', (M + 4, CH - 22), MUTED)
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
                S.release(app)
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
        app.pre_drive()
        app.tick_pad(st)
        sim.drive(st, dt)
        app.tick_link(dt)
        elog.update(st)
        app.timer_update(dt, st)
        render(canvas, fonts, big, cam, scene, app, S, st, elog.lines)
        ui.present(window, canvas)
        pygame.display.flip()


if __name__ == '__main__':
    main()
