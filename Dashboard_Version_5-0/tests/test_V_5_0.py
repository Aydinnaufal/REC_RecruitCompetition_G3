"""test_V_5_0.py - checks for the V 5.0 link layer, Port Doctor, Bluetooth LE / Classic transports, dashboard logic and the ESP32 firmware. No window, no hardware.
Run from this folder:  python test_V_5_0.py        (g++ is needed for the firmware-in-the-loop tests; they are skipped without it)

Stand-ins (all in this folder, used only when the real thing is not installed):
  testshim/serial   a tiny pyserial replacement on a pseudo-terminal      testshim/bleak   a fake bleak whose "robot" is comms.VirtualRobotTransport
  fw_host/          a fake Arduino core + BLE library so esp32_firmware_V_5_0.ino compiles and runs on a PC, its radio being a pty
What they do NOT cover: real Windows Bluetooth, real bleak / pyserial behaviour, the real ESP32 BLE and Classic stacks, real hardware.
"""
import os, subprocess, sys, tempfile, threading, time, pty, tty
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                         # Dashboard_Version_5-0 : the dashboard, comms_V_5_0.py, the other modules
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
SHIM = os.path.join(HERE, 'testshim')
try:
    import serial                                    # real pyserial if present
    REAL_PYSERIAL = True
except ImportError:
    sys.path.insert(0, SHIM)
    import serial
    REAL_PYSERIAL = False
try:
    import bleak
    REAL_BLEAK = True
except ImportError:
    if SHIM not in sys.path: sys.path.insert(0, SHIM)
    import bleak
    REAL_BLEAK = False
import comms_V_5_0 as cm

TESTS = []
def test(f): TESTS.append(f); return f


class Clock:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t
    def run(self, link, sec, dt=1 / 60.0):
        for _ in range(int(round(sec / dt))):
            self.t += dt
            link.update()


def connected(profile='rec1', **kw):
    c = Clock(); l = cm.Link(c)
    l.connect(lambda: cm.VirtualRobotTransport('simple', c, **kw), profile)
    return c, l


def wait_for(cond, link, timeout=6.0, step=0.01):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        link.update()
        if cond(): return True
        time.sleep(step)
    return False


# ------------------------------------------------------------------------------------------ framing and helpers
@test
def framing_roundtrip_and_noise():
    f = cm.encode('CMD', 7, -50, 50, 0, 100, 1)
    assert cm.FrameParser().feed(f) == [('frame', 'CMD', ['7', '-50', '50', '0', '100', '1'])]
    p = cm.FrameParser()
    assert p.feed(f[:5]) + p.feed(f[5:]) == [('frame', 'CMD', ['7', '-50', '50', '0', '100', '1'])], 'split across reads'
    assert p.feed(b'boot ok\r\n') == [('text', 'boot ok')]
    bad = bytearray(f); bad[4] ^= 1
    assert p.feed(bytes(bad))[0][0] == 'bad' and p.bad == 1

@test
def friendly_errors_are_plain_sentences():
    fe = cm.friendly_error
    assert 'busy' in fe(PermissionError(13, 'Access is denied.'))
    assert 'Bluetooth did not answer' in fe(OSError('[WinError 121] The semaphore timeout period has expired.'))
    assert 'not found' in fe(IOError("could not open port 'COM9': FileNotFoundError(2, 'The system cannot find the file specified.')"))
    assert fe(ValueError('something odd')) == 'something odd'

@test
def bluetooth_ports_are_recognised_and_listed_first():
    assert cm.is_bluetooth_port('COM5', 'Standard Serial over Bluetooth link (COM5)', 'BTHENUM\\{00001101}_LOCALMFG&0002')
    assert cm.is_bluetooth_port('/dev/rfcomm0')
    assert not cm.is_bluetooth_port('COM3', 'USB-SERIAL CH340 (COM3)', 'USB VID:PID=1A86:7523')
    sc = cm.scan_ports(comports=[P3('COM3', 'USB-SERIAL CH340', 'USB VID:PID=1A86:7523'), P3('COM1', 'Communications Port', 'ACPI\\PNP0501'),
                                 P3('COM5', 'Standard Serial over Bluetooth link', 'BTHENUM\\x')], registry={})
    ports = sc.ports
    assert [p.device for p in ports] == ['COM5', 'COM1', 'COM3'] and ports[0].bluetooth
    assert [p.device for p in ports if cm.probeable(p)] == ['COM5', 'COM3'], 'motherboard COM1 is not probed'

class P3:                                              # what pyserial's comports() items look like
    def __init__(s, d, desc, hwid): s.device, s.description, s.hwid = d, desc, hwid


# ------------------------------------------------------------------------------------------ link behaviour (virtual robot)
@test
def handshake_telemetry_latency():
    c, l = connected(latency_ms=40, jitter_ms=0)
    assert l.state == cm.Link.HANDSHAKE
    c.run(l, 3.0)
    assert l.state == cm.Link.CONNECTED and l.hello['robot'] == 'simple' and l.hello['version'] == 'V_5_0'
    s = l.stats
    assert 18 < s['tx_rate'] < 22 and 18 < s['tel_rate'] < 22, (s['tx_rate'], s['tel_rate'])
    assert 35 < s['rtt_avg'] < 140 and s['bad'] == 0 and l.loss_percent() == 0 and l.quality() >= 3

@test
def hello_is_repeated_until_the_robot_is_ready():
    """V_4_0 asked once and gave up. A robot that only starts listening after 1.2 s (Bluetooth still connecting) must still be found."""
    c, l = connected(silent_for=1.2)
    c.run(l, 1.0)
    assert l.state == cm.Link.HANDSHAKE and not l.hello
    c.run(l, 2.0)
    assert l.state == cm.Link.CONNECTED and l.hello.get('fw') == 'fake-esp32'
    assert l.stats['tx'] > 20 and not any(k == 'err' and 'silent' in m for _, k, m in l.log), 'never gave up'

@test
def silent_robot_shows_no_reply_then_recovers():
    c, l = connected(silent_for=6.0)
    c.run(l, 3.0)
    assert l.state == cm.Link.NO_REPLY and l.connected and l.quality() == 1
    assert any(k == 'err' and 'silent' in m for _, k, m in l.log)
    c.run(l, 5.0)                                        # the robot starts listening at t = 6 s; HELLO? is still being sent every second
    assert l.state == cm.Link.CONNECTED and l.hello, l.state

@test
def disarmed_sends_zeros_armed_drives_and_failsafe():
    c, l = connected()
    l.set_command(cm.Command(60, 60, 0, 0, True))
    c.run(l, 2.0)
    assert l.telemetry['vl'] == 0 and not (l.telemetry['flags'] & cm.F_ARMED)
    l.armed = True
    c.run(l, 1.5)
    assert l.telemetry['vl'] > 50 and l.telemetry['flags'] & cm.F_ARMED
    robot = l.transport.robot
    l.transport.to_robot.clear(); l._next_tx = 1e9        # the laptop stops talking
    c.run(l, 1.0)
    assert robot.failsafe and abs(robot.vl) < 5 and l.telemetry['flags'] & cm.F_FAILSAFE

@test
def reconnect_never_rearms():
    c, l = connected()
    l.auto_reconnect = True
    l.armed = True; l.set_command(cm.Command(70, 70, 0, 0, True))
    c.run(l, 2.0); assert l.telemetry['vl'] > 50
    l.transport.cut = True
    c.run(l, 0.2)
    assert l.state == cm.Link.ERROR and not l.armed, 'a failed link disarms'
    assert any('disarmed' in m for _, k, m in l.log)
    c.run(l, 4.0)                                        # auto-reconnect builds a new transport
    assert l.state == cm.Link.CONNECTED and not l.armed and abs(l.telemetry['vl']) < 3
    l.armed = True; l.disconnect()
    assert not l.armed, 'disconnect disarms too'

@test
def estop_and_stale():
    c, l = connected()
    l.armed = True; l.set_command(cm.Command(80, 80, 0, 0, True))
    c.run(l, 2.0)
    l.estop(); assert not l.armed
    c.run(l, 1.0); assert abs(l.telemetry['vl']) < 3
    l.transport.silent_until = c.t + 100                 # the robot goes completely quiet (a PONG would otherwise keep the link alive)
    c.run(l, 1.2); assert l.state == cm.Link.STALE and l.quality() == 1
    l.transport.silent_until = 0.0
    c.run(l, 0.5); assert l.state == cm.Link.CONNECTED

@test
def legacy_profile_no_handshake():
    c = Clock(); l = cm.Link(c)
    class Sink(cm.Transport):
        name = 'sink'
        def __init__(s): super().__init__(); s.sent = b''
        def open(s): s.is_open = True
        def write(s, d): s.sent += d
    sink = Sink()
    l.connect(lambda: sink, 'legacy')
    assert l.state == cm.Link.CONNECTED
    l.armed = True; l.set_command(cm.Command(70, 70, 90, 0, True))
    c.run(l, 1.0)
    assert b'F' in sink.sent and b'C' in sink.sent and b'$' not in sink.sent
    l.disconnect(); assert sink.sent.endswith(b'S')


# ------------------------------------------------------------------------------------------ Auto-find
class Silent(cm.Transport):
    name = 'silent'
    def __init__(self, fail=None): super().__init__(); self.fail = fail; self.handshake_timeout = 0.6; self.open_budget = 1.0
    def open(self):
        if self.fail: self.last_error = self.fail; raise IOError(self.fail)
        self.is_open = True
    def write(self, d): pass

class Slow(cm.Transport):
    name = 'slow'
    def __init__(self): super().__init__(); self.handshake_timeout = 0.6; self.open_budget = 0.3; self._stop = False
    def start_open(self):
        self.opening = True
        threading.Thread(target=lambda: (time.sleep(5), setattr(self, 'opening', False)), daemon=True).start()

@test
def scanner_picks_the_port_that_answers():
    P = cm.PortInfo
    ports = [P('COM3', 'USB', 'USB VID', False), P('COM7', 'BT', 'BTHENUM', True), P('COM5', 'BT', 'BTHENUM', True), P('COM9', 'BT', 'BTHENUM', True)]
    def factory(p):
        if p.device == 'COM5': return cm.VirtualRobotTransport('simple', time.monotonic, latency_ms=10)
        if p.device == 'COM9': return Slow()
        return Silent(fail='port busy' if p.device == 'COM7' else None)
    sc = cm.PortScanner(ports, factory)
    assert [p.device for p in sc.ports] == ['COM5', 'COM7', 'COM9', 'COM3'], 'Bluetooth first'
    sc.start()
    t = time.monotonic()
    while sc.running and time.monotonic() - t < 8: time.sleep(0.02)
    assert not sc.running and sc.found and sc.found[0].device == 'COM5' and sc.found[1]['fw'] == 'fake-esp32', sc.message
    assert 'COM5' in sc.message
    # nobody answers
    sc2 = cm.PortScanner([P('COM7', 'BT', 'BTHENUM', True), P('COM9', 'BT', 'BTHENUM', True), P('COM3', 'USB', 'USB VID', False)], lambda p: Slow() if p.device == 'COM9' else Silent(fail='port busy' if p.device == 'COM7' else None))
    sc2.start()
    t = time.monotonic()
    while sc2.running and time.monotonic() - t < 8: time.sleep(0.02)
    assert sc2.found is None and 'no robot answered' in sc2.message
    res = dict((d, info) for d, ok, info in sc2.results)
    assert 'busy' in res['COM7'] and res['COM9'] == 'open timed out' and res['COM3'] == 'no answer', res

@test
def scanner_with_no_ports_and_cancel():
    sc = cm.PortScanner([]); sc.start(); time.sleep(0.2)
    assert not sc.running and 'pair the robot' in sc.message
    P = cm.PortInfo
    sc = cm.PortScanner([P('COM3', 'USB', 'USB VID', False), P('COM4', 'USB', 'USB VID', False)], lambda p: Silent()); sc.start(); sc.cancel(); time.sleep(1.0)
    assert not sc.running and sc.found is None


# ------------------------------------------------------------------------------------------ real SerialTransport, no robot
@test
def serial_open_failure_is_reported_not_raised():
    c = Clock(); l = cm.Link(c)
    l.connect(lambda: cm.SerialTransport('/dev/does-not-exist', 115200), 'rec1')
    assert l.state in (cm.Link.CONNECTING, cm.Link.ERROR), 'opening runs in a thread: connect() returns at once'
    assert wait_for(lambda: l.state == cm.Link.ERROR, l, 3)
    assert 'not found' in l.last_error or 'could not open' in l.last_error, l.last_error

@test
def bluetooth_port_retries_then_gives_up():
    t = cm.SerialTransport('/dev/does-not-exist', 115200, bluetooth=True, retry_wait=0.05)
    assert t.retries == 3 and t.handshake_timeout == 8.0 and '(BT)' in t.name
    t0 = time.monotonic(); t.start_open()
    while t.opening and time.monotonic() - t0 < 5: time.sleep(0.01)
    assert not t.is_open and t.last_error and time.monotonic() - t0 >= 0.1


# ------------------------------------------------------------------------------------------ real firmware on a pty
FW = {}
def build_firmware(mode=1):
    """Compiles the real sketch for a PC. mode: 0 = BT_NONE, 1 = BT_CLASSIC, 2 = BT_BLE (with the BLE stand-in)."""
    if mode in FW: return FW[mode]
    exe = os.path.join(HERE, 'fw_host', 'fw_host_%d' % mode)
    src = os.path.join(ROOT, 'esp32_firmware_V_5_0', 'esp32_firmware_V_5_0.ino')
    extra = ['-DHAVE_BLE_STUB'] if mode == 2 else []
    try:
        r = subprocess.run(['g++', '-std=gnu++17', '-O1', '-Wall', '-Wno-unused-variable', '-DBT_MODE=%d' % mode] + extra + ['-I' + os.path.join(HERE, 'fw_host'), '-include', 'Arduino.h', '-x', 'c++', src,
                            os.path.join(HERE, 'fw_host', 'harness.cpp'), '-o', exe], capture_output=True, text=True)
    except FileNotFoundError:
        FW[mode] = None; FW['err'] = 'g++ not available'
        return None
    sketch_warnings = [l for l in r.stderr.splitlines() if 'esp32_firmware_V_5_0.ino' in l and 'warning' in l]
    FW['warn%d' % mode] = sketch_warnings
    FW[mode] = exe if r.returncode == 0 else None
    FW['err'] = r.stderr
    return FW[mode]


class FirmwareRig:
    """The real sketch, its Bluetooth stream on a pty, plus a reader thread for the PINS lines it prints."""
    def __init__(self, mode=1):
        exe = build_firmware(mode)
        assert exe, 'firmware did not compile:\n' + FW.get('err', '')
        self.master, slave = pty.openpty()
        tty.setraw(self.master); tty.setraw(slave)
        self.slave_path = os.ttyname(slave)
        self.slave = slave                                  # keep it open so the pty survives while the link opens and closes it
        self.proc = subprocess.Popen([exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE, pass_fds=(self.master,), preexec_fn=lambda: os.dup2(self.master, 3) if self.master != 3 else None, bufsize=0)
        self.pins, self.text, self._stop = {}, [], False
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for raw in iter(self.proc.stdout.readline, b''):
            line = raw.decode('ascii', 'replace').strip()
            if line.startswith('PINS'):
                self.pins = dict(kv.split('=') for kv in line.split()[1:]); self.pins = {k: int(v) for k, v in self.pins.items()}
            else:
                self.text.append(line)

    def gyro(self, raw):
        self.proc.stdin.write(('gz %d\n' % raw).encode())

    def send(self, line):
        self.proc.stdin.write((line + '\n').encode())

    def close(self):
        try: self.proc.stdin.write(b'quit\n')
        except Exception: pass
        try: self.proc.wait(timeout=2)
        except Exception: self.proc.kill()
        for fd in (self.master, self.slave):
            try: os.close(fd)
            except OSError: pass

    def link(self, bluetooth=True, profile='rec1'):
        l = cm.Link(time.monotonic)
        l.connect(lambda: cm.SerialTransport(self.slave_path, 115200, bluetooth=bluetooth, retry_wait=0.05), profile)
        return l


def rig_mode(mode):
    def deco(f):
        def wrapper():
            if not build_firmware(mode):
                if 'not available' in FW.get('err', '') or not FW.get('err'):
                    print('  (skipped: g++ not available)'); return
                raise AssertionError('firmware did not compile:\n' + FW['err'])
            rig = FirmwareRig(mode)
            try:
                time.sleep(0.2)
                f(rig)
            finally:
                rig.close()
        wrapper.__name__ = f.__name__
        return wrapper
    return deco

with_rig = rig_mode(1)                                # Bluetooth Classic build: the stream is a pty that looks like a COM port
with_ble_rig = rig_mode(2)                            # Bluetooth LE build with the BLE stand-in


@test
@with_rig
def firmware_handshake_and_telemetry_over_the_port(rig):
    l = rig.link()
    assert l.state in (cm.Link.CONNECTING, cm.Link.HANDSHAKE)          # the port opens in a thread: it may already be open
    assert wait_for(lambda: l.state == cm.Link.CONNECTED and l.stats['tel'] > 20, l, 6), (l.state, list(l.log)[-4:])
    assert l.hello == dict(proto='1', fw='rec-esp32', robot='simple', version='V_5_0'), l.hello
    s = l.stats
    time.sleep(1.0)
    for _ in range(100): l.update(); time.sleep(0.01)
    assert 14 < l.stats['tel_rate'] < 26 and l.stats['bad'] == 0, (l.stats['tel_rate'], l.stats['bad'])
    assert l.stats['rtt_avg'] is not None and l.stats['rtt_avg'] < 150
    t = l.telemetry
    assert 7800 < t['vbat_mv'] < 8200 and not (t['flags'] & cm.F_LOWBAT), t
    assert l.transport.dropped == 0
    l.disconnect()

@test
@with_rig
def firmware_motors_only_move_when_armed_and_stop_on_failsafe(rig):
    l = rig.link()
    l.set_command(cm.Command(60, 60, 0, 0, True))
    assert wait_for(lambda: l.state == cm.Link.CONNECTED, l, 6)
    for _ in range(40): l.update(); time.sleep(0.01)
    assert rig.pins['ena'] == 0 and rig.pins['enb'] == 0 and l.telemetry['vl'] == 0, 'disarmed: no output'
    l.armed = True
    assert wait_for(lambda: rig.pins.get('ena', 0) > 0, l, 2)
    p = rig.pins
    assert p['in1'] == 1 and p['in2'] == 0 and p['in3'] == 1 and p['in4'] == 0, p        # both forward
    assert abs(p['ena'] - 153) <= 3 and p['ena'] == p['enb'], p                              # 60 % of 255 (map(1..100 -> 0..255) gives 151)
    assert wait_for(lambda: l.telemetry['vl'] == 60 and l.telemetry['flags'] & cm.F_ARMED, l, 1)
    l.set_command(cm.Command(-50, 50, 100, 0, True))                                       # spin on the spot, close the claw
    assert wait_for(lambda: rig.pins['in1'] == 0 and rig.pins['in2'] == 1 and rig.pins['in3'] == 1 and rig.pins['in4'] == 0, l, 2), rig.pins
    assert wait_for(lambda: rig.pins['grip'] == 30, l, 2), rig.pins                       # closed = 30 degrees
    assert wait_for(lambda: l.telemetry['grip'] == 100, l, 1), l.telemetry
    # the laptop goes quiet: the firmware stops the wheels by itself
    t0 = time.monotonic(); l._next_tx = 1e12
    while rig.pins['ena'] != 0 and time.monotonic() - t0 < 2: time.sleep(0.005)
    stopped = time.monotonic() - t0
    assert rig.pins['ena'] == 0 and rig.pins['enb'] == 0, rig.pins
    assert 0.25 < stopped < 0.9, 'failsafe is 400 ms, took %.2f s' % stopped
    l._next_tx = 0
    assert wait_for(lambda: l.telemetry['flags'] & cm.F_FAILSAFE == 0 and rig.pins['ena'] > 0, l, 2), 'commands resume after the link comes back'
    l.estop()
    assert wait_for(lambda: rig.pins['ena'] == 0 and not (l.telemetry['flags'] & cm.F_ARMED), l, 1)
    l.disconnect()

@test
@with_rig
def firmware_gyro_zero_and_yaw_sign(rig):
    l = rig.link()
    assert wait_for(lambda: l.state == cm.Link.CONNECTED, l, 6)
    assert wait_for(lambda: l.telemetry and l.telemetry['flags'] & cm.F_GYRO_OK, l, 3), 'gyro calibration (1 s) finishes'
    rig.gyro(1310)                                                                         # 10 deg/s counter-clockwise
    time.sleep(2.0)
    for _ in range(20): l.update(); time.sleep(0.01)
    y = l.telemetry['yaw']
    assert 15 < y < 24 and 9 < l.telemetry['gz'] < 11, (y, l.telemetry)
    rig.gyro(0); l.send_frame('ZERO')
    assert wait_for(lambda: abs(l.telemetry['yaw']) < 1.0 and any('ACK ZERO' in m for _, k, m in l.log), l, 2)
    l.disconnect()

@test
@with_rig
def firmware_ignores_garbage_and_old_laptops(rig):
    l = rig.link()
    assert wait_for(lambda: l.state == cm.Link.CONNECTED, l, 6)
    l.transport.ser.write(b'\xff\x00 garbage without newline')
    l.transport.ser.write(b'\n$CMD,1,100,100,0,0,1*00\n')                                 # wrong checksum: must not arm the robot
    l.transport.ser.write(cm.encode('FUTURE', 1, 2, 3))                                    # unknown frame: ignored
    l.transport.ser.write(b'A' * 400 + b'\n')                                              # runaway line
    for _ in range(60): l.update(); time.sleep(0.01)
    assert rig.pins.get('ena', 0) == 0 and not (l.telemetry['flags'] & cm.F_ARMED) and l.state == cm.Link.CONNECTED
    l.send_frame('CFG', 'gc', 45)
    assert wait_for(lambda: any('ACK CFG' in m for _, k, m in l.log), l, 1)
    l.disconnect()

@test
@with_rig
def firmware_two_connections_in_a_row_and_never_arms_itself(rig):
    """Pair, connect, armed and driving, drop the port, connect again: the robot must come back disarmed."""
    l = rig.link()
    l.set_command(cm.Command(60, 60, 0, 0, True))
    assert wait_for(lambda: l.state == cm.Link.CONNECTED, l, 6)
    l.armed = True
    assert wait_for(lambda: rig.pins.get('ena', 0) > 0, l, 2)
    l.transport.close()                                                                    # the port vanishes under the link
    assert wait_for(lambda: l.state == cm.Link.ERROR, l, 2) and not l.armed
    t0 = time.monotonic()
    while rig.pins['ena'] != 0 and time.monotonic() - t0 < 2: time.sleep(0.01)
    assert rig.pins['ena'] == 0, 'robot stopped by its failsafe'
    l.connect(lambda: cm.SerialTransport(rig.slave_path, 115200, bluetooth=True, retry_wait=0.05), 'rec1')
    assert wait_for(lambda: l.state == cm.Link.CONNECTED and l.stats['tel'] > 5, l, 6)
    for _ in range(40): l.update(); time.sleep(0.01)
    assert not l.armed and rig.pins['ena'] == 0 and l.telemetry['vl'] == 0
    l.disconnect()

@test
@with_rig
def autofind_finds_the_firmware_on_a_port(rig):
    P = cm.PortInfo
    ports = [P('/dev/does-not-exist', 'Standard Serial over Bluetooth link', 'BTHENUM\\a', True), P(rig.slave_path, 'Standard Serial over Bluetooth link', 'BTHENUM\\b', True)]
    sc = cm.PortScanner(ports, lambda p: cm.SerialTransport(p.device, 115200, bluetooth=p.bluetooth, retries=1))
    sc.start()
    t = time.monotonic()
    while sc.running and time.monotonic() - t < 15: time.sleep(0.05)
    assert sc.found and sc.found[0].device == rig.slave_path and sc.found[1]['fw'] == 'rec-esp32', (sc.message, sc.results)
    assert sc.results[0][1] is False and 'not found' in sc.results[0][2] or 'could not open' in sc.results[0][2], sc.results



# ============================================================================================== V_5_0: ports and the Port Doctor
import contextlib, socket as _socket

@contextlib.contextmanager
def module_missing(name):
    """Makes `import name` fail inside the block (what a Python without that package does)."""
    saved = {k: v for k, v in sys.modules.items() if k == name or k.startswith(name + '.')}
    for k in saved: sys.modules.pop(k)
    sys.modules[name] = None
    try:
        yield
    finally:
        sys.modules.pop(name, None)
        sys.modules.update(saved)

BT_OUT = 'BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0002\\7&1F5E4B6F&0&A0B1C2D3E4F5_C00000000'
BT_IN = 'BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0000\\7&1F5E4B6F&0&000000000000_00000000'

@test
def missing_pyserial_is_explained_not_hidden():
    with module_missing('serial'):
        sc = cm.scan_ports(registry={})
    assert sc.ports == [] and sc.serial_version is None
    assert 'pyserial is not installed' in sc.problems[0] and sys.executable in sc.problems[0] and '-m pip install pyserial' in sc.problems[0], sc.problems
    with module_missing('serial'):
        t = cm.SerialTransport('COM5'); t.start_open()
        for _ in range(100):
            if not t.opening: break
            time.sleep(0.01)
    assert '-m pip install pyserial' in t.last_error, t.last_error

@test
def windows_registry_is_a_second_opinion():
    sc = cm.scan_ports(comports=[P3('COM3', 'USB-SERIAL CH340', 'USB VID:PID=1A86:7523')], registry={'COM3': '\\Device\\USBSER000', 'COM5': '\\Device\\BthModem0', 'COM1': '\\Device\\Serial0'})
    by = {p.device: p for p in sc.ports}
    assert set(by) == {'COM1', 'COM3', 'COM5'} and by['COM5'].bluetooth and not by['COM3'].bluetooth
    assert cm.probeable(by['COM5']) and cm.probeable(by['COM3']) and not cm.probeable(by['COM1']), 'onboard Serial0 is not probed'
    assert any('COM5' in p and 'pyserial did not list' in p for p in sc.problems)
    sc = cm.scan_ports(comports=[], registry={})
    assert sc.ports == []                                     # a PC with nothing at all: empty, and diagnose() explains it

@test
def incoming_bluetooth_ports_are_marked_and_never_used():
    sc = cm.scan_ports(comports=[P3('COM6', 'Standard Serial over Bluetooth link (COM6)', BT_IN), P3('COM5', 'Standard Serial over Bluetooth link (COM5)', BT_OUT),
                                 P3('COM7', 'Silicon Labs CP210x', 'USB VID:PID=10C4:EA60')], registry={})
    by = {p.device: p for p in sc.ports}
    assert by['COM5'].role == 'out' and by['COM5'].mac == 'A0:B1:C2:D3:E4:F5' and by['COM6'].role == 'in' and by['COM7'].role == ''
    assert [p.device for p in sc.ports] == ['COM5', 'COM7', 'COM6'], 'Incoming last'
    ps = cm.PortScanner(sc.ports)
    assert [p.device for p in ps.ports] == ['COM5', 'COM7'], 'auto-find never opens the Incoming port (it would wait for ever)'

@test
def paired_devices_are_parsed_with_their_mac():
    txt = ('REC-Robot|BTHENUM\\DEV_A0B1C2D3E4F5\\7&1&0&BLUETOOTHDEVICE_A0B1C2D3E4F5|OK\n'
           'Standard Serial over Bluetooth link|BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}_VID&0001\\DEV_A0B1C2D3E4F5|OK\n'
           'Buds|BTHENUM\\DEV_112233445566\\x|Unknown\nLE thing|BTHLE\\DEV_AABBCCDDEEFF\\y|OK\nnoise line\n')
    d = {x.mac: x for x in cm.parse_paired(txt)}
    assert d['A0:B1:C2:D3:E4:F5'].name == 'REC-Robot' and d['A0:B1:C2:D3:E4:F5'].present and d['A0:B1:C2:D3:E4:F5'].kind == 'classic'
    assert not d['11:22:33:44:55:66'].present and d['AA:BB:CC:DD:EE:FF'].kind == 'le' and len(d) == 3
    assert cm.valid_mac('A0:B1:C2:D3:E4:F5') and cm.valid_mac('a0-b1-c2-d3-e4-f5') and not cm.valid_mac('COM5') and not cm.valid_mac('A0:B1:C2')

def doctor(ports=(), registry=None, paired=(), no_serial=False):
    sc = cm.scan_ports(comports=list(ports), registry=registry or {})
    if no_serial:
        sc.serial_version = None
        sc.problems.append('pyserial is not installed for this Python (No module named serial). Run:  %s' % cm.install_hint('pyserial'))
    else:
        sc.serial_version = '3.5'
    return cm.diagnose(scan=sc, paired=list(paired))

@test
def doctor_names_the_problem_and_the_fix():
    R = cm.PairedDevice
    r = doctor(no_serial=True)
    assert any(lv == 'bad' and 'pyserial is NOT installed' in t for lv, t in r.lines) and '-m pip install pyserial' in r.steps[0]
    # paired but Windows made no Outgoing COM port: the classic problem
    r = doctor(paired=[R('REC-Robot', 'A0:B1:C2:D3:E4:F5', True, 'classic')])
    assert any(lv == 'bad' and 'no Outgoing COM port' in t for lv, t in r.lines)
    assert any('pair it again' in s for s in r.steps) and any('Bluetooth Classic (MAC)' in s for s in r.steps)
    # an LE-only robot
    r = doctor(paired=[R('REC-Robot', 'A0:B1:C2:D3:E4:F5', True, 'le')])
    assert any('Bluetooth LE' in t for lv, t in r.lines)
    # everything fine
    r = doctor(ports=[P3('COM5', 'Standard Serial over Bluetooth link', BT_OUT)], paired=[R('REC-Robot', 'A0:B1:C2:D3:E4:F5', True, 'classic')])
    assert any('use this one' in t for lv, t in r.lines) and 'Looks fine' in r.steps[0], r.steps
    txt = r.text()
    assert 'NEXT STEPS' in txt and '1. ' in txt and 'A0:B1:C2:D3:E4:F5' in txt
    # only the Incoming port exists
    r = doctor(ports=[P3('COM6', 'Standard Serial over Bluetooth link', BT_IN)], paired=[R('REC-Robot', 'A0:B1:C2:D3:E4:F5', True, 'classic')])
    assert any('INCOMING' in t for lv, t in r.lines) and any(lv == 'bad' for lv, t in r.lines)
    with module_missing('bleak'):
        r = doctor(ports=[P3('COM7', 'CP210x', 'USB VID:PID=10C4:EA60')])
    assert any('bleak is not installed' in t for lv, t in r.lines) and any('-m pip install bleak' in s for s in r.steps)


# ============================================================================================== V_5_0: Bluetooth LE
ROBOT_ADDR = 'AA:BB:CC:DD:EE:01'

def ble_world(robot='simple', fail=None, adv=None, scan_error=None):
    bleak.BleakScanner.ADV = adv if adv is not None else [('Mi Band 6', 'AA:BB:CC:DD:EE:02', -70, []), ('REC-Robot', ROBOT_ADDR, -55, [cm.NUS_SERVICE]), ('', 'AA:BB:CC:DD:EE:03', -40, [])]
    bleak.BleakScanner.ERROR = scan_error
    bleak.BleakClient.ROBOTS = {ROBOT_ADDR: robot}
    bleak.BleakClient.FAIL = fail
    bleak.BleakClient.LOG = []

def run_scan(sc, limit=5):
    sc.start()
    t = time.monotonic()
    while sc.running and time.monotonic() - t < limit: time.sleep(0.01)
    assert not sc.running

@test
def ble_scan_puts_the_robot_first_and_drops_noise():
    ble_world()
    sc = cm.BleScanner(0.2); run_scan(sc)
    assert [d.name for d in sc.devices] == ['REC-Robot', 'Mi Band 6'], 'nameless beacons are dropped'
    assert sc.robot().address == ROBOT_ADDR and sc.robot().nus
    ble_world(adv=[('Mi Band 6', 'AA:BB:CC:DD:EE:02', -70, [])])
    sc = cm.BleScanner(0.2); run_scan(sc)
    assert sc.robot() is None and len(sc.devices) == 1

@test
def ble_scan_errors_are_plain_sentences():
    ble_world(scan_error='Bluetooth radio is turned off')
    sc = cm.BleScanner(0.2); run_scan(sc)
    assert 'Bluetooth is off' in sc.error, sc.error
    with module_missing('bleak'):
        sc = cm.BleScanner(0.2); run_scan(sc)
    assert 'bleak is not installed' in sc.error and '-m pip install bleak' in sc.error

@test
def ble_link_full_session_with_a_pretend_robot():
    ble_world()
    l = cm.Link(time.monotonic)
    l.connect(lambda: cm.BleTransport(ROBOT_ADDR, 'REC-Robot'), 'rec1')
    assert l.state == cm.Link.CONNECTING and 'BLE REC-Robot' in l.transport.name
    assert wait_for(lambda: l.state == cm.Link.CONNECTED and l.stats['tel'] > 10, l, 8), (l.state, list(l.log)[-4:])
    assert l.hello.get('fw') == 'fake-esp32' and l.stats['bad'] == 0 and l.transport.chunk == 180
    l.set_command(cm.Command(60, 60, 0, 0, True))
    for _ in range(60): l.update(); time.sleep(0.01)
    assert l.telemetry['vl'] == 0, 'disarmed: zeros'
    l.armed = True
    assert wait_for(lambda: l.telemetry['vl'] > 40, l, 3), l.telemetry
    assert all(resp is False and len(d) <= 180 for _, d, resp in bleak.BleakClient.LOG), 'write-without-response, one piece each'
    # the robot walks away: the link fails, disarms, and says why
    l.transport._client.drop()
    assert wait_for(lambda: l.state == cm.Link.ERROR, l, 3) and not l.armed
    assert 'lost' in l.last_error, l.last_error
    l.disconnect()

@test
def ble_problems_are_named():
    ble_world(robot=None)
    l = cm.Link(time.monotonic); l.connect(lambda: cm.BleTransport(ROBOT_ADDR), 'rec1')
    assert wait_for(lambda: l.state == cm.Link.ERROR, l, 6) and 'no REC serial service' in l.last_error and 'BT_BLE' in l.last_error, l.last_error
    ble_world(fail='Device with address AA:BB:CC:DD:EE:01 was not found')
    l = cm.Link(time.monotonic); l.connect(lambda: cm.BleTransport(ROBOT_ADDR), 'rec1')
    assert wait_for(lambda: l.state == cm.Link.ERROR, l, 8) and 'not found' in l.last_error and 'Refresh' in l.last_error, l.last_error
    with module_missing('bleak'):
        l = cm.Link(time.monotonic); l.connect(lambda: cm.BleTransport(ROBOT_ADDR), 'rec1')
        assert wait_for(lambda: l.state == cm.Link.ERROR, l, 4) and '-m pip install bleak' in l.last_error, l.last_error

@test
def ble_radio_backlog_drops_commands_instead_of_adding_delay():
    ble_world()
    t = cm.BleTransport(ROBOT_ADDR); t.open()
    t._pending = 4
    t.write(b'x'); t.write(b'y')
    assert t.dropped == 2 and bleak.BleakClient.LOG == []
    t._pending = 0
    t.close()


# ============================================================================================== V_5_0: Bluetooth Classic by address (no COM port)
class FakeSock:
    """One end of a socketpair that behaves like an RFCOMM socket: connect((mac, channel)) is recorded, optionally refused."""
    def __init__(self, end, log, refuse=()):
        self.end, self.log, self.refuse, self.ok = end, log, refuse, False
    def settimeout(self, t): self.end.settimeout(t)
    def connect(self, addr):
        self.log.append(addr)
        if addr[1] in self.refuse:
            raise OSError('[WinError 10061] No connection could be made because the target machine actively refused it')
        self.ok = True
    def send(self, d): return self.end.send(d)
    def recv(self, n): return self.end.recv(n)
    def close(self):
        if self.ok: self.end.close()                           # a refused connect has nothing to close in this stand-in

def bridge(peer, stop):
    """Plays the robot on the far end of the socketpair."""
    vt = cm.VirtualRobotTransport('simple', time.monotonic, latency_ms=5, jitter_ms=1); vt.open()
    peer.settimeout(0.005)
    while not stop.is_set():
        try:
            d = peer.recv(4096)
            if d == b'': break
            vt.write(d)
        except (_socket.timeout, BlockingIOError):
            pass
        except OSError:
            break
        out = vt.read()
        if out:
            try: peer.sendall(out)
            except OSError: break

@test
def rfcomm_link_without_a_com_port():
    a, b = _socket.socketpair()
    stop, log = threading.Event(), []
    threading.Thread(target=bridge, args=(b, stop), daemon=True).start()
    mk = lambda: FakeSock(a, log, refuse=(1,))                  # channel 1 refuses, channel 2 works: the transport tries the next one
    l = cm.Link(time.monotonic)
    l.connect(lambda: cm.RfcommTransport('A0:B1:C2:D3:E4:F5', 'REC-Robot', sock_factory=mk), 'rec1')
    assert wait_for(lambda: l.state == cm.Link.CONNECTED and l.stats['tel'] > 10, l, 8), (l.state, list(l.log)[-4:])
    assert [c for _, c in log] == [1, 2] and log[0][0] == 'A0:B1:C2:D3:E4:F5'
    l.armed = True; l.set_command(cm.Command(50, 50, 0, 0, True))
    assert wait_for(lambda: l.telemetry['vl'] > 30, l, 3)
    b.close(); stop.set()                                       # the robot closes the connection
    assert wait_for(lambda: l.state == cm.Link.ERROR, l, 3) and not l.armed and ('closed' in l.last_error or 'reset' in l.last_error or 'failed' in l.last_error), l.last_error
    l.disconnect()

@test
def rfcomm_errors_are_plain_sentences():
    def refuse_all():
        class S:
            def settimeout(s, t): pass
            def connect(s, addr): raise OSError('[WinError 10060] A connection attempt failed because the connected party did not respond')
            def close(s): pass
        return S()
    t = cm.RfcommTransport('A0:B1:C2:D3:E4:F5', sock_factory=refuse_all); t.start_open()
    for _ in range(300):
        if not t.opening: break
        time.sleep(0.01)
    assert not t.is_open and 'no answer from the robot' in t.last_error and 'pair' in t.last_error, t.last_error
    assert 'denied' in cm.rfcomm_friendly(OSError('[WinError 10013] An attempt was made to access a socket in a way forbidden'))
    if not hasattr(_socket, 'AF_BLUETOOTH'):
        t = cm.RfcommTransport('A0:B1:C2:D3:E4:F5'); t.start_open()
        for _ in range(100):
            if not t.opening: break
            time.sleep(0.01)
        assert 'no Bluetooth socket support' in t.last_error


# ============================================================================================== V_5_0: firmware, Bluetooth LE build and USB-only build
@test
@rig_mode(2)
def ble_firmware_handshake_telemetry_and_chunked_frames(rig):
    l = rig.link()
    assert wait_for(lambda: l.state == cm.Link.CONNECTED and l.stats['tel'] > 12, l, 8), (l.state, list(l.log)[-4:])
    assert l.hello == dict(proto='1', fw='rec-esp32', robot='simple', version='V_5_0'), l.hello
    for _ in range(150): l.update(); time.sleep(0.01)
    assert 7 < l.stats['tel_rate'] < 13, 'BLE telemetry is 10 Hz: %.1f' % l.stats['tel_rate']
    assert l.stats['bad'] == 0 and l.loss_percent() == 0, 'frames longer than 20 bytes are split and put back together'
    assert 7800 < l.telemetry['vbat_mv'] < 8200
    assert any('laptop connected' in t for t in rig.text) and any('Bluetooth LE' in t for t in rig.text), rig.text[:4]
    l.disconnect()

@test
@rig_mode(2)
def ble_firmware_disarms_when_the_wireless_link_drops(rig):
    l = rig.link()
    l.set_command(cm.Command(60, 60, 0, 0, True))
    assert wait_for(lambda: l.state == cm.Link.CONNECTED, l, 8)
    l.armed = True
    assert wait_for(lambda: rig.pins.get('ena', 0) > 0, l, 3)
    rig.send('bledrop')                                         # the BLE connection drops (laptop away, Bluetooth off)
    t0 = time.monotonic()
    while rig.pins['ena'] != 0 and time.monotonic() - t0 < 2: l.update(); time.sleep(0.005)
    assert rig.pins['ena'] == 0 and time.monotonic() - t0 < 0.5, 'stopped at once, not after the 400 ms failsafe'
    assert any('laptop disconnected' in t for t in rig.text)
    l.disconnect()
    # a new connection comes back disarmed
    rig.send('bleup')
    l2 = rig.link()
    assert wait_for(lambda: l2.state == cm.Link.CONNECTED and l2.stats['tel'] > 5, l2, 8)
    for _ in range(30): l2.update(); time.sleep(0.01)
    assert not l2.armed and rig.pins['ena'] == 0 and not (l2.telemetry['flags'] & cm.F_ARMED)
    l2.disconnect()

@test
@rig_mode(0)
def usb_only_build_runs_and_says_so(rig):
    time.sleep(0.8)
    assert any('USB only' in t for t in rig.text), rig.text
    assert any('[status]' in t and 'client n/a' in t and 'A0:B1:C2:D3:E4:F5' in t for t in rig.text), rig.text
    assert not FW.get('warn0') and not FW.get('warn1') and not FW.get('warn2'), 'the sketch compiles without warnings: %s' % [FW.get(k) for k in ('warn0', 'warn1', 'warn2')]


# ============================================================================================== V_5_0: the dashboard (needs pygame; skipped without it)
def pygame_ok():
    os.environ.setdefault('SDL_VIDEODRIVER', 'dummy')
    try:
        import pygame
        return True
    except ImportError:
        return False

def needs_pygame(f):
    def wrapper():
        if not pygame_ok():
            print('  (skipped: pygame not installed)'); return
        f()
    wrapper.__name__ = f.__name__
    return wrapper

class Clk:
    def __init__(self): self.t = 0.0
    def __call__(self): return self.t

def make_app(robot='simple', settings=None, clock=None):
    import tempfile
    import controller_dashboard_V_5_0 as d5
    d = tempfile.mkdtemp()
    app = d5.App50(robot, 'blue', clock=clock or time.monotonic, settings_path=settings or os.path.join(d, 'settings.json'))
    app.report_dir = d
    S = d5.build_ui(app, dict(demo=lambda: 0, learn=lambda: 0, save=lambda: 0, default=lambda: 0, fullscreen=lambda: 0))
    return d5, app, S

def settle(app, cond, limit=8.0):
    end = time.monotonic() + limit
    while time.monotonic() < end:
        app.tick_link()
        if cond(): return True
        time.sleep(0.01)
    return False

@test
@needs_pygame
def dashboard_remembers_the_new_settings():
    d5, app, S = make_app()
    path = app.settings_path
    app.use_manual_port('COM7')
    app.transport_kind, app.ble_addr, app.ble_name, app.mac, app.mac_name = 'ble', ROBOT_ADDR, 'REC-Robot', 'A0:B1:C2:D3:E4:F5', 'REC-Robot'
    app.save_settings()
    d5b, app2, _ = make_app(settings=path)
    assert (app2.transport_kind, app2.ble_addr, app2.ble_name, app2.mac, app2.port) == ('ble', ROBOT_ADDR, 'REC-Robot', 'A0:B1:C2:D3:E4:F5', 'COM7')
    open(path, 'w').write('{broken')
    d5c, app3, _ = make_app(settings=path)                    # a damaged settings file must not stop the program
    assert app3.transport_kind == 'virtual'

@test
@needs_pygame
def dashboard_manual_port_and_address():
    d5, app, S = make_app()
    app.use_manual_port('7')
    assert app.port == 'COM7' and app.transport_kind == 'serial' and any(p.device == 'COM7' for p in app.ports)
    app.refresh_ports()
    assert app.port == 'COM7', 'a port typed by hand survives Refresh'
    app.use_manual_mac('COM7'); assert 'not a Bluetooth address' in app.doctor_note and app.transport_kind == 'serial'
    app.use_manual_mac('a0-b1-c2-d3-e4-f5'); assert app.mac == 'A0:B1:C2:D3:E4:F5' and app.transport_kind == 'rfcomm' and app.target_ready()

@test
@needs_pygame
def dashboard_explains_an_empty_port_list_and_refuses_the_incoming_port():
    d5, app, S = make_app()
    app.transport_kind = 'serial'
    with module_missing('serial'):
        app.refresh_ports()
    assert app.ports == [] and 'pyserial is missing' in app.hint() and 'DOCTOR' in app.hint()
    app.toggle_connection()
    assert app.link.state == cm.Link.DISCONNECTED and 'DOCTOR' in app.link.last_error
    app.ports = [cm.PortInfo('COM6', 'Standard Serial over Bluetooth link', BT_IN, True, 'in', ''), cm.PortInfo('COM5', 'Standard Serial over Bluetooth link', BT_OUT, True, 'out', 'A0:B1:C2:D3:E4:F5')]
    app.port = 'COM6'
    assert 'Incoming' in app.hint() and 'Outgoing' in app.hint()
    app.paired = [cm.PairedDevice('REC-Robot', 'A0:B1:C2:D3:E4:F5', True, 'classic')]
    assert 'REC-Robot' in app.port_label(app.ports[1]) and 'skip' in app.port_label(app.ports[0])

@test
@needs_pygame
def dashboard_ble_scan_selects_the_robot_and_connects():
    ble_world()
    d5, app, S = make_app()
    app.transport_kind = 'ble'
    assert 'press Refresh' in app.hint() or 'Scan' in app.hint() or 'scan' in app.hint()
    app.refresh()
    assert app.scanning_ble() and 'scanning' in app.hint()
    assert settle(app, lambda: not app.scanning_ble())
    assert app.ble_addr == ROBOT_ADDR and app.ble_name == 'REC-Robot' and 'found REC-Robot' in app.hint()
    d5.refresh_dynamic(app)
    assert app.dd_port.title == 'BLE DEVICE' and app.dd_port.options[0][0] == ROBOT_ADDR and app.dd_port.get() == ROBOT_ADDR
    app.toggle_connection()
    assert settle(app, lambda: app.link.state == cm.Link.CONNECTED and app.link.stats['tel'] > 5), (app.link.state, list(app.link.log)[-3:])
    assert isinstance(app.link.transport, cm.BleTransport) and not app.link.armed
    app.toggle_connection(); assert app.link.state == cm.Link.DISCONNECTED

@test
@needs_pygame
def dashboard_auto_find_over_ble_connects_by_itself():
    ble_world()
    d5, app, S = make_app()
    app.transport_kind = 'ble'
    app.toggle_scan()
    assert settle(app, lambda: app.link.state == cm.Link.CONNECTED and app.link.stats['tel'] > 5, 12), (app.link.state, app.ble_msg)
    app.link.disconnect()
    ble_world(adv=[('Mi Band 6', 'AA:BB:CC:DD:EE:02', -70, [])])
    d5, app, S = make_app(); app.transport_kind = 'ble'
    app.toggle_scan()
    assert settle(app, lambda: not app.scanning_ble()) and app.link.state == cm.Link.DISCONNECTED and 'none named REC' in app.hint() and not app.ble_addr
    app.toggle_connection(); assert 'press Refresh' in app.link.last_error, app.link.last_error
    ble_world(scan_error='Bluetooth radio is turned off')
    d5, app, S = make_app(); app.transport_kind = 'ble'; app.refresh()
    assert settle(app, lambda: not app.scanning_ble()) and 'Bluetooth is off' in app.hint()

@test
@needs_pygame
def dashboard_doctor_runs_in_the_background_and_saves_a_report():
    d5, app, S = make_app()
    assert settle(app, lambda: app.report is not None and not app.doctor_running), 'the check runs at start-up'
    canned = cm.diagnose(scan=cm.scan_ports(comports=[], registry={}), paired=[cm.PairedDevice('REC-Robot', 'A0:B1:C2:D3:E4:F5', True, 'classic'), cm.PairedDevice('Buds', '11:22:33:44:55:66', False, 'le')])
    real = cm.diagnose
    cm.diagnose = lambda *a, **k: canned
    try:
        app.mac = app.mac_name = ''
        app.start_doctor()
        assert app.doctor_running or app.report is canned
        assert settle(app, lambda: app.report is canned and not app.doctor_running)
    finally:
        cm.diagnose = real
    assert [d.mac for d in app.paired] == ['A0:B1:C2:D3:E4:F5'] and app.mac == 'A0:B1:C2:D3:E4:F5' and app.mac_name == 'REC-Robot', 'the paired robot is picked for Classic (MAC) mode'
    app.save_report()
    f = os.path.join(app.report_dir, 'port_report_V_5_0.txt')
    assert os.path.exists(f) and 'NEXT STEPS' in open(f, encoding='utf-8').read() and 'saved' in app.doctor_note
    def boom(*a, **k): raise RuntimeError('powershell exploded')
    cm.diagnose = boom
    try:
        app.start_doctor(); assert settle(app, lambda: not app.doctor_running)
    finally:
        cm.diagnose = real
    assert any('the check itself failed' in t for lv, t in app.report.lines)

@test
@needs_pygame
def dashboard_widgets_follow_the_transport():
    d5, app, S = make_app()
    for kind, title in (('serial', 'PORT'), ('ble', 'BLE DEVICE'), ('rfcomm', 'PAIRED ROBOT')):
        app.transport_kind = kind; d5.refresh_dynamic(app)
        assert app.dd_port.title == title, (kind, app.dd_port.title)
    app.transport_kind = 'virtual'
    cut = [b for b in S.link.buttons if getattr(b, 'show', None)]
    assert len(cut) == 2 and all(b.show() for b in cut)
    app.transport_kind = 'ble'
    assert not any(b.show() for b in cut) and not cut[0].click(cut[0].rect.center), 'Cut cable only exists for the virtual robot'
    # tabs: DOCTOR shows its own widgets, text boxes only take the keyboard while their tab is showing
    app.tab = 'doctor'
    assert S.doctor in S.active(app) and S.link not in S.active(app)
    box = S.doctor.inputs[0]; box.focus = True
    assert S.typing(app)
    app.tab = 'link'; S.drop_focus(app)
    assert not box.focus and not S.typing(app)
    app.tab = 'doctor'
    b = [x for x in S.doctor.buttons if x.label == 'Save report'][0]
    assert not b.is_enabled() or app.report is not None

@test
@needs_pygame
def dashboard_virtual_robot_still_works_and_stays_disarmed():
    d5, app, S = make_app('tower', clock=Clk())
    app.transport_kind = 'virtual'
    app.toggle_connection()
    for _ in range(240):
        app.clock.t += 1 / 60; app.tick_link()
    assert app.link.state == cm.Link.CONNECTED and not app.link.armed
    app.toggle_arm(); assert app.link.armed
    app.estop(); assert not app.link.armed
    app.toggle_connection()

@test
@needs_pygame
def dashboard_renders_every_tab_transport_and_state():
    import pygame, controller_viewer as cv, controller_dashboard as cd
    from robot_sim import Camera
    ble_world()
    d5, app, S = make_app('tower', clock=Clk())
    fonts, big = cd.make_fonts()
    canvas, scene, cam = pygame.Surface((d5.CW, d5.CH)), pygame.Surface((d5.VP[2], d5.VP[3])), Camera(d5.VP[2], d5.VP[3], 400)
    m = xi.default_mapping() if 'xi' in globals() else None
    import xbox_input as xi2
    m = xi2.default_mapping(); a, b, h = cv.demo_raw(1.0); st = xi2.apply_mapping(m, a, b, h, 'DEMO')
    app.paired = [cm.PairedDevice('REC-Robot', 'A0:B1:C2:D3:E4:F5', True, 'classic')]
    app.ble_devices = [cm.BleDevice('REC-Robot', ROBOT_ADDR, -50, True), cm.BleDevice('Mi Band 6', 'AA:BB:CC:DD:EE:02', -70, False)]
    app.ble_addr, app.ble_name, app.mac = ROBOT_ADDR, 'REC-Robot', 'A0:B1:C2:D3:E4:F5'
    settle(app, lambda: app.report is not None and not app.doctor_running)
    for kind in ('virtual', 'serial', 'ble', 'rfcomm'):
        app.transport_kind = kind
        for tab in ('link', 'doctor', 'settings', 'help'):
            app.tab = tab
            for mode in ('game', 'monitor'):
                app.set_mode(mode)
                d5.render(canvas, fonts, big, cam, scene, app, S, st, m, ['x'])
    # a very long report and a note must not crash the drawing
    app.tab = 'doctor'
    for _ in range(60): app.report.add('warn', 'a very long line ' * 8)
    app.report.steps[:] = ['step number %d ' % i + 'long text ' * 30 for i in range(9)]
    app.doctor_note = 'note ' * 40
    d5.render(canvas, fonts, big, cam, scene, app, S, st, m, ['x'])
    app.doctor_running, app.report = True, None
    d5.render(canvas, fonts, big, cam, scene, app, S, st, m, ['x'])

if __name__ == '__main__':
    print('pyserial:', 'REAL' if REAL_PYSERIAL else 'test stand-in (testshim/serial)')
    bad = 0
    for t in TESTS:
        t0 = time.monotonic()
        try:
            t(); print('PASS %-62s %.1fs' % (t.__name__, time.monotonic() - t0))
        except Exception:
            bad += 1; import traceback; print('FAIL', t.__name__); traceback.print_exc()
    print('%d of %d passed' % (len(TESTS) - bad, len(TESTS)))
    sys.exit(1 if bad else 0)
