"""test_V_4_1.py - checks for the V 4.1 link layer, the Bluetooth handling and the ESP32 firmware. No window, no hardware.
Run:  python test_V_4_1.py        (needs g++ for the firmware-in-the-loop tests; they are skipped without it)

Firmware-in-the-loop: esp32_firmware_V_4_1.ino is compiled on the PC against fw_host/Arduino.h (a stand-in for the Arduino core). Its Bluetooth stream is
one end of a pseudo-terminal; comms_V_4_1.SerialTransport opens the other end like a COM port. So the real Link, the real parser and the real sketch logic talk.
If the real pyserial is not installed, testshim/serial is used (a tiny stand-in on the same pty). It is only a stand-in.
"""
import os, subprocess, sys, tempfile, threading, time, pty, tty
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
try:
    import serial                                    # real pyserial if present
    REAL_PYSERIAL = True
except ImportError:
    sys.path.insert(0, os.path.join(HERE, 'testshim'))
    import serial
    REAL_PYSERIAL = False
import comms_V_4_1 as cm

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
    from serial.tools import list_ports
    class P:                                              # what pyserial's comports() items look like
        def __init__(s, d, desc, hwid): s.device, s.description, s.hwid = d, desc, hwid
    real = list_ports.comports
    list_ports.comports = lambda: [P('COM3', 'USB-SERIAL CH340', 'USB VID:PID=1A86:7523'), P('COM1', 'Communications Port', 'ACPI\\PNP0501'),
                                   P('COM5', 'Standard Serial over Bluetooth link', 'BTHENUM\\x')]
    try:
        ports = cm.list_serial_ports()
    finally:
        list_ports.comports = real
    assert [p.device for p in ports] == ['COM5', 'COM1', 'COM3'] and ports[0].bluetooth
    assert [p.device for p in ports if cm.probeable(p)] == ['COM5', 'COM3'], 'motherboard COM1 is not probed'


# ------------------------------------------------------------------------------------------ link behaviour (virtual robot)
@test
def handshake_telemetry_latency():
    c, l = connected(latency_ms=40, jitter_ms=0)
    assert l.state == cm.Link.HANDSHAKE
    c.run(l, 3.0)
    assert l.state == cm.Link.CONNECTED and l.hello['robot'] == 'simple' and l.hello['version'] == 'V_4_1'
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
def build_firmware():
    if 'exe' in FW: return FW['exe']
    exe = os.path.join(HERE, 'fw_host', 'fw_host')
    src = os.path.join(HERE, 'esp32_firmware_V_4_1.ino')
    r = subprocess.run(['g++', '-std=gnu++17', '-O1', '-I' + os.path.join(HERE, 'fw_host'), '-include', 'Arduino.h', '-x', 'c++', src,
                        os.path.join(HERE, 'fw_host', 'harness.cpp'), '-o', exe], capture_output=True, text=True)
    FW['exe'] = exe if r.returncode == 0 else None
    FW['err'] = r.stderr
    return FW['exe']


class FirmwareRig:
    """The real sketch, its Bluetooth stream on a pty, plus a reader thread for the PINS lines it prints."""
    def __init__(self):
        exe = build_firmware()
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


def with_rig(f):
    def wrapper():
        if not build_firmware():
            if 'g++' in FW.get('err', '') or not FW.get('err'):
                print('  (skipped: g++ not available)'); return
            raise AssertionError('firmware did not compile:\n' + FW['err'])
        rig = FirmwareRig()
        try:
            time.sleep(0.2)
            f(rig)
        finally:
            rig.close()
    wrapper.__name__ = f.__name__
    return wrapper


@test
@with_rig
def firmware_handshake_and_telemetry_over_the_port(rig):
    l = rig.link()
    assert l.state == cm.Link.CONNECTING
    assert wait_for(lambda: l.state == cm.Link.CONNECTED and l.stats['tel'] > 20, l, 6), (l.state, list(l.log)[-4:])
    assert l.hello == dict(proto='1', fw='rec-esp32', robot='simple', version='V_4_1'), l.hello
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
