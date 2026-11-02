"""
comms_V_4_0.py - laptop <-> robot communication for the dashboard (no pygame, testable anywhere). See COMMS_ARCHITECTURE_V_4_0.md.

Layers (top to bottom)
    Link            connection state machine, handshake, heartbeat, ping, failsafe-friendly TX schedule, statistics, terminal log
    Profile         how a Command becomes bytes:  'rec1' (framed lines, with telemetry back)   or   'legacy' (the one-letter F B L R S C O protocol of esp32_remote_test.ino)
    FrameParser     bytes -> frames / plain text lines / bad frames
    Transport       something that moves bytes:  SerialTransport (USB, Bluetooth COM port, rfcomm)  |  VirtualRobotTransport (a fake ESP32 for testing the whole stack)

Frame format (rec1), one line, ASCII so it can be read in any serial monitor:
    $TYPE,field,field,...*HH\\n        HH = two hex digits, XOR of every character between '$' and '*'
Laptop -> robot:  CMD (every 50 ms, doubles as the heartbeat), PING, HELLO?, CFG, ZERO, STOP
Robot  -> laptop: HELLO, TEL (20 Hz), PONG, ACK, LOG;   any line that does not start with '$' is shown in the terminal as plain text.
"""
import math, random, time
from collections import deque

PROTO = 1
TX_HZ = 20.0                 # CMD rate (also the heartbeat)
PING_EVERY = 1.0
STALE_AFTER = 0.6            # connected but nothing heard for this long -> STALE
HELLO_TIMEOUT = 2.0
FAILSAFE_MS = 300            # the robot stops when it has heard no valid CMD for this long
BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 460800]
PROFILES = {'rec1': 'REC-link v1 (framed, telemetry)', 'legacy': 'Legacy letters (F B L R S C O)'}

# TEL flags
F_ARMED, F_FAILSAFE, F_GYRO_OK, F_GYRO_CAL, F_LOWBAT = 1, 2, 4, 8, 16
LOW_BATTERY_MV = 6600        # 2S Li-ion: about 3.3 V per cell


# ----------------------------------------------------------------------------------------------- framing
def checksum(body):
    c = 0
    for ch in body.encode('ascii', 'replace'):
        c ^= ch
    return c


def encode(ftype, *fields):
    body = ','.join([ftype] + [_fmt(f) for f in fields])
    return ('$%s*%02X\n' % (body, checksum(body))).encode('ascii')


def _fmt(v):
    if isinstance(v, float):
        return ('%.2f' % v).rstrip('0').rstrip('.') if v != int(v) else str(int(v))
    return str(v)


class FrameParser:
    """Feed it bytes as they arrive. Returns events: ('frame', type, [fields]) | ('bad', text) | ('text', line)."""

    def __init__(self, maxline=240):
        self.buf = bytearray()
        self.maxline = maxline
        self.bad = 0

    def feed(self, data):
        out = []
        for b in data:
            if b == 10:                                   # \n ends a line
                out += self._line(bytes(self.buf).decode('ascii', 'replace').rstrip('\r'))
                self.buf.clear()
            elif len(self.buf) < self.maxline:
                self.buf.append(b)
            else:                                         # runaway line: drop it
                self.buf.clear(); self.bad += 1
                out.append(('bad', 'line too long'))
        return out

    def _line(self, s):
        if not s:
            return []
        if s[0] != '$':
            return [('text', s)]
        if '*' not in s:
            self.bad += 1
            return [('bad', s)]
        body, _, hh = s[1:].rpartition('*')
        try:
            ok = int(hh[:2], 16) == checksum(body)
        except ValueError:
            ok = False
        if not ok:
            self.bad += 1
            return [('bad', s)]
        parts = body.split(',')
        return [('frame', parts[0], parts[1:])]


# ----------------------------------------------------------------------------------------------- command
class Command:
    """What the laptop wants the robot to do. Wheels in percent of full speed (-100..100), grip 0..100 (0 open), lift -100..100 (tower box)."""

    def __init__(self, vl=0, vr=0, grip=0, lift=0, enable=False):
        self.vl, self.vr, self.grip, self.lift, self.enable = vl, vr, grip, lift, enable

    def clamped(self):
        c = lambda v, lo, hi: int(max(lo, min(hi, round(v))))
        return Command(c(self.vl, -100, 100), c(self.vr, -100, 100), c(self.grip, 0, 100), c(self.lift, -100, 100), bool(self.enable))

    def zero(self): return Command(0, 0, 0, 0, False)


def encode_command(profile, seq, cmd):
    c = cmd.clamped()
    if profile == 'rec1':
        return encode('CMD', seq, c.vl, c.vr, c.grip, c.lift, 1 if c.enable else 0)
    # legacy: one letter per update; the sketch has no speed or lift, only direction and open / close
    if not c.enable:
        return b'S'
    if abs(c.vl) < 15 and abs(c.vr) < 15:
        letter = 'S'
    elif c.vl > 0 and c.vr > 0 and abs(c.vl - c.vr) < 40:
        letter = 'F'
    elif c.vl < 0 and c.vr < 0 and abs(c.vl - c.vr) < 40:
        letter = 'B'
    else:
        letter = 'L' if c.vl < c.vr else 'R'
    return letter.encode()


def legacy_grip_letter(grip, last):
    """Legacy robots take C / O only when the gripper changes: close above 50 %, open below."""
    want = 'C' if grip > 50 else 'O'
    return (want.encode(), want) if want != last else (b'', last)


# ----------------------------------------------------------------------------------------------- transports
class Transport:
    name = 'transport'

    def __init__(self):
        self.is_open, self.last_error = False, ''

    def open(self): raise NotImplementedError
    def close(self): self.is_open = False
    def write(self, data): raise NotImplementedError
    def read(self): return b''


def list_serial_ports():
    """[(device, description)] of the real serial / Bluetooth COM ports. Empty when pyserial is not installed."""
    try:
        from serial.tools import list_ports
        return [(p.device, p.description) for p in list_ports.comports()]
    except Exception:
        return []


class SerialTransport(Transport):
    def __init__(self, port, baud=115200):
        super().__init__()
        self.port, self.baud, self.ser = port, baud, None
        self.name = '%s @ %d' % (port, baud)

    def open(self):
        try:
            import serial
            self.ser = serial.Serial(self.port, self.baud, timeout=0, write_timeout=0.2)
            self.is_open, self.last_error = True, ''
        except Exception as e:
            self.is_open, self.last_error = False, str(e)
            raise IOError(str(e))

    def close(self):
        try:
            if self.ser: self.ser.close()
        except Exception:
            pass
        self.ser, self.is_open = None, False

    def write(self, data):
        try:
            self.ser.write(data)
        except Exception as e:
            self.last_error = str(e)
            raise IOError(str(e))

    def read(self):
        try:
            n = self.ser.in_waiting
            return self.ser.read(n) if n else b''
        except Exception as e:
            self.last_error = str(e)
            raise IOError(str(e))


class FakeRobot:
    """A stand-in ESP32 that speaks rec1: parses frames, obeys the failsafe, simulates wheels / gyro / battery and answers with TEL.
    Time is passed in (seconds), so tests can run it without sleeping."""

    def __init__(self, robot_type='simple', seed=1):
        self.type = robot_type
        self.rnd = random.Random(seed)
        self.parser = FrameParser()
        self.reset(0.0)

    def reset(self, now):
        self.t0 = now
        self.last = now
        self.last_cmd = None                  # time of the last valid CMD
        self.armed = False
        self.vl = self.vr = 0.0               # applied wheel percent
        self.tv = [0.0, 0.0]
        self.grip = self.lift = 0.0
        self.tgrip = self.tlift = 0.0
        self.yaw = 0.0                        # degrees, counter-clockwise positive (like the MPU6050 notes)
        self.gz = 0.0
        self.bias = 0.4                       # deg/s, zeroed by the start-up calibration
        self.cal_until = now + 1.0
        self.ack = 0
        self.tseq = 0
        self.next_tel = now
        self.vbat = 8300.0
        self.failsafe = True
        self.out = []                         # bytes to send back
        self.cfg = {}

    def boot_banner(self):
        return [encode('HELLO', PROTO, 'fake-esp32', self.type, 'V_4_0')]

    def rx(self, data, now):
        replies = []
        for ev in self.parser.feed(data):
            if ev[0] != 'frame':
                continue
            _, t, f = ev
            try:
                if t == 'CMD':
                    self.ack = int(f[0])
                    self.tv = [int(f[1]), int(f[2])]
                    self.tgrip, self.tlift = int(f[3]), int(f[4])
                    self.armed = f[5] == '1'
                    self.last_cmd = now
                elif t == 'PING':
                    replies.append(encode('PONG', f[0], int((now - self.t0) * 1000)))
                elif t == 'HELLO?':
                    replies += self.boot_banner()
                elif t == 'CFG':
                    self.cfg[f[0]] = f[1]
                    replies.append(encode('ACK', 'CFG', f[0]))
                elif t == 'ZERO':
                    self.yaw = 0.0
                    replies.append(encode('ACK', 'ZERO'))
                elif t == 'STOP':
                    self.tv, self.armed = [0, 0], False
                    replies.append(encode('ACK', 'STOP'))
            except (IndexError, ValueError):
                replies.append(encode('LOG', 'bad %s frame' % t))
        return replies

    def step(self, now):
        dt = max(0.0, now - self.last)
        self.last = now
        self.failsafe = self.last_cmd is None or (now - self.last_cmd) * 1000 > FAILSAFE_MS
        tv = [0.0, 0.0] if (self.failsafe or not self.armed) else self.tv
        k = min(1.0, dt * 8.0)                                            # motors follow with a lag
        self.vl += (tv[0] - self.vl) * k
        self.vr += (tv[1] - self.vr) * k
        tg, tl = (0.0, 0.0) if (self.failsafe or not self.armed) else (self.tgrip, self.tlift)
        self.grip += max(-200 * dt, min(200 * dt, tg - self.grip))
        self.lift = max(0.0, min(100.0, self.lift + (tl if not self.failsafe else 0.0) * 0.4 * dt))
        w = (self.vr - self.vl) / 100.0 * (30.0 / 15.0)                    # rad/s from the wheel difference: full speed 30 cm/s on a 15 cm track
        self.gz = math.degrees(w) + (self.bias if now < self.cal_until else self.rnd.gauss(0, 0.15))
        self.yaw += math.degrees(w) * dt                                   # CCW positive
        self.vbat = 8300.0 - 8.0 * (now - self.t0) - 6.0 * (abs(self.vl) + abs(self.vr))
        out = []
        if now >= self.next_tel:
            self.next_tel += 1.0 / TX_HZ
            if self.next_tel < now: self.next_tel = now + 1.0 / TX_HZ
            fl = (F_ARMED if self.armed else 0) | (F_FAILSAFE if self.failsafe else 0) | (F_GYRO_CAL if now < self.cal_until else F_GYRO_OK) \
                | (F_LOWBAT if self.vbat < LOW_BATTERY_MV else 0)
            self.tseq = (self.tseq + 1) % 65536
            out.append(encode('TEL', self.tseq, self.ack, int((now - self.t0) * 1000), round(self.yaw, 1), round(self.gz, 1),
                              int(round(self.vl)), int(round(self.vr)), int(round(self.grip)), int(round(self.lift)), int(self.vbat), fl))
        return out


class VirtualRobotTransport(Transport):
    """Pretends to be a serial port with a FakeRobot on the other end. latency_ms / loss / jitter_ms make the link behave badly on purpose."""

    def __init__(self, robot_type='simple', clock=time.monotonic, latency_ms=25, jitter_ms=8, loss=0.0, seed=3):
        super().__init__()
        self.name = 'Virtual robot (test)'
        self.clock = clock
        self.robot = FakeRobot(robot_type, seed)
        self.latency, self.jitter, self.loss = latency_ms / 1000.0, jitter_ms / 1000.0, loss
        self.rnd = random.Random(seed)
        self.to_robot, self.to_laptop = deque(), deque()       # (deliver_time, bytes)
        self.cut = False                                       # simulate pulling the cable

    def open(self):
        self.is_open, self.last_error = True, ''
        self.robot.reset(self.clock())
        for b in self.robot.boot_banner():
            self._send_back(b)

    def close(self):
        self.is_open = False
        self.to_robot.clear(); self.to_laptop.clear()

    def _delay(self):
        return max(0.0, self.latency + self.rnd.uniform(-self.jitter, self.jitter)) / 2.0

    def _send_back(self, data):
        if self.loss and self.rnd.random() < self.loss:
            return
        self.to_laptop.append((self.clock() + self._delay(), data))

    def write(self, data):
        if not self.is_open or self.cut:
            if self.cut: raise IOError('cable unplugged')
            raise IOError('port closed')
        if self.loss and self.rnd.random() < self.loss:
            return
        self.to_robot.append((self.clock() + self._delay(), data))

    def read(self):
        if self.cut:
            raise IOError('cable unplugged')
        now = self.clock()
        while self.to_robot and self.to_robot[0][0] <= now:
            _, d = self.to_robot.popleft()
            for r in self.robot.rx(d, now):
                self._send_back(r)
        for t in self.robot.step(now):
            self._send_back(t)
        out = b''
        while self.to_laptop and self.to_laptop[0][0] <= now:
            out += self.to_laptop.popleft()[1]
        return out


# ----------------------------------------------------------------------------------------------- link
class Link:
    """The connection. Call update() every frame (about 60 Hz). Everything the panel shows is in .stats / .telemetry / .log / .state."""
    DISCONNECTED, CONNECTING, HANDSHAKE, CONNECTED, STALE, ERROR = 'DISCONNECTED', 'CONNECTING', 'HANDSHAKE', 'CONNECTED', 'STALE', 'ERROR'

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.transport, self.profile = None, 'rec1'
        self.parser = FrameParser()
        self.state = self.DISCONNECTED
        self.armed = False
        self.auto_reconnect = False
        self.cmd = Command()
        self.log = deque(maxlen=200)              # (time, 'tx' | 'rx' | 'sys' | 'err', text)
        self.telemetry = {}
        self.hello = {}
        self.stats = {}
        self._reset_stats()
        self._factory = None
        self._retry_at = 0.0
        self._grip_letter = ''

    # ---- bookkeeping
    def _reset_stats(self):
        self.stats = dict(tx=0, rx=0, bad=0, tx_bytes=0, rx_bytes=0, tel=0, lost=0, rtt=None, rtt_avg=None, rtt_max=0.0, rtt_min=None,
                          tx_rate=0.0, rx_rate=0.0, tel_rate=0.0, since_rx=None, connected_at=None)
        self._seq, self._tel_seq_last = 0, None
        self._sent = {}
        self._rate_t, self._rate_n = self.clock(), (0, 0, 0)
        self._last_rx = None
        self._next_tx = self._next_ping = 0.0
        self._ping_sent = {}
        self._hello_deadline = None
        self._rtts = deque(maxlen=40)
        self.telemetry, self.hello = {}, {}

    def say(self, kind, text):
        self.log.append((self.clock(), kind, text))

    @property
    def connected(self): return self.state in (self.CONNECTED, self.STALE)

    # ---- connect / disconnect
    def connect(self, factory, profile='rec1'):
        """factory() returns a fresh Transport (so auto-reconnect can build it again)."""
        self.disconnect(quiet=True)
        self._factory, self.profile = factory, profile
        self._open()

    def _open(self):
        self.state = self.CONNECTING
        self._reset_stats()
        self.parser = FrameParser()
        self._grip_letter = ''
        try:
            self.transport = self._factory()
            self.transport.open()
        except Exception as e:
            self._fail('open failed: %s' % e)
            return
        self.say('sys', 'opened %s [%s]' % (self.transport.name, self.profile))
        now = self.clock()
        self.stats['connected_at'] = now
        if self.profile == 'rec1':
            self.state = self.HANDSHAKE
            self._hello_deadline = now + HELLO_TIMEOUT
            self._write(encode('HELLO?'), 'HELLO?')
        else:
            self.state = self.CONNECTED          # legacy robots say nothing back: an open port is all we can know
            self.say('sys', 'legacy profile: no telemetry, no handshake')

    def disconnect(self, quiet=False):
        if self.transport:
            try:
                if self.transport.is_open and self.profile == 'rec1':
                    self.transport.write(encode('STOP'))
                elif self.transport.is_open:
                    self.transport.write(b'S')
            except Exception:
                pass
            self.transport.close()
        was = self.state != self.DISCONNECTED
        self.transport, self.state = None, self.DISCONNECTED
        self.auto_retry_blocked = True
        if was and not quiet:
            self.say('sys', 'disconnected')

    def _fail(self, msg):
        self.say('err', msg)
        self.state = self.ERROR
        if self.transport:
            try: self.transport.close()
            except Exception: pass
        self._retry_at = self.clock() + 2.0
        self.auto_retry_blocked = False

    auto_retry_blocked = True

    # ---- send
    def _write(self, data, shown=None):
        try:
            self.transport.write(data)
        except Exception as e:
            self._fail('write failed: %s' % e)
            return False
        self.stats['tx'] += 1
        self.stats['tx_bytes'] += len(data)
        if shown is not None:
            self.say('tx', shown)
        return True

    def send_raw(self, text):
        """Terminal input: sends the text plus a newline exactly as typed."""
        if self.transport and self.transport.is_open:
            self._write(text.encode('ascii', 'replace') + b'\n', text)

    def send_frame(self, ftype, *fields):
        if self.transport and self.transport.is_open:
            data = encode(ftype, *fields)
            self._write(data, data.decode().strip())

    def set_command(self, cmd): self.cmd = cmd

    def estop(self):
        """Disarm and tell the robot to stop at once."""
        self.armed = False
        if self.transport and self.transport.is_open:
            self._write(encode('STOP') if self.profile == 'rec1' else b'S', 'STOP')
        self.say('sys', 'E-STOP: output disarmed')

    # ---- every frame
    def update(self):
        now = self.clock()
        if self.state == self.ERROR and self.auto_reconnect and not self.auto_retry_blocked and now >= self._retry_at:
            self.say('sys', 'reconnecting...')
            self._open()
        if not self.transport or self.state in (self.DISCONNECTED, self.ERROR):
            return
        try:
            data = self.transport.read()
        except Exception as e:
            self._fail('read failed: %s' % e)
            return
        if data:
            self.stats['rx_bytes'] += len(data)
            self._last_rx = now
            for ev in self.parser.feed(data):
                self._event(ev, now)
        self._timers(now)

    def _event(self, ev, now):
        s = self.stats
        if ev[0] == 'text':
            self.say('rx', ev[1]); s['rx'] += 1
        elif ev[0] == 'bad':
            s['bad'] += 1; self.say('err', 'bad frame: %s' % ev[1][:60])
        else:
            _, t, f = ev
            s['rx'] += 1
            try:
                if t == 'TEL':
                    self._on_tel(f, now)
                elif t == 'PONG':
                    t0 = self._ping_sent.pop(int(f[0]), None)
                    if t0 is not None: self._rtt(now - t0)
                elif t == 'HELLO':
                    self.hello = dict(proto=f[0], fw=f[1], robot=f[2], version=f[3] if len(f) > 3 else '')
                    self.say('rx', '$HELLO ' + ' '.join(f))
                    if self.state == self.HANDSHAKE: self.state = self.CONNECTED
                else:
                    self.say('rx', '$%s %s' % (t, ' '.join(f)))
            except (IndexError, ValueError):
                s['bad'] += 1; self.say('err', 'malformed %s' % t)

    def _on_tel(self, f, now):
        s = self.stats
        tseq, ack = int(f[0]), int(f[1])
        self.telemetry = dict(tseq=tseq, ack=ack, t_ms=int(f[2]), yaw=float(f[3]), gz=float(f[4]), vl=int(f[5]), vr=int(f[6]), grip=int(f[7]),
                              lift=int(f[8]), vbat_mv=int(f[9]), flags=int(f[10]), at=now)
        s['tel'] += 1
        if self._tel_seq_last is not None:
            gap = (tseq - self._tel_seq_last) % 65536
            if 1 < gap < 1000: s['lost'] += gap - 1
        self._tel_seq_last = tseq
        t0 = self._sent.pop(ack, None)
        if t0 is not None: self._rtt(now - t0)
        for k in [k for k in self._sent if (ack - k) % 65536 < 30000 and k != ack]:
            self._sent.pop(k, None)
        if self.state == self.STALE: self.state = self.CONNECTED
        if self.state == self.HANDSHAKE: self.state = self.CONNECTED

    def _rtt(self, r):
        s = self.stats
        s['rtt'] = r * 1000
        self._rtts.append(r * 1000)
        s['rtt_avg'] = sum(self._rtts) / len(self._rtts)
        s['rtt_max'] = max(s['rtt_max'], r * 1000)
        s['rtt_min'] = r * 1000 if s['rtt_min'] is None else min(s['rtt_min'], r * 1000)

    def _timers(self, now):
        s = self.stats
        if self.state == self.HANDSHAKE and now > self._hello_deadline:
            self.say('err', 'no HELLO: is the robot running the rec1 firmware? (use the Legacy profile for esp32_remote_test.ino)')
            self.state = self.CONNECTED                   # keep the port open; the terminal still works
            self._hello_deadline = None
        s['since_rx'] = None if self._last_rx is None else now - self._last_rx
        if self.profile == 'rec1' and self.state == self.CONNECTED and s['since_rx'] is not None and s['since_rx'] > STALE_AFTER and self.hello:
            self.state = self.STALE
            self.say('err', 'link stale: nothing heard for %.1f s' % s['since_rx'])
        if self.state in (self.CONNECTED, self.STALE, self.HANDSHAKE):
            if now >= self._next_tx:
                self._next_tx = max(self._next_tx + 1.0 / TX_HZ, now - 0.1)
                self._send_cmd(now)
            if self.profile == 'rec1' and now >= self._next_ping:
                self._next_ping = now + PING_EVERY
                self._seq = (self._seq + 1) % 65536
                self._ping_sent[self._seq] = now
                if len(self._ping_sent) > 8: self._ping_sent.pop(next(iter(self._ping_sent)))
                self._write(encode('PING', self._seq))
        if now - self._rate_t >= 1.0:
            dt = now - self._rate_t
            tx, rx, tel = s['tx'], s['rx'], s['tel']
            s['tx_rate'], s['rx_rate'], s['tel_rate'] = (tx - self._rate_n[0]) / dt, (rx - self._rate_n[1]) / dt, (tel - self._rate_n[2]) / dt
            self._rate_n, self._rate_t = (tx, rx, tel), now

    def _send_cmd(self, now):
        cmd = self.cmd if self.armed else self.cmd.zero()
        if self.armed:
            cmd.enable = True
        if self.profile == 'rec1':
            self._seq = (self._seq + 1) % 65536
            self._sent[self._seq] = now
            if len(self._sent) > 60: self._sent.pop(next(iter(self._sent)))
            self._write(encode_command('rec1', self._seq, cmd))
        else:
            self._write(encode_command('legacy', 0, cmd))
            if self.armed:
                b, self._grip_letter = legacy_grip_letter(cmd.grip, self._grip_letter)
                if b: self._write(b, b.decode())

    # ---- helpers for the panel
    def telemetry_age(self):
        t = self.telemetry.get('at')
        return None if t is None else self.clock() - t

    def uptime(self):
        c = self.stats.get('connected_at')
        return 0.0 if c is None or not self.connected else self.clock() - c

    def loss_percent(self):
        s = self.stats
        total = s['tel'] + s['lost']
        return 0.0 if total == 0 else 100.0 * s['lost'] / total

    def quality(self):
        """0..4 bars from latency, loss and freshness. 0 when not connected."""
        if self.state == self.STALE: return 1
        if not self.connected: return 0
        if self.profile != 'rec1': return 3
        q = 4
        rtt = self.stats['rtt_avg']
        if rtt is None: return 2
        if rtt > 60: q -= 1
        if rtt > 150: q -= 1
        if self.loss_percent() > 2: q -= 1
        if self.loss_percent() > 10: q -= 1
        return max(1, q)


# ----------------------------------------------------------------------------------------------- telemetry -> picture
class TelemetryPose:
    """Turns telemetry into the picture data of robot_sim3.snapshot(). Position is dead reckoning (wheel percent x 30 cm/s, heading from the gyro):
    fine for watching, not for navigation."""
    MAX_CM_S = 30.0

    def __init__(self, robot_type='simple'):
        self.type, self.x, self.y, self.al, self.ar, self.t = robot_type, 0.0, 0.0, 0.0, 0.0, None
        self.yaw0 = 0.0

    def zero(self, tel): self.yaw0 = tel.get('yaw', 0.0)

    def reset(self): self.x = self.y = 0.0

    def snapshot(self, tel, at=None):
        if not tel:
            return None
        th = math.radians(tel['yaw'])
        if self.t is not None and at is not None:
            dt = max(0.0, min(0.2, at - self.t))
            v = (tel['vl'] + tel['vr']) / 200.0 * self.MAX_CM_S
            self.x += -math.sin(th) * v * dt
            self.y += math.cos(th) * v * dt
            self.al += tel['vl'] / 100.0 * self.MAX_CM_S * dt / 3.0
            self.ar += tel['vr'] / 100.0 * self.MAX_CM_S * dt / 3.0
        self.t = at
        d = dict(type=self.type, x=self.x, y=self.y, th=th, ref_th=math.radians(self.yaw0), heading=(-(tel['yaw'] - self.yaw0)) % 360,
                 vl=tel['vl'] / 100.0, vr=tel['vr'] / 100.0, al=self.al, ar=self.ar, speed=(tel['vl'] + tel['vr']) / 200.0 * self.MAX_CM_S,
                 omega=math.radians(tel['gz']), holding=False, cubes=[], turn_on=False)
        if self.type == 'simple':
            d.update(arm_l=-40.0 + 46.0 * tel['grip'] / 100.0, arm_r=-40.0 + 46.0 * tel['grip'] / 100.0, cmd=tel['grip'] / 100.0)
        else:
            d.update(lift=0.6 + 11.4 * tel['lift'] / 100.0, pad=3.0 * tel['grip'] / 100.0, cmd=tel['grip'] / 100.0)
        return d
