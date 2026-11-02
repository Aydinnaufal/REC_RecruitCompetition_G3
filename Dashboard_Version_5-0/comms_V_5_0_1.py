"""
comms_V_5_0_1.py - laptop <-> robot communication for the dashboard (no pygame, testable anywhere). Replaces comms_V_4_1.py.

What is new in V_5_0 (the fix for "no ports found" and "cannot connect to the ESP32 over Bluetooth"):
  * scan_ports() / list_serial_ports() no longer hide errors. A missing pyserial, a Windows registry-only port and an unusable
    "Incoming" Bluetooth COM port are all reported. Windows' registry is read as a second source, so a Bluetooth COM port that
    pyserial misses is still listed. Bluetooth "Incoming" COM ports are recognised and never used (opening them waits for ever).
  * diagnose(): the "Port Doctor". One call collects Python, pyserial, bleak, COM ports, paired Bluetooth devices (with MAC addresses)
    and ends with a plain list of next steps. Used by the dashboard's DOCTOR tab and by port_doctor_V_5_0.py.
  * BleTransport + BleScanner: Bluetooth LOW ENERGY (Nordic UART service). The dashboard scans and lists the robot by name,
    no Windows pairing and no COM port are needed. Needs  pip install bleak.
  * RfcommTransport: Bluetooth Classic straight to the robot's MAC address with Python sockets, no COM port needed.

Everything below "ports" in V_4_1 (framing, Link, FakeRobot, PortScanner, TelemetryPose) is kept as it was.


What is new in V_4_1 (all of it is about making Bluetooth dependable):
  * A Bluetooth COM port can take several seconds to open. Opening now happens in a background thread, so the window never freezes.
  * The handshake keeps asking: HELLO? is repeated every 0.5 s (V_4_0 sent it once, so a robot that was not ready yet was never noticed).
    Nothing heard in time -> state NO REPLY (the port is open but the robot is silent), and HELLO? keeps going until it answers.
  * Opening a port no longer toggles DTR / RTS, so a USB ESP32 does not reset every time the dashboard connects.
  * A full Bluetooth buffer drops one command instead of killing the link. Port errors are turned into plain sentences.
  * Reconnecting (or any error / disconnect) always DISARMS the output. V_4_0 documented this but did not do it.
  * PortScanner ("Auto-find"): tries every Bluetooth / USB COM port with HELLO? and reports the one that answers.
  * Robot failsafe is 400 ms (was 300): Bluetooth can deliver a burst of commands late, and a stop on every hiccup feels jerky.

Layers (top to bottom), unchanged from V_4_0:
    Link            connection state machine, handshake, heartbeat, ping, statistics, terminal log
    Profile         'rec1' (framed lines, telemetry back)   or   'legacy' (one-letter F B L R S C O protocol)
    FrameParser     bytes -> frames / plain text lines / bad frames
    Transport       SerialTransport (USB or Bluetooth COM port)  |  VirtualRobotTransport (a fake ESP32 for testing)

Frame format (rec1), one ASCII line:   $TYPE,field,field,...*HH\\n     HH = two hex digits, XOR of every character between '$' and '*'
Laptop -> robot:  CMD (every 50 ms, doubles as the heartbeat), PING, HELLO?, CFG, ZERO, STOP
Robot  -> laptop: HELLO, TEL (20 Hz), PONG, ACK, LOG;   any line that does not start with '$' is shown in the terminal as plain text.
"""
import math, os, random, re, socket, subprocess, sys, threading, time
from collections import deque, namedtuple

PROTO = 1
VERSION = 'V_5_0'      # protocol/firmware version string stays; this file is revision 1 (BLE link no longer dies on one failed write)
TX_HZ = 20.0                 # CMD rate (also the heartbeat)
PING_EVERY = 1.0
STALE_AFTER = 0.6            # connected but nothing heard for this long -> STALE
HELLO_TIMEOUT = 2.0          # default; a transport can ask for longer (Bluetooth: 8 s)
HELLO_EVERY = 0.5            # HELLO? is repeated this often until the robot answers
FAILSAFE_MS = 400            # the robot stops when it has heard no valid CMD for this long
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


# ----------------------------------------------------------------------------------------------- errors in plain words
def friendly_error(e):
    """Turns the cryptic text pyserial / Windows give into one sentence the person can act on."""
    s = str(e).strip() or e.__class__.__name__
    low = s.lower()
    if 'permission' in low or 'access is denied' in low or 'errno 13' in low or 'resource busy' in low or 'device or resource busy' in low:
        return 'port busy or access denied: close the Arduino Serial Monitor, PuTTY or any other program using it (%s)' % s[:60]
    if 'semaphore' in low or 'errno 121' in low or 'winerror 121' in low or 'timed out' in low:
        return 'Bluetooth did not answer: is the robot powered, in range and paired? (%s)' % s[:60]
    if 'filenotfound' in low or 'no such file' in low or 'errno 2' in low or 'cannot find the file' in low or 'winerror 2' in low or 'could not open port' in low and 'find' in low:
        return 'port not found: re-plug the USB cable or pair the robot again, then press Refresh (%s)' % s[:60]
    if 'unplug' in low or 'device not configured' in low or 'input/output error' in low or 'errno 5' in low or 'errno 6' in low or 'winerror 22' in low:
        return 'the port went away (cable unplugged or Bluetooth dropped): %s' % s[:60]
    return s


# ----------------------------------------------------------------------------------------------- ports
PortInfo = namedtuple('PortInfo', 'device description hwid bluetooth role mac', defaults=('', ''))
# role: 'out' = Windows "Outgoing" Bluetooth COM port (the one to use), 'in' = "Incoming" (never use), '' = not Bluetooth / unknown
ZERO_MAC = '000000000000'


def is_bluetooth_port(device, description='', hwid=''):
    d, w = (description or '').lower(), (hwid or '').upper()
    return 'bluetooth' in d or 'BTHENUM' in w or 'BTHMODEM' in w or (device or '').startswith('/dev/rfcomm') or 'bluetooth' in (device or '').lower()


def fmt_mac(m12):
    m = (m12 or '').upper().replace(':', '').replace('-', '')
    return ':'.join(m[i:i + 2] for i in range(0, 12, 2)) if len(m) == 12 else (m12 or '')


def bt_role(device, hwid):
    """('out', 'AA:BB:..') / ('in', '') / ('', '') for a COM port. On Windows the hardware id of a Bluetooth serial port ends in the robot's
    MAC address for an Outgoing port and in all zeros for an Incoming one."""
    w = (hwid or '').upper()
    if 'BTHENUM' not in w and 'BTHMODEM' not in w:
        return ('out', '') if (device or '').startswith('/dev/rfcomm') else ('', '')
    m = re.findall(r'(?<![0-9A-F])([0-9A-F]{12})(?![0-9A-F])', w)
    if m and m[-1] != ZERO_MAC:
        return 'out', fmt_mac(m[-1])
    if m and m[-1] == ZERO_MAC:
        return 'in', ''
    return '', ''


def _make_port(device, description, hwid):
    bt = is_bluetooth_port(device, description, hwid)
    role, mac = bt_role(device, hwid) if bt else ('', '')
    return PortInfo(device, description or '', hwid or '', bt, role, mac)


def _registry_ports():
    """{'COM5': '\\Device\\BthModem0', ...} from Windows' own list of serial ports (empty on other systems). A second opinion next to pyserial."""
    try:
        import winreg
        out = {}
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'HARDWARE\DEVICEMAP\SERIALCOMM') as k:
            i = 0
            while True:
                try:
                    name, val, _ = winreg.EnumValue(k, i)
                except OSError:
                    break
                out[str(val)] = str(name)
                i += 1
        return out
    except Exception:
        return {}


class PortScan:
    """Result of scan_ports(): .ports (PortInfo list), .problems (plain sentences), .serial_version, .registry (COM -> device name)."""

    def __init__(self):
        self.ports, self.problems, self.serial_version, self.registry = [], [], None, {}


def install_hint(module):
    return '"%s" -m pip install %s' % (sys.executable, module)


def scan_ports(comports=None, registry=None):
    """Every COM port the PC offers, with the reason when there are none. comports / registry can be given for tests."""
    r = PortScan()
    rows = None
    try:
        if comports is None:
            import serial
            from serial.tools import list_ports
            r.serial_version = getattr(serial, '__version__', '?')
            comports = list_ports.comports()
        rows = [(p.device, p.description, p.hwid) for p in comports]
    except ImportError as e:
        r.problems.append('pyserial is not installed for this Python (%s). Run:  %s' % (e, install_hint('pyserial')))
    except Exception as e:
        r.problems.append('pyserial failed while listing ports: %s' % e)
    seen = set()
    for dev, desc, hw in (rows or []):
        seen.add(dev.upper())
        r.ports.append(_make_port(dev, desc, hw))
    r.registry = _registry_ports() if registry is None else dict(registry)
    for com, name in r.registry.items():                       # ports Windows knows but pyserial did not list
        if com.upper() in seen:
            continue
        bt = 'bthmodem' in name.lower()
        r.ports.append(PortInfo(com, 'Bluetooth link (from registry)' if bt else 'from registry: ' + name.split('\\')[-1],
                                'REGISTRY ' + name, bt, '', ''))
        if rows is not None:
            r.problems.append('%s is known to Windows but pyserial did not list it' % com)
    r.ports.sort(key=lambda p: (p.role == 'in', not p.bluetooth, p.device))
    return r


LAST_SCAN = PortScan()


def list_serial_ports():
    """[PortInfo] of the COM ports, usable Bluetooth first. The reason for an empty list is in LAST_SCAN.problems."""
    global LAST_SCAN
    LAST_SCAN = scan_ports()
    return LAST_SCAN.ports


def probeable(p):
    """Ports worth sending HELLO? to: Outgoing Bluetooth links and USB serial adapters. Skips the Incoming Bluetooth port (opening it waits for ever)
    and motherboard COM1 and the like."""
    if getattr(p, 'role', '') == 'in':
        return False
    w = (p.hwid or '').upper()
    if w.startswith('REGISTRY'):
        return p.bluetooth or not re.search(r'\\DEVICE\\SERIAL\d', w)
    return bool(p.bluetooth or w.startswith(('USB', 'FTDIBUS')) or p.device.startswith(('/dev/ttyUSB', '/dev/ttyACM', '/dev/cu.usb', '/dev/cu.SLAB')))


# ----------------------------------------------------------------------------------------------- transports
class Transport:
    name = 'transport'
    is_bluetooth = False
    handshake_timeout = HELLO_TIMEOUT
    open_budget = 3.0            # how long a scan waits for this transport to open

    def __init__(self):
        self.is_open, self.last_error, self.opening = False, '', False

    def open(self): raise NotImplementedError

    def start_open(self):
        """Begin opening. Simple transports open at once; SerialTransport opens in a thread and keeps .opening True meanwhile."""
        self.opening = True
        try:
            self.open()
        finally:
            self.opening = False

    def close(self): self.is_open = False
    def write(self, data): raise NotImplementedError
    def read(self): return b''


class SerialTransport(Transport):
    """A COM port: USB cable or a paired Bluetooth link (Windows 'Outgoing' COM port, Linux /dev/rfcomm0).
    Opens in a background thread (a Bluetooth port can take seconds and fails slowly when the robot is off) and never touches DTR / RTS."""

    def __init__(self, port, baud=115200, bluetooth=False, retries=None, retry_wait=1.0):
        super().__init__()
        self.port, self.baud, self.ser = port, baud, None
        self.is_bluetooth = bool(bluetooth)
        self.retries = retries if retries is not None else (3 if self.is_bluetooth else 1)
        self.retry_wait = retry_wait
        self.handshake_timeout = 8.0 if self.is_bluetooth else 2.5
        self.open_budget = 14.0 if self.is_bluetooth else 3.0
        self.name = '%s%s @ %d' % (port, ' (BT)' if self.is_bluetooth else '', baud)
        self.dropped = 0
        self._cancel = False
        self._lock = threading.Lock()

    def start_open(self):
        self.opening, self._cancel, self.last_error = True, False, ''
        threading.Thread(target=self._open_worker, daemon=True).start()

    def open(self):
        """Blocking open (used by tools that do not mind waiting)."""
        self.start_open()
        while self.opening:
            time.sleep(0.02)
        if not self.is_open:
            raise IOError(self.last_error or 'open failed')

    def _open_worker(self):
        err = ''
        try:
            import serial
            for attempt in range(max(1, self.retries)):
                if self._cancel:
                    break
                try:
                    ser = serial.Serial()
                    ser.port, ser.baudrate = self.port, self.baud
                    ser.timeout, ser.write_timeout = 0, 0.3
                    ser.dtr, ser.rts = False, False            # do not pulse the ESP32 reset circuit
                    ser.open()
                except Exception as e:
                    err = friendly_error(e)
                    if attempt + 1 < self.retries and not self._cancel:
                        time.sleep(self.retry_wait)
                    continue
                with self._lock:
                    if self._cancel:
                        ser.close()
                    else:
                        self.ser, self.is_open, err = ser, True, ''
                break
        except ImportError:
            err = 'pyserial is not installed for this Python: run  ' + install_hint('pyserial')
        except Exception as e:
            err = friendly_error(e)
        self.last_error = err if not self.is_open else ''
        self.opening = False

    def close(self):
        self._cancel = True
        with self._lock:
            try:
                if self.ser: self.ser.close()
            except Exception:
                pass
            self.ser, self.is_open = None, False

    def write(self, data):
        try:
            self.ser.write(data)
        except Exception as e:
            if type(e).__name__ == 'SerialTimeoutException':      # the Bluetooth buffer is full: drop this one, the next CMD is 50 ms away
                self.dropped += 1
                return
            self.last_error = friendly_error(e)
            raise IOError(self.last_error)

    def read(self):
        try:
            n = self.ser.in_waiting
            return self.ser.read(n) if n else b''
        except Exception as e:
            self.last_error = friendly_error(e)
            raise IOError(self.last_error)


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
        return [encode('HELLO', PROTO, 'fake-esp32', self.type, VERSION)]

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
    """Pretends to be a serial port with a FakeRobot on the other end. latency_ms / loss / jitter_ms make the link behave badly on purpose.
    silent_for = seconds after opening during which the robot hears and says nothing (a robot that is still booting / not connected yet)."""

    def __init__(self, robot_type='simple', clock=time.monotonic, latency_ms=25, jitter_ms=8, loss=0.0, seed=3, silent_for=0.0):
        super().__init__()
        self.name = 'Virtual robot (test)'
        self.clock = clock
        self.robot = FakeRobot(robot_type, seed)
        self.latency, self.jitter, self.loss = latency_ms / 1000.0, jitter_ms / 1000.0, loss
        self.rnd = random.Random(seed)
        self.to_robot, self.to_laptop = deque(), deque()       # (deliver_time, bytes)
        self.cut = False                                       # simulate pulling the cable
        self.silent_for, self.silent_until = silent_for, 0.0

    def open(self):
        self.is_open, self.last_error = True, ''
        self.robot.reset(self.clock())
        self.silent_until = self.clock() + self.silent_for
        if not self.silent_for:
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
        if self.clock() < self.silent_until:
            return                                             # nobody is listening yet
        if self.loss and self.rnd.random() < self.loss:
            return
        self.to_robot.append((self.clock() + self._delay(), data))

    def read(self):
        if self.cut:
            raise IOError('cable unplugged')
        now = self.clock()
        if now < self.silent_until:
            return b''
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


# ----------------------------------------------------------------------------------------------- auto-find
class PortScanner:
    """"Auto-find robot": opens each candidate port in turn (Bluetooth first), sends HELLO? and waits for a HELLO or TEL.
    Runs in a thread; the dashboard reads .running / .message / .found / .results every frame. factory(PortInfo) -> Transport."""

    def __init__(self, ports, factory=None, only_probeable=True, clock=time.monotonic):
        ports = list(ports)
        self.ports = [p for p in ports if probeable(p)] if only_probeable else ports
        self.ports.sort(key=lambda p: (not p.bluetooth, p.device))
        self.factory = factory or (lambda p: SerialTransport(p.device, 115200, bluetooth=p.bluetooth))
        self.clock = clock
        self.running, self.found, self.results = False, None, []     # found = (PortInfo, hello dict)
        self.message, self.current = '', ''
        self._cancel = False

    def start(self):
        self.running, self.found, self.results, self._cancel = True, None, [], False
        self.message = 'scanning %d port(s)...' % len(self.ports) if self.ports else ''
        threading.Thread(target=self._run, daemon=True).start()

    def cancel(self):
        self._cancel = True

    def _run(self):
        try:
            if not self.ports:
                self.message = 'no Bluetooth/USB ports found: pair the robot or plug it in, then Refresh'
                return
            for i, p in enumerate(self.ports):
                if self._cancel:
                    self.message = 'scan stopped'
                    return
                self.current = p.device
                self.message = 'trying %s%s (%d of %d)...' % (p.device, ' BT' if p.bluetooth else '', i + 1, len(self.ports))
                ok, info = self._probe(p)
                self.results.append((p.device, ok, info if isinstance(info, str) else 'robot'))
                if ok:
                    self.found = (p, info)
                    self.message = 'found the robot on %s' % p.device
                    return
            self.message = 'no robot answered on %d port(s): powered on, paired, running V_4_1?' % len(self.ports)
        except Exception as e:
            self.message = 'scan failed: %s' % friendly_error(e)
        finally:
            self.running = False

    def _probe(self, p):
        t = self.factory(p)
        try:
            try:
                t.start_open()
            except Exception as e:                       # a transport may fail at once: that is this port's result, not the scan's
                return False, friendly_error(e)
            t_open = self.clock() + t.open_budget
            while t.opening and self.clock() < t_open and not self._cancel:
                time.sleep(0.02)
            if t.opening:
                return False, 'open timed out'
            if not t.is_open:
                return False, t.last_error or 'could not open'
            parser = FrameParser()
            end = self.clock() + t.handshake_timeout
            next_hello = 0.0
            while self.clock() < end and not self._cancel:
                now = self.clock()
                if now >= next_hello:
                    next_hello = now + HELLO_EVERY
                    try: t.write(encode('HELLO?'))
                    except Exception as e: return False, friendly_error(e)
                try: data = t.read()
                except Exception as e: return False, friendly_error(e)
                for ev in parser.feed(data):
                    if ev[0] == 'frame' and ev[1] == 'HELLO':
                        f = ev[2]
                        return True, dict(proto=f[0], fw=f[1], robot=f[2], version=f[3] if len(f) > 3 else '')
                    if ev[0] == 'frame' and ev[1] in ('TEL', 'PONG'):
                        return True, {}
                time.sleep(0.02)
            return False, 'no answer'
        finally:
            try: t.write(encode('STOP'))
            except Exception: pass
            t.close()


# ----------------------------------------------------------------------------------------------- link
class Link:
    """The connection. Call update() every frame (about 60 Hz). Everything the panel shows is in .stats / .telemetry / .log / .state."""
    DISCONNECTED, CONNECTING, HANDSHAKE, CONNECTED, STALE, NO_REPLY, ERROR = \
        'DISCONNECTED', 'CONNECTING', 'HANDSHAKE', 'CONNECTED', 'STALE', 'NO REPLY', 'ERROR'

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
        self.last_error = ''
        self.heard = False
        self._reset_stats()
        self._factory = None
        self._retry_at = 0.0
        self._grip_letter = ''
        self._next_hello = 0.0

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
        self.heard = False

    def say(self, kind, text):
        self.log.append((self.clock(), kind, text))

    @property
    def connected(self): return self.state in (self.CONNECTED, self.STALE, self.NO_REPLY)

    @property
    def active(self):
        """True from the moment Connect is pressed until Disconnect or an error."""
        return self.state not in (self.DISCONNECTED, self.ERROR)

    # ---- connect / disconnect
    def connect(self, factory, profile='rec1'):
        """factory() returns a fresh Transport (so auto-reconnect can build it again)."""
        self.disconnect(quiet=True)
        self._factory, self.profile = factory, profile
        self._open()

    def _open(self):
        self.armed = False                       # a new connection always starts disarmed
        self.state = self.CONNECTING
        self._reset_stats()
        self.parser = FrameParser()
        self._grip_letter = ''
        try:
            self.transport = self._factory()
            self.transport.start_open()
        except Exception as e:
            self._fail('open failed: %s' % friendly_error(e))
            return
        self.say('sys', 'opening %s%s ...' % (self.transport.name, '  (Bluetooth can take a few seconds)' if self.transport.is_bluetooth else ''))
        self._finish_open()

    def _finish_open(self):
        t = self.transport
        if t.opening:
            return
        if not t.is_open:
            self._fail('open failed: %s' % (t.last_error or 'unknown error'))
            return
        now = self.clock()
        self.stats['connected_at'] = now
        self.say('sys', 'port open [%s]' % self.profile)
        if self.profile == 'rec1':
            self.state = self.HANDSHAKE
            self._hello_deadline = now + t.handshake_timeout
            self._write(encode('HELLO?'), 'HELLO?')
            self._next_hello = now + HELLO_EVERY
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
        self.armed = False
        self.auto_retry_blocked = True
        if was and not quiet:
            self.say('sys', 'disconnected')

    def _fail(self, msg):
        self.last_error = msg
        self.say('err', msg)
        if self.armed:
            self.say('sys', 'output disarmed because the link failed')
        self.armed = False
        self.state = self.ERROR
        if self.transport:
            try: self.transport.close()
            except Exception: pass
        self._retry_at = self.clock() + (3.0 if (self.transport and self.transport.is_bluetooth) else 2.0)
        self.auto_retry_blocked = False

    auto_retry_blocked = True

    # ---- send
    def _write(self, data, shown=None):
        try:
            self.transport.write(data)
        except Exception as e:
            self._fail('write failed: %s' % friendly_error(e))
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
        if self.state == self.CONNECTING and self.transport:
            self._finish_open()
        if not self.transport or self.state in (self.DISCONNECTED, self.ERROR, self.CONNECTING):
            return
        try:
            data = self.transport.read()
        except Exception as e:
            self._fail('read failed: %s' % friendly_error(e))
            return
        if data:
            self.stats['rx_bytes'] += len(data)
            self._last_rx = now
            for ev in self.parser.feed(data):
                self._event(ev, now)
        self._timers(now)

    def _heard_robot(self):
        """Any valid frame proves the robot is there."""
        self.heard = True
        if self.state in (self.HANDSHAKE, self.NO_REPLY, self.STALE):
            if self.state != self.STALE:
                self.say('sys', 'robot answered')
            self.state = self.CONNECTED

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
                    self._heard_robot()
                    t0 = self._ping_sent.pop(int(f[0]), None)
                    if t0 is not None: self._rtt(now - t0)
                elif t == 'HELLO':
                    self.hello = dict(proto=f[0], fw=f[1], robot=f[2], version=f[3] if len(f) > 3 else '')
                    self.say('rx', '$HELLO ' + ' '.join(f))
                    self._heard_robot()
                else:
                    self._heard_robot()
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
        self._heard_robot()

    def _rtt(self, r):
        s = self.stats
        s['rtt'] = r * 1000
        self._rtts.append(r * 1000)
        s['rtt_avg'] = sum(self._rtts) / len(self._rtts)
        s['rtt_max'] = max(s['rtt_max'], r * 1000)
        s['rtt_min'] = r * 1000 if s['rtt_min'] is None else min(s['rtt_min'], r * 1000)

    def _timers(self, now):
        s = self.stats
        rec1 = self.profile == 'rec1'
        if rec1 and self.state == self.HANDSHAKE:
            if now >= self._next_hello:
                self._next_hello = now + HELLO_EVERY
                self._write(encode('HELLO?'))
            if self._hello_deadline is not None and now > self._hello_deadline:
                self.state = self.NO_REPLY
                self._next_hello = now + 1.0
                self.say('err', 'port is open but the robot is silent: is it powered and running the V_4_1 sketch? (for esp32_remote_test.ino use the Legacy profile)')
        elif rec1 and self.state == self.NO_REPLY and now >= self._next_hello:
            self._next_hello = now + 1.0
            self._write(encode('HELLO?'))
        elif rec1 and self.state in (self.CONNECTED, self.STALE) and not self.hello and now >= self._next_hello:
            self._next_hello = now + 2.0                  # telemetry arrived but the banner did not: ask again so the version shows
            self._write(encode('HELLO?'))
        s['since_rx'] = None if self._last_rx is None else now - self._last_rx
        if rec1 and self.state == self.CONNECTED and s['since_rx'] is not None and s['since_rx'] > STALE_AFTER and self.heard:
            self.state = self.STALE
            self.say('err', 'link stale: nothing heard for %.1f s' % s['since_rx'])
        if self.state in (self.CONNECTED, self.STALE, self.HANDSHAKE, self.NO_REPLY):
            if now >= self._next_tx:
                self._next_tx = max(self._next_tx + 1.0 / TX_HZ, now - 0.1)
                self._send_cmd(now)
            if rec1 and now >= self._next_ping:
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
        if self.state in (self.STALE, self.NO_REPLY): return 1
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


# =============================================================================================== V_5_0: Port Doctor, Bluetooth LE, direct Bluetooth Classic
import platform

NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'       # Nordic UART service: what the V_5_0 firmware offers in BLE mode
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'            # laptop -> robot (write)
NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'            # robot -> laptop (notify)
ROBOT_NAME_PREFIX = 'REC'                                  # the firmware advertises as "REC-Robot"


def bleak_missing_text(e=None):
    return 'bleak is not installed for this Python%s. Run:  %s' % ((' (%s)' % e) if e else '', install_hint('bleak'))


def _powershell(script, timeout=12):
    """Runs a PowerShell one-liner and returns its text, or '' when this is not Windows / it failed. Never opens a console window."""
    if os.name != 'nt':
        return ''
    flags = getattr(subprocess, 'CREATE_NO_WINDOW', 0)
    try:
        r = subprocess.run(['powershell', '-NoProfile', '-NonInteractive', '-Command', script], capture_output=True, text=True, timeout=timeout, creationflags=flags)
        return r.stdout or ''
    except Exception:
        return ''


PairedDevice = namedtuple('PairedDevice', 'name mac present kind')      # kind: 'classic' | 'le'


def parse_paired(text):
    """Parses lines  FriendlyName|InstanceId|Status  from Get-PnpDevice -Class Bluetooth into PairedDevice (one per MAC, services merged)."""
    devs = {}
    for line in (text or '').splitlines():
        parts = line.strip().split('|')
        if len(parts) < 3:
            continue
        name, inst, status = parts[0].strip(), parts[1].strip(), parts[2].strip()
        m = re.search(r'DEV_([0-9A-Fa-f]{12})', inst)
        if not m:
            continue
        mac = fmt_mac(m.group(1))
        kind = 'le' if inst.upper().startswith('BTHLE') else 'classic'
        old = devs.get((mac, kind))
        is_node = '{' not in inst                                       # the device node itself, not one of its services
        if old is None or (is_node and not old[2]):
            devs[(mac, kind)] = (name if is_node or not old else old[0], status.upper() == 'OK' or (old[1] if old else False), is_node or (old[2] if old else False))
        else:
            devs[(mac, kind)] = (old[0], old[1] or status.upper() == 'OK', old[2])
    return [PairedDevice(n, mac, ok, kind) for (mac, kind), (n, ok, _) in sorted(devs.items(), key=lambda kv: kv[0][0])]


def paired_devices():
    """Bluetooth devices Windows has paired, with MAC addresses. [] on other systems or when PowerShell says nothing."""
    txt = _powershell("Get-PnpDevice -Class Bluetooth -ErrorAction SilentlyContinue | ForEach-Object { $_.FriendlyName + '|' + $_.InstanceId + '|' + $_.Status }")
    return parse_paired(txt)


class Report:
    """Output of diagnose(): lines = [(level, text)] with level ok / warn / bad / info, steps = what to do next, in order."""

    def __init__(self):
        self.lines, self.steps, self.ports, self.paired, self.scan = [], [], [], [], None
        self.when = time.strftime('%Y-%m-%d %H:%M:%S')

    def add(self, level, text): self.lines.append((level, text))
    def step(self, text):
        if text not in self.steps: self.steps.append(text)

    def text(self):
        out = ['Robot Station port report  %s' % self.when, '']
        out += ['[%s] %s' % ({'ok': ' ok ', 'warn': 'warn', 'bad': 'FAIL', 'info': 'info'}[lv], t) for lv, t in self.lines]
        out += ['', 'NEXT STEPS'] + ['%d. %s' % (i + 1, t) for i, t in enumerate(self.steps)]
        return '\n'.join(out) + '\n'


def diagnose(scan=None, paired=None, check_bleak=True):
    """The Port Doctor. Collects the facts and turns them into next steps. scan / paired can be given for tests."""
    R = Report()
    R.add('info', 'Python %s on %s' % (platform.python_version(), platform.platform()))
    R.add('info', 'running: %s' % sys.executable)
    sc = scan if scan is not None else scan_ports()
    R.scan, R.ports = sc, sc.ports
    # --- pyserial
    if sc.serial_version:
        R.add('ok', 'pyserial %s is installed' % sc.serial_version)
    elif scan is None or any('pyserial' in p for p in sc.problems):
        R.add('bad', 'pyserial is NOT installed for this Python: COM ports cannot be listed or opened')
        R.step('Install pyserial for the Python that runs the dashboard:   %s' % install_hint('pyserial'))
    # --- ports
    usable = [p for p in sc.ports if p.role != 'in']
    if sc.ports:
        R.add('ok' if usable else 'warn', '%d COM port(s) found' % len(sc.ports))
        for p in sc.ports:
            kind = ('Bluetooth OUTGOING to %s (use this one)' % (p.mac or '?')) if p.role == 'out' else \
                   'Bluetooth INCOMING (do not use)' if p.role == 'in' else 'Bluetooth' if p.bluetooth else 'USB / serial'
            R.add('info', '   %s  %s  [%s]' % (p.device, (p.description or '')[:40], kind))
    else:
        R.add('bad' if not sc.problems else 'warn', 'no COM ports found')
    for pr in sc.problems:
        if 'pyserial is not installed' not in pr:
            R.add('warn', pr)
    # --- paired Bluetooth devices
    pd = paired if paired is not None else paired_devices()
    R.paired = pd
    if os.name == 'nt' or paired is not None:
        if pd:
            R.add('ok', '%d paired Bluetooth device(s):' % len(pd))
            for d in pd:
                R.add('info', '   %s  %s  %s%s' % (d.name[:28], d.mac, 'LE' if d.kind == 'le' else 'Classic', '' if d.present else '  (not in range / off)'))
        else:
            R.add('warn', 'Windows lists no paired Bluetooth devices (or the Bluetooth radio is off / missing)')
    robots = [d for d in pd if d.name.upper().startswith(ROBOT_NAME_PREFIX) or 'ESP32' in d.name.upper()]
    robot_classic = [d for d in robots if d.kind == 'classic']
    out_ports = [p for p in sc.ports if p.role == 'out']
    # --- bleak / sockets
    if check_bleak:
        try:
            import bleak
            R.add('ok', 'bleak %s is installed: Bluetooth LE scanning works' % getattr(bleak, '__version__', '?'))
        except ImportError as e:
            R.add('warn', bleak_missing_text(e))
            R.step('For Bluetooth LE (no pairing needed) install bleak:   %s' % install_hint('bleak'))
    R.add('ok' if hasattr(socket, 'AF_BLUETOOTH') else 'warn',
          'Python Bluetooth sockets %s (direct mode)' % ('are available' if hasattr(socket, 'AF_BLUETOOTH') else 'are NOT available in this Python'))
    # --- verdict
    if robot_classic and not out_ports:
        R.add('bad', 'REC-Robot is paired but Windows made no Outgoing COM port for it')
        R.step('Remove REC-Robot in Windows Bluetooth settings, switch the robot on and pair it again (wait for "Paired").')
        R.step('Or skip the COM port: Transport = "Bluetooth Classic (MAC)", or flash the BLE firmware and use "Bluetooth LE".')
    if robots and not robot_classic and any(d.kind == 'le' for d in robots):
        R.add('info', 'the robot is paired as a Bluetooth LE device: use Transport = "Bluetooth LE" (no COM port exists for LE)')
    if not robots and pd is not None and os.name == 'nt':
        R.add('info', 'no paired device named REC-... Fine for Bluetooth LE (it needs no pairing). Classic mode needs pairing first.')
    if not sc.ports and not sc.problems and os.name == 'nt':
        R.step('No COM ports at all: for a USB cable install the CP210x or CH340 driver of your board and re-plug it (Device Manager > Ports).')
    if usable and not R.steps:
        R.step('Looks fine. Transport = "USB / Bluetooth COM", pick the port (Outgoing for Bluetooth) and press Connect, or press Auto-find.')
    if not R.steps:
        R.step('Use Transport = "Bluetooth LE" and press Refresh to scan for REC-Robot.')
    return R


# ----------------------------------------------------------------------------------------------- Bluetooth LE
BleDevice = namedtuple('BleDevice', 'name address rssi nus')


def ble_label(d):
    return '%s  %s' % ((d.name or '(no name)')[:16], d.address[-8:] if len(d.address) > 8 else d.address)


class BleScanner:
    """Scans for Bluetooth LE devices for `timeout` seconds in a background thread. .devices is sorted: the REC robot first, then by signal."""

    def __init__(self, timeout=5.0):
        self.timeout, self.running, self.devices, self.error, self.done = timeout, False, [], '', False
        self.started = 0.0

    def start(self):
        self.running, self.error, self.done, self.started = True, '', False, time.monotonic()
        threading.Thread(target=self._run, daemon=True).start()

    def progress(self): return min(1.0, (time.monotonic() - self.started) / self.timeout) if self.running else 1.0

    def _run(self):
        try:
            import asyncio
            try:
                from bleak import BleakScanner
            except ImportError as e:
                self.error = bleak_missing_text(e)
                return

            async def go():
                return await BleakScanner.discover(timeout=self.timeout, return_adv=True)
            found = asyncio.run(go())
            devs = []
            for key, val in (found.items() if isinstance(found, dict) else []):
                dev, adv = val if isinstance(val, tuple) else (val, None)
                uuids = [u.lower() for u in (getattr(adv, 'service_uuids', None) or [])]
                name = getattr(adv, 'local_name', None) or getattr(dev, 'name', None) or ''
                devs.append(BleDevice(name, getattr(dev, 'address', str(key)), getattr(adv, 'rssi', None) if adv else None, NUS_SERVICE in uuids))
            devs.sort(key=lambda d: (not (d.nus or d.name.upper().startswith(ROBOT_NAME_PREFIX)), -(d.rssi if d.rssi is not None else -200)))
            self.devices = [d for d in devs if d.name or d.nus]          # nameless beacons are noise
        except Exception as e:
            self.error = ble_friendly(e)
        finally:
            self.running, self.done = False, True

    def robot(self):
        for d in self.devices:
            if d.nus or d.name.upper().startswith(ROBOT_NAME_PREFIX):
                return d
        return None


def ble_friendly(e):
    s = str(e).strip() or e.__class__.__name__
    low = s.lower()
    if 'turned off' in low or 'bluetooth is off' in low or 'radio' in low or 'not available' in low or 'org.bluez' in low:
        return 'Bluetooth is off or missing on this PC: switch it on in Windows settings (%s)' % s[:70]
    if 'access' in low and 'denied' in low:
        return 'Windows denied Bluetooth access: allow Bluetooth for desktop apps in Settings > Privacy (%s)' % s[:60]
    if 'timeout' in low or 'timed out' in low:
        return 'the robot did not answer in time: switch it on, bring it close, and make sure no other device is connected to it'
    if 'not found' in low or 'could not find' in low:
        return 'device not found: press Refresh to scan again (the robot must be on and advertising)'
    return friendly_error(e)


class BleTransport(Transport):
    """Bluetooth LE to the robot's Nordic-UART service. All bleak calls run in one thread with its own asyncio loop (that is also what
    keeps pygame's window thread out of Windows' COM apartment problems). Writes never block the dashboard; if the radio falls behind,
    the oldest-unsent command is simply dropped (the next CMD is 50 ms away)."""
    name = 'ble'
    is_bluetooth = True
    handshake_timeout = 8.0
    open_budget = 22.0

    def __init__(self, address, label='', client_factory=None):
        super().__init__()
        self.address, self.label = address, label
        self.name = 'BLE %s' % (label or address)
        self.client_factory = client_factory
        self._rx, self._lock = deque(), threading.Lock()
        self._loop = self._stop = self._client = None
        self._pending, self.dropped, self.chunk = 0, 0, 20
        self._lost = False
        self._write_fails, self.write_errors, self.last_write_error = 0, 0, ''
        self._cancel = False

    def start_open(self):
        self.opening, self._cancel, self.last_error, self._lost = True, False, '', False
        threading.Thread(target=self._thread, daemon=True).start()

    def open(self):
        self.start_open()
        while self.opening:
            time.sleep(0.02)
        if not self.is_open:
            raise IOError(self.last_error or 'open failed')

    def _thread(self):
        import asyncio
        loop = asyncio.new_event_loop()
        self._loop = loop
        try:
            loop.run_until_complete(self._session())
        except Exception as e:
            if not self.is_open:
                self.last_error = ble_friendly(e)
            else:
                self.last_error = ble_friendly(e)
                self._lost = True
        finally:
            self.is_open = False
            self.opening = False
            try: loop.close()
            except Exception: pass

    async def _session(self):
        import asyncio
        self._stop = asyncio.Event()
        factory = self.client_factory
        if factory is None:
            try:
                from bleak import BleakClient
            except ImportError as e:
                self.last_error = bleak_missing_text(e)
                self.opening = False
                return
            factory = BleakClient
        client = factory(self.address, disconnected_callback=self._on_disconnect)
        self._client = client
        last = ''
        for attempt in range(2):
            if self._cancel:
                self.opening = False
                return
            try:
                await client.connect(timeout=10.0)
                last = ''
                break
            except Exception as e:
                last = ble_friendly(e)
        if last:
            self.last_error, self.opening = last, False
            return
        try:
            svcs = client.services
            have = {str(getattr(c, 'uuid', '')).lower() for s in svcs for c in getattr(s, 'characteristics', [])}
            if NUS_RX not in have or NUS_TX not in have:
                self.last_error = ('connected, but this device has no REC serial service: is it the robot, and is the BLE firmware (BT_MODE = BT_BLE) flashed?')
                self.opening = False
                await client.disconnect()
                return
            await client.start_notify(NUS_TX, self._on_notify)
            mtu = getattr(client, 'mtu_size', 23) or 23
            self.chunk = max(20, min(180, int(mtu) - 3))
        except Exception as e:
            self.last_error, self.opening = ble_friendly(e), False
            try: await client.disconnect()
            except Exception: pass
            return
        self.is_open, self.opening = True, False
        await self._stop.wait()
        try: await client.disconnect()
        except Exception: pass

    def _on_notify(self, _char, data):
        with self._lock:
            self._rx.append(bytes(data))

    def _on_disconnect(self, _client):
        if self.is_open and not self._cancel:
            extra = (' after write errors: ' + self.last_write_error) if self.last_write_error else ''
            self.last_error = 'Bluetooth LE link was dropped by Windows or the robot (out of range, robot reset, or radio interference)' + extra
            self._lost = True

    async def _awrite(self, data):
        """One failed write is NOT the end of the link (Windows reports a busy radio as an error now and then). Only a long unbroken run of
        failures, or Windows saying the device is gone, is."""
        try:
            for i in range(0, len(data), self.chunk):
                await self._client.write_gatt_char(NUS_RX, data[i:i + self.chunk], response=False)
            self._write_fails = 0
        except Exception as e:
            self._write_fails += 1
            self.write_errors += 1
            self.last_write_error = '%s: %s' % (e.__class__.__name__, (str(e) or '-')[:80])
            gone = getattr(self._client, 'is_connected', True) is False
            if gone or self._write_fails >= 20:                      # 20 in a row = a second of nothing getting through
                self.last_error = ble_friendly(e) if not gone else 'Bluetooth LE device is gone (%s)' % self.last_write_error
                self._lost = True
        finally:
            self._pending -= 1

    def write(self, data):
        if not self.is_open or self._loop is None:
            raise IOError(self.last_error or 'Bluetooth LE link is not open')
        if self._lost:
            raise IOError(self.last_error)
        if self._pending >= 4:                           # the radio is behind: drop this one rather than build up delay
            self.dropped += 1
            return
        import asyncio
        self._pending += 1
        asyncio.run_coroutine_threadsafe(self._awrite(bytes(data)), self._loop)

    def read(self):
        if self._lost:
            raise IOError(self.last_error or 'Bluetooth LE link lost')
        if self.is_open and getattr(self._client, 'is_connected', True) is False and not self._cancel:
            self._lost = True
            self.last_error = 'Bluetooth LE link lost (Windows reports the device disconnected)'
            raise IOError(self.last_error)
        with self._lock:
            if not self._rx:
                return b''
            out = b''.join(self._rx)
            self._rx.clear()
        return out

    def close(self):
        self._cancel = True
        try:
            if self._loop and self._stop and not self._loop.is_closed():
                self._loop.call_soon_threadsafe(self._stop.set)
        except Exception:
            pass
        self.is_open = False


# ----------------------------------------------------------------------------------------------- Bluetooth Classic without a COM port
class RfcommTransport(Transport):
    """Bluetooth Classic (SPP) straight to a MAC address with Python's own sockets: no Windows COM port is involved. The robot must be paired.
    Needs Python 3.9+ (Windows) with Bluetooth socket support. Tries RFCOMM channels 1, 2, 3."""
    is_bluetooth = True
    handshake_timeout = 8.0
    open_budget = 35.0

    def __init__(self, mac, label='', channels=(1, 2, 3), sock_factory=None, connect_timeout=8.0):
        super().__init__()
        self.mac, self.label, self.channels = mac, label, channels
        self.name = 'BT %s' % (label or mac)
        self.sock_factory, self.connect_timeout = sock_factory, connect_timeout
        self.sock = None
        self._cancel = False
        self.dropped = 0

    def start_open(self):
        self.opening, self._cancel, self.last_error = True, False, ''
        threading.Thread(target=self._worker, daemon=True).start()

    def open(self):
        self.start_open()
        while self.opening:
            time.sleep(0.02)
        if not self.is_open:
            raise IOError(self.last_error or 'open failed')

    def _worker(self):
        err = ''
        try:
            if self.sock_factory is None and not hasattr(socket, 'AF_BLUETOOTH'):
                err = 'this Python has no Bluetooth socket support: use Transport = COM port, or Bluetooth LE'
            else:
                for ch in self.channels:
                    if self._cancel:
                        break
                    s = None
                    try:
                        s = self.sock_factory() if self.sock_factory else socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
                        s.settimeout(self.connect_timeout)
                        s.connect((self.mac, ch))
                        s.settimeout(0)
                        self.sock, self.is_open, err = s, True, ''
                        break
                    except Exception as e:
                        err = rfcomm_friendly(e)
                        try:
                            if s: s.close()
                        except Exception:
                            pass
                        if 'denied' in err or 'not paired' in err:
                            break
        except Exception as e:
            err = rfcomm_friendly(e)
        self.last_error = '' if self.is_open else err
        self.opening = False

    def write(self, data):
        try:
            self.sock.send(data)
        except (BlockingIOError, InterruptedError):
            self.dropped += 1
        except Exception as e:
            self.last_error = rfcomm_friendly(e)
            raise IOError(self.last_error)

    def read(self):
        try:
            d = self.sock.recv(4096)
        except (BlockingIOError, InterruptedError):
            return b''
        except Exception as e:
            self.last_error = rfcomm_friendly(e)
            raise IOError(self.last_error)
        if d == b'':
            self.last_error = 'the robot closed the Bluetooth connection'
            raise IOError(self.last_error)
        return d

    def close(self):
        self._cancel = True
        try:
            if self.sock: self.sock.close()
        except Exception:
            pass
        self.sock, self.is_open = None, False


def rfcomm_friendly(e):
    s = str(e).strip() or e.__class__.__name__
    low = s.lower()
    if '10060' in low or 'timed out' in low or 'timeout' in low:
        return 'no answer from the robot over Bluetooth: switch it on, bring it close, and pair it first (%s)' % s[:50]
    if '10061' in low or 'refused' in low or '10064' in low or '10065' in low:
        return 'the robot refused the connection: wrong channel, or something else is connected to it (%s)' % s[:50]
    if '10013' in low or 'permission' in low or 'denied' in low:
        return 'Windows denied the Bluetooth connection: pair the robot first (%s)' % s[:50]
    if 'invalid' in low and 'address' in low:
        return 'that is not a Bluetooth address: it looks like AA:BB:CC:DD:EE:FF'
    return friendly_error(e)


def valid_mac(text):
    return bool(re.fullmatch(r'([0-9A-Fa-f]{2}[:\-]){5}[0-9A-Fa-f]{2}', (text or '').strip()))
