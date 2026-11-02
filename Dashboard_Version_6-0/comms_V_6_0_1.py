"""comms_V_6_0_1.py - the link between the dashboard and the ESP32: USB cable (COM port) or Bluetooth LE. Needs pyserial and bleak (see requirements).

Frames:   $TYPE,f1,f2*HH\\n   (HH = XOR of everything between $ and *, in hex).  Plain text lines without $ are shown as they are.
Laptop -> robot   CMD,seq,vl,vr,grip,lift,enable   (every 50 ms, it is also the heartbeat)     PING,n     HELLO?     STOP
Robot -> laptop   HELLO,proto,fw,robot,version     TEL,seq,ack,ms,yaw,gz,vl,vr,grip,lift,vbat_mv,flags     PONG,n     ACK,...
Safety: a new or lost link is always DISARMED; while disarmed the robot is sent zeros; the robot's own failsafe stops it after 400 ms of silence."""
import math, sys, threading, time
from collections import deque, namedtuple

TX_HZ = 20.0                  # CMD rate
PING_EVERY = 1.0
STALE_AFTER = 0.6             # connected but nothing heard for this long -> STALE
HELLO_EVERY = 0.5
BAUD = 115200
F_ARMED, F_FAILSAFE, F_GYRO_OK, F_GYRO_CAL, F_LOWBAT = 1, 2, 4, 8, 16
NUS_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e'     # Nordic UART service, what the firmware offers in BLE mode
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'          # laptop -> robot (write)
NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'          # robot -> laptop (notify)
ROBOT_NAME_PREFIX = 'REC'                                # the firmware advertises as "REC-Robot"


def install_hint(module): return '"%s" -m pip install %s' % (sys.executable, module)


# ---------------------------------------------------------------- frames
def checksum(body):
    c = 0
    for ch in body.encode('ascii', 'replace'):
        c ^= ch
    return c


def _fmt(v):
    if isinstance(v, float):
        return ('%.2f' % v).rstrip('0').rstrip('.') if v != int(v) else str(int(v))
    return str(v)


def encode(ftype, *fields):
    body = ','.join([ftype] + [_fmt(f) for f in fields])
    return ('$%s*%02X\n' % (body, checksum(body))).encode('ascii')


class FrameParser:
    """Feed it bytes as they arrive. Returns events: ('frame', type, [fields]) | ('bad', text) | ('text', line)."""

    def __init__(self, maxline=240):
        self.buf, self.maxline, self.bad = bytearray(), maxline, 0

    def feed(self, data):
        out = []
        for b in data:
            if b == 10:
                out += self._line(bytes(self.buf).decode('ascii', 'replace').rstrip('\r'))
                self.buf.clear()
            elif len(self.buf) < self.maxline:
                self.buf.append(b)
            else:
                self.buf.clear(); self.bad += 1
                out.append(('bad', 'line too long'))
        return out

    def _line(self, s):
        if not s:
            return []
        if s[0] != '$':
            return [('text', s)]
        body, _, hh = s[1:].rpartition('*')
        try:
            ok = '*' in s and int(hh[:2], 16) == checksum(body)
        except ValueError:
            ok = False
        if not ok:
            self.bad += 1
            return [('bad', s)]
        parts = body.split(',')
        return [('frame', parts[0], parts[1:])]


class Command:
    """What the laptop wants the robot to do: wheels in percent (-100..100), pad 0..100 (0 = open), box lift -100..100 (up positive)."""

    def __init__(self, vl=0, vr=0, grip=0, lift=0, enable=False):
        self.vl, self.vr, self.grip, self.lift, self.enable = vl, vr, grip, lift, enable

    def clamped(self):
        c = lambda v, lo, hi: int(max(lo, min(hi, round(v))))
        return Command(c(self.vl, -100, 100), c(self.vr, -100, 100), c(self.grip, 0, 100), c(self.lift, -100, 100), bool(self.enable))

    def zero(self): return Command(0, 0, 0, 0, False)


def friendly_error(e):
    """The cryptic text pyserial / bleak / Windows give, as one sentence the person can act on."""
    s = str(e).strip() or e.__class__.__name__
    low = s.lower()
    if 'permission' in low or 'access is denied' in low or 'errno 13' in low or 'resource busy' in low:
        return 'port busy or access denied: close the Arduino Serial Monitor or any other program using it (%s)' % s[:60]
    if 'filenotfound' in low or 'no such file' in low or 'errno 2' in low or 'cannot find the file' in low or 'winerror 2' in low:
        return 'port not found: re-plug the USB cable, then press Refresh (%s)' % s[:60]
    if 'unplug' in low or 'device not configured' in low or 'input/output error' in low or 'errno 5' in low or 'winerror 22' in low:
        return 'the port went away (cable unplugged): %s' % s[:60]
    if 'turned off' in low or 'bluetooth is off' in low or 'radio' in low or 'org.bluez' in low:
        return 'Bluetooth is off or missing on this PC: switch it on in Windows settings (%s)' % s[:70]
    if 'timeout' in low or 'timed out' in low:
        return 'the robot did not answer in time: switch it on, bring it close, and make sure nothing else is connected to it'
    if 'not found' in low or 'could not find' in low:
        return 'device not found: press Scan again (the robot must be on and advertising)'
    return s


# ---------------------------------------------------------------- serial (USB cable)
Port = namedtuple('Port', 'device description')


def list_serial_ports():
    """COM ports, USB first. The Windows Bluetooth 'Incoming' ports (address 000000000000) are left out: opening them can hang."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    out = []
    for p in list_ports.comports():
        if str(getattr(p, 'hwid', '')).upper().endswith('000000000000'):
            continue
        out.append(Port(p.device, getattr(p, 'description', '') or ''))
    return sorted(out, key=lambda p: ('BLUETOOTH' in p.description.upper(), p.device))


class Transport:
    name, handshake_timeout = 'transport', 2.5

    def __init__(self):
        self.is_open, self.last_error, self.opening = False, '', False

    def start_open(self): raise NotImplementedError
    def close(self): self.is_open = False
    def write(self, data): raise NotImplementedError
    def read(self): return b''


class SerialTransport(Transport):
    """A COM port. Opens in a background thread and never touches DTR / RTS (that would reset the ESP32)."""

    def __init__(self, port, baud=BAUD):
        super().__init__()
        self.port, self.baud, self.ser = port, baud, None
        self.name = '%s @ %d' % (port, baud)
        self._cancel, self._lock = False, threading.Lock()

    def start_open(self):
        self.opening, self._cancel, self.last_error = True, False, ''
        threading.Thread(target=self._worker, daemon=True).start()

    def _worker(self):
        err = ''
        try:
            import serial
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout, ser.write_timeout = self.port, self.baud, 0, 0.3
            ser.dtr, ser.rts = False, False
            ser.open()
            with self._lock:
                if self._cancel: ser.close()
                else: self.ser, self.is_open = ser, True
        except ImportError:
            err = 'pyserial is not installed for this Python: run  ' + install_hint('pyserial')
        except Exception as e:
            err = friendly_error(e)
        self.last_error = '' if self.is_open else err
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
            if type(e).__name__ == 'SerialTimeoutException':      # buffer full: drop this one, the next CMD is 50 ms away
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


# ---------------------------------------------------------------- Bluetooth LE
BleDevice = namedtuple('BleDevice', 'name address rssi nus')


def bleak_missing_text(e=None): return 'bleak is not installed for this Python: run  ' + install_hint('bleak')


class BleScanner:
    """Scans for Bluetooth LE devices in a background thread. .devices: the REC robot first, then by signal strength."""

    def __init__(self, timeout=5.0):
        self.timeout, self.running, self.devices, self.error, self.started = timeout, False, [], '', 0.0

    def start(self):
        self.running, self.error, self.started = True, '', time.monotonic()
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
            found = asyncio.run(BleakScanner.discover(timeout=self.timeout, return_adv=True))
            devs = []
            for key, val in (found.items() if isinstance(found, dict) else []):
                dev, adv = val if isinstance(val, tuple) else (val, None)
                uuids = [u.lower() for u in (getattr(adv, 'service_uuids', None) or [])]
                name = getattr(adv, 'local_name', None) or getattr(dev, 'name', None) or ''
                devs.append(BleDevice(name, getattr(dev, 'address', str(key)), getattr(adv, 'rssi', None) if adv else None, NUS_SERVICE in uuids))
            devs.sort(key=lambda d: (not (d.nus or d.name.upper().startswith(ROBOT_NAME_PREFIX)), -(d.rssi if d.rssi is not None else -200)))
            self.devices = [d for d in devs if d.name or d.nus]          # nameless beacons are noise
        except Exception as e:
            self.error = friendly_error(e)
        finally:
            self.running = False

    def robot(self):
        for d in self.devices:
            if d.nus or d.name.upper().startswith(ROBOT_NAME_PREFIX):
                return d
        return None


class BleTransport(Transport):
    """Bluetooth LE to the robot's Nordic-UART service. All bleak calls run in one thread with its own asyncio loop. Writes never block the
    dashboard; if the radio falls behind the newest command is dropped (the next one is 50 ms away). One failed write does NOT end the link:
    only Windows reporting the device gone, or 20 failures in a row, does."""
    handshake_timeout = 8.0

    def __init__(self, address, label='', client_factory=None):
        super().__init__()
        self.address, self.name, self.client_factory = address, 'BLE %s' % (label or address), client_factory
        self._rx, self._lock = deque(), threading.Lock()
        self._loop = self._stop = self._client = None
        self._pending, self.chunk, self._lost, self._cancel = 0, 20, False, False
        self._write_fails, self.last_write_error = 0, ''

    def start_open(self):
        self.opening, self._cancel, self.last_error, self._lost = True, False, '', False
        threading.Thread(target=self._thread, daemon=True).start()

    def _thread(self):
        import asyncio
        loop = asyncio.new_event_loop()
        self._loop = loop
        try:
            loop.run_until_complete(self._session())
        except Exception as e:
            self.last_error = friendly_error(e)
            self._lost = self.is_open
        finally:
            self.is_open = self.opening = False
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
                self.last_error, self.opening = bleak_missing_text(e), False
                return
            factory = BleakClient
        client = self._client = factory(self.address, disconnected_callback=self._on_disconnect)
        last = ''
        for _ in range(2):
            if self._cancel:
                self.opening = False
                return
            try:
                await client.connect(timeout=10.0)
                last = ''
                break
            except Exception as e:
                last = friendly_error(e)
        if last:
            self.last_error, self.opening = last, False
            return
        try:
            have = {str(getattr(c, 'uuid', '')).lower() for s in client.services for c in getattr(s, 'characteristics', [])}
            if NUS_RX not in have or NUS_TX not in have:
                self.last_error, self.opening = 'connected, but this device is not the REC robot (no serial service): is the BLE firmware flashed?', False
                await client.disconnect()
                return
            await client.start_notify(NUS_TX, self._on_notify)
            self.chunk = max(20, min(180, int(getattr(client, 'mtu_size', 23) or 23) - 3))
        except Exception as e:
            self.last_error, self.opening = friendly_error(e), False
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
            self.last_error = ('Bluetooth LE link was dropped by Windows or the robot (out of range, robot reset, or radio interference)'
                               + ((' after write errors: ' + self.last_write_error) if self.last_write_error else ''))
            self._lost = True

    async def _awrite(self, data):
        try:
            for i in range(0, len(data), self.chunk):
                await self._client.write_gatt_char(NUS_RX, data[i:i + self.chunk], response=False)
            self._write_fails = 0
        except Exception as e:
            self._write_fails += 1
            self.last_write_error = '%s: %s' % (e.__class__.__name__, (str(e) or '-')[:80])
            gone = getattr(self._client, 'is_connected', True) is False
            if gone or self._write_fails >= 20:
                self.last_error = 'Bluetooth LE device is gone (%s)' % self.last_write_error if gone else friendly_error(e)
                self._lost = True
        finally:
            self._pending -= 1

    def write(self, data):
        if not self.is_open or self._loop is None:
            raise IOError(self.last_error or 'Bluetooth LE link is not open')
        if self._lost:
            raise IOError(self.last_error)
        if self._pending >= 4:                                   # the radio is behind: drop this one
            return
        import asyncio
        self._pending += 1
        asyncio.run_coroutine_threadsafe(self._awrite(bytes(data)), self._loop)

    def read(self):
        if self.is_open and not self._cancel and not self._lost and getattr(self._client, 'is_connected', True) is False:
            self._lost, self.last_error = True, 'Bluetooth LE link lost (Windows reports the device disconnected)'
        if self._lost:
            raise IOError(self.last_error or 'Bluetooth LE link lost')
        with self._lock:
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


# ---------------------------------------------------------------- the connection
class Link:
    """Call update() every frame. Everything the panel shows is in .state .stats .telemetry .hello .log .last_error."""
    DISCONNECTED, CONNECTING, HANDSHAKE, CONNECTED, STALE, NO_REPLY, ERROR = \
        'DISCONNECTED', 'CONNECTING', 'HANDSHAKE', 'CONNECTED', 'STALE', 'NO REPLY', 'ERROR'

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.transport, self._factory = None, None
        self.parser = FrameParser()
        self.state, self.armed, self.cmd = self.DISCONNECTED, False, Command()
        self.log = deque(maxlen=200)                    # (time, 'tx' | 'rx' | 'sys' | 'err', text)
        self.last_error = ''
        self._reset_stats()

    def _reset_stats(self):
        self.stats = dict(tx=0, rx=0, bad=0, tel=0, lost=0, rtt=None, rtt_avg=None, rtt_max=0.0, rtt_min=None,
                          tx_rate=0.0, rx_rate=0.0, tel_rate=0.0, since_rx=None, connected_at=None)
        self._seq, self._tel_seq_last = 0, None
        self._sent, self._ping_sent, self._rtts = {}, {}, deque(maxlen=40)
        self._rate_t, self._rate_n = self.clock(), (0, 0, 0)
        self._last_rx, self._next_tx, self._next_ping, self._next_hello = None, 0.0, 0.0, 0.0
        self._hello_deadline = None
        self.telemetry, self.hello, self.heard = {}, {}, False

    def say(self, kind, text): self.log.append((self.clock(), kind, text))

    @property
    def connected(self): return self.state in (self.CONNECTED, self.STALE, self.NO_REPLY)

    @property
    def active(self): return self.state not in (self.DISCONNECTED, self.ERROR)

    # ---- connect / disconnect
    def connect(self, factory):
        """factory() returns a fresh Transport."""
        self.disconnect(quiet=True)
        self._factory = factory
        self.armed = False                               # a new connection always starts disarmed
        self.state = self.CONNECTING
        self._reset_stats()
        self.parser = FrameParser()
        try:
            self.transport = factory()
            self.transport.start_open()
        except Exception as e:
            self._fail('open failed: %s' % friendly_error(e))
            return
        self.say('sys', 'opening %s ...' % self.transport.name)

    def _finish_open(self):
        t = self.transport
        if t.opening:
            return
        if not t.is_open:
            self._fail('open failed: %s' % (t.last_error or 'unknown error'))
            return
        now = self.clock()
        self.stats['connected_at'] = now
        self.say('sys', 'link open')
        self.state = self.HANDSHAKE
        self._hello_deadline = now + t.handshake_timeout
        self._write(encode('HELLO?'), 'HELLO?')
        self._next_hello = now + HELLO_EVERY

    def disconnect(self, quiet=False):
        if self.transport:
            try:
                if self.transport.is_open: self.transport.write(encode('STOP'))
            except Exception:
                pass
            self.transport.close()
        was = self.state != self.DISCONNECTED
        self.transport, self.state, self.armed = None, self.DISCONNECTED, False
        if was and not quiet:
            self.say('sys', 'disconnected')

    def _fail(self, msg):
        self.last_error = msg
        self.say('err', msg)
        if self.armed:
            self.say('sys', 'output disarmed because the link failed')
        self.armed, self.state = False, self.ERROR
        if self.transport:
            try: self.transport.close()
            except Exception: pass

    # ---- send
    def _write(self, data, shown=None):
        try:
            self.transport.write(data)
        except Exception as e:
            self._fail('write failed: %s' % friendly_error(e))
            return False
        self.stats['tx'] += 1
        if shown is not None:
            self.say('tx', shown)
        return True

    def send_raw(self, text):
        """Terminal input: the text plus a newline, exactly as typed."""
        if self.transport and self.transport.is_open:
            self._write(text.encode('ascii', 'replace') + b'\n', text)

    def set_command(self, cmd): self.cmd = cmd

    def estop(self):
        self.armed = False
        if self.transport and self.transport.is_open:
            self._write(encode('STOP'), 'STOP')
        self.say('sys', 'E-STOP: output disarmed')

    # ---- every frame
    def update(self):
        now = self.clock()
        if self.state == self.CONNECTING and self.transport:
            self._finish_open()
        if not self.transport or self.state in (self.DISCONNECTED, self.ERROR, self.CONNECTING):
            return
        try:
            data = self.transport.read()
        except Exception as e:
            self._fail('link lost: %s' % friendly_error(e))
            return
        if data:
            self._last_rx = now
            for ev in self.parser.feed(data):
                self._event(ev, now)
        self._timers(now)

    def _heard_robot(self):
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
        if self.state == self.HANDSHAKE:
            if now >= self._next_hello:
                self._next_hello = now + HELLO_EVERY
                self._write(encode('HELLO?'))
            if self._hello_deadline is not None and now > self._hello_deadline:
                self.state, self._next_hello = self.NO_REPLY, now + 1.0
                self.say('err', 'link is open but the robot is silent: is it powered and running the V_5_0 firmware?')
        elif self.state == self.NO_REPLY and now >= self._next_hello:
            self._next_hello = now + 1.0
            self._write(encode('HELLO?'))
        elif self.state in (self.CONNECTED, self.STALE) and not self.hello and now >= self._next_hello:
            self._next_hello = now + 2.0                  # telemetry arrived but the banner did not: ask again so the version shows
            self._write(encode('HELLO?'))
        s['since_rx'] = None if self._last_rx is None else now - self._last_rx
        if self.state == self.CONNECTED and s['since_rx'] is not None and s['since_rx'] > STALE_AFTER and self.heard:
            self.state = self.STALE
            self.say('err', 'link stale: nothing heard for %.1f s' % s['since_rx'])
        if self.state in (self.CONNECTED, self.STALE, self.HANDSHAKE, self.NO_REPLY):
            if now >= self._next_tx:
                self._next_tx = max(self._next_tx + 1.0 / TX_HZ, now - 0.1)
                self._send_cmd(now)
            if now >= self._next_ping:
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
        cmd = self.cmd.clamped() if self.armed else self.cmd.zero()
        cmd.enable = self.armed
        self._seq = (self._seq + 1) % 65536
        self._sent[self._seq] = now
        if len(self._sent) > 60: self._sent.pop(next(iter(self._sent)))
        self._write(encode('CMD', self._seq, cmd.vl, cmd.vr, cmd.grip, cmd.lift, 1 if cmd.enable else 0))

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
        rtt = self.stats['rtt_avg']
        if rtt is None: return 2
        q = 4 - (rtt > 60) - (rtt > 150) - (self.loss_percent() > 2) - (self.loss_percent() > 10)
        return max(1, q)


# ---------------------------------------------------------------- telemetry -> the picture
class TelemetryPose:
    """Turns telemetry into the picture data of sim.Robot.snapshot(). Position is dead reckoning (wheel percent x 30 cm/s, heading from the
    gyro): fine for watching, not for navigation."""
    MAX_CM_S = 30.0

    def __init__(self):
        self.x = self.y = self.al = self.ar = self.yaw0 = 0.0
        self.t = None

    def zero(self, tel): self.yaw0 = tel.get('yaw', 0.0)

    def reset(self): self.x = self.y = 0.0

    def snapshot(self, tel, at=None):
        th = math.radians(tel['yaw'])
        if self.t is not None and at is not None:
            dt = max(0.0, min(0.2, at - self.t))
            v = (tel['vl'] + tel['vr']) / 200.0 * self.MAX_CM_S
            self.x += -math.sin(th) * v * dt
            self.y += math.cos(th) * v * dt
            self.al += tel['vl'] / 100.0 * self.MAX_CM_S * dt / 3.0
            self.ar += tel['vr'] / 100.0 * self.MAX_CM_S * dt / 3.0
        self.t = at
        return dict(x=self.x, y=self.y, th=th, ref_th=math.radians(self.yaw0), heading=(-(tel['yaw'] - self.yaw0)) % 360,
                    vl=tel['vl'] / 100.0, vr=tel['vr'] / 100.0, al=self.al, ar=self.ar, speed=(tel['vl'] + tel['vr']) / 200.0 * self.MAX_CM_S,
                    omega=math.radians(tel['gz']), lift=0.6 + 11.4 * tel['lift'] / 100.0, pad=3.0 * tel['grip'] / 100.0, cmd=tel['grip'] / 100.0,
                    holding=False, touching=False, turn_on=False, snapping=False, mode='arcade', rt=0.0, speed_scale=1.0, source='robot')
