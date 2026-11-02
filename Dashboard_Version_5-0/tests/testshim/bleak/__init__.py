"""testshim/bleak - a stand-in for the bleak API surface that comms_V_5_0 uses (BleakScanner.discover, BleakClient). Used ONLY by test_V_5_0.py when the real
bleak is not installed. A fake client talks to comms.VirtualRobotTransport, so the real BleTransport + Link + parser run against a pretend robot.
Tests set:  BleakScanner.ADV (what a scan finds)  and  BleakClient.ROBOTS {address: robot-type or None (no UART service)}  and  BleakClient.FAIL (error text)."""
import asyncio, sys, time, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..')))
__version__ = '0.0-testshim'
NUS_RX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
NUS_TX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'


class BleakError(Exception): pass


class _Dev:
    def __init__(self, address, name): self.address, self.name = address, name


class _Adv:
    def __init__(self, name, rssi, uuids): self.local_name, self.rssi, self.service_uuids = name, rssi, uuids


class BleakScanner:
    ADV = []            # [(name, address, rssi, [uuids])]
    ERROR = None

    @classmethod
    async def discover(cls, timeout=5.0, return_adv=False, **kw):
        await asyncio.sleep(min(timeout, 0.1))
        if cls.ERROR: raise BleakError(cls.ERROR)
        return {a: (_Dev(a, n), _Adv(n, r, u)) for n, a, r, u in cls.ADV}


class _Char:
    def __init__(self, uuid): self.uuid = uuid


class _Svc:
    def __init__(self, chars): self.characteristics = [_Char(c) for c in chars]


class BleakClient:
    ROBOTS = {}         # address -> 'simple' | 'tower' | None (connects, but has no UART service)
    FAIL = None
    LOG = []

    def __init__(self, address, disconnected_callback=None, **kw):
        self.address, self._dcb = address, disconnected_callback
        self.mtu_size = 185
        self._notify, self._t, self._task, self.connected = None, None, None, False

    async def connect(self, timeout=10.0):
        await asyncio.sleep(0.05)
        if BleakClient.FAIL: raise BleakError(BleakClient.FAIL)
        if self.address not in BleakClient.ROBOTS: raise BleakError('Device with address %s was not found' % self.address)
        self.connected = True

    @property
    def services(self):
        return [_Svc([NUS_RX, NUS_TX] if BleakClient.ROBOTS.get(self.address) else ['00002a00-0000-1000-8000-00805f9b34fb'])]

    async def start_notify(self, uuid, cb):
        import comms_V_5_0 as cm
        self._notify = cb
        self._t = cm.VirtualRobotTransport(BleakClient.ROBOTS[self.address], time.monotonic, latency_ms=8, jitter_ms=2)
        self._t.open()
        self._task = asyncio.get_event_loop().create_task(self._pump())

    async def _pump(self):
        while self.connected:
            d = self._t.read()
            for i in range(0, len(d), 20):                       # a real link delivers 20 byte notifications at MTU 23
                self._notify(None, bytearray(d[i:i + 20]))
            await asyncio.sleep(0.01)

    async def write_gatt_char(self, uuid, data, response=True):
        if not self.connected: raise BleakError('not connected')
        BleakClient.LOG.append((uuid, bytes(data), response))
        self._t.write(bytes(data))

    def drop(self):
        """Test helper: the robot walks away."""
        self.connected = False
        if self._dcb: self._dcb(self)

    async def disconnect(self):
        self.connected = False
        if self._task:
            self._task.cancel()
            await asyncio.sleep(0)
