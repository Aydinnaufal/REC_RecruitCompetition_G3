"""testshim/serial - a tiny stand-in for pyserial, used ONLY by test_V_5_0.py when the real pyserial is not installed.
It opens a pseudo-terminal path (the 'COM port' of the firmware test rig) the way pyserial would. It is not a serial driver."""
import os, select, struct, fcntl, termios, tty
__version__ = '0.0-testshim'


class SerialException(IOError): pass
class SerialTimeoutException(SerialException): pass


class Serial:
    def __init__(self, port=None, baudrate=9600, timeout=None, write_timeout=None, **kw):
        self.port, self.baudrate, self.timeout, self.write_timeout = port, baudrate, timeout, write_timeout
        self.dtr = self.rts = None
        self.fd = None
        if port is not None:
            self.open()

    def open(self):
        try:
            self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            tty.setraw(self.fd)
        except OSError as e:
            raise SerialException("could not open port %r: %s(%d, %r)" % (self.port, type(e).__name__, e.errno or 0, e.strerror))

    @property
    def is_open(self): return self.fd is not None

    @property
    def in_waiting(self):
        buf = fcntl.ioctl(self.fd, termios.FIONREAD, struct.pack('i', 0))
        return struct.unpack('i', buf)[0]

    def read(self, n=1):
        try:
            return os.read(self.fd, n)
        except BlockingIOError:
            return b''

    def write(self, data):
        try:
            return os.write(self.fd, data)
        except BlockingIOError:
            raise SerialTimeoutException('Write timeout')

    def close(self):
        if self.fd is not None:
            try: os.close(self.fd)
            except OSError: pass
        self.fd = None
