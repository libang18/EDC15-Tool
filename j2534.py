"""Windows SAE J2534-1 04.04 transport for the OpenPort K-line."""

import ctypes as C
import os
from pathlib import Path
import struct
import time
from contextlib import suppress
from dataclasses import dataclass

if os.name == 'nt':
    import winreg


U32 = C.c_uint32
PTR = C.c_void_p
ISO9141 = 3
ISO14230 = 4
RAW_K_LINE = 0x1200
DATA_RATE, LOOPBACK, P1_MAX, P3_MIN, P4_MIN = 1, 3, 7, 10, 12
GET_CONFIG, SET_CONFIG, READ_VBATT, FIVE_BAUD_INIT, FAST_INIT = 1, 2, 3, 4, 5
CLEAR_RX_BUFFER = 8
ERR_TIMEOUT, ERR_BUFFER_EMPTY, ERR_DEVICE_IN_USE = 9, 16, 14
RX_INDICATIONS = 0x0F
BITS = struct.calcsize('P') * 8


class DriverError(RuntimeError):
    def __init__(self, operation, code, detail):
        self.code = code
        super().__init__(f'{operation}: J2534 0x{code:02X} — {detail}')


@dataclass(frozen=True)
class Driver:
    name: str
    path: Path


def registered_drivers():
    if os.name != 'nt':
        return []
    view = winreg.KEY_WOW64_64KEY if BITS == 64 else winreg.KEY_WOW64_32KEY
    drivers = []
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r'SOFTWARE\PassThruSupport.04.04',
                             access=winreg.KEY_READ | view)
    except OSError:
        return []
    with key:
        for index in range(winreg.QueryInfoKey(key)[0]):
            with suppress(OSError):
                name = winreg.EnumKey(key, index)
                with winreg.OpenKey(key, name) as entry:
                    library = Path(os.path.expandvars(winreg.QueryValueEx(entry, 'FunctionLibrary')[0]))
                    if 'openport' in name.lower() and library.is_file():
                        drivers.append(Driver(name, library))
    return sorted(drivers, key=lambda driver: ('winusb' not in driver.name.lower(), driver.name))


class Message(C.Structure):
    _fields_ = [(name, U32) for name in ('protocol', 'status', 'flags', 'stamp', 'size', 'extra')]
    _fields_ += [('data', C.c_ubyte * 4128)]

    def __init__(self, protocol=0, data=b''):
        super().__init__()
        if len(data) > len(self.data):
            raise ValueError('Frame is longer than the J2534 buffer.')
        self.protocol = protocol
        self.size = self.extra = len(data)
        self.data[:len(data)] = data

    @property
    def payload(self):
        if self.size > len(self.data):
            raise ValueError(f'Driver returned an invalid frame length {self.size}.')
        end = self.extra if 0 < self.extra <= self.size else self.size
        return bytes(self.data[:end])


class Config(C.Structure):
    _fields_ = [('parameter', U32), ('value', U32)]


class ConfigList(C.Structure):
    _fields_ = [('size', U32), ('items', C.POINTER(Config))]


class ByteArray(C.Structure):
    _fields_ = [('size', U32), ('data', C.POINTER(C.c_ubyte))]


class OpenPort:
    def __init__(self, path, trace=None):
        self.path = Path(path).resolve()
        self.trace = trace or (lambda *_: None)
        self.device = None
        self.channel = None
        self.protocol = None

    def __enter__(self):
        if os.name != 'nt':
            raise RuntimeError('The J2534 adapter requires Windows.')
        try:
            self.dll = C.WinDLL(str(self.path), winmode=0x1100)
        except OSError as error:
            raise RuntimeError(f'Cannot load {self.path} in {BITS}-bit Python: {error}') from error
        for name, args in {
            'Open': (PTR, PTR), 'Close': (U32,), 'ReadVersion': (U32, PTR, PTR, PTR),
            'Connect': (U32, U32, U32, U32, PTR), 'Disconnect': (U32,),
            'Ioctl': (U32, U32, PTR, PTR), 'StartMsgFilter': (U32, U32, PTR, PTR, PTR, PTR),
            'ReadMsgs': (U32, PTR, PTR, U32), 'WriteMsgs': (U32, PTR, PTR, U32),
            'GetLastError': (PTR,),
        }.items():
            function = getattr(self.dll, f'PassThru{name}')
            function.argtypes = args
            function.restype = C.c_int32
        device = U32()
        self.call('Open', None, C.byref(device))
        self.device = device.value
        return self

    def __exit__(self, *_):
        with suppress(DriverError):
            self.disconnect()
        with suppress(DriverError):
            self.call('Close', self.device)
        self.device = None

    def call(self, name, *args, allowed=()):
        result = getattr(self.dll, f'PassThru{name}')(*args)
        if result and result not in allowed:
            message = C.create_string_buffer(256)
            self.dll.PassThruGetLastError(message)
            detail = message.value.decode('mbcs', errors='replace') or 'No driver description'
            raise DriverError(name, result, detail)
        return result

    def info(self):
        details = [C.create_string_buffer(256) for _ in range(3)]
        self.call('ReadVersion', self.device, *details)
        return tuple(part.value.decode('mbcs', errors='replace') for part in details)

    def voltage(self):
        value = U32()
        self.call('Ioctl', self.device, READ_VBATT, None, C.byref(value))
        return value.value / 1000

    def disconnect(self):
        if self.channel is not None:
            self.call('Disconnect', self.channel)
            self.channel = None

    def connect(self, protocol, baud, p1, p3, p4):
        self.disconnect()
        channel = U32()
        self.call('Connect', self.device, protocol, RAW_K_LINE, baud, C.byref(channel))
        self.channel, self.protocol = channel.value, protocol
        # J2534 04.04 uses 0.5 ms units for P1, P3 and P4.
        self.configure({LOOPBACK: 0, P1_MAX: p1 * 2, P3_MIN: p3 * 2, P4_MIN: p4 * 2})
        actual = self.configure({DATA_RATE: 0}, read=True)[DATA_RATE]
        if actual != baud:
            raise RuntimeError(f'Driver set {actual} Bd instead of the requested {baud} Bd.')
        mask, pattern, filter_id = Message(protocol, b'\x00'), Message(protocol, b'\x00'), U32()
        self.call('StartMsgFilter', self.channel, 1, C.byref(mask), C.byref(pattern),
                  None, C.byref(filter_id))
        self.clear()
        self.trace(f'K-line channel {protocol}, {actual} Bd, P1/P3/P4 = {p1}/{p3}/{p4} ms')

    def configure(self, values, read=False):
        items = (Config * len(values))(*(Config(key, value) for key, value in values.items()))
        config = ConfigList(len(items), items)
        self.call('Ioctl', self.channel, GET_CONFIG if read else SET_CONFIG, C.byref(config), None)
        return {item.parameter: item.value for item in items}

    def clear(self):
        self.call('Ioctl', self.channel, CLEAR_RX_BUFFER, None, None)

    def fast_init(self, wire):
        request, reply = Message(self.protocol, wire), Message()
        self.trace('FAST_INIT TX', wire)
        result = self.call('Ioctl', self.channel, FAST_INIT, C.byref(request), C.byref(reply),
                           allowed=(7, ERR_TIMEOUT, ERR_BUFFER_EMPTY))
        if reply.size:
            self.trace('FAST_INIT RX', reply.payload)
            return reply.payload
        if result:
            self.trace(f'FAST_INIT has no immediate reply (0x{result:02X})')
        return b''

    def slow_init(self, address):
        address_buffer = (C.c_ubyte * 1)(address)
        key_buffer = (C.c_ubyte * 2)()
        request, reply = ByteArray(1, address_buffer), ByteArray(2, key_buffer)
        result = self.call('Ioctl', self.channel, FIVE_BAUD_INIT, C.byref(request), C.byref(reply),
                           allowed=(7, ERR_TIMEOUT, ERR_BUFFER_EMPTY))
        if result:
            return b''
        if not 1 <= reply.size <= len(key_buffer):
            raise RuntimeError(f'5-baud init returned an unexpected number of key bytes: {reply.size}.')
        return bytes(key_buffer[:reply.size])

    def send(self, wire, timeout):
        message, count = Message(self.protocol, wire), U32(1)
        self.trace('TX', wire)
        self.call('WriteMsgs', self.channel, C.byref(message), C.byref(count), timeout)
        if count.value != 1:
            raise RuntimeError('Driver did not confirm the frame was sent.')

    def receive(self, timeout, *, echo=False):
        deadline = time.monotonic() + timeout / 1000
        message = Message()
        while time.monotonic() < deadline:
            count = U32(1)
            remaining = max(1, int((deadline - time.monotonic()) * 1000))
            self.call('ReadMsgs', self.channel, C.byref(message), C.byref(count), min(remaining, 50),
                      allowed=(ERR_TIMEOUT, ERR_BUFFER_EMPTY))
            expected_status = (message.status & RX_INDICATIONS) == (1 if echo else 0)
            if count.value and expected_status and message.size:
                data = message.payload
                self.trace('RX echo' if echo else 'RX', data)
                return data
            if not count.value:
                time.sleep(0.001)
        return b''
