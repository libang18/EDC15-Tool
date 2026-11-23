"""VAG EDC15P/V flash and EEPROM access through RAM loaders."""

import hashlib
import os
import re
import statistics
import tempfile
import time
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from struct import pack_into, unpack_from

from j2534 import ISO9141, ISO14230, LOOPBACK, P1_MAX, DriverError


ROOT = Path(__file__).resolve().parent
REFERENCE = ROOT / 'reference' / 'ECU_Flasher_UNO_EDC15.ino'
# The flash RAM loader is part of the ecu-tool firmware source. That project has no licence, so the file
# is not redistributed here: it is fetched from the author's repository at this exact revision.
REFERENCE_URL = ('https://raw.githubusercontent.com/fjvva/ecu-tool/5fea32b7becdeac14d591069e85ed70859533d53/'
                 'Portable%20Firmware/HW%20V2/ECU_Flasher_UNO_EDC15/ECU_Flasher_UNO_EDC15.ino')
LOADER_HASH = 'c040d62df848f2f4731c2714fc5d71e344c585823142f9c10dcfbf23ca6836b0'
EEPROM_LOADER = ROOT / 'reference' / 'kw1281test' / 'Loader.bin'
EEPROM_LOADER_HASH = '4460abc8b7290d49093693ad53f4dbbc5186e4599d24e12befdf263ba2ce26e3'
EEPROM_SIZE = 512
FLASH_SIZE = 0x80000
BOOT_OFFSET = 0x7C000
BOSCH_SW_HEADER = 0x10040
BOSCH_SW_HEADER_SIZE = 32
ACK = b'\x76'
# An EDC15P can return this FAST_INIT response immediately after a KW1281 session.
KW1281_WAKE_REPLY = bytes.fromhex('83 06 01 7f 03 13 1f')
PART_NUMBER = re.compile(rb'0[0-9A-Z]{2} ?906 ?[0-9]{3} ?[A-Z]{0,2}(?![A-Z0-9])')
BOSCH_SW_ID = re.compile(rb'(?<![0-9])1037[0-9]{6}(?![0-9])')
VIN = re.compile(r'\b(?=[A-HJ-NPR-Z0-9]*[A-HJ-NPR-Z])[A-HJ-NPR-Z0-9]{17}\b')
IMMOBILIZER_ID = re.compile(r'\b(?:AUZ|SEZ|SKZ|VWZ)[A-Z0-9]{11}\b')
KEY_WORDS = {'EDC15P': (0xDA1C, 0xF781),
             'EDC15V': (0x508D, 0xA647)}


class ProtocolError(RuntimeError):
    pass


class ResponseTimeout(ProtocolError):
    pass


class WakeupTimeout(ResponseTimeout):
    pass


class Cancelled(RuntimeError):
    pass


@dataclass(frozen=True)
class Timing:
    init_p4: int = 10
    init_pause: int = 75
    init_timeout: int = 900
    fast_p1: int = 2
    fast_p4: int = 0
    loader_pause: int = 5
    response_timeout: int = 700
    erase_timeout: int = 12000
    read_block: int = 254

    def __post_init__(self):
        bounds = {'init_p4': (0, 20), 'init_pause': (30, 200), 'init_timeout': (300, 5000), 'fast_p1': (1, 20),
                  'fast_p4': (0, 2), 'loader_pause': (5, 100), 'response_timeout': (150, 5000),
                  'erase_timeout': (4000, 60000), 'read_block': (16, 254)}
        for name, (low, high) in bounds.items():
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f'{name} must be an integer in the range {low}–{high}.')


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def loader_blocks(path=REFERENCE):
    if not path.is_file():
        raise FileNotFoundError(f'The flash RAM loader source {path.name} is missing. Download it from the window '
                                'when asked, or run: python app.py --fetch-loader')
    source = path.read_text(encoding='utf-8')
    match = re.search(r'\bEDC15_Loader\[\]\s+PROGMEM\s*=\s*\{(.*?)\};', source, re.S)
    if match is None:
        raise ValueError('EDC15_Loader is missing from the reference file.')
    code = re.sub(r'//[^\n]*|/\*.*?\*/', '', match[1], flags=re.S)
    data = bytes(int(value, 16) for value in re.findall(r'\b0x[0-9a-fA-F]{2}\b', code))
    if sha256(data) != LOADER_HASH:
        raise ValueError('RAM loader SHA-256 does not match the reference ecu-tool revision.')
    return tuple(data[offset:offset + 257] for offset in range(0, len(data), 257))


def download_reference(path=REFERENCE, url=REFERENCE_URL):
    """Fetch the ecu-tool firmware source and keep it only if its RAM loader has the expected SHA-256."""
    with urllib.request.urlopen(url, timeout=30) as response:
        data = response.read()
    temporary = path.with_name(path.name + '.download')
    temporary.write_bytes(data)
    try:
        loader_blocks(temporary)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return path


def eeprom_loader(path=EEPROM_LOADER):
    """Patch the RAM end address and boot-ROM checksum as in kw1281test (MIT)."""
    raw = path.read_bytes()
    if sha256(raw) != EEPROM_LOADER_HASH:
        raise ValueError('EEPROM RAM loader SHA-256 does not match the reference kw1281test revision.')
    image = bytearray((len(raw) + 11) // 8 * 8)
    image[:len(raw)] = raw
    pack_into('<H', image, 0x0E, 0xE000 + len(image))
    checksum_offset = len(image) - 4
    high, low = 0xEFCD, 0x8631
    for offset in range(0, checksum_offset, 8):
        first, second = unpack_from('<HH', image, offset)
        low ^= first
        shift = high & 15
        low = ((low << shift) | (low >> (16 - shift))) & 0xFFFF
        carry = low & 1 if shift else 0
        high = ((high - second - carry) & 0xFFFF) ^ low
        if offset + 4 < checksum_offset:
            third, fourth = unpack_from('<HH', image, offset + 4)
            low = (low - third - 1 + 0xDAAD) & 0xFFFF
            high ^= fourth
            shift = low & 15
            high = ((high >> shift) | (high << (16 - shift))) & 0xFFFF
    pack_into('<HH', image, checksum_offset, (low - 0xAB85) & 0xFFFF, 0xDF9B ^ high)
    return bytes(image)


def parts(image):
    return {value.replace(b' ', b'').decode('ascii') for value in PART_NUMBER.findall(image)}


def bosch_software_id(data):
    numbers = set(BOSCH_SW_ID.findall(data))
    return numbers.pop().decode('ascii') if len(numbers) == 1 else None


def format_identification(records, extra_records=()):
    lines = []
    for data in records:
        if not data:
            continue
        if len(data) == 5 and data[0] == 0:
            coding = int.from_bytes(data[1:3], 'big')
            workshop = int.from_bytes(data[3:5], 'big') | ((coding & 1) << 16)
            lines.extend((f'Coding: {coding >> 1:05d}', f'WSC: {workshop:05d}'))
            continue
        # The first character of the VAG part number can advertise extra identification in bit 7.
        number_text = bytes((data[0] & 0x7F,)) + data[1:]
        part = PART_NUMBER.match(number_text)
        if part:
            number = part[0].replace(b' ', b'').decode('ascii')
            lines.append(f'Part number: {number}')
            data = number_text[part.end():].strip()
            if data and all(32 <= value < 127 for value in data):
                lines.append(f'Description: {data.decode("ascii")}')
        elif all(32 <= value < 127 for value in data):
            value = ' '.join(data.decode('ascii').split())
            if value:
                label = 'SW VAG' if value.startswith('SG ') else 'Other data'
                lines.append(f'{label}: {value}')
    extra = ''.join(chr(value) if 32 <= value < 127 else ' '
                    for record in extra_records for value in record)
    match = VIN.search(extra)
    if match:
        lines.append(f'VIN: {match[0]}')
        extra = VIN.sub(' ', extra, count=1)
    match = IMMOBILIZER_ID.search(extra)
    if match:
        lines.append(f'Immobilizer ID: {match[0]}')
        extra = IMMOBILIZER_ID.sub(' ', extra, count=1)
    extra = ' '.join(extra.split())
    if extra:
        lines.append(f'Extended identification: {extra}')
    if not lines:
        raise ProtocolError('The ECU did not provide readable identification.')
    return '\n'.join(lines)


def validate_image(image):
    if len(image) != FLASH_SIZE:
        raise ValueError(f'BIN must be exactly {FLASH_SIZE:,} bytes, got {len(image):,}.')
    if len(set(image[BOOT_OFFSET:])) < 8 or not parts(image):
        raise ValueError('BIN lacks a recognizable VAG part number or boot sector content.')


def check_compatibility(current, candidate, profile):
    validate_image(current)
    validate_image(candidate)
    matching = parts(current) & parts(candidate)
    if not matching:
        raise ValueError('The connected ECU and the selected BIN do not share the required VAG part number.')
    return ', '.join(sorted(matching))


def save_image(path, data, overwrite=False):
    path = Path(path)
    with tempfile.NamedTemporaryFile(dir=path.parent, prefix='.edc15-', delete=False) as output:
        temporary = Path(output.name)
        try:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        except BaseException:
            output.close()
            temporary.unlink()
            raise
    try:
        if overwrite:
            temporary.replace(path)
        elif os.name == 'nt':
            temporary.rename(path)
        else:
            os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def with_checksum(data):
    return data + bytes((sum(data) & 0xFF,))


def compact(payload):
    return with_checksum(bytes((len(payload),)) + payload)


def addressed(payload):
    return with_checksum(bytes((0x80 | len(payload), 1, 0xF1)) + payload)


def programming_key(seed, profile):
    low_xor, high_xor = KEY_WORDS[profile]
    high, low = int.from_bytes(seed[:2], 'big'), int.from_bytes(seed[2:], 'big')
    for _ in range(5):
        carry = high & 0x8000
        high = ((high << 1) | (low >> 15)) & 0xFFFF
        low = (low << 1) & 0xFFFF
        if carry:
            low = (low | 1) ^ low_xor
            high ^= high_xor
    return high.to_bytes(2, 'big') + low.to_bytes(2, 'big')


class Session:
    def __init__(self, port, profile, timing=Timing(), trace=print, cancelled=lambda: False):
        if profile not in KEY_WORDS:
            raise ValueError(f'Unknown ECU profile {profile}.')
        self.port = port
        self.profile = profile
        self.timing = timing
        self.trace = trace
        self.cancelled = cancelled
        self.buffer = bytearray()
        self.diagnostic_open = False
        self.loaded = False
        self.erase_sent = False
        self.eeprom_busy = False
        self.separate_wakeup = False

    def check_cancel(self):
        if not (self.erase_sent or self.eeprom_busy) and self.cancelled():
            raise Cancelled('Operation cancelled.')

    def frame(self, *, address=False, timeout=None, initial=b'', one_byte=False, wakeup=False):
        wait = self.timing.response_timeout if timeout is None else timeout
        deadline = time.monotonic() + wait / 1000
        if initial and address and len(initial) == (initial[0] & 0x3F) + 3:
            # Some drivers return the FAST_INIT reply without a checksum.
            initial = with_checksum(initial)
        self.buffer.extend(initial)
        while True:
            self.check_cancel()
            if self.buffer:
                header = self.buffer[0]
                if one_byte and header == 0x55:
                    del self.buffer[0]
                    return b'\x55'
                if address and not header & 0x80:
                    raise ProtocolError(f'Expected an addressed frame, received {self.buffer.hex(" ")}.')
                header_length = 3 if address else (1 if header else 2)
                if len(self.buffer) >= header_length:
                    length = (header & 0x3F) if address else (header or self.buffer[1])
                    total = header_length + length + 1
                    if len(self.buffer) >= total:
                        raw = bytes(self.buffer[:total])
                        del self.buffer[:total]
                        if sum(raw[:-1]) & 0xFF != raw[-1]:
                            raise ProtocolError(f'Bad frame checksum {raw[:12].hex(" ")}.')
                        if (address and raw[1:3] not in (b'\xf1\x01', b'\x00\x01')
                                and not (wakeup and raw == KW1281_WAKE_REPLY)):
                            raise ProtocolError(f'Unexpected KWP addresses: {raw[1:3].hex(" ")}.')
                        return raw[header_length:-1]
            remaining = int((deadline - time.monotonic()) * 1000)
            if remaining <= 0:
                partial = f' Partial frame: {self.buffer[:12].hex(" ")}.' if self.buffer else ''
                raise ResponseTimeout(f'The ECU did not complete its reply within {wait} ms.{partial}')
            self.buffer.extend(self.port.receive(remaining))

    def send(self, wire):
        self.check_cancel()
        self.port.send(wire, self.timing.response_timeout)

    @staticmethod
    def expect(reply, expected, stage, prefix=False):
        matched = reply.startswith(expected) if prefix else reply == expected
        if not matched:
            raise ProtocolError(f'{stage}: expected {expected.hex(" ")}, received {reply.hex(" ")}.')
        return reply

    def swap(self, request, expected, stage, address=False, prefix=False):
        self.send(addressed(request) if address else compact(request))
        try:
            reply = self.frame(address=address)
        except ResponseTimeout as error:
            raise ResponseTimeout(f'{stage}: {error}') from error
        return self.expect(reply, expected, stage, prefix=prefix)

    def wake(self):
        timing = self.timing
        other_p4 = 14 if timing.init_p4 == 10 else 10
        attempts = ((ISO14230, timing.init_p4, False), (ISO14230, other_p4, False), (ISO9141, 10, True))
        attempt_errors = []
        for protocol, p4, separate in attempts:
            self.check_cancel()
            if attempt_errors:
                self.pause(350)
            self.port.connect(protocol, 10400, 20, 0, p4)
            self.buffer.clear()
            self.separate_wakeup = separate
            name = 'ISO14230' if protocol == ISO14230 else 'ISO9141 RAW'
            mode = 'separate pulse and frame' if separate else 'pulse with frame'
            self.trace(f'Wake-up 0x01 · {name} · P4 {p4} ms · {mode}')
            try:
                reply = self.init_reply()
            except (ResponseTimeout, DriverError) as error:
                if isinstance(error, DriverError) and (not separate or error.code not in (1, 3, 5, 10, 15)):
                    raise
                self.trace(str(error))
                attempt_errors.append(f'{name}, P4 {p4} ms')
                continue
            self.trace(f'ECU replied to FAST_INIT: {reply.hex(" ")}')
            if reply not in (b'\x7f\x00\x13', KW1281_WAKE_REPLY[3:-1]):
                self.expect(reply, b'\xc1\x6b\x8f', 'Programming wake-up type')
            self.diagnostic_open = True
            return reply
        raise WakeupTimeout('The ECU did not reply to the programming FAST_INIT at 10400 Bd. '
                            f'Attempts: {"; ".join(attempt_errors)}.')

    def init_reply(self):
        self.check_cancel()
        wire = addressed(b'\x81')
        if self.separate_wakeup:
            self.port.fast_init(b'')
            self.port.send(wire, self.timing.init_timeout)
            return self.frame(address=True, timeout=self.timing.init_timeout, wakeup=True)
        return self.frame(address=True, initial=self.port.fast_init(wire), timeout=self.timing.init_timeout,
                          wakeup=True)

    def diagnose_line(self):
        """Check adapter echo and standard diagnostic wake-up after the programming wake-up failed."""
        self.check_cancel()
        self.port.connect(ISO9141, 10400, 20, 0, self.timing.init_p4)
        # KWP TesterPresent addressed to the diagnostic tester at 0xF0.
        wire = with_checksum(b'\x81\xf0\xf1\x3e')
        self.port.configure({LOOPBACK: 1})
        try:
            self.port.send(wire, self.timing.init_timeout)
            echo_ok = self.port.receive(self.timing.init_timeout, echo=True) == wire
        finally:
            self.port.configure({LOOPBACK: 0})
            self.port.clear()
        self.trace('OpenPort K-line echo: ' + ('matching frame' if echo_ok else 'frame did not come back identical'))
        for rate, address in ((9600, 0x01), (10400, 0x11)):
            self.check_cancel()
            self.pause(350)
            self.port.connect(ISO9141, rate, 1, 0, 0)
            try:
                keys = self.port.slow_init(address)
            except DriverError as error:
                if error.code not in (1, 3, 5, 15):
                    raise
                self.trace(f'Driver does not support 5-baud init: {error}')
                break
            self.trace(f'5-baud 0x{address:02X}, {rate} Bd: ' + (keys.hex(' ') if keys else 'no reply'))
            if keys:
                return echo_ok, f'ECU replied to 5-baud 0x{address:02X} at {rate} Bd (keys {keys.hex(" ")}).'
        return echo_ok, ''

    def pause(self, milliseconds=None):
        time.sleep((self.timing.init_pause if milliseconds is None else milliseconds) / 1000)

    def start_diagnostics(self):
        self.pause()
        self.send(addressed(b'\x10\x85'))
        reply = self.frame(address=True)
        if reply == b'\x7f\x10\x78':
            reply = self.frame(address=True)
        self.expect(reply, b'\x50\x85\x2f', 'Diagnostic session')

    def stop_diagnostics(self):
        self.pause()
        self.swap(b'\x82', b'\xc2', 'StopCommunication', address=True)
        self.diagnostic_open = False

    def probe(self):
        self.wake()
        self.start_diagnostics()
        self.stop_diagnostics()
        self.trace('Diagnostic session confirmed and closed.')

    def start_kw1281(self, address):
        self.check_cancel()
        self.buffer.clear()
        for baud in (9600, 10400):
            self.port.connect(ISO9141, baud, 1, 0, 0)
            keys = self.port.slow_init(address)
            if keys:
                if keys == b'\x6b\x8f':
                    message = f'Control module {address:02X} replied in KWP2000 mode.'
                    if address == 0x01:
                        message = ('The ECU replied in KWP2000 mode. Identification data of this unit is read over KW1281. '
                                   'After other operations finish, power-cycle the ECU and try identification again.')
                    raise ProtocolError(message)
                self.expect(keys, b'\x01\x8a', 'KW1281 keys')
                self.trace(f'KW1281 module {address:02X} · {baud} Bd')
                return
            self.check_cancel()
            self.pause(350)
        raise ResponseTimeout(f'No response to KW1281 wake-up from module {address:02X}.')

    def identify(self):
        self.start_kw1281(0x01)
        records = []
        extra_records = None
        try:
            for index in range(16):
                counter, title, data = self.kw_receive_block()
                next_counter = (counter + 1) & 0xFF
                if self.cancelled() or title == 0x09 or index == 15 or (extra_records is not None and title == 0x0A):
                    break
                request = 0x09
                if title == 0xF6:
                    if extra_records is None:
                        records.append(data)
                        if records[0] and records[0][0] & 0x80 and len(data) == 5 and data[0] == 0:
                            self.trace('The ECU offers extended KW1281 identification.')
                            extra_records = []
                            request = 0x00
                    else:
                        extra_records.append(data)
                self.kw_send_block(next_counter, request)
            self.kw_send_block(next_counter, 0x06)
            self.trace('KW1281 session closed.')
            if index == 15 and title not in (0x09, 0x0A):
                raise ProtocolError('The KW1281 identification exchange exceeded 16 blocks.')
        except ProtocolError as error:
            if extra_records is None:
                raise
            self.trace(f'Extended identification: {error}')
        self.check_cancel()
        return format_identification(records, extra_records or ())

    def read_immobilizer_status(self):
        # Ross-Tech's Immo III measuring block 023, field 4, from Instruments or Immobilizer.
        states = {4: 'Replacement cluster, adaptation needed', 5: 'Service locked, data programmed',
                  6: 'Adapted, normal operation', 7: 'Key adaptation in progress'}
        for address in (0x17, 0x25):
            try:
                data = self.read_kw_group(address, 23)
                if len(data) != 12:
                    raise ProtocolError('Immo III group 023 did not contain four measuring values.')
                format_id, a, b = data[9:12]
                # Keep the numeric conversions exact so a fractional value cannot look like a status.
                scaled = {8: a * b, 28: (b - a) * 10, 32: (b - 256 if b >= 128 else b) * 10,
                          54: ((a << 8) | b) * 10}.get(format_id)
                if scaled is None or scaled % 10:
                    raise ProtocolError('Unsupported encoding for the Immo III status field.')
                code = scaled // 10
                return f'{states.get(code, f"Unknown status code {code}")} · module {address:02X}'
            except ProtocolError as error:
                self.trace(f'Immo III status at module {address:02X}: {error}')
        raise ProtocolError('No supported Immo III status reply from instruments (17) or immobilizer (25).')

    def read_kw_group(self, address, group):
        self.start_kw1281(address)
        next_counter = None
        try:
            for _ in range(16):
                counter, title, _ = self.kw_receive_block()
                next_counter = (counter + 1) & 0xFF
                self.check_cancel()
                if title == 0x09:
                    break
                if title != 0xF6:
                    raise ProtocolError('Unexpected KW1281 block during controller identification.')
                self.kw_send_block(next_counter, 0x09)
                next_counter = None
            else:
                raise ProtocolError('KW1281 controller identification exceeded 16 blocks.')

            self.kw_send_block(next_counter, 0x29, bytes((group,)))
            next_counter = None
            counter, title, data = self.kw_receive_block()
            next_counter = (counter + 1) & 0xFF
            self.check_cancel()
            if title != 0xE7:
                raise ProtocolError(f'KW1281 group {group:03d} is unavailable (reply 0x{title:02X}).')
            return data
        finally:
            if next_counter is not None:
                try:
                    self.kw_send_block(next_counter, 0x06)
                except (ProtocolError, DriverError) as error:
                    self.trace(f'KW1281 session close: {error}')

    def kw_receive_block(self):
        length = self.kw_byte(max(1500, self.timing.init_timeout))
        if length < 3:
            raise ProtocolError(f'Invalid KW1281 block length: {length}.')
        block = bytearray((length,))
        for _ in range(length):
            self.port.send(bytes((block[-1] ^ 0xFF,)), 100)
            block.append(self.kw_byte(300))
        if block[-1] != 0x03:
            raise ProtocolError('KW1281 block has no valid end byte.')
        return block[1], block[2], bytes(block[3:-1])

    def kw_byte(self, timeout):
        if not self.buffer:
            self.buffer.extend(self.port.receive(timeout))
        if not self.buffer:
            raise ResponseTimeout('The control module stopped replying during the KW1281 session.')
        value = self.buffer[0]
        del self.buffer[0]
        return value

    def kw_send_block(self, counter, title, data=b''):
        for value in bytes((3 + len(data), counter, title)) + data:
            self.port.send(bytes((value,)), 100)
            expected = value ^ 0xFF
            reply = self.kw_byte(300)
            if reply != expected:
                raise ProtocolError(f'KW1281 did not acknowledge byte 0x{value:02X}, received 0x{reply:02X}.')
        self.port.send(b'\x03', 100)

    def open_programming_session(self):
        recovery = self.wake()[0] == 0xC1
        if recovery:
            self.pause()
            self.swap(b'\x10\x85\x87', b'\x50\x85\x87', 'Switch to 124800 Bd', address=True)
        else:
            self.start_diagnostics()
            self.stop_diagnostics()
            self.pause()
            self.buffer.clear()
            self.expect(self.init_reply(), b'\xc1\x6b\x8f', 'Second FAST_INIT')
            self.diagnostic_open = True
            self.pause()
            self.swap(b'\x10\x85\x87', b'\x50\x85\x87', 'Switch to 124800 Bd')
        self.diagnostic_open = False
        self.port.connect(ISO9141, 124800, self.timing.fast_p1, 0, self.timing.fast_p4)
        if recovery:
            self.pause()
            self.swap(b'\x83\x00', b'\xc3\x00\x00\x50\x03\xff\x00', 'ECU parameters', address=True)
            self.pause()
            self.swap(b'\x83\x03\x00\x50\x03\xff\x00', b'\xc3\x03', 'ECU parameters', address=True)
            self.pause()
        self.pause(25)
        seed = self.swap(b'\x27\x41\x00', b'\x67\x41', 'Seed request', prefix=True)
        if len(seed) != 6:
            raise ProtocolError(f'Expected a four-byte seed, received {seed.hex(" ")}.')
        key = programming_key(seed[2:], self.profile)
        self.swap(b'\x27\x42' + key, b'\x67\x42\x34', 'ECU authorization')

    def upload_loader(self, size, blocks):
        self.pause(25)
        request = bytes.fromhex('3440e00000') + size.to_bytes(3, 'big')
        self.swap(request, b'\x74\xff', 'RAM loader preparation')
        for number, block in enumerate(blocks, 1):
            self.pause(self.timing.loader_pause)
            self.send(with_checksum(block))
            self.expect(self.frame(), ACK, f'RAM loader, block {number}')
        self.pause(self.timing.loader_pause)

    def start(self, blocks):
        self.open_programming_session()
        self.upload_loader(0x420, blocks)
        self.swap(bytes.fromhex('36c3c389c545da630b'), ACK, 'RAM loader start')
        self.loaded = True
        self.pause(self.timing.loader_pause)
        self.configure_loader()
        self.trace('RAM loader is responding.')

    def start_eeprom(self, image):
        blocks = []
        for offset in range(0, len(image), 254):
            payload = b'\x36' + image[offset:offset + 254]
            length = bytes((len(payload),))
            header = b'\x00' + length if len(payload) > 63 else length
            blocks.append(header + payload)
        self.open_programming_session()
        self.upload_loader(len(image), blocks)
        self.swap(b'\x31\x02', b'\x7f\x31\x22', 'EEPROM RAM loader start')
        self.loaded = True
        self.pause(75)
        # With 0.5 ms P1, OpenPort delivers the raw EEPROM stream in short frames.
        self.port.configure({P1_MAX: 1})
        self.trace('EEPROM RAM loader is responding. K-line P1 = 0.5 ms.')

    def dump_eeprom(self):
        self.send(compact(b'\xa6'))
        ack = compact(ACK)
        total = EEPROM_SIZE + 2 * len(ack)
        deadline = time.monotonic() + max(2000, self.timing.response_timeout) / 1000
        # Drain each complete dump before handling cancel and closing the loader.
        while len(self.buffer) < total:
            remaining = int((deadline - time.monotonic()) * 1000)
            if remaining <= 0:
                raise ResponseTimeout(f'Incomplete EEPROM reply ({len(self.buffer)}/{total} B including acknowledgements).')
            self.buffer.extend(self.port.receive(remaining))
        response = bytes(self.buffer[:total])
        del self.buffer[:total]
        if response[:len(ack)] != ack or response[-len(ack):] != ack:
            raise ProtocolError('The EEPROM loader did not send valid start and end acknowledgements.')
        return response[len(ack):-len(ack)]

    def read_eeprom(self, progress):
        first = self.dump_eeprom()
        progress('Verifying EEPROM with a second read', 1, 2)
        second = self.dump_eeprom()
        self.check_cancel()
        if first != second:
            raise ProtocolError('The two EEPROM dumps differ.')
        self.trace(f'EEPROM: two identical dumps of {len(first)} B, SHA-256 {sha256(first)}.')
        progress('Verifying EEPROM with a second read', 2, 2)
        return first

    def write_eeprom_byte(self, address, value):
        # Loader command 0xA7/0xA8 writes one byte to the lower/upper 256 B block. After its ACK the
        # loader clocks the device address and then reads two raw bytes (word address, data) from
        # the UART, one at a time, so they are sent separately with a pause the loader can absorb.
        self.send(compact(b'\xa7' if address < 0x100 else b'\xa8'))
        self.expect(self.frame(), ACK, 'EEPROM write command')
        self.eeprom_busy = True
        for byte in (address & 0xFF, value):
            self.pause(5)
            self.port.send(bytes((byte,)), self.timing.response_timeout)
        self.expect(self.frame(), ACK, f'EEPROM write at 0x{address:03X}')
        self.eeprom_busy = False
        self.pause(12)  # 24C04 write cycle (max. 10 ms) before the next start condition

    def write_eeprom(self, image, progress):
        """Write only the bytes that differ, then compare a fresh dump with the image."""
        if len(image) != EEPROM_SIZE:
            raise ValueError(f'EEPROM image must be exactly {EEPROM_SIZE} bytes, got {len(image):,}.')
        # The dump also sets up the loader's EEPROM pins; the write commands rely on it.
        current = self.dump_eeprom()
        changes = [address for address in range(EEPROM_SIZE) if current[address] != image[address]]
        self.trace(f'EEPROM: {len(changes)} of {EEPROM_SIZE} bytes differ.')
        for done, address in enumerate(changes, 1):
            self.write_eeprom_byte(address, image[address])
            progress('Writing EEPROM', done, len(changes))
        actual = self.dump_eeprom()
        if actual != image:
            mismatch = next(index for index, (a, b) in enumerate(zip(actual, image)) if a != b)
            raise ProtocolError(f'EEPROM content after writing differs from the image at 0x{mismatch:03X}.')
        self.trace(f'EEPROM written and verified, SHA-256 {sha256(image)}.')
        return len(changes)

    def configure_loader(self):
        is_p = self.profile != 'EDC15V'
        window = bytes.fromhex('31020800000fbfff')
        self.swap(bytes.fromhex('31021000000fbfff') if is_p else window,
                  b'\x7f\x31\x22', 'Switch to RAM loader')
        if is_p:
            for address in (0x080000, 0x100000):
                self.pause(self.timing.loader_pause)
                response = self.swap(b'\x23' + address.to_bytes(3, 'big') + b'\x10',
                                     b'\x36', 'EDC15P addressing setup', prefix=True)
                if len(response) != 17:
                    raise ProtocolError('Unexpected reply length during EDC15P addressing setup.')
            self.pause(self.timing.loader_pause)
            self.swap(window, b'\x7f', 'Flash address range selection')

    def read_block(self, offset, size):
        if not 0 <= offset < offset + size <= FLASH_SIZE or size > 254:
            raise ValueError('Block range is outside the 512 KiB flash.')
        request = b'\x23' + (0x080000 + offset).to_bytes(3, 'big') + bytes((size,))
        try:
            self.send(compact(request))
            reply = self.frame()
            if len(reply) != size + 1 or reply[0] != 0x36:
                raise ProtocolError(f'Invalid reply: {len(reply)} B, {reply[:8].hex(" ")}.')
        except ProtocolError as error:
            raise type(error)(f'Read at 0x{offset:05X}: {error}') from error
        return reply[1:]

    def read_bosch_software(self):
        header = self.read_block(BOSCH_SW_HEADER, BOSCH_SW_HEADER_SIZE)
        return bosch_software_id(header)

    def test_read(self):
        offset = 0x72000
        length = self.timing.read_block
        samples = []
        halves = length // 2
        reference = self.read_block(offset, halves) + self.read_block(offset + halves, length - halves)
        for _ in range(12):
            mark = time.monotonic()
            if self.read_block(offset, length) != reference:
                raise ProtocolError('Repeated reads of the same address returned different data.')
            samples.append((time.monotonic() - mark) * 1000)
        self.trace(f'12 identical replies of {length} B, median {statistics.median(samples):.1f} ms, '
                   f'maximum {max(samples):.1f} ms.')

    def read_flash(self, stage, progress):
        image = bytearray()
        started = time.monotonic()
        for offset in range(0, FLASH_SIZE, self.timing.read_block):
            block = self.read_block(offset, min(self.timing.read_block, FLASH_SIZE - offset))
            image.extend(block)
            progress(stage, len(image), FLASH_SIZE)
        self.trace(f'{stage}: {len(image):,} B in {time.monotonic() - started:.1f} s, SHA-256 {sha256(image)}')
        return bytes(image)

    def erase_and_program(self, candidate, progress):
        progress('Erasing flash', 0, FLASH_SIZE)
        self.send(compact(b'\xa5\x0f'))
        self.expect(self.frame(), ACK, 'Erase start acknowledgement')
        self.expect(self.frame(timeout=self.timing.erase_timeout), ACK, 'Erase completion acknowledgement')
        done = 0
        for start, end in ((BOOT_OFFSET, FLASH_SIZE), (0, BOOT_OFFSET)):
            for offset in range(start, end, 250):
                block = candidate[offset:min(offset + 250, end)]
                address = (0x200000 + offset).to_bytes(3, 'big')
                self.send(compact(b'\x36' + address + block))
                self.expect(self.frame(), ACK, f'Write at 0x{offset:05X}')
                done += len(block)
                progress('Writing flash', done, FLASH_SIZE)

    def write_direct(self, candidate, progress):
        """Erase the flash and write the BIN without backups, checks or read-back."""
        if len(candidate) != FLASH_SIZE:
            raise ValueError(f'BIN must be exactly {FLASH_SIZE:,} bytes, got {len(candidate):,}.')
        self.erase_sent = True
        try:
            self.erase_and_program(candidate, progress)
        except Exception as error:
            raise ProtocolError(f'Write interrupted after the erase command was sent. '
                                f'Keep the ECU powered. {error}') from error
        self.erase_sent = False

    def write_flash(self, candidate, backup, progress, *, self_test=False):
        if candidate is not None:
            validate_image(candidate)
        self.trace('Transfer check before backup.')
        self.test_read()
        current = self.read_flash('Backup', progress)
        if self_test:
            # Safe live write test: exactly the current ECU content is written back.
            # It exercises the full erase + program + verify cycle, yet the unit keeps
            # its original content with correct checksums (it is guaranteed to boot).
            candidate = current
            self.trace('Write self-test: the current ECU content will be written back.')
        else:
            self.trace(f'Matching part number: {check_compatibility(current, candidate, self.profile)}')
        save_image(backup, current)
        if Path(backup).read_bytes() != current:
            raise OSError('The saved backup does not match the ECU content.')
        self.trace(f'Backup saved to {backup}')
        if current == candidate and not self_test:
            self.trace('ECU content matches the selected BIN. No write was needed.')
            return False
        voltage = self.port.voltage()
        if not 12.2 <= voltage <= 14.8:
            raise RuntimeError(f'OBD voltage is {voltage:.2f} V. Use a stable 12.2–14.8 V for writing.')
        self.pause(250)
        self.check_cancel()
        self.erase_sent = True
        try:
            self.erase_and_program(candidate, progress)
            actual = self.read_flash('Verifying write', progress)
            if actual != candidate:
                mismatch = next(index for index, (a, b) in enumerate(zip(actual, candidate)) if a != b)
                raise ProtocolError(f'Flash content after writing differs from the BIN starting at address 0x{mismatch:05X}.')
        except Exception as error:
            raise ProtocolError(f'Write interrupted after the erase command was sent. Keep the ECU powered. '
                                f'Backup: {backup}. {error}') from error
        self.erase_sent = False
        return True

    def close(self):
        # Mid-byte the loader still waits for raw EEPROM bytes; a shutdown frame would be written as data.
        if self.erase_sent or self.eeprom_busy or not (self.loaded or self.diagnostic_open):
            return
        # When cancelling, the session shutdown must still complete.
        self.cancelled = lambda: False
        self.buffer.clear()
        if self.loaded:
            self.port.send(compact(b'\xa2'), self.timing.response_timeout)
            reply = self.frame(one_byte=True)
            if reply == ACK:
                reply = self.frame(one_byte=True)
            self.expect(reply, b'\x55', 'RAM loader session shutdown')
            self.loaded = False
            self.trace('RAM loader session closed.')
        else:
            self.stop_diagnostics()
