"""EDC15 flash and EEPROM access through the C167 bootstrap loader (hardware boot mode).

The ECU must be powered up with its boot pin grounded, so the C167 starts its on-chip
bootstrap loader (BSL) instead of the firmware. The BSL contract is fixed by the silicon:
the host sends 0x00 for auto-baud, the CPU answers with its identification byte (0xC5 on
the C167) and then executes exactly 32 bytes loaded to 0FA40H.

The RAM programs uploaded here come unchanged from kw1281test (MIT, see
reference/kw1281test): a 32-byte stage-1 that receives a 382-byte command monitor to
0F800H, the monitor itself (PING/PEEK/POKE/READ/WRITE/CALL over half-duplex K-line at
28800 Bd) and a flash helper (Am29F400 autoselect, chip erase, program) called through it.
The OpenPort removes the echo of what it transmits, so unlike a plain KKL cable the host
never has to consume its own bytes.

The 24C04 EEPROM has no boot-mode loader. The host clocks its I2C bus itself with monitor
PEEK/POKE on P2.8 (SDA) and P2.9 (SCL), the pins and sequence of the kw1281test EEPROM loader.
"""

import time

from edc15 import FLASH_SIZE, EEPROM_SIZE, ROOT, Cancelled, ProtocolError, ResponseTimeout, sha256
from j2534 import ISO9141


LOADERS = ROOT / 'reference' / 'kw1281test'
STAGE1 = LOADERS / 'Loader-boot-stage1.bin'
STAGE1_HASH = '4e897f43d06cbd57d8c988caac5a6283b68654e38a074eda094acd7316c87f5c'
MONITOR = LOADERS / 'Loader-boot-monitor.bin'
MONITOR_HASH = '2cb845dffe2c89a78d7d4ac8ff929307a4bbfbf7a73686d9b64bffee2ace04a3'
HELPER = LOADERS / 'Loader-boot-flash.bin'
HELPER_HASH = 'bdbd5ef54115af8ed814aa924d04dd4030d2f5fb924262a7061a7feb971969e5'
BOOT_FILES = (STAGE1, MONITOR, HELPER)
BAUD = 28800
CPU_C167 = 0xC5
STAGE1_READY = 0xA5
READY, ACK, OK, ERR, NAK = 0xB0, 0xB1, 0xB2, 0xBE, 0xBF
OP_PING, OP_PEEK, OP_POKE, OP_READ, OP_WRITE, OP_CALL = 0x20, 0x21, 0x22, 0x23, 0x24, 0x25
READ_CHUNK = 2048
BYTE_MS = 10_000 / BAUD  # 8N1: ten bit times per byte

HELPER_ADDRESS = 0xF600   # flash helper load address and entry (below the monitor at 0F800H)
PROGRAM_BUFFER = 0xFC00   # 512-byte staging buffer in internal RAM
PROGRAM_BLOCK = 0x200
HELPER_PROGRAM, HELPER_CHIP_ERASE, HELPER_AUTOSELECT = 0x00, 0x02, 0x06
ERASE_TIMEOUT = 180_000   # the helper gives up on its own after its polling budget
FLASH_IDS = {0x2223: 'Am29F400BT', 0x22AB: 'Am29F400BB', 0x00D5: 'M29F400BT', 0x00D6: 'M29F400BB'}

P2, DP2 = 0xFFC0, 0xFFC2
SDA, SCL = 1 << 8, 1 << 9
EEPROM_DEVICE = 0xAC      # 24C04 with A2 = A1 = 1 on the EDC15 board; bit 1 selects the 256 B block
EEPROM_PAGE = 16


class Board:
    def __init__(self, read_base, read_registers, write_segment, write_registers):
        self.read_base = read_base
        self.read_registers = read_registers
        self.read_segment = read_base >> 16
        self.write_segment = write_segment
        self.write_registers = write_registers


# External bus setup per board, written with POKE. The C167 starts the BSL with the external
# bus in its reset configuration, so the flash must be mapped first.
# EDC15V/VM+: the board-fixed values kw1281test verified on 038906012GN/012N
#   (read on CS0 at 0x800000, write through the CS3 write-only window at 0x400000).
# EDC15P: BUSCON0 gets the same 16-bit demultiplexed CS0 timing; boot-mode read at 0x080000 is
#   verified on a test unit. Writes go through the CS3 write-only window that unit's firmware sets
#   up (ADDRSEL3 = 2807H, BUSCON3 = 84ADH), i.e. 0x200000, where the KWP2000 RAM loader writes.
BOARDS = {
    'EDC15P': Board(0x080000, ((0xFF0C, 0x04AD),),
                    0x20, ((0xFF0C, 0x04AD), (0xFE1C, 0x2807), (0xFF18, 0x84AD))),
    'EDC15V': Board(0x800000, ((0xFF12, 0xE204), (0xFF0C, 0x04AD), (0xFE18, 0x3803), (0xFE1A, 0x2008),
                               (0xFE1C, 0x0000), (0xFE1E, 0x0000), (0xFF14, 0x040D), (0xFF16, 0x04AD),
                               (0xFF18, 0x0000), (0xFF1A, 0x0000)),
                    0x40, ((0xFF12, 0xE204), (0xFE18, 0x0000), (0xFE1A, 0x0000), (0xFE1C, 0x0000),
                           (0xFE1E, 0x0000), (0xFF14, 0x0000), (0xFF16, 0x0000), (0xFF18, 0x0000),
                           (0xFF1A, 0x0000), (0xFE1C, 0x4008), (0xFF18, 0x848E), (0xFF0C, 0x04AD))),
}


def board_name(profile):
    return 'EDC15V' if profile == 'EDC15V' else 'EDC15P'


def load_loader(path, expected):
    data = path.read_bytes()
    if sha256(data) != expected:
        raise ValueError(f'{path.name}: SHA-256 does not match the bundled kw1281test revision.')
    return data


def xor(data):
    value = 0
    for byte in data:
        value ^= byte
    return value


class BootSession:
    def __init__(self, port, profile, trace=print, cancelled=lambda: False):
        self.port = port
        self.name = board_name(profile)
        self.board = BOARDS[self.name]
        self.trace = trace
        self.cancelled = cancelled
        self.buffer = bytearray()
        self.erase_sent = False
        self.stage1 = load_loader(STAGE1, STAGE1_HASH)
        self.monitor = load_loader(MONITOR, MONITOR_HASH)
        self.helper_image = load_loader(HELPER, HELPER_HASH)
        self.p2 = self.dp2 = 0

    # ---- K-line and monitor protocol ----

    def check_cancel(self):
        # Once the erase is under way the write has to finish; boot mode can always redo it.
        if not self.erase_sent and self.cancelled():
            raise Cancelled('Operation cancelled.')

    def send(self, data):
        self.check_cancel()
        self.port.send(bytes(data), int(len(data) * BYTE_MS) + 500)

    def receive(self, count, timeout, stage):
        deadline = time.monotonic() + timeout / 1000
        while len(self.buffer) < count:
            self.check_cancel()
            remaining = int((deadline - time.monotonic()) * 1000)
            if remaining <= 0:
                partial = f' Received {self.buffer[:16].hex(" ")}.' if self.buffer else ''
                raise ResponseTimeout(f'{stage}: no complete reply within {timeout} ms '
                                      f'({len(self.buffer)}/{count} B).{partial}')
            self.buffer.extend(self.port.receive(min(remaining, 200)))
        data = bytes(self.buffer[:count])
        del self.buffer[:count]
        return data

    def expect(self, value, stage, timeout=1000):
        reply = self.receive(1, timeout, stage)[0]
        if reply != value:
            raise ProtocolError(f'{stage}: expected 0x{value:02X}, received 0x{reply:02X}.')

    def connect(self, timeout=2000):
        self.port.connect(ISO9141, BAUD, 1, 0, 0)
        self.buffer.clear()
        self.trace(f'Boot mode · {BAUD} Bd · {self.name}')
        self.send(b'\x00')
        try:
            reply = self.receive(1, timeout, 'BSL identification')[0]
        except ResponseTimeout as error:
            raise ResponseTimeout('The ECU did not answer the bootstrap loader handshake. Power it up with the '
                                  'boot pin grounded and start the operation again.') from error
        if reply == NAK:
            # 0x00 is not a monitor command: the monitor from an earlier operation is still running.
            self.trace('Boot monitor from the previous operation is still running.')
        else:
            self.trace(f'BSL identification byte 0x{reply:02X}')
            if reply != CPU_C167:
                self.trace(f'Expected 0x{CPU_C167:02X} (C167); continuing.')
            self.send(self.stage1)
            self.expect(STAGE1_READY, 'Stage-1 loader')
            self.send(self.monitor)
            self.expect(READY, 'Boot monitor', timeout=int(len(self.monitor) * BYTE_MS) + 1000)
        self.ping()
        self.trace('Boot monitor is running.')

    def configure(self, registers):
        for address, value in registers:
            self.poke(address, value)
            readback = self.peek(address)
            note = '' if readback == value else f', reads back 0x{readback:04X}'
            self.trace(f'Register 0x{address:04X} = 0x{value:04X}{note}')

    def command(self, opcode):
        self.send(bytes((opcode,)))
        status = self.receive(1, 1000, f'Command 0x{opcode:02X}')[0]
        if status == NAK:
            raise ProtocolError(f'The boot monitor rejected command 0x{opcode:02X}.')
        if status != ACK:
            raise ProtocolError(f'Command 0x{opcode:02X}: expected ACK, received 0x{status:02X}.')

    def ping(self):
        self.command(OP_PING)
        self.expect(OK, 'PING')

    def peek(self, address):
        self.command(OP_PEEK)
        self.send(address.to_bytes(3, 'little'))
        reply = self.receive(3, 1000, 'PEEK')
        if reply[2] != OK:
            raise ProtocolError(f'PEEK 0x{address:06X}: expected OK, received 0x{reply[2]:02X}.')
        return int.from_bytes(reply[:2], 'little')

    def poke(self, address, value):
        self.command(OP_POKE)
        self.send(address.to_bytes(3, 'little') + value.to_bytes(2, 'little'))
        self.expect(OK, f'POKE 0x{address:06X}')

    def read_block(self, address, count):
        self.command(OP_READ)
        self.send(address.to_bytes(3, 'little') + count.to_bytes(2, 'little'))
        reply = self.receive(count + 2, int(count * BYTE_MS * 1.5) + 1000, f'READ 0x{address:06X}')
        data, checksum, status = reply[:count], reply[count], reply[count + 1]
        if status != OK:
            raise ProtocolError(f'READ 0x{address:06X}: expected OK, received 0x{status:02X}.')
        if checksum != xor(data):
            raise ProtocolError(f'READ 0x{address:06X}: checksum 0x{checksum:02X}, computed 0x{xor(data):02X}.')
        return data

    def write_ram(self, address, data):
        self.command(OP_WRITE)
        self.send(address.to_bytes(3, 'little') + len(data).to_bytes(2, 'little') + data)
        checksum, status = self.receive(2, int(len(data) * BYTE_MS) + 1000, f'WRITE 0x{address:06X}')
        if status == ERR:
            raise ProtocolError(f'WRITE 0x{address:06X}: RAM read-back did not match.')
        if status != OK:
            raise ProtocolError(f'WRITE 0x{address:06X}: expected OK, received 0x{status:02X}.')
        if checksum != xor(data):
            raise ProtocolError(f'WRITE 0x{address:06X}: checksum 0x{checksum:02X}, computed 0x{xor(data):02X}.')

    def call(self, entry, registers, timeout=2000):
        """Run a RAM routine with R8..R15 = registers and return R8..R15 afterwards."""
        self.command(OP_CALL)
        self.send(entry.to_bytes(3, 'little') + b''.join(value.to_bytes(2, 'little') for value in registers))
        reply = self.receive(17, timeout, f'CALL 0x{entry:04X}')
        if reply[16] != OK:
            raise ProtocolError(f'CALL 0x{entry:04X}: expected OK, received 0x{reply[16]:02X}.')
        return [int.from_bytes(reply[index:index + 2], 'little') for index in range(0, 16, 2)]

    # ---- flash ----

    def read_flash(self, progress):
        self.configure(self.board.read_registers)
        image = bytearray()
        started = time.monotonic()
        for offset in range(0, FLASH_SIZE, READ_CHUNK):
            image.extend(self.read_block(self.board.read_base + offset, READ_CHUNK))
            progress('Boot mode read', len(image), FLASH_SIZE)
        self.trace(f'Boot mode read: {len(image):,} B in {time.monotonic() - started:.1f} s, '
                   f'SHA-256 {sha256(image)}')
        return bytes(image)

    def helper(self, selector, segment=0, *values, timeout=2000):
        board = self.board
        registers = [selector, board.write_segment + segment, board.read_segment + segment, *values]
        return self.call(HELPER_ADDRESS, registers + [0] * (8 - len(registers)), timeout)

    def write_flash(self, image, progress):
        """Erase the whole chip and program the image, skipping blocks that stay erased."""
        if len(image) != FLASH_SIZE:
            raise ValueError(f'BIN must be exactly {FLASH_SIZE:,} bytes, got {len(image):,}.')
        self.configure(self.board.write_registers)
        self.write_ram(HELPER_ADDRESS, self.helper_image)
        maker = self.helper(HELPER_AUTOSELECT, 0, 0)[1]
        device = self.helper(HELPER_AUTOSELECT, 0, 1)[1]
        chip = FLASH_IDS.get(device)
        if chip is None:
            raise ProtocolError(f'The flash did not answer autoselect as a 29F400 (manufacturer 0x{maker:04X}, '
                                f'device 0x{device:04X}). Write commands do not reach the chip; nothing was erased.')
        self.trace(f'Flash chip {chip} (manufacturer 0x{maker:04X}, device 0x{device:04X})')
        self.erase_sent = True
        try:
            progress('Erasing flash', 0, FLASH_SIZE)
            started = time.monotonic()
            status = self.helper(HELPER_CHIP_ERASE, timeout=ERASE_TIMEOUT)[7]
            if status:
                raise ProtocolError(f'Chip erase failed (helper status 0x{status:02X}).')
            self.trace(f'Chip erased in {time.monotonic() - started:.1f} s.')
            for offset in range(0, FLASH_SIZE, PROGRAM_BLOCK):
                block = image[offset:offset + PROGRAM_BLOCK]
                if block.count(0xFF) != PROGRAM_BLOCK:
                    self.write_ram(PROGRAM_BUFFER, block)
                    status = self.helper(HELPER_PROGRAM, offset >> 16, offset & 0xFFFF, PROGRAM_BUFFER,
                                         PROGRAM_BLOCK)[7]
                    if status:
                        raise ProtocolError(f'Programming failed at 0x{offset:05X} (helper status 0x{status:02X}).')
                progress('Writing flash', offset + PROGRAM_BLOCK, FLASH_SIZE)
        except Exception as error:
            raise ProtocolError(f'Boot mode write interrupted after the erase. Keep the ECU in boot mode and '
                                f'write again. {error}') from error
        self.erase_sent = False
        self.trace(f'Boot mode write finished, SHA-256 {sha256(image)}')

    # ---- EEPROM over bit-banged I2C ----

    def set_port(self, p2=None, dp2=None):
        if p2 is not None and p2 != self.p2:
            self.p2 = p2
            self.poke(P2, p2)
        if dp2 is not None and dp2 != self.dp2:
            self.dp2 = dp2
            self.poke(DP2, dp2)

    def lines(self, sda=None, scl=None):
        p2 = self.p2
        if sda is not None:
            p2 = p2 | SDA if sda else p2 & ~SDA
        if scl is not None:
            p2 = p2 | SCL if scl else p2 & ~SCL
        self.set_port(p2=p2)

    def i2c_begin(self):
        # Reading P2 returns the pin levels, not the output latch, so the latch is always written
        # (both lines high) before the pins are switched to outputs.
        self.p2, self.dp2 = self.peek(P2) | SDA | SCL, self.peek(DP2)
        self.poke(P2, self.p2)
        self.set_port(dp2=self.dp2 | SDA | SCL)

    def i2c_end(self):
        self.set_port(dp2=self.dp2 & ~(SDA | SCL))

    def i2c_start(self):
        self.lines(sda=1)
        self.lines(scl=1)
        self.lines(sda=0)
        self.lines(scl=0)

    def i2c_stop(self):
        self.lines(sda=0)
        self.lines(scl=1)
        self.lines(sda=1)

    def i2c_write(self, value):
        """Clock out one byte MSB first and return True when the EEPROM acknowledges it."""
        for bit in range(7, -1, -1):
            self.lines(sda=value >> bit & 1)
            self.lines(scl=1)
            self.lines(scl=0)
        self.set_port(dp2=self.dp2 & ~SDA)
        self.lines(scl=1)
        acknowledged = not self.peek(P2) & SDA
        self.lines(sda=0, scl=0)
        self.set_port(dp2=self.dp2 | SDA)
        return acknowledged

    def i2c_read(self, acknowledge):
        self.set_port(dp2=self.dp2 & ~SDA)
        value = 0
        for _ in range(8):
            self.lines(scl=1)
            value = value << 1 | (1 if self.peek(P2) & SDA else 0)
            self.lines(scl=0)
        self.lines(sda=0 if acknowledge else 1)
        self.set_port(dp2=self.dp2 | SDA)
        self.lines(scl=1)
        self.lines(scl=0)
        return value

    def i2c_send(self, value, stage):
        if not self.i2c_write(value):
            self.i2c_stop()
            raise ProtocolError(f'EEPROM did not acknowledge {stage} (0x{value:02X}).')

    def read_eeprom(self, progress):
        started = time.monotonic()
        self.i2c_begin()
        try:
            self.i2c_start()
            self.i2c_send(EEPROM_DEVICE, 'the device address')
            self.i2c_send(0x00, 'the word address')
            self.i2c_start()
            self.i2c_send(EEPROM_DEVICE | 1, 'the read address')
            image = bytearray()
            for index in range(EEPROM_SIZE):
                image.append(self.i2c_read(index < EEPROM_SIZE - 1))
                progress('Boot mode EEPROM read', len(image), EEPROM_SIZE)
            self.i2c_stop()
        finally:
            self.i2c_end()
        self.trace(f'Boot mode EEPROM read: {len(image)} B in {time.monotonic() - started:.1f} s, '
                   f'SHA-256 {sha256(image)}')
        return bytes(image)

    def write_eeprom(self, image, progress):
        if len(image) != EEPROM_SIZE:
            raise ValueError(f'EEPROM image must be exactly {EEPROM_SIZE} bytes, got {len(image):,}.')
        started = time.monotonic()
        self.i2c_begin()
        try:
            for address in range(0, EEPROM_SIZE, EEPROM_PAGE):
                device = EEPROM_DEVICE | (address >> 8) << 1
                self.i2c_start()
                self.i2c_send(device, 'the device address')
                self.i2c_send(address & 0xFF, 'the word address')
                for value in image[address:address + EEPROM_PAGE]:
                    self.i2c_send(value, f'data at 0x{address:03X}')
                self.i2c_stop()
                for _ in range(20):  # acknowledge polling until the page write cycle is over
                    self.i2c_start()
                    ready = self.i2c_write(device)
                    self.i2c_stop()
                    if ready:
                        break
                else:
                    raise ProtocolError(f'EEPROM stayed busy after writing the page at 0x{address:03X}.')
                progress('Boot mode EEPROM write', address + EEPROM_PAGE, EEPROM_SIZE)
        finally:
            self.i2c_end()
        self.trace(f'Boot mode EEPROM write: {EEPROM_SIZE} B in {time.monotonic() - started:.1f} s, '
                   f'SHA-256 {sha256(image)}')
