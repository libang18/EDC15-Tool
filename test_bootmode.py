import random
import unittest
from collections import deque

from bootmode import (ACK, BAUD, BOARDS, DP2, ERR, HELPER, HELPER_ADDRESS, HELPER_HASH, MONITOR, MONITOR_HASH, NAK,
                      OK, P2, PROGRAM_BUFFER, READY, SCL, SDA, STAGE1, STAGE1_HASH, STAGE1_READY, BootSession,
                      load_loader, xor)
from edc15 import EEPROM_SIZE, FLASH_SIZE, Cancelled, ProtocolError, ResponseTimeout
from j2534 import ISO9141


class Eeprom24C04:
    """Edge-level I2C model of the 24C04 at device address 0xAC/0xAE."""

    def __init__(self, data):
        self.memory = bytearray(data)
        self.sda = self.scl = 1
        self.drive = None          # None = released, 0 = pulling SDA low
        self.state = 'idle'
        self.bits = self.value = 0
        self.acking = False
        self.pointer = 0
        self.pending = {}

    def line(self, master_sda):
        return 0 if self.drive == 0 else master_sda

    def update(self, master_sda, scl):
        sda = self.line(master_sda)
        if self.scl and scl and sda != self.sda:
            self.start() if not sda else self.stop()
        elif not self.scl and scl:
            self.rising(sda)
        elif self.scl and not scl:
            self.falling()
        self.sda, self.scl = self.line(master_sda), scl

    def start(self):
        self.state, self.bits, self.value, self.drive, self.acking = 'address', 0, 0, None, False

    def stop(self):
        for address, value in self.pending.items():
            self.memory[address] = value
        self.pending.clear()
        self.state, self.drive = 'idle', None

    def rising(self, sda):
        if self.state == 'send':
            if self.phase == 'data':
                self.bits += 1
            elif self.phase == 'master_ack':
                self.master_ack = not sda
        elif self.state in ('address', 'word', 'data') and not self.acking:
            self.value = self.value << 1 | sda
            self.bits += 1

    def falling(self):
        if self.state == 'send':
            if self.phase == 'address_ack':      # end of the ACK clock for the read address
                self.phase, self.bits = 'data', 0
                self.drive_bit()
            elif self.phase == 'data' and self.bits < 8:
                self.drive_bit()
            elif self.phase == 'data':
                self.phase, self.drive = 'master_ack', None
            elif self.master_ack:
                self.pointer = (self.pointer + 1) % EEPROM_SIZE
                self.phase, self.bits = 'data', 0
                self.drive_bit()
            else:
                self.state, self.drive = 'wait', None
            return
        if self.state not in ('address', 'word', 'data'):
            return
        if self.acking:
            self.acking, self.drive, self.bits, self.value = False, None, 0, 0
            if self.state == 'send':
                self.drive_bit()
        elif self.bits == 8:
            self.byte(self.value)

    def byte(self, value):
        if self.state == 'address':
            if value & 0xFC != 0xAC:
                self.state = 'wait'
                return
            self.block = value >> 1 & 1
            self.drive, self.acking = 0, True
            if value & 1:
                self.state, self.phase, self.acking = 'send', 'address_ack', False
            else:
                self.state = 'word'
            return
        self.drive, self.acking = 0, True
        if self.state == 'word':
            self.pointer = self.block << 8 | value
            self.state = 'data'
        else:
            self.pending[self.pointer] = value
            self.pointer = self.pointer & ~0xF | (self.pointer + 1) & 0xF

    def drive_bit(self):
        self.drive = None if self.memory[self.pointer] >> (7 - self.bits) & 1 else 0


class SimulatedBootEcu:
    """Byte-level model of the C167 BSL, the kw1281test monitor and its flash helper."""

    def __init__(self, image, board, answer=True, eeprom=bytes(EEPROM_SIZE)):
        self.flash = bytearray(image)
        self.board = board
        self.answer = answer
        self.state = 'bsl'
        self.pending = bytearray()
        self.rx = deque()
        self.channels = []
        self.registers = {P2: 0, DP2: 0}
        self.ram = {}
        self.corrupt_reads = False
        self.erase_count = 0
        self.uploads = 0
        self.eeprom = Eeprom24C04(eeprom)
        self.stage1 = load_loader(STAGE1, STAGE1_HASH)
        self.monitor = load_loader(MONITOR, MONITOR_HASH)
        self.helper = load_loader(HELPER, HELPER_HASH)

    def connect(self, protocol, baud, *_timings):
        self.channels.append((protocol, baud))

    def reply(self, data):
        # Deliver replies in uneven chunks, as the J2534 driver splits K-line messages.
        data = bytes(data)
        while data:
            size = random.randint(1, 700)
            self.rx.append(data[:size])
            data = data[size:]

    def receive(self, _timeout):
        return self.rx.popleft() if self.rx else b''

    def take(self, count):
        if len(self.pending) < count:
            return None
        data = bytes(self.pending[:count])
        del self.pending[:count]
        return data

    def send(self, wire, _timeout):
        self.pending.extend(wire)
        while self.step():
            pass

    # ---- port and I2C pins ----

    def pins(self):
        p2, dp2 = self.registers[P2], self.registers[DP2]
        sda = p2 & SDA if dp2 & SDA else SDA
        scl = p2 & SCL if dp2 & SCL else SCL
        return bool(sda), bool(scl)

    def poke(self, address, value):
        self.registers[address] = value
        if address in (P2, DP2):
            self.eeprom.update(*map(int, self.pins()))

    def peek(self, address):
        if address == P2:
            sda, scl = self.pins()
            value = self.registers[P2] & ~(SDA | SCL)
            return value | (SDA if self.eeprom.line(int(sda)) else 0) | (SCL if scl else 0)
        return self.registers.get(address, 0)

    # ---- flash helper ----

    def commands_reach_flash(self):
        return all(self.registers.get(address) == value for address, value in self.board.write_registers)

    def helper_call(self, regs):
        selector, write_segment, read_segment = regs[:3]
        segment = write_segment - self.board.write_segment
        assert read_segment - self.board.read_segment == segment
        reach = self.commands_reach_flash()
        if selector == 6:
            regs[1] = (0x2223 if regs[3] else 0x0001) if reach else int.from_bytes(self.flash[regs[3] * 2:][:2], 'little')
            regs[7] = 0
        elif selector == 2:
            if reach:
                self.erase_count += 1
                self.flash[:] = b'\xff' * FLASH_SIZE
            regs[7] = 0 if reach else 0x31
        elif selector == 0:
            offset = segment << 16 | regs[3]
            for index in range(regs[5]):
                self.flash[offset + index] &= self.ram[regs[4] + index]
            regs[7] = 0 if bytes(self.flash[offset:offset + regs[5]]) == bytes(
                self.ram[regs[4] + index] for index in range(regs[5])) else 1
        else:
            regs[7] = 1
        return regs

    def step(self):
        if self.state == 'bsl':
            if self.take(1) is None:
                return False
            if self.answer:
                self.reply(b'\xc5')
                self.state = 'stage1'
            return False
        if self.state == 'stage1':
            data = self.take(32)
            if data is None:
                return False
            assert data == self.stage1
            self.reply(bytes((STAGE1_READY,)))
            self.state = 'monitor'
            return True
        if self.state == 'monitor':
            data = self.take(len(self.monitor))
            if data is None:
                return False
            assert data == self.monitor
            self.uploads += 1
            self.reply(bytes((READY,)))
            self.state = 'command'
            return True
        if self.state == 'command':
            data = self.take(1)
            if data is None:
                return False
            if not 0x20 <= data[0] <= 0x25:
                self.reply(bytes((NAK,)))
                return True
            self.reply(bytes((ACK,)))
            self.state = data[0]
            return True
        needed = {0x20: 0, 0x21: 3, 0x22: 5, 0x23: 5, 0x24: 5, 0x25: 19}[self.state]
        if self.state == 0x24 and len(self.pending) >= 5:
            needed += int.from_bytes(self.pending[3:5], 'little')
        data = self.take(needed)
        if data is None:
            return False
        address = int.from_bytes(data[:3], 'little')
        if self.state == 0x20:
            self.reply(bytes((OK,)))
        elif self.state == 0x21:
            self.reply(self.peek(address).to_bytes(2, 'little') + bytes((OK,)))
        elif self.state == 0x22:
            self.poke(address, int.from_bytes(data[3:], 'little'))
            self.reply(bytes((OK,)))
        elif self.state == 0x23:
            count = int.from_bytes(data[3:], 'little')
            offset = address - self.board.read_base
            block = bytes(self.flash[offset:offset + count]) if 0 <= offset < FLASH_SIZE else b'\xff' * count
            checksum = xor(block) ^ (1 if self.corrupt_reads else 0)
            self.reply(block + bytes((checksum, OK)))
        elif self.state == 0x24:
            payload = data[5:]
            for index, value in enumerate(payload):
                self.ram[address + index] = value
            if address == HELPER_ADDRESS:
                assert payload == self.helper
            self.reply(bytes((xor(payload), OK)))
        else:
            assert address == HELPER_ADDRESS
            regs = [int.from_bytes(data[3 + index:5 + index], 'little') for index in range(0, 16, 2)]
            regs = self.helper_call(regs)
            self.reply(b''.join(value.to_bytes(2, 'little') for value in regs) + bytes((OK,)))
        self.state = 'command'
        return True


def fixture_image():
    image = bytearray(random.Random(28).randbytes(FLASH_SIZE))
    image[0x72010:0x7201C] = b'038906019AF\x00'
    image[0x40000:0x48000] = b'\xff' * 0x8000
    return bytes(image)


class BootModeTests(unittest.TestCase):
    def make(self, profile='EDC15P', **options):
        image = fixture_image()
        ecu = SimulatedBootEcu(image, BOARDS['EDC15V' if profile == 'EDC15V' else 'EDC15P'], **options)
        return image, ecu, BootSession(ecu, profile, trace=lambda *_: None)

    def test_edc15p_reads_full_flash_after_minimal_bus_setup(self):
        image, ecu, session = self.make()
        session.connect()
        self.assertEqual(image, session.read_flash(lambda *_: None))
        self.assertEqual([(ISO9141, BAUD)], ecu.channels)
        self.assertEqual(0x04AD, ecu.registers[0xFF0C])
        self.assertNotIn(0xFE1C, ecu.registers)

    def test_edc15v_uses_the_kw1281test_read_mode_registers(self):
        image, ecu, session = self.make('EDC15V')
        session.connect()
        session.configure(BOARDS['EDC15V'].read_registers)
        self.assertEqual(image[:2048], session.read_block(0x800000, 2048))
        self.assertEqual(0x3803, ecu.registers[0xFE18])

    def test_flash_write_erases_and_programs_non_blank_blocks(self):
        _, ecu, session = self.make()
        candidate = bytearray(random.Random(7).randbytes(FLASH_SIZE))
        candidate[0x10000:0x20000] = b'\xff' * 0x10000
        session.connect()
        session.write_flash(bytes(candidate), lambda *_: None)
        self.assertEqual(1, ecu.erase_count)
        self.assertEqual(bytes(candidate), bytes(ecu.flash))
        self.assertEqual(0x2807, ecu.registers[0xFE1C])
        self.assertEqual(0x84AD, ecu.registers[0xFF18])
        self.assertFalse(session.erase_sent)

    def test_flash_write_stops_before_erase_when_autoselect_fails(self):
        image, ecu, session = self.make()
        session.connect()
        ecu.board = type(ecu.board)(ecu.board.read_base, ecu.board.read_registers, ecu.board.write_segment,
                                    ((0xFE1C, 0x1234),))
        with self.assertRaisesRegex(ProtocolError, 'nothing was erased'):
            session.write_flash(fixture_image(), lambda *_: None)
        self.assertEqual(0, ecu.erase_count)
        self.assertEqual(image, bytes(ecu.flash))

    def test_flash_write_ignores_cancel_after_the_erase(self):
        _, ecu, session = self.make()
        session.connect()
        calls = []
        session.cancelled = lambda: bool(calls)
        candidate = bytes(random.Random(9).randbytes(FLASH_SIZE))
        session.write_flash(candidate, lambda stage, *_: calls.append(stage))
        self.assertEqual(candidate, bytes(ecu.flash))

    def test_eeprom_read_and_page_write_over_bit_banged_i2c(self):
        original = random.Random(4).randbytes(EEPROM_SIZE)
        _, ecu, session = self.make(eeprom=original)
        session.connect()
        self.assertEqual(original, session.read_eeprom(lambda *_: None))
        new = random.Random(5).randbytes(EEPROM_SIZE)
        session.write_eeprom(new, lambda *_: None)
        self.assertEqual(new, bytes(ecu.eeprom.memory))
        self.assertEqual(new, session.read_eeprom(lambda *_: None))
        self.assertEqual(0, ecu.registers[DP2] & (SDA | SCL))

    def test_eeprom_missing_acknowledge_is_reported(self):
        _, ecu, session = self.make()
        session.connect()
        ecu.eeprom.byte = lambda value: setattr(ecu.eeprom, 'state', 'wait')
        with self.assertRaisesRegex(ProtocolError, 'did not acknowledge the device address'):
            session.read_eeprom(lambda *_: None)

    def test_reconnect_reuses_the_running_monitor(self):
        _, ecu, session = self.make()
        session.connect()
        session.connect()
        self.assertEqual(1, ecu.uploads)

    def test_checksum_mismatch_is_reported(self):
        _, ecu, session = self.make()
        session.connect()
        ecu.corrupt_reads = True
        with self.assertRaisesRegex(ProtocolError, 'checksum'):
            session.read_block(0x080000, 256)

    def test_missing_handshake_asks_for_power_up_in_boot_mode(self):
        _, _, session = self.make(answer=False)
        with self.assertRaisesRegex(ResponseTimeout, 'boot pin grounded'):
            session.connect(timeout=50)

    def test_rejected_command_is_reported(self):
        _, _, session = self.make()
        session.connect()
        with self.assertRaisesRegex(ProtocolError, 'rejected command 0x27'):
            session.command(0x27)

    def test_cancel_stops_the_read(self):
        _, _, session = self.make()
        session.connect()
        session.cancelled = lambda: True
        with self.assertRaises(Cancelled):
            session.read_flash(lambda *_: None)


if __name__ == '__main__':
    unittest.main()
