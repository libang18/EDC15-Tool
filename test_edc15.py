import random
import tempfile
import unittest
from collections import deque
from pathlib import Path
from unittest.mock import call, patch

from edc15 import REFERENCE
from edc15 import (BOOT_OFFSET, FLASH_SIZE, BOSCH_SW_HEADER, EEPROM_SIZE, Cancelled, ProtocolError, Session, Timing,
                   addressed, compact, eeprom_loader, loader_blocks, programming_key, save_image, sha256, with_checksum)
from j2534 import DATA_RATE, ISO14230, LOOPBACK, P1_MAX, P3_MIN, P4_MIN, OpenPort


# The flash RAM loader source is downloaded on demand (python app.py --fetch-loader), not bundled.
NEEDS_LOADER = unittest.skipUnless(REFERENCE.is_file(), 'flash RAM loader source not downloaded')


def fixture_image():
    image = bytearray(random.Random(19).randbytes(FLASH_SIZE))
    image[0x72010:0x7201C] = b'038906019AF\x00'
    return bytes(image)


class SimulatedPort:
    def __init__(self, image):
        self.image = bytearray(image)
        self.rx = deque()
        self.writes = []
        self.erase_count = 0
        self.reject_write = False
        self.voltage_value = 13.7

    def receive(self, _timeout):
        return self.rx.popleft() if self.rx else b''

    def voltage(self):
        return self.voltage_value

    def send(self, wire, _timeout):
        if wire[0] != len(wire) - 2 or wire != with_checksum(wire[:-1]):
            raise AssertionError('Invalid request frame.')
        payload = wire[1:-1]
        if payload == b'\xa5\x0f':
            self.erase_count += 1
            self.image[:] = b'\xff' * FLASH_SIZE
            self.rx.append(bytes.fromhex('017677017677'))
        elif payload[0] == 0x23:
            offset = int.from_bytes(payload[1:4], 'big') - 0x080000
            reply = compact(b'\x36' + self.image[offset:offset + payload[4]])
            self.rx.extend((reply[:1], reply[1:7], reply[7:]))
        elif payload[0] == 0x36:
            offset = int.from_bytes(payload[1:4], 'big') - 0x200000
            block = payload[4:]
            self.image[offset:offset + len(block)] = block
            self.writes.append((offset, len(block)))
            self.rx.append(compact(b'\x7f' if self.reject_write else b'\x76'))
        else:
            raise AssertionError(f'Unexpected service {payload[0]:02x}.')


class TranscriptPort:
    def __init__(self, frames):
        self.frames = deque(frames)
        self.reply = deque()
        self.channels = []

    def connect(self, protocol, baud, *_timings):
        self.channels.append((protocol, baud))

    def fast_init(self, wire):
        return self.exchange(wire)

    def send(self, wire, _timeout):
        self.reply.append(self.exchange(wire))

    def exchange(self, wire):
        expected, reply = self.frames.popleft()
        if wire != expected:
            raise AssertionError(f'Expected {expected[:12].hex()}, got {wire[:12].hex()}.')
        return reply

    def receive(self, _timeout):
        return self.reply.popleft()


class EDC15Tests(unittest.TestCase):
    def make_session(self, image):
        port = SimulatedPort(image)
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        return session, port

    @NEEDS_LOADER
    def test_reference_and_addressed_frames(self):
        self.assertEqual([257] * 4, [len(block) for block in loader_blocks()])
        self.assertEqual(bytes.fromhex('8101f181f4'), addressed(b'\x81'))
        self.assertEqual(bytes.fromhex('001d69a0'), programming_key(b'\x00\x00\xeb\x4d', 'EDC15P'))

    def test_eeprom_loader_matches_the_hardware_verified_image(self):
        self.assertEqual('b66924a398e5988cfedd8f899f095930574f0b5c62c19903ea0de74cca78f93b',
                         sha256(eeprom_loader()))

    def test_eeprom_verifies_two_dumps_and_closes_loader(self):
        image = random.Random(15).randbytes(EEPROM_SIZE)
        ack = bytes.fromhex('017677')
        request = compact(b'\xa6')
        port = TranscriptPort([(request, ack + image + ack), (request, ack + image + ack),
                               (compact(b'\xa2'), ack + b'\x55')])
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        session.loaded = True
        self.assertEqual(image, session.read_eeprom(lambda *_: None))
        session.close()
        self.assertFalse(session.loaded)
        self.assertFalse(port.frames)

    def test_eeprom_rejects_different_dumps(self):
        ack = bytes.fromhex('017677')
        request = compact(b'\xa6')
        port = TranscriptPort([(request, ack + bytes(EEPROM_SIZE) + ack),
                               (request, ack + b'\xff' * EEPROM_SIZE + ack)])
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        with self.assertRaisesRegex(ProtocolError, 'EEPROM dumps differ'):
            session.read_eeprom(lambda *_: None)

    @patch('edc15.time.sleep')
    def test_eeprom_write_changes_only_differing_bytes_and_verifies(self, _sleep):
        current = bytearray(random.Random(16).randbytes(EEPROM_SIZE))
        image = bytearray(current)
        image[0x012] ^= 0xFF
        image[0x1F0] ^= 0x55
        ack = bytes.fromhex('017677')
        dump = compact(b'\xa6')
        frames = [(dump, ack + bytes(current) + ack)]
        for command, address in ((b'\xa7', 0x012), (b'\xa8', 0x1F0)):
            frames += [(compact(command), ack), (bytes((address & 0xFF,)), b''), (bytes((image[address],)), ack)]
        frames += [(dump, ack + bytes(image) + ack), (compact(b'\xa2'), ack + b'\x55')]
        port = TranscriptPort(frames)
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        session.loaded = True
        self.assertEqual(2, session.write_eeprom(bytes(image), lambda *_: None))
        session.close()
        self.assertFalse(port.frames)

    @patch('edc15.time.sleep')
    def test_eeprom_write_reports_a_mismatching_read_back(self, _sleep):
        current = bytes(EEPROM_SIZE)
        image = b'\x01' + current[1:]
        ack = bytes.fromhex('017677')
        port = TranscriptPort([(compact(b'\xa6'), ack + current + ack), (compact(b'\xa7'), ack), (b'\x00', b''),
                               (b'\x01', ack), (compact(b'\xa6'), ack + current + ack)])
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        with self.assertRaisesRegex(ProtocolError, 'differs from the image at 0x000'):
            session.write_eeprom(image, lambda *_: None)

    def test_incomplete_configuration(self):
        for name, value in [('read_block', 255), ('init_p4', -1), ('response_timeout', True)]:
            with self.subTest(name=name), self.assertRaises(ValueError):
                Timing(**{name: value})

    def test_kline_configuration_converts_milliseconds_to_j2534_units(self):
        port = OpenPort('unused.dll')
        port.device = 1
        with patch.object(port, 'call'), patch.object(port, 'configure', side_effect=[None, {DATA_RATE: 10400}]) as config:
            port.connect(ISO14230, 10400, 20, 3, 10)
        self.assertEqual(config.call_args_list, [call({LOOPBACK: 0, P1_MAX: 40, P3_MIN: 6, P4_MIN: 20}),
                                                call({DATA_RATE: 0}, read=True)])

    def test_identification_handles_part_number_flag_and_closes_kw1281(self):
        identification = b'\x01\xf6\xb038906019AF 1,9l R4 EDC\x03'
        first_block = bytes((len(identification),)) + identification
        ack_block = b'\x03\x03\x09\x03'
        frames = [(bytes((sent ^ 0xFF,)), bytes((reply,))) for sent, reply in zip(first_block, first_block[1:])]
        frames.extend([(b'\x03', b'\xfc'), (b'\x02', b'\xfd'), (b'\x09', b'\xf6'), (b'\x03', ack_block[:1])])
        frames.extend((bytes((sent ^ 0xFF,)), bytes((reply,))) for sent, reply in zip(ack_block, ack_block[1:]))
        frames.extend([(b'\x03', b'\xfc'), (b'\x04', b'\xfb'), (b'\x06', b'\xf9'), (b'\x03', b'')])
        port = TranscriptPort(frames)
        port.reply.append(first_block[:1])
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        with patch.object(port, 'slow_init', return_value=b'\x01\x8a', create=True):
            self.assertEqual('Part number: 038906019AF\nDescription: 1,9l R4 EDC', session.identify())
        self.assertEqual([(3, 9600)], port.channels)
        self.assertFalse(port.frames)

    def test_identification_reads_coding_and_advertised_extra_records(self):
        port = TranscriptPort([])
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        frames = [(1, 0xF6, b'\xb038906019AF 1,9l R4 EDC'), (3, 0xF6, bytes.fromhex('00000535c5')),
                  (5, 0xF6, b'TMBGS26Y223456789  SKZ7Z0A1234567'), (7, 0x09, b'')]
        with (patch.object(port, 'slow_init', return_value=b'\x01\x8a', create=True),
              patch.object(session, 'kw_receive_block', side_effect=frames),
              patch.object(session, 'kw_send_block') as send):
            result = session.identify()
        self.assertEqual(send.call_args_list, [call(2, 0x09), call(4, 0x00), call(6, 0x09), call(8, 0x06)])
        self.assertIn('Coding: 00002\nWSC: 79301', result)
        self.assertIn('VIN: TMBGS26Y223456789\nImmobilizer ID: SKZ7Z0A1234567', result)

    def test_immo_status_uses_read_only_group_and_closes_both_modules(self):
        frames = []

        def send_block(data, reply=b''):
            frames.extend((bytes((value,)), bytes((value ^ 0xFF,))) for value in data)
            frames.append((b'\x03', reply[:1]))
            frames.extend((bytes((value ^ 0xFF,)), bytes((following,))) for value, following in zip(reply, reply[1:]))

        acknowledgement = b'\x03\x01\x09\x03'
        request = b'\x04\x02\x29\x17'
        send_block(request, b'\x03\x03\x0a\x03')
        send_block(b'\x03\x04\x06')
        send_block(request, b'\x0f\x03\xe7' + bytes(9) + b'\x36\x00\x06\x03')
        send_block(b'\x03\x04\x06')
        port = TranscriptPort(frames)
        session = Session(port, 'EDC15P', trace=lambda *_: None)

        def wake(_address):
            port.reply.clear()
            port.reply.append(acknowledgement[:1])
            pending = [(bytes((value ^ 0xFF,)), bytes((following,)))
                       for value, following in zip(acknowledgement, acknowledgement[1:])]
            port.frames.extendleft(reversed(pending))
            return b'\x01\x8a'

        with patch.object(port, 'slow_init', side_effect=wake, create=True) as slow_init:
            self.assertEqual('Adapted, normal operation · module 25', session.read_immobilizer_status())
        self.assertEqual([call(0x17), call(0x25)], slow_init.call_args_list)
        self.assertFalse(port.frames)

    def test_reads_bosch_software_from_the_current_flash_header(self):
        for number in (b'1037354466', b'1037354619'):
            with self.subTest(number=number):
                image = bytearray(fixture_image())
                image[BOSCH_SW_HEADER:BOSCH_SW_HEADER + 32] = b'\xff' * 6 + number + b'\xff' * 16
                session, _ = self.make_session(image)
                self.assertEqual(number.decode('ascii'), session.read_bosch_software())

    def test_wakeup_accepts_specific_reply_after_kw1281(self):
        reply = bytes.fromhex('83 06 01 7f 03 13 1f')
        port = TranscriptPort([(addressed(b'\x81'), reply)])
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        self.assertEqual(b'\x7f\x03\x13', session.wake())
        with self.assertRaisesRegex(ProtocolError, 'Unexpected KWP addresses'):
            session.frame(address=True, initial=reply)

    def test_reads_complete_image_across_split_frames_and_last_block(self):
        image = fixture_image()
        session, _ = self.make_session(image)
        session.test_read()
        self.assertEqual(image, session.read_flash('Read', lambda *_: None))

    @NEEDS_LOADER
    @patch('edc15.time.sleep')
    def test_normal_session_switches_baud_and_loads_ram(self, _sleep):
        blocks = loader_blocks()
        frames = [(bytes.fromhex(request), bytes.fromhex(reply)) for request, reply in (
            ('8101f181f4', '8300017f001316'),
            ('8201f1108509', '83f1017f10787c83f10150852f79'),
            ('8101f182f5', '81f101c235'),
            ('8101f181f4', '83f101c16b8f'),  # FAST_INIT without the trailing checksum.
            ('031085871f', '035085875f'),
            ('032741006b', '0667410000eb4de6'),
            ('062742001d69a095', '03674234e0'),
            ('083440e0000000042080', '0274ff75'),
        )]
        frames.extend((with_checksum(block), b'\x01\x76\x77') for block in blocks)
        for request, reply in (
            ('36c3c389c545da630b', '76'), ('31021000000fbfff', '7f3122'),
            ('2308000010', '36' + 'ab' * 16), ('2310000010', '36' + 'cd' * 16),
            ('31020800000fbfff', '7f'), ('a2', ''),
        ):
            frames.append((compact(bytes.fromhex(request)), compact(bytes.fromhex(reply)) if reply else b'\x55'))
        port = TranscriptPort(frames)
        session = Session(port, 'EDC15P', trace=lambda *_: None)
        session.start(blocks)
        self.assertTrue(session.loaded)
        self.assertEqual([(4, 10400), (3, 124800)], port.channels)
        session.close()
        self.assertFalse(port.frames)

    @patch('edc15.time.sleep')
    def test_programs_boot_first_and_verifies_full_flash(self, _sleep):
        original = fixture_image()
        candidate = original[:0x15300] + b'\x11\x22' + original[0x15302:]
        session, port = self.make_session(original)
        with tempfile.TemporaryDirectory() as directory:
            backup = Path(directory, 'before.bin')
            self.assertTrue(session.write_flash(candidate, backup, lambda *_: None))
            self.assertEqual(original, backup.read_bytes())
        self.assertEqual(1, port.erase_count)
        self.assertEqual((BOOT_OFFSET, 250), port.writes[0])
        self.assertEqual(FLASH_SIZE, sum(size for _, size in port.writes))
        self.assertEqual(candidate, port.image)

    @patch('edc15.time.sleep')
    def test_self_test_rewrites_current_content(self, _sleep):
        original = fixture_image()
        session, port = self.make_session(original)
        with tempfile.TemporaryDirectory() as directory:
            backup = Path(directory, 'before.bin')
            self.assertTrue(session.write_flash(None, backup, lambda *_: None, self_test=True))
            self.assertEqual(original, backup.read_bytes())
        self.assertEqual(1, port.erase_count)
        self.assertEqual((BOOT_OFFSET, 250), port.writes[0])
        self.assertEqual(FLASH_SIZE, sum(size for _, size in port.writes))
        self.assertEqual(original, port.image)

    def test_refuses_mismatched_file_before_erasing(self):
        original = fixture_image()
        candidate = original.replace(b'038906019AF', b'038906019AJ')
        session, port = self.make_session(original)
        with tempfile.TemporaryDirectory() as directory, self.assertRaisesRegex(ValueError, 'part number'):
            session.write_flash(candidate, Path(directory, 'before.bin'), lambda *_: None)
        self.assertEqual(0, port.erase_count)

    def test_existing_backup_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            backup = Path(directory, 'before.bin')
            save_image(backup, b'original')
            with self.assertRaises(FileExistsError):
                save_image(backup, b'new')
            self.assertEqual(b'original', backup.read_bytes())

    @patch('edc15.time.sleep')
    def test_write_errors_are_reported_without_retries(self, _sleep):
        original = fixture_image()
        candidate = bytes((original[0] ^ 1,)) + original[1:]
        session, port = self.make_session(original)
        port.reject_write = True
        with tempfile.TemporaryDirectory() as directory:
            backup = Path(directory, 'before.bin')
            with self.assertRaisesRegex(ProtocolError, 'Keep the ECU powered'):
                session.write_flash(candidate, backup, lambda *_: None)
            self.assertEqual(original, backup.read_bytes())
        self.assertTrue(session.erase_sent)
        self.assertEqual([(BOOT_OFFSET, 250)], port.writes)

    def test_cancel_before_transfer(self):
        session, _ = self.make_session(fixture_image())
        session.cancelled = lambda: True
        with self.assertRaises(Cancelled):
            session.read_block(0, 16)

    @patch('edc15.time.sleep')
    def test_cancellation_does_not_prevent_diagnostic_session_cleanup(self, _sleep):
        port = TranscriptPort([(bytes.fromhex('8101f182f5'), bytes.fromhex('81f101c235'))])
        session = Session(port, 'EDC15P', trace=lambda *_: None, cancelled=lambda: True)
        session.diagnostic_open = True
        session.close()
        self.assertFalse(session.diagnostic_open)
        self.assertFalse(port.frames)


if __name__ == '__main__':
    unittest.main()
