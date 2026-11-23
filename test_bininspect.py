import unittest
from struct import pack_into

from bininspect import eeprom_fields, set_eeprom_field, set_eeprom_immo


def eeprom_image():
    image = bytearray(512)
    for offset in (0x140, 0x172):
        image[offset:offset + 17] = b'TMBGS26Y223456789'
    for offset in (0x131, 0x163):
        image[offset:offset + 14] = b'SKZ7Z0A1234567'
    for offset in (0x12E, 0x160):
        pack_into('<H', image, offset, 123)
    image[0x1B0] = image[0x1DE] = 0x73
    return bytes(image)


class EepromInspectionTests(unittest.TestCase):
    def test_switching_immo_changes_only_its_copies_and_round_trips(self):
        original = eeprom_image()
        result = set_eeprom_immo(original, False)
        self.assertEqual([0x1B0, 0x1DE], [i for i, pair in enumerate(zip(original, result)) if pair[0] != pair[1]])
        self.assertEqual('OFF', eeprom_fields(result)['Immo status'])
        self.assertEqual(original, set_eeprom_immo(result, True))
        self.assertEqual('00123', eeprom_fields(result)['Login / PIN'])

    def test_disagreeing_copies_are_reported_and_immo_copies_can_be_reconciled(self):
        image = bytearray(eeprom_image())
        image[0x1DE] = 0x60
        image[0x160] ^= 1
        fields = eeprom_fields(image)
        self.assertEqual('Copies differ', fields['Immo status'])
        self.assertEqual('Copies differ', fields['Login / PIN'])
        self.assertEqual('ON', eeprom_fields(set_eeprom_immo(image, True))['Immo status'])

    def test_unknown_immo_layout_cannot_be_patched(self):
        image = bytearray(eeprom_image())
        image[0x1B0] = image[0x1DE] = 0xFF
        self.assertIn('Unknown', eeprom_fields(image)['Immo status'])
        with self.assertRaises(ValueError):
            set_eeprom_immo(image, False)
        with self.assertRaises(ValueError):
            set_eeprom_immo(bytes(512), False)
        with self.assertRaises(ValueError):
            set_eeprom_immo(image[:-1], True)

    def test_erased_login_is_unavailable(self):
        image = bytearray(eeprom_image())
        for offset in (0x12E, 0x160):
            pack_into('<H', image, offset, 0xFFFF)
        self.assertEqual('Unavailable', eeprom_fields(image)['Login / PIN'])

    def test_identity_edits_update_both_records_and_checksum_bytes_only(self):
        original = eeprom_image()
        vectors = (
            ('Login / PIN', '09876', (0x12E, 0x160), bytes.fromhex('94 26 e7'), '09876'),
            ('VIN', 'wvwzzz1jz3w123456', (0x140, 0x172), b'WVWZZZ1JZ3W123456\xe0', 'WVWZZZ1JZ3W123456'),
            ('Immobilizer ID', 'vwz7z0a7654321', (0x131, 0x163), b'VWZ7Z0A7654321\xcd', 'VWZ7Z0A7654321'),
        )
        for label, value, offsets, record, displayed in vectors:
            with self.subTest(label=label):
                result = set_eeprom_field(original, label, value)
                allowed = set()
                for offset in offsets:
                    self.assertEqual(record, result[offset:offset + len(record)])
                    allowed.update(range(offset, offset + len(record)))
                self.assertTrue(all(original[i] == result[i] for i in range(512) if i not in allowed))
                self.assertEqual(displayed, eeprom_fields(result)[label])
                self.assertEqual(512, len(result))

    def test_invalid_identity_input_cannot_modify_the_source(self):
        original = bytearray(eeprom_image())
        for label, value in (
                ('Login / PIN', ''), ('Login / PIN', '10000'), ('Login / PIN', '-1'),
                ('Login / PIN', '12.3'), ('Login / PIN', '１２３'), ('Login / PIN', '000001'),
                ('VIN', 'WVWZZZ1JZ3W12345'), ('VIN', 'WVWZZZ1JZ3W12345Q'),
                ('Immobilizer ID', 'VWZ7Z0A765432'), ('Immobilizer ID', 'XXX7Z0A7654321')):
            with self.subTest(label=label, value=value), self.assertRaises(ValueError):
                set_eeprom_field(original, label, value)
            self.assertEqual(eeprom_image(), original)
        with self.assertRaises(ValueError):
            set_eeprom_field(bytes(512), 'Login / PIN', '1234')

    def test_login_boundaries_and_disagreeing_copies_can_be_repaired(self):
        original = bytearray(eeprom_image())
        original[0x160] ^= 1
        for code, record in (('00000', b'\x00\x00\x55'), ('09999', b'\x0f\x27\x7d')):
            result = set_eeprom_field(original, 'Login / PIN', code)
            self.assertEqual(code, eeprom_fields(result)['Login / PIN'])
            self.assertEqual(record, result[0x12E:0x131])
            self.assertEqual(record, result[0x160:0x163])
