"""Offline inspection and EEPROM editing of VAG EDC15 memory dumps."""

from struct import iter_unpack, unpack_from

from edc15 import (BOSCH_SW_HEADER, BOSCH_SW_HEADER_SIZE, EEPROM_SIZE, IMMOBILIZER_ID, VIN,
                   bosch_software_id, parts, validate_image)

IMMO_OFFSETS = (0x1B0, 0x1DE)
IMMO_VALUES = {True: 0x73, False: 0x60}
EEPROM_RECORDS = {
    'Login / PIN': ((0x12E, 0x160), 0x55),
    'VIN': ((0x140, 0x172), 0xAE),
    'Immobilizer ID': ((0x131, 0x163), 0xBA),
}


def _identity_copies(image, offsets, pattern, size):
    copies = [image[offset:offset + size].decode('ascii', errors='replace') for offset in offsets]
    valid = [value for value in copies if pattern.fullmatch(value)]
    if not valid:
        return 'Unavailable', False
    if len(valid) == 1:
        return f'{valid[0]} (one valid copy)', True
    if valid[0] != valid[1]:
        return 'Copies differ', True
    return valid[0], True


def eeprom_fields(image):
    """Read identity and mileage for the supported VAG 24C04 layout."""
    if len(image) != EEPROM_SIZE:
        raise ValueError(f'EEPROM BIN must be exactly {EEPROM_SIZE} bytes, got {len(image):,}.')

    vin, has_vin = _identity_copies(image, (0x140, 0x172), VIN, 17)
    immo_id, has_immo_id = _identity_copies(image, (0x131, 0x163), IMMOBILIZER_ID, 14)
    if not (has_vin or has_immo_id):
        raise ValueError('The EEPROM BIN has no recognizable VAG identity in the supported 24C04 layout.')

    # The odometer stores hundredths of a kilometre. The top nibble belongs to another field.
    distance, mirror = (unpack_from('<I', image, offset)[0] & 0x0FFFFFFF for offset in (0x1BF, 0x1ED))
    if distance != mirror:
        mileage = 'Copies differ'
    elif distance == 0x0FFFFFFF:
        mileage = 'Unavailable'
    else:
        kilometres, fraction = divmod(distance, 100)
        mileage = f'{kilometres:,}.{fraction:02d} km'

    primary, mirror = (image[offset] for offset in IMMO_OFFSETS)
    if primary != mirror:
        immo = 'Copies differ'
    else:
        immo = {0x73: 'ON', 0x60: 'OFF'}.get(primary, f'Unknown (0x{primary:02X})')

    login, mirror = (unpack_from('<H', image, offset)[0] for offset in (0x12E, 0x160))
    if login != mirror:
        login_text = 'Copies differ'
    elif login > 9999:
        login_text = 'Unavailable'
    else:
        login_text = f'{login:05d}'

    return {'VIN': vin, 'Immobilizer ID': immo_id, 'ECU odometer': mileage,
            'Immo status': immo, 'Login / PIN': login_text}


def set_eeprom_immo(image, enabled):
    """Set both immobilizer copies in a supported VAG 24C04 dump."""
    eeprom_fields(image)
    if any(image[offset] not in IMMO_VALUES.values() for offset in IMMO_OFFSETS):
        raise ValueError('Unrecognized immobilizer data in this EEPROM layout.')
    result = bytearray(image)
    for offset in IMMO_OFFSETS:
        result[offset] = IMMO_VALUES[enabled]
    return bytes(result)


def set_eeprom_field(image, label, value):
    """Update both identity records and their seeded XOR checksums."""
    eeprom_fields(image)
    value = value.strip().upper()
    if label == 'Login / PIN':
        if not value.isascii() or not value.isdecimal() or len(value) > 5 or int(value) > 9999:
            raise ValueError('Login / PIN must be a decimal code from 00000 through 09999.')
        data = int(value).to_bytes(2, 'little')
    elif label == 'VIN':
        if not VIN.fullmatch(value):
            raise ValueError('VIN must contain 17 letters or digits, with at least one letter. I, O and Q are excluded.')
        data = value.encode('ascii')
    elif label == 'Immobilizer ID':
        if not IMMOBILIZER_ID.fullmatch(value):
            raise ValueError('Immo ID must contain 14 letters or digits and begin with AUZ, SEZ, SKZ or VWZ.')
        data = value.encode('ascii')
    else:
        raise ValueError(f'Unsupported EEPROM field {label}.')

    offsets, checksum = EEPROM_RECORDS[label]
    for byte in data:
        checksum ^= byte
    record = data + bytes((checksum,))
    result = bytearray(image)
    for offset in offsets:
        result[offset:offset + len(record)] = record
    return bytes(result)


def inspect_eeprom(image):
    return '\n'.join(f'{label}: {value}' for label, value in eeprom_fields(image).items())


def _flash_checksum(region, low, high):
    """Calculate the 32-bit Bosch TDI V4.1 checksum of one region."""
    words = iter_unpack('<H', region)
    for (first,) in words:
        (second,) = next(words)
        low ^= first
        shift = high & 15
        low = ((low << shift) | (low >> (16 - shift))) & 0xFFFF
        carry = (low & 1) if shift else 0
        high = ((high - second - carry) & 0xFFFF) ^ low
        # These ranges end with two words after the last full four-word group.
        third = next(words, None)
        if third is None:
            break
        (fourth,) = next(words)
        low = (low - third[0] + 0xDAAC) & 0xFFFF
        high ^= fourth
        shift = low & 15
        high = ((high >> shift) | (high << (16 - shift))) & 0xFFFF
    return ((high ^ 0xDF9B) << 16) | ((low - 0x8631 + 0xDAAC) & 0xFFFF)


def check_flash(image):
    """Verify the identification and calibration regions of the one supported image."""
    validate_image(image)
    software = bosch_software_id(image[BOSCH_SW_HEADER:BOSCH_SW_HEADER + BOSCH_SW_HEADER_SIZE])
    if '038906019AF' not in parts(image) or software != '1037354466':
        raise ValueError('Checksum layout currently supported for 038906019AF, Bosch SW 1037354466.')

    regions = [(0x10000, 0x14000)]
    for bank in (0x50000, 0x60000, 0x70000):
        start, end = bank - 0x4000, bank + 0xC000
        # The middle bank is unused in the stock image. The outer banks must be checked even if erased.
        if bank == 0x60000 and image.count(0xC3, start, end) == end - start:
            continue
        regions.extend(((start, bank), (bank, bank + 0xB80), (bank + 0xB80, end)))

    mismatches = []
    for start, end in regions:
        seeds = (0, 0) if start == 0x10000 else (0x8631, 0xEFCD)
        stored = unpack_from('<I', image, end - 4)[0]
        if _flash_checksum(image[start:end - 4], *seeds) != stored:
            mismatches.append((start, end - 5))

    report = [f'Bosch TDI V4.1. {len(regions) - len(mismatches)}/{len(regions)} checksums match.',
              'Coverage: identification and calibration regions of 038906019AF.']
    report.extend(f'Checksum mismatch for 0x{start:05X} through 0x{end:05X}.' for start, end in mismatches)
    return '\n'.join(report), bool(mismatches)
