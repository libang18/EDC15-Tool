# EDC15 Tool

A Windows desktop application written in Python and Tkinter for the Tactrix OpenPort 2.0. It reads and writes the 512 KiB flash and the 24C04 EEPROM of VAG EDC15P/P+ and EDC15V units over K-line and J2534 04.04, either over OBD or in boot mode. The default profile is `EDC15P`.

## Driver

The original Tactrix driver relies on the kernel driver `openport.sys`, which Windows 11 blocks when Memory Integrity is on. EDC15 Tool was developed and tested with **Op20WinUSB**, a user-mode J2534 driver for the OpenPort 2.0 that works on Windows 11, comes in 32-bit and 64-bit builds and supports K-line including the 5-baud init that ECU identification needs: [libang18/Op20WinUSB](https://github.com/libang18/Op20WinUSB).

Download `Op20WinUSB-Setup.msi` from its [latest release](https://github.com/libang18/Op20WinUSB/releases/latest), run it, then pick **Op20WinUSB - Openport 2.0** in the driver list. Other registered OpenPort J2534 04.04 drivers also appear in the list, but they have not been tested with this tool.

## Running

Double-click **Run.cmd** or run `python -B app.py`. You need Python 3.10 or newer with Tkinter and a registered OpenPort J2534 DLL with the same bitness as Python. The application lists the registered OpenPort drivers, for example the 64-bit **Op20WinUSB**.

1. Select the driver and the ECU profile: `EDC15P` for EDC15P/P+ units, `EDC15V` for EDC15V/VM+.
2. **Identify ECU** shows the part number, description and six-digit Bosch SW at the top of the window. The Bosch number is read from the 32-byte flash header through the RAM loader, and the full ten-digit number is written to the log. Below it are the VAG diagnostic SW version and the available coding, WSC, VIN and immobilizer ID.
3. **Read Flash** saves 524,288 bytes and prints the SHA-256. Logs are stored in the `logs` folder.
4. **Read EEPROM…** saves a 512-byte backup of the 24C04 chip. The application compares two complete dumps byte by byte and writes the SHA-256 to the log. **Write EEPROM…** writes a 512-byte file over OBD with the same loader. It dumps the EEPROM, writes only the bytes that differ (one byte per loader command 0xA7/0xA8) and then compares a fresh dump with the file before the loader exits. Cancel is disabled while bytes are being written. **EEPROM Tool…** opens a separate window for inspecting backups and also displays new backups when the window is open. EEPROM write over OBD has not been tested on hardware yet.
5. **Write self-test** (command line only, `--writetest`) is a safe live check of writing. It reads the flash, erases it and writes back exactly the same content, then compares the whole flash byte by byte. This exercises the full erase and program cycle on a real unit, yet the ECU keeps its original content and correct checksums, so it boots normally.
6. **Write Flash** asks for a 512 KiB BIN and writes it immediately: no confirmation, no backup, no part number, voltage or checksum checks, and no read-back. Only the file size is checked (exactly 512 KiB). The boot sector is written first. Make a backup beforehand with **Read Flash**.
7. **BOOT MODE** works through the C167 bootstrap loader. Power the ECU up with the boot pin grounded, then start an operation. The link runs at 28,800 Bd. The first operation uploads the stage-1 loader and command monitor; later operations reuse the running monitor until the ECU is powered off.
   - **Read Flash…** configures the external bus and reads 512 KiB in 2 KiB XOR-checked blocks (about 3 minutes).
   - **Write Flash** asks for a 512 KiB BIN and writes it without confirmation. It configures the flash write window, uploads the flash helper and reads the chip ID. If the chip does not answer as a 29F400, it stops before erasing. Otherwise it erases the whole chip and programs every block that is not blank; the helper verifies each programmed word. Cancel is disabled once erasing starts. If a write fails, keep the ECU in boot mode and write again.
   - **Read EEPROM…** and **Write EEPROM…** clock the 24C04 I2C bus from the PC with monitor commands on P2.8 (SDA) and P2.9 (SCL), the pins the EEPROM loader uses. Each takes about 2 minutes. The write takes a 512-byte file, writes it in 16-byte pages without confirmation and waits for each page write to finish; it does not read the EEPROM back.

The sidebar holds the driver and profile selectors and **Timing…**. The **K-line** and **Boot mode** tabs each list Flash and EEPROM with their buttons. The voltage indicator at the bottom of the sidebar shows OBD pin 16 voltage while no operation is running: green for 12.2–14.8 V, red below 11 V or when the adapter is unavailable, yellow otherwise. The TX (yellow) and RX (green) lamps below it stay dark while the K-line is idle and flash brighter with every transmitted or received frame.

Use a stable power supply and keep the engine off while writing. Once erasing starts, keep power and USB connected until the write finishes.

## Inspect saved BINs

- **EEPROM Tool → Open BIN…** displays the file path and SHA-256 of a 512-byte backup. For the supported VAG 24C04 layout it also reads the VIN, immobilizer ID, ECU odometer, Immo ON/OFF status and five-digit Login/PIN, checks both stored copies and reports discrepancies.
- **Immo ON / Immo OFF** updates both immobilizer copies in the loaded BIN. **Undo changes** restores the last loaded or saved content. **Save BIN as…** saves the result, with a new filename suggested. Use **Write EEPROM…** to select the saved file for writing. The tool asks about unsaved changes when loading another file or closing. [VAG 24C04 layout reference](https://github.com/muki01/Bosch_EDC15_EEPROM_Tool/blob/main/src/Edc15EepromTool/Core/Edc15Eeprom.cs)
- **Edit…** beside Login/PIN, VIN or Immobilizer ID changes that field in both copies and recalculates its record checksum. Login/PIN accepts 00000 through 09999, VIN requires 17 characters and Immobilizer ID requires 14 characters with a supported VAG prefix. Letter input is converted to uppercase. These changes use the same Save and Undo controls. The seeded XOR checksums for the VIN and Immo ID were checked against a real backup. [Record checksum reference](https://gist.github.com/PL125/e81404ad44888518c56d43c7fded2f7f)
- **EEPROM Tool → Read Immo III status** reads the live adaptation state from measuring block 023 on a supported KW1281 instrument cluster (17) or separate immobilizer module (25). This requires the vehicle's immobilizer system to be connected. [Immo III measuring blocks](https://www.ross-tech.com/vag-com/cars/Immo3-measuring-blocks.html)
- **Flash → Checksum…** verifies the Bosch TDI V4.1 checksums for the identification and calibration regions of `038906019AF` with Bosch SW `1037354466`. Such a dump has seven matching regions. The check calculates a 32-bit value per region and compares it with the value stored in the image. [V4.1 checksum reference](https://github.com/Blackfrosch/VAGEDCSuite/blob/master/EDC15P_checksum.cs)

BIN inspection, editing and flash checksum checks work offline. Immobilizer editing requires recognizable VAG identity and known immobilizer values in the supported 24C04 layout. Live Immo III status requires a connected vehicle. `--check-flash` returns exit code 1 for a mismatch or an unsupported image.

## Timing and diagnostics

The **Timing** dialog saves changes to `timing.json`; **Defaults** restores the built-in values. The default P4 during initialization at 10400 Bd is 10 ms and the reply window is 900 ms. At 124800 Bd the fast channel uses P1 2 ms and P4 0 ms. The application converts P1 and P4 from milliseconds to J2534 04.04 steps of 0.5 ms. The **Log TX/RX frames** option also stores the content of the transferred frames.

If the ECU stops replying during a read, the application tries to close the session. The BIN is saved only after the complete flash has been received. If the ECU stays stuck, power-cycle it after the operation ends and start a new read. After an error once erasing or writing has started, keep the ECU powered.

```powershell
python -B app.py --fetch-loader
python -B app.py --adapter
python -B app.py --probe --trace
python -B app.py --identify
python -B app.py --writetest .\EDC15_backup.bin
python -B app.py --test
python -B app.py --read .\dumps\EDC15_flash.bin
python -B app.py --read-eeprom .\EDC15_EEPROM.bin
python -B app.py --inspect-eeprom .\dumps\EDC15_EEPROM.bin
python -B app.py --check-flash .\dumps\EDC15_flash.bin
python -B app.py --write-eeprom .\EDC15_EEPROM.bin
python -B app.py --boot-read .\EDC15_flash_boot.bin
python -B app.py --boot-write .\EDC15_flash.bin
python -B app.py --boot-read-eeprom .\EDC15_EEPROM_boot.bin
python -B app.py --boot-write-eeprom .\EDC15_EEPROM.bin
python -B -m unittest -v
```

On an EDC15P test unit, the diagnostic session at 10400 Bd and a test of 12 identical reads of a 254-byte block at 124800 Bd were confirmed. The median response time was 31 ms. A complete 512 KiB read took 56.8 s.

Verified on that unit: flash read and one flash write over OBD (69 s; the application does not read the flash back), EEPROM read and write over OBD (the write is checked by a read-back), boot-mode flash read (191 s) and boot-mode EEPROM read (115 s). Not tested on hardware yet: boot-mode flash write, boot-mode EEPROM write, the `--writetest` self-test and everything on EDC15V.

Keep your own backups in `dumps/`. That folder and `logs/` hold the VIN, immobilizer ID and login of your vehicle, so both are excluded from git.

## Reference RAM loader

Flash access over OBD uses the RAM loader from the V2 firmware of the `fjvva/ecu-tool` project. That project has no licence, so its file is not part of this repository. The first time you start a flash operation over OBD, the application offers to download `ECU_Flasher_UNO_EDC15.ino` from the author's repository; `python -B app.py --fetch-loader` does the same from the command line. The file is saved to `reference/` and kept only if the SHA-256 of the extracted loader matches. EEPROM access, boot mode and the offline tools work without it. [Source revision used](https://github.com/fjvva/ecu-tool/blob/5fea32b7becdeac14d591069e85ed70859533d53/Portable%20Firmware/HW%20V2/ECU_Flasher_UNO_EDC15/ECU_Flasher_UNO_EDC15.ino)

EEPROM reading uses `reference/kw1281test/Loader.bin` with the bundled MIT license. The application verifies the loader SHA-256, prepares the checksum for running in RAM and uses P1 0.5 ms while reading. [kw1281test revision used](https://github.com/gmenounos/kw1281test/blob/fe72e4fb77fdb4b57ac8a0cb7214e77b060a33b9/EDC15/Loader.a66)

## Boot mode

Boot mode uses `reference/kw1281test/Loader-boot-stage1.bin`, `Loader-boot-monitor.bin` and `Loader-boot-flash.bin` unchanged, with their `.a66` sources, under the bundled MIT license. The application verifies their SHA-256. [kw1281test revision used](https://github.com/gmenounos/kw1281test/tree/47769da6935e5e5e80288a755caada5757e8a411/EDC15)

kw1281test verified boot mode on EDC15V/VM+ (`038906012GN`, `038906012N`); for EDC15V the application uses the same read and write bus registers (read at `0x800000`, write window at `0x400000`). On EDC15P, boot-mode flash read is verified on a test unit: only BUSCON0 is set (`0x04AD`, 16-bit demultiplexed CS0) and the flash is read at `0x080000`. For EDC15P writes the application sets the CS3 write-only window exactly as that unit's own firmware does (ADDRSEL3 `0x2807`, BUSCON3 `0x84AD`), which is where the KWP2000 loader writes (`0x200000`). Boot-mode EEPROM read is verified on the same unit. Boot-mode flash write and boot-mode EEPROM write have not been tested on hardware yet.

## Appearance

The window uses the stock Tk `clam` theme with flat controls. All colours are set at the top of `Application.build` in `app.py`; `outline` is the single hairline colour used for fields, tabs and the panel edge. `accent` is the one blue used for primary buttons, the progress bar and the focused field.

## Licence

Copyright (C) 2026 libang18

EDC15 Tool is free software: you can redistribute it and modify it under the terms of the GNU General Public License, version 3, as published by the Free Software Foundation. It comes with no warranty. The full text is in [LICENSE](LICENSE).

The files in `reference/kw1281test` come from kw1281test and remain under the MIT licence included in that folder.
