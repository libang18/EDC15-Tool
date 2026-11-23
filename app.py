"""Windows desktop flasher for the Bosch EDC15P/V over OpenPort 2.0."""

import argparse
import ctypes
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
from queue import Empty, Queue
from threading import Event, Lock, Thread
import time
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

from bininspect import (EEPROM_RECORDS, IMMO_OFFSETS, IMMO_VALUES, check_flash, eeprom_fields,
                        inspect_eeprom, set_eeprom_field, set_eeprom_immo)
from edc15 import (EEPROM_SIZE, FLASH_SIZE, KEY_WORDS, REFERENCE, ROOT, Cancelled, ProtocolError, ResponseTimeout, Session,
                   Timing, WakeupTimeout, download_reference, eeprom_loader, loader_blocks, parts, save_image, sha256)
from bootmode import BOOT_FILES, BootSession
from j2534 import BITS, ERR_DEVICE_IN_USE, Driver, DriverError, OpenPort, registered_drivers


SETTINGS = ROOT / 'timing.json'
TIMING_LABELS = {
    'init_p4': '10400 Bd · P4 [ms]', 'init_pause': '10400 Bd · pause between steps [ms]',
    'init_timeout': 'ECU wake-up timeout [ms]',
    'fast_p1': '124800 Bd · P1 [ms]', 'fast_p4': '124800 Bd · P4 [ms]',
    'loader_pause': 'RAM loader block pause [ms]', 'response_timeout': 'Response timeout [ms]',
    'erase_timeout': 'Erase timeout [ms]', 'read_block': 'Read block size [B]',
}
OPERATION_LABELS = {'probe': 'Connection check', 'identify': 'ECU identification', 'test': 'Transfer test',
                    'immostatus': 'Immo III status',
                    'read': 'Reading flash', 'eeprom': 'Reading EEPROM', 'eepromwrite': 'Writing EEPROM', 'write': 'Writing flash',
                    'writetest': 'Write self-test', 'bootread': 'Boot mode flash read',
                    'bootwrite': 'Boot mode flash write', 'booteeprom': 'Boot mode EEPROM read',
                    'booteepromwrite': 'Boot mode EEPROM write'}
BOOT_OPERATIONS = ('bootread', 'bootwrite', 'booteeprom', 'booteepromwrite')
FLASH_OPERATIONS = ('test', 'read', 'write', 'writetest')
LAMP_TICK = 50  # ms
LAMP_COLOURS = {'TX': ('#5a4a12', '#ffd43b'), 'RX': ('#164a26', '#40e070')}  # dark, bright


def run(operation, driver, profile, timing, log, progress, cancelled=lambda: False,
        *, output=None, candidate=None, backup=None, overwrite=False, trace_frames=False, activity=None):
    blocks = loader_blocks() if operation in FLASH_OPERATIONS else None
    eeprom_image = eeprom_loader() if operation in ('eeprom', 'eepromwrite') else None
    log(f'{driver.name} | {driver.path} | {profile}')
    log('Settings: ' + ', '.join(f'{key}={value}' for key, value in asdict(timing).items()))

    def trace(message, data=None):
        if data is not None and activity is not None:
            activity('RX' if 'RX' in message else 'TX')
        if data is None or message.startswith('FAST_INIT') or (trace_frames and operation not in ('eeprom', 'eepromwrite')):
            log(message if data is None else f'{message} {data.hex(" ")}')

    try:
        with OpenPort(driver.path, trace=trace) as port:
            firmware, dll, api = port.info()
            log(f'Firmware {firmware}, driver {dll}, J2534 {api}, pin 16 {port.voltage():.2f} V')
            if operation == 'adapter':
                return 'OpenPort available.'
            if operation in BOOT_OPERATIONS:
                boot = BootSession(port, profile, trace, cancelled)
                progress('Boot mode handshake', 0, 1)
                boot.connect()
                if operation == 'bootwrite':
                    boot.write_flash(candidate, progress)
                    return 'Boot mode: flash written.'
                if operation == 'booteepromwrite':
                    boot.write_eeprom(candidate, progress)
                    return 'Boot mode: EEPROM written.'
                if operation == 'booteeprom':
                    image = boot.read_eeprom(progress)
                    save_image(output, image, overwrite)
                    log(f'EEPROM backup saved to {output}')
                    return f'Boot mode: read {len(image)} B of EEPROM.'
                image = boot.read_flash(progress)
                save_image(output, image, overwrite)
                log(f'Part numbers in BIN: {", ".join(sorted(parts(image))) or "not found"}')
                log(f'BIN saved to {output}')
                return 'Boot mode: read 512 KiB of flash.'
            session = Session(port, profile, timing, trace, cancelled)
            progress('Reading Immo III status' if operation == 'immostatus' else 'Waking ECU', 0, 1)
            try:
                if operation == 'probe':
                    session.probe()
                    return 'ECU diagnostic session verified.'
                if operation == 'immostatus':
                    return session.read_immobilizer_status()
                if operation == 'eeprom':
                    session.start_eeprom(eeprom_image)
                    progress('Reading EEPROM', 0, 2)
                    image = session.read_eeprom(progress)
                    save_image(output, image, overwrite)
                    log(f'EEPROM backup saved to {output}')
                    return f'Read and verified {len(image)} B of EEPROM.'
                if operation == 'eepromwrite':
                    session.start_eeprom(eeprom_image)
                    progress('Reading EEPROM', 0, 1)
                    changed = session.write_eeprom(candidate, progress)
                    return f'EEPROM written: {changed} B changed, content verified.'
                if operation == 'identify':
                    result = session.identify()
                    progress('Reading Bosch SW', 0, 1)
                    try:
                        session.pause(350)
                        session.start(loader_blocks())
                        software = session.read_bosch_software()
                    except (ProtocolError, DriverError, OSError, ValueError) as error:
                        log(f'Could not read Bosch SW: {error}')
                    else:
                        if software:
                            return f'{result}\nSW Bosch: {software}'
                        log('No unambiguous Bosch SW number was found in the flash header.')
                    return result
                session.start(blocks)
                if operation == 'test':
                    session.test_read()
                    result = 'Transfer test finished.'
                elif operation == 'read':
                    image = session.read_flash('Reading flash', progress)
                    session.check_cancel()
                    save_image(output, image, overwrite)
                    ids = ', '.join(sorted(parts(image))) or 'not found'
                    log(f'Part numbers in BIN: {ids}')
                    log(f'BIN saved to {output}')
                    result = 'Read 512 KiB of flash.'
                elif operation == 'writetest':
                    session.write_flash(None, backup, progress, self_test=True)
                    result = 'Write self-test: ECU rewritten with its own content and verified byte by byte.'
                else:
                    session.write_direct(candidate, progress)
                    result = 'Write finished (not verified).'
            except WakeupTimeout as error:
                log(str(error))
                progress('K-line diagnostics', 0, 1)
                echo_ok, slow_reply = session.diagnose_line()
                if slow_reply and operation == 'probe':
                    return f'{slow_reply} Programming FAST_INIT got no reply.'
                if slow_reply:
                    detail = f'{slow_reply} Programming FAST_INIT got no reply.'
                elif echo_ok:
                    detail = ('OpenPort K-line echo passed. The ECU did not reply to the programming '
                              'wake-up or to the slow diagnostic initialization. '
                              'If the ECU stopped replying after a previous operation, power-cycle it before the next attempt.')
                else:
                    detail = 'The K-line frame did not echo back identically on the OpenPort and the ECU did not reply.'
                raise WakeupTimeout(f'{OPERATION_LABELS[operation]} · Could not wake the ECU.\n\n{detail}') from error
            except ResponseTimeout as error:
                raise ResponseTimeout(f'{error}\n\nIf the ECU is stuck, power-cycle it before the next attempt.') from error
            finally:
                try:
                    session.close()
                except Exception as close_error:
                    log(f'Session shutdown: {close_error}')
            return result
    except DriverError as error:
        if error.code == ERR_DEVICE_IN_USE:
            raise RuntimeError(f'{error}. Close other diagnostic programs. If the adapter is not in use there, '
                               'start the application directly from its folder in Windows.') from error
        raise


def work_area(window):
    """Usable rectangle (x, y, width, height) of the monitor that shows the window."""
    try:
        user32 = ctypes.windll.user32
        user32.MonitorFromWindow.restype = ctypes.c_void_p
        user32.MonitorFromWindow.argtypes = (ctypes.c_void_p, ctypes.c_uint32)
        user32.GetMonitorInfoW.argtypes = (ctypes.c_void_p, ctypes.c_void_p)
        # MONITORINFO: cbSize, rcMonitor, rcWork (taskbar excluded), dwFlags.
        info = (ctypes.c_int32 * 10)(40)
        monitor = user32.MonitorFromWindow(window.winfo_id(), 2)  # nearest monitor
        if user32.GetMonitorInfoW(monitor, ctypes.byref(info)):
            left, top, right, bottom = info[5:9]
            return left, top, right - left, bottom - top
    except (AttributeError, OSError):
        pass
    return 0, 0, window.winfo_screenwidth(), window.winfo_screenheight()


def center(window, anchor=None, size=None):
    """Centre the window on the monitor of `anchor` (default: its own monitor)."""
    window.update_idletasks()
    width, height = size or (window.winfo_reqwidth(), window.winfo_reqheight())
    left, top, area_width, area_height = work_area(anchor or window)
    x = left + max(0, (area_width - width) // 2)
    y = top + max(0, (area_height - height) // 2 - 16)  # the title bar sits above the client area
    window.geometry(f'{width}x{height}+{x}+{y}')


class EepromTool(tk.Toplevel):
    def __init__(self, app):
        super().__init__(app.root, background='#1e2229')
        self.app = app
        self.image = None
        self.saved_image = None
        self.path = None
        self.title('EEPROM Tool')
        self.transient(app.root)
        self.protocol('WM_DELETE_WINDOW', self.close)
        self.bind('<Escape>', lambda _: self.close())

        toolbar = ttk.Frame(self, style='Rail.TFrame', padding=(16, 18))
        toolbar.pack(side='left', fill='y')
        ttk.Label(toolbar, text='EEPROM', style='Brand.TLabel').pack(anchor='w')
        ttk.Label(toolbar, text='24C04  /  512 B', style='RailMuted.TLabel').pack(anchor='w')
        ttk.Separator(toolbar).pack(fill='x', pady=16)
        self.actions = (
            ttk.Button(toolbar, text='Open BIN…', command=self.load, style='Accent.TButton'),
            ttk.Button(toolbar, text='Read EEPROM…', command=lambda: app.read('eeprom', parent=self)),
            ttk.Button(toolbar, text='Write EEPROM…', command=lambda: app.write_eeprom('eepromwrite', parent=self)),
            ttk.Button(toolbar, text='Read Immo III status', command=lambda: app.begin('immostatus', parent=self)),
        )
        for button in self.actions:
            button.pack(fill='x', pady=(0, 6))

        panel = ttk.Frame(self, padding=(18, 12))
        panel.pack(fill='both', expand=True)
        ttk.Label(panel, text='EEPROM Tool', style='Title.TLabel').pack(anchor='w', pady=(0, 8))
        self.fields = {}
        self.edit_buttons = {}
        self.bin_labels = ('VIN', 'Immobilizer ID', 'ECU odometer', 'Immo status', 'Login / PIN')
        for title, labels in [('BIN file', ('BIN file', 'SHA-256')), ('EEPROM data', self.bin_labels),
                              ('ECU connection', ('Live Immo III',))]:
            group = ttk.Frame(panel, style='Card.TFrame', padding=(12, 7))
            group.pack(fill='x', pady=(0, 6))
            group.columnconfigure(1, weight=1)
            ttk.Label(group, text=title, style='Heading.TLabel').grid(row=0, column=0, columnspan=3, sticky='w')
            for row, label in enumerate(labels, 1):
                value = self.fields[label] = tk.StringVar(self, value='—')
                ttk.Label(group, text='Path' if label == 'BIN file' else label, width=13,
                          style='CardMuted.TLabel').grid(row=row, column=0, sticky='w')
                ttk.Entry(group, textvariable=value, state='readonly', style='Identification.TEntry',
                          font=('Consolas', 10), width=64 if label == 'SHA-256' else 32).grid(
                    row=row, column=1, sticky='ew', pady=2)
                if label in EEPROM_RECORDS:
                    button = ttk.Button(group, text='Edit…', command=lambda field=label: self.edit_field(field))
                    button.grid(row=row, column=2, padx=(8, 0))
                    self.edit_buttons[label] = button

        editing = ttk.Frame(panel)
        editing.pack(fill='x', pady=(3, 0))
        self.on_button = ttk.Button(editing, text='Immo ON', command=lambda: self.set_immo(True))
        self.off_button = ttk.Button(editing, text='Immo OFF', command=lambda: self.set_immo(False))
        self.undo_button = ttk.Button(editing, text='Undo changes', command=self.undo)
        self.save_button = ttk.Button(editing, text='Save BIN as…', command=self.save, style='Accent.TButton')
        for button in (self.on_button, self.off_button):
            button.pack(side='left', padx=(0, 6))
        for button in (self.save_button, self.undo_button):
            button.pack(side='right', padx=(6, 0))
        self.edit_status = tk.StringVar(self, value='Open a BIN or read an EEPROM backup.')
        ttk.Label(panel, textvariable=self.edit_status, style='Muted.TLabel').pack(anchor='w', pady=(6, 8))

        operation = ttk.Frame(panel)
        operation.pack(side='bottom', fill='x')
        operation.columnconfigure(0, weight=1)
        ttk.Progressbar(operation, variable=app.progress, maximum=100).grid(row=1, column=0, sticky='ew')
        ttk.Label(operation, textvariable=app.status, style='Muted.TLabel', wraplength=480).grid(
            row=0, column=0, sticky='w', pady=(0, 3))
        self.cancel_button = ttk.Button(operation, text='Cancel', command=app.cancelled.set, state='disabled')
        self.cancel_button.grid(row=0, column=1, rowspan=2, padx=(12, 0))
        self.set_busy(app.busy)
        self.update_idletasks()
        self.minsize(self.winfo_reqwidth(), self.winfo_reqheight())
        center(self, anchor=app.root)

    def set_busy(self, busy):
        for button in self.actions:
            button.state(['disabled' if busy else '!disabled'])
        editable = self.image is not None and self.fields['Immo status'].get() != 'Unavailable for this layout'
        for button in self.edit_buttons.values():
            button.state(['!disabled' if editable and not busy else 'disabled'])
        if editable:
            editable = all(self.image[offset] in IMMO_VALUES.values() for offset in IMMO_OFFSETS)
        for button in (self.on_button, self.off_button):
            button.state(['!disabled' if editable and not busy else 'disabled'])
        self.undo_button.state(['!disabled' if self.image != self.saved_image and not busy else 'disabled'])
        self.save_button.state(['!disabled' if self.image is not None and not busy else 'disabled'])

    def display_image(self):
        try:
            fields = eeprom_fields(self.image)
        except ValueError:
            fields = dict.fromkeys(self.bin_labels, 'Unavailable for this layout')
        fields.update({'BIN file': str(self.path), 'SHA-256': sha256(self.image)})
        for label, value in fields.items():
            self.fields[label].set(value)
        modified = self.image != self.saved_image
        self.title('EEPROM Tool' + (' *' if modified else ''))
        self.edit_status.set('Unsaved changes. Save the BIN before writing it to the ECU.' if modified
                             else 'BIN loaded. Changes apply to this file buffer.')
        self.set_busy(self.app.busy)

    def keep_changes(self):
        if self.image == self.saved_image:
            return True
        choice = messagebox.askyesnocancel('EEPROM Tool', 'Save the edited BIN before continuing?', parent=self)
        if choice is None:
            return False
        return self.save() if choice else True

    def close(self):
        if self.keep_changes():
            self.destroy()

    def set_immo(self, enabled):
        if self.app.busy or self.image is None:
            return
        try:
            self.image = set_eeprom_immo(self.image, enabled)
        except ValueError as error:
            messagebox.showerror('EEPROM Tool', str(error), parent=self)
            return
        self.display_image()

    def edit_field(self, label):
        if self.app.busy or self.image is None:
            return
        current = self.fields[label].get()
        if current in ('Copies differ', 'Unavailable', 'Unavailable for this layout'):
            current = ''
        value = simpledialog.askstring('Edit ' + label, label, initialvalue=current, parent=self)
        if value is None:
            return
        try:
            self.image = set_eeprom_field(self.image, label, value)
        except ValueError as error:
            messagebox.showerror('EEPROM Tool', str(error), parent=self)
            return
        self.display_image()

    def undo(self):
        if self.app.busy or self.saved_image is None:
            return
        self.image = self.saved_image
        self.display_image()

    def save(self):
        if self.app.busy or self.image is None:
            return False
        filename = filedialog.asksaveasfilename(
            parent=self, title='Save EEPROM BIN', initialdir=str(self.path.parent),
            initialfile=f'{self.path.stem}_edited.bin',
            defaultextension='.bin', filetypes=[('BIN file', '*.bin'), ('All files', '*.*')])
        if not filename:
            return False
        path = Path(filename)
        try:
            save_image(path, self.image, overwrite=path.exists())
        except OSError as error:
            messagebox.showerror('EEPROM Tool', str(error), parent=self)
            return False
        self.path = path
        self.saved_image = self.image
        self.display_image()
        self.edit_status.set('BIN saved. Use Write EEPROM… to select it for writing.')
        return True

    def load(self, path=None):
        if self.app.busy:
            return
        if path is None:
            filename = filedialog.askopenfilename(parent=self, title='Open EEPROM BIN',
                                                  filetypes=[('BIN file', '*.bin'), ('All files', '*.*')])
            if not filename:
                return
            path = Path(filename)
        try:
            with path.open('rb') as stream:
                image = stream.read(EEPROM_SIZE + 1)
            if len(image) != EEPROM_SIZE:
                raise ValueError(f'EEPROM BIN must be exactly {EEPROM_SIZE} bytes.')
        except (OSError, ValueError) as error:
            messagebox.showerror('EEPROM Tool', str(error), parent=self)
            return
        if not self.keep_changes():
            return
        self.path = path
        self.image = self.saved_image = image
        self.display_image()


class Application:
    def __init__(self, root):
        self.root = root
        self.root.title('EDC15 Tool · OpenPort 2.0')
        center(self.root, size=(980, 780))
        self.root.minsize(860, 680)
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.events = Queue()
        self.busy = False
        self.cancelled = Event()
        self.timing = Timing()
        self.drivers = []
        self.driver = tk.StringVar()
        self.profile = tk.StringVar(value='EDC15P')
        self.status = tk.StringVar(value='Ready')
        self.trace_frames = tk.BooleanVar(value=False)
        self.progress = tk.DoubleVar()
        self.identification = {label: tk.StringVar(value='—') for label in ('Part number', 'Description', 'SW Bosch')}
        self.identification_details = tk.StringVar(value='ECU identification not loaded yet.')
        self.voltage_text = tk.StringVar(value='Voltage: —')
        self.voltage_path = None
        self.port_lock = Lock()
        self.activity = {'TX': 0, 'RX': 0}
        self.seen = dict(self.activity)
        self.lit = {'TX': False, 'RX': False}
        self.widgets = []
        self.eeprom_tool = None
        self.build()
        try:
            if SETTINGS.is_file():
                self.timing = Timing(**json.loads(SETTINGS.read_text(encoding='utf-8')))
        except (OSError, ValueError, TypeError) as error:
            self.log(f'Loading timing: {error}')
        self.refresh_drivers()
        Thread(target=self.watch_voltage, daemon=True).start()
        self.root.after(70, self.poll)
        self.root.after(LAMP_TICK, self.blink)

    def build(self):
        # Slate greys (a slight blue cast instead of pure black) and one calm blue accent.
        background, rail, surface = '#1e2229', '#171a20', '#282d36'
        foreground, muted = '#eef1f5', '#a7b0bd'
        outline = '#3b4250'  # one quiet hairline for every field, tab and panel edge
        accent = '#4f8fe0'   # primary buttons, progress and focus
        self.root.configure(bg=background)
        style = ttk.Style(self.root)
        style.theme_use('clam')
        style.configure('.', background=background, foreground=foreground, font=('Segoe UI', 10))
        for name, color in [('Rail', rail), ('Card', surface)]:
            style.configure(name + '.TFrame', background=color)
            style.configure(name + '.TLabel', background=color, foreground=foreground)
            style.configure(name + 'Muted.TLabel', background=color, foreground=muted)
        style.configure('Brand.TLabel', background=rail, font=('Segoe UI Semibold', 23))
        style.configure('Title.TLabel', font=('Segoe UI Semibold', 19))
        style.configure('Heading.TLabel', background=surface, font=('Segoe UI Semibold', 12))
        style.configure('Muted.TLabel', foreground=muted)
        style.configure('TButton', background='#353c48', padding=(10, 6), relief='flat',
                        bordercolor='#414958', focuscolor='#8fa3bd')
        style.map('TButton', background=[('disabled', '#272c34'), ('pressed', '#4b5464'), ('active', '#404856')],
                  foreground=[('disabled', '#7c8593')])
        style.configure('Accent.TButton', background=accent, foreground='#ffffff', bordercolor=accent,
                        focuscolor='#d6e6fa')
        style.map('Accent.TButton', background=[('disabled', '#272c34'), ('pressed', '#3f7bc8'), ('active', '#63a0ea')],
                  foreground=[('disabled', '#7c8593'), ('!disabled', '#ffffff')])
        style.configure('TEntry', fieldbackground='#191c22', insertcolor=foreground, padding=5,
                        bordercolor=outline, lightcolor='#191c22', darkcolor='#191c22')
        style.map('TEntry', bordercolor=[('focus', accent)],
                  lightcolor=[('readonly', '#20242c')], darkcolor=[('readonly', '#20242c')])
        style.configure('Identification.TEntry', padding=(6, 4))
        style.map('TEntry', fieldbackground=[('disabled', '#20242c'), ('readonly', '#20242c')],
                  foreground=[('disabled', '#7c8593')])
        style.configure('TCombobox', padding=5, arrowcolor=muted, background='#2c323c',
                        bordercolor=outline, lightcolor='#2c323c', darkcolor='#2c323c')
        # The selectors are locked while an operation runs; without these the theme paints them white.
        style.map('TCombobox', bordercolor=[('focus', accent)],
                  arrowcolor=[('disabled', '#5d6673'), ('active', foreground)],
                  background=[('disabled', '#272c34'), ('pressed', '#404856'), ('active', '#353c48')])
        style.map('TCombobox', fieldbackground=[('disabled', '#272c34'), ('readonly', '#2c323c')],
                  foreground=[('disabled', '#7c8593'), ('readonly', foreground)],
                  selectbackground=[('disabled', '#272c34'), ('readonly', '#2c323c')],
                  selectforeground=[('disabled', '#7c8593'), ('readonly', foreground)])
        # clam leaves the four corner pixels of a field unpainted, so the widget background shows there.
        # Its built-in map turns that background light for read-only fields (bright corner dots) and
        # for a hovered selector button, so both are pinned to dark colours here.
        style.configure('TEntry', background=outline)
        style.map('TEntry', background=[('readonly', outline), ('disabled', outline)])
        self.root.option_add('*TCombobox*Listbox.background', surface)
        self.root.option_add('*TCombobox*Listbox.foreground', foreground)
        self.root.option_add('*TCombobox*Listbox.selectBackground', '#3b4250')
        # Flat tabs of equal size: the selected one takes the panel colour, the other stays in the window.
        style.configure('TNotebook', background=background, bordercolor=outline, lightcolor=surface,
                        darkcolor=surface, tabmargins=(0, 0, 0, 0))
        style.configure('TNotebook.Tab', padding=(16, 6), background=background, foreground=muted,
                        bordercolor=outline, lightcolor=background, darkcolor=background, focuscolor=surface)
        style.map('TNotebook.Tab', background=[('selected', surface), ('active', '#242931')],
                  foreground=[('selected', foreground)], lightcolor=[('selected', surface)],
                  darkcolor=[('selected', surface)], padding=[('selected', (16, 6))],
                  expand=[('selected', (0, 0, 0, 0))])
        style.configure('Horizontal.TProgressbar', background=accent, troughcolor='#323843',
                        bordercolor='#323843', lightcolor=accent, darkcolor=accent, thickness=7)
        style.configure('TCheckbutton', foreground=muted, indicatorbackground='#191c22',
                        indicatorforeground=foreground, upperbordercolor=outline, lowerbordercolor=outline)
        style.map('TCheckbutton', background=[('active', background)],
                  indicatorbackground=[('active', '#2c323c')])
        style.configure('Vertical.TScrollbar', background='#434b59', troughcolor=rail, bordercolor=rail,
                        lightcolor='#434b59', darkcolor='#434b59', arrowcolor=muted)
        style.map('Vertical.TScrollbar', background=[('disabled', rail), ('active', '#535d6d')],
                  arrowcolor=[('disabled', rail)], bordercolor=[('disabled', rail)],
                  lightcolor=[('disabled', rail)], darkcolor=[('disabled', rail)])
        style.configure('TSeparator', background='#353b47')

        def action(parent, text, command, style_name='TButton'):
            button = ttk.Button(parent, text=text, command=command, style=style_name)
            self.widgets.append(button)
            return button

        sidebar = ttk.Frame(self.root, style='Rail.TFrame', padding=(16, 18))
        sidebar.pack(side='left', fill='y')
        ttk.Label(sidebar, text='EDC15 Tool', style='Brand.TLabel').pack(anchor='w')
        ttk.Label(sidebar, text='OpenPort 2.0  /  K-line', style='RailMuted.TLabel').pack(anchor='w')
        ttk.Separator(sidebar).pack(fill='x', pady=18)
        ttk.Label(sidebar, text=f'J2534 driver ({BITS}-bit)', style='RailMuted.TLabel').pack(anchor='w')
        self.driver_combo = ttk.Combobox(sidebar, textvariable=self.driver, state='readonly', width=22)
        self.driver_combo.pack(fill='x', pady=(4, 12))
        ttk.Label(sidebar, text='ECU profile', style='RailMuted.TLabel').pack(anchor='w')
        self.profile_combo = ttk.Combobox(sidebar, textvariable=self.profile, values=tuple(KEY_WORDS),
                                          state='readonly', width=22)
        self.profile_combo.pack(fill='x', pady=(4, 10))
        settings = ttk.Frame(sidebar, style='Rail.TFrame')
        settings.pack(fill='x')
        action(settings, 'Refresh', self.refresh_drivers).pack(side='left', expand=True, fill='x')
        action(settings, 'Timing…', self.edit_timing).pack(side='left', expand=True, fill='x', padx=(5, 0))

        telemetry = ttk.Frame(sidebar, style='Rail.TFrame')
        telemetry.pack(side='bottom', fill='x')
        ttk.Separator(telemetry).pack(fill='x', pady=12)
        voltage = ttk.Frame(telemetry, style='Rail.TFrame')
        voltage.pack(fill='x')
        self.voltage_lamp = tk.Canvas(voltage, width=14, height=14, bg=rail, highlightthickness=0)
        self.voltage_lamp.pack(side='left')
        self.voltage_dot = self.voltage_lamp.create_oval(2, 2, 12, 12, fill='#e5484d', outline='')
        ttk.Label(voltage, textvariable=self.voltage_text, style='Rail.TLabel').pack(side='left', padx=6)
        traffic = ttk.Frame(telemetry, style='Rail.TFrame')
        traffic.pack(fill='x', pady=(6, 0))
        self.lamps = {}
        for name in ('RX', 'TX'):
            lamp = tk.Canvas(traffic, width=14, height=14, bg=rail, highlightthickness=0)
            lamp.pack(side='left')
            dark, bright = LAMP_COLOURS[name]
            self.lamps[name] = (lamp, lamp.create_oval(2, 2, 12, 12, fill=dark, outline=''), dark, bright)
            ttk.Label(traffic, text=name, style='RailMuted.TLabel').pack(side='left', padx=(6, 16))

        pane = ttk.Frame(self.root, padding=(18, 12))
        pane.pack(fill='both', expand=True)
        ttk.Label(pane, text='ECU workspace', style='Title.TLabel').pack(anchor='w', pady=(0, 7))
        ecu = ttk.Frame(pane, style='Card.TFrame', padding=(12, 9))
        ecu.pack(fill='x')
        for column, (label, value) in enumerate(self.identification.items()):
            ecu.columnconfigure(column, weight=2 if label == 'Description' else 1)
            ttk.Label(ecu, text=label, style='CardMuted.TLabel').grid(row=0, column=column, sticky='w')
            ttk.Entry(ecu, textvariable=value, state='readonly', style='Identification.TEntry',
                      width=8).grid(row=1, column=column, sticky='ew', padx=(0, 8))
        action(ecu, 'Identify ECU', lambda: self.begin('identify')).grid(row=1, column=3)
        details = ttk.Label(ecu, textvariable=self.identification_details, style='CardMuted.TLabel')
        details.grid(row=2, column=0, columnspan=4, sticky='w', pady=(6, 0))
        ecu.bind('<Configure>', lambda event: details.configure(wraplength=max(100, event.width - 26)))

        modes = ttk.Notebook(pane)
        modes.pack(fill='x', pady=(12, 0))
        def mode(title):
            page = ttk.Frame(modes, style='Card.TFrame', padding=(12, 6))
            page.columnconfigure(0, weight=1)
            modes.add(page, text=title)
            return page

        def memory_row(page, row, label, detail, commands):
            ttk.Label(page, text=label, style='Heading.TLabel').grid(row=row, column=0, sticky='sw')
            ttk.Label(page, text=detail, style='CardMuted.TLabel').grid(row=row+1, column=0, sticky='nw')
            for column, (caption, callback) in enumerate(commands, 1):
                action(page, caption, callback, 'Accent.TButton' if column == 1 else 'TButton').grid(
                    row=row, rowspan=2, column=column, sticky='ew', padx=(6, 0), pady=6)
            if row == 0:
                ttk.Separator(page).grid(row=2, column=0, columnspan=4, sticky='ew', pady=4)

        normal = mode('K-line')
        memory_row(normal, 0, 'Flash', '29F400BT · 512 KiB', [('Read Flash', self.read), ('Write Flash', self.write),
                                                  ('Checksum…', self.check_flash_bin)])
        memory_row(normal, 3, 'EEPROM', '24C04 · 512 B', [('Read EEPROM…', lambda: self.read('eeprom')),
                   ('Write EEPROM…', lambda: self.write_eeprom('eepromwrite')), ('EEPROM Tool…', self.open_eeprom_tool)])
        ttk.Label(normal, text='KWP2000 RAM loader · 124800 Bd   |   Power the ECU up normally.',
                  style='CardMuted.TLabel').grid(row=5, column=0, columnspan=4, sticky='w', pady=(6, 2))
        boot = mode('Boot mode')
        memory_row(boot, 0, 'Flash', '29F400BT · 512 KiB', [('Read Flash…', lambda: self.read('bootread')),
                                                 ('Write Flash', self.boot_write)])
        memory_row(boot, 3, 'EEPROM', '24C04 · 512 B', [('Read EEPROM…', lambda: self.read('booteeprom')),
                   ('Write EEPROM…', lambda: self.write_eeprom('booteepromwrite'))])
        ttk.Label(boot, text='C167 BSL · 28800 Bd   |   Power the ECU up with the boot pin grounded.', style='CardMuted.TLabel').grid(
            row=5, column=0, columnspan=3, sticky='w', pady=(6, 2))

        operation = ttk.Frame(pane)
        operation.pack(fill='x', pady=(10, 8))
        operation.columnconfigure(0, weight=1)
        status = ttk.Label(operation, textvariable=self.status)
        status.grid(row=0, column=0, sticky='w')
        operation.bind('<Configure>', lambda event: status.configure(wraplength=max(100, event.width - 120)))
        ttk.Progressbar(operation, variable=self.progress, maximum=100).grid(row=1, column=0, sticky='ew')
        self.stop_button = ttk.Button(operation, text='Cancel', command=self.cancelled.set, state='disabled')
        self.stop_button.grid(row=0, column=1, rowspan=2, padx=(10, 0))

        journal = ttk.Frame(pane)
        journal.pack(fill='x', pady=(6, 4))
        ttk.Label(journal, text='Activity log', font=('Segoe UI Semibold', 12)).pack(side='left')
        frame_check = ttk.Checkbutton(journal, text='Log TX/RX frames', variable=self.trace_frames)
        frame_check.pack(side='right')
        self.widgets.append(frame_check)
        log_panel = ttk.Frame(pane)
        log_panel.pack(fill='both', expand=True)
        self.log_text = tk.Text(log_panel, bg=rail, fg='#d7dce3', wrap='word', height=6,
                                font=('Consolas', 10), padx=12, pady=9, relief='flat', state='disabled')
        scrollbar = ttk.Scrollbar(log_panel, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        self.log_text.pack(side='left', fill='both', expand=True)
        self.root.bind('<Control-a>', self.select_log)

    def count_activity(self, direction):
        # Called from the worker thread for every frame; the GUI samples the counters.
        self.activity[direction] += 1

    def blink(self):
        for name, (lamp, dot, dark, bright) in self.lamps.items():
            active = self.activity[name] != self.seen[name]
            self.seen[name] = self.activity[name]
            # Continuous traffic alternates bright and dark so the lamp visibly flashes.
            lit = active and not self.lit[name]
            if lit != self.lit[name]:
                lamp.itemconfigure(dot, fill=bright if lit else dark)
                self.lit[name] = lit
        self.root.after(LAMP_TICK, self.blink)

    def show_voltage(self, voltage):
        # Red: no reading or below 11 V. Green: 12.2–14.8 V. Yellow: anything else.
        if voltage is None:
            colour, text = '#e5484d', 'Voltage: —'
        else:
            text = f'Voltage: {voltage:.2f} V'
            colour = '#e5484d' if voltage < 11 else '#35c759' if 12.2 <= voltage <= 14.8 else '#f2c81f'
        self.voltage_lamp.itemconfigure(self.voltage_dot, fill=colour)
        self.voltage_text.set(text)

    def watch_voltage(self):
        # The adapter is opened only briefly and under the lock so it never collides with operations.
        while True:
            path = self.voltage_path
            if path is not None and self.port_lock.acquire(blocking=False):
                try:
                    with OpenPort(path) as port:
                        voltage = port.voltage()
                except Exception:
                    voltage = None
                finally:
                    self.port_lock.release()
                if self.voltage_path is not None:
                    self.events.put(('voltage', voltage))
            time.sleep(1.5)

    def select_log(self, event):
        if event.widget == self.log_text:
            self.log_text.tag_add('sel', '1.0', 'end-1c')
            return 'break'

    def refresh_drivers(self):
        previous = self.driver.get()
        self.drivers = registered_drivers()
        labels = [driver.name for driver in self.drivers]
        self.driver_combo.configure(values=labels)
        if labels:
            self.driver.set(previous if previous in labels else labels[0])
        else:
            self.driver.set('')
            self.log(f'No {BITS}-bit OpenPort J2534 04.04 driver found.')

    def show_identification(self, result):
        rows = [line.split(': ', 1) for line in result.splitlines()]
        fields = dict(rows)
        if 'SW Bosch' in fields:
            fields['SW Bosch'] = fields['SW Bosch'][-6:]
        for label, variable in self.identification.items():
            variable.set(fields.get(label, '—'))
        has_core_fields = bool(self.identification.keys() & fields.keys())
        self.identification_details.set('  ·  '.join(f'{label} {value}' for label, value in rows
                                                     if label not in self.identification
                                                     and (label != 'Other data' or not has_core_fields)))

    def edit_timing(self):
        window = tk.Toplevel(self.root, background='#1e2229')
        window.title('K-line timing')
        window.transient(self.root)
        window.grab_set()
        panel = ttk.Frame(window, padding=16)
        panel.pack(fill='both', expand=True)
        ttk.Label(panel, text='K-line timing', style='Title.TLabel').grid(
            row=0, column=0, columnspan=2, sticky='w', pady=(0, 8))
        form = ttk.Frame(panel, style='Card.TFrame', padding=(12, 8))
        form.grid(row=1, column=0, columnspan=2, sticky='ew')
        entries = {}
        for row, (key, label) in enumerate(TIMING_LABELS.items()):
            value = entries[key] = tk.StringVar(value=str(getattr(self.timing, key)))
            ttk.Label(form, text=label, style='Card.TLabel').grid(row=row, column=0, sticky='w', pady=4, padx=(0, 15))
            ttk.Entry(form, textvariable=value, width=9).grid(row=row, column=1, sticky='ew', pady=4)

        def save():
            try:
                timing = Timing(**{key: int(value.get()) for key, value in entries.items()})
                save_image(SETTINGS, json.dumps(asdict(timing), indent=2).encode('utf-8'), overwrite=True)
            except (ValueError, OSError) as error:
                messagebox.showerror('Timing', str(error), parent=window)
                return
            self.timing = timing
            self.log('Timing saved.')
            window.destroy()

        def defaults():
            for key, value in asdict(Timing()).items():
                entries[key].set(str(value))

        ttk.Button(panel, text='Defaults', command=defaults).grid(row=2, column=0, sticky='w', pady=(10, 0))
        ttk.Button(panel, text='Save', command=save, style='Accent.TButton').grid(row=2, column=1, sticky='e', pady=(10, 0))
        window.bind('<Escape>', lambda _: window.destroy())
        center(window, anchor=self.root)

    def open_eeprom_tool(self):
        if self.eeprom_tool is None or not self.eeprom_tool.winfo_exists():
            self.eeprom_tool = EepromTool(self)
        self.eeprom_tool.deiconify()
        self.eeprom_tool.lift()
        self.eeprom_tool.focus_set()

    def read(self, operation='read', *, parent=None):
        title, filename = {'eeprom': ('Save EEPROM backup', 'EDC15_EEPROM.bin'),
                           'bootread': ('Save flash read in boot mode', 'EDC15_flash_boot.bin'),
                           'booteeprom': ('Save EEPROM read in boot mode', 'EDC15_EEPROM_boot.bin')}.get(
            operation, ('Save complete flash', 'EDC15_flash.bin'))
        chosen = filedialog.asksaveasfilename(parent=parent or self.root, title=title,
                                               initialfile=filename, defaultextension='.bin',
                                               filetypes=[('BIN file', '*.bin'), ('All files', '*.*')])
        if chosen:
            path = Path(chosen)
            self.begin(operation, parent=parent, output=path, overwrite=path.exists())

    def choose_flash_image(self, title):
        filename = filedialog.askopenfilename(parent=self.root, title='Select a 512 KiB BIN to write',
                                               filetypes=[('BIN file', '*.bin'), ('All files', '*.*')])
        if not filename:
            return None
        path = Path(filename)
        try:
            image = path.read_bytes()
        except OSError as error:
            messagebox.showerror(title, str(error), parent=self.root)
            return None
        # The only requirement: a write always replaces the full 512 KiB.
        if len(image) != FLASH_SIZE:
            messagebox.showerror(title, f'BIN must be exactly {FLASH_SIZE:,} bytes, '
                                        f'the file has {len(image):,}.', parent=self.root)
            return None
        self.log(f'BIN to write: {path} · SHA-256 {sha256(image)}')
        return image

    def write(self, operation='write'):
        image = self.choose_flash_image(OPERATION_LABELS[operation])
        if image is not None:
            self.begin(operation, candidate=image)

    def boot_write(self):
        self.write('bootwrite')

    def check_flash_bin(self):
        title = 'Flash checksum'
        filename = filedialog.askopenfilename(parent=self.root, title=title,
                                               filetypes=[('BIN file', '*.bin'), ('All files', '*.*')])
        if not filename:
            return
        path = Path(filename)
        try:
            report, mismatch = check_flash(path.read_bytes())
        except (OSError, ValueError) as error:
            messagebox.showerror(title, str(error), parent=self.root)
            return
        show_result = messagebox.showwarning if mismatch else messagebox.showinfo
        show_result(title, report, detail=path.name, parent=self.root)

    def write_eeprom(self, operation, *, parent=None):
        parent = parent or self.root
        title = OPERATION_LABELS[operation]
        filename = filedialog.askopenfilename(parent=parent, title='Select a 512 B EEPROM image to write',
                                               filetypes=[('BIN file', '*.bin'), ('All files', '*.*')])
        if not filename:
            return
        path = Path(filename)
        try:
            image = path.read_bytes()
        except OSError as error:
            messagebox.showerror(title, str(error), parent=parent)
            return
        if len(image) != EEPROM_SIZE:
            messagebox.showerror(title, f'The EEPROM image must be exactly {EEPROM_SIZE} bytes, '
                                        f'the file has {len(image):,}.', parent=parent)
            return
        self.log(f'EEPROM image: {path} · SHA-256 {sha256(image)}')
        self.begin(operation, parent=parent, candidate=image)

    def begin(self, operation, *, parent=None, **kwargs):
        if self.busy:
            return
        if operation == 'identify':
            for variable in self.identification.values():
                variable.set('—')
            self.identification_details.set('Loading ECU identification…')
        index = self.driver_combo.current()
        if index < 0:
            error = f'Select a {BITS}-bit J2534 driver.'
            if operation == 'identify':
                self.status.set(error)
                self.identification_details.set(error)
                self.log(error)
            else:
                messagebox.showerror('OpenPort', error, parent=parent or self.root)
            return
        if operation in FLASH_OPERATIONS and not REFERENCE.is_file() and not self.fetch_loader(parent or self.root):
            return
        missing = [path for path in BOOT_FILES if not path.is_file()] if operation in BOOT_OPERATIONS else []
        if missing:
            messagebox.showerror('Boot mode', 'Boot loader file is missing\n' + '\n'.join(map(str, missing)),
                                 parent=self.root)
            return
        self.cancelled.clear()
        self.progress.set(0)
        self.status.set('Connecting…')
        if operation == 'immostatus' and self.eeprom_tool is not None:
            self.eeprom_tool.fields['Live Immo III'].set('—')
        self.set_busy(True)
        args = (operation, self.drivers[index], self.profile.get(), self.timing)
        Thread(target=self.worker, args=args, kwargs={'trace_frames': self.trace_frames.get(), **kwargs},
               daemon=True).start()

    def fetch_loader(self, parent):
        """Offer to download the flash RAM loader source, which is not bundled. True once it is in place."""
        if not messagebox.askyesno(
                'RAM loader', 'Flash access over OBD needs the RAM loader from the ecu-tool project. It is not '
                'bundled with EDC15 Tool.\n\nDownload it now from github.com/fjvva/ecu-tool (45 KB)? '
                'The file is checked by SHA-256 and saved to the reference folder.', parent=parent):
            return False
        try:
            download_reference()
        except (OSError, ValueError) as error:
            messagebox.showerror('RAM loader', f'The download failed.\n\n{error}', parent=parent)
            return False
        self.log(f'RAM loader source saved to {REFERENCE}')
        return True

    def worker(self, operation, *args, **kwargs):
        log_path = ROOT / 'logs' / f'{datetime.now():%Y%m%d-%H%M%S-%f}.log'
        failure_event = 'identify_incomplete' if operation == 'identify' else 'error'
        last_update = 0.0
        last_stage = ''

        def progress(stage, done, total):
            nonlocal last_update, last_stage
            now = time.monotonic()
            if stage != last_stage or now - last_update >= 0.12 or done == total:
                self.events.put(('progress', stage, done, total))
                last_update, last_stage = now, stage

        try:
            log_path.parent.mkdir(exist_ok=True)
            with log_path.open('x', encoding='utf-8', buffering=1) as stream:
                def log(line):
                    formatted = f'{datetime.now():%H:%M:%S.%f}'[:-3] + f'  {line}'
                    stream.write(formatted + '\n')
                    if len(line) < 180 or not line.startswith(('TX ', 'RX ')):
                        self.events.put(('log', formatted))

                log(f'Log file: {log_path}')
                try:
                    with self.port_lock:
                        result = run(operation, *args, log, progress, self.cancelled.is_set,
                                     activity=self.count_activity, **kwargs)
                    for line in result.splitlines():
                        log(line)
                    if operation == 'eeprom':
                        self.events.put(('eeprom_saved', result, kwargs['output']))
                    elif operation == 'immostatus':
                        self.events.put(('immo_status', result))
                    else:
                        self.events.put(('identified' if operation == 'identify' else 'done', result))
                except Cancelled as error:
                    log(str(error))
                    self.events.put((failure_event if operation == 'identify' else 'cancelled', str(error)))
                except Exception as error:
                    log(str(error))
                    self.events.put((failure_event, str(error)))
        except OSError as error:
            self.events.put((failure_event, f'Log file: {error}'))

    def set_busy(self, busy):
        self.busy = busy
        for widget in self.widgets:
            widget.state(['disabled' if busy else '!disabled'])
        if self.eeprom_tool is not None and self.eeprom_tool.winfo_exists():
            self.eeprom_tool.set_busy(busy)
        for combo in (self.driver_combo, self.profile_combo):
            combo.configure(state='disabled' if busy else 'readonly')
        self.set_cancel_enabled(busy)

    def set_cancel_enabled(self, enabled):
        state = 'normal' if enabled else 'disabled'
        self.stop_button.configure(state=state)
        if self.eeprom_tool is not None and self.eeprom_tool.winfo_exists():
            self.eeprom_tool.cancel_button.configure(state=state)

    def log(self, line):
        self.log_text.configure(state='normal')
        self.log_text.insert('end', line + '\n')
        line_count = int(self.log_text.index('end-1c').split('.')[0])
        if line_count > 1200:
            self.log_text.delete('1.0', f'{line_count - 1100}.0')
        self.log_text.see('end')
        self.log_text.configure(state='disabled')

    def poll(self):
        index = self.driver_combo.current()
        self.voltage_path = None if self.busy or index < 0 else self.drivers[index].path
        for _ in range(150):
            try:
                kind, *args = self.events.get_nowait()
            except Empty:
                break
            if kind == 'log':
                self.log(args[0])
            elif kind == 'voltage':
                self.show_voltage(args[0])
            elif kind == 'progress':
                stage, done, total = args
                percent = 100 * done / total
                self.progress.set(percent)
                self.status.set(f'{stage} · {percent:.1f} %')
                if stage in ('Erasing flash', 'Writing EEPROM'):
                    self.set_cancel_enabled(False)
            elif kind == 'identified':
                self.set_busy(False)
                self.progress.set(100)
                self.status.set('ECU identification loaded.')
                self.show_identification(args[0])
            elif kind == 'identify_incomplete':
                self.set_busy(False)
                self.status.set('ECU identification incomplete.')
                self.identification_details.set(args[0].split('\n', 1)[0])
            elif kind == 'immo_status':
                self.set_busy(False)
                self.progress.set(100)
                self.status.set('Immo III status read.')
                if self.eeprom_tool is not None and self.eeprom_tool.winfo_exists():
                    self.eeprom_tool.fields['Live Immo III'].set(args[0])
            else:
                self.set_busy(False)
                self.status.set(args[0] if kind != 'error' else 'Operation failed')
                if kind in ('done', 'eeprom_saved'):
                    self.progress.set(100)
                    if kind == 'eeprom_saved' and self.eeprom_tool is not None and self.eeprom_tool.winfo_exists():
                        self.eeprom_tool.load(args[1])
                elif kind == 'error':
                    messagebox.showerror('EDC15 Tool', args[0], parent=self.root)
        self.root.after(70, self.poll)

    def close(self):
        if self.busy:
            self.root.bell()
        else:
            if self.eeprom_tool is not None and self.eeprom_tool.winfo_exists():
                if not self.eeprom_tool.keep_changes():
                    return
            self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_mutually_exclusive_group()
    actions.add_argument('--fetch-loader', action='store_true',
                         help='download the flash RAM loader source from the ecu-tool project')
    for operation in ('adapter', 'probe', 'identify', 'test'):
        actions.add_argument(f'--{operation}', dest='operation', action='store_const', const=operation)
    actions.add_argument('--read', type=Path, metavar='BIN')
    actions.add_argument('--read-eeprom', type=Path, metavar='BIN')
    actions.add_argument('--write-eeprom', type=Path, metavar='BIN')
    actions.add_argument('--inspect-eeprom', type=Path, metavar='BIN')
    actions.add_argument('--check-flash', type=Path, metavar='BIN')
    actions.add_argument('--writetest', type=Path, metavar='BACKUP')
    actions.add_argument('--boot-read', type=Path, metavar='BIN')
    actions.add_argument('--boot-write', type=Path, metavar='BIN')
    actions.add_argument('--boot-read-eeprom', type=Path, metavar='BIN')
    actions.add_argument('--boot-write-eeprom', type=Path, metavar='BIN')
    parser.add_argument('--driver', type=Path, metavar='DLL')
    parser.add_argument('--profile', choices=KEY_WORDS, default='EDC15P')
    parser.add_argument('--trace', action='store_true')
    args = parser.parse_args()
    if args.fetch_loader:
        try:
            print(f'RAM loader source saved to {download_reference()}')
            return 0
        except (OSError, ValueError) as error:
            parser.exit(1, f'{error}\n')
    if offline_bin := args.inspect_eeprom or args.check_flash:
        try:
            image = offline_bin.read_bytes()
            report, mismatch = check_flash(image) if args.check_flash else (inspect_eeprom(image), False)
            print(report)
            return int(mismatch)
        except (OSError, ValueError) as error:
            parser.exit(1, f'{error}\n')

    boot = {'bootread': args.boot_read, 'bootwrite': args.boot_write, 'booteeprom': args.boot_read_eeprom,
            'booteepromwrite': args.boot_write_eeprom, 'eepromwrite': args.write_eeprom}
    if not (args.operation or args.read or args.read_eeprom or args.writetest or any(boot.values())):
        root = tk.Tk()
        Application(root)
        root.mainloop()
        return 0
    drivers = registered_drivers() if args.driver is None else [Driver('Custom J2534', args.driver)]
    if not drivers:
        parser.error(f'No {BITS}-bit OpenPort driver was found.')
    try:
        timing = Timing(**json.loads(SETTINGS.read_text(encoding='utf-8'))) if SETTINGS.is_file() else Timing()
        if args.read_eeprom:
            operation, output = 'eeprom', args.read_eeprom
        elif any(boot.values()):
            operation, output = next((name, path) for name, path in boot.items() if path)
        else:
            operation = args.operation or ('writetest' if args.writetest else 'read')
            output = args.read
        candidate = None
        if operation in ('bootwrite', 'booteepromwrite', 'eepromwrite'):
            candidate, output = output.read_bytes(), None
        result = run(operation, drivers[0], args.profile, timing, lambda line: print(line, flush=True),
                     lambda *_: None, output=output, candidate=candidate, backup=args.writetest,
                     trace_frames=args.trace)
        print(result)
        return 0
    except (RuntimeError, OSError, ValueError, TypeError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
