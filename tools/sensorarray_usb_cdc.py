"""Exact-role Linux USB CDC transport; no libusb or administrator dependency.

CDC0 on development 303a:4042 is the formal record stream. CDC1 is debug.
Selection uses read-only sysfs metadata before and after one ordinary tty open;
it never guesses a role from tty numbering or detaches/reconfigures USB.
"""
from pathlib import Path
import re

USB_VID, USB_PID = 0x303A, 0x4042


class UsbCdcError(OSError):
    pass


def find_cdc_tty(serial, *, vid=USB_VID, pid=USB_PID, interface_number=0,
                 sys_tty=Path('/sys/class/tty'), dev_root=Path('/dev')):
    if not isinstance(serial, str) or not re.fullmatch(r'[0-9A-Fa-f]{12}', serial):
        raise ValueError('USB serial must be twelve MAC hex digits without colons')
    if vid != USB_VID or pid not in {0x4041, USB_PID} or interface_number not in {0, 2}:
        raise ValueError('unsupported_USB_CDC_role_identity')
    if interface_number == 0 and pid != USB_PID:
        raise ValueError('formal_CDC_requires_303a4042_interface0')
    matches = []
    for tty in sorted(sys_tty.glob('ttyACM*')):
        try:
            path = (tty / 'device').resolve(strict=True)
            ancestors = (path, *path.parents)
            interface = next((p for p in ancestors if (p / 'bInterfaceNumber').is_file()), None)
            device = next((p for p in ancestors if (p / 'idVendor').is_file()), None)
            if interface is None or device is None: continue
            def read(base, field): return (base / field).read_text().strip()
            if (read(device, 'idVendor').lower(), read(device, 'idProduct').lower(), read(device, 'serial').upper()) != (
                    f'{vid:04x}', f'{pid:04x}', serial.upper()):
                continue
            descriptor = tuple(read(interface, key).lower() for key in
                               ('bInterfaceNumber', 'bInterfaceClass', 'bInterfaceSubClass', 'bInterfaceProtocol'))
            if descriptor != (f'{interface_number:02x}', '02', '02', '00'): continue
            matches.append({'path': str(dev_root / tty.name), 'sysfs_device': str(device),
                            'sysfs_interface': str(interface), 'vid': f'{vid:04x}', 'pid': f'{pid:04x}',
                            'usb_serial': serial.upper(), 'interface_number': interface_number,
                            'interface_class': 2, 'interface_subclass': 2, 'interface_protocol': 0,
                            'speed_mbps_reported_by_sysfs': read(device, 'speed') if (device / 'speed').is_file() else None})
        except FileNotFoundError:
            continue  # Enumeration/removal race: never open an unverified tty.
    if len(matches) != 1:
        raise UsbCdcError('USB_CDC_role_not_enumerated' if not matches else 'USB_CDC_role_ambiguous_serial')
    return matches[0]


class UsbCdcPort:
    transport = 'usb_cdc'

    def __init__(self, serial, *, vid=USB_VID, pid=USB_PID, metadata_finder=find_cdc_tty, port_factory=None):
        if vid != USB_VID or pid != USB_PID:
            raise ValueError('formal_CDC_requires_303a4042_interface0')
        if port_factory is None:
            from capture_reference_fixture import PosixPort
            port_factory = PosixPort
        self._port = None
        self._metadata = metadata_finder(serial, vid=vid, pid=pid, interface_number=0)
        try:
            self._port = port_factory(self._metadata['path'])
            if metadata_finder(serial, vid=vid, pid=pid, interface_number=0) != self._metadata:
                raise UsbCdcError('USB_CDC_identity_changed_during_open')
        except BaseException as original:
            try: self.close()
            except Exception as cleanup:
                if hasattr(original, 'add_note'): original.add_note('CDC cleanup: ' + str(cleanup))
            raise

    def metadata(self):
        return {'transport': self.transport, 'capability': 'usb_cdc_data_v1',
                **self._metadata, 'vid': USB_VID, 'pid': USB_PID,
                'role': 'formal_DATA_CTRL_shared_record_stream', 'debug_interface_used': False,
                'libusb_used': False, 'modem_bit_ioctl': False, 'reset_requested': False,
                'input_flushed': False, 'configuration_changed': False,
                'ownership': 'one_tty_reader_flock_TIOCEXCL', 'existing_readers': 'unverified',
                'baud_setting': 115200, 'baud_setting_is_not_USB_wire_rate': True,
                'os_CDC_open_line_state': 'OS_controlled_not_claimed_side_effect_free',
                'tx_acceptance': 'OS_tty_write_not_firmware_command_applied_or_PC_receipt'}

    def read_some(self, max_bytes, deadline):
        if self._port is None: raise UsbCdcError('USB_CDC_closed')
        return self._port.read_some(max_bytes, deadline)

    def write_some(self, data, deadline):
        if self._port is None: raise UsbCdcError('USB_CDC_closed')
        return self._port.write_some(data, deadline)

    def close(self):
        port, self._port = self._port, None
        if port is not None: port.close()
