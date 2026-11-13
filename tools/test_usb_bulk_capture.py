#!/usr/bin/env python3
"""Formal dual-interface capture regressions using synthetic USB boundaries.

Never enumerates/opens USB hardware. Encoder fixtures are existing unmodified
production-encoder outputs; the runner scenario separately uses synthetic data.
"""
from collections import deque
import contextlib
import ctypes
from array import array
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest

from capture_reference_fixture import CaptureOptions, Collector, CaptureError, validate_transport_options
from run_firmware_benchmark import BenchmarkCollector, BenchmarkOptions, Configuration, run_benchmark
from sensorarray_usb_bulk import UsbBulkPort, UsbBulkError, ROLES, USB_PID
from test_firmware_benchmark import Firmware
from text_protocol import TextProtocolParser, parse_fields

ROOT = Path(__file__).resolve().parents[1]
SERIAL = "80B54ECDD9AC"


class FakeUSBError(OSError):
    def __init__(self, code):
        super().__init__("synthetic_USB_error:" + str(code))
        self.backend_error_code = code


class Interface:
    def __init__(self, role):
        spec = ROLES[role]
        self.bInterfaceNumber, self.bAlternateSetting = spec["interface"], 0
        self.bInterfaceClass, self.bInterfaceSubClass, self.bInterfaceProtocol = 0xFF, 0x53, spec["protocol"]
        self.endpoints = [SimpleNamespace(bEndpointAddress=spec[direction], bmAttributes=2, wMaxPacketSize=64)
                          for direction in ("in", "out")]

    def __iter__(self):
        return iter(self.endpoints)


class Config:
    bConfigurationValue = 1

    def __init__(self):
        self.interfaces = {(0, 0): Interface("data"), (1, 0): Interface("ctrl")}

    def __getitem__(self, key):
        return self.interfaces[key]


class Device:
    bus, address = 3, 10

    def __init__(self, *, serial=SERIAL):
        self.serial_number = serial
        self.usb_configuration = Config()
        self.incoming = {0x81: deque(), 0x82: deque()}
        self.write_plan, self.write_calls, self.read_calls = deque(), [], []
        self.written = bytearray()
        self.active_drivers = set()

    def get_active_configuration(self):
        return self.usb_configuration

    def is_kernel_driver_active(self, number):
        return number in self.active_drivers

    def read(self, endpoint, size, timeout):
        self.read_calls.append((endpoint, size, timeout))
        self.assert_endpoint(endpoint, {0x81, 0x82})
        if not self.incoming[endpoint]:
            raise FakeUSBError(-7)
        item = self.incoming[endpoint].popleft()
        if isinstance(item, BaseException):
            raise item
        if len(item) > size:
            self.incoming[endpoint].appendleft(item[size:])
        return item[:size]

    def write(self, endpoint, data, timeout):
        self.assert_endpoint(endpoint, {0x02})
        self.write_calls.append((endpoint, bytes(data), timeout))
        count = self.write_plan.popleft() if self.write_plan else len(data)
        if isinstance(count, BaseException):
            raise count
        if 0 <= count <= len(data):
            self.written.extend(data[:count])
        return count

    @staticmethod
    def assert_endpoint(actual, allowed):
        if actual not in allowed:
            raise AssertionError(f"unexpected endpoint {actual:#x}")


class Util:
    def __init__(self):
        self.claims, self.releases, self.disposed = [], [], []
        self.fail_claim = None

    def claim_interface(self, device, number):
        if number == self.fail_claim:
            raise FakeUSBError(-6)
        self.claims.append(number)

    def release_interface(self, device, number):
        self.releases.append(number)

    def dispose_resources(self, device):
        self.disposed.append(device)


def port_for(device, *, devices=None, util=None, **kwargs):
    util = util or Util()
    requests = []
    def find(**filters):
        requests.append(filters)
        return iter(devices if devices is not None else [device])
    core = SimpleNamespace(find=find, USBError=FakeUSBError)
    port = UsbBulkPort(SERIAL, usb_core=core, usb_util=util, **kwargs)
    return port, util, requests


class FirmwareDevice(Device):
    """Synthetic protocol firmware through the real UsbBulkPort API."""
    config_line = Firmware.config

    def __init__(self, *, lost_terminal=False, boot_changed=False):
        super().__init__()
        self.next_id, self.rev = 10, 1
        self.mrid = self.rrid = self.prid = 1
        self.mgen = self.rgen = self.pgen = 1
        self.mode, self.rows, self.profile, self.rowmodes = "CAP", 8, "FULL", "CCCCCCCC"
        self.lost_terminal, self.lost_ack, self.stale_ack = lost_terminal, False, False
        self.rail, self.fps_cap = True, 0
        self.wrong_identity = self.wrong_mode = self.wrong_unit = False
        self.frames, self.commands, self.sent = 0, [], bytearray()
        self.full_since = self.tail_due = None
        self.command_pending = bytearray()
        self.generating_frame = False
        self.last_frame = 0.0
        self.boot_changed = boot_changed

    def config(self):
        line = self.config_line()
        return line.replace("session=AABBCCDDEE112233", "session=9988776655443322") if self.boot_changed and self.rev > 1 else line

    def send(self, value):
        encoded = value.encode("ascii")
        self.sent.extend(encoded)
        target = 0x81 if self.generating_frame else 0x82
        self.incoming[target].extend(encoded[i:i + 37] for i in range(0, len(encoded), 37))

    def write(self, endpoint, data, timeout):
        count = super().write(endpoint, data, timeout)
        self.command_pending.extend(data[:count])
        while b"\n" in self.command_pending:
            command, _, tail = self.command_pending.partition(b"\n")
            self.command_pending[:] = tail
            if not command.strip():
                continue
            command = command.decode("ascii").strip()
            if command == "USB?":
                self.commands.append(command)
                self.send("USB,backend=otg,ready=1,dataIf=0,ctrlIf=1,synthetic=1\n")
            elif command == "PROTO?":
                self.commands.append(command)
                self.send("PROTO,version=2,caps=config_query_v1/config_identity_v1/usb_vendor_bulk_v1\n")
            else:
                Firmware.reply(self, command)
        return count

    def read(self, endpoint, size, timeout):
        if endpoint == 0x81 and not self.incoming[endpoint] and time.monotonic() - self.last_frame > 0.001:
            self.generating_frame = True
            Firmware.frame(self)
            self.generating_frame = False
            self.last_frame = time.monotonic()
        time.sleep(0.00005)
        return super().read(endpoint, size, timeout)


class UsbBulkTests(unittest.TestCase):
    def options(self, directory, **kwargs):
        return CaptureOptions(None, 1, Path(directory), "bulk", transport="usb_bulk", usb_serial=SERIAL, **kwargs)

    def test_exact_roles_and_no_CDC_or_JTAG_claim(self):
        device = Device()
        port, util, requests = port_for(device)
        self.assertEqual(requests, [{"find_all": True, "idVendor": 0x303A, "idProduct": 0x4041}])
        self.assertEqual(util.claims, [1, 0])
        self.assertEqual(port.metadata()["roles"]["ctrl"]["out"], 0x02)
        self.assertFalse(port.metadata()["cdc_used"])
        port.close()
        self.assertEqual(util.releases, [0, 1])
        self.assertEqual(util.disposed, [device])

    def test_reject_old_products_and_missing_identity_without_enumeration(self):
        for options in ({"pid": 0x1001}, {"pid": 0x4001}, {"serial": "80:B5:4E:CD:D9:AC"}):
            values = {"serial": SERIAL, **options}
            with self.assertRaises(ValueError):
                UsbBulkPort(**values, usb_core=SimpleNamespace(), usb_util=Util())

    def test_descriptor_mismatch_never_claims_anything(self):
        for mutation in ("protocol", "class", "endpoint", "type", "mps", "alternate"):
            device, util = Device(), Util()
            interface = device.usb_configuration.interfaces[(1, 0)]
            if mutation == "protocol": interface.bInterfaceProtocol = 1
            if mutation == "class": interface.bInterfaceClass = 2
            if mutation == "endpoint": interface.endpoints[0].bEndpointAddress = 0x83
            if mutation == "type": interface.endpoints[0].bmAttributes = 3
            if mutation == "mps": interface.endpoints[0].wMaxPacketSize = 512
            if mutation == "alternate": interface.bAlternateSetting = 1
            with self.assertRaises(UsbBulkError, msg=mutation):
                port_for(device, util=util)
            self.assertEqual(util.claims, [])
            self.assertEqual(util.disposed, [device])

    def test_ambiguous_serial_and_partial_claim_cleanup(self):
        first, second, util = Device(), Device(), Util()
        with self.assertRaisesRegex(UsbBulkError, "ambiguous"):
            port_for(first, devices=[first, second], util=util)
        self.assertEqual(util.disposed, [first, second])
        util = Util(); util.fail_claim = 0
        with self.assertRaises(FakeUSBError):
            port_for(first, util=util)
        self.assertEqual(util.releases, [1])
        self.assertEqual(util.disposed, [first])

    def test_timeout_ZLP_partial_write_and_real_disconnect(self):
        device = Device(); port, _, _ = port_for(device)
        device.incoming[0x82].extend([b"", b"CTRL,ok=1\n", FakeUSBError(-4)])
        device.incoming[0x81].append(b"DATA,part")
        limit = time.monotonic() + 1
        self.assertIsNone(port.read_some(4096, limit))  # ZLP is not EOF.
        self.assertEqual(port.read_some(4096, limit), ("data", b"DATA,part"))
        self.assertEqual(port.read_some(4096, limit), ("ctrl", b"CTRL,ok=1\n"))
        self.assertIsNone(port.read_some(4096, limit))  # Empty timeout.
        with self.assertRaisesRegex(UsbBulkError, "ctrl_read_failed"):
            port.read_some(4096, limit)
        device.write_plan.extend([3, FakeUSBError(-7), 2])
        self.assertEqual(port.write_some(b"abcdef", limit), 3)
        self.assertEqual(port.write_some(b"def", limit), 0)
        self.assertEqual(port.write_some(b"def", limit), 2)
        self.assertEqual(device.written, b"abcde")
        self.assertTrue(all(1 <= call[2] <= 10 for call in device.write_calls))
        self.assertEqual(port.write_some(b"x", time.monotonic() - 1), 0)
        port.close()

    def test_real_encoder_chunk_boundaries_and_endpoint_raw_separation(self):
        fixtures = [ROOT / "tests/fixtures/config_identity" / name for name in
                    ("cap_1.frame", "volt_8.frame", "res_extreme.frame", "mixed_extreme.frame")]
        for fixture in fixtures:
            wire = fixture.read_bytes()
            for size in (1, 63, 64, 65, 2304, 4096):
                device = Device(); port, _, _ = port_for(device)
                ctrl = b"CONFIG,session=AABBCCDDEE112233,bootId=7,rev=8\nUSB,ready=1\n"
                device.incoming[0x81].extend(wire[i:i + size] for i in range(0, len(wire), size))
                device.incoming[0x82].extend([ctrl[:7], ctrl[7:19], ctrl[19:]])
                raw, ctrl_raw, events = io.BytesIO(), io.BytesIO(), io.StringIO()
                collector = Collector(self.options(Path('/tmp')), port, raw, events, ctrl_raw=ctrl_raw)
                parser, frames = TextProtocolParser(), []
                while device.incoming[0x81] or device.incoming[0x82]:
                    collector.pump(time.monotonic() + 1)
                    while collector.lines:
                        frame = parser.feed_line(collector.lines.popleft())
                        if frame is not None: frames.append(frame)
                self.assertEqual(len(frames), 1, (fixture.name, size))
                self.assertTrue(frames[0].crc_ok, (fixture.name, size))
                self.assertEqual(parser.counters.crc_errors, 0)
                self.assertEqual(raw.getvalue(), wire)
                self.assertEqual(ctrl_raw.getvalue(), ctrl)
                self.assertEqual(collector.line_buffer, b"")
                self.assertEqual(collector.ctrl_line_buffer, b"")
                recorded = [json.loads(line) for line in events.getvalue().splitlines()]
                self.assertEqual(sum(x['length'] for x in recorded if x['type'] == 'rx'), len(wire))
                self.assertEqual(sum(x['length'] for x in recorded if x['type'] == 'ctrl_rx'), len(ctrl))
                port.close()

    def test_collector_only_writes_CTRL_and_capability_is_exact(self):
        device = Device(); port, _, _ = port_for(device)
        device.write_plan.extend([2, 0, 3])
        c = Collector(self.options(Path('/tmp')), port, io.BytesIO(), io.StringIO(), ctrl_raw=io.BytesIO())
        c.send("CONFIG?", time.monotonic() + 1)
        self.assertEqual(device.written, b"\nCONFIG?\n")
        self.assertGreater(c.partial_writes, 0)
        with self.assertRaises(CaptureError): c.set_usb("FULL", time.monotonic() + 1)
        with self.assertRaises(CaptureError): c.validate_protocol({"version": "2", "caps": "usb_vendor_bulk_v10"})
        with self.assertRaises(CaptureError): c.validate_protocol({"version": "2", "caps": "config_query_v10/usb_vendor_bulk_v1"})
        c.validate_protocol({"version": "2", "caps": "config_query_v1/usb_vendor_bulk_v1"})
        port.close()

    def test_DATA_cannot_satisfy_CTRL_query(self):
        device = Device(); port, _, _ = port_for(device)
        device.incoming[0x82].extend([b"CONFIG,rev=", b"2\n"])
        device.incoming[0x81].append(b"CONFIG,rev=1\n")
        raw, ctrl = io.BytesIO(), io.BytesIO()
        c = Collector(self.options(Path('/tmp')), port, raw, io.StringIO(), ctrl_raw=ctrl)
        result = c.query('CONFIG?', 'CONFIG', time.monotonic() + 1)
        self.assertEqual(result['rev'], '2')
        self.assertEqual(raw.getvalue(), b"CONFIG,rev=1\n")
        self.assertEqual(ctrl.getvalue(), b"CONFIG,rev=2\n")
        port.close()

    def test_disconnected_partial_raw_remains_original(self):
        device = Device(); port, _, _ = port_for(device)
        device.incoming[0x82].append(b"CONFIG,par")
        device.incoming[0x81].extend([b"V,seq=123,par", FakeUSBError(-4)])
        raw, ctrl = io.BytesIO(), io.BytesIO()
        c = Collector(self.options(Path('/tmp')), port, raw, io.StringIO(), ctrl_raw=ctrl)
        for _ in range(3): c.pump(time.monotonic() + 1)
        with self.assertRaises(UsbBulkError): c.pump(time.monotonic() + 1)
        self.assertEqual(raw.getvalue(), b"V,seq=123,par")
        self.assertEqual(ctrl.getvalue(), b"CONFIG,par")
        self.assertEqual(c.capture_trailers, 0)
        port.close()

    def test_real_runner_bulk_window_query_recovery_and_two_raw_files(self):
        for lost_terminal in (False, True):
            device = FirmwareDevice(lost_terminal=lost_terminal)
            port, util, _ = port_for(device)
            with tempfile.TemporaryDirectory() as directory:
                options = BenchmarkOptions(None, Path(directory), "bulk", transport="usb_bulk", usb_serial=SERIAL,
                                           capture_seconds=0.04, warmup_seconds=0.01, prepare_seconds=2,
                                           cleanup_seconds=0.8, command_timeout=0.15, progress_seconds=1)
                with contextlib.redirect_stdout(io.StringIO()):
                    result = run_benchmark(options, port_factory=lambda _: port)
                self.assertEqual(result["status"], "COMPLETE", result)
                report = json.loads((Path(directory) / 'bulk.summary.json').read_text())
                self.assertEqual(report['usb_restore'], 'NOT_APPLICABLE_BULK_ALWAYS_FULL')
                self.assertTrue(report['capture_duration_satisfied'])
                self.assertEqual(report['identity_mismatches'], 0)
                self.assertTrue(report['capture_crc_valid_frames'])
                self.assertEqual(report['raw_scope'], 'DATA_endpoint_only')
                self.assertNotIn(b'CONFIG,', Path(report['raw_file']).read_bytes())
                self.assertIn(b'CONFIG,', Path(report['ctrl_raw_file']).read_bytes())
                self.assertFalse(any(x.startswith('USBSTREAM') for x in device.commands))
                self.assertTrue(all(call[0] == 0x02 for call in device.write_calls))
                if lost_terminal:
                    self.assertTrue(all(t['verification'] == 'ACK_CONFIG_RECOVERED_TERMINAL' for t in report['transitions']))
                with self.assertRaises(FileExistsError):
                    run_benchmark(options, port_factory=lambda _: (_ for _ in ()).throw(AssertionError('must not open')))
            self.assertEqual(util.releases, [0, 1])

    def test_changed_session_is_not_carried_as_current_configuration(self):
        device = FirmwareDevice(boot_changed=True); port, _, _ = port_for(device)
        with tempfile.TemporaryDirectory() as directory:
            options = BenchmarkOptions(None, Path(directory), 'reboot', transport='usb_bulk', usb_serial=SERIAL,
                                       capture_seconds=0.02, warmup_seconds=0.01, prepare_seconds=1,
                                       cleanup_seconds=0.3, command_timeout=0.08, progress_seconds=1)
            with contextlib.redirect_stdout(io.StringIO()):
                result = run_benchmark(options, port_factory=lambda _: port)
            report = json.loads((Path(directory) / 'reboot.summary.json').read_text())
            self.assertEqual(result['status'], 'ERROR')
            self.assertIn('boot_changed_during_transition', report['error'])
            self.assertEqual(report['capture_seconds'], 0)

    @unittest.skipUnless(importlib.util.find_spec("usb"), "PyUSB optional in system Python; executed in HIL venv")
    def test_installed_pyusb_partial_timeout_preserves_count_without_USB_context(self):
        import usb.backend.libusb1 as implementation
        # Execute the installed backend methods with an injected C-function
        # boundary. No get_backend(), libusb_init(), device find/open, or I/O.
        handle = SimpleNamespace(handle=None)
        def transfer(handle, endpoint, data, length, transferred, timeout):
            ctypes.cast(transferred, ctypes.POINTER(ctypes.c_int))[0] = 3
            return implementation.LIBUSB_ERROR_TIMEOUT
        read = implementation._LibUSB._LibUSB__read
        write = implementation._LibUSB._LibUSB__write
        self.assertEqual(read(None, transfer, handle, 0x81, 0, array('B', [0] * 64), 2), 3)
        self.assertEqual(write(None, transfer, handle, 0x02, 1, array('B', [1] * 64), 2), 3)

    def test_close_failure_keeps_benchmark_evidence_and_original_error(self):
        device = FirmwareDevice(boot_changed=True); port, util, _ = port_for(device)
        def fail_release(device, interface):
            raise FakeUSBError(-4)
        util.release_interface = fail_release
        with tempfile.TemporaryDirectory() as directory:
            options = BenchmarkOptions(None, Path(directory), 'release', transport='usb_bulk', usb_serial=SERIAL,
                                       capture_seconds=0.02, warmup_seconds=0.01, prepare_seconds=1,
                                       cleanup_seconds=0.3, command_timeout=0.08, progress_seconds=1)
            with contextlib.redirect_stdout(io.StringIO()):
                result = run_benchmark(options, port_factory=lambda _: port)
            report = json.loads((Path(directory) / 'release.summary.json').read_text())
            index = json.loads((Path(directory) / 'release.benchmark.json').read_text())
            self.assertIn('boot_changed_during_transition', report['error'])
            self.assertIn('usb_release_failed', index['close_error'])
            self.assertEqual(index['status'], 'ERROR')
            self.assertTrue(Path(report['ctrl_raw_file']).exists())
            self.assertEqual(util.disposed, [device])


if __name__ == '__main__':
    unittest.main()
