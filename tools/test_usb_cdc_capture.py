#!/usr/bin/env python3
"""Formal CDC host tests: synthetic sysfs/PTY only; never opens a USB board."""
from collections import deque
import contextlib
from dataclasses import replace
import errno
import io
import json
import os
from pathlib import Path
import pty
import select
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

from capture_reference_fixture import Collector, CaptureOptions, CaptureError, validate_transport_options, transport_usb_pid
from sensorarray_usb_cdc import UsbCdcPort, UsbCdcError, find_cdc_tty
from run_firmware_benchmark import BenchmarkOptions, configurations
from text_protocol import TextProtocolParser

ROOT = Path(__file__).resolve().parents[1]
STAGE = ROOT / 'docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts'
sys.path.insert(0, str(STAGE))
import capture_cdc_benchmark as stage
import observe_cdc_debug as debug
import observe_bulk_boot as boot
from test_bulk_stage import StageFirmware
from test_usb_bulk_capture import SERIAL
from test_firmware_benchmark import Firmware


class CdcFirmware(StageFirmware):
    def __init__(self, **kwargs):
        self.shared = deque()
        super().__init__(**kwargs)

    def send(self, text):
        if text.startswith('PROTO,'):
            caps = '/'.join(sorted(stage.CdcBenchmarkCollector.required_capabilities | self.optional_caps))
            self.shared.append(('PROTO,version=2,wires=ascii,ctrlMax=512,dataMax=2304,caps=' + caps + ',adcCrc=crc8\n').encode())
            return
        if text.startswith('USB,'):
            fields = {key: '0' for key in stage.shared.EXTRA_REQUIRED['USB?'].split()}
            fields.update(backend='otg_cdc_v1', init='1', mounted='1', err='0x0', pcReceipt='unknown', dataDtr='1')
            self.shared.append(('USB,' + ','.join(f'{k}={v}' for k,v in fields.items()) + '\n').encode())
            return
        # StageFirmware completes full synthetic query schemas before routing.
        super().send(text)
        endpoint = 0x81 if self.generating_frame else 0x82
        chunks = self.incoming[endpoint]
        while chunks:
            chunk = chunks.popleft()
            self.shared.append(chunk)

    def read_some(self, size, deadline):
        if not self.shared and time.monotonic() - self.last_frame > 0.001:
            self.generating_frame = True; Firmware.frame(self); self.generating_frame = False
            self.last_frame = time.monotonic()
        time.sleep(0.00005)
        if not self.shared: return None
        chunk = self.shared.popleft()
        return 'data', chunk

    def write_some(self, data, deadline):
        return self.write(0x02, data, 1)

    def close(self): pass

    def write(self, endpoint, data, timeout):
        # Capture's newline-delimited commands are short; synthesize the new
        # immutable USBSTREAM query, never enable a setter for this backend.
        if b'USBSTREAM?' in data:
            self.commands.append('USBSTREAM?')
            self.shared.append(b'USBSTREAM,v=CDC,scope=otg_data,dataEvery=1,debug=cdc,control=shared_cdc\n')
            return len(data)
        return super().write(endpoint, data, timeout)


class CdcTests(unittest.TestCase):
    def metadata(self, path='/dev/synthetic'):
        return {'path': path, 'usb_serial': SERIAL, 'vid': '303a', 'pid': '4042', 'interface_number': 0}

    def test_exact_sysfs_role_and_debug_rejection(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); tty = root / 'tty'; tty.mkdir()
            def candidate(name, pid='4042', interface='00', serial=SERIAL):
                device = root / name; device.mkdir()
                for key, value in {'idVendor': '303a', 'idProduct': pid, 'serial': serial, 'speed': '12'}.items():
                    (device / key).write_text(value)
                itf = device / 'interface'; itf.mkdir()
                for key, value in {'bInterfaceNumber': interface, 'bInterfaceClass': '02', 'bInterfaceSubClass': '02', 'bInterfaceProtocol': '00'}.items():
                    (itf / key).write_text(value)
                entry = tty / name; entry.mkdir(); (entry / 'device').symlink_to(itf)
            candidate('ttyACM0', interface='02')
            candidate('ttyACM1', pid='1001')
            candidate('ttyACM2', serial='000000000001')
            with self.assertRaisesRegex(UsbCdcError, 'not_enumerated'): find_cdc_tty(SERIAL, sys_tty=tty)
            candidate('ttyACM3')
            self.assertEqual(find_cdc_tty(SERIAL, sys_tty=tty)['path'], '/dev/ttyACM3')
            self.assertEqual(debug.find_debug_tty(SERIAL, pid=0x4042, sys_tty=tty)['path'], '/dev/ttyACM0')
            candidate('ttyACM4')
            with self.assertRaisesRegex(UsbCdcError, 'ambiguous'): find_cdc_tty(SERIAL, sys_tty=tty)

    def test_options_default_PID_and_wrong_role_fail_before_open(self):
        options = CaptureOptions(None, 1, Path('/tmp'), 'cdc', transport='usb_cdc', usb_serial=SERIAL)
        self.assertEqual(transport_usb_pid(options), 0x4042)
        validate_transport_options(options)
        for bad in (replace(options, port='/dev/debug'), replace(options, usb_pid=0x4041), replace(options, usb_serial='bad')):
            with self.assertRaises(ValueError): validate_transport_options(bad)
        with self.assertRaises(ValueError): UsbCdcPort(SERIAL, pid=0x4041, metadata_finder=lambda *a, **k: self.fail('no lookup'))

    def test_real_PosixPort_PTY_path_keeps_encoder_bytes_and_no_modem_calls(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: self.metadata(path))
        wire = (ROOT / 'tests/fixtures/config_identity/cap_1.frame').read_bytes()
        try:
            os.write(master, wire)
            raw = b''; deadline = time.monotonic() + 1
            while len(raw) < len(wire) and time.monotonic() < deadline:
                result = port.read_some(65536, deadline)
                if result: raw += result[1]
            self.assertEqual(raw, wire)
            self.assertEqual(port.write_some(b'\nCONFIG?\n', deadline), 9)
            self.assertEqual(os.read(master, 4096), b'\nCONFIG?\n')
            self.assertFalse(port.metadata()['libusb_used'])
            self.assertFalse(port.metadata()['modem_bit_ioctl'])
            parser = TextProtocolParser(); frames = [f for line in raw.decode().splitlines() if (f := parser.feed_line(line))]
            self.assertEqual(len(frames), 1); self.assertTrue(frames[0].crc_ok)
        finally:
            port.close(); os.close(master); os.close(slave)

    def test_exact_capability_and_shared_stream_query(self):
        class Reader:
            def read_some(self, *args): return 'data', b'CONFIG,rev=9\n'
            def write_some(self, data, deadline): return len(data)
            def close(self): pass
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: self.metadata(), port_factory=lambda _: Reader())
        c = Collector(CaptureOptions(None, 1, Path('/tmp'), 'cdc', transport='usb_cdc'), port, io.BytesIO(), io.StringIO())
        for caps in ('usb_vendor_bulk_v1/config_query_v1', 'usb_cdc_data_v10/config_query_v1'):
            with self.assertRaises(CaptureError): c.validate_protocol({'version': '2', 'caps': caps})
        c.validate_protocol({'version': '2', 'caps': 'usb_cdc_data_v1/config_query_v1'})
        self.assertEqual(c.query('CONFIG?', 'CONFIG', time.monotonic() + 1)['rev'], '9')
        with self.assertRaises(CaptureError): c.set_usb('FULL', time.monotonic() + 1)
        self.assertEqual(c.transport_result()['raw_scope'], 'formal_CDC_DATA_CTRL_shared_stream')

    def test_CDC_stage_full_capture_periodic_queries_no_bulk_or_setter(self):
        device = CdcFirmware()
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: self.metadata(), port_factory=lambda _: device)
        interval, budget = stage.CdcBenchmarkCollector.diagnostic_interval_seconds, stage.CdcBenchmarkCollector.periodic_budget_seconds
        stage.CdcBenchmarkCollector.diagnostic_interval_seconds = .04
        stage.CdcBenchmarkCollector.periodic_budget_seconds = .1
        try:
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                opts = BenchmarkOptions(None, Path(directory), 'cdc', transport='usb_cdc', usb_serial=SERIAL,
                                        capture_seconds=.3, warmup_seconds=.01, prepare_seconds=2, cleanup_seconds=1, command_timeout=.08)
                index = stage.run_benchmark(opts, port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'cdc.summary.json').read_text())
                self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
                self.assertGreater(report['capture_crc_valid_frames'], 0)
                self.assertTrue(any(w['name'] == 'periodic' for w in report['diagnostic_windows']))
                self.assertEqual(report['usb_restore'], 'NOT_APPLICABLE_CDC_ALWAYS_FULL')
                self.assertNotIn('ctrl_raw_file', report)
                self.assertIsNone(report['ctrl_rx_bytes'])
                self.assertFalse((Path(directory) / 'cdc.ctrl.rx.bin').exists())
                self.assertFalse(any(command.startswith(('USBSTREAM=', 'USBBOOT', 'RESTART')) for command in device.commands))
                self.assertFalse(set(stage.shared.FRAME_OUTPUT_REQUIRED) & set(device.commands))
                self.assertEqual(len(configurations(replace(opts, suite=True))), 32)
                self.assertEqual(configurations(replace(opts, stability=True))[0].seconds, 600)
        finally:
            stage.CdcBenchmarkCollector.diagnostic_interval_seconds, stage.CdcBenchmarkCollector.periodic_budget_seconds = interval, budget

    def test_identity_change_during_open_closes_selected_tty(self):
        reader = mock.Mock(); calls = []
        def metadata(*args, **kwargs):
            calls.append(1)
            return self.metadata('/dev/synthetic' if len(calls) == 1 else '/dev/changed')
        with self.assertRaisesRegex(UsbCdcError, 'identity_changed'):
            UsbCdcPort(SERIAL, metadata_finder=metadata, port_factory=lambda _: reader)
        reader.close.assert_called_once_with()

    def test_CDC_optional_service_queries_preserve_shared_raw_and_capture_window(self):
        device = CdcFirmware(optional_caps=stage.shared.OPTIONAL_CAPABILITIES.values(), truncate='SCHEDWHY?')
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: self.metadata(), port_factory=lambda _: device)
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            opts = BenchmarkOptions(None, Path(directory), 'cdc-service', transport='usb_cdc', usb_serial=SERIAL,
                                    capture_seconds=.1, warmup_seconds=.01, prepare_seconds=2,
                                    cleanup_seconds=1, command_timeout=.08)
            index = stage.run_benchmark(opts, port_factory=lambda _: port)
            report = json.loads((Path(directory) / 'cdc-service.summary.json').read_text())
            self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
            self.assertTrue(report['capture_duration_satisfied'])
            self.assertGreaterEqual(report['capture_seconds'], .1)
            raw = Path(report['raw_file']).read_bytes()
            for name in ('before', 'after'):
                for command in stage.shared.OPTIONAL_REQUIRED:
                    q = report['snapshots'][name][command]
                    self.assertEqual(q['reply_channel'], 'data')
                    self.assertIn((q['reply_line'] + '\n').encode(), raw)
                    self.assertLessEqual(q['started_monotonic'], q['reply_event_monotonic'])
                    self.assertLessEqual(q['reply_event_monotonic'], q['monotonic'])
                    self.assertLessEqual(q['monotonic'], report['snapshots'][name]['CONFIG?_closing']['started_monotonic'])
                    if name == 'before': self.assertLessEqual(q['monotonic'], report['capture_monotonic_start'])
                    else: self.assertGreaterEqual(q['started_monotonic'], report['capture_monotonic_end'])
            self.assertEqual(device.query_counts['SCHEDWHY?'], 3)
            self.assertEqual(device.query_counts['BLESVC?'], 2)
            self.assertTrue(all(device.query_counts[c] == 2 for c in stage.shared.FRAME_OUTPUT_REQUIRED))
            self.assertTrue(all(not set(stage.shared.FRAME_OUTPUT_REQUIRED) & set(window['queries'])
                                for window in report['diagnostic_windows'] if window['name'] == 'periodic'))
            self.assertNotIn('ctrl_raw_file', report)
            self.assertIn('shared_diagnostic_source_sha256', report)
            self.assertFalse(any(c.startswith(('USBSTREAM=', 'USBBOOT', 'RESTART', 'FPS=')) for c in device.commands))

    def test_CDC_cpu_owner_real_PTY_full_capture_and_unknown_role(self):
        # The firmware is synthetic, but every command/reply traverses the
        # actual POSIX reader and shared CDC collector through a real PTY.
        unknown = {k: 'unknown' for k in ('taskNumber', 'runUs', 't0Us', 't1Us', 'observedAtUs', 'ageUs')}
        device = CdcFirmware(optional_caps={stage.shared.CPU_OWNER_CAPABILITY}, truncate='CPUOWNER?2',
                             cpu_values={6: dict(unknown, available='0', known='0', stale='1', reason='not_observed')})
        master, slave = pty.openpty(); path = os.ttyname(slave)
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: self.metadata(path))
        os.close(slave); os.set_blocking(master, False)
        stopped = threading.Event(); errors = []
        def serve():
            pending = bytearray()
            try:
                while not stopped.is_set():
                    readable, _, _ = select.select([master], [], [], .001)
                    if readable:
                        try: command = os.read(master, 4096)
                        except OSError as error:
                            if error.errno == errno.EIO: break  # Collector closed the PTY after cleanup.
                            raise
                        if command: device.write_some(command, time.monotonic()+.1)
                    received = device.read_some(65536, time.monotonic()+.01)
                    if received: pending.extend(received[1])
                    if len(pending) > 262144: raise AssertionError('synthetic PTY backlog exceeded bound')
                    if pending and select.select([], [master], [], 0)[1]:
                        written = os.write(master, pending)
                        del pending[:written]
            except BaseException as error: errors.append(error)
        thread = threading.Thread(target=serve, daemon=True); thread.start()
        try:
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                options = BenchmarkOptions(None, Path(directory), 'cdc-cpu', transport='usb_cdc', usb_serial=SERIAL,
                                           capture_seconds=.05, warmup_seconds=.01, prepare_seconds=2,
                                           cleanup_seconds=1, command_timeout=.08)
                index = stage.run_benchmark(options, port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'cdc-cpu.summary.json').read_text())
                self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
                self.assertGreaterEqual(report['capture_seconds'], .05)
                self.assertGreater(report['capture_crc_valid_frames'], 0)
                self.assertEqual(report['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
                raw = Path(report['raw_file']).read_bytes()
                for side in ('before', 'after'):
                    snap = report['snapshots'][side]
                    for command, _ in stage.shared.CPU_OWNER_QUERIES:
                        query = snap[command]
                        self.assertEqual(query['reply_channel'], 'data')
                        self.assertEqual(query['fields']['role'], command[-1])
                        self.assertIn((query['reply_line']+'\n').encode(), raw)
                        self.assertLessEqual(query['reply_received_monotonic'], query['reply_event_monotonic'])
                        self.assertLessEqual(query['monotonic'], snap['CONFIG?_closing']['started_monotonic'])
                    self.assertEqual(snap['CPUOWNER?6']['fields']['runUs'], 'unknown')
                    self.assertFalse(snap['CPUOWNER?6']['cpu_owner_evidence']['usable_for_pairing'])
                self.assertEqual(device.query_counts['CPUOWNER?2'], 3)
                self.assertFalse(report['modem_bit_ioctl'])
                self.assertFalse(any(c.startswith(('USBSTREAM=', 'USBBOOT', 'RESTART', 'FPS=')) for c in device.commands))
        finally:
            stopped.set(); thread.join(1); port.close(); os.close(master)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])

    def test_CDC_stage_CLI_refuses_legacy_or_bulk_before_open(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stderr(io.StringIO()):
            with mock.patch.object(stage.shared.base, 'open_capture_port', side_effect=AssertionError('must reject before open')):
                for arguments in (['--transport', 'posix', '--port', '/dev/debug'], ['--transport=usb_bulk', '--usb-serial', SERIAL]):
                    with self.assertRaises(SystemExit) as failure:
                        stage.main([*arguments, '--output-directory', directory, '--label', 'rejected'])
                    self.assertEqual(failure.exception.code, 2)
            self.assertFalse(list(Path(directory).iterdir()))

    def test_readonly_CDC_boot_observer_does_not_create_fake_CTRL_raw(self):
        device = CdcFirmware()
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: self.metadata(), port_factory=lambda _: device)
        with tempfile.TemporaryDirectory() as directory:
            options = boot.Options(Path(directory), 'cdcboot', SERIAL, duration=2, observe_seconds=.01,
                                   command_timeout=.1, transport='usb_cdc')
            result = boot.observe(options, port_factory=lambda *args, **kwargs: port)
            self.assertEqual(result['status'], 'QUERIES_COMPLETE', result.get('error'))
            self.assertTrue(all(command.endswith('?') for command in device.commands))
            self.assertNotIn('ctrl_raw_file', result)
            self.assertFalse((Path(directory) / 'cdcboot.ctrl.rx.bin').exists())
            self.assertEqual(result['replies']['USBSTREAM?']['control'], 'shared_cdc')
            self.assertEqual(result['build_identity']['elf_sha256'], 'ab' * 32)
            self.assertIsNone(result['build_identity']['application_binary_sha256'])

    def test_unknown_DTR_and_shared_callback_counts_remain_unknown(self):
        fields = {key: '0' for key in stage.shared.EXTRA_REQUIRED['USB?'].split()}
        fields.update(backend='otg_cdc_v1', err='0x0', pcReceipt='unknown', dataDtr='unknown', debugDtr='unknown')
        self.assertEqual(stage.CdcBenchmarkCollector.validate_fields('USB?', fields)['dataDtr'], 'unknown')
        queue = {key: '0' for key in stage.shared.EXTRA_REQUIRED['USBQ?'].split()}
        queue.update(callbackResult='unknown', dCallbackBytes='unknown', cCallbackBytes='unknown')
        self.assertEqual(stage.CdcBenchmarkCollector.validate_fields('USBQ?', queue)['dCallbackBytes'], 'unknown')
        with self.assertRaises(CaptureError):
            stage.CdcBenchmarkCollector.validate_fields('BUILD?', {'idf':'test','target':'s3','project':'SensorArray','proto':'2'})


if __name__ == '__main__': unittest.main()
