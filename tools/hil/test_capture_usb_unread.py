#!/usr/bin/env python3
"""Synthetic CDC/clock/signal tests only; no device enumeration or access."""
import contextlib
from dataclasses import replace
import hashlib
import io
import json
import os
from pathlib import Path
import signal
import tempfile
import time
import unittest
from unittest.mock import patch

import capture_usb_unread as capture
from sensorarray_usb_cdc import UsbCdcPort
from test_usb_cdc_capture import CdcFirmware


SYNTHETIC_SERIAL = '001122AABBCC'


class Firmware(CdcFirmware):
    def __init__(self, *, optional=True, late_fault=None, ready=True, rail=True):
        super().__init__(optional_caps=('ble_flow_v1', 'ble_snapshot_v1', 'heap_v1') if optional else ())
        self.late_fault, self.ready, self.rail = late_fault, ready, rail
        self.seen_volt = False
        self.closed = self.reads = self.writes = 0
        self.config_fault = None
        self.silent = False

    def config(self):
        text = super().config().replace('state=CAPACITANCE', 'state=' + {
            'CAP': 'CAPACITANCE', 'VOLT': 'VOLTAGE', 'RES': 'RESISTANCE'}[self.mode])
        if self.config_fault == 'boot':
            text = text.replace('bootId=1', 'bootId=2')
        if self.config_fault == 'old':
            text = text.replace('rev=' + str(self.rev), 'rev=0')
        return text

    def send(self, text):
        if text.startswith('MODE,') and 'active=VOLT,' in text:
            text = text.replace('state=CAPACITANCE', 'state=VOLTAGE')
        super().send(text.replace('target=pty', 'target=esp32s3'))

    def write(self, endpoint, data, timeout):
        self.writes += 1
        command = data.strip().decode()
        if command == 'MODE=VOLT':
            self.seen_volt = True
        if command == 'MODE=CAP' and self.seen_volt and self.late_fault:
            self.shared.append((self.late_fault + '\n').encode())
        if command in {'READY?', 'MEM?'}:
            self.commands.append(command)
            if command == 'READY?':
                text = f'READY,ready={int(self.ready)},stage=ready,err=0x0,bootId=1,boot=1\n'
            else:
                text = ('MEM,v=1,readout=sequential,unit=bytes,intCaps=internal8,intFree=20000,intLargest=18000,intMin=17000,'
                        'extFree=8000000,extLargest=7900000,scan=created,scanStack=16384,beforeFree=38000,beforeLargest=35000,'
                        'afterFree=20000,afterLargest=18000,err=0x0,sampleUs=123\n')
            self.shared.append(text.encode())
            return len(data)
        return super().write(endpoint, data, timeout)

    def read_some(self, size, deadline):
        self.reads += 1
        if self.silent:
            time.sleep(min(.002, max(0, deadline-time.monotonic())))
            return None
        return super().read_some(size, deadline)

    def close(self):
        self.closed += 1


class Tests(unittest.TestCase):
    def setUp(self):
        # This test uses millisecond device deadlines and synthetic transport.
        # The size or dirty state of the developer's checkout is unrelated.
        def git(command, **kwargs):
            if command[1] == 'rev-parse':
                return '0' * 40 + '\n'
            self.assertIn(command[1], ('diff', 'status'))
            return b''
        git_patch = patch.object(capture.subprocess, 'check_output', side_effect=git)
        git_patch.start()
        self.addCleanup(git_patch.stop)

    def options(self, folder, **kw):
        return replace(capture.Options(Path(folder), 'synthetic-unread', usb_serial=SYNTHETIC_SERIAL, rail_avdd_uv=3350000, rail_avss_uv=-1795000,
                                       rail_source_note='synthetic fixture', hold_seconds=.04,
            total_seconds=2.5, prepare_seconds=.7, after_seconds=.35, cleanup_seconds=.5,
            command_timeout=.04, expected_elf_sha256='ab'*32), **kw)

    def run_case(self, device, options, hook=None):
        def port_factory(_settings):
            return UsbCdcPort(SYNTHETIC_SERIAL,
                metadata_finder=lambda *a, **k: {'path': '/dev/synthetic-never-opened', 'usb_serial': SYNTHETIC_SERIAL,
                    'vid': '303a', 'pid': '4042', 'interface_number': 0}, port_factory=lambda _: device)
        first = True
        hold_io = []
        def sleep(seconds):
            nonlocal first
            before = (device.reads, device.writes)
            if first and hook:
                first = False; hook()
            time.sleep(seconds)
            hold_io.append((before, (device.reads, device.writes)))
        start = time.monotonic()
        with contextlib.redirect_stdout(io.StringIO()):
            result = capture.run_capture(options, port_factory=port_factory, sleeper=sleep)
        self.assertLess(time.monotonic()-start, options.total_seconds+.25)
        self.assertEqual(device.closed, 1)
        self.assertTrue(all(a == b for a, b in hold_io), hold_io)
        self.assertFalse(any(c.startswith(('USBSTREAM=', 'TX=', 'CAL=', 'USBBOOT', 'RESTART', 'RESET', 'FPS=', 'SPI=')) for c in device.commands))
        raw = Path(result['raw_file']).read_bytes()
        self.assertEqual(result['raw_sha256'], hashlib.sha256(raw).hexdigest())
        self.assertEqual(result['rx_bytes'], len(raw))
        self.assertEqual(result, json.loads(Path(result['summary_file']).read_text()))
        self.assertIsNone(result['USB_hold_PC_FPS'])
        self.assertEqual(result['USB_hold_PC_receipt'], 'NOT_OBSERVED')
        return result, raw

    def test_real_collector_zero_hold_io_full_stream_raw_and_cap8(self):
        device = Firmware(rail=False)
        device.shared.append(b'PREEXISTING,record=preserved\n')
        with tempfile.TemporaryDirectory() as folder:
            options = self.options(folder)
            result, raw = self.run_case(device, options)
            self.assertEqual(result['status'], 'UNREAD_HOLD_COMPLETED_RECOVERY_VERIFIED', result['error'])
            self.assertEqual(result['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
            self.assertEqual((device.mode, device.rows), ('CAP', 8))
            self.assertEqual(device.commands.count('RAILCFG=3350000,-1795000'), 1)
            self.assertTrue(result['volatile_rail_written'])
            self.assertIn(b'PREEXISTING,record=preserved\n', raw)
            hold = result['hold']
            self.assertEqual((hold['read_calls'], hold['write_calls'], hold['forbidden_read_calls'], hold['forbidden_write_calls']), (0, 0, 0, 0))
            self.assertEqual(hold['raw_offset_before'], hold['raw_offset_after'])
            self.assertGreaterEqual(hold['actual_seconds'], options.hold_seconds)
            self.assertEqual(device.commands.count('MEM?'), 2)
            self.assertEqual(device.commands.count('BLEFLOW?'), 2)
            self.assertEqual(device.commands.count('BLESNAP?'), 2)
            for name in ('before', 'after'):
                window = result['snapshots'][name]
                self.assertEqual(window['outcome'], 'COHERENT_QUERY_VERIFIED')
                for record in window['queries'].values():
                    if 'line' in record:
                        self.assertIn((record['line']+'\n').encode(), raw)
                        self.assertLessEqual(record['started_monotonic'], record['received_monotonic'])
                        self.assertLessEqual(record['received_monotonic'], record['finished_monotonic'])
            with self.assertRaises(FileExistsError):
                capture.run_capture(options, port_factory=lambda _: self.fail('must not reopen'))

    def test_no_optional_capability_never_queries_it(self):
        with tempfile.TemporaryDirectory() as folder:
            device = Firmware(optional=False)
            result, _ = self.run_case(device, self.options(folder))
            self.assertEqual(result['status'], 'UNREAD_HOLD_COMPLETED_RECOVERY_VERIFIED', result['error'])
            for command in ('MEM?', 'BLEFLOW?', 'BLESNAP?'):
                self.assertNotIn(command, device.commands)
                self.assertEqual(result['snapshots']['before']['queries'][command]['status'], 'CAPABILITY_UNAVAILABLE')

    def test_stop_file_and_actual_SIGTERM_interrupt_hold_but_recover(self):
        for mode in ('file', 'signal', 'keyboard'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as folder:
                device = Firmware()
                stop = Path(folder)/'stop'
                def hook():
                    if mode == 'file': stop.touch()
                    elif mode == 'signal': os.kill(os.getpid(), signal.SIGTERM)
                    else: raise KeyboardInterrupt('synthetic_interrupt')
                result, _ = self.run_case(device, self.options(folder, stop_file=stop), hook)
                self.assertEqual(result['status'], 'ERROR')
                self.assertEqual(result['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
                self.assertEqual(result['hold']['status'], 'INCOMPLETE')
                self.assertEqual(result['hold']['read_calls'], 0)
                if mode == 'signal': self.assertEqual(result['signals'], [signal.SIGTERM])

    def test_after_timeout_keeps_original_total_budget_and_unknown_cleanup(self):
        with tempfile.TemporaryDirectory() as folder:
            device = Firmware()
            result, _ = self.run_case(device, self.options(folder), lambda: setattr(device, 'silent', True))
            self.assertEqual(result['status'], 'ERROR')
            self.assertIsNotNone(result['after_error'])
            self.assertIsNotNone(result['cleanup_error'])
            self.assertEqual(result['error'], result['after_error'])
            self.assertEqual(result['safe_cap_recovery'], 'FAILED_OR_UNKNOWN')
            for query in result['query_attempts']:
                self.assertLessEqual(query['phase_deadline'], result['total_deadline_monotonic'])

    def test_cleanup_fault_and_saturated_reset_sample_list_are_not_hidden(self):
        for fault in ('MFAULT,seq=99,err=0x108', 'RST,syntheticCleanup=17'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as folder:
                device = Firmware(late_fault=fault)
                def hook():
                    if fault.startswith('RST'):
                        device.shared.append(''.join(f'RST,syntheticHold={i}\n' for i in range(16)).encode())
                result, raw = self.run_case(device, self.options(folder), hook)
                self.assertEqual(result['status'], 'ERROR')
                self.assertEqual(result['safe_cap_recovery'], 'FAILED_OR_UNKNOWN')
                self.assertIn(fault.encode(), raw)
                if fault.startswith('RST'):
                    self.assertEqual(result['reset_event_count'], 17)
                    self.assertEqual(len(result['reset_observations']), 16)
                    self.assertEqual(result['error'], result['after_error'])
                else: self.assertEqual(result['fault_events'], 1)

    def test_old_config_or_different_boot_cannot_pass_after_bracket(self):
        for kind in ('old', 'boot'):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as folder:
                device = Firmware()
                result, _ = self.run_case(device, self.options(folder), lambda: setattr(device, 'config_fault', kind))
                self.assertEqual(result['status'], 'ERROR')
                self.assertIsNotNone(result['after_error'])
                if kind == 'boot': self.assertEqual(result['safe_cap_recovery'], 'FAILED_OR_UNKNOWN')

    def test_unready_or_wrong_build_never_mutates(self):
        for ready, elf in ((False, 'ab'*32), (True, 'cd'*32)):
            with self.subTest(ready=ready), tempfile.TemporaryDirectory() as folder:
                device = Firmware(ready=ready)
                result, _ = self.run_case(device, self.options(folder, expected_elf_sha256=elf))
                self.assertEqual(result['status'], 'ERROR')
                self.assertIsNone(result['hold'])
                self.assertFalse(any('=' in command for command in device.commands))
                self.assertEqual(result['safe_cap_recovery'], 'UNKNOWN_AUTHORITY_UNVERIFIED')

    def test_hold_wrapper_rejects_io_before_delegate(self):
        device = Firmware(); port = capture.AuditedPort(device); port.holding = True
        with self.assertRaises(capture.CaptureError): port.read_some(1, 99)
        with self.assertRaises(capture.CaptureError): port.write_some(b'unsafe', 99)
        self.assertEqual((device.reads, device.writes), (0, 0))
        self.assertEqual((port.hold_read_calls, port.hold_write_calls), (1, 1))

    def test_full_120_second_hold_with_injected_monotonic_clock(self):
        collector = object.__new__(capture.UnreadCollector)
        collector.limits = capture.Options(Path('/tmp'), 'synthetic-clock', usb_serial=SYNTHETIC_SERIAL)
        collector.cancel = capture.Cancellation()
        collector.total_deadline = 200.0
        collector.raw_bytes = 77
        collector.port = capture.AuditedPort(Firmware())
        events = []
        collector.event = lambda kind, **fields: events.append((kind, fields))
        clock = [10.0]
        def sleep(seconds):
            self.assertLessEqual(seconds, .1)
            self.assertTrue(collector.port.holding)
            clock[0] += seconds
        with patch.object(capture.time, 'monotonic', side_effect=lambda: clock[0]):
            collector.hold_unread(sleeper=sleep)
        self.assertEqual(collector.hold['actual_seconds'], 120.0)
        self.assertEqual((collector.hold['read_calls'], collector.hold['write_calls']), (0, 0))
        self.assertFalse(collector.port.closed)
        self.assertEqual([kind for kind, _ in events], ['unread_hold_start', 'unread_hold_end'])
        collector.total_deadline = 150.0
        with patch.object(capture.time, 'monotonic', return_value=20.0):
            with self.assertRaisesRegex(capture.CaptureError, 'insufficient_full_unread_hold_budget'):
                collector.hold_unread(sleeper=lambda _: self.fail('no shortened hold'))

    def test_private_memory_ready_schema_unknown_and_range(self):
        fields = {key: 'unknown' for key in capture.MEM_FIELDS.split()}
        fields.update(v='1', readout='sequential', unit='bytes', intCaps='internal8', scan='unknown', sampleUs='0')
        for key in ('intFree', 'intLargest', 'intMin', 'extFree', 'extLargest'):
            fields[key] = str(2**32-1)
        self.assertEqual(capture.UnreadCollector.validate_fields('MEM?', fields), fields)
        for key, value in (('scanStack', '0'), ('intFree', str(2**32)), ('sampleUs', str(2**64)), ('readout', 'atomic')):
            with self.subTest(key=key), self.assertRaises(capture.CaptureError):
                capture.UnreadCollector.validate_fields('MEM?', {**fields, key: value})
        with self.assertRaises(capture.CaptureError):
            capture.UnreadCollector.validate_fields('READY?', {'ready':'1','stage':'ready','err':'0x0','bootId':'1'})


if __name__ == '__main__':
    unittest.main()
