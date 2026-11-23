#!/usr/bin/env python3
"""Synthetic firmware/CDC regressions; this test never enumerates a board."""
from collections import deque
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import tempfile
import time
import unittest
import zlib

import capture_mode_switches as capture
from sensorarray_usb_cdc import UsbCdcPort
from test_usb_cdc_capture import CdcFirmware
from test_firmware_benchmark import Firmware
from text_protocol import parse_fields


class Device(CdcFirmware):
    def __init__(self, *, lose_terminal=False, missing_frames=False, fault=None,
                 bad_protocol=False, wrong_legacy=False, old_frame=False, future_frame=False):
        super().__init__()
        self.lose_terminal = lose_terminal
        self.lost_once = False
        self.missing_frames = missing_frames
        self.inject_fault = fault
        self.injected_fault = False
        self.bad_protocol = bad_protocol
        self.wrong_legacy = wrong_legacy
        self.future_frame = future_frame
        self.old_frame = old_frame
        self.saved_old_frame = None
        self.closed = 0

    def config(self):
        # Correct the older shared fixture's fixed CAP state for homogeneous
        # ADS modes, matching production MeasurementStateName semantics.
        state = {'CAP': 'CAPACITANCE', 'VOLT': 'VOLTAGE', 'RES': 'RESISTANCE'}[self.mode]
        return super().config().replace('state=CAPACITANCE', 'state=' + state)

    def send(self, text):
        if text.startswith('PROTO,') and self.bad_protocol:
            self.shared.append(b'PROTO,version=2,wires=ascii,ctrlMax=512,dataMax=2304,caps=config_query_v1,adcCrc=crc8\n')
            return
        if text.startswith('MAPP,') and self.lose_terminal and not self.lost_once:
            self.lost_once = True
            return
        if self.generating_frame and (self.wrong_legacy or self.future_frame):
            # Synthetic fault record, recomputed CRC: the identity/domain
            # validator, rather than a fabricated CRC failure, must reject it.
            lines = text.strip().splitlines()
            if lines[0].startswith('C,') and self.wrong_legacy:
                fields = parse_fields(lines[0]); fields.update(gen=str(self.mgen), rid=str(self.mrid))
                lines[0] = 'C,' + ','.join(f'{k}={v}' for k, v in fields.items())
                tail = parse_fields(lines[-1]); tail.update(gen=str(self.mgen), rid=str(self.mrid))
            elif self.future_frame:
                tag = lines[0].split(',', 1)[0]
                fields = parse_fields(lines[0]); fields['rev'] = str(self.rev + 1)
                lines[0] = tag + ',' + ','.join(f'{k}={v}' for k, v in fields.items())
                tail = parse_fields(lines[-1]); tail['rev'] = fields['rev']
            else:
                super().send(text)
                return
            tail['crc'] = f"{zlib.crc32(''.join(line + chr(10) for line in lines[:-1]).encode()):08X}"
            lines[-1] = 'K,' + ','.join(f'{k}={v}' for k, v in tail.items())
            text = '\n'.join(lines) + '\n'
        super().send(text)
        if text.startswith('CONFIG,') and self.saved_old_frame:
            self.shared.extend(self.saved_old_frame)
            self.saved_old_frame = None
        if text.startswith('MACK,') and self.inject_fault and not self.injected_fault:
            self.shared.append((self.inject_fault + '\n').encode())
            self.injected_fault = True

    def write(self, endpoint, data, timeout):
        if self.old_frame and b'MODE=VOLT' in data:
            self.old_frame = False
            existing, self.shared = self.shared, deque()
            self.generating_frame = True
            Firmware.frame(self)
            self.generating_frame = False
            self.saved_old_frame = list(self.shared)
            self.shared = existing
        return super().write(endpoint, data, timeout)

    def read_some(self, size, deadline):
        if self.missing_frames and not self.shared and self.rev > 1:
            time.sleep(min(.001, max(0, deadline - time.monotonic())))
            return None
        return super().read_some(size, deadline)

    def close(self):
        self.closed += 1


class Tests(unittest.TestCase):
    def options(self, directory, **kwargs):
        return replace(capture.Options(Path(directory), 'synthetic-50', total_seconds=5,
                       prepare_seconds=.8, cleanup_seconds=.5, command_timeout=.06,
                       frame_timeout=.15), **kwargs)

    def run_case(self, device, options):
        opened = []
        def factory(settings):
            opened.append(settings)
            return UsbCdcPort(capture.SERIAL,
                metadata_finder=lambda *a, **k: {'path': '/dev/synthetic-not-opened', 'usb_serial': capture.SERIAL,
                                               'vid': '303a', 'pid': '4042', 'interface_number': 0},
                port_factory=lambda _: device)
        start = time.monotonic()
        result = capture.run_capture(options, port_factory=factory)
        self.assertLess(time.monotonic() - start, options.total_seconds + .15)
        self.assertEqual(len(opened), 1)
        self.assertEqual(device.closed, 1)
        self.assertFalse(any(command.startswith(('USBSTREAM=', 'CAL=', 'USBBOOT', 'RESTART', 'RESET',
                                                'FPS=', 'ADSCHK=', 'RECOVER', 'SPI=')) for command in device.commands))
        self.assertFalse(result['input_flushed'])
        raw = Path(result['raw_file']).read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), result['raw_sha256'])
        self.assertEqual(len(raw), result['rx_bytes'])
        self.assertEqual(json.loads(Path(result['summary_file']).read_text()), result)
        events = [json.loads(line) for line in Path(result['events_file']).read_text().splitlines()]
        offset = 0
        for event in events:
            if event['type'] == 'rx':
                self.assertEqual(event['offset'], offset)
                offset += event['length']
        self.assertEqual(offset, len(raw))
        return result, events

    def test_expected_image_guard_precedes_fifty_plan_and_blocks_wrong_boot_cleanup(self):
        class BoundDevice(Device):
            def send(self, text):
                if text.startswith('BUILD,'):
                    text = text.replace('target=pty', 'target=esp32s3')
                super().send(text)
        for elf, boot, successful in (('ab'*32, 1, True), ('cd'*32, 1, False), ('ab'*32, 2, False)):
            with self.subTest(elf=elf, boot=boot), tempfile.TemporaryDirectory() as directory:
                device = BoundDevice()
                result, events = self.run_case(device, self.options(directory, expected_elf_sha256=elf,
                    expected_session='AABBCCDDEE112233', expected_boot_id=boot))
                self.assertTrue(result['image_authority']['required'])
                if successful:
                    self.assertEqual(result['status'], 'COMPLETE_50_IDENTITY_VERIFIED', result['error'])
                    self.assertEqual([x['purpose'] for x in result['image_authority']['brackets']], ['prepare', 'cleanup'])
                    self.assertEqual(result['verified_transitions'], 50)
                    first = next(i for i, e in enumerate(events) if e['type'] == 'first_mutation_authorized')
                    self.assertTrue(any(e['type'] == 'image_authority_verified' for e in events[:first]))
                else:
                    self.assertEqual(result['status'], 'ERROR')
                    self.assertEqual(result['attempted_transitions'], 0)
                    self.assertFalse(any('=' in c for c in device.commands))
                    self.assertEqual(result['safe_cap_recovery'], 'NOT_ATTEMPTED_AUTHORITY_UNVERIFIED')

    def test_fifty_real_changes_first_frames_atomic_config_and_cap_rows_domain(self):
        device = Device(old_frame=True)
        device.shared.append(b'PREEXISTING,synthetic=1\n')
        with tempfile.TemporaryDirectory() as directory:
            opts = self.options(directory)
            result, events = self.run_case(device, opts)
            self.assertEqual(result['status'], 'COMPLETE_50_IDENTITY_VERIFIED', result.get('error'))
            self.assertEqual(result['verified_transitions'], 50)
            self.assertEqual(result['remaining_transitions'], 0)
            self.assertEqual(result['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
            self.assertEqual(result['stale_queued_frames'], 1)
            expected = [('ROWMODES=' + capture.PROFILE) if mode == 'MIXED' else ('MODE=' + mode) for mode in capture.PLAN]
            self.assertEqual([c for c in device.commands if '=' in c], expected)
            self.assertEqual(len(result['transitions']), 50)
            self.assertTrue(all(step['transition']['verification'] == 'ACK_TERMINAL_CONFIG' for step in result['steps']))
            self.assertTrue(Path(result['raw_file']).read_bytes().startswith(b'PREEXISTING,synthetic=1\n'))
            for step in result['steps']:
                self.assertNotEqual(step['before_CONFIG']['mode'], step['target'])
                self.assertEqual(step['closing_CONFIG'], step['transition']['config'])
                header = step['first_frame']['header']
                self.assertEqual(tuple(header[k] for k in capture.benchmark.IDENTITY),
                                 tuple(step['closing_CONFIG'][k] for k in capture.benchmark.IDENTITY))
                if step['target'] == 'CAP':
                    self.assertEqual(header['gen'], header['rgen'])
                    self.assertNotEqual(header['gen'], header['mgen'])
            self.assertEqual(len([e for e in events if e['type'] == 'mode_switch_verified']), 50)
            original = Path(result['raw_file']).read_bytes()
            with self.assertRaises(FileExistsError):
                capture.run_capture(opts, port_factory=lambda _: self.fail('collision must fail before port open'))
            self.assertEqual(Path(result['raw_file']).read_bytes(), original)

    def test_lost_terminal_recovered_without_replaying_setter(self):
        with tempfile.TemporaryDirectory() as directory:
            device = Device(lose_terminal=True)
            result, _ = self.run_case(device, self.options(directory))
            self.assertEqual(result['status'], 'COMPLETE_50_IDENTITY_VERIFIED', result.get('error'))
            self.assertEqual(result['steps'][0]['transition']['verification'], 'ACK_CONFIG_RECOVERED_TERMINAL')
            self.assertEqual(len([c for c in device.commands if '=' in c]), 50)

    def test_missing_new_frame_fails_bounded_and_cleans_up_cap8(self):
        with tempfile.TemporaryDirectory() as directory:
            device = Device(missing_frames=True)
            result, _ = self.run_case(device, self.options(directory))
            self.assertEqual(result['status'], 'ERROR')
            self.assertIn('new_frame_deadline_expired', result['error'])
            self.assertEqual(result['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
            self.assertEqual(result['verified_transitions'], 0)
            self.assertEqual([c for c in device.commands if '=' in c], ['MODE=VOLT', 'MODE=CAP'])
            self.assertLess(result['elapsed_seconds'], .7)

    def test_fault_or_reset_fails_fast_even_before_first_frame(self):
        for fault in ('MFAULT,seq=17,err=0x108,synthetic=1', 'RST,reason=synthetic_reset'):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as directory:
                device = Device(fault=fault)
                result, _ = self.run_case(device, self.options(directory))
                self.assertIn('firmware_fault_or_reset', result['error'])
                self.assertEqual(result['status'], 'ERROR')
                self.assertEqual(result['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
                self.assertEqual([c for c in device.commands if '=' in c], ['MODE=VOLT', 'MODE=CAP'])
                self.assertIn(fault.encode(), Path(result['raw_file']).read_bytes())

    def test_failed_cleanup_retains_first_error_and_does_not_replay_setter(self):
        class MissingCleanupAck(Device):
            def write(self, endpoint, data, timeout):
                if b'MODE=CAP' in data:
                    self.lost_ack = True
                return super().write(endpoint, data, timeout)
        with tempfile.TemporaryDirectory() as directory:
            device = MissingCleanupAck(missing_frames=True)
            result, _ = self.run_case(device, self.options(directory))
            self.assertIn('new_frame_deadline_expired', result['error'])
            self.assertIn('transition_ack_unconfirmed:MODE=CAP', result['cleanup_error'])
            self.assertEqual(result['safe_cap_recovery'], 'FAILED_OR_UNVERIFIED')
            self.assertEqual([c for c in device.commands if '=' in c], ['MODE=VOLT', 'MODE=CAP'])

    def test_closing_atomic_config_detects_concurrent_identity_change(self):
        class ConcurrentRows(Device):
            changed_queries = 0
            def write(self, endpoint, data, timeout):
                if b'CONFIG?' in data and self.rev > 1:
                    self.changed_queries += 1
                    if self.changed_queries == 2:
                        self.rev += 1
                        self.rgen += 1
                        self.rrid = 500  # Synthetic external controller; no harness setter.
                return super().write(endpoint, data, timeout)
        with tempfile.TemporaryDirectory() as directory:
            device = ConcurrentRows()
            result, _ = self.run_case(device, self.options(directory))
            self.assertIn('configuration_changed_during_frame_verification', result['error'])
            self.assertEqual(result['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
            self.assertEqual(result['applied_transitions'], 1)
            self.assertEqual(result['verified_transitions'], 0)
            self.assertEqual([c for c in device.commands if '=' in c], ['MODE=VOLT', 'MODE=CAP'])

    def test_bad_authority_never_mutates_even_for_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            device = Device(bad_protocol=True)
            result, _ = self.run_case(device, self.options(directory))
            self.assertEqual(result['status'], 'ERROR')
            self.assertEqual(result['safe_cap_recovery'], 'NOT_ATTEMPTED_AUTHORITY_UNVERIFIED')
            self.assertFalse(any('=' in c for c in device.commands))

    def test_cap_wrong_legacy_mode_domain_and_future_revision_fail(self):
        for options, reason in ((dict(wrong_legacy=True), 'frame_legacy_domain_mismatch'),
                                (dict(future_frame=True), 'unexpected_future_frame_revision')):
            with self.subTest(options=options), tempfile.TemporaryDirectory() as directory:
                device = Device(**options)
                result, _ = self.run_case(device, self.options(directory))
                self.assertEqual(result['status'], 'ERROR')
                self.assertIn(reason, result['error'])
                self.assertEqual(result['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
                self.assertLessEqual(len([c for c in device.commands if '=' in c]), 2)

    def test_missing_rail_only_explicit_ram_calibration_and_no_save(self):
        with tempfile.TemporaryDirectory() as directory:
            device = Device()
            device.rail = False
            result, _ = self.run_case(device, self.options(directory))
            self.assertEqual(result['status'], 'COMPLETE_50_IDENTITY_VERIFIED', result.get('error'))
            self.assertTrue(result['volatile_rail_written'])
            setters = [c for c in device.commands if '=' in c]
            self.assertEqual(setters[0], 'RAILCFG=3350000,-1795000')
            self.assertEqual(len(setters), 51)
            self.assertIn('not precision', result['rail_source_note'])

    def test_absolute_budget_and_counter_wrap_are_bounded(self):
        with tempfile.TemporaryDirectory() as directory:
            device = Device(missing_frames=True)
            result, _ = self.run_case(device, self.options(directory, total_seconds=.4,
                                  prepare_seconds=.1, cleanup_seconds=.1, frame_timeout=10))
            self.assertEqual(result['status'], 'ERROR')
            self.assertLess(result['elapsed_seconds'], .45)
        self.assertTrue(capture.forward_or_equal(0, 0xffffffff))
        self.assertFalse(capture.forward_or_equal(0xffffffff, 0))
        with self.assertRaises(ValueError):
            capture.validate_options(capture.Options(Path('/tmp'), 'invalid', total_seconds=301))


if __name__ == '__main__':
    unittest.main()
