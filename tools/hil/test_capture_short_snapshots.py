#!/usr/bin/env python3
"""Synthetic CDC/clock regression only; never opens a board or BLE adapter."""
import contextlib
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import capture_short_snapshots as short
import test_usb_cdc_capture as cdc_test
from sensorarray_usb_cdc import UsbCdcPort
from run_firmware_benchmark import BenchmarkOptions
from test_usb_bulk_capture import SERIAL


class TxFirmware(cdc_test.CdcFirmware):
    def __init__(self, *, tx='rel', lose_setter=False, late_fault=False):
        super().__init__(optional_caps=short.cdc.shared.OPTIONAL_CAPABILITIES.values())
        self.tx = tx
        self.lose_setter = lose_setter
        self.late_fault = late_fault

    def write(self, endpoint, data, timeout):
        command = data.strip().decode()
        if command == 'TX?' or command.startswith('TX='):
            self.commands.append(command)
            if command.startswith('TX='):
                self.tx = command.split('=', 1)[1].lower()
                if self.late_fault and self.tx == 'rel':
                    self.shared.append(b'MFAULT,seq=999,err=0x108,state=DEGRADED,route=SAFE\n')
                if self.lose_setter and self.tx == 'short':
                    return len(data)  # Setter took effect; ACK was lost.
            self.shared.append(('ACK,cmd=TX,v=' + self.tx + '\n').encode())
            return len(data)
        return super().write(endpoint, data, timeout)


class ShortTests(unittest.TestCase):
    def capture(self, device, *, mode='CAP', rows=8, **identity):
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: cdc_test.CdcTests().metadata(),
                          port_factory=lambda _: device)
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            options = BenchmarkOptions(None, Path(directory), 'snap-short', transport='usb_cdc',
                usb_serial=SERIAL, mode=mode, rows=rows, capture_seconds=.06,
                warmup_seconds=.01, prepare_seconds=2, cleanup_seconds=1, command_timeout=.06, **identity)
            with short.cdc.shared.bulk_collector_selected(short.ShortSnapshotCollector):
                index = short.cdc.shared.base.run_benchmark(options, port_factory=lambda _: port)
            report = json.loads((Path(directory) / 'snap-short.summary.json').read_text())
            raw = Path(report['raw_file']).read_bytes()
            events = [json.loads(line) for line in Path(report['events_file']).read_text().splitlines()]
            self.assertEqual(sum(e['type'] == 'base_finished' for e in events), 1)
            self.assertEqual(sum(e['type'] == 'finished' for e in events), 1)
            self.assertEqual(events[-1]['type'], 'finished')
            for key in ('status', 'error', 'rx_bytes', 'tx_restore', 'firmware_fault_events', 'reset_event_count'):
                self.assertEqual(events[-1][key], report[key], key)
        return index, report, raw

    def test_expected_image_is_bound_before_short_setter_and_wrong_image_has_no_mutation(self):
        class BoundTxFirmware(TxFirmware):
            def send(self, text):
                if text.startswith('BUILD,'):
                    text = text.replace('target=pty', 'target=esp32s3')
                super().send(text)
        for elf, status in (('ab'*32, 'COMPLETE'), ('cd'*32, 'ERROR')):
            with self.subTest(elf=elf):
                device = BoundTxFirmware()
                index, report, _ = self.capture(device, expected_elf_sha256=elf,
                    expected_session='AABBCCDDEE112233', expected_boot_id=1)
                self.assertEqual(index['status'], status, report.get('error'))
                self.assertTrue(report['image_authority']['required'])
                if status == 'COMPLETE':
                    self.assertEqual(report['tx_restore'], 'QUERY_VERIFIED')
                    self.assertEqual(device.tx, 'rel')
                    self.assertEqual(report['image_authority']['first_mutation']['expected']['elfSha256'], elf)
                else:
                    self.assertFalse(any('=' in c for c in device.commands))
                    self.assertFalse(report['tx_change_attempted'])
                    self.assertEqual(report['safe_cap_recovery'], 'NOT_ATTEMPTED_AUTHORITY_UNVERIFIED')

    def test_actual_collector_preserves_duration_raw_restore_and_scope(self):
        device = TxFirmware()
        index, report, raw = self.capture(device)
        self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
        self.assertTrue(report['capture_duration_satisfied'])
        self.assertEqual(report['tx_restore'], 'QUERY_VERIFIED')
        self.assertEqual(report['safe_cap_recovery'], 'CAP8_QUERY_VERIFIED')
        self.assertEqual(device.tx, 'rel')
        self.assertEqual([x for x in device.commands if x.startswith('TX=')], ['TX=SHORT', 'TX=REL'])
        self.assertIn(b'ACK,cmd=TX,v=short\n', raw)
        self.assertIn(b'ACK,cmd=TX,v=rel\n', raw)
        self.assertEqual(report['rx_bytes'], len(raw))
        self.assertFalse(report['normal_FULL_loss_budget_applicable'])
        self.assertEqual(report['snapshot_acceptance'], 'NOT_EVALUATED')

    def test_lost_setter_reply_restores_without_replaying_setter(self):
        device = TxFirmware(lose_setter=True)
        _, report, _ = self.capture(device)
        self.assertEqual(report['status'], 'ERROR')
        self.assertEqual(report['tx_restore'], 'QUERY_VERIFIED')
        self.assertEqual(device.tx, 'rel')
        self.assertEqual(device.commands.count('TX=SHORT'), 1)
        self.assertEqual(device.commands.count('TX=REL'), 1)

    def test_original_short_remains_short_and_unknown_never_mutates(self):
        for value, expected in [('short', 'CAPTURED'), ('unknown', 'ERROR')]:
            with self.subTest(value=value):
                device = TxFirmware(tx=value)
                _, report, _ = self.capture(device)
                self.assertEqual(report['status'], expected, report.get('error'))
                self.assertEqual(device.tx, value)
                self.assertFalse(any(x.startswith('TX=') for x in device.commands))

    def test_wrong_configuration_does_not_mutate_tx(self):
        device = TxFirmware()
        _, report, _ = self.capture(device, rows=1)
        self.assertEqual(report['status'], 'ERROR')
        self.assertIn('requires_CAP8', report['error'])
        self.assertFalse(any(x.startswith('TX=') for x in device.commands))

    def test_restore_fault_is_retained_and_invalidates_success(self):
        device = TxFirmware(late_fault=True)
        index, report, raw = self.capture(device)
        self.assertEqual(index['status'], 'ERROR')
        self.assertEqual(report['status'], 'ERROR')
        self.assertIn('tx_restore_new_fault_or_reset', report['error'])
        self.assertEqual(report['firmware_fault_events'], 1)
        self.assertEqual(report['safe_cap_recovery'], 'INVALIDATED_BY_LATER_FAULT_OR_RESET')
        self.assertEqual(report['prior_cap_recovery'], 'CAP8_QUERY_VERIFIED')
        self.assertEqual(report['rx_bytes'], len(raw))
        self.assertIn(b'MFAULT,seq=999,', raw)

    def test_restore_reset_after_bounded_sample_list_is_full(self):
        class ResetFirmware(TxFirmware):
            def __init__(self):
                super().__init__()
                for i in range(16):
                    self.shared.append(('RST,syntheticHistory=' + str(i) + '\n').encode())
            def write(self, endpoint, data, timeout):
                if data.strip() == b'TX=REL':
                    self.shared.append(b'RST,syntheticRestore=17\n')
                return super().write(endpoint, data, timeout)
        index, report, raw = self.capture(ResetFirmware())
        self.assertEqual(index['status'], 'ERROR')
        self.assertEqual(report['status'], 'ERROR')
        self.assertEqual(report['reset_event_count'], 17)
        self.assertEqual(len(report['reset_observations']), 16)
        self.assertEqual(raw.count(b'RST,'), 17)
        self.assertEqual(report['safe_cap_recovery'], 'INVALIDATED_BY_LATER_FAULT_OR_RESET')

    def test_restore_never_extends_total_deadline_and_keeps_primary_failure(self):
        collector = object.__new__(short.ShortSnapshotCollector)
        collector.events = io.StringIO()
        collector.tx_change_attempted = True
        collector.original_tx = 'full'
        collector.tx_restore = 'NOT_NEEDED'
        collector.total_deadline = 9
        collector.options = SimpleNamespace(command_timeout=4)
        collector.event = lambda *a, **k: None
        collector.fault_events = 0
        collector.faults = []
        collector.reset_observations = []
        collector.reset_event_count = 0
        collector.interrupted = False
        collector.raw_bytes = collector.tx_bytes = collector.partial_writes = 0
        collector.started = 0
        collector.transport_result = lambda: {}
        deadlines = []
        def restore(mode, deadline):
            deadlines.append(deadline)
            raise short.CaptureError('phase_deadline_expired')
        collector.set_tx_once = restore
        with patch.object(short.cdc.CdcBenchmarkCollector, 'run',
                          return_value={'status': 'ERROR', 'error': 'original_failure'}), \
             patch.object(short.time, 'monotonic', return_value=10):
            report = collector.run()
        self.assertEqual(deadlines, [9])
        self.assertEqual(report['error'], 'original_failure')
        self.assertEqual(report['tx_restore'], 'FAILED_OR_UNKNOWN')


if __name__ == '__main__':
    unittest.main()
