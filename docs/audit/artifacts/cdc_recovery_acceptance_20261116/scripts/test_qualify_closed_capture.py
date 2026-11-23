#!/usr/bin/env python3
"""Synthetic PTY and offline gate regressions; never new board measurements."""
from copy import deepcopy
from contextlib import redirect_stdout
import io
import base64
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import qualify_closed_capture as gate
import test_firmware_benchmark as fixture
from capture_reference_fixture import PosixPort
from run_firmware_benchmark import run_benchmark

ELF = 'a' * 64
EXPECTED = gate.ExpectedCaptureIdentity(ELF, 'AABBCCDDEE112233', 1)
IDENTITY = dict(session=EXPECTED.session, bootId='1', rev='1', mgen='2', mrid='3',
                rgen='4', rrid='5', pgen='6', prid='7')


def synthetic_report(missing=2, received=100):
    counts = dict(ordered_integrity_valid_frames=received, missing_frames=0)
    group = {'identity': dict(IDENTITY), 'host_complete_frame_intervals': {
        'known': True, 'first_complete_host_seconds': .05, 'last_complete_host_seconds': 119.95},
        'rates': {'device_span_seconds': 119.9, 'capture_fps': 65.0}}
    return {'ble_journal': {'schema_valid': True, 'recording_end_observed': True,
                           'collector_summary_verification': {'status': 'MATCH'},
                           'whole_journal_critical_records': {}, 'all_fragment_counters_by_channel': {}},
            'captures': [{'label': 'synthetic', 'status': 'RECORDED', 'requested': {'requested_seconds': 120},
                          'window': {'requested_seconds_met': True, 'capture_start_seconds': 0, 'capture_end_seconds': 120,
                                     'capture_monotonic_start': 0, 'capture_monotonic_end': 120},
                          'expected_config_identity': dict(IDENTITY), 'configuration_groups': [deepcopy(group)],
                          'capture_counts': dict(counts), 'ble': {
                              'status': 'RECORDED', 'configuration_groups': [deepcopy(group)],
                              'capture_frame_counts': {**counts, 'missing_frames': missing},
                              'fragment_integrity': {'fully_contained_by_channel': {}, 'edge_or_unknown_by_channel': {}},
                              'fully_contained_data_diagnostics': {}, 'USB_comparison': {}}}]}


class DecisionTests(unittest.TestCase):
    def assess(self, report):
        return gate.assess_science(report, 'synthetic', EXPECTED)

    def test_normal_gap_denominator_and_fixed_budget(self):
        self.assertEqual(self.assess(synthetic_report(1, 39))['status'], 'PASS_NUMERIC_SCREEN')
        result = self.assess(synthetic_report(1, 38))
        self.assertEqual(result['status'], 'FAIL')
        self.assertEqual(result['missing_fraction'], 1 / 39)
        self.assertEqual(result['overall_acceptance'], 'NOT_ESTABLISHED')

    def test_two_adjacent_frames_do_not_prove_full_window_coverage(self):
        report = synthetic_report(0, 2)
        capture = report['captures'][0]
        capture['capture_counts']['ordered_integrity_valid_frames'] = 10000
        ble = capture['ble']
        ble['USB_comparison']['USB_frames_without_BLE_match'] = 9998
        group = ble['configuration_groups'][0]
        group['host_complete_frame_intervals']['last_complete_host_seconds'] = .065
        group['rates']['device_span_seconds'] = .015
        result = self.assess(report)
        self.assertEqual(result['missing_fraction'], 0)
        self.assertEqual(result['status'], 'UNKNOWN')
        self.assertIn('BLE_full_window_observation_insufficient', result['unknown'])
        # Two old adjacent frames delivered at opposite host edges remain
        # insufficient: their actual device timestamp span is still .015 s.
        group['host_complete_frame_intervals']['last_complete_host_seconds'] = 119.95
        self.assertEqual(self.assess(report)['status'], 'UNKNOWN')

    def test_period_over_one_second_or_missing_clock_requires_review(self):
        report = synthetic_report(0)
        report['captures'][0]['ble']['configuration_groups'][0]['rates']['capture_fps'] = .5
        self.assertEqual(self.assess(report)['status'], 'UNKNOWN')
        del report['captures'][0]['configuration_groups'][0]['host_complete_frame_intervals']
        self.assertEqual(self.assess(report)['status'], 'UNKNOWN')

    def test_unknown_incomplete_or_old_analysis_cannot_pass(self):
        for edit in ('closed', 'summary', 'span', 'fault_scope'):
            with self.subTest(edit=edit):
                report = synthetic_report()
                if edit == 'closed': report['ble_journal']['recording_end_observed'] = False
                if edit == 'summary': report['ble_journal']['collector_summary_verification']['status'] = 'PREFIX_ONLY'
                if edit == 'span': del report['captures'][0]['ble']['fragment_integrity']
                if edit == 'fault_scope': del report['ble_journal']['whole_journal_critical_records']
                self.assertEqual(self.assess(report)['status'], 'UNKNOWN')

    def test_window_errors_and_whole_journal_control_faults_are_not_loss_tolerance(self):
        for kind in ('ascii_crc_errors', 'identity_errors', 'wire_contract_errors', 'duplicate_or_old_frames'):
            with self.subTest(kind=kind):
                report = synthetic_report(0)
                report['captures'][0]['ble']['capture_frame_counts'][kind] = 1
                self.assertEqual(self.assess(report)['status'], 'FAIL')
        for key, value in [('whole_journal_critical_records', {'fault_L': 1}),
                           ('all_fragment_counters_by_channel', {'C': {'crc_fail': 1}})]:
            report = synthetic_report(0)
            report['ble_journal'][key] = value
            self.assertEqual(self.assess(report)['status'], 'FAIL')

    def test_edge_crc_retained_not_window_loss_but_unknown_malformed_is_unknown(self):
        report = synthetic_report(0)
        fragments = report['captures'][0]['ble']['fragment_integrity']
        fragments['edge_or_unknown_by_channel'] = {'D': {'crc_fail': 1}}
        result = self.assess(report)
        self.assertEqual(result['status'], 'PASS_NUMERIC_SCREEN')
        self.assertEqual(result['edge_or_unknown_fragment_diagnostics']['D']['crc_fail'], 1)
        fragments['fully_contained_by_channel'] = {'D': {'crc_fail': 1}}
        self.assertEqual(self.assess(report)['status'], 'FAIL')
        fragments['fully_contained_by_channel'] = {}
        fragments['edge_or_unknown_by_channel']['D']['dropped'] = 1
        self.assertEqual(self.assess(report)['status'], 'UNKNOWN')

    def test_invalid_points_are_preserved_and_cross_boot_or_raw_mismatch_fails(self):
        report = synthetic_report(0)
        report['captures'][0]['ble']['capture_frame_counts']['invalid_points'] = 500
        self.assertEqual(self.assess(report)['status'], 'PASS_NUMERIC_SCREEN')
        report['captures'][0]['ble']['configuration_groups'][0]['identity']['bootId'] = '2'
        self.assertEqual(self.assess(report)['status'], 'FAIL')
        report = synthetic_report(0)
        report['captures'][0]['ble']['USB_comparison']['different_wire_bytes_sha256'] = 1
        self.assertEqual(self.assess(report)['status'], 'FAIL')


class BoundFirmware(fixture.Firmware):
    """Synthetic source identity, with the real collector/PTY reader/writer."""
    def reply(self, command):
        if command == 'BUILD?':
            self.commands.append(command)
            self.send('BUILD,idf=synthetic,target=esp32s3,project=SensorArray,proto=2,version=fixture,elfSha256=' + ELF + '\n')
        elif command == 'PROTO?':
            self.commands.append(command)
            self.send('PROTO,version=2,caps=config_query_v1/config_identity_v1/usb_cdc_data_v1\n')
        elif command == 'USB?':
            self.commands.append(command)
            self.send('USB,backend=otg_cdc_v1,init=1,mounted=1,err=0x0,synthetic=1\n')
        else:
            super().reply(command)


class SyntheticCdcPty(PosixPort):
    # Explicit test double. No USB controller or device is opened by this test.
    transport = 'usb_cdc'


class AuthorityTraceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        device = BoundFirmware()
        device.profile = 'FULL'
        try:
            options = fixture.BenchmarkTests().options(
                device, self.directory, port=None, transport='usb_cdc', usb_serial='001122334455', usb_pid=0x4042,
                expected_elf_sha256=ELF, expected_session=EXPECTED.session, expected_boot_id=1)
            with redirect_stdout(io.StringIO()):
                index = run_benchmark(options, port_factory=lambda _: SyntheticCdcPty(device.path))
            self.assertEqual(index['status'], 'COMPLETE', index)
            self.summary_path = Path(index['results'][0]['summary_file'])
            self.summary = json.loads(self.summary_path.read_text())
            self.events_path = Path(self.summary['events_file'])
            self.events = gate.events_file(self.events_path)
        finally:
            device.close()

    def tearDown(self):
        self.temporary.cleanup()

    def verify(self):
        return gate.verify_formal(self.summary_path, EXPECTED, .1)

    def write_events(self):
        self.events_path.write_text(''.join(json.dumps(e) + '\n' for e in self.events))

    def test_actual_collector_trace_has_bound_prepare_cleanup_and_real_CAP8_reply(self):
        _, evidence = self.verify()
        self.assertEqual([b['purpose'] for b in evidence['actual_read_brackets']], ['prepare', 'cleanup'])

    def test_summary_only_wrong_ELF_and_missing_final_finished_rejected(self):
        self.summary['image_authority']['brackets'][0]['BUILD']['elfSha256'] = 'b' * 64
        self.summary_path.write_text(json.dumps(self.summary))
        with self.assertRaises(ValueError): self.verify()
        self.events.pop()
        self.write_events()
        with self.assertRaisesRegex(ValueError, 'finished'): self.verify()

    def test_setter_before_image_bracket_and_forged_reply_raw_binding_rejected(self):
        original = deepcopy(self.events)
        first = next(i for i, e in enumerate(self.events) if e['type'] == 'tx_begin')
        self.events[first]['command'] = 'RESTART'
        self.events[first]['wire_ascii'] = 'RESTART\n'
        self.write_events()
        with self.assertRaises(ValueError): self.verify()
        self.events = original
        reply = next(e for e in self.events if e['type'] == 'reply' and e.get('command') == 'BUILD?')
        reply['line'] = reply['line'].replace(ELF, 'b' * 64)
        self.write_events()
        with self.assertRaisesRegex(ValueError, 'raw'): self.verify()

    def test_final_failed_cleanup_and_noncontiguous_RX_are_unknown(self):
        self.summary['safe_cap_recovery'] = 'FAILED_OR_UNKNOWN'
        self.summary_path.write_text(json.dumps(self.summary))
        with self.assertRaises(ValueError): self.verify()
        self.summary['safe_cap_recovery'] = 'CAP8_QUERY_VERIFIED'
        self.summary_path.write_text(json.dumps(self.summary))
        next(e for e in self.events if e['type'] == 'rx')['offset'] = 1
        self.write_events()
        with self.assertRaisesRegex(ValueError, 'contiguous'): self.verify()


class FullClosedCaseTests(unittest.TestCase):
    """Actual collectors/parser/gate over synthetic in-memory CDC, no hardware."""
    def test_full_composed_gate_and_observer_failures(self):
        import test_usb_cdc_capture as cdc
        from sensorarray_usb_cdc import UsbCdcPort
        from test_usb_bulk_capture import SERIAL
        from run_firmware_benchmark import BenchmarkOptions
        class Bound(cdc.CdcFirmware):
            def send(self, text):
                if text.startswith('BUILD,'): text = text.replace('target=pty', 'target=esp32s3')
                super().send(text)
        device = Bound(optional_caps={'ble_flow_v1', 'ble_snapshot_v1'}, output_values={
            'BLEFLOW?': dict(active='1', gen='3', stream='1', stage='full'),
            'BLESNAP?': dict(gen='3', stream='1')})
        port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: cdc.CdcTests().metadata(), port_factory=lambda _: device)
        expected = gate.ExpectedCaptureIdentity('ab' * 32, EXPECTED.session, 1)
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            options = BenchmarkOptions(None, directory, 'synthetic', transport='usb_cdc', usb_serial=SERIAL,
                mode='VOLT', rows=1, capture_seconds=.12, warmup_seconds=.01, prepare_seconds=2, cleanup_seconds=1,
                command_timeout=.1, expected_elf_sha256=expected.elf_sha256, expected_session=expected.session, expected_boot_id=1)
            with redirect_stdout(io.StringIO()): index = cdc.stage.run_benchmark(options, port_factory=lambda _: port)
            self.assertEqual(index['status'], 'COMPLETE', index)
            formal_path = directory / 'synthetic.summary.json'
            formal = gate.load(formal_path)
            events = gate.events_file(Path(formal['events_file']))
            raw = Path(formal['raw_file']).read_bytes()
            start, end = formal['capture_monotonic_start'], formal['capture_monotonic_end']
            origin = start - next(e['t'] for e in events if e['type'] == 'capture_start')
            def event(kind, seconds, **fields):
                return {'schema_version': 1, 'type': kind, 'monotonic_ns': round(seconds * 1e9), **fields}
            records = [event('metadata', start - 1, collector='capture_benchmark_ble'),
                       event('connected', start - .5), event('subscriptions_ready', start - .4,
                                                               all_channels_subscribed=['C', 'D', 'L'])]
            buffer, packet = b'', None
            for e in events:
                if e['type'] != 'rx': continue
                buffer += raw[e['offset']:e['offset'] + e['length']]
                while b'\n' in buffer:
                    wire, _, buffer = buffer.partition(b'\n')
                    tag = wire.split(b',', 1)[0]
                    if tag in (b'C', b'V', b'R', b'M') and b',seq=' in wire:
                        packet = b''
                    if packet is not None and (tag in (b'C', b'V', b'R', b'M', b'MR', b'K') or tag in (b'D0', b'D1', b'D2', b'D3', b'P0', b'P1', b'P2', b'P3')):
                        packet += wire + b'\n'
                    if tag == b'K' and packet is not None:
                        when = origin + e['t']
                        if start <= when <= end:
                            records.append(event('notification', when, channel='D', length=len(packet),
                                                 payload_base64=base64.b64encode(packet).decode()))
                        packet = None
            records.extend([event('recording_end', end + .02), event('disconnected', end + .03)])
            journal = directory / 'synthetic.ble.notifications.jsonl'
            journal.write_text(''.join(json.dumps(e) + '\n' for e in records))
            ble_path = directory / 'synthetic.ble.summary.json'
            ble = {'status': 'RECORDED', 'error': None, 'cleanup_errors': [], 'unexpected_disconnect': False,
                   'journal_file': str(journal), 'journal_sha256': hashlib.sha256(journal.read_bytes()).hexdigest(),
                   'journal_bytes': journal.stat().st_size}
            ble_path.write_text(json.dumps(ble))
            debug_path = directory / 'synthetic.debug.summary.json'
            debug_raw = directory / 'synthetic.debug.rx.bin'; debug_raw.write_bytes(b'')
            debug_events = directory / 'synthetic.debug.events.jsonl'
            metadata = {'usb_serial': SERIAL, 'interface_number': 2}
            debug_events.write_text(json.dumps({'type': 'opened', 'monotonic': start - .2, 'metadata': metadata}) + '\n' +
                                    json.dumps({'type': 'finished', 'monotonic': end + .02, 'status': 'STOPPED', 'error': None}) + '\n')
            debug = {'status': 'STOPPED', 'error': None, 'faults': [], 'reset_observations': [],
                     'metadata': metadata, 'raw_file': str(debug_raw), 'events_file': str(debug_events),
                     'raw_sha256': hashlib.sha256(b'').hexdigest(), 'rx_bytes': 0, 'commands_written': 0}
            debug_path.write_text(json.dumps(debug))
            result = gate.qualify_closed_case(formal_path, ble_path, debug_path, expected, .1)
            self.assertEqual(result['status'], 'PASS_CASE_SCREEN', result.get('science'))
            self.assertEqual(result['overall_acceptance'], 'NOT_ESTABLISHED')
            ble['unexpected_disconnect'] = True; ble_path.write_text(json.dumps(ble))
            with self.assertRaisesRegex(ValueError, 'BLE_observer'): gate.qualify_closed_case(formal_path, ble_path, debug_path, expected, .1)
            ble['unexpected_disconnect'] = False; ble_path.write_text(json.dumps(ble))
            debug['metadata']['usb_serial'] = '000000000000'; debug_path.write_text(json.dumps(debug))
            with self.assertRaisesRegex(ValueError, 'debug_board'): gate.qualify_closed_case(formal_path, ble_path, debug_path, expected, .1)


if __name__ == '__main__':
    unittest.main()
