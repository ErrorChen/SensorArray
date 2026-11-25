#!/usr/bin/env python3
"""Synthetic diagnostic fixtures and actual collector/formatter-schema tests.

No board is opened. Diagnostic lines below are synthetic, not original logs.
"""
import contextlib
import gzip
import importlib.machinery
import importlib.util
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
import sys

import analyze_ble_protection as analyzer
from test_bulk_stage import StageFirmware, output_fields, stage, port_for, Firmware

# The v1 analyzer recognizes the exact collector digest. Load that frozen input
# only with the synthetic CDC port and the current test module search path.
legacy_path = stage.ROOT / 'tests/fixtures/hil/short_v1/source/capture_short_snapshots.py.txt'
loader = importlib.machinery.SourceFileLoader('legacy_short_snapshot_fixture', str(legacy_path))
short = importlib.util.module_from_spec(importlib.util.spec_from_loader(loader.name, loader))
original_search_path = sys.path[:]
try:
    loader.exec_module(short)
finally:
    sys.path[:] = original_search_path
import capture_short_snapshots as current_short
import test_usb_cdc_capture as cdc_fixture
from sensorarray_usb_cdc import UsbCdcPort
from test_usb_bulk_capture import SERIAL


def fixture(transport='usb_cdc'):
    identity = analyzer.parse_line(Firmware().config(), 'CONFIG')
    summary = {'status': 'CAPTURED', 'transport': transport, 'snapshots': {}, 'reset_observations': [],
               'capture_monotonic_start': 10., 'capture_monotonic_end': 20.,
               'optional_service_diagnostics': {c: {'capability': stage.OPTIONAL_CAPABILITIES[c], 'advertised': True}
                                                for c in analyzer.COMMANDS}}
    for phase, offset in (('before', 0.), ('after', 20.)):
        window = {'CONFIG?': {'fields': dict(identity), 'monotonic': offset + 1.},
                  'CONFIG?_closing': {'fields': dict(identity), 'started_monotonic': offset + 2., 'monotonic': offset + 2.1},
                  'BLELINK?': {'fields': {'gen': '7'}}, 'BLETX?': {'fields': {'gen': '7'}}}
        for index, command in enumerate(analyzer.COMMANDS):
            fields = output_fields(command)
            fields.update({key: '10' if phase == 'before' else '15' for key in analyzer.COUNTERS[command]})
            if command in ('BLEFLOW?', 'BLESNAP?'): fields.update(gen='7', stream='9')
            if command == 'BLEFLOW?':
                fields.update(active='1', stage='full')
                if phase == 'after': fields.update(stage='pressure', evidence='1', rejectPm='150', localServiceMilliFps='64000')
            if command == 'BLESNAP?':
                fields.update(known='1', lastSeq='100' if phase == 'before' else '200', lastRev=identity['rev'],
                              lastCompleteAgeUs='5000')
                if phase == 'after': fields.update(gapKnown='1', maxCompleteGapUs='1200000', periodicPending='1')
            start = offset + 1.1 + index * .1
            window[command] = {'fields': fields, 'started_monotonic': start,
                               'reply_event_monotonic': start + .02, 'monotonic': start + .03,
                               'reply_channel': 'ctrl' if transport == 'usb_bulk' else 'data'}
        summary['snapshots'][phase] = window
    return summary


def evidence(summary):
    raw, replies = {'data': set(), 'ctrl': set()}, set()
    channel = 'ctrl' if summary['transport'] == 'usb_bulk' else 'data'
    for phase, window in summary['snapshots'].items():
        for command, record in window.items():
            if command not in (*analyzer.COMMANDS, 'CONFIG?', 'CONFIG?_closing'): continue
            command = 'CONFIG?' if command.startswith('CONFIG?') else command
            line = command[:-1] + ',' + ','.join(f'{k}={v}' for k, v in record['fields'].items())
            record['reply_line'] = line
            raw[channel].add(line); replies.add((phase, command, line))
    return raw, replies


def run(summary):
    return analyzer.analyze_document(summary, *evidence(summary))


def write_fixture(directory, summary=None, compressed=False):
    summary = fixture() if summary is None else summary
    raw, replies = evidence(summary)
    for channel in ('data', 'ctrl'):
        if channel == 'ctrl' and summary['transport'] != 'usb_bulk': continue
        # These are synthetic diagnostic records only, not fabricated frames.
        payload = ('\n'.join(sorted(raw[channel])) + '\n').encode()
        path = directory / (channel + '.bin')
        if compressed:
            with gzip.open(path.with_suffix('.bin.gz'), 'wb') as target: target.write(payload)
        else: path.write_bytes(payload)
        summary['raw_file' if channel == 'data' else 'ctrl_raw_file'] = str(path)
        summary['raw_sha256' if channel == 'data' else 'ctrl_raw_sha256'] = hashlib.sha256(payload).hexdigest()
    events = [dict(type='reply', phase=p, command=c, line=line) for p, c, line in sorted(replies)]
    events.append(dict(type='finished', status=summary['status'], error=None))
    path = directory / 'events.jsonl'
    path.write_text(''.join(json.dumps(e) + '\n' for e in events))
    summary['events_file'] = str(path)
    path = directory / 'formal.summary.json'
    path.write_text(json.dumps(summary))
    return path


def short_fixture(directory, *, late_fault=False, late_reset=False, lost_restore=False, collector_module=short):
    if collector_module is short:
        assert hashlib.sha256(Path(short.__file__).read_bytes()).hexdigest() == analyzer.LEGACY_SHORT_SOURCE
    class Device(cdc_fixture.CdcFirmware):
        def __init__(self):
            super().__init__(optional_caps=stage.OPTIONAL_CAPABILITIES.values())
            self.tx = 'rel'

        def send(self, value):
            for command in ('BLEFLOW?', 'BLESNAP?'):
                self.output_values.setdefault(command, {}).update(gen=str(self.mgen), stream='9')
            super().send(value)

        def write(self, endpoint, data, timeout):
            command = data.strip().decode()
            if lost_restore and data.strip() == b'TX=REL':
                self.commands.append('TX=REL'); self.tx = 'rel'
                return len(data)  # Synthetic lost ACK; cannot claim restoration from state.
            if late_reset and data.strip() == b'TX=REL':
                self.shared.append(b'RST,syntheticRestore=17\n')
            if command == 'TX?' or command.startswith('TX='):
                self.commands.append(command)
                if command.startswith('TX='):
                    self.tx = command.split('=', 1)[1].lower()
                    if late_fault and self.tx == 'rel':
                        self.shared.append(b'MFAULT,seq=999,err=0x108,state=DEGRADED,route=SAFE\n')
                self.shared.append(('ACK,cmd=TX,v='+self.tx+'\n').encode())
                return len(data)
            return super().write(endpoint, data, timeout)
    from run_firmware_benchmark import BenchmarkOptions
    device = Device()
    if late_reset:
        for i in range(16):
            device.shared.append(('RST,syntheticHistory=' + str(i) + '\n').encode())
    port = UsbCdcPort(SERIAL, metadata_finder=lambda *a, **k: cdc_fixture.CdcTests().metadata(), port_factory=lambda _: device)
    options = BenchmarkOptions(None, directory, 'short-roundtrip', transport='usb_cdc', usb_serial=SERIAL,
        mode='CAP', rows=8, capture_seconds=.03, warmup_seconds=.01, prepare_seconds=2,
        cleanup_seconds=1, command_timeout=.06)
    with contextlib.redirect_stdout(io.StringIO()), collector_module.cdc.shared.bulk_collector_selected(collector_module.ShortSnapshotCollector):
        collector_module.cdc.shared.base.run_benchmark(options, port_factory=lambda _: port)
    return directory / 'short-roundtrip.summary.json'


class ProtectionAnalysis(unittest.TestCase):
    def test_independent_query_intervals_local_counters_and_real_gap(self):
        result = run(fixture())
        self.assertEqual(result['configuration']['identity']['bootId'], '1')
        self.assertEqual(result['BLE_epoch']['stream'], '9')
        for command, row in result['queries'].items():
            self.assertEqual(row['status'], 'OBSERVED_LOCAL_COUNTERS', row)
            self.assertTrue(all(value['delta'] == 5 for value in row['counter_deltas'].values()), command)
            self.assertAlmostEqual(row['host_snapshot_interval_seconds']['minimum'], 19.97)
            self.assertAlmostEqual(row['host_snapshot_interval_seconds']['maximum'], 20.03)
        flow, snapshot = (result['queries'][c] for c in ('BLEFLOW?', 'BLESNAP?'))
        self.assertNotEqual(flow['before']['host_query_start_monotonic'], snapshot['before']['host_query_start_monotonic'])
        self.assertEqual(flow['after']['observed_state']['stage'], 'pressure')
        self.assertEqual(flow['after']['observed_state']['ratePm'], '1000')
        self.assertEqual(snapshot['before']['observed_state']['maxCompleteGapUs'], 'unknown')
        self.assertEqual(snapshot['after']['observed_state']['maxCompleteGapUs'], '1200000')
        self.assertEqual(snapshot['after']['observed_state']['periodicPending'], '1')
        self.assertEqual(result['PC_missing_frames_adjusted'], 'NOT_COMPUTED')
        self.assertEqual(result['overall_acceptance'], 'NOT_EVALUATED')
        self.assertEqual(result['PC_snapshot_receipt'], 'unknown')

    def test_bounded_mod32_mod64_wrap_reset_and_unknown(self):
        for bits in (32, 64):
            self.assertEqual(analyzer.bounded_delta(str((1 << bits) - 2), '3', bits, 100)['delta'], 5)
            self.assertEqual(analyzer.bounded_delta('100', '0', bits, 1000)['delta'], 'unknown')
            self.assertEqual(analyzer.bounded_delta('0', '101', bits, 100)['delta'], 'unknown')
            self.assertEqual(analyzer.bounded_delta('0', str(1 << (bits - 1)), bits, 1 << bits)['delta'], 'unknown')
        summary = fixture()
        for command, key, bits in [('BLEFLOW?', 'intentionalSkip', 64), ('BLESNAP?', 'queued', 32)]:
            summary['snapshots']['before'][command]['fields'][key] = str((1 << bits) - 2)
            summary['snapshots']['after'][command]['fields'][key] = '3'
        result = run(summary)
        self.assertEqual(result['queries']['BLEFLOW?']['counter_deltas']['intentionalSkip']['delta'], 5)
        summary['snapshots']['after']['SNAPNET?']['fields']['queued'] = '0'
        result = run(summary)['queries']['SNAPNET?']
        self.assertEqual(result['status'], 'PARTIAL_UNKNOWN_COUNTERS')
        self.assertEqual(result['counter_deltas']['queued']['delta'], 'unknown')
        self.assertEqual(result['counter_deltas']['bleExpired']['delta'], 5)

    def test_cross_boot_CONFIG_or_reset_never_compares_counters(self):
        for key, value in [('rev', '2'), ('bootId', '2'), ('session', 'FFFF0000FFFF0000')]:
            with self.subTest(key=key):
                summary = fixture(); summary['snapshots']['after']['CONFIG?_closing']['fields'][key] = value
                result = run(summary)
                self.assertTrue(all(row['status'] == 'UNKNOWN' and not row['counter_deltas'] for row in result['queries'].values()))
        summary = fixture(); summary['reset_observations'] = [{'type': 'boot'}]
        self.assertIn('reset_observed', run(summary)['configuration']['reason'])

    def test_connection_stream_and_sender_generation_changes_invalidate_pairing(self):
        for phase, command, field in [('after', 'BLEFLOW?', 'stream'), ('after', 'BLESNAP?', 'gen'),
                                       ('before', 'BLELINK?', 'gen'), ('after', 'BLETX?', 'gen')]:
            summary = fixture(); summary['snapshots'][phase][command]['fields'][field] = '4294967295'
            result = run(summary)
            self.assertEqual(result['BLE_epoch']['status'], 'UNKNOWN')
            self.assertTrue(all(not row['counter_deltas'] for row in result['queries'].values()))

    def test_pair_duration_bound_and_independent_capabilities(self):
        summary = fixture()
        for record in summary['snapshots']['after'].values():
            for key in ('monotonic', 'started_monotonic', 'reply_event_monotonic'):
                if key in record: record[key] += 2000.
        result = run(summary)
        self.assertTrue(all(row['reason'] == 'query_pair_interval_out_of_bound' and not row['counter_deltas']
                            for row in result['queries'].values()))
        summary = fixture()
        for command in ('BLESNAP?', 'SNAPSRC?', 'SNAPNET?'):
            summary['optional_service_diagnostics'][command]['advertised'] = False
            for phase in ('before', 'after'): del summary['snapshots'][phase][command]
        self.assertEqual(run(summary)['queries']['BLEFLOW?']['status'], 'OBSERVED_LOCAL_COUNTERS')

    def test_missing_mismatched_or_timed_out_original_evidence_is_unknown(self):
        summary = fixture(); raw, replies = evidence(summary)
        record = summary['snapshots']['after']['BLESNAP?']
        replies.remove(('after', 'BLESNAP?', record['reply_line']))
        self.assertIn('reply_not_in_original', analyzer.analyze_document(summary, raw, replies)['queries']['BLESNAP?']['reason'])
        for key, value in [('started_monotonic', 20.5), ('monotonic', 22.5), ('reply_event_monotonic', float('nan'))]:
            summary = fixture(); summary['snapshots']['after']['BLESNAP?'][key] = value
            self.assertEqual(run(summary)['queries']['BLESNAP?']['status'], 'UNKNOWN')
        summary = fixture(); raw, replies = evidence(summary)
        summary['snapshots']['after']['SNAPNET?']['fields']['queued'] = '123'
        self.assertIn('summary_fields_differ', analyzer.analyze_document(summary, raw, replies)['queries']['SNAPNET?']['reason'])
        summary = fixture(); summary['snapshots']['after']['SNAPNET?']['fields']['readout'] = 'atomic'
        self.assertIn('readout', run(summary)['queries']['SNAPNET?']['reason'])
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            analyzer.parse_line('BLEFLOW,v=1,v=1', 'BLEFLOW')

    def test_never_complete_snapshot_and_old_caps_preserve_unknown(self):
        summary = fixture()
        for phase in ('before', 'after'):
            summary['snapshots'][phase]['BLESNAP?']['fields'] = output_fields('BLESNAP?', gen='7', stream='9')
        result = run(summary)['queries']['BLESNAP?']
        self.assertEqual(result['status'], 'OBSERVED_LOCAL_COUNTERS')
        self.assertEqual(result['after']['observed_state']['known'], '0')
        self.assertEqual(result['after']['observed_state']['lastCompleteAgeUs'], 'unknown')
        self.assertEqual(result['after']['observed_state']['maxCompleteGapUs'], 'unknown')
        summary = fixture(); summary['optional_service_diagnostics'] = {}
        for phase in ('before', 'after'):
            for command in analyzer.COMMANDS: del summary['snapshots'][phase][command]
        self.assertTrue(all(row['reason'] == 'capability_not_advertised' for row in run(summary)['queries'].values()))

    def test_raw_hash_finished_tail_and_exclusive_output(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = write_fixture(root)
            report = analyzer.analyze_file(path)
            self.assertEqual(report['queries']['BLESNAP?']['status'], 'OBSERVED_LOCAL_COUNTERS')
            self.assertEqual(report['inputs']['summary']['sha256'], hashlib.sha256(path.read_bytes()).hexdigest())
            output = root / 'analysis.json'
            with contextlib.redirect_stdout(io.StringIO()): self.assertEqual(analyzer.main(['--summary', str(path), '--output', str(output)]), 0)
            saved = output.read_bytes()
            with self.assertRaises(FileExistsError): analyzer.main(['--summary', str(path), '--output', str(output)])
            self.assertEqual(output.read_bytes(), saved)
            raw = root / 'data.bin'; raw.write_bytes(raw.read_bytes() + b'X')
            with self.assertRaisesRegex(ValueError, 'raw_sha256_mismatch'): analyzer.analyze_file(path)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = write_fixture(root)
            events = root / 'events.jsonl'; events.write_bytes(events.read_bytes().rstrip(b'\n'))
            with self.assertRaisesRegex(ValueError, 'incomplete_events_tail'): analyzer.analyze_file(path)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = write_fixture(root)
            events = root / 'events.jsonl'; events.write_text('\n'.join(events.read_text().splitlines()[:-1]) + '\n')
            with self.assertRaisesRegex(ValueError, 'matching_finished_event_required'): analyzer.analyze_file(path)
            summary = json.loads(path.read_text()); summary['status'] = 'RUNNING'; path.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError, 'closed_formal_summary_required'): analyzer.analyze_file(path)

    def test_compressed_bulk_raw_and_incomplete_diagnostic_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = write_fixture(root, fixture('usb_bulk'), compressed=True)
            result = analyzer.analyze_file(path)
            self.assertEqual(result['queries']['SNAPNET?']['status'], 'OBSERVED_LOCAL_COUNTERS')
            self.assertTrue(result['inputs']['ctrl']['path'].endswith('.gz'))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); path = write_fixture(root)
            summary = json.loads(path.read_text())
            wanted = summary['snapshots']['after']['SNAPNET?']['reply_line'].encode()
            raw = root / 'data.bin'; data = raw.read_bytes().replace(wanted + b'\n', b'') + wanted
            raw.write_bytes(data); summary['raw_sha256'] = hashlib.sha256(data).hexdigest(); path.write_text(json.dumps(summary))
            self.assertEqual(analyzer.analyze_file(path)['queries']['SNAPNET?']['status'], 'UNKNOWN')

    def test_closed_actual_collector_output_roundtrip(self):
        # Actual collector, retry, CONFIG bracket, raw and event writers; the
        # endpoint is synthetic USB, not a HIL measurement or hand-written CRC.
        class EpochFirmware(StageFirmware):
            def send(self, value):
                for command in ('BLEFLOW?', 'BLESNAP?'):
                    self.output_values.setdefault(command, {}).update(gen=str(self.mgen), stream='9')
                super().send(value)
        from run_firmware_benchmark import BenchmarkOptions
        from test_usb_bulk_capture import SERIAL
        device = EpochFirmware(optional_caps={'ble_flow_v1', 'ble_snapshot_v1'})
        port, _, _ = port_for(device)
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            root = Path(directory)
            options = BenchmarkOptions(None, root, 'synthetic-roundtrip', transport='usb_bulk', usb_serial=SERIAL,
                                       capture_seconds=.03, warmup_seconds=.01, prepare_seconds=2,
                                       cleanup_seconds=1, command_timeout=.08)
            index = stage.run_benchmark(options, port_factory=lambda _: port)
            self.assertEqual(index['status'], 'COMPLETE')
            report = analyzer.analyze_file(root / 'synthetic-roundtrip.summary.json')
            self.assertTrue(all(row['status'] == 'OBSERVED_LOCAL_COUNTERS' for row in report['queries'].values()), report)

    def test_actual_SHORT_subclass_finally_tail_and_raw_coverage(self):
        with tempfile.TemporaryDirectory() as directory:
            path = short_fixture(Path(directory))
            before = path.read_bytes()
            result = analyzer.analyze_file(path)
            closure = result['closure']
            self.assertEqual(closure['kind'], 'legacy_SHORT_restore_tail_verified')
            self.assertFalse(closure['explicit_final_finished_event'])
            self.assertEqual(closure['restore'], 'QUERY_VERIFIED')
            self.assertEqual(closure['last_event']['command'], 'TX?')
            self.assertEqual([e['command'] for e in closure['restore_replies']], ['TX=REL', 'TX?'])
            self.assertGreater(closure['restore_replies'][1]['raw_read_begin'], closure['restore_replies'][0]['raw_read_begin'])
            self.assertGreater(closure['tail_RX_bytes'], 0)
            self.assertEqual(closure['CAP8_query_before_TX_restore']['CONFIG?']['rows'], '8')
            self.assertTrue(all(row['status'] == 'OBSERVED_LOCAL_COUNTERS' for row in result['queries'].values()))
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(result['PC_snapshot_receipt'], 'unknown')
            self.assertEqual(result['overall_acceptance'], 'NOT_EVALUATED')

    def test_actual_SHORT_late_fault_or_17th_reset_keeps_ERROR(self):
        for late_reset in (False, True):
            with self.subTest(reset=late_reset), tempfile.TemporaryDirectory() as directory:
                path = short_fixture(Path(directory), late_fault=not late_reset, late_reset=late_reset)
                result = analyzer.analyze_file(path)
                self.assertEqual(result['recording_status'], 'ERROR')
                self.assertEqual(result['closure']['new_reset_events' if late_reset else 'new_fault_events'], 1)
                self.assertEqual(result['closure']['summary_status'], 'ERROR')
                summary = json.loads(path.read_text())
                self.assertEqual(summary['safe_cap_recovery'], 'INVALIDATED_BY_LATER_FAULT_OR_RESET')
                if late_reset:
                    self.assertEqual(summary['reset_event_count'], 17)
                    self.assertEqual(len(summary['reset_observations']), 16)
                    self.assertTrue(all(not row['counter_deltas'] for row in result['queries'].values()))
                summary['status'] = 'CAPTURED'; summary['error'] = None
                path.write_text(json.dumps(summary))
                with self.assertRaisesRegex(ValueError, 'summary_status_or_first_error'):
                    analyzer.analyze_file(path)

    def test_actual_SHORT_failed_restore_remains_failed_or_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            path = short_fixture(Path(directory), lost_restore=True)
            result = analyzer.analyze_file(path)
            self.assertEqual(result['recording_status'], 'ERROR')
            self.assertEqual(result['closure']['restore'], 'FAILED_OR_UNKNOWN')
            self.assertEqual(result['closure']['last_event']['type'], 'tx_restore_error')
            self.assertEqual(result['closure']['restore_replies'], [])
            self.assertEqual(result['overall_acceptance'], 'NOT_EVALUATED')

    def test_actual_SHORT_v2_uses_standard_finished_for_success_and_late_failure(self):
        for failure in ('none', 'fault', 'reset', 'lost_ack'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as directory:
                path = short_fixture(Path(directory), collector_module=current_short,
                    late_fault=failure=='fault', late_reset=failure=='reset', lost_restore=failure=='lost_ack')
                summary = json.loads(path.read_text())
                events = [json.loads(line) for line in Path(summary['events_file']).read_text().splitlines()]
                self.assertEqual(summary['runner_variant'], 'formal_cdc_ble_short_snapshot_diagnostic_v2')
                self.assertEqual(sum(event['type']=='base_finished' for event in events), 1)
                self.assertEqual(sum(event['type']=='finished' for event in events), 1)
                self.assertEqual(events[-1]['type'], 'finished')
                result = analyzer.analyze_file(path)
                self.assertEqual(result['closure']['kind'], 'matching_finished_event')
                self.assertTrue(result['closure']['explicit_final_finished_event'])
                self.assertEqual(result['recording_status'], 'CAPTURED' if failure=='none' else 'ERROR')
                for key in ('status', 'error', 'rx_bytes', 'tx_restore', 'firmware_fault_events', 'reset_event_count'):
                    self.assertEqual(events[-1][key], summary[key])
                self.assertEqual(result['overall_acceptance'], 'NOT_EVALUATED')

    def test_SHORT_truncated_wrong_status_or_unrelated_tail_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = short_fixture(Path(directory))
            summary = json.loads(path.read_text())
            ep = Path(summary['events_file']); original_events = ep.read_bytes()
            original_summary = path.read_bytes()
            cases = ('missing_reply', 'missing_rx', 'wrong_reply', 'unexpected_command', 'replayed_setter',
                     'count_mismatch', 'false_fault_count', 'unsupported_source', 'wrong_status', 'missing_tail')
            for case in cases:
                with self.subTest(case=case):
                    s = json.loads(original_summary); events = [json.loads(line) for line in original_events.splitlines()]
                    finish = next(i for i, event in enumerate(events) if event['type'] == 'finished')
                    tail = events[finish+1:]
                    if case == 'missing_reply': events.pop()
                    elif case == 'missing_rx':
                        events.pop(max(i for i, e in enumerate(events) if e['type'] == 'rx'))
                    elif case == 'wrong_reply': events[-1]['line'] = 'ACK,cmd=TX,v=full'
                    elif case == 'unexpected_command':
                        tail[0]['command'] = 'RESTART'; tail[0]['wire_ascii'] = '\nRESTART\n'
                    elif case == 'replayed_setter': events.insert(finish+2, dict(tail[0]))
                    elif case == 'count_mismatch': s['rx_bytes'] -= 1
                    elif case == 'false_fault_count': s['firmware_fault_events'] = 1
                    elif case == 'unsupported_source': s['source_sha256'] = '0'*64
                    elif case == 'wrong_status': s['status'] = 'ERROR'; s['error'] = 'invented'
                    elif case == 'missing_tail': events = events[:finish+1]
                    path.write_text(json.dumps(s)); ep.write_text(''.join(json.dumps(e)+'\n' for e in events))
                    with self.assertRaises(ValueError): analyzer.analyze_file(path)
            path.write_bytes(original_summary); ep.write_bytes(original_events)
            # The full raw hash is honestly updated in this synthetic tamper:
            # a matching older ACK still cannot satisfy the final query range.
            rp = Path(summary['raw_file']); raw = rp.read_bytes()
            position = raw.rfind(b'ACK,cmd=TX,v=rel\n'); self.assertGreater(position, 0)
            changed = raw[:position]+raw[position:].replace(b'v=rel', b'v=bad', 1)
            rp.write_bytes(changed); summary['raw_sha256'] = hashlib.sha256(changed).hexdigest()
            path.write_text(json.dumps(summary))
            with self.assertRaisesRegex(ValueError, 'ACK_not_in_its_RX_range'): analyzer.analyze_file(path)

    def test_generic_record_cannot_gain_SHORT_tail_exception(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_fixture(Path(directory)); summary = json.loads(path.read_text())
            ep = Path(summary['events_file'])
            with ep.open('a') as target:
                target.write(json.dumps({'type':'reply','phase':'tx_restore','command':'TX?','line':'ACK,cmd=TX,v=rel'})+'\n')
            with self.assertRaisesRegex(ValueError, 'matching_finished_event_required'): analyzer.analyze_file(path)


if __name__ == '__main__': unittest.main()
