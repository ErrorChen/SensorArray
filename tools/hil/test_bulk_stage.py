#!/usr/bin/env python3
"""Stage wrapper and boot observer self-tests; synthetic USB only."""
from dataclasses import replace
import ast
import contextlib
import io
import json
from pathlib import Path
import re
import sys
import tempfile
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import capture_bulk_benchmark as stage
import observe_bulk_boot as boot
import observe_cdc_debug as debug
from capture_reference_fixture import CaptureError
from run_firmware_benchmark import BenchmarkOptions
from sensorarray_usb_bulk import UsbBulkError
from test_usb_bulk_capture import Device, FirmwareDevice, port_for, SERIAL
from test_firmware_benchmark import Firmware
from text_protocol import parse_fields


def cpu_owner_fields(index, **updates):
    fields = {'v': '1', 'session': 'AABBCCDDEE112233', 'role': str(index), 'enabled': '1',
              'available': '1', 'known': '1', 'conflict': '0', 'stale': '0', 'reason': 'ok',
              'taskNumber': str(30 + index), 'runUs': '100000', 't0Us': '10000000',
              't1Us': '10000001', 'observedAtUs': '10000001', 'ageUs': '5000',
              'sampleMinUs': '1000000', 'staleAfterUs': '3000000', 'scope': 'switch_out',
              'isr': 'may_include', 'sdk': 'not_isolated', 'blocked': 'unknown'}
    fields.update(updates)
    return fields


def output_fields(command, **updates):
    fields = {key: '0' for key in stage.FRAME_OUTPUT_REQUIRED[command].split()}
    fields.update(stage.FRAME_OUTPUT_LITERALS[command])
    if command == 'BLEFLOW?':
        fields.update(stage='inactive', ratePm='1000', localServiceMilliFps='unknown')
    if command == 'BLESNAP?':
        fields.update(lastSeq='unknown', lastRev='unknown', lastCompleteAgeUs='unknown', maxCompleteGapUs='unknown')
    fields.update(updates)
    return fields


def formatter_line(source, tag):
    """Join adjacent C string literals, including the split SNAP* formatters."""
    literal = r'"(?:\\.|[^"\\])*"'
    for match in re.finditer(literal + r'(?:\s*' + literal + r')*', source):
        tokens = re.findall(literal, match.group())
        if tokens and tokens[0].startswith('"' + tag + ','):
            line = ''.join(ast.literal_eval(token) for token in tokens)
            if line.endswith('\n'):
                return line.rstrip('\n')
    return None


class StageFirmware(FirmwareDevice):
    def __init__(self, *, truncate=None, change_revision=False, watchdog=False,
                 optional_caps=(), service_revision_change=False, drop_optional=None,
                 service_unknown_once=False, cpu_values=None, cpu_revision_change=False,
                 output_values=None, revision_change_on=None):
        super().__init__()
        self.truncate = truncate
        self.query_counts = {}
        self.change_revision = change_revision
        self.watchdog = watchdog
        self.optional_caps = frozenset(optional_caps)
        self.service_revision_change = service_revision_change
        self.drop_optional = drop_optional
        self.service_unknown_once = service_unknown_once
        self.cpu_values = cpu_values or {}
        self.cpu_revision_change = cpu_revision_change
        self.cpu_command = None
        self.output_values = output_values or {}
        self.revision_change_on = revision_change_on

    def send(self, value):
        tag = value.split(',', 1)[0]
        command = {'ARL': 'RAIL?', 'ABAT': 'BAT?'}.get(tag, tag + '?')
        if tag == 'CPUOWNER': command = self.cpu_command
        if command == self.drop_optional:
            return
        schemas = {**stage.retry.REQUIRED, **stage.EXTRA_REQUIRED, **stage.OBSERVATION_REQUIRED, 'BOOT?': boot.BOOT_REQUIRED,
                   'BUILD?': 'idf target project proto version elfSha256'}
        if command in schemas:
            fields = parse_fields(value.strip())
            defaults = {'project': 'SensorArray', 'wires': 'ascii', 'ctrlMax': '512', 'dataMax': '2304',
                        'adcCrc': 'crc8', 'layout': 'HOMOGENEOUS', 'pending': 'NONE', 'stackUnit': 'bytes',
                        'enumerated': 'unknown', 'primary': 'present', 'secondary': 'present', 'ads': 'present',
                        'physicalSafe': 'unknown', 'scope': 'all_candidates', 'done': '1', 'joined': '1', 'restored': '1',
                        'gen': str(self.mgen), 'rid': str(self.mrid), 'initError': '0x0', 'err': '0x0',
                        'phyReq': '0x0', 'connReq': '0x0', 'dleReq': '0x0', 'sdkLastErr': '0x0',
                        'backend': 'otg_bulk_v1', 'init': '1', 'mounted': '1', 'pcReceipt': 'unknown',
                        'callbackResult': 'unknown', 'registered': '1', 'idleKnown': '1', 'idleGen': '47',
                        'tickHz': '100', 'budgetTicks': '4', 'backstopTicks': '4', 'maxUnits': '32',
                        'quantumTicks': '2', 'bootId': '1', 'prevErr': '0x0'}
            if command == 'SCHED?':
                defaults.update({worker: '12/1/4' for worker in stage.WORKERS})
            if command == 'BLESVC?':
                defaults.update(v='1', scope='boot', wall='elapsed', cpu='unknown', blocked='unknown',
                                wallUs='18446744073709551615', completed='4294967295')
            if command == 'SCHEDWHY?':
                defaults.update(v='1', order='quantum/backstop/units', overlap='1', init='0x0')
                defaults.update({worker: '1/2/3' for worker in stage.WORKERS})
            if command in stage.FRAME_OUTPUT_REQUIRED:
                defaults.update(output_fields(command, **self.output_values.get(command, {})))
            if command == 'BUILD?': defaults.update(version='synthetic-test', elfSha256='ab' * 32)
            if command in stage.CPU_OWNER_REQUIRED:
                role = int(command[-1])
                fields.update(cpu_owner_fields(role, **self.cpu_values.get(role, {})))
            for field in schemas[command].split():
                fields.setdefault(field, defaults.get(field, '0'))
            if command == 'PROTO?':
                fields['caps'] = '/'.join(sorted(stage.CAPABILITIES | {'config_identity_v1'} | self.optional_caps))
            if command == 'USB?': fields['backend'] = 'otg_bulk_v1'
            if command == self.truncate and self.query_counts.get(command) == 1:
                fields.pop(next(reversed(fields)))
            value = tag + ',' + ','.join(f'{k}={v}' for k, v in fields.items()) + '\n'
        super().send(value)

    def write(self, endpoint, data, timeout):
        count = Device.write(self, endpoint, data, timeout)
        self.command_pending.extend(data[:count])
        while b'\n' in self.command_pending:
            command, _, tail = self.command_pending.partition(b'\n')
            self.command_pending[:] = tail
            if not command.strip(): continue
            command = command.decode().strip()
            self.query_counts[command] = self.query_counts.get(command, 0) + 1
            if command in {**stage.EXTRA_REQUIRED, **stage.OBSERVATION_REQUIRED} or command == 'BOOT?':
                self.commands.append(command)
                if self.change_revision and command == 'USBQ?': self.rev += 1
                if self.service_revision_change and command == 'SCHEDWHY?': self.rev += 1
                if command == self.revision_change_on: self.rev += 1
                if self.service_unknown_once and command == 'BLESVC?' and self.query_counts[command] == 1:
                    self.send('ERR,cmd=UNKNOWN,reason=unsupported\n')
                if command in stage.CPU_OWNER_REQUIRED:
                    self.cpu_command = command
                    if self.cpu_revision_change and command == 'CPUOWNER?6': self.rev += 1
                    self.send('CPUOWNER,synthetic=1\n')
                else:
                    self.send(command[:-1] + ',synthetic=1\n')
                if self.watchdog and command == 'BLETX?':
                    self.send('E (123) task_wdt: Task watchdog got triggered. synthetic=1\n')
            else:
                Firmware.reply(self, command)
        return count


class StageTests(unittest.TestCase):
    def options(self, directory):
        return BenchmarkOptions(None, Path(directory), 'bulkstage', transport='usb_bulk', usb_serial=SERIAL,
                                capture_seconds=0.3, warmup_seconds=0.01, prepare_seconds=2,
                                cleanup_seconds=1, command_timeout=0.08, progress_seconds=1)

    def test_required_schema_matches_actual_production_formatters(self):
        sources = '\n'.join((ROOT / p).read_text() for p in
                            ['main/control/sensorarrayUsbControl.c', 'main/control/sensorarrayLinkDiagnostics.c',
                             'main/control/sensorarraySnapshotDiagnostics.c',
                             'components/sensorarrayWorkBudget/sensorarrayCpuOwner.c', 'main/main.c'])
        for command, required in {**stage.EXTRA_REQUIRED, **stage.OPTIONAL_REQUIRED}.items():
            tag = command[:-1]
            line = formatter_line(sources, tag)
            self.assertIsNotNone(line, command)
            # BLESVC v1 gained additive CONF counters after the older images;
            # preserve its legacy required subset and all received fields.
            if command == 'BLESVC?': self.assertLessEqual(set(required.split()), set(parse_fields(line)), command)
            elif command == 'USBQ?':
                self.assertEqual(set(required.split()) | {'dQueueCap', 'dLifetimeUs'}, set(parse_fields(line)), command)
            else: self.assertEqual(set(required.split()), set(parse_fields(line)), command)
        self.assertEqual(set(stage.CPU_OWNER_FIELDS.split()), set(parse_fields(formatter_line(sources, 'CPUOWNER'))))
        self.assertEqual(set(boot.BOOT_REQUIRED.split()), set(parse_fields(formatter_line(sources, 'BOOT'))))

    def test_before_after_periodic_CTRL_records_and_no_stream_setter(self):
        device = StageFirmware(truncate='USBQ?'); port, _, _ = port_for(device)
        old_interval, old_budget = stage.BulkBenchmarkCollector.diagnostic_interval_seconds, stage.BulkBenchmarkCollector.periodic_budget_seconds
        stage.BulkBenchmarkCollector.diagnostic_interval_seconds = 0.04
        stage.BulkBenchmarkCollector.periodic_budget_seconds = 0.1
        try:
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(self.options(directory), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
                self.assertEqual(report['identity_mismatches'], 0)
                self.assertTrue(report['capture_duration_satisfied'])
                self.assertTrue(any(w['name'] == 'periodic' for w in report['diagnostic_windows']))
                for name in ('before', 'after'):
                    self.assertTrue(set(stage.EXTRA_REQUIRED) <= set(report['snapshots'][name]))
                raw = Path(report['raw_file']).read_bytes()
                ctrl = Path(report['ctrl_raw_file']).read_bytes()
                for command in stage.EXTRA_REQUIRED:
                    self.assertIn(command[:-1].encode() + b',', ctrl)
                    self.assertNotIn(command[:-1].encode() + b',', raw)
                self.assertTrue(any(a['outcome'] == 'failed' and 'reply_missing_fields' in a.get('error', '')
                                    for a in report['query_attempts']))
                self.assertFalse(any(c.startswith(('USBBOOT', 'USBSTREAM', 'RESTART')) for c in device.commands))
                self.assertFalse(any(c in stage.OPTIONAL_REQUIRED for c in device.commands))
                self.assertFalse(any(c in stage.CPU_OWNER_REQUIRED for c in device.commands))
                self.assertTrue(all(not x['advertised'] for x in report['optional_service_diagnostics'].values()))
        finally:
            stage.BulkBenchmarkCollector.diagnostic_interval_seconds, stage.BulkBenchmarkCollector.periodic_budget_seconds = old_interval, old_budget

    def test_optional_observations_exact_caps_before_after_raw_and_bounded_retry(self):
        device = StageFirmware(optional_caps=stage.OPTIONAL_CAPABILITIES.values(),
                               truncate='BLESVC?', service_unknown_once=True)
        port, _, _ = port_for(device)
        old_interval, old_budget = stage.BulkBenchmarkCollector.diagnostic_interval_seconds, stage.BulkBenchmarkCollector.periodic_budget_seconds
        stage.BulkBenchmarkCollector.diagnostic_interval_seconds = .04
        stage.BulkBenchmarkCollector.periodic_budget_seconds = .1
        try:
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(self.options(directory), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
                self.assertTrue(report['capture_duration_satisfied'])
                self.assertGreaterEqual(report['capture_seconds'], .3)
                self.assertEqual(device.query_counts['BLESVC?'], 3)  # one truncated reply + retry + after
                self.assertEqual(device.query_counts['SCHEDWHY?'], 2)
                self.assertTrue(all(device.query_counts[c] == 2 for c in stage.FRAME_OUTPUT_REQUIRED))
                self.assertTrue(all(x['advertised'] for x in report['optional_service_diagnostics'].values()))
                ctrl = Path(report['ctrl_raw_file']).read_bytes()
                for name in ('before', 'after'):
                    snap = report['snapshots'][name]
                    for command in stage.OPTIONAL_REQUIRED:
                        q = snap[command]
                        self.assertIn((q['reply_line'] + '\n').encode(), ctrl)
                        self.assertEqual(parse_fields(q['reply_line']), q['fields'])
                        self.assertEqual(q['reply_channel'], 'ctrl')
                        self.assertLessEqual(q['started_monotonic'], q['reply_event_monotonic'])
                        self.assertLessEqual(q['reply_event_monotonic'], q['monotonic'])
                        self.assertLessEqual(q['monotonic'], snap['CONFIG?_closing']['started_monotonic'])
                        if name == 'before': self.assertLessEqual(q['monotonic'], report['capture_monotonic_start'])
                        else: self.assertGreaterEqual(q['started_monotonic'], report['capture_monotonic_end'])
                periodic = [w for w in report['diagnostic_windows'] if w['name'] == 'periodic']
                self.assertTrue(periodic)
                self.assertTrue(all(not set(stage.OPTIONAL_REQUIRED) & set(w['queries']) for w in periodic))
                attempts = [a for a in report['query_attempts'] if a['command'] == 'BLESVC?' and a['phase'] == 'before']
                self.assertEqual([a['outcome'] for a in attempts], ['failed', 'complete_reply'])
                self.assertEqual(attempts[0]['phase_deadline'], attempts[1]['phase_deadline'])
                events = [json.loads(line) for line in Path(report['events_file']).read_text().splitlines()]
                self.assertTrue(any(e['type'] == 'uncorrelated_error' and e['command'] == 'BLESVC?' for e in events))
                self.assertEqual(report['query_retry_limit'], 3)
                self.assertFalse(report['setters_retried'])
        finally:
            stage.BulkBenchmarkCollector.diagnostic_interval_seconds, stage.BulkBenchmarkCollector.periodic_budget_seconds = old_interval, old_budget

    def test_independent_optional_caps_and_no_prefix_match(self):
        for caps, expected in (({'ble_service_obs_v1'}, {'BLESVC?'}),
                               ({'work_reason_v1'}, {'SCHEDWHY?'}),
                               ({'ble_flow_v1'}, {'BLEFLOW?'}),
                               ({'ble_snapshot_v1'}, {'BLESNAP?', 'SNAPSRC?', 'SNAPNET?'}),
                               ({'ble_service_obs_v10', 'work_reason_v10', 'cpu_owner_v10', 'ble_flow_v10', 'ble_snapshot_v10'}, set())):
            device = StageFirmware(optional_caps=caps); port, _, _ = port_for(device)
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(replace(self.options(directory), capture_seconds=.03), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
                self.assertEqual(set(device.commands) & set(stage.OPTIONAL_REQUIRED), expected)
                self.assertFalse(any(c in stage.CPU_OWNER_REQUIRED for c in device.commands))
                for name in ('before', 'after'):
                    self.assertEqual(set(report['snapshots'][name]) & set(stage.OPTIONAL_REQUIRED), expected)

    def test_optional_timeout_revision_change_and_semantics_are_not_success(self):
        for params, needle in (({'drop_optional': 'BLESVC?'}, 'readonly_query_failed:BLESVC?:attempts=3'),
                               ({'service_revision_change': True}, 'configuration_changed')):
            device = StageFirmware(optional_caps=stage.OPTIONAL_CAPABILITIES.values(), **params); port, _, _ = port_for(device)
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(self.options(directory), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'ERROR')
                self.assertIn(needle, report['error'])
                self.assertEqual(report['capture_seconds'], 0)
                self.assertFalse(report['setters_retried'])
                self.assertLess(report['elapsed_seconds'], report['duration_total_budget_seconds'])
        svc = {k: '0' for k in stage.OPTIONAL_REQUIRED['BLESVC?'].split()}
        svc.update(stage.OPTIONAL_LITERALS['BLESVC?'])
        for key, value in (('cpu', 'measured'), ('wallUs', str(1 << 64)), ('completed', str(1 << 32)), ('scope', 'connection')):
            with self.assertRaises(CaptureError): stage.validate_reply('BLESVC?', {**svc, key: value})
        why = {k: '0' for k in stage.OPTIONAL_REQUIRED['SCHEDWHY?'].split()}
        why.update(stage.OPTIONAL_LITERALS['SCHEDWHY?']); why['init'] = '0x0'
        why.update({w: '1/2/3' for w in stage.WORKERS})
        for key, value in (('overlap', '0'), ('order', 'units/backstop/quantum'), ('bt', '1/2'), ('bt', '1/2/4294967296')):
            with self.assertRaises(CaptureError): stage.validate_reply('SCHEDWHY?', {**why, key: value})

    def test_output_observations_zero_max_unknown_and_additive_fields(self):
        u32, u64 = str((1 << 32) - 1), str((1 << 64) - 1)
        for command in stage.FRAME_OUTPUT_REQUIRED:
            with self.subTest(command=command):
                zero = output_fields(command, future='preserved')
                self.assertEqual(stage.validate_reply(command, zero), zero)
                maximum = output_fields(command)
                for key, value in list(maximum.items()):
                    if key not in stage.FRAME_OUTPUT_LITERALS[command] and value == '0': maximum[key] = u32
                if command == 'BLEFLOW?':
                    maximum.update(active='1', evidence='1', stage='unknown', ratePm='1000', rejectPm='1000',
                                   pressureWindows='3', localServiceMilliFps=u32,
                                   intentionalSkip=u64, probeSelected=u64, cleanUs=u64)
                elif command == 'BLESNAP?':
                    maximum.update(known='1', gapKnown='1', firstPending='1', periodicPending='1',
                                   lastSeq=u32, lastRev=u32, lastCompleteAgeUs=u64, maxCompleteGapUs=u64)
                self.assertEqual(stage.validate_reply(command, maximum), maximum)
                for key in stage.FRAME_OUTPUT_REQUIRED[command].split():
                    missing = dict(zero); del missing[key]
                    with self.subTest(missing=key), self.assertRaises(CaptureError): stage.validate_reply(command, missing)
                for key in stage.FRAME_OUTPUT_LITERALS[command]:
                    with self.subTest(literal=key), self.assertRaises(CaptureError):
                        stage.validate_reply(command, {**zero, key: 'not_' + zero[key]})
        # No-sample evidence and historical completion age stay unknown, not 0.
        for values in ({'active': '1', 'stage': 'full'},
                       {'active': '1', 'stage': 'pressure', 'evidence': '1', 'rejectPm': '200'},
                       {'active': '1', 'stage': 'limited', 'ratePm': '0'},
                       {'active': '0', 'stage': 'inactive', 'ratePm': '50'}):
            fields = output_fields('BLEFLOW?', **values)
            self.assertEqual(stage.validate_reply('BLEFLOW?', fields)['localServiceMilliFps'], 'unknown')
        fields = output_fields('BLESNAP?', known='1', lastSeq='0', lastRev='0', gapKnown='1', maxCompleteGapUs='0')
        self.assertEqual(stage.validate_reply('BLESNAP?', fields)['lastCompleteAgeUs'], 'unknown')
        # Wrapped cumulative totals need not be ordered; this is not an atomic
        # cross-layer conservation equation or evidence of peer reception.
        fields = output_fields('SNAPNET?', queued='0', bleAccepted=u32, admissionMiss='1')
        self.assertEqual(stage.validate_reply('SNAPNET?', fields), fields)

    def test_output_observations_reject_bad_ranges_and_unsupported_claims(self):
        invalid = {
            'BLEFLOW?': [dict(active='2'), dict(active='1'), dict(stage='full'), dict(stage='invented'),
                         dict(active='1', stage='full', ratePm='950'),
                         dict(active='1', stage='limited', ratePm='1000'), dict(evidence='1'),
                         dict(active='1', stage='full', localServiceMilliFps='0'),
                         dict(ratePm='1001'), dict(rejectPm='1001'), dict(pressureWindows='4'),
                         dict(pressureWindows='256'), dict(intentionalSkip=str(1 << 64)),
                         dict(probeSelected=str(1 << 64)), dict(cleanUs=str(1 << 64)),
                         dict(attempts=str(1 << 32)), dict(capacityReject='unknown'),
                         dict(peerLoss='0'), dict(rssi='-60'), dict(gen=' 0'), dict(stream='-1'),
                         dict(active='1', stage='full', evidence='1', localServiceMilliFps=str(1 << 32))],
            'BLESNAP?': [dict(known='2'), dict(known='1'), dict(lastSeq='0'), dict(lastRev='0'),
                         dict(lastCompleteAgeUs='0'), dict(gapKnown='1'), dict(maxCompleteGapUs='0'),
                         dict(known='1', lastSeq='0', lastRev='0', gapKnown='1'),
                         dict(known='1', lastSeq=str(1 << 32), lastRev='0'),
                         dict(known='1', lastSeq='0', lastRev='0', lastCompleteAgeUs=str(1 << 64)),
                         dict(known='1', lastSeq='0', lastRev='0', gapKnown='1', maxCompleteGapUs=str(1 << 64)),
                         dict(firstPending='2'), dict(periodicPending='unknown'), dict(localComplete=str(1 << 32)),
                         dict(pcReceipt='confirmed')],
            'SNAPSRC?': [dict(queued=str(1 << 32)), dict(candidateMiss='unknown'), dict(firstMiss='-1'),
                         dict(stage='ble_admission'), dict(pcReceipt='confirmed')],
            'SNAPNET?': [dict(policySkip=str(1 << 32)), dict(bleAccepted='unknown'), dict(readout='atomic'),
                         dict(stage='local_complete'), dict(obsolete='0.0'), dict(pcReceipt='confirmed')],
        }
        for command, cases in invalid.items():
            for values in cases:
                with self.subTest(command=command, values=values), self.assertRaises(CaptureError):
                    stage.validate_reply(command, output_fields(command, **values))

    def test_output_observation_config_change_and_invalid_reply_fail_before_capture(self):
        cases = [({'revision_change_on': 'SNAPNET?'}, 'configuration_changed'),
                 ({'output_values': {'SNAPNET?': {'readout': 'atomic'}}}, 'reply_invalid_ble_output_semantics:SNAPNET?:readout'),
                 ({'output_values': {'BLESNAP?': {'lastCompleteAgeUs': '0'}}}, 'reply_invalid_ble_output_semantics:BLESNAP?:unknown_completion')]
        for params, needle in cases:
            device = StageFirmware(optional_caps={'ble_flow_v1', 'ble_snapshot_v1'}, **params); port, _, _ = port_for(device)
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(self.options(directory), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'ERROR')
                self.assertIn(needle, report['error'])
                self.assertEqual(report['capture_seconds'], 0)
                self.assertFalse(report['setters_retried'])
                self.assertLess(report['elapsed_seconds'], report['duration_total_budget_seconds'])

    def test_output_query_timeout_keeps_original_deadline_and_does_not_retry_setters(self):
        for command in stage.FRAME_OUTPUT_REQUIRED:
            device = StageFirmware(optional_caps={'ble_flow_v1', 'ble_snapshot_v1'}, drop_optional=command)
            port, _, _ = port_for(device)
            try:
                with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()), stage.bulk_collector_selected():
                    options = self.options(directory)
                    collector = stage.BulkBenchmarkCollector(options, stage.base.configurations(options)[0], port,
                                                             io.BytesIO(), io.StringIO(), ctrl_raw=io.BytesIO())
                    collector.bulk_capabilities_verified = True
                    collector.reported_capabilities = {'ble_flow_v1', 'ble_snapshot_v1'}
                    deadline = time.monotonic() + .025
                    with self.assertRaisesRegex(CaptureError, re.escape('readonly_query_failed:' + command + ':attempts=1')):
                        collector.query(command, command[:-1], deadline)
                    self.assertEqual(device.query_counts[command], 1)
                    self.assertEqual(device.commands, [command])
                    self.assertEqual(collector.query_attempts[-1]['phase_deadline'], deadline)
                    self.assertLess(time.monotonic(), deadline + .05)
            finally:
                port.close()

    def test_cpu_owner_cap_brackets_raw_indexed_tags_and_retry(self):
        values = {0: {'runUs': '0'}, 5: {'stale': '1', 'ageUs': '3000000'},
                  6: {'available': '0', 'known': '0', 'stale': '1', 'reason': 'not_observed',
                      **{k: 'unknown' for k in ('taskNumber', 'runUs', 't0Us', 't1Us', 'observedAtUs', 'ageUs')}}}
        device = StageFirmware(optional_caps={stage.CPU_OWNER_CAPABILITY}, cpu_values=values, truncate='CPUOWNER?3')
        port, _, _ = port_for(device)
        old_interval, old_budget = stage.BulkBenchmarkCollector.diagnostic_interval_seconds, stage.BulkBenchmarkCollector.periodic_budget_seconds
        stage.BulkBenchmarkCollector.diagnostic_interval_seconds = .04
        stage.BulkBenchmarkCollector.periodic_budget_seconds = .1
        try:
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(self.options(directory), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
                self.assertGreaterEqual(report['capture_seconds'], .3)
                raw = Path(report['ctrl_raw_file']).read_bytes()
                for side in ('before', 'after'):
                    snap = report['snapshots'][side]
                    for command, tag in stage.CPU_OWNER_QUERIES:
                        q = snap[command]
                        self.assertEqual(tag, 'CPUOWNER')
                        self.assertEqual(q['fields']['role'], command[-1])
                        self.assertEqual(q['fields']['session'], snap['CONFIG?']['fields']['session'])
                        self.assertIn((q['reply_line'] + '\n').encode(), raw)
                        self.assertEqual(q['reply_channel'], 'ctrl')
                        self.assertLessEqual(q['started_monotonic'], q['reply_received_monotonic'])
                        self.assertLessEqual(q['reply_received_monotonic'], q['reply_event_monotonic'])
                        self.assertLessEqual(q['reply_event_monotonic'], q['monotonic'])
                        self.assertLessEqual(q['monotonic'], snap['CONFIG?_closing']['started_monotonic'])
                        self.assertGreater(q['reply_rx_chunk_length'], 0)
                    self.assertTrue(snap['CPUOWNER?0']['cpu_owner_evidence']['usable_for_pairing'])
                    self.assertTrue(snap['CPUOWNER?5']['cpu_owner_evidence']['historical'])
                    self.assertFalse(snap['CPUOWNER?5']['cpu_owner_evidence']['usable_for_pairing'])
                    self.assertEqual(snap['CPUOWNER?6']['fields']['runUs'], 'unknown')
                    self.assertEqual(snap['CPUOWNER?6']['cpu_owner_evidence']['current_cpu_utilization'], 'NOT_DERIVED')
                periodic = [w for w in report['diagnostic_windows'] if w['name'] == 'periodic']
                self.assertTrue(periodic)
                self.assertTrue(all(not set(stage.CPU_OWNER_REQUIRED) & set(w['queries']) for w in periodic))
                self.assertEqual(device.query_counts['CPUOWNER?3'], 3)
                self.assertTrue(all(device.query_counts[c] == 2 for c in stage.CPU_OWNER_REQUIRED if c != 'CPUOWNER?3'))
                attempts = [a for a in report['query_attempts'] if a['command'] == 'CPUOWNER?3' and a['phase'] == 'before']
                self.assertEqual([a['outcome'] for a in attempts], ['failed', 'complete_reply'])
                self.assertEqual(attempts[0]['phase_deadline'], attempts[1]['phase_deadline'])
                self.assertFalse(report['setters_retried'])
                self.assertTrue(report['optional_cpu_owner_diagnostics']['advertised'])
        finally:
            stage.BulkBenchmarkCollector.diagnostic_interval_seconds, stage.BulkBenchmarkCollector.periodic_budget_seconds = old_interval, old_budget

    def test_cpu_owner_unavailable_and_invalid_semantics(self):
        unknown = {k: 'unknown' for k in ('taskNumber', 'runUs', 't0Us', 't1Us', 'observedAtUs', 'ageUs')}
        valid = [cpu_owner_fields(0, runUs='0'), cpu_owner_fields(0, stale='1', ageUs='3000000'),
                 cpu_owner_fields(0, enabled='0', available='0', known='0', stale='1', reason='disabled', **unknown),
                 cpu_owner_fields(0, available='0', known='0', stale='1', reason='not_observed', **unknown),
                 cpu_owner_fields(0, available='0', known='0', stale='1', reason='clock_regressed', **unknown),
                 cpu_owner_fields(0, available='0', conflict='1', reason='writer_conflict', runUs='unknown'),
                 cpu_owner_fields(0, available='0', reason='counter_regressed', runUs='unknown'),
                 cpu_owner_fields(0, available='0', reason='clock_regressed', runUs='unknown', ageUs='unknown', stale='1'),
                 cpu_owner_fields(0, taskNumber=str((1 << 32)-1), runUs=str((1 << 64)-1),
                                  t0Us=str((1 << 64)-1), t1Us=str((1 << 64)-1), observedAtUs=str((1 << 64)-1))]
        for fields in valid: self.assertEqual(stage.validate_reply('CPUOWNER?0', fields), fields)
        invalid = [('role', '1'), ('session', 'FFFF'), ('enabled', '2'), ('available', '0'),
                   ('conflict', '1'), ('known', '0'), ('reason', 'made_up'), ('taskNumber', str(1 << 32)),
                   ('runUs', str(1 << 64)), ('t0Us', str(1 << 64)), ('ageUs', str(1 << 64)),
                   ('t0Us', '10000002'), ('observedAtUs', '10000000'), ('ageUs', 'unknown'),
                   ('stale', '1'), ('scope', 'CPU'), ('sdk', 'isolated'), ('sampleMinUs', '0'),
                   ('staleAfterUs', '0'), ('blocked', '0')]
        for key, value in invalid:
            with self.subTest(key=key, value=value), self.assertRaises(CaptureError):
                stage.validate_reply('CPUOWNER?0', cpu_owner_fields(0, **{key: value}))
        for key in stage.CPU_OWNER_FIELDS.split():
            fields = cpu_owner_fields(0); del fields[key]
            with self.subTest(missing=key), self.assertRaises(CaptureError): stage.validate_reply('CPUOWNER?0', fields)
        disabled = {role: cpu_owner_fields(role, enabled='0', available='0', known='0', stale='1', reason='disabled', **unknown)
                    for role in range(7)}
        device = StageFirmware(optional_caps={stage.CPU_OWNER_CAPABILITY}, cpu_values=disabled); port, _, _ = port_for(device)
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            index = stage.run_benchmark(replace(self.options(directory), capture_seconds=.03), port_factory=lambda _: port)
            report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
            self.assertEqual(index['status'], 'COMPLETE', report.get('error'))
            self.assertTrue(all(report['snapshots']['after'][c]['fields']['runUs'] == 'unknown' for c in stage.CPU_OWNER_REQUIRED))

    def test_cpu_owner_role_session_revision_and_deadline_fail_bounded(self):
        cases = [({'cpu_values': {0: {'role': '1'}}}, 'reply_invalid_cpu_owner:CPUOWNER?0:role'),
                 ({'cpu_values': {0: {'session': '0000000000000001'}}}, 'reply_cpu_owner_session_mismatch'),
                 ({'cpu_revision_change': True}, 'configuration_changed'),
                 ({'drop_optional': 'CPUOWNER?0'}, 'readonly_query_failed:CPUOWNER?0:attempts=3')]
        for params, needle in cases:
            device = StageFirmware(optional_caps={stage.CPU_OWNER_CAPABILITY}, **params); port, _, _ = port_for(device)
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(self.options(directory), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'ERROR')
                self.assertIn(needle, report['error'])
                self.assertEqual(report['capture_seconds'], 0)
                self.assertFalse(report['setters_retried'])
                self.assertLess(report['elapsed_seconds'], report['duration_total_budget_seconds'])
        device = StageFirmware(optional_caps={stage.CPU_OWNER_CAPABILITY}, drop_optional='CPUOWNER?0')
        port, _, _ = port_for(device)
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            options = self.options(directory)
            collector = stage.BulkBenchmarkCollector(options, stage.base.configurations(options)[0], port,
                                                     io.BytesIO(), io.StringIO(), ctrl_raw=io.BytesIO())
            collector.bulk_capabilities_verified = True
            collector.reported_capabilities = {stage.CPU_OWNER_CAPABILITY}
            deadline = time.monotonic() + .025
            with self.assertRaisesRegex(CaptureError, 'readonly_query_failed:CPUOWNER\\?0:attempts=1'):
                collector.query('CPUOWNER?0', 'CPUOWNER', deadline)
            self.assertEqual(device.query_counts['CPUOWNER?0'], 1)
            self.assertLess(time.monotonic(), deadline + .05)
            for bad in ('CPUOWNER?7', 'CPUOWNER?0?', 'CPUOWNER=0'):
                with self.assertRaises(CaptureError): collector.query(bad, 'CPUOWNER', time.monotonic()+.1)
                self.assertNotIn(bad, device.commands)
        port.close()

    def test_CLI_cannot_open_POSIX_even_with_explicit_transport_override(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stderr(io.StringIO()):
            with mock.patch.object(stage.base, 'open_capture_port', side_effect=AssertionError('must reject before open')):
                for option in (['--transport', 'posix'], ['--transport=posix'], ['--trans', 'posix']):
                    with self.assertRaises(SystemExit) as stopped:
                        stage.main([*option, '--port', '/dev/not_a_target', '--output-directory', directory, '--label', 'rejected'])
                    self.assertEqual(stopped.exception.code, 2)
            self.assertFalse(list(Path(directory).iterdir()))

    def test_revision_change_and_watchdog_remain_failures(self):
        for kwargs in ({'change_revision': True}, {'watchdog': True}):
            device = StageFirmware(**kwargs); port, _, _ = port_for(device)
            with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
                index = stage.run_benchmark(self.options(directory), port_factory=lambda _: port)
                report = json.loads((Path(directory) / 'bulkstage.summary.json').read_text())
                self.assertEqual(index['status'], 'ERROR')
                self.assertTrue('configuration_changed' in report['error'] if kwargs.get('change_revision') else
                                'firmware_fault' in report['error'], report['error'])
                self.assertEqual(report['capture_seconds'], 0)

    def test_enumeration_poll_is_bounded_half_second_and_never_reconfigures(self):
        now, waits, attempts, calls = [0.0], [], [], []
        def clock(): return now[0]
        def sleep(seconds): waits.append(seconds); now[0] += seconds
        token = object()
        def factory(serial, **kwargs):
            calls.append((serial, kwargs))
            if len(calls) < 3: raise UsbBulkError('usb_device_not_found')
            return token
        options = boot.Options(Path('/tmp'), 'fake', SERIAL)
        result = boot.wait_for_bulk(options, 2, attempts, factory=factory, clock=clock, sleep=sleep)
        self.assertIs(result, token)
        self.assertEqual(waits, [0.5, 0.5])
        self.assertEqual(calls[0][1], {'vid': 0x303A, 'pid': 0x4041})
        def missing(*args, **kwargs): raise UsbBulkError('usb_device_not_found')
        with self.assertRaisesRegex(UsbBulkError, 'enumeration_deadline'):
            boot.wait_for_bulk(options, 1.6, [], factory=missing, clock=clock, sleep=sleep)
        self.assertAlmostEqual(now[0], 1.6)
        late = mock.Mock()
        def too_late(*args, **kwargs): now[0] = 3; return late
        with self.assertRaisesRegex(UsbBulkError, 'enumeration_deadline'):
            boot.wait_for_bulk(options, 2, [], factory=too_late, clock=clock, sleep=sleep)
        late.close.assert_called_once_with()

    def test_boot_observer_only_queries_and_preserves_both_streams(self):
        device = StageFirmware(); port, _, _ = port_for(device)
        with tempfile.TemporaryDirectory() as directory:
            options = boot.Options(Path(directory), 'boot', SERIAL, duration=2, observe_seconds=0.01, command_timeout=0.1)
            result = boot.observe(options, port_factory=lambda *args, **kwargs: port)
            self.assertEqual(result['status'], 'QUERIES_COMPLETE', result)
            self.assertTrue(all(c.endswith('?') for c in device.commands))
            self.assertNotIn('BOARD?', device.commands)
            self.assertIn('BOARD?', result['unavailable_queries'])
            self.assertNotEqual(Path(result['raw_file']).read_bytes(), Path(result['ctrl_raw_file']).read_bytes())
            self.assertFalse(result['USBBOOT_requested'])
            with self.assertRaises(FileExistsError): boot.observe(options, port_factory=lambda *a, **k: None)

    def test_boot_library_ValueError_keeps_failure_summary_and_empty_raw(self):
        def fail(*args, **kwargs): raise ValueError('device has no langid: synthetic reproduction')
        with tempfile.TemporaryDirectory() as directory:
            result = boot.observe(boot.Options(Path(directory), 'failure', SERIAL, 1, 0, 0.1), port_factory=fail)
            self.assertEqual(result['status'], 'ERROR')
            self.assertEqual(result['error_class'], 'ValueError')
            self.assertEqual(json.loads((Path(directory) / 'failure.summary.json').read_text())['error'], result['error'])
            self.assertEqual(Path(result['raw_file']).read_bytes(), b'')

    def test_CDC_selection_requires_exact_serial_product_and_interface(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); tty = root / 'tty'; tty.mkdir()
            def candidate(name, serial=SERIAL, pid='4041', number='02', protocol='00'):
                device = root / ('usb-' + name); device.mkdir()
                for field, value in {'idVendor': '303a', 'idProduct': pid, 'serial': serial, 'speed': '12'}.items():
                    (device / field).write_text(value)
                interface = device / '1.2'; interface.mkdir()
                for field, value in {'bInterfaceNumber': number, 'bInterfaceClass': '02',
                                     'bInterfaceSubClass': '02', 'bInterfaceProtocol': protocol}.items():
                    (interface / field).write_text(value)
                entry = tty / name; entry.mkdir(); (entry / 'device').symlink_to(interface)
            candidate('ttyACM0', pid='1001')
            candidate('ttyACM1', serial='000000000001')
            candidate('ttyACM2', number='00')
            candidate('ttyACM3', protocol='01')
            with self.assertRaisesRegex(CaptureError, 'not_enumerated'):
                debug.find_debug_tty(SERIAL, sys_tty=tty)
            candidate('ttyACM4')
            found = debug.find_debug_tty(SERIAL, sys_tty=tty)
            self.assertEqual(found['path'], '/dev/ttyACM4')
            self.assertEqual(found['interface_protocol'], 0)
            candidate('ttyACM5')
            with self.assertRaisesRegex(CaptureError, 'ambiguous'):
                debug.find_debug_tty(SERIAL, sys_tty=tty)
        source = (ROOT / 'components/sensorarrayUsbOtg/sensorarrayUsbDescriptors.c').read_text()
        self.assertIn('TUD_CDC_DESCRIPTOR(2, 6, 0x83, 8, 0x04, 0x84, 64)', source)
        macro = (ROOT / 'managed_components/espressif__tinyusb/src/device/usbd.h').read_text()
        self.assertIn('TUSB_CLASS_CDC, CDC_COMM_SUBCLASS_ABSTRACT_CONTROL_MODEL, CDC_COMM_PROTOCOL_NONE', macro)

    def test_CDC_debug_is_separate_raw_never_command_or_formal_frame_evidence(self):
        chunks = [b'ADSHEALTH,crcErrors=2\nSTK', b'50,synthetic=1\n',
                  b'E (123) task_wdt: Task watchdog got triggered.\n', b'unfinished']
        class Reader:
            closed = False
            def read_some(self, size, deadline):
                if chunks:
                    return 'data', chunks.pop(0)
                stop.write_text('requested by synthetic test')
                return None
            def close(self): self.closed = True
            def write_some(self, *args): raise AssertionError('debug must never send commands')
        with tempfile.TemporaryDirectory() as directory:
            stop = Path(directory) / 'observer.stop'
            reader = Reader()
            metadata = {'path': '/dev/synthetic_CDC', 'usb_serial': SERIAL, 'interface_number': 2}
            result = debug.observe(debug.Options(Path(directory), 'debug', SERIAL, 1, 0.1, stop),
                                   metadata_finder=lambda _: metadata, port_factory=lambda _: reader)
            self.assertEqual(result['status'], 'STOPPED')
            self.assertEqual(result['commands_written'], 0)
            self.assertFalse(result['formal_scientific_frame_evidence'])
            self.assertEqual(result['tag_counts']['ADSHEALTH'], 1)
            self.assertEqual(result['tag_counts']['STK50'], 1)
            self.assertEqual(len(result['faults']), 1)
            self.assertEqual(result['trailing_partial_line_bytes'], len(b'unfinished'))
            self.assertEqual(Path(result['raw_file']).read_bytes(),
                             b'ADSHEALTH,crcErrors=2\nSTK50,synthetic=1\nE (123) task_wdt: Task watchdog got triggered.\nunfinished')
            self.assertTrue(reader.closed)


if __name__ == '__main__':
    unittest.main()
