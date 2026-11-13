#!/usr/bin/env python3
"""Offline auxiliary-stream regressions; synthetic clocks/diagnostics, real encoder frame."""
import gzip
import hashlib
import json
import importlib.util
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest

from analyze_firmware_benchmark import analyse_capture, analyse_directory
from sensorarray_bulk_diagnostics import SCHEMAS
from test_analyze_firmware_benchmark import RECORDED_STK50_COMPLETE, synthetic_perf
from text_protocol import parse_fields

ROOT = Path(__file__).resolve().parents[1]
SERIAL = '80B54ECDD9AC'


def diagnostic(tag, **overrides):
    fields = {key: '0' for key in SCHEMAS[tag].split()}
    defaults = {'backend': 'otg_bulk_v1', 'pcReceipt': 'unknown', 'callbackResult': 'unknown',
                'state': 'CAPACITANCE', 'mode': 'CAP', 'profile': 'CCCCCCCC',
                'session': 'AABBCCDDEE112233', 'bootId': '7', 'rev': '8', 'rows': '1',
                'mgen': '4', 'mrid': '3', 'rgen': '2', 'rrid': '1', 'pgen': '5', 'prid': '3',
                'mterm': 'applied', 'pterm': 'applied', 'rterm': 'applied', 'phy': '1/1',
                'sub': '111', 'bt': '10/1/2', 'tr': '10/1/2', 'log': '10/1/2', 'ser': '10/1/2',
                'bc': '10/1/2', 'sc': '10/1/2', 'wc': '10/1/2'}
    fields.update({key: value for key, value in defaults.items() if key in fields})
    fields.update({key: str(value) for key, value in overrides.items()})
    return (tag + ',' + ','.join(f'{key}={value}' for key, value in fields.items()) + '\n').encode()


class BulkDiagnosticsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.root = Path(self.tmp.name)
        self.wire = (ROOT / 'tests/fixtures/config_identity/cap_1.frame').read_bytes()
        self.config = parse_fields(diagnostic('CONFIG').decode())
        self.events = [{'type': 'capture_start', 't': 1, 'monotonic': 101},
                       {'type': 'rx', 't': 1.2, 'offset': 0, 'length': len(self.wire)},
                       {'type': 'capture_end', 't': 3, 'monotonic': 103}]
        self.summary = {'status': 'CAPTURED', 'transport': 'usb_bulk', 'mode_requested': 'CAP', 'rows_requested': 1,
                        'capture_requested_seconds': 2, 'capture_seconds': 2,
                        'capture_monotonic_start': 101, 'capture_monotonic_end': 103,
                        'transport_metadata': {'usb_serial': SERIAL},
                        'raw_sha256': hashlib.sha256(self.wire).hexdigest(),
                        'snapshots': {phase: {'CONFIG?': {'fields': self.config}} for phase in ('before', 'after')}}
        (self.root / 'case.rx.bin').write_bytes(self.wire)
        self.ctrl_chunks = [(0.8, diagnostic('CONFIG') + diagnostic('USBQ', dSubmit=10) +
                             diagnostic('BLELINK', gen=1, phyKnown=1, txPhy=2, rxPhy=2) + diagnostic('BLETX', gen=1, sdkAccepted=20) +
                             synthetic_perf(pub=10).encode() + b'\n'),
                            (3.2, diagnostic('CONFIG') + diagnostic('USBQ', dSubmit=20) +
                             diagnostic('BLELINK', gen=1, phyKnown=1, txPhy=2, rxPhy=2) + diagnostic('BLETX', gen=1, sdkAccepted=30) +
                             synthetic_perf(pub=20).encode() + b'\n')]
        self.debug_chunks = [(101.2, diagnostic('ADSHEALTH', crcValidEven=100, crcValidOdd=100, crcErrors=0)),
                             (101.5, RECORDED_STK50_COMPLETE.encode() + b'\n'),
                             (102.2, diagnostic('ADSHEALTH', crcValidEven=200, crcValidOdd=180, crcErrors=20))]

    def tearDown(self): self.tmp.cleanup()

    def write_ctrl(self):
        payload = b''
        events = list(self.events)
        for when, chunk in self.ctrl_chunks:
            events.append({'type': 'ctrl_rx', 't': when, 'offset': len(payload), 'length': len(chunk)})
            payload += chunk
        events.sort(key=lambda e: e['t'])
        (self.root / 'case.ctrl.rx.bin').write_bytes(payload)
        (self.root / 'case.events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in events))
        self.summary['ctrl_raw_sha256'] = hashlib.sha256(payload).hexdigest()
        (self.root / 'case.summary.json').write_text(json.dumps(self.summary))

    def write_debug(self, *, serial=SERIAL, status='STOPPED', compressed=False):
        raw, events = b'', [{'type': 'opened', 'monotonic': 100}]
        for when, chunk in self.debug_chunks:
            # Deliberately split lines over seven-byte read boundaries.
            for offset in range(0, len(chunk), 7):
                part = chunk[offset:offset+7]
                events.append({'type': 'rx', 'monotonic': when, 'offset': len(raw), 'length': len(part)})
                raw += part
        event_bytes = ''.join(json.dumps(e) + '\n' for e in events).encode()
        for suffix, payload in (('.debug.rx.bin', raw), ('.debug.events.jsonl', event_bytes)):
            path = self.root / ('debug' + suffix + ('.gz' if compressed else ''))
            path.write_bytes(gzip.compress(payload) if compressed else payload)
        summary = {'status': status, 'raw_sha256': hashlib.sha256(raw).hexdigest(),
                   'started_monotonic': 99, 'ended_monotonic': 104,
                   'metadata': {'usb_serial': serial, 'vid': '303a', 'pid': '4041', 'interface_number': 2}}
        path = self.root / 'debug.debug.summary.json'; path.write_text(json.dumps(summary))
        return path

    def analyse(self, debug=True, **kwargs):
        self.write_ctrl()
        paths = [self.write_debug(**kwargs)] if debug else []
        return analyse_capture(self.root, 'case', self.summary, debug_summaries=paths)

    def test_independent_streams_real_encoder_and_crc_denominator(self):
        self.ctrl_chunks.append((2, self.wire))
        self.ctrl_chunks.sort()
        self.debug_chunks.append((102.4, self.wire)); self.debug_chunks.sort()
        result = self.analyse()
        self.assertEqual(result['capture_counts']['trusted_complete_frames'], 1)
        self.assertEqual(result['whole_raw_parser_counters']['crc_errors'], 0)
        aux = result['bulk_diagnostics']
        self.assertTrue(aux['boot_continuity'])
        self.assertEqual(aux['control_queries']['USBQ']['counters']['dSubmit']['delta'], 10)
        self.assertEqual(aux['adc_crc']['checked_attempts'], 200)
        self.assertEqual(aux['adc_crc']['rate'], 0.1)
        self.assertFalse(aux['adc_crc']['within_limit'])
        self.assertEqual(aux['CTRL']['counts']['ignored_scientific_records'], len(self.wire.splitlines()))
        self.assertEqual(aux['CDC']['counts']['ignored_scientific_records'], len(self.wire.splitlines()))
        self.assertIsNotNone(aux['memory']['heap'])

    def test_missing_CDC_is_unknown_and_CTRL_not_scientific_receipt(self):
        aux = self.analyse(debug=False)['bulk_diagnostics']
        self.assertEqual(aux['CDC']['status'], 'NOT_OBSERVED')
        self.assertIsNone(aux['adc_crc']['rate'])
        self.assertIsNone(aux['memory'])
        self.assertEqual(aux['WDT_evidence'], 'NOT_OBSERVED')

    def test_partial_and_malformed_diagnostics_do_not_change_scientific_quality(self):
        self.debug_chunks = [(101.1, b'ADSHEALTH,crcErrors=123\n' + RECORDED_STK50_COMPLETE.split(',tSlot')[0].encode() + b'\n'),
                             (102, b'ADSHEALTH,crcValidEven=100')]
        result = self.analyse()
        aux = result['bulk_diagnostics']
        self.assertIsNone(aux['adc_crc']['rate'])
        self.assertIsNone(aux['memory']['heap'])
        self.assertEqual(aux['CDC']['rejected_records']['UNTERMINATED_FINAL_LINE'], 1)
        self.assertEqual(result['capture_counts']['trusted_complete_frames'], 1)

    def test_distinct_absolute_clock_and_faults_only_inside_selected_window(self):
        self.debug_chunks.insert(0, (100.5, b'E task_wdt: Task watchdog got triggered.\n'))
        self.debug_chunks.append((103.5, b'MFAULT,seq=8\n'))
        aux = self.analyse()['bulk_diagnostics']
        self.assertEqual(aux['CDC']['faults']['counts_by_phase'], {'warmup': 1, 'cleanup': 1})
        self.assertAlmostEqual(aux['adc_crc']['health']['elapsed_seconds'], 1.0)

    def test_cross_boot_no_counter_delta_and_same_data_frame_remains(self):
        self.ctrl_chunks[-1] = (3.2, diagnostic('CONFIG', session='BBBBBBBBBBBBBBBB') + diagnostic('USBQ', dSubmit=20))
        result = self.analyse(); aux = result['bulk_diagnostics']
        self.assertFalse(aux['boot_continuity'])
        self.assertIsNone(aux['adc_crc']['rate'])
        self.assertIsNone(aux['control_queries']['USBQ']['counters']['dSubmit']['delta'])
        self.assertEqual(result['capture_counts']['trusted_complete_frames'], 1)

    def test_debug_identity_and_live_summary_rejected_before_raw(self):
        for kwargs, reason in (({'serial': '000000000001'}, 'DEBUG_DEVICE_IDENTITY_MISMATCH'),
                               ({'status': 'RUNNING'}, 'DEBUG_SUMMARY_NOT_FINALIZED')):
            aux = self.analyse(**kwargs)['bulk_diagnostics']
            self.assertEqual(aux['debug_sources_excluded'][0]['reason'], reason)
            self.assertIsNone(aux['adc_crc']['rate'])

    def test_corrupt_hash_or_event_coverage_is_not_accepted(self):
        self.write_ctrl(); path = self.write_debug()
        (self.root / 'debug.debug.rx.bin').write_bytes(b'changed\n')
        aux = analyse_capture(self.root, 'case', self.summary, debug_summaries=[path])['bulk_diagnostics']
        self.assertIn('mismatch', aux['debug_sources_excluded'][0]['reason'])
        self.assertIsNone(aux['adc_crc']['rate'])
        self.summary['ctrl_raw_sha256'] = '0' * 64
        aux = analyse_capture(self.root, 'case', self.summary)['bulk_diagnostics']
        self.assertEqual(aux['CTRL']['status'], 'UNAVAILABLE_OR_INVALID')

    def test_gzip_inputs_and_autodiscovery_skip_debug_as_capture(self):
        self.analyse(compressed=True)
        result = analyse_directory(self.root)
        self.assertEqual(len(result['captures']), 1)
        self.assertEqual(result['captures'][0]['bulk_diagnostics']['adc_crc']['rate'], 0.1)

    def test_valid_small_heap_is_kept_and_no_terminal_retry_count_inferred(self):
        self.debug_chunks[1] = (101.5, RECORDED_STK50_COMPLETE.replace('8377960/8341148/8336464', '20/12/9').encode() + b'\n')
        aux = self.analyse()['bulk_diagnostics']
        self.assertEqual(aux['memory']['heap']['firmware_lifetime_minimum_bytes'], 9)
        self.assertNotIn('crcRetries', aux['adc_crc']['health']['counters'])

    def test_missing_absolute_clock_and_missing_CONFIG_bracket_do_not_guess(self):
        self.summary.pop('capture_monotonic_start')
        aux = self.analyse()['bulk_diagnostics']
        self.assertEqual(aux['reason'], 'ABSOLUTE_CAPTURE_CLOCK_UNAVAILABLE_OR_INCONSISTENT')
        self.summary['capture_monotonic_start'] = 101
        self.ctrl_chunks[-1] = (3.2, diagnostic('USBQ', dSubmit=20))
        aux = self.analyse()['bulk_diagnostics']
        self.assertFalse(aux['boot_continuity'])
        self.assertIsNone(aux['adc_crc']['rate'])

    def test_actual_production_diagnostic_field_order_matches_strict_schema(self):
        paths = {'CONFIG': 'core/measure/sensorarrayConfigIdentity.c',
                 'USB': 'main/control/sensorarrayUsbControl.c', 'USBQ': 'main/control/sensorarrayUsbControl.c',
                 'BLELINK': 'main/control/sensorarrayLinkDiagnostics.c', 'BLETX': 'main/control/sensorarrayLinkDiagnostics.c',
                 'SCHED': 'main/main.c', 'ADSHEALTH': 'core/measure/ads/sensorarrayAdsMatrix.c',
                 'ADSSPI': 'core/measure/ads/sensorarrayAdsMatrix.c', 'BL50': 'main/output/sensorarrayAsyncLog.c'}
        for tag, path in paths.items():
            source = (ROOT / path).read_text()
            begin = source.index('"' + tag + ','); end = source.index('\\n"', begin) + 3
            fmt = ''.join(re.findall(r'"((?:\\.|[^"\\])*)"', source[begin:end]))
            self.assertEqual(list(parse_fields(fmt)), SCHEMAS[tag].split(), tag)

    def test_CDC_reset_inside_window_blocks_ADC_delta_and_truncated_WDT_positive_retained(self):
        self.debug_chunks.insert(1, (101.4, b'RST,bootId=8\n'))
        self.debug_chunks.append((102.8, b'E task_wdt: Task watchdog got triggered.'))
        aux = self.analyse()['bulk_diagnostics']
        self.assertFalse(aux['boot_continuity'])
        self.assertIsNone(aux['adc_crc']['rate'])
        self.assertEqual(aux['CDC']['faults']['counts_by_phase']['capture'], 1)
        self.assertFalse(aux['CDC']['faults']['examples'][0]['newline_present'])

    def test_debug_archive_is_byte_verified_and_keeps_originals(self):
        path = self.write_debug()
        script = ROOT / 'docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts/archive_cdc_debug.py'
        spec = importlib.util.spec_from_file_location('archive_cdc_debug', script)
        archive = importlib.util.module_from_spec(spec); spec.loader.exec_module(archive)
        result = archive.archive(path, self.root / 'archive.json')
        self.assertTrue(result['originals_retained'])
        for item in result['files']:
            self.assertEqual(gzip.decompress(Path(item['gzip']).read_bytes()), Path(item['source']).read_bytes())
        with self.assertRaises(FileExistsError): archive.archive(path, self.root / 'again.json')
        summary = json.loads(path.read_text()); summary['status'] = 'RUNNING'; path.write_text(json.dumps(summary))
        with self.assertRaisesRegex(ValueError, 'not finalized'): archive.archive(path, self.root / 'live.json')

    def test_partial_debug_retains_positive_watchdog_without_claiming_full_coverage(self):
        self.write_ctrl()
        self.debug_chunks = [(101.2, b'E task_wdt: Task watchdog got triggered.\n')]
        path = self.write_debug()
        summary = json.loads(path.read_text()); summary['ended_monotonic'] = 101.3
        path.write_text(json.dumps(summary))
        aux = analyse_capture(self.root, 'case', self.summary, debug_summaries=[path])['bulk_diagnostics']
        self.assertEqual(aux['status'], 'INDEPENDENT_STREAMS_PARTIAL')
        self.assertEqual(aux['CDC']['faults']['counts_by_phase']['capture'], 1)
        self.assertIsNone(aux['adc_crc']['rate'])

    def test_wrong_event_clock_is_unknown_not_zero_CRC(self):
        self.write_ctrl(); path = self.write_debug()
        events = self.root / 'debug.debug.events.jsonl'
        lines = [json.loads(line) for line in events.read_text().splitlines()]
        lines[1].pop('monotonic')
        events.write_text(''.join(json.dumps(line) + '\n' for line in lines))
        aux = analyse_capture(self.root, 'case', self.summary, debug_summaries=[path])['bulk_diagnostics']
        self.assertEqual(aux['debug_sources_excluded'][0]['reason'], 'CDC_invalid_receipt_time')
        self.assertIsNone(aux['adc_crc']['rate'])

    def test_stage_CLI_emits_actual_independent_report_without_device_access(self):
        self.analyse()
        output = self.root / 'offline-report.json'
        command = [sys.executable, str(ROOT / 'docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts/analyze_bulk_stage.py'),
                   '--directory', str(self.root), '--output', str(output), '--debug-summary', str(self.root / 'debug.debug.summary.json')]
        result = subprocess.run(command, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(output.read_text())
        self.assertFalse(report['hardware_actions'])
        self.assertEqual(report['captures'][0]['capture_counts']['trusted_complete_frames'], 1)
        self.assertEqual(report['captures'][0]['bulk_diagnostics']['adc_crc']['rate'], 0.1)

    def test_formal_CDC0_shared_raw_control_and_independent_PID4042_debug(self):
        self.summary['transport'] = 'usb_cdc'
        self.summary['transport_metadata']['pid'] = 0x4042
        chunks = [(0.8, diagnostic('CONFIG') + diagnostic('USB', backend='otg_cdc_v1', dataDtr='unknown', debugDtr='unknown') + diagnostic('USBQ', dSubmit=10, dCallbackBytes='unknown', cCallbackBytes='unknown')),
                  (1.2, self.wire), (3.2, diagnostic('CONFIG') + diagnostic('USBQ', dSubmit=20, dCallbackBytes='unknown', cCallbackBytes='unknown'))]
        payload, events = b'', [self.events[0], self.events[-1]]
        for when, wire in chunks:
            events.append({'type': 'rx', 't': when, 'offset': len(payload), 'length': len(wire)})
            payload += wire
        self.summary['raw_sha256'] = hashlib.sha256(payload).hexdigest()
        (self.root / 'case.rx.bin').write_bytes(payload)
        (self.root / 'case.events.jsonl').write_text(''.join(json.dumps(e) + '\n' for e in sorted(events,key=lambda e:e['t'])))
        (self.root / 'case.summary.json').write_text(json.dumps(self.summary))
        path = self.write_debug()
        wrong = analyse_capture(self.root, 'case', self.summary, debug_summaries=[path])
        self.assertEqual(wrong['otg_diagnostics']['debug_sources_excluded'][0]['reason'], 'DEBUG_DEVICE_IDENTITY_MISMATCH')
        debug = json.loads(path.read_text()); debug['metadata']['pid'] = '4042'; path.write_text(json.dumps(debug))
        result = analyse_capture(self.root, 'case', self.summary, debug_summaries=[path])
        self.assertEqual(result['capture_counts']['trusted_complete_frames'], 1)
        self.assertNotIn('bulk_diagnostics', result)
        aux = result['otg_diagnostics']
        self.assertEqual(aux['control_source'], 'formal_CDC0_shared_DATA_CTRL_raw')
        self.assertEqual(aux['status'], 'SHARED_CDC0_AND_DEBUG_OBSERVED')
        self.assertEqual(aux['control_queries']['USBQ']['counters']['dSubmit']['delta'], 10)
        self.assertIsNone(aux['control_queries']['USBQ']['counters']['dCallbackBytes']['delta'])
        self.assertEqual(aux['adc_crc']['rate'], 0.1)


if __name__ == '__main__': unittest.main()
