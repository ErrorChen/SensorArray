#!/usr/bin/env python3
"""Offline regression of performance time bounds, SF50 selection and source binding."""
import gzip
import hashlib
from pathlib import Path
import tempfile
import unittest

from closed_matrix_performance import bound_bytes, collector_clock, counter_rates, debug_observations


def sf50(start, end, fps, rows=8):
    return (f'SF50,seq={start}-{end},n={end-start+1},rows={rows},cfps=30.00,efps={fps:.2f},'
            'ofps=1.0/2.0/0.0,bad=0/0/0,drop=0/0/0,q=0/0\n').encode()


def observe(lines):
    events, raw = [], bytearray()
    for i, line in enumerate(lines):
        events.append({'type': 'rx', 'monotonic': 10. + i, 'offset': len(raw), 'length': len(line)})
        raw.extend(line)
    return debug_observations(bytes(raw), events, {'configuration_groups': [
        {'rows': 8, 'sequence': {'first': 100, 'last': 300}}]},
        {'capture_monotonic_start': 11., 'capture_monotonic_end': 15.})


class PerformanceTests(unittest.TestCase):
    def test_sf50_uses_measured_firmware_rates_in_both_time_and_scientific_seq_bounds(self):
        observed, stack = observe([sf50(100, 109, 99.), sf50(90, 105, 90.), sf50(110, 159, 20.),
                                   sf50(160, 209, 24.), sf50(210, 259, 88., rows=7),
                                   sf50(210, 259, 99.)[:-2] + b'\n', sf50(260, 299, 77.)])
        self.assertEqual(observed['SF50_count'], 2)
        self.assertEqual(observed['emit_fps'], {'min': 20., 'median': 22., 'max': 24.})
        self.assertEqual(observed['first']['receipt_monotonic'], 12.)
        self.assertEqual(observed['last']['last_sequence'], 209)
        self.assertEqual(observed['excluded']['malformed_in_window'], 1)
        self.assertEqual(stack['status'], 'UNKNOWN')

    def test_sf50_repeated_sequence_intervals_cannot_bias_an_emit_estimate(self):
        observed, _ = observe([b'\n', sf50(100, 149, 20.), sf50(100, 149, 90.)])
        self.assertEqual(observed['status'], 'UNKNOWN')
        self.assertNotIn('emit_fps', observed)

    def test_collector_origin_uses_independent_snapshots_and_preserves_event_write_delay(self):
        snapshots = {p: {q: {'monotonic': 1050., 't': 50.} for q in ('PERF?', 'USBQ?', 'BLETX?')}
                     for p in ('before', 'after')}
        summary = {'snapshots': snapshots, 'capture_monotonic_start': 1100., 'capture_monotonic_end': 1220.}
        events = [{'type': 'capture_start', 'monotonic': 1100., 't': 100.0001},
                  {'type': 'capture_end', 'monotonic': 1220., 't': 220.012}]
        clock = collector_clock(events, summary)
        self.assertAlmostEqual(clock['end_event_write_delay_seconds'], .012)
        self.assertEqual(clock['collector_origin_monotonic'], 1000.)
        snapshots['after']['USBQ?']['monotonic'] += 3.
        with self.assertRaises(ValueError):
            collector_clock(events, summary)

    def test_missing_counter_is_unknown_while_observed_zero_is_preserved(self):
        stats = counter_rates({'status': 'OBSERVED', 'elapsed_seconds': 10.,
                               'counters': {'dSubmit': {'delta': 0}, 'cSubmit': {'delta': 20}}}, ('dSubmit','cSubmit','dCallbackBytes'))
        self.assertEqual(stats['per_second'], {'dSubmit': 0., 'cSubmit': 2., 'dCallbackBytes': None})

    def test_uses_the_bound_raw_or_matching_archive_and_rejects_changed_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp).resolve()
            path = directory / 'case.rx.bin'
            raw = b'original'
            ref = {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}
            Path(str(path) + '.gz').write_bytes(gzip.compress(raw))
            self.assertEqual(bound_bytes(ref, directory), raw)
            path.write_bytes(b'changed')
            with self.assertRaises(ValueError):
                bound_bytes(ref, directory)


if __name__ == '__main__':
    unittest.main()
