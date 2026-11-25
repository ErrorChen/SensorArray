#!/usr/bin/env python3
"""Offline regression for the closed matrix exporter’s capture-duration evidence."""
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from summarize_closed_matrix import combine_case, read_case


def records(mode='CAP', rows=8, seconds=122):
    label = f'test-{mode.lower()}{rows}'
    identity = {'elfSha256': 'a' * 64, 'session': 'ABCDEF0123456789', 'bootId': '2'}
    science = {'status': 'PASS_NUMERIC_SCREEN'}
    command = {'label': label, 'mode': mode, 'rows': rows, 'capture_requested_seconds': seconds,
               'status': 'PASS_CASE_SCREEN', 'gate_status': 'PASS_CASE_SCREEN', 'science': science,
               'command': ['collector', '--label', label, '--mode', mode, '--rows', str(rows),
                           '--capture-seconds', str(seconds), '--expected-elf', identity['elfSha256'],
                           '--expected-session', identity['session'], '--expected-boot-id', identity['bootId']],
               'child_termination_known': True, 'exit_code': 0, 'stop_reason': None,
               'observer_closure': [{'kind': kind, 'termination_known': True, 'exit_code': 0, 'cleanup_errors': []}
                                    for kind in ('ble', 'debug')]}
    host_seconds = seconds + .001
    summary = {'benchmark_kind': 'configuration', 'benchmark_mode': mode, 'rows_requested': rows,
               'capture_requested_seconds': seconds, 'capture_seconds': host_seconds,
               'capture_monotonic_start': 50., 'capture_monotonic_end': 50. + host_seconds,
               'capture_duration_satisfied': True, 'safe_cap_recovery': 'CAP8_QUERY_VERIFIED',
               'image_authority': {'expected': identity}}
    capture = {'label': label, 'status': 'RECORDED', 'requested': {'requested_seconds': seconds},
               'window': {'requested_seconds_met': True},
               'configuration_groups': [{'rates': {'device_span_seconds': seconds - .08}}]}
    gate = {'status': 'PASS_CASE_SCREEN', 'science': science, 'expected': identity,
            'analysis': {'captures': [capture]}}
    return command, summary, gate


def replay(command, summary, gate):
    # Rebind all hashes so a negative case reaches the check under test.
    with tempfile.TemporaryDirectory() as tmp:
        matrix = Path(tmp).resolve()
        case = matrix / command['label']
        case.mkdir()
        formal = case / (case.name + '.summary.json')
        formal.write_bytes(json.dumps(summary).encode())
        reference = {'path': str(formal), 'sha256': hashlib.sha256(formal.read_bytes()).hexdigest()}
        command['summary_sha256'] = reference['sha256']
        gate['formal_authority'] = {'summary': reference}
        gate['analysis']['captures'][0]['files'] = {'summary': reference}
        (case / (case.name + '.command.json')).write_text(json.dumps(command))
        (case / (case.name + '.gate.json')).write_text(json.dumps(gate))
        row = read_case(matrix, 'test', command['mode'], command['rows'])
        return combine_case([row])


class DurationTests(unittest.TestCase):
    def test_122_second_cap8_and_res8_retain_separate_host_and_device_spans(self):
        for mode in ('CAP', 'RES'):
            with self.subTest(mode=mode):
                row = replay(*records(mode))
                self.assertEqual(row['status'], 'PASS_CASE_SCREEN')
                self.assertEqual(row['capture_requested_seconds'], 122)
                self.assertEqual(row['metrics']['capture_seconds'], 122.001)
                self.assertEqual(row['metrics']['device_span_seconds'], 121.92)

    def test_historical_commands_without_explicit_duration_only_accept_120_seconds(self):
        for seconds, status in ((120, 'PASS_CASE_SCREEN'), (122, 'UNKNOWN')):
            with self.subTest(seconds=seconds):
                command, summary, gate = records(seconds=seconds)
                del command['capture_requested_seconds']
                self.assertEqual(replay(command, summary, gate)['status'], status)

    def test_margin_excludes_other_cases_and_other_durations(self):
        for mode, rows, seconds in [('CAP', 7, 122), ('RES', 1, 122), ('VOLT', 8, 122),
                                   ('MIXED', 8, 122), ('CAP', 8, 121), ('RES', 8, 123)]:
            with self.subTest(mode=mode, rows=rows, seconds=seconds):
                self.assertEqual(replay(*records(mode, rows, seconds))['status'], 'UNKNOWN')

    def test_collector_argv_command_and_formal_summary_must_agree(self):
        for mismatch in ('argv duration', 'duplicate argv duration', 'argv mode', 'command', 'summary', 'gate'):
            with self.subTest(mismatch=mismatch):
                command, summary, gate = records()
                argv = command['command']
                if mismatch == 'argv duration':
                    argv[argv.index('--capture-seconds') + 1] = '120'
                elif mismatch == 'duplicate argv duration':
                    argv.extend(['--capture-seconds', '122'])
                elif mismatch == 'argv mode':
                    argv[argv.index('--mode') + 1] = 'VOLT'
                elif mismatch == 'command':
                    command['capture_requested_seconds'] = 120
                elif mismatch == 'summary':
                    summary['capture_requested_seconds'] = 120
                else:
                    gate['analysis']['captures'][0]['requested']['requested_seconds'] = 120
                row = replay(command, summary, gate)
                self.assertEqual(row['status'], 'UNKNOWN')
                self.assertIsNone(row['metrics'])

    def test_declared_122_seconds_require_satisfied_actual_host_duration_and_brackets(self):
        for changes in ({'capture_seconds': 120., 'capture_monotonic_end': 170.},
                        {'capture_monotonic_end': 170.}, {'capture_duration_satisfied': False},
                        {'capture_seconds': float('nan')}):
            with self.subTest(changes=changes):
                command, summary, gate = records()
                summary.update(changes)
                self.assertEqual(replay(command, summary, gate)['status'], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
