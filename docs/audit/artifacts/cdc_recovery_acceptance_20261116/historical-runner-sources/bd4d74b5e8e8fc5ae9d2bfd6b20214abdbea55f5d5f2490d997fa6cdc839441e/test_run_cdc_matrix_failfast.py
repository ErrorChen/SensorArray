#!/usr/bin/env python3
"""Synthetic wrapper regressions. Never open USB/BLE or run the HIL child."""
from copy import deepcopy
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import run_cdc_matrix_failfast as runner


ELF = 'a' * 64
BOOT = ('1234567890ABCDEF', '7')


def good_summary():
    return {
        'status': 'CAPTURED', 'error': None, 'capture_seconds': 120.001,
        'capture_duration_satisfied': True,
        'safe_cap_recovery': 'CAP8_QUERY_VERIFIED',
        'replies': {'BUILD?': {'elfSha256': ELF}},
        'snapshots': {'before': {'CONFIG?': {'fields': {
            'session': BOOT[0], 'bootId': BOOT[1]}}}},
    }


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Child:
    """A controllable process boundary; no subprocess is spawned by this fake."""
    def __init__(self, *, ignores_interrupt=False, kill_unknown=False):
        self.returncode = None
        self.pid = 123456
        self.ignores_interrupt = ignores_interrupt
        self.kill_unknown = kill_unknown
        self.signals = []
        self.killed = False
        self.pending_wait_timeouts = []

    def poll(self):
        return self.returncode

    def send_signal(self, value):
        self.signals.append(value)
        if not self.ignores_interrupt:
            self.returncode = 0

    def wait(self, timeout=None):
        if self.returncode is not None:
            return self.returncode
        self.pending_wait_timeouts.append(timeout)
        if timeout is None:
            raise AssertionError('unbounded wait on an unconfirmed child')
        raise subprocess.TimeoutExpired(['synthetic-child'], timeout)

    def kill(self):
        self.killed = True
        if not self.kill_unknown:
            self.returncode = -signal.SIGKILL


class CaseValidationTests(unittest.TestCase):
    def test_image_duration_cleanup_and_boot_identity(self):
        summary = good_summary()
        self.assertEqual(runner.validate_case(summary, ELF, None), (None, BOOT))
        self.assertEqual(runner.validate_case(summary, ELF, BOOT), (None, BOOT))
        cases = [
            ('status', 'ERROR'), ('error', 'MFAULT'), ('capture_seconds', 119.999),
            ('capture_duration_satisfied', False), ('safe_cap_recovery', 'FAILED'),
        ]
        for field, value in cases:
            with self.subTest(field=field):
                changed = deepcopy(summary)
                changed[field] = value
                error, unchanged = runner.validate_case(changed, ELF, BOOT)
                self.assertTrue(error)
                self.assertEqual(unchanged, BOOT)
        self.assertTrue(runner.validate_case(summary, 'b' * 64, BOOT)[0])
        for changed_boot in (('FEDCBA0987654321', '7'), (BOOT[0], '8')):
            changed = deepcopy(summary)
            fields = changed['snapshots']['before']['CONFIG?']['fields']
            fields.update(session=changed_boot[0], bootId=changed_boot[1])
            self.assertTrue(runner.validate_case(changed, ELF, BOOT)[0])

    def test_unknown_or_malformed_boot_does_not_establish_a_session(self):
        for session, boot in ((None, '7'), ('', '7'), ('BAD', '7'),
                              (BOOT[0], None), (BOOT[0], ''), (BOOT[0], '-1'),
                              (BOOT[0], str(1 << 32))):
            with self.subTest(session=session, boot=boot):
                summary = good_summary()
                summary['snapshots']['before']['CONFIG?']['fields'].update(session=session, bootId=boot)
                error, established = runner.validate_case(summary, ELF, None)
                self.assertTrue(error)
                self.assertIsNone(established)


class ChildFenceTests(unittest.TestCase):
    def invoke(self, child, directory, *, observer_error=None):
        clock = Clock()
        logs = Path(directory) / 'child.command.txt'
        stop = Path(directory) / 'stop'
        observer = patch.object(runner, 'observer_problem', side_effect=observer_error) if observer_error else \
            patch.object(runner, 'observer_problem', return_value=None)
        with patch.object(runner.subprocess, 'Popen', return_value=child), observer, \
                patch.object(runner.time, 'monotonic', clock.monotonic), \
                patch.object(runner.time, 'sleep', clock.sleep):
            result = runner.run_child(['synthetic-never-executed'], logs, [], stop, timeout=.1, grace=.2)
        self.assertEqual(result['log_sha256'], hashlib.sha256(logs.read_bytes()).hexdigest())
        return result

    def test_deadline_interrupts_and_retains_reason_even_if_child_exits_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            child = Child()
            result = self.invoke(child, directory)
            self.assertEqual(child.signals, [signal.SIGINT])
            self.assertEqual(result['exit_code'], 0)
            self.assertIn('child_deadline', result['stop_reason'])
            self.assertFalse(child.killed)

    def test_grace_expiry_kills_without_claiming_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            child = Child(ignores_interrupt=True)
            result = self.invoke(child, directory)
            self.assertTrue(child.killed)
            self.assertEqual(result['exit_code'], -signal.SIGKILL)
            self.assertIn('unknown', result['stop_reason'])
            self.assertTrue(all(t is not None and t > 0 for t in child.pending_wait_timeouts))

    def test_unconfirmed_kill_reaping_is_bounded_and_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            child = Child(ignores_interrupt=True, kill_unknown=True)
            result = self.invoke(child, directory)
            self.assertTrue(child.killed)
            self.assertIsNone(result['exit_code'])
            self.assertIn('unknown', result['stop_reason'])
            self.assertTrue(all(t is not None and t > 0 for t in child.pending_wait_timeouts))

    def test_observer_io_error_still_cleans_up_child(self):
        with tempfile.TemporaryDirectory() as directory:
            child = Child()
            result = self.invoke(child, directory, observer_error=OSError('synthetic observer stat failure'))
            self.assertEqual(child.signals, [signal.SIGINT])
            self.assertIsNotNone(child.poll())
            self.assertIn('synthetic observer stat failure', result['stop_reason'])

    def test_interrupt_during_grace_still_attempts_kill_and_reap(self):
        class InterruptedGrace(Child):
            def wait(self, timeout=None):
                if not self.killed:
                    raise KeyboardInterrupt()
                return super().wait(timeout)
        with tempfile.TemporaryDirectory() as directory:
            child = InterruptedGrace(ignores_interrupt=True)
            result = self.invoke(child, directory)
            self.assertTrue(child.killed)
            self.assertEqual(result['exit_code'], -signal.SIGKILL)
            self.assertTrue(result['child_termination_known'])
            self.assertIn('KeyboardInterrupt', result['stop_reason'])

    def test_kill_error_retains_pid_and_reports_unknown_termination(self):
        class KillFails(Child):
            def kill(self):
                self.killed = True
                raise OSError('synthetic kill failure')
        with tempfile.TemporaryDirectory() as directory:
            child = KillFails(ignores_interrupt=True)
            result = self.invoke(child, directory)
            self.assertEqual(result['child_pid'], child.pid)
            self.assertFalse(result['child_termination_known'])
            self.assertIsNone(result['exit_code'])
            self.assertIn('kill_error:OSError', result['stop_reason'])
            self.assertIn('child_termination_unknown', result['stop_reason'])

    def test_actual_short_process_only_executes_python_print(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'child.txt'
            result = runner.run_child([sys.executable, '-B', '-c', 'print("SYNTHETIC_CHILD_ONLY")'],
                                      path, [], Path(directory) / 'stop', timeout=2, grace=.2)
            self.assertEqual(result['exit_code'], 0)
            self.assertIsNone(result['stop_reason'])
            self.assertEqual(path.read_text(), 'SYNTHETIC_CHILD_ONLY\n')


class SuiteTests(unittest.TestCase):
    def args(self, directory):
        return ['--output-directory', str(directory), '--label', 'synthetic-suite',
                '--expected-elf', ELF, '--expected-session', BOOT[0], '--expected-boot-id', BOOT[1],
                '--ble-address', '80:B5:4E:CD:D9:AE', '--stop-file', str(Path(directory) / 'stop')]

    def run_main(self, args, fake_child, *, gate_status='PASS_CASE_SCREEN', start_error=None, close_error=False):
        order = []
        def git(command, **kwargs):
            return 'synthetic-head\n' if kwargs.get('text') else b'synthetic-diff'
        def start(directory, label, options):
            if start_error:
                raise RuntimeError(start_error)
            paths = [directory / (label + '.' + kind + '.summary.json') for kind in ('ble', 'debug')]
            for path in paths: path.touch()
            order.append(('start', label))
            return [], directory / 'observer.stop', paths
        def close(owners, stop):
            order.append(('close', stop.parent.name))
            return [{'exit_code': None if close_error else 0, 'cleanup_errors': [],
                     'termination_known': not close_error}]
        def gate(summary, ble, debug, expected):
            self.assertEqual(order[-1], ('close', summary.parent.name))
            self.assertEqual(expected.output(), {'elfSha256': ELF, 'session': BOOT[0], 'bootId': BOOT[1]})
            order.append(('gate', summary.parent.name))
            return {'status': gate_status, 'science': {'synthetic_orchestration_only': True}}
        with patch.object(runner, 'run_child', side_effect=fake_child) as child, \
                patch.object(runner, 'start_observers', side_effect=start) as starter, \
                patch.object(runner, 'close_observers', side_effect=close) as closer, \
                patch.object(runner, 'qualify_closed_case', side_effect=gate) as qualifier, \
                patch.object(runner.subprocess, 'check_output', side_effect=git), redirect_stdout(io.StringIO()):
            code = runner.main(args)
        return code, json.loads((Path(args[1]) / 'synthetic-suite.failfast.json').read_text()), child, starter, closer, qualifier

    @staticmethod
    def finish_case(command, log_path, observers, stop_path, **kwargs):
        label = command[command.index('--label') + 1]
        directory = Path(command[command.index('--output-directory') + 1])
        (directory / (label + '.summary.json')).write_text(json.dumps(good_summary()))
        log_path.write_text('SYNTHETIC_WRAPPER_CHILD\n')
        return {'exit_code': 0, 'stop_reason': None, 'elapsed_seconds': .001,
                'log_sha256': hashlib.sha256(log_path.read_bytes()).hexdigest()}

    def test_plan_32_complete_120s_cases_each_closes_then_gates_same_expected_image(self):
        with tempfile.TemporaryDirectory() as directory:
            code, result, child, starter, closer, gate = self.run_main(self.args(directory), self.finish_case)
            self.assertEqual(code, 0)
            self.assertEqual(result['status'], 'SCREENED_32_NOT_FINAL_ACCEPTANCE')
            self.assertFalse(result['not_started'])
            self.assertEqual([x.call_count for x in (child, starter, closer, gate)], [32] * 4)
            observed = []
            for call in child.call_args_list:
                command = call.args[0]
                get = lambda key: command[command.index(key) + 1]
                observed.append((get('--mode'), int(get('--rows'))))
                self.assertEqual(get('--capture-seconds'), '120')
                self.assertEqual(get('--profile'), 'CVRCVRCV')
                self.assertEqual((get('--rail-avdd-uv'), get('--rail-avss-uv')), ('3350000', '-1795000'))
                self.assertEqual((get('--expected-elf'), get('--expected-session'), get('--expected-boot-id')), (ELF, *BOOT))
                self.assertNotIn('--suite', command)
            self.assertEqual(tuple(observed), runner.PLAN)

    def test_first_collector_error_stops_even_when_CAP_cleanup_succeeded(self):
        def failed(*args, **kwargs):
            value = self.finish_case(*args, **kwargs)
            value['exit_code'] = 1
            return value
        with tempfile.TemporaryDirectory() as directory:
            code, result, child, _, closer, gate = self.run_main(self.args(directory), failed)
            self.assertEqual(code, 1)
            self.assertEqual(child.call_count, 1)
            self.assertEqual(closer.call_count, 1)
            gate.assert_not_called()
            self.assertEqual(len(result['not_started']), 31)

    def test_numeric_FAIL_or_UNKNOWN_stops_without_retry_after_closed_observers(self):
        for verdict in ('FAIL', 'UNKNOWN'):
            with self.subTest(verdict=verdict), tempfile.TemporaryDirectory() as directory:
                code, result, child, _, closer, gate = self.run_main(self.args(directory), self.finish_case, gate_status=verdict)
                self.assertEqual(code, 1)
                self.assertEqual([x.call_count for x in (child, closer, gate)], [1] * 3)
                self.assertEqual(result['error'], 'closed_case_gate_' + verdict)
                self.assertEqual(len(result['not_started']), 31)

    def test_malformed_summary_retains_attempt_and_cleanup_before_stop(self):
        def malformed(*args, **kwargs):
            value = self.finish_case(*args, **kwargs)
            command = args[0]
            path = Path(command[command.index('--output-directory') + 1]) / (command[command.index('--label') + 1] + '.summary.json')
            path.write_text('{incomplete')
            return value
        with tempfile.TemporaryDirectory() as directory:
            code, result, child, _, closer, gate = self.run_main(self.args(directory), malformed)
            self.assertEqual(code, 1)
            self.assertEqual([child.call_count, closer.call_count], [1, 1])
            gate.assert_not_called()
            self.assertEqual(len(result['not_started']), 31)

    def test_boot_change_in_second_case_stops_before_numeric_comparison(self):
        calls = 0
        def changed(*args, **kwargs):
            nonlocal calls
            calls += 1
            value = self.finish_case(*args, **kwargs)
            if calls == 2:
                command = args[0]
                path = Path(command[command.index('--output-directory') + 1]) / (command[command.index('--label') + 1] + '.summary.json')
                summary = good_summary()
                summary['snapshots']['before']['CONFIG?']['fields']['bootId'] = '8'
                path.write_text(json.dumps(summary))
            return value
        with tempfile.TemporaryDirectory() as directory:
            code, result, child, _, closer, gate = self.run_main(self.args(directory), changed)
            self.assertEqual(code, 1)
            self.assertEqual([child.call_count, closer.call_count, gate.call_count], [2, 2, 1])
            self.assertIn('boot_changed_or_unknown', result['error'])
            self.assertEqual(len(result['not_started']), 30)

    def test_readiness_failure_never_starts_formal_child(self):
        with tempfile.TemporaryDirectory() as directory:
            code, result, child, _, closer, gate = self.run_main(self.args(directory), self.finish_case, start_error='not_ready')
            self.assertEqual(code, 1)
            child.assert_not_called()
            gate.assert_not_called()
            self.assertIn('not_ready', result['error'])

    def test_unknown_observer_owner_prevents_gate_and_next_case(self):
        with tempfile.TemporaryDirectory() as directory:
            code, result, child, _, _, gate = self.run_main(self.args(directory), self.finish_case, close_error=True)
            self.assertEqual(code, 1)
            self.assertEqual(child.call_count, 1)
            gate.assert_not_called()
            self.assertIn('observer_closure', result['error'])

    def test_prior_gzip_never_overwrites_or_opens_case(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'synthetic-suite-mixed8.rx.bin.gz'
            path.write_bytes(b'ORIGINAL_SYNTHETIC_BYTES')
            with patch.object(runner, 'start_observers') as starter, patch('sys.stderr', new=io.StringIO()), self.assertRaises(SystemExit):
                runner.main(self.args(directory))
            starter.assert_not_called()
            self.assertEqual(path.read_bytes(), b'ORIGINAL_SYNTHETIC_BYTES')


class ObserverTests(unittest.TestCase):
    def test_unknown_observer_termination_has_bounded_waits_and_retained_owner(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'observer.log'
            log = path.open('x')
            child = Child(ignores_interrupt=True, kill_unknown=True)
            result = runner.close_observers([{'kind': 'ble', 'process': child, 'log': log,
                                              'log_path': path, 'command': ['synthetic-only']}],
                                             Path(directory) / 'stop')
            self.assertFalse(result[0]['termination_known'])
            self.assertEqual(result[0]['pid'], child.pid)
            self.assertTrue(result[0]['cleanup_errors'])
            self.assertEqual(child.pending_wait_timeouts, [12, 2, 2])

    def test_identity_failure_after_spawn_still_reclaims_owned_process(self):
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as directory:
            child = Child()
            args = SimpleNamespace(ble_address='00:11:22:33:44:55', ble_adapter='hci0',
                                   usb_serial='001122334455', stop_file=Path(directory) / 'operator.stop')
            with patch.object(runner.subprocess, 'Popen', return_value=child), \
                    patch.object(runner, 'process_identity', side_effect=ProcessLookupError('synthetic')):
                with self.assertRaises(ProcessLookupError):
                    runner.start_observers(Path(directory), 'synthetic', args)
            self.assertEqual(child.poll(), 0)
            self.assertEqual(child.signals, [signal.SIGINT])
            self.assertTrue((Path(directory) / 'synthetic.observer-start-failure.json').exists())

    def test_process_stat_start_identity_and_zombie_handling(self):
        # /proc stat field22 is starttime; the comm field may contain spaces
        # and parentheses. Parse after its last closing parenthesis.
        fields = ['S'] + ['0'] * 18 + ['987654321']
        text = '123 (synthetic owner (worker)) ' + ' '.join(fields) + '\n'
        with patch.object(runner.Path, 'read_text', return_value=text), patch.object(runner.os, 'kill') as kill:
            self.assertEqual(runner.process_identity(123), '987654321')
            kill.assert_called_once_with(123, 0)
        with patch.object(runner.Path, 'read_text', return_value=text.replace(') S ', ') Z ')), \
                patch.object(runner.os, 'kill') as kill, self.assertRaises(ProcessLookupError):
            runner.process_identity(123)
        kill.assert_not_called()

    def test_empty_summary_requires_same_live_process_identity(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'observer.summary.json'
            self.assertIn('observer_not_started', runner.observer_problem([path]))
            path.touch()
            with patch.object(runner, 'process_identity', return_value='old-start'):
                self.assertIsNone(runner.observer_problem([path], [(123, 'old-start')]))
            with patch.object(runner, 'process_identity', return_value='new-start'):
                self.assertIn('observer_process_replaced', runner.observer_problem([path], [(123, 'old-start')]))
            with patch.object(runner, 'process_identity', side_effect=ProcessLookupError(123)):
                self.assertIn('observer_process_not_alive', runner.observer_problem([path], [(123, 'old-start')]))
            path.write_text('{"status":"RECORDED"}')
            self.assertIn('observer_already_closed', runner.observer_problem([path]))


if __name__ == '__main__':
    unittest.main()
