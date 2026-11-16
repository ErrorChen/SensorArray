#!/usr/bin/env python3
"""Synthetic early-failure USBBOOT tests using real Collector and fake I/O/time.

No real hardware, esptool, reset, flash, tty, or sysfs access occurs. The existing
recovery tests supply only their fake environment classes; their tests do not run.
"""
from __future__ import annotations

import argparse
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import request_early_rom as early
from test_software_recovery import Environment as RecoveryEnvironment

ELF = 'ab' * 32
WORK = Path(tempfile.mkdtemp(prefix='sensorarray-early-rom-synthetic-'))
CASES = []


class Environment(RecoveryEnvironment):
    def __init__(self, mode='normal'):
        super().__init__(mode, 'USBBOOT')
        self.reply_counts = {}
        self.partial_cut = False
        if mode == 'wrong_new_mac':
            self.new['serial'] = self.new['serial_raw'] = '80B54E000000'

    def inventory(self):
        result = super().inventory()
        if self.mode == 'duplicate_new' and self.action_time is not None and self.clock.now() - self.action_time >= .5:
            other = deepcopy(self.new)
            other['devnum'] += 1
            other['sysfs_path'] += '-other'
            return [deepcopy(self.new), other]
        return result

    def open(self, serial):
        port = super().open(serial)
        if self.mode == 'partial_write_failure':
            original = port.write_some
            def partial(data, deadline):
                if self.partial_cut:
                    raise OSError('SYNTHETIC failure after incomplete action prefix')
                if b'USBBOOT' in data:
                    self.partial_cut = True
                    return original(data[:4], deadline)
                return original(data, deadline)
            port.write_some = partial
        return port

    def reply(self, command, selected):
        self.reply_counts[command] = self.reply_counts.get(command, 0) + 1
        if command == 'BUILD?':
            return ('BUILD,idf=v5.5.1,target=esp32s3,project=SensorArray,proto=2,version=bb230d0-synthetic,elfSha256=' +
                    ('cd' * 32 if self.mode == 'wrong_elf' else ELF[:-1] if self.mode == 'truncated_elf' else ELF))
        if command == 'PROTO?':
            caps = 'usb_rom_handoff_v1/usb_cdc_data_v1/restart_query_v1/session64'
            if self.mode == 'missing_capability':
                caps = caps.replace('usb_rom_handoff_v1/', '')
            return 'PROTO,version=2,caps=' + caps
        if command in {'BOOT?', 'READY?'}:
            boot_id = 18 if self.mode == 'boot_mismatch' and command == 'READY?' else 17
            stage = 'running' if self.mode == 'wrong_stage' else 'scan_task_create_failed'
            error = '0x108' if self.mode == 'wrong_error' else '0x101'
            ready = '1' if self.mode == 'ready' or (self.mode == 'ready_changed' and self.reply_counts[command] > 1) else '0'
            common = f'bootId={boot_id},boot=4,stage={stage},err={error},ready={ready}'
            return ('BOOT,' + common + ',seq=' + ('1' if self.mode == 'progress' else '0') if command == 'BOOT?' else 'READY,' + common)
        if command == 'USB?':
            return ('USB,backend=otg_cdc_v1,init=1,err=0x0,mounted=' + ('0' if self.mode == 'unmounted' else '1') +
                    ',suspended=0,romAvailable=' + ('0' if self.mode == 'rom_unavailable' else '1'))
        if command == 'RESTART?':
            session = '0000000000000000' if self.mode == 'zero_session' else '1111222233334444'
            if self.mode == 'session_changed' and self.reply_counts[command] > 1:
                session = 'AAAABBBBCCCCDDDD'
            return ('RESTART,session=' + session + ',bootId=17,busy=' + ('1' if self.mode == 'pending' else '0') +
                    ',phase=idle,pendingId=0,lastId=0')
        raise RuntimeError('Unexpected command in fake early-state fixture: ' + command)


class EarlyRomTests(unittest.TestCase):
    def run_case(self, mode, *, expected_status, attempts, completed_actions=None):
        env = Environment(mode)
        out = WORK / mode
        result = early.observe(early.Options(out, mode, ELF, duration=2.5, query_timeout=.2, poll_seconds=.05),
                               inventory=env.inventory, port_factory=env.open, clock=env.clock.now,
                               sleep=env.clock.sleep, sysfs_exists=env.sysfs_exists)
        self.assertEqual(result['status'], expected_status, result)
        self.assertEqual(result['action_attempts'], attempts)
        self.assertEqual(len(env.actions), attempts if completed_actions is None else completed_actions)
        self.assertLessEqual(env.peak_active, 1)
        self.assertEqual(env.active, 0)
        self.assertLessEqual(env.opens, 1)
        self.assertLessEqual(result['elapsed_seconds'], 2.5 + 1e-9)
        self.assertNotIn('CONFIG?', env.commands)
        self.assertFalse(any(command in {'RESTART', 'USBBOOT'} for command in env.commands if command != 'USBBOOT'))
        self.assertFalse(result['ROM_port_opened'])
        self.assertFalse(result['esptool_executed'])
        self.assertFalse(result['flash_written'])
        self.assertFalse(result['system_USB_reset_requested'])
        self.assertFalse(result['automatic_action_retry'])
        raw = Path(result['raw_file']).read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), result['raw_sha256'])
        self.assertEqual(len(raw), result['rx_bytes'])
        self.assertEqual(result, json.loads((out / (mode + '.summary.json')).read_text()))
        events = [json.loads(line) for line in Path(result['events_file']).read_text().splitlines()]
        self.assertEqual(sum(event.get('type') == 'single_action_attempt' for event in events), attempts)
        self.assertTrue(all(left['monotonic'] <= right['monotonic'] for left, right in zip(events, events[1:])))
        CASES.append({'case': mode, 'status': 'PASS_SYNTHETIC', 'helper_status': result['status'],
                      'commands': env.commands, 'action_attempts': attempts, 'complete_action_commands': len(env.actions),
                      'logical_reader_opens': env.opens, 'peak_readers': env.peak_active,
                      'elapsed_fake_seconds': result['elapsed_seconds'], 'summary': str(out / (mode + '.summary.json'))})
        return env, result

    def test_identified_failure_single_action_and_independent_rom_enumeration(self):
        env, result = self.run_case('normal', expected_status='ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED', attempts=1)
        self.assertGreater(result['partial_writes'], 0)
        self.assertIsNotNone(result['old_epoch_absent_monotonic'])
        self.assertIsNotNone(result['old_interface_absent_monotonic'])
        self.assertEqual(result['new_device']['pid'], '1001')
        self.assertNotEqual(result['old_device']['devnum'], result['new_device']['devnum'])
        self.assertEqual(env.commands.count('USBBOOT'), 1)

    def test_identity_state_or_capability_mismatch_prevents_action(self):
        for mode in ('wrong_mac', 'wrong_open_role', 'wrong_elf', 'truncated_elf', 'missing_capability',
                     'wrong_stage', 'wrong_error', 'ready', 'progress', 'boot_mismatch', 'zero_session',
                     'pending', 'unmounted', 'rom_unavailable', 'session_changed', 'ready_changed', 'no_query_replies'):
            with self.subTest(mode=mode):
                self.run_case(mode, expected_status='ERROR', attempts=0)

    def test_publications_or_reused_identity_do_not_prove_enumeration(self):
        for mode in ('ack_without_enumeration', 'cached_old_present', 'path_reused_without_observed_absence',
                     'same_epoch_reappears', 'wrong_new_mac', 'duplicate_new'):
            with self.subTest(mode=mode):
                _, result = self.run_case(mode, expected_status='DEADLINE_WITHOUT_REQUIRED_ROM_ENUMERATION', attempts=1)
                self.assertEqual(result['elapsed_seconds'], 2.5)
                if mode == 'ack_without_enumeration':
                    self.assertTrue(result['publications'])
                if mode == 'path_reused_without_observed_absence':
                    self.assertIsNone(result['old_interface_absent_monotonic'])

    def test_lost_ack_and_uncertain_completed_write_never_replay(self):
        for mode in ('ack_lost', 'action_write_unknown'):
            with self.subTest(mode=mode):
                _, result = self.run_case(mode, expected_status='ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED', attempts=1)
                if mode == 'ack_lost':
                    self.assertEqual(result['publications'], [])
                else:
                    self.assertEqual(result['action_host_write'], 'outcome_unknown')

    def test_partial_action_failure_is_preserved_and_never_replayed(self):
        _, result = self.run_case('partial_write_failure', expected_status='DEADLINE_WITHOUT_REQUIRED_ROM_ENUMERATION',
                                  attempts=1, completed_actions=0)
        self.assertEqual(result['action_host_write'], 'outcome_unknown')
        self.assertEqual(result['action_known_host_accepted_bytes'], 4)

    def test_full_hash_argument_and_existing_output_are_non_bypassable(self):
        env = Environment()
        with self.assertRaises(ValueError):
            early.observe(early.Options(WORK / 'invalid-hash', 'invalid-hash', ELF[:16]), inventory=env.inventory,
                          port_factory=env.open, clock=env.clock.now, sleep=env.clock.sleep, sysfs_exists=env.sysfs_exists)
        self.assertEqual(env.opens, 0)
        out = WORK / 'existing'
        out.mkdir()
        original = out / 'existing.rx.bin'
        original.write_bytes(b'ORIGINAL_DO_NOT_CHANGE')
        with self.assertRaises(FileExistsError):
            early.observe(early.Options(out, 'existing', ELF), inventory=env.inventory, port_factory=env.open,
                          clock=env.clock.now, sleep=env.clock.sleep, sysfs_exists=env.sysfs_exists)
        self.assertEqual(original.read_bytes(), b'ORIGINAL_DO_NOT_CHANGE')
        self.assertEqual(env.opens, 0)
        self.assertFalse(any(isinstance(node, ast.Assert) for node in ast.walk(ast.parse(Path(early.__file__).read_text()))))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError('test output exists')
    sources = [Path(early.__file__), Path(early.shared.__file__), Path(early.shared.capture.__file__), Path(__file__)]
    before = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(EarlyRomTests))
    after = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    report = {'status': 'PASS_SYNTHETIC' if result.wasSuccessful() and before == after else 'FAIL',
              'tests_run': result.testsRun, 'case_count': len(CASES), 'failures': len(result.failures), 'errors': len(result.errors),
              'command': [sys.executable, *(['-O'] if sys.flags.optimize else []), *sys.argv],
              'optimization_level': sys.flags.optimize, 'source_sha256_before': before, 'source_sha256_after': after,
              'hardware_actions': False, 'temporary_root': str(WORK), 'cases': CASES,
              'failure_details': [{'test': str(test), 'traceback': trace} for test, trace in result.failures + result.errors],
              'limits': ['Fake transport/clock/sysfs only; no physical enumeration, firmware response or reset verified.',
                         'RACK/RAPP tests are synthetic local publication events, never physical recovery proof.']}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'status': report['status'], 'tests': result.testsRun, 'cases': len(CASES), 'evidence': str(args.output)}))
    return 0 if report['status'] == 'PASS_SYNTHETIC' else 1


if __name__ == '__main__':
    raise SystemExit(main())
