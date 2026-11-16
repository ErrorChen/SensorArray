#!/usr/bin/env python3
"""Synthetic safety regression: fake subprocess, serial, esptool and sysfs.

Run once normally and once with python -O. No real esptool module is imported,
no device is opened, no flash or reset occurs. Temporary copied scripts and
case records are retained so the exact tested source can be audited.
"""
from __future__ import annotations

import argparse
import ast
import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[5]
REL = Path('docs/audit/artifacts/cdc_recovery_acceptance_20261116/scripts')
SOURCES = {name: (ROOT / REL / name).read_bytes() for name in
           ('flash_app_usj.py', 'exit_usj_download_watchdog.py')}
RECORDS = []
WORK = Path(tempfile.mkdtemp(prefix='sensorarray-usj-safety-'))


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load_copy(path):
    spec = importlib.util.spec_from_file_location('synthetic_' + path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class SafetyTests(unittest.TestCase):
    def fixture(self, name):
        repo = WORK / name
        scripts = repo / REL
        scripts.mkdir(parents=True)
        for filename, source in SOURCES.items():
            (scripts / filename).write_bytes(source)
        (repo / 'main').mkdir()
        (repo / 'main/main.c').write_text('/* SYNTHETIC, never compiled */\n')
        (repo / 'sdkconfig').write_text('# SYNTHETIC configuration\n')
        (repo / 'partitions.csv').write_text(
            '# Name,Type,Subtype,Offset,Size,Flags\n'
            'nvs,data,nvs,0x9000,0x6000,\nphy_init,data,phy,0xf000,0x1000,\n'
            'factory,app,factory,0x10000,0x300000,\ncalib,data,nvs,0x310000,0x10000,\n')
        (repo / 'build').mkdir()
        app = repo / 'build/SensorArray.bin'
        app.write_bytes(b'SYNTHETIC_NOT_EXECUTABLE\0' * 200)
        manifest = {'head': 'synthetic-not-a-commit', 'build_version': 'synthetic',
                    'results': [{'exit_code': 0}], 'source_changed_during_run': False,
                    'inputs_after': {name: digest((repo / name).read_bytes()) for name in
                                     ('main/main.c', 'sdkconfig', 'partitions.csv')},
                    'app_sha256': digest(app.read_bytes()), 'app_bytes': app.stat().st_size}
        flasher = {'app': {'file': 'SensorArray.bin', 'offset': '0x10000', 'encrypted': 'false'},
                   'flash_settings': {'flash_mode': 'dio', 'flash_size': '16MB', 'flash_freq': '80m'},
                   'extra_esptool_args': {'chip': 'esp32s3'}}
        return repo, scripts, manifest, flasher

    def flash_case(self, name, *, mutation=None, mac='80:b5:4e:cd:d9:ac', chip='ESP32-S3',
                   before='no_reset', identity_exit=0, write_exit=0, marker='Hash of data verified.\n',
                   helper_exit=0, helper_events=None, timeout=None, existing=False):
        repo, scripts, manifest, flasher = self.fixture(name)
        app = repo / 'build/SensorArray.bin'
        out = scripts.parent
        if mutation == 'app_hash':
            app.write_bytes(bytes([app.read_bytes()[0] ^ 1]) + app.read_bytes()[1:])
        elif mutation == 'app_size':
            app.write_bytes(app.read_bytes() + b'changed')
        elif mutation == 'source_hash':
            (repo / 'main/main.c').write_text('/* changed synthetic source */\n')
        elif mutation == 'offset':
            flasher['app']['offset'] = '0x310000'
        elif mutation == 'oversize':
            app.write_bytes(b'X' * (0x300000 + 1))
            manifest.update(app_bytes=app.stat().st_size, app_sha256=digest(app.read_bytes()))
        elif mutation in {'partition_offset', 'partition_overlap', 'partition_capacity'}:
            partition = repo / 'partitions.csv'
            text = partition.read_text()
            if mutation == 'partition_offset':
                text = text.replace('factory,0x10000', 'factory,0x20000').replace('0x300000', '0x2F0000')
            elif mutation == 'partition_overlap':
                text = text.replace('0x310000', '0x300000')
            else:
                text = text.replace('0x300000', '0x1000')
            partition.write_text(text)
            manifest['inputs_after']['partitions.csv'] = digest(partition.read_bytes())
        elif mutation == 'manifest_failed':
            manifest['results'][0]['exit_code'] = 1
        elif mutation == 'manifest_empty':
            manifest['results'] = []
        elif mutation == 'manifest_missing_partition':
            del manifest['inputs_after']['partitions.csv']
        elif mutation == 'source_changed_flag':
            manifest['source_changed_during_run'] = True
        elif mutation == 'flash_settings':
            flasher['flash_settings']['flash_size'] = '32MB'
        (out / (name + '-build-manifest.json')).write_text(json.dumps(manifest))
        (repo / 'build/flasher_args.json').write_text(json.dumps(flasher))
        collision = out / (name + '-flash-usj-watchdog.txt')
        if existing:
            collision.write_text('ORIGINAL_EVIDENCE_DO_NOT_OVERWRITE\n')
        script = scripts / 'flash_app_usj.py'
        module = load_copy(script)
        calls = []
        if helper_events is None:
            helper_events = [{'watchdog_requested': True, 'watchdog_call_returned': True,
                              'app_boot_verified': False}]

        def fake_run(command, **kwargs):
            # Never delegate to a real process. All device-facing boundaries
            # must be routed through this mock, even in optimized Python.
            self.assertIsInstance(command, list)
            self.assertNotIn('shell', kwargs)
            self.assertEqual(kwargs['cwd'], repo)
            self.assertEqual(command[command.index('--port') + 1], '/dev/ttyACM_SYNTHETIC')
            calls.append(list(command))
            if 'flash_id' in command:
                stage, code = 'identity', identity_exit
                self.assertEqual(kwargs['timeout'], 180)
                kwargs['stdout'].write('SYNTHETIC MOCK OUTPUT ONLY\nChip is ' + chip + ' (fake)\nMAC: ' + mac + '\n')
                if mutation == 'source_after_identity':
                    (repo / 'main/main.c').write_text('/* changed while identity mock ran */\n')
            elif 'write_flash' in command:
                stage, code = 'app', write_exit
                self.assertEqual(kwargs['timeout'], 180)
                kwargs['stdout'].write('SYNTHETIC MOCK WRITE; NO HARDWARE\n' + marker)
            else:
                stage, code = 'watchdog', helper_exit
                self.assertEqual(command[1], str(scripts / 'exit_usj_download_watchdog.py'))
                self.assertEqual(kwargs['timeout'], 30)
                for event in helper_events:
                    kwargs['stdout'].write(json.dumps(event) + '\n')
            if stage == timeout:
                raise subprocess.TimeoutExpired(command, kwargs['timeout'])
            return SimpleNamespace(returncode=code)

        captured = io.StringIO()
        exception = None
        with mock.patch.object(module.subprocess, 'run', side_effect=fake_run), contextlib.redirect_stdout(captured):
            try:
                code = module.main([name, '--port', '/dev/ttyACM_SYNTHETIC', '--before', before])
            except ValueError as error:
                code, exception = 1, str(error)
        result_path = out / (name + '-flash-usj-result.json')
        report = json.loads(result_path.read_text()) if result_path.exists() else None
        if report is not None:
            self.assertFalse(report['app_boot_verified'])
            self.assertFalse(report['nvs_or_calibration_written'])
            self.assertFalse(report['partition_written'])
            self.assertFalse(report['bootloader_written'])
            self.assertFalse(report['efuse_written'])
            self.assertEqual(report['automatic_retries'], 0)
        for command in calls:
            self.assertFalse(set(command) & {'hard_reset', 'erase_flash', 'erase_region', '--encrypt'})
            if 'write_flash' in command:
                index = command.index('write_flash')
                self.assertEqual(command[index - 4:index], ['--before', 'no_reset', '--after', 'no_reset'])
                self.assertEqual(command[-2:], ['0x10000', str(app)])
                self.assertEqual(command[index + 1:], ['--flash_mode', 'dio', '--flash_freq', '80m',
                                                      '--flash_size', '16MB', '0x10000', str(app)])
        RECORDS.append({'case': name, 'kind': 'SYNTHETIC_FLASH_PROCESS', 'exit_code': code,
                        'commands': calls, 'result': report, 'exception': exception,
                        'stdout': captured.getvalue(), 'fixture_root': str(repo)})
        if existing:
            self.assertEqual(collision.read_text(), 'ORIGINAL_EVIDENCE_DO_NOT_OVERWRITE\n')
        return code, calls, report

    def test_success_all_identity_before_options_one_write_one_watchdog(self):
        for before in ('no_reset', 'default_reset', 'usb_reset'):
            code, calls, result = self.flash_case('success-' + before, before=before)
            self.assertEqual(code, 0)
            self.assertEqual(len(calls), 3)
            self.assertEqual(calls[0][-5:], ['--before', before, '--after', 'no_reset', 'flash_id'])
            self.assertTrue(result['flash_verified'])
            self.assertTrue(result['watchdog_requested'])
            self.assertTrue(result['watchdog_call_returned'])
            self.assertEqual(result['app_partition']['offset'], 0x10000)
            self.assertLessEqual(result['app_partition']['erase_end_exclusive'], 0x310000)

    def test_manifest_hash_offset_capacity_fail_before_device_operation(self):
        for mutation in ('app_hash', 'app_size', 'source_hash', 'offset', 'oversize',
                         'partition_offset', 'partition_overlap', 'partition_capacity',
                         'manifest_failed', 'manifest_empty', 'manifest_missing_partition',
                         'source_changed_flag', 'flash_settings'):
            with self.subTest(mutation=mutation):
                code, calls, result = self.flash_case('preflight-' + mutation, mutation=mutation)
                self.assertEqual(code, 1)
                self.assertEqual(calls, [])
                self.assertFalse(result['flash_verified'])
                self.assertFalse(result['watchdog_requested'])

    def test_wrong_mac_or_chip_and_identity_failure_never_write_or_reset(self):
        for name, kwargs in [('mac', {'mac': '80:b5:4e:00:00:00'}),
                             ('chip', {'chip': 'ESP32-C3'}), ('failed', {'identity_exit': 5}),
                             ('timeout', {'timeout': 'identity'}),
                             ('changed', {'mutation': 'source_after_identity'})]:
            code, calls, result = self.flash_case('identity-' + name, **kwargs)
            self.assertEqual(code, 1)
            self.assertEqual(len(calls), 1)
            self.assertFalse(result['flash_verified'])
            self.assertFalse(result['watchdog_helper_started'])

    def test_write_failure_timeout_or_nonreal_hash_marker_never_reset(self):
        for name, kwargs in [('failed', {'write_exit': 4}), ('timeout', {'timeout': 'app'}),
                             ('missing', {'marker': ''}),
                             ('embedded', {'marker': 'NOT verified: Hash of data verified.\n'}),
                             ('duplicate', {'marker': 'Hash of data verified.\nHash of data verified.\n'})]:
            code, calls, result = self.flash_case('write-' + name, **kwargs)
            self.assertEqual(code, 1)
            self.assertEqual(len(calls), 2)
            self.assertFalse(result['flash_verified'])
            self.assertFalse(result['watchdog_requested'])

    def test_helper_failure_timeout_or_unknown_never_retries_or_claims_boot(self):
        for name, kwargs in [('failed', {'helper_exit': 1, 'helper_events': [{'watchdog_requested': False}]}),
                             ('disconnect', {'helper_exit': 1, 'helper_events': [{'watchdog_requested': True, 'watchdog_call_returned': False}]}),
                             ('timeout', {'timeout': 'watchdog', 'helper_events': []}),
                             ('missing', {'helper_events': []})]:
            code, calls, result = self.flash_case('helper-' + name, **kwargs)
            self.assertEqual(code, 1)
            self.assertEqual(len(calls), 3)
            self.assertEqual(sum('write_flash' in call for call in calls), 1)
            self.assertTrue(result['flash_verified'])
            self.assertTrue(result['watchdog_helper_started'])
            self.assertFalse(result['watchdog_call_returned'])
            if name in {'timeout', 'missing'}:
                self.assertIsNone(result['watchdog_requested'])

    def test_existing_evidence_prevents_all_subprocesses(self):
        code, calls, result = self.flash_case('existing-evidence', existing=True)
        self.assertEqual(code, 1)
        self.assertEqual(calls, [])
        self.assertIsNone(result)

    def helper_case(self, name, *, sysfs_mac='80:B5:4E:CD:D9:AC', vid='303a', pid='1001',
                    mac='80b54ecdd9ac', secure=False, chip_id=9, usj=True, stuck=False,
                    connect_error=False, watchdog_error=False, close_error=False,
                    argument_mac='80:b5:4e:cd:d9:ac', dtr_after_connect=False):
        repo, scripts, _, _ = self.fixture(name)
        fake_sys = repo / 'synthetic-sys/class/tty'
        usb = repo / 'synthetic-sys/devices/usb1/1-1'
        base = usb / '1-1:1.0/tty/ttyACM_SYNTHETIC'
        base.mkdir(parents=True)
        for key, value in {'idVendor': vid, 'idProduct': pid, 'serial': sysfs_mac, 'devnum': '999'}.items():
            (usb / key).write_text(value + '\n')
        (fake_sys / 'ttyACM_SYNTHETIC').mkdir(parents=True)
        (fake_sys / 'ttyACM_SYNTHETIC/device').symlink_to(base)
        module = load_copy(scripts / 'exit_usj_download_watchdog.py')
        events = []
        actual_path = Path

        def fake_path(value):
            return fake_sys if str(value) == '/sys/class/tty' else actual_path(value)

        class FakeSerial:
            def __init__(self, **kwargs):
                events.append({'serial_construct': kwargs})
                self.dtr, self.rts, self.port = True, True, None
            def open(self):
                events.append({'serial_open': self.port, 'dtr': self.dtr, 'rts': self.rts})
            def close(self):
                events.append({'serial_close': True})
                if close_error:
                    raise OSError('SYNTHETIC close failure')

        class FakeS3:
            CHIP_NAME = 'ESP32-S3'
            RTC_CNTL_OPTION1_REG, GPIO_STRAP_REG = 0x100, 0x104
            RTC_CNTL_FORCE_DOWNLOAD_BOOT_MASK = 8
            def __init__(self, port, baud):
                self.port, self.secure_download_mode, self.force_cleared = port, secure, False
                events.append({'sdk_construct': baud})
            def connect(self, **kwargs):
                events.append({'sdk_connect': kwargs, 'sdk_post_connect_register_effects': 'not simulated; may disable watchdogs'})
                self.port.dtr = dtr_after_connect
                if connect_error:
                    raise OSError('SYNTHETIC connect failure')
            def get_chip_id(self):
                events.append({'chip_id_read': chip_id})
                return chip_id
            def read_mac(self):
                events.append({'rom_mac_read': mac})
                return bytes.fromhex(mac)
            def uses_usb_jtag_serial(self):
                return usj
            def read_reg(self, address):
                events.append({'read_reg': address})
                if address == self.GPIO_STRAP_REG:
                    return 1
                return 0x108 if not self.force_cleared or stuck else 0x100
            def write_reg(self, address, value, mask):
                events.append({'masked_write': [address, value, mask]})
                self.force_cleared = True
            def watchdog_reset(self):
                events.append({'watchdog_reset': True})
                if watchdog_error:
                    raise OSError('SYNTHETIC disconnect at watchdog request; outcome unknown')

        serial_module = ModuleType('serial')
        serial_module.Serial = FakeSerial
        esptool_module, targets_module, s3_module = (ModuleType(name) for name in ('esptool', 'esptool.targets', 'esptool.targets.esp32s3'))
        s3_module.ESP32S3ROM = FakeS3
        captured = io.StringIO()
        with mock.patch.object(module, 'Path', side_effect=fake_path), mock.patch.dict(sys.modules, {
                'serial': serial_module, 'esptool': esptool_module, 'esptool.targets': targets_module,
                'esptool.targets.esp32s3': s3_module}), contextlib.redirect_stdout(captured):
            code = module.main(['--port', '/dev/ttyACM_SYNTHETIC', '--mac', argument_mac])
        result = json.loads(captured.getvalue().splitlines()[-1])
        self.assertFalse(result['app_boot_verified'])
        self.assertEqual(result['flash_writes'], 0)
        self.assertEqual(result['efuse_writes'], 0)
        self.assertLessEqual(sum('watchdog_reset' in item for item in events), 1)
        self.assertLessEqual(sum('sdk_connect' in item for item in events), 1)
        for event in events:
            if 'sdk_connect' in event:
                self.assertEqual(event['sdk_connect'], {'mode': 'no_reset', 'attempts': 1})
            if 'serial_open' in event:
                self.assertFalse(event['dtr'])
                self.assertFalse(event['rts'])
            if 'serial_construct' in event:
                self.assertTrue(event['serial_construct']['exclusive'])
                self.assertIsNone(event['serial_construct']['port'])
        RECORDS.append({'case': name, 'kind': 'SYNTHETIC_HELPER_SERIAL_SDK_SYSFS', 'exit_code': code,
                        'events': events, 'result': result, 'stdout': captured.getvalue(), 'fixture_root': str(repo)})
        return code, events, result

    def test_helper_success_sysfs_chip_mac_force_readback_one_official_call(self):
        code, events, result = self.helper_case('helper-direct-success')
        self.assertEqual(code, 0)
        self.assertEqual([event['masked_write'] for event in events if 'masked_write' in event], [[0x100, 0, 8]])
        self.assertEqual(sum('watchdog_reset' in event for event in events), 1)
        self.assertTrue(result['watchdog_requested'])
        self.assertTrue(result['watchdog_call_returned'])
        self.assertEqual(result['force_after'] & 8, 0)
        self.assertEqual(sum('serial_close' in event for event in events), 1)

    def test_helper_bad_sysfs_or_argument_never_opens_device(self):
        for name, kwargs in [('vid', {'vid': '9999'}), ('pid', {'pid': '4042'}),
                             ('mac', {'sysfs_mac': '80:b5:4e:00:00:00'}),
                             ('argument', {'argument_mac': '80:b5:4e:cd:d9'})]:
            code, events, result = self.helper_case('helper-sysfs-' + name, **kwargs)
            self.assertEqual(code, 1)
            self.assertEqual(events, [])
            self.assertFalse(result['watchdog_requested'])

    def test_helper_security_chip_mac_transport_or_connect_failure_never_explicitly_writes(self):
        for name, kwargs in [('secure', {'secure': True}), ('chip', {'chip_id': 8}),
                             ('mac', {'mac': '80b54e000000'}), ('transport', {'usj': False}),
                             ('dtr', {'dtr_after_connect': True}), ('connect', {'connect_error': True})]:
            code, events, result = self.helper_case('helper-reject-' + name, **kwargs)
            self.assertEqual(code, 1)
            self.assertFalse(any('masked_write' in event or 'watchdog_reset' in event for event in events))
            self.assertFalse(result['watchdog_requested'])
            self.assertEqual(sum('serial_close' in event for event in events), 1)

    def test_helper_force_readback_failure_prevents_watchdog(self):
        code, events, result = self.helper_case('helper-force-stuck', stuck=True)
        self.assertEqual(code, 1)
        self.assertEqual(sum('masked_write' in event for event in events), 1)
        self.assertFalse(any('watchdog_reset' in event for event in events))
        self.assertFalse(result['watchdog_requested'])

    def test_helper_disconnect_is_uncertain_no_retry_and_preserves_cleanup_error(self):
        code, events, result = self.helper_case('helper-disconnect-cleanup', watchdog_error=True, close_error=True)
        self.assertEqual(code, 1)
        self.assertEqual(sum('watchdog_reset' in event for event in events), 1)
        self.assertTrue(result['watchdog_requested'])
        self.assertFalse(result['watchdog_call_returned'])
        self.assertIn('disconnect', result['error']['message'])
        self.assertIn('close failure', result['cleanup_error']['message'])

    def test_no_assert_safety_checks_in_either_script(self):
        for name, source in SOURCES.items():
            tree = ast.parse(source, filename=name)
            self.assertFalse(any(isinstance(node, ast.Assert) for node in ast.walk(tree)), name)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError('test evidence already exists')
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(SafetyTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    current = {name: digest((ROOT / REL / name).read_bytes()) for name in SOURCES}
    original = {name: digest(source) for name, source in SOURCES.items()}
    unchanged = current == original
    report = {'schema': 'flash_usj_safety_synthetic_v1',
              'status': 'PASS_SYNTHETIC' if result.wasSuccessful() and unchanged else 'FAIL',
              'tests_run': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
              'optimization_level': sys.flags.optimize, 'command': [sys.executable, *(['-O'] if sys.flags.optimize else []), *sys.argv],
              'source_sha256_before': original, 'source_sha256_after': current,
              'source_unchanged_during_run': unchanged, 'test_source_sha256': digest(Path(__file__).read_bytes()),
              'hardware_actions': False, 'real_esptool_imported': False,
              'case_count': len(RECORDS), 'cases': RECORDS, 'temporary_root': str(WORK),
              'failure_details': [{'test': str(test), 'traceback': trace} for test, trace in result.failures + result.errors],
              'limits': ['Synthetic mocks exercise process gating and helper control flow, not physical flash or boot.',
                         'Normal SDK connect is kept; its watchdog-register effects before MAC readback are not simulated.',
                         'No success status here verifies application enumeration, BUILD identity, USB power or calibration.']}
    with args.output.open('x') as stream:
        json.dump(report, stream, indent=2)
        stream.write('\n')
    print(json.dumps({'status': report['status'], 'tests_run': result.testsRun, 'cases': len(RECORDS),
                      'optimization_level': sys.flags.optimize, 'evidence': str(args.output)}))
    return 0 if result.wasSuccessful() and unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
