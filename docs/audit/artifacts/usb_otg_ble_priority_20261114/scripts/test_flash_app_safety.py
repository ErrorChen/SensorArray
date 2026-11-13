#!/usr/bin/env python3
"""Execute an unmodified flash_app.py copy with synthetic files/subprocess only.

No esptool, device, serial or flash command is executed. The temporary cases
are retained under /tmp so copied script hashes and generated command logs can
be audited separately from any real flashing evidence.
"""
import argparse
import contextlib
import hashlib
import io
import json
import os
from pathlib import Path
import runpy
import sys
import tempfile
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[5]
RELATIVE_SCRIPT = Path('docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts/flash_app.py')


def sha(data): return hashlib.sha256(data).hexdigest()


def run_case(work, source, name, *, before='default_reset', mac='80:b5:4e:cd:d9:ac',
             identity_code=0, mutation=None):
    repo = work / name
    script = repo / RELATIVE_SCRIPT
    script.parent.mkdir(parents=True)
    script.write_bytes(source)
    assert sha(script.read_bytes()) == sha(source)
    out = script.parents[1]
    (repo / 'main').mkdir()
    source_file = repo / 'main/main.c'
    source_file.write_text('/* SYNTHETIC flash-script test source, never firmware */\n')
    (repo / 'build').mkdir()
    app = repo / 'build/SensorArray.bin'
    app.write_bytes(b'SYNTHETIC_NON_EXECUTABLE_APP_FIXTURE\x00\xff')
    manifest = {'head': 'synthetic-not-a-commit', 'build_version': 'synthetic-safety-test',
                'results': [{'exit_code': 0}], 'source_changed_during_run': False,
                'inputs_after': {'main/main.c': sha(source_file.read_bytes())},
                'app_sha256': sha(app.read_bytes()), 'app_bytes': app.stat().st_size}
    (out / (name + '-build-manifest.json')).write_text(json.dumps(manifest))
    (repo / 'build/flasher_args.json').write_text(json.dumps({
        'app': {'file': app.name, 'offset': '0x10000'},
        'flash_settings': {'flash_mode': 'dio', 'flash_size': '16MB', 'flash_freq': '80m'}}))
    if mutation == 'app_hash': app.write_bytes(app.read_bytes() + b'changed')
    if mutation == 'source_hash': source_file.write_text('/* SYNTHETIC source changed after build */\n')
    commands = []
    def fake_run(command, **kwargs):
        # This is the sole subprocess boundary. Never delegate to the OS.
        assert isinstance(command, list) and kwargs['timeout'] == 180
        assert kwargs.get('shell', False) is False
        commands.append(list(command))
        if 'flash_id' in command:
            kwargs['stdout'].write('SYNTHETIC esptool identity output; not a real device\nMAC: ' + mac + '\n')
            return SimpleNamespace(returncode=identity_code)
        assert 'write_flash' in command, 'unexpected subprocess command'
        kwargs['stdout'].write('SYNTHETIC app write return only; NO hardware action occurred\n')
        return SimpleNamespace(returncode=0)
    failure = None
    old_cwd = Path.cwd()
    captured = io.StringIO()
    try:
        with mock.patch('subprocess.run', side_effect=fake_run), mock.patch.object(
                sys, 'argv', [str(script), name, '--port', '/dev/SYNTHETIC_NEVER_OPEN', '--before', before]), contextlib.redirect_stdout(captured):
            try: runpy.run_path(str(script), run_name='__main__')
            except (AssertionError, RuntimeError, SystemExit) as error:
                failure = {'class': type(error).__name__, 'message': str(error)}
    finally:
        os.chdir(old_cwd)
    success = mutation is None and identity_code == 0 and mac == '80:b5:4e:cd:d9:ac'
    if success:
        assert failure is None, failure
        assert len(commands) == 2
        first, second = commands
        assert first[-5:] == ['--before', before, '--after', 'no_reset', 'flash_id']
        write = second.index('write_flash')
        assert second[write-4:write] == ['--before', 'no_reset', '--after', 'hard_reset']
        assert second[write+1:] == ['--flash_mode', 'dio', '--flash_freq', '80m', '--flash_size', '16MB', '0x10000', str(app)]
        result = json.loads((out / (name + '-flash-result.json')).read_text())
        assert result['app_sha256'] == manifest['app_sha256']
        assert not result['nvs_or_calibration_written'] and not result['partition_written'] and not result['bootloader_written']
    else:
        assert failure is not None
        assert len(commands) == (0 if mutation else 1)
        assert not any('write_flash' in command for command in commands)
        assert not (out / (name + '-flash-app.txt')).exists()
        assert not (out / (name + '-flash-result.json')).exists()
    for command in commands:
        assert command[command.index('--port') + 1] == '/dev/SYNTHETIC_NEVER_OPEN'
        assert command[command.index('--connect-attempts') + 1] == '2'
        assert command[command.index('--chip') + 1] == 'esp32s3'
        assert command[command.index('--baud') + 1] == '460800'
        assert not set(command) & {'erase_flash', 'erase_region'}
    return {'case': name, 'status': 'PASS_SYNTHETIC', 'copy': str(script),
            'copied_script_sha256': sha(script.read_bytes()), 'commands': commands,
            'observed_exception': failure, 'expected_command_count': 2 if success else 0 if mutation else 1,
            'output_directory': str(out), 'stdout': captured.getvalue()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists(): raise FileExistsError('test evidence already exists')
    original = ROOT / RELATIVE_SCRIPT
    source = original.read_bytes()
    work = Path(tempfile.mkdtemp(prefix='sensorarray-flash-safety-'))
    cases = []
    for before in ('default_reset', 'usb_reset', 'no_reset'):
        cases.append(run_case(work, source, 'synthetic-success-' + before, before=before))
    cases.append(run_case(work, source, 'synthetic-wrong-mac', mac='80:b5:4e:00:00:00'))
    cases.append(run_case(work, source, 'synthetic-identity-failure', identity_code=5))
    cases.append(run_case(work, source, 'synthetic-app-hash', mutation='app_hash'))
    cases.append(run_case(work, source, 'synthetic-source-hash', mutation='source_hash'))
    assert original.read_bytes() == source, 'real script changed during test; result is not current-source validation'
    report = {'schema': 'flash_script_safety_synthetic_v1', 'status': 'PASS', 'cases_passed': len(cases),
              'evidence_type': 'SYNTHETIC_SUBPROCESS_AND_FILES_NOT_A_FLASH', 'hardware_actions': False,
              'real_script_modified': False, 'real_script': str(original), 'real_script_sha256': sha(source),
              'test_source_sha256': sha(Path(__file__).read_bytes()), 'temporary_fixture_root': str(work),
              'cases': cases, 'limits': ['No esptool or USB/serial access was executed.',
                'Mocks verify host command construction and gating, not ROM mode, physical MAC identity or flash success.',
                'App-only assertion covers the one offset/file pair selected from the validated synthetic flasher arguments.']}
    with args.output.open('x') as stream: json.dump(report, stream, indent=2); stream.write('\n')
    print(json.dumps({'status': 'PASS_SYNTHETIC', 'cases': len(cases), 'evidence': str(args.output)}))
    return 0


if __name__ == '__main__': raise SystemExit(main())
