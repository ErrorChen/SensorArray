#!/usr/bin/env python3
"""Run independent CDC configuration windows, stopping on the first error.

Host windows default to 120 seconds. With --fixture-margin, CAP8 and RES8
request 122 seconds. Fixture timing still uses the original device timestamps
of the first and last accepted frames.

The existing collector owns the port, capture and CAP8 cleanup. This wrapper
does not read or flush USB, retry a failed case, reset, flash or save calibration.
Each case owns separate BLE/debug observers. Close and join both before the
existing real parser qualifies that case; FAIL or UNKNOWN stops the suite.
Each child has a bounded interrupt/kill fence; a killed child is never called
safe. Raw evidence is exclusive-create and retained even on failure.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from qualify_closed_capture import qualify_closed_case, ExpectedCaptureIdentity

ROOT = Path(__file__).resolve().parents[5]
RUNNER = ROOT / 'docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts/capture_cdc_benchmark.py'
BLE_RUNNER = ROOT / 'tools/capture_benchmark_ble.py'
DEBUG_RUNNER = RUNNER.with_name('observe_cdc_debug.py')
PLAN = tuple((mode, rows) for mode in ('CAP', 'VOLT', 'RES', 'MIXED') for rows in range(1, 9))
RAIL_NOTE = ('User DMM at ADS supply pins: AVDD 3.33-3.37 V, AVSS -1.795 +/-0.005 V; '
             'nominal RAM-only calibration, estimated noise under 10 mV, not precision acceptance')


def process_identity(pid):
    """Linux owner-process identity; an empty summary is not liveness proof."""
    text = Path(f'/proc/{pid}/stat').read_text()
    fields = text[text.rfind(')') + 2:].split()
    if fields[0] == 'Z':
        raise ProcessLookupError(pid)
    os.kill(pid, 0)
    return fields[19]


def observer_problem(paths, identities=()):
    for path in paths:
        if not path.exists():
            return 'observer_not_started:' + str(path)
        if path.stat().st_size:
            return 'observer_already_closed:' + str(path)
    for pid, identity in identities:
        try:
            if process_identity(pid) != identity:
                return 'observer_process_replaced:' + str(pid)
        except (OSError, ValueError, IndexError):
            return 'observer_process_not_alive:' + str(pid)
    return None


def validate_case(summary, expected_elf, previous_boot, requested_seconds=120):
    if summary.get('status') != 'CAPTURED' or summary.get('error'):
        return 'collector_error', previous_boot
    if (summary.get('capture_requested_seconds', 120) != requested_seconds or
            summary.get('capture_seconds', 0) < requested_seconds or not summary.get('capture_duration_satisfied') or
            summary.get('safe_cap_recovery') != 'CAP8_QUERY_VERIFIED'):
        return 'duration_or_cleanup_unverified', previous_boot
    if summary.get('replies', {}).get('BUILD?', {}).get('elfSha256') != expected_elf:
        return 'running_image_changed_or_unknown', previous_boot
    fields = summary.get('snapshots', {}).get('before', {}).get('CONFIG?', {}).get('fields', {})
    boot = (fields.get('session'), fields.get('bootId'))
    if (not re.fullmatch(r'[0-9A-Fa-f]{16}', boot[0] or '') or
            not re.fullmatch(r'[0-9]+', boot[1] or '') or int(boot[1]) >= 2**32 or
            (previous_boot is not None and boot != previous_boot)):
        return 'boot_changed_or_unknown', previous_boot
    return None, boot


def run_child(command, log_path, observer_paths, stop_path, timeout=190, grace=12,
              observer_identities=(), reap_timeout=2):
    started = time.monotonic()
    reason = None
    code, child = None, None
    with log_path.open('x') as log:
        child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        try:
            while child.poll() is None:
                reason = (observer_problem(observer_paths, observer_identities) or
                          ('operator_stop_requested' if stop_path.exists() else None) or
                          ('child_deadline' if time.monotonic() - started >= timeout else None))
                if reason:
                    break
                time.sleep(0.25)
        except KeyboardInterrupt:
            reason = 'operator_interrupt'
        except Exception as error:
            reason = f'monitor_error:{type(error).__name__}:{error}'
        finally:
            if child.poll() is None:
                # The existing collector catches SIGINT and runs its cleanup.
                try:
                    child.send_signal(signal.SIGINT)
                except (Exception, KeyboardInterrupt) as error:
                    reason = (reason or 'cleanup_signal_error') + ';' + type(error).__name__
                try:
                    child.wait(timeout=grace)
                except (Exception, KeyboardInterrupt) as error:
                    reason = (reason or 'cleanup_wait_error') + ';' + type(error).__name__
                if child.poll() is None:
                    reason = (reason or 'child_unresponsive') + ';forced_kill_cleanup_unknown'
                    try:
                        child.kill()
                    except (Exception, KeyboardInterrupt) as error:
                        reason += ';kill_error:' + type(error).__name__
                    try:
                        child.wait(timeout=reap_timeout)
                    except (Exception, KeyboardInterrupt) as error:
                        reason += ';reap_error:' + type(error).__name__
            code = child.poll()
            if code is None:
                reason = (reason or 'child_unresponsive') + ';child_termination_unknown'
    return {'exit_code': code, 'stop_reason': reason,
            'child_pid': child.pid, 'child_termination_known': code is not None,
            'elapsed_seconds': time.monotonic() - started,
            'log_sha256': hashlib.sha256(log_path.read_bytes()).hexdigest()}


def close_observers(owners, stop_path):
    """Request closure, then bound every process join; never assume killed safe."""
    stop_error = None
    try:
        stop_path.touch(exist_ok=True)
    except OSError as error:
        stop_error = 'stop_marker:' + str(error)
    records = []
    for item in owners:
        child, errors = item['process'], [stop_error] if stop_error else []
        try:
            if stop_error:
                child.send_signal(signal.SIGINT)
            child.wait(timeout=12)
        except (Exception, KeyboardInterrupt) as error:
            errors.append('grace:' + type(error).__name__)
            try:
                child.send_signal(signal.SIGINT)
                child.wait(timeout=2)
            except (Exception, KeyboardInterrupt) as error:
                errors.append('interrupt:' + type(error).__name__)
        if child.poll() is None:
            try:
                child.kill()
                child.wait(timeout=2)
            except (Exception, KeyboardInterrupt) as error:
                errors.append('kill_or_reap:' + type(error).__name__)
        try:
            item['log'].close()
        except OSError as error:
            errors.append('log_close:' + str(error))
        try:
            log_hash = hashlib.sha256(item['log_path'].read_bytes()).hexdigest()
        except OSError as error:
            log_hash = None
            errors.append('log_hash:' + str(error))
        records.append({'kind': item['kind'], 'pid': child.pid, 'exit_code': child.poll(),
                        'termination_known': child.poll() is not None, 'cleanup_errors': errors,
                        'log_sha256': log_hash,
                        'command': item['command']})
    return records


def start_observers(directory, label, args):
    """Start per-case read-only observers; readiness excludes capture time."""
    stop = directory / (label + '.observers.stop')
    owners = []
    try:
        for kind, command in (
            ('ble', [sys.executable, str(BLE_RUNNER), '--address', args.ble_address,
                     '--adapter', args.ble_adapter, '--output-directory', str(directory),
                     '--label', label, '--duration', '300', '--stop-file', str(stop)]),
            ('debug', [sys.executable, str(DEBUG_RUNNER), '--usb-serial', args.usb_serial,
                       '--usb-pid', '0x4042', '--output-directory', str(directory),
                       '--label', label, '--duration', '300', '--enumerate-seconds', '30',
                       '--stop-file', str(stop)])):
            log_path = directory / (label + '.' + kind + '.command.txt')
            log = log_path.open('x')
            try:
                child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            except BaseException:
                log.close()
                raise
            owners.append({'kind': kind, 'process': child, 'log': log, 'log_path': log_path,
                           'command': command})
            owners[-1]['identity'] = process_identity(child.pid)
        deadline = time.monotonic() + 45
        ready = directory / (label + '.ble.ready.json')
        debug_events = directory / (label + '.debug.events.jsonl')
        paths = [directory / (label + '.' + kind + '.summary.json') for kind in ('ble', 'debug')]
        while time.monotonic() < deadline:
            if args.stop_file.exists():
                raise RuntimeError('operator_stop_requested')
            if any(item['process'].poll() is not None for item in owners):
                raise RuntimeError('observer_exited_before_ready')
            if any(path.exists() and path.stat().st_size for path in paths):
                raise RuntimeError('observer_closed_before_ready')
            subscribed = False
            if ready.exists():
                try:
                    record = json.loads(ready.read_text())
                    subscribed = (set(record.get('all_channels_subscribed', [])) == {'C', 'D', 'L'} and
                                  record.get('address', '').upper() == args.ble_address.upper())
                except (ValueError, OSError):
                    pass  # File may still be in its single writer's bounded write.
            opened = False
            if debug_events.exists():
                for line in debug_events.read_bytes().splitlines(keepends=True):
                    if line.endswith(b'\n'):
                        event = json.loads(line)
                        if event.get('type') == 'opened':
                            opened = True
                        if event.get('type') in {'firmware_fault', 'reset_observed', 'finished'}:
                            raise RuntimeError('debug_problem_before_ready')
            if subscribed and opened and all(path.exists() for path in paths):
                return owners, stop, paths
            time.sleep(.05)
        raise RuntimeError('observer_readiness_deadline')
    except BaseException:
        closure = close_observers(owners, stop)
        with (directory / (label + '.observer-start-failure.json')).open('x') as stream:
            json.dump(closure, stream, indent=2)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--label', required=True)
    parser.add_argument('--expected-elf', required=True)
    parser.add_argument('--expected-session', required=True)
    parser.add_argument('--expected-boot-id', required=True, type=int)
    parser.add_argument('--ble-address', required=True)
    parser.add_argument('--ble-adapter', default='hci0')
    parser.add_argument('--stop-file', required=True, type=Path)
    parser.add_argument('--usb-serial', default='80B54ECDD9AC')
    parser.add_argument('--start-case', choices=[f'{mode.lower()}{rows}' for mode, rows in PLAN],
                        default='cap1', help='Start a fresh recording at this configuration. Earlier cases require their own evidence.')
    parser.add_argument('--end-case', choices=[f'{mode.lower()}{rows}' for mode, rows in PLAN],
                        default='mixed8', help='Last configuration to record, inclusive.')
    parser.add_argument('--pause-formal-reader', action='store_true',
                        help='Inject one 650 ms application read pause in an isolated VOLT2 screening run.')
    parser.add_argument('--fixture-margin', action='store_true',
                        help='Request 122 host seconds for CAP8 and RES8. All other cases request 120 seconds.')
    args = parser.parse_args(argv)
    try:
        expected = ExpectedCaptureIdentity(args.expected_elf, args.expected_session, args.expected_boot_id)
    except ValueError as error:
        parser.error(str(error))
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.label):
        parser.error('invalid label')
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', args.usb_serial) or not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.ble_address):
        parser.error('explicit USB serial and BLE address required')
    out = args.output_directory.resolve()
    out.mkdir(parents=True, exist_ok=True)
    # Check historical flat paths and new per-case directories before any port
    # opens. An existing attempt is never retried or overwritten.
    names = [f'{m.lower()}{r}' for m, r in PLAN]
    first, last = names.index(args.start_case), names.index(args.end_case)
    if last < first:
        parser.error('end-case precedes start-case')
    selected = PLAN[first:last + 1]
    if args.pause_formal_reader and selected != (('VOLT', 2),):
        parser.error('the application read pause requires an isolated volt2 case')
    for mode, rows in selected:
        stem = f'{args.label}-{mode.lower()}{rows}'
        if (out / stem).exists() or list(out.glob(stem + '.*')):
            parser.error('case output already exists; choose a new label')
    if args.stop_file.exists():
        parser.error('stop file already exists')
    plan = [{'mode': mode, 'rows': rows, 'capture_requested_seconds':
             122 if args.fixture_margin and (mode, rows) in {('CAP', 8), ('RES', 8)} else 120}
            for mode, rows in selected]
    durations = {case['capture_requested_seconds'] for case in plan}
    result = {'status': 'INCOMPLETE', 'expected': expected.output(),
              'plan': plan, 'results': [],
              'excluded_from_this_run': [{'mode': m, 'rows': r} for m, r in PLAN[:first] + PLAN[last + 1:]],
              'formal_reader_pause_requested': args.pause_formal_reader,
              'automatic_retry': False, 'one_port_session': False,
              'fixture_margin': args.fixture_margin,
              'capture_seconds_per_case': plan[0]['capture_requested_seconds'] if len(durations) == 1 else None,
              'mixed_profile': 'CVRCVRCV',
              'per_case_observers': 'independently_started_closed_joined_before_analysis',
              'measurement_acceptance': 'NOT_ESTABLISHED',
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    index_path = out / (args.label + '.failfast.json')
    with index_path.open('x') as index:
        try:
            for case in plan:
                mode, rows = case['mode'], case['rows']
                seconds = case['capture_requested_seconds']
                if args.stop_file.exists():
                    result['error'] = 'operator_stop_requested'
                    break
                label = f'{args.label}-{mode.lower()}{rows}'
                directory = out / label
                directory.mkdir()
                capture = Path(__file__).with_name('capture_cdc_short_pause.py') if args.pause_formal_reader else RUNNER
                command = [sys.executable, str(capture), '--usb-serial', args.usb_serial,
                           '--usb-pid', '0x4042', '--output-directory', str(directory), '--label', label,
                           '--mode', mode, '--rows', str(rows), '--profile', 'CVRCVRCV',
                           '--capture-seconds', str(seconds), '--rail-avdd-uv', '3350000',
                           '--rail-avss-uv', '-1795000', '--rail-source-note', RAIL_NOTE,
                           '--expected-elf', expected.elf_sha256, '--expected-session', expected.session,
                           '--expected-boot-id', str(expected.boot_id)]
                item = {'label': label, **case, 'command': command,
                        'started_utc': datetime.now(timezone.utc).isoformat(),
                        'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                        'dirty_diff_sha256': hashlib.sha256(subprocess.check_output(['git', 'diff', '--binary'], cwd=ROOT)).hexdigest(),
                        'status': 'STARTED', 'error': None}
                result['results'].append(item)
                owners, stop, paths = [], None, []
                try:
                    owners, stop, paths = start_observers(directory, label, args)
                    identities = [(o['process'].pid, o['identity']) for o in owners]
                    item.update(run_child(command, directory / (label + '.command.txt'), paths, args.stop_file,
                                          observer_identities=identities, timeout=seconds + 70))
                    if item['exit_code'] != 0 or item['stop_reason']:
                        item['error'] = item['stop_reason'] or 'child_nonzero_exit'
                except (Exception, KeyboardInterrupt) as error:
                    item['error'] = f'{type(error).__name__}:{error}'
                finally:
                    if stop is not None:
                        try:
                            item['observer_closure'] = close_observers(owners, stop)
                            if any(r['exit_code'] != 0 or r['cleanup_errors'] or not r['termination_known']
                                   for r in item['observer_closure']):
                                item['error'] = item['error'] or 'observer_closure_failed_or_unknown'
                        except (Exception, KeyboardInterrupt) as error:
                            item['observer_closure_error'] = str(error)
                            item['error'] = item['error'] or 'observer_closure_unknown'
                if not item['error']:
                    try:
                        summary_path = directory / (label + '.summary.json')
                        summary = json.loads(summary_path.read_text())
                        if args.pause_formal_reader and not summary.get('formal_reader_pause_verified'):
                            raise ValueError('application_read_pause_missing_or_outside_window')
                        item['summary_sha256'] = hashlib.sha256(summary_path.read_bytes()).hexdigest()
                        error, _ = validate_case(summary, expected.elf_sha256, (expected.session, str(expected.boot_id)), seconds)
                        if error:
                            raise ValueError(error)
                        gate = qualify_closed_case(summary_path, paths[0], paths[1], expected, minimum_seconds=seconds)
                        with (directory / (label + '.gate.json')).open('x') as record:
                            json.dump(gate, record, indent=2)
                        item['gate_status'] = gate['status']
                        item['science'] = gate.get('science')
                        if gate['status'] != 'PASS_CASE_SCREEN':
                            item['error'] = 'closed_case_gate_' + gate['status']
                    except (Exception, KeyboardInterrupt) as error:
                        item['error'] = f'closed_case_gate_unknown:{type(error).__name__}:{error}'
                item['status'] = 'ERROR' if item['error'] else 'PASS_CASE_SCREEN'
                with (directory / (label + '.command.json')).open('x') as record:
                    json.dump(item, record, indent=2)
                print(json.dumps({'completed_cases': len(result['results']), **item}), flush=True)
                if item['error']:
                    result['error'] = item['error']
                    break
            if len(result['results']) == len(selected) and not result.get('error'):
                result['status'] = 'SCREENED_SELECTED_CASES' if selected != PLAN else 'SCREENED_32_NOT_FINAL_ACCEPTANCE'
        except (Exception, KeyboardInterrupt) as error:
            result['error'] = f'{type(error).__name__}:{error}'
        finally:
            result['not_started'] = result['plan'][len(result['results']):]
            json.dump(result, index, indent=2)
            index.write('\n')
    return 0 if result['status'] in {'SCREENED_32_NOT_FINAL_ACCEPTANCE', 'SCREENED_SELECTED_CASES'} else 1


if __name__ == '__main__':
    raise SystemExit(main())
