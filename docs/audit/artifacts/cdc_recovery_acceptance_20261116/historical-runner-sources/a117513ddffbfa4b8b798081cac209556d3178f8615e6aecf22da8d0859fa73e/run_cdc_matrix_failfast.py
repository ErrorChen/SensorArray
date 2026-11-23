#!/usr/bin/env python3
"""Run 32 independent 120-second CDC configurations, stopping on first error.

The existing collector owns the port, capture and CAP8 cleanup. This wrapper
does not read or flush USB, retry a failed case, reset, flash or save calibration.
Observers must already be recording. Their early exit stops subsequent cases.
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

ROOT = Path(__file__).resolve().parents[5]
RUNNER = ROOT / 'docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts/capture_cdc_benchmark.py'
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


def validate_case(summary, expected_elf, previous_boot):
    if summary.get('status') != 'CAPTURED' or summary.get('error'):
        return 'collector_error', previous_boot
    if (summary.get('capture_seconds', 0) < 120 or not summary.get('capture_duration_satisfied') or
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


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--label', required=True)
    parser.add_argument('--expected-elf', required=True)
    parser.add_argument('--ble-summary', required=True, type=Path)
    parser.add_argument('--debug-summary', required=True, type=Path)
    parser.add_argument('--ble-ready', required=True, type=Path)
    parser.add_argument('--ble-pid', required=True, type=int)
    parser.add_argument('--debug-pid', required=True, type=int)
    parser.add_argument('--stop-file', required=True, type=Path)
    parser.add_argument('--usb-serial', default='80B54ECDD9AC')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', args.label) or not re.fullmatch(r'[0-9a-f]{64}', args.expected_elf):
        parser.error('invalid label/ELF hash')
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', args.usb_serial):
        parser.error('invalid board serial')
    out = args.output_directory.resolve()
    out.mkdir(parents=True, exist_ok=True)
    # Preflight every name before sending any command, including prior gzip.
    for mode, rows in PLAN:
        if list(out.glob(f'{args.label}-{mode.lower()}{rows}.*')):
            parser.error('case output already exists; choose a new label')
    result = {'status': 'INCOMPLETE', 'expected_elf': args.expected_elf,
              'plan': [{'mode': m, 'rows': r} for m, r in PLAN], 'results': [],
              'automatic_retry': False, 'one_port_session': False,
              'capture_seconds_per_case': 120, 'mixed_profile': 'CVRCVRCV',
              'measurement_acceptance': 'NOT_EVALUATED',
              'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    index_path = out / (args.label + '.failfast.json')
    observers = [args.ble_summary.resolve(), args.debug_summary.resolve()]
    ready = json.loads(args.ble_ready.read_text())
    if set(ready.get('all_channels_subscribed', [])) != {'C', 'D', 'L'}:
        parser.error('BLE subscriptions not ready')
    identities = [(pid, process_identity(pid)) for pid in (args.ble_pid, args.debug_pid)]
    result['observer_processes'] = identities
    with index_path.open('x') as index:
        boot = None
        try:
            for mode, rows in PLAN:
                problem = observer_problem(observers, identities)
                if problem or args.stop_file.exists():
                    result['error'] = problem or 'operator_stop_requested'
                    break
                label = f'{args.label}-{mode.lower()}{rows}'
                command = [sys.executable, str(RUNNER), '--usb-serial', args.usb_serial,
                           '--usb-pid', '0x4042', '--output-directory', str(out), '--label', label,
                           '--mode', mode, '--rows', str(rows), '--profile', 'CVRCVRCV',
                           '--capture-seconds', '120', '--rail-avdd-uv', '3350000',
                           '--rail-avss-uv', '-1795000', '--rail-source-note', RAIL_NOTE]
                item = {'label': label, 'mode': mode, 'rows': rows, 'command': command,
                        'started_utc': datetime.now(timezone.utc).isoformat(),
                        'head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
                        'dirty_diff_sha256': hashlib.sha256(subprocess.check_output(['git', 'diff', '--binary'], cwd=ROOT)).hexdigest()}
                result['results'].append(item)
                item['status'] = 'STARTED'
                try:
                    item.update(run_child(command, out / (label + '.command.txt'), observers, args.stop_file,
                                          observer_identities=identities))
                    if item['exit_code'] != 0 or item['stop_reason']:
                        item['error'] = item['stop_reason'] or 'child_nonzero_exit'
                    else:
                        summary_path = out / (label + '.summary.json')
                        summary = json.loads(summary_path.read_text())
                        item['summary_sha256'] = hashlib.sha256(summary_path.read_bytes()).hexdigest()
                        item['error'], boot = validate_case(summary, args.expected_elf, boot)
                except (Exception, KeyboardInterrupt) as error:
                    item['error'] = f'{type(error).__name__}:{error}'
                item['status'] = 'ERROR' if item['error'] else 'CAPTURED'
                with (out / (label + '.command.json')).open('x') as record:
                    json.dump(item, record, indent=2)
                print(json.dumps({'completed_cases': len(result['results']), **item}), flush=True)
                if item['error']:
                    result['error'] = item['error']
                    break
            if len(result['results']) == 32 and not result.get('error'):
                result['status'] = 'CAPTURED_32_NOT_YET_ANALYZED'
        except (Exception, KeyboardInterrupt) as error:
            result['error'] = f'{type(error).__name__}:{error}'
        finally:
            result['not_started'] = result['plan'][len(result['results']):]
            json.dump(result, index, indent=2)
            index.write('\n')
    return 0 if result['status'] == 'CAPTURED_32_NOT_YET_ANALYZED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
