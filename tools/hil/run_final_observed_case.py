#!/usr/bin/env python3
"""Run one reviewed final600, final50 or short_v2 command with two independent observers."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

import qualify_closed_capture as gate
from fixture_rail import add_rail_arguments, rail_arguments
from run_cdc_matrix_failfast import BLE_RUNNER, DEBUG_RUNNER, ROOT, RUNNER, close_observers, process_identity, run_child
from capture_mode_switches import PLAN, require_config
from sensorarray_capture_identity import IDENTITY_FIELDS, readonly_command, verify_identity_evidence
from text_protocol import parse_fields

# The collectors own their shorter phase/cleanup limits. These ceilings allow
# process startup and reserve observer time for readiness and orderly closure.
DEADLINE = {'final600': 690, 'final50': 315, 'short_v2': 205}


def save(path, record):
    with path.open('x') as stream:
        json.dump(record, stream, indent=2)
        stream.write('\n')


def plan_case(args, expected):
    case, label = args.case, args.label
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', args.usb_serial):
        raise ValueError('USB serial must contain twelve hexadecimal digits')
    if not re.fullmatch(r'(?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}', args.ble_address):
        raise ValueError('BLE address must contain six hexadecimal octets')
    serial = args.usb_serial.upper()
    directory = args.output_directory.resolve() / label
    stop_file = str(directory / (label + '.observers.stop'))
    common = ['--output-directory', str(directory), '--label', label]
    usb = ['--usb-serial', serial]
    pid = ['--usb-pid', '0x4042']
    identity = ['--expected-elf', expected.elf_sha256, '--expected-session', expected.session,
                '--expected-boot-id', str(expected.boot_id)]
    if case == 'final600':
        collector = RUNNER
        options = [*pid, '--mode', 'VOLT', '--rows', '1', '--stability']
        suffix, observer_seconds = '-stability-volt1.summary.json', 780
    elif case == 'final50':
        collector = RUNNER.with_name('capture_mode_switches.py')
        options = ['--total-seconds', '300']
        suffix, observer_seconds = '.summary.json', 450
    else:
        collector = RUNNER.with_name('capture_short_snapshots.py')
        options = [*pid, '--mode', 'CAP', '--rows', '8', '--capture-seconds', '120']
        suffix, observer_seconds = '.summary.json', 300
    observe = [*common, '--duration', str(observer_seconds), '--stop-file', stop_file]
    return {
        'case': case, 'label': label, 'directory': str(directory), 'expected': expected.output(),
        'formal': [sys.executable, str(collector), *usb, *common, *options, *rail_arguments(args), *identity],
        'BLE_observer': [sys.executable, str(BLE_RUNNER), '--address', args.ble_address,
                         '--adapter', args.ble_adapter, *observe],
        'debug_observer': [sys.executable, str(DEBUG_RUNNER), *usb, *pid, '--enumerate-seconds', '30', *observe],
        'formal_summary': str(directory / (label + suffix)),
        'BLE_summary': str(directory / (label + '.ble.summary.json')),
        'debug_summary': str(directory / (label + '.debug.summary.json')),
        'stop_file': stop_file, 'operator_stop': str(directory / (label + '.stop')),
        'formal_timeout_seconds': DEADLINE[case], 'generator': gate.reference(Path(__file__)),
    }


def launch_observer(kind, command, directory, label):
    log_path = directory / f'{label}.{kind}.command.txt'
    log = log_path.open('x')
    try:
        child = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    except BaseException:
        log.close()
        raise
    return {'kind': kind, 'process': child, 'log': log, 'log_path': log_path, 'command': command}


def await_ready(plan, owners, summaries, address, serial):
    directory, label = Path(plan['directory']), plan['label']
    ready_path = directory / (label + '.ble.ready.json')
    events_path = directory / (label + '.debug.events.jsonl')
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        gate.require(not Path(plan['operator_stop']).exists(), 'operator_stop_requested')
        gate.require(not Path(plan['stop_file']).exists(), 'observers_stopped_before_formal')
        gate.require(all(o['process'].poll() is None for o in owners), 'observer_exited_before_ready')
        gate.require(not any(p.exists() and p.stat().st_size for p in summaries), 'observer_closed_before_ready')
        opened = False
        if events_path.exists():
            for line in events_path.read_bytes().splitlines(keepends=True):
                if not line.endswith(b'\n'):
                    continue
                event = json.loads(line)
                gate.require(event['type'] not in {'firmware_fault', 'reset_observed', 'finished'}, 'debug_problem_before_ready')
                if event['type'] == 'opened':
                    meta = event['metadata']
                    gate.require((meta['usb_serial'], meta['interface_number']) == (serial, 2), 'debug_role_or_serial_mismatch')
                    opened = True
        if opened and all(p.exists() for p in summaries) and ready_path.exists():
            try:
                ready = gate.load(ready_path)
            except json.JSONDecodeError:
                pass  # The single writer may be between create and close.
            else:
                gate.require(ready['address'].upper() == address.upper() and
                             set(ready['all_channels_subscribed']) == {'C', 'D', 'L'}, 'BLE_readiness_mismatch')
                return ready
        time.sleep(.1)
    raise TimeoutError('observer_readiness_deadline')


def verify_journal(journal, start, end):
    gate.require(journal.get('schema_valid') and journal.get('recording_end_observed') and
                 journal['collector_summary_verification']['status'] == 'MATCH', 'closed_BLE_journal_unverified')
    gate.require(any(lo <= round(start * 1e9) and hi >= round(end * 1e9)
                     for lo, hi in journal['ready_intervals_monotonic_ns']), 'BLE_subscription_coverage_incomplete')
    gate.require(not any(journal['whole_journal_critical_records'].values()), 'BLE_fault_or_reset')
    for channel in ('C', 'L'):
        counters = journal['all_fragment_counters_by_channel'].get(channel, {})
        gate.require(not any(counters.get(k, 0) for k in ('crc_fail', 'dropped', 'missing')), 'BLE_critical_fragment_error')


class SwitchReceipts:
    """Collect independently parsed BLE receipts for the actual USB configuration identities."""
    def __init__(self, steps):
        self.by_identity = {}
        self.records = []
        for step in steps:
            config = step['closing_CONFIG']
            requested = {'mode': config['mode'], 'rows': int(config['rows']), 'profile': config['profile']}
            record = {'step': step['index'], 'mode': config['mode'], 'rev': config['rev'],
                      'first_USB_sequence': int(step['first_frame']['header']['seq']), 'BLE_frames': 0,
                      'first_BLE_sequence': None, 'first_USB_sequence_seen_on_BLE': False}
            self.records.append(record)
            self.by_identity[tuple(config[k] for k in IDENTITY_FIELDS)] = (
                record, gate.benchmark.expected_wire_contract(requested, config))

    def compare(self, _label, header, frame, _digest):
        target = self.by_identity.get(tuple(header.get(k) for k in IDENTITY_FIELDS))
        if target:
            record, contract = target
            gate.require(not gate.benchmark.frame_wire_contract_errors(frame, header, contract), 'BLE_transition_wire_contract')
            if record['first_BLE_sequence'] is None:
                record['first_BLE_sequence'] = frame.sequence
            record['BLE_frames'] += 1
            record['first_USB_sequence_seen_on_BLE'] |= frame.sequence == record['first_USB_sequence']

    def result(self, _label):
        return {}  # Multi-configuration run has no fixed-configuration loss denominator.


def qualify_switches(summary_path, summary, journal_path, expected, start, end):
    gate.require(summary['status'] == 'COMPLETE_50_IDENTITY_VERIFIED' and
                 summary['verified_transitions'] == 50 and len(summary['steps']) == 50 and
                 not any(summary.get(k) for k in ('error', 'cleanup_error', 'close_error', 'fault_events', 'reset_observations')) and
                 summary['safe_cap_recovery'] == 'CAP8_QUERY_VERIFIED', '50_USB_transitions_or_cleanup_unverified')
    raw = gate.read_bytes(gate.resolve(summary['raw_file'], summary_path))
    events = gate.events_file(gate.resolve(summary['events_file'], summary_path))
    gate.require(len(raw) == summary['rx_bytes'] and hashlib.sha256(raw).hexdigest() == summary['raw_sha256'], 'formal_raw_mismatch')
    gate.reject_fault_reset_text(raw)
    replies = gate.verify_query_trace(events, raw)
    verified, brackets = [], summary['image_authority']
    begin, purpose = None, None
    for index, event in enumerate(events):
        if event['type'] == 'image_authority_begin':
            begin, purpose = index, event['purpose']
            gate.require(event['expected'] == expected.output(), 'wrong_bracket_expectation')
        elif event['type'] == 'image_authority_verified':
            bracket = [r for r in replies if begin < r['event_index'] < index]
            gate.require([r['command'] for r in bracket] == ['CONFIG?', 'BUILD?', 'CONFIG?'], 'missing_image_bracket')
            evidence = verify_identity_evidence(expected, bracket[1]['fields'], bracket[0]['fields'], bracket[2]['fields'],
                                                require_running=purpose == 'prepare')
            gate.require(event['evidence'] == evidence and event['purpose'] == purpose, 'image_bracket_mismatch')
            verified.append({'purpose': purpose, **evidence})
            begin = None
        elif event['type'] == 'tx_begin' and not readonly_command(event['command']):
            gate.require(verified and begin is None and (event['phase'] != 'cleanup' or purpose == 'cleanup'),
                         'setter_outside_image_authority')
    gate.require([v['purpose'] for v in verified] == ['prepare', 'cleanup'] and verified == brackets['brackets'] and
                 brackets['required'] and brackets['expected'] == expected.output(), 'incomplete_expected_image_brackets')
    configs = [r for r in replies if r['command'] == 'CONFIG?']
    gate.require(all((r['fields']['session'], r['fields']['bootId']) == (expected.session, str(expected.boot_id))
                     for r in configs), 'CONFIG_boot_changed')
    gate.require(configs[-1]['phase'] == 'cleanup' and configs[-1]['fields'] == summary['cleanup_CONFIG'], 'missing_actual_CAP8_cleanup')
    require_config(configs[-1]['fields'], 'CAP')
    applied = [e for e in events if e['type'] == 'mode_switch_verified']
    first = [e for e in events if e['type'] == 'first_matching_frame']
    gate.require(len(applied) == len(first) == 50, 'first_frame_or_step_events_missing')
    for index, (step, mode, event, frame) in enumerate(zip(summary['steps'], PLAN, applied, first), 1):
        config = step['closing_CONFIG']
        require_config(config, mode)
        gate.require((event['index'], event['target'], step['status']) == (index, mode, 'VERIFIED') and
                     (step['index'], step['target']) == (index, mode) and step['first_frame']['crc_ok'] and
                     all(frame[k] == v for k, v in step['first_frame'].items()) and
                     all(frame['header'][k] == config[k] for k in IDENTITY_FIELDS) and
                     any(r['fields'] == config for r in configs), 'verified_USB_step_mismatch')
    receipts = SwitchReceipts(summary['steps'])
    window = {'label': summary['label'], 'window': {'capture_monotonic_start': start, 'capture_monotonic_end': end}}
    journal = gate.benchmark.analyse_ble_journal(journal_path, [window], receipts)
    verify_journal(journal, start, end)
    groups = window['ble']['configuration_groups']
    gate.require(groups and all((g['identity'].get('session'), g['identity'].get('bootId')) ==
                                (expected.session, str(expected.boot_id)) for g in groups), 'BLE_boot_unobserved_or_changed')
    seen = sum(bool(row['BLE_frames']) for row in receipts.records)
    gate.require(seen > 0, 'BLE_transition_frames_unobserved')
    return {'status': 'COMPLETE_50_USB_VERIFIED_OBSERVERS_CLOSED', 'verified_transitions': 50, 'cleanup_CONFIG': configs[-1]['fields'],
            'image_brackets': verified, 'BLE_journal': journal, 'BLE_transitions_with_matching_frames': seen,
            'BLE_transition_receipts': receipts.records, 'normal_FULL_loss_budget_applicable': False,
            'BLE_capture_frame_counts': window['ble']['capture_frame_counts'],
            'BLE_fragment_integrity': window['ble']['fragment_integrity'],
            'BLE_contained_data_diagnostics': window['ble']['fully_contained_data_diagnostics']}


def qualify_short(summary_path, journal_path, debug_path, expected, start, end):
    summary, authority = gate.verify_formal(summary_path, expected, 120)
    gate.require(summary['runner_variant'] == 'formal_cdc_ble_short_snapshot_diagnostic_v2' and
                 (summary['benchmark_mode'], summary['rows_requested'], summary['capture_requested_seconds']) == ('CAP', 8, 120) and
                 summary['normal_FULL_loss_budget_applicable'] is False, 'SHORT_v2_profile_or_window_missing')
    events = gate.events_file(gate.resolve(summary['events_file'], summary_path))
    base = [i for i, e in enumerate(events) if e['type'] == 'base_finished']
    profile = [i for i, e in enumerate(events) if e['type'] == 'short_snapshot_profile']
    gate.require(len(base) == len(profile) == 1 and profile[0] < base[0] < len(events) - 1, 'SHORT_closure_order')
    original = summary['original_tx']
    initial = [e for e in events[:profile[0]] if e['type'] == 'reply' and e['command'] == 'TX?']
    gate.require(original in ('short', 'rel', 'full') and parse_fields(initial[0]['line']) == {'cmd': 'TX', 'v': original} and
                 parse_fields(initial[-1]['line']) == {'cmd': 'TX', 'v': 'short'}, 'original_or_applied_TX_unknown')
    restore = [e for e in events[base[0]+1:] if e['type'] == 'reply']
    commands = ['TX=' + original.upper(), 'TX?'] if original != 'short' else []
    gate.require([e['command'] for e in restore] == commands and
                 all(parse_fields(e['line']) == {'cmd': 'TX', 'v': original} for e in restore) and
                 summary['tx_change_attempted'] == bool(commands) and events[-1].get('tx_restore') == summary['tx_restore'] and
                 summary['tx_restore'] == ('QUERY_VERIFIED' if commands else 'NOT_NEEDED'), 'TX_restore_unverified')
    report = gate.benchmark.analyse_directory(summary_path.parent, journal_path, [debug_path])
    verify_journal(report['ble_journal'], start, end)
    capture, = report['captures']
    ble = capture['ble']
    gate.require(capture['status'] == ble['status'] == 'RECORDED' and ble['observed_identity_subset_of_USB'] and
                 len(ble['configuration_groups']) == len(capture['configuration_groups']) == 1, 'SHORT_capture_or_identity_incomplete')
    group = ble['configuration_groups'][0]
    for counts in (ble['capture_frame_counts'], capture['capture_counts']):
        gate.require(not any(counts.get(k) for k in gate.INTEGRITY_ERRORS), 'SHORT_frame_integrity_error')
    gate.require(not capture['capture_counts'].get('missing_frames') and not any(ble['fully_contained_data_diagnostics'].values()) and
                 not any(ble['fragment_integrity']['fully_contained_by_channel'].get('D', {}).get(k)
                         for k in ('crc_fail', 'dropped', 'missing')), 'SHORT_USB_gap_or_BLE_integrity_error')
    matched = ble['USB_comparison']
    gate.require(group['identity']['session'] == expected.session and group['identity']['bootId'] == str(expected.boot_id) and
                 matched['same_wire_bytes_sha256'] >= 2 and not matched['different_wire_bytes_sha256'] and
                 group['host_complete_frame_intervals']['known'], 'independent_matching_full_snapshots_missing')
    protection = gate.protection.analyze_file(summary_path)
    gate.require(all(q['status'] == 'OBSERVED_LOCAL_COUNTERS' for q in protection['queries'].values()) and
                 protection['queries']['SNAPSRC?']['counter_deltas']['periodicQueued']['delta'] > 0, 'periodic_snapshot_telemetry_unknown')
    return {'status': 'SHORT_V2_FULL_SNAPSHOTS_OBSERVED', 'original_tx': original, 'tx_restore': summary['tx_restore'],
            'normal_FULL_loss_budget_applicable': False, 'matching_full_snapshots': matched['same_wire_bytes_sha256'],
            'BLE_intervals': group['host_complete_frame_intervals'], 'formal_authority': authority,
            'analysis': report, 'protection': protection}


def qualify(plan, expected, start, end):
    formal, ble_path, debug_path = (Path(plan[k]) for k in ('formal_summary', 'BLE_summary', 'debug_summary'))
    summary, ble, debug = map(gate.load, (formal, ble_path, debug_path))
    serial = plan['formal'][plan['formal'].index('--usb-serial') + 1]
    address = plan['BLE_observer'][plan['BLE_observer'].index('--address') + 1]
    gate.require(summary['transport_metadata']['usb_serial'] == serial and
                 summary['transport_metadata']['interface_number'] == 0, 'wrong_formal_board_or_interface')
    gate.require(debug['status'] == 'STOPPED' and ble['status'] == 'RECORDED' and ble['stop_reason'] == 'stop_file' and
                 ble['selected_device']['address'].upper() == address.upper() and set(ble['subscribed_channels']) == {'C', 'D', 'L'} and
                 not ble['unexpected_disconnect'] and not ble['error'] and not ble['cleanup_errors'], 'observer_closure_unverified')
    debug_evidence = gate.verify_debug(debug_path, start, end, serial)
    journal = gate.resolve(ble['journal_file'], ble_path)
    if plan['case'] == 'final600':
        gate.require((summary['benchmark_kind'], summary['benchmark_mode'], summary['rows_requested'],
                      summary['capture_requested_seconds']) == ('stability', 'VOLT', 1, 600), 'full_600s_VOLT1_required')
        result = gate.qualify_closed_case(formal, ble_path, debug_path, expected, minimum_seconds=600)
        verify_journal(result['analysis']['ble_journal'], start, end)
    elif plan['case'] == 'final50':
        gate.require(summary['evidence'] == 'LIVE_HIL_ATTEMPT', 'physical_transition_capture_required')
        result = qualify_switches(formal, summary, journal, expected, start, end)
    else:
        result = qualify_short(formal, journal, debug_path, expected, start, end)
    result.update(expected=expected.output(), debug=debug_evidence, BLE_summary=gate.reference(ble_path),
                  formal_summary=gate.reference(formal), overall_acceptance='NOT_ESTABLISHED')
    return result


def run(plan, expected):
    directory, label = Path(plan['directory']), plan['label']
    directory.parent.mkdir(parents=True, exist_ok=True)
    directory.mkdir()  # Atomic reservation. A partial attempt also owns its label.
    save(directory / (label + '.plan.json'), plan)
    stop, operator = Path(plan['stop_file']), Path(plan['operator_stop'])
    summaries = [Path(plan[k]) for k in ('BLE_summary', 'debug_summary')]
    record = {'case': plan['case'], 'expected': expected.output(), 'status': 'UNKNOWN', 'errors': []}
    owners = []
    try:
        print('Starting independent BLE and debug observers', flush=True)
        for kind, key in [('ble', 'BLE_observer'), ('debug', 'debug_observer')]:
            owners.append(launch_observer(kind, plan[key], directory, label))
        address = plan['BLE_observer'][plan['BLE_observer'].index('--address') + 1]
        serial = plan['debug_observer'][plan['debug_observer'].index('--usb-serial') + 1]
        record['ready'] = await_ready(plan, owners, summaries, address, serial)
        identities = [(o['process'].pid, process_identity(o['process'].pid)) for o in owners]
        print(f"Observers ready. Running {label}. Formal output: {directory}/{label}.formal.command.txt", flush=True)
        record['formal_start'] = time.monotonic()
        record['formal'] = run_child(plan['formal'], directory / (label + '.formal.command.txt'), summaries, operator,
                                     timeout=plan['formal_timeout_seconds'], grace=15, observer_identities=identities)
        record['formal_end'] = time.monotonic()
        child = record['formal']
        if child['exit_code'] != 0 or child['stop_reason']:
            record['errors'].append(child['stop_reason'] or 'formal_nonzero_exit')
    except (Exception, KeyboardInterrupt) as error:
        record['errors'].append(f'{type(error).__name__}:{error}')
    finally:
        try:
            stop.touch(exist_ok=False)
        except OSError as error:
            record['errors'].append(f'observer_stop:{error}')
        record['observer_closure'] = close_observers(owners, stop)
    observers = record['observer_closure']
    if len(observers) != 2 or any(o['exit_code'] != 0 or not o['termination_known'] or o['cleanup_errors'] for o in observers):
        record['errors'].append('observer_closure_failed_or_unknown')
    formal_closed = record.get('formal', {}).get('child_termination_known', False)
    closed_paths = [(o['kind'], summaries[0 if o['kind'] == 'ble' else 1]) for o in observers if o['termination_known']]
    if formal_closed:
        closed_paths.append(('formal', Path(plan['formal_summary'])))
    for kind, path in closed_paths:
        try:
            summary = gate.load(path)
            record[kind + '_summary'] = {k: summary[k] for k in ('status', 'error', 'cleanup_error', 'close_error', 'faults',
                                        'reset_observations', 'safe_cap_recovery', 'verified_transitions', 'capture_seconds',
                                        'tx_restore', 'stop_reason') if k in summary}
            record[kind + '_summary']['file'] = gate.reference(path)
        except (OSError, ValueError) as error:
            record['errors'].append(f'{kind}_summary:{error}')
    if formal_closed and len(observers) == 2 and all(o['termination_known'] for o in observers):
        print('Collectors exited. Checking closed evidence.', flush=True)
        try:
            result = qualify(plan, expected, record['formal_start'], record['formal_end'])
            if not record['errors']:
                record['status'] = result['status']
        except (Exception, KeyboardInterrupt) as error:
            message = f'closed_gate:{type(error).__name__}:{error}'
            record['errors'].append(message)
            result = {'status': 'UNKNOWN', 'reason': message, 'expected': expected.output(), 'overall_acceptance': 'NOT_ESTABLISHED'}
        path = directory / (label + '.gate.json')
        save(path, result)
        record['gate'], record['gate_status'] = gate.reference(path), result['status']
    if record.get('formal_summary', {}).get('status') == 'ERROR':
        record['status'] = 'FAIL'
    save(directory / (label + '.result.json'), record)
    print(json.dumps({k: v for k, v in record.items() if k in ('status', 'errors', 'formal_summary', 'gate')}), flush=True)
    return 0 if record['status'] in {'PASS_CASE_SCREEN', 'COMPLETE_50_USB_VERIFIED_OBSERVERS_CLOSED',
                                    'SHORT_V2_FULL_SNAPSHOTS_OBSERVED'} else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case', choices=DEADLINE)
    parser.add_argument('--label', required=True)
    parser.add_argument('--output-directory', type=Path, default=ROOT / 'validation_artifacts')
    parser.add_argument('--ble-address', required=True)
    parser.add_argument('--ble-adapter', default='hci0')
    parser.add_argument('--usb-serial', required=True)
    add_rail_arguments(parser)
    parser.add_argument('--expected-elf', required=True)
    parser.add_argument('--expected-session', required=True)
    parser.add_argument('--expected-boot-id', required=True, type=int)
    parser.add_argument('--print-plan', action='store_true', help='Print collector commands without opening hardware or creating case output.')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,79}', args.label):
        parser.error('invalid case label')
    try:
        expected = gate.ExpectedCaptureIdentity(args.expected_elf, args.expected_session, args.expected_boot_id)
        plan = plan_case(args, expected)
        if args.print_plan:
            print(json.dumps(plan, indent=2))
            return 0
        def stop(_sig, _frame):
            raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM, stop)
        return run(plan, expected)
    except (OSError, ValueError) as error:
        parser.exit(2, f'{error}\n')


if __name__ == '__main__':
    raise SystemExit(main())
