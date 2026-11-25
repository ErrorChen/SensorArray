#!/usr/bin/env python3
"""Offline BLE output/protection diagnostics from a closed formal summary.

Each query retains its own host interval. These are local accounting counters,
not RF loss, peer receipt, or acceptance verdicts. No hardware is accessed.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import re

import capture_bulk_benchmark as schema

COMMANDS = tuple(schema.FRAME_OUTPUT_REQUIRED)
COUNTERS = {
    'BLEFLOW?': dict(intentionalSkip=64, probeSelected=64, attempts=32, capacityReject=32),
    'BLESNAP?': dict.fromkeys('queued localComplete failed admitMiss deadlineMiss superseded staleNormal'.split(), 32),
    'SNAPSRC?': dict.fromkeys('queued firstQueued periodicQueued candidateMiss firstMiss superseded firstSuperseded'.split(), 32),
    'SNAPNET?': dict.fromkeys('queued admissionMiss firstMiss derived superseded retry expired bleExpired wifiExpired bleAccepted bleFailed policySkip obsolete'.split(), 32),
}
# A conservative analysis plausibility ceiling, not a measured event rate.
# Together these bounds permit at most one uint32 rollover, below half range.
MAX_EVENTS_PER_SECOND = 1_000_000
MAX_PAIR_SECONDS = 1800
UNKNOWN = 'unknown'
# This recorded v1 implementation writes base `finished`, then restores TX in
# finally and updates the summary. It has no second final event. Limit the
# historical adapter to the reviewed source, not arbitrary post-finish logs.
LEGACY_SHORT_SOURCE = '545d0ef0779d06b88bb48a753dd69050e1cf3d2fe6a899bb1010447079ab5c1f'
SHORT_VARIANT = 'formal_cdc_ble_short_snapshot_diagnostic_v1'


def parse_line(line: str, tag: str) -> dict[str, str]:
    if not isinstance(line, str) or '\n' in line or '\r' in line:
        raise ValueError('invalid_reply_line')
    parts = line.split(',')
    if parts[0] != tag:
        raise ValueError('wrong_reply_tag')
    fields = {}
    for item in parts[1:]:
        key, separator, value = item.partition('=')
        if not separator or not key or not value or key in fields:
            raise ValueError('malformed_or_duplicate_field')
        fields[key] = value
    return fields


def monotonic(value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
        raise ValueError('invalid_host_monotonic')
    return value


def bounded_delta(before: str, after: str, bits: int, maximum: int) -> dict:
    a = schema.unsigned(before, 'offline', 'before', bits)
    b = schema.unsigned(after, 'offline', 'after', bits)
    delta = (b - a) % (1 << bits)
    bound = min(maximum, (1 << (bits - 1)) - 1)
    if maximum < 0 or delta > bound:
        return {'delta': UNKNOWN, 'reason': 'reset_or_increment_exceeds_bound', 'bits': bits, 'bound': bound}
    return {'delta': delta, 'bits': bits, 'bound': bound, 'rollover_observed': b < a,
            'assumption': 'at_most_one_rollover_within_analysis_event_bound'}


def config_brackets(summary: dict, raw: dict, replies: set) -> dict:
    snapshots = summary['snapshots']
    identities, brackets = [], {}
    for phase in ('before', 'after'):
        snapshots_phase = snapshots[phase]
        brackets[phase] = {}
        for key in ('CONFIG?', 'CONFIG?_closing'):
            record = snapshots_phase[key]
            fields = record['fields']
            schema.validate_reply('CONFIG?', fields)
            identity = tuple(fields[k] for k in schema.CONFIG_CONSISTENCY)
            if not re.fullmatch(r'[0-9A-Fa-f]{16}', fields['session']):
                raise ValueError('invalid_CONFIG_session')
            identities.append(identity)
            matches = [line for p, command, line in replies if p == phase and command == 'CONFIG?'
                       and line in raw['ctrl' if summary['transport'] == 'usb_bulk' else 'data']
                       and parse_line(line, 'CONFIG') == fields]
            if not matches:
                raise ValueError('CONFIG_not_in_original_raw_and_event_phase')
            brackets[phase][key] = {'fields': fields, 'original_reply_line': matches[0],
                                    'host_query_end_monotonic': monotonic(record['monotonic'])}
            if key == 'CONFIG?_closing':
                brackets[phase][key]['host_query_start_monotonic'] = monotonic(record['started_monotonic'])
        if snapshots_phase['CONFIG?']['monotonic'] > snapshots_phase['CONFIG?_closing']['started_monotonic']:
            raise ValueError('CONFIG_bracket_time_order')
    if len(set(identities)) != 1:
        raise ValueError('CONFIG_bracket_session_boot_or_revision_changed')
    if summary.get('reset_observations'):
        raise ValueError('reset_observed_no_cross_boot_pairing')
    return {'identity': dict(zip(schema.CONFIG_CONSISTENCY, identities[0])), 'brackets': brackets}


def query_record(summary: dict, phase: str, command: str, raw: dict, replies: set) -> dict:
    window = summary['snapshots'][phase]
    record = window[command]
    line, fields = record['reply_line'], record['fields']
    if parse_line(line, command[:-1]) != fields:
        raise ValueError('summary_fields_differ_from_original_reply')
    schema.validate_frame_output(command, fields)
    channel = record['reply_channel']
    if channel != ('ctrl' if summary['transport'] == 'usb_bulk' else 'data'):
        raise ValueError('unexpected_reply_channel')
    if line not in raw[channel] or (phase, command, line) not in replies:
        raise ValueError('reply_not_in_original_raw_and_event_phase')
    start, event, end = (monotonic(record[k]) for k in ('started_monotonic', 'reply_event_monotonic', 'monotonic'))
    if not monotonic(window['CONFIG?']['monotonic']) <= start <= event <= end <= monotonic(window['CONFIG?_closing']['started_monotonic']):
        raise ValueError('query_outside_CONFIG_bracket')
    capture_start = monotonic(summary['capture_monotonic_start'])
    capture_end = monotonic(summary['capture_monotonic_end'])
    if capture_end < capture_start:
        raise ValueError('invalid_capture_interval')
    if (phase == 'before' and end > capture_start) or (phase == 'after' and start < capture_end):
        raise ValueError('optional_query_inside_capture')
    return {'fields': fields, 'original_reply_line': line, 'reply_channel': channel,
            'host_query_start_monotonic': start, 'host_reply_event_monotonic': event,
            'host_query_end_monotonic': end}


def sampled_epoch(queries: dict, summary: dict) -> dict:
    epochs = []
    for command in ('BLEFLOW?', 'BLESNAP?'):
        row = queries[command]
        if row['endpoints_validated']:
            epochs.extend((row[p]['fields']['gen'], row[p]['fields']['stream']) for p in ('before', 'after'))
        elif row['advertised']:
            raise ValueError('advertised_BLE_epoch_observation_unavailable')
    if not epochs:
        raise ValueError('BLE_epoch_observations_unavailable')
    if len(set(epochs)) != 1:
        raise ValueError('BLE_connection_or_stream_generation_changed')
    for phase in ('before', 'after'):
        for command in ('BLELINK?', 'BLETX?'):
            observed = summary['snapshots'][phase].get(command)
            if observed and observed['fields']['gen'] != epochs[0][0]:
                raise ValueError('BLE_link_or_sender_generation_changed')
    return {'gen': epochs[0][0], 'stream': epochs[0][1],
            'scope': 'matching_sampled_endpoints_not_proof_of_unobserved_link_behavior'}


def endpoint_view(command: str, record: dict) -> dict:
    fields = record['fields']
    if command == 'BLEFLOW?':
        names = 'active stage ratePm evidence rejectPm pressureWindows cleanUs localServiceMilliFps peerLoss rssi'.split()
    elif command == 'BLESNAP?':
        names = 'known lastSeq lastRev lastCompleteAgeUs gapKnown maxCompleteGapUs firstPending periodicPending dueUs pcReceipt'.split()
    else:
        names = ('stage', 'pcReceipt')
    # Preserve unknown verbatim. These gauges are not cumulative deltas.
    return {key: fields[key] for key in names}


def analyze_document(summary: dict, raw: dict, replies: set) -> dict:
    report = {'schema_version': 1, 'status': 'ANALYZED', 'overall_acceptance': 'NOT_EVALUATED',
              'recording_status': summary.get('status'), 'PC_snapshot_receipt': UNKNOWN,
              'RF_loss': UNKNOWN, 'RSSI': UNKNOWN, 'PC_missing_frames_adjusted': 'NOT_COMPUTED',
              'window_scope': 'separate_before_after_query_intervals_not_capture_endpoints',
              'counter_bound_policy': {'max_pair_seconds': MAX_PAIR_SECONDS,
                                       'max_local_events_per_second': MAX_EVENTS_PER_SECOND,
                                       'purpose': 'analysis_plausibility_guard_not_measured_firmware_rate'},
              'semantics': [
                  'intentionalSkip counts local admission decisions; it is never subtracted from PC sequence gaps.',
                  'Source queue, network admission and BLE local completion have different accounting boundaries.',
                  'SNAPNET readout is sequential; its counters are not one atomic cross-layer conservation equation.',
                  'bleExpired/wifiExpired are sink obligations of a descriptor; summing them is not a count of unique lost scientific frames.',
                  'lastCompleteAgeUs and maxCompleteGapUs refer to local whole-frame completion, not sample age or PC receipt.',
                  'SHORT ordinary previews can create intentional full-frame sequence gaps; no FULL 2.5% acceptance budget is applied here.',
                  'Unknown values remain unknown; endpoint stage/rate cannot establish behavior throughout the interval.',
              ], 'queries': {}}
    context_error = None
    try:
        if summary['transport'] not in {'usb_cdc', 'usb_bulk'}:
            raise ValueError('formal_USB_transport_required')
        report['configuration'] = config_brackets(summary, raw, replies)
    except (KeyError, TypeError, ValueError, schema.CaptureError) as error:
        context_error = str(error)
        report['configuration'] = {'status': 'UNKNOWN', 'reason': context_error}
    advertised = summary.get('optional_service_diagnostics', {})
    for command in COMMANDS:
        cap = advertised.get(command, {})
        row = {'capability': schema.OPTIONAL_CAPABILITIES[command],
               'advertised': cap.get('advertised') is True and cap.get('capability') == schema.OPTIONAL_CAPABILITIES[command],
               'endpoints_validated': False, 'status': 'UNKNOWN', 'reason': None,
               'before': None, 'after': None, 'counter_deltas': {}}
        report['queries'][command] = row
        try:
            if not row['advertised']:
                raise ValueError('capability_not_advertised')
            for phase in ('before', 'after'):
                row[phase] = query_record(summary, phase, command, raw, replies)
                row[phase]['observed_state'] = endpoint_view(command, row[phase])
            row['endpoints_validated'] = True
        except (KeyError, TypeError, ValueError, schema.CaptureError) as error:
            row['reason'] = str(error)
    epoch_error = None
    try:
        report['BLE_epoch'] = sampled_epoch(report['queries'], summary)
    except (KeyError, TypeError, ValueError) as error:
        epoch_error = str(error)
        report['BLE_epoch'] = {'status': 'UNKNOWN', 'reason': epoch_error}
    for command, row in report['queries'].items():
        try:
            if not row['endpoints_validated']:
                continue
            if context_error or epoch_error:
                raise ValueError(context_error or epoch_error)
            before, after = row['before'], row['after']
            minimum = after['host_query_start_monotonic'] - before['host_query_end_monotonic']
            maximum = after['host_query_end_monotonic'] - before['host_query_start_monotonic']
            if minimum <= 0 or maximum > MAX_PAIR_SECONDS:
                raise ValueError('query_pair_interval_out_of_bound')
            row['host_snapshot_interval_seconds'] = {'minimum': minimum, 'maximum': maximum,
                                                    'scope': 'local_counter_snapshot_lies_within_each_host_query'}
            event_bound = math.ceil(maximum * MAX_EVENTS_PER_SECOND)
            row['counter_deltas'] = {key: bounded_delta(before['fields'][key], after['fields'][key], bits, event_bound)
                                     for key, bits in COUNTERS[command].items()}
            row['status'] = 'OBSERVED_LOCAL_COUNTERS'
            if any(value['delta'] == UNKNOWN for value in row['counter_deltas'].values()):
                row['status'] = 'PARTIAL_UNKNOWN_COUNTERS'
        except (KeyError, TypeError, ValueError, schema.CaptureError) as error:
            row['reason'] = str(error)
    return report


def evidence_path(text: str, summary_path: Path) -> Path:
    path = Path(text)
    candidates = (path, summary_path.parent / path.name, summary_path.parent / path)
    for candidate in candidates:
        if candidate.is_file(): return candidate
        compressed = candidate.with_name(candidate.name + '.gz')
        if compressed.is_file(): return compressed
    raise ValueError('missing_evidence:' + text)


def file_sha(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(65536), b''): digest.update(chunk)
    return digest.hexdigest()


def read_raw(path: Path, expected_sha: str) -> tuple[set, dict]:
    digest, total, carry, discard = hashlib.sha256(), 0, b'', False
    lines = set()
    prefixes = tuple((command[:-1] + ',').encode('ascii') for command in COMMANDS) + (b'CONFIG,', b'MODE,')
    opener = gzip.open if path.suffix == '.gz' else open
    with opener(path, 'rb') as source:
        for chunk in iter(lambda: source.read(65536), b''):
            digest.update(chunk); total += len(chunk)
            complete = (carry + chunk).split(b'\n'); carry = complete.pop()
            for line in complete:
                if discard: discard = False; continue
                if line.startswith(prefixes):
                    lines.add(line.rstrip(b'\r').decode('ascii'))
            if len(carry) > 8192: carry = b''; discard = True
    if digest.hexdigest() != expected_sha:
        raise ValueError('raw_sha256_mismatch:' + str(path))
    return lines, {'path': str(path), 'recorded_bytes_sha256': digest.hexdigest(), 'recorded_bytes': total,
                   'stored_file_sha256': file_sha(path), 'incomplete_tail_used': False}


def integer(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError('invalid_event_counter')
    return value


def raw_reply_between(info: dict, line: str, begin: int, end: int) -> bool:
    """Bind a restore reply to bytes read after that command's actual write.

    A matching ACK elsewhere in the recording cannot close this operation.
    Include one prefix byte to verify the original complete-line boundary.
    """
    path = Path(info['path'])
    start = max(0, begin - 1)
    with (gzip.open if path.suffix == '.gz' else open)(path, 'rb') as source:
        source.seek(start)
        data = source.read(end-start)
    if len(data) != end-start:
        raise ValueError('restore_raw_range_unavailable')
    for suffix in (b'\n', b'\r\n'):
        needle = line.encode('ascii') + suffix
        index = data.find(needle)
        while index >= 0:
            if start+index >= begin and (start+index == 0 or index > 0 and data[index-1] == 10):
                return True
            index = data.find(needle, index+1)
    return False


def validate_legacy_short_closure(summary: dict, events: list, raw: dict, inputs: dict) -> dict:
    if (summary.get('source_sha256') != LEGACY_SHORT_SOURCE or summary.get('transport') != 'usb_cdc' or
            summary.get('ble_output_test_profile') != 'SHORT_WITH_PROTECTED_FULL_SNAPSHOTS' or
            summary.get('tx_change_attempted') is not True or summary.get('original_tx') not in {'rel', 'full'} or
            summary.get('normal_FULL_loss_budget_applicable') is not False or summary.get('snapshot_acceptance') != 'NOT_EVALUATED'):
        raise ValueError('unsupported_legacy_SHORT_closure_contract')
    finishes = [i for i, event in enumerate(events) if event.get('type') == 'finished']
    if len(finishes) != 1:
        raise ValueError('one_base_finished_event_required')
    index = finishes[0]
    base, tail = events[index], events[index+1:]
    if (not tail or base.get('phase') != 'cleanup' or base.get('status') not in {'CAPTURED', 'ERROR'} or
            (base.get('status') == 'CAPTURED') != (base.get('error') is None)):
        raise ValueError('legacy_SHORT_base_finish_or_restore_tail_invalid')
    elapsed = monotonic(summary['elapsed_seconds'])
    if elapsed > monotonic(summary['duration_total_budget_seconds']):
        raise ValueError('legacy_SHORT_total_budget_exceeded')
    previous, cursor, tx_bytes = 0., 0, 0
    rx_at = {}
    for i, event in enumerate(events):
        stamp = monotonic(event['t'])
        if not previous <= stamp <= elapsed:
            raise ValueError('legacy_SHORT_event_time_order')
        previous = stamp
        if event.get('type') == 'rx':
            if event.get('channel') != 'data' or integer(event['offset']) != cursor or integer(event['length']) == 0:
                raise ValueError('legacy_SHORT_RX_coverage_invalid')
            cursor += event['length']
        if event.get('type') in {'tx_complete', 'tx_incomplete'}:
            tx_bytes += integer(event['accepted_bytes'])
        rx_at[i] = cursor
    if cursor != inputs['data']['recorded_bytes'] or cursor != integer(summary['rx_bytes']):
        raise ValueError('legacy_SHORT_final_RX_count_mismatch')
    if any(tx_bytes != integer(summary[key]) for key in ('tx_host_accepted_bytes', 'tx_os_accepted_bytes')):
        raise ValueError('legacy_SHORT_final_TX_count_mismatch')
    faults = [event for event in events if event.get('type') == 'firmware_fault']
    resets = [event for event in events if event.get('type') == 'reset_observed']
    if len(faults) != integer(summary['firmware_fault_events']) or len(resets) != integer(summary['reset_event_count']):
        raise ValueError('legacy_SHORT_final_fault_reset_count_mismatch')
    expected_resets = [{k: event[k] for k in ('phase', 'line', 'channel')} for event in resets[:16]]
    if summary.get('reset_observations') != expected_resets:
        raise ValueError('legacy_SHORT_reset_examples_mismatch')
    profiles = [event for event in events[:index] if event.get('type') == 'short_snapshot_profile']
    if len(profiles) != 1 or profiles[0].get('original_tx') != summary['original_tx']:
        raise ValueError('legacy_SHORT_original_profile_not_observed')
    setter = 'TX=' + summary['original_tx'].upper()
    allowed = {'tx_begin', 'tx_complete', 'tx_incomplete', 'rx', 'reply', 'readonly_query_attempt',
               'readonly_query_failure', 'readonly_query_recovered', 'uncorrelated_error', 'command_error',
               'reset_observed', 'firmware_fault', 'tx_restore_error', 'tx_restore_observation_error', 'line_overflow'}
    active_command = None
    active_begin = None
    completed_write = False
    setter_begins = setter_replies = query_replies = 0
    query_attempts = 0
    restore_replies = []
    for i, event in enumerate(tail, index+1):
        kind = event.get('type')
        if event.get('phase') != 'tx_restore' or kind not in allowed:
            raise ValueError('unexpected_event_after_base_finished')
        if 'command' in event and event['command'] not in {setter, 'TX?'}:
            raise ValueError('unexpected_command_after_base_finished')
        if kind == 'tx_begin':
            active_command = event['command']; active_begin = rx_at[i]
            completed_write = False
            if event.get('channel') != 'data' or event.get('wire_ascii') != '\n'+active_command+'\n':
                raise ValueError('legacy_SHORT_restore_wire_invalid')
            if active_command == setter:
                setter_begins += 1
                if setter_begins != 1 or query_attempts:
                    raise ValueError('legacy_SHORT_restore_setter_replayed')
            elif setter_replies != 1 or query_attempts == 0:
                raise ValueError('legacy_SHORT_query_before_setter_ACK')
        elif kind == 'tx_complete':
            if event.get('command') != active_command or event['accepted_bytes'] != len(active_command)+2:
                raise ValueError('legacy_SHORT_restore_write_not_complete')
            completed_write = True
        elif kind == 'readonly_query_attempt':
            query_attempts += 1
            if event.get('command') != 'TX?' or integer(event['attempt']) != query_attempts or query_attempts > 3:
                raise ValueError('legacy_SHORT_query_attempt_order')
        elif kind == 'reply':
            if not completed_write or event.get('command') != active_command or active_begin is None:
                raise ValueError('legacy_SHORT_reply_without_complete_write')
            if parse_line(event['line'], 'ACK') != {'cmd': 'TX', 'v': summary['original_tx']}:
                raise ValueError('legacy_SHORT_restore_ACK_mismatch')
            if not raw_reply_between(inputs['data'], event['line'], active_begin, rx_at[i]):
                raise ValueError('legacy_SHORT_restore_ACK_not_in_its_RX_range')
            restore_replies.append({'command': active_command, 'line': event['line'], 'event_t': event['t'],
                                    'raw_read_begin': active_begin, 'raw_read_end': rx_at[i]})
            if active_command == setter: setter_replies += 1
            else: query_replies += 1
            completed_write = False
    restore_errors = [event for event in tail if event['type'] == 'tx_restore_error']
    observation_errors = [event for event in tail if event['type'] == 'tx_restore_observation_error']
    restore_error = summary.get('tx_restore_error')
    if restore_error is None:
        if summary.get('tx_restore') != 'QUERY_VERIFIED' or restore_errors or (setter_begins, setter_replies, query_replies) != (1, 1, 1):
            raise ValueError('legacy_SHORT_verified_restore_not_proven')
        if tail[-1]['type'] not in {'reply', 'readonly_query_recovered', 'tx_restore_observation_error'}:
            raise ValueError('legacy_SHORT_restore_tail_not_closed')
    elif (summary.get('tx_restore') != 'FAILED_OR_UNKNOWN' or len(restore_errors) != 1 or
          restore_errors[0].get('reason') != restore_error or tail[-1]['type'] not in {'tx_restore_error', 'tx_restore_observation_error'}):
        raise ValueError('legacy_SHORT_restore_failure_not_closed')
    new_faults = sum(event['type'] == 'firmware_fault' for event in tail)
    new_resets = sum(event['type'] == 'reset_observed' for event in tail)
    observation_error = 'tx_restore_new_fault_or_reset' if new_faults or new_resets else None
    if (summary.get('tx_restore_observation_error') != observation_error or len(observation_errors) != bool(observation_error) or
            any(event.get('reason') != observation_error for event in observation_errors)):
        raise ValueError('legacy_SHORT_restore_fault_reset_not_reported')
    if observation_error and summary.get('safe_cap_recovery') != 'INVALIDATED_BY_LATER_FAULT_OR_RESET':
        raise ValueError('legacy_SHORT_late_fault_did_not_invalidate_CAP')
    error = base.get('error') or observation_error or ('tx_restore:' + restore_error if restore_error else None)
    if summary.get('error') != error or summary['status'] != ('ERROR' if error else 'CAPTURED'):
        raise ValueError('legacy_SHORT_summary_status_or_first_error_mismatch')
    cap_state = summary.get('prior_cap_recovery') if observation_error else summary.get('safe_cap_recovery')
    cap_proof = None
    if cap_state == 'CAP8_QUERY_VERIFIED':
        records = {}
        for command, tag in (('MODE?', 'MODE'), ('CONFIG?', 'CONFIG')):
            matches = [e for e in events[:index] if e.get('type') == 'reply' and e.get('phase') == 'cleanup' and e.get('command') == command]
            if not matches or matches[-1]['line'] not in raw['data']:
                raise ValueError('legacy_SHORT_CAP_recovery_query_missing')
            records[command] = parse_line(matches[-1]['line'], tag)
        config, mode = records['CONFIG?'], records['MODE?']
        if ((config.get('state'), config.get('mode'), config.get('rows'), config.get('profile'), config.get('err')) !=
                ('CAPACITANCE', 'CAP', '8', 'CCCCCCCC', '0x0') or
                (mode.get('state'), mode.get('active'), mode.get('route'), mode.get('pending'), mode.get('intref'), mode.get('refmux')) !=
                ('CAPACITANCE', 'CAP', 'CAP', 'NONE', '0', '24')):
            raise ValueError('legacy_SHORT_CAP_recovery_query_invalid')
        if any(config.get(k) != summary['snapshots']['after']['CONFIG?']['fields'].get(k) for k in ('session', 'bootId')):
            raise ValueError('legacy_SHORT_recovery_boot_changed')
        cap_proof = records
    elif summary['status'] == 'CAPTURED':
        raise ValueError('legacy_SHORT_success_without_CAP_recovery')
    return {'kind': 'legacy_SHORT_restore_tail_verified', 'explicit_final_finished_event': False,
            'reviewed_source_sha256': LEGACY_SHORT_SOURCE, 'base_finished': base,
            'last_event': tail[-1], 'summary_status': summary['status'], 'restore': summary['tx_restore'],
            'restore_replies': restore_replies, 'tail_events': len(tail), 'tail_RX_bytes': cursor-rx_at[index],
            'new_fault_events': new_faults, 'new_reset_events': new_resets,
            'CAP8_query_before_TX_restore': cap_proof,
            'scope': 'hashed_final_summary_plus_complete_raw_event_coverage_and_recorded_restore_protocol',
            'limits': ['No second finished event was recorded by this historical subclass.',
                       'CAP8 was queried before the TX-only restore; no new post-restore CONFIG query is invented.',
                       'Local restore ACK/query does not establish BLE PC snapshot delivery.']}


def analyze_file(summary_path: Path) -> dict:
    summary_bytes = summary_path.read_bytes()
    summary = json.loads(summary_bytes)
    if summary.get('status') not in {'CAPTURED', 'ERROR'}:
        raise ValueError('closed_formal_summary_required')
    raw, inputs = {}, {'summary': {'path': str(summary_path), 'sha256': hashlib.sha256(summary_bytes).hexdigest()}}
    for channel, path_key, sha_key in (('data', 'raw_file', 'raw_sha256'), ('ctrl', 'ctrl_raw_file', 'ctrl_raw_sha256')):
        if summary.get(path_key):
            raw[channel], inputs[channel] = read_raw(evidence_path(summary[path_key], summary_path), summary[sha_key])
    events_path = evidence_path(summary['events_file'], summary_path)
    events_file_sha = file_sha(events_path)
    opener = gzip.open if events_path.suffix == '.gz' else open
    replies, final, digest, events = set(), None, hashlib.sha256(), []
    with opener(events_path, 'rb') as source:
        for line in source:
            digest.update(line)
            if not line.endswith(b'\n'): raise ValueError('incomplete_events_tail')
            event = json.loads(line)
            events.append(event)
            final = event
            if event.get('type') == 'reply' and event.get('phase') in {'before', 'after'}:
                if event.get('command') in (*COMMANDS, 'CONFIG?'):
                    replies.add((event['phase'], event['command'], event['line']))
    if summary.get('runner_variant') == SHORT_VARIANT and summary.get('tx_change_attempted') is True:
        closure = validate_legacy_short_closure(summary, events, raw, inputs)
    else:
        if not final or final.get('type') != 'finished' or final.get('status') != summary['status']:
            raise ValueError('matching_finished_event_required')
        closure = {'kind': 'matching_finished_event', 'explicit_final_finished_event': True, 'event': final}
    for channel in raw:
        if file_sha(Path(inputs[channel]['path'])) != inputs[channel]['stored_file_sha256']:
            raise ValueError('raw_changed_while_reading')
    inputs['events'] = {'path': str(events_path), 'recorded_bytes_sha256': digest.hexdigest(),
                        'stored_file_sha256': events_file_sha}
    if file_sha(events_path) != events_file_sha:
        raise ValueError('events_changed_while_reading')
    if summary_path.read_bytes() != summary_bytes:
        raise ValueError('summary_changed_while_reading')
    result = analyze_document(summary, raw, replies)
    result.update(inputs=inputs, closure=closure, hardware_actions=[], original_recordings_modified=False,
                  generator={'path': str(Path(__file__)), 'sha256': file_sha(Path(__file__)),
                             'schema_source_sha256': file_sha(Path(schema.__file__))})
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--summary', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args(argv)
    # Refuse an existing path before parsing potentially large recordings.
    if args.output.exists(): raise FileExistsError(str(args.output))
    result = analyze_file(args.summary)
    with args.output.open('x') as target:
        json.dump(result, target, indent=2, sort_keys=True, allow_nan=False); target.write('\n')
    print(args.output)
    return 0


if __name__ == '__main__': raise SystemExit(main())
