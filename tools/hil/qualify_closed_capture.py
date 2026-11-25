#!/usr/bin/env python3
"""New offline, fail-closed gate for one fully closed USB+BLE capture.

PASS_CASE_SCREEN authorizes the runner's next case, not project acceptance,
analog accuracy or delivery of every critical message. Original bytes and the
existing scientific parser are authoritative. No I/O to hardware occurs here.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import analyze_firmware_benchmark as benchmark
import analyze_ble_protection as protection
from sensorarray_capture_identity import ExpectedCaptureIdentity, readonly_command, verify_identity_evidence
from analyze_usb_unread import (finite, load, read_bytes, reference, resolve,
                               verify_query_trace, reject_fault_reset_text)

CRITERIA = Path(__file__).with_name('ble_acceptance_v2.json')
CRITERIA_SHA = '75b390cc277bca9e8194252b4896f6e6066d560a41f8d2c578230915c775f68a'
INTEGRITY_ERRORS = ('rejected_untrusted_frames', 'ascii_crc_errors', 'identity_errors',
                    'wire_contract_errors', 'unit_errors', 'duplicate_or_old_frames')


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def events_file(path):
    data = read_bytes(path)
    require(data.endswith(b'\n'), 'incomplete_events_tail')
    return [json.loads(line) for line in data.splitlines()]


def verify_formal(summary_path, expected, minimum_seconds):
    summary = load(summary_path)
    require(summary.get('transport') == 'usb_cdc', 'formal_CDC_required')
    require(summary.get('status') == 'CAPTURED' and not summary.get('error') and
            not summary.get('interrupted'), 'formal_capture_failed')
    require(summary.get('capture_duration_satisfied') is True and
            summary.get('safe_cap_recovery') == 'CAP8_QUERY_VERIFIED', 'duration_or_CAP8_unknown')
    start, end = summary['capture_monotonic_start'], summary['capture_monotonic_end']
    require(finite(start) and finite(end) and end - start >= minimum_seconds and
            abs(end - start - summary['capture_seconds']) < 1e-6, 'capture_window_incomplete')
    require(summary.get('firmware_fault_events') == 0 and not summary.get('firmware_faults') and
            not summary.get('reset_observations'), 'formal_fault_or_reset')
    raw_path, event_path = resolve(summary['raw_file'], summary_path), resolve(summary['events_file'], summary_path)
    raw, events = read_bytes(raw_path), events_file(event_path)
    require(hashlib.sha256(raw).hexdigest() == summary['raw_sha256'] and len(raw) == summary['rx_bytes'],
            'formal_raw_hash_or_length_mismatch')
    require(events and events[-1].get('type') == 'finished' and events[-1].get('status') == 'CAPTURED'
            and events[-1].get('error') is None and
            sum(e.get('type') == 'finished' for e in events) == 1, 'formal_final_finished_required')
    replies = verify_query_trace(events, raw)
    reject_fault_reset_text(raw)
    require(not any(e.get('type') in {'firmware_fault', 'reset_observed', 'frame_boot_changed',
                                     'tx_incomplete', 'image_authority_rejected', 'mutation_blocked'} for e in events),
            'formal_trace_failed')
    require(summary.get('image_authority', {}).get('required') is True and
            summary['image_authority'].get('expected') == expected.output(), 'expected_image_authority_missing')
    by_index = {r['event_index']: r for r in replies}
    bracket, evidence, purpose, observed = None, None, None, []
    first_mutation, capture_ended = None, False
    for index, event in enumerate(events):
        kind = event.get('type')
        if kind == 'image_authority_begin':
            require(bracket is None and event.get('expected') == expected.output(), 'authority_begin_invalid')
            bracket, evidence, purpose = [], None, event.get('purpose')
            require(purpose in {'prepare', 'cleanup'}, 'authority_purpose_unknown')
        elif kind == 'reply' and bracket is not None:
            bracket.append(by_index[index])
        elif kind == 'image_authority_verified':
            require(bracket is not None and [r['command'] for r in bracket] == ['CONFIG?', 'BUILD?', 'CONFIG?'],
                    'authority_read_bracket_missing')
            evidence = verify_identity_evidence(expected, bracket[1]['fields'], bracket[0]['fields'],
                                                bracket[2]['fields'], require_running=purpose == 'prepare')
            require(event.get('purpose') == purpose and event.get('evidence') == evidence,
                    'authority_event_not_bound_to_actual_replies')
            observed.append({'purpose': purpose, **evidence})
            bracket = None
        elif kind == 'capture_end':
            capture_ended = True
        elif kind == 'tx_begin' and not readonly_command(event.get('command')):
            require(evidence is not None and bracket is None and (not capture_ended or purpose == 'cleanup'),
                    'setter_without_prior_image_authority')
            if first_mutation is None:
                first_mutation = evidence
    require([b['purpose'] for b in observed] == ['prepare', 'cleanup'] and
            observed == summary['image_authority'].get('brackets') and first_mutation is not None and
            first_mutation == summary['image_authority'].get('first_mutation'), 'authority_summary_trace_mismatch')
    final_config = [r['fields'] for r in replies if r['command'] == 'CONFIG?' and r['phase'] == 'cleanup'][-1]
    require((final_config.get('mode'), final_config.get('rows'), final_config.get('state'),
             final_config.get('profile'), final_config.get('err'),
             final_config.get('session'), final_config.get('bootId')) ==
            ('CAP', '8', 'CAPACITANCE', 'CCCCCCCC', '0x0', expected.session, str(expected.boot_id)),
            'CAP8_not_in_actual_cleanup_reply')
    for name, expected_time in [('capture_start', start), ('capture_end', end)]:
        matches = [e for e in events if e.get('type') == name]
        require(len(matches) == 1 and matches[0].get('monotonic') == expected_time, 'capture_event_window_mismatch')
    proto_replies = [r['fields'] for r in replies if r['command'] == 'PROTO?']
    require(proto_replies and all(p == proto_replies[0] for p in proto_replies), 'PROTO_reply_missing_or_changed')
    return summary, {'summary': reference(summary_path), 'raw': reference(raw_path), 'events': reference(event_path),
                     'actual_read_brackets': observed, 'actual_PROTO_fields': proto_replies[0]}


def verify_debug(summary_path, start, end, usb_serial):
    summary = load(summary_path)
    require(summary.get('status') in {'STOPPED', 'OBSERVED'} and not summary.get('error') and
            not summary.get('close_error') and not summary.get('faults') and not summary.get('reset_observations'),
            'debug_observer_failed_or_faulted')
    raw_path, event_path = resolve(summary['raw_file'], summary_path), resolve(summary['events_file'], summary_path)
    raw, events = read_bytes(raw_path), events_file(event_path)
    require(hashlib.sha256(raw).hexdigest() == summary['raw_sha256'] and len(raw) == summary['rx_bytes'],
            'debug_raw_hash_or_length_mismatch')
    require(events[-1].get('type') == 'finished' and events[-1].get('status') == summary['status'] and
            events[-1].get('error') is None and events[-1].get('monotonic', 0) >= end,
            'debug_not_closed_after_capture')
    opened = [e for e in events if e.get('type') == 'opened']
    require(len(opened) == 1 and opened[0]['monotonic'] <= start, 'debug_not_open_through_window')
    require(isinstance(usb_serial, str) and bool(usb_serial) and
            summary.get('metadata', {}).get('usb_serial') == usb_serial and
            summary['metadata'].get('interface_number') == 2 and
            opened[0].get('metadata') == summary['metadata'], 'debug_board_or_role_unknown')
    offset, previous = 0, None
    for event in events:
        when = event.get('monotonic')
        require(finite(when) and (previous is None or when >= previous), 'debug_event_time_invalid')
        previous = when
        require(event.get('type') not in {'firmware_fault', 'reset_observed'}, 'debug_fault_or_reset')
        if event.get('type') == 'rx':
            require(event.get('offset') == offset and type(event.get('length')) is int and event['length'] > 0,
                    'debug_RX_not_contiguous')
            offset += event['length']
    require(offset == len(raw) and summary.get('commands_written') == 0, 'debug_RX_or_readonly_unverified')
    reject_fault_reset_text(raw)
    return {'summary': reference(summary_path), 'raw': reference(raw_path), 'events': reference(event_path)}


def assess_science(report, label, expected):
    """Numeric/integrity screen, also usable for labeled historical replay.

    This alone does not establish pre-mutation authority or final HIL PASS.
    """
    failures, unknown, coverage = [], [], {}
    matches = [c for c in report.get('captures', []) if c.get('label') == label]
    if len(matches) != 1:
        return {'status': 'UNKNOWN', 'failures': [], 'unknown': ['one_capture_required']}
    capture = matches[0]
    ble, journal = capture.get('ble', {}), report.get('ble_journal', {})
    if capture.get('status') != 'RECORDED' or not capture.get('window', {}).get('requested_seconds_met'):
        failures.append('USB_capture_window_or_contract_failed')
    if ble.get('status') != 'RECORDED':
        failures.append('BLE_capture_window_or_contract_failed')
    if (journal.get('schema_valid') is not True or journal.get('recording_end_observed') is not True or
            journal.get('collector_summary_verification', {}).get('status') != 'MATCH'):
        unknown.append('closed_BLE_journal_not_verified')
    if journal.get('counts', {}).get('oversize_data_lines', 0):
        unknown.append('BLE_DATA_line_integrity_unknown')
    if 'whole_journal_critical_records' not in journal:
        unknown.append('whole_journal_critical_diagnostics_unavailable')
    elif any(journal['whole_journal_critical_records'].values()):
        failures.append('BLE_journal_fault_or_reset')
    for kind in ('FAULT', 'RESET'):
        if any(capture.get('faults_and_resets', {}).get(kind, {}).get('counts_by_receipt_phase', {}).values()):
            failures.append('USB_' + kind)
    for channel in ('C', 'L'):
        if any(journal.get('all_fragment_counters_by_channel', {}).get(channel, {}).get(k, 0)
               for k in ('crc_fail', 'dropped', 'missing')):
            failures.append('whole_journal_' + channel + '_fragment_integrity')
    fragments = ble.get('fragment_integrity')
    if not fragments:
        unknown.append('fully_contained_fragment_attribution_unavailable')
    else:
        if any(fragments['fully_contained_by_channel'].get('D', {}).get(k, 0)
               for k in ('crc_fail', 'dropped', 'missing')):
            failures.append('fully_contained_DATA_fragment_integrity')
        # Edge CRC has a recorded span and remains a diagnostic; malformed
        # envelopes with no trustworthy span cannot receive a PASS.
        if fragments['edge_or_unknown_by_channel'].get('D', {}).get('dropped', 0):
            unknown.append('DATA_envelope_attribution_unknown')
    if any(ble.get('fully_contained_data_diagnostics', {}).values()):
        failures.append('fully_contained_DATA_parser_error')
    usb_counts, ble_counts = capture.get('capture_counts', {}), ble.get('capture_frame_counts', {})
    for sink, counts in [('USB', usb_counts), ('BLE', ble_counts)]:
        if any(counts.get(key, 0) for key in INTEGRITY_ERRORS):
            failures.append(sink + '_scientific_integrity')
        groups = capture.get('configuration_groups', []) if sink == 'USB' else ble.get('configuration_groups', [])
        if len(groups) != 1:
            unknown.append(sink + '_single_configuration_not_established')
        else:
            ident = groups[0].get('identity', {})
            if (ident.get('session'), ident.get('bootId')) != (expected.session, str(expected.boot_id)):
                failures.append(sink + '_expected_boot_mismatch')
            if any(ident.get(k) != capture.get('expected_config_identity', {}).get(k)
                   for k in ('session', 'bootId', 'rev', 'mgen', 'mrid', 'rgen', 'rrid', 'pgen', 'prid')):
                failures.append(sink + '_immutable_configuration_mismatch')
            intervals, rates = groups[0].get('host_complete_frame_intervals', {}), groups[0].get('rates', {})
            window = capture.get('window', {})
            window_start = window.get('capture_start_seconds' if sink == 'USB' else 'capture_monotonic_start')
            window_end = window.get('capture_end_seconds' if sink == 'USB' else 'capture_monotonic_end')
            first, last = intervals.get('first_complete_host_seconds'), intervals.get('last_complete_host_seconds')
            requested = capture.get('requested', {}).get('requested_seconds')
            device_span, capture_fps = rates.get('device_span_seconds'), rates.get('capture_fps')
            adequate = (intervals.get('known') is True and
                        all(finite(v) for v in (window_start, window_end, first, last, requested, device_span, capture_fps)) and
                        window_start <= first <= last <= window_end and first - window_start <= 1.0 and
                        window_end - last <= 1.0 and device_span >= requested - 2.0 and capture_fps >= 1.0)
            coverage[sink] = {'sufficient': adequate, 'first_complete_host_seconds': first,
                              'last_complete_host_seconds': last, 'window_start_seconds': window_start,
                              'window_end_seconds': window_end, 'device_span_seconds': device_span,
                              'requested_seconds': requested, 'capture_fps': capture_fps,
                              'edge_allowance_seconds': 1.0,
                              'rule': 'separate FULL observation sufficiency; not BLE delivery deadline or redefinition of endpoint gap metric'}
            if not adequate:
                unknown.append(sink + '_full_window_observation_insufficient')
        if counts.get('ordered_integrity_valid_frames', 0) < 2:
            unknown.append(sink + '_insufficient_trusted_frames')
    budget = load(CRITERIA)['normal_link']['maximum_complete_frame_gap_fraction']
    if usb_counts.get('missing_frames', 0):
        failures.append('USB_complete_sequence_gap')
    if ble.get('USB_comparison', {}).get('different_wire_bytes_sha256', 0):
        failures.append('cross_sink_original_bytes_mismatch')
    received, missing = ble_counts.get('ordered_integrity_valid_frames', 0), ble_counts.get('missing_frames', 0)
    fraction = missing / (received + missing) if received + missing else None
    if fraction is None:
        unknown.append('BLE_gap_denominator_unknown')
    elif fraction > budget:
        failures.append('BLE_normal_complete_frame_gap_budget_exceeded')
    return {'status': 'FAIL' if failures else 'UNKNOWN' if unknown else 'PASS_NUMERIC_SCREEN',
            'failures': failures, 'unknown': unknown, 'received': received, 'missing': missing,
            'expected_within_BLE_endpoints': received + missing, 'missing_fraction': fraction,
            'normal_budget_fraction': budget, 'overall_acceptance': 'NOT_ESTABLISHED',
            'FULL_observation_coverage': coverage,
            'edge_or_unknown_fragment_diagnostics': fragments.get('edge_or_unknown_by_channel') if fragments else None,
            'complete_is_not_all_points_valid': True}


def qualify_closed_case(summary_path, ble_summary_path, debug_summary_path, expected, minimum_seconds=120):
    require(hashlib.sha256(CRITERIA.read_bytes()).hexdigest() == CRITERIA_SHA, 'criteria_source_changed')
    formal, authority = verify_formal(summary_path, expected, minimum_seconds)
    require({'ble_flow_v1', 'ble_snapshot_v1'} <= set(authority['actual_PROTO_fields'].get('caps', '').split('/')),
            'required_capabilities_not_in_actual_PROTO')
    debug = verify_debug(debug_summary_path, formal['capture_monotonic_start'], formal['capture_monotonic_end'],
                         formal.get('transport_metadata', {}).get('usb_serial'))
    ble_summary = load(ble_summary_path)
    require(ble_summary.get('status') == 'RECORDED' and not ble_summary.get('error') and
            not ble_summary.get('cleanup_errors') and ble_summary.get('unexpected_disconnect') is False,
            'BLE_observer_not_closed_successfully')
    journal = resolve(ble_summary['journal_file'], ble_summary_path)
    report = benchmark.analyse_directory(summary_path.parent, ble_journal=journal, debug_summaries=[debug_summary_path])
    label = summary_path.name.removesuffix('.summary.json')
    science = assess_science(report, label, expected)
    local = protection.analyze_file(summary_path)
    require(all(q['status'] == 'OBSERVED_LOCAL_COUNTERS' for q in local['queries'].values()),
            'required_protection_telemetry_unknown')
    flow = local['queries']['BLEFLOW?']
    require(all(flow[phase]['fields']['ratePm'] == '1000' for phase in ('before', 'after')) and
            flow['counter_deltas']['intentionalSkip']['delta'] == 0, 'FULL_nominal_policy_not_observed')
    for evidence in (*[authority[key] for key in ('summary', 'raw', 'events')], *debug.values()):
        require(reference(ROOT / evidence['path']) == evidence, 'closed_evidence_changed_during_analysis')
    return {'schema_version': 1, 'status': 'PASS_CASE_SCREEN' if science['status'] == 'PASS_NUMERIC_SCREEN' else science['status'],
            'scope': 'closed_normal_FULL_case_screen_before_next_case; not_project_or_precision_acceptance',
            'expected': expected.output(), 'criteria': reference(CRITERIA), 'formal_authority': authority,
            'debug': debug, 'BLE_summary': reference(ble_summary_path), 'science': science,
            'analysis': report, 'protection': local, 'hardware_actions': [], 'automatic_retry': False,
            'overall_acceptance': 'NOT_ESTABLISHED', 'generator': reference(Path(__file__))}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('summary', 'ble-summary', 'debug-summary', 'output'):
        parser.add_argument('--' + name, required=True, type=Path)
    parser.add_argument('--expected-elf', required=True)
    parser.add_argument('--expected-session', required=True)
    parser.add_argument('--expected-boot-id', type=int, required=True)
    args = parser.parse_args(argv)
    expected = ExpectedCaptureIdentity(args.expected_elf, args.expected_session, args.expected_boot_id)
    with args.output.open('x') as stream:
        try:
            result = qualify_closed_case(args.summary, args.ble_summary, args.debug_summary, expected)
        except (ValueError, KeyError, TypeError, OSError) as error:
            result = {'status': 'UNKNOWN', 'error': str(error), 'hardware_actions': [], 'overall_acceptance': 'NOT_ESTABLISHED'}
        json.dump(result, stream, indent=2)
        stream.write('\n')
    print(json.dumps({key: result.get(key) for key in ('status', 'error', 'science')}))
    return 0 if result['status'] == 'PASS_CASE_SCREEN' else 1


if __name__ == '__main__':
    raise SystemExit(main())
