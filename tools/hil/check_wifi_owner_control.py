#!/usr/bin/env python3
"""Bounded AP/OFF command and terminal-cache HIL, with one formal CDC reader.

No flash, reset, NVS calibration, radio-fault injection or UDP-receipt claim.
The initial Wi-Fi output gate must be OFF; the same boot is restored to OFF.
Backend failure is recorded separately from continued USB control service.
"""
from contextlib import ExitStack
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from capture_reference_fixture import CaptureError, CaptureOptions, Collector
from sensorarray_capture_identity import CaptureImageAuthority, ExpectedCaptureIdentity, validate_config
from sensorarray_usb_cdc import UsbCdcPort
from text_protocol import parse_fields


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--label', required=True)
    p.add_argument('--usb-serial', required=True)
    p.add_argument('--output-directory', type=Path, default=ROOT / 'validation_artifacts')
    p.add_argument('--expected-elf', required=True)
    p.add_argument('--expected-session', required=True)
    p.add_argument('--expected-boot', required=True)
    p.add_argument('--allocation-trace', action='store_true',
                   help='Read MEMFAIL? before/after AP on a known supporting image')
    a = p.parse_args()
    if not all(c.isalnum() or c in '-_' for c in a.label):
        raise ValueError('safe_new_label_required')
    expected = ExpectedCaptureIdentity(a.expected_elf, a.expected_session, a.expected_boot)
    authority = CaptureImageAuthority(expected)
    directory = a.output_directory.resolve() / a.label
    directory.parent.mkdir(parents=True, exist_ok=True)
    directory.mkdir(exist_ok=False)
    paths = {key: directory / (a.label + suffix) for key, suffix in
             (('raw', '.rx.bin'), ('events', '.events.jsonl'), ('summary', '.summary.json'))}
    started = time.monotonic()
    deadline = started + 45.0
    work_deadline = deadline - 6.0
    result = {'expected': expected.output(), 'requests': [], 'queries': [],
              'status': 'ERROR', 'error': None, 'off_restore': 'NOT_NEEDED',
              'SDK_stall_injection': 'NOT_RUN', 'UDP_PC_receipt': 'NOT_RUN',
              'physical_off': 'UNKNOWN', 'source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    port = None
    collector = None
    attempted = False
    seen_request_ids = set()

    with ExitStack() as stack:
        raw = stack.enter_context(paths['raw'].open('xb'))
        events = stack.enter_context(paths['events'].open('x'))

        def same_wifi_boot(fields):
            if (str(fields.get('session', '')).upper() != expected.session or
                    fields.get('boot') != str(expected.boot_id) or fields.get('bootKnown') != '1'):
                authority.revoke()
                raise CaptureError('wifi_reply_boot_identity_mismatch')

        def query(command, tag, end):
            began = time.monotonic()
            fields = collector.query(command, tag, end)
            result['queries'].append({'command': command, 'fields': fields,
                                      'elapsed_seconds': time.monotonic() - began})
            if collector.faults or collector.reset_observations:
                authority.revoke()
                raise CaptureError('fault_or_reset_observed')
            if command == 'CONFIG?':
                try:
                    validate_config(expected, fields, require_running=False)
                except ValueError:
                    authority.revoke()
                    raise
            return fields

        def request(mode, end):
            nonlocal attempted
            authority.require_command('WIFI=' + mode)
            command_deadline = min(end, time.monotonic() + 2.0)
            collector.lines.clear()  # Old bytes remain in the immutable raw log.
            attempted = True
            collector.send('WIFI=' + mode, command_deadline)
            line = collector.wait(lambda text: text.startswith(('ACK,', 'ERR,')) and
                                  parse_fields(text).get('cmd') == 'WIFI' and
                                  parse_fields(text).get('v') == mode and
                                  'accepted' in parse_fields(text), command_deadline, 'WIFI=' + mode)
            reply = parse_fields(line)
            same_wifi_boot(reply)
            scope = 'backend_and_gate' if mode == 'AP' else 'output_gate_only'
            accepted = reply.get('accepted')
            reply_error = int(reply.get('e', ''), 16)
            if (reply.get('scope') != scope or accepted not in {'0', '1'} or
                    (accepted == '1' and (not line.startswith('ACK,') or reply_error != 0)) or
                    (accepted == '0' and (not line.startswith('ERR,') or reply_error == 0))):
                raise CaptureError('wifi_setter_reply_contract_invalid')
            item = {'command': 'WIFI=' + mode, 'reply': reply, 'terminal': None}
            result['requests'].append(item)
            request_id = int(reply.get('id', '0'))
            if not 0 < request_id < (1 << 32):
                raise CaptureError('wifi_request_not_accepted:' + line)
            if request_id in seen_request_ids:
                raise CaptureError('wifi_request_id_reused_in_short_chain')
            seen_request_ids.add(request_id)
            epoch = int(reply.get('epoch', '-1'))
            if not 0 <= epoch < (1 << 32):
                raise CaptureError('wifi_ACK_epoch_invalid')
            terminal_deadline = min(end, time.monotonic() + 7.0)
            while time.monotonic() < terminal_deadline:
                fields = query('WIFIREQ?' + str(request_id), 'WIFIREQ', terminal_deadline)
                same_wifi_boot(fields)
                if fields.get('id') != str(request_id) or fields.get('known') != '1':
                    raise CaptureError('wifi_request_cache_identity_missing')
                if fields.get('requested') != mode:
                    raise CaptureError('wifi_request_cache_mode_mismatch')
                if fields.get('epoch') != str(epoch) or fields.get('scope') != scope:
                    raise CaptureError('wifi_request_terminal_epoch_or_scope_mismatch')
                if fields.get('outcome') in {'applied', 'failed', 'quarantined', 'superseded'}:
                    if fields['outcome'] == 'applied' and (accepted != '1' or
                            int(fields.get('err', ''), 16) != 0 or int(fields.get('cleanupErr', ''), 16) != 0):
                        raise CaptureError('wifi_applied_terminal_has_failure')
                    item['terminal'] = fields
                    return fields
                # Keep reading frames while waiting, inside the original deadline.
                collector.pump(min(terminal_deadline, time.monotonic() + 0.1))
            raise CaptureError('wifi_terminal_deadline')

        def allocation_trace(end):
            fields = query('MEMFAIL?', 'MEMFAIL', end)
            if (str(fields.get('session', '')).upper() != expected.session or
                    fields.get('bootId') != str(expected.boot_id) or fields.get('identityKnown') != '1' or
                    str(fields.get('elfSha256', '')).lower() != expected.elf_sha256):
                authority.revoke()
                raise CaptureError('allocation_trace_identity_mismatch')
            if any(fields.get(key) != value for key, value in
                   (('v', '1'), ('scope', 'boot'), ('regKnown', '1'), ('registered', '1'),
                    ('countScope', 'recorded'), ('coverage', 'since_register'), ('fn', 'opaque_pointer'))):
                raise CaptureError('allocation_trace_unavailable')
            return fields

        try:
            port = UsbCdcPort(a.usb_serial)
            collector = Collector(CaptureOptions(None, 45, directory, a.label,
                                  command_timeout=2, transport='usb_cdc', usb_serial=a.usb_serial),
                                  port, raw, events)
            collector.total_deadline = deadline
            collector.phase = 'preflight'
            opening = query('CONFIG?', 'CONFIG', work_deadline)
            build = query('BUILD?', 'BUILD', work_deadline)
            closing = query('CONFIG?', 'CONFIG', work_deadline)
            result['authority'] = authority.verify(build, opening, closing)
            if (closing['mode'], closing['rows']) != ('CAP', '8'):
                raise CaptureError('requires_initial_CAP8_no_mode_mutation')
            caps = query('CAPS2?', 'CAPS2', work_deadline)
            if 'wifi_owner_v1' not in caps.get('caps', '').split('/'):
                raise CaptureError('wifi_owner_capability_required')
            initial = query('WIFI?', 'ACK', work_deadline)
            same_wifi_boot(initial)
            result['initial_wifi'] = initial
            if initial.get('outputGate') != '0' or initial.get('pending') != '0':
                raise CaptureError('initial_wifi_not_idle_OFF')
            result['memory_before'] = query('MEM?', 'MEM', work_deadline)
            if a.allocation_trace:
                result['allocation_before'] = allocation_trace(work_deadline)
            collector.phase = 'wifi_request'
            first = request('AP', work_deadline)
            result['first_AP_terminal'] = first
            result['latest_AP_terminal'] = first
            if first['outcome'] == 'applied':
                # Same-profile repeat and OFF -> AP use new request identities.
                for mode in ('AP', 'OFF', 'AP'):
                    terminal = request(mode, work_deadline)
                    if mode == 'AP':
                        result['latest_AP_terminal'] = terminal
                    if terminal['outcome'] != 'applied':
                        raise CaptureError('wifi_repeat_or_gate_transition_failed:' + mode)
                result['status'] = 'AP_AND_CONTROL_QUERY_OBSERVED'
            else:
                result['status'] = 'BACKEND_UNAVAILABLE_CONTROL_QUERY_OBSERVED'
            result['guard_after'] = query('WIFIGUARD?', 'WIFIGUARD', work_deadline)
            same_wifi_boot(result['guard_after'])
            if any(result['guard_after'].get(k) not in {'0', '1'} for k in ('quarantined', 'active', 'ownerBusy')):
                raise CaptureError('wifi_guard_fields_unknown')
            if result['status'] == 'AP_AND_CONTROL_QUERY_OBSERVED':
                current = query('WIFI?', 'ACK', work_deadline)
                same_wifi_boot(current)
                result['last_AP_state'] = current
                if (result['guard_after']['quarantined'] != '0' or current.get('quarantined') != '0' or
                        current.get('backend') != 'ready' or current.get('outputGate') != '1' or
                        current.get('pending') != '0' or current.get('id') != result['latest_AP_terminal']['id']):
                    raise CaptureError('applied_AP_no_longer_available')
            result['memory_after'] = query('MEM?', 'MEM', work_deadline)
            if a.allocation_trace:
                result['allocation_after'] = allocation_trace(work_deadline)
            result['config_after'] = query('CONFIG?', 'CONFIG', work_deadline)
            if result['config_after'] != closing:
                raise CaptureError('configuration_changed_during_wifi_test')
        except (Exception, KeyboardInterrupt) as error:
            result.update(status='ERROR', error=str(error) or type(error).__name__)
        finally:
            if attempted and authority.verified:
                try:
                    collector.phase = 'restore_output_gate'
                    query('CONFIG?', 'CONFIG', deadline)
                    terminal = request('OFF', deadline)
                    state = query('WIFI?', 'ACK', deadline)
                    same_wifi_boot(state)
                    if terminal['outcome'] != 'applied' or state.get('outputGate') != '0':
                        raise CaptureError('OFF_not_verified')
                    if result['status'] == 'AP_AND_CONTROL_QUERY_OBSERVED' and state.get('quarantined') != '0':
                        raise CaptureError('wifi_quarantined_during_restore')
                    result['off_restore'] = 'SAME_BOOT_OUTPUT_GATE_QUERY_VERIFIED'
                    # The real firmware retains two request results. After
                    # this OFF, only the immediately preceding AP is required
                    # to remain cached; an earlier request may be not_cached.
                    original = result.get('latest_AP_terminal')
                    if original:
                        retained = query('WIFIREQ?' + original['id'], 'WIFIREQ', deadline)
                        same_wifi_boot(retained)
                        if any(retained.get(k) != original.get(k) for k in
                               ('known', 'id', 'requested', 'epoch', 'scope', 'outcome', 'err', 'cleanupErr')):
                            raise CaptureError('OFF_changed_original_AP_terminal')
                        result['latest_AP_after_OFF'] = retained
                        first = result['first_AP_terminal']
                        if first['id'] != original['id']:
                            expired = query('WIFIREQ?' + first['id'], 'WIFIREQ', deadline)
                            same_wifi_boot(expired)
                            if expired.get('known') != '0' or expired.get('reason') != 'not_cached':
                                raise CaptureError('two_entry_terminal_cache_eviction_not_observed')
                            result['first_AP_after_cache_eviction'] = expired
                except (Exception, KeyboardInterrupt) as error:
                    result['off_restore'] = 'FAILED_OR_UNKNOWN'
                    result['cleanup_error'] = str(error) or type(error).__name__
                    result['error'] = result['error'] or 'OFF_restore:' + result['cleanup_error']
                    result['status'] = 'ERROR'
            elif attempted:
                result['off_restore'] = 'NOT_SENT_AUTHORITY_REVOKED'
                result['status'] = 'ERROR'
            if collector:
                result.update(faults=collector.faults, resets=collector.reset_observations,
                              rx_bytes=collector.raw_bytes)
                collector.event('finished', status=result['status'], error=result['error'],
                                off_restore=result['off_restore'])
            if port:
                port.close()
            raw.flush()
    result.update(elapsed_seconds=time.monotonic() - started,
                  inputs={str(paths[k]): hashlib.sha256(paths[k].read_bytes()).hexdigest()
                          for k in ('raw', 'events')})
    with paths['summary'].open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps({'status': result['status'], 'error': result['error'], 'summary': str(paths['summary'])}))
    return 0 if result['status'] == 'AP_AND_CONTROL_QUERY_OBSERVED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
