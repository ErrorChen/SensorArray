#!/usr/bin/env python3
"""Single USBBOOT for the identified scan_task_create_failed/0x101 early state.

Requires the caller's complete expected application ELF SHA256 and live
BUILD/PROTO/BOOT/READY/RESTART/USB identity. CONFIG is unavailable in this known
startup failure; this is not a general bypass of the normal recovery gate.
One deadline covers queries, the one action attempt and sysfs observation.
No esptool, flash, modem reset, input flush, installation or action retry.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import time
from dataclasses import dataclass

import check_software_recovery as shared

SERIAL = shared.SERIAL
CaptureError = shared.capture.CaptureError


@dataclass(frozen=True)
class Options:
    output_directory: Path
    label: str
    expected_elf: str
    duration: float = 60.0
    query_timeout: float = 2.0
    poll_seconds: float = 0.1


def require(condition, message):
    if not condition:
        raise CaptureError(message)


def uint32(value, name):
    require(isinstance(value, str) and re.fullmatch(r'[0-9]+', value) is not None and
            int(value) <= 0xFFFFFFFF, 'invalid_unsigned_field:' + name)
    return int(value)


def validate(replies, expected_elf):
    build, proto, boot, ready, restart, usb = (replies[name] for name in
        ('BUILD?', 'PROTO?', 'BOOT?', 'READY?', 'RESTART?', 'USB?'))
    require(all(build.get(key) for key in ('idf', 'target', 'project', 'proto', 'version', 'elfSha256')),
            'BUILD_identity_incomplete')
    require(build['target'] == 'esp32s3' and build['project'] == 'SensorArray' and build['proto'] == '2',
            'BUILD_target_project_protocol_mismatch')
    require(re.fullmatch(r'[0-9A-Fa-f]{64}', build['elfSha256']) is not None and
            build['elfSha256'].lower() == expected_elf.lower(), 'BUILD_full_ELF_hash_mismatch')
    caps = set(re.split(r'[/;,|\s]+', proto.get('caps', '')))
    require(proto.get('version') == '2' and
            {'usb_rom_handoff_v1', 'usb_cdc_data_v1', 'restart_query_v1', 'session64'} <= caps,
            'required_early_recovery_capabilities_unavailable')
    for name, fields in (('BOOT', boot), ('READY', ready)):
        require(fields.get('stage') == 'scan_task_create_failed' and fields.get('ready') == '0',
                name + '_not_the_authorized_early_failure')
        require(re.fullmatch(r'0x[0-9a-fA-F]+', fields.get('err', '')) is not None and
                int(fields['err'], 16) == 0x101, name + '_error_not_ESP_ERR_NO_MEM_0x101')
        uint32(fields.get('bootId'), name + '.bootId')
        uint32(fields.get('boot'), name + '.boot')
    require(uint32(boot.get('seq'), 'BOOT.seq') == 0, 'BOOT_has_measurement_progress_not_early_failure')
    require(boot['boot'] == ready['boot'], 'BOOT_READY_boot_count_mismatch')
    session = restart.get('session', '')
    require(re.fullmatch(r'[0-9a-fA-F]{16}', session) is not None and int(session, 16) != 0,
            'RESTART_session_unavailable')
    uint32(restart.get('bootId'), 'RESTART.bootId')
    uint32(restart.get('lastId'), 'RESTART.lastId')
    require(restart.get('busy') == '0' and restart.get('pendingId') == '0', 'restart_already_pending')
    require(boot['bootId'] == ready['bootId'] == restart['bootId'], 'BOOT_READY_RESTART_boot_identity_mismatch')
    require(usb.get('backend') == 'otg_cdc_v1' and
            all(usb.get(key) == '1' for key in ('init', 'mounted', 'romAvailable')),
            'formal_CDC_ROM_handoff_state_unavailable')
    require(usb.get('err') in {'0x0', '0x00', '0x00000000'} and usb.get('suspended') == '0',
            'USB_error_or_suspended')


def observe(options, *, inventory=shared.usb_inventory, port_factory=shared.UsbCdcPort,
            clock=time.monotonic, sleep=time.sleep, sysfs_exists=lambda path: Path(path).exists()):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', options.label):
        raise ValueError('invalid label')
    if not re.fullmatch(r'[0-9A-Fa-f]{64}', options.expected_elf):
        raise ValueError('expected-elf requires the full 64-hex application ELF SHA256')
    if (not all(math.isfinite(value) for value in (options.duration, options.query_timeout, options.poll_seconds)) or
            not (0 < options.duration <= 60 and 0 < options.query_timeout <= 3 and .05 <= options.poll_seconds <= .5)):
        raise ValueError('invalid single-deadline timing budget')
    out = options.output_directory
    out.mkdir(parents=True, exist_ok=True)
    if any(out.glob(options.label + '.*')):
        raise FileExistsError('existing evidence; choose a new label')
    paths = {kind: out / (options.label + suffix) for kind, suffix in
             (('raw', '.rx.bin'), ('events', '.events.jsonl'), ('summary', '.summary.json'))}
    started = clock()
    deadline = started + options.duration
    port = collector = None
    result = {'schema': 'identified_early_scan_failure_usbboot_v1', 'status': 'ERROR',
              'expected_elf_sha256': options.expected_elf.lower(), 'usb_serial': SERIAL,
              'deadline_seconds': options.duration, 'action_attempts': 0,
              'old_epoch_absent_monotonic': None, 'old_interface_absent_monotonic': None,
              'ROM_port_opened': False, 'esptool_executed': False, 'flash_written': False,
              'system_USB_reset_requested': False, 'automatic_action_retry': False,
              'input_flushed': False, 'CONFIG_requested': False,
              'RACK_RAPP_semantics': 'local publications only; not enumeration, ROM MAC or flasher proof',
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'shared_helper_sha256': hashlib.sha256(Path(shared.__file__).read_bytes()).hexdigest()}
    with paths['raw'].open('xb') as raw, paths['events'].open('x') as events, paths['summary'].open('x') as summary, shared.collector_clock(clock):
        def event(kind, **fields):
            events.write(json.dumps({'type': kind, 'monotonic': clock(), 't': clock() - started, **fields}) + '\n')
            events.flush()

        def close_reader(reason):
            nonlocal port
            if port is not None:
                closing, port = port, None
                try:
                    closing.close()
                    event('reader_closed', reason=reason)
                except Exception as error:
                    result['reader_close_error'] = repr(error)
                    event('reader_close_failed', reason=reason, error=repr(error))

        try:
            initial = inventory()
            event('inventory_initial', devices=initial)
            matching = [device for device in initial if device.get('serial') == SERIAL]
            require(len(matching) == 1 and shared.formal_role(matching[0]), 'unique_authorized_formal_CDC0_not_enumerated')
            old = matching[0]
            old_key = shared.epoch_key(old)
            baseline_keys = {shared.epoch_key(device) for device in initial}
            old_interface = next(interface['sysfs_path'] for interface in old['interfaces'] if interface['number'] == '00')
            result['old_device'] = old
            require(clock() < deadline, 'deadline_before_open')
            port = port_factory(SERIAL)
            metadata = port.metadata()
            require(metadata.get('transport') == 'usb_cdc' and shared.normalize_serial(metadata.get('usb_serial')) == SERIAL and
                    metadata.get('interface_number') == 0 and metadata.get('sysfs_device') == old['sysfs_path'] and
                    metadata.get('sysfs_interface') == old_interface, 'opened_port_identity_or_role_mismatch')
            require(shared.same_formal_epoch(old, inventory()), 'enumeration_epoch_changed_during_open')
            result['transport_metadata'] = metadata
            collector = shared.EpochCollector(shared.capture.CaptureOptions(None, max(.001, deadline-clock()), out,
                options.label, command_timeout=options.query_timeout, transport='usb_cdc', usb_serial=SERIAL),
                port, raw, events, writer=event, clock=clock, epoch=0)
            collector.total_deadline = deadline
            collector.phase = 'identified_early_failure_preflight'
            replies = {}
            for command, tag in (('BUILD?', 'BUILD'), ('PROTO?', 'PROTO'), ('RESTART?', 'RESTART'),
                                 ('BOOT?', 'BOOT'), ('READY?', 'READY'), ('USB?', 'USB')):
                replies[command] = collector.query(command, tag, deadline)
            validate(replies, options.expected_elf)
            result['before'] = replies
            closing = dict(replies)
            for command, tag in (('BOOT?', 'BOOT'), ('READY?', 'READY'), ('RESTART?', 'RESTART')):
                closing[command] = collector.query(command, tag, deadline)
            validate(closing, options.expected_elf)
            require(all(closing['RESTART?'].get(key) == replies['RESTART?'].get(key) for key in ('session', 'bootId')),
                    'boot_session_changed_during_preflight')
            require(closing['BOOT?']['boot'] == replies['BOOT?']['boot'], 'boot_count_changed_during_preflight')
            result['closing'] = closing
            require(shared.same_formal_epoch(old, inventory()), 'enumeration_epoch_changed_before_action')
            require(clock() < deadline, 'deadline_before_action')
            collector.phase = 'single_usbboot_action'
            result.update(action_attempts=1, action_started_monotonic=clock())
            event('single_action_attempt', command='USBBOOT')
            accepted_before = collector.tx_bytes
            try:
                collector.send('USBBOOT', min(deadline, clock() + options.query_timeout))
                result['action_host_write'] = 'all_bytes_accepted_by_OS_only'
            except (OSError, CaptureError) as error:
                result.update(action_host_write='outcome_unknown', action_write_error=str(error))
            result['action_known_host_accepted_bytes'] = collector.tx_bytes - accepted_before
            polls = 0
            while clock() < deadline and polls < math.ceil(options.duration / options.poll_seconds) + 1:
                iteration = clock()
                polls += 1
                devices = inventory()
                event('inventory_poll', poll=polls, devices=devices)
                keys = {shared.epoch_key(device) for device in devices}
                if old_key not in keys and result['old_epoch_absent_monotonic'] is None:
                    result['old_epoch_absent_monotonic'] = clock()
                    event('old_epoch_absent')
                    close_reader('old_epoch_absent_in_sysfs')
                if not sysfs_exists(old_interface) and result['old_interface_absent_monotonic'] is None:
                    result['old_interface_absent_monotonic'] = clock()
                    event('old_interface_absent')
                targets = [device for device in devices if device.get('serial') == SERIAL and
                           device.get('vid') == '303a' and device.get('pid') == '1001' and device.get('ttys') and
                           shared.epoch_key(device) not in baseline_keys and
                           (device['busnum'], device['devnum']) != (old['busnum'], old['devnum'])]
                if (result['old_epoch_absent_monotonic'] is not None and result['old_interface_absent_monotonic'] is not None and
                        len(targets) == 1 and not result.get('reader_close_error')):
                    result.update(status='ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED', new_device=targets[0],
                                  new_enumeration_monotonic=clock())
                    break
                if port is not None:
                    try:
                        collector.pump(min(deadline, clock() + options.poll_seconds / 2))
                        collector.lines.clear()
                    except (OSError, CaptureError) as error:
                        event('reader_interrupted', error=str(error))
                        close_reader('reader_interrupted')
                if clock() < deadline:
                    sleep(max(0, min(options.poll_seconds - (clock() - iteration), deadline - clock())))
            result['polls'] = polls
            if result['status'] != 'ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED':
                result['status'] = 'DEADLINE_WITHOUT_REQUIRED_ROM_ENUMERATION'
        except (Exception, KeyboardInterrupt) as error:
            result.update(status='ERROR', error=str(error) or 'interrupted')
            event('error', reason=result['error'])
        finally:
            close_reader('finished')
            if result.get('reader_close_error'):
                result['status'] = 'ERROR'
            raw.flush()
            result.update(raw_file=str(paths['raw']), events_file=str(paths['events']),
                          raw_sha256=hashlib.sha256(paths['raw'].read_bytes()).hexdigest(),
                          rx_bytes=collector.raw_bytes if collector else 0,
                          tx_host_accepted_bytes=collector.tx_bytes if collector else 0,
                          partial_writes=collector.partial_writes if collector else 0,
                          trailing_partial_line_bytes=len(collector.line_buffer) if collector else 0,
                          publications=list(collector.publications) if collector else [],
                          replies=dict(collector.replies) if collector else {},
                          elapsed_seconds=clock() - started)
            event('finished', status=result['status'], action_attempts=result['action_attempts'])
            summary.write(json.dumps(result, indent=2) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--label', required=True)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--expected-elf', required=True)
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args(argv)
    result = observe(Options(**vars(args)))
    print(json.dumps({key: result.get(key) for key in ('status', 'error', 'action_attempts', 'elapsed_seconds')}), flush=True)
    return 0 if result['status'] == 'ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
