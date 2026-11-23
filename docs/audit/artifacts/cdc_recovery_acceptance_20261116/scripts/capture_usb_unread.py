#!/usr/bin/env python3
"""Keep formal CDC0 open without application reads/writes for 120 seconds.

This is an unread-client stress capture, not USB receive-throughput evidence.
The kernel may continue accepting bytes. BLE/debug observation is independent.
Use a unique label; no input flush, USBSTREAM setter, reset, flash or NVS write.
Every operation, including after queries and CAP8 recovery, shares one budget.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack, contextmanager
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'docs/audit/artifacts/usb_otg_ble_priority_20261114/scripts'))
import capture_cdc_benchmark as cdc
import capture_reference_fixture as fixture
import run_firmware_benchmark as benchmark
import sensorarray_usb_cdc
from capture_reference_fixture import CaptureError, open_capture_port
from text_protocol import parse_fields

SERIAL = '80B54ECDD9AC'
RAIL_NOTE = ('Temporary RAM values from user DMM at ADS supply pins/decoupling: '
             'AVDD 3.33..3.37 V, AVSS -1.795 +/-0.005 V; selected 3.350/-1.795 V. '
             'Possible measurement-induced mains noise; not precision calibration.')
MEM_FIELDS = ('v readout unit intCaps intFree intLargest intMin extFree extLargest scan '
              'scanStack beforeFree beforeLargest afterFree afterLargest err sampleUs')
CONFIG_KEYS = benchmark.IDENTITY + ('boundary', 'state', 'mode', 'rows', 'profile', 'err')


@dataclass(frozen=True)
class Options:
    output_directory: Path
    label: str
    usb_serial: str = SERIAL
    hold_seconds: float = 120.0
    total_seconds: float = 200.0
    prepare_seconds: float = 35.0
    after_seconds: float = 15.0
    cleanup_seconds: float = 20.0
    command_timeout: float = 2.0
    stop_file: Path | None = None
    expected_elf_sha256: str | None = None


class Cancellation:
    def __init__(self):
        self.reason = None
        self.signals = []

    def signal(self, number, _frame=None):
        self.signals.append(number)
        self.reason = self.reason or 'signal:' + str(number)

    def check(self, stop_file):
        if stop_file is not None and Path(stop_file).exists():
            self.reason = self.reason or 'stop_file'
        if self.reason:
            raise CaptureError('interrupted:' + self.reason)


@contextmanager
def signal_scope(cancel):
    old = {number: signal.signal(number, cancel.signal) for number in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for number, handler in old.items():
            signal.signal(number, handler)


class AuditedPort:
    """One existing CDC owner; the hold cannot delegate any device I/O."""
    def __init__(self, port):
        self.port = port
        self.holding = False
        self.read_calls = self.write_calls = self.hold_read_calls = self.hold_write_calls = 0
        self.closed = False

    def metadata(self):
        return self.port.metadata()

    def read_some(self, size, deadline):
        self.read_calls += 1
        if self.holding:
            self.hold_read_calls += 1
            raise CaptureError('forbidden_read_during_unread_hold')
        return self.port.read_some(size, deadline)

    def write_some(self, data, deadline):
        self.write_calls += 1
        if self.holding:
            self.hold_write_calls += 1
            raise CaptureError('forbidden_write_during_unread_hold')
        return self.port.write_some(data, deadline)

    def close(self):
        self.closed = True
        return self.port.close()


def u32(value, key):
    return cdc.shared.unsigned(value, 'USB_UNREAD', key)


def require_target(config, mode, rows):
    expected = {'CAP': ('CAPACITANCE', 'C' * 8), 'VOLT': ('VOLTAGE', 'V' * 8)}[mode]
    if (config.get('state'), config.get('profile'), config.get('mode'), config.get('rows'), config.get('err')) != (
            *expected, mode, str(rows), '0x0'):
        raise CaptureError('configuration_target_unverified:' + str(config))


class UnreadCollector(cdc.CdcBenchmarkCollector):
    variant = 'formal_cdc_unread_hold_v1'
    runner_source_file = __file__
    query_requirements = {**cdc.CdcBenchmarkCollector.query_requirements,
                          'READY?': 'ready stage err bootId boot', 'MEM?': MEM_FIELDS}

    @classmethod
    def validate_fields(cls, command, fields):
        if command not in {'READY?', 'MEM?'}:
            result = super().validate_fields(command, fields)
            if command == 'CONFIG?':
                for key in (*benchmark.IDENTITY[1:], 'boundary'):
                    u32(fields.get(key), key)
            return result
        required = cls.query_requirements[command].split()
        if any(not fields.get(key) for key in required):
            raise cdc.shared.retry.InvalidQueryReply('incomplete_unread_query:' + command)
        if command == 'READY?':
            if fields['ready'] not in {'0', '1'} or not re.fullmatch(r'[A-Za-z0-9_]+', fields['stage']):
                raise cdc.shared.retry.InvalidQueryReply('invalid_READY_state')
            u32(fields['bootId'], 'bootId'); u32(fields['boot'], 'boot')
            if not re.fullmatch(r'0x[0-9a-fA-F]{1,8}', fields['err']):
                raise cdc.shared.retry.InvalidQueryReply('invalid_READY_error')
            return fields
        literals = {'v': '1', 'readout': 'sequential', 'unit': 'bytes', 'intCaps': 'internal8'}
        if any(fields[key] != value for key, value in literals.items()) or fields['scan'] not in {'unknown', 'pending', 'created', 'failed'}:
            raise cdc.shared.retry.InvalidQueryReply('invalid_MEM_semantics')
        for key in ('intFree', 'intLargest', 'intMin', 'extFree', 'extLargest'):
            u32(fields[key], key)
        for key in ('scanStack', 'beforeFree', 'beforeLargest', 'afterFree', 'afterLargest'):
            known = fields['scan'] != 'unknown' if key in {'scanStack', 'beforeFree', 'beforeLargest'} else fields['scan'] in {'created', 'failed'}
            if known:
                u32(fields[key], key)
            elif fields[key] != 'unknown':
                raise cdc.shared.retry.InvalidQueryReply('invalid_MEM_unknown:' + key)
        if fields['scan'] in {'created', 'failed'}:
            if not re.fullmatch(r'0x[0-9a-fA-F]{1,8}', fields['err']) or (int(fields['err'], 16) == 0) != (fields['scan'] == 'created'):
                raise cdc.shared.retry.InvalidQueryReply('invalid_MEM_scan_result')
        elif fields['err'] != 'unknown':
            raise cdc.shared.retry.InvalidQueryReply('invalid_MEM_unknown_error')
        cdc.shared.unsigned(fields['sampleUs'], command, 'sampleUs', 64)
        return fields

    def __init__(self, *args, limits, start, cancel, **kwargs):
        self.limits, self.cancel = limits, cancel
        self.reset_event_count = self.boot_change_events = 0
        self.fault_baseline = self.reset_baseline = self.boot_baseline = 0
        self.authority_verified = False
        self.boot_identity = self.build_identity = None
        self.last_reply = {}
        self.initial = {}
        self.snapshot_attempts = []
        self.hold = None
        self.cleanup_config = None
        super().__init__(*args, **kwargs)
        self.total_deadline = start + limits.total_seconds
        self.prepare_deadline = min(start + limits.prepare_seconds,
            self.total_deadline - limits.hold_seconds - limits.after_seconds - limits.cleanup_seconds)
        self.periodic_active = True  # Disable inherited periodic queries; only explicit brackets run.

    def event(self, kind, **fields):
        if kind == 'reset_observed':
            self.reset_event_count += 1  # Separate from the parent's bounded 16-example list.
        if kind == 'reply':
            self.last_reply[fields['command']] = {'line': str(fields['line']), 'received_monotonic': time.monotonic()}
        super().event(kind, **fields)

    def observe(self, line):
        super().observe(line)
        fields = parse_fields(line)
        if self.boot_identity and line.split(',', 1)[0] in {'C', 'V', 'R', 'M'} and 'seq' in fields:
            if any(fields.get(key) != self.boot_identity[key] for key in ('session', 'bootId')):
                self.boot_change_events += 1
                self.event('frame_boot_change', line=str(line), expected=self.boot_identity)

    def check_fault(self, baseline=0):
        if (self.fault_events > self.fault_baseline or self.reset_event_count > self.reset_baseline or
                self.boot_change_events > self.boot_baseline):
            raise CaptureError('firmware_fault_or_reset_during_' + self.phase)

    def pump(self, deadline):
        if self.phase not in {'after', 'cleanup'}:
            self.cancel.check(self.limits.stop_file)
        super().pump(deadline)
        self.check_fault()

    def query(self, command, tag, deadline):
        begin = time.monotonic()
        fields = super().query(command, tag, deadline)
        self.check_fault()
        self.last_reply[command].update(started_monotonic=begin, finished_monotonic=time.monotonic(), fields=dict(fields))
        return fields

    def verify_boot(self, config):
        if self.boot_identity and any(config.get(k) != v for k, v in self.boot_identity.items()):
            raise CaptureError('boot_identity_changed')

    def verify_build(self, fields):
        if fields.get('target') != 'esp32s3' or fields.get('project') != 'SensorArray':
            raise CaptureError('unexpected_BUILD_target_or_project')
        if self.limits.expected_elf_sha256 and fields['elfSha256'].lower() != self.limits.expected_elf_sha256.lower():
            raise CaptureError('unexpected_BUILD_ELF')
        if self.build_identity is not None and fields != self.build_identity:
            raise CaptureError('BUILD_changed')

    def verify_ready(self, ready, config):
        if ready['ready'] != '1' or int(ready['err'], 16) != 0 or ready['stage'] == 'unknown' or ready['bootId'] != config['bootId']:
            raise CaptureError('READY_not_verified_for_CONFIG_boot')

    def prepare(self):
        self.phase = 'prepare'
        self.cancel.check(self.limits.stop_file)
        for command in ('USB?', 'BUILD?', 'PROTO?', 'READY?', 'CONFIG?'):
            self.initial[command] = self.query(command, command[:-1], self.prepare_deadline)
        self.verify_build(self.initial['BUILD?'])
        self.verify_ready(self.initial['READY?'], self.initial['CONFIG?'])
        self.build_identity = dict(self.initial['BUILD?'])
        self.boot_identity = {key: self.initial['CONFIG?'][key] for key in ('session', 'bootId')}
        self.expected_identity = self.initial['CONFIG?']
        self.authority_verified = True
        if self.expected_identity.get('mode') != 'CAP' or self.expected_identity.get('profile') != 'CCCCCCCC':
            self.transition('MODE', 'CAP', self.prepare_deadline)
        self.verify_cap(self.prepare_deadline)
        self.ensure_rail()  # Existing bounded RAILCFG RAM path; no CAL=SAVE.
        if self.expected_identity.get('rows') != '1':
            self.transition('ROWS', '1', self.prepare_deadline)
        self.transition('MODE', 'VOLT', self.prepare_deadline)
        require_target(self.expected_identity, 'VOLT', 1)
        self.snapshot('before', self.prepare_deadline)

    def snapshot(self, name, deadline):
        self.phase = name
        entry = {'started_monotonic': time.monotonic(), 'deadline': min(deadline, self.total_deadline), 'queries': {}, 'outcome': 'INCOMPLETE'}
        self.snapshots[name] = entry
        commands = ['CONFIG?', 'BUILD?', 'READY?', 'PROTO?', 'USB?', 'USBQ?', 'USBSTREAM?', 'FPS?', 'MODE?', 'RAIL?']
        for command, capability in (('BLEFLOW?', 'ble_flow_v1'), ('BLESNAP?', 'ble_snapshot_v1'), ('MEM?', 'heap_v1')):
            if capability in self.reported_capabilities:
                commands.append(command)
            else:
                entry['queries'][command] = {'status': 'CAPABILITY_UNAVAILABLE', 'capability': capability}
        try:
            for command in commands + ['CONFIG?']:
                fields = self.query(command, 'ARL' if command == 'RAIL?' else command[:-1], deadline)
                key = 'CONFIG?_closing' if command == 'CONFIG?' and 'CONFIG?' in entry['queries'] else command
                entry['queries'][key] = dict(self.last_reply[command])
            q = entry['queries']; opening, closing = q['CONFIG?']['fields'], q['CONFIG?_closing']['fields']
            self.verify_boot(opening); self.verify_boot(closing)
            if any(opening.get(k) != closing.get(k) or closing.get(k) != self.expected_identity.get(k) for k in CONFIG_KEYS):
                raise CaptureError('CONFIG_bracket_or_expected_identity_changed')
            require_target(closing, 'VOLT', 1)
            self.verify_build(q['BUILD?']['fields']); self.verify_ready(q['READY?']['fields'], closing)
            fps, mode = q['FPS?']['fields'], q['MODE?']['fields']
            if fps['cfcap'] != '0' or fps['ofcap'] != '0':
                raise CaptureError('existing_FPS_limit_no_automatic_change')
            if (mode.get('state'), mode.get('active'), mode.get('pending'), mode.get('route'),
                    mode.get('profile'), mode.get('intref'), mode.get('refmux')) != (
                    'VOLTAGE', 'VOLT', 'NONE', 'VOLT', 'VVVVVVVV', '0', '24'):
                raise CaptureError('VOLT_reference_state_unverified')
            if not self.rail_valid(q['RAIL?']['fields']):
                raise CaptureError('external_rail_no_longer_valid')
            for a, b in (('BLEFLOW?', 'BLESNAP?'),):
                if 'fields' in q[a] and 'fields' in q[b] and any(q[a]['fields'][k] != q[b]['fields'][k] for k in ('gen', 'stream')):
                    raise CaptureError('BLE_epoch_changed_within_snapshot')
            entry['outcome'] = 'COHERENT_QUERY_VERIFIED'
        finally:
            entry['finished_monotonic'] = time.monotonic()

    def hold_unread(self, *, sleeper=time.sleep):
        self.cancel.check(self.limits.stop_file)
        start = time.monotonic()
        end = start + self.limits.hold_seconds
        if end > self.total_deadline - self.limits.after_seconds - self.limits.cleanup_seconds:
            raise CaptureError('insufficient_full_unread_hold_budget')
        self.phase = 'unread_hold'
        self.hold = {'started_monotonic': start, 'requested_seconds': self.limits.hold_seconds,
                     'deadline_monotonic': end, 'raw_offset_before': self.raw_bytes,
                     'read_calls_before': self.port.read_calls, 'write_calls_before': self.port.write_calls,
                     'status': 'INCOMPLETE', 'FD_kept_open': True}
        self.event('unread_hold_start', **self.hold)
        self.port.holding = True
        try:
            while time.monotonic() < end:
                self.cancel.check(self.limits.stop_file)
                sleeper(max(0.0, min(.1, end - time.monotonic())))
            self.cancel.check(self.limits.stop_file)
            self.hold['status'] = 'COMPLETE'
        finally:
            finished = time.monotonic()
            self.port.holding = False
            self.hold.update(finished_monotonic=finished, actual_seconds=finished-start,
                raw_offset_after=self.raw_bytes, read_calls=self.port.read_calls-self.hold['read_calls_before'],
                write_calls=self.port.write_calls-self.hold['write_calls_before'],
                forbidden_read_calls=self.port.hold_read_calls, forbidden_write_calls=self.port.hold_write_calls,
                USB_PC_receipt='NOT_OBSERVED', USB_PC_FPS=None,
                kernel_buffering='possible; later draining is not hold-time PC receipt',
                BLE_debug_evidence='requires_independent_observers')
            self.event('unread_hold_end', **self.hold)

    def cleanup(self):
        self.phase = 'cleanup'
        self.fault_baseline, self.reset_baseline, self.boot_baseline = self.fault_events, self.reset_event_count, self.boot_change_events
        if not self.authority_verified:
            self.safe_cap_recovery = 'UNKNOWN_AUTHORITY_UNVERIFIED'
            return
        self.safe_cap_recovery = 'UNKNOWN_ATTEMPTED'
        deadline = min(self.total_deadline, time.monotonic() + self.limits.cleanup_seconds)
        config = self.query('CONFIG?', 'CONFIG', deadline)
        self.verify_boot(config)  # Never use a previous boot's request IDs.
        self.expected_identity = config
        if config.get('mode') != 'CAP' or config.get('profile') != 'CCCCCCCC' or config.get('state') != 'CAPACITANCE':
            self.transition('MODE', 'CAP', deadline)
        if self.expected_identity.get('rows') != '8':
            self.transition('ROWS', '8', deadline)
        self.verify_cap(deadline)
        self.cleanup_config = self.query('CONFIG?', 'CONFIG', deadline)
        self.verify_boot(self.cleanup_config); require_target(self.cleanup_config, 'CAP', 8)
        self.verify_build(self.query('BUILD?', 'BUILD', deadline))
        self.verify_ready(self.query('READY?', 'READY', deadline), self.cleanup_config)
        self.check_fault()
        self.safe_cap_recovery = 'CAP8_QUERY_VERIFIED'


def validate_options(o):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', o.label) or not re.fullmatch(r'[0-9A-Fa-f]{12}', o.usb_serial):
        raise ValueError('unique_simple_label_and_exact_serial_required')
    for key in ('hold_seconds', 'total_seconds', 'prepare_seconds', 'after_seconds', 'cleanup_seconds', 'command_timeout'):
        if not math.isfinite(getattr(o, key)) or getattr(o, key) <= 0:
            raise ValueError('invalid_deadline:' + key)
    if o.total_seconds > 300 or o.hold_seconds + o.prepare_seconds + o.after_seconds + o.cleanup_seconds > o.total_seconds:
        raise ValueError('total_budget_must_include_hold_and_reserved_query_cleanup')
    if o.expected_elf_sha256 and not re.fullmatch(r'[0-9A-Fa-f]{64}', o.expected_elf_sha256):
        raise ValueError('expected_ELF_must_be_sha256')


def run_capture(options, *, port_factory=open_capture_port, sleeper=time.sleep, cancel=None):
    validate_options(options)
    cancel = cancel or Cancellation()
    directory = Path(options.output_directory).resolve(); directory.mkdir(parents=True, exist_ok=True)
    paths = benchmark.record_paths(directory, options.label)
    for path in paths.values():
        if path.exists() or Path(str(path)+'.gz').exists():
            raise FileExistsError(path)
    start = time.monotonic()
    sources = (Path(__file__), Path(cdc.__file__), Path(cdc.shared.__file__), Path(cdc.shared.retry.__file__),
               Path(benchmark.__file__), Path(fixture.__file__), Path(sensorarray_usb_cdc.__file__))
    result = {'status': 'ERROR', 'error': None, 'cleanup_error': None, 'after_error': None,
        'evidence': 'LIVE_HIL_ATTEMPT' if port_factory is open_capture_port else 'INJECTED_TEST_PORT',
        'options': {**asdict(options), 'output_directory': str(directory), 'stop_file': str(options.stop_file) if options.stop_file else None},
        'raw_file': str(paths['raw']), 'events_file': str(paths['events']), 'summary_file': str(paths['summary']),
        'input_flushed': False, 'NVS_written': False, 'reset_requested': False, 'USBSTREAM_setter': False,
        'rail_source_note': RAIL_NOTE, 'USB_hold_PC_FPS': None, 'USB_hold_PC_receipt': 'NOT_OBSERVED',
        'scientific_accuracy': 'NOT_ESTABLISHED', 'acceptance': 'NOT_EVALUATED',
        'source_sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}
    result['HEAD'] = subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True, timeout=3).strip()
    result['tracked_dirty_diff_sha256'] = hashlib.sha256(subprocess.check_output(['git', 'diff', '--binary'], cwd=ROOT, timeout=3)).hexdigest()
    dirty = subprocess.check_output(['git', 'status', '--porcelain=v1', '-uno'], cwd=ROOT, timeout=3)
    result['tracked_working_tree_dirty'] = bool(dirty)
    result['tracked_status_sha256'] = hashlib.sha256(dirty).hexdigest()
    settings = benchmark.BenchmarkOptions(None, directory, options.label, mode='VOLT', rows=1,
        capture_seconds=options.hold_seconds, warmup_seconds=0, prepare_seconds=options.prepare_seconds,
        cleanup_seconds=options.cleanup_seconds, command_timeout=options.command_timeout,
        rail_avdd_uv=3350000, rail_avss_uv=-1795000, rail_source_note=RAIL_NOTE, transport='usb_cdc', usb_serial=options.usb_serial)
    config = benchmark.Configuration(options.label, 'VOLT', 1, 'CVRCVRCV', options.hold_seconds, 'USB_UNREAD')
    collector = port = None
    with ExitStack() as stack, signal_scope(cancel), cdc.shared.bulk_collector_selected(UnreadCollector):
        raw = benchmark.HashingRaw(stack.enter_context(paths['raw'].open('xb')))
        events = stack.enter_context(paths['events'].open('x', encoding='utf-8'))
        summary = stack.enter_context(paths['summary'].open('x', encoding='utf-8'))
        try:
            cancel.check(options.stop_file)
            port = AuditedPort(port_factory(settings))
            collector = UnreadCollector(settings, config, port, raw, events, limits=options, start=start, cancel=cancel)
            collector.prepare(); collector.hold_unread(sleeper=sleeper)
        except (Exception, KeyboardInterrupt) as error:
            result['error'] = f'{type(error).__name__}:{error}'
        finally:
            if collector:
                # Reserve recovery even if end queries time out. Old faults
                # remain failures, but must not block the safe recovery attempt.
                if collector.authority_verified:
                    collector.fault_baseline, collector.reset_baseline, collector.boot_baseline = collector.fault_events, collector.reset_event_count, collector.boot_change_events
                    try:
                        collector.snapshot('after', min(collector.total_deadline-options.cleanup_seconds,
                            time.monotonic()+options.after_seconds))
                    except (Exception, KeyboardInterrupt) as error:
                        result['after_error'] = f'{type(error).__name__}:{error}'
                        result['error'] = result['error'] or result['after_error']
                try:
                    collector.cleanup()
                except (Exception, KeyboardInterrupt) as error:
                    result['cleanup_error'] = f'{type(error).__name__}:{error}'
                    result['error'] = result['error'] or result['cleanup_error']
                    collector.safe_cap_recovery = 'FAILED_OR_UNKNOWN'
                result.update(initial=collector.initial, snapshots=collector.snapshots, hold=collector.hold,
                    transitions=collector.transitions, safe_cap_recovery=collector.safe_cap_recovery,
                    cleanup_CONFIG=collector.cleanup_config, query_attempts=collector.query_attempts,
                    fault_events=collector.fault_events, faults=list(collector.faults),
                    reset_event_count=collector.reset_event_count, reset_observations=collector.reset_observations,
                    frame_boot_change_events=collector.boot_change_events,
                    volatile_rail_written=collector.volatile_rail_written, rx_bytes=collector.raw_bytes,
                    trailing_partial_line_bytes=len(collector.line_buffer), **collector.transport_result())
                if collector.fault_events or collector.reset_event_count or collector.boot_change_events:
                    result['error'] = result['error'] or 'firmware_fault_or_reset_observed'
            if port:
                try:
                    port.close()
                except Exception as error:
                    result['close_error'] = str(error); result['error'] = result['error'] or 'port_close_failed'
            result.update(raw_sha256=raw.sha.hexdigest(), elapsed_seconds=time.monotonic()-start,
                          total_deadline_monotonic=start+options.total_seconds, signals=cancel.signals,
                          stop_reason=cancel.reason)
            hold = result.get('hold') or {}
            if (not result['error'] and result.get('safe_cap_recovery') == 'CAP8_QUERY_VERIFIED' and
                    hold.get('status') == 'COMPLETE' and hold.get('read_calls') == hold.get('write_calls') == 0 and
                    hold.get('actual_seconds', 0) >= options.hold_seconds):
                result['status'] = 'UNREAD_HOLD_COMPLETED_RECOVERY_VERIFIED'
            if collector:
                collector.event('finished', status=result['status'], error=result['error'])
            json.dump(result, summary, indent=2); summary.write('\n'); summary.flush()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--label', required=True)
    parser.add_argument('--usb-serial', default=SERIAL)
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--expected-elf-sha256')
    args = parser.parse_args(argv)
    result = run_capture(Options(**vars(args)))
    print(json.dumps({k: result.get(k) for k in ('status', 'error', 'cleanup_error', 'hold', 'safe_cap_recovery', 'summary_file')}))
    return 0 if result['status'] == 'UNREAD_HOLD_COMPLETED_RECOVERY_VERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
