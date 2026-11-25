#!/usr/bin/env python3
"""Bounded 50-transition HIL capture through the formal CDC0 owner.

Uses the current benchmark transition/CONFIG recovery implementation, never the
legacy fixture MODE helper. Exactly 50 test changes follow CAP8 preparation;
preparation and necessary CAP8 cleanup are recorded separately. A complete,
CRC-valid frame proves its wire/configuration identity, not point accuracy.
Raw records are exclusive-create; neither device input nor prior files are
flushed. No reset, USBSTREAM setter, speed command, CAL=SAVE or NVS write.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import capture_cdc_benchmark as cdc
import capture_reference_fixture
import run_firmware_benchmark as benchmark
from fixture_rail import add_rail_arguments, rail_arguments
import sensorarray_usb_cdc
import text_protocol
from capture_reference_fixture import CaptureError, open_capture_port
from text_protocol import TextProtocolParser, parse_fields

PROFILE = 'CVRCVRCV'
PLAN = tuple(('VOLT', 'CAP', 'RES', 'CAP', 'MIXED', 'CAP')[i % 6] for i in range(50))
CONFIG_KEYS = benchmark.IDENTITY + ('boundary', 'state', 'mode', 'rows', 'profile', 'err')


@dataclass(frozen=True)
class Options:
    output_directory: Path
    label: str
    usb_serial: str
    total_seconds: float = 300.0
    prepare_seconds: float = 30.0
    cleanup_seconds: float = 10.0
    command_timeout: float = 2.0
    frame_timeout: float = 3.0
    rail_avdd_uv: int | None = None
    rail_avss_uv: int | None = None
    rail_source_note: str | None = None
    expected_elf_sha256: str | None = None
    expected_session: str | None = None
    expected_boot_id: int | None = None


def unsigned(value, field):
    if not re.fullmatch(r'[0-9]+', value or '') or int(value) >= 1 << 32:
        raise CaptureError('invalid_u32:' + field)
    return int(value)


def forward_or_equal(value, reference):
    """RFC1982-style half-range relation, used only inside one boot/session."""
    return ((value - reference) & 0xffffffff) < 0x80000000


def require_config(config, mode=None):
    for key in (*benchmark.IDENTITY[1:], 'boundary'):
        unsigned(config.get(key), key)
    if config.get('state') not in {'CAPACITANCE', 'VOLTAGE', 'RESISTANCE'} or config.get('err') != '0x0':
        raise CaptureError('configuration_not_running:' + str(config))
    if mode is not None:
        profile = PROFILE if mode == 'MIXED' else {'CAP': 'C', 'VOLT': 'V', 'RES': 'R'}[mode] * 8
        if (config.get('mode'), config.get('rows'), config.get('profile')) != (mode, '8', profile):
            raise CaptureError('configuration_target_mismatch:' + str(config))
        expected_state = {'CAP': 'CAPACITANCE', 'VOLT': 'VOLTAGE', 'RES': 'RESISTANCE'}.get(mode)
        if expected_state is not None and config.get('state') != expected_state:
            raise CaptureError('configuration_state_mode_mismatch:' + str(config))


class ModeSwitchCollector(cdc.CdcBenchmarkCollector):
    def __init__(self, settings, configuration, port, raw, events, *, limits, start):
        self.limits = limits
        self.authority_verified = False
        self.cleanup_fault_baseline = self.cleanup_reset_baseline = 0
        self.steps = []
        self.initial = {}
        self.first_frame = None
        self.frame_parser = None
        self.frame_error = None
        self.stale_frames = 0
        self.stale_examples = []
        self.matching_frames = 0
        self.cleanup_config = None
        super().__init__(settings, configuration, port, raw, events)
        # One shared monotonic budget includes opening/preparation and reserves
        # cleanup. Benchmark periodic windows are unnecessary for this harness.
        self.total_deadline = start + limits.total_seconds
        self.work_deadline = self.total_deadline - limits.cleanup_seconds
        self.prepare_deadline = min(start + limits.prepare_seconds, self.work_deadline)
        self.periodic_active = True
        self.event('mode_switch_plan', modes=list(PLAN), rows=8, profile=PROFILE,
                   total_deadline=self.total_deadline, work_deadline=self.work_deadline,
                   rail_source_note=limits.rail_source_note, scientific_accuracy='NOT_ESTABLISHED')

    def check_fault(self, baseline=0):
        fault_base = max(baseline, self.cleanup_fault_baseline) if self.phase == 'cleanup' else 0
        reset_base = self.cleanup_reset_baseline if self.phase == 'cleanup' else 0
        if self.fault_events > fault_base or len(self.reset_observations) > reset_base:
            raise CaptureError('firmware_fault_or_reset_during_' + self.phase)

    def pump(self, deadline):
        super().pump(deadline)
        self.check_fault()
        if self.phase == 'capture' and self.frame_error:
            raise CaptureError(self.frame_error)

    def observe(self, line):
        super().observe(line)  # Current CDC observation/provenance handling.
        if self.phase != 'capture' or self.frame_parser is None:
            return
        fields, tag = parse_fields(line), line.split(',', 1)[0]
        if tag == 'M' and 'seq' not in fields:
            return
        if tag in {'C', 'V', 'R', 'M'} and 'seq' in fields:
            self.frame_header, self.frame_tag = fields, tag
        frame = self.frame_parser.feed_line(line)
        if frame is None:
            return
        self.parsed_frames += 1
        self.crc_valid_frames += int(frame.crc_ok)
        header, target = self.frame_header or {}, self.expected_identity
        try:
            if not frame.crc_ok:
                raise CaptureError('first_frame_crc_failed')
            if any(header.get(k) != target.get(k) for k in ('session', 'bootId')):
                raise CaptureError('frame_boot_changed')
            if any(fields.get(k) != header.get(k) for k in ('session', 'bootId', 'rev')):
                raise CaptureError('frame_header_trailer_identity_mismatch')
            rev, desired = unsigned(header.get('rev'), 'rev'), unsigned(target.get('rev'), 'rev')
            if rev != desired:
                if forward_or_equal(desired, rev):
                    self.stale_frames += 1
                    if len(self.stale_examples) < 8:
                        self.stale_examples.append(dict(header))
                    self.event('queued_old_frame', header=header, expected_revision=desired)
                    return  # Retain original identity; do not relabel an old queued frame.
                raise CaptureError('unexpected_future_frame_revision')
            if any(header.get(k) != target.get(k) for k in benchmark.IDENTITY):
                raise CaptureError('frame_config_identity_mismatch')
            if frame.rows != 8:
                raise CaptureError('frame_rows_mismatch')
            if not forward_or_equal(unsigned(str(frame.sequence), 'seq'), unsigned(target.get('boundary'), 'boundary')):
                raise CaptureError('frame_before_applied_boundary')
            expected_tag = {'CAP': 'C', 'VOLT': 'V', 'RES': 'R', 'MIXED': 'M'}[self.configuration.mode]
            if self.frame_tag != expected_tag:
                raise CaptureError('frame_mode_mismatch')
            if expected_tag == 'M':
                legacy = {k: target[k] for k in ('rgen', 'rrid', 'pgen', 'prid')}
                if frame.profile != PROFILE or set(frame.row_frames) != set(range(1, 9)):
                    raise CaptureError('frame_profile_mismatch')
                for row, char in enumerate(PROFILE, 1):
                    expected = {'C': ('CAP', 'pF', -6), 'V': ('VOLT', 'V', -6), 'R': ('RES', 'ohm', -3)}[char]
                    actual = frame.row_frames[row]
                    if (actual.mode, actual.unit, actual.scale) != expected:
                        raise CaptureError('frame_mixed_row_units_mismatch')
            else:
                domain = 'r' if expected_tag == 'C' else 'm'
                legacy = {'gen': target[domain + 'gen'], 'rid': target[domain + 'rid']}
                if expected_tag != 'C' and (frame.mode, frame.unit, frame.scale) != {
                        'V': ('VOLT', 'V', -6), 'R': ('RES', 'ohm', -3)}[expected_tag]:
                    raise CaptureError('frame_units_mismatch')
            if any(header.get(k) != v or fields.get(k) != v for k, v in legacy.items()):
                raise CaptureError('frame_legacy_domain_mismatch')
            self.matching_frames += 1
            if self.first_frame is None:
                self.first_frame = {'header': dict(header), 'trailer': fields,
                                    'received_monotonic': time.monotonic(),
                                    'raw_chunk_end_offset': self.raw_bytes,
                                    'expected_mask': frame.expected_mask, 'acquired_mask': frame.acquired_mask,
                                    'crc_ok': True, 'legacy_domain': 'rows' if expected_tag == 'C' else
                                    'rows/profile' if expected_tag == 'M' else 'mode',
                                    'point_validity': 'NOT_ASSERTED; masks/invalid tokens preserved in raw'}
                self.event('first_matching_frame', **self.first_frame)
        except CaptureError as error:
            self.frame_error = self.frame_error or str(error)

    def prepare(self):
        self.phase = 'prepare'
        for command, tag in (('USB?', 'USB'), ('BUILD?', 'BUILD'), ('PROTO?', 'PROTO'), ('CAL?', 'CAL')):
            self.initial[command] = self.query(command, tag, self.prepare_deadline)
        config = self.query('CONFIG?', 'CONFIG', self.prepare_deadline)
        self.initial['CONFIG?'] = dict(config)
        self.expected_identity = config
        self.verify_mutation_authority(self.prepare_deadline)
        config = self.expected_identity
        self.authority_verified = True
        if config.get('mode') != 'CAP' or config.get('profile') != 'CCCCCCCC' or config.get('state') != 'CAPACITANCE':
            config = self.transition('MODE', 'CAP', self.prepare_deadline)
        if config.get('rows') != '8':
            config = self.transition('ROWS', '8', self.prepare_deadline)
        self.verify_cap(self.prepare_deadline)
        require_config(self.query('CONFIG?', 'CONFIG', self.prepare_deadline), 'CAP')
        self.ensure_rail()  # Use the current external calibration or the operator’s measured RAM values.
        config = self.query('CONFIG?', 'CONFIG', self.prepare_deadline)
        require_config(config, 'CAP')
        self.expected_identity = config
        self.initial['prepared_CONFIG'] = dict(config)

    def verify_new_frame(self, mode):
        self.configuration = benchmark.Configuration(self.settings.label, mode, 8, PROFILE, 0, 'mode_switch_50')
        self.first_frame, self.frame_error = None, None
        self.frame_header, self.frame_tag = None, None
        self.frame_parser = TextProtocolParser()
        self.phase = 'capture'
        self.capture_deadline = min(self.work_deadline, time.monotonic() + self.limits.frame_timeout)
        # These already-consumed lines remain in raw; this never flushes tty RX.
        self.lines.clear()
        try:
            while time.monotonic() < self.capture_deadline:
                self.pump(self.capture_deadline)
                self.lines.clear()
                if self.first_frame:
                    return dict(self.first_frame)
            raise CaptureError('new_frame_deadline_expired')
        finally:
            self.phase = 'transitions'
            self.frame_parser = None

    def run_plan(self):
        self.prepare()
        for index, mode in enumerate(PLAN, 1):
            self.phase = 'transitions'
            self.check_fault()
            before = dict(self.expected_identity)
            if before.get('mode') == mode:
                raise CaptureError('plan_requires_real_configuration_change')
            step = {'index': index, 'target': mode, 'before_CONFIG': before,
                    'started_monotonic': time.monotonic(), 'status': 'STARTED'}
            self.steps.append(step)
            kind, value = ('ROWMODES', PROFILE) if mode == 'MIXED' else ('MODE', mode)
            after = self.transition(kind, value, self.work_deadline)
            step['transition'] = dict(self.transitions[-1])
            require_config(after, mode)
            if (after['rev'] == before['rev'] or
                    not forward_or_equal(unsigned(after['rev'], 'rev'), unsigned(before['rev'], 'rev'))):
                raise CaptureError('configuration_revision_did_not_advance')
            step['first_frame'] = self.verify_new_frame(mode)
            closing = self.query('CONFIG?', 'CONFIG', self.work_deadline)
            if any(closing.get(key) != after.get(key) for key in CONFIG_KEYS):
                raise CaptureError('configuration_changed_during_frame_verification')
            require_config(closing, mode)
            step.update(closing_CONFIG=closing, status='VERIFIED', finished_monotonic=time.monotonic())
            self.event('mode_switch_verified', index=index, target=mode, request_id=step['transition']['request_id'])

    def cleanup(self):
        self.phase = 'cleanup'
        self.frame_parser = None
        self.cleanup_fault_baseline, self.cleanup_reset_baseline = self.fault_events, len(self.reset_observations)
        if not self.authority_verified:
            self.safe_cap_recovery = 'NOT_ATTEMPTED_AUTHORITY_UNVERIFIED'
            return
        self.safe_cap_recovery = 'ATTEMPTED'
        deadline = min(self.total_deadline, time.monotonic() + self.limits.cleanup_seconds)
        self.verify_mutation_authority(deadline, cleanup=True)
        # Read current authority first, including after a reset. Never compare
        # old/new boot counters or replay an unconfirmed setter.
        config = self.query('CONFIG?', 'CONFIG', deadline)
        self.expected_identity = config
        if config.get('mode') != 'CAP' or config.get('profile') != 'CCCCCCCC' or config.get('state') != 'CAPACITANCE':
            config = self.transition('MODE', 'CAP', deadline)
        if config.get('rows') != '8':
            self.transition('ROWS', '8', deadline)
        self.verify_cap(deadline)
        self.cleanup_config = self.query('CONFIG?', 'CONFIG', deadline)
        require_config(self.cleanup_config, 'CAP')
        self.safe_cap_recovery = 'CAP8_QUERY_VERIFIED'


def validate_options(options):
    rail_arguments(options)
    benchmark.expected_from_options(options.expected_elf_sha256, options.expected_session, options.expected_boot_id)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,95}', options.label):
        raise ValueError('label must be a unique simple filename component')
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', options.usb_serial):
        raise ValueError('usb_serial must contain exactly twelve hex digits')
    for key in ('total_seconds', 'prepare_seconds', 'cleanup_seconds', 'command_timeout', 'frame_timeout'):
        if not math.isfinite(getattr(options, key)) or getattr(options, key) <= 0:
            raise ValueError('invalid_positive_deadline:' + key)
    if options.total_seconds > 300 or options.prepare_seconds + options.cleanup_seconds >= options.total_seconds:
        raise ValueError('total deadline must be <=300s with a reserved cleanup interval')


def run_capture(options, *, port_factory=open_capture_port):
    validate_options(options)
    directory = Path(options.output_directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    paths = benchmark.record_paths(directory, options.label)
    for path in paths.values():
        if path.exists() or Path(str(path) + '.gz').exists():
            raise FileExistsError(path)
    start = time.monotonic()
    settings = benchmark.BenchmarkOptions(
        None, directory, options.label, mode='CAP', rows=8, profile=PROFILE,
        warmup_seconds=0, prepare_seconds=options.prepare_seconds, cleanup_seconds=options.cleanup_seconds,
        command_timeout=options.command_timeout, rail_avdd_uv=options.rail_avdd_uv, rail_avss_uv=options.rail_avss_uv,
        rail_source_note=options.rail_source_note, transport='usb_cdc', usb_serial=options.usb_serial,
        expected_elf_sha256=options.expected_elf_sha256, expected_session=options.expected_session,
        expected_boot_id=options.expected_boot_id)
    configuration = benchmark.Configuration(options.label, 'CAP', 8, PROFILE,
        options.total_seconds - options.prepare_seconds - options.cleanup_seconds, 'mode_switch_50')
    result = {'status': 'ERROR', 'evidence': 'LIVE_HIL_ATTEMPT' if port_factory is open_capture_port else 'INJECTED_TEST_PORT',
              'label': options.label, 'requested_transitions': 50, 'plan': list(PLAN),
              'rail_avdd_uv': options.rail_avdd_uv, 'rail_avss_uv': options.rail_avss_uv, 'rail_source_note': options.rail_source_note,
              'scientific_accuracy': 'NOT_ESTABLISHED', 'options': {**asdict(options), 'output_directory': str(directory)},
              'raw_file': str(paths['raw']), 'events_file': str(paths['events']), 'summary_file': str(paths['summary']),
              'error': None, 'cleanup_error': None, 'input_flushed': False, 'nonvolatile_write': False,
              'reset_requested': False, 'usbstream_setter': False,
              'source_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in (Path(__file__), Path(cdc.__file__), Path(cdc.shared.__file__),
                                             Path(benchmark.__file__), Path(cdc.shared.retry.__file__),
                                             Path(capture_reference_fixture.__file__), Path(sensorarray_usb_cdc.__file__),
                                             Path(text_protocol.__file__))}}
    collector, port = None, None
    with ExitStack() as stack, cdc.shared.bulk_collector_selected(cdc.CdcBenchmarkCollector):
        raw = benchmark.HashingRaw(stack.enter_context(paths['raw'].open('xb')))
        events = stack.enter_context(paths['events'].open('x', encoding='utf-8'))
        summary = stack.enter_context(paths['summary'].open('x', encoding='utf-8'))
        try:
            port = port_factory(settings)
            collector = ModeSwitchCollector(settings, configuration, port, raw, events, limits=options, start=start)
            collector.run_plan()
        except (Exception, KeyboardInterrupt) as error:
            result['error'] = f'{type(error).__name__}:{error}'
            if collector:
                collector.event('mode_switch_failure', error=result['error'])
        finally:
            if collector:
                try:
                    collector.cleanup()
                except (Exception, KeyboardInterrupt) as error:
                    result['cleanup_error'] = f'{type(error).__name__}:{error}'
                    collector.safe_cap_recovery = 'FAILED_OR_UNVERIFIED'
                    collector.event('cleanup_failure', error=result['cleanup_error'])
                result.update(initial=collector.initial, steps=collector.steps, transitions=collector.transitions,
                              verified_transitions=sum(s['status'] == 'VERIFIED' for s in collector.steps),
                              attempted_transitions=len(collector.steps),
                              applied_transitions=sum('transition' in s for s in collector.steps),
                              safe_cap_recovery=collector.safe_cap_recovery, cleanup_CONFIG=collector.cleanup_config,
                              query_attempts=collector.query_attempts, fault_events=collector.fault_events,
                              faults=list(collector.faults), reset_observations=collector.reset_observations,
                              parsed_frames=collector.parsed_frames, crc_valid_frames=collector.crc_valid_frames,
                              matching_frames=collector.matching_frames, stale_queued_frames=collector.stale_frames,
                              stale_queued_examples=collector.stale_examples,
                              rx_bytes=collector.raw_bytes, trailing_partial_line_bytes=len(collector.line_buffer),
                              volatile_rail_written=collector.volatile_rail_written,
                              image_authority=collector.image_authority_result(),
                              **collector.transport_result())
            if port:
                try:
                    port.close()
                except Exception as error:
                    result['close_error'] = f'{type(error).__name__}:{error}'
            result.update(raw_sha256=raw.sha.hexdigest(), elapsed_seconds=time.monotonic() - start)
            result['remaining_transitions'] = 50 - result.get('verified_transitions', 0)
            result['not_started_transitions'] = 50 - result.get('attempted_transitions', 0)
            if (not result['error'] and not result['cleanup_error'] and not result.get('close_error') and
                    result.get('verified_transitions') == 50 and result.get('safe_cap_recovery') == 'CAP8_QUERY_VERIFIED'):
                result['status'] = 'COMPLETE_50_IDENTITY_VERIFIED'
            json.dump(result, summary, indent=2)
            summary.write('\n')
            summary.flush()
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', required=True, type=Path)
    parser.add_argument('--label', required=True)
    parser.add_argument('--usb-serial', required=True)
    add_rail_arguments(parser)
    parser.add_argument('--total-seconds', type=float, default=300.0)
    parser.add_argument('--expected-elf', dest='expected_elf_sha256', required=True)
    parser.add_argument('--expected-session', required=True)
    parser.add_argument('--expected-boot-id', required=True, type=int)
    args = parser.parse_args(argv)
    result = run_capture(Options(**vars(args)))
    print(json.dumps({key: result.get(key) for key in ('status', 'verified_transitions', 'remaining_transitions',
          'elapsed_seconds', 'safe_cap_recovery', 'error', 'cleanup_error', 'summary_file')}), flush=True)
    return 0 if result['status'] == 'COMPLETE_50_IDENTITY_VERIFIED' else 1


if __name__ == '__main__':
    raise SystemExit(main())
