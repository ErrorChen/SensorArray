#!/usr/bin/env python3
"""Observe authorized app enumeration via formal bulk; read-only commands only.

500 ms bounded enumeration polling, separate immutable DATA/CTRL raw streams.
No USBBOOT, reset, configuration selection, CDC open, setters or input flush.
BOARD? is currently unavailable in production; it is recorded as NOT_AVAILABLE.
"""
from contextlib import ExitStack
from dataclasses import dataclass
import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from capture_reference_fixture import CaptureError, CaptureOptions, Collector, transport_usb_pid
from sensorarray_usb_bulk import UsbBulkPort, UsbBulkError, USB_VID, USB_PID
from sensorarray_usb_cdc import UsbCdcPort
from capture_bulk_benchmark import EXTRA_REQUIRED, CONFIG_CONSISTENCY, validate_reply, unsigned

QUERIES = (('BUILD?', 'BUILD'), ('PROTO?', 'PROTO'), ('CONFIG?', 'CONFIG'), ('BOOT?', 'BOOT'),
           ('USB?', 'USB'), ('USBQ?', 'USBQ'), ('BLELINK?', 'BLELINK'), ('BLETX?', 'BLETX'),
           ('SCHED?', 'SCHED'), ('SPI?', 'SPI'), ('FRONTENDS?', 'FRONTENDS'), ('MODE?', 'MODE'))
POLL_SECONDS = 0.5
BOOT_REQUIRED = 'boot bootId reset stage err seq heap heapMin prevStage prevErr prevHeap guard autoRestarts windowAgeS ready'


def validate_boot(fields):
    if any(not fields.get(key) for key in BOOT_REQUIRED.split()):
        raise CaptureError('incomplete_BOOT_reply')
    for key in ('boot', 'bootId', 'seq', 'heap', 'heapMin', 'prevHeap', 'autoRestarts', 'windowAgeS', 'ready'):
        unsigned(fields[key], 'BOOT?', key, 64 if key == 'windowAgeS' else 32)
    for key in ('err', 'prevErr'):
        if not re.fullmatch(r'0x[0-9A-Fa-f]{1,8}', fields[key]):
            raise CaptureError('invalid_BOOT_error:' + key)
    if fields['ready'] not in {'0', '1'}:
        raise CaptureError('invalid_BOOT_ready')


@dataclass(frozen=True)
class Options:
    output_directory: Path
    label: str
    usb_serial: str
    duration: float = 90.0
    observe_seconds: float = 30.0
    command_timeout: float = 3.0
    usb_vid: int = USB_VID
    usb_pid: int | None = None
    transport: str = 'usb_bulk'


def wait_for_bulk(options, deadline, attempts, *, factory=UsbBulkPort, clock=time.monotonic, sleep=time.sleep):
    while clock() < deadline:
        attempt = {'started_monotonic': clock()}
        attempts.append(attempt)
        try:
            port = factory(options.usb_serial, vid=options.usb_vid, pid=transport_usb_pid(options))
            if clock() >= deadline:
                attempt['outcome'] = 'matched_after_deadline_not_admitted'
                try: port.close()
                except OSError as error: attempt['cleanup_error'] = str(error)
                raise UsbBulkError('formal_bulk_enumeration_deadline')
            attempt.update(outcome='matched_roles_opened', finished_monotonic=clock())
            return port
        except OSError as error:
            attempt.update(outcome='failed', error=str(error), finished_monotonic=clock())
            retryable = str(error) in {'usb_device_not_found', 'USB_CDC_role_not_enumerated'} or getattr(error, 'backend_error_code', None) in {-4, -5}
            if not retryable:
                raise
        sleep(min(POLL_SECONDS, max(0.0, deadline - clock())))
    raise UsbBulkError('formal_bulk_enumeration_deadline')


def observe(options, *, port_factory=None):
    if options.transport not in {'usb_bulk', 'usb_cdc'}:
        raise ValueError('boot observer requires an explicit formal USB transport')
    port_factory = port_factory or (UsbCdcPort if options.transport == 'usb_cdc' else UsbBulkPort)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', options.label):
        raise ValueError('invalid label')
    if not (0 < options.duration <= 600 and 0 <= options.observe_seconds < options.duration and
            0 < options.command_timeout <= 10):
        raise ValueError('invalid observation timing budgets')
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', options.usb_serial):
        raise ValueError('USB serial must be twelve MAC hex digits without colons')
    options.output_directory.mkdir(parents=True, exist_ok=True)
    paths = {name: options.output_directory / (options.label + suffix) for name, suffix in
             [('raw', '.rx.bin'), ('ctrl', '.ctrl.rx.bin'), ('events', '.events.jsonl'), ('summary', '.summary.json')]}
    if options.transport == 'usb_cdc': paths.pop('ctrl')
    if any(path.exists() or Path(str(path) + '.gz').exists() for path in paths.values()):
        raise FileExistsError('observation output exists; use a new label')
    queries = QUERIES + ((('USBSTREAM?', 'USBSTREAM'),) if options.transport == 'usb_cdc' else ())
    result = {'status': 'RUNNING', 'transport': options.transport, 'enumeration_attempts': [], 'enumeration_poll_seconds': POLL_SECONDS,
              'enumeration_timing': ('sleep_at_most_500ms; read_only_sysfs_then_tty_open; OS_enumeration_is_not_hard_realtime'
                                     if options.transport == 'usb_cdc' else 'sleep_at_most_500ms; each_USB_descriptor_transfer_at_most_250ms; OS_enumeration_is_not_hard_realtime'),
              'requested_queries': [command for command, _ in queries],
              'unavailable_queries': {'BOARD?': 'NOT_AVAILABLE_in_current_production_source'},
              'USBBOOT_requested': False, 'reset_requested': False, 'calibration_written': False,
              'CDC_opened': False, 'configuration_changed': False}
    started = time.monotonic()
    deadline = started + options.duration
    port, collector = None, None
    with ExitStack() as stack:
        raw = stack.enter_context(paths['raw'].open('xb'))
        ctrl = stack.enter_context(paths['ctrl'].open('xb')) if 'ctrl' in paths else None
        events = stack.enter_context(paths['events'].open('x', encoding='utf-8'))
        summary = stack.enter_context(paths['summary'].open('x', encoding='utf-8'))
        try:
            port = wait_for_bulk(options, deadline, result['enumeration_attempts'], factory=port_factory)
            result['CDC_opened'] = options.transport == 'usb_cdc'
            remaining = deadline - time.monotonic()
            collector = Collector(CaptureOptions(None, remaining, options.output_directory, options.label,
                                                 command_timeout=options.command_timeout, transport=options.transport,
                                                 usb_vid=options.usb_vid, usb_pid=options.usb_pid,
                                                 usb_serial=options.usb_serial), port, raw, events, ctrl_raw=ctrl)
            collector.total_deadline = deadline
            collector.phase = 'boot_observation'
            collector.event('enumeration', attempts=result['enumeration_attempts'], poll_seconds=POLL_SECONDS)
            until = min(deadline, time.monotonic() + options.observe_seconds)
            while time.monotonic() < until:
                collector.pump(until)
                collector.lines.clear()
            collector.phase = 'boot_queries'
            opening = None
            for command, tag in queries:
                fields = collector.query(command, tag, deadline)
                if command == 'PROTO?': collector.validate_protocol(fields)
                if command == 'BOOT?': validate_boot(fields)
                elif options.transport == 'usb_cdc':
                    from capture_cdc_benchmark import CdcBenchmarkCollector
                    CdcBenchmarkCollector.validate_fields(command, fields)
                else: validate_reply(command, fields)
                if command == 'BUILD?':
                    result['build_identity'] = {'version': fields['version'], 'elf_sha256': fields['elfSha256'],
                        'application_binary_sha256': None,
                        'hash_scope': 'firmware application ELF identity, not the flashed .bin SHA256'}
                if command == 'CONFIG?': opening = fields
            closing = collector.query('CONFIG?', 'CONFIG', deadline)
            validate_reply('CONFIG?', closing)
            if opening is None or any(opening.get(key) != closing.get(key) for key in CONFIG_CONSISTENCY):
                raise CaptureError('boot_query_configuration_changed')
            if collector.replies['BOOT?']['bootId'] != closing['bootId']:
                raise CaptureError('BOOT_CONFIG_identity_mismatch')
            if collector.faults:
                raise CaptureError('firmware_fault_observed_on_bulk')
            result['status'] = 'QUERIES_COMPLETE'
        except (Exception, KeyboardInterrupt) as error:
            result.update(status='INTERRUPTED' if isinstance(error, KeyboardInterrupt) else 'ERROR',
                          error=str(error) or 'interrupted', error_class=type(error).__name__)
        finally:
            if collector is not None:
                result.update(collector.transport_result(), replies=collector.replies,
                              firmware_faults=collector.faults, reset_observations=collector.reset_observations,
                              rx_bytes=collector.raw_bytes, trailing_partial_line_bytes=len(collector.line_buffer))
                collector.event('finished', status=result['status'], error=result.get('error'))
            if port is not None:
                try: port.close()
                except Exception as error:
                    result.update(status='ERROR', close_error=str(error))
            raw.flush()
            if ctrl:
                ctrl.flush()
                result.update(ctrl_raw_file=str(paths['ctrl']), ctrl_raw_sha256=hashlib.sha256(paths['ctrl'].read_bytes()).hexdigest())
            result.update(raw_file=str(paths['raw']), events_file=str(paths['events']),
                          raw_sha256=hashlib.sha256(paths['raw'].read_bytes()).hexdigest(),
                          elapsed_seconds=time.monotonic() - started,
                          source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          debug_diagnostics_observed=False,
                          CDC_WDT_ADC_CRC_heap_evidence='requires_separate_debug_observer',
                          boot_observation_scope='post_enumeration_formal_USB_queries_not_ROM_or_debug_CDC_boot_log')
            summary.write(json.dumps(result, indent=2) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--usb-serial', required=True)
    parser.add_argument('--transport', choices=('usb_bulk', 'usb_cdc'), default='usb_bulk')
    parser.add_argument('--usb-vid', type=lambda value: int(value, 0), default=USB_VID)
    parser.add_argument('--usb-pid', type=lambda value: int(value, 0))
    parser.add_argument('--duration', type=float, default=90)
    parser.add_argument('--observe-seconds', type=float, default=30)
    parser.add_argument('--command-timeout', type=float, default=3)
    options = Options(**vars(parser.parse_args(argv)))
    result = observe(options)
    print(json.dumps({key: result.get(key) for key in ('status', 'error', 'raw_file', 'ctrl_raw_file', 'elapsed_seconds')}), flush=True)
    return 0 if result['status'] == 'QUERIES_COMPLETE' else 1


if __name__ == '__main__':
    raise SystemExit(main())
