#!/usr/bin/env python3
"""Independent Linux CDC debug observer; never a formal DATA/CTRL collector.

Select explicit 303a:4041 or 303a:4042, stable serial and CDC interface 2 using
read-only sysfs metadata. No commands, USBBOOT, input flush or modem-bit ioctl.
Opening a CDC tty can cause OS line-state requests; firmware requires CDC DTR
for debug delivery. This is not a promise that OS open behaviour is side-effect
free, nor evidence that the scientific bulk stream has arrived on the PC.
"""
from contextlib import ExitStack
from dataclasses import dataclass
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from capture_reference_fixture import CaptureError, PosixPort

VID, PID, INTERFACE = '303a', '4041', '02'


def find_debug_tty(serial, *, pid=0x4041, sys_tty=Path('/sys/class/tty'), dev_root=Path('/dev')):
    """Metadata only; IF2 debug exists on vendor4041 and dual-CDC4042."""
    from sensorarray_usb_cdc import find_cdc_tty, UsbCdcError
    try:
        return find_cdc_tty(serial, pid=pid, interface_number=2, sys_tty=sys_tty, dev_root=dev_root)
    except UsbCdcError as error:
        reason = 'CDC_debug_not_enumerated' if str(error) == 'USB_CDC_role_not_enumerated' else 'CDC_debug_ambiguous_serial'
        raise CaptureError(reason) from error


@dataclass(frozen=True)
class Options:
    output_directory: Path
    label: str
    usb_serial: str
    duration: float = 9000
    enumerate_seconds: float = 60
    stop_file: Path | None = None
    usb_pid: int = 0x4041


def observe(options, *, metadata_finder=None, port_factory=PosixPort):
    if options.usb_pid not in {0x4041, 0x4042}:
        raise ValueError("debug CDC requires PID4041 or PID4042")
    metadata_finder = metadata_finder or (lambda serial: find_debug_tty(serial, pid=options.usb_pid))
    if not re.fullmatch(r'[A-Za-z0-9_-]+', options.label):
        raise ValueError('invalid label')
    if not (0 < options.duration <= 86400 and 0 < options.enumerate_seconds <= options.duration):
        raise ValueError('invalid observation budgets')
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', options.usb_serial):
        raise ValueError('invalid stable USB serial')
    if options.stop_file is not None and options.stop_file.exists():
        raise FileExistsError('stop file already exists; use a fresh observer label')
    options.output_directory.mkdir(parents=True, exist_ok=True)
    paths = {key: options.output_directory / (options.label + suffix) for key, suffix in
             [('raw', '.debug.rx.bin'), ('events', '.debug.events.jsonl'), ('summary', '.debug.summary.json')]}
    if any(p.exists() or Path(str(p) + '.gz').exists() for p in paths.values()):
        raise FileExistsError('debug evidence exists; use a fresh observer label')
    result = {'status': 'RUNNING', 'source': 'CDC_debug_interface_2',
              'requested_usb_pid': f'{options.usb_pid:04x}', 'requested_usb_serial': options.usb_serial.upper(),
              'formal_scientific_frame_evidence': False, 'commands_written': 0,
              'automatic_reconnect': False, 'USBBOOT_requested': False, 'reset_requested': False,
              'modem_bit_ioctl': False, 'input_flushed': False,
              'os_CDC_open_line_state': 'OS_controlled_not_claimed_side_effect_free',
              'baud_setting_is_not_USB_wire_rate': True, 'enumeration_attempts': [],
              'stop_file': str(options.stop_file) if options.stop_file else None,
              'faults': [], 'reset_observations': []}
    started = time.monotonic()
    deadline, enumerate_deadline = started + options.duration, started + options.enumerate_seconds
    port, count, pending, tags, sha = None, 0, bytearray(), Counter(), hashlib.sha256()
    with ExitStack() as stack:
        raw = stack.enter_context(paths['raw'].open('xb'))
        events = stack.enter_context(paths['events'].open('x', encoding='utf-8'))
        summary = stack.enter_context(paths['summary'].open('x', encoding='utf-8'))
        def event(kind, **fields):
            now = time.monotonic()
            events.write(json.dumps({'type': kind, 't': now - started, 'monotonic': now,
                                     'source': 'cdc_debug', **fields}) + '\n')
            events.flush()
        try:
            while time.monotonic() < enumerate_deadline:
                if options.stop_file is not None and options.stop_file.exists():
                    result['status'] = 'STOPPED'
                    break
                try:
                    metadata = metadata_finder(options.usb_serial)
                    result['enumeration_attempts'].append({'monotonic': time.monotonic(), 'outcome': 'matched', **metadata})
                    port = port_factory(metadata['path'])
                    # Recheck after opening, without a second reader. Path reuse
                    # during enumeration must not silently select another board.
                    if metadata_finder(options.usb_serial) != metadata:
                        raise CaptureError('CDC_identity_changed_during_open')
                    result['metadata'] = metadata
                    event('opened', metadata=metadata, commands_written=0)
                    break
                except CaptureError as error:
                    if str(error) != 'CDC_debug_not_enumerated' or port is not None:
                        raise
                    result['enumeration_attempts'].append({'monotonic': time.monotonic(), 'outcome': str(error)})
                    time.sleep(min(0.5, max(0, enumerate_deadline - time.monotonic())))
            if port is None and result['status'] != 'STOPPED':
                raise CaptureError('CDC_debug_enumeration_deadline')
            next_progress = time.monotonic() + 20
            while port is not None and time.monotonic() < deadline:
                if options.stop_file is not None and options.stop_file.exists():
                    result['status'] = 'STOPPED'
                    break
                received = port.read_some(65536, deadline)
                if received is None:
                    continue
                channel, chunk = received
                if channel != 'data' or not chunk:
                    raise CaptureError('invalid_CDC_read_result')
                if raw.write(chunk) != len(chunk):
                    raise CaptureError('CDC_raw_partial_file_write')
                raw.flush()
                sha.update(chunk)
                event('rx', offset=count, length=len(chunk))
                count += len(chunk)
                pending.extend(chunk)
                while b'\n' in pending:
                    line, _, tail = pending.partition(b'\n')
                    pending[:] = tail
                    text = line.rstrip(b'\r').decode('ascii', errors='backslashreplace')
                    tag = text.split(',', 1)[0]
                    tags[tag if len(tag) <= 48 and re.fullmatch(r'[A-Z0-9_]+', tag) else 'other'] += 1
                    if text.startswith(('MFAULT,', 'APP_FATAL', 'FDC_FATAL')) or any(s in text.lower() for s in
                            ('guru meditation', 'task watchdog got triggered', 'interrupt wdt timeout')):
                        event('firmware_fault', line=text)
                        if len(result['faults']) < 64:
                            result['faults'].append({'monotonic': time.monotonic(), 'line': text})
                    if text.startswith(('RST,', 'ESP-ROM:')) or 'rst:0x' in text.lower():
                        event('reset_observed', line=text, causation='unknown')
                        if len(result['reset_observations']) < 64:
                            result['reset_observations'].append({'monotonic': time.monotonic(), 'line': text})
                if len(pending) > 65536:
                    raise CaptureError('CDC_debug_line_overflow_raw_preserved')
                if time.monotonic() >= next_progress:
                    print(json.dumps({'type': 'CDC_debug_progress', 'rx_bytes': count,
                                      'elapsed_seconds': time.monotonic() - started}), flush=True)
                    next_progress = time.monotonic() + 20
            if result['status'] == 'RUNNING':
                result['status'] = 'OBSERVED'
        except (OSError, CaptureError, KeyboardInterrupt) as error:
            result.update(status='INTERRUPTED' if isinstance(error, KeyboardInterrupt) else 'ERROR',
                          error=str(error) or 'interrupted')
        finally:
            if port is not None:
                try: port.close()
                except OSError as error:
                    result.update(status='ERROR', close_error=str(error))
            event('finished', status=result['status'], error=result.get('error'), rx_bytes=count)
            result.update(raw_file=str(paths['raw']), events_file=str(paths['events']),
                          raw_sha256=sha.hexdigest(), rx_bytes=count, tag_counts=dict(tags),
                          trailing_partial_line_bytes=len(pending),
                          started_monotonic=started, ended_monotonic=time.monotonic(),
                          source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          observation_status_is_not_health_PASS=True,
                          diagnostic_loss='CDC_is_best_effort; absent_records_are_NOT_OBSERVED_not_success')
            summary.write(json.dumps(result, indent=2) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--usb-serial', required=True)
    parser.add_argument('--usb-pid', type=lambda value: int(value, 0), default=0x4041, help='4041 vendor bulk; use 0x4042 for dual CDC')
    parser.add_argument('--duration', type=float, default=9000)
    parser.add_argument('--enumerate-seconds', type=float, default=60)
    parser.add_argument('--stop-file', type=Path)
    result = observe(Options(**vars(parser.parse_args(argv))))
    print(json.dumps({key: result.get(key) for key in ('status', 'error', 'rx_bytes', 'raw_file')}), flush=True)
    return 0 if result['status'] in {'OBSERVED', 'STOPPED'} else 1


if __name__ == '__main__':
    raise SystemExit(main())
