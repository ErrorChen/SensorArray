#!/usr/bin/env python3
"""Explicit single-request software recovery observation; never runs esptool.

Default: read-only formal CDC0 identity queries. --request-rom sends USBBOOT
once; --request-restart sends RESTART once. RACK/RAPP are publication evidence,
not reset proof. A ROM PID/tty observation is not a flasher/MAC verification.
Every reader epoch has its own raw file; no partial line survives a reconnect.
"""
import argparse
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
import errno
import hashlib
import json
import math
from pathlib import Path
import re
import stat
import sys
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
import capture_reference_fixture as capture
from sensorarray_usb_cdc import UsbCdcPort
from text_protocol import parse_fields

IDENTITY = ('session', 'bootId', 'rev', 'mgen', 'mrid', 'rgen', 'rrid', 'pgen', 'prid')
QUERIES = (('BUILD?', 'BUILD'), ('PROTO?', 'PROTO'), ('CONFIG?', 'CONFIG'),
           ('USB?', 'USB'), ('RESTART?', 'RESTART'))


def normalize_serial(value):
    if not isinstance(value, str) or not re.fullmatch(r'[0-9A-Fa-f:]+', value): return None
    value = value.replace(':', '').upper()
    return value if re.fullmatch(r'[0-9A-F]{12}', value) else None


def usb_inventory(usb_root=Path('/sys/bus/usb/devices'), tty_root=Path('/sys/class/tty')):
    """Read sysfs only; do not open tty/libusb or trigger driver binding."""
    devices = []
    for path in sorted(usb_root.iterdir()):
        try:
            if not (path / 'idVendor').is_file(): continue
            read = lambda field: (path / field).read_text().strip()
            vid, pid = read('idVendor').lower(), read('idProduct').lower()
            if vid != '303a' or pid not in {'4042', '1001'}: continue
            physical = path.resolve(strict=True)
            serial = read('serial')
            interfaces = []
            for interface in sorted(usb_root.glob(path.name + ':*')):
                try:
                    interfaces.append({'sysfs_path': str(interface.resolve(strict=True)),
                        'number': (interface / 'bInterfaceNumber').read_text().strip().lower(),
                        'class': (interface / 'bInterfaceClass').read_text().strip().lower(),
                        'subclass': (interface / 'bInterfaceSubClass').read_text().strip().lower(),
                        'protocol': (interface / 'bInterfaceProtocol').read_text().strip().lower()})
                except FileNotFoundError: continue
            ttys = []
            for tty in sorted(tty_root.glob('ttyACM*')):
                try:
                    target = (tty / 'device').resolve(strict=True)
                    if physical == target or physical in target.parents: ttys.append('/dev/' + tty.name)
                except FileNotFoundError: continue
            devices.append({'sysfs_path': str(physical), 'sysfs_name': path.name, 'vid': vid, 'pid': pid,
                            'busnum': int(read('busnum')), 'devnum': int(read('devnum')),
                            'serial_raw': serial, 'serial': normalize_serial(serial),
                            'interfaces': interfaces, 'ttys': ttys,
                            'speed_mbps_sysfs': read('speed') if (path / 'speed').is_file() else None})
        except FileNotFoundError: continue
    return devices


def epoch_key(device):
    return (device['sysfs_path'], device['busnum'], device['devnum'], device['vid'], device['pid'], device['serial'])


def formal_role(device):
    return device['pid'] == '4042' and any((i['number'], i['class'], i['subclass'], i['protocol']) ==
                                         ('00', '02', '02', '00') for i in device['interfaces'])


def same_formal_epoch(expected, devices):
    matches = [d for d in devices if epoch_key(d) == epoch_key(expected)]
    if len(matches) != 1 or not formal_role(matches[0]): return False
    expected_interfaces = [i for i in expected['interfaces'] if i['number'] == '00']
    actual_interfaces = [i for i in matches[0]['interfaces'] if i['number'] == '00']
    return actual_interfaces == expected_interfaces


def tty_node_metadata(paths):
    """Read node ownership only; never chmod, open or request privileges."""
    nodes = []
    for path in paths:
        node = {'path': path, 'uid': None, 'gid': None, 'mode': None}
        try:
            info = Path(path).stat()
            node.update(exists=True, uid=info.st_uid, gid=info.st_gid,
                        mode=f'{stat.S_IMODE(info.st_mode):04o}',
                        character_device=stat.S_ISCHR(info.st_mode), stat_errno=None)
        except OSError as error:
            node.update(exists=False if error.errno == errno.ENOENT else None,
                        stat_errno=error.errno, stat_error=str(error))
        nodes.append(node)
    return nodes


class PendingEnumerationNode(Exception):
    """Only a port-factory ENOENT/EACCES, never an evidence-file error."""
    def __init__(self, error):
        super().__init__(str(error)); self.original = error


@contextmanager
def collector_clock(clock):
    # Collector has a module-local time reference. This process has one reader;
    # replacing that reference allows fake clocks without editing shared code
    # or changing the global stdlib time module. Production uses real monotonic.
    previous = capture.time
    capture.time = SimpleNamespace(monotonic=clock)
    try: yield
    finally: capture.time = previous


class RecordedLines(deque):
    def __init__(self, observer): super().__init__(); self.observer = observer
    def append(self, line): self.observer(line); super().append(line)


class EpochCollector(capture.Collector):
    def __init__(self, *args, writer, clock, epoch, **kwargs):
        self.writer, self.clock, self.epoch = writer, clock, epoch
        self.publications = []
        self.restart_cache_snapshots = []
        super().__init__(*args, **kwargs)
        self.lines = RecordedLines(self.observe_line)

    def event(self, kind, **fields): self.writer(kind, epoch=self.epoch, phase=self.phase, **fields)
    def progress(self): pass
    def observe_line(self, line):
        tag = line.split(',', 1)[0]
        if tag in {'RACK', 'RAPP', 'RERR', 'ERR', 'RESTART'}:
            fields = parse_fields(line)
            self.event('control_record', line=line, fields=fields)
            if tag in {'RACK', 'RAPP'} and fields.get('cmd') == 'RESTART' and len(self.publications) < 32:
                self.publications.append({'tag': tag, 'fields': fields, 'monotonic': self.clock()})
            if tag == 'RESTART' and len(self.restart_cache_snapshots) < 16:
                self.restart_cache_snapshots.append({'fields': fields, 'monotonic': self.clock(),
                                                     'phase': self.phase})


@dataclass(frozen=True)
class Options:
    output_directory: Path
    label: str
    usb_serial: str
    action: str | None = None
    duration: float = 60.0
    query_timeout: float = 2.0
    poll_seconds: float = 0.1


def validate_snapshot(replies, action=None):
    build, proto, config, usb, restart = (replies[c] for c in ('BUILD?', 'PROTO?', 'CONFIG?', 'USB?', 'RESTART?'))
    if not build.get('version') or not re.fullmatch(r'[0-9A-Fa-f]{64}', build.get('elfSha256', '')):
        raise capture.CaptureError('BUILD_identity_incomplete')
    caps = set(re.split(r'[/;,|\s]+', proto.get('caps', '')))
    required = {'usb_cdc_data_v1', 'config_query_v1', 'restart_query_v1'}
    if action == 'USBBOOT': required.add('usb_rom_handoff_v1')
    if proto.get('version') != '2' or not required <= caps:
        raise capture.CaptureError('required_capabilities_unavailable')
    if not re.fullmatch(r'[0-9A-Fa-f]{16}', config.get('session', '')) or int(config['session'], 16) == 0:
        raise capture.CaptureError('CONFIG_session_unavailable')
    for key in IDENTITY[1:]:
        if not re.fullmatch(r'\d+', config.get(key, '')) or int(config[key]) > 0xFFFFFFFF:
            raise capture.CaptureError('CONFIG_identity_incomplete:' + key)
    if usb.get('backend') != 'otg_cdc_v1' or usb.get('init') != '1' or usb.get('mounted') != '1':
        raise capture.CaptureError('formal_CDC_state_unavailable')
    if action == 'USBBOOT' and usb.get('romAvailable') != '1':
        raise capture.CaptureError('ROM_handoff_unavailable')
    if any(restart.get(key) != config.get(key) for key in ('session', 'bootId')):
        raise capture.CaptureError('RESTART_CONFIG_boot_mismatch')
    if restart.get('busy') not in {'0', '1'} or not re.fullmatch(r'\d+', restart.get('lastId', '')):
        raise capture.CaptureError('RESTART_cache_incomplete')


def observe(options, *, inventory=usb_inventory, port_factory=UsbCdcPort, clock=time.monotonic,
            sleep=time.sleep, sysfs_exists=lambda path: Path(path).exists(), node_metadata=tty_node_metadata):
    if not re.fullmatch(r'[A-Za-z0-9_-]+', options.label): raise ValueError('invalid label')
    serial = normalize_serial(options.usb_serial)
    if serial is None:
        raise ValueError('explicit 12-digit USB serial required')
    if options.action not in {None, 'USBBOOT', 'RESTART'}: raise ValueError('invalid action')
    if not (0 < options.duration <= 60 and 0 < options.query_timeout <= 3 and .05 <= options.poll_seconds <= .5):
        raise ValueError('invalid finite timing budget')
    out = options.output_directory; out.mkdir(parents=True, exist_ok=True)
    if any(out.glob(options.label + '.*')): raise FileExistsError('existing evidence; use a new label')
    paths = {'events': out / (options.label + '.events.jsonl'), 'summary': out / (options.label + '.summary.json')}
    started, epochs, collector, port, raw = clock(), [], None, None, None
    active_epoch, reader_release_known = None, True
    deadline = started + options.duration
    result = {'status': 'RUNNING', 'requested_action': options.action, 'action_attempts': 0,
              'deadline_seconds': options.duration, 'poll_seconds': options.poll_seconds,
              'old_epoch_absent_monotonic': None, 'old_interface_absent_monotonic': None,
              'restart_queries_after_action': [], 'epochs': epochs,
              'post_enumeration_open_waits': [], 'post_enumeration_open_attempts': 0,
              'esptool_executed': False, 'flash_written': False, 'automatic_action_retry': False,
              'RACK_RAPP_semantics': 'local publication only, not reset or delivery proof',
              'generation_comparison': 'session equality only; sequence/generation magnitudes never compared across boots'}
    with paths['events'].open('x') as events, paths['summary'].open('x') as summary, collector_clock(clock):
        def event(kind, **fields):
            events.write(json.dumps({'type': kind, 'monotonic': clock(), 't': clock() - started, **fields}) + '\n'); events.flush()
        def close_epoch(reason):
            nonlocal collector, port, raw, active_epoch, reader_release_known
            if active_epoch is None: return
            epoch = active_epoch
            if collector:
                epoch.update(rx_bytes=collector.raw_bytes, partial_line_bytes=len(collector.line_buffer),
                             publications=list(collector.publications), replies=dict(collector.replies),
                             restart_cache_snapshots=list(collector.restart_cache_snapshots))
            try:
                if port is not None: port.close()
            except Exception as error:
                epoch['close_error'] = str(error); reader_release_known = False
                result['reader_close_error'] = str(error)
            port = None
            if raw:
                raw.flush(); raw.close(); raw = None
            digest = hashlib.sha256()
            if Path(epoch['raw_file']).is_file():
                with Path(epoch['raw_file']).open('rb') as stream:
                    for chunk in iter(lambda: stream.read(1024 * 1024), b''): digest.update(chunk)
                epoch['raw_sha256'] = digest.hexdigest()
            epoch.update(close_reason=reason, closed_monotonic=clock(), reader_release_known=reader_release_known)
            event('reader_closed', epoch=epoch['epoch'], reason=reason, partial_line_bytes=epoch.get('partial_line_bytes', 0))
            collector = None; active_epoch = None
        def open_epoch(device, *, allow_node_wait=False):
            nonlocal collector, port, raw, active_epoch
            if port is not None or not reader_release_known: raise RuntimeError('reader_overlap_or_unknown_release_refused')
            index = len(epochs)
            raw_path = out / f'{options.label}.epoch{index}.rx.bin'
            active_epoch = {'epoch': index, 'device': device, 'raw_file': str(raw_path),
                            'open_started_monotonic': clock(), 'partial_line_initial_bytes': 0,
                            'open_succeeded': False}
            epochs.append(active_epoch)
            if allow_node_wait and not same_formal_epoch(device, inventory()):
                raise capture.CaptureError('post_enumeration_identity_changed_before_open')
            try:
                port = port_factory(serial)
            except OSError as error:
                if allow_node_wait and error.errno in {errno.ENOENT, errno.EACCES}:
                    active_epoch['open_errno'] = error.errno
                    raise PendingEnumerationNode(error) from error
                raise
            metadata = port.metadata()
            expected_interface = next(i['sysfs_path'] for i in device['interfaces'] if i['number'] == '00')
            if (metadata.get('transport') != 'usb_cdc' or normalize_serial(metadata.get('usb_serial')) != serial or
                    metadata.get('interface_number') != 0 or metadata.get('sysfs_device') != device['sysfs_path'] or
                    metadata.get('sysfs_interface') != expected_interface):
                raise capture.CaptureError('opened_port_identity_or_role_mismatch')
            if not same_formal_epoch(device, inventory()):
                raise capture.CaptureError('enumeration_epoch_changed_during_open')
            raw = raw_path.open('xb')
            active_epoch.update(metadata=metadata, opened_monotonic=clock(), open_succeeded=True)
            collector = EpochCollector(capture.CaptureOptions(None, max(.001, deadline-clock()), out, options.label,
                command_timeout=options.query_timeout, transport='usb_cdc', usb_serial=serial),
                port, raw, events, writer=event, clock=clock, epoch=index)
            collector.total_deadline = deadline
            collector.phase = 'preflight' if index == 0 else 'post_restart_queries'
        def snapshot(action=None):
            replies = {}
            for command, tag in QUERIES: replies[command] = collector.query(command, tag, deadline)
            closing = collector.query('CONFIG?', 'CONFIG', deadline)
            if any(replies['CONFIG?'].get(key) != closing.get(key) for key in IDENTITY):
                raise capture.CaptureError('configuration_changed_during_queries')
            validate_snapshot(replies, action)
            return replies
        try:
            initial = inventory(); event('inventory_initial', devices=initial)
            matching = [d for d in initial if d.get('serial') == serial]
            if len(matching) != 1 or not formal_role(matching[0]):
                raise capture.CaptureError('unique_authorized_formal_CDC0_not_enumerated')
            old = matching[0]; old_key = epoch_key(old)
            baseline_keys = {epoch_key(d) for d in initial}
            old_interface = next(i['sysfs_path'] for i in old['interfaces'] if i['number'] == '00')
            result['old_device'] = old
            open_epoch(old)
            before = snapshot(options.action); result['before'] = before
            if options.action is None:
                result['status'] = 'READONLY_COMPLETE'
            else:
                if before['RESTART?']['busy'] != '0': raise capture.CaptureError('restart_already_pending_no_action_sent')
                if clock() >= deadline: raise capture.CaptureError('deadline_before_action_no_action_sent')
                # Increment before any write: failure/partial acceptance is
                # outcome-unknown and is never followed by replay of the action.
                result['action_attempts'] = 1; result['action_started_monotonic'] = clock()
                collector.phase = 'action'
                event('single_action_attempt', command=options.action)
                try:
                    collector.send(options.action, min(deadline, clock()+options.query_timeout))
                    result['action_host_write'] = 'all_bytes_accepted_by_OS_only'
                except (OSError, capture.CaptureError) as error:
                    result['action_host_write'] = 'outcome_unknown'; result['action_write_error'] = str(error)
                next_query, polls, opened_new, confirmed_target = clock()+1, 0, False, None
                max_polls = math.ceil(options.duration / options.poll_seconds) + 1
                while clock() < deadline and polls < max_polls:
                    iteration = clock(); polls += 1
                    devices = inventory(); event('inventory_poll', poll=polls, devices=devices)
                    if confirmed_target is not None and not same_formal_epoch(confirmed_target, devices):
                        result['post_enumeration_identity_change'] = {'expected': confirmed_target, 'observed': devices}
                        event('post_enumeration_identity_changed', expected=confirmed_target, observed=devices)
                        raise capture.CaptureError('post_enumeration_identity_changed_no_reopen')
                    keys = {epoch_key(d) for d in devices}
                    if old_key not in keys and result['old_epoch_absent_monotonic'] is None:
                        result['old_epoch_absent_monotonic'] = clock(); event('old_epoch_absent')
                        close_epoch('old_epoch_absent_in_sysfs')
                    # A new devnum alone proves a different enumeration epoch,
                    # but does not prove we sampled the old interface removal.
                    # If the path disappears and is reused between polls, leave
                    # absence unknown; never relabel cached/new sysfs as a gap.
                    if not sysfs_exists(old_interface):
                        if result['old_interface_absent_monotonic'] is None:
                            result['old_interface_absent_monotonic'] = clock(); event('old_interface_absent')
                    target_pid = '1001' if options.action == 'USBBOOT' else '4042'
                    targets = [d for d in devices if d.get('serial') == serial and d['pid'] == target_pid and
                               epoch_key(d) not in baseline_keys and d.get('ttys') and
                               (d['busnum'], d['devnum']) != (old['busnum'], old['devnum'])]
                    if result['old_epoch_absent_monotonic'] is not None and result['old_interface_absent_monotonic'] is not None and len(targets) == 1:
                        target = targets[0]
                        result['new_device'] = target
                        result['new_enumeration_monotonic'] = result.get('new_enumeration_monotonic', clock())
                        if options.action == 'USBBOOT':
                            result['status'] = 'ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED'
                            result['ROM_port_opened'] = False
                            break
                        if not formal_role(target): raise capture.CaptureError('new_CDC_role_mismatch')
                        if confirmed_target is None: confirmed_target = target
                        if not opened_new:
                            close_epoch('replace_reader_for_new_epoch')
                            result['post_enumeration_open_attempts'] += 1
                            try:
                                open_epoch(confirmed_target, allow_node_wait=True); opened_new = True
                                result['post_enumeration_reader_ready_monotonic'] = clock()
                                result['post_enumeration_open_wait_resolved'] = True
                            except PendingEnumerationNode as pending:
                                error = pending.original
                                wait = {'monotonic': clock(), 'errno': error.errno,
                                        'errno_name': errno.errorcode.get(error.errno, 'unknown'),
                                        'error_class': type(error).__name__, 'error': str(error),
                                        'tty_nodes': node_metadata(target.get('ttys', [])),
                                        'remaining_deadline_seconds': max(0, deadline-clock())}
                                result['post_enumeration_open_waits'].append(wait)
                                result['post_enumeration_open_wait_resolved'] = False
                                event('post_enumeration_node_open_wait', **wait)
                                close_epoch('post_enumeration_node_not_ready')
                        # Read-only retries may wait for startup publication.
                        # They never create another restart request.
                        if opened_new:
                            try:
                                after = snapshot(); result['after'] = after
                                if after['CONFIG?']['session'] == before['CONFIG?']['session']:
                                    raise capture.CaptureError('new_enumeration_same_session_not_a_verified_restart')
                                if (after['BUILD?']['version'] != before['BUILD?']['version'] or
                                        after['BUILD?']['elfSha256'].lower() != before['BUILD?']['elfSha256'].lower()):
                                    raise capture.CaptureError('post_restart_BUILD_identity_changed')
                                result['status'] = 'APPLICATION_RESTART_OBSERVED'; break
                            except capture.CaptureError as error:
                                event('post_restart_query_incomplete', error=str(error))
                                result['post_restart_error'] = str(error)
                    if collector and not opened_new:
                        try:
                            if clock() >= next_query and len(result['restart_queries_after_action']) < 8:
                                # Do not block a whole query timeout across the
                                # removal/add window. Replies are recorded by
                                # observe_line while the ordinary reader pumps.
                                collector.send('RESTART?', min(deadline, clock()+options.poll_seconds))
                                result['restart_queries_after_action'].append({'monotonic': clock(), 'status': 'query_sent_only'})
                                next_query = clock()+1
                            else: collector.pump(min(deadline, clock()+options.poll_seconds/2)); collector.lines.clear()
                        except (OSError, capture.CaptureError) as error:
                            event('reader_interrupted', error=str(error)); close_epoch('reader_interrupted')
                    if clock() < deadline: sleep(max(0, min(options.poll_seconds-(clock()-iteration), deadline-clock())))
                result['polls'] = polls
                if result['status'] == 'RUNNING':
                    result['status'] = 'DEADLINE_WITHOUT_REQUIRED_ENUMERATION_OR_IDENTITY'
                    if result['post_enumeration_open_waits'] and not opened_new:
                        result['failure_reason'] = 'post_enumeration_node_open_deadline'
                        result['last_open_errno'] = result['post_enumeration_open_waits'][-1]['errno']
        except (Exception, KeyboardInterrupt) as error:
            result.update(status='INTERRUPTED' if isinstance(error, KeyboardInterrupt) else 'ERROR',
                          error=str(error) or 'interrupted', error_class=type(error).__name__)
            event('error', error=result['error'], error_class=result['error_class'])
        finally:
            close_epoch('observation_finished')
            if not reader_release_known:
                result['observation_status_before_cleanup_error'] = result['status']
                result['status'] = 'ERROR'
            result.update(started_monotonic=started, ended_monotonic=clock(), elapsed_seconds=clock()-started,
                          events_file=str(paths['events']), source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          ELF_hash_is_not_bin_hash=True, system_USB_reset_requested=False,
                          causal_scope='enumeration and identities observed after one action; external interventions cannot be excluded by this tool')
            if 'action_started_monotonic' in result and 'new_enumeration_monotonic' in result:
                result['action_to_enumeration_seconds'] = result['new_enumeration_monotonic'] - result['action_started_monotonic']
            event('finished', status=result['status'])
            summary.write(json.dumps(result, indent=2) + '\n')
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--usb-serial', required=True)
    parser.add_argument('--duration', type=float, default=60)
    action = parser.add_mutually_exclusive_group()
    action.add_argument('--request-rom', action='store_const', const='USBBOOT', dest='action')
    action.add_argument('--request-restart', action='store_const', const='RESTART', dest='action')
    result = observe(Options(**vars(parser.parse_args(argv))))
    print(json.dumps({k: result.get(k) for k in ('status', 'error', 'action_attempts', 'elapsed_seconds')}), flush=True)
    return 0 if result['status'] in {'READONLY_COMPLETE', 'ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED', 'APPLICATION_RESTART_OBSERVED'} else 1


if __name__ == '__main__': raise SystemExit(main())
