#!/usr/bin/env python3
"""Synthetic recovery-observer regression; never opens a real device.

Executes the real observer and Collector with fake ports, monotonic time and
sysfs snapshots. Temporary raw/events are retained and explicitly synthetic.
No esptool, reset, flash, libusb or /dev operation is performed.
"""
import argparse
from copy import deepcopy
import errno
import hashlib
import json
from pathlib import Path
import tempfile

import check_software_recovery as recovery


class Clock:
    def __init__(self): self.value = 100.0
    def now(self): return self.value
    def sleep(self, seconds):
        assert seconds >= 0
        self.value += seconds


SYNTHETIC_SERIAL = '001122AABBCC'


def device(*, pid='4042', number=10, serial=SYNTHETIC_SERIAL):
    path = '/SYNTHETIC/sys/devices/usb3/3-1'
    return {'sysfs_path': path, 'sysfs_name': '3-1', 'vid': '303a', 'pid': pid,
            'busnum': 3, 'devnum': number, 'serial': recovery.normalize_serial(serial),
            'serial_raw': serial, 'speed_mbps_sysfs': '12',
            'interfaces': [{'sysfs_path': path + '/3-1:1.0', 'number': '00',
                            'class': '02', 'subclass': '02', 'protocol': '00'}],
            'ttys': ['/dev/SYNTHETIC_NEVER_OPEN']}


class FakePort:
    def __init__(self, env, selected):
        self.env, self.selected = env, deepcopy(selected)
        self.buffer, self.input = bytearray(), bytearray()
        self.closed = False
        env.active += 1; env.peak_active = max(env.peak_active, env.active)

    def metadata(self):
        return {'transport': 'usb_cdc', 'usb_serial': SYNTHETIC_SERIAL,
                'sysfs_device': self.selected['sysfs_path'],
                'sysfs_interface': self.selected['interfaces'][0]['sysfs_path'],
                'interface_number': 2 if self.env.mode == 'wrong_open_role' else 0,
                'tx_acceptance': 'SYNTHETIC_OS_write_only_not_device_evidence'}

    def connected(self):
        return recovery.epoch_key(self.selected) in {recovery.epoch_key(d) for d in self.env.inventory()}

    def write_some(self, data, deadline):
        assert not self.closed and self.env.clock.now() < deadline
        if not self.connected(): raise OSError('SYNTHETIC_device_removed')
        # Short writes exercise the existing Collector's suffix-only handling.
        accepted = min(len(data), 7)
        self.input.extend(data[:accepted])
        self.env.clock.sleep(min(.001, deadline - self.env.clock.now()))
        while b'\n' in self.input:
            command, _, remaining = self.input.partition(b'\n'); self.input[:] = remaining
            if not command: continue
            command = command.decode('ascii'); self.env.commands.append(command)
            if command in {'USBBOOT', 'RESTART'}:
                self.env.actions.append(command)
                assert self.env.action_time is None, 'the action was replayed'
                self.env.action_time = self.env.clock.now()
                if self.env.mode != 'ack_lost':
                    self.buffer.extend(b'RACK,cmd=RESTART,id=9,state=accepted\n')
                    self.buffer.extend(b'RAPP,cmd=RESTART,id=9,state=restarting,target=' +
                                       (b'rom_download' if command == 'USBBOOT' else b'application') + b'\n')
                if self.env.mode == 'partial_then_restart': self.buffer.extend(b'SYNTHETIC_OLD_PARTIAL')
                if self.env.mode == 'action_write_unknown':
                    raise OSError('SYNTHETIC_write_completion_unknown_after_acceptance')
            elif self.env.mode != 'no_query_replies':
                self.buffer.extend((self.env.reply(command, self.selected) + '\n').encode('ascii'))
        return accepted

    def read_some(self, size, deadline):
        assert not self.closed
        if not self.connected(): raise OSError('SYNTHETIC_device_removed')
        remaining = max(0, deadline - self.env.clock.now())
        self.env.clock.sleep(min(.001 if self.buffer else .02, remaining))
        if not self.buffer: return None
        count = min(size, len(self.buffer), 47)
        value = bytes(self.buffer[:count]); del self.buffer[:count]
        return 'data', value

    def close(self):
        if not self.closed:
            self.closed = True; self.env.active -= 1


class Environment:
    def __init__(self, mode='normal', action='USBBOOT'):
        self.mode, self.action = mode, action
        self.clock, self.commands, self.actions = Clock(), [], []
        self.action_time = None
        self.active = self.peak_active = self.opens = 0
        self.failed_node_opens = 0
        self.old = device(serial='80B54E000000' if mode == 'wrong_mac' else SYNTHETIC_SERIAL)
        self.new = device(pid='1001' if action == 'USBBOOT' else '4042', number=11,
                          serial='00:11:22:aa:bb:cc' if action == 'USBBOOT' else SYNTHETIC_SERIAL)

    def inventory(self):
        if self.action_time is None or self.mode in {'ack_without_enumeration', 'no_query_replies', 'wrong_mac', 'pending'}:
            return [deepcopy(self.old)]
        age = self.clock.now() - self.action_time
        if age < .2: return [deepcopy(self.old)]
        if self.mode == 'cached_old_present': return [deepcopy(self.old), deepcopy(self.new)]
        if self.mode == 'path_reused_without_observed_absence': return [deepcopy(self.new)]
        if age < .5: return []
        if self.mode == 'same_epoch_reappears': return [deepcopy(self.old)]
        if self.failed_node_opens and self.mode in {'node_epoch_changed', 'node_wrong_mac', 'node_role_changed'}:
            changed = deepcopy(self.new)
            if self.mode == 'node_epoch_changed': changed['devnum'] += 1
            if self.mode == 'node_wrong_mac': changed['serial'] = changed['serial_raw'] = '80B54E000000'
            if self.mode == 'node_role_changed': changed['interfaces'][0]['number'] = '02'
            return [changed]
        return [deepcopy(self.new)]

    def sysfs_exists(self, path):
        return any(i['sysfs_path'] == path for d in self.inventory() for i in d['interfaces'])

    def open(self, serial):
        assert serial == SYNTHETIC_SERIAL
        assert self.active == 0, 'a second logical reader was opened'
        matches = [d for d in self.inventory() if recovery.formal_role(d) and d['serial'] == serial]
        assert len(matches) == 1
        assert matches[0]['pid'] == '4042', 'ROM must never be opened'
        if matches[0]['devnum'] != self.old['devnum']:
            pending = self.mode in {'node_eacces_forever', 'node_epoch_changed', 'node_wrong_mac', 'node_role_changed'} or (
                self.mode in {'node_eacces_transient', 'node_enoent_transient'} and self.failed_node_opens < 2)
            if pending:
                self.failed_node_opens += 1
                code = errno.ENOENT if self.mode == 'node_enoent_transient' else errno.EACCES
                raise OSError(code, 'SYNTHETIC_node_not_ready', '/dev/SYNTHETIC_NEVER_OPEN')
        self.opens += 1
        return FakePort(self, matches[0])

    def node_metadata(self, paths):
        missing = self.mode == 'node_enoent_transient'
        return [{'path': path, 'exists': not missing, 'uid': None if missing else 0,
                 'gid': None if missing else 0, 'mode': None if missing else '0660',
                 'stat_errno': errno.ENOENT if missing else None,
                 'fixture': 'SYNTHETIC_udev_node_state'} for path in paths]

    def reply(self, command, selected):
        assert command in {c for c, _ in recovery.QUERIES}, command
        is_new = selected['devnum'] != self.old['devnum']
        session = 'AAAABBBBCCCCDDDD' if is_new and self.mode != 'same_session' else '1111222233334444'
        boot = 1 if is_new else 17  # Deliberately decreases; never compare magnitudes across boots.
        if command == 'BUILD?':
            version = 'changed-image' if is_new and self.mode == 'changed_build' else '4a183bc-synthetic'
            return 'BUILD,idf=v5.5.5,target=esp32s3,project=SensorArray,proto=2,version=' + version + ',elfSha256=' + 'ab'*32
        if command == 'PROTO?':
            caps = 'usb_cdc_data_v1/config_query_v1/restart_query_v1/usb_rom_handoff_v1'
            if self.mode == 'missing_capability': caps = caps.replace('/usb_rom_handoff_v1', '')
            return 'PROTO,version=2,caps=' + caps
        if command == 'CONFIG?':
            # New session counters intentionally reset, not monotonic globally.
            n = '1' if is_new else '901'
            return 'CONFIG,session=' + session + ',bootId=' + str(boot) + ''.join(',' + k + '=' + n for k in recovery.IDENTITY[2:])
        if command == 'USB?':
            return 'USB,backend=otg_cdc_v1,init=1,err=0x0,mounted=1,suspended=0,romAvailable=1,dataDtr=1,debugDtr=0'
        return ('RESTART,session=' + session + ',bootId=' + str(boot) +
                ',busy=' + ('1' if self.mode == 'pending' else '0') +
                ',phase=idle,pendingId=0,pendingKnown=0,pendingRev=0,lastId=0,last=none,reason=none,'
                'acceptedKnown=0,acceptedRev=0,resultKnown=0,rev=0,boundary=0,originalError=0x0,'
                'publishError=0x0,terminalQueued=0,serialQueued=0,serialError=0x0,netQueued=0,netError=0x0,target=application')


def run_case(work, name, *, mode='normal', action='USBBOOT', expected, actions=1, opens=1):
    env = Environment(mode, action)
    out = work / name
    result = recovery.observe(recovery.Options(out, name, usb_serial=SYNTHETIC_SERIAL, action=action, duration=2.5, query_timeout=.25),
                              inventory=env.inventory, port_factory=env.open, clock=env.clock.now,
                              sleep=env.clock.sleep, sysfs_exists=env.sysfs_exists, node_metadata=env.node_metadata)
    assert result['status'] == expected, (name, result)
    assert result['action_attempts'] == actions and len(env.actions) == actions, (name, env.actions)
    assert env.opens == opens and env.peak_active <= 1 and env.active == 0
    assert result['elapsed_seconds'] <= 2.5 + 1e-9
    assert not result['esptool_executed'] and not result['flash_written']
    assert not result['automatic_action_retry'] and not result['system_USB_reset_requested']
    assert result['requested_action'] == action
    assert result == json.loads((out / (name + '.summary.json')).read_text())
    for epoch in result['epochs']:
        assert epoch['partial_line_initial_bytes'] == 0
        if 'raw_sha256' in epoch:
            raw = Path(epoch['raw_file']).read_bytes()
            assert hashlib.sha256(raw).hexdigest() == epoch['raw_sha256']
            assert len(raw) == epoch['rx_bytes']
    if mode == 'ack_lost': assert result['epochs'][0]['publications'] == []
    if mode == 'ack_without_enumeration':
        assert result['epochs'][0]['publications']
        assert result['restart_queries_after_action'], 'read-only cache query was never attempted'
        assert any(item['phase'] == 'action' for item in result['epochs'][0]['restart_cache_snapshots'])
        assert any(json.loads(line).get('type') == 'control_record' and
                   json.loads(line).get('line', '').startswith('RESTART,')
                   for line in Path(result['events_file']).read_text().splitlines())
    if mode == 'partial_then_restart':
        assert result['epochs'][0]['partial_line_bytes'] == len(b'SYNTHETIC_OLD_PARTIAL')
        assert b'SYNTHETIC_OLD_PARTIAL' not in Path(result['epochs'][1]['raw_file']).read_bytes()
        assert result['after']['CONFIG?']['session'] != result['before']['CONFIG?']['session']
        assert int(result['after']['CONFIG?']['bootId']) < int(result['before']['CONFIG?']['bootId'])
    if mode == 'path_reused_without_observed_absence':
        assert result['old_epoch_absent_monotonic'] is not None
        assert result['old_interface_absent_monotonic'] is None
    if mode == 'cached_old_present': assert result['old_epoch_absent_monotonic'] is None
    if mode == 'action_write_unknown': assert result['action_host_write'] == 'outcome_unknown'
    if mode.startswith('node_'):
        waits = result['post_enumeration_open_waits']
        assert waits and len(waits) == env.failed_node_opens
        code = errno.ENOENT if mode == 'node_enoent_transient' else errno.EACCES
        assert all(w['errno'] == code and w['tty_nodes'][0]['fixture'].startswith('SYNTHETIC') for w in waits)
        assert len({(e['device']['busnum'], e['device']['devnum']) for e in result['epochs'][1:]}) == 1
        if mode == 'node_eacces_forever':
            assert result['elapsed_seconds'] == 2.5 and result['last_open_errno'] == errno.EACCES
            assert result['failure_reason'] == 'post_enumeration_node_open_deadline'
            assert not result['post_enumeration_open_wait_resolved'] and env.opens == 1
        elif mode in {'node_epoch_changed', 'node_wrong_mac', 'node_role_changed'}:
            assert env.failed_node_opens == 1 and env.opens == 1
            assert result['error'] == 'post_enumeration_identity_changed_no_reopen'
            assert 'post_enumeration_identity_change' in result
        else:
            assert len(waits) == 2 and result['post_enumeration_open_attempts'] == 3
            assert result['post_enumeration_open_wait_resolved']
            assert result['epochs'][-1]['open_succeeded'] and env.opens == 2
        assert all(not e['open_succeeded'] and not Path(e['raw_file']).exists()
                   for e in result['epochs'][1:] if 'open_errno' in e)
    return {'case': name, 'status': 'PASS_SYNTHETIC', 'observer_status': result['status'],
            'commands': env.commands, 'action_count': len(env.actions), 'reader_opens': env.opens,
            'peak_logical_readers': env.peak_active, 'fake_elapsed_seconds': result['elapsed_seconds'],
            'summary': str(out / (name + '.summary.json'))}


def test_sysfs(work):
    root = work / 'synthetic_sysfs'
    usb, tty = root / 'bus/usb/devices', root / 'class/tty'
    usb.mkdir(parents=True); tty.mkdir(parents=True)
    physical = root / 'physical/usb3/3-1'; physical.mkdir(parents=True)
    fields = {'idVendor': '303a', 'idProduct': '1001', 'serial': '00:11:22:aa:bb:cc',
              'busnum': '3', 'devnum': '11', 'speed': '12'}
    for name, value in fields.items(): (physical / name).write_text(value + '\n')
    (usb / '3-1').symlink_to(physical, target_is_directory=True)
    interface = physical / '3-1:1.0'; interface.mkdir()
    for name, value in {'bInterfaceNumber': '00', 'bInterfaceClass': '02',
                        'bInterfaceSubClass': '02', 'bInterfaceProtocol': '00'}.items():
        (interface / name).write_text(value + '\n')
    (usb / '3-1:1.0').symlink_to(interface, target_is_directory=True)
    (tty / 'ttyACM8').mkdir(); (tty / 'ttyACM8/device').symlink_to(interface, target_is_directory=True)
    inventory = recovery.usb_inventory(usb, tty)
    assert len(inventory) == 1 and inventory[0]['serial'] == SYNTHETIC_SERIAL
    assert inventory[0]['serial_raw'] == '00:11:22:aa:bb:cc'
    assert inventory[0]['pid'] == '1001' and inventory[0]['ttys'] == ['/dev/ttyACM8']
    assert not recovery.formal_role(inventory[0])
    assert recovery.normalize_serial('wrong') is None
    return {'case': 'synthetic_sysfs_ROM_colon_MAC', 'status': 'PASS_SYNTHETIC', 'inventory': inventory}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, help='Optional new JSON file; retains case evidence beside it.')
    args = parser.parse_args(argv)
    if args.output is None:
        with tempfile.TemporaryDirectory(prefix='sensorarray-software-recovery-synthetic-') as directory:
            run_cases(Path(directory), None)
    else:
        if args.output.exists():
            raise FileExistsError('existing test evidence')
        directory = args.output.with_name(args.output.name + '.evidence')
        directory.mkdir()
        run_cases(directory, args.output)


def run_cases(work, output):
    source = Path(recovery.__file__).read_bytes()
    ok_rom = 'ROM_ENUMERATION_OBSERVED_NOT_FLASHER_VERIFIED'
    ok_app = 'APPLICATION_RESTART_OBSERVED'
    deadline = 'DEADLINE_WITHOUT_REQUIRED_ENUMERATION_OR_IDENTITY'
    cases = [
        run_case(work, 'readonly', action=None, expected='READONLY_COMPLETE', actions=0),
        run_case(work, 'rom_once', expected=ok_rom),
        run_case(work, 'lost_ACK_enum_success', mode='ack_lost', expected=ok_rom),
        run_case(work, 'ACK_no_enum_timeout', mode='ack_without_enumeration', expected=deadline),
        run_case(work, 'cached_old_no_success', mode='cached_old_present', expected=deadline),
        run_case(work, 'reused_path_not_proof_of_removal', mode='path_reused_without_observed_absence', expected=deadline),
        run_case(work, 'same_epoch_not_new', mode='same_epoch_reappears', action='RESTART', expected=deadline),
        run_case(work, 'wrong_MAC_refused', mode='wrong_mac', expected='ERROR', actions=0, opens=0),
        run_case(work, 'query_timeout_no_action', mode='no_query_replies', expected='ERROR', actions=0),
        run_case(work, 'pending_request_no_action', mode='pending', expected='ERROR', actions=0),
        run_case(work, 'missing_capability_no_action', mode='missing_capability', expected='ERROR', actions=0),
        run_case(work, 'debug_role_refused', mode='wrong_open_role', expected='ERROR', actions=0),
        run_case(work, 'unknown_write_never_replayed', mode='action_write_unknown', expected=ok_rom),
        run_case(work, 'restart_new_epoch_empty_partial', mode='partial_then_restart', action='RESTART', expected=ok_app, opens=2),
        run_case(work, 'same_session_not_verified', mode='same_session', action='RESTART', expected=deadline, opens=2),
        run_case(work, 'changed_BUILD_not_verified', mode='changed_build', action='RESTART', expected=deadline, opens=2),
        run_case(work, 'transient_EACCES_then_ready', mode='node_eacces_transient', action='RESTART', expected=ok_app, opens=2),
        run_case(work, 'transient_ENOENT_then_ready', mode='node_enoent_transient', action='RESTART', expected=ok_app, opens=2),
        run_case(work, 'persistent_EACCES_original_deadline', mode='node_eacces_forever', action='RESTART', expected=deadline),
        run_case(work, 'epoch_change_during_permission_wait_refused', mode='node_epoch_changed', action='RESTART', expected='ERROR'),
        run_case(work, 'wrong_MAC_during_permission_wait_refused', mode='node_wrong_mac', action='RESTART', expected='ERROR'),
        run_case(work, 'interface_change_during_permission_wait_refused', mode='node_role_changed', action='RESTART', expected='ERROR'),
        test_sysfs(work),
    ]
    # Exclusivity is evidence protection, not an idempotent action retry.
    first = cases[0]['summary']; existing = Path(first).parent
    env = Environment(action=None)
    try:
        recovery.observe(recovery.Options(existing, 'readonly', usb_serial=SYNTHETIC_SERIAL), inventory=env.inventory, port_factory=env.open,
                         clock=env.clock.now, sleep=env.clock.sleep, sysfs_exists=env.sysfs_exists)
    except FileExistsError: pass
    else: raise AssertionError('existing evidence was overwritten')
    assert env.opens == 0 and not env.commands
    cases.append({'case': 'existing_evidence_not_overwritten', 'status': 'PASS_SYNTHETIC'})
    assert Path(recovery.__file__).read_bytes() == source, 'observer changed during test'
    report = {'fixture_type': 'SYNTHETIC_ONLY', 'hardware_accessed': False, 'status': 'PASS',
              'case_count': len(cases), 'cases': cases, 'retained_evidence_directory': str(work) if output is not None else None,
              'observer_source': str(Path(recovery.__file__)), 'observer_sha256': hashlib.sha256(source).hexdigest(),
              'test_source_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'scope': 'real observer and Collector with fake port/clock/sysfs; not HIL or flash validation'}
    if output is not None:
        with output.open('x') as stream:
            json.dump(report, stream, indent=2)
            stream.write('\n')
    print(json.dumps({'status': report['status'], 'synthetic_cases': len(cases),
                      'output': str(output) if output is not None else None}))


if __name__ == '__main__': main()
