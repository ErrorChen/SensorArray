#!/usr/bin/env python3
"""Synthetic controller tests; never instantiate a real CDC device."""
from collections import deque
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

SOURCE = Path(__file__).with_name('check_wifi_owner_control.py')
spec = importlib.util.spec_from_file_location('wifi_owner_hil_control_test', SOURCE)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
REAL_WAIT = probe.Collector.wait
from sensorarray_capture_identity import CONFIG_FIELDS

ELF = 'a' * 64
SESSION = '1234567890ABCDEF'


class Port:
    closed = False

    def __init__(self, serial):
        assert serial == '80B54ECDD9AC'
        Port.closed = False

    def close(self):
        Port.closed = True


class Collector:
    scenario = ''
    sent = []

    def __init__(self, options, port, raw, events):
        self.raw, self.events = raw, events
        self.faults, self.reset_observations = [], []
        self.raw_bytes = 0
        self.lines = deque()
        self.cache = {}
        self.mode = 'OFF'
        self.pending = None
        self.next_id = 0
        Collector.sent = []
        self.config = {key: '0' for key in CONFIG_FIELDS}
        self.config.update(session=SESSION, bootId='1', rev='1', boundary='1',
                           state='CAPACITANCE', mode='CAP', rows='8', profile='CCCCCCCC',
                           err='0x0', merr='0x0', rerr='0x0', perr='0x0')

    def event(self, kind, **fields):
        self.events.write(json.dumps({'kind': kind, **fields}) + '\n')

    def query(self, command, tag, deadline):
        if command == 'CONFIG?':
            return dict(self.config)
        if command == 'BUILD?':
            return dict(idf='synthetic', target='esp32s3', project='SensorArray', proto='2',
                        version='synthetic', elfSha256='b' * 64 if self.scenario == 'wrong_image' else ELF)
        if command == 'CAPS2?':
            return {'caps': 'wifi_owner_v1'}
        if command == 'WIFI?':
            return dict(cmd='WIFI', session=SESSION, boot='1', bootKnown='1',
                        outputGate='0' if self.mode == 'OFF' else '1', pending='0',
                        id=str(self.next_id), quarantined='0', backend='ready')
        if command == 'WIFIGUARD?':
            return dict(session=SESSION, boot='1', bootKnown='1', active='0', ownerBusy='0',
                        quarantined='1' if self.scenario == 'guard_quarantine' else '0')
        if command == 'MEM?':
            return {'synthetic': '1'}
        if command == 'MEMFAIL?':
            fields = dict(v='1', scope='boot', session=SESSION, bootId='1', identityKnown='1',
                          elfSha256=ELF, regKnown='1', registered='1', countScope='recorded',
                          coverage='since_register', fn='opaque_pointer', incomplete='0',
                          count='1' if Collector.sent else '0')
            if self.scenario == 'trace_wrong_boot':
                fields['bootId'] = '2'
            if self.scenario == 'trace_wrong_elf_after' and Collector.sent:
                fields['elfSha256'] = 'b' * 64
            if self.scenario == 'trace_unregistered':
                fields['registered'] = '0'
            return fields
        if command.startswith('WIFIREQ?'):
            identifier = int(command[8:])
            item = dict(self.cache.get(identifier, dict(session=SESSION, boot='1', bootKnown='1',
                        id=str(identifier), known='0', outcome='unknown', reason='not_cached')))
            if self.scenario == 'different_boot':
                item['session'] = '0' * 16
            if self.scenario == 'wrong_epoch':
                item['epoch'] = '100'
            if self.scenario == 'terminal_epoch_overwritten' and len(Collector.sent) == 5 and identifier == 4:
                item['epoch'] = '123'
            if self.scenario == 'applied_with_error' and item.get('outcome') == 'applied':
                item['err'] = '0x101'
            return item
        raise AssertionError(command)

    def send(self, command, deadline):
        assert command in ('WIFI=AP', 'WIFI=OFF')
        Collector.sent.append(command)
        if command == 'WIFI=AP' and self.scenario == 'bootstrap_rejected':
            # Real lazy-resource failure precedes request allocation. The
            # actual Collector.wait rejects ERR before invoking its predicate.
            self.pending = ('ERR,cmd=WIFI,v=AP,ok=0,ver=2,accepted=0,pending=0,session=' + SESSION +
                            ',boot=1,bootKnown=1,id=0,epoch=0,outcome=unknown,e=0x101,scope=backend_and_gate')
            return
        self.next_id += 1
        identifier = self.next_id
        self.mode = command.split('=')[1]
        outcome = 'applied'
        if self.mode == 'AP' and self.scenario in ('backend_unavailable', 'terminal_overwritten'):
            outcome = 'quarantined'
        item = dict(session=SESSION, boot='1', bootKnown='1', known='1', id=str(identifier),
                    requested=self.mode, outcome=outcome, err='0x101' if outcome != 'applied' else '0x0',
                    cleanupErr='0x103' if outcome != 'applied' else '0x0', epoch=str(identifier),
                    scope='backend_and_gate' if self.mode == 'AP' else 'output_gate_only')
        self.cache[identifier] = item
        if len(self.cache) > 2:
            del self.cache[next(iter(self.cache))]
        if self.mode == 'OFF' and self.scenario == 'terminal_overwritten':
            self.cache[1]['outcome'] = 'applied'
        self.pending = 'ACK,cmd=WIFI,accepted=1,e=0x0,v=' + self.mode + ',' + ','.join(f'{k}={v}' for k, v in item.items())
        if self.scenario == 'false_accepted':
            self.pending = self.pending.replace('accepted=1', 'accepted=0')
        if self.scenario == 'duplicate_ACK' and identifier == 2:
            self.pending = self.pending.replace('id=2', 'id=1')

    def wait(self, predicate, deadline, command):
        if ((self.scenario == 'lost_AP_ACK' and len(Collector.sent) == 1) or
                (self.scenario == 'lost_OFF_ACK' and len(Collector.sent) == 5)):
            raise probe.CaptureError('synthetic_lost_ACK')
        return REAL_WAIT(self, predicate, deadline, command)

    def next_line(self, deadline):
        return self.pending

    def pump(self, deadline):
        raise AssertionError('terminal fixtures must finish without polling')


def main():
    scenarios = ('success', 'wrong_image', 'different_boot', 'backend_unavailable',
                 'lost_AP_ACK', 'lost_OFF_ACK', 'terminal_overwritten', 'guard_quarantine',
                 'false_accepted', 'applied_with_error', 'wrong_epoch', 'duplicate_ACK', 'bootstrap_rejected',
                 'terminal_epoch_overwritten', 'trace_success', 'trace_wrong_boot',
                 'trace_wrong_elf_after', 'trace_unregistered')
    for scenario in scenarios:
        with tempfile.TemporaryDirectory(prefix='wifi-control-synthetic-') as temp:
            base = Path(temp)
            source = base / 'scripts' / SOURCE.name
            source.parent.mkdir()
            source.write_bytes(SOURCE.read_bytes())
            Collector.scenario = scenario
            args = [str(source), '--label', scenario, '--usb-serial', '80B54ECDD9AC', '--output-directory', str(base), '--expected-elf', ELF,
                    '--expected-session', SESSION, '--expected-boot', '1']
            if scenario.startswith('trace_'):
                args.append('--allocation-trace')
            with patch.object(probe, '__file__', str(source)), patch.object(probe, 'Collector', Collector), \
                    patch.object(probe, 'UsbCdcPort', Port), patch.object(sys, 'argv', args), redirect_stdout(io.StringIO()):
                code = probe.main()
            result = json.loads((base / scenario / (scenario + '.summary.json')).read_text())
            assert Port.closed
            if scenario in ('success', 'trace_success'):
                assert code == 0 and result['status'] == 'AP_AND_CONTROL_QUERY_OBSERVED'
                assert Collector.sent == ['WIFI=AP', 'WIFI=AP', 'WIFI=OFF', 'WIFI=AP', 'WIFI=OFF']
            else:
                assert code == 1
            if scenario == 'trace_success':
                assert result['allocation_before']['count'] == '0'
                assert result['allocation_after']['count'] == '1'
            elif scenario in ('trace_wrong_boot', 'trace_unregistered'):
                assert Collector.sent == [] and result['off_restore'] == 'NOT_NEEDED'
            elif scenario == 'trace_wrong_elf_after':
                assert Collector.sent == ['WIFI=AP', 'WIFI=AP', 'WIFI=OFF', 'WIFI=AP']
                assert result['off_restore'] == 'NOT_SENT_AUTHORITY_REVOKED'
            if scenario == 'wrong_image':
                assert Collector.sent == [] and result['off_restore'] == 'NOT_NEEDED'
            elif scenario == 'different_boot':
                assert Collector.sent == ['WIFI=AP'] and result['off_restore'] == 'NOT_SENT_AUTHORITY_REVOKED'
            elif scenario == 'backend_unavailable':
                assert Collector.sent == ['WIFI=AP', 'WIFI=OFF']
                assert result['status'] == 'BACKEND_UNAVAILABLE_CONTROL_QUERY_OBSERVED'
                assert result['latest_AP_after_OFF']['outcome'] == 'quarantined'
            elif scenario == 'bootstrap_rejected':
                assert Collector.sent == ['WIFI=AP', 'WIFI=OFF']
                assert result['status'] == 'ERROR' and result['error'].startswith('device_error:WIFI=AP:ERR,')
                assert result['off_restore'] == 'SAME_BOOT_OUTPUT_GATE_QUERY_VERIFIED'
                assert 'first_AP_terminal' not in result
            elif scenario == 'lost_AP_ACK':
                assert Collector.sent == ['WIFI=AP', 'WIFI=OFF']
                assert result['off_restore'] == 'SAME_BOOT_OUTPUT_GATE_QUERY_VERIFIED'
                assert result['error'] == 'synthetic_lost_ACK'
            elif scenario in ('lost_OFF_ACK', 'terminal_overwritten', 'terminal_epoch_overwritten'):
                assert result['status'] == 'ERROR' and result['off_restore'] == 'FAILED_OR_UNKNOWN'
            elif scenario in ('guard_quarantine', 'false_accepted', 'applied_with_error', 'wrong_epoch', 'duplicate_ACK'):
                assert result['status'] == 'ERROR'
                if scenario == 'duplicate_ACK':
                    assert Collector.sent == ['WIFI=AP', 'WIFI=AP', 'WIFI=OFF']
            print('WIFI_CONTROL_CAPTURE_SYNTHETIC_PASS,' + scenario)
    print('scenarios=18,hardware=NOT_RUN,real_adapter_constructed=0')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
