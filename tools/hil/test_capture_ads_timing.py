#!/usr/bin/env python3
"""Synthetic protocol transcripts; these are not hardware measurements."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import capture_ads_timing as runner
import capture_reference_fixture as capture

HASH = 'a' * 64
IDENTITY = 'session=1234567890ABCDEF,bootId=1,rev=4,mgen=2,mrid=3,rgen=1,rrid=1,pgen=1,prid=1'

class Clock:
    def __init__(self): self.now = 100.0
    def monotonic(self): return self.now

class Port:
    def __init__(self, clock, behavior=None):
        self.clock, self.behavior = clock, behavior or {}
        self.pending = bytearray(); self.tx = bytearray(); self.commands = []; self.closed = False
        self.acted = bool(self.behavior.get('cached')); self.configs = 0; self.states = 0
    def metadata(self): return {'transport': 'usb_cdc', 'tx_acceptance': 'synthetic_transcript'}
    def close(self): self.closed = True
    def read_some(self, size, deadline):
        self.clock.now += .0001
        if self.pending:
            part = bytes(self.pending[:size]); del self.pending[:size]
            return 'data', part
        self.clock.now = min(deadline, self.clock.now + .05)
        return None
    def write_some(self, data, deadline):
        self.tx.extend(data)
        while b'\n' in self.tx:
            command, _, tail = self.tx.partition(b'\n'); self.tx[:] = tail
            if command:
                command = command.decode(); self.commands.append(command)
                reply = self.reply(command)
                if reply: self.pending.extend(reply.encode() + b'\n')
        return len(data)
    def line(self, tag, fields):
        return tag + ',' + ','.join(str(key) + '=' + str(value) for key, value in fields.items())
    def detail(self, fields):
        return dict(diagnostic=1, **({'probe':'TAIL'} if self.behavior.get('tail') else {}), id=8, **fields)
    def groups(self):
        tail = self.behavior.get('tail')
        return [(g, tail and g % 2 == 1, not tail and g % 2 == 1) for g in range(4)]
    def records(self):
        records = []
        for g, gpio, pulse in self.groups():
            records.extend((g, gpio, False, attempt, attempt == 1) for attempt in (1,2,3))
            if pulse: records.extend((g, gpio, True, attempt, False) for attempt in (2,3))
        return records
    def raw_record(self, index):
        g,gpio,held,attempt,bad = self.records()[index]
        payload = bytes.fromhex('C07B73C3' if attempt == 3 else 'C07B73C2')
        computed = runner.crc_by_division(payload)
        received = computed ^ int(bad)
        known = bad and not gpio
        fields = self.detail(dict(g=g, **({'source':'spi55_gpio1' if gpio else 'spi56'} if self.behavior.get('tail') else {}),
            attempt=attempt, kind='held_repeat' if held else 'primary', raw=int.from_bytes(payload,'big',signed=True),
            rawValid=int(not bad), fresh=int(not held and not bad), status='68', crcValid=int(not bad),
            parity=computed & 1, wireKnown=int(known), bytes=('68'+payload.hex()+f'{received:02X}').upper() if known else 'unknown',
            computed=f'{computed:02X}' if gpio or known else 'unknown', received=f'{received:02X}' if gpio or known else 'unknown',
            xor=f'{computed ^ received:02X}' if gpio or known else 'unknown', err='0x109' if bad else '0x0',
            observedUs=1100+g*1000+attempt*10, startCallUs=0, waitUs=10, readUs=80,
            cfg=10+g+(attempt if self.groups()[g][2] else 0), drdy='1/1/3'))
        if self.behavior.get('bad_crc_report') and index == 0: fields['computed']='FE'
        return fields
    def tail_record(self,index):
        g,gpio,held,attempt,bad = self.records()[index]
        raw = self.raw_record(index)
        payload = int(raw['raw']).to_bytes(4,'big',signed=True)
        received = runner.crc_by_division(payload) ^ int(bad)
        return self.detail(dict(g=g,attempt=attempt,source='spi55_gpio1' if gpio else 'spi56',
            tailApplicable=int(gpio),spiBits=55 if gpio else 56,lsbKnown=0,
            prefix=('0068'+payload.hex()+f'{received & 254:02X}').upper() if gpio else 'unknown',
            gpioKnown=int(gpio),gpioBit=(received & 1) if gpio else 0,assembledKnown=int(gpio),
            assembledCrc=f'{received:02X}' if gpio else '00',attempted=int(gpio),spiDone=int(gpio),frameDone=int(gpio),
            routeRestored=int(gpio),physicalVerified=0,hz=750000 if gpio else 0,actualHz=754000 if gpio else 0,
            startUs=1000+g*1000,spiUs=100 if gpio else 0,riseUs=110 if gpio else 0,sampleUs=111 if gpio else 0,
            fallUs=112 if gpio else 0,restoreUs=114 if gpio else 0))
    def reply(self, cmd):
        if cmd == 'BUILD?': return 'BUILD,version=test,elfSha256=' + HASH
        if cmd == 'PROTO?': return 'PROTO,version=2,caps=usb_cdc_data_v1/config_query_v1/ads_timing_probe_v1' + ('/ads_tail_probe_v1' if self.behavior.get('tail') else '') + ('/ads_tail_io_v1' if self.behavior.get('io') else '')
        if cmd == 'CONFIG?':
            self.configs += 1
            identity = IDENTITY.replace('rev=4','rev=5') if self.configs > 1 and self.behavior.get('changed') else IDENTITY
            return 'CONFIG,' + identity + ',state=CAPACITANCE,mode=CAP,err=0x0'
        if cmd == 'ROUTE?': return 'ROUTE,requested=CAP,mode=CAP,applied=1,known=1,verified=1,originalError=0x0,cleanupError=0x0'
        if cmd == 'SPI?': return 'SPI,phase=done,done=1,joined=1,timedOut=0,qualified=1,restored=1,hz=750000,actualHz=754000,err=0x0,run=0x0,cleanup=0x0'
        if cmd in {'ADSCHK=TIMING,200', 'ADSCHK=TAIL,200'}:
            self.acted = True
            return '' if self.behavior.get('ack_lost') else 'ACK,cmd=ADSCHK,timing=1,id=8,perGroup=200,groups=4,status=accepted' + (',probe=TAIL' if self.behavior.get('tail') else '')
        if cmd == 'ADSTIMING?':
            self.states += 1
            busy = int(not self.acted and self.behavior.get('prebusy',False))
            ident = 8 if self.acted else 0
            if self.behavior.get('replaced') and self.states > 2: ident=9
            return self.line('ADSTIMING',dict(session='1234567890ABCDEF',bootId=1,busy=busy,
                phase='pending' if busy else 'idle',ackLocal=0,pendingId=0,pendingRev=0,lastId=ident,
                last='completed' if ident else 'none',reason='probe_complete' if ident else 'none',acceptedRev=4 if ident else 0,
                resultKnown=int(bool(ident)),rev=4 if ident else 0,boundary=2,originalError='0x109' if ident else '0x0',
                cleanupError='0x0',result=int(bool(ident)),terminalQueued=int(bool(ident)),groups=4,perGroup=200,
                diagnostic=1,storageKnown=1,storageReady=int(not self.behavior.get('storage_unavailable',False)),
                storageBytes=14448,storageError='0x0',**({'pendingProbe':'none','lastProbe':'TAIL' if ident else 'none'} if self.behavior.get('tail') else {})))
        if cmd == 'ADSTIMING?RESULT,8':
            return self.line('ADSTIMING',self.detail(dict(science=0,seq=99,groups=4,n=200,done=1,mutated=1,
                restoreKnown=1,restoreVerified=int(not self.behavior.get('restore_unknown')),isolate=0,
                hz=750000,actualHz=754000,first='0x109',terminal='0x0',cleanup='0x0',startUs=900,endUs=5100,
                gen='1/15',crcGood='10/1190',crcBad='0/20',records=len(self.records()),reason='complete')))
        if cmd.startswith('ADSTIMING?GROUP,8,'):
            g=int(cmd.rsplit(',',1)[1]); _,gpio,pulse=self.groups()[g]
            return self.line('ADSTIMINGGROUP',self.detail(dict(g=g,
                **({'source':'spi55_gpio1' if gpio else 'spi56'} if self.behavior.get('tail') else {}),
                mode='PULSE' if pulse else 'CONT',pga='bypass' if g>=2 else 'gain1',n=200,a=200,v=195,crc=5,
                even=100,odd=95,up=5,down=0,other=0,status=0,freshErr=0,timeout=0,fenceMiss=0,regErr=0,
                heldA=200 if pulse else 0,heldV=200 if pulse else 0,heldCrc=0,heldUp=0,heldDown=0,heldOther=0,
                done=1,err='0x0',reason='complete')))
        if cmd.startswith('ADSTIMING?TIME,8,'):
            g=int(cmd.rsplit(',',1)[1]); _,gpio,pulse=self.groups()[g]
            return self.line('ADSTIMINGTIME',self.detail(dict(g=g,startUs=1000+g*1000,endUs=1900+g*1000,
                gen=10+g,records=5 if pulse else 3,omitted=0,heldEven=100 if pulse else 0,heldOdd=100 if pulse else 0,
                heldStatus=0,heldFreshErr=0,heldTimeout=0)))
        if cmd.startswith('ADSTIMING?REG,8,'):
            return self.line('ADSTIMINGREG',self.detail(dict(phase='restored' if cmd[-1]=='1' else 'saved',known=1,
                ID='03',POWER='02',INTERFACE='06',MODE0='00',MODE1='00',MODE2='0F',INPMUX='9A',REFMUX='24',OCAL='000000',FSCAL='000040')))
        if cmd.startswith('ADSTIMING?RECORD,8,'):
            return self.line('ADSTIMINGRAW',self.raw_record(int(cmd.rsplit(',',1)[1])))
        if cmd.startswith('ADSTIMING?TAIL,8,'):
            return self.line('ADSTIMINGTAIL',self.tail_record(int(cmd.rsplit(',',1)[1])))
        if cmd.startswith('ADSTIMING?IO,8,'):
            g,gpio,held,attempt,bad=self.records()[int(cmd.rsplit(',',1)[1])]
            return self.line('ADSTIMINGIO',self.detail(dict(g=g,attempt=attempt,
                source='spi55_gpio1' if gpio else 'spi56',stage='complete' if gpio else 'not_applicable',
                clockKnown=int(gpio),misoKnown=int(gpio),clkFunc=2 if gpio else 0,clkSig=68 if gpio else 0,
                clkIE=int(gpio),clkOE=int(gpio),clkPeriph=int(gpio),clkInv=0,clkOD=0,
                misoFunc=2 if gpio else 0,misoSig=69 if gpio else 0,misoIE=int(gpio),misoOE=int(gpio),
                misoPeriph=int(gpio),misoInv=0,misoOD=0,clkLevelKnown=int(gpio),clkLevel=int(gpio),physicalVerified=0)))
        return cmd[:-1] + ',state=synthetic'

class TranscriptTests(unittest.TestCase):
    def mutated(self, command, changes, behavior=None, probe='TIMING', when=lambda port: True):
        original = Port.reply
        def reply(port, actual):
            text = original(port, actual)
            if actual == command and when(port):
                fields = runner.parse_fields(text)
                for key, value in changes.items():
                    if value is None: fields.pop(key, None)
                    else: fields[key] = str(value)
                return port.line(text.split(',',1)[0], fields)
            return text
        with patch.object(Port, 'reply', reply):
            return self.perform(behavior, probe=probe)

    def perform(self, behavior=None, expected=HASH, probe='TIMING', result_id=None, expected_session=None):
        clock = Clock(); port = Port(clock, behavior)
        with tempfile.TemporaryDirectory() as tmp:
            options = SimpleNamespace(label='synthetic', usb_serial='001122AABBCC', expected_elf=expected, probe=probe, output_directory=Path(tmp),
                                      result_id=result_id,expected_session=expected_session)
            with patch.object(runner, 'time', clock), patch.object(capture, 'time', clock):
                result = runner.run(options, port_factory=lambda serial: port)
            recorded = json.loads((Path(tmp) / 'synthetic.summary.json').read_text())
            self.assertEqual(recorded, result)
            self.assertTrue(port.closed)
            self.assertLessEqual(clock.now - 100, 90)
            self.assertLessEqual(sum(command.startswith('ADSCHK=') for command in port.commands), 1)
            return result, port
    def test_complete_retains_bad_crc_and_independent_check(self):
        result, port = self.perform()
        self.assertEqual(result['status'], 'DIAGNOSTIC_COMPLETE')
        self.assertEqual(result['raw_records'][0]['independent_crc']['xor'], 1)
        self.assertFalse(result['raw_records'][0]['independent_crc']['crc_matches'])
        self.assertEqual(port.commands.count('ADSCHK=TIMING,200'), 1)
    def test_missing_ack_is_not_resent(self):
        result, port = self.perform({'ack_lost': True})
        self.assertEqual(result['status'], 'ERROR'); self.assertEqual(result['action_attempts'], 1)
        self.assertEqual(port.commands.count('ADSCHK=TIMING,200'), 1)
    def test_full_hash_mismatch_prevents_action(self):
        result, port = self.perform(expected='b'*64)
        self.assertEqual(result['status'], 'ERROR'); self.assertEqual(result['action_attempts'], 0)
    def test_busy_prevents_action(self):
        result, port = self.perform({'prebusy': True})
        self.assertEqual(result['status'], 'ERROR'); self.assertEqual(result['action_attempts'], 0)
    def test_storage_unavailable_prevents_action(self):
        result, port = self.perform({'storage_unavailable': True})
        self.assertEqual(result['error'], 'diagnostic_storage_unavailable')
        self.assertEqual(result['action_attempts'], 0)
    def test_independent_crc_mismatch_is_error(self):
        result, _ = self.perform({'bad_crc_report': True})
        self.assertEqual(result['error'], 'independent_ADC_CRC_recomputation_disagrees')
    def test_config_change_is_error(self):
        result, _ = self.perform({'changed': True})
        self.assertEqual(result['error'], 'configuration_changed_during_diagnostic')
    def test_unverified_restore_is_incomplete(self):
        result, _ = self.perform({'restore_unknown': True})
        self.assertEqual(result['status'], 'DIAGNOSTIC_INCOMPLETE')
    def test_replaced_result_is_error(self):
        result, _ = self.perform({'replaced': True})
        self.assertEqual(result['error'], 'diagnostic_result_replaced')
    def test_tail_requires_separate_capability(self):
        result, port = self.perform(probe='TAIL')
        self.assertEqual(result['error'], 'diagnostic_capability_missing')
        self.assertEqual(result['action_attempts'], 0)
    def test_tail_complete_keeps_split_provenance_and_bad_crc(self):
        result, port = self.perform({'tail': True}, probe='TAIL')
        self.assertEqual(result['status'], 'DIAGNOSTIC_COMPLETE')
        crc = result['raw_records'][3]['independent_crc']
        self.assertTrue(crc['split_observation'])
        self.assertFalse(crc['wire_observed'])
        self.assertEqual(crc['physically_sampled_gpio_bit'], 1)
        self.assertEqual(crc['xor'], 1)
        self.assertEqual(port.commands.count('ADSCHK=TAIL,200'), 1)
        self.assertFalse(any(command.startswith('ADSTIMING?IO,') for command in port.commands))
    def test_split_validation_rejects_missing_or_false_provenance(self):
        port = Port(Clock(), {'tail': True})
        fields = runner.parse_fields(port.reply('ADSTIMING?RECORD,8,3'))
        tail = runner.parse_fields(port.reply('ADSTIMING?TAIL,8,3'))
        for changes in ({'prefix':'0068C07B73C2FD'}, {'gpioKnown':'0'},
                        {'assembledCrc':'FC'}, {'sampleUs':'99'}, {'g':'0'},
                        {'physicalVerified':'1'}, {'gpioBit':'2'}):
            with self.subTest(changes=changes), self.assertRaises(runner.CaptureError):
                runner.verify_tail_record(fields, dict(tail, **changes))
        good_fields = dict(fields, received='FC', xor='00')
        good_tail = dict(tail, gpioBit='0', assembledCrc='FC')
        self.assertTrue(runner.verify_tail_record(good_fields, good_tail)['crc_matches'])
        self.assertEqual(runner.verify_tail_record(fields, dict(tail, assembledKnown='0'))['reason'],
                         'incomplete_split_observation')
    def test_known_wire_examples(self):
        self.assertEqual(runner.crc_by_division(bytes.fromhex('C07B73C2')), 0xFC)
        self.assertEqual(runner.crc_by_division(bytes.fromhex('D63C6F05')), 0x8F)
        self.assertEqual(runner.verify_wire_record({'wireKnown':'0'})['wire_observed'], False)

    def test_nonzero_terminal_cleanup_or_nonCRC_first_is_incomplete(self):
        for field in ('terminal','cleanup','first'):
            with self.subTest(field=field):
                result, port = self.mutated('ADSTIMING?RESULT,8', {field:'0x107'})
                self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')
                self.assertEqual(len(result['raw_records']),16)
                self.assertEqual(port.commands.count('ADSCHK=TIMING,200'),1)

    def test_incomplete_groups_cannot_be_completed_by_summary(self):
        for changes in ({'a':1,'done':0,'err':'0x107'}, {'even':99}, {'heldA':199}):
            with self.subTest(changes=changes):
                result,_ = self.mutated('ADSTIMING?GROUP,8,1',changes)
                self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')

    def test_truncated_summary_group_time_register_or_raw_is_error(self):
        for command, key in (('ADSTIMING?RESULT,8','terminal'),('ADSTIMING?GROUP,8,0','reason'),
                ('ADSTIMING?TIME,8,0','heldTimeout'),('ADSTIMING?REG,8,1','FSCAL'),
                ('ADSTIMING?RECORD,8,0','drdy')):
            with self.subTest(command=command):
                result,_=self.mutated(command,{key:None})
                self.assertEqual(result['status'],'ERROR')
                self.assertIn('missing_fields',result['error'])

    def test_wrong_register_phase_is_error(self):
        result,_=self.mutated('ADSTIMING?REG,8,1',{'phase':'saved'})
        self.assertEqual(result['error'],'register_phase_mismatch')

    def test_unknown_registers_are_incomplete(self):
        result,_=self.mutated('ADSTIMING?REG,8,1',{'known':0})
        self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')

    def test_old_ACK_id_cannot_claim_cached_result(self):
        result,port=self.mutated('ADSTIMING?',{'lastId':8},when=lambda port: not port.acted)
        self.assertEqual(result['error'],'ACK_reuses_preexisting_cached_id')
        self.assertEqual(port.commands.count('ADSCHK=TIMING,200'),1)
        self.assertNotIn('ADSTIMING?RESULT,8',port.commands)

    def test_final_state_different_boot_is_error(self):
        for changes in ({'session':'1234567890ABCDE0'},{'bootId':2}):
            result,_=self.mutated('ADSTIMING?',changes,when=lambda port:port.states>=3)
            self.assertEqual(result['error'],'probe_state_boot_identity_mismatch')

    def test_terminal_revision_is_tied_to_accepted_configuration(self):
        for key in ('acceptedRev','rev'):
            result,_=self.mutated('ADSTIMING?',{key:5},when=lambda port:port.acted)
            self.assertEqual(result['error'],'terminal_configuration_identity_mismatch')

    def test_closing_cached_terminal_change_is_error(self):
        result,_=self.mutated('ADSTIMING?',{'originalError':'0x107'},when=lambda port:port.states>=3)
        self.assertEqual(result['error'],'cached_terminal_changed')

    def test_tail_kind_is_mandatory_on_ACK_summary_state_and_detail(self):
        for command, changes in (('ADSCHK=TAIL,200',{'probe':None}),
                ('ADSTIMING?RESULT,8',{'probe':None}),('ADSTIMING?GROUP,8,0',{'probe':None}),
                ('ADSTIMING?',{'lastProbe':None})):
            result,_=self.mutated(command,changes,{'tail':True},'TAIL')
            self.assertEqual(result['status'],'ERROR')

    def test_group_mode_PGA_and_source_are_checked(self):
        for changes in ({'mode':'PULSE'},{'pga':'bypass'},{'source':'spi56'},{'source':None}):
            result,_=self.mutated('ADSTIMING?GROUP,8,1',changes,{'tail':True},'TAIL')
            self.assertEqual(result['error'],'group_profile_or_source_mismatch')

    def test_record_flat_index_config_and_duplicates_are_checked(self):
        for command,changes,error in (
                ('ADSTIMING?RECORD,8,0',{'g':1},'record_flat_index_group_mismatch'),
                ('ADSTIMING?RECORD,8,0',{'cfg':11},'invalid_record_value_or_configuration'),
                ('ADSTIMING?RECORD,8,1',{'attempt':1},'duplicate_record_identity'),
                ('ADSTIMING?TIME,8,0',{'records':2},'group_record_category_count_mismatch')):
            result,_=self.mutated(command,changes)
            self.assertEqual(result['error'],error)

    def test_bad_wire_crc_cannot_be_claimed_valid(self):
        result,_=self.mutated('ADSTIMING?RECORD,8,0',{'rawValid':1,'crcValid':1})
        self.assertEqual(result['error'],'bad_CRC_record_claimed_valid')

    def test_split_metadata_missing_or_incomplete_is_not_complete(self):
        result,_=self.mutated('ADSTIMING?TAIL,8,3',{'assembledKnown':0},{'tail':True},'TAIL')
        self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')
        result,_=self.mutated('ADSTIMING?TAIL,8,3',{'routeRestored':0},{'tail':True},'TAIL')
        self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')
        result,_=self.mutated('ADSTIMING?TAIL,8,3',{'restoreUs':None},{'tail':True},'TAIL')
        self.assertEqual(result['status'],'ERROR')

    def test_preconditions_refuse_unsupported_mode_unknown_route_and_SPI(self):
        for command,changes in (('CONFIG?',{'mode':'RES','state':'RESISTANCE'}),
                ('ROUTE?',{'known':0}),('SPI?',{'joined':0}),('SPI?',{'qualified':0}),
                ('ADSTIMING?',{'storageKnown':0})):
            result,port=self.mutated(command,changes)
            self.assertEqual(result['status'],'ERROR')
            self.assertEqual(result['action_attempts'],0)
            self.assertFalse(any(cmd.startswith('ADSCHK=') for cmd in port.commands))

    def test_query_error_cannot_fall_back_to_preexisting_result(self):
        original=Port.reply
        def reply(port,command):
            if command=='ADSTIMING?RESULT,8': return 'ERR,cmd=ADSTIMING,reason=query_unavailable,err=0x103'
            return original(port,command)
        with patch.object(Port,'reply',reply): result,port=self.perform()
        self.assertEqual(result['status'],'ERROR')
        self.assertIn('device_error:ADSTIMING?RESULT,8',result['error'])
        self.assertEqual(port.commands.count('ADSCHK=TIMING,200'),1)

    def test_PULSE_START_generation_and_uint32_wrap(self):
        port=Port(Clock())
        stats=runner.parse_fields(port.reply('ADSTIMING?GROUP,8,1'))
        timing=runner.parse_fields(port.reply('ADSTIMING?TIME,8,1'))
        timing['gen']='4294967295'
        fields=port.raw_record(3)
        fields={key:str(value) for key,value in fields.items()}
        fields['cfg']='0'
        runner.validate_record(fields,1,stats,timing,'TIMING',set())
        fields['cfg']='4294967295'
        with self.assertRaisesRegex(runner.CaptureError,'invalid_record_value_or_configuration'):
            runner.validate_record(fields,1,stats,timing,'TIMING',set())

    def failed_reply(self, original):
        def reply(port,command):
            text=original(port,command)
            if not port.acted: return text
            fields=runner.parse_fields(text)
            if command=='CONFIG?': fields.update(rev='5',state='DEGRADED',mode='NONE',err='0x103')
            if command=='ADSTIMING?': fields.update(last='failed',reason='probe_fault_requires_isolation',
                acceptedRev='4',rev='5',originalError='0x103')
            if command=='ADSTIMING?RESULT,8':fields.update(done='0',isolate='1',first='0x103',terminal='0x103')
            if command=='ADSTIMING?GROUP,8,1':fields.pop('mode',None)
            if command=='ADSTIMING?TIME,8,2':fields.pop('gen',None)
            return port.line(text.split(',',1)[0],fields)
        return reply

    def test_failed_terminal_revision_change_collects_all_details(self):
        with patch.object(Port,'reply',self.failed_reply(Port.reply)):
            result,port=self.perform()
        self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')
        self.assertEqual(result['accepted_revision'],'4');self.assertEqual(result['terminal_revision'],'5')
        self.assertTrue(result['configuration_changed']);self.assertTrue(result['validation_errors'])
        self.assertEqual(len(result['group_results']),4);self.assertEqual(len(result['registers']),2)
        self.assertEqual(len(result['raw_records']),16)
        self.assertEqual(port.commands.count('ADSCHK=TIMING,200'),1)
        self.assertIn('ADSTIMING?RECORD,8,15',port.commands)

    def test_readonly_failed_cached_result_has_zero_actions(self):
        with patch.object(Port,'reply',self.failed_reply(Port.reply)):
            result,port=self.perform({'cached':True,'tail':True},probe='TAIL',result_id=8,
                                     expected_session='1234567890ABCDEF')
        self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')
        self.assertTrue(result['query_only']);self.assertEqual(result['action_attempts'],0)
        self.assertTrue(result['collection_complete'])
        self.assertTrue(result['cached_revision_differs_from_current'])
        self.assertEqual(len(result['raw_records']),12)
        self.assertIn('ADSTIMING?TAIL,8,11',port.commands)
        self.assertFalse(any(command.startswith('ADSCHK=') or '=' in command for command in port.commands))

    def test_readonly_wrong_session_ELF_or_cached_ID_refuses_details(self):
        for args,error in (({'expected_session':'1234567890ABCDE0'},'cached_result_expected_session_mismatch'),
                           ({'expected': 'b'*64},'running_full_BUILD_mismatch'),
                           ({'result_id':7},'cached_result_unavailable_or_replaced')):
            options=dict(result_id=8,expected_session='1234567890ABCDEF');options.update(args)
            result,port=self.perform({'cached':True},**options)
            self.assertEqual(result['error'],error);self.assertEqual(result['action_attempts'],0)
            self.assertFalse(any(command.startswith('ADSTIMING?RESULT') or '=' in command for command in port.commands))

    def test_readonly_requires_explicit_session_before_open(self):
        with tempfile.TemporaryDirectory() as tmp:
            options=SimpleNamespace(label='synthetic', usb_serial='001122AABBCC',expected_elf=HASH,probe='TAIL',result_id=8,
                                    expected_session=None,output_directory=Path(tmp))
            with self.assertRaisesRegex(ValueError,'requires explicit'):
                runner.run(options,port_factory=lambda serial:self.fail('port must remain unopened'))

    def test_readonly_query_failure_keeps_remaining_records(self):
        original=Port.reply
        def reply(port,command):
            if command=='ADSTIMING?GROUP,8,1':return 'ERR,cmd=ADSTIMING,reason=query_unavailable,err=0x103'
            return original(port,command)
        with patch.object(Port,'reply',reply):
            result,port=self.perform({'cached':True},result_id=8,expected_session='1234567890ABCDEF')
        self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')
        self.assertFalse(result['collection_complete']);self.assertEqual(len(result['query_errors']),1)
        self.assertEqual(len(result['raw_records']),16)
        self.assertIn('ADSTIMING?RECORD,8,15',port.commands)
        self.assertEqual(result['action_attempts'],0)

    def test_IO_capability_gates_new_queries_and_keeps_peripheral_OE_semantics(self):
        result,port=self.perform({'tail':True,'io':True},probe='TAIL')
        self.assertEqual(result['status'],'DIAGNOSTIC_COMPLETE')
        self.assertEqual(sum(command.startswith('ADSTIMING?IO,') for command in port.commands),12)
        self.assertEqual(result['raw_records'][3]['io_evidence']['misoOE'],'1')
        self.assertEqual(result['raw_records'][3]['io_evidence']['misoPeriph'],'1')
        self.assertFalse(result['raw_records'][3]['io_validation']['physical_verified'])

    def test_IO_wrong_identity_or_physical_claim_is_error(self):
        for changes,error in (({'g':0},'tail_IO_record_identity_mismatch'),
                              ({'physicalVerified':1},'unsupported_physical_IO_verification_claim')):
            result,_=self.mutated('ADSTIMING?IO,8,3',changes,{'tail':True,'io':True},'TAIL')
            self.assertEqual(result['error'],error)

    def test_readonly_IO_collects_without_action(self):
        result,port=self.perform({'tail':True,'io':True,'cached':True},probe='TAIL',result_id=8,
                                 expected_session='1234567890ABCDEF')
        self.assertTrue(result['collection_complete']);self.assertEqual(result['action_attempts'],0)
        self.assertEqual(sum(command.startswith('ADSTIMING?IO,') for command in port.commands),12)

    def test_readonly_unknown_record_count_has_bounded_fallback(self):
        original=Port.reply
        def reply(port,command):
            if command.startswith('ADSTIMING?RECORD,8,') and int(command.rsplit(',',1)[1])>=len(port.records()):
                return 'ERR,cmd=ADSTIMING,reason=query_unavailable,err=0x104'
            text=original(port,command)
            if command in {'ADSTIMING?RESULT,8','ADSTIMING?TIME,8,0'}:
                fields=runner.parse_fields(text);fields.pop('records',None)
                return port.line(text.split(',',1)[0],fields)
            return text
        with patch.object(Port,'reply',reply):
            result,port=self.perform({'cached':True},result_id=8,expected_session='1234567890ABCDEF')
        self.assertEqual(result['status'],'DIAGNOSTIC_INCOMPLETE')
        self.assertEqual(result['record_count_source'],'bounded_fallback_32_unknown_count')
        self.assertEqual(sum(command.startswith('ADSTIMING?RECORD,') for command in port.commands),32)
        self.assertEqual(result['action_attempts'],0);self.assertFalse(result['collection_complete'])

    def test_readonly_cached_result_replaced_is_never_success(self):
        # Cached mode has only two STATE queries; explicitly replace on the second.
        original=Port.reply
        def reply(port,command):
            text=original(port,command)
            if command=='ADSTIMING?' and port.states>=2:return text.replace('lastId=8','lastId=9')
            return text
        with patch.object(Port,'reply',reply):
            result,port=self.perform({'cached':True},result_id=8,expected_session='1234567890ABCDEF')
        self.assertEqual(result['error'],'cached_result_boot_or_id_changed')
        self.assertFalse(result['collection_complete']);self.assertEqual(result['action_attempts'],0)

if __name__ == '__main__': unittest.main()
