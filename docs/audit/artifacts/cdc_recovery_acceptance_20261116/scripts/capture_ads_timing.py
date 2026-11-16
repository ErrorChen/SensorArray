#!/usr/bin/env python3
"""One explicit ADS timing request; immutable raw records and read-only result queries.

No reset, rail calibration, speed selection or configuration setter. The sole
mutation is one explicitly selected ADSCHK=TIMING,200 or ADSCHK=TAIL,200. Missing ACK is outcome unknown, never a resend.
This is a diagnostic, not a scientific accuracy or throughput acceptance test.
With --result-id and --expected-session, this tool performs cached-result queries
only. Exit zero then means collection finished, never that the diagnostic passed.
"""
import argparse
from contextlib import ExitStack
import hashlib
import json
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / 'tools'))
from capture_reference_fixture import CaptureOptions, CaptureError, Collector
from sensorarray_usb_cdc import UsbCdcPort
from text_protocol import parse_fields

SERIAL = '80B54ECDD9AC'
IDENTITY = ('session', 'bootId', 'rev', 'mgen', 'mrid', 'rgen', 'rrid', 'pgen', 'prid')
QUERIES = (('BUILD?', 'BUILD'), ('PROTO?', 'PROTO'), ('CONFIG?', 'CONFIG'),
           ('ROUTE?', 'ROUTE'), ('SPI?', 'SPI'), ('ADSTIMING?', 'ADSTIMING'),
           ('BLESVC?', 'BLESVC'), ('SCHEDWHY?', 'SCHEDWHY'))


def require(fields, keys, context):
    missing = [key for key in keys.split() if key not in fields]
    if missing:
        raise CaptureError(context + '_missing_fields:' + '/'.join(missing))


def unsigned(fields, key, bits=32):
    value = fields.get(key, '')
    if not re.fullmatch(r'[0-9]+', value) or int(value) >= 1 << bits:
        raise CaptureError('invalid_unsigned_field:' + key)
    return int(value)


def flag(fields, key):
    if fields.get(key) not in {'0', '1'}:
        raise CaptureError('invalid_boolean_field:' + key)
    return fields[key] == '1'


def error_code(fields, key):
    value = fields.get(key, '')
    if not re.fullmatch(r'0x[0-9a-fA-F]{1,8}', value):
        raise CaptureError('invalid_error_field:' + key)
    return int(value, 16)


def kind_matches(fields, expected, key='probe'):
    # Images predating TAIL omit these fields and have TIMING semantics only.
    return fields.get(key, 'TIMING' if expected == 'TIMING' else None) == expected


def validate_state(state, config, probe_kind, ident=None, allow_failed_revision=False):
    require(state, 'session bootId busy phase ackLocal pendingId pendingRev lastId last reason '
            'acceptedRev resultKnown rev boundary originalError cleanupError result terminalQueued '
            'groups perGroup diagnostic storageKnown storageReady storageBytes storageError', 'state')
    if state['session'] != config.get('session') or state['bootId'] != config.get('bootId'):
        raise CaptureError('probe_state_boot_identity_mismatch')
    for key in ('bootId', 'pendingId', 'pendingRev', 'lastId', 'acceptedRev', 'rev',
                'boundary', 'groups', 'perGroup', 'storageBytes'):
        unsigned(state, key)
    for key in ('busy', 'ackLocal', 'resultKnown', 'result', 'terminalQueued',
                'diagnostic', 'storageKnown', 'storageReady'):
        flag(state, key)
    for key in ('originalError', 'cleanupError', 'storageError'):
        error_code(state, key)
    if state['groups'] != '4' or state['perGroup'] != '200' or state['diagnostic'] != '1':
        raise CaptureError('probe_state_geometry_mismatch')
    if ident is not None:
        if state['lastId'] != ident or state['busy'] != '0':
            raise CaptureError('diagnostic_result_replaced')
        failed_transition = allow_failed_revision and state['last'] in {'failed', 'rejected'}
        if (state['phase'] != 'idle' or state['pendingId'] != '0' or
                state['pendingRev'] != '0' or state['ackLocal'] != '0' or
                (state['resultKnown'] != '1' and not failed_transition) or state['acceptedRev'] != config.get('rev') or
                (state['rev'] != config.get('rev') and not failed_transition) or not kind_matches(state, probe_kind, 'lastProbe')):
            raise CaptureError('terminal_configuration_identity_mismatch')
        if state['last'] not in {'completed', 'failed', 'rejected'}:
            raise CaptureError('invalid_terminal_outcome')


def validate_preconditions(pre, probe_kind):
    config, route, spi = (pre[key] for key in ('CONFIG?', 'ROUTE?', 'SPI?'))
    require(config, ' '.join(IDENTITY) + ' state mode err', 'config')
    if not re.fullmatch(r'[0-9A-Fa-f]{16}', config['session']):
        raise CaptureError('invalid_config_session')
    for key in IDENTITY[1:]:
        unsigned(config, key)
    expected_state = {'CAP': 'CAPACITANCE', 'VOLT': 'VOLTAGE'}.get(config['mode'])
    if expected_state is None or config['state'] != expected_state or error_code(config, 'err'):
        raise CaptureError('diagnostic_requires_active_CAP_or_VOLT')
    require(route, 'requested mode applied known verified originalError cleanupError', 'route')
    if (route['mode'] != config['mode'] or route['requested'] != config['mode'] or
            any(not flag(route, key) for key in ('applied', 'known', 'verified')) or
            error_code(route, 'originalError') or error_code(route, 'cleanupError')):
        raise CaptureError('diagnostic_requires_known_software_route')
    # This is a software-readback prerequisite, never an analog isolation claim.
    require(spi, 'phase done joined timedOut qualified restored hz actualHz err run cleanup', 'spi')
    if (spi['phase'] != 'done' or any(not flag(spi, key) for key in ('done', 'joined', 'qualified', 'restored')) or
            flag(spi, 'timedOut') or any(error_code(spi, key) for key in ('err', 'run', 'cleanup')) or
            not unsigned(spi, 'hz') or not unsigned(spi, 'actualHz')):
        raise CaptureError('diagnostic_requires_joined_qualified_SPI')
    validate_state(pre['ADSTIMING?'], config, probe_kind)
    if (pre['ADSTIMING?']['storageKnown'] != '1' or pre['ADSTIMING?']['storageReady'] != '1' or
            error_code(pre['ADSTIMING?'], 'storageError')):
        raise CaptureError('diagnostic_storage_unavailable')


def validate_summary(probe, spi):
    require(probe, 'diagnostic science id seq groups n done mutated restoreKnown restoreVerified isolate '
            'hz actualHz first terminal cleanup startUs endUs gen crcGood crcBad records reason', 'summary')
    for key in ('id', 'seq', 'groups', 'n', 'hz', 'actualHz', 'records'):
        unsigned(probe, key)
    for key in ('diagnostic', 'science', 'done', 'mutated', 'restoreKnown', 'restoreVerified', 'isolate'):
        flag(probe, key)
    for key in ('first', 'terminal', 'cleanup'):
        error_code(probe, key)
    for key in ('gen', 'crcGood', 'crcBad'):
        parts = probe[key].split('/')
        if len(parts) != 2:
            raise CaptureError('invalid_counter_pair:' + key)
        for part in parts:
            unsigned({'value': part}, 'value')
    if (probe['science'] != '0' or probe['groups'] != '4' or probe['n'] != '200' or
            unsigned(probe, 'records') > 32):
        raise CaptureError('invalid_summary_geometry')
    if unsigned(probe, 'endUs', 64) < unsigned(probe, 'startUs', 64):
        raise CaptureError('summary_time_order_invalid')
    if probe['done'] == '1' and (probe['hz'] != spi['hz'] or probe['actualHz'] != spi['actualHz']):
        raise CaptureError('diagnostic_clock_changed')


def validate_group(stats, timing, group, probe_kind):
    counts = ('n a v crc even odd up down other status freshErr timeout fenceMiss regErr '
              'heldA heldV heldCrc heldUp heldDown heldOther')
    require(stats, 'diagnostic id g mode pga ' + counts + ' done err reason', 'group')
    require(timing, 'diagnostic id g startUs endUs gen records omitted heldEven heldOdd heldStatus '
            'heldFreshErr heldTimeout', 'group_time')
    for key in counts.split():
        unsigned(stats, key)
    for key in ('gen', 'records', 'omitted', 'heldEven', 'heldOdd', 'heldStatus', 'heldFreshErr', 'heldTimeout'):
        unsigned(timing, key)
    flag(stats, 'done'); error_code(stats, 'err')
    if unsigned(timing, 'endUs', 64) < unsigned(timing, 'startUs', 64):
        raise CaptureError('group_time_order_invalid')
    if stats['n'] != '200' or unsigned(timing, 'records') > 8:
        raise CaptureError('invalid_group_geometry')
    pulse = probe_kind == 'TIMING' and group % 2 == 1
    source = 'spi55_gpio1' if probe_kind == 'TAIL' and group % 2 == 1 else 'spi56'
    # An unstarted group can retain zero-initialized metadata after early refusal.
    if unsigned(stats, 'a') or flag(stats, 'done'):
        if (stats['mode'] != ('PULSE' if pulse else 'CONT') or
                stats['pga'] != ('bypass' if group >= 2 else 'gain1') or
                stats.get('source', 'spi56' if probe_kind == 'TIMING' else None) != source):
            raise CaptureError('group_profile_or_source_mismatch')
    if (sum(unsigned(stats, key) for key in ('up', 'down', 'other')) != unsigned(stats, 'crc') or
            sum(unsigned(stats, key) for key in ('heldUp', 'heldDown', 'heldOther')) != unsigned(stats, 'heldCrc')):
        raise CaptureError('group_CRC_classification_mismatch')
    completed = (flag(stats, 'done') and error_code(stats, 'err') == 0 and
        unsigned(stats, 'a') == 200 and unsigned(stats, 'v') + unsigned(stats, 'crc') == 200 and
        unsigned(stats, 'even') + unsigned(stats, 'odd') == unsigned(stats, 'v') and
        all(unsigned(stats, key) == 0 for key in ('status', 'freshErr', 'timeout', 'fenceMiss', 'regErr')) and
        all(unsigned(timing, key) == 0 for key in ('heldStatus', 'heldFreshErr', 'heldTimeout')))
    if pulse:
        completed &= (unsigned(stats, 'heldA') == 200 and
            unsigned(stats, 'heldV') + unsigned(stats, 'heldCrc') == 200 and
            unsigned(timing, 'heldEven') + unsigned(timing, 'heldOdd') == unsigned(stats, 'heldV'))
    else:
        completed &= all(unsigned(stats, key) == 0 for key in ('heldA', 'heldV', 'heldCrc'))
        completed &= unsigned(timing, 'heldEven') == unsigned(timing, 'heldOdd') == 0
    if completed:
        categories = sum(unsigned(stats, key) > 0 for key in ('even', 'odd', 'up', 'down', 'other',
                                                              'heldUp', 'heldDown', 'heldOther'))
        categories += sum(unsigned(timing, key) > 0 for key in ('heldEven', 'heldOdd'))
        if (unsigned(timing, 'records') != min(categories, 8) or
                unsigned(timing, 'omitted') != max(categories - 8, 0)):
            raise CaptureError('group_record_category_count_mismatch')
    return bool(completed)


def validate_registers(fields, index):
    require(fields, 'diagnostic id phase known ID POWER INTERFACE MODE0 MODE1 MODE2 INPMUX REFMUX OCAL FSCAL', 'registers')
    if fields['phase'] != ('restored' if index else 'saved'):
        raise CaptureError('register_phase_mismatch')
    flag(fields, 'known')
    for key in ('ID', 'POWER', 'INTERFACE', 'MODE0', 'MODE1', 'MODE2', 'INPMUX', 'REFMUX', 'OCAL', 'FSCAL'):
        if not re.fullmatch(r'[0-9A-Fa-f]{' + ('6' if key in {'OCAL', 'FSCAL'} else '2') + r'}', fields[key]):
            raise CaptureError('invalid_register_bytes:' + key)


def validate_record(fields, expected_group, stats, timing, probe_kind, seen):
    require(fields, 'diagnostic id g attempt kind raw rawValid fresh status crcValid parity wireKnown '
            'bytes computed received xor err observedUs startCallUs waitUs readUs cfg drdy', 'record')
    if fields['g'] != str(expected_group):
        raise CaptureError('record_flat_index_group_mismatch')
    source = 'spi55_gpio1' if probe_kind == 'TAIL' and expected_group % 2 else 'spi56'
    if fields.get('source', 'spi56' if probe_kind == 'TIMING' else None) != source:
        raise CaptureError('record_source_mismatch')
    attempt = unsigned(fields, 'attempt')
    if not 1 <= attempt <= 200 or fields['kind'] not in {'primary', 'held_repeat'}:
        raise CaptureError('invalid_record_attempt_or_kind')
    if fields['kind'] == 'held_repeat' and (probe_kind != 'TIMING' or expected_group % 2 != 1):
        raise CaptureError('unexpected_held_record')
    identity = (expected_group, fields['kind'], attempt)
    if identity in seen:
        raise CaptureError('duplicate_record_identity')
    seen.add(identity)
    for key in ('rawValid', 'fresh', 'crcValid', 'parity', 'wireKnown'):
        flag(fields, key)
    for key in ('startCallUs', 'waitUs', 'readUs', 'cfg'):
        unsigned(fields, key)
    # TIME.gen is captured before the group's first PULSE START. Each actual
    # START increments configurationGeneration once, including wrap to zero.
    expected_generation = (unsigned(timing, 'gen') + (attempt if stats['mode'] == 'PULSE' else 0)) & 0xFFFFFFFF
    if (unsigned(fields, 'cfg') != expected_generation or not re.fullmatch(r'-?[0-9]+', fields['raw']) or
            not -(1 << 31) <= int(fields['raw']) < 1 << 31 or
            not re.fullmatch(r'[0-9A-Fa-f]{2}', fields['status'])):
        raise CaptureError('invalid_record_value_or_configuration')
    generations = fields['drdy'].split('/')
    if len(generations) != 3:
        raise CaptureError('invalid_record_DRDY_generations')
    for value in generations:
        unsigned({'value': value}, 'value')
    observed = unsigned(fields, 'observedUs', 64)
    if not unsigned(timing, 'startUs', 64) <= observed <= unsigned(timing, 'endUs', 64):
        raise CaptureError('record_outside_group_time')
    err = error_code(fields, 'err')
    if flag(fields, 'fresh') and (fields['kind'] == 'held_repeat' or not flag(fields, 'rawValid') or
            not flag(fields, 'crcValid') or err):
        raise CaptureError('invalid_record_freshness_claim')


def crc_by_division(data):
    dividend = int.from_bytes(data, 'big') << 8
    for bit in range(len(data) * 8 + 7, 7, -1):
        if dividend & (1 << bit):
            dividend ^= 0x107 << (bit - 8)
    return dividend & 255


def verify_wire_record(fields):
    """Independent polynomial division, not the production CRC loop."""
    if fields.get('wireKnown') != '1':
        return {'wire_observed': False, 'reason': 'firmware_did_not_capture_good_packet_bytes'}
    wire = fields.get('bytes', '')
    if not re.fullmatch(r'[0-9A-Fa-f]{12}', wire):
        raise CaptureError('incomplete_diagnostic_wire_record')
    values = bytes.fromhex(wire)
    computed = crc_by_division(values[1:5])
    if (fields.get('computed', '').upper() != f'{computed:02X}' or
        fields.get('received', '').upper() != f'{values[5]:02X}' or
        fields.get('xor', '').upper() != f'{computed ^ values[5]:02X}'):
        raise CaptureError('independent_ADC_CRC_recomputation_disagrees')
    if 'raw' in fields and (int(fields['raw']) != int.from_bytes(values[1:5], 'big', signed=True) or
            fields['status'].upper() != f'{values[0]:02X}' or int(fields['parity']) != computed & 1):
        raise CaptureError('wire_record_value_disagrees')
    if computed != values[5] and any(fields.get(key) == '1' for key in ('crcValid', 'rawValid', 'fresh')):
        raise CaptureError('bad_CRC_record_claimed_valid')
    return {'wire_observed': True, 'computed': computed, 'received': values[5],
            'xor': computed ^ values[5], 'crc_matches': computed == values[5]}


def verify_tail_record(fields, tail):
    """Keep 55 controller bits and the observed GPIO bit distinct; never guess CRC."""
    require(tail, 'diagnostic probe id g attempt source tailApplicable spiBits lsbKnown prefix gpioKnown '
            'gpioBit assembledKnown assembledCrc attempted spiDone frameDone routeRestored physicalVerified '
            'hz actualHz startUs spiUs riseUs sampleUs fallUs restoreUs', 'tail')
    for key in ('tailApplicable', 'lsbKnown', 'gpioKnown', 'gpioBit', 'assembledKnown',
                'attempted', 'spiDone', 'frameDone', 'routeRestored', 'physicalVerified'):
        flag(tail, key)
    for key in ('id', 'g', 'attempt', 'spiBits', 'hz', 'actualHz', 'spiUs', 'riseUs', 'sampleUs', 'fallUs', 'restoreUs'):
        unsigned(tail, key)
    unsigned(tail, 'startUs', 64)
    if not re.fullmatch(r'[0-9A-Fa-f]{2}', tail['assembledCrc']):
        raise CaptureError('invalid_assembled_CRC_field')
    if any(fields.get(key) != tail.get(key) for key in ('id', 'g', 'attempt', 'source')):
        raise CaptureError('tail_record_identity_mismatch')
    if tail.get('physicalVerified') != '0':
        raise CaptureError('unsupported_physical_verification_claim')
    if fields.get('source') == 'spi56':
        if (tail.get('tailApplicable') != '0' or tail.get('spiBits') != '56' or
                tail.get('gpioKnown') != '0' or tail.get('assembledKnown') != '0'):
            raise CaptureError('standard_record_mislabeled_as_GPIO_tail')
        return verify_wire_record(fields)
    if (fields.get('source') != 'spi55_gpio1' or fields.get('wireKnown') != '0' or
            fields.get('bytes') != 'unknown' or tail.get('tailApplicable') != '1' or
            tail.get('spiBits') != '55' or tail.get('lsbKnown') != '0'):
        raise CaptureError('invalid_split_tail_provenance')
    result = {'wire_observed': False, 'split_observation': True,
              'physical_signal_integrity_verified': False}
    if tail.get('assembledKnown') != '1':
        return dict(result, reason='incomplete_split_observation')
    if any(tail.get(key) != '1' for key in ('attempted', 'spiDone', 'gpioKnown', 'frameDone')):
        raise CaptureError('assembled_tail_missing_physical_observation')
    if not re.fullmatch(r'[0-9A-Fa-f]{14}', tail.get('prefix', '')):
        raise CaptureError('incomplete_SPI55_prefix')
    prefix = bytes.fromhex(tail['prefix'])
    if prefix[6] & 1 or tail.get('gpioBit') not in {'0', '1'}:
        raise CaptureError('invalid_SPI55_unknown_bit_or_GPIO_bit')
    received = prefix[6] | int(tail['gpioBit'])
    computed = crc_by_division(prefix[2:6])
    if (tail.get('assembledCrc', '').upper() != f'{received:02X}' or
            fields.get('computed', '').upper() != f'{computed:02X}' or
            fields.get('received', '').upper() != f'{received:02X}' or
            fields.get('xor', '').upper() != f'{computed ^ received:02X}'):
        raise CaptureError('independent_split_ADC_CRC_recomputation_disagrees')
    if (fields.get('raw') is not None and
            (int(fields['raw']) != int.from_bytes(prefix[2:6], 'big', signed=True) or
             fields['status'].upper() != f'{prefix[1]:02X}' or int(fields['parity']) != computed & 1)):
        raise CaptureError('split_record_value_disagrees')
    if computed != received and any(fields.get(key) == '1' for key in ('crcValid', 'rawValid', 'fresh')):
        raise CaptureError('bad_split_CRC_record_claimed_valid')
    times = [int(tail[key]) for key in ('spiUs', 'riseUs', 'sampleUs', 'fallUs', 'restoreUs')]
    if times != sorted(times) or min(times) < 0:
        raise CaptureError('tail_observation_time_order_invalid')
    return dict(result, prefix_hex=prefix.hex().upper(), physically_sampled_gpio_bit=int(tail['gpioBit']),
                computed=computed, received=received, xor=computed ^ received,
                crc_matches=computed == received, route_restored=tail.get('routeRestored') == '1')


def same_identity(before, after):
    return all(before.get(key) is not None and before[key] == after.get(key) for key in IDENTITY)


def verify_io_record(fields, io):
    require(io, 'diagnostic probe id g attempt source stage clockKnown misoKnown '
            'clkFunc clkSig clkIE clkOE clkPeriph clkInv clkOD misoFunc misoSig misoIE misoOE '
            'misoPeriph misoInv misoOD clkLevelKnown clkLevel physicalVerified', 'tail_io')
    if any(fields.get(key) != io.get(key) for key in ('id', 'g', 'attempt', 'source')):
        raise CaptureError('tail_IO_record_identity_mismatch')
    if io['physicalVerified'] != '0':
        raise CaptureError('unsupported_physical_IO_verification_claim')
    for key in ('clockKnown', 'misoKnown', 'clkIE', 'clkOE', 'clkPeriph', 'clkInv', 'clkOD',
                'misoIE', 'misoOE', 'misoPeriph', 'misoInv', 'misoOD', 'clkLevelKnown', 'clkLevel'):
        flag(io, key)
    for key, maximum in (('clkFunc',7), ('misoFunc',7), ('clkSig',511), ('misoSig',511)):
        if unsigned(io,key)>maximum: raise CaptureError('invalid_IO_configuration_field:'+key)
    if not re.fullmatch(r'[a-z0-9_]+',io['stage']): raise CaptureError('invalid_IO_stage')
    return {'stage':io['stage'], 'software_configuration_only':True, 'physical_verified':False}


def read_cached_result(options, port_factory=UsbCdcPort):
    """Recover one explicit cached result. This path has no action/send call.

    Partial or failed diagnostic records are retained without requiring success
    metadata. Collection success is separate from diagnostic success.
    """
    if not re.fullmatch(r'[A-Za-z0-9_-]+', options.label): raise ValueError('invalid label')
    if not re.fullmatch(r'[0-9a-fA-F]{64}', options.expected_elf): raise ValueError('full ELF hash required')
    expected_session = getattr(options, 'expected_session', None)
    if not expected_session or not re.fullmatch(r'[0-9a-fA-F]{16}', expected_session):
        raise ValueError('--result-id requires explicit --expected-session of 16 hex digits')
    ident = str(options.result_id)
    if not re.fullmatch(r'[1-9][0-9]{0,9}', ident) or int(ident) > 0xFFFFFFFF:
        raise ValueError('invalid cached result id')
    kind = getattr(options, 'probe', 'TIMING')
    if kind not in {'TIMING', 'TAIL'}: raise ValueError('unsupported diagnostic kind')
    out = options.output_directory; out.mkdir(parents=True, exist_ok=True)
    paths = {key: out / (options.label + suffix) for key, suffix in
             [('raw', '.rx.bin'), ('events', '.events.jsonl'), ('summary', '.summary.json')]}
    if any(p.exists() or Path(str(p) + '.gz').exists() for p in paths.values()):
        raise FileExistsError('evidence exists; use a new label')
    started = time.monotonic(); deadline = started + 90.0
    result = {'status':'RUNNING', 'mode':'READ_ONLY_CACHED_RESULT', 'query_only':True,
              'probe_kind':kind, 'result_id':ident, 'expected_session':expected_session.upper(),
              'action':None, 'action_attempts':0, 'automatic_action_retry':False,
              'boot_reset_requested':False, 'calibration_written':False, 'configuration_setter_sent':False,
              'deadline_seconds':90, 'scientific_acceptance':'NOT_APPLICABLE_cached_diagnostic_only',
              'collection_complete':False, 'detail_validation':'identity_and_available_CRC_only_not_success_acceptance',
              'query_errors':[], 'validation_errors':[],
              'group_results':[], 'registers':[], 'raw_records':[]}
    collector = port = None
    with ExitStack() as stack:
        raw=stack.enter_context(paths['raw'].open('xb')); events=stack.enter_context(paths['events'].open('x'))
        summary=stack.enter_context(paths['summary'].open('x'))
        try:
            port=port_factory(SERIAL)
            collector=Collector(CaptureOptions(None,90,out,options.label,command_timeout=2.0,
                transport='usb_cdc',usb_serial=SERIAL,usb_pid=0x4042),port,raw,events)
            collector.total_deadline=deadline; collector.phase='read_only_identity'
            before={command:collector.query(command,tag,deadline) for command,tag in
                    (('BUILD?','BUILD'),('PROTO?','PROTO'),('CONFIG?','CONFIG'),('ADSTIMING?','ADSTIMING'))}
            result['before']=before
            if before['BUILD?'].get('elfSha256','').lower()!=options.expected_elf.lower():
                raise CaptureError('running_full_BUILD_mismatch')
            collector.validate_protocol(before['PROTO?'])
            required='ads_tail_probe_v1' if kind=='TAIL' else 'ads_timing_probe_v1'
            if required not in before['PROTO?'].get('caps','').split('/'):
                raise CaptureError('diagnostic_capability_missing')
            io_available=kind=='TAIL' and 'ads_tail_io_v1' in before['PROTO?'].get('caps','').split('/')
            result['io_capability']=io_available
            config=before['CONFIG?']; state=before['ADSTIMING?']
            if config.get('session','').upper()!=expected_session.upper():
                raise CaptureError('cached_result_expected_session_mismatch')
            validate_state(state,config,kind)
            if state['busy']!='0' or state['lastId']!=ident or state['result']!='1':
                raise CaptureError('cached_result_unavailable_or_replaced')
            if not kind_matches(state,kind,'lastProbe'):
                raise CaptureError('cached_result_probe_kind_mismatch')
            result['terminal']=state
            def query(command,tag):
                # Explicit allowlist prevents adding a setter to this recovery path.
                if not (command in {'CONFIG?','ROUTE?','SPI?','FRONTENDS?','ADSTIMING?'} or
                        any(command.startswith('ADSTIMING?'+part+',') for part in
                            ('RESULT','GROUP','TIME','REG','RECORD','TAIL','IO'))):
                    raise CaptureError('non_read_only_command_refused')
                try: return collector.query(command,tag,deadline)
                except CaptureError as error:
                    result['query_errors'].append({'command':command,'error':str(error)})
                    if time.monotonic()>=deadline: raise
                    return {}
            def detail(part,tag,index=None):
                command='ADSTIMING?'+part+','+ident+(','+str(index) if index is not None else '')
                fields=query(command,tag)
                if fields and (fields.get('id')!=ident or fields.get('diagnostic')!='1' or
                               not kind_matches(fields,kind)):
                    result['validation_errors'].append({'command':command,'error':'cached_detail_identity_mismatch'})
                return fields
            for command,tag in (('ROUTE?','ROUTE'),('SPI?','SPI'),('FRONTENDS?','FRONTENDS')):
                before[command]=query(command,tag)
            collector.phase='read_only_cached_details'
            probe=detail('RESULT','ADSTIMING');result['probe']=probe
            for group in range(4):
                result['group_results'].append({'stats':detail('GROUP','ADSTIMINGGROUP',group),
                                                'timing':detail('TIME','ADSTIMINGTIME',group)})
            result['registers']=[detail('REG','ADSTIMINGREG',index) for index in range(2)]
            try: count=unsigned(probe,'records')
            except CaptureError:
                try:
                    counts=[unsigned(entry['timing'],'records') for entry in result['group_results']]
                    if any(value>8 for value in counts): raise CaptureError('invalid_group_record_count')
                    count=sum(counts)
                    result['record_count_source']='group_time_sum_after_missing_summary_count'
                except CaptureError:
                    count=32
                    result['record_count_source']='bounded_fallback_32_unknown_count'
                    result['validation_errors'].append({'scope':'records','error':'record_count_unknown'})
            else: result['record_count_source']='summary'
            if count>32: raise CaptureError('record_count_out_of_bounds')
            for index in range(count):
                record={'index':index,'fields':detail('RECORD','ADSTIMINGRAW',index)}
                result['raw_records'].append(record)
                if kind=='TAIL': record['tail_evidence']=detail('TAIL','ADSTIMINGTAIL',index)
                if io_available:record['io_evidence']=detail('IO','ADSTIMINGIO',index)
                try:
                    record['independent_crc']=(verify_tail_record(record['fields'],record['tail_evidence'])
                        if kind=='TAIL' else verify_wire_record(record['fields']))
                except (CaptureError,ValueError,KeyError) as error:
                    record['validation_error']=str(error)
                    result['validation_errors'].append({'record':index,'error':str(error)})
                if io_available:
                    try: record['io_validation']=verify_io_record(record['fields'],record['io_evidence'])
                    except (CaptureError,ValueError,KeyError) as error:
                        result['validation_errors'].append({'record':index,'io_error':str(error)})
            collector.phase='read_only_closing_identity'
            after={command:query(command,tag) for command,tag in
                   (('CONFIG?','CONFIG'),('ADSTIMING?','ADSTIMING'),('ROUTE?','ROUTE'),('SPI?','SPI'),('FRONTENDS?','FRONTENDS'))}
            result['after']=after;result['closing_config']=after['CONFIG?'];result['closing_probe_state']=after['ADSTIMING?']
            validate_state(after['ADSTIMING?'],config,kind)
            if (after['CONFIG?'].get('session')!=config.get('session') or after['CONFIG?'].get('bootId')!=config.get('bootId') or
                    after['ADSTIMING?']['lastId']!=ident or after['ADSTIMING?']['busy']!='0' or
                    not kind_matches(after['ADSTIMING?'],kind,'lastProbe')):
                raise CaptureError('cached_result_boot_or_id_changed')
            if any(after['ADSTIMING?'].get(key)!=state.get(key) for key in
                   ('last','acceptedRev','resultKnown','rev','originalError','cleanupError','result')):
                raise CaptureError('cached_terminal_changed')
            result['configuration_changed']=not same_identity(config,after['CONFIG?'])
            result['accepted_revision']=state['acceptedRev'];result['terminal_revision']=state['rev']
            result['cached_revision_differs_from_current']=state['acceptedRev']!=config.get('rev')
            result['collection_complete']=not result['query_errors'] and not result['validation_errors']
            # This recovery command does not re-run or retrospectively accept a diagnostic.
            result['status']='DIAGNOSTIC_INCOMPLETE'
        except (Exception,KeyboardInterrupt) as error:
            result.update(status='INTERRUPTED' if isinstance(error,KeyboardInterrupt) else 'ERROR',
                          error=str(error),error_class=type(error).__name__)
        finally:
            if collector:
                result.update(collector.transport_result(),firmware_faults=collector.faults,
                    reset_observations=collector.reset_observations,trailing_partial_line_bytes=len(collector.line_buffer),rx_bytes=collector.raw_bytes)
                collector.event('finished',status=result['status'],error=result.get('error'))
            if port:
                try:port.close()
                except Exception as error:result.update(status='ERROR',close_error=str(error),collection_complete=False)
            raw.flush();result.update(raw_sha256=hashlib.sha256(paths['raw'].read_bytes()).hexdigest(),
                source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),elapsed_seconds=time.monotonic()-started)
            summary.write(json.dumps(result,indent=2)+'\n')
    return result


def run(options, port_factory=UsbCdcPort):
    if getattr(options, 'result_id', None) is not None:
        return read_cached_result(options, port_factory)
    if not re.fullmatch(r'[A-Za-z0-9_-]+', options.label): raise ValueError('invalid label')
    if not re.fullmatch(r'[0-9a-fA-F]{64}', options.expected_elf): raise ValueError('full ELF hash required')
    out = options.output_directory
    out.mkdir(parents=True, exist_ok=True)
    paths = {key: out / (options.label + suffix) for key, suffix in
             [('raw', '.rx.bin'), ('events', '.events.jsonl'), ('summary', '.summary.json')]}
    if any(p.exists() or Path(str(p) + '.gz').exists() for p in paths.values()):
        raise FileExistsError('evidence exists; use a new label')
    probe_kind = getattr(options, 'probe', 'TIMING')
    if probe_kind not in {'TIMING', 'TAIL'}: raise ValueError('unsupported diagnostic kind')
    started = time.monotonic()
    deadline = started + 90.0
    result = {'status': 'RUNNING', 'probe_kind': probe_kind, 'action': 'ADSCHK=' + probe_kind + ',200', 'action_attempts': 0,
              'automatic_action_retry': False, 'deadline_seconds': 90,
              'scientific_acceptance': 'NOT_APPLICABLE_diagnostic_only',
              'boot_reset_requested': False, 'calibration_written': False,
              'configuration_setter_sent': False, 'group_results': [], 'raw_records': []}
    collector, port = None, None
    with ExitStack() as stack:
        raw = stack.enter_context(paths['raw'].open('xb'))
        events = stack.enter_context(paths['events'].open('x'))
        summary = stack.enter_context(paths['summary'].open('x'))
        try:
            port = port_factory(SERIAL)
            collector = Collector(CaptureOptions(None, 90, out, options.label,
                command_timeout=2.0, transport='usb_cdc', usb_serial=SERIAL,
                usb_pid=0x4042), port, raw, events)
            collector.total_deadline = deadline
            collector.phase = 'preconditions'
            pre = {command: collector.query(command, tag, deadline) for command, tag in QUERIES}
            result['before'] = pre
            if pre['BUILD?'].get('elfSha256', '').lower() != options.expected_elf.lower():
                raise CaptureError('running_full_BUILD_mismatch')
            collector.validate_protocol(pre['PROTO?'])
            required_probe_cap = 'ads_tail_probe_v1' if probe_kind == 'TAIL' else 'ads_timing_probe_v1'
            if required_probe_cap not in pre['PROTO?'].get('caps', '').split('/'):
                raise CaptureError('diagnostic_capability_missing')
            io_available=probe_kind=='TAIL' and 'ads_tail_io_v1' in pre['PROTO?'].get('caps','').split('/')
            result['io_capability']=io_available
            if pre['ADSTIMING?'].get('busy') != '0':
                raise CaptureError('diagnostic_owner_busy')
            if pre['ADSTIMING?'].get('storageReady') == '0':
                raise CaptureError('diagnostic_storage_unavailable')
            config = pre['CONFIG?']
            validate_preconditions(pre, probe_kind)
            collector.phase = 'acceptance'
            collector.lines.clear()
            action_deadline = min(deadline, time.monotonic() + 3)
            result['action_attempts'] = 1  # Even a partial host write may have effects.
            collector.send(result['action'], action_deadline)
            ack = parse_fields(collector.wait(lambda line: line.startswith('ACK,') and
                parse_fields(line).get('cmd') == 'ADSCHK' and parse_fields(line).get('timing') == '1',
                action_deadline, result['action']))
            ident = ack.get('id', '')
            if not re.fullmatch(r'[1-9][0-9]{0,9}', ident) or int(ident) > 0xFFFFFFFF:
                raise CaptureError('invalid_ACK_id')
            require(ack, 'cmd timing id perGroup groups status', 'ack')
            if ack['perGroup'] != '200' or ack['groups'] != '4' or ack['status'] != 'accepted':
                raise CaptureError('invalid_ACK_acceptance')
            if ident == pre['ADSTIMING?']['lastId']:
                raise CaptureError('ACK_reuses_preexisting_cached_id')
            if not kind_matches(ack, probe_kind):
                raise CaptureError('ACK_probe_kind_mismatch')
            result['ack'] = ack
            collector.phase = 'await_cached_terminal'
            terminal_deadline = min(deadline, time.monotonic() + 14)
            result['terminal_queries'] = []
            terminal = None
            while time.monotonic() < terminal_deadline:
                state = collector.query('ADSTIMING?', 'ADSTIMING', terminal_deadline)
                result['terminal_queries'].append(state)
                validate_state(state, config, probe_kind)
                if state.get('lastId') == ident and state.get('busy') == '0':
                    validate_state(state, config, probe_kind, ident, allow_failed_revision=True)
                    terminal = state
                    break
                until = min(terminal_deadline, time.monotonic() + .15)
                while time.monotonic() < until:
                    collector.pump(until)
                    collector.lines.clear()
            if terminal is None: raise CaptureError('terminal_unknown_no_action_retry')
            result['terminal'] = terminal
            result['accepted_revision'] = terminal['acceptedRev']
            result['terminal_revision'] = terminal['rev']
            partial = terminal['last'] != 'completed'
            result['validation_errors'] = []
            if terminal.get('result') != '1': raise CaptureError('diagnostic_rejected_or_no_result')
            collector.phase = 'immutable_result_queries'
            def assess(scope, function, *args):
                try: return function(*args)
                except (CaptureError, ValueError, KeyError, TypeError) as error:
                    if not partial: raise
                    result['validation_errors'].append({'scope':scope,'error':str(error)})
                    return None
            def detail(kind, tag, index=None):
                command = 'ADSTIMING?' + kind + ',' + ident + (',' + str(index) if index is not None else '')
                try: fields = collector.query(command, tag, deadline)
                except CaptureError as error:
                    if not partial or time.monotonic() >= deadline: raise
                    result['validation_errors'].append({'command':command,'error':str(error)})
                    return {}
                if fields.get('id') != ident or fields.get('diagnostic') != '1':
                    if not partial: raise CaptureError('result_id_mismatch')
                    result['validation_errors'].append({'command':command,'error':'result_id_mismatch'})
                    return fields
                if not kind_matches(fields, probe_kind):
                    if not partial: raise CaptureError('result_probe_kind_mismatch')
                    result['validation_errors'].append({'command':command,'error':'result_probe_kind_mismatch'})
                if kind in {'GROUP', 'TIME'} and fields.get('g') != str(index):
                    if not partial: raise CaptureError('result_group_mismatch')
                    result['validation_errors'].append({'command':command,'error':'result_group_mismatch'})
                return fields
            probe = detail('RESULT', 'ADSTIMING')
            result['probe'] = probe
            assess('summary', validate_summary, probe, pre['SPI?'])
            groups_complete = True
            for group in range(4):
                entry = {'stats': detail('GROUP', 'ADSTIMINGGROUP', group),
                         'timing': detail('TIME', 'ADSTIMINGTIME', group)}
                result['group_results'].append(entry)
                groups_complete &= assess('group_' + str(group), validate_group,
                    entry['stats'], entry['timing'], group, probe_kind) is True
                if not partial and unsigned(entry['timing'], 'startUs', 64) and not (
                        unsigned(probe, 'startUs', 64) <= unsigned(entry['timing'], 'startUs', 64) <=
                        unsigned(entry['timing'], 'endUs', 64) <= unsigned(probe, 'endUs', 64)):
                    raise CaptureError('group_outside_probe_time')
            result['registers'] = []
            for index in range(2):
                registers = detail('REG', 'ADSTIMINGREG', index)
                result['registers'].append(registers)
                assess('register_' + str(index), validate_registers, registers, index)
            count = assess('record_count', unsigned, probe, 'records')
            if count is None:
                counts = [assess('fallback_group_count',unsigned,entry['timing'],'records') for entry in result['group_results']]
                if all(value is not None and value<=8 for value in counts):
                    count=sum(counts)
                    result['record_count_source']='group_time_sum_after_missing_summary_count'
                else:
                    count=32
                    result['record_count_source']='bounded_fallback_32_unknown_count'
            if not 0 <= count <= 32: raise CaptureError('record_count_out_of_bounds')
            record_groups = []
            for group, entry in enumerate(result['group_results']):
                group_count = assess('group_record_count_' + str(group), unsigned, entry['timing'], 'records')
                if group_count is not None and group_count > 8:
                    if not partial: raise CaptureError('group_record_count_out_of_bounds')
                    result['validation_errors'].append({'scope':'group_'+str(group),'error':'group_record_count_out_of_bounds'})
                    group_count = 0
                record_groups.extend([group] * (group_count or 0))
            if len(record_groups) != count:
                if not partial: raise CaptureError('record_count_disagrees_with_groups')
                result['validation_errors'].append({'scope':'records','error':'record_count_disagrees_with_groups'})
            seen_records = set()
            split_complete = True
            for index in range(count):
                fields = detail('RECORD', 'ADSTIMINGRAW', index)
                record = {'index': index, 'fields': fields}
                result['raw_records'].append(record)
                if index < len(record_groups):
                    group = record_groups[index]
                    entry = result['group_results'][group]
                    assess('record_' + str(index), validate_record, fields, group,
                        entry['stats'], entry['timing'], probe_kind, seen_records)
                if probe_kind == 'TAIL':
                    record['tail_evidence'] = detail('TAIL', 'ADSTIMINGTAIL', index)
                    record['independent_crc'] = assess('tail_crc_' + str(index), verify_tail_record, fields, record['tail_evidence'])
                    tail = record['tail_evidence']
                    if tail.get('source') == 'spi55_gpio1':
                        split_complete &= tail.get('assembledKnown') == '1' and tail.get('routeRestored') == '1'
                        if not partial and tail['spiDone'] == '1' and (tail['hz'] != probe['hz'] or tail['actualHz'] != probe['actualHz']):
                            raise CaptureError('tail_clock_mismatch')
                else:
                    record['independent_crc'] = assess('wire_crc_' + str(index), verify_wire_record, fields)
                if io_available:
                    record['io_evidence']=detail('IO','ADSTIMINGIO',index)
                    record['io_validation']=assess('tail_io_'+str(index),verify_io_record,fields,record['io_evidence'])
            collector.phase = 'closing_identity'
            closing = collector.query('CONFIG?', 'CONFIG', deadline)
            final_state = collector.query('ADSTIMING?', 'ADSTIMING', deadline)
            result.update(closing_config=closing, closing_probe_state=final_state)
            if closing.get('session') != config.get('session') or closing.get('bootId') != config.get('bootId'):
                raise CaptureError('boot_changed_during_diagnostic')
            result['configuration_changed'] = not same_identity(config, closing)
            if result['configuration_changed'] and not partial:
                raise CaptureError('configuration_changed_during_diagnostic')
            validate_state(final_state, config, probe_kind, ident, allow_failed_revision=True)
            if any(final_state[key] != terminal[key] for key in ('last', 'reason', 'acceptedRev',
                    'resultKnown', 'rev', 'boundary', 'originalError', 'cleanupError', 'result')):
                raise CaptureError('cached_terminal_changed')
            if collector.reset_observations or collector.faults and not partial:
                raise CaptureError('firmware_fault_or_reset_observed')
            complete = (not partial and not result['validation_errors'] and probe['done'] == '1' and probe['restoreKnown'] == '1' and
                probe['restoreVerified'] == '1' and probe['isolate'] == '0' and
                error_code(probe, 'terminal') == error_code(probe, 'cleanup') == 0 and
                error_code(probe, 'first') in {0, 0x109} and groups_complete and split_complete and
                all(registers['known'] == '1' for registers in result['registers']) and
                terminal['last'] == 'completed' and error_code(terminal, 'cleanupError') == 0 and
                error_code(terminal, 'originalError') in {0, 0x109})
            result['status'] = 'DIAGNOSTIC_COMPLETE' if complete else 'DIAGNOSTIC_INCOMPLETE'
        except (Exception, KeyboardInterrupt) as error:
            result.update(status='INTERRUPTED' if isinstance(error, KeyboardInterrupt) else 'ERROR',
                          error=str(error), error_class=type(error).__name__)
        finally:
            if collector:
                result.update(collector.transport_result(), firmware_faults=collector.faults,
                              reset_observations=collector.reset_observations,
                              trailing_partial_line_bytes=len(collector.line_buffer), rx_bytes=collector.raw_bytes)
                collector.event('finished', status=result['status'], error=result.get('error'))
            if port:
                try: port.close()
                except Exception as error: result.update(status='ERROR', close_error=str(error))
            raw.flush()
            result.update(raw_sha256=hashlib.sha256(paths['raw'].read_bytes()).hexdigest(),
                          source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                          elapsed_seconds=time.monotonic() - started)
            summary.write(json.dumps(result, indent=2) + '\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', type=Path, required=True)
    parser.add_argument('--label', required=True)
    parser.add_argument('--expected-elf', required=True)
    parser.add_argument('--probe', choices=('TIMING', 'TAIL'), default='TIMING')
    parser.add_argument('--result-id', type=int, help='Read this cached ID only; never send ADSCHK or a setter')
    parser.add_argument('--expected-session', help='Required with --result-id: exact 16-hex current boot session')
    args = parser.parse_args()
    result = run(args)
    print(json.dumps({key: result[key] for key in ('status', 'action_attempts', 'elapsed_seconds')}), flush=True)
    return 0 if (result['status'] == 'DIAGNOSTIC_COMPLETE' or
                 result.get('query_only') and result.get('collection_complete')) else 1

if __name__ == '__main__':
    raise SystemExit(main())
