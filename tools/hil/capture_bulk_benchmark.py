#!/usr/bin/env python3
"""Formal dual-interface bulk benchmark; never uses CDC for scientific evidence.

Before/after and periodic read-only USB/USBQ/BLELINK/BLETX/SCHED observations
share existing deadlines. DATA keeps flowing while CTRL queries run. Defaults:
120 s configurations, 600 s stability, 5 s warmup, 10 s diagnostic interval.
No USBBOOT, restart, NVS, input flush or automatic USB reconfiguration.
Optional service/output/protection/CPUOWNER observations are capability-gated and run only in
the existing before/after CONFIG brackets, outside the measured capture.
"""
from contextlib import contextmanager
import hashlib
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools'))
from capture_reference_fixture import CaptureError
import run_firmware_benchmark_retry as retry

base = retry.original
VARIANT = 'formal_bulk_diagnostics_v1'
WORKERS = ('bt', 'tr', 'log', 'ser', 'bc', 'sc', 'wc')
CAPABILITIES = frozenset(('config_query_v1', 'usb_vendor_bulk_v1', 'work_quantum_v2'))
EXTRA_REQUIRED = {
    'USB?': 'backend init err mounted suspended dma epoch resets mounts unmounts rxBytes rxDrop rxOver lastRxUs lastTxUs romAvailable quiesced pcReceipt dataDtr debugDtr',
    'USBQ?': 'dDrop cDrop dSubmit cSubmit expired cExpired partial dBytes cBytes dCallbackBytes cCallbackBytes queueMaxUs stackMinBytes callbackResult',
    'BLELINK?': 'gen connected mtu phyKnown txPhy rxPhy connKnown interval1250us latency timeout10ms phyReq connReq dleReq phyStatus connStatus dleStatus dleObserved dlePeerKnown dleTx dleRx stale',
    'BLETX?': 'gen dDequeued lDequeued cDequeued dMaxAgeUs lMaxAgeUs cMaxAgeUs sdkActive sdkStartUs sdkEndUs sdkCalls sdkAccepted sdkErrors sdkMaxUs sdkLastErr txPrio ctrlPrio pcReceipt',
    'SCHED?': 'initError registered idleKnown tickHz budgetTicks maxUnits graceUnits backstopTicks quantumTicks idleGen idleAgeTicks maxIdleGapTicks ' + ' '.join(WORKERS),
}
FRAME_OUTPUT_REQUIRED = {
    'BLEFLOW?': 'v scope counters gen stream active stage ratePm intentionalSkip probeSelected attempts capacityReject evidence rejectPm pressureWindows cleanUs localServiceMilliFps probeUs cleanTtlUs peerLoss rssi',
    'BLESNAP?': 'v scope counters gen stream queued localComplete failed admitMiss deadlineMiss superseded staleNormal known lastSeq lastRev lastCompleteAgeUs gapKnown maxCompleteGapUs firstPending periodicPending dueUs pcReceipt',
    'SNAPSRC?': 'v scope queued firstQueued periodicQueued candidateMiss firstMiss superseded firstSuperseded stage pcReceipt',
    'SNAPNET?': 'v scope readout queued admissionMiss firstMiss derived superseded retry expired bleExpired wifiExpired bleAccepted bleFailed policySkip obsolete stage pcReceipt',
}
OPTIONAL_REQUIRED = {
    'BLESVC?': 'v gen scope active started completed wallUs idleChanged idleSame idleUnknown wall cpu blocked',
    'SCHEDWHY?': 'v order overlap init registered idleKnown idleGen ' + ' '.join(WORKERS),
    **FRAME_OUTPUT_REQUIRED,
}
OPTIONAL_CAPABILITIES = {
    'BLESVC?': 'ble_service_obs_v1', 'SCHEDWHY?': 'work_reason_v1',
    'BLEFLOW?': 'ble_flow_v1', 'BLESNAP?': 'ble_snapshot_v1',
    'SNAPSRC?': 'ble_snapshot_v1', 'SNAPNET?': 'ble_snapshot_v1',
}
CPU_OWNER_CAPABILITY = 'cpu_owner_v1'
CPU_OWNER_QUERIES = tuple((f'CPUOWNER?{role}', 'CPUOWNER') for role in range(7))
CPU_OWNER_FIELDS = ('v session role enabled available known conflict stale reason taskNumber runUs '
                    't0Us t1Us observedAtUs ageUs sampleMinUs staleAfterUs scope isr sdk blocked')
CPU_OWNER_REQUIRED = {command: CPU_OWNER_FIELDS for command, _ in CPU_OWNER_QUERIES}
OBSERVATION_REQUIRED = {**OPTIONAL_REQUIRED, **CPU_OWNER_REQUIRED}
OBSERVATION_CAPABILITIES = {**OPTIONAL_CAPABILITIES,
                           **{command: CPU_OWNER_CAPABILITY for command in CPU_OWNER_REQUIRED}}
DIAGNOSTICS = tuple((command, command[:-1]) for command in EXTRA_REQUIRED)
CONFIG_CONSISTENCY = base.IDENTITY + ('state', 'mode', 'rows', 'profile', 'boundary', 'err')
BASE_VALIDATOR = retry.validate_reply
HEX_FIELDS = {'err', 'initError', 'phyReq', 'connReq', 'dleReq', 'sdkLastErr'}
BOOL_FIELDS = {'init', 'mounted', 'suspended', 'dma', 'romAvailable', 'quiesced', 'connected', 'dataDtr', 'debugDtr',
               'phyKnown', 'connKnown', 'dleObserved', 'dlePeerKnown', 'sdkActive', 'registered', 'idleKnown', 'active'}
U64_FIELDS = {'rxBytes', 'dBytes', 'cBytes', 'dCallbackBytes', 'cCallbackBytes', 'sdkStartUs', 'sdkEndUs', 'wallUs'}
OPTIONAL_LITERALS = {
    'BLESVC?': {'v': '1', 'scope': 'boot', 'wall': 'elapsed', 'cpu': 'unknown', 'blocked': 'unknown'},
    'SCHEDWHY?': {'v': '1', 'order': 'quantum/backstop/units', 'overlap': '1'},
}
FRAME_OUTPUT_LITERALS = {
    'BLEFLOW?': {'v': '1', 'scope': 'local_output', 'counters': 'boot',
                 'probeUs': '1000000', 'cleanTtlUs': '3000000', 'peerLoss': 'unknown', 'rssi': 'unknown'},
    'BLESNAP?': {'v': '1', 'scope': 'local_complete', 'counters': 'boot',
                 'dueUs': '1000000', 'pcReceipt': 'unknown'},
    'SNAPSRC?': {'v': '1', 'scope': 'boot', 'stage': 'source_queue', 'pcReceipt': 'unknown'},
    'SNAPNET?': {'v': '1', 'scope': 'boot', 'readout': 'sequential',
                 'stage': 'ble_admission', 'pcReceipt': 'unknown'},
}


def unsigned(value, command, field, bits=32):
    if not re.fullmatch(r'[0-9]+', value or '') or int(value) >= 1 << bits:
        raise retry.InvalidQueryReply(f'reply_invalid_unsigned:{command}:{field}')
    return int(value)


def validate_frame_output(command, fields):
    """Validate local counters without treating admission as peer delivery.

    Counter ordering is deliberately not inferred from modulo boot totals.
    Unknown historical age/service remains unknown; it is never changed to 0.
    """
    def reject(field):
        raise retry.InvalidQueryReply('reply_invalid_ble_output_semantics:' + command + ':' + field)
    required = FRAME_OUTPUT_REQUIRED[command].split()
    missing = [key for key in required if not fields.get(key)]
    if missing:
        raise retry.InvalidQueryReply('reply_missing_fields:' + command + ':' + ','.join(missing))
    booleans = {'active', 'evidence', 'known', 'gapKnown', 'firstPending', 'periodicPending'}
    u64 = {'intentionalSkip', 'probeSelected', 'cleanUs', 'lastCompleteAgeUs', 'maxCompleteGapUs'}
    maybe_unknown = {'localServiceMilliFps', 'lastSeq', 'lastRev', 'lastCompleteAgeUs', 'maxCompleteGapUs'}
    for key in required:
        value = fields[key]
        if key in FRAME_OUTPUT_LITERALS[command]:
            if value != FRAME_OUTPUT_LITERALS[command][key]: reject(key)
        elif key in booleans:
            if value not in {'0', '1'}: reject(key)
        elif command == 'BLEFLOW?' and key == 'stage':
            if value not in {'inactive', 'full', 'pressure', 'limited', 'unknown'}: reject(key)
        elif key in maybe_unknown and value == 'unknown':
            pass
        else:
            bits = 64 if key in u64 else (16 if key in {'ratePm', 'rejectPm'} else 8 if key == 'pressureWindows' else 32)
            number = unsigned(value, command, key, bits)
            if key in {'ratePm', 'rejectPm'} and number > 1000: reject(key)
            if key == 'pressureWindows' and number > 3: reject(key)
    if command == 'BLEFLOW?':
        active, evidence = fields['active'] == '1', fields['evidence'] == '1'
        stage, rate = fields['stage'], int(fields['ratePm'])
        if stage != 'unknown' and (stage == 'inactive') == active: reject('active_stage')
        if stage in {'full', 'pressure'} and rate != 1000: reject('full_rate')
        if stage == 'limited' and rate >= 1000: reject('limited_rate')
        if evidence and not active: reject('inactive_evidence')
        if fields['localServiceMilliFps'] != 'unknown' and not (active and evidence): reject('service_evidence')
    elif command == 'BLESNAP?':
        known, gap_known = fields['known'] == '1', fields['gapKnown'] == '1'
        if known:
            if fields['lastSeq'] == 'unknown' or fields['lastRev'] == 'unknown': reject('known_identity')
        elif any(fields[key] != 'unknown' for key in ('lastSeq', 'lastRev', 'lastCompleteAgeUs')):
            reject('unknown_completion')
        if gap_known:
            if not known or fields['maxCompleteGapUs'] == 'unknown': reject('known_gap')
        elif fields['maxCompleteGapUs'] != 'unknown': reject('unknown_gap')
    return fields


def validate_cpu_owner(command, fields):
    """Validate the actual CPUOWNER formatter, preserving unavailable history."""
    def reject(field):
        raise retry.InvalidQueryReply('reply_invalid_cpu_owner:' + command + ':' + field)
    missing = [key for key in CPU_OWNER_FIELDS.split() if not fields.get(key)]
    if missing:
        raise retry.InvalidQueryReply('reply_missing_fields:' + command + ':' + ','.join(missing))
    if fields['role'] != command[-1]: reject('role')
    if not re.fullmatch(r'[0-9A-Fa-f]{16}', fields['session']): reject('session')
    for key in ('enabled', 'available', 'known', 'conflict', 'stale'):
        if fields[key] not in {'0', '1'}: reject(key)
    literals = {'v': '1', 'sampleMinUs': '1000000', 'staleAfterUs': '3000000',
                'scope': 'switch_out', 'isr': 'may_include', 'sdk': 'not_isolated', 'blocked': 'unknown'}
    for key, expected in literals.items():
        if fields[key] != expected: reject(key)
    enabled, available, known, conflict, stale = (fields[k] == '1' for k in
        ('enabled', 'available', 'known', 'conflict', 'stale'))
    reason = fields['reason']
    if reason not in {'disabled', 'not_observed', 'ok', 'writer_conflict', 'counter_regressed', 'clock_regressed'}:
        reject('reason')
    if not enabled:
        if available or known or conflict or not stale or reason != 'disabled': reject('disabled_semantics')
    elif reason == 'disabled' or conflict != (reason == 'writer_conflict'):
        reject('enabled_conflict_semantics')
    if available != (enabled and known and not conflict and reason == 'ok'):
        reject('available_semantics')
    sample_fields = ('taskNumber', 't0Us', 't1Us', 'observedAtUs')
    if known:
        if reason in {'disabled', 'not_observed'}: reject('known_reason')
        values = {key: unsigned(fields[key], command, key, 32 if key == 'taskNumber' else 64)
                  for key in sample_fields}
        if values['t1Us'] < values['t0Us'] or values['observedAtUs'] != values['t1Us']:
            reject('sample_time_order')
    else:
        if any(fields[key] != 'unknown' for key in sample_fields): reject('unknown_sample')
        if available or not stale or reason in {'ok', 'counter_regressed'}: reject('unknown_semantics')
    if available:
        unsigned(fields['runUs'], command, 'runUs', 64)
    elif fields['runUs'] != 'unknown': reject('unavailable_runUs')
    if fields['ageUs'] == 'unknown':
        if not stale or available: reject('unknown_age')
    else:
        if not known: reject('unknown_sample_age')
        age = unsigned(fields['ageUs'], command, 'ageUs', 64)
        if stale != (age >= int(fields['staleAfterUs'])): reject('stale_age')
    return fields


def cpu_owner_evidence(fields):
    return {'usable_for_pairing': fields['available'] == '1' and fields['stale'] == '0',
            'historical': fields['known'] == '1' and fields['stale'] == '1',
            'reason': fields['reason'], 'current_cpu_utilization': 'NOT_DERIVED',
            'interpretation': 'Unavailable is unknown, not zero. Stale retains historical values only.'}


def validate_reply(command, fields, *, backend='otg_bulk_v1'):
    if command in FRAME_OUTPUT_REQUIRED:
        return validate_frame_output(command, fields)
    if command in CPU_OWNER_REQUIRED:
        return validate_cpu_owner(command, fields)
    if command == 'BUILD?':
        if any(not fields.get(key) for key in ('idf', 'target', 'project', 'proto', 'version', 'elfSha256')):
            raise retry.InvalidQueryReply('complete_BUILD_identity_required')
        if len(fields['version']) > 32 or not re.fullmatch(r'[0-9A-Fa-f]{64}', fields['elfSha256']):
            raise retry.InvalidQueryReply('invalid_BUILD_version_or_ELF_hash')
        if fields['proto'] != '2':
            raise retry.InvalidQueryReply('BUILD_protocol_2_required')
        # BUILD.version is an application string; PROTO.version is numeric.
        # Do not reuse the legacy cross-command numeric field-name set here.
        return fields
    if command not in EXTRA_REQUIRED and command not in OPTIONAL_REQUIRED:
        return BASE_VALIDATOR(command, fields)
    required = {**EXTRA_REQUIRED, **OPTIONAL_REQUIRED}[command].split()
    missing = [key for key in required if not fields.get(key)]
    if missing:
        raise retry.InvalidQueryReply('reply_missing_fields:' + command + ':' + ','.join(missing))
    for key in required:
        value = fields[key]
        if key in OPTIONAL_LITERALS.get(command, {}):
            if value != OPTIONAL_LITERALS[command][key]:
                raise retry.InvalidQueryReply('reply_invalid_observation_semantics:' + command + ':' + key)
        elif key == 'backend':
            if value != backend:
                raise retry.InvalidQueryReply('formal_USB_backend_required:' + backend)
        elif key in {'pcReceipt', 'callbackResult'}:
            if value != 'unknown':
                raise retry.InvalidQueryReply('unsupported_receipt_claim:' + command + ':' + key)
        elif key in HEX_FIELDS or (command == 'SCHEDWHY?' and key == 'init'):
            if not re.fullmatch(r'0x[0-9A-Fa-f]{1,8}', value):
                raise retry.InvalidQueryReply('reply_invalid_error:' + command + ':' + key)
        elif key in {'dataDtr', 'debugDtr'} and value == 'unknown':
            pass
        elif backend == 'otg_cdc_v1' and key in {'dCallbackBytes', 'cCallbackBytes'} and value == 'unknown':
            pass
        elif key in BOOL_FIELDS:
            if value not in {'0', '1'}:
                raise retry.InvalidQueryReply('reply_invalid_boolean:' + command + ':' + key)
        elif command in {'SCHED?', 'SCHEDWHY?'} and key in WORKERS:
            values = value.split('/')
            if len(values) != 3:
                raise retry.InvalidQueryReply('reply_invalid_worker_tuple:' + key)
            for part in values:
                unsigned(part, command, key)
        elif key in {'idleAgeTicks', 'maxIdleGapTicks'} and value == 'unknown':
            pass
        elif key in {'lastRxUs', 'lastTxUs'}:
            if not re.fullmatch(r'-?[0-9]+', value) or not -(1 << 63) <= int(value) < (1 << 63):
                raise retry.InvalidQueryReply('reply_invalid_int64:' + command + ':' + key)
        else:
            unsigned(value, command, key, 64 if key in U64_FIELDS else 32)
    return fields


def scheduler_health(fields):
    reasons = []
    if int(fields['initError'], 16) != 0: reasons.append('hook_init_error')
    if fields['registered'] != '1': reasons.append('hook_unregistered')
    if fields['idleKnown'] != '1' or fields['idleGen'] == '0': reasons.append('idle_progress_unknown')
    for key in ('idleAgeTicks', 'maxIdleGapTicks'):
        if fields[key] in {'unknown', '4294967295'}: reasons.append(key + '_unknown')
    if any(int(fields[key]) == 0 for key in ('tickHz', 'budgetTicks', 'maxUnits', 'quantumTicks')):
        reasons.append('scheduler_budget_unavailable')
    return {'usable': not reasons, 'reasons': reasons,
            'interpretation': 'Actual idle observations; counters are not CPU utilization or proof of watchdog safety.'}


class BulkBenchmarkCollector(retry.RetryBenchmarkCollector):
    expected_transport = 'usb_bulk'
    expected_backend = 'otg_bulk_v1'
    required_capabilities = CAPABILITIES
    query_requirements = {**EXTRA_REQUIRED, **OBSERVATION_REQUIRED, 'BUILD?': 'idf target project proto version elfSha256'}
    diagnostics = DIAGNOSTICS
    variant = VARIANT
    runner_source_file = __file__
    diagnostic_interval_seconds = 10.0
    periodic_budget_seconds = 2.0

    @classmethod
    def validate_fields(cls, command, fields):
        return validate_reply(command, fields, backend=cls.expected_backend)

    def __init__(self, *args, **kwargs):
        self.bulk_capabilities_verified = False
        self.reported_capabilities = set()
        self.observation_reply_records = {}
        self.cpu_owner_rx = {}
        self.cpu_owner_session = None
        self.diagnostic_windows = []
        self.snapshot_attempts = []
        self.scheduler_observations = {}
        self.periodic_active = False
        self.next_periodic_at = None
        super().__init__(*args, **kwargs)
        if self.transport_metadata['transport'] != self.expected_transport:
            raise CaptureError('formal_USB_wrapper_transport_mismatch:' + self.expected_transport)
        self.event('bulk_runner_variant' if self.is_bulk else 'cdc_runner_variant', variant=self.variant,
                   required_capabilities=sorted(self.required_capabilities),
                   periodic_seconds=self.diagnostic_interval_seconds, debug_CDC_scientific_evidence=False)

    def event(self, kind, **fields):
        if kind in {'rx', 'ctrl_rx'}:
            self.cpu_owner_rx[fields['channel']] = {
                'reply_received_monotonic': time.monotonic(),
                'reply_rx_chunk_offset': fields['offset'], 'reply_rx_chunk_length': fields['length'],
                'reply_receipt_scope': 'host_recorded_rx_chunk_ending_line'}
        if kind == 'reply' and fields.get('command') in OBSERVATION_REQUIRED:
            # Preserve the actual selected reply, never reconstruct it from
            # parsed fields. Exact terminators/bytes remain in the raw stream.
            line = fields['line']
            self.observation_reply_records[fields['command']] = {
                'reply_line': str(line), 'reply_event_monotonic': time.monotonic(),
                'reply_channel': getattr(line, 'channel', None)}
            if fields['command'] in CPU_OWNER_REQUIRED:
                self.observation_reply_records[fields['command']].update(getattr(line, 'cpu_owner_receipt', {}))
        super().event(kind, **fields)

    def observe(self, line):
        if line.startswith('CPUOWNER,'):
            line.cpu_owner_receipt = dict(self.cpu_owner_rx.get(getattr(line, 'channel', None), {}))
        super().observe(line)

    def query_cpu_owner(self, command, phase_deadline):
        # Frozen retry wrapper intentionally allows only trailing-? commands.
        # These seven indexed reads use the original single-query collector
        # with the same attempt/deadline policy, without broadening setters.
        limit = min(phase_deadline, self.total_deadline)
        baseline_faults, attempts, last_error = self.fault_events, 0, 'phase_deadline_expired'
        for attempt in range(1, retry.MAX_QUERY_ATTEMPTS + 1):
            if time.monotonic() >= limit: break
            attempts = attempt
            entry = {'command': command, 'phase': self.phase, 'attempt': attempt,
                     'started_monotonic': time.monotonic(), 'phase_deadline': limit}
            self.query_attempts.append(entry)
            self.event('readonly_query_attempt', command=command, attempt=attempt, phase_deadline=limit)
            try:
                fields = super(retry.RetryBenchmarkCollector, self).query(command, 'CPUOWNER', limit)
                validate_cpu_owner(command, fields)
                sessions = [self.cpu_owner_session]
                if self.expected_identity: sessions.append(self.expected_identity['session'])
                if any(session and fields['session'].upper() != session.upper() for session in sessions):
                    raise retry.InvalidQueryReply('reply_cpu_owner_session_mismatch:' + command)
                if self.fault_events > baseline_faults:
                    raise CaptureError('firmware_fault_during_readonly_query:' + command)
                entry['outcome'] = 'complete_reply'
                if attempt > 1: self.event('readonly_query_recovered', command=command, attempts=attempt)
                return fields
            except CaptureError as error:
                last_error = str(error)
                entry.update(outcome='failed', error=last_error)
                retryable = (isinstance(error, retry.InvalidQueryReply) or last_error == 'reply_deadline_expired'
                             or last_error.startswith('reply_timeout:') or 'reason=snapshot_busy' in last_error)
                self.event('readonly_query_failure', command=command, attempt=attempt, error=last_error, retryable=retryable)
                if not retryable or self.fault_events > baseline_faults: raise
                if attempt == retry.MAX_QUERY_ATTEMPTS or time.monotonic() >= limit: break
                gap_end = min(limit, time.monotonic() + retry.RETRY_GAP_SECONDS)
                while time.monotonic() < gap_end:
                    self.pump(gap_end); self.lines.clear()
                if self.fault_events > baseline_faults:
                    raise CaptureError('firmware_fault_during_readonly_query:' + command)
            finally:
                entry['finished_monotonic'] = time.monotonic()
        raise CaptureError(f'readonly_query_failed:{command}:attempts={attempts}:{last_error}')

    def query(self, command, tag, phase_deadline):
        # Base preparation queries USB? before PROTO; both are read-only.
        if command in OBSERVATION_REQUIRED and OBSERVATION_CAPABILITIES[command] not in self.reported_capabilities:
            raise CaptureError('optional_diagnostic_capability_not_verified:' + command)
        if command in {**EXTRA_REQUIRED, **OBSERVATION_REQUIRED} and command != 'USB?' and not self.bulk_capabilities_verified:
            raise CaptureError('bulk_diagnostic_capabilities_not_verified')
        self.observation_reply_records.pop(command, None)
        if command in CPU_OWNER_REQUIRED:
            if tag != 'CPUOWNER': raise CaptureError('cpu_owner_reply_tag_required')
            fields = self.query_cpu_owner(command, phase_deadline)
        else:
            fields = super().query(command, tag, phase_deadline)
        if command == 'PROTO?':
            caps = set(re.split(r'[/;,|\s]+', fields.get('caps', '')))
            missing = self.required_capabilities - caps
            if fields.get('version') != '2' or missing:
                raise CaptureError('bulk_diagnostic_capability_required:' + ','.join(sorted(missing)))
            self.bulk_capabilities_verified = True
            self.reported_capabilities = caps
        return fields

    def diagnostic_window(self, name, deadline, *, full=False):
        # Periodic queries stay in phase=capture, so their DATA remains science
        # evidence and their CTRL receipt timing remains in the same window.
        for attempt in range(2):
            opening = self.query('CONFIG?', 'CONFIG', deadline)
            self.cpu_owner_session = opening['session']
            window = {'CONFIG?': {'fields': opening, 'monotonic': time.monotonic(), 't': time.monotonic() - self.started}}
            entry = {'name': name, 'attempt': attempt + 1, 'started_monotonic': time.monotonic(),
                     'deadline': min(deadline, self.total_deadline), 'queries': window}
            self.snapshot_attempts.append(entry)
            queries = tuple((c, t) for c, t in base.QUERIES if c not in {'CONFIG?', 'USBSTREAM?'}) if full else ()
            def window_queries():
                yield from queries
                yield from self.diagnostics
                # Evaluate after this window's PROTO query. Optional counters
                # must not add periodic work or extend the capture deadline.
                if full and name in {'before', 'after'}:
                    for command, capability in OPTIONAL_CAPABILITIES.items():
                        if capability in self.reported_capabilities:
                            yield command, command[:-1]
                    if CPU_OWNER_CAPABILITY in self.reported_capabilities:
                        yield from CPU_OWNER_QUERIES
                yield 'CONFIG?', 'CONFIG'
            try:
                for command, tag in window_queries():
                    begin = time.monotonic()
                    fields = self.query(command, tag, deadline)
                    end = time.monotonic()
                    window['CONFIG?_closing' if command == 'CONFIG?' else command] = {
                        'fields': fields, 'started_monotonic': begin, 'monotonic': end, 't': end - self.started}
                    if command in OBSERVATION_REQUIRED:
                        window[command].update(self.observation_reply_records[command])
                    if command in CPU_OWNER_REQUIRED:
                        window[command]['cpu_owner_evidence'] = cpu_owner_evidence(fields)
                closing = window['CONFIG?_closing']['fields']
                if any(opening.get(key) != closing.get(key) for key in CONFIG_CONSISTENCY):
                    entry['outcome'] = 'configuration_changed'
                    self.event('snapshot_retry', name=name, attempt=attempt + 1)
                    continue
                if self.expected_identity and any(closing.get(k) != self.expected_identity.get(k) for k in base.IDENTITY):
                    raise CaptureError('configuration_changed_during_bulk_diagnostics')
                health = scheduler_health(window['SCHED?']['fields'])
                entry['scheduler_health'] = health
                entry['BLE_generation_coherent'] = window['BLELINK?']['fields']['gen'] == window['BLETX?']['fields']['gen']
                if not health['usable']:
                    raise CaptureError('scheduler_evidence_unavailable:' + ','.join(health['reasons']))
                self.scheduler_observations[name] = health
                self.diagnostic_windows.append(entry)
                entry['outcome'] = 'complete_coherent_configuration'
                self.event('bulk_diagnostic_window', name=name, revision=closing['rev'], queries=list(window))
                return window
            except CaptureError as error:
                entry['error'] = str(error)
                raise
            finally:
                self.cpu_owner_session = None
                entry['finished_monotonic'] = time.monotonic()
        raise CaptureError('configuration_changed_during_bulk_diagnostics')

    def snapshot_window(self, name, deadline):
        self.phase = name
        self.snapshots[name] = self.diagnostic_window(name, deadline, full=True)
        return self.snapshots[name]

    def pump(self, deadline):
        super().pump(deadline)
        if self.phase != 'capture' or self.periodic_active:
            return
        now = time.monotonic()
        if self.next_periodic_at is None:
            self.next_periodic_at = now + self.diagnostic_interval_seconds
        limit = min(deadline, self.capture_deadline, self.total_deadline)
        if now < self.next_periodic_at or limit - now < self.periodic_budget_seconds:
            return
        self.periodic_active = True
        try:
            self.diagnostic_window('periodic', min(limit, now + self.periodic_budget_seconds))
            self.check_fault()
        finally:
            self.periodic_active = False
            self.next_periodic_at = time.monotonic() + self.diagnostic_interval_seconds

    def run(self):
        result = super().run()
        result.update(runner_variant=self.variant, diagnostic_windows=self.diagnostic_windows,
                      diagnostic_snapshot_attempts=self.snapshot_attempts,
                      scheduler_observations=self.scheduler_observations,
                      periodic_interval_seconds=self.diagnostic_interval_seconds,
                      periodic_budget_seconds=self.periodic_budget_seconds,
                      source_sha256=hashlib.sha256(Path(self.runner_source_file).read_bytes()).hexdigest(),
                      shared_diagnostic_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                      optional_service_diagnostics={command: {
                          'capability': capability, 'advertised': capability in self.reported_capabilities}
                          for command, capability in OPTIONAL_CAPABILITIES.items()},
                      optional_service_window_scope='before_after_CONFIG_brackets_only_not_capture',
                      optional_cpu_owner_diagnostics={
                          'capability': CPU_OWNER_CAPABILITY, 'advertised': CPU_OWNER_CAPABILITY in self.reported_capabilities,
                          'queries': list(CPU_OWNER_REQUIRED), 'roles': dict(enumerate(WORKERS)),
                          'window_scope': 'before_after_CONFIG_brackets_only_not_capture',
                          'delta_rules': 'Pair only same session and taskNumber with available=1,known=1,conflict=0,stale=0 at both samples; require non-regressing runUs and ordered non-overlapping t0Us/t1Us brackets. Never compare across boot, map unknown to zero, or infer current CPU from stale samples.',
                          'counter_scope': 'switch_out accumulator; current running slice omitted, ISR may be included, SDK not isolated, blocked time unknown; collector does not derive CPU percentages'},
                      control_queries_are_not_measurement_accuracy=True,
                      USBBOOT_requested=False, reconnect_requested=False)
        return result


@contextmanager
def bulk_collector_selected(collector_class=BulkBenchmarkCollector):
    old = base.BenchmarkCollector, retry.REQUIRED, retry.validate_reply, base.validate_transport_options
    def require_formal_bulk(options):
        if options.transport != collector_class.expected_transport:
            raise ValueError('formal_USB_wrapper_transport_mismatch:' + collector_class.expected_transport)
        old[3](options)
    base.BenchmarkCollector = collector_class
    base.validate_transport_options = require_formal_bulk
    retry.REQUIRED = {**retry.REQUIRED, **collector_class.query_requirements}
    retry.validate_reply = collector_class.validate_fields
    try:
        yield
    finally:
        base.BenchmarkCollector, retry.REQUIRED, retry.validate_reply, base.validate_transport_options = old


def run_benchmark(options, **kwargs):
    if options.transport != 'usb_bulk':
        raise ValueError('formal_bulk_wrapper_requires_--transport_usb_bulk')
    with bulk_collector_selected():
        return base.run_benchmark(options, **kwargs)


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if '--transport' not in args:
        args = ['--transport', 'usb_bulk', *args]
    with bulk_collector_selected():
        return base.main(args)


if __name__ == '__main__':
    raise SystemExit(main())
