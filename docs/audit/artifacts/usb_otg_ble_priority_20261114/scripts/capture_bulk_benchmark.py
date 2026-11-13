#!/usr/bin/env python3
"""Formal dual-interface bulk benchmark; never uses CDC for scientific evidence.

Before/after and periodic read-only USB/USBQ/BLELINK/BLETX/SCHED observations
share existing deadlines. DATA keeps flowing while CTRL queries run. Defaults:
120 s configurations, 600 s stability, 5 s warmup, 10 s diagnostic interval.
No USBBOOT, restart, NVS, input flush or automatic USB reconfiguration.
"""
from contextlib import contextmanager
import hashlib
from pathlib import Path
import re
import sys
import time

ROOT = Path(__file__).resolve().parents[5]
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
DIAGNOSTICS = tuple((command, command[:-1]) for command in EXTRA_REQUIRED)
CONFIG_CONSISTENCY = base.IDENTITY + ('state', 'mode', 'rows', 'profile', 'boundary', 'err')
BASE_VALIDATOR = retry.validate_reply
HEX_FIELDS = {'err', 'initError', 'phyReq', 'connReq', 'dleReq', 'sdkLastErr'}
BOOL_FIELDS = {'init', 'mounted', 'suspended', 'dma', 'romAvailable', 'quiesced', 'connected', 'dataDtr', 'debugDtr',
               'phyKnown', 'connKnown', 'dleObserved', 'dlePeerKnown', 'sdkActive', 'registered', 'idleKnown'}
U64_FIELDS = {'rxBytes', 'dBytes', 'cBytes', 'dCallbackBytes', 'cCallbackBytes', 'sdkStartUs', 'sdkEndUs'}


def unsigned(value, command, field, bits=32):
    if not re.fullmatch(r'[0-9]+', value or '') or int(value) >= 1 << bits:
        raise retry.InvalidQueryReply(f'reply_invalid_unsigned:{command}:{field}')
    return int(value)


def validate_reply(command, fields, *, backend='otg_bulk_v1'):
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
    if command not in EXTRA_REQUIRED:
        return BASE_VALIDATOR(command, fields)
    required = EXTRA_REQUIRED[command].split()
    missing = [key for key in required if not fields.get(key)]
    if missing:
        raise retry.InvalidQueryReply('reply_missing_fields:' + command + ':' + ','.join(missing))
    for key in required:
        value = fields[key]
        if key == 'backend':
            if value != backend:
                raise retry.InvalidQueryReply('formal_USB_backend_required:' + backend)
        elif key in {'pcReceipt', 'callbackResult'}:
            if value != 'unknown':
                raise retry.InvalidQueryReply('unsupported_receipt_claim:' + command + ':' + key)
        elif key in HEX_FIELDS:
            if not re.fullmatch(r'0x[0-9A-Fa-f]{1,8}', value):
                raise retry.InvalidQueryReply('reply_invalid_error:' + command + ':' + key)
        elif key in {'dataDtr', 'debugDtr'} and value == 'unknown':
            pass
        elif backend == 'otg_cdc_v1' and key in {'dCallbackBytes', 'cCallbackBytes'} and value == 'unknown':
            pass
        elif key in BOOL_FIELDS:
            if value not in {'0', '1'}:
                raise retry.InvalidQueryReply('reply_invalid_boolean:' + command + ':' + key)
        elif command == 'SCHED?' and key in WORKERS:
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
    query_requirements = {**EXTRA_REQUIRED, 'BUILD?': 'idf target project proto version elfSha256'}
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

    def query(self, command, tag, phase_deadline):
        # Base preparation queries USB? before PROTO; both are read-only.
        if command in EXTRA_REQUIRED and command != 'USB?' and not self.bulk_capabilities_verified:
            raise CaptureError('bulk_diagnostic_capabilities_not_verified')
        fields = super().query(command, tag, phase_deadline)
        if command == 'PROTO?':
            caps = set(re.split(r'[/;,|\s]+', fields.get('caps', '')))
            missing = self.required_capabilities - caps
            if fields.get('version') != '2' or missing:
                raise CaptureError('bulk_diagnostic_capability_required:' + ','.join(sorted(missing)))
            self.bulk_capabilities_verified = True
        return fields

    def diagnostic_window(self, name, deadline, *, full=False):
        # Periodic queries stay in phase=capture, so their DATA remains science
        # evidence and their CTRL receipt timing remains in the same window.
        for attempt in range(2):
            opening = self.query('CONFIG?', 'CONFIG', deadline)
            window = {'CONFIG?': {'fields': opening, 'monotonic': time.monotonic(), 't': time.monotonic() - self.started}}
            entry = {'name': name, 'attempt': attempt + 1, 'started_monotonic': time.monotonic(),
                     'deadline': min(deadline, self.total_deadline), 'queries': window}
            self.snapshot_attempts.append(entry)
            queries = tuple((c, t) for c, t in base.QUERIES if c not in {'CONFIG?', 'USBSTREAM?'}) if full else ()
            try:
                for command, tag in queries + self.diagnostics + (('CONFIG?', 'CONFIG'),):
                    begin = time.monotonic()
                    fields = self.query(command, tag, deadline)
                    end = time.monotonic()
                    window['CONFIG?_closing' if command == 'CONFIG?' else command] = {
                        'fields': fields, 'started_monotonic': begin, 'monotonic': end, 't': end - self.started}
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
