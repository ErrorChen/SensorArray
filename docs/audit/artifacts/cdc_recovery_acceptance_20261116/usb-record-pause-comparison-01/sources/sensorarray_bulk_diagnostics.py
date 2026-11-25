"""Offline, independent CTRL/CDC evidence. Never feed these bytes to science parsing.

This module accepts only finalized summaries. It validates every raw byte hash
and every receipt offset, retaining separate streams and monotonic clocks.
Diagnostics have no scientific ASCII CRC: structural acceptance is not proof
of bit integrity, delivery, physical isolation, or complete log coverage.
"""
from bisect import bisect_right
from array import array
from collections import Counter, defaultdict
import gzip
import hashlib
import json
import math
from pathlib import Path
import re

from text_protocol import parse_fields

SCHEMAS = {
    'CONFIG': 'session bootId rev boundary state mode rows profile err mgen mrid rgen rrid pgen prid mtermId mterm merr ptermId pterm perr rtermId rterm rerr',
    'USB': 'backend init err mounted suspended dma epoch resets mounts unmounts rxBytes rxDrop rxOver lastRxUs lastTxUs romAvailable quiesced pcReceipt dataDtr debugDtr',
    'USBQ': 'dDrop cDrop dSubmit cSubmit expired cExpired partial dBytes cBytes dCallbackBytes cCallbackBytes queueMaxUs stackMinBytes callbackResult dQueueCap dLifetimeUs',
    'BLELINK': 'gen connected mtu phyKnown txPhy rxPhy connKnown interval1250us latency timeout10ms phyReq connReq dleReq phyStatus connStatus dleStatus dleObserved dlePeerKnown dleTx dleRx stale',
    'BLETX': 'gen dDequeued lDequeued cDequeued dMaxAgeUs lMaxAgeUs cMaxAgeUs sdkActive sdkStartUs sdkEndUs sdkCalls sdkAccepted sdkErrors sdkMaxUs sdkLastErr txPrio ctrlPrio pcReceipt',
    'SCHED': 'initError registered idleKnown tickHz budgetTicks maxUnits graceUnits backstopTicks quantumTicks idleGen idleAgeTicks maxIdleGapTicks bt tr log ser bc sc wc',
    'ADSHEALTH': 'seq engineFrameCount modeEntryFrameCount healthInterval healthReadbackDue precisionFrame err crcValidEven crcValidOdd crcErrors crcRetries',
    'ADSSPI': 'seq mode crcFailures crcRetries crcRecoveredReads crcTotal retryTotal firstS firstD firstError terminalError frameError',
    'BL50': 'conn sub mtu phy mode mq ms md sentD sentL dropD dropL dropC ctrlRetry ctrlExhaust fs fe cg stale conf tiny',
}
COUNTERS = {
    'USB': ('resets', 'mounts', 'unmounts', 'rxBytes', 'rxDrop', 'rxOver'),
    'USBQ': ('dDrop', 'cDrop', 'dSubmit', 'cSubmit', 'expired', 'cExpired', 'partial', 'dBytes', 'cBytes', 'dCallbackBytes', 'cCallbackBytes'),
    'BLETX': ('dDequeued', 'lDequeued', 'cDequeued', 'sdkCalls', 'sdkAccepted', 'sdkErrors'),
    'ADSHEALTH': ('crcValidEven', 'crcValidOdd', 'crcErrors'),
    'ADSSPI': ('crcTotal', 'retryTotal'),
    'BL50': ('mq', 'ms', 'md', 'sentD', 'sentL', 'dropD', 'dropL', 'dropC', 'ctrlRetry', 'ctrlExhaust', 'fs', 'fe', 'cg', 'stale', 'conf'),
}
TERMINAL_UNKNOWN = {'PERF': 'heapMin', 'ADSHEALTH': 'crcRetries', 'ADSSPI': 'frameError',
                    'BL50': 'tiny', 'BLELINK': 'stale', 'SCHED': 'wc', 'CONFIG': 'rerr'}
TEXT_FIELDS = {'backend', 'pcReceipt', 'callbackResult', 'state', 'mode', 'profile',
               'mterm', 'pterm', 'rterm', 'session', 'sub', 'phy'}
WORKERS = {'bt', 'tr', 'log', 'ser', 'bc', 'sc', 'wc'}
LEGACY_LAYOUTS = {
    'USB': SCHEMAS['USB'].removesuffix(' dataDtr debugDtr'),
    'USBQ': SCHEMAS['USBQ'].removesuffix(' dQueueCap dLifetimeUs'),
}


def number(value):
    try: return int(value, 16 if str(value).lower().startswith('0x') else 10)
    except (TypeError, ValueError): return None


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def existing(path):
    path = Path(path)
    if path.exists(): return path
    compressed = Path(str(path) + '.gz')
    if compressed.exists(): return compressed
    raise FileNotFoundError(str(path))


def open_raw(path):
    return gzip.open(path, 'rb') if path.suffix == '.gz' else path.open('rb')


def record_error(tag, line, fields, memory_validator, expected_backend='otg_bulk_v1'):
    if tag in {'PERF', 'STK50'}:
        return memory_validator(tag, line)
    if tag not in SCHEMAS:
        return None
    tokens = line.split(',')[1:]
    keys = [part.partition('=')[0] for part in tokens]
    if keys not in (SCHEMAS[tag].split(), LEGACY_LAYOUTS.get(tag, '').split()) or any('=' not in part for part in tokens):
        return 'INCOMPLETE_OR_UNRECOGNIZED_FIELD_LAYOUT'
    for key in keys:
        value = fields.get(key, '')
        if not value: return 'EMPTY_' + key
        if key == 'session':
            if not re.fullmatch(r'[0-9A-Fa-f]{16}', value) or int(value, 16) == 0: return 'INVALID_SESSION'
        elif key == 'profile':
            if not re.fullmatch(r'[CVR]{8}', value): return 'INVALID_PROFILE'
        elif key in {'pcReceipt', 'callbackResult'}:
            if value != 'unknown': return 'UNSUPPORTED_RECEIPT_CLAIM'
        elif key == 'backend':
            if value != expected_backend: return 'WRONG_USB_BACKEND'
        elif tag == 'SCHED' and key in WORKERS:
            if not re.fullmatch(r'\d+/\d+/\d+', value): return 'INVALID_WORKER_TUPLE'
        elif key in TEXT_FIELDS:
            continue
        elif tag == 'SCHED' and key in {'idleAgeTicks', 'maxIdleGapTicks'} and value == 'unknown':
            continue
        elif key in {'dataDtr', 'debugDtr'} and value == 'unknown':
            continue
        elif expected_backend == 'otg_cdc_v1' and key in {'dCallbackBytes', 'cCallbackBytes'} and value == 'unknown':
            continue
        else:
            n = number(value)
            if n is None or (key not in {'lastRxUs', 'lastTxUs'} and n < 0) or n >= (1 << 64):
                return 'INVALID_NUMERIC_' + key
    if tag == 'CONFIG' and (not 1 <= number(fields['rows']) <= 8 or number(fields['rev']) == 0):
        return 'INVALID_CONFIG_RANGE'
    return None


class Bucket:
    def __init__(self):
        self.count = 0
        self.first = self.last = self.before = self.after = self.inside_first = self.inside_last = None

    def add(self, item):
        self.count += 1
        self.first = self.first or item
        self.last = item
        if item['phase'] == 'warmup': self.before = item
        elif item['phase'] == 'cleanup': self.after = self.after or item
        else:
            self.inside_first = self.inside_first or item
            self.inside_last = item

    def report(self, keys=(), continuity=False, only_capture=False):
        first = self.inside_first if only_capture else self.before or self.inside_first
        last = self.inside_last if only_capture else self.after or self.inside_last
        counters = {}
        for key in keys:
            a = number(first['fields'].get(key)) if first else None
            b = number(last['fields'].get(key)) if last else None
            reason = ('NOT_OBSERVED' if a is None or b is None or first is last else
                      'BOOT_CONTINUITY_UNVERIFIED' if not continuity else
                      'COUNTER_DECREASE_OR_WRAP_UNRESOLVED' if b < a else None)
            counters[key] = {'first': a, 'last': b, 'delta': b - a if reason is None else None, 'unavailable': reason}
        return {'status': 'OBSERVED' if first or last else 'NOT_OBSERVED', 'records_in_stream': self.count,
                'first': first, 'last': last, 'counters': counters,
                'elapsed_seconds': last['monotonic'] - first['monotonic'] if first and last and first is not last else None,
                'window': 'first_last_diagnostic_receipts_inside_capture' if only_capture else 'query_receipts_bracketing_capture_not_exact_boundaries'}


def read_stream(raw_path, events_path, expected_sha, *, event_type, origin, bounds, source, memory_validator, expected_backend='otg_bulk_v1'):
    """Fully verify one closed stream. Output keeps bounded examples/first/last."""
    raw_path, events_path = existing(raw_path), existing(events_path)
    offsets, receipts, next_offset, last_when = array('Q'), array('d'), 0, None
    opened, event_sha, event_bytes = None, hashlib.sha256(), 0
    with open_raw(events_path) as events:
        for wire in events:
            event_sha.update(wire); event_bytes += len(wire)
            item = json.loads(wire)
            if not isinstance(item, dict): raise ValueError(source + '_invalid_event_object')
            if item.get('type') == 'opened' and source == 'CDC': opened = item.get('monotonic')
            if item.get('type') != event_type: continue
            offset, length = item.get('offset'), item.get('length')
            when = item.get('monotonic') if origin is None else origin + item.get('t', math.nan)
            if type(offset) is not int or type(length) is not int or length <= 0 or offset != next_offset or offset + length >= 1 << 64:
                raise ValueError(source + '_noncontiguous_receipt_offsets')
            if not finite(when) or (last_when is not None and when < last_when):
                raise ValueError(source + '_invalid_receipt_time')
            offsets.append(offset); receipts.append(when)
            next_offset += length; last_when = when
    buckets = defaultdict(Bucket)
    counts, rejected = Counter(), Counter()
    faults, resets, rejected_examples = [], [], []
    fault_counts, reset_counts = Counter(), Counter()
    heap, stacks, boots = {}, {}, set()
    digest, total, discarding = hashlib.sha256(), 0, False
    with open_raw(raw_path) as stream:
        while True:
            wire = stream.readline(131073)
            if not wire: break
            offset = total; total += len(wire); digest.update(wire)
            idx = bisect_right(offsets, total - 1) - 1
            if idx < 0 or total > next_offset: raise ValueError(source + '_raw_bytes_outside_receipts')
            when = receipts[idx]
            phase = 'warmup' if when < bounds[0] else 'cleanup' if when > bounds[1] else 'capture'
            item = {'source': source, 'byte_offset': offset, 'monotonic': when, 'phase': phase}
            if discarding or len(wire) > 131072:
                rejected['OVERSIZE'] += int(not discarding); discarding = not wire.endswith(b'\n'); continue
            try: line = wire.rstrip(b'\r\n').decode('ascii')
            except UnicodeDecodeError:
                rejected['NON_ASCII'] += 1; continue
            tag, fields = line.split(',', 1)[0], parse_fields(line)
            # Positive fault text is retained even if its final newline was lost.
            is_fault = tag in {'MFAULT', 'MERR', 'RMERR', 'APP_FATAL', 'FDC_FATAL'} or any(
                s in line.lower() for s in ('watchdog got triggered', 'interrupt wdt timeout', 'guru meditation'))
            is_reset = line.startswith(('RST,', 'ESP-ROM:')) or 'rst:0x' in line.lower()
            for matches, examples, counter in ((is_fault, faults, fault_counts), (is_reset, resets, reset_counts)):
                if matches:
                    counter[phase] += 1
                    if len(examples) < 32: examples.append({**item, 'line': line, 'newline_present': wire.endswith(b'\n')})
            if not wire.endswith(b'\n'):
                rejected['UNTERMINATED_FINAL_LINE'] += 1; continue
            if tag in {'C', 'V', 'R', 'M', 'MR', 'K'} or re.fullmatch(r'[DP][0-9]+', tag):
                counts['ignored_scientific_records'] += 1; continue
            error = record_error(tag, line, fields, memory_validator, expected_backend)
            if error:
                rejected[tag + ':' + error] += 1
                if len(rejected_examples) < 16: rejected_examples.append({**item, 'reason': error, 'line': line})
                continue
            counts[tag if len(tag) < 48 else 'other'] += 1
            if tag in SCHEMAS or tag in {'PERF', 'STK50'}:
                buckets[tag].add({**item, 'fields': fields, 'unverified_terminal_field': TERMINAL_UNKNOWN.get(tag)})
            if tag == 'CONFIG': boots.add((fields['session'], fields['bootId']))
            if tag == 'STK50' and phase == 'capture':
                initial, current, minimum = map(int, fields['heap'].split('/'))
                heap['observed_free_bytes'] = min(heap.get('observed_free_bytes', current), current)
                heap['firmware_lifetime_minimum_bytes'] = min(heap.get('firmware_lifetime_minimum_bytes', minimum), minimum)
                for key in ('log', 'transport', 'usb', 'bleTx', 'bleCtrl', 'serialCtrl'):
                    allocated, remaining = map(int, fields[key].split('/'))
                    if allocated: stacks[key] = min(stacks.get(key, remaining), remaining)
    if total != next_offset: raise ValueError(source + '_receipt_byte_coverage_mismatch')
    if not expected_sha or digest.hexdigest() != expected_sha: raise ValueError(source + '_raw_SHA256_mismatch')
    return {'source': source, 'raw': {'path': str(raw_path), 'bytes': total, 'uncompressed_sha256': digest.hexdigest()},
            'events': {'path': str(events_path), 'rx_records': len(offsets),
                       'uncompressed_bytes': event_bytes, 'uncompressed_sha256': event_sha.hexdigest(),
                       'hash_scope': 'computed_at_analysis; offsets_and_times_structurally_validated; collector_has_no_expected_event_hash'}, 'status': 'VERIFIED_RECORDING',
            'first_receipt_monotonic': receipts[0] if receipts else None, 'last_receipt_monotonic': last_when,
            'opened_monotonic': opened, 'counts': dict(counts), 'rejected_records': dict(rejected),
            'counts_scope': 'whole independent raw stream; fault/reset phases and selected counter endpoints use the capture window',
            'rejected_examples': rejected_examples, 'faults': {'counts_by_phase': dict(fault_counts), 'examples': faults},
            'resets': {'counts_by_phase': dict(reset_counts), 'examples': resets},
            'heap': heap or None, 'stack_minimum_remaining_bytes': stacks or None, '_buckets': buckets, '_boots': boots}


def public_stream(stream):
    return {key: value for key, value in stream.items() if not key.startswith('_')}


def analyse_bulk_diagnostics(directory, label, summary, data_boots, window, debug_summaries, memory_validator, perf_counters):
    shared_cdc = summary.get('transport') == 'usb_cdc'
    backend = 'otg_cdc_v1' if shared_cdc else 'otg_bulk_v1'
    report = {'status': 'PARTIAL_OR_UNAVAILABLE', 'formal_science_source': 'DATA_only',
              'CTRL': {'status': 'NOT_OBSERVED'}, 'CDC': {'status': 'NOT_OBSERVED'},
              'boot_continuity': False, 'adc_crc': {'status': 'NOT_OBSERVED', 'rate': None},
              'WDT_evidence': 'NOT_OBSERVED', 'memory': None,
              'semantics': ['CTRL/CDC never enter scientific CRC, identity, frame-gap or FPS calculations.',
                            'Missing diagnostics are NOT_OBSERVED, not zero or proof of healthy operation.',
                            'Diagnostic layouts have no ASCII checksum; terminal numeric fields remain unverified.',
                            'USB accepted/callback bytes and BLE SDK acceptance do not acknowledge PC receipt.',
                            'SCHED observes idle progress and bounded work; it is not CPU utilization.']}
    report['transport'] = summary.get('transport')
    report['control_source'] = 'formal_CDC0_shared_DATA_CTRL_raw' if shared_cdc else 'independent_vendor_CTRL_raw'
    if shared_cdc:
        report['formal_science_source'] = 'CDC0_scientific_records_in_shared_raw_only'
        report['semantics'][0] = 'CDC0 control records are selected from the same raw stream; CDC1 debug never enters scientific CRC, identity, gap or FPS calculations.'
    start, end = summary.get('capture_monotonic_start'), summary.get('capture_monotonic_end')
    ws, we = window.get('capture_start_seconds'), window.get('capture_end_seconds')
    if not all(finite(v) for v in (start, end, ws, we)) or end < start or abs((end-start)-(we-ws)) > 0.01:
        report['reason'] = 'ABSOLUTE_CAPTURE_CLOCK_UNAVAILABLE_OR_INCONSISTENT'; return report
    bounds, origin = (start, end), start - ws
    report['clock_alignment'] = {'capture_start_monotonic': start, 'capture_end_monotonic': end,
        'CTRL_origin_monotonic': origin, 'start_end_origin_residual_seconds': (end - we) - origin,
        'basis': 'CTRL relative event time anchored to capture_start; CDC explicit host monotonic timestamps; no device timestamp substitution'}
    ctrl = None
    try:
        ctrl = read_stream(directory / (label + ('.rx.bin' if shared_cdc else '.ctrl.rx.bin')), directory / (label + '.events.jsonl'),
                           summary.get('raw_sha256' if shared_cdc else 'ctrl_raw_sha256'),
                           event_type='rx' if shared_cdc else 'ctrl_rx', origin=origin, bounds=bounds,
                           source='SHARED_CDC0_CONTROL_RECORDS' if shared_cdc else 'CTRL', memory_validator=memory_validator,
                           expected_backend=backend)
        report['CTRL'] = public_stream(ctrl)
    except (OSError, ValueError, TypeError, KeyError, EOFError) as error:
        report['CTRL'] = {'status': 'UNAVAILABLE_OR_INVALID', 'reason': str(error)}
    serial = summary.get('transport_metadata', {}).get('usb_serial')
    candidates, excluded = [], []
    for path in debug_summaries:
        path = Path(path)
        try:
            debug = json.loads(path.read_text())
            if debug.get('status') not in {'OBSERVED', 'STOPPED', 'ERROR', 'INTERRUPTED'} or not debug.get('raw_sha256'):
                raise ValueError('DEBUG_SUMMARY_NOT_FINALIZED')
            meta = debug.get('metadata', {})
            if (not serial or meta.get('usb_serial') != serial or meta.get('vid') != '303a' or
                    meta.get('pid') != ('4042' if shared_cdc else '4041') or meta.get('interface_number') != 2):
                raise ValueError('DEBUG_DEVICE_IDENTITY_MISMATCH')
            ds, de = debug.get('started_monotonic'), debug.get('ended_monotonic')
            if not finite(ds) or not finite(de) or de < ds or ds > end or de < start:
                raise ValueError('DEBUG_WINDOW_NOT_OVERLAPPING')
            stem = path.name[:-len('.debug.summary.json')]
            stream = read_stream(path.parent / (stem + '.debug.rx.bin'), path.parent / (stem + '.debug.events.jsonl'),
                                 debug['raw_sha256'], event_type='rx', origin=None, bounds=bounds, source='CDC',
                                 memory_validator=memory_validator, expected_backend=backend)
            if not finite(stream['opened_monotonic']):
                raise ValueError('DEBUG_OPEN_TIME_UNAVAILABLE')
            stream['capture_window_coverage'] = ('FULL_OBSERVER_WINDOW' if stream['opened_monotonic'] <= start and de >= end else 'PARTIAL_OBSERVER_WINDOW')
            stream['observer_status'] = debug['status']
            stream['summary'] = {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
            candidates.append(stream)
        except (OSError, ValueError, TypeError, KeyError, EOFError) as error:
            excluded.append({'summary': str(path), 'reason': str(error)})
    report['debug_sources_excluded'] = excluded
    cdc = candidates[0] if len(candidates) == 1 else None
    if cdc: report['CDC'] = public_stream(cdc)
    elif candidates:
        report['CDC'] = {'status': 'AMBIGUOUS_OVERLAPPING_DEBUG_RECORDINGS', 'count': len(candidates),
                         'sources': [public_stream(candidate) for candidate in candidates],
                         'counter_policy': 'never merge or double count independent overlapping recordings'}
    continuity = bool(ctrl and len(data_boots) == 1 and ctrl['_boots'] == set(map(tuple, data_boots)) and
                      not sum(ctrl['resets']['counts_by_phase'].values()))
    if cdc and cdc['resets']['counts_by_phase'].get('capture'): continuity = False
    report['boot_continuity'] = continuity
    if ctrl:
        report['control_queries'] = {tag: ctrl['_buckets'][tag].report(COUNTERS.get(tag, ()), continuity)
                                     for tag in ('CONFIG', 'USB', 'USBQ', 'BLELINK', 'BLETX', 'SCHED')}
        report['control_queries']['PERF'] = ctrl['_buckets']['PERF'].report(perf_counters, continuity)
        config = ctrl['_buckets']['CONFIG']
        if not config.before or not config.after:
            report['boot_continuity'] = continuity = False
            # Query counters cannot claim continuity from a single same-boot reply.
            for value in report['control_queries'].values():
                for counter in value['counters'].values():
                    counter.update(delta=None, unavailable='CONFIG_BRACKET_NOT_OBSERVED')
        link = report['control_queries']['BLELINK']
        report['BLE_link_evidence'] = {'status': link['status'], 'first': link['first'], 'last': link['last'],
            'known_flags_authoritative': ('phyKnown', 'connKnown', 'dleObserved', 'dlePeerKnown'),
            'unverified_terminal_field': 'stale', 'MTU_not_inferred_from_backend': True}
        first, last = link['first'], link['last']
        ble_connection_stable = bool(first and last and first['fields']['gen'] == last['fields']['gen'])
        if not ble_connection_stable:
            for counter in report['control_queries']['BLETX']['counters'].values():
                counter.update(delta=None, unavailable='BLE_CONNECTION_CONTINUITY_UNVERIFIED')
    if cdc:
        health = cdc['_buckets']['ADSHEALTH'].report(COUNTERS['ADSHEALTH'], continuity, True)
        events = cdc['_buckets']['ADSSPI'].report(COUNTERS['ADSSPI'], continuity, True)
        values = [health['counters'][key]['delta'] for key in COUNTERS['ADSHEALTH']]
        denominator = sum(values) if all(v is not None for v in values) else None
        report['adc_crc'] = {'status': 'OBSERVED' if denominator else 'NOT_OBSERVED',
            'health': health, 'crc_events': events, 'valid_even_delta': values[0], 'valid_odd_delta': values[1],
            'rejected_delta': values[2], 'checked_attempts': denominator,
            'rate': values[2] / denominator if denominator else None,
            'limit': 0.025, 'within_limit': (values[2] / denominator <= 0.025) if denominator else None,
            'denominator': 'crcValidEven + crcValidOdd + crcErrors deltas; checked reads, including retry attempts, not frames',
            'scope': 'first/last complete ADSHEALTH receipt inside capture, not exact window endpoints',
            'observer_window_coverage': cdc['capture_window_coverage'],
            'crcRetries_terminal_token': 'UNKNOWN_without_following_marker; ADSSPI retryTotal available independently'}
        report['BLE_firmware'] = cdc['_buckets']['BL50'].report(COUNTERS['BL50'], continuity and bool(ctrl and ble_connection_stable), True)
        report['WDT_evidence'] = cdc['capture_window_coverage'] + '; absent_WDT_text_does_not_prove_no_watchdog_event'
        report['memory'] = {'heap': cdc['heap'], 'stack_minimum_remaining_bytes': cdc['stack_minimum_remaining_bytes'],
            'scope': 'complete_capture_window_STK50_only', 'CPU_utilization': None, 'maximum_contiguous_heap_block': None}
    if ctrl and cdc:
        prefix = 'SHARED_CDC0_AND_DEBUG' if shared_cdc else 'INDEPENDENT_STREAMS'
        report['status'] = prefix + ('_OBSERVED' if cdc['capture_window_coverage'] == 'FULL_OBSERVER_WINDOW' else '_PARTIAL')
    return report
