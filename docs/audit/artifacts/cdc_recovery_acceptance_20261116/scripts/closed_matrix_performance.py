"""Layered performance evidence for summarize_closed_matrix.py --performance."""
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
from math import isfinite
from pathlib import Path
import re
from statistics import median

from analyze_cpu_owner import ROLES
from analyze_usb_unread import ROOT, verify_query_trace
from analyze_firmware_benchmark import PERF_COUNTERS, memory_telemetry_record_error
from sensorarray_bulk_diagnostics import analyse_bulk_diagnostics
from summarize_diagnostic_screen import layer_window
from summarize_reference_capture import RxTimeline
from text_protocol import parse_fields


# Full emitted SF50 layout. Its terminal queue field has no checksum and is unused.
DECIMAL = rb'[0-9]+\.[0-9]+'
SF50 = re.compile(rb'SF50,seq=(\d+)-(\d+),n=(\d+),rows=(\d+),cfps=(' + DECIMAL +
                  rb'),efps=(' + DECIMAL + rb'),ofps=' + DECIMAL + rb'/' + DECIMAL + rb'/' + DECIMAL +
                  rb',bad=\d+/\d+/\d+,drop=\d+/\d+/\d+,q=\d+/\d+\r?\n')
READ_ERRORS = (OSError, KeyError, ValueError, TypeError, IndexError)


def require(ok, reason):
    if not ok:
        raise ValueError(reason)


def bound_bytes(reference, directory):
    path = (ROOT / reference['path']).resolve()
    require(path.parent == directory, 'source outside closed case')
    if path.is_file():
        data = path.read_bytes()
        require(hashlib.sha256(data).hexdigest() == reference['sha256'], 'source digest mismatch: ' + path.name)
        return gzip.decompress(data) if path.suffix == '.gz' else data
    with gzip.open(str(path) + '.gz', 'rb') as stream:
        data = stream.read()
    require(hashlib.sha256(data).hexdigest() == reference['sha256'], 'archived source digest mismatch: ' + path.name)
    return data


def unknown(reason):
    return {'status': 'UNKNOWN', 'reason': str(reason)}


def debug_observations(raw, events, capture, summary):
    group, = capture['configuration_groups']
    first, last = (group['sequence'][k] for k in ('first', 'last'))
    mask = 2**32 - 1
    progress = (last - first) & mask
    require(progress < 2**31, 'ambiguous scientific sequence range')
    start, end = summary['capture_monotonic_start'], summary['capture_monotonic_end']
    # The existing offset lookup uses t. CDC1 already stamped every read on the
    # common host monotonic clock used by the formal capture boundaries.
    timeline = RxTimeline([{**e, 't': e['monotonic']} for e in events if e['type'] == 'rx'])
    samples, excluded, stack_samples = [], Counter(), []
    offset, previous_end = 0, -1
    for line in raw.splitlines(keepends=True):
        begin, offset = offset, offset + len(line)
        if not line.startswith((b'SF50,', b'STK50,')):
            continue
        head = timeline.at(begin)['host_rx_seconds']
        tail = timeline.at(offset - 1)['host_rx_seconds']
        if line.startswith(b'STK50,'):
            if tail is not None and start <= tail <= end and line.endswith(b'\n'):
                text = line.rstrip(b'\r\n').decode('ascii', errors='replace')
                if memory_telemetry_record_error('STK50', text) is None:
                    stack_samples.append({'receipt_monotonic': tail, 'sequence': int(parse_fields(text)['seq'])})
            continue
        if head is None or tail is None or not start <= head <= tail <= end:
            excluded['outside_host_window'] += 1
            continue
        m = SF50.fullmatch(line)
        if not m:
            excluded['malformed_in_window'] += 1
            continue
        seq0, seq1, n, rows = map(int, m.groups()[:4])
        lo, hi = (seq0 - first) & mask, (seq1 - first) & mask
        if (not 0 <= seq0 <= mask or not 0 <= seq1 <= mask or rows != group['rows'] or
                not 0 <= lo <= hi <= progress or not 1 <= n <= hi - lo + 1):
            excluded['outside_scientific_group_or_invalid_range'] += 1
            continue
        if lo <= previous_end:
            excluded['overlapping_or_out_of_order'] += 1
            continue
        previous_end = hi
        samples.append({'first_sequence': seq0, 'last_sequence': seq1, 'n': n, 'receipt_monotonic': tail,
                        'capture_fps': float(m[5]), 'emit_fps': float(m[6])})
    result = {'status': 'OBSERVED' if samples else 'UNKNOWN', 'SF50_count': len(samples),
              'excluded': dict(excluded), 'reported_frame_count': sum(x['n'] for x in samples),
              'first': samples[0] if samples else None, 'last': samples[-1] if samples else None,
              'exact_whole_capture_emit_fps': None,
              'scope': 'diagnostic SF50 intervals wholly inside the host window and trusted scientific sequence range'}
    if samples:
        for field in ('capture_fps', 'emit_fps'):
            values = [x[field] for x in samples]
            result[field] = {'min': min(values), 'median': median(values), 'max': max(values)}
    if excluded.get('overlapping_or_out_of_order'):
        result = {**unknown('overlapping or out-of-order SF50 sequence windows'), 'excluded': dict(excluded)}
    stack_window = {'status': 'OBSERVED' if stack_samples else 'UNKNOWN', 'count': len(stack_samples),
                    'first': stack_samples[0] if stack_samples else None, 'last': stack_samples[-1] if stack_samples else None,
                    'scope': 'complete structurally verified STK50 records, last-byte host receipt inside formal window'}
    return result, stack_window


def collector_clock(events, summary):
    # For these query snapshots t and monotonic come from one saved 'end' value.
    # The capture events separately store the measurement boundary and the time
    # their event line was written. A busy host can write that line later.
    origins = [summary['snapshots'][p][k]['monotonic'] - summary['snapshots'][p][k]['t']
               for p in ('before', 'after') for k in ('PERF?', 'USBQ?', 'BLETX?')]
    require(all(isfinite(v) for v in origins) and max(origins) - min(origins) < 1e-6,
            'could not bind one collector clock origin')
    origin = median(origins)
    result = {'status': 'OBSERVED', 'collector_origin_monotonic': origin, 'paired_query_samples': len(origins)}
    for edge in ('start', 'end'):
        e, = [e for e in events if e['type'] == 'capture_' + edge]
        require(e['monotonic'] == summary['capture_monotonic_' + edge], 'formal boundary timestamp mismatch')
        delay = e['t'] + origin - e['monotonic']
        require(isfinite(delay) and delay >= -1e-6, 'event writer appeared before capture boundary')
        result[edge + '_event_write_delay_seconds'] = delay
    return result


def query_timing_and_heap(events, raw, summary, expected, clock):
    replies = verify_query_trace(events, raw)
    # MEM? snapshots are sequential. Keep their capability and sample time.
    heap_samples = []
    for reply in replies:
        fields = reply['fields']
        if (reply['command'] == 'MEM?' and reply['phase'] in ('before', 'after') and
                fields.get('v') == '1' and fields.get('unit') == 'bytes' and fields.get('intCaps') == 'internal8' and
                fields.get('readout') == 'sequential' and all(fields.get(k, '').isdigit() for k in
                    ('intFree', 'intLargest', 'intMin', 'extFree', 'extLargest', 'sampleUs'))):
            heap_samples.append({'phase': reply['phase'], 'collector_reply_seconds': reply['reply_t'],
                                 **{k: int(fields[k]) for k in ('intFree', 'intLargest', 'intMin',
                                                                'extFree', 'extLargest', 'sampleUs')}})
    heap = {'status': 'OBSERVED' if heap_samples else 'UNKNOWN', 'samples': heap_samples,
            'reason': None if heap_samples else 'no validated before/after MEM? reply',
            'scope': 'same-case before/after MEM? replies; internal8 and external heaps sampled sequentially',
            'DMA_largest_bytes': None}
    try:
        require(summary['safe_cap_recovery'] == 'CAP8_QUERY_VERIFIED', 'CAP8 recovery not verified')
        capture_end, = [e['t'] for e in events if e['type'] == 'capture_end']
        cleanup = [e for e in events if e.get('phase') == 'cleanup' and e['t'] >= capture_end]
        send, = [e['t'] for e in cleanup if e['type'] == 'tx_begin' and e.get('command') == 'MODE=CAP']
        route, = [e['t'] for e in cleanup if e['type'] == 'cap_reference_query_verified']
        verified = [e for e in cleanup if e['type'] == 'transition_verified']
        require([e['command'] for e in verified] == ['MODE=CAP', 'ROWS=8'], 'ambiguous recovery setters')
        config, = [r for r in replies if r['phase'] == 'cleanup' and r['command'] == 'CONFIG?' and r['tx_t'] >= route]
        f = config['fields']
        require((f['session'], f['bootId'], f['mode'], f['rows'], f['profile'], f['state'], f['err']) ==
                (expected['session'], expected['bootId'], 'CAP', '8', 'CCCCCCCC', 'CAPACITANCE', '0x0') and
                (f['mtermId'], f['rtermId']) == tuple(str(e['request_id']) for e in verified) and
                capture_end <= send <= verified[0]['t'] <= verified[1]['t'] <= route <= config['reply_t'],
                'recovery identity, sequence or chronology mismatch')
        origin = clock.get('collector_origin_monotonic')
        recovery = {'status': 'CAP8_QUERY_VERIFIED', 'clock': 'collector-relative host seconds',
                    'capture_end_event_written': capture_end, 'MODE_CAP_tx_begin': send, 'ROWS_8_verified': verified[1]['t'],
                    'CAP_reference_verified': route, 'final_CAP8_CONFIG_reply': config['reply_t'],
                    'from_first_restore_setter_seconds': config['reply_t'] - send,
                    'from_capture_end_including_after_queries_seconds': (
                        config['reply_t'] + origin - summary['capture_monotonic_end'] if origin is not None else None),
                    'final_revision': f['rev']}
    except READ_ERRORS as error:
        recovery = unknown(error)
    return recovery, heap


def counter_rates(window, names):
    value = layer_window(window)
    delta = {k: value['counters'].get(k, {}).get('delta') for k in names}
    seconds = value['elapsed_seconds']
    return {'status': value['status'], 'scope': value['scope'], 'first_receipt_monotonic': value['first_receipt'],
            'last_receipt_monotonic': value['last_receipt'], 'seconds': seconds, 'delta': delta,
            'per_second': {k: n / seconds if n is not None and seconds and seconds > 0 else None
                           for k, n in delta.items()},
            'unavailable': {k: value['counters'].get(k, {}).get('unavailable', 'NOT_OBSERVED')
                            for k, n in delta.items() if n is None}}


def performance(attempt):
    m, refs = attempt['metrics'], attempt['sources']
    directory = Path(refs['gate']['path']).parent
    gate = json.loads(bound_bytes(refs['gate'], directory))
    summary = json.loads(bound_bytes(refs['formal'], directory))
    c, = gate['analysis']['captures']
    groups, ble = c['configuration_groups'], c['ble']
    diagnostic = c.get('otg_diagnostics') or {}
    provenance = {'source': 'closed_gate', 'gate_status': diagnostic.get('status'), 'gate_reason': diagnostic.get('reason')}
    authority = gate['formal_authority']
    try:
        raw = bound_bytes(authority['raw'], directory)
        events = [json.loads(x) for x in bound_bytes(authority['events'], directory).splitlines()]
        try:
            clock = collector_clock(events, summary)
        except READ_ERRORS as error:
            clock = unknown(error)
        recovery, largest = query_timing_and_heap(events, raw, summary, gate['expected'], clock)
        if diagnostic.get('reason') == 'ABSOLUTE_CAPTURE_CLOCK_UNAVAILABLE_OR_INCONSISTENT' and clock.get('status') == 'OBSERVED':
            # Reuse the standard analyzer and its unchanged schemas and continuity
            # checks. Recover the original timebase independently of event-write delay.
            try:
                for reference in gate['debug'].values():
                    bound_bytes(reference, directory)
                origin = clock['collector_origin_monotonic']
                window = {'capture_' + k + '_seconds': summary['capture_monotonic_' + k] - origin
                          for k in ('start', 'end')}
                diagnostic = analyse_bulk_diagnostics(directory, c['label'], summary, c['observed_boot_sessions'], window,
                    [ROOT / gate['debug']['summary']['path']], memory_telemetry_record_error, PERF_COUNTERS)
                provenance.update(source='bound_formal_clock_replay', replay_status=diagnostic['status'],
                                  replay_reason=diagnostic.get('reason'), clock=clock)
            except READ_ERRORS as error:
                provenance['replay_issue'] = str(error)
    except READ_ERRORS as error:
        recovery, largest, clock = unknown(error), unknown(error), unknown(error)
    queries = diagnostic.get('control_queries') or {}
    cpu = m['CPU']
    data = {'status': 'OBSERVED', 'identity': gate['expected'], 'capture_requested_seconds': m['capture_requested_seconds'],
            'capture_seconds': m['capture_seconds'], 'capture_fps': m['capture_fps'], 'device_span_seconds': m['device_span_seconds'],
            'host_start_monotonic': summary['capture_monotonic_start'], 'host_end_monotonic': summary['capture_monotonic_end'],
            'host': {}, 'PERF': counter_rates(queries.get('PERF', {}), ('pub', 'emitQueued')),
            'USBQ': counter_rates(queries.get('USBQ', {}), ('dSubmit', 'cSubmit', 'dBytes', 'cBytes',
                                                           'dCallbackBytes', 'cCallbackBytes', 'dDrop', 'expired', 'partial')),
            'BLETX': counter_rates(queries.get('BLETX', {}), ('dDequeued', 'lDequeued', 'cDequeued', 'sdkCalls', 'sdkAccepted', 'sdkErrors')),
            'BL50': counter_rates(diagnostic.get('BLE_firmware', {}), ('sentD', 'sentL', 'fs', 'fe', 'dropD')),
            'physical_bus_completion': {'USB': 'UNKNOWN', 'BLE': 'UNKNOWN',
                                        'scope': 'no per-frame USB bus completion or BLE air trace in formal recordings'},
            'BLESVC': cpu.get('BLE_SDK_separate_query_window', unknown('no bound BLESVC window')),
            'CPU': {'whole_device_percent': None, 'SDK_CPU_time': None, 'blocked_time': None, 'roles': []},
            'memory': diagnostic.get('memory') or m['memory'], 'recovery': recovery, 'heap_largest': largest,
            'clock': clock, 'diagnostic_provenance': provenance,
            'sources': {'formal': {k: authority[k] for k in ('summary', 'raw', 'events')}, 'debug': gate['debug']}}
    for name, group in [('USB', groups), ('BLE', ble.get('configuration_groups', []))]:
        group = group[0] if len(group) == 1 else {}
        times = group.get('host_complete_frame_intervals', {})
        data['host'][name] = {k: m[name][k] for k in ('fps', 'received', 'missing', 'missing_fraction', 'invalid_points')}
        data['host'][name].update(sequence=group.get('sequence'), first_complete_host_seconds=times.get('first_complete_host_seconds'),
                                  last_complete_host_seconds=times.get('last_complete_host_seconds'),
                                  time_coordinate='collector-relative seconds' if name == 'USB' else 'host monotonic seconds')
    for role in cpu.get('roles', []):
        item = {k: role.get(k) for k in ('name', 'status', 'reason', 'runtime_delta_us', 'observation_span_us',
                                          'runtime_per_own_observation_span_percent')}
        for phase in ('before', 'after'):
            sample = role.get(phase) or {}
            item[phase] = {'device_observed_us': sample.get('values', {}).get('observedAtUs'),
                           'host_received_monotonic': sample.get('host_reply_received_monotonic')}
        data['CPU']['roles'].append(item)
    try:
        debug = gate['debug']
        raw = bound_bytes(debug['raw'], directory)
        events = [json.loads(x) for x in bound_bytes(debug['events'], directory).splitlines()]
        data['SF50'], data['STK50_window'] = debug_observations(raw, events, c, summary)
    except READ_ERRORS as error:
        data['SF50'] = unknown(error)
        data['STK50_window'] = unknown(error)
    return data


def performance_report(report):
    result = {key: report[key] for key in ('matrices', 'started_utc', 'firmware_identities', 'case_status_counts',
              'verified_pass_case_count', 'scientific_failure_case_count', 'all_32_same_boot_case_screens_pass')}
    result.update(cases=[], hardware_actions=[], generated_by={'path': str(Path(__file__).resolve()),
                  'sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()})
    for case in report['cases']:
        row = {key: case[key] for key in ('mode', 'rows', 'status')}
        row['attempts'] = []
        for attempt in case['attempts']:
            detail = {k: attempt.get(k) for k in ('case', 'stage', 'status', 'gate_status', 'error', 'evidence_issue',
                                                   'started_utc', 'sources')}
            if attempt['metrics'] is not None:
                try:
                    detail['performance'] = performance(attempt)
                except READ_ERRORS as error:
                    detail['performance'] = unknown(error)
            row['attempts'].append(detail)
        result['cases'].append(row)
    result['finished_utc'] = datetime.now(timezone.utc).isoformat()
    return result


def fmt(value, places=3):
    return 'UNKNOWN' if value is None else f'{value:,.{places}f}'


def performance_table(report):
    observed = [(a['case'], a['status'], a['performance']) for case in report['cases'] for a in case['attempts']
                if a.get('performance', {}).get('status') == 'OBSERVED']
    lines = ['# 已闭合矩阵性能 / Closed matrix performance', '',
             f"已闭合且可核对的性能窗口 / Verified performance windows **{len(observed)}**. " +
             ', '.join(f"{k} {v}" for k, v in report['case_status_counts'].items()) + '.']
    for identity in report['firmware_identities']:
        lines += ['', f"ELF `{identity['elfSha256']}`, session `{identity['session']}`, boot `{identity['bootId']}`."]
    lines += ['', '| 工况 / Attempt | 判定 / Gate | 主机窗 / Host s | Capture FPS | Emit FPS · SF50 median [min–max] | '
              'USB DATA 提交 / submits (records/s) | BLE DATA 本地发送 / sentD (messages/s) | BLE SDK 接受 / accepted (calls/s) | '
              'USB 实收 / host frames @ FPS | BLE 实收 / host frames @ FPS |',
              '|---|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for label, status, data in observed:
        efps = data['SF50'].get('emit_fps', {})
        emit = (f"{fmt(efps['median'],2)} [{fmt(efps['min'],2)}–{fmt(efps['max'],2)}]" if efps else 'UNKNOWN')
        row = [label, status, fmt(data['capture_seconds'], 6), fmt(data['capture_fps']), emit]
        for tag, counter in [('USBQ', 'dSubmit'), ('BL50', 'sentD'), ('BLETX', 'sdkAccepted')]:
            win = data[tag]
            row.append(f"{fmt(win['delta'][counter],0)} ({fmt(win['per_second'][counter])})")
        for sink in ('USB', 'BLE'):
            host = data['host'][sink]
            row.append(f"{fmt(host['received'],0)} @ {fmt(host['fps'])}")
        lines.append('| ' + ' | '.join(row) + ' |')
    stacks = ('log', 'transport', 'bleTx', 'bleCtrl', 'serialCtrl')
    lines += ['', 'SF50 各区间报告固件 emit 速率。USBQ 按 DATA 记录、BL50 按 DATA 消息、BLETX 按 SDK 调用计数，各有独立时间跨度。'
              'USB 物理总线完成与 BLE 空口完成均为 UNKNOWN。 / SF50 measures firmware emit intervals. '
              'USBQ counts DATA records, BL50 counts local DATA messages, and BLETX counts SDK calls over separate intervals. '
              'Physical USB and BLE completions are UNKNOWN.', '',
              '| 工况 / Attempt | CPU % · ' + ' / '.join(ROLES) + ' | 总空闲最低观测 / 生命周期最低 · '
              'observed free / lifetime min (B) | 最大连续 internal / external (B) | '
              '栈余量 / Stack (B) · ' + ' / '.join(stacks) + ' | 恢复 / CAP8 ms · setter / capture end |',
              '|---|---|---:|---:|---|---:|']
    for label, _, data in observed:
        cpu = {r['name']: r['runtime_per_own_observation_span_percent'] for r in data['CPU']['roles']}
        memory = data['memory'] or {}
        heap, stack = memory.get('heap') or {}, memory.get('stack_minimum_remaining_bytes') or {}
        sizes = ' / '.join(fmt(heap.get(k), 0) for k in ('observed_free_bytes', 'firmware_lifetime_minimum_bytes'))
        samples = data['heap_largest'].get('samples', [])
        largest = ' / '.join(fmt(min((s[k] for s in samples), default=None), 0) for k in ('intLargest', 'extLargest'))
        recovery = data['recovery']
        times = [recovery.get(k) for k in ('from_first_restore_setter_seconds', 'from_capture_end_including_after_queries_seconds')]
        row = [label, ' / '.join(fmt(cpu.get(k), 2) for k in ROLES), sizes, largest,
               ' / '.join(fmt(stack.get(k),0) for k in stacks), ' / '.join(fmt(v*1000) if v is not None else 'UNKNOWN' for v in times)]
        lines.append('| ' + ' | '.join(row) + ' |')
    lines += ['', 'CPU 为各任务独立采样跨度的 runtime 比例。总 heap 含可用 PSRAM，连续块按实际 MEM? 采样列出最小观测值。'
              '恢复两列均截止最终同 boot CAP8 CONFIG 实际回读。 / CPU ratios use each owner’s own interval. '
              'Total heap includes available PSRAM. Largest blocks use the minimum of observed MEM? samples by capability. '
              'Both recovery intervals end at the final same-boot CAP8 CONFIG reply. JSON retains the time bounds and every attempt.']
    unavailable = [a['case']+' ('+a['status']+')' for r in report['cases'] for a in r['attempts']
                   if a.get('performance', {}).get('status') not in ('OBSERVED',) and a['stage'] != 'OPEN']
    if unavailable:
        lines += ['', '无可核对性能窗口 / Attempts without verified performance windows — ' + ', '.join(unavailable) + '.']
    return '\n'.join(lines) + '\n'
