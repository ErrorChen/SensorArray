#!/usr/bin/env python3
"""Export closed CAP/VOLT/RES/MIXED matrix evidence to JSON and a bilingual Markdown table."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from math import isfinite
from pathlib import Path
import re

import analyze_cpu_owner
from run_cdc_matrix_failfast import ExpectedCaptureIdentity, PLAN, ROOT
from summarize_diagnostic_screen import capture_summary, missing_observations


def require(condition, reason):
    if not condition:
        raise ValueError(reason)


def local_file(path, directory):
    path = path.resolve()
    require(path.parent == directory, 'evidence outside case directory')
    return path


def read_record(path):
    raw = path.read_bytes()
    return json.loads(raw), {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest()}


def argument(argv, flag):
    require(isinstance(argv, list) and argv.count(flag) == 1, 'missing or duplicate argument ' + flag)
    index = argv.index(flag) + 1
    require(index < len(argv) and isinstance(argv[index], str), 'missing value for ' + flag)
    return argv[index]


def sink_metrics(fps, counts, missing):
    received, lost = counts.get('ordered_integrity_valid_frames'), missing['count']
    denominator = received + lost if received is not None and lost is not None else None
    return {'fps': fps, 'received': received, 'missing': lost, 'missing_evidence': missing,
            'missing_fraction': lost / denominator if denominator else None,
            'invalid_points': counts.get('invalid_points'), 'unacquired_points': counts.get('unacquired_points')}


def closed_metrics(directory, formal_path, summary, gate, summary_digest):
    authority = gate['formal_authority']
    capture, = gate['analysis']['captures']
    require((ROOT / authority['summary']['path']).resolve() == formal_path and
            authority['summary']['sha256'] == capture['files']['summary']['sha256'] == summary_digest and
            capture['label'] == formal_path.name.removesuffix('.summary.json'), 'gate/formal summary binding mismatch')
    require(summary['image_authority']['expected'] == gate['expected'], 'summary/gate image identity mismatch')
    requested = summary['capture_requested_seconds']
    require(capture['requested']['requested_seconds'] == requested and
            capture['window']['requested_seconds_met'] is True, 'closed gate capture request mismatch')
    groups, ble = capture.get('configuration_groups', []), capture.get('ble', {})
    usb_counts, ble_counts = capture.get('capture_counts', {}), ble.get('capture_frame_counts', {})
    rates = groups[0]['rates'] if len(groups) == 1 else {}
    stats = {'capture_requested_seconds': requested, 'capture_seconds': summary['capture_seconds'],
             'device_span_seconds': rates.get('device_span_seconds'), 'capture_fps': rates.get('capture_fps'),
             'USB': sink_metrics(rates.get('host_complete_receive_fps'), usb_counts, missing_observations(usb_counts, groups)),
             'BLE': sink_metrics(ble.get('comparative_host_fps'), ble_counts,
                                 missing_observations(ble_counts, ble.get('configuration_groups', []))),
             'USB_status': capture['status'], 'BLE_status': ble.get('status', 'UNKNOWN'), 'BLE_reason': ble.get('reason'),
             'ADC_CRC': capture.get('otg_diagnostics', {}).get('adc_crc', {'status': 'NOT_OBSERVED'}),
             'memory': capture.get('otg_diagnostics', {}).get('memory', {'status': 'NOT_OBSERVED'}),
             'science': gate['science'], 'cleanup': summary['safe_cap_recovery']}
    # The journal has already been analyzed by the gate. CPU replay uses only
    # the closed formal raw/event pair, cross-checked against that same gate.
    try:
        for key, field in [('raw', 'raw_file'), ('events', 'events_file')]:
            source = local_file(analyze_cpu_owner.evidence_path(summary[field], formal_path), directory)
            require(source == (ROOT / authority[key]['path']).resolve(), key + ' path differs from closed gate')
        cpu = analyze_cpu_owner.analyze_file(formal_path)
        require(all(cpu['inputs'][k]['sha256'] == authority[k]['sha256'] for k in ('raw', 'events', 'summary')),
                'CPU inputs differ from closed gate hashes')
        stats['CPU'] = cpu
    except (OSError, ValueError, KeyError, TypeError) as error:
        stats['CPU'] = {'status': 'UNKNOWN', 'reason': str(error)}
    if len(groups) == 1 and 'configuration_identity' in stats['CPU']:
        try:
            screen = capture_summary(gate['analysis'], summary, cpu)
            for sink in ('USB', 'BLE'):
                stats[sink] = sink_metrics(screen[sink]['FPS'], screen[sink]['counts'], screen[sink]['missing_observations'])
            stats.update(capture_fps=screen['capture_fps'], memory=screen['memory'], ADC_CRC=screen['ADC_CRC'])
        except (ValueError, KeyError, TypeError) as error:
            stats['combined_screen_issue'] = str(error)
    return stats


def read_case(matrix, prefix, mode, rows):
    label = f'{prefix}-{mode.lower()}{rows}'
    directory = matrix / label
    row = {'case': label, 'matrix': str(matrix), 'mode': mode, 'rows': rows, 'status': 'NOT_STARTED', 'sources': {},
           'stage': 'NOT_STARTED', 'requested_identity': None,
           'command_status': None, 'gate_status': None, 'metrics': None}
    if not directory.exists():
        return row
    try:
        row['stage'] = 'UNVERIFIED'
        require(directory.resolve() == directory, 'case directory points outside matrix')
        command_path = local_file(directory / (label + '.command.json'), directory)
        if not command_path.exists():
            row.update(status='NO_CLOSED_COMMAND', stage='OPEN')
            return row
        command, row['sources']['command'] = read_record(command_path)
        require((command['label'], command['mode'], command['rows']) == (label, mode, rows), 'command/case mismatch')
        require(command['status'] in ('ERROR', 'PASS_CASE_SCREEN'), 'command has no final status')
        row.update(status=command['status'], command_status=command['status'], error=command.get('error'),
                   started_utc=command.get('started_utc'))
        argv = command['command']
        flags = ('--expected-elf', '--expected-session', '--expected-boot-id')
        row['requested_identity'] = ExpectedCaptureIdentity(*(argument(argv, flag) for flag in flags)).output()
        require(tuple(argument(argv, f) for f in ('--label', '--mode', '--rows')) == (label, mode, str(rows)),
                'collector arguments disagree with case label, mode or rows')
        # Commands made before the optional fixture margin could only request 120 seconds.
        requested = command.get('capture_requested_seconds', 120)
        allowed = (120, 122) if (mode, rows) in {('CAP', 8), ('RES', 8)} else (120,)
        require(requested in allowed and float(argument(argv, '--capture-seconds')) == requested,
                'unsupported or inconsistent command capture duration')
        row['capture_requested_seconds'] = requested
        gate_path = local_file(directory / (label + '.gate.json'), directory)
        formal_path = local_file(directory / (label + '.summary.json'), directory)
        if command.get('child_pid') or 'child_termination_known' in command or formal_path.exists():
            row['stage'] = 'FORMAL'
        if not gate_path.exists():
            preflight_errors = {'observer_closed_before_ready', 'observer_exited_before_ready',
                                'debug_problem_before_ready', 'observer_readiness_deadline'}
            if (row['stage'] == 'UNVERIFIED' and command['status'] == 'ERROR' and
                    str(command.get('error')).removeprefix('RuntimeError:') in preflight_errors):
                row['stage'] = 'PREFLIGHT'
            if command['status'] == 'PASS_CASE_SCREEN':
                row['status'] = 'UNKNOWN'
            row['evidence_issue'] = 'closed command has no gate'
            return row
        owners = command.get('observer_closure', [])
        require(command.get('child_termination_known') is True and command['exit_code'] == 0 and
                not command['stop_reason'] and len(owners) == 2 and {o['kind'] for o in owners} == {'ble', 'debug'} and
                all(o['termination_known'] and o['exit_code'] == 0 and not o['cleanup_errors'] for o in owners),
                'formal or observer termination unconfirmed')
        gate, row['sources']['gate'] = read_record(gate_path)
        row['gate_status'] = gate['status']
        row['gate_identity'] = gate.get('expected')
        require(row['gate_identity'] == row['requested_identity'], 'command/gate ELF, session or boot ID mismatch')
        require(command['gate_status'] == gate['status'] and command.get('science') == gate.get('science'),
                'closed command/gate verdict mismatch')
        row['status'] = gate['status'] if gate['status'] != 'PASS_CASE_SCREEN' else command['status']
        summary, row['sources']['formal'] = read_record(formal_path)
        digest = row['sources']['formal']['sha256']
        require(command['summary_sha256'] == digest, 'closed command/formal summary hash mismatch')
        require((summary['benchmark_kind'], summary['benchmark_mode'], summary['rows_requested'],
                 summary['capture_requested_seconds']) == ('configuration', mode, rows, requested),
                'formal summary disagrees with case or capture request')
        actual = summary['capture_seconds']
        host_span = summary['capture_monotonic_end'] - summary['capture_monotonic_start']
        require(summary['capture_duration_satisfied'] is True and isfinite(actual) and
                actual >= requested and host_span >= requested and abs(host_span - actual) < 1e-6,
                'formal host window does not verify the requested duration')
        row['expected'] = gate['expected']
        row['metrics'] = closed_metrics(directory, formal_path, summary, gate, digest)
    except (OSError, ValueError, TypeError, KeyError, IndexError) as error:
        row['evidence_issue'] = str(error)
        row['status'] = row['status'] if row['status'] in ('FAIL', 'ERROR') else 'UNKNOWN'
    return row


def combine_case(rows):
    attempts = [r for r in rows if r['stage'] != 'NOT_STARTED']
    formal = [r for r in attempts if r['stage'] == 'FORMAL']
    # Numerical results have no bearing on the displayed window. Keep all
    # attempts, including later formal windows with different outcomes.
    primary = next(iter(formal or attempts), rows[0])
    row = dict(primary)
    row.update(attempts=attempts, displayed_window=primary['case'] if formal else None)
    if not attempts:
        return row
    failed = [r for r in formal if r['gate_status'] == 'FAIL' and
              (r.get('metrics') or {}).get('science', {}).get('status') == 'FAIL']
    row['failed_scientific_windows'] = [r['case'] for r in failed]
    unverified = [r for r in attempts if r['stage'] == 'UNVERIFIED' or
                  r['stage'] == 'FORMAL' and (r['status'] != 'PASS_CASE_SCREEN' or r['metrics'] is None)]
    if failed:
        row['status'] = 'FAIL'
    elif unverified:
        row['status'] = 'ERROR' if any(r['status'] == 'ERROR' for r in unverified) else 'UNKNOWN'
    elif any(r['stage'] == 'OPEN' for r in attempts):
        row['status'] = 'NO_CLOSED_COMMAND'
    elif not formal:
        row['status'] = 'PREFLIGHT_ERROR'
    elif any(r['stage'] == 'PREFLIGHT' for r in attempts):
        row['status'] = ('PASS_AFTER_PREFLIGHT_ERROR' if attempts[-1]['stage'] == 'FORMAL'
                         else 'PASS_WITH_LATER_PREFLIGHT_ERROR')
    return row


def display(value, places=3):
    return f'{value:.{places}f}' if value is not None else '—'


def reception(sink):
    if not sink:
        return '—'
    fraction = sink['missing_fraction']
    rate = display(100 * fraction, 4) + '%' if fraction is not None else '—'
    values = ' / '.join(str(sink[k]) if sink[k] is not None else '—' for k in ('received', 'missing'))
    return f'{values} ({rate})'


def table(report):
    counts = report['case_status_counts']
    lines = [f"已核实通过 / Verified passing cases **{report['verified_pass_case_count']} / {len(PLAN)}**. "
             f"观察器预检失败后通过 / Passed after observer preflight failure **{counts.get('PASS_AFTER_PREFLIGHT_ERROR', 0)}**. "
             f"科学窗口失败的配置 / Cases with a failed scientific window **{report['scientific_failure_case_count']}**.", '',
             '| 模式 / Mode | 行 / Rows | 状态 / Status | 采集 / Capture FPS | USB FPS | BLE FPS | '
             'USB 实收 / 缺口 (率) · received / missing (%) | BLE 实收 / 缺口 (率) · received / missing (%) | '
             'ADC CRC 拒绝 / 检查 · rejected / checked | 无效点 / Invalid points (USB / BLE) |',
             '|---|---:|---|---:|---:|---:|---:|---:|---|---:|']
    for row in report['cases']:
        m = row['metrics'] or {}
        usb, ble, adc = m.get('USB', {}), m.get('BLE', {}), m.get('ADC_CRC', {})
        crc = adc.get('status', '—')
        if adc.get('rate') is not None:
            crc = f"{adc['rejected_delta']} / {adc['checked_attempts']} ({100 * adc['rate']:.4f}%)"
        invalid = ' / '.join(str(x['invalid_points']) if x.get('invalid_points') is not None else '—' for x in (usb, ble))
        status = row['status']
        if len(row['attempts']) > 1:
            history = ' → '.join(r['case'] + ' ' + ('PREFLIGHT_ERROR' if r['stage'] == 'PREFLIGHT' else r['status'])
                                 for r in row['attempts'])
            status += ' (' + history + ')'
        lines.append('| ' + ' | '.join([row['mode'], str(row['rows']), status, display(m.get('capture_fps')),
                                        display(usb.get('fps')), display(ble.get('fps')), reception(usb), reception(ble),
                                        crc, invalid]) + ' |')
    lines.extend(['', '缺口率基于同一接收端观察到的完整帧序列。ADC CRC 使用诊断窗口内的已检查读取次数。'
                  '破折号表示没有可核实的数据。', '',
                  'Gap rates use complete-frame sequence observations at each receiver. ADC CRC uses checked reads '
                  'within its diagnostic window. A dash indicates unavailable evidence.'])
    if len(report['matrices']) > 1:
        lines.extend(['', '多次尝试时，表内数值取所列目录顺序中的第一个正式窗口。箭头列出尝试顺序，'
                      'JSON 保留每次结果。每个配置只计一次。', '',
                      'For repeated attempts, the table shows the first formal window in input-directory order. '
                      'Arrows give the attempt order. The JSON retains each result. Each configuration counts once.'])
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True, type=Path, action='append',
                        help='Repeat in chronological order to combine same-ELF/session/boot attempts.')
    parser.add_argument('--label', action='append', help='Defaults to directory names. If supplied, provide one per directory.')
    parser.add_argument('--json', required=True, type=Path)
    parser.add_argument('--markdown', required=True, type=Path)
    args = parser.parse_args()
    matrices = [p.resolve() for p in args.directory]
    labels = args.label or [p.name for p in matrices]
    if len(matrices) != len(set(matrices)) or len(labels) != len(matrices):
        parser.error('use distinct directories and either zero labels or one label per directory')
    sources = list(zip(matrices, labels))
    if any(not m.is_dir() or not re.fullmatch(r'[A-Za-z0-9_-]+', label) for m, label in sources):
        parser.error('existing matrix directory and valid matrix label required')
    started = datetime.now(timezone.utc).isoformat()
    outputs = [args.json.resolve(), args.markdown.resolve()]
    case_dirs = {m / f'{label}-{mode.lower()}{rows}' for m, label in sources for mode, rows in PLAN}
    indices = {m / (label + '.failfast.json') for m, label in sources}
    if ([p.suffix.lower() for p in outputs] != ['.json', '.md'] or len(set(outputs)) != 2 or
            any(p.exists() or p.name.lower() == 'readme.md' or p in indices or
            any(d in p.parents for d in case_dirs) for p in outputs)):
        parser.error('choose two new report files outside the individual case directories')
    cases = [combine_case([read_case(m, label, mode, rows) for m, label in sources]) for mode, rows in PLAN]
    attempts = [a for case in cases for a in case['attempts']]
    identities = {json.dumps(a[k], sort_keys=True) for a in attempts for k in ('requested_identity', 'gate_identity')
                  if a.get(k)}
    if len(matrices) > 1:
        if len(identities) > 1:
            parser.error('cannot merge different ELF, session or boot identities: ' + ' | '.join(sorted(identities)))
        if any(a['sources'].get('command') and not a['requested_identity'] for a in attempts):
            parser.error('cannot bind every finalized command to an expected ELF, session and boot ID')
        for case in cases:
            starts = [datetime.fromisoformat(a['started_utc']) for a in case['attempts'] if a.get('started_utc')]
            if starts != sorted(starts):
                parser.error('supply directories oldest first for ' + case['mode'] + str(case['rows']))
    counts = Counter(r['status'] for r in cases)
    passed = counts['PASS_CASE_SCREEN'] + counts['PASS_AFTER_PREFLIGHT_ERROR']
    matrices_info = []
    for m, label in sources:
        own = [a for a in attempts if a['matrix'] == str(m)]
        proof = ('CLOSED_GATE' if any(a['metrics'] is not None for a in own) else
                 'COMMAND_EXPECTATION_ONLY' if any(a['requested_identity'] for a in own) else 'NO_BOUND_CLOSED_ATTEMPT')
        matrices_info.append({'directory': str(m), 'label': label, 'identity_evidence': proof})
    report = {'matrices': matrices_info, 'started_utc': started,
              'finished_utc': datetime.now(timezone.utc).isoformat(), 'case_status_counts': dict(counts),
              'firmware_identities': [json.loads(i) for i in sorted({json.dumps(a['expected'], sort_keys=True)
                                      for a in attempts if a['metrics'] is not None})],
              'verified_pass_case_count': passed,
              'scientific_failure_case_count': sum(bool(r.get('failed_scientific_windows')) for r in cases),
              'unique_cases_with_only_passing_formal_windows': passed + counts['PASS_WITH_LATER_PREFLIGHT_ERROR'],
              'unique_cases_without_preflight_or_formal_failure': counts['PASS_CASE_SCREEN'],
              'all_32_same_boot_case_screens_pass': passed == len(PLAN) and len(identities) == 1,
              'cases': cases}
    if len(matrices) == 1:
        report.update(matrix=str(matrices[0]), label=labels[0])
    bodies = [json.dumps(report, ensure_ascii=False, indent=2) + '\n', table(report)]
    created = []
    try:
        for path, body in zip(outputs, bodies):
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('x', encoding='utf-8') as stream:
                created.append(path)
                stream.write(body)
    except BaseException:
        for path in created:
            path.unlink(missing_ok=True)
        raise
    print(json.dumps({'json': str(outputs[0]), 'markdown': str(outputs[1]), 'verified_pass_case_count': passed,
                      'scientific_failure_case_count': report['scientific_failure_case_count'], 'case_status_counts': dict(counts)}))


if __name__ == '__main__':
    main()
