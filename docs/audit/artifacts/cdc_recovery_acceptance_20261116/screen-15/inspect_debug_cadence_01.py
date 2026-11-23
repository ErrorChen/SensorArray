"""Read closed debug bytes and receipt offsets; do not equate summaries with science frames."""
import bisect
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

BASE = Path(__file__).resolve().parents[1]
OUT = Path(__file__).with_name('debug-cadence-observation-01.json')
assert not OUT.exists()
number = rb'[0-9]+\.[0-9]+'
pattern = re.compile(rb'SF50,seq=([0-9]+)-([0-9]+),n=([0-9]+),rows=([0-9]+),cfps='+number+
                     rb',efps='+number+rb',ofps='+number+rb'/'+number+rb'/'+number+
                     rb',bad=[0-9]+/[0-9]+/[0-9]+,drop=[0-9]+/[0-9]+/[0-9]+,q=[0-9]+/[0-9]+\r?\n')
results = []
for label in ('screen-14', 'screen-15'):
    summary_path = BASE / label / f'{label}-volt1.summary.json'
    raw_path = BASE / label / f'{label}-debug.debug.rx.bin'
    event_path = BASE / label / f'{label}-debug.debug.events.jsonl'
    summary = json.loads(summary_path.read_text())
    start, end = summary['capture_monotonic_start'], summary['capture_monotonic_end']
    events = [json.loads(line) for line in event_path.open()]
    reads = [e for e in events if e['type'] == 'rx']
    ends = [e['offset'] + e['length'] for e in reads]
    assert all(a < b for a,b in zip(ends, ends[1:]))
    raw = raw_path.read_bytes()
    assert ends[-1] == len(raw)
    offset = 0
    selected = []
    all_rows = []
    rejected = []
    for line in raw.splitlines(keepends=True):
        begin = offset
        offset += len(line)
        if not line.startswith(b'SF50,'):
            continue
        match = pattern.fullmatch(line)
        if not match:
            rejected.append({'offset': begin, 'bytes': len(line), 'why': 'incomplete_or_unexpected_layout'})
            continue
        read = reads[bisect.bisect_left(ends, offset)]
        receipt = read['monotonic']
        row = {'offset': begin, 'bytes': len(line), 'receipt_monotonic': receipt,
               'seq_first': int(match[1]), 'seq_last': int(match[2]),
               'n': int(match[3]), 'rows': int(match[4])}
        all_rows.append(row)
        if start <= receipt <= end:
            selected.append(row)
    assert all_rows and selected
    results.append({'label': label, 'capture_monotonic_start': start, 'capture_monotonic_end': end,
                    'complete_observer_SF50': len(all_rows),
                    'complete_observer_n_histogram': dict(sorted(Counter(r['n'] for r in all_rows).items())),
                    'capture_receipt_SF50': len(selected),
                    'capture_receipt_n_histogram': dict(sorted(Counter(r['n'] for r in selected).items())),
                    'capture_first_last': [selected[0], selected[-1]], 'rejected_SF50_layout': rejected,
                    'inputs': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                               for p in [summary_path, raw_path, event_path]]})
result = {'status': 'CLOSED_DEBUG_CADENCE_OBSERVED', 'hardware_actions': [],
          'original_recordings_modified': False, 'screens': results,
          'scope': 'Only complete structurally checked SF50 lines; capture membership uses the host timestamp of the read completing the line. This is diagnostic cadence, not scientific frame rate or air timing.',
          'generator': {'path': str(Path(__file__)), 'sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}}
with OUT.open('x') as stream:
    json.dump(result, stream, indent=2); stream.write('\n')
print(json.dumps({'status': result['status'], 'screens': [
    {k: s[k] for k in ['label', 'complete_observer_SF50', 'complete_observer_n_histogram',
                       'capture_receipt_SF50', 'capture_receipt_n_histogram', 'rejected_SF50_layout']}
    for s in results]}))
