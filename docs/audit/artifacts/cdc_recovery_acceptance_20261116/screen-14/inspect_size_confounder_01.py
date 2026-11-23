"""Compare observed message sizes across closed windows, not a controlled causal A/B."""
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parents[1]
OUT = Path(__file__).with_name('recorded-size-confounder-01.json')
assert not OUT.exists()
results = []
for label in ('screen-13', 'screen-14'):
    summary_path = BASE / label / f'{label}-volt1.summary.json'
    journal_path = BASE / label / f'{label}-ble.ble.notifications.jsonl'
    summary = json.loads(summary_path.read_text())
    start, end = summary['capture_monotonic_start'], summary['capture_monotonic_end']
    groups = {}
    for line in journal_path.open():
        item = json.loads(line)
        if item.get('type') != 'notification' or item.get('channel') != 'D':
            continue
        if not start <= item['monotonic_ns'] / 1e9 <= end:
            continue
        value = base64.b64decode(item['payload_base64'], validate=True)
        if not value.startswith(b'G,D,'):
            continue
        header, payload = value.split(b'\n', 1)
        fields = header.decode().split(',')
        mid, index, count, chunk, total = map(int, fields[2:7])
        assert len(payload) == chunk
        groups.setdefault((mid, count, total, fields[7]), {})[index] = payload
    sizes = Counter()
    sequence_digits = Counter()
    for (_, count, total, _), parts in groups.items():
        if len(parts) != count:
            continue
        payload = b''.join(parts[i] for i in range(count))
        assert len(payload) == total
        sizes[total] += 1
        header = payload.split(b'\n', 1)[0].decode()
        seq_fields = [x for x in header.split(',') if x.startswith('seq=')]
        assert len(seq_fields) == 1
        sequence_digits[len(seq_fields[0].split('=', 1)[1])] += 1
    results.append({'label': label, 'complete_messages': sum(sizes.values()),
                    'logical_message_bytes_histogram': dict(sorted(sizes.items())),
                    'logical_message_bytes_min_max': [min(sizes), max(sizes)],
                    'most_common_logical_bytes': [{'bytes': k, 'messages': v} for k,v in sizes.most_common(1)],
                    'sequence_decimal_digit_histogram': dict(sorted(sequence_digits.items())),
                    'inputs': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                               for p in (summary_path, journal_path)]})
result = {'status': 'RECORDED_MESSAGE_SIZE_CONFOUNDER', 'hardware_actions': [],
          'original_recordings_modified': False, 'screens': results,
          'limit': 'Different boots/uptime/sequence widths change logical message lengths. These windows do not isolate packing as the sole cause of the FPS difference. No over-air measurement.',
          'generator': {'path': str(Path(__file__)), 'sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}}
with OUT.open('x') as stream:
    json.dump(result, stream, indent=2); stream.write('\n')
print(json.dumps({'status': result['status'], 'screens': [
    {'label': s['label'], 'range': s['logical_message_bytes_min_max'],
     'mode': s['most_common_logical_bytes'], 'sequence_digits': s['sequence_decimal_digit_histogram']}
    for s in results]}))
