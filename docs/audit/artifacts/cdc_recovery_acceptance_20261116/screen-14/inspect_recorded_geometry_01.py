"""Read closed BLE recordings; preserve bytes and distinguish observed ATT from LL models."""
import base64
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import zlib

BASE = Path(__file__).resolve().parents[1]
OUT = Path(__file__).with_name('recorded-geometry-01.json')
assert not OUT.exists(), 'exclusive result only'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def inspect(label):
    summary_path = BASE / label / f'{label}-volt1.summary.json'
    journal_path = BASE / label / f'{label}-ble.ble.notifications.jsonl'
    summary = json.loads(summary_path.read_text())
    assert summary['status'] == 'CAPTURED'
    start, end = summary['capture_monotonic_start'], summary['capture_monotonic_end']
    groups = {}
    counts = Counter()
    observed_lengths = Counter()
    errors = []
    for line_number, line in enumerate(journal_path.open(), 1):
        event = json.loads(line)
        if event.get('type') != 'notification' or event.get('channel') != 'D':
            continue
        timestamp = event['monotonic_ns'] / 1e9
        if not start <= timestamp <= end:
            continue
        data = base64.b64decode(event['payload_base64'], validate=True)
        assert len(data) == event['length']
        counts['D_notifications'] += 1
        observed_lengths[len(data)] += 1
        if not data.startswith(b'G,'):
            counts['raw_notifications'] += 1
            continue
        header, payload = data.split(b'\n', 1)
        fields = header.decode('ascii').split(',')
        assert len(fields) == 8 and fields[:2] == ['G', 'D']
        mid, index, count, chunk, total = map(int, fields[2:7])
        crc = int(fields[7], 16)
        assert len(payload) == chunk and 0 <= index < count
        key = mid, count, total, crc
        group = groups.setdefault(key, {})
        if index in group:
            errors.append({'line': line_number, 'kind': 'duplicate_index', 'mid': mid, 'index': index})
            continue
        group[index] = {'payload': payload, 'value_bytes': len(data), 'header_bytes': len(header)+1,
                        'line': line_number, 'monotonic_ns': event['monotonic_ns']}
    geometry = Counter()
    model = Counter()
    examples = []
    total_lengths = Counter()
    for key, group in groups.items():
        mid, count, total, crc = key
        if len(group) != count:
            counts['incomplete_groups_at_capture_boundaries_or_inside_window'] += 1
            continue
        payload = b''.join(group[i]['payload'] for i in range(count))
        assert len(payload) == total and zlib.crc32(payload) & 0xffffffff == crc
        counts['complete_envelope_messages_crc_valid'] += 1
        total_lengths[total] += 1
        if count != 2:
            counts['complete_non_two_fragment_messages'] += 1
            continue
        lengths = tuple(group[i]['value_bytes'] for i in range(2))
        geometry[lengths] += 1
        counts['complete_two_fragment_messages'] += 1
        if lengths[0] <= 495 and lengths[1] <= 244:
            counts['two_fragment_messages_meeting_495_244_targets'] += 1
        base, extra = divmod(total, 2)
        old_lengths = tuple(len(f'G,D,{mid},{i},2,{base+(i<extra)},{total},{crc:08X}\n'.encode())
                            + base+(i<extra) for i in range(2))
        pdu_actual_geometry = sum(math.ceil((n+7)/251) for n in lengths)
        pdu_balanced_geometry = sum(math.ceil((n+7)/251) for n in old_lengths)
        model[(pdu_actual_geometry, pdu_balanced_geometry)] += 1
        if len(examples) < 3:
            examples.append({'message_id': mid, 'logical_bytes': total,
                             'observed_value_bytes': lengths,
                             'observed_header_bytes': [group[i]['header_bytes'] for i in range(2)],
                             'raw_journal_lines': [group[i]['line'] for i in range(2)],
                             'reassembled_sha256': hashlib.sha256(payload).hexdigest(),
                             'envelope_crc32': f'{crc:08X}',
                             'balanced_header_arithmetic_value_bytes_not_recorded': old_lengths})
    assert not errors
    return {'label': label,
            'capture_host_monotonic_seconds': {'start': start, 'end': end},
            'inputs': [{'path': str(p), 'sha256': sha(p)} for p in [summary_path, journal_path]],
            'counts': dict(counts),
            'observed_notification_lengths': dict(sorted(observed_lengths.items())),
            'complete_two_fragment_observed_value_geometry': [
                {'value_bytes': list(k), 'messages': v} for k,v in sorted(geometry.items())],
            'logical_message_length_min_max': [min(total_lengths), max(total_lengths)],
            'LL_assumption_model_not_measured': {
                'payload_bytes_assumed': 251, 'L2CAP_ATT_overhead_bytes_per_notification': 7,
                'peer_effective_DLE': 'UNKNOWN', 'over_air_trace': 'NOT_RUN',
                'formula': 'sum(ceil((observed ATT value bytes + 7)/251))',
                'comparison': [{'observed_geometry_assumed_PDUs': k[0],
                                'balanced_same_message_assumed_PDUs': k[1], 'messages': v}
                               for k,v in sorted(model.items())]},
            'examples': examples,
            'interpretation': 'ATT lengths and payload CRC are observed from host callbacks; LL counts are conditional arithmetic, not bus/radio measurements or throughput causation.'}

result = {'status': 'RECORDED_ATT_GEOMETRY_VERIFIED_LL_MODEL_ONLY', 'hardware_actions': [],
          'original_recordings_modified': False,
          'scope': 'D notifications whose callback timestamps lie inside each closed formal capture window; incomplete boundary groups are reported separately.',
          'screens': [inspect('screen-13'), inspect('screen-14')],
          'generator': {'path': str(Path(__file__)), 'sha256': sha(Path(__file__))}}
with OUT.open('x') as stream:
    json.dump(result, stream, indent=2); stream.write('\n')
print(json.dumps({'status': result['status'], 'screens': [
    {'label': s['label'], 'counts': s['counts'], 'geometry': s['complete_two_fragment_observed_value_geometry']}
    for s in result['screens']]}))
