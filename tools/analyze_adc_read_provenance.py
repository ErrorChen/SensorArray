#!/usr/bin/env python3
"""Independently check rejected ADC records without relabelling split reads as wire captures."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

from text_protocol import parse_fields


def crc_division(data):
    value = int.from_bytes(data, 'big') << 8
    for bit in range(len(data) * 8 + 7, 7, -1):
        if value & (1 << bit):
            value ^= 0x107 << (bit - 8)
    return value & 255


def hex_bytes(fields, key, length):
    value = fields.get(key, '')
    if not re.fullmatch('[0-9A-Fa-f]{' + str(length * 2) + '}', value):
        raise ValueError('missing_or_incomplete:' + key)
    return bytes.fromhex(value)


def decode_record(line):
    if not line.endswith(b'\n'):
        raise ValueError('incomplete_line')
    if not line.startswith(b'ADSSPIBYTES,'):
        raise ValueError('unsupported_record_tag')
    fields = parse_fields(line.decode('ascii').strip())
    if fields.get('len') != '6' or fields.get('crcMode') != '2' or fields.get('statusEnabled') != '1':
        raise ValueError('unsupported_payload_shape_or_integrity_mode')
    source = fields.get('source', 'legacy_spi56')
    result = {'seq': fields.get('seq'), 'source': source,
              'legacy_source_assumption': 'source' not in fields,
              'physical_signal_integrity_verified': False}
    if source in {'legacy_spi56', 'spi56', 'spi_bytes'}:
        if fields.get('wireKnown', '1' if source == 'legacy_spi56' else None) != '1':
            raise ValueError('standard_wire_not_known')
        payload = hex_bytes(fields, 'bytes', 6)
        result.update(wire_observed=True, wire_bytes=payload.hex().upper())
    elif source == 'spi55_gpio1':
        if fields.get('wireKnown') != '0' or fields.get('bytes') != 'unknown':
            raise ValueError('split_read_mislabeled_as_continuous_wire')
        if fields.get('gpioKnown') != '1' or fields.get('gpioBit') not in {'0', '1'}:
            raise ValueError('GPIO_sample_not_known')
        prefix = hex_bytes(fields, 'spiPrefix', 7)
        if prefix[6] & 1:
            raise ValueError('unreceived_SPI_bit_must_remain_unknown_zero_placeholder')
        payload = hex_bytes(fields, 'assembledBytes', 6)
        observed = prefix[1:6] + bytes([prefix[6] | int(fields['gpioBit'])])
        if payload != observed:
            raise ValueError('assembly_disagrees_with_physical_observations')
        if fields.get('tailStage') != 'complete':
            raise ValueError('CRC_record_without_complete_tail_transport')
        result.update(wire_observed=False, spi_prefix_hex=prefix.hex().upper(),
                      gpio_bit=int(fields['gpioBit']), assembled_bytes=payload.hex().upper())
    else:
        raise ValueError('unknown_read_source:' + source)
    computed = crc_division(payload[1:5])
    received = payload[5]
    if fields.get('computed', '').upper() != f'{computed:02X}' or fields.get('received', '').upper() != f'{received:02X}':
        raise ValueError('independent_CRC_disagrees_with_report')
    delta = computed ^ received
    result.update(computed=f'{computed:02X}', received=f'{received:02X}', xor=f'{delta:02X}',
                  status_unprotected=f'{payload[0]:02X}',
                  direction='0_to_1' if delta == 1 and received & 1 else '1_to_0' if delta == 1 else 'other')
    return result


def analyze(directory, phase):
    if not re.fullmatch('[A-Za-z0-9_-]+', phase):
        raise ValueError('invalid phase')
    raw_path = directory / (phase + '-debug.debug.rx.bin')
    summary_path = directory / (phase + '-debug.debug.summary.json')
    analysis_path = directory / (phase + '-analysis-01.json')
    raw = raw_path.read_bytes()
    summary = json.loads(summary_path.read_text())
    sha = hashlib.sha256(raw).hexdigest()
    if summary.get('raw_sha256') != sha:
        raise ValueError('capture_not_closed_or_raw_hash_mismatch')
    analysis = json.loads(analysis_path.read_text())
    health = analysis['captures'][0]['otg_diagnostics']['adc_crc']['health']
    if not health.get('first') or not health.get('last'):
        raise ValueError('no_complete_health_window')
    lo, hi = health['first']['byte_offset'], health['last']['byte_offset']
    records, errors = [], []
    offset = 0
    for line in raw.splitlines(keepends=True):
        start = offset
        offset += len(line)
        if not lo <= start <= hi or not line.startswith(b'ADSSPIBYTES,'):
            continue
        try:
            record = decode_record(line)
            record['byte_offset'] = start
            records.append(record)
        except (ValueError, UnicodeError) as error:
            errors.append({'byte_offset': start, 'error': str(error)})
    return {'status': 'ANALYSIS_COMPLETE' if not errors else 'ANALYSIS_FOUND_INVALID_EVIDENCE',
            'raw_file': str(raw_path), 'raw_sha256': sha,
            'analysis_file': str(analysis_path), 'analysis_sha256': hashlib.sha256(analysis_path.read_bytes()).hexdigest(),
            'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'window_offsets_inclusive': [lo, hi], 'records': records, 'errors': errors,
            'source_counts': dict(Counter(r['source'] for r in records)),
            'xor_by_source': {source: dict(Counter(r['xor'] for r in records if r['source'] == source))
                              for source in sorted({r['source'] for r in records})},
            'limits': 'Bounded rejected-read records, not all ADC attempts. STATUS is outside ADC CRC. No bit repair, inferred samples, physical waveform verification or acceptance PASS.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', required=True, type=Path)
    parser.add_argument('--phase', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--evidence-kind', required=True, choices=('recorded', 'synthetic'))
    args = parser.parse_args()
    result = analyze(args.directory, args.phase)
    result['input_evidence_kind'] = args.evidence_kind
    with args.output.open('x') as output:
        json.dump(result, output, indent=2)
        output.write('\n')
    print(json.dumps({key: result[key] for key in ('status', 'source_counts', 'xor_by_source')}))
    return int(bool(result['errors']))


if __name__ == '__main__':
    raise SystemExit(main())
