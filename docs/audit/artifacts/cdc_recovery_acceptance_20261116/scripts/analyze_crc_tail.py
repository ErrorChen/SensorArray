#!/usr/bin/env python3
"""Analyze complete real ADSSPIBYTES lines from a closed, hashed capture only."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from capture_ads_timing import crc_by_division
from text_protocol import parse_fields

p=argparse.ArgumentParser(); p.add_argument('--directory', type=Path, required=True)
p.add_argument('--phase', required=True); p.add_argument('--output', type=Path, required=True)
a=p.parse_args()
if not re.fullmatch(r'[A-Za-z0-9_-]+', a.phase): raise ValueError('invalid phase')
rawpath=a.directory/(a.phase+'-debug.debug.rx.bin')
analysispath=a.directory/(a.phase+'-analysis-01.json')
summarypath=a.directory/(a.phase+'-debug.debug.summary.json')
summary=json.loads(summarypath.read_text())
raw=rawpath.read_bytes()
# Observer must have completed; do not silently analyze an active file.
if summary.get('status') not in {'COMPLETE','OBSERVED','STOPPED','RECORDED','QUERIES_COMPLETE'}:
    if not summary.get('raw_sha256'): raise ValueError('observer_not_closed_or_unrecognized')
rawsha=hashlib.sha256(raw).hexdigest()
if summary.get('raw_sha256') != rawsha: raise ValueError('closed_raw_hash_mismatch')
j=json.loads(analysispath.read_text())
health=j['captures'][0]['otg_diagnostics']['adc_crc']['health']
lo,hi=health['first']['byte_offset'],health['last']['byte_offset']
records=[]; malformed=[]; mismatches=[]; xor=Counter(); direction=Counter(); status=Counter()
offset=0
for wireline in raw.splitlines(keepends=True):
    start=offset; offset+=len(wireline)
    if not lo<=start<=hi or not wireline.startswith(b'ADSSPIBYTES,'): continue
    if not wireline.endswith(b'\n'):
        malformed.append({'offset':start,'reason':'no_complete_line_delimiter'}); continue
    fields=parse_fields(wireline.decode('ascii').strip())
    if not re.fullmatch('[0-9A-Fa-f]{12}',fields.get('bytes','')) or fields.get('len')!='6':
        malformed.append({'offset':start,'reason':'not_full_six_byte_packet'}); continue
    data=bytes.fromhex(fields['bytes']); calculated=crc_by_division(data[1:5]); received=data[5]
    if fields.get('computed','').upper()!=f'{calculated:02X}' or fields.get('received','').upper()!=f'{received:02X}':
        mismatches.append({'offset':start,'fields':fields,'independent_computed':calculated})
    delta=calculated^received; xor[f'{delta:02X}']+=1; status[f'{data[0]:02X}']+=1
    key='0_to_1' if delta==1 and received&1 else '1_to_0' if delta==1 else 'other'
    direction[key]+=1
    records.append({'byte_offset':start,'seq':fields.get('seq'),'bytes':fields['bytes'],
                    'computed':f'{calculated:02X}','received':f'{received:02X}','xor':f'{delta:02X}',
                    'direction':key})
result={'source':'closed_real_debug_capture_not_synthetic','raw_file':str(rawpath),'raw_sha256':rawsha,
        'analysis_file':str(analysispath),'analysis_sha256':hashlib.sha256(analysispath.read_bytes()).hexdigest(),
        'window_offsets_inclusive':[lo,hi],'complete_packet_records':len(records),'xor_counts':dict(xor),
        'directions':dict(direction),'wire_status_counts_untrusted':dict(status),
        'independent_recomputation_mismatches':mismatches,'malformed_or_truncated':malformed,'records':records,
        'limitations':'Rate-limited first-bad packet evidence only; not every failed read. CRC excludes STATUS. No bit repair, new raw frame or causal conclusion.'}
with a.output.open('x') as f: json.dump(result,f,indent=2); f.write('\n')
print(json.dumps({k:result[k] for k in ['complete_packet_records','xor_counts','directions','independent_recomputation_mismatches','malformed_or_truncated']}))
raise SystemExit(1 if mismatches else 0)
