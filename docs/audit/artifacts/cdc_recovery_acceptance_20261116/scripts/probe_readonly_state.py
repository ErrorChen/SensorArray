#!/usr/bin/env python3
"""One bounded read-only startup/state probe. No retries or control mutations."""
from contextlib import ExitStack
import argparse, hashlib, json, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'tools'))
from capture_reference_fixture import CaptureOptions,Collector
from sensorarray_usb_cdc import UsbCdcPort
p=argparse.ArgumentParser();p.add_argument('--label',required=True);a=p.parse_args()
if not all(c.isalnum() or c in '-_' for c in a.label):raise ValueError('label')
out=Path(__file__).resolve().parents[1]
paths={k:out/(a.label+v) for k,v in [('raw','.rx.bin'),('events','.events.jsonl'),('summary','.summary.json')]}
if any(v.exists() for v in paths.values()):raise FileExistsError('new label required')
queries=['BUILD?','PROTO?','BOOT?','READY?','FRONTENDS?','RESTART?','SPI?','CONFIG?','ADSTIMING?','SCHED?']
result={'readonly':True,'attempts':[]};port=None;collector=None;start=time.monotonic();deadline=start+30
with ExitStack() as stack:
 raw=stack.enter_context(paths['raw'].open('xb'));events=stack.enter_context(paths['events'].open('x'));summary=stack.enter_context(paths['summary'].open('x'))
 try:
  port=UsbCdcPort('80B54ECDD9AC');collector=Collector(CaptureOptions(None,30,out,a.label,command_timeout=2,transport='usb_cdc',usb_serial='80B54ECDD9AC'),port,raw,events);collector.total_deadline=deadline
  for command in queries:
   item={'command':command,'monotonic':time.monotonic()};result['attempts'].append(item)
   try:
    item['fields']=collector.query(command,'CPUOWNER' if command.startswith('CPUOWNER?') else command[:-1],deadline)
    if command=='PROTO?' and 'adc_finalbit_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.append('SPISOURCE?')
    if command=='PROTO?' and 'cpu_owner_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.extend('CPUOWNER?'+str(role) for role in range(7))
     queries.append('CONFIG?')
   except Exception as error:item['error']=str(error)
  result['status']='READONLY_PROBE_RECORDED'
 except Exception as error:result.update(status='ERROR',error=str(error))
 finally:
  if collector:result.update(faults=collector.faults,resets=collector.reset_observations,rx_bytes=collector.raw_bytes)
  if port:port.close()
  raw.flush();result.update(elapsed_seconds=time.monotonic()-start,raw_sha256=hashlib.sha256(paths['raw'].read_bytes()).hexdigest());json.dump(result,summary,indent=2);summary.write('\n')
print(json.dumps(result,indent=2))
