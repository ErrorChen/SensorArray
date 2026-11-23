#!/usr/bin/env python3
"""One bounded read-only startup/state probe. No retries or control mutations."""
from contextlib import ExitStack
import argparse, hashlib, json, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(ROOT/'tools'))
from capture_reference_fixture import CaptureOptions,Collector,PosixPort
from sensorarray_usb_cdc import UsbCdcPort
from check_software_recovery import usb_inventory
p=argparse.ArgumentParser();p.add_argument('--label',required=True);p.add_argument('--ble-guard',action='store_true',help='Probe the additive BLEGUARD v1 query on a known supporting image');p.add_argument('--ble-log',action='store_true',help='Probe the additive BLELOG v1 query on a known supporting image');p.add_argument('--usj',action='store_true',help='Read the same board application recovery over USB Serial/JTAG, without modem-line reset');p.add_argument('--panic',action='store_true',help='Read PANIC?/1/2 on a known supporting image');p.add_argument('--memfail',action='store_true',help='Read MEMFAIL? on a known supporting image');a=p.parse_args()
if not all(c.isalnum() or c in '-_' for c in a.label):raise ValueError('label')
out=Path(__file__).resolve().parents[1]
paths={k:out/(a.label+v) for k,v in [('raw','.rx.bin'),('events','.events.jsonl'),('summary','.summary.json')]}
if any(v.exists() for v in paths.values()):raise FileExistsError('new label required')
queries=['BUILD?','PROTO?','BOOT?','READY?','FRONTENDS?','RESTART?','SPI?','CONFIG?','ADSTIMING?','SCHED?']
if a.ble_guard:queries.append('BLEGUARD?')
if a.ble_log:queries.append('BLELOG?')
if a.panic:queries.extend(('PANIC?','PANIC?1','PANIC?2'))
if a.memfail:queries.append('MEMFAIL?')
result={'readonly':True,'attempts':[]};port=None;collector=None;start=time.monotonic();deadline=start+30
with ExitStack() as stack:
 raw=stack.enter_context(paths['raw'].open('xb'));events=stack.enter_context(paths['events'].open('x'));summary=stack.enter_context(paths['summary'].open('x'))
 try:
  if a.usj:
   devices=[d for d in usb_inventory() if d['serial']=='80B54ECDD9AC' and d['pid']=='1001']
   if len(devices)!=1 or len(devices[0]['ttys'])!=1:raise ValueError('unique_USJ_board_required')
   result['inventory']=devices;port=PosixPort(devices[0]['ttys'][0])
  else:port=UsbCdcPort('80B54ECDD9AC')
  collector=Collector(CaptureOptions(None,30,out,a.label,command_timeout=2,transport='posix' if a.usj else 'usb_cdc',usb_serial='80B54ECDD9AC'),port,raw,events);collector.total_deadline=deadline
  for command in queries:
   item={'command':command,'monotonic':time.monotonic()};result['attempts'].append(item)
   try:
    tag = ('CPUOWNER' if command.startswith('CPUOWNER?') else
           'PANIC' if command.startswith('PANIC?') else
           'WIFIQ' if command.startswith('WIFIQ?') else
           'ACK' if command == 'WIFI?' else command[:-1])
    item['fields']=collector.query(command,tag,deadline)
    if command=='PROTO?' and 'caps2_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.append('CAPS2?')
    if command=='CAPS2?' and 'wifi_owner_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.extend(('WIFI?','WIFIGUARD?','WIFIQ?D','WIFIQ?L','WIFIQ?C','CPUOWNER?7'))
    if command=='PROTO?' and 'adc_finalbit_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.append('SPISOURCE?')
    if command=='PROTO?' and 'cpu_owner_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.extend('CPUOWNER?'+str(role) for role in range(7))
     queries.append('CONFIG?')
    if command=='PROTO?' and 'heap_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.append('MEM?')
    if command=='PROTO?' and 'ble_flow_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.append('BLEFLOW?')
    if command=='PROTO?' and 'ble_snapshot_v1' in str(item['fields'].get('caps','')).split('/'):
     queries.extend(('BLESNAP?','SNAPSRC?','SNAPNET?'))
   except Exception as error:item['error']=str(error)
  result['status']='READONLY_PROBE_RECORDED'
 except Exception as error:result.update(status='ERROR',error=str(error))
 finally:
  if collector:result.update(faults=collector.faults,resets=collector.reset_observations,rx_bytes=collector.raw_bytes)
  if port:port.close()
  raw.flush();result.update(elapsed_seconds=time.monotonic()-start,raw_sha256=hashlib.sha256(paths['raw'].read_bytes()).hexdigest());json.dump(result,summary,indent=2);summary.write('\n')
print(json.dumps(result,indent=2))
raise SystemExit(1 if result['status']=='ERROR' or any('error' in item for item in result['attempts']) else 0)
