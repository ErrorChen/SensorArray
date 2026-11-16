#!/usr/bin/env python3
"""One read-only retrieval of cached TAIL id7 on the identified boot; no action."""
import hashlib,json,sys,time
from pathlib import Path
from contextlib import ExitStack
ROOT=Path(__file__).resolve().parents[5];sys.path.insert(0,str(ROOT/'tools'))
from capture_reference_fixture import CaptureOptions,Collector
from sensorarray_usb_cdc import UsbCdcPort
out=Path(__file__).resolve().parents[1];label='tail-failure-context-01';start=time.monotonic();deadline=start+30
result={'readonly':True,'actions_sent':0,'cached_id':7,'replies':[]};port=None;collector=None
with ExitStack() as stack:
 raw=stack.enter_context((out/(label+'.rx.bin')).open('xb'));events=stack.enter_context((out/(label+'.events.jsonl')).open('x'));summary=stack.enter_context((out/(label+'.summary.json')).open('x'))
 try:
  port=UsbCdcPort('80B54ECDD9AC');collector=Collector(CaptureOptions(None,30,out,label,command_timeout=2,transport='usb_cdc',usb_serial='80B54ECDD9AC'),port,raw,events);collector.total_deadline=deadline
  def query(command,tag):
   fields=collector.query(command,tag,deadline);result['replies'].append({'command':command,'fields':fields});return fields
  assert query('BUILD?','BUILD')['elfSha256']=='2cd47eb02b556120fc9459a0cc2ec9c394ca95ac5313a7c4284b4267800cf93b'
  before=query('ADSTIMING?','ADSTIMING');assert before['session']=='4C618C351450A839' and before['lastId']=='7' and before['busy']=='0' and before['lastProbe']=='TAIL'
  probe=query('ADSTIMING?RESULT,7','ADSTIMING');assert probe['id']=='7';n=int(probe['records']);assert 0<=n<=32
  for i in range(4):
   query(f'ADSTIMING?GROUP,7,{i}','ADSTIMINGGROUP');query(f'ADSTIMING?TIME,7,{i}','ADSTIMINGTIME')
  for i in range(2):query(f'ADSTIMING?REG,7,{i}','ADSTIMINGREG')
  for i in range(n):
   query(f'ADSTIMING?RECORD,7,{i}','ADSTIMINGRAW');query(f'ADSTIMING?TAIL,7,{i}','ADSTIMINGTAIL')
  for cmd in ('CONFIG?','ROUTE?','SPI?','FRONTENDS?'):query(cmd,cmd[:-1])
  after=query('ADSTIMING?','ADSTIMING');assert after['session']==before['session'] and after['lastId']=='7' and after['busy']=='0'
  result['status']='CACHED_FAILURE_RECORDED_NOT_ACCEPTANCE_PASS'
 except Exception as error:result.update(status='ERROR',error=str(error),error_class=type(error).__name__)
 finally:
  if collector:result.update(faults=collector.faults,resets=collector.reset_observations,rx_bytes=collector.raw_bytes)
  if port:port.close()
  raw.flush();result.update(elapsed_seconds=time.monotonic()-start,raw_sha256=hashlib.sha256((out/(label+'.rx.bin')).read_bytes()).hexdigest(),source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest());json.dump(result,summary,indent=2);summary.write('\n')
print(json.dumps({'status':result['status'],'error':result.get('error'),'replies':len(result['replies'])}))
raise SystemExit(result['status']=='ERROR')
