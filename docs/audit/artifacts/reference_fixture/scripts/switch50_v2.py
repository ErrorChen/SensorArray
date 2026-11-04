"""Separate control-identity checks from independently recorded measurement failures.
A valid MAPP/frame identity does not promise electrically valid reference data.
Exit code covers control/identity only; summary always reports measurement status.
"""
"""Real passive-fixture CAP/RES control test; no voltage calibration or NVS write."""
import sys,time,json,hashlib
from pathlib import Path
sys.path.insert(0,'/home/csc/esp/workspace/SensorArray/tools')
from capture_reference_fixture import PosixPort,Collector,CaptureOptions,CaptureError
from text_protocol import TextProtocolParser,parse_fields

label=sys.argv[1]
out=Path('/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/reference_fixture')
rawpath=out/(label+'.rx.bin');eventpath=out/(label+'.events.jsonl');reportpath=out/(label+'.summary.json')
results=[];measurement_failures=[];failure=None;original=None;boot=None
with rawpath.open('xb') as raw,eventpath.open('x') as events,reportpath.open('x') as report:
 port=PosixPort('/dev/ttyACM0');c=Collector(CaptureOptions('/dev/ttyACM0',120,out,label),port,raw,events)
 c.prepare_deadline=c.capture_deadline
 try:
  original=c.query('USBSTREAM?','USBSTREAM',c.prepare_deadline)
  c.transition('MODE','CAP');c.transition('ROWS',8)
  c.set_usb('FULL',c.prepare_deadline)
  for cycle in range(50):
   mode='RES' if cycle%2==0 else 'CAP'
   c.phase='prepare';c.transition('MODE',mode)
   state=c.query('CONFIG?','CONFIG',c.prepare_deadline)
   if state.get('mode')!=mode or state.get('rows')!='8' or state.get('mterm')!='applied' or state.get('mrid')!=c.replies['MODE?'].get('rid'):
    raise CaptureError('authority_does_not_confirm_request')
   current_boot=(state.get('session'),state.get('bootId'))
   if boot is not None and boot!=current_boot:raise CaptureError('unexpected_boot_change')
   boot=current_boot
   parser=TextProtocolParser();header=None;observed=[];c.phase='capture'
   deadline=c.command_deadline(c.capture_deadline)
   while len(observed)<3:
    if c.faults:raise CaptureError('firmware_fault:'+c.faults[-1])
    line=c.next_line(deadline)
    if line.startswith(('C,','R,','V,','M,')):header=parse_fields(line)
    frame=parser.feed_line(line)
    if frame is None:continue
    if not frame.crc_ok:raise CaptureError('frame_crc')
    if not header:raise CaptureError('missing_header')
    for key in ('session','bootId','rev','mgen','mrid','rgen','rrid','pgen','prid'):
     if header.get(key)!=state.get(key):raise CaptureError('frame_config_identity:'+key)
    trailer=parse_fields(line)
    for key in ('session','bootId','rev'):
     if trailer.get(key)!=header.get(key):raise CaptureError('trailer_identity:'+key)
    if frame.rows!=8 or ((mode=='CAP')!=(type(frame).__name__=='CapFrame')) or (mode=='RES' and getattr(frame,'mode',None)!='RES'):
     raise CaptureError('frame_type_or_rows')
    if frame.sequence<int(state['boundary']):raise CaptureError('frame_before_applied_boundary')
    observed.append(frame.sequence)
   cell=None
   if mode=='RES':
    cell=c.query('CELL?=S1D1','CELL',c.prepare_deadline)
    if cell.get('mode')!='RES':raise CaptureError('S1D1_mode_mismatch_after_RES_application')
    if cell.get('valid')!='1' or cell.get('fresh')!='1':
     measurement_failures.append({'cycle':cycle+1,'cell':cell.copy()})
    crc=0
    for byte in int(cell['raw']).to_bytes(4,'big',signed=True):
     crc^=byte
     for _ in range(8):crc=((crc<<1)^0x07)&255 if crc&128 else (crc<<1)&255
    cell['derived_crc_of_diagnostic_raw']=f'{crc:02X}'
    cell['derived_crc_is_not_recorded_wire_crc']=True
   results.append({'cycle':cycle+1,'mode':mode,'authority':state,'verified_frame_sequences':observed,'S1D1_snapshot':cell})
   c.event('transition_verified',**results[-1]);print('verified',cycle+1,mode,observed,flush=True)
 except Exception as error:
  failure=str(error);c.event('test_error',error=failure)
 finally:
  c.phase='cleanup'
  if original:
   try:
    c.set_usb(original['v'],c.total_deadline)
    restored=c.query('USBSTREAM?','USBSTREAM',c.total_deadline)
    if any(restored.get(k)!=original.get(k) for k in ('v','dataEvery','diagEvery')):raise CaptureError('USB_restore_mismatch')
   except Exception as error:failure=(failure+';' if failure else '')+str(error)
  port.close()
 raw.flush();result={'status':'FAIL' if failure or len(results)!=50 or measurement_failures else 'CONTROL_PASS_MEASUREMENT_UNCALIBRATED','control_identity_status':'FAIL' if failure or len(results)!=50 else 'PASS','measurement_status':'FAIL' if measurement_failures else 'NOT_PRECISION_ACCEPTED','measurement_failures':measurement_failures,'error':failure,'completed_requests':len(results),'results':results,'firmware_faults':c.faults,'reset_observations':c.reset_observations,'raw_sha256':hashlib.sha256(rawpath.read_bytes()).hexdigest(),'elapsed_seconds':time.monotonic()-c.started,'calibration_written':False}
 report.write(json.dumps(result,indent=2)+'\n');print(result['status'],result['error'],flush=True)
sys.exit(0 if result['control_identity_status']=='PASS' else 1)
