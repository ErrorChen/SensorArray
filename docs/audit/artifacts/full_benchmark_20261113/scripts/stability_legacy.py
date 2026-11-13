"""One exact 600-second stability window; real serial, no reset or NVS writes."""
import hashlib,json,subprocess,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5];OUT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from capture_reference_fixture import Collector,CaptureOptions,PosixPort,CaptureError
COMMANDS=[('CONFIG?','CONFIG'),('PERF?','PERF'),('USBSTREAM?','USBSTREAM'),('FPS?','FPS'),('MODE?','MODE'),('FRONTENDS?','FRONTENDS'),('SPI?','SPI'),('BAT?','ABAT'),('RAIL?','ARL'),('BOOT?','BOOT')]
class Stability(Collector):
 def __init__(self,*args):
  self.snapshots={'before':{},'after':{}};self.snapshot_errors={};self.armed=False;self.after_done=False
  super().__init__(*args)
 def event(self,kind,**fields):
  if kind=='capture_start':
   self.capture_deadline=self.capture_started+600.0
   if self.capture_deadline+15.0>self.total_deadline:raise CaptureError('insufficient_total_budget_for_600_seconds')
  fields.setdefault('monotonic',time.monotonic());fields.setdefault('offset',self.raw_bytes)
  super().event(kind,**fields)
 def snapshot(self,name,deadline):
  for cmd,tag in COMMANDS:
   fields=self.query(cmd,tag,deadline)
   self.snapshots[name][cmd]={'fields':fields,'monotonic':time.monotonic(),'t':time.monotonic()-self.started}
   if cmd=='CONFIG?':
    if fields.get('mode')!='VOLT' or fields.get('rows')!='1':raise CaptureError('stability_configuration_changed')
    if name=='after' and any(fields.get(k)!=self.snapshots['before'][cmd]['fields'].get(k) for k in ('session','bootId','rev','mgen','mrid','rgen','rrid','pgen','prid')):raise CaptureError('stability_identity_changed')
   if cmd=='FPS?' and (fields.get('cfcap')!='0' or fields.get('ofcap')!='0'):raise CaptureError('unexpected_fps_limit')
 def transition(self,kind,value):
  if kind=='MODE' and value=='VOLT':
   super().transition('MODE','CAP')
   rail=self.query('RAIL?','ARL',self.prepare_deadline)
   expected={'src':'external','rail':'5145000','avdd':'3350000','avss':'-1795000','rv':'1','rs':'ok','split':'external_measured'}
   if any(rail.get(k)!=v for k,v in expected.items()):raise CaptureError('external_rail_calibration_not_verified')
  super().transition(kind,value)
 def query(self,command,tag,deadline):
  f=super().query(command,tag,deadline)
  if command=='MODE?' and f.get('active')=='VOLT' and (f.get('intref')!='0' or f.get('refmux')!='24'):raise CaptureError('VOLT_reference_not_verified')
  return f
 def set_usb(self,mode,deadline):
  if self.phase=='cleanup' and not self.after_done:
   self.after_done=True
   if self.faults:
    end=min(time.monotonic()+2,self.total_deadline-10)
    self.event('fault_tail_start')
    while time.monotonic()<end:self.pump(end);self.lines.clear()
    self.event('fault_tail_end')
   try:self.snapshot('after',min(time.monotonic()+8,self.total_deadline-3))
   except Exception as e:self.snapshot_errors['after']=str(e);self.event('snapshot_error',name='after',error=str(e))
  result=super().set_usb(mode,deadline)
  if self.phase=='prepare' and mode=='FULL' and not self.armed:
   self.armed=True;self.phase='warmup';self.event('warmup_start',requested_seconds=5)
   end=time.monotonic()+5
   while time.monotonic()<end:self.pump(end);self.lines.clear()
   self.event('warmup_end');self.phase='before';self.snapshot('before',self.prepare_deadline);self.phase='prepare'
  return result
label=sys.argv[1];rawpath=OUT/(label+'.rx.bin');failure=None
with rawpath.open('xb') as raw,(OUT/(label+'.events.jsonl')).open('x') as events,(OUT/(label+'.summary.json')).open('x') as report:
 port=PosixPort('/dev/ttyACM0');c=Stability(CaptureOptions('/dev/ttyACM0',680,OUT,label,mode='VOLT',rows=1),port,raw,events)
 try:
  result=c.run()
  c.phase='final_restore';c.prepare_deadline=min(time.monotonic()+8,c.total_deadline)
  c.transition('MODE','CAP');c.transition('ROWS',8)
  result['final_config']=c.query('CONFIG?','CONFIG',c.prepare_deadline)
 except Exception as e:
  failure=str(e);c.event('final_error',error=failure);result=locals().get('result',{'status':'ERROR','error':failure})
 finally:port.close()
 raw.flush()
 result.update(benchmark_kind='stability',capture_requested_seconds=600,warmup_requested_seconds=5,capture_monotonic_start=c.capture_started,capture_monotonic_end=c.capture_finished,snapshots=c.snapshots,snapshot_errors=c.snapshot_errors,final_restore_error=failure,raw_file=str(rawpath),raw_sha256=hashlib.sha256(rawpath.read_bytes()).hexdigest(),head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),nonvolatile_calibration_written=False)
 result['full_requested_window_completed']=result.get('capture_seconds',0)>=600 and result.get('status')=='CAPTURED'
 json.dump(result,report,indent=2);report.write('\n')
 print(json.dumps({k:result.get(k) for k in ['status','error','capture_seconds','capture_frame_trailers','full_requested_window_completed','snapshot_errors','final_restore_error']}),flush=True)
raise SystemExit(0 if result['full_requested_window_completed'] and not failure and not c.snapshot_errors else 1)
