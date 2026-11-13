import hashlib,json,sys,subprocess
from pathlib import Path
root=Path(__file__).resolve().parents[5]
out=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'tools'))
from capture_reference_fixture import Collector,CaptureOptions,PosixPort
label=sys.argv[1];rawpath=out/(label+'.rx.bin')
with rawpath.open('xb') as raw,(out/(label+'.events.jsonl')).open('x') as events:
 p=PosixPort('/dev/ttyACM0');c=Collector(CaptureOptions('/dev/ttyACM0',45,out,label),p,raw,events);failure=None
 try:
  for command,tag in [('BUILD?','BUILD'),('PROTO?','PROTO'),('BOOT?','BOOT'),('CONFIG?','CONFIG'),('MODE?','MODE'),('ROWS?','ROWS'),('ROWMODES?','ROWMODES'),('FPS?','FPS'),('SPI?','SPI'),('FRONTENDS?','FRONTENDS'),('RAIL?','ARL'),('BAT?','ABAT'),('CAL?','CAL'),('USBSTREAM?','USBSTREAM'),('PERF?','PERF'),('TX?','TX'),('ST?','ST'),('BTX?','BTX'),('WIFI?','WIFI')]:
   print(command,json.dumps(c.query(command,tag,c.total_deadline)),flush=True)
 except Exception as e:failure=str(e);c.event('error',reason=failure)
 finally:p.close()
 raw.flush()
 d={'status':'ERROR' if failure else 'QUERIES_COMPLETE','error':failure,'replies':c.replies,'raw_sha256':hashlib.sha256(rawpath.read_bytes()).hexdigest(),'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),'flash_reset_or_calibration_written':False}
 with (out/(label+'.summary.json')).open('x') as f:json.dump(d,f,indent=2)
 if failure:raise SystemExit(failure)
