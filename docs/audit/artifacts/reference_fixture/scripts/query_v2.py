import sys,time,json,hashlib
from pathlib import Path
sys.path.insert(0,'/home/csc/esp/workspace/SensorArray/tools')
from capture_reference_fixture import PosixPort,Collector,CaptureOptions
label=sys.argv[1]
commands=sys.argv[2:] or ['CONFIG?','MODE?','ROWS?','ROWMODES?','CELL?=S1D1','CELL?=S8D8','CELL?=S4D4','CELL?=S5D5','RAIL?','BAT?','USBSTREAM?','PERF?']
directory=Path('/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/reference_fixture')
rawpath=directory/(label+'.rx.bin');eventpath=directory/(label+'.events.jsonl');reportpath=directory/(label+'.summary.json')
with rawpath.open('xb') as raw,eventpath.open('x') as events,reportpath.open('x') as report:
 port=PosixPort('/dev/ttyACM0')
 c=Collector(CaptureOptions('/dev/ttyACM0',45,directory,label),port,raw,events)
 try:
  for command in commands:
   tag={'RAIL?':'ARL','CELL?':'CELL','BAT?':'ABAT'}.get(command.split('=')[0],command.split('=')[0].rstrip('?'))
   try:
    fields=c.query(command,tag,c.total_deadline)
    print(command,json.dumps(fields),flush=True)
   except Exception as e:
    c.event('query_error',command=command,error=str(e));print('ERROR',command,str(e),flush=True)
 finally:port.close()
 raw.flush();report.write(json.dumps({'commands':commands,'replies':c.replies,'raw_sha256':hashlib.sha256(rawpath.read_bytes()).hexdigest(),'modem_bit_ioctl':False,'reset_observations':c.reset_observations,'calibration_written':False},indent=2)+'\n')
