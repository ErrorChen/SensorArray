import serial,time,json,datetime,pathlib,sys
out=pathlib.Path('/home/csc/esp/workspace/SensorArray/docs/audit/artifacts/reference_fixture')
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ');rawpath=out/(stamp+'-probe.rx.bin');eventpath=out/(stamp+'-probe.events.jsonl')
s=serial.Serial();s.port='/dev/ttyACM0';s.baudrate=115200;s.timeout=.1;s.write_timeout=1;s.exclusive=True;s.dtr=False;s.rts=False
commands=['BUILD?','PROTO?','CONFIG?','MODE?','ROWS?','ROWMODES?','CAL?','RAIL?','USBSTREAM?']
with rawpath.open('xb') as raw,eventpath.open('x') as events:
 s.open();start=time.monotonic();buffer=b''
 def drain(seconds):
  global buffer
  deadline=time.monotonic()+seconds
  while time.monotonic()<deadline:
   b=s.read(max(1,min(s.in_waiting,8192)))
   if not b:continue
   raw.write(b);raw.flush();buffer+=b
   while b'\n' in buffer:
    line,buffer=buffer.split(b'\n',1)
    txt=line.decode('ascii',errors='backslashreplace').rstrip('\r')
    events.write(json.dumps({'t':time.monotonic()-start,'rx':txt})+'\n');events.flush()
    if txt.startswith(('BUILD,','PROTO,','CONFIG,','MODE,','ROWS,','ROWMODES,','CAL,','ARL,','USBSTREAM,','ERR,','MFAULT,','APP_FATAL','Guru')): print(txt,flush=True)
 try:
  drain(2)
  for c in commands:
   events.write(json.dumps({'t':time.monotonic()-start,'tx':c})+'\n');events.flush();s.write((c+'\n').encode());drain(1.2)
  drain(1)
 finally:s.close()
print(json.dumps({'raw':str(rawpath),'events':str(eventpath),'bytes':rawpath.stat().st_size}),flush=True)
