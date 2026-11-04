import subprocess,sys,json,hashlib,time
from pathlib import Path
root=Path('/home/csc/esp/workspace/SensorArray');out=root/'docs/audit/artifacts/reference_fixture'
m=json.loads((out/'crc-followup-build-manifest.json').read_text());app=root/'build/SensorArray.bin'
assert m['exit_code']==0 and hashlib.sha256(app.read_bytes()).hexdigest()==m['app_sha256']
base=[sys.executable,'-m','esptool','--chip','esp32s3','--port','/dev/ttyACM0','--baud','460800','--before','default_reset','--after','hard_reset'];records=[]
for name,args in [('identity',['flash_id']),('app',['write_flash','--flash_mode','dio','--flash_freq','80m','--flash_size','16MB','0x10000',str(app)])]:
 p=out/('crc-followup-flash-'+name+'.txt');cmd=base+args
 with p.open('x') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=180)
 records.append({'command':cmd,'exit_code':r.returncode,'log':str(p.relative_to(root))});print(p.read_text()[-1800:],flush=True)
 if r.returncode:raise RuntimeError(name+' failed')
 if name=='identity' and '80:b5:4e:cd:d9:ac' not in p.read_text().lower():raise RuntimeError('device mismatch')
with (out/'crc-followup-flash-result.json').open('x') as f:json.dump({'head':m['head'],'build_version':m['build_version'],'app_sha256':m['app_sha256'],'app_bytes':m['app_bytes'],'nvs_or_calibration_written':False,'partition_written':False,'bootloader_written':False,'commands':records},f,indent=2)
