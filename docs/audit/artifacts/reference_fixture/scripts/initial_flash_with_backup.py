import subprocess,sys,datetime,pathlib,json,hashlib,os
root=pathlib.Path('/home/csc/esp/workspace/SensorArray');out=root/'docs/audit/artifacts/reference_fixture'
stamp=datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ');backup_dir=root/'validation_artifacts'/('device-backup-'+stamp);backup_dir.mkdir(parents=True,exist_ok=False);os.chmod(backup_dir,0o700)
backup=backup_dir/'flash_before_0x000000_0x320000.bin'
base=[sys.executable,'-m','esptool','--chip','esp32s3','--port','/dev/ttyACM0','--baud','460800','--before','default_reset','--after','hard_reset']
records=[]
def run(name,args):
 cmd=base+args;log=out/(stamp+'-'+name+'.txt');print('START',name,flush=True)
 with log.open('x') as f:
  p=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,timeout=180)
 txt=log.read_text();records.append({'command':cmd,'exit_code':p.returncode,'log':str(log.relative_to(root))});print(txt[-2000:],flush=True)
 if p.returncode:raise RuntimeError(name+' failed')
 return txt
info=run('flash-id',['flash_id'])
if '80:b5:4e:cd:d9:ac' not in info.lower():raise RuntimeError('unexpected device identity')
run('backup',['read_flash','0','0x320000',str(backup)]);os.chmod(backup,0o600)
original=backup.read_bytes();table=(root/'build/partition_table/partition-table.bin').read_bytes()
if original[0x8000:0x8000+len(table)]!=table:raise RuntimeError('partition mismatch')
m=json.loads((root/'docs/audit/artifacts/reliability_fix/final-manifest.json').read_text());app=root/'build/SensorArray.bin'
if hashlib.sha256(app.read_bytes()).hexdigest()!=m['artifacts']['build/SensorArray.bin']['sha256']:raise RuntimeError('approved image changed')
run('flash',['write_flash','--flash_mode','dio','--flash_freq','80m','--flash_size','16MB','0x0',str(root/'build/bootloader/bootloader.bin'),'0x10000',str(app)])
result={'firmware_commit':m['firmware_fix_commit'],'app_sha256':m['artifacts']['build/SensorArray.bin']['sha256'],'device_serial':'80:B5:4E:CD:D9:AC','backup_path':str(backup),'backup_sha256':hashlib.sha256(original).hexdigest(),'backup_bytes':len(original),'partition_identical':True,'nvs_or_calibration_written':False,'partition_table_written':False,'commands':records}
p=out/(stamp+'-flash-result.json');p.write_text(json.dumps(result,indent=2)+'\n');print('RESULT',p,flush=True)
