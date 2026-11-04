import subprocess,json,hashlib,time,sys
from pathlib import Path
out=Path('docs/audit/artifacts/reference_fixture');label=sys.argv[1];log=out/(label+'-build.txt');manifest=out/(label+'-build-manifest.json')
assert not log.exists() and not manifest.exists()
cmd=['bash','-lc','source /home/csc/esp/esp-idf-v5.5.5/export.sh >/dev/null && idf.py reconfigure build'];start=time.monotonic()
with log.open('x') as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT)
result={'command':cmd,'exit_code':r.returncode,'duration_seconds':time.monotonic()-start,'log':str(log),'head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'dirty_diff_sha256':hashlib.sha256(subprocess.check_output(['git','diff','--binary'])).hexdigest(),'status':subprocess.check_output(['git','status','--short'],text=True)}
if r.returncode==0:
 result.update(app_sha256=hashlib.sha256(Path('build/SensorArray.bin').read_bytes()).hexdigest(),app_bytes=Path('build/SensorArray.bin').stat().st_size,build_version=json.loads(Path('build/project_description.json').read_text())['project_version'])
 names=set(subprocess.check_output(['git','ls-files'],text=True).splitlines())|{'sdkconfig','sdkconfig.defaults','CMakeLists.txt','dependencies.lock','partitions.csv'}
 result['source_files']=[{'path':name,'sha256':hashlib.sha256(Path(name).read_bytes()).hexdigest()} for name in sorted(names) if Path(name).is_file() and (name.startswith(('main/','core/','components/')) or name in {'sdkconfig','sdkconfig.defaults','CMakeLists.txt','dependencies.lock','partitions.csv'})]
with manifest.open('x') as f:json.dump(result,f,indent=2)
print('build_exit',r.returncode);print(log.read_text()[-2500:]);sys.exit(r.returncode)
