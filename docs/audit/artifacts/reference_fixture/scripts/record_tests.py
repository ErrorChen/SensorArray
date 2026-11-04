import hashlib,json,os,subprocess,sys,time
from pathlib import Path
out=Path('docs/audit/artifacts/reference_fixture'); label=sys.argv[1]
manifest=out/(label+'-test-results.json'); assert not manifest.exists()
result={'head':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),'dirty_diff_sha256':hashlib.sha256(subprocess.check_output(['git','diff','--binary'])).hexdigest(),'results':[]}
for script in sorted(Path('tools').glob('test_*.py')):
 log=out/(script.stem+'-'+label+'.txt'); command=['python3',str(script)]; start=time.monotonic()
 with log.open('x') as f:
  try:r=subprocess.run(command,env={**os.environ,'CC':'/usr/bin/cc'},stdout=f,stderr=subprocess.STDOUT,timeout=180); code=r.returncode
  except subprocess.TimeoutExpired:code=124; f.write('\nTEST_RUNNER_TIMEOUT_180_SECONDS\n')
 result['results'].append({'command':command,'env':{'CC':'/usr/bin/cc'},'exit_code':code,'duration_seconds':time.monotonic()-start,'log':str(log)})
 print(script.name,code,flush=True)
with manifest.open('x') as f:json.dump(result,f,indent=2)
sys.exit(1 if any(x['exit_code'] for x in result['results']) else 0)
