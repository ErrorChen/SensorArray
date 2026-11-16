#!/usr/bin/env python3
"""Ten planned independent probes, only after successful restoration; no failed-action retry."""
from pathlib import Path
import subprocess,sys,json,hashlib,time
stage=Path(__file__).resolve().parents[1];script=Path(__file__).resolve().parent
expected='a2557c8f07f1e1735313315dd65bd71f4e2b79b58bbf284338888fa9e2220d5c';session='18407FDC95220620'
out=stage/'tail-repeat-batch-01.json';assert not out.exists()
result={'status':'RUNNING','planned_independent_requests':10,'failure_retries':0,'required_full_elf':expected,'required_session':session,'results':[]};started=time.monotonic()
try:
 prior=json.loads((stage/'tail-probe-02.summary.json').read_text());assert prior['status']=='DIAGNOSTIC_COMPLETE' and prior['before']['CONFIG?']['session']==session
 for n in range(3,13):
  label=f'tail-probe-{n:02d}'
  command=[sys.executable,str(script/'record_command.py'),'--label',label,'--timeout','100','--',sys.executable,str(script/'capture_ads_timing.py'),'--output-directory',str(stage),'--label',label,'--expected-elf',expected,'--probe','TAIL']
  run=subprocess.run(command,timeout=110)
  p=stage/(label+'.summary.json');d=json.loads(p.read_text()) if p.exists() else {}
  entry={'label':label,'command':command,'exit_code':run.returncode,'status':d.get('status'),'summary_sha256':hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None};result['results'].append(entry)
  if run.returncode or d.get('status')!='DIAGNOSTIC_COMPLETE' or d['before']['CONFIG?']['session']!=session or d['probe']['restoreVerified']!='1' or d['probe']['isolate']!='0':
   result['status']='STOPPED_ON_UNSUCCESSFUL_OR_CHANGED_STATE';break
  print(json.dumps({'label':label,'groups_crc':[int(g['stats']['crc']) for g in d['group_results']]}),flush=True)
 else:result['status']='ALL_TEN_DIAGNOSTICS_COMPLETE_NOT_SCIENTIFIC_ACCEPTANCE'
except Exception as e:result.update(status='ERROR',error=str(e))
finally:
 result['elapsed_seconds']=time.monotonic()-started;result['source_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest();out.write_text(json.dumps(result,indent=2)+'\n')
raise SystemExit(result['status']!='ALL_TEN_DIAGNOSTICS_COMPLETE_NOT_SCIENTIFIC_ACCEPTANCE')
