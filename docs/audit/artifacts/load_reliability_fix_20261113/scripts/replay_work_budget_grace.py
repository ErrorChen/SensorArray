#!/usr/bin/env python3
"""Replay the saved pre-fix C and current C; no hardware or raw-file changes."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

parser=argparse.ArgumentParser()
parser.add_argument('--output-prefix', type=Path)
args=parser.parse_args()
root=next(p for p in Path(__file__).resolve().parents if (p/'components/sensorarrayWorkBudget').is_dir())
sys.path.insert(0,str(root/'tools'))
from test_rowmodes_ack_guard import findCompiler
base=Path(__file__).resolve().parent.parent
harness=base/'work-budget-grace-replay-01.c.txt'
sources={'before':base/'work-budget-before-grace-01.c.txt','after':root/'components/sensorarrayWorkBudget/sensorarrayWorkBudget.c'}
compiler,native=findCompiler()
assert native, 'native execution required'
results={}
with tempfile.TemporaryDirectory(prefix='sensorarray-grace-replay-') as directory:
    for label,source in sources.items():
        target=Path(directory)/label
        command=[compiler,'-std=c11','-Wall','-Wextra','-Werror','-x','c',str(harness),
                 '-DWORK_BUDGET_SOURCE="'+str(source)+'"']
        for include in (root/'tests/host/work_budget_stubs',root/'tests/host/include',root/'components/sensorarrayWorkBudget/include'):
            command+=['-I',str(include)]
        command+=['-o',str(target)]
        compile_run=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        assert compile_run.returncode==0, compile_run.stdout
        run=subprocess.run([str(target)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10)
        results[label]={'source':str(source.relative_to(root)),'sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                        'compile_command':command,'compile_exit_code':compile_run.returncode,'run_exit_code':run.returncode,'output':run.stdout}
assert results['before']['run_exit_code']==1 and 'yields=100,' in results['before']['output']
assert results['after']['run_exit_code']==0 and 'yields=0,' in results['after']['output']
report={'status':'REPRODUCED_AND_FIXED','harness_sha256':hashlib.sha256(harness.read_bytes()).hexdigest(),
        'results':results,'boundary':'A real CPU0 idle callback occurs before each checkpoint, then worker first resumes at age=2 ticks. Same C harness against both implementations. Synthetic scheduler, not HIL or a throughput forecast.'}
if args.output_prefix:
    output_json=Path(str(args.output_prefix)+'.json'); output_text=Path(str(args.output_prefix)+'.txt')
    assert not output_json.exists() and not output_text.exists(), 'refuse evidence overwrite'
    with output_json.open('x') as out: json.dump(report,out,indent=2);out.write('\n')
    with output_text.open('x') as out:
        for label,result in results.items(): out.write(label+'\n'+result['output']+'exit_code='+str(result['run_exit_code'])+'\n')
print(json.dumps({'status':report['status'],'before':results['before']['output'].strip(),'after':results['after']['output'].strip()}))
