#!/usr/bin/env python3
"""Replay before/after BLE fragment scheduling against the same b44 WorkBudget."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

parser=argparse.ArgumentParser();parser.add_argument('--output-prefix',type=Path);args=parser.parse_args()
root=next(p for p in Path(__file__).resolve().parents if (p/'components/sensorarrayBle').is_dir())
base=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(root/'tools'))
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler
revision='b44c37b51067e376e73aa44535ea47d175846b67'
sources={'before':base/'ble-before-fragment-batch-01.c.txt','after':root/'components/sensorarrayBle/sensorarrayBle.c'}
harness=(base/'ble-fragment-batch-replay-01.c.txt').read_text()
compiler,native=findCompiler();assert native
results={}
with tempfile.TemporaryDirectory(prefix='sensorarray-ble-batch-replay-') as temporary:
    temporary=Path(temporary)
    for name,path in [('sensorarrayWorkBudget.c','components/sensorarrayWorkBudget/sensorarrayWorkBudget.c'),
                      ('sensorarrayWorkBudget.h','components/sensorarrayWorkBudget/include/sensorarrayWorkBudget.h')]:
        (temporary/name).write_bytes(subprocess.check_output(['git','show',revision+':'+path],cwd=root))
    frame=subprocess.check_output(['git','show',revision+':tests/fixtures/config_identity/volt_1.frame'],cwd=root)
    (temporary/'work_budget_frame.h').write_text('static const uint8_t referenceFrame[] = {'+','.join(str(b) for b in frame)+'};\n')
    for label,path in sources.items():
        source=path.read_text();expanded=harness
        for signature in ('static uint32_t sensorarrayBleCrc32(', 'static size_t sensorarrayBleMaxPayloadSize(',
                          'static bool sensorarrayBlePlanFragments(', 'static esp_err_t sensorarrayBleSendFragment(',
                          'static esp_err_t sensorarrayBleSendPayload('):
            name=signature.split('(')[0].split()[-1]
            expanded=expanded.replace('/* '+name+' */',function(source,signature))
        generated=temporary/(label+'.c');generated.write_text(expanded);target=temporary/label
        command=[compiler,'-std=c11','-Wall','-Wextra','-Werror','-I',str(temporary)]
        for include in (root/'tests/host/work_budget_stubs',root/'tests/host/include'):
            command+=['-I',str(include)]
        command += [str(generated),'-o',str(target)]
        compile_run=subprocess.run(command,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        assert compile_run.returncode==0,compile_run.stdout
        run=subprocess.run([str(target)],text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=10)
        results[label]={'source':str(path.relative_to(root)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
                        'compile_command':command,'compile_exit_code':compile_run.returncode,'run_exit_code':run.returncode,'output':run.stdout}
assert results['before']['run_exit_code']==1 and 'checkpoints=3,yields=2,' in results['before']['output']
assert results['after']['run_exit_code']==0 and 'checkpoints=1,yields=0,' in results['after']['output']
report={'status':'REPRODUCED_AND_FIXED','global_work_budget_revision':revision,
        'reference_frame_sha256':hashlib.sha256(frame).hexdigest(),'reference_frame_kind':'Synthetic fixture from real firmware encoder, original ASCII CRC retained',
        'results':results,'limits':'Same global WorkBudget; only actual BLE source bodies differ. Synthetic no-radio SDK acceptances and idleAge2, not HIL throughput.'}
if args.output_prefix:
    json_path=Path(str(args.output_prefix)+'.json');text_path=Path(str(args.output_prefix)+'.txt')
    assert not json_path.exists() and not text_path.exists(),'refuse evidence overwrite'
    with json_path.open('x') as out:json.dump(report,out,indent=2);out.write('\n')
    with text_path.open('x') as out:
        for label,result in results.items():out.write(label+'\n'+result['output']+'exit_code='+str(result['run_exit_code'])+'\n')
print(json.dumps({'status':report['status'],'before':results['before']['output'].strip(),'after':results['after']['output'].strip()}))
