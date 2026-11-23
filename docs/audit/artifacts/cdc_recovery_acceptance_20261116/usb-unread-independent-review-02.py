#!/usr/bin/env python3
"""Independent read-only replay. Mutations are synthetic in-memory only."""
import copy, datetime, hashlib, importlib.util, json, subprocess, sys, time
from pathlib import Path
root=Path(__file__).resolve().parents[4]
stage=Path(__file__).resolve().parent
sys.path.insert(0,str(stage/'scripts'))
import analyze_usb_unread as a
actual=stage/'usb-unread-01/usb-unread-01-hold.summary.json'
summary=a.load(actual)
raw_path=a.resolve(summary['raw_file'],actual)
events_path=a.resolve(summary['events_file'],actual)
raw=a.read_bytes(raw_path)
events=[json.loads(line) for line in a.read_bytes(events_path).splitlines()]
post_path=stage/'ble-snapshot-postunread-01.summary.json'
post_raw_path=a.resolve('ble-snapshot-postunread-01.rx.bin',post_path)
post_events_path=a.resolve('ble-snapshot-postunread-01.events.jsonl',post_path)
post=a.load(post_path); post_raw=a.read_bytes(post_raw_path)
post_events=[json.loads(line) for line in a.read_bytes(post_events_path).splitlines()]
input_paths=[actual,raw_path,events_path,post_path,post_raw_path,post_events_path,Path(a.__file__),stage/'scripts/test_analyze_usb_unread.py',stage/'usb-unread-01/usb-unread-analysis-01.json',stage/'usb-unread-01/usb-unread-analysis-02.json']
before={str(p):a.sha(p) for p in input_paths}
report={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':a.sha(Path(a.__file__)),'hardware_actions':[],'production_modified':False,'commands':[],'probes':[]}
def command(args):
 t=time.monotonic();r=subprocess.run(args,cwd=root,text=True,capture_output=True,timeout=30)
 rec={'command':args,'exit_code':r.returncode,'elapsed_seconds':time.monotonic()-t,'stdout':r.stdout,'stderr':r.stderr};report['commands'].append(rec)
 assert r.returncode==0, rec
 return r
command(['python3',str(stage/'scripts/test_analyze_usb_unread.py')])
old=json.loads(command(['python3',str(stage/'usb-unread-independent-review-01.py')]).stdout)
for p in old['probes']:
 expected='ACCEPTED' if p['name'] in {'baseline_validate_hold','full_original_analysis_reexecution'} else 'REJECTED'
 assert p['result']==expected,p
report['original_negative_replay']=old
config,build=a.validate_hold(summary,events,raw)
def rejected(name,fn):
 try:fn()
 except ValueError as e:report['probes'].append({'name':name,'result':'REJECTED','error':str(e)})
 else:raise AssertionError(name+' accepted')
def validate_post(value):return a.validate_post_probe(value,post_events,post_raw,summary['hold']['finished_monotonic'],config,build)
for name,mutate in [
 ('post_error',lambda p:p.update(status='ERROR')),
 ('post_bad_hash',lambda p:p.update(raw_sha256='0'*64)),
 ('post_wrong_length',lambda p:p.update(rx_bytes=0)),
 ('post_old_monotonic',lambda p:p['attempts'][0].update(monotonic=1.0)),
 ('post_fabricated_MEM',lambda p:next(x for x in p['attempts'] if x['command']=='MEM?')['fields'].update(intFree='424242')),
 ('post_fault',lambda p:p.update(faults=['MFAULT,synthetic=1']))]:
 changed=copy.deepcopy(post);mutate(changed);rejected(name,lambda:validate_post(changed))
fields,closure=validate_post(post);assert fields['CONFIG?']['rows']=='8' and not closure['explicit_final_finished_event']
report['real_post_closure']=closure
records=[stage/n for n in ('usb-unread-01-hold-command.json','usb-unread-01-ble-command.json','usb-unread-01-debug-command.json','ble-snapshot-postunread-01-command.json')]
result=a.analyze(actual,stage/'usb-unread-01/usb-unread-01-ble.ble.notifications.jsonl',stage/'usb-unread-01/usb-unread-01-debug.debug.summary.json',post_path,stage/'ble-snapshot-04-validated-identity.json',records)
result=json.loads(json.dumps(result)) # Compare JSON artifacts to JSON-normalized live structures.
old_result=a.load(stage/'usb-unread-01/usb-unread-analysis-01.json');new_result=a.load(stage/'usb-unread-01/usb-unread-analysis-02.json')
for key in ('status','BLE_endpoint_gap','USB_PC_FPS','hold'):
 assert result[key]==old_result[key]==new_result[key],key
assert result['ble']['configuration_groups']==old_result['ble']['configuration_groups']==new_result['ble']['configuration_groups']
report['real_analysis']={'status':result['status'],'BLE_endpoint_gap':result['BLE_endpoint_gap'],'USB_PC_FPS':result['USB_PC_FPS'],'hold_seconds':result['hold']['actual_seconds'],'max_PC_complete_gap':result['ble']['configuration_groups'][0]['host_complete_frame_intervals']['max_seconds'],'old_and_new_numeric_fields_equal':True}
report['input_hashes_before']=before;report['input_hashes_after']={str(p):a.sha(p) for p in input_paths}
assert report['input_hashes_before']==report['input_hashes_after']
report['all_inputs_unchanged']=True
report['conclusion']='PASS_SCOPED: original final/cleanup, actual TX event and post-probe evidence-binding counterexamples rejected; 10 maintained tests and complete closed original analysis passed. No new HIL, firmware or RF claim.'
print(json.dumps(report,ensure_ascii=False,indent=2))
