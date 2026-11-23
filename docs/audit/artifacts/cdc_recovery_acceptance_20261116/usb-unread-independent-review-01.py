import copy, hashlib, importlib.util, json, subprocess, sys, tempfile
from datetime import datetime, timezone
from pathlib import Path
root=Path('/home/csc/esp/workspace/SensorArray')
stage=root/'docs/audit/artifacts/cdc_recovery_acceptance_20261116'
sys.path.insert(0,str(stage/'scripts'))
import analyze_usb_unread as a
source=Path(a.__file__)
actual=stage/'usb-unread-01/usb-unread-01-hold.summary.json'
s=a.load(actual); raw=a.read_bytes(a.resolve(s['raw_file'],actual)); events=[json.loads(l) for l in a.read_bytes(a.resolve(s['events_file'],actual)).splitlines()]
input_paths=[actual,a.resolve(s['raw_file'],actual),a.resolve(s['events_file'],actual),stage/'ble-snapshot-postunread-01.summary.json',source,stage/'scripts/test_analyze_usb_unread.py']
before={str(p):a.sha(p) for p in input_paths}
report={'created_utc':datetime.now(timezone.utc).isoformat(),'source_sha256':a.sha(source),'test_sha256':a.sha(stage/'scripts/test_analyze_usb_unread.py'),'hardware_actions':[],'original_recordings_modified':False,'probes':[]}
def probe(name,fn):
 try:
  result=fn(); report['probes'].append({'name':name,'result':'ACCEPTED','detail':result})
 except Exception as e:
  report['probes'].append({'name':name,'result':'REJECTED','exception':type(e).__name__+':'+str(e)})
probe('baseline_validate_hold',lambda:a.validate_hold(s,events,raw)[0]['mode'])
probe('missing_final_finished_event',lambda:a.validate_hold(s,events[:-1],raw)[0]['mode'])
e=copy.deepcopy(events);e[-1].update(status='ERROR',error='late_cleanup_fault')
probe('final_finished_disagrees_with_summary',lambda:a.validate_hold(s,e,raw)[0]['mode'])
x=copy.deepcopy(s);x.update(error='late_cleanup_fault',cleanup_error='timeout',safe_cap_recovery='FAILED_OR_UNKNOWN',cleanup_CONFIG=None,fault_events=1,reset_event_count=17,frame_boot_change_events=1)
probe('summary_faults_unknown_cleanup_still_labeled_success',lambda:a.validate_hold(x,events,raw)[0]['mode'])
e=copy.deepcopy(events);i=next(i for i,v in enumerate(e) if v['type']=='unread_hold_start');t=e[i]['t']
e[i+1:i+1]=[{'t':t+.1,'type':'tx_begin','phase':'unread_hold','command':'CONFIG?','wire_ascii':'\nCONFIG?\n','channel':'data'}, {'t':t+.2,'type':'tx_complete','phase':'unread_hold','command':'CONFIG?','accepted_bytes':9,'acceptance':'OS_tty_write_not_firmware_command_applied_or_PC_receipt'}]
probe('real_tx_event_names_inside_zero_IO_hold',lambda:a.validate_hold(s,e,raw)[0]['mode'])
args=[actual,stage/'usb-unread-01/usb-unread-01-ble.ble.notifications.jsonl',stage/'usb-unread-01/usb-unread-01-debug.debug.summary.json',stage/'ble-snapshot-postunread-01.summary.json',stage/'ble-snapshot-04-validated-identity.json']
def baseline():
 r=a.analyze(*args);return {'status':r['status'],'BLE_endpoint_gap':r['BLE_endpoint_gap'],'USB_PC_FPS':r['USB_PC_FPS'],'hold_seconds':r['hold']['actual_seconds'],'max_PC_complete_gap':r['ble']['configuration_groups'][0]['host_complete_frame_intervals']['max_seconds']}
probe('full_original_analysis_reexecution',baseline)
with tempfile.TemporaryDirectory(prefix='sensorarray-unread-review-') as temp:
 post=a.load(args[3]);post.update(status='ERROR',raw_sha256='0'*64,rx_bytes=0,faults=['MFAULT,synthetic=1'])
 for attempt in post['attempts']:
  attempt['monotonic']=1.0
  if attempt['command']=='MEM?' and 'fields' in attempt:attempt['fields']['intFree']='424242'
 p=Path(temp)/'post-mutated.summary.json';p.write_text(json.dumps(post))
 def bad_post():
  r=a.analyze(*args[:3],p,args[4]);return {'status':r['status'],'collector_recovery':r['post_hold']['collector_recovery'],'intFree':r['post_hold']['MEM']['intFree']}
 probe('unbound_post_summary_error_bad_hash_old_time_and_fabricated_MEM',bad_post)
report['input_hashes_before']=before
report['input_hashes_after']={str(p):a.sha(p) for p in input_paths}
report['all_inputs_unchanged']=report['input_hashes_before']==report['input_hashes_after']
print(json.dumps(report,indent=2))
