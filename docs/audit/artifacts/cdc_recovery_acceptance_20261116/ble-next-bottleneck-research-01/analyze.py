"""Read only closed evidence. Python shape replay is not production C/HIL."""
import base64,collections,datetime,hashlib,json,math,statistics,subprocess,zlib
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'components/sensorarrayBle').is_dir())
STAGE=ROOT/'docs/audit/artifacts/cdc_recovery_acceptance_20261116'
OUT=Path(__file__).resolve().parent
IDF=Path('/home/csc/esp/esp-idf-v5.5.5')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def header(mid,index,count,chunk,total,crc):return f'G,D,{mid},{index},{count},{chunk},{total},{crc:08X}\n'.encode()
def header_len(mid,index,count,chunk,total):return 18+sum(len(str(x)) for x in (mid,index,count,chunk,total))
def plan(total,maximum,mid,crc):
 trials=0
 for count in range(1,total+1):
  base,extra=divmod(total,count)
  if not base:return None,trials
  lengths=[]
  for index in range(count):
   chunk=base+(index<extra);trials+=1;h=header(mid,index,count,chunk,total,crc)
   assert len(h)==header_len(mid,index,count,chunk,total)
   if len(h)+chunk>maximum:break
   lengths.append(len(h)+chunk)
  else:return lengths,trials
 return None,trials
reports=[]
for n in (7,8,9,10):
 label=f'screen-{n:02d}';d=STAGE/label;p=d/f'{label}-volt1.summary.json'
 summary=json.loads(p.read_text());analysis=json.loads((d/f'{label}-analysis-01.json').read_text());capture=analysis['captures'][0]
 a,b=summary['snapshots']['before'],summary['snapshots']['after'];x,y=a['BLESVC?']['fields'],b['BLESVC?']['fields']
 calls=int(y['completed'])-int(x['completed']);wall=int(y['wallUs'])-int(x['wallUs']);dt=b['BLESVC?']['monotonic']-a['BLESVC?']['monotonic']
 cpu={}
 for role in (0,1,2,3):
  f,g=a[f'CPUOWNER?{role}']['fields'],b[f'CPUOWNER?{role}']['fields']
  assert f['session']==g['session'] and f['taskNumber']==g['taskNumber'] and f['stale']==g['stale']=='0'
  dr=int(g['runUs'])-int(f['runUs']);elapsed=int(g['t1Us'])-int(f['t1Us']);cpu[str(role)]={'own_run_us':dr,'sample_window_us':elapsed,'scheduled_share_percent':100*dr/elapsed}
 row={'label':label,'capture_window_seconds':summary['capture_seconds'],'usb_complete_fps':summary['host_complete_frame_fps'],
  'ble_complete_frames':capture['ble']['capture_frame_counts']['trusted_complete_frames'],
  'ble_complete_fps':capture['ble']['capture_frame_counts']['trusted_complete_frames']/summary['capture_seconds'],
  'sdk_query_window_seconds':dt,'sdk_completed_delta':calls,'sdk_wall_us_delta':wall,'sdk_mean_wall_us':wall/calls,
  'sdk_wall_is_not_cpu_or_blocking_proof':True,'conf_error_delta':int(y['confError'])-int(x['confError']),
  'conf_match_semantics':'generic local SDK status; unmatched epoch unknown; not PC ACK',
  'actual_link_before':a['BLELINK?']['fields'],'actual_link_after':b['BLELINK?']['fields'],
  'cpu_separate_sample_windows':cpu,'bl_diagnostic_window':capture['otg_diagnostics']['BLE_firmware'],
  'summary_sha256':sha(p),'analysis_sha256':sha(d/f'{label}-analysis-01.json')}
 if n in (9,10):
  bs=json.loads((d/f'{label}-ble.ble.summary.json').read_text());journal=ROOT/bs['journal_file'];assert bs['recording_end_monotonic_ns'] and sha(journal)==bs['journal_sha256']
  start,end=summary['capture_monotonic_start'],summary['capture_monotonic_end'];pending={};complete=[];sizes=collections.Counter();headers=collections.Counter();times=[];notifications=collections.Counter()
  for line in journal.open():
   event=json.loads(line);t=event.get('monotonic_ns',0)/1e9
   if event.get('type')!='notification' or not start<=t<=end:continue
   payload=base64.b64decode(event['payload_base64'],validate=True);assert len(payload)==event['length'];notifications[event['channel']]+=1;times.append(t)
   if event['channel']!='D' or not payload.startswith(b'G,D,'):continue
   raw,body=payload.split(b'\n',1);fields=raw.split(b',');mid,index,count,chunk,total=map(int,fields[2:7]);crc=int(fields[7],16)
   assert len(body)==chunk and len(raw)+1==header_len(mid,index,count,chunk,total)
   sizes[len(payload)]+=1;headers[len(raw)+1]+=1
   item=pending.setdefault(mid,{'count':count,'total':total,'crc':crc,'parts':{}});assert (count,total,crc)==(item['count'],item['total'],item['crc']);assert index not in item['parts'];item['parts'][index]=body
   if len(item['parts'])==count:
    data=b''.join(item['parts'][i] for i in range(count));assert len(data)==total and zlib.crc32(data)==crc
    old,trials=plan(total,244,mid,crc);large,newtrials=plan(total,512,mid,crc);assert len(old)==count
    complete.append({'length':total,'count':count,'old_lengths':old,'mtu517_balanced_lengths':large,'planner_trial_snprintf_count':trials,
        'old_ideal_pdu_min':sum(math.ceil((n+7)/251) for n in old),'mtu517_ideal_pdu_min':sum(math.ceil((n+7)/251) for n in large)})
    del pending[mid]
  gaps=sorted((b-a)*1000 for a,b in zip(times,times[1:]));row['notification_geometry']={
   'journal_sha256':bs['journal_sha256'],'notifications_by_channel':dict(notifications),'complete_G_data_messages':len(complete),'boundary_partial_messages':len(pending),
   'message_lengths':dict(collections.Counter(x['length'] for x in complete)), 'notification_value_lengths':dict(sizes),'G_header_lengths':dict(headers),
   'planner_trial_snprintf_total':sum(x['planner_trial_snprintf_count'] for x in complete),'observed_G_fragment_count':dict(collections.Counter(x['count'] for x in complete)),
   'mtu517_balanced_hypothetical_notify_count':sum(len(x['mtu517_balanced_lengths']) for x in complete),
   'observed_notify_count_complete_messages':sum(x['count'] for x in complete),
   'ideal_LL_PDU_min_251_old':sum(x['old_ideal_pdu_min'] for x in complete),'ideal_LL_PDU_min_251_mtu517_balanced':sum(x['mtu517_ideal_pdu_min'] for x in complete),
   'ideal_PDU_scope':'Mathematical lower bound with ATT3+L2CAP4 and operative251-byte LL payload; not measured air/HCI completions. DLE callback lacks peer identity.',
   'host_callback_gap_ms_percentiles':{str(q):gaps[round((len(gaps)-1)*q/100)] for q in (50,75,90,99)},
   'shape_replay':'Python translation of current balanced planner, validated against every observed complete G count/header/body length. Not nativeC equivalence or a firmware change.'}
 reports.append(row)
paths=[ROOT/'components/sensorarrayBle/sensorarrayBle.c',ROOT/'main/output/sensorarrayTextProtocol.c',ROOT/'core/transport/sensorarrayTransport.c',ROOT/'tools/text_protocol.py',
 IDF/'components/bt/host/bluedroid/api/esp_gatts_api.c',IDF/'components/bt/host/bluedroid/api/esp_gatt_common_api.c',IDF/'components/bt/common/osi/thread.c',
 IDF/'components/bt/common/btc/core/btc_task.c',IDF/'components/bt/host/bluedroid/bta/gatt/bta_gatts_api.c',IDF/'components/bt/host/bluedroid/bta/gatt/bta_gatts_act.c',
 IDF/'components/bt/host/bluedroid/stack/l2cap/l2c_api.c',IDF/'components/bt/host/bluedroid/stack/l2cap/l2c_link.c',IDF/'components/esp_rom/include/esp_rom_crc.h',IDF/'components/esp_rom/patches/esp_rom_crc.c']
result={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'scope':'Read-only closed evidence through screen10; no production/SDK edits or target build/HIL',
 'head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'idf_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=IDF,text=True).strip(),
 'evidence':reports,'source_sha256':{str(p):sha(p) for p in paths},
 'findings':[
  {'classification':'PROVEN_SOURCE_AND_HIL','finding':'598 bytes require at least3 notifications under MTU247 (ATT value max244), even without G overhead. Current3 fragments are not redundant.'},
  {'classification':'PROVEN_SOURCE','finding':'FAST sends need_confirm=false and never awaits CONF. There is no extra set_attr_value or unconditional tick sleep in the success path. One public API enqueue per fragment.'},
  {'classification':'PROVEN_SOURCE_WITH_UNMEASURED_ATTRIBUTION','finding':'Public API allocates/copies, increments BTC count under an unbounded binary-semaphore wait, posts BTC queue0 (default100) with portMAX_DELAY. BTC19 posts BTU20; local CONF returns to BTC. Higher-priority drain and same-priority rescheduling can all occur before API return.4ms wall is not4ms CPU or an ACK wait.'},
  {'classification':'PROVEN_SOURCE','finding':'L2CAP drains while controller_le_xmit_window>0 and sent_not_acked<link quota; HCI completed-packet events replenish window. It is not fixed one-notification-per-connection-event application serialization.'},
  {'classification':'UNKNOWN','finding':'No current capture records actual controller-credit window or HCI completion timing. cg0 and confError0 do not establish unsaturated radio, and host callback spacing is not over-air timing.'},
  {'classification':'PROVEN_SOURCE_CANDIDATE','finding':'Replace bitwise CRC computation with esp_rom_crc32_le(0,...) on target, retaining scientificCRC and both enqueue/dequeue checks and all error branches. SDK header/ROM mapping define same reflectedIEEE polynomial/seed/finalization. Keep independent portablebitwise host oracle; no benefit magnitude yet.'},
  {'classification':'PROVEN_SOURCE_CANDIDATE','finding':'G header length is18 plus decimal widths(mid,index,count,chunk,total). Replace only planner trialsnprintf with this calculation; keep actualwire formatter, all bounds and identical balanced chunks. Three-fragment records currently incur5 trial plus3 wire snprintf calls. NativeC boundary equivalence still required.'},
  {'classification':'LEGAL_EXPERIMENT_NOT_FIX','finding':'PreferredMTU517 is supported by officialIDF throughput examples and current512 scratch/attributes. Actual peer negotiation must be observed; fallback247 remains valid. With current balanced2-fragment plan, operativeDLE251 needs ideal4 LL PDUs instead of3. FewerSDK calls can trade against morecontroller/air segments; no byte-only throughput claim.'},
  {'classification':'OBSERVATION_CANDIDATE','finding':'esp_ble_get_sendable_packets_num() reads global controllerLEwindow without adding a mutex or enqueue, documentedonly BLErunningcore/singleconnection. SampleonTXowner boundary and count zero/min/max, taggedunknown if linkcondition fails. Do not gate sends on it. Perconnectionget_cur_sendable adds portMAX_DELAY semaphore and is not a free/nonblocking hotpath probe.'},
  {'classification':'OUT_OF_SCOPE','finding':'Successful genericCONF still makes two payloadcopies subsequently hidden by cb len0. SDK ownership/lifetime needs bothcopies today; do not remove them from application or bypasspublicAPI. SDK is read-only.'}
 ],
 'minimal_next_steps':['First strict-equivalence ROMCRC and planner arithmetic, keeping wire/CRC checks/budgets/queues/priorities/ADC SPI CRC8 unchanged; actualC differential and target HIL required.',
 'If still capacitylimited, independent MTU517-only120sVOLT1 USB+BLE trial on sameO2/160MHz/asserts: require actualMTU callback,2M/interval6 evidence; retainoriginalbalancedplanner and controltests for a clean onevariable comparison.',
 'Compare completeframes/seq gaps/identity/CRC and separate querywindow SDKwall/calls/ownCPU, not only notification count. Ifpeer remains247, classify largerMTU conditionNOT_RUN.',
 'Do not implement finitepriorityboost; prior research found no gain. Do not remove safety/logs, expandqueues or imposecapture limits. Outer deadlines cannot cancelanSDK call.']}
with (OUT/'report.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps({'report':str(OUT/'report.json'),'screens':[x['label'] for x in reports],'screen10_sdk_mean_wall_us':reports[-1]['sdk_mean_wall_us'],'geometry':reports[-1]['notification_geometry']},sort_keys=True))
