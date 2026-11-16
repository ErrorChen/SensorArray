from pathlib import Path
import json,hashlib,datetime,subprocess,base64,collections,statistics,zlib,re
ROOT=Path('/home/csc/esp/workspace/SensorArray')
BASE=ROOT/'docs/audit/artifacts/cdc_recovery_acceptance_20261116'
SDK=Path('/home/csc/esp/esp-idf-v5.5.5')

def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''): h.update(b)
 return h.hexdigest()
def ref(p): return {'path':str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),'bytes':p.stat().st_size,'sha256':sha(p)}
def percentile(a,p):
 a=sorted(a)
 if not a:return None
 x=(len(a)-1)*p;i=int(x);return a[i]+(a[min(i+1,len(a)-1)]-a[i])*(x-i)
def summary(a):
 return {'n':len(a),'min':min(a) if a else None,'p50':percentile(a,.5),'p90':percentile(a,.9),'p99':percentile(a,.99),'max':max(a) if a else None}
def trip(s):return [int(x) for x in s.split('/')]
def delta(a,b,bits=32):return (int(b)-int(a))%(1<<bits)

def querywindow(s,tag):
 a,b=s['snapshots']['before'][tag],s['snapshots']['after'][tag]
 return {'before':a,'after':b,'receipt_elapsed_seconds':b['monotonic']-a['monotonic'],
         'scope':'two query replies; not exact capture window. Each query has its own bracket.'}
def debug_window(n,start,end):
 rawp=BASE/n/(n+'-debug.debug.rx.bin');eventp=BASE/n/(n+'-debug.debug.events.jsonl')
 raw=rawp.read_bytes();buf=b'';buf_start=0;count=collections.Counter();sizes=collections.Counter(); records=collections.defaultdict(list)
 for line in eventp.open():
  e=json.loads(line)
  if e['type']!='rx':continue
  chunk=raw[e['offset']:e['offset']+e['length']]
  if not buf:buf_start=e['offset']
  buf+=chunk
  while b'\n' in buf:
   text,buf=buf.split(b'\n',1);length=len(text)+1;off=buf_start;buf_start+=length
   if start <= e['monotonic'] <= end:
    tag=text.split(b',',1)[0].decode('ascii','replace');count[tag]+=1;sizes[tag]+=length
    if tag in ('BL50','STK','TXDROP'):
     fields=dict(x.split('=',1) for x in text.decode('ascii','replace').split(',')[1:] if '=' in x)
     records[tag].append({'monotonic':e['monotonic'],'offset':off,'fields':fields})
 return {'receipt_window':[start,end],'line_time_semantics':'time of receive chunk containing final LF; may include queued device logs',
         'complete_lines':sum(count.values()),'complete_line_bytes':sum(sizes.values()),
         'tags':{k:{'count':count[k],'bytes':sizes[k]} for k in count},
         'STK':{'first':records['STK'][0] if records['STK'] else None,'last':records['STK'][-1] if records['STK'] else None},
         'TXDROP':{'count':len(records['TXDROP']),'reasons':dict(collections.Counter(r['fields'].get('reason','unknown') for r in records['TXDROP']))}}

def journal_window(n,start,end):
 p=BASE/n/(n+'-ble.ble.notifications.jsonl');times=collections.defaultdict(list);counts=collections.Counter();size=collections.Counter();lens=collections.defaultdict(list);messages={};complete=[];bad=0
 for line in p.open():
  e=json.loads(line)
  if e['type']!='notification':continue
  t=e['monotonic_ns']/1e9;c=e['channel'];payload=base64.b64decode(e['payload_base64'])
  assert len(payload)==e['length']
  if start<=t<=end:
   times[c].append(t);counts[c]+=1;size[c]+=len(payload);lens[c].append(len(payload))
  if not payload.startswith(b'G,'):continue
  head,chunk=payload.split(b'\n',1);v=head.decode('ascii').split(',')
  assert len(v)==8 and int(v[5])==len(chunk)
  key=(c,int(v[2]));idx=int(v[3]);count=int(v[4]);total=int(v[6]);crc=int(v[7],16)
  m=messages.setdefault(key,{'start':t,'end':t,'parts':{},'wire':0,'count':count,'total':total,'crc':crc,'channel':c})
  assert count==m['count'] and total==m['total'] and crc==m['crc'] and idx not in m['parts']
  m['end']=t;m['parts'][idx]=chunk;m['wire']+=len(payload)
  if len(m['parts'])==count:
   data=b''.join(m['parts'][i] for i in range(count));assert len(data)==total
   good=(zlib.crc32(data)&0xffffffff)==crc
   if start<=m['start'] and m['end']<=end:
    bad+=not good
    complete.append({'channel':c,'fragments':count,'bytes':total,'notification_bytes':m['wire'],'latency_ms':1000*(m['end']-m['start']),'tag':data.split(b',',1)[0].decode('ascii','replace')})
   del messages[key]
 duration=end-start
 out={'scope':'exact summary capture monotonic window; notification callback timestamps, not RF event timing','duration_s':duration,'fragment_crc_failures':bad,'channels':{}}
 for c in sorted(counts):
  gaps=[1000*(b-a) for a,b in zip(times[c],times[c][1:])]
  out['channels'][c]={'notifications':counts[c],'bytes':size[c],'notifications_per_s':counts[c]/duration,'bytes_per_s':size[c]/duration,'notification_length':summary(lens[c]),'inter_notification_gap_ms':summary(gaps)}
  msgs=[m for m in complete if m['channel']==c]
  out['channels'][c]['complete_messages']={'n':len(msgs),'tags':dict(collections.Counter(m['tag'] for m in msgs)),
   'fragment_counts':dict(collections.Counter(m['fragments'] for m in msgs)),'payload_bytes':summary([m['bytes'] for m in msgs]),
   'notification_bytes':summary([m['notification_bytes'] for m in msgs]),'first_to_last_fragment_ms':summary([m['latency_ms'] for m in msgs])}
 return out

result={'schema_version':1,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
 'review_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
 'scope':'Read-only comparison of closed screen04/screen05. Scientific SPI path and source workload changed; not an isolated BLE or scheduler experiment.',
 'analysis_script':ref(Path(__file__)),'screens':{}}
for n in ('screen-04','screen-05'):
 p=BASE/n/(n+'-volt1.summary.json');s=json.loads(p.read_text());ap=BASE/n/(n+'-analysis-01.json');a=json.loads(ap.read_text())['captures'][0]
 assert s['status']=='CAPTURED' and s['capture_duration_satisfied'] and not s['firmware_faults'] and not s['reset_observations']
 start,end=s['capture_monotonic_start'],s['capture_monotonic_end']
 refs=[p,ap,BASE/n/(n+'-ble.ble.summary.json'),BASE/n/(n+'-ble.ble.notifications.jsonl'),BASE/n/(n+'-debug.debug.summary.json'),BASE/n/(n+'-debug.debug.rx.bin'),BASE/n/(n+'-debug.debug.events.jsonl'),BASE/n/(n+'-volt1.rx.bin')]
 d={'evidence':[ref(x) for x in refs],'actual_BUILD':a['provenance_queries']['BUILD']['first']['fields'],
    'capture_window':{'start':start,'end':end,'seconds':end-start},
    'usb_rates':a['configuration_groups'][0]['rates'],'usb_counts':a['capture_counts'],
    'ble_rates':a['ble']['configuration_groups'][0]['rates'],'ble_counts':a['ble']['capture_frame_counts'],
    'ble_fragment_counts':a['ble']['fragments_by_channel'],'cross_transport':a['ble']['USB_comparison'],
    'BLELINK':{'before':s['snapshots']['before']['BLELINK?']['fields'],'after':s['snapshots']['after']['BLELINK?']['fields']},
    'SPI':{'before':s['snapshots']['before']['SPI?']['fields'],'after':s['snapshots']['after']['SPI?']['fields']},
    'ble_pool_window':a['otg_diagnostics']['BLE_firmware'], 'adc_crc_window_context_not_radio_causality':a['otg_diagnostics']['adc_crc']['health'],
    'query_windows':{},'independent_notifications':journal_window(n,start,end),'independent_debug':debug_window(n,start,end)}
 for tag in ('BLESVC?','SCHED?','SCHEDWHY?','BLETX?','PERF?','USBQ?'):
  w=querywindow(s,tag);bf,af=w['before']['fields'],w['after']['fields'];fields={}
  if tag=='BLESVC?':
   for k in ('started','completed','wallUs','idleChanged','idleSame','idleUnknown'):fields[k]=delta(bf[k],af[k],64 if k=='wallUs' else 32)
   w['counter_delta']=fields; w['mean_call_wall_us']=fields['wallUs']/fields['completed'];w['sdk_wall_fraction']=fields['wallUs']/1e6/w['receipt_elapsed_seconds'];w['completed_calls_per_s']=fields['completed']/w['receipt_elapsed_seconds']
   w['aggregate_elapsed_per_completed_call_us']=1e6*w['receipt_elapsed_seconds']/fields['completed'];w['outside_observed_sdk_intervals_per_completed_call_us']=w['aggregate_elapsed_per_completed_call_us']-w['mean_call_wall_us'];w['sdk_active_endpoints']=[bf['active'],af['active']]
   w['interpretation']='Aggregate SDK invocation wall time. Not task CPU time, proof of blocking, or PC/airtime completion. Possible boundary-active call makes exact containment approximate; maximum call durations retained separately.'
  elif tag in ('SCHED?','SCHEDWHY?'):
   for k in ('bt','tr','log','ser','bc','sc','wc'):
    b0,a0=trip(bf[k]),trip(af[k]); fields[k]=[delta(x,y) for x,y in zip(b0,a0)] if tag=='SCHEDWHY?' else {'checkpoints':delta(b0[0],a0[0]),'yields':delta(b0[1],a0[1]),'maxBurstUnits_before':b0[2],'maxBurstUnits_after':a0[2]}
   w['workers']=fields;w['idleGeneration_delta_skip_zero']=(int(af['idleGen'])-int(bf['idleGen']))%0xffffffff;w['init_before_after']=[bf.get('initError',bf.get('init')),af.get('initError',af.get('init'))]
   if tag=='SCHED?': w['maxIdleGapTicks_endpoints']=[int(bf['maxIdleGapTicks']),int(af['maxIdleGapTicks'])]
   else:w['reason_order']='quantum/backstop/units; reasons can overlap at one yield; do not add or exactly join to separate SCHED query window'
  elif tag=='BLETX?':
   w['counter_delta']={k:delta(bf[k],af[k]) for k in ('dDequeued','lDequeued','cDequeued','sdkCalls','sdkAccepted','sdkErrors')};w['maxima_endpoints']={k:[int(bf[k]),int(af[k])] for k in ('dMaxAgeUs','lMaxAgeUs','cMaxAgeUs','sdkMaxUs')}
  elif tag=='PERF?':w['counter_delta']={k:delta(bf[k],af[k]) for k in ('pub','fresh','stale','dropOut','dropEvent','usbSent','usbDrop','queueDrop')}
  else:w['counter_delta']={k:delta(bf[k],af[k]) for k in ('dDrop','cDrop','dSubmit','cSubmit','expired','cExpired','partial')}
  d['query_windows'][tag]=w
 result['screens'][n]=d

s4,s5=[result['screens'][n] for n in ('screen-04','screen-05')]
q4,q5=[s['query_windows']['BLESVC?'] for s in (s4,s5)]
for s in (s4,s5):
 w=s['query_windows']['BLESVC?'];d=w['counter_delta']
 assert d['idleChanged']+d['idleSame']+d['idleUnknown']==d['completed']
 s['sdk_idle_changed_fraction']=d['idleChanged']/d['completed']
 s['capture_complete_frame_rate_usb']=s['usb_counts']['complete_frames']/s['capture_window']['seconds']
 s['capture_complete_frame_rate_ble']=s['ble_counts']['complete_frames']/s['capture_window']['seconds']
 s['new_conf_observations']='NOT_AVAILABLE_IN_MEASURED_FIRMWARE; fe/sdkErrors retain old meanings'
 s['per_channel_service']={}
 for ch,c in s['independent_notifications']['channels'].items():
  s['per_channel_service'][ch]={'notifications':c['notifications'],'notifications_per_s':c['notifications_per_s'],'bytes_per_s':c['bytes_per_s'],'fragments_per_message':c['complete_messages']['fragment_counts'],'frame_bytes':c['complete_messages']['payload_bytes'],'inter_notification_ms':c['inter_notification_gap_ms']}
D5=s5['independent_notifications']['channels']['D'];L5=s5['independent_notifications']['channels']['L']
result['comparison']={
 'capture_fps_screen04':s4['usb_rates']['capture_fps'],'capture_fps_screen05':s5['usb_rates']['capture_fps'],
 'ble_exact120window_fps_screen04':s4['capture_complete_frame_rate_ble'],'ble_exact120window_fps_screen05':s5['capture_complete_frame_rate_ble'],
 'usb_exact120window_fps_screen04':s4['capture_complete_frame_rate_usb'],'usb_exact120window_fps_screen05':s5['capture_complete_frame_rate_usb'],
 'sdk_mean_wall_us_screen04':q4['mean_call_wall_us'],'sdk_mean_wall_us_screen05':q5['mean_call_wall_us'],
 'sdk_wall_fraction_screen04':q4['sdk_wall_fraction'],'sdk_wall_fraction_screen05':q5['sdk_wall_fraction'],
 'completed_calls_per_s_screen04':q4['completed_calls_per_s'],'completed_calls_per_s_screen05':q5['completed_calls_per_s'],
 'sdk_idle_changed_fraction_screen04':s4['sdk_idle_changed_fraction'],'sdk_idle_changed_fraction_screen05':s5['sdk_idle_changed_fraction'],
 'outside_sdk_bracket_us_per_call_screen04':q4['outside_observed_sdk_intervals_per_completed_call_us'],'outside_sdk_bracket_us_per_call_screen05':q5['outside_observed_sdk_intervals_per_completed_call_us'],
 'screen05_observed_DplusL_notifications_per_s':D5['notifications_per_s']+L5['notifications_per_s'],
 'screen05_required_DplusL_notifications_per_s_at_capture':3*s5['usb_rates']['capture_fps']+L5['notifications_per_s'],
 'screen05_LOG_notification_fraction':L5['notifications']/(D5['notifications']+L5['notifications']),
 'arithmetic_only_no_LOG_science_fps_at_same_total_notify_rate':(D5['notifications_per_s']+L5['notifications_per_s'])/3,
 'arithmetic_limit':'Not a proposal to remove LOG; not RF capacity; assumes unchanged total notify service and all science frames use3D notifications.',
 'causality_limit':'Different image/boot and scientific SPI conversion path; ADC retries/logs/source FPS changed. No task CPU or HCI credit series. SDK wall intervals include possible blocking/preemption and cannot be CPU utilization.'}
result['observations']=[
 'screen05 preserves negotiated7.5ms/2MPHY/MTU247. Confirmed short interval alone has not met the capture rate.',
 'Both screens science payloads use three DATA notifications. Host G-envelope CRC replay and original parser results are retained.',
 'Large losses remain at bounded BLE admission/pool and Transport queue layers; each counter window has its own timestamps and must not be summed as an exact120s loss total.',
 'screen05 removes observed runtime ADC CRC failures and has no USB sequence gap, but BLE is still substantially below source rate. This narrows the problem without identifying radio versus SDK/scheduler causality.',
 'Legacy fe=0/sdkErrors=0 excludes observed immediate API/fragment errors only. The now-added generic CONF counters were not present during either capture and cannot retroactively exclude later local SDK rejections.',
 'Generic CONF counters are observation only. No task priorities, queues, budgets, retry behavior or SDK source changed in that patch.'
]
result['limits']=['No hardware/build actions in this analysis.','Neither query windows nor debug reception windows are exact per-frame event time.','No inference of CPU time from SDK wall duration.','No link-epoch attribution of unavailable asynchronous CONF events.','No measured BLE improvement is claimed for the unflashed observation patch.']
out=BASE/'screen-05/screen-05-ble-service-review-01.json'
with out.open('x') as f:json.dump(result,f,indent=2,ensure_ascii=False);f.write('\n')
print(out.relative_to(ROOT));print(json.dumps(result['comparison'],indent=2))
for n,s in result['screens'].items():
 print(n,'BLE_pool',s['ble_pool_window'])
 print(n,'BLETX',s['query_windows']['BLETX?']['counter_delta'])
 print(n,'PERF',s['query_windows']['PERF?']['counter_delta'])
 print(n,'SCHED',s['query_windows']['SCHED?']['workers']['bt'])
 print(n,'SCHEDWHY',s['query_windows']['SCHEDWHY?']['workers']['bt'])
 print(n,'SDK',s['query_windows']['BLESVC?']['counter_delta'])
