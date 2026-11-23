#!/usr/bin/env python3
"""Offline association of closed BLE gap brackets / BA samples with query windows.
Actual analyzer CRC/identity/wire/order gates are reused unchanged. Missing frames
have no fabricated timestamp. Interval association is not causal attribution.
"""
import argparse,copy,gzip,hashlib,json,math,sys
from collections import Counter
from datetime import datetime,timezone
from pathlib import Path
ROOT=Path(__file__).resolve().parents[5]
STAGE=ROOT/'docs/audit/artifacts/cdc_recovery_acceptance_20261116'
sys.path.insert(0,str(ROOT/'tools'))
import analyze_firmware_benchmark as analyzer

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def actual(p):return p if p.exists() else p.with_name(p.name+'.gz')
def readbytes(p):return gzip.decompress(p.read_bytes()) if p.suffix=='.gz' else p.read_bytes()
def merged(intervals):
 result=[]
 for a,b in sorted(intervals):
  if result and a<=result[-1][1]:result[-1][1]=max(result[-1][1],b)
  else:result.append([a,b])
 return result

def classify(a,b,intervals):
 if not all(isinstance(x,(int,float)) and math.isfinite(x) for x in [a,b]) or b<a:return 'unknown'
 if any(a>=x and b<=y for x,y in intervals):return 'inside'
 if any(a<=y and b>=x for x,y in intervals):return 'boundary_uncertain'
 return 'outside'

def count_intervals(records,intervals,weight):
 counts=Counter({'inside':0,'boundary_uncertain':0,'outside':0,'unknown':0})
 for row in records:counts[classify(row['start'],row['end'],intervals)]+=row[weight]
 return dict(counts)

def test_classifier():
 w=merged([[1.,2.],[1.5,2.5],[4.,5.]])
 assert w==[[1.,2.5],[4.,5.]]
 assert classify(1.1,2.4,w)=='inside'
 assert classify(.9,1.1,w)=='boundary_uncertain'
 assert classify(2.4,4.1,w)=='boundary_uncertain'
 assert classify(2.6,3.9,w)=='outside'
 assert classify(None,1.,w)=='unknown'
 assert classify(2.,1.,w)=='unknown'
 assert count_intervals([{'start':0.,'end':.8,'n':2},{'start':.9,'end':1.1,'n':3}],w,'n')=={'inside':0,'boundary_uncertain':3,'outside':2,'unknown':0}

def debug_ba(s,summary,inputs):
 q=STAGE/f'screen-{s}';p=q/f'screen-{s}-debug.debug.summary.json';debug=json.loads(p.read_text())
 raw=actual(ROOT/debug['raw_file']);ep=actual(ROOT/debug['events_file']);b=readbytes(raw)
 assert hashlib.sha256(b).hexdigest()==debug['raw_sha256']
 for f in [p,raw,ep]:inputs[str(f.relative_to(ROOT))]=sha(f)
 start,end=summary['capture_monotonic_start'],summary['capture_monotonic_end'];buf=b'';samples=[]
 for text in readbytes(ep).decode().splitlines():
  event=json.loads(text)
  if event.get('type')!='rx':continue
  buf+=b[event['offset']:event['offset']+event['length']];lines=buf.split(b'\n');buf=lines.pop()
  for line in lines:
   if not(start<=event['monotonic']<=end and line.startswith(b'STK50,')):continue
   fields=dict(x.split('=',1) for x in line.decode().strip().split(',')[1:] if '=' in x)
   samples.append({'receipt':event['monotonic'],'seq':int(fields['seq']),'ba':int(fields['ba'])})
 intervals=[]
 for i,(a,b) in enumerate(zip(samples,samples[1:])):
  assert b['ba']>=a['ba']
  if b['ba']>a['ba']:intervals.append({'id':i,'start':a['receipt'],'end':b['receipt'],'ba_delta':b['ba']-a['ba'],'after_sample_seq':a['seq'],'before_sample_seq':b['seq']})
 return samples,intervals

def run(s,inputs):
 analysis_path=STAGE/f'screen-{s}/screen-{s}-analysis-01.json';summary_path=STAGE/f'screen-{s}/screen-{s}-volt1.summary.json'
 historical=json.loads(analysis_path.read_text());summary=json.loads(summary_path.read_text())
 assert historical['ble_journal']['recording_end_observed'] and historical['ble_journal']['schema_valid']
 journal=actual(ROOT/historical['ble_journal']['source']['path'])
 for p in [analysis_path,summary_path,journal]:inputs[str(p.relative_to(ROOT))]=sha(p)
 expected=historical['captures'][0]['ble']['capture_frame_counts'];captures=copy.deepcopy(historical['captures'])
 gaps=[];old_add=analyzer.Group.add
 def add(group,*args,**kwargs):
  count=group.counts['ordered_integrity_valid_frames'];sequence=group.last_sequence;host=group.ordered_host_previous;device_time=group.last
  value=old_add(group,*args,**kwargs)
  if group.counts['ordered_integrity_valid_frames']==count+1 and sequence is not None:
   distance=(group.last_sequence-sequence)&0xffffffff
   if distance>1:
    gaps.append({'id':len(gaps),'identity':group.identity,'start':host,'end':group.ordered_host_previous,'after_sequence':sequence,'before_sequence':group.last_sequence,'missing':distance-1,'after_device_timestamp_us':device_time,'before_device_timestamp_us':group.last})
  return value
 analyzer.Group.add=add;fp=analyzer.Fingerprints(':memory:')
 try:replay=analyzer.analyse_ble_journal(journal,captures,fp)
 finally:analyzer.Group.add=old_add;fp.close()
 actual_counts=captures[0]['ble']['capture_frame_counts']
 assert actual_counts==expected,(s,actual_counts,expected)
 assert sum(g['missing'] for g in gaps)==expected['missing_frames']
 assert replay['recording_end_observed'] and replay['schema_valid']
 windows=[{'id':i,'start':w['started_monotonic'],'end':w['finished_monotonic']} for i,w in enumerate(summary['diagnostic_windows']) if w['name']=='periodic']
 assert len(windows)==11 and all(w['end']>=w['start'] for w in windows)
 max_age=max(int(w['queries']['BLETX?']['fields']['dMaxAgeUs']) for w in summary['diagnostic_windows'] if 'BLETX?' in w['queries'])
 assert max_age<=68000
 samples,ba=debug_ba(s,summary,inputs)
 tests={}
 for label,tail in [('query_only',0.),('query_plus_observed_max_ble_queue_age',max_age/1e6),('query_plus_conservative_68ms',.068)]:
  intervals=merged([[w['start'],w['end']+tail] for w in windows]);gap_counts=count_intervals(gaps,intervals,'missing');ba_counts=count_intervals(ba,intervals,'ba_delta')
  tests[label]={'tail_seconds':tail,'windows':intervals,'total_window_seconds':sum(b-a for a,b in intervals),'BLE_missing_by_bracket':gap_counts,'BA_delta_by_sample_bracket':ba_counts,'maximum_BLE_missing_associated':gap_counts['inside']+gap_counts['boundary_uncertain'],'minimum_BLE_missing_outside':gap_counts['outside'],'maximum_BA_delta_associated':ba_counts['inside']+ba_counts['boundary_uncertain'],'minimum_BA_delta_outside':ba_counts['outside']}
 for record in gaps:record['association_68ms']=classify(record['start'],record['end'],tests['query_plus_conservative_68ms']['windows'])
 for record in ba:record['association_68ms']=classify(record['start'],record['end'],tests['query_plus_conservative_68ms']['windows'])
 return {'screen':s,'capture':[summary['capture_monotonic_start'],summary['capture_monotonic_end']],'replayed_counts_equal_closed_original':True,'capture_frame_counts':actual_counts,'source_recording_end_observed':replay['recording_end_observed'],'BLE_max_observed_queue_age_us':max_age,'periodic_windows':windows,'associations':tests,'BLE_gap_brackets':gaps,'BA_sample_count':len(samples),'BA_first':samples[0],'BA_last':samples[-1],'BA_positive_delta_brackets':ba}

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();test_classifier();inputs={}
 report={'kind':'offline_gap_interval_association_not_causal_attribution','created_utc':datetime.now(timezone.utc).isoformat(),'script':str(Path(__file__).relative_to(ROOT)),'script_sha256':sha(Path(__file__)),'analyzer_sha256':sha(ROOT/'tools/analyze_firmware_benchmark.py'),'classifier_tests':'PASS seven synthetic assertions; no scientific fixture or CRC generated','scope':'All gap brackets use the unmodified analyzer ordered CRC+identity+wire-complete subset. Invalid scientific points remain valid complete protocol frames. Neither missing-frame receipt times nor raw scientific frames are reconstructed.','runs':[run(s,inputs) for s in ['19','20']],'inputs':inputs,'limitations':['Bracket-inside means the two observed complete-frame endpoints lie inside a diagnostic association window; this is not proof the absent frame was emitted, or lost then.','Boundary-crossing brackets are uncertain and are only included in the upper association count.','BLE queue age is an observed maximum, not a proven end-to-end upper bound; cascade delays may extend beyond it.','BA timestamps belong to debug receipt of cumulative STK50 snapshots, not the precise mutation instant. All increments within a crossing sample interval remain uncertain.','Firmware, scheduling, host query load and RF have not been randomized. This report tests temporal proximity only.','No additional hardware action or production/source modification occurs.'],'hardware_actions':[],'original_recordings_modified':False}
 with args.output.open('x') as f:json.dump(report,f,indent=2,ensure_ascii=False);f.write('\n')
 for run_result in report['runs']:
  print(json.dumps({'screen':run_result['screen'],'missing':run_result['capture_frame_counts']['missing_frames'],'BA_delta':run_result['BA_last']['ba']-run_result['BA_first']['ba'],'associations':run_result['associations']},indent=2))
 print('OUTPUT',args.output,'SHA256',sha(args.output))
if __name__=='__main__':main()
