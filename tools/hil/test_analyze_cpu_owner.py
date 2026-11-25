#!/usr/bin/env python3
"""Synthetic diagnostics only; no hardware or scientific frame fabrication."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import analyze_cpu_owner as analyzer


def fixture():
    identity={key:"1" for key in analyzer.IDENTITY}
    identity.update(session="0123456789ABCDEF",state="VOLTAGE",mode="VOLT",rows="1",profile="VVVVVVVV")
    summary={"snapshots":{}}
    for phase, offset in (("before",0),("after",10_000_000)):
        snap={"CONFIG?":{"fields":dict(identity)},"CONFIG?_closing":{"fields":dict(identity)}}
        for role in range(7):
            t=1_000_000+offset+role*100
            f=dict(v="1",session=identity["session"],role=str(role),enabled="1",available="1",known="1",
                   conflict="0",stale="0",reason="ok",taskNumber=str(role+8),runUs=str((1<<32)+offset//5),
                   t0Us=str(t),t1Us=str(t+5),observedAtUs=str(t+5),ageUs="10000",sampleMinUs="1000000",
                   staleAfterUs="3000000",scope="switch_out",isr="may_include",sdk="not_isolated",blocked="unknown")
            snap[f"CPUOWNER?{role}"]={"fields":f}
        summary["snapshots"][phase]=snap
    return summary


def run(summary, absent=False):
    raw=set();events=set()
    for phase,snap in summary["snapshots"].items():
        for command,item in snap.items():
            tag="CPUOWNER" if command.startswith("CPUOWNER?") else "CONFIG"
            wire_command=command if tag=="CPUOWNER" else "CONFIG?"
            line=tag+","+",".join(f"{k}={v}" for k,v in item["fields"].items())
            item["reply_line"]=line;raw.add(line);events.add((phase,wire_command,line))
    if absent: events={v for v in events if v[:2]!=("after","CPUOWNER?0")}
    return analyzer.analyze_document(summary,raw,events)


class CpuAnalysis(unittest.TestCase):
    def test_independent_brackets_and_u64(self):
        s=fixture();s['snapshots']['after']['CPUOWNER?1']['fields'].update(t0Us='12000100',t1Us='12000105',observedAtUs='12000105')
        r=run(s);self.assertEqual(r['paired_roles'],list(range(7)))
        self.assertEqual(r['roles'][0]['runtime_delta_us'],2_000_000)
        self.assertEqual(r['roles'][0]['runtime_per_own_observation_span_percent'],20)
        self.assertNotEqual(r['roles'][0]['observation_span_us'],r['roles'][1]['observation_span_us'])
        self.assertIsNone(r['aggregate_role_percent']);self.assertEqual(r['blocked_time'],'UNKNOWN')

    def test_stale_zero_is_unknown(self):
        s=fixture()
        for phase in ('before','after'):s['snapshots'][phase]['CPUOWNER?4']['fields'].update(stale='1',ageUs='4000000',runUs='0')
        r=run(s)['roles'][4];self.assertEqual(r['status'],'UNKNOWN');self.assertIsNone(r['runtime_delta_us'])

    def test_not_observed(self):
        s=fixture()
        for phase in ('before','after'):
            f=s['snapshots'][phase]['CPUOWNER?6']['fields'];f.update(known='0',available='0',stale='1',reason='not_observed')
            for key in ('taskNumber','runUs','t0Us','t1Us','observedAtUs','ageUs'):f[key]='unknown'
        self.assertEqual(run(s)['roles'][6]['status'],'UNKNOWN')

    def test_rejections(self):
        changes=[{'session':'FEDCBA9876543210'},{'taskNumber':'999'},{'runUs':'1'},
                 {'t0Us':'1','t1Us':'2','observedAtUs':'2'},{'t1Us':'1'},
                 {'available':'0','conflict':'1','reason':'writer_conflict','runUs':'unknown'},
                 {'role':'3'},{'ageUs':'3000000'},{'runUs':str(1<<64)}]
        for change in changes:
            with self.subTest(change=change):
                s=fixture();s['snapshots']['after']['CPUOWNER?0']['fields'].update(change)
                r=run(s)['roles'][0];self.assertEqual(r['status'],'UNKNOWN');self.assertIsNone(r['runtime_delta_us'])

    def test_changed_CONFIG_invalidates_pairing(self):
        s=fixture();s['snapshots']['after']['CONFIG?_closing']['fields']['rev']='2'
        self.assertEqual(run(s)['paired_roles'],[])

    def test_missing_event_and_schema(self):
        self.assertEqual(run(fixture(),absent=True)['roles'][0]['status'],'UNKNOWN')
        s=fixture();del s['snapshots']['after']['CPUOWNER?0']['fields']['blocked']
        self.assertEqual(run(s)['roles'][0]['status'],'UNKNOWN')

    def test_raw_hash_and_incomplete_line(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);s=fixture();run(s)
            lines=[];events=[]
            for phase,snap in s['snapshots'].items():
                for cmd,item in snap.items():
                    wire_command=cmd if cmd.startswith('CPUOWNER?') else 'CONFIG?'
                    lines.append(item['reply_line']);events.append(dict(type='reply',phase=phase,command=wire_command,line=item['reply_line']))
            raw=('\n'.join(lines)+'\n').encode();(p/'raw.bin').write_bytes(raw)
            (p/'events.jsonl').write_text('\n'.join(json.dumps(e) for e in events)+'\n')
            s.update(raw_file=str(p/'raw.bin'),events_file=str(p/'events.jsonl'),raw_sha256=hashlib.sha256(raw).hexdigest())
            (p/'summary.json').write_text(json.dumps(s));self.assertEqual(analyzer.analyze_file(p/'summary.json')['paired_roles'],list(range(7)))
            (p/'raw.bin').write_bytes(raw+b'X')
            with self.assertRaisesRegex(ValueError,'raw_sha256_mismatch'):analyzer.analyze_file(p/'summary.json')
            raw=raw.rstrip(b'\n');(p/'raw.bin').write_bytes(raw);s['raw_sha256']=hashlib.sha256(raw).hexdigest()
            (p/'summary.json').write_text(json.dumps(s));self.assertEqual(analyzer.analyze_file(p/'summary.json')['roles'][6]['status'],'UNKNOWN')


if __name__=='__main__':unittest.main()
