#!/usr/bin/env python3
"""Synthetic independent probe. Uses the real collector, never hardware."""
from pathlib import Path
import sys,json,hashlib
STAGE=Path(__file__).resolve().parent
sys.path.insert(0,str(STAGE/'scripts'))
import test_capture_short_snapshots as fixtures
class RestoreFault(fixtures.TxFirmware):
    def write(self,endpoint,data,timeout):
        if data.strip()==b'TX=REL':
            self.shared.append(b'MFAULT,seq=999,err=0x108,state=DEGRADED,route=SAFE\n')
        return super().write(endpoint,data,timeout)
results=[]
for label,device in [('normal',fixtures.TxFirmware()),('fault_during_restore',RestoreFault())]:
    index,report,raw=fixtures.ShortTests().capture(device)
    result={'case':label,'indexStatus':index['status'],'summaryStatus':report['status'],'txRestore':report['tx_restore'],'reportedRxBytes':report['rx_bytes'],'actualRawBytes':len(raw),'rawShaMatches':report['raw_sha256']==hashlib.sha256(raw).hexdigest(),'reportedFaults':report['firmware_fault_events'],'rawContainsRestoreFault':b'MFAULT,seq=999' in raw,'restoredTx':device.tx}
    assert result['rawShaMatches']
    assert result['reportedRxBytes']<result['actualRawBytes']
    if label=='fault_during_restore':
        assert result['rawContainsRestoreFault'] and result['reportedFaults']==0 and result['summaryStatus']=='CAPTURED'
    results.append(result)
print(json.dumps({'meaning':'BUG_REPRODUCED_not_acceptance','synthetic':True,'hardware':'NOT_RUN','results':results},indent=2))
