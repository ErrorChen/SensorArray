#!/usr/bin/env python3
"""Synthetic real-collector reset observation saturation regression; no hardware."""
from pathlib import Path
import sys,json,hashlib
STAGE=Path(__file__).resolve().parent
sys.path.insert(0,str(STAGE/'scripts'))
import test_capture_short_snapshots as fixtures
class RestoreReset(fixtures.TxFirmware):
    def __init__(self):
        super().__init__()
        for i in range(16):self.shared.append(('RST,syntheticHistory='+str(i)+'\n').encode())
    def write(self,endpoint,data,timeout):
        if data.strip()==b'TX=REL':self.shared.append(b'RST,syntheticRestore=17\n')
        return super().write(endpoint,data,timeout)
index,report,raw=fixtures.ShortTests().capture(RestoreReset())
result={'indexStatus':index['status'],'summaryStatus':report['status'],'restore':report['tx_restore'],'reportedResetSamples':len(report['reset_observations']),'rawResetLines':raw.count(b'RST,'),'rawContainsRestoreReset':b'RST,syntheticRestore=17' in raw,'observationError':report['tx_restore_observation_error'],'safeCapRecovery':report['safe_cap_recovery'],'rawShaMatches':report['raw_sha256']==hashlib.sha256(raw).hexdigest()}
assert result['rawContainsRestoreReset'] and result['rawResetLines']==17 and result['reportedResetSamples']==16
assert result['summaryStatus']=='CAPTURED' and result['indexStatus']=='COMPLETE' and not result['observationError']
print(json.dumps({'meaning':'BUG_REPRODUCED_not_acceptance','synthetic':True,'hardware':'NOT_RUN','result':result},indent=2))
