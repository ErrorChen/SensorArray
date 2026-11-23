#!/usr/bin/env python3
"""Actual extracted production call counts; synthetic host scheduling, not target timing.
No production/harness/tool source is edited. Original maintained tests still execute.
"""
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[5]
sys.path.insert(0,str(root/'tools'))
import test_ble_priority as runner
original_read=Path.read_text
original_function=runner.function
harness_path=root/'tests/host/test_ble_priority.c'
fields='sync observe crcCalls crcBytes asciiCalls asciiBytes descriptor checks gatt locks'.split()
declarations='static unsigned long long '+','.join('probe_'+x for x in fields)+';\n'
probe=r"""
static void independentBleHotPathCounts(void)
{
    for (unsigned typed=0u;typed<2u;typed++) {
        reset();s_ble.stats.mtu=517u;
        sensorarrayBleFramePublication_t p=typedPublication(1u,1u,0u);
        p.normalData=p.fullData;p.normalLength=p.fullLength;
        if (typed) {assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();}
        PROBE_RESET
        for(unsigned i=0u;i<100u;i++) {
            nowUs+=15000u;p.identity.sequence=i+2u;
            assert((typed ? sensorarrayBleNotifyFrameForGeneration(&p,7u) :
                sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA,p.fullData,p.fullLength,7u))==ESP_OK);
            runToIdle();sentLength=sentChannelCount=0u;
        }
        printf("BLE_HOT_PATH_COUNTS,typed=%u,frames=100,PROBE_FORMAT,hardware=NOT_RUN,target_cpu_time=UNKNOWN\n",typed,PROBE_ARGUMENTS);
    }
}
""".replace('PROBE_RESET',';'.join('probe_'+x+'=0u' for x in fields)+';').replace('PROBE_FORMAT',','.join(x+'=%llu' for x in fields)).replace('PROBE_ARGUMENTS',','.join('probe_'+x for x in fields))
def read(self,*args,**kwargs):
 text=original_read(self,*args,**kwargs)
 if self.resolve()==harness_path:
  text=text.replace('/* PRODUCTION_FUNCTIONS */',declarations+'/* PRODUCTION_FUNCTIONS */')
  text=text.replace('int main(',probe+'\nint main(')
  text=text.replace('    testLogDataAdmissionPolicy();','    independentBleHotPathCounts(); testLogDataAdmissionPolicy();')
  text=text.replace('assert(lockDepth == 0u); lockDepth++;','assert(lockDepth == 0u); probe_locks++; lockDepth++;')
  # The macro above is expanded before declarations only in later production
  # bodies, so declarations remain before every use.
 return text
def function(text,signature):
 body=original_function(text,signature)
 counts={
 'sensorarrayBleFrameFlowSyncLocked':'probe_sync++;',
 'sensorarrayBleFrameFlowObserveLocked':'probe_observe++;',
 'sensorarrayBleCrc32':'probe_crcCalls++;probe_crcBytes+=length;',
 'sensorarrayBleValidateAscii':'probe_asciiCalls++;probe_asciiBytes+=length;',
 'sensorarrayBleDescriptorCurrent':'probe_descriptor++;',
 'sensorarrayBleSendContextMatches':'probe_checks++;',
 'sensorarrayBleSendGattValue':'probe_gatt++;',
 }
 for name,injection in counts.items():
  if name+'(' in signature:
   body=body.replace('{','{\n'+injection,1);break
 return body
Path.read_text=read
runner.function=function
runner.main()
