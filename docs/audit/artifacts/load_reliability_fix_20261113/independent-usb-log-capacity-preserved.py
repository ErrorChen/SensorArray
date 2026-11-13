#!/usr/bin/env python3
"""Review DATA isolation while retaining the original five-slot LOG capacity.

Keeps the rejected two-slot experiment and its evidence intact. Test inputs
below are synthetic records, not claimed to be recorded firmware faults.
"""
from pathlib import Path
import importlib.util

path = Path(__file__).with_name("independent-usb-data-log-fixed.py")
spec = importlib.util.spec_from_file_location("independent_fixed", path)
review = importlib.util.module_from_spec(spec)
spec.loader.exec_module(review)
review.CASES = r'''
    assert(SERIAL_IO_LOG_SLOT_COUNT==5u && SERIAL_IO_LOG_QUEUE_COUNT==4u);
    resetClock(); scenario=0; writeBudget=-1; outputLength=0;
    assert(registeredVfs.write(0,"HOLD_OUT=",9)==9);
    assert(registeredVfs.write(1,"HOLD_ERR=",9)==9);
    sensorarraySerialIoStats_t auditBefore,auditAfter;
    sensorarraySerialIoGetStats(&auditBefore);
    for(unsigned round=0;round<16;round++) {
        for(unsigned i=0;i<3;i++)
            assert(sensorarraySerialIoQueue("DIAG\n",5,false)==ESP_OK);
        assert(sensorarraySerialIoQueue("DIAG_DROP\n",10,false)==ESP_ERR_TIMEOUT);
        for(unsigned i=0;i<4;i++) {
            assert(sensorarraySerialIoQueueData("SCI\n",4)==ESP_OK);
            assert(sensorarraySerialIoQueue("CTL\n",4,true)==ESP_OK);
        }
        drain();
        for(unsigned i=0;i<9;i++) assert(!s_inUse[i]);
        assert(s_inUse[9] && s_inUse[10]);
        for(unsigned i=11;i<SERIAL_IO_SLOT_COUNT;i++) assert(!s_inUse[i]);
    }
    sensorarraySerialIoGetStats(&auditAfter);
    assert(auditAfter.dataRejected==auditBefore.dataRejected);
    assert(auditAfter.logRejected==auditBefore.logRejected+16);
    assert(auditAfter.dataSubmitted==auditBefore.dataSubmitted+64);
    assert(auditAfter.prioritySubmitted==auditBefore.prioritySubmitted+64);
    assert(auditAfter.logSubmitted==auditBefore.logSubmitted+48);
    assert(outputLength==16*47);
    for(unsigned round=0;round<16;round++)
        assert(memcmp(output+round*47,"CTL\nCTL\nCTL\nCTL\nSCI\nSCI\nSCI\nSCI\nDIAG\nDIAG\nDIAG\n",47)==0);
    assert(registeredVfs.write(0,"ok\n",3)==3);
    assert(registeredVfs.write(1,"ok\n",3)==3);
    drain();
    for(unsigned i=0;i<SERIAL_IO_SLOT_COUNT;i++) assert(!s_inUse[i]);
    fprintf(stderr,"AUDIT_PRESERVED_LOG_ISOLATION,rounds=16,dataSubmitted=64,controlSubmitted=64,logSubmitted=48,dataRejected=0,logRejected=16,allReleased=1\n");

    /* Seven distinct fault-like records: the former four-entry queue keeps
     * exactly the same first four records, including primary context/mask. */
    resetClock(); scenario=0; writeBudget=-1; outputLength=0;
    static const char *auditFaults[]={
        "ADSFAULT,seq=42,stage=REFMUX_COMPARE,err=0x108\n",
        "ADSFAULTCTX,seq=42,originalError=0x108,cleanupError=0x107\n",
        "ADSFAULTREG,seq=42,reg=15,expected=00,actual=01,mask=3F\n",
        "ADSFAULTTXN,seq=42,kind=RAIL,id=10\n",
        "ADSFAULTTXN,seq=42,kind=BATTERY,id=11\n",
        "ADSFAULTTXN,seq=42,kind=CHECK,id=12\n",
        "ADSSPIBYTES,seq=42,synthetic=1\n"
    };
    sensorarraySerialIoGetStats(&auditBefore);
    for(unsigned i=0;i<7;i++) {
        size_t n=strlen(auditFaults[i]);
        assert(registeredVfs.write(0,auditFaults[i],n)==(ssize_t)n);
    }
    assert(sensorarraySerialIoQueueData("SCI\n",4)==ESP_OK);
    assert(sensorarraySerialIoQueue("MFAULT\n",7,true)==ESP_OK);
    drain();
    sensorarraySerialIoGetStats(&auditAfter);
    assert(auditAfter.logRejected==auditBefore.logRejected+3);
    assert(auditAfter.lineDropped==auditBefore.lineDropped+3);
    assert(auditAfter.dataRejected==auditBefore.dataRejected);
    assert(memcmp(output,"MFAULT\nSCI\n",11)==0);
    size_t auditPosition=11;
    for(unsigned i=0;i<4;i++) {
        size_t n=strlen(auditFaults[i]);
        assert(memcmp(output+auditPosition,auditFaults[i],n)==0);
        auditPosition+=n;
    }
    assert(outputLength==auditPosition);
    fprintf(stderr,"AUDIT_PRESERVED_FAULT_BURST,records=7,firstRecordsPreserved=4,tailRejected=3,dataPreserved=1,controlPreserved=1\n");

    resetClock(); scenario=1; writeBudget=2; outputLength=0;
    injectPriorityAfterShort=true; injectedPriority=false; injectDataAfterShort=true;
    injectControlOnIdle=true;
    sensorarraySerialIoGetStats(&auditBefore);
    assert(sensorarraySerialIoQueue("LOG_OLD\n",8,false)==ESP_OK);
    drain();
    sensorarraySerialIoGetStats(&auditAfter);
    assert(outputLength==18 && memcmp(output,"LO\nIDLE_RECOVERED\n",18)==0);
    assert(auditAfter.normalExpired==auditBefore.normalExpired+2);
    assert(auditAfter.priorityExpired==auditBefore.priorityExpired+1);
    assert(auditAfter.partialWrite==auditBefore.partialWrite+1);
    assert(auditAfter.prioritySubmitted==auditBefore.prioritySubmitted+1);
    for(unsigned i=0;i<SERIAL_IO_SLOT_COUNT;i++) assert(!s_inUse[i]);
    injectPriorityAfterShort=false; injectDataAfterShort=false;
    fprintf(stderr,"AUDIT_PRESERVED_LOG_PREFIX_IDLE_RECOVERY,normalExpired=2,priorityExpired=1,partial=1,newControl=1,delimiterPersistedAcrossIdle=1,allReleased=1\n");
'''

if __name__ == "__main__":
    review.main()
