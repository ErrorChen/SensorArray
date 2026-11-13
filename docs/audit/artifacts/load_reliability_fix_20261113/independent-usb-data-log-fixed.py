#!/usr/bin/env python3
"""Additional independent schedules against the complete production SerialIo.

Extends the boundary harness in a temporary file. This is a CPU/RTOS/driver
simulation, not a claim of real USB service timing or guaranteed log delivery.
"""
from pathlib import Path
import hashlib
import json
import os
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]

CASES = r'''
    /* Two indefinitely incomplete log streams must not erode science or
     * control capacity over repeated independent drain/wake rounds. */
    resetClock(); scenario=0; writeBudget=-1; outputLength=0;
    assert(registeredVfs.write(0,"HOLD_OUT=",9)==9);
    assert(registeredVfs.write(1,"HOLD_ERR=",9)==9);
    sensorarraySerialIoStats_t auditBefore,auditAfter;
    sensorarraySerialIoGetStats(&auditBefore);
    for(unsigned round=0;round<16;round++) {
        assert(sensorarraySerialIoQueue("DIAG_DROP\n",10,false)==ESP_ERR_TIMEOUT);
        for(unsigned i=0;i<4;i++) {
            assert(sensorarraySerialIoQueueData("SCI\n",4)==ESP_OK);
            assert(sensorarraySerialIoQueue("CTL\n",4,true)==ESP_OK);
        }
        drain();
        for(unsigned i=0;i<9;i++) assert(!s_inUse[i]);
        assert(s_inUse[9] && s_inUse[10]);
    }
    sensorarraySerialIoGetStats(&auditAfter);
    assert(auditAfter.dataRejected==auditBefore.dataRejected);
    assert(auditAfter.logRejected==auditBefore.logRejected+16);
    assert(auditAfter.dataSubmitted==auditBefore.dataSubmitted+64);
    assert(auditAfter.prioritySubmitted==auditBefore.prioritySubmitted+64);
    assert(outputLength==16*32);
    for(unsigned round=0;round<16;round++)
        assert(memcmp(output+round*32,"CTL\nCTL\nCTL\nCTL\nSCI\nSCI\nSCI\nSCI\n",32)==0);
    assert(registeredVfs.write(0,"ok\n",3)==3);
    assert(registeredVfs.write(1,"ok\n",3)==3);
    drain();
    for(unsigned i=0;i<SERIAL_IO_SLOT_COUNT;i++) assert(!s_inUse[i]);
    fprintf(stderr,"AUDIT_PARTIAL_LOG_ISOLATION,rounds=16,dataSubmitted=64,controlSubmitted=64,dataRejected=0,logRejected=16,allReleased=1\n");

    /* Normal fault diagnostic text is deliberately best effort: two reserved
     * slots do not promise to retain an unserviced six-record fault burst. */
    resetClock(); scenario=0; writeBudget=-1; outputLength=0;
    sensorarraySerialIoGetStats(&auditBefore);
    for(unsigned i=0;i<6;i++)
        assert(registeredVfs.write(0,"FAULT_DIAGNOSTIC\n",17)==17);
    assert(sensorarraySerialIoQueueData("SCI\n",4)==ESP_OK);
    assert(sensorarraySerialIoQueue("MFAULT\n",7,true)==ESP_OK);
    drain();
    sensorarraySerialIoGetStats(&auditAfter);
    assert(auditAfter.logRejected==auditBefore.logRejected+4);
    assert(auditAfter.lineDropped==auditBefore.lineDropped+4);
    assert(auditAfter.dataRejected==auditBefore.dataRejected);
    assert(outputLength==45 && memcmp(output,"MFAULT\nSCI\n",11)==0);
    fprintf(stderr,"AUDIT_DIAGNOSTIC_LIMIT,sixTextRecords=6,logRejected=4,dataPreserved=1,controlPreserved=1,notGuaranteedDiagnosticDelivery=1\n");

    /* A LOG short prefix then no host exhausts normal and injected control
     * lifetimes. On the NEXT wake of the SAME TX invocation, restore the
     * host: delimiter state must survive the idle wait before fresh control. */
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
    fprintf(stderr,"AUDIT_LOG_PREFIX_IDLE_RECOVERY,normalExpired=2,priorityExpired=1,partial=1,newControl=1,delimiterPersistedAcrossIdle=1,allReleased=1\n");
'''


def main():
    source = ROOT / "core/transport/sensorarraySerialIo.c"
    original_source = source.read_bytes()
    test_source = (ROOT / "tests/host/test_serial_io.c").read_text()
    runner = (ROOT / "tools/test_serial_io.py").read_text()
    print(json.dumps({"production_source": str(source),
        "sha256": hashlib.sha256(original_source).hexdigest(),
        "harness_sha256": hashlib.sha256(test_source.encode()).hexdigest(),
        "runner_sha256": hashlib.sha256(runner.encode()).hexdigest(),
        "synthetic_schedules": True, "hardware_access": False}), flush=True)
    with tempfile.TemporaryDirectory(prefix="sa-usb-fixed-review-") as temp:
        temp = Path(temp)
        source_snapshot = temp / "serial_io.c"
        source_snapshot.write_bytes(original_source)
        test = temp / "audit.c"
        needle = '    printf("SERIAL_IO_PASS,allocation='
        assert test_source.count(needle) == 1
        test.write_text(test_source.replace(needle, CASES + needle))
        runner = runner.replace('str(root / "tests/host/test_serial_io.c")', repr(str(test)))
        sys.argv = ["test_serial_io.py", "--source", str(source_snapshot)]
        os.environ.setdefault("CC", "/usr/bin/gcc")
        exec(compile(runner, str(ROOT / "tools/test_serial_io.py"), "exec"),
             {"__file__": str(ROOT / "tools/test_serial_io.py"), "__name__": "__main__"})
    assert source.read_bytes() == original_source, "production changed during independent test"
    print("AUDIT_SOURCE_UNCHANGED=1", flush=True)


if __name__ == "__main__":
    main()
