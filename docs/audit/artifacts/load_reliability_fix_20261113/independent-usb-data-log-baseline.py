#!/usr/bin/env python3
"""Read-only replay of DATA/LOG contention in the actually flashed baseline.

The scheduling cases are synthetic. The production SerialIo source, host
boundary harness and runner are read from the immutable baseline commit. No
wire frame is fabricated and no production source is changed.
"""
from pathlib import Path
import hashlib
import json
import os
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[4]
BASE = "b44c37b51067e376e73aa44535ea47d175846b67"


def at(path):
    return subprocess.check_output(["git", "show", f"{BASE}:{path}"], cwd=ROOT).decode()


case = r'''
    resetClock(); scenario=0; writeBudget=-1; outputLength=0;
    for(unsigned i=0;i<4;i++)
        assert(registeredVfs.write(0,"DIAGNOSTIC\n",11)==11);
    esp_err_t scienceAfterLogs=sensorarraySerialIoQueue(encodedFrame,sizeof(encodedFrame)-1,false);
    assert(scienceAfterLogs==ESP_ERR_TIMEOUT && s_queues[0]->count==4);
    assert(sensorarraySerialIoQueue("CTRL\n",5,true)==ESP_OK);
    fprintf(stderr,"AUDIT_LOG_BURST,logs=4,dataError=0x%x,normalQueued=%u,ctrlAdmitted=1\n",scienceAfterLogs,s_queues[0]->count);
    drain();
    assert(outputLength==49 && memcmp(output,"CTRL\n",5)==0);

    resetClock(); scenario=0; writeBudget=-1; outputLength=0;
    assert(registeredVfs.write(0,"OUT=",4)==4);
    assert(registeredVfs.write(1,"ERR=",4)==4);
    for(unsigned i=0;i<3;i++)
        assert(sensorarraySerialIoQueue("SUMMARY\n",8,false)==ESP_OK);
    unsigned normalUsed=0;
    for(unsigned i=0;i<5;i++) normalUsed += s_inUse[i]?1u:0u;
    esp_err_t scienceAfterFragments=sensorarraySerialIoQueue(encodedFrame,sizeof(encodedFrame)-1,false);
    assert(scienceAfterFragments==ESP_ERR_TIMEOUT && s_queues[0]->count==3 && normalUsed==5);
    fprintf(stderr,"AUDIT_STDIO_FRAGMENTS,partialStreams=2,logs=3,dataError=0x%x,normalQueued=%u,normalSlotsUsed=%u\n",scienceAfterFragments,s_queues[0]->count,normalUsed);
    drain();
    assert(registeredVfs.write(0,"done\n",5)==5);
    assert(registeredVfs.write(1,"done\n",5)==5);
    drain();
'''


def main():
    sources = {p: at(p) for p in (
        "core/transport/sensorarraySerialIo.c",
        "tests/host/test_serial_io.c",
        "tools/test_serial_io.py",
        "main/output/sensorarrayFrameOutput.c",
    )}
    print(json.dumps({"baseline": BASE, "source_sha256": {
        p: hashlib.sha256(v.encode()).hexdigest() for p, v in sources.items()
    }, "synthetic_schedule": True, "hardware_access": False}), flush=True)
    with tempfile.TemporaryDirectory(prefix="sa-usb-data-log-review-") as temp:
        temp = Path(temp)
        source = temp / "baseline_serial_io.c"
        source.write_text(sources["core/transport/sensorarraySerialIo.c"])
        test = sources["tests/host/test_serial_io.c"]
        needle = '    printf("SERIAL_IO_PASS,allocation='
        assert test.count(needle) == 1
        test = test.replace(needle, case + needle)
        test_file = temp / "audit_serial_io.c"
        test_file.write_text(test)
        script = sources["tools/test_serial_io.py"]
        script = script.replace('str(root / "tests/host/test_serial_io.c")', repr(str(test_file)))
        script = script.replace('(root / "main/output/sensorarrayFrameOutput.c").read_text()',
                                repr(sources["main/output/sensorarrayFrameOutput.c"]))
        sys.argv = ["test_serial_io.py", "--source", str(source)]
        os.environ.setdefault("CC", "/usr/bin/gcc")
        exec(compile(script, str(ROOT / "tools/test_serial_io.py"), "exec"),
             {"__file__": str(ROOT / "tools/test_serial_io.py"), "__name__": "__main__"})


if __name__ == "__main__":
    main()
