#!/usr/bin/env python3
"""Run the actual worker/coordinator functions in a pthread FreeRTOS shim.

Hardware calls are synthetic, explicitly injected delays/failures. Production
worker/coordinator code is extracted verbatim, not reimplemented in this test.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
source = (root / "core/measure/fdc/sensorarrayFdcRowEpoch.c").read_text()
worker = source[source.index("bool sensorarrayMeasureFdcWorkersQuiescent"):source.index("esp_err_t sensorarrayMeasureEnsureFdcWorkers")]
start = source.index("esp_err_t sensorarrayMeasureReadFdcMatrixRowParallelEpoch(")
coordinator = source[start:source.index("esp_err_t __attribute__((unused)) sensorarrayMeasureReadFdcRuntimeChannelConfigs", start)]
header = (root / "core/measure/fdc/sensorarrayFdcInternal.h").read_text()
def struct(name):
    end = header.index("} " + name + ";") + len("} " + name + ";")
    begin = header.rfind("typedef struct {", 0, end)
    return header[begin:end]
row_fields = sorted(set(re.findall(r"rowTiming->(\w+)", coordinator)))
device_fields = sorted(set(re.findall(r"(?:primaryTiming|secondaryTiming)->(\w+)", coordinator)) | {"row", "deviceId"})
prelude = (root / "tests/host/fdc_worker_runtime_prelude.h").read_text()
prelude = prelude.replace("/* ROW_FIELDS */", "\n".join("uint64_t " + k + ";" for k in row_fields))
prelude = prelude.replace("/* DEVICE_FIELDS */", "\n".join("uint64_t " + k + ";" for k in device_fields))
prelude = prelude.replace("/* FIRMWARE_TYPES */", "\n".join(struct(n) for n in ["sensorarrayFdcWorkerResult_t", "sensorarrayFdcWorkerTrace_t", "sensorarrayFdcFrameReadTracker_t", "sensorarrayFdcWorkerJob_t"]))
with tempfile.TemporaryDirectory(prefix="sensorarray-fdc-runtime-") as temp:
    temp = Path(temp)
    unit = temp / "worker_runtime.c"
    unit.write_text(prelude + "\n" + worker + "\n" + coordinator + "\n" + (root / "tests/host/fdc_worker_runtime_main.c").read_text())
    exe = temp / "test"
    subprocess.run([os.environ.get("CC", "cc"), "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", "-pthread", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-I", str(root / "core/measure/fdc"), str(unit), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True, timeout=20, env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0:detect_stack_use_after_return=1"})
