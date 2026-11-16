#!/usr/bin/env python3
"""Execute real timing control/mailbox/main-boundary code with synthetic hardware.

The reentrant producer hook is inserted after the real shared reservation and
before ControlPost publishes its local slot; the second transport must lose the
existing global configuration arbiter. No ADC samples here are HIL evidence.
"""
from pathlib import Path
import os
import subprocess
import tempfile

from test_main_config_commit import function
from test_rowmodes_ack_guard import writeFakeHeaders

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    source = (ROOT / "main/main.c").read_text()
    boundary = function(source, "static void sensorarrayRunAdsTimingCommand(")
    runtime = function(source, "static esp_err_t sensorarrayRuntimeQueryCommand(")
    command = function(runtime, '    if (strncmp(command, "ADSCHK=TIMING", 13u) == 0)')
    callbacks = "\n".join(function(source, name) for name in [
        "static void sensorarrayControlReplyPublished(",
        "static void sensorarrayControlReplyFailed("])
    apply = function(source, "static void sensorarrayApplyPendingCommands(")
    assert apply.index("sensorarrayAdsTimingControlExpire") < apply.index("while (sensorarrayCommandMailboxTryReceive")
    assert apply.index("SENSORARRAY_COMMAND_ADS_TIMING") < apply.index("sensorarrayFrontendsHeld()")
    assert "sensorarrayRunAdsTimingCommand(ctx, &command);\n            return;" in apply
    assert source.count("sensorarrayAdsTimingControlHandles(command)") == 2
    assert source.count("sensorarrayLinkDiagnosticsHandles(command)") == 2
    assert source.count("ads_timing_probe_v1/ble_service_obs_v1/work_reason_v1") == 2
    probe = (ROOT / "core/measure/ads/sensorarrayAdsTimingProbe.c").read_text()
    formatters = probe[probe.index("static size_t sensorarrayAdsTimingFormatted("):]
    adc = (ROOT / "components/ads126xAdc/include/ads126xAdc.h").read_text()
    end = adc.index("} ads126xAdc1RegisterSnapshot_t;") + len("} ads126xAdc1RegisterSnapshot_t;")
    snapshot = adc[adc.rfind("typedef struct {", 0, end):end]
    test = (ROOT / "tests/host/test_ads_timing_control.c").read_text()
    test = test.replace("/* EXISTING_MAILBOX_TEST */", '#include "' + str(ROOT / "tests/host/test_rowmodes_ack_guard.c") + '"')
    test = test.replace("/* PRODUCTION_FORMATTERS */", formatters)
    test = test.replace("/* PRODUCTION_BOUNDARY */", boundary)
    test = test.replace("/* PRODUCTION_CALLBACKS */", callbacks)
    test = test.replace("/* PRODUCTION_COMMAND */", command)
    with tempfile.TemporaryDirectory(prefix="sensorarray-ads-timing-control-") as directory:
        tmp = Path(directory); fake = tmp / "fake"; fake.mkdir()
        writeFakeHeaders(fake)
        freertos = fake / "freertos/FreeRTOS.h"
        f = freertos.read_text(); start = f.index("typedef struct { int locked; }")
        f = f[:start] + """#include <pthread.h>
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(p) pthread_mutex_lock(p)
#define portEXIT_CRITICAL(p) pthread_mutex_unlock(p)
"""
        freertos.write_text(f)
        (fake / "ads126xAdc.h").write_text('#pragma once\n#include <stdint.h>\n#include "esp_err.h"\n' +
            snapshot + '\ntypedef struct ads126xAdcHandle ads126xAdcHandle_t;\n')
        unit = tmp / "unit.c"; unit.write_text(test)
        executable = tmp / "test"
        cmd = [os.environ.get("CC", "/usr/bin/gcc"), "-std=c11", "-D_XOPEN_SOURCE=700",
               "-Wall", "-Wextra", "-Werror", "-pthread", "-O1", "-g"]
        for path in [fake, ROOT / "tests/host/include", ROOT / "core/config",
                     ROOT / "core/measure/include", ROOT / "core/measure/ads",
                     ROOT / "core/transport", ROOT / "main/control"]:
            cmd += ["-I", str(path)]
        subprocess.run(cmd + [str(unit), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=20)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
