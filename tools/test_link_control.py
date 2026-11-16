#!/usr/bin/env python3
"""Compile/run the actual USB/BLE diagnostic formatters and restart ACK gate."""
from pathlib import Path
import os
import subprocess
import tempfile
from test_rowmodes_ack_guard import findCompiler


def main():
    root = Path(__file__).resolve().parents[1]
    compiler, native = findCompiler()
    if not native:
        raise RuntimeError("native execution is required, not compile-only")
    with tempfile.TemporaryDirectory(prefix="sensorarray-link-control-") as name:
        temp = Path(name)
        (temp / "freertos").mkdir()
        (temp / "freertos/FreeRTOS.h").write_text("""#pragma once
#include <pthread.h>
#include <stdint.h>
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(p) pthread_mutex_lock(p)
#define portEXIT_CRITICAL(p) pthread_mutex_unlock(p)
""")
        (temp / "freertos/task.h").write_text("#pragma once\ntypedef void *TaskHandle_t;\n")
        (temp / "sensorarrayTransport.h").write_text("""#pragma once
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
esp_err_t sensorarrayTransportPublishLifecycleObserved(const char *, size_t, bool *);
esp_err_t sensorarrayTransportPublishRuntimeFault(const char *, ...);
""")
        sources = [root / name for name in [
            "tests/host/test_link_control.c",
            "main/control/sensorarrayUsbControl.c",
            "main/control/sensorarrayLinkDiagnostics.c",
            "main/control/sensorarrayRestartControl.c",
            "core/transport/sensorarrayCriticalText.c",
        ]]
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pthread", "-g",
                   "-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        for include in [temp, root / "tests/host/include", root / "main/control",
                        root / "core/measure/include", root / "core/config", root / "core/transport",
                        root / "components/sensorarrayBle/include", root / "components/sensorarrayUsbOtg/include",
                        root / "components/sensorarrayWorkBudget/include",
                        root / "components/sensorarrayWifi/include"]:
            command += ["-I", str(include)]
        executable = temp / "test"
        subprocess.run(command + [str(p) for p in sources] + ["-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True, timeout=10,
                       env=dict(os.environ, ASAN_OPTIONS="detect_leaks=0"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
