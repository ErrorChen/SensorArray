#!/usr/bin/env python3
"""Execute actual BLE/timer initialization against synthetic SDK failures."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
from test_main_config_commit import function
from test_rowmodes_ack_guard import findCompiler


def main():
    root = Path(__file__).resolve().parents[1]
    source = (root / "components/sensorarrayBle/sensorarrayBle.c").read_text()
    harness = (root / "tests/host/test_ble_watchdog_init.c").read_text()
    definitions = source[source.index("#ifndef CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN"):
                         source.index("/*\n * Statistics are observed")]
    definitions = '#include "sensorarrayBleOutputSchedule.h"\n' + definitions
    state = "\n".join(re.search(r"^static " + typ + " " + name + r"[^\n]*", source, re.M)[0]
                       for typ, name in (("bool", "s_bleInitAttempted"), ("esp_err_t", "s_bleInitResult")))
    signatures = ("static void sensorarrayBleStatsSetInitError(",
                  "static esp_power_level_t sensorarrayBleHighPower(",
                  "static esp_err_t sensorarrayBleStep(",
                  "static esp_err_t sensorarrayBleFinishInit(",
                  "static esp_err_t sensorarrayBleInitPayloadStorage(", "esp_err_t sensorarrayBleInit(")
    harness = harness.replace("/* PRODUCTION_TYPES */", definitions)
    harness = harness.replace("/* PRODUCTION_INIT_STATE */", state)
    harness = harness.replace("/* PRODUCTION_INIT_FUNCTIONS */", "\n\n".join(function(source, s) for s in signatures))
    compiler, native = findCompiler()
    assert native, "execution required"
    with tempfile.TemporaryDirectory(prefix="sensorarray-ble-guard-") as directory:
        temp = Path(directory)
        (temp / "esp_timer.h").write_text("""#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
typedef void *esp_timer_handle_t;
enum { ESP_TIMER_TASK=0 };
typedef struct { void (*callback)(void *); void *arg; int dispatch_method; const char *name; bool skip_unhandled_events; } esp_timer_create_args_t;
esp_err_t esp_timer_create(const esp_timer_create_args_t *, esp_timer_handle_t *);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t, uint64_t);
""")
        generated = temp / "guard.c"
        generated.write_text(harness)
        output = temp / "guard"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-g",
                   "-fsanitize=address,undefined", "-fno-omit-frame-pointer"]
        for include in (temp, root / "tests/host/include", root / "components/sensorarrayBle/include",
                        root / "components/sensorarrayBle"):
            command += ["-I", str(include)]
        subprocess.run(command + [str(generated), "-o", str(output)], check=True)
        subprocess.run([str(output)], check=True, timeout=10,
                       env=dict(os.environ, ASAN_OPTIONS="detect_leaks=0"))


if __name__ == "__main__":
    main()
