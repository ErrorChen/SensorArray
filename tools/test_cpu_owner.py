#!/usr/bin/env python3
"""Execute optional owner runtime code with a synthetic scheduler, no HIL."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

from test_main_config_commit import function

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    main_source = (ROOT / "main/main.c").read_text()
    test = (ROOT / "tests/host/test_cpu_owner.c").read_text()
    for label, signature in (("EARLY", "static esp_err_t sensorarrayEarlyRuntimeQueryCommand("),
                             ("NORMAL", "static esp_err_t sensorarrayRuntimeQueryCommand(")):
        body = function(main_source, signature)
        branch = function(body, "    if (sensorarrayCpuOwnerHandles(command))")
        generated = ("static esp_err_t actual" + label.title() +
                     "(const char *command, char *response, size_t responseSize) {\n" +
                     "const uint64_t s_configSessionId=session;\n" + branch +
                     "\nreturn ESP_ERR_NOT_FOUND;\n}\n")
        test = test.replace(f"/* ACTUAL_{label}_DISPATCH */", generated)
    proto = re.search(r'"PROTO,version=%u,wires=ascii[^"\n]*"', main_source).group(0)
    test = test.replace("/* ACTUAL_PROTO */", proto)
    assert main_source.count("/cpu_owner_v1,") == 2
    config = (ROOT / "main/Kconfig.projbuild").read_text()
    config = config[config.index("config SENSORARRAY_CPU_OWNER_DIAGNOSTICS"):]
    assert "default n" in config and "depends on !FREERTOS_SMP && !FREERTOS_UNICORE" in config
    for dependency in ("FREERTOS_GENERATE_RUN_TIME_STATS", "FREERTOS_RUN_TIME_COUNTER_TYPE_U64",
                       "FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER"):
        assert dependency in config
    assert not re.search(r"^\s+select\s", config, re.M)
    module = (ROOT / "components/sensorarrayWorkBudget/sensorarrayCpuOwner.c").read_text()
    for forbidden in ("uxTaskGetSystemState(", "uxTaskGetStackHighWaterMark(", "vTaskDelay(", "malloc(", "xTaskCreate("):
        assert forbidden not in module
    with tempfile.TemporaryDirectory(prefix="sensorarray-cpu-owner-") as directory:
        tmp = Path(directory); (tmp / "freertos").mkdir()
        (tmp / "sdkconfig.h").write_text("#pragma once\n")
        (tmp / "esp_timer.h").write_text("#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n")
        (tmp / "esp_freertos_hooks.h").write_text('''#pragma once
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "esp_err.h"
typedef bool (*esp_freertos_idle_cb_t)(void);
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t, UBaseType_t);
''')
        (tmp / "freertos/FreeRTOS.h").write_text('''#pragma once
#include <assert.h>
#include <pthread.h>
#include <stdint.h>
typedef uint32_t TickType_t;
typedef uint32_t UBaseType_t;
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
void testEnter(portMUX_TYPE *);
void testExit(portMUX_TYPE *);
#define portENTER_CRITICAL(p) testEnter(p)
#define portEXIT_CRITICAL(p) testExit(p)
#define configRUN_TIME_COUNTER_TYPE uint64_t
#define configTICK_RATE_HZ 100u
#define portTICK_TYPE_IS_ATOMIC 1
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((ms)/10u)
#define pdTRUE 1
#define pdFALSE 0
#define pdPASS 1
#define configASSERT(x) assert(x)
''')
        (tmp / "freertos/task.h").write_text('''#pragma once
#include "freertos/FreeRTOS.h"
typedef void *TaskHandle_t;
typedef enum { eReady=1 } eTaskState;
typedef struct { TaskHandle_t xHandle; char *pcTaskName; uint32_t xTaskNumber;
                 configRUN_TIME_COUNTER_TYPE ulRunTimeCounter; } TaskStatus_t;
void vTaskGetInfo(TaskHandle_t, TaskStatus_t *, int, eTaskState);
TickType_t xTaskGetTickCount(void);
int xPortGetCoreID(void);
void vTaskDelay(TickType_t);
''')
        unit = tmp / "unit.c"; unit.write_text(test)
        base = [os.environ.get("CC", "/usr/bin/gcc"), "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-pthread", "-O1", "-g"]
        for include in (tmp, ROOT / "tests/host/work_budget_stubs", ROOT / "tests/host/include",
                        ROOT / "components/sensorarrayWorkBudget/include", ROOT / "components/sensorarrayWorkBudget"):
            base += ["-I", str(include)]
        for enabled in (0, 1):
            executable = tmp / f"test-{enabled}"
            options = [f"-DCONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS={enabled}",
                       "-DCONFIG_FREERTOS_SMP=0", "-DCONFIG_FREERTOS_UNICORE=0",
                       f"-DCONFIG_FREERTOS_GENERATE_RUN_TIME_STATS={enabled}",
                       f"-DCONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64={enabled}",
                       f"-DCONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER={enabled}"]
            subprocess.run(base + options + [str(unit), "-o", str(executable)], check=True)
            subprocess.run([str(executable)], check=True, timeout=15)
        for macro, rejected in (("CONFIG_FREERTOS_SMP", 1), ("CONFIG_FREERTOS_UNICORE", 1),
                                ("CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64", 0),
                                ("CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER", 0)):
            invalid = [option for option in options if not option.startswith(f"-D{macro}=")]
            invalid += [f"-D{macro}={rejected}"]
            failure = subprocess.run(base + invalid + ["-fsyntax-only", str(unit)], text=True, capture_output=True)
            assert failure.returncode != 0 and "CPU owner observation requires" in failure.stderr
        print("CPUOWNER_CONFIG_GUARD,unsupported_kernels_or_clock=4,expected_compile_rejections=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
