#!/usr/bin/env python3
"""Run the real retained-panic module with fake SDK boundaries, no target I/O."""
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    description = json.loads((ROOT / 'build/project_description.json').read_text())
    sdk = Path(description['idf_path'])
    # Use the pinned SDK's actual panic_info_t, not a hand-written shadow.
    panic_header = sdk / 'components/esp_system/include/esp_private/panic_internal.h'
    context = (sdk / 'components/xtensa/include/xtensa_context.h').read_text()
    frame = context.split('STRUCT_BEGIN\nSTRUCT_FIELD', 1)[1].split('STRUCT_END(XtExcFrame)', 1)[0]
    frame = 'STRUCT_FIELD' + frame
    assert all(re.search(r'\b' + name + r'\)', frame) for name in ('pc', 'a0', 'a1', 'exccause', 'excvaddr'))
    with tempfile.TemporaryDirectory(prefix='sensorarray-panic-host-') as tmp:
        p = Path(tmp)
        (p / 'esp_private').mkdir(); (p / 'soc').mkdir()
        (p / 'esp_private/panic_internal.h').write_bytes(panic_header.read_bytes())
        (p / 'sdkconfig.h').write_text('#pragma once\n#define CONFIG_IDF_TARGET_ESP32S3 1\n')
        (p / 'esp_macros.h').write_text('#pragma once\n')
        (p / 'soc/soc_caps.h').write_text('#pragma once\n#define SOC_CPU_CORES_NUM 2\n')
        (p / 'soc/soc.h').write_text('''#pragma once
#include <stdint.h>
extern uintptr_t testDramLow, testDramHigh;
#define SOC_DRAM_LOW testDramLow
#define SOC_DRAM_HIGH testDramHigh
''')
        (p / 'esp_attr.h').write_text('#pragma once\n#define IRAM_ATTR\n#define DRAM_ATTR\n#define RTC_NOINIT_ATTR\n')
        (p / 'esp_system.h').write_text('''#pragma once
typedef enum { ESP_RST_UNKNOWN, ESP_RST_POWERON, ESP_RST_EXT, ESP_RST_SW,
 ESP_RST_PANIC, ESP_RST_INT_WDT, ESP_RST_TASK_WDT, ESP_RST_WDT,
 ESP_RST_DEEPSLEEP, ESP_RST_BROWNOUT, ESP_RST_SDIO, ESP_RST_USB,
 ESP_RST_JTAG, ESP_RST_EFUSE, ESP_RST_PWR_GLITCH, ESP_RST_CPU_LOCKUP } esp_reset_reason_t;
''')
        (p / 'xtensa_context.h').write_text('''#pragma once
#include <stdint.h>
#define XCHAL_HAVE_LOOPS 1
#define STRUCT_FIELD(ctype,size,asname,name) uint32_t name;
typedef struct {
''' + frame + '\n} XtExcFrame;\n#undef STRUCT_FIELD\n')
        for mode, flags in [('optimized', ['-O2']), ('sanitized', ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie'])]:
            binary = p / ('test-' + mode)
            command = [os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror', *flags,
                       '-I', str(p), '-I', str(ROOT / 'tests/host/include'), '-I', str(ROOT / 'main/control'),
                       str(ROOT / 'tests/host/test_panic_diagnostics.c'), '-o', str(binary)]
            subprocess.run(command, check=True)
            environment = os.environ.copy()
            if mode == 'sanitized':
                # This sandbox uses ptrace; LeakSanitizer cannot enumerate its
                # threads. The module has no allocations. Keep ASan/UBSan on.
                environment['ASAN_OPTIONS'] = 'detect_leaks=0:abort_on_error=1'
                environment['UBSAN_OPTIONS'] = 'halt_on_error=1'
            subprocess.run([str(binary)], check=True, env=environment)
            print('PANIC_HOST_VARIANT_PASS,' + mode, flush=True)
    main_source = (ROOT / 'main/main.c').read_text()
    assert main_source.count('sensorarrayPanicDiagnosticsHandles(command)') == 2
    assert main_source.count('sensorarrayPanicDiagnosticsInit(s_configSessionId, s_bootBreadcrumb.bootId,') == 1
    assert main_source.index('sensorarrayPanicDiagnosticsInit(s_configSessionId') < main_source.index('sensorarrayTransportConfigureWifiBoot(s_configSessionId')
    cmake = (ROOT / 'main/CMakeLists.txt').read_text()
    assert 'control/sensorarrayPanicDiagnostics.c' in cmake and '-Wl,--wrap=esp_panic_handler' in cmake
    print('PANIC_MAIN_QUERY_STARTUP_LINK_HOOK_PASS,target_link_and_IRAM_gate_pending=1')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
