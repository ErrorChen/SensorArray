#!/usr/bin/env python3
"""Compile the actual low-frequency memory formatter with synthetic heaps."""
import os
import re
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def main():
    with tempfile.TemporaryDirectory(prefix='sensorarray-memory-diagnostics-') as tmp:
        p = Path(tmp)
        (p / 'freertos').mkdir()
        (p / 'freertos/FreeRTOS.h').write_text('''#pragma once
#include <assert.h>
#include <pthread.h>
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
extern _Thread_local unsigned lockDepth;
static inline void memoryTestEnter(portMUX_TYPE *m) { assert(!lockDepth); assert(!pthread_mutex_lock(m)); lockDepth++; }
static inline int memoryTestTry(portMUX_TYPE *m) { if(pthread_mutex_trylock(m))return 0; assert(!lockDepth); lockDepth++; return 1; }
static inline void memoryTestExit(portMUX_TYPE *m) { assert(lockDepth==1);lockDepth--;assert(!pthread_mutex_unlock(m)); }
#define portENTER_CRITICAL(m) memoryTestEnter(m)
#define portEXIT_CRITICAL(m) memoryTestExit(m)
#define portTRY_ENTER_CRITICAL_ISR(m,t) ((void)(t),memoryTestTry(m))
#define portEXIT_CRITICAL_ISR(m) memoryTestExit(m)
''')
        (p / 'esp_heap_caps.h').write_text('''#pragma once
#include <stddef.h>
#define MALLOC_CAP_INTERNAL 1u
#define MALLOC_CAP_8BIT 2u
#define MALLOC_CAP_SPIRAM 4u
typedef struct {size_t total_free_bytes, largest_free_block, minimum_free_bytes;} multi_heap_info_t;
#include <stdint.h>
#include "esp_err.h"
typedef void (*esp_alloc_failed_hook_t)(size_t,uint32_t,const char *);
esp_err_t heap_caps_register_failed_alloc_callback(esp_alloc_failed_hook_t);
void heap_caps_get_info(multi_heap_info_t *, unsigned);
''')
        (p / 'esp_attr.h').write_text('#pragma once\n#define IRAM_ATTR\n#define DRAM_ATTR\n')
        (p / 'esp_timer.h').write_text('#pragma once\n#include <stdint.h>\nint64_t esp_timer_get_time(void);\n')
        header = (ROOT / 'core/transport/sensorarrayTransport.h').read_text()
        pod = re.search(r'typedef\s+struct\s*\{[^{}]*\}\s*sensorarrayTransportMemoryDiagnostics_t\s*;', header)
        assert pod, 'actual Transport memory POD required'
        (p / 'sensorarrayTransport.h').write_text('#pragma once\n#include <stdint.h>\n#include <stdbool.h>\n#include "esp_err.h"\n' +
            pod.group(0) + '\nvoid sensorarrayTransportGetMemoryDiagnostics(sensorarrayTransportMemoryDiagnostics_t *);\n')
        (p / 'test.c').write_text(r'''
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayMemoryDiagnostics.h"
#include "esp_heap_caps.h"
#include "sensorarrayTransport.h"
_Thread_local unsigned lockDepth;
static unsigned calls;
esp_err_t heap_caps_register_failed_alloc_callback(esp_alloc_failed_hook_t callback) { assert(callback && !lockDepth); return ESP_OK; }
static multi_heap_info_t internal = {22000, 18000, 20000};
static multi_heap_info_t external = {8000000, 7900000, 7800000};
static sensorarrayTransportMemoryDiagnostics_t transport;
void sensorarrayTransportGetMemoryDiagnostics(sensorarrayTransportMemoryDiagnostics_t *out) {
    assert(!lockDepth); *out = transport;
}
void heap_caps_get_info(multi_heap_info_t *out, unsigned caps) {
    assert(!lockDepth);
    assert(caps == (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT) ||
           caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    *out = caps & MALLOC_CAP_INTERNAL ? internal : external;
    calls++;
}
int64_t esp_timer_get_time(void) { assert(!lockDepth); return INT64_MAX; }
int main(void) {
    char response[520];
    assert(sensorarrayMemoryDiagnosticsFormat(NULL,512) == ESP_ERR_INVALID_ARG);
    assert(sensorarrayMemoryDiagnosticsFormat(response,0) == ESP_ERR_INVALID_ARG);
    assert(!calls);
    sensorarrayMemoryDiagnosticsScanEnd(ESP_OK); /* Cannot invent a begin. */
    assert(sensorarrayMemoryDiagnosticsFormat(response,512) == ESP_OK);
    assert(strstr(response,"scan=unknown,scanStack=unknown,beforeFree=unknown,beforeLargest=unknown,afterFree=unknown,afterLargest=unknown,err=unknown,"));
    sensorarrayMemoryDiagnosticsScanBegin(16384);
    assert(sensorarrayMemoryDiagnosticsFormat(response,512) == ESP_OK);
    assert(strstr(response,"scan=pending,scanStack=16384,beforeFree=22000,beforeLargest=18000,afterFree=unknown,"));
    internal.total_free_bytes=13000; internal.largest_free_block=8000;
    sensorarrayMemoryDiagnosticsScanEnd(ESP_ERR_NO_MEM);
    assert(sensorarrayMemoryDiagnosticsFormat(response,512) == ESP_OK);
    assert(strstr(response,"scan=failed,scanStack=16384,beforeFree=22000,beforeLargest=18000,afterFree=13000,afterLargest=8000,err=0x101,"));
    assert(strstr(response,"intCaps=internal8,intFree=13000,intLargest=8000,"));
    assert(strstr(response,"extFree=8000000,extLargest=7900000,"));
    transport.payloadStorageReady = false;
    transport.payloadStorageError = ESP_ERR_NO_MEM;
    transport.payloadInternalBytes = 1120;
    transport.payloadExternalBytes = 0;
    assert(sensorarrayMemoryDiagnosticsFormat(response,512) == ESP_OK);
    assert(strstr(response,"poolReady=0,poolErr=0x101,poolInt=1120,poolExt=0\n"));
    transport.payloadStorageReady = true;
    transport.payloadStorageError = ESP_OK;
    transport.payloadExternalBytes = 16135;
    assert(sensorarrayMemoryDiagnosticsFormat(response,512) == ESP_OK);
    assert(strstr(response,"poolReady=1,poolErr=0x0,poolInt=1120,poolExt=16135\n"));
    internal=(multi_heap_info_t){UINT32_MAX,UINT32_MAX,UINT32_MAX}; external=internal;
    transport.payloadStorageError = (esp_err_t)UINT32_MAX;
    transport.payloadInternalBytes = transport.payloadExternalBytes = UINT32_MAX;
    sensorarrayMemoryDiagnosticsScanBegin(UINT32_MAX);
    sensorarrayMemoryDiagnosticsScanEnd((esp_err_t)UINT32_MAX);
    assert(sensorarrayMemoryDiagnosticsFormat(response,512) == ESP_OK);
    assert(strstr(response,"err=0xFFFFFFFF,sampleUs=9223372036854775807,"));
    assert(strstr(response,"poolReady=1,poolErr=0xFFFFFFFF,poolInt=4294967295,poolExt=4294967295\n"));
    size_t needed=strlen(response)+1u;
    printf("MEM max_bytes=%zu capacity=512 synthetic_heaps=1\n",needed-1u);
    assert(needed<=512);
    for (size_t capacity=1;capacity<needed;capacity++) {
        memset(response,0x5a,sizeof response);
        assert(sensorarrayMemoryDiagnosticsFormat(response,capacity) == ESP_ERR_INVALID_SIZE);
        assert(response[0]=='\0' && response[capacity]==0x5a && !lockDepth);
    }
    assert(sensorarrayMemoryDiagnosticsFormat(response,needed) == ESP_OK);
    sensorarrayMemoryDiagnosticsScanBegin(16384);
    sensorarrayMemoryDiagnosticsScanEnd(ESP_OK);
    assert(sensorarrayMemoryDiagnosticsFormat(response,512) == ESP_OK);
    assert(strstr(response,"scan=created,scanStack=16384,"));
    puts("PASS memory caps, allocation bracket, unknown states, readonly lock exclusion and bounds");
}
''')
        binary = p / 'test'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror', '-pthread',
            '-I', str(p), '-I', str(ROOT / 'tests/host/include'), '-I', str(ROOT / 'main/control'),
            str(ROOT / 'main/control/sensorarrayMemoryDiagnostics.c'), str(p / 'test.c'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        for optimize in (True, False):
            failure_binary = p / ('failures_opt' if optimize else 'failures_ubsan')
            flags = ['-O2'] if optimize else ['-O1', '-g', '-fsanitize=undefined', '-fno-omit-frame-pointer']
            subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', *flags, '-Wall', '-Wextra', '-Werror', '-pthread',
                '-I', str(p), '-I', str(ROOT / 'tests/host/include'), '-I', str(ROOT / 'main/control'),
                str(ROOT / 'tests/host/test_memory_failures.c'), '-o', str(failure_binary)], check=True)
            subprocess.run([str(failure_binary)], check=True, timeout=15)
        main_source = (ROOT / 'main/main.c').read_text()
        assert main_source.count('sensorarrayMemoryDiagnosticsFailureFormat(response, responseSize)') == 2
        assert main_source.count('sensorarrayMemoryDiagnosticsInit(s_configSessionId, s_bootBreadcrumb.bootId,') == 1
        assert main_source.index('sensorarrayMemoryDiagnosticsInit(s_configSessionId') < main_source.index('sensorarrayTransportConfigureWifiBoot(s_configSessionId')
        print('MEMFAIL_BOOT_HOOKS,early_and_runtime=2,registration_before_network=1,production_injection_command=0')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
