#include "sensorarrayMemoryDiagnostics.h"
#include <stdbool.h>
#include <stdio.h>
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sensorarrayTransport.h"

typedef struct {
    bool begun;
    bool completed;
    uint32_t stackBytes;
    uint32_t beforeFree;
    uint32_t beforeLargest;
    uint32_t afterFree;
    uint32_t afterLargest;
    esp_err_t result;
} sensorarrayScanMemory_t;

static sensorarrayScanMemory_t s_scanMemory;
static portMUX_TYPE s_memoryMux = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    size_t requestedSize;
    uint32_t caps;
    uintptr_t functionName;
} sensorarrayAllocationFailure_t;

typedef struct {
    uint64_t sessionId;
    uint32_t bootId;
    bool identityKnown;
    bool elfKnown;
    char elfSha256[65];
    bool attempted;
    bool registrationKnown;
    esp_err_t registrationError;
    uint32_t count;
    bool saturated;
    sensorarrayAllocationFailure_t first;
    sensorarrayAllocationFailure_t last;
} sensorarrayAllocationFailures_t;

/* This observer can run in the allocating caller's ISR/cache-disabled
 * context. No payload, pointer target or metadata lives in PSRAM. The only
 * callback calls are the IRAM port's nonwaiting lock attempt/release. */
static DRAM_ATTR sensorarrayAllocationFailures_t s_allocationFailures;
static DRAM_ATTR portMUX_TYPE s_allocationFailureMux = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR _Alignas(4) uint32_t s_allocationCaptureIncomplete;
_Static_assert(sizeof(sensorarrayAllocationFailures_t) <= 192u,
    "Failure snapshot must remain fixed and small");
/* Pinned Xtensa GCC reports always_lock_free(4,0)==false under its selected
 * flags, although these aligned relaxed load/store operations compile to
 * memw + l32i/s32i without a helper. Keep C atomic access (no RMW), assert the
 * storage requirement, and require the actual target disassembly gate; an
 * RMW or library-based replacement is not valid in this no-wait path. */
_Static_assert(sizeof(s_allocationCaptureIncomplete) == 4u &&
    __alignof__(s_allocationCaptureIncomplete) >= 4u,
    "Failure-path incomplete marker requires one aligned 32-bit word");

static void IRAM_ATTR __attribute__((noinline, no_stack_protector))
sensorarrayMemoryAllocationFailed(size_t requestedSize, uint32_t caps,
    const char *functionName)
{
    if (!portTRY_ENTER_CRITICAL_ISR(&s_allocationFailureMux, 0)) {
        /* One aligned atomic store, no retry loop. Lost observations are
         * explicit; a failure path must never wait for another core. */
        __atomic_store_n(&s_allocationCaptureIncomplete, 1u, __ATOMIC_RELAXED);
        return;
    }
    if (s_allocationFailures.count == 0u) {
        s_allocationFailures.first.requestedSize = requestedSize;
        s_allocationFailures.first.caps = caps;
        s_allocationFailures.first.functionName = (uintptr_t)functionName;
    }
    s_allocationFailures.last.requestedSize = requestedSize;
    s_allocationFailures.last.caps = caps;
    s_allocationFailures.last.functionName = (uintptr_t)functionName;
    if (s_allocationFailures.count != UINT32_MAX) {
        s_allocationFailures.count++;
    } else {
        s_allocationFailures.saturated = true;
    }
    portEXIT_CRITICAL_ISR(&s_allocationFailureMux);
}

esp_err_t sensorarrayMemoryDiagnosticsInit(uint64_t sessionId, uint32_t bootId,
    const uint8_t elfSha256[32])
{
    if (!portTRY_ENTER_CRITICAL_ISR(&s_allocationFailureMux, 0)) {
        return ESP_ERR_TIMEOUT;
    }
    if (s_allocationFailures.attempted) {
        esp_err_t result = s_allocationFailures.registrationKnown ?
            s_allocationFailures.registrationError : ESP_ERR_INVALID_STATE;
        portEXIT_CRITICAL_ISR(&s_allocationFailureMux);
        return result;
    }
    s_allocationFailures.attempted = true;
    s_allocationFailures.sessionId = sessionId;
    s_allocationFailures.bootId = bootId;
    s_allocationFailures.identityKnown = sessionId != 0u && bootId != 0u;
    if (elfSha256) {
        static const char hex[] = "0123456789abcdef";
        for (unsigned i = 0u; i < 32u; ++i) {
            s_allocationFailures.elfSha256[2u * i] = hex[elfSha256[i] >> 4];
            s_allocationFailures.elfSha256[2u * i + 1u] = hex[elfSha256[i] & 15u];
        }
        s_allocationFailures.elfSha256[64] = '\0';
        s_allocationFailures.elfKnown = true;
    }
    portEXIT_CRITICAL_ISR(&s_allocationFailureMux);
    /* No allocation in the pinned SDK's registration API. Do not hold the
     * observer lock while calling it: a callback may start before it returns. */
    esp_err_t result = heap_caps_register_failed_alloc_callback(sensorarrayMemoryAllocationFailed);
    portENTER_CRITICAL(&s_allocationFailureMux);
    s_allocationFailures.registrationError = result;
    s_allocationFailures.registrationKnown = true;
    portEXIT_CRITICAL(&s_allocationFailureMux);
    return result;
}

esp_err_t sensorarrayMemoryDiagnosticsFailureFormat(char *response, size_t size)
{
    if (!response || size == 0u) { return ESP_ERR_INVALID_ARG; }
    if (!portTRY_ENTER_CRITICAL_ISR(&s_allocationFailureMux, 0)) {
        response[0] = '\0';
        return ESP_ERR_TIMEOUT;
    }
    sensorarrayAllocationFailures_t snapshot = s_allocationFailures;
    bool incomplete = __atomic_load_n(&s_allocationCaptureIncomplete, __ATOMIC_RELAXED) != 0u;
    portEXIT_CRITICAL_ISR(&s_allocationFailureMux);
    /* API names are opaque addresses, never dereferenced even here. A name
     * identifies the allocator API, not the Wi-Fi caller. For *_prefer, the
     * SDK reports only ultimate failure and the last attempted caps. Sizes
     * exclude alignment/poison overhead. First/last/count cover only recorded
     * callbacks since registration; try-lock misses mark incomplete capture. */
    char firstSize[24], lastSize[24], firstCaps[16], lastCaps[16];
    char firstFunction[24], lastFunction[24], registrationError[16];
    const char *fs = "unknown", *ls = "unknown", *fc = "unknown", *lc = "unknown";
    const char *ff = "unknown", *lf = "unknown", *err = "unknown";
    bool known = snapshot.count != 0u;
    if (known) {
        (void)snprintf(firstSize, sizeof firstSize, "%llu", (unsigned long long)snapshot.first.requestedSize);
        (void)snprintf(lastSize, sizeof lastSize, "%llu", (unsigned long long)snapshot.last.requestedSize);
        (void)snprintf(firstCaps, sizeof firstCaps, "0x%lX", (unsigned long)snapshot.first.caps);
        (void)snprintf(lastCaps, sizeof lastCaps, "0x%lX", (unsigned long)snapshot.last.caps);
        fs = firstSize; ls = lastSize; fc = firstCaps; lc = lastCaps;
        if (snapshot.first.functionName) {
            (void)snprintf(firstFunction, sizeof firstFunction, "0x%llX", (unsigned long long)snapshot.first.functionName);
            ff = firstFunction;
        }
        if (snapshot.last.functionName) {
            (void)snprintf(lastFunction, sizeof lastFunction, "0x%llX", (unsigned long long)snapshot.last.functionName);
            lf = lastFunction;
        }
    }
    if (snapshot.registrationKnown) {
        (void)snprintf(registrationError, sizeof registrationError, "0x%lX",
            (unsigned long)(uint32_t)snapshot.registrationError);
        err = registrationError;
    }
    int n = snprintf(response, size,
        "MEMFAIL,v=1,scope=boot,session=%016llx,bootId=%lu,identityKnown=%u,elfSha256=%s,"
        "attempted=%u,regKnown=%u,registered=%u,regErr=%s,known=%u,count=%lu,saturated=%u,incomplete=%u,"
        "firstSize=%s,firstCaps=%s,firstFn=%s,lastSize=%s,lastCaps=%s,lastFn=%s,"
        "fn=opaque_pointer,countScope=recorded,coverage=since_register\n",
        (unsigned long long)snapshot.sessionId, (unsigned long)snapshot.bootId,
        snapshot.identityKnown ? 1u : 0u, snapshot.elfKnown ? snapshot.elfSha256 : "unknown",
        snapshot.attempted ? 1u : 0u, snapshot.registrationKnown ? 1u : 0u,
        snapshot.registrationKnown && snapshot.registrationError == ESP_OK ? 1u : 0u,
        err, known ? 1u : 0u, (unsigned long)snapshot.count, snapshot.saturated ? 1u : 0u,
        incomplete ? 1u : 0u, fs, fc, ff, ls, lc, lf);
    if (n < 0 || (size_t)n >= size) {
        response[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

void sensorarrayMemoryDiagnosticsScanBegin(uint32_t stackBytes)
{
    multi_heap_info_t info = {0};
    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    portENTER_CRITICAL(&s_memoryMux);
    s_scanMemory = (sensorarrayScanMemory_t){.begun = true,
        .stackBytes = stackBytes, .beforeFree = info.total_free_bytes,
        .beforeLargest = info.largest_free_block};
    portEXIT_CRITICAL(&s_memoryMux);
}

void sensorarrayMemoryDiagnosticsScanEnd(esp_err_t result)
{
    multi_heap_info_t info = {0};
    heap_caps_get_info(&info, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    portENTER_CRITICAL(&s_memoryMux);
    if (s_scanMemory.begun) {
        s_scanMemory.afterFree = info.total_free_bytes;
        s_scanMemory.afterLargest = info.largest_free_block;
        s_scanMemory.result = result;
        s_scanMemory.completed = true;
    }
    portEXIT_CRITICAL(&s_memoryMux);
}

static const char *sensorarrayMemoryNumber(bool known, uint32_t value,
                                          char text[16])
{
    if (!known) { return "unknown"; }
    (void)snprintf(text, 16u, "%lu", (unsigned long)value);
    return text;
}

esp_err_t sensorarrayMemoryDiagnosticsFormat(char *response, size_t size)
{
    if (!response || size == 0u) { return ESP_ERR_INVALID_ARG; }
    sensorarrayScanMemory_t scan;
    portENTER_CRITICAL(&s_memoryMux);
    scan = s_scanMemory;
    portEXIT_CRITICAL(&s_memoryMux);
    /* Heap traversal/formatting stays outside the snapshot lock. A concurrent
     * task may allocate between observations; no delta is claimed as the
     * scan task's exclusive allocation, and no PSRAM total proves DMA/stack
     * availability. The normal FreeRTOS task requires internal 8-bit memory. */
    multi_heap_info_t internal = {0}, external = {0};
    heap_caps_get_info(&internal, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    heap_caps_get_info(&external, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    sensorarrayTransportMemoryDiagnostics_t transport = {0};
    sensorarrayTransportGetMemoryDiagnostics(&transport);
    char numbers[5][16], error[16];
    const char *state = !scan.begun ? "unknown" : !scan.completed ? "pending" :
                        scan.result == ESP_OK ? "created" : "failed";
    const char *err = "unknown";
    if (scan.completed) {
        (void)snprintf(error, sizeof error, "0x%lX", (unsigned long)(uint32_t)scan.result);
        err = error;
    }
    int n = snprintf(response, size,
        "MEM,v=1,readout=sequential,unit=bytes,intCaps=internal8,intFree=%lu,intLargest=%lu,intMin=%lu,"
        "extFree=%lu,extLargest=%lu,scan=%s,scanStack=%s,beforeFree=%s,beforeLargest=%s,"
        "afterFree=%s,afterLargest=%s,err=%s,sampleUs=%llu,"
        "poolReady=%u,poolErr=0x%lX,poolInt=%lu,poolExt=%lu\n",
        (unsigned long)internal.total_free_bytes, (unsigned long)internal.largest_free_block,
        (unsigned long)internal.minimum_free_bytes, (unsigned long)external.total_free_bytes,
        (unsigned long)external.largest_free_block, state,
        sensorarrayMemoryNumber(scan.begun, scan.stackBytes, numbers[0]),
        sensorarrayMemoryNumber(scan.begun, scan.beforeFree, numbers[1]),
        sensorarrayMemoryNumber(scan.begun, scan.beforeLargest, numbers[2]),
        sensorarrayMemoryNumber(scan.completed, scan.afterFree, numbers[3]),
        sensorarrayMemoryNumber(scan.completed, scan.afterLargest, numbers[4]),
        err, (unsigned long long)esp_timer_get_time(),
        transport.payloadStorageReady ? 1u : 0u,
        (unsigned long)(uint32_t)transport.payloadStorageError,
        (unsigned long)transport.payloadInternalBytes,
        (unsigned long)transport.payloadExternalBytes);
    if (n < 0 || (size_t)n >= size) {
        response[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}
