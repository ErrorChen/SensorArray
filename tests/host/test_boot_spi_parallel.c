/* Host shim for actual startup boot owner code. Threads synthesize interleavings;
 * suspend retains a fake task handle, not an actual ESP32 ISR/task lifetime. */
#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_RESPONSE 0x108
#define CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_BUDGET_MS 8000
#define CONFIG_SENSORARRAY_ADS_SPI_PROBE_TASK_STACK 6144
#define CONFIG_SENSORARRAY_MISC_CORE 0
typedef struct { bool adsReady; uint32_t sentinel; } sensorarrayState_t;
typedef void *TaskHandle_t;
typedef int BaseType_t;
typedef pthread_mutex_t portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED PTHREAD_MUTEX_INITIALIZER
#define portENTER_CRITICAL(p) assert(pthread_mutex_lock(p) == 0)
#define portEXIT_CRITICAL(p) assert(pthread_mutex_unlock(p) == 0)
#define pdPASS 1

/* PRODUCTION_RESULT_TYPES */

static atomic_uint_fast64_t mockNowUs;
static _Thread_local int mockCore = 1;
static int64_t esp_timer_get_time(void) { return (int64_t)atomic_load(&mockNowUs); }
static int xPortGetCoreID(void) { return mockCore; }
static void vTaskDelay(unsigned ticks)
{
    assert(ticks == 1u);
    atomic_fetch_add(&mockNowUs, 10000u);
    sched_yield();
}
#if CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE
static pthread_mutex_t mockMux = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t mockCond = PTHREAD_COND_INITIALIZER;
static pthread_t mockThread;
static bool mockThreadActive;
static bool mockCreateFail, mockAllowStart, mockAllowReturn, mockEntered;
static unsigned mockCreates, mockRuns;
static uint64_t mockDeadline;
static void (*mockEntry)(void *);
static void *mockEntryArg;
static sensorarrayAdsSpiProbeResult_t mockResult;
static esp_err_t mockError;
static sensorarrayState_t mockState = { true, 0x1234u };
static void mockWait(bool *condition)
{
    struct timespec until;
    assert(clock_gettime(CLOCK_REALTIME, &until) == 0);
    until.tv_sec += 2;
    while (!*condition) {
        assert(pthread_cond_timedwait(&mockCond, &mockMux, &until) == 0);
    }
}
static void *mockTaskWrapper(void *arg)
{
    (void)arg;
    mockCore = 0;
    assert(pthread_mutex_lock(&mockMux) == 0);
    mockWait(&mockAllowStart);
    assert(pthread_mutex_unlock(&mockMux) == 0);
    mockEntry(mockEntryArg);
    assert(!"production worker must park, never return");
    return NULL;
}
static BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *), const char *name,
    unsigned stack, void *arg, unsigned priority, TaskHandle_t *out, int core)
{
    assert(strcmp(name, "ads_spi_boot") == 0 && stack == 6144u);
    assert(priority == 8u && core == 0);
    mockCreates++;
    if (mockCreateFail) { return 0; }
    mockEntry = entry;
    mockEntryArg = arg;
    *out = &mockThread;
    mockThreadActive = true;
    assert(pthread_create(&mockThread, NULL, mockTaskWrapper, NULL) == 0);
    return pdPASS;
}
static unsigned uxTaskGetStackHighWaterMark(TaskHandle_t task)
{
    assert(task == NULL && mockCore == 0);
    return 2048u;
}
static void vTaskSuspend(TaskHandle_t task)
{
    assert(task == NULL && mockCore == 0);
    pthread_exit(NULL); /* The fake task handle above remains stable. */
}
static esp_err_t sensorarrayAdsSpiProbeRun(sensorarrayState_t *state,
    uint64_t deadlineUs, sensorarrayAdsSpiProbeResult_t *out);
#endif

/* PRODUCTION_BOOT_JOB */

#if CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE
static esp_err_t sensorarrayAdsSpiProbeRun(sensorarrayState_t *state,
    uint64_t deadlineUs, sensorarrayAdsSpiProbeResult_t *out)
{
    assert(state == &mockState && state->sentinel == 0x1234u);
    assert(out == &s_bootSpiJob.workResult);
    assert(pthread_mutex_lock(&mockMux) == 0);
    mockRuns++;
    mockDeadline = deadlineUs;
    /* A partial worker write is intentionally invalid as a public snapshot. */
    out->qualified = true;
    out->candidatesTested = 999u;
    mockEntered = true;
    assert(pthread_cond_broadcast(&mockCond) == 0);
    mockWait(&mockAllowReturn);
    *out = mockResult;
    esp_err_t err = mockError;
    assert(pthread_mutex_unlock(&mockMux) == 0);
    return err;
}
static void resetBoot(void)
{
    assert(!mockThreadActive);
    memset(&s_bootSpiJob, 0, sizeof(s_bootSpiJob));
    mockCreateFail = false;
    mockAllowStart = true;
    mockAllowReturn = false;
    mockEntered = false;
    mockCreates = mockRuns = 0u;
    mockDeadline = 0;
    mockError = ESP_OK;
    mockResult = (sensorarrayAdsSpiProbeResult_t) {
        .qualified = true, .restored = true,
        .selectedHz = 2000000, .actualHz = 2000000,
        .candidatesTested = 1, .validSamples = 1280,
        .crcValidEven = 640, .crcValidOdd = 640,
    };
    atomic_store(&mockNowUs, 1000u);
}
static void waitEntered(void)
{
    assert(pthread_mutex_lock(&mockMux) == 0);
    mockWait(&mockEntered);
    assert(pthread_mutex_unlock(&mockMux) == 0);
}
static void finishWorker(uint64_t endUs)
{
    atomic_store(&mockNowUs, endUs);
    assert(pthread_mutex_lock(&mockMux) == 0);
    mockAllowStart = mockAllowReturn = true;
    assert(pthread_cond_broadcast(&mockCond) == 0);
    assert(pthread_mutex_unlock(&mockMux) == 0);
    assert(pthread_join(mockThread, NULL) == 0);
    mockThreadActive = false;
    assert(sensorarrayBootSpiCopy().phase == SENSORARRAY_BOOT_SPI_DONE);
}
static void startWorker(void)
{
    assert(sensorarrayBootSpiStart(&mockState) == ESP_OK);
    assert(mockCreates == 1u);
    waitEntered();
    assert(!sensorarrayFrontendStartupAdsOwnerQuiescent());
    assert(mockDeadline == 8001000u);
    assert(sensorarrayBootSpiStart(&mockState) == ESP_ERR_INVALID_STATE);
}
static void testCreateFailure(void)
{
    resetBoot();
    mockCreateFail = true;
    assert(sensorarrayBootSpiStart(&mockState) == ESP_ERR_NO_MEM);
    assert(sensorarrayBootSpiJoin() == ESP_ERR_NO_MEM);
    assert(sensorarrayBootSpiCopy().phase == SENSORARRAY_BOOT_SPI_CREATE_FAILED);
    assert(mockCreates == 1u && mockRuns == 0u);
}
static void testOverlapAndLateI2c(void)
{
    resetBoot();
    startWorker();
    atomic_store(&mockNowUs, 2000u);
    sensorarrayBootSpiNoteI2c(true);
    finishWorker(100000u);
    atomic_store(&mockNowUs, 20000000u); /* I2C outlasts the SPI budget. */
    sensorarrayBootSpiNoteI2c(false);
    assert(sensorarrayBootSpiJoin() == ESP_OK);
    sensorarrayBootSpiSnapshot_t s = sensorarrayBootSpiCopy();
    assert(s.overlapUs == 98000u && s.joinedUs == 20000000u);
    assert(s.result.qualified && s.result.restored && !s.timedOut);
    assert(sensorarrayFrontendStartupAdsOwnerQuiescent());
    assert(s.spiCore == 0 && s.i2cCore == 1 && s.stackMinBytes == 2048u);
    assert(sensorarrayBootSpiStart(&mockState) == ESP_ERR_INVALID_STATE);
    sensorarrayBootSpiLogJoin(ESP_OK);
}
static void testActiveTimeoutAndLateDone(void)
{
    resetBoot();
    startWorker();
    sensorarrayBootSpiSnapshot_t s = sensorarrayBootSpiCopy();
    assert(s.phase == SENSORARRAY_BOOT_SPI_RUNNING);
    assert(!s.result.qualified && s.result.candidatesTested == 0u);
    char reply[512];
    assert(sensorarrayFrontendStartupFormatSpi(reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "done=0") && strstr(reply, "qualified=0"));
    /* Let production Join consume one-tick waits through its finite deadline. */
    assert(sensorarrayBootSpiJoin() == ESP_ERR_TIMEOUT);
    s = sensorarrayBootSpiCopy();
    assert(s.timedOut && s.phase == SENSORARRAY_BOOT_SPI_RUNNING);
    assert(mockState.sentinel == 0x1234u && mockThreadActive);
    finishWorker(s.deadlineUs + 100000u);
    assert(sensorarrayBootSpiJoin() == ESP_ERR_TIMEOUT);
    s = sensorarrayBootSpiCopy();
    assert(s.timedOut && s.result.qualified && s.joinError == ESP_ERR_TIMEOUT);
    assert(sensorarrayFrontendStartupAdsOwnerQuiescent()); /* Released, still failed. */
    assert(sensorarrayFrontendStartupFormatSpi(reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "timedOut=1") && strstr(reply, "err=0x107"));
}
static void testUnconsumedPost(void)
{
    resetBoot();
    mockAllowStart = false;
    assert(sensorarrayBootSpiStart(&mockState) == ESP_OK);
    assert(sensorarrayBootSpiJoin() == ESP_ERR_TIMEOUT);
    assert(sensorarrayBootSpiCopy().phase == SENSORARRAY_BOOT_SPI_POSTED);
    assert(!sensorarrayFrontendStartupAdsOwnerQuiescent());
    assert(mockRuns == 0u);
    finishWorker(9001000u);
    assert(mockDeadline == 8001000u && mockRuns == 1u);
    assert(sensorarrayBootSpiJoin() == ESP_ERR_TIMEOUT);
}
static void testFailureAndRestore(void)
{
    resetBoot();
    mockError = ESP_FAIL;
    mockResult.error = ESP_FAIL;
    mockResult.cleanupError = ESP_ERR_INVALID_STATE;
    mockResult.qualified = mockResult.restored = false;
    startWorker();
    finishWorker(100000u);
    assert(sensorarrayBootSpiJoin() == ESP_FAIL);
    assert(sensorarrayBootSpiCopy().result.cleanupError == ESP_ERR_INVALID_STATE);
    for (unsigned which = 0; which < 4; ++which) {
        resetBoot();
        if (which == 0) { mockResult.qualified = false; }
        if (which == 1) { mockResult.restored = false; }
        if (which == 2) { mockResult.selectedHz = 0; }
        if (which == 3) { mockResult.actualHz = 0; }
        startWorker();
        finishWorker(100000u);
        assert(sensorarrayBootSpiJoin() == ESP_ERR_INVALID_RESPONSE);
    }
    resetBoot();
    startWorker();
    finishWorker(9000000u); /* Even a claimed success cannot exceed the budget. */
    assert(sensorarrayBootSpiJoin() == ESP_ERR_TIMEOUT);
}
#endif

static void testQueryBound(void)
{
    sensorarrayBootSpiSnapshot_t *p = &s_bootSpiJob.published;
    p->phase = SENSORARRAY_BOOT_SPI_CREATE_FAILED;
    p->result.selectedHz = p->result.actualHz = UINT32_MAX;
    p->result.candidatesTested = p->result.validSamples = UINT32_MAX;
    p->result.attemptedSamples = UINT32_MAX;
    p->result.crcValidEven = p->result.crcValidOdd = UINT32_MAX;
    p->result.crcErrors = p->result.statusErrors = UINT32_MAX;
    p->result.registerErrors = p->result.drdyTimeouts = UINT32_MAX;
    p->joinError = p->runError = p->result.cleanupError = -1;
    p->stackMinBytes = UINT32_MAX;
    p->startedUs = p->endedUs = p->overlapUs = UINT64_MAX;
    char reply[512];
    assert(sensorarrayFrontendStartupFormatSpi(reply, sizeof(reply)) == ESP_OK);
    printf("SPI_QUERY_MAX,length=%lu,capacity=512\n", (unsigned long)strlen(reply));
    assert(strlen(reply) < sizeof(reply) && reply[strlen(reply) - 1] == '\n');
    assert(sensorarrayFrontendStartupFormatSpi(reply, 16) == ESP_ERR_INVALID_SIZE);
}
int main(void)
{
#if CONFIG_SENSORARRAY_ADS_SPI_AUTOTUNE_ENABLE
    testCreateFailure();
    testOverlapAndLateI2c();
    testActiveTimeoutAndLateDone();
    testUnconsumedPost();
    testFailureAndRestore();
#else
    sensorarrayState_t state = { .adsReady = true };
    atomic_store(&mockNowUs, 1000u);
    assert(sensorarrayBootSpiStart(&state) == ESP_OK);
    sensorarrayBootSpiNoteI2c(true);
    sensorarrayBootSpiNoteI2c(false);
    assert(sensorarrayBootSpiJoin() == ESP_OK);
    assert(sensorarrayBootSpiCopy().phase == SENSORARRAY_BOOT_SPI_SKIPPED);
    assert(sensorarrayFrontendStartupAdsOwnerQuiescent());
    assert(!sensorarrayBootSpiCopy().result.qualified);
    sensorarrayBootSpiLogJoin(ESP_OK);
#endif
    testQueryBound();
    puts("boot SPI production lifetime/deadline/query: PASS");
    return 0;
}
