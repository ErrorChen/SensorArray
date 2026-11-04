#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "sensorarrayFdcWorkerLifecycle.h"
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_TIMEOUT 0x107
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_NOT_FOUND 0x105
#define SENSORARRAY_MATRIX_ROWS 8
#define CONFIG_SENSORARRAY_FDC_WORKER_SYNC_TIMEOUT_MS 20
#define CONFIG_SENSORARRAY_FDC_INTB_DIRECT_DATA_ENABLE 0
#define CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG 0
#define configTICK_RATE_HZ 1000u
#define portMAX_DELAY UINT32_MAX
#define pdTRUE 1
#define pdFALSE 0
#define SENSORARRAY_FDC_WORKER_ACK_PRIMARY_BIT 1u
#define SENSORARRAY_FDC_WORKER_ACK_SECONDARY_BIT 2u
#define SENSORARRAY_FDC_WORKER_DONE_PRIMARY_BIT 4u
#define SENSORARRAY_FDC_WORKER_DONE_SECONDARY_BIT 8u
#define SENSORARRAY_FDC_WORKER_ACK_ALL_BITS 3u
#define SENSORARRAY_FDC_WORKER_DONE_ALL_BITS 12u
typedef uint32_t TickType_t;
typedef uint32_t EventBits_t;
typedef int BaseType_t;
typedef enum {SENSORARRAY_FDC_DEV_PRIMARY, SENSORARRAY_FDC_DEV_SECONDARY} sensorarrayFdcDeviceId_t;
typedef struct { bool ready; void *handle; } sensorarrayFdcDeviceState_t;
typedef struct { sensorarrayFdcDeviceState_t devices[2]; } sensorarrayState_t;
typedef struct { uint32_t value; } sensorarrayFdcAutoscanSamples_t;
typedef struct { uint32_t value; } sensorarrayFdcRuntimeChannelConfig_t;
typedef struct { uint32_t value; } sensorarrayFdcReadyState_t;
typedef struct { uint32_t value; } sensorarrayFdcDeviceRead4Result_t;
typedef struct { /* ROW_FIELDS */ } sensorarrayFdcRowTiming_t;
typedef struct { /* DEVICE_FIELDS */ } sensorarrayFdcDeviceTiming_t;
/* FIRMWARE_TYPES */
typedef struct {
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    unsigned notifications;
    bool stop;
    unsigned id;
} TestTask;
typedef struct {
    bool initialized;
    sensorarrayFdcDeviceId_t devId;
    TestTask *task;
    uint32_t generation, preparedEpoch;
    int preparedErr;
    bool rowConfigPrepared;
    sensorarrayFdcWorkerJob_t *job;
    sensorarrayFdcJobLifecycle_t lifecycle;
    esp_err_t acknowledgedError;
    sensorarrayFdcWorkerJob_t persistentJob;
    sensorarrayFdcWorkerResult_t persistentResult;
    sensorarrayFdcWorkerTrace_t persistentTrace;
    sensorarrayFdcAutoscanSamples_t persistentSamples;
    sensorarrayFdcRuntimeChannelConfig_t persistentConfigs[4];
    sensorarrayFdcDeviceTiming_t persistentTiming;
    sensorarrayFdcFrameReadTracker_t persistentReadTracker;
} sensorarrayFdcWorkerContext_t;
static sensorarrayFdcWorkerContext_t s_fdcWorkers[2];
static int s_fdcWorkerEvents;
static _Thread_local TestTask *currentTask;
static pthread_mutex_t eventMutex = PTHREAD_MUTEX_INITIALIZER;
static EventBits_t eventBits;
static int scenario;
static unsigned runCount[2];
static unsigned consumed[2];
static sensorarrayState_t appState;
static int quietPrintf(const char *fmt, ...) { return 0; }
#define printf quietPrintf
static int64_t esp_timer_get_time(void)
{
    struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000000 + now.tv_nsec / 1000;
}
static void delayUs(uint32_t us)
{
    struct timespec delay = {.tv_sec=us / 1000000, .tv_nsec=(long)(us % 1000000) * 1000};
    nanosleep(&delay, NULL);
}
static uint64_t sensorarrayMeasureFdcAbsDeltaUs(uint64_t a, uint64_t b) { return a > b ? a - b : b - a; }
static uint64_t sensorarrayMeasureElapsedUs(int64_t start) { return (uint64_t)(esp_timer_get_time() - start); }
static BaseType_t xTaskNotifyGive(TestTask *task)
{
    pthread_mutex_lock(&task->mutex); task->notifications++; pthread_cond_signal(&task->cond); pthread_mutex_unlock(&task->mutex); return pdTRUE;
}
static uint32_t ulTaskNotifyTake(int clear, TickType_t ticks)
{
    TestTask *task = currentTask;
    pthread_mutex_lock(&task->mutex);
    while (!task->notifications && !task->stop) {
        if (ticks == portMAX_DELAY) { pthread_cond_wait(&task->cond, &task->mutex); }
        else {
            struct timespec until; clock_gettime(CLOCK_REALTIME, &until);
            until.tv_nsec += (long)ticks * 1000000;
            until.tv_sec += until.tv_nsec / 1000000000; until.tv_nsec %= 1000000000;
            if (pthread_cond_timedwait(&task->cond, &task->mutex, &until) != 0) { break; }
        }
    }
    if (task->stop) { pthread_mutex_unlock(&task->mutex); pthread_exit(NULL); }
    unsigned result = task->notifications;
    if (clear) { task->notifications = 0; } else if (result) { task->notifications--; }
    pthread_mutex_unlock(&task->mutex);
    if (result && task->id == 1 && scenario == 1 && consumed[1]++ == 0) { delayUs(90000); }
    if (result && task->id == 1 && scenario == 3 && sensorarrayFdcJobPhase(&s_fdcWorkers[1].lifecycle) == SENSORARRAY_FDC_JOB_RUNNING) { delayUs(90000); }
    return result;
}
static EventBits_t xEventGroupSetBits(int events, EventBits_t bits)
{
    pthread_mutex_lock(&eventMutex);
    if (scenario != 4 || !(bits & SENSORARRAY_FDC_WORKER_DONE_ALL_BITS)) { eventBits |= bits; }
    EventBits_t value = eventBits; pthread_mutex_unlock(&eventMutex); return value;
}
static EventBits_t xEventGroupClearBits(int events, EventBits_t bits)
{
    pthread_mutex_lock(&eventMutex); EventBits_t value = eventBits; eventBits &= ~bits; pthread_mutex_unlock(&eventMutex); return value;
}
static EventBits_t xEventGroupWaitBits(int events, EventBits_t bits, int clear, int all, TickType_t ticks)
{
    delayUs(ticks * 1000);
    pthread_mutex_lock(&eventMutex); EventBits_t result = eventBits;
    if (clear) { eventBits &= ~bits; }
    pthread_mutex_unlock(&eventMutex); return result;
}
static void vTaskDelete(void *task) { abort(); }
static esp_err_t sensorarrayMeasureEnsureFdcWorkers(void) { return ESP_OK; }
static sensorarrayFdcWorkerContext_t *sensorarrayMeasureFdcWorkerContext(unsigned id) { return &s_fdcWorkers[id]; }
static const char *sensorarrayMeasureFdcDeviceName(unsigned id) { return id ? "secondary" : "primary"; }
static EventBits_t sensorarrayMeasureFdcWorkerAckBit(unsigned id) { return 1u << id; }
static EventBits_t sensorarrayMeasureFdcWorkerDoneBit(unsigned id) { return 4u << id; }
static uint32_t sensorarrayMeasureFdcRowDeviceWatchdogHardTimeoutUs(void) { return 20000; }
static sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcState(sensorarrayState_t *state, unsigned id) { return &state->devices[id]; }
static esp_err_t sensorarrayMeasureFdcSetSleepMode(sensorarrayFdcDeviceState_t *state, bool sleep, sensorarrayFdcDeviceTiming_t *timing)
{
    if (currentTask->id == 1 && scenario == 2) { delayUs(90000); }
    return ESP_OK;
}
static void sensorarrayMeasureFdcPrepareIntbEpoch(sensorarrayFdcWorkerContext_t *ctx, uint32_t epoch) { }
static esp_err_t sensorarrayMeasureSelectFdcRowWhileSleeping(sensorarrayState_t *state, uint8_t row, uint32_t epoch, uint32_t seq, sensorarrayFdcRowTiming_t *timing) { return ESP_OK; }
static esp_err_t sensorarrayMeasureFdcPrepareDeviceWhileSleeping(sensorarrayState_t *state, uint8_t row, uint32_t epoch, uint32_t seq, unsigned id, sensorarrayFdcRuntimeChannelConfig_t *config, sensorarrayFdcDeviceTiming_t *timing)
{
    if (scenario == 5) { return ESP_ERR_INVALID_STATE; } /* mode/route failure before RUN */
    config[0].value = epoch; return ESP_OK;
}
static esp_err_t sensorarrayMeasureFdcRunDeviceEpochAfterSleep(sensorarrayState_t *state, uint8_t row, uint32_t epoch, uint32_t seq, unsigned id, sensorarrayFdcAutoscanSamples_t *samples, sensorarrayFdcRuntimeChannelConfig_t *configs, sensorarrayFdcReadyState_t *ready, sensorarrayFdcDeviceRead4Result_t *read4, sensorarrayFdcDeviceTiming_t *timing, sensorarrayFdcFrameReadTracker_t *tracker, const char *reason, uint64_t deadline)
{
    runCount[id]++;
    assert(samples == &s_fdcWorkers[id].persistentSamples);
    assert(configs == s_fdcWorkers[id].persistentConfigs);
    assert(timing == &s_fdcWorkers[id].persistentTiming);
    assert(tracker == &s_fdcWorkers[id].persistentReadTracker);
    assert(state == &appState);
    if (id == 1 && scenario == 6) { delayUs(90000); }
    else { delayUs(1000); }
    samples->value = epoch;
    tracker->okRead[row - 1][id] = true;
    return ESP_OK;
}
static bool sensorarrayMeasureFdcWorkerResultIsGood(const sensorarrayFdcWorkerResult_t *r, uint8_t row, uint32_t epoch, unsigned id) { return r->err == ESP_OK && r->epochId == epoch && r->row == row && r->devId == id; }
static bool sensorarrayMeasureFdcShouldLogNormalFrame(uint32_t seq) { return false; }
static void sensorarrayMeasureFdcDiscardWorkerResult(sensorarrayFdcWorkerResult_t *r, sensorarrayFdcAutoscanSamples_t *s, sensorarrayFdcWorkerTrace_t *t, const char *reason) { s->value = 0; }
static void sensorarrayMeasureFdcDeferDeviceRepairAfterParallel(sensorarrayState_t *state, uint8_t row, uint32_t epoch, uint32_t seq, unsigned id, sensorarrayFdcAutoscanSamples_t *s, const sensorarrayFdcWorkerResult_t *r, sensorarrayFdcRowTiming_t *t, const char *reason) { s->value = 0; }
static void sensorarrayMeasureFdcFinalizeParallelTiming(uint8_t row, uint32_t epoch, sensorarrayFdcDeviceTiming_t *p, sensorarrayFdcDeviceTiming_t *s, sensorarrayFdcRowTiming_t *r, esp_err_t pe, esp_err_t se) { }
