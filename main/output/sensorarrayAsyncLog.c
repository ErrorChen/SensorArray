#include "sensorarrayAsyncLog.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayFrameOutput.h"
#include "sensorarrayTypes.h"

#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_ENABLE
#define CONFIG_SENSORARRAY_ASYNC_LOG_ENABLE 1
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS
#define CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS 4
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_LEN
#define CONFIG_SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_LEN 32
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_SUMMARY_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_ASYNC_LOG_SUMMARY_EVERY_N_FRAMES 20
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK
#define CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK 12288
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY
#define CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY 7
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE
#define CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE CONFIG_SENSORARRAY_COMM_TASK_CORE
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES
#define CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES 1
#endif

enum {
    SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT =
        (CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS < 2) ? 2 : CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS,
    SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT =
        (CONFIG_SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_LEN < 1) ? 1 : CONFIG_SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_LEN,
};

typedef enum {
    SENSORARRAY_ASYNC_LOG_EVENT_OVERRUN = 0,
    SENSORARRAY_ASYNC_LOG_EVENT_FRAME_ERROR,
} sensorarrayAsyncLogEventType_t;

typedef struct {
    sensorarrayFrame_t frame;
    uint64_t measureFrameUs;
    int64_t publishedUs;
} sensorarrayAsyncLogFrameSlot_t;

typedef struct {
    sensorarrayAsyncLogEventType_t type;
    uint32_t sequence;
    union {
        struct {
            int64_t elapsedUs;
            int64_t periodUs;
        } overrun;
        struct {
            esp_err_t readErr;
            uint64_t capValidMask;
            uint64_t errorMask;
            uint64_t warnMask;
            uint8_t validCount;
            uint8_t freshCount;
            uint8_t hardwareZeroRawCount;
            uint8_t notReadyCount;
            uint8_t zeroBeforeReadyCount;
            uint8_t zeroAfterDrdyCount;
            uint8_t i2cErrorCount;
            uint8_t unreadWithoutDrdyCount;
            uint8_t softInvalidCount;
            uint8_t hardInvalidCount;
            uint8_t staleUnreadDrainCount;
            uint8_t invalidSentinelCount;
            bool allRawZero;
            bool bootOk;
        } frameError;
    } data;
} sensorarrayAsyncLogEvent_t;

typedef struct {
    bool active;
    uint32_t seqStart;
    uint32_t seqEnd;
    int64_t windowStartUs;
    uint64_t publishedStart;
    uint64_t droppedFrameStart;
    uint64_t droppedEventStart;
    uint64_t outputFrames;
    uint64_t frameAgeTotalUs;
    uint64_t frameAgeMaxUs;
    uint64_t outTotalUs;
    uint64_t outMaxUs;
    uint64_t measureFrameTotalUs;
    uint64_t measureFrameMaxUs;
    uint32_t queueDepthMax;
} sensorarrayAsyncLogSummary_t;

typedef struct {
    uint64_t publishedFrames;
    uint64_t droppedOutputFrames;
    uint64_t droppedEventLogs;
} sensorarrayAsyncLogSharedStats_t;

static portMUX_TYPE s_asyncLogMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayAsyncLogFrameSlot_t s_frameSlots[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static uint8_t s_frameSlotState[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static uint8_t s_pendingSlots[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static uint8_t s_pendingReadIndex;
static uint8_t s_pendingCount;

static StaticQueue_t s_eventQueueStruct;
static uint8_t s_eventQueueStorage[SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT *
                                   sizeof(sensorarrayAsyncLogEvent_t)];
static QueueHandle_t s_eventQueue;
static TaskHandle_t s_logTaskHandle;
static bool s_asyncLogStarted;
static sensorarrayAsyncLogSharedStats_t s_sharedStats;

static uint64_t sensorarrayAsyncLogElapsedPositiveUs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    return elapsedUs > 0 ? (uint64_t)elapsedUs : 0u;
}

static sensorarrayAsyncLogSharedStats_t sensorarrayAsyncLogReadStats(void)
{
    sensorarrayAsyncLogSharedStats_t stats;
    portENTER_CRITICAL(&s_asyncLogMux);
    stats = s_sharedStats;
    portEXIT_CRITICAL(&s_asyncLogMux);
    return stats;
}

static void sensorarrayAsyncLogNotifyTask(void)
{
    if (s_logTaskHandle) {
        xTaskNotifyGive(s_logTaskHandle);
    }
}

static void sensorarrayAsyncLogIncrementDroppedEvent(void)
{
    portENTER_CRITICAL(&s_asyncLogMux);
    s_sharedStats.droppedEventLogs++;
    portEXIT_CRITICAL(&s_asyncLogMux);
}

static uint8_t sensorarrayAsyncLogFindFreeSlotLocked(void)
{
    for (uint8_t i = 0u; i < SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT; ++i) {
        if (s_frameSlotState[i] == 0u) {
            return i;
        }
    }
    return UINT8_MAX;
}

static bool sensorarrayAsyncLogDropOldestQueuedFrameLocked(void)
{
    if (s_pendingCount == 0u) {
        return false;
    }
    uint8_t droppedSlot = s_pendingSlots[s_pendingReadIndex];
    s_frameSlotState[droppedSlot] = 0u;
    s_pendingReadIndex = (uint8_t)((s_pendingReadIndex + 1u) %
                                   SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT);
    s_pendingCount--;
    s_sharedStats.droppedOutputFrames++;
    return true;
}

static bool sensorarrayAsyncLogPopFrame(sensorarrayFrame_t *outFrame,
                                        uint64_t *outMeasureFrameUs,
                                        int64_t *outPublishedUs,
                                        uint32_t *outQueueDepth)
{
    if (!outFrame || !outMeasureFrameUs || !outPublishedUs || !outQueueDepth) {
        return false;
    }

    portENTER_CRITICAL(&s_asyncLogMux);
    if (s_pendingCount == 0u) {
        portEXIT_CRITICAL(&s_asyncLogMux);
        return false;
    }
    uint8_t slot = s_pendingSlots[s_pendingReadIndex];
    s_pendingReadIndex = (uint8_t)((s_pendingReadIndex + 1u) %
                                   SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT);
    s_pendingCount--;
    *outFrame = s_frameSlots[slot].frame;
    *outMeasureFrameUs = s_frameSlots[slot].measureFrameUs;
    *outPublishedUs = s_frameSlots[slot].publishedUs;
    *outQueueDepth = s_pendingCount;
    s_frameSlotState[slot] = 0u;
    portEXIT_CRITICAL(&s_asyncLogMux);
    return true;
}

static void sensorarrayAsyncLogPrintEvent(const sensorarrayAsyncLogEvent_t *event)
{
    if (!event) {
        return;
    }

    switch (event->type) {
    case SENSORARRAY_ASYNC_LOG_EVENT_OVERRUN:
        printf("OV,s=%lu,fu=%lld,pu=%lld,o=1,async=1\n",
               (unsigned long)event->sequence,
               (long long)event->data.overrun.elapsedUs,
               (long long)event->data.overrun.periodUs);
        break;
    case SENSORARRAY_ASYNC_LOG_EVENT_FRAME_ERROR:
        if (event->data.frameError.capValidMask == 0u) {
            printf("MATRIXFDC_DIAG,stage=all_invalid_frame,seq=%lu,errorMask=0x%016llX,readErr=0x%lx,bootOk=%u,freshCount=%u,hardwareZeroRawCount=%u,notReadyCount=%u,zeroBeforeReadyCount=%u,zeroAfterDrdyCount=%u,i2cErrorCount=%u,unreadWithoutDrdyCount=%u,softInvalidCount=%u,hardInvalidCount=%u,staleUnreadDrainCount=%u,invalidSentinelCount=%u,rawAllZero=%u,async=1\n",
                   (unsigned long)event->sequence,
                   (unsigned long long)event->data.frameError.errorMask,
                   (unsigned long)event->data.frameError.readErr,
                   event->data.frameError.bootOk ? 1u : 0u,
                   (unsigned)event->data.frameError.freshCount,
                   (unsigned)event->data.frameError.hardwareZeroRawCount,
                   (unsigned)event->data.frameError.notReadyCount,
                   (unsigned)event->data.frameError.zeroBeforeReadyCount,
                   (unsigned)event->data.frameError.zeroAfterDrdyCount,
                   (unsigned)event->data.frameError.i2cErrorCount,
                   (unsigned)event->data.frameError.unreadWithoutDrdyCount,
                   (unsigned)event->data.frameError.softInvalidCount,
                   (unsigned)event->data.frameError.hardInvalidCount,
                   (unsigned)event->data.frameError.staleUnreadDrainCount,
                   (unsigned)event->data.frameError.invalidSentinelCount,
                   event->data.frameError.allRawZero ? 1u : 0u);
        } else {
            printf("FRAME_ERROR,s=%lu,err=0x%lx,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,vc=%u,fc=%u,async=1\n",
                   (unsigned long)event->sequence,
                   (unsigned long)event->data.frameError.readErr,
                   (unsigned long long)event->data.frameError.capValidMask,
                   (unsigned long long)event->data.frameError.warnMask,
                   (unsigned long long)event->data.frameError.errorMask,
                   (unsigned)event->data.frameError.validCount,
                   (unsigned)event->data.frameError.freshCount);
        }
        break;
    default:
        printf("ASYNC_LOG_EVENT_UNKNOWN,type=%u,s=%lu\n",
               (unsigned)event->type,
               (unsigned long)event->sequence);
        break;
    }
}

static void sensorarrayAsyncLogDrainEvents(void)
{
    if (!s_eventQueue) {
        return;
    }

    sensorarrayAsyncLogEvent_t event;
    while (xQueueReceive(s_eventQueue, &event, 0) == pdTRUE) {
        sensorarrayAsyncLogPrintEvent(&event);
    }
}

static void sensorarrayAsyncLogSummarySetBaseline(sensorarrayAsyncLogSummary_t *summary,
                                                  const sensorarrayAsyncLogSharedStats_t *stats)
{
    if (!summary || !stats) {
        return;
    }
    *summary = (sensorarrayAsyncLogSummary_t){
        .publishedStart = stats->publishedFrames,
        .droppedFrameStart = stats->droppedOutputFrames,
        .droppedEventStart = stats->droppedEventLogs,
    };
}

static void sensorarrayAsyncLogSummaryBegin(sensorarrayAsyncLogSummary_t *summary,
                                            const sensorarrayFrame_t *frame)
{
    if (!summary || !frame) {
        return;
    }
    uint64_t publishedStart = summary->publishedStart;
    uint64_t droppedFrameStart = summary->droppedFrameStart;
    uint64_t droppedEventStart = summary->droppedEventStart;
    *summary = (sensorarrayAsyncLogSummary_t){
        .active = true,
        .seqStart = frame->sequence,
        .seqEnd = frame->sequence,
        .windowStartUs = esp_timer_get_time(),
        .publishedStart = publishedStart,
        .droppedFrameStart = droppedFrameStart,
        .droppedEventStart = droppedEventStart,
    };
}

static void sensorarrayAsyncLogUpdateSummary(sensorarrayAsyncLogSummary_t *summary,
                                             const sensorarrayFrame_t *frame,
                                             uint64_t measureFrameUs,
                                             uint64_t frameAgeUs,
                                             uint64_t outputUs,
                                             uint32_t queueDepth)
{
    if (!summary || !frame) {
        return;
    }
    if (!summary->active) {
        sensorarrayAsyncLogSummaryBegin(summary, frame);
    }

    summary->seqEnd = frame->sequence;
    summary->outputFrames++;
    summary->frameAgeTotalUs += frameAgeUs;
    if (frameAgeUs > summary->frameAgeMaxUs) {
        summary->frameAgeMaxUs = frameAgeUs;
    }
    summary->outTotalUs += outputUs;
    if (outputUs > summary->outMaxUs) {
        summary->outMaxUs = outputUs;
    }
    summary->measureFrameTotalUs += measureFrameUs;
    if (measureFrameUs > summary->measureFrameMaxUs) {
        summary->measureFrameMaxUs = measureFrameUs;
    }
    if (queueDepth > summary->queueDepthMax) {
        summary->queueDepthMax = queueDepth;
    }
}

static void sensorarrayAsyncLogMaybePrintSummary(sensorarrayAsyncLogSummary_t *summary)
{
    if (!summary || !summary->active) {
        return;
    }

    sensorarrayAsyncLogSharedStats_t stats = sensorarrayAsyncLogReadStats();
    uint32_t every = (uint32_t)CONFIG_SENSORARRAY_ASYNC_LOG_SUMMARY_EVERY_N_FRAMES;
    if (every == 0u) {
        every = 20u;
    }
    uint64_t measureFrames = stats.publishedFrames - summary->publishedStart;
    if (measureFrames < every) {
        return;
    }

    int64_t elapsedUsSigned = esp_timer_get_time() - summary->windowStartUs;
    uint64_t elapsedUs = elapsedUsSigned > 0 ? (uint64_t)elapsedUsSigned : 1u;
    uint64_t measureFpsX100 = (measureFrames * 100000000ull) / elapsedUs;
    uint64_t outputFpsX100 = (summary->outputFrames * 100000000ull) / elapsedUs;
    uint64_t frameAgeAvgUs = summary->outputFrames ?
        (summary->frameAgeTotalUs / summary->outputFrames) : 0u;
    uint64_t outAvgUs = summary->outputFrames ?
        (summary->outTotalUs / summary->outputFrames) : 0u;
    uint64_t measureAvgUs = summary->outputFrames ?
        (summary->measureFrameTotalUs / summary->outputFrames) : 0u;
    uint64_t droppedFrames = stats.droppedOutputFrames - summary->droppedFrameStart;
    uint64_t droppedEvents = stats.droppedEventLogs - summary->droppedEventStart;

    printf("LOG20,s0=%lu,s1=%lu,n=%llu,measureFps=%llu.%02llu,outputFps=%llu.%02llu,frameAgeAvgUs=%llu,frameAgeMaxUs=%llu,outputQueueDepth=%lu,qDepthMax=%lu,droppedOutputFrames=%llu,droppedEventLogs=%llu,outUsAvg=%llu,outUsMax=%llu,measureFrameUsAvg=%llu,measureFrameUsMax=%llu,stackHighWaterWords=%u\n",
           (unsigned long)summary->seqStart,
           (unsigned long)summary->seqEnd,
           (unsigned long long)measureFrames,
           (unsigned long long)(measureFpsX100 / 100ull),
           (unsigned long long)(measureFpsX100 % 100ull),
           (unsigned long long)(outputFpsX100 / 100ull),
           (unsigned long long)(outputFpsX100 % 100ull),
           (unsigned long long)frameAgeAvgUs,
           (unsigned long long)summary->frameAgeMaxUs,
           (unsigned long)s_pendingCount,
           (unsigned long)summary->queueDepthMax,
           (unsigned long long)droppedFrames,
           (unsigned long long)droppedEvents,
           (unsigned long long)outAvgUs,
           (unsigned long long)summary->outMaxUs,
           (unsigned long long)measureAvgUs,
           (unsigned long long)summary->measureFrameMaxUs,
           (unsigned)uxTaskGetStackHighWaterMark(NULL));

    sensorarrayAsyncLogSummarySetBaseline(summary, &stats);
}

static void sensorarrayAsyncLogTask(void *arg)
{
    (void)arg;
    sensorarrayAsyncLogSummary_t summary = {0};
    sensorarrayFrame_t frame;

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000u));
        sensorarrayAsyncLogDrainEvents();

        uint64_t measureFrameUs = 0u;
        int64_t publishedUs = 0;
        uint32_t queueDepth = 0u;
        while (sensorarrayAsyncLogPopFrame(&frame, &measureFrameUs, &publishedUs, &queueDepth)) {
            sensorarrayAsyncLogDrainEvents();

            int64_t outputStartUs = esp_timer_get_time();
            uint64_t frameAgeUs = (publishedUs > 0 && outputStartUs > publishedUs) ?
                (uint64_t)(outputStartUs - publishedUs) : 0u;
            (void)sensorarrayFrameOutputPrint(&frame);
            uint64_t outputUs = sensorarrayAsyncLogElapsedPositiveUs(outputStartUs);

            sensorarrayAsyncLogUpdateSummary(&summary,
                                             &frame,
                                             measureFrameUs,
                                             frameAgeUs,
                                             outputUs,
                                             queueDepth);
            sensorarrayAsyncLogMaybePrintSummary(&summary);
        }
        sensorarrayAsyncLogDrainEvents();
    }
}

esp_err_t sensorarrayAsyncLogInit(void)
{
    if (!CONFIG_SENSORARRAY_ASYNC_LOG_ENABLE) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (s_asyncLogStarted) {
        return ESP_OK;
    }

    memset(s_frameSlots, 0, sizeof(s_frameSlots));
    memset(s_frameSlotState, 0, sizeof(s_frameSlotState));
    memset(s_pendingSlots, 0, sizeof(s_pendingSlots));
    s_pendingReadIndex = 0u;
    s_pendingCount = 0u;
    s_sharedStats = (sensorarrayAsyncLogSharedStats_t){0};

    s_eventQueue = xQueueCreateStatic(SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT,
                                      sizeof(sensorarrayAsyncLogEvent_t),
                                      s_eventQueueStorage,
                                      &s_eventQueueStruct);
    if (!s_eventQueue) {
        return ESP_ERR_NO_MEM;
    }

    BaseType_t taskOk;
    if (CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE < 0) {
        taskOk = xTaskCreate(sensorarrayAsyncLogTask,
                             "sensorarrayLogTask",
                             CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK,
                             NULL,
                             CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY,
                             &s_logTaskHandle);
    } else {
        taskOk = xTaskCreatePinnedToCore(sensorarrayAsyncLogTask,
                                         "sensorarrayLogTask",
                                         CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK,
                                         NULL,
                                         CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY,
                                         &s_logTaskHandle,
                                         CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE);
    }
    if (taskOk != pdPASS || !s_logTaskHandle) {
        s_eventQueue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_asyncLogStarted = true;
    printf("APP_LOG_INIT,mode=async,frameSlots=%u,eventQueueLen=%u,summaryEvery=%u,taskPrio=%u,taskCore=%d,dropOldFrames=%u\n",
           (unsigned)SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT,
           (unsigned)SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT,
           (unsigned)CONFIG_SENSORARRAY_ASYNC_LOG_SUMMARY_EVERY_N_FRAMES,
           (unsigned)CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY,
           (int)CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE,
           CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES ? 1u : 0u);
    return ESP_OK;
}

bool sensorarrayAsyncLogIsRunning(void)
{
    return s_asyncLogStarted && s_logTaskHandle != NULL;
}

esp_err_t sensorarrayAsyncLogPublishFrameSnapshot(const sensorarrayFrame_t *frame,
                                                  uint64_t measureFrameUs)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayAsyncLogIsRunning()) {
        return ESP_ERR_INVALID_STATE;
    }

    int64_t publishedUs = esp_timer_get_time();
    bool notify = false;
    esp_err_t result = ESP_OK;
    uint8_t slot = UINT8_MAX;
    portENTER_CRITICAL(&s_asyncLogMux);
    s_sharedStats.publishedFrames++;
    slot = sensorarrayAsyncLogFindFreeSlotLocked();
    if (slot == UINT8_MAX && CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES) {
        (void)sensorarrayAsyncLogDropOldestQueuedFrameLocked();
        slot = sensorarrayAsyncLogFindFreeSlotLocked();
    }
    if (slot == UINT8_MAX) {
        s_sharedStats.droppedOutputFrames++;
        result = ESP_ERR_TIMEOUT;
    } else {
        s_frameSlotState[slot] = 2u;
    }
    portEXIT_CRITICAL(&s_asyncLogMux);

    if (result == ESP_OK) {
        /*
         * The full frame snapshot is intentionally copied after reserving a slot
         * but before publishing that slot to the pending ring. This keeps the
         * critical section short while still ensuring the consumer never sees a
         * partially copied frame.
         */
        s_frameSlots[slot] = (sensorarrayAsyncLogFrameSlot_t){
            .frame = *frame,
            .measureFrameUs = measureFrameUs,
            .publishedUs = publishedUs,
        };

        portENTER_CRITICAL(&s_asyncLogMux);
        uint8_t writeIndex = (uint8_t)((s_pendingReadIndex + s_pendingCount) %
                                       SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT);
        s_pendingSlots[writeIndex] = slot;
        s_pendingCount++;
        s_frameSlotState[slot] = 1u;
        portEXIT_CRITICAL(&s_asyncLogMux);
        notify = true;
    }

    if (notify) {
        sensorarrayAsyncLogNotifyTask();
    }
    return result;
}

static esp_err_t sensorarrayAsyncLogPublishEvent(const sensorarrayAsyncLogEvent_t *event)
{
    if (!event) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayAsyncLogIsRunning() || !s_eventQueue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(s_eventQueue, event, 0) != pdTRUE) {
        sensorarrayAsyncLogIncrementDroppedEvent();
        return ESP_ERR_TIMEOUT;
    }
    sensorarrayAsyncLogNotifyTask();
    return ESP_OK;
}

esp_err_t sensorarrayAsyncLogPublishOverrun(uint32_t sequence,
                                            int64_t elapsedUs,
                                            int64_t periodUs)
{
    sensorarrayAsyncLogEvent_t event = {
        .type = SENSORARRAY_ASYNC_LOG_EVENT_OVERRUN,
        .sequence = sequence,
        .data.overrun = {
            .elapsedUs = elapsedUs,
            .periodUs = periodUs,
        },
    };
    return sensorarrayAsyncLogPublishEvent(&event);
}

esp_err_t sensorarrayAsyncLogPublishFrameError(const sensorarrayFrame_t *frame,
                                               esp_err_t readErr,
                                               bool allRawZero,
                                               bool bootOk)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t invalidSentinelCount =
        (uint8_t)(SENSORARRAY_MATRIX_CELL_COUNT - frame->validCount);
    sensorarrayAsyncLogEvent_t event = {
        .type = SENSORARRAY_ASYNC_LOG_EVENT_FRAME_ERROR,
        .sequence = frame->sequence,
        .data.frameError = {
            .readErr = readErr,
            .capValidMask = frame->capValidMask,
            .errorMask = frame->errorMask,
            .warnMask = frame->warnMask,
            .validCount = frame->validCount,
            .freshCount = frame->freshCount,
            .hardwareZeroRawCount = frame->hardwareZeroRawCount,
            .notReadyCount = frame->notReadyCount,
            .zeroBeforeReadyCount = frame->zeroBeforeReadyCount,
            .zeroAfterDrdyCount = frame->zeroAfterDrdyCount,
            .i2cErrorCount = frame->i2cErrorCount,
            .unreadWithoutDrdyCount = frame->unreadWithoutDrdyCount,
            .softInvalidCount = frame->softInvalidCount,
            .hardInvalidCount = frame->hardInvalidCount,
            .staleUnreadDrainCount = frame->staleUnreadDrainCount,
            .invalidSentinelCount = invalidSentinelCount,
            .allRawZero = allRawZero,
            .bootOk = bootOk,
        },
    };
    return sensorarrayAsyncLogPublishEvent(&event);
}
