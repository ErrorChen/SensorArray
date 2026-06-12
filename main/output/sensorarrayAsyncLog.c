#include "sensorarrayAsyncLog.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "boardSupport.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFrameOutput.h"
#include "sensorarrayNetStatus.h"
#include "sensorarrayTypes.h"

#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_ENABLE
#define CONFIG_SENSORARRAY_ASYNC_LOG_ENABLE 1
#endif
#ifndef CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS
#define CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS 8
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
#ifndef CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG
#define CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG 0
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
    uint32_t physicalSweepStart;
    uint32_t physicalSweepEnd;
    int64_t windowStartUs;
    uint64_t publishedStart;
    uint64_t freshFrameStart;
    uint64_t staleFrameStart;
    uint64_t mixedFrameStart;
    uint64_t rowFreshMaskBadStart;
    uint64_t primaryFreshMaskBadStart;
    uint64_t secondaryFreshMaskBadStart;
    uint64_t freshCellStart;
    uint64_t physicalSweepTotalUsStart;
    uint64_t rowStepTotalUsStart;
    uint64_t frameStartIntervalTotalUsStart;
    uint64_t frameStartIntervalCountStart;
    uint64_t measureFrameTotalUsStart;
    uint64_t droppedFrameStart;
    uint64_t droppedEventStart;
    uint64_t processedFrames;
    uint64_t outputFrames;
    uint64_t freshFrames;
    uint64_t staleFrames;
    uint64_t mixedFrames;
    uint64_t rowFreshMaskBad;
    uint64_t primaryFreshMaskBad;
    uint64_t secondaryFreshMaskBad;
    uint64_t freshCells;
    uint64_t physicalSweepTotalUs;
    uint64_t physicalSweepMaxUs;
    uint64_t rowStepTotalUs;
    uint64_t rowStepMaxUs;
    uint64_t frameStartIntervalTotalUs;
    uint64_t frameStartIntervalCount;
    uint64_t frameStartIntervalMaxUs;
    uint64_t lastFrameStartUs;
    uint64_t emitIntervalTotalUs;
    uint64_t emitIntervalCount;
    uint64_t emitIntervalMaxUs;
    uint64_t lastEmitUs;
    uint64_t frameAgeTotalUs;
    uint64_t frameAgeMaxUs;
    uint64_t outTotalUs;
    uint64_t outMaxUs;
    uint64_t measureFrameTotalUs;
    uint64_t measureFrameMaxUs;
    uint32_t queueDepthMax;
    sensorarrayFdcFrameTelemetry_t telemetry;
    uint64_t cacheApplyI2cMaxUs;
    sensorarrayAdsGapSnapshot_t adsGapStart;
    sensorarrayAdsGapSnapshot_t adsGap;
    uint32_t adsOffsetCount;
    double adsOffsetMean;
    double adsOffsetM2;
    uint32_t fdcTheoryReadyUs;
    uint32_t fdcTheoryFrameReadyUs;
    uint32_t fdcTheorySwitchDelayUs;
    uint32_t fdcFrefHz;
    uint16_t fdcRcount;
    uint16_t fdcSettleCount;
    uint16_t fdcClockDividers;
    uint16_t fdcDriveCurrent;
    uint16_t fdcConfig;
    uint16_t fdcMuxConfig;
    uint8_t fdcDeglitch;
    bool fdcSensorActivateFullCurrent;
    bool fdcHighCurrentDrive;
} sensorarrayAsyncLogSummary_t;

typedef struct {
    uint64_t publishedFrames;
    uint64_t freshFrames;
    uint64_t staleFrames;
    uint64_t mixedFrames;
    uint64_t rowFreshMaskBad;
    uint64_t primaryFreshMaskBad;
    uint64_t secondaryFreshMaskBad;
    uint64_t freshCells;
    uint64_t physicalSweepTotalUs;
    uint64_t physicalSweepMaxUs;
    uint64_t rowStepTotalUs;
    uint64_t rowStepMaxUs;
    uint64_t frameStartIntervalTotalUs;
    uint64_t frameStartIntervalCount;
    uint64_t frameStartIntervalMaxUs;
    uint64_t lastFrameStartUs;
    uint64_t measureFrameTotalUs;
    uint64_t measureFrameMaxUs;
    uint64_t droppedOutputFrames;
    uint64_t droppedEventLogs;
} sensorarrayAsyncLogSharedStats_t;

typedef struct {
    uint32_t frames;
    uint32_t sampleCount[SENSORARRAY_MATRIX_CELL_COUNT];
    double mean[SENSORARRAY_MATRIX_CELL_COUNT];
    double m2[SENSORARRAY_MATRIX_CELL_COUNT];
    uint32_t lastRaw[SENSORARRAY_MATRIX_CELL_COUNT];
    bool haveLastRaw[SENSORARRAY_MATRIX_CELL_COUNT];
    double pfMeanDriftMax;
    uint32_t rawJumpMax;
    uint32_t invalidCells;
} sensorarrayPrecisionWindow_t;

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
static sensorarrayPrecisionWindow_t s_precisionWindow;

static uint64_t sensorarrayAsyncLogElapsedPositiveUs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    return elapsedUs > 0 ? (uint64_t)elapsedUs : 0u;
}

static void sensorarrayAsyncLogPrecisionReset(void)
{
    memset(&s_precisionWindow, 0, sizeof(s_precisionWindow));
}

static void sensorarrayAsyncLogPrecisionUpdate(const sensorarrayFrame_t *frame)
{
    if (!frame) {
        return;
    }
    s_precisionWindow.frames++;
    for (size_t cell = 0u; cell < SENSORARRAY_MATRIX_CELL_COUNT; ++cell) {
        uint64_t bit = 1ull << cell;
        double pf = frame->capTotalPf[cell];
        bool valid = (frame->capValidMask & bit) != 0u &&
                     (frame->freshMask & bit) != 0u &&
                     isfinite(pf);
        if (!valid) {
            s_precisionWindow.invalidCells++;
            continue;
        }

        uint32_t count = ++s_precisionWindow.sampleCount[cell];
        double delta = pf - s_precisionWindow.mean[cell];
        s_precisionWindow.mean[cell] += delta / (double)count;
        double delta2 = pf - s_precisionWindow.mean[cell];
        s_precisionWindow.m2[cell] += delta * delta2;
        double drift = fabs(delta2);
        if (drift > s_precisionWindow.pfMeanDriftMax) {
            s_precisionWindow.pfMeanDriftMax = drift;
        }

        uint32_t raw = frame->raw28[cell];
        if (s_precisionWindow.haveLastRaw[cell]) {
            uint32_t last = s_precisionWindow.lastRaw[cell];
            uint32_t jump = raw >= last ? raw - last : last - raw;
            if (jump > s_precisionWindow.rawJumpMax) {
                s_precisionWindow.rawJumpMax = jump;
            }
        }
        s_precisionWindow.lastRaw[cell] = raw;
        s_precisionWindow.haveLastRaw[cell] = true;
    }
}

static uint64_t sensorarrayAsyncLogEstimateI2cBits(uint32_t writeCount,
                                                   uint32_t readCount,
                                                   uint32_t writeBytes,
                                                   uint32_t readBytes)
{
    uint64_t dataBitsWithAck = ((uint64_t)writeBytes + readBytes) * 9ull;
    uint64_t addressBitsWithAck = ((uint64_t)writeCount + ((uint64_t)readCount * 2ull)) * 9ull;
    uint64_t framingBits = ((uint64_t)writeCount + readCount) * 2ull;
    return dataBitsWithAck + addressBitsWithAck + framingBits;
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
    /*
     * Keep the final production and emission timestamps across report windows.
     * Otherwise the gap occupied by printing LOG20/FPS20/... is omitted from
     * the next window and the reported host emission rate is biased high.
     */
    uint64_t lastFrameStartUs = summary->lastFrameStartUs;
    uint64_t lastEmitUs = summary->lastEmitUs;
    *summary = (sensorarrayAsyncLogSummary_t){
        .publishedStart = stats->publishedFrames,
        .freshFrameStart = stats->freshFrames,
        .staleFrameStart = stats->staleFrames,
        .mixedFrameStart = stats->mixedFrames,
        .rowFreshMaskBadStart = stats->rowFreshMaskBad,
        .primaryFreshMaskBadStart = stats->primaryFreshMaskBad,
        .secondaryFreshMaskBadStart = stats->secondaryFreshMaskBad,
        .freshCellStart = stats->freshCells,
        .physicalSweepTotalUsStart = stats->physicalSweepTotalUs,
        .rowStepTotalUsStart = stats->rowStepTotalUs,
        .frameStartIntervalTotalUsStart = stats->frameStartIntervalTotalUs,
        .frameStartIntervalCountStart = stats->frameStartIntervalCount,
        .measureFrameTotalUsStart = stats->measureFrameTotalUs,
        .droppedFrameStart = stats->droppedOutputFrames,
        .droppedEventStart = stats->droppedEventLogs,
        .lastFrameStartUs = lastFrameStartUs,
        .lastEmitUs = lastEmitUs,
    };
    sensorarrayAsyncLogPrecisionReset();
}

static void sensorarrayAsyncLogSummaryBegin(sensorarrayAsyncLogSummary_t *summary,
                                             const sensorarrayFrame_t *frame)
{
    if (!summary || !frame) {
        return;
    }
    uint64_t publishedStart = summary->publishedStart;
    uint64_t freshFrameStart = summary->freshFrameStart;
    uint64_t staleFrameStart = summary->staleFrameStart;
    uint64_t mixedFrameStart = summary->mixedFrameStart;
    uint64_t rowFreshMaskBadStart = summary->rowFreshMaskBadStart;
    uint64_t primaryFreshMaskBadStart = summary->primaryFreshMaskBadStart;
    uint64_t secondaryFreshMaskBadStart = summary->secondaryFreshMaskBadStart;
    uint64_t freshCellStart = summary->freshCellStart;
    uint64_t physicalSweepTotalUsStart = summary->physicalSweepTotalUsStart;
    uint64_t rowStepTotalUsStart = summary->rowStepTotalUsStart;
    uint64_t frameStartIntervalTotalUsStart = summary->frameStartIntervalTotalUsStart;
    uint64_t frameStartIntervalCountStart = summary->frameStartIntervalCountStart;
    uint64_t measureFrameTotalUsStart = summary->measureFrameTotalUsStart;
    uint64_t droppedFrameStart = summary->droppedFrameStart;
    uint64_t droppedEventStart = summary->droppedEventStart;
    uint64_t lastFrameStartUs = summary->lastFrameStartUs;
    uint64_t lastEmitUs = summary->lastEmitUs;
    *summary = (sensorarrayAsyncLogSummary_t){
        .active = true,
        .seqStart = frame->sequence,
        .seqEnd = frame->sequence,
        .physicalSweepStart = frame->physicalSweepId,
        .physicalSweepEnd = frame->physicalSweepId,
        .windowStartUs = esp_timer_get_time(),
        .publishedStart = publishedStart,
        .freshFrameStart = freshFrameStart,
        .staleFrameStart = staleFrameStart,
        .mixedFrameStart = mixedFrameStart,
        .rowFreshMaskBadStart = rowFreshMaskBadStart,
        .primaryFreshMaskBadStart = primaryFreshMaskBadStart,
        .secondaryFreshMaskBadStart = secondaryFreshMaskBadStart,
        .freshCellStart = freshCellStart,
        .physicalSweepTotalUsStart = physicalSweepTotalUsStart,
        .rowStepTotalUsStart = rowStepTotalUsStart,
        .frameStartIntervalTotalUsStart = frameStartIntervalTotalUsStart,
        .frameStartIntervalCountStart = frameStartIntervalCountStart,
        .measureFrameTotalUsStart = measureFrameTotalUsStart,
        .droppedFrameStart = droppedFrameStart,
        .droppedEventStart = droppedEventStart,
        .lastFrameStartUs = lastFrameStartUs,
        .lastEmitUs = lastEmitUs,
        .adsGapStart = frame->adsGap,
        .adsGap = frame->adsGap,
    };
}

static void sensorarrayAsyncLogUpdateSummary(sensorarrayAsyncLogSummary_t *summary,
                                             const sensorarrayFrame_t *frame,
                                             uint64_t measureFrameUs,
                                             uint64_t frameAgeUs,
                                             uint64_t outputUs,
                                             uint32_t queueDepth,
                                             bool emitted)
{
    if (!summary || !frame) {
        return;
    }
    if (!summary->active) {
        sensorarrayAsyncLogSummaryBegin(summary, frame);
    }

    summary->seqEnd = frame->sequence;
    summary->physicalSweepEnd = frame->physicalSweepId;
    summary->processedFrames++;
    sensorarrayAsyncLogPrecisionUpdate(frame);
    summary->freshCells += frame->freshCount;
    summary->physicalSweepTotalUs += frame->physicalSweepUs;
    if (frame->physicalSweepUs > summary->physicalSweepMaxUs) {
        summary->physicalSweepMaxUs = frame->physicalSweepUs;
    }
    summary->rowStepTotalUs += frame->rowStepUsAvg * SENSORARRAY_MATRIX_ROWS;
    if (frame->rowStepUsMax > summary->rowStepMaxUs) {
        summary->rowStepMaxUs = frame->rowStepUsMax;
    }
    if (summary->lastFrameStartUs != 0u && frame->frameStartUs > summary->lastFrameStartUs) {
        uint64_t intervalUs = frame->frameStartUs - summary->lastFrameStartUs;
        summary->frameStartIntervalTotalUs += intervalUs;
        summary->frameStartIntervalCount++;
        if (intervalUs > summary->frameStartIntervalMaxUs) {
            summary->frameStartIntervalMaxUs = intervalUs;
        }
    }
    summary->lastFrameStartUs = frame->frameStartUs;
    if (frame->freshFrame) {
        summary->freshFrames++;
    }
    if (frame->stale) {
        summary->staleFrames++;
    }
    if (frame->mixedEpoch) {
        summary->mixedFrames++;
    }
    if (frame->rowFreshMask != 0xFFu) {
        summary->rowFreshMaskBad++;
    }
    if (frame->primaryFreshMask != 0xFFu) {
        summary->primaryFreshMaskBad++;
    }
    if (frame->secondaryFreshMask != 0xFFu) {
        summary->secondaryFreshMaskBad++;
    }
    if (emitted) {
        summary->outputFrames++;
        if (summary->lastEmitUs != 0u && frame->emitUs > summary->lastEmitUs) {
            uint64_t intervalUs = frame->emitUs - summary->lastEmitUs;
            summary->emitIntervalTotalUs += intervalUs;
            summary->emitIntervalCount++;
            if (intervalUs > summary->emitIntervalMaxUs) {
                summary->emitIntervalMaxUs = intervalUs;
            }
        }
        summary->lastEmitUs = frame->emitUs;
        summary->frameAgeTotalUs += frameAgeUs;
        if (frameAgeUs > summary->frameAgeMaxUs) {
            summary->frameAgeMaxUs = frameAgeUs;
        }
        summary->outTotalUs += outputUs;
        if (outputUs > summary->outMaxUs) {
            summary->outMaxUs = outputUs;
        }
    }
    summary->measureFrameTotalUs += measureFrameUs;
    if (measureFrameUs > summary->measureFrameMaxUs) {
        summary->measureFrameMaxUs = measureFrameUs;
    }
    if (queueDepth > summary->queueDepthMax) {
        summary->queueDepthMax = queueDepth;
    }

#define SENSORARRAY_ASYNC_TELEMETRY_ADD(field) \
    summary->telemetry.field += frame->telemetry.field
    SENSORARRAY_ASYNC_TELEMETRY_ADD(routeUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(settleUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(cacheCompareUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(cacheApplyI2cUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(readyWaitUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(statusReadUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(dataReadUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(mergeUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(primaryWorkerRunUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(secondaryWorkerRunUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerStartSkewUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerDoneSkewUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerNotifyUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(rowSleepBarrierUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerPreReleaseUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerSleepAckWaitUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerStartGiveUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerWaitPrimaryUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerWaitSecondaryUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(parentWaitBothUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(workerJoinUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(frameMaskUpdateUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(frameBookkeepingUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(frameQueueUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus0ReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus1ReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus0WriteCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus1WriteCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus0ReadBytes);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus1ReadBytes);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus0WriteBytes);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus1WriteBytes);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus0TotalUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus1TotalUs);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus0TransactionCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBus1TransactionCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cOrderedDataReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBurstDataReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cBurstFallbackCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cSequenceDataReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cSequenceTransactionCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cSequenceFallbackCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cSequenceErrorCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cNackCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cTimeoutCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(i2cRecoveryCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(directDataReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(directDataFallbackCount);
    summary->telemetry.directDataFallbackReasonMask |=
        frame->telemetry.directDataFallbackReasonMask;
    SENSORARRAY_ASYNC_TELEMETRY_ADD(statusReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(statusSavedReadCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(cacheCompareCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(cacheDiffRows);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(cacheWriteCount);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(cacheRestartRows);
    SENSORARRAY_ASYNC_TELEMETRY_ADD(cacheSkipRows);
#undef SENSORARRAY_ASYNC_TELEMETRY_ADD
    summary->telemetry.dataReadMode = frame->telemetry.dataReadMode;
    summary->telemetry.sequenceRegsPerTransaction =
        frame->telemetry.sequenceRegsPerTransaction;
    summary->telemetry.sequenceTransactionsPerRow =
        frame->telemetry.sequenceTransactionsPerRow;
    if (frame->telemetry.cacheApplyI2cUs > summary->cacheApplyI2cMaxUs) {
        summary->cacheApplyI2cMaxUs = frame->telemetry.cacheApplyI2cUs;
    }
    summary->adsGap = frame->adsGap;
    if (frame->adsGap.initialized && frame->adsGap.jobsRun != 0u) {
        summary->adsOffsetCount++;
        double offsetUv = (double)frame->adsGap.ain9OffsetUv;
        double delta = offsetUv - summary->adsOffsetMean;
        summary->adsOffsetMean += delta / (double)summary->adsOffsetCount;
        double delta2 = offsetUv - summary->adsOffsetMean;
        summary->adsOffsetM2 += delta * delta2;
    }
    summary->fdcTheoryReadyUs = frame->fdcTheoryReadyUs;
    summary->fdcTheoryFrameReadyUs = frame->fdcTheoryFrameReadyUs;
    summary->fdcTheorySwitchDelayUs = frame->fdcTheorySwitchDelayUs;
    summary->fdcFrefHz = frame->fdcFrefHz;
    summary->fdcRcount = frame->fdcRcount;
    summary->fdcSettleCount = frame->fdcSettleCount;
    summary->fdcClockDividers = frame->fdcClockDividers;
    summary->fdcDriveCurrent = frame->fdcDriveCurrent;
    summary->fdcConfig = frame->fdcConfig;
    summary->fdcMuxConfig = frame->fdcMuxConfig;
    summary->fdcDeglitch = frame->fdcDeglitch;
    summary->fdcSensorActivateFullCurrent = frame->fdcSensorActivateFullCurrent;
    summary->fdcHighCurrentDrive = frame->fdcHighCurrentDrive;
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
    uint64_t measureFrames = summary->processedFrames;
    if (measureFrames < every) {
        return;
    }

    uint64_t frameStartIntervalAvgUs = summary->frameStartIntervalCount ?
        summary->frameStartIntervalTotalUs / summary->frameStartIntervalCount : 0u;
    uint64_t emitIntervalAvgUs = summary->emitIntervalCount ?
        summary->emitIntervalTotalUs / summary->emitIntervalCount : 0u;
    uint64_t measureFpsX100 = frameStartIntervalAvgUs ?
        100000000ull / frameStartIntervalAvgUs : 0u;
    uint64_t outputFpsX100 = emitIntervalAvgUs ?
        100000000ull / emitIntervalAvgUs : 0u;
    uint64_t physicalFpsX100 = measureFpsX100;
    uint64_t freshFrames = summary->freshFrames;
    uint64_t staleFrames = summary->staleFrames;
    uint64_t mixedFrames = summary->mixedFrames;
    uint64_t rowFreshMaskBad = summary->rowFreshMaskBad;
    uint64_t primaryFreshMaskBad = summary->primaryFreshMaskBad;
    uint64_t secondaryFreshMaskBad = summary->secondaryFreshMaskBad;
    uint64_t cellFreshFpsX100 = measureFrames ?
        (measureFpsX100 * summary->freshCells) /
            (measureFrames * (uint64_t)SENSORARRAY_MATRIX_CELL_COUNT) : 0u;
    uint64_t frameAgeAvgUs = summary->outputFrames ?
        (summary->frameAgeTotalUs / summary->outputFrames) : 0u;
    uint64_t outAvgUs = summary->outputFrames ?
        (summary->outTotalUs / summary->outputFrames) : 0u;
    uint64_t measureAvgUs = measureFrames ?
        (summary->measureFrameTotalUs / measureFrames) : 0u;
    uint64_t physicalSweepAvgUs = measureFrames ?
        (summary->physicalSweepTotalUs / measureFrames) : 0u;
    uint64_t capEmitIntervalAvgUs = emitIntervalAvgUs;
    uint64_t rowSteps = measureFrames * SENSORARRAY_MATRIX_ROWS;
    uint64_t rowStepAvgUs = rowSteps ? (summary->rowStepTotalUs / rowSteps) : 0u;
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

    printf("FPS20,frame0=%lu,frame1=%lu,sweep0=%lu,sweep1=%lu,n=%llu,coreFps=%llu.%02llu,physFps=%llu.%02llu,emitFps=%llu.%02llu,cellFreshFps=%llu.%02llu,coreFrameUsAvg=%llu,physicalSweepUsAvg=%llu,capEmitIntervalUsAvg=%llu,outputUsAvg=%llu,queueDepth=%lu,dropped=%llu,staleFrames=%llu,mixedFrames=%llu\n",
           (unsigned long)summary->seqStart,
           (unsigned long)summary->seqEnd,
           (unsigned long)summary->physicalSweepStart,
           (unsigned long)summary->physicalSweepEnd,
           (unsigned long long)measureFrames,
           (unsigned long long)(measureFpsX100 / 100ull),
           (unsigned long long)(measureFpsX100 % 100ull),
           (unsigned long long)(physicalFpsX100 / 100ull),
           (unsigned long long)(physicalFpsX100 % 100ull),
           (unsigned long long)(outputFpsX100 / 100ull),
           (unsigned long long)(outputFpsX100 % 100ull),
           (unsigned long long)(cellFreshFpsX100 / 100ull),
           (unsigned long long)(cellFreshFpsX100 % 100ull),
           (unsigned long long)measureAvgUs,
           (unsigned long long)physicalSweepAvgUs,
           (unsigned long long)capEmitIntervalAvgUs,
           (unsigned long long)outAvgUs,
           (unsigned long)s_pendingCount,
           (unsigned long long)droppedFrames,
           (unsigned long long)staleFrames,
           (unsigned long long)mixedFrames);
    printf("PHY20,frameCount=%llu,rowSteps=%llu,rowStepUsAvg=%llu,rowStepUsMax=%llu,frameStartIntervalUsAvg=%llu,frameStartIntervalUsMax=%llu,physicalSweepUsAvg=%llu,physicalSweepUsMax=%llu\n",
           (unsigned long long)measureFrames,
           (unsigned long long)rowSteps,
           (unsigned long long)rowStepAvgUs,
           (unsigned long long)summary->rowStepMaxUs,
           (unsigned long long)frameStartIntervalAvgUs,
           (unsigned long long)summary->frameStartIntervalMaxUs,
           (unsigned long long)physicalSweepAvgUs,
           (unsigned long long)summary->physicalSweepMaxUs);
    printf("FRESH20,frames=%llu,freshFrames=%llu,staleFrames=%llu,mixedEpochFrames=%llu,rowFreshMaskBad=%llu,primaryFreshMaskBad=%llu,secondaryFreshMaskBad=%llu,primarySecondaryEpochMismatch=%llu\n",
           (unsigned long long)measureFrames,
           (unsigned long long)freshFrames,
           (unsigned long long)staleFrames,
           (unsigned long long)mixedFrames,
           (unsigned long long)rowFreshMaskBad,
           (unsigned long long)primaryFreshMaskBad,
           (unsigned long long)secondaryFreshMaskBad,
           (unsigned long long)mixedFrames);

    BoardSupportI2cBusInfo_t bus0 = {0};
    BoardSupportI2cBusInfo_t bus1 = {0};
    (void)boardSupportGetI2cBusInfo(false, &bus0);
    (void)boardSupportGetI2cBusInfo(true, &bus1);
    sensorarrayFdcFrameTelemetry_t *telemetry = &summary->telemetry;
    uint64_t bus0Bits = sensorarrayAsyncLogEstimateI2cBits(telemetry->i2cBus0WriteCount,
                                                           telemetry->i2cBus0ReadCount,
                                                           telemetry->i2cBus0WriteBytes,
                                                           telemetry->i2cBus0ReadBytes);
    uint64_t bus1Bits = sensorarrayAsyncLogEstimateI2cBits(telemetry->i2cBus1WriteCount,
                                                           telemetry->i2cBus1ReadCount,
                                                           telemetry->i2cBus1WriteBytes,
                                                           telemetry->i2cBus1ReadBytes);
    uint64_t bus0WireUs = bus0.FrequencyHz ? (bus0Bits * 1000000ull) / bus0.FrequencyHz : 0u;
    uint64_t bus1WireUs = bus1.FrequencyHz ? (bus1Bits * 1000000ull) / bus1.FrequencyHz : 0u;
    uint64_t bus0DriverUs =
        telemetry->i2cBus0TotalUs > bus0WireUs ? telemetry->i2cBus0TotalUs - bus0WireUs : 0u;
    uint64_t bus1DriverUs =
        telemetry->i2cBus1TotalUs > bus1WireUs ? telemetry->i2cBus1TotalUs - bus1WireUs : 0u;
    printf("I2C20,bus0Hz=%lu,bus1Hz=%lu,bus0ReadsLogical=%lu,bus1ReadsLogical=%lu,bus0Transactions=%lu,bus1Transactions=%lu,bus0Writes=%lu,bus1Writes=%lu,bus0BytesRx=%lu,bus1BytesRx=%lu,bus0WireUs=%llu,bus1WireUs=%llu,bus0MeasuredUs=%llu,bus1MeasuredUs=%llu,bus0DriverUs=%llu,bus1DriverUs=%llu,dataReadMode=%s,seqRegsPerTxn=%u,seqTxnPerRow=%u,sequenceReads=%lu,sequenceTransactions=%lu,sequenceFallbacks=%lu,seqErr=%lu,burstReads=%lu,orderedReads=%lu,fallbackReads=%lu,nack=%lu,timeout=%lu,recover=%lu\n",
           (unsigned long)bus0.FrequencyHz,
           (unsigned long)bus1.FrequencyHz,
           (unsigned long)telemetry->i2cBus0ReadCount,
           (unsigned long)telemetry->i2cBus1ReadCount,
           (unsigned long)telemetry->i2cBus0TransactionCount,
           (unsigned long)telemetry->i2cBus1TransactionCount,
           (unsigned long)telemetry->i2cBus0WriteCount,
           (unsigned long)telemetry->i2cBus1WriteCount,
           (unsigned long)telemetry->i2cBus0ReadBytes,
           (unsigned long)telemetry->i2cBus1ReadBytes,
           (unsigned long long)bus0WireUs,
           (unsigned long long)bus1WireUs,
           (unsigned long long)telemetry->i2cBus0TotalUs,
           (unsigned long long)telemetry->i2cBus1TotalUs,
           (unsigned long long)bus0DriverUs,
           (unsigned long long)bus1DriverUs,
           Fdc2214CapDataReadModeName((FdcDataReadMode)telemetry->dataReadMode),
           (unsigned)telemetry->sequenceRegsPerTransaction,
           (unsigned)telemetry->sequenceTransactionsPerRow,
           (unsigned long)telemetry->i2cSequenceDataReadCount,
           (unsigned long)telemetry->i2cSequenceTransactionCount,
           (unsigned long)telemetry->i2cSequenceFallbackCount,
           (unsigned long)telemetry->i2cSequenceErrorCount,
           (unsigned long)telemetry->i2cBurstDataReadCount,
           (unsigned long)telemetry->i2cOrderedDataReadCount,
           (unsigned long)telemetry->i2cBurstFallbackCount,
           (unsigned long)telemetry->i2cNackCount,
           (unsigned long)telemetry->i2cTimeoutCount,
           (unsigned long)telemetry->i2cRecoveryCount);

    uint64_t directAccepted = telemetry->directDataReadCount >= telemetry->directDataFallbackCount ?
        telemetry->directDataReadCount - telemetry->directDataFallbackCount : 0u;
    uint64_t directValidRateX100 = telemetry->directDataReadCount ?
        (directAccepted * 10000ull) / telemetry->directDataReadCount : 0u;
    printf("FAST20,directDataReads=%lu,statusReads=%lu,fallbacks=%lu,fallbackReasonMask=0x%lX,directValidRate=%llu.%02llu,statusSavedReads=%lu\n",
           (unsigned long)telemetry->directDataReadCount,
           (unsigned long)telemetry->statusReadCount,
           (unsigned long)telemetry->directDataFallbackCount,
           (unsigned long)telemetry->directDataFallbackReasonMask,
           (unsigned long long)(directValidRateX100 / 100ull),
           (unsigned long long)(directValidRateX100 % 100ull),
           (unsigned long)telemetry->statusSavedReadCount);

    uint64_t cacheCompareAvgUs = telemetry->cacheCompareCount ?
        telemetry->cacheCompareUs / telemetry->cacheCompareCount : 0u;
    uint64_t cacheApplyAvgUs = telemetry->cacheRestartRows ?
        telemetry->cacheApplyI2cUs / telemetry->cacheRestartRows : 0u;
    printf("CACHE20,compare=%lu,compareUsAvg=%llu,diffRows=%lu,writes=%lu,applyUsAvg=%llu,applyUsMax=%llu,restartRows=%lu,skipRows=%lu\n",
           (unsigned long)telemetry->cacheCompareCount,
           (unsigned long long)cacheCompareAvgUs,
           (unsigned long)telemetry->cacheDiffRows,
           (unsigned long)telemetry->cacheWriteCount,
           (unsigned long long)cacheApplyAvgUs,
           (unsigned long long)summary->cacheApplyI2cMaxUs,
           (unsigned long)telemetry->cacheRestartRows,
           (unsigned long)telemetry->cacheSkipRows);

    uint64_t rowDeviceCount = measureFrames * SENSORARRAY_MATRIX_ROWS;
    printf("PIPE20,routeUsAvg=%llu,settleUsAvg=%llu,cacheCompareUsAvg=%llu,cacheApplyUsAvg=%llu,readyWaitUsAvg=%llu,statusReadUsAvg=%llu,dataReadUsAvg=%llu,mergeUsAvg=%llu,primaryRunUsAvg=%llu,secondaryRunUsAvg=%llu,barrierSkewUsAvg=%llu,doneSkewUsAvg=%llu\n",
           (unsigned long long)(rowDeviceCount ? telemetry->routeUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->settleUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->cacheCompareUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->cacheApplyI2cUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->readyWaitUs / (rowDeviceCount * 2u) : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->statusReadUs / (rowDeviceCount * 2u) : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->dataReadUs / (rowDeviceCount * 2u) : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->mergeUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->primaryWorkerRunUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->secondaryWorkerRunUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerStartSkewUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerDoneSkewUs / rowDeviceCount : 0u));

    uint64_t routeAvgUs = rowDeviceCount ? telemetry->routeUs / rowDeviceCount : 0u;
    uint64_t settleAvgUs = rowDeviceCount ? telemetry->settleUs / rowDeviceCount : 0u;
    uint64_t cacheCompareRowAvgUs =
        rowDeviceCount ? telemetry->cacheCompareUs / rowDeviceCount : 0u;
    uint64_t cacheApplyRowAvgUs =
        rowDeviceCount ? telemetry->cacheApplyI2cUs / rowDeviceCount : 0u;
    uint64_t mergeAvgUs = rowDeviceCount ? telemetry->mergeUs / rowDeviceCount : 0u;
    uint64_t primaryRunAvgUs =
        rowDeviceCount ? telemetry->primaryWorkerRunUs / rowDeviceCount : 0u;
    uint64_t secondaryRunAvgUs =
        rowDeviceCount ? telemetry->secondaryWorkerRunUs / rowDeviceCount : 0u;
    uint64_t maxWorkerRunAvgUs =
        primaryRunAvgUs > secondaryRunAvgUs ? primaryRunAvgUs : secondaryRunAvgUs;
    uint64_t attributedRowUs = routeAvgUs + settleAvgUs + cacheCompareRowAvgUs +
                               cacheApplyRowAvgUs + maxWorkerRunAvgUs + mergeAvgUs;
    uint64_t coordinatorResidualAvgUs =
        rowStepAvgUs > attributedRowUs ? rowStepAvgUs - attributedRowUs : 0u;
    printf("ROWPIPE20,rowRouteUsAvg=%llu,rowSettleUsAvg=%llu,cacheCompareUsAvg=%llu,workerNotifyUsAvg=%llu,rowSleepBarrierUsAvg=%llu,workerPreReleaseUsAvg=%llu,workerSleepAckWaitUsAvg=%llu,workerStartGiveUsAvg=%llu,workerWaitPrimaryUsAvg=%llu,workerWaitSecondaryUsAvg=%llu,parentWaitBothUsAvg=%llu,workerJoinUsAvg=%llu,rowMergeUsAvg=%llu,frameMaskUpdateUsAvg=%llu,frameBookkeepingUsAvg=%llu,frameQueueUsAvg=%llu,rowCoordinatorResidualUsAvg=%llu\n",
           (unsigned long long)routeAvgUs,
           (unsigned long long)settleAvgUs,
           (unsigned long long)cacheCompareRowAvgUs,
           (unsigned long long)(rowDeviceCount ? telemetry->workerNotifyUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->rowSleepBarrierUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerPreReleaseUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerSleepAckWaitUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerStartGiveUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerWaitPrimaryUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerWaitSecondaryUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->parentWaitBothUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->workerJoinUs / rowDeviceCount : 0u),
           (unsigned long long)mergeAvgUs,
           (unsigned long long)(rowDeviceCount ? telemetry->frameMaskUpdateUs / rowDeviceCount : 0u),
           (unsigned long long)(rowDeviceCount ? telemetry->frameBookkeepingUs / rowDeviceCount : 0u),
           (unsigned long long)(measureFrames ? telemetry->frameQueueUs / measureFrames : 0u),
           (unsigned long long)coordinatorResidualAvgUs);

    double pfStdMax = 0.0;
    double pfStdTotal = 0.0;
    uint32_t pfStdCells = 0u;
    for (size_t cell = 0u; cell < SENSORARRAY_MATRIX_CELL_COUNT; ++cell) {
        uint32_t count = s_precisionWindow.sampleCount[cell];
        if (count > 1u) {
            double std = sqrt(s_precisionWindow.m2[cell] / (double)(count - 1u));
            if (std > pfStdMax) {
                pfStdMax = std;
            }
            pfStdTotal += std;
            pfStdCells++;
        }
    }
    double pfStdAvg = pfStdCells != 0u ? pfStdTotal / (double)pfStdCells : 0.0;
    bool precisionGuardPass =
        s_precisionWindow.invalidCells == 0u &&
        freshFrames == measureFrames &&
        staleFrames == 0u &&
        mixedFrames == 0u &&
        rowFreshMaskBad == 0u &&
        primaryFreshMaskBad == 0u &&
        secondaryFreshMaskBad == 0u &&
        telemetry->i2cSequenceFallbackCount == 0u &&
        telemetry->i2cSequenceErrorCount == 0u;
    printf("PREC20,frames=%lu,cells=%u,pfMeanDriftMax=%.9f,pfStdMax=%.9f,pfStdAvg=%.9f,rawJumpMax=%lu,invalidCells=%lu,precisionGuard=%s\n",
           (unsigned long)s_precisionWindow.frames,
           (unsigned)SENSORARRAY_MATRIX_CELL_COUNT,
           s_precisionWindow.pfMeanDriftMax,
           pfStdMax,
           pfStdAvg,
           (unsigned long)s_precisionWindow.rawJumpMax,
           (unsigned long)s_precisionWindow.invalidCells,
           precisionGuardPass ? "pass" : "fail");

    uint64_t readyWaitAvgUs = rowDeviceCount ?
        telemetry->readyWaitUs / (rowDeviceCount * 2u) : 0u;
    uint64_t dataReadAvgUs = rowDeviceCount ?
        telemetry->dataReadUs / (rowDeviceCount * 2u) : 0u;
    int64_t theoryGapUs = (int64_t)readyWaitAvgUs - (int64_t)summary->fdcTheoryReadyUs;
    printf("FDCSPD20,rCnt=0x%04X,setCnt=0x%04X,act=%s,hiDrv=%u,idrive=0x%04X,deglitch=%u,fref=%lu,theoryReadyUs=%lu,theoryFrameUs=%lu,switchDelayUs=%lu,readyWaitUsAvg=%llu,dataReadUsAvg=%llu,coordUsAvg=%llu,gapUs=%lld,profile=%s,safeFast=%s,config=0x%04X,mux=0x%04X\n",
           summary->fdcRcount,
           summary->fdcSettleCount,
           summary->fdcSensorActivateFullCurrent ? "full" : "low",
           summary->fdcHighCurrentDrive ? 1u : 0u,
           summary->fdcDriveCurrent,
           (unsigned)summary->fdcDeglitch,
           (unsigned long)summary->fdcFrefHz,
           (unsigned long)summary->fdcTheoryReadyUs,
           (unsigned long)summary->fdcTheoryFrameReadyUs,
           (unsigned long)summary->fdcTheorySwitchDelayUs,
           (unsigned long long)readyWaitAvgUs,
           (unsigned long long)dataReadAvgUs,
           (unsigned long long)coordinatorResidualAvgUs,
           (long long)theoryGapUs,
           CONFIG_SENSORARRAY_FDC_RUNTIME_PROFILE_NAME,
           CONFIG_SENSORARRAY_FDC_SPEED_PROFILE_SAFE_FAST ? "report-only" : "off",
           summary->fdcConfig,
           summary->fdcMuxConfig);

    const sensorarrayAdsGapSnapshot_t *ads = &summary->adsGap;
    uint32_t gapWindows = ads->windows - summary->adsGapStart.windows;
    uint32_t gapRun = ads->jobsRun - summary->adsGapStart.jobsRun;
    uint32_t gapSkip = ads->jobsSkip - summary->adsGapStart.jobsSkip;
    uint32_t gapOverrun = ads->overrunCount - summary->adsGapStart.overrunCount;
    uint32_t dmaReads = ads->dmaReadCount - summary->adsGapStart.dmaReadCount;
    uint64_t dmaReadUs = ads->dmaReadUs - summary->adsGapStart.dmaReadUs;
    printf("ACQGAP20,windows=%lu,adsJobsRun=%lu,adsJobsSkip=%lu,adsOverrun=%lu,guardUs=%lu,avgSlackUs=%lu,minSlackUs=%lu,fallbackToBoundary=%u,fdcMask=0x%02X,adsMask=0x%02X\n",
           (unsigned long)gapWindows,
           (unsigned long)gapRun,
           (unsigned long)gapSkip,
           (unsigned long)gapOverrun,
           (unsigned long)ads->guardUs,
           (unsigned long)ads->avgSlackUs,
           (unsigned long)ads->minSlackUs,
           ads->fallbackToBoundary ? 1u : 0u,
           (unsigned)SENSORARRAY_FDC_ROW_RESOURCE_MASK,
           (unsigned)SENSORARRAY_ADS_DIRECT_RESOURCE_MASK);
    printf("ADSDMA20,dmaReadUsAvg=%llu,pollReadUsAvg=0,selected=%s,spiErr=%lu,drdyTimeout=%lu\n",
           (unsigned long long)(dmaReads ? dmaReadUs / dmaReads : 0u),
           ads->dmaCapable ? "dma" : "unavailable",
           (unsigned long)ads->spiErrorCount,
           (unsigned long)ads->drdyTimeoutCount);
    double adsOffsetStdUv = summary->adsOffsetCount > 1u ?
        sqrt(summary->adsOffsetM2 / (double)(summary->adsOffsetCount - 1u)) : 0.0;
    printf("ADS20,start=%d,drdy=%d,init=%u,type=ADS1263,id=0x%02X,adc=ADC1,dma=%u,rate=%lu,ain9OffsetUv=%ld,ain9StdUv=%.1f,ain8RawUv=%ld,battMv=%ld,battValid=%u,drdyTimeout=%lu,spiErr=%lu,diagAgeFrames=%lu,gapJobsRun=%lu,gapJobsSkip=%lu,gapOverrun=%lu\n",
           CONFIG_SENSORARRAY_ADS_START_GPIO,
           CONFIG_SENSORARRAY_ADS_DRDY_GPIO,
           ads->initialized ? 1u : 0u,
           ads->id,
           ads->dmaCapable ? 1u : 0u,
           (unsigned long)ads126xAdcDataRateCodeToSps(ads->rateCode),
           (long)ads->ain9OffsetUv,
           adsOffsetStdUv,
           (long)ads->ain8RawUv,
           (long)ads->batteryMv,
           ads->batteryValid ? 1u : 0u,
           (unsigned long)ads->drdyTimeoutCount,
           (unsigned long)ads->spiErrorCount,
           (unsigned long)ads->sampleAgeFrames,
           (unsigned long)gapRun,
           (unsigned long)gapSkip,
           (unsigned long)gapOverrun);

    sensorarrayNetStatus_t netStatus = {
        .timestampUs = summary->lastFrameStartUs,
        .sequence = summary->seqEnd,
        .physFpsX100 = (uint32_t)physicalFpsX100,
        .cellFreshFpsX100 = (uint32_t)cellFreshFpsX100,
        .emitFpsX100 = (uint32_t)outputFpsX100,
        .rowStepUsAvg = (uint32_t)rowStepAvgUs,
        .readyWaitUsAvg = (uint32_t)readyWaitAvgUs,
        .dataReadUsAvg = (uint32_t)dataReadAvgUs,
        .coordinatorResidualUsAvg = (uint32_t)coordinatorResidualAvgUs,
        .sequenceFallbacks = telemetry->i2cSequenceFallbackCount,
        .sequenceErrors = telemetry->i2cSequenceErrorCount,
        .nack = telemetry->i2cNackCount,
        .timeout = telemetry->i2cTimeoutCount,
        .recover = telemetry->i2cRecoveryCount,
        .directValidRateX100 = (uint32_t)directValidRateX100,
        .pfStdAvgNano = (uint32_t)(pfStdAvg * 1000000000.0),
        .pfStdMaxNano = (uint32_t)(pfStdMax * 1000000000.0),
        .adsAin8RawUv = ads->ain8RawUv,
        .batteryMv = ads->batteryMv,
        .adsAin9OffsetUv = ads->ain9OffsetUv,
        .heapFree = esp_get_free_heap_size(),
        .heapMin = esp_get_minimum_free_heap_size(),
        .logStackHighWater = uxTaskGetStackHighWaterMark(NULL),
        .rowFreshMask = rowFreshMaskBad == 0u ? 0xFFu : 0u,
        .primaryFreshMask = primaryFreshMaskBad == 0u ? 0xFFu : 0u,
        .secondaryFreshMask = secondaryFreshMaskBad == 0u ? 0xFFu : 0u,
        .stale = staleFrames != 0u,
        .mixed = mixedFrames != 0u,
        .precisionPass = precisionGuardPass,
        .batteryValid = ads->batteryValid,
    };
    (void)sensorarrayNetStatusPublish(&netStatus);

    sensorarrayAsyncLogSummarySetBaseline(summary, &stats);
}

static void sensorarrayAsyncLogTask(void *arg)
{
    (void)arg;
    printf("TASKCORE,name=log,core=%d,expected=%d\n",
           (int)xPortGetCoreID(),
           CONFIG_SENSORARRAY_LOG_TASK_CORE);
    sensorarrayAsyncLogSummary_t summary = {0};
    sensorarrayFrame_t frame;
    uint32_t framesSinceIdleYield = 0u;

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
            bool emitted = frame.freshFrame || CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG;
            uint64_t outputUs = 0u;
            if (emitted) {
                frame.emitUs = (uint64_t)outputStartUs;
                (void)sensorarrayFrameOutputPrint(&frame);
                outputUs = sensorarrayAsyncLogElapsedPositiveUs(outputStartUs);
            }

            sensorarrayAsyncLogUpdateSummary(&summary,
                                             &frame,
                                             measureFrameUs,
                                             frameAgeUs,
                                             outputUs,
                                             queueDepth,
                                             emitted);
            sensorarrayAsyncLogMaybePrintSummary(&summary);
            framesSinceIdleYield++;
            if (framesSinceIdleYield >= 40u) {
                framesSinceIdleYield = 0u;
                vTaskDelay(1u);
            }
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
    printf("APP_LOG_INIT,mode=async,frameFormat=%s,frameSlots=%u,eventQueueLen=%u,summaryEvery=%u,taskPrio=%u,taskCore=%d,dropOldFrames=%u\n",
           CONFIG_SENSORARRAY_FRAME_OUTPUT_BINARY_COMPACT_V1 ? "binary_v1" : "text",
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
    s_sharedStats.freshFrames += frame->freshFrame ? 1u : 0u;
    s_sharedStats.staleFrames += frame->stale ? 1u : 0u;
    s_sharedStats.mixedFrames += frame->mixedEpoch ? 1u : 0u;
    s_sharedStats.rowFreshMaskBad += frame->rowFreshMask != 0xFFu ? 1u : 0u;
    s_sharedStats.primaryFreshMaskBad += frame->primaryFreshMask != 0xFFu ? 1u : 0u;
    s_sharedStats.secondaryFreshMaskBad += frame->secondaryFreshMask != 0xFFu ? 1u : 0u;
    s_sharedStats.freshCells += frame->freshCount;
    s_sharedStats.physicalSweepTotalUs += frame->physicalSweepUs;
    if (frame->physicalSweepUs > s_sharedStats.physicalSweepMaxUs) {
        s_sharedStats.physicalSweepMaxUs = frame->physicalSweepUs;
    }
    s_sharedStats.rowStepTotalUs += frame->rowStepUsAvg * SENSORARRAY_MATRIX_ROWS;
    if (frame->rowStepUsMax > s_sharedStats.rowStepMaxUs) {
        s_sharedStats.rowStepMaxUs = frame->rowStepUsMax;
    }
    if (s_sharedStats.lastFrameStartUs != 0u &&
        frame->frameStartUs > s_sharedStats.lastFrameStartUs) {
        uint64_t intervalUs = frame->frameStartUs - s_sharedStats.lastFrameStartUs;
        s_sharedStats.frameStartIntervalTotalUs += intervalUs;
        s_sharedStats.frameStartIntervalCount++;
        if (intervalUs > s_sharedStats.frameStartIntervalMaxUs) {
            s_sharedStats.frameStartIntervalMaxUs = intervalUs;
        }
    }
    s_sharedStats.lastFrameStartUs = frame->frameStartUs;
    s_sharedStats.measureFrameTotalUs += measureFrameUs;
    if (measureFrameUs > s_sharedStats.measureFrameMaxUs) {
        s_sharedStats.measureFrameMaxUs = measureFrameUs;
    }
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
        s_frameSlots[slot].frame.telemetry.frameQueueUs =
            (uint64_t)(esp_timer_get_time() - publishedUs);

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
