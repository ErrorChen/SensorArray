#include "sensorarrayAsyncLog.h"

#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "boardSupport.h"
#include "sensorarrayCommandMailbox.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayBle.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFrameOutput.h"
#include "sensorarrayNetStatus.h"
#include "sensorarrayOutputPolicy.h"
#include "sensorarrayTextProtocol.h"
#include "sensorarrayTransport.h"
#include "sensorarrayTypes.h"
#include "sensorarrayUsbSink.h"

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
#define CONFIG_SENSORARRAY_ASYNC_LOG_SUMMARY_EVERY_N_FRAMES 50
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
#ifndef CONFIG_SENSORARRAY_BLE_CAP_TEXT_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_BLE_CAP_TEXT_EVERY_N_FRAMES 0
#endif

#define SENSORARRAY_ASYNC_EVENT_TEXT_MAX 384u

enum {
    SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT =
        (CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS < 2) ? 2 : CONFIG_SENSORARRAY_ASYNC_LOG_FRAME_SLOTS,
    SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT =
        (CONFIG_SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_LEN < 1) ? 1 : CONFIG_SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_LEN,
};

typedef enum {
    SENSORARRAY_ASYNC_LOG_EVENT_OVERRUN = 0,
    SENSORARRAY_ASYNC_LOG_EVENT_FRAME_ERROR,
    SENSORARRAY_ASYNC_LOG_EVENT_COMMAND_APPLIED,
    SENSORARRAY_ASYNC_LOG_EVENT_TEXT,
} sensorarrayAsyncLogEventType_t;

typedef struct {
    sensorarrayMeasurementMode_t mode;
    uint64_t frames;
    uint64_t precisionFrames;
    uint64_t frameUs;
    uint64_t frameUsMax;
    uint64_t rowRouteUs;
    uint64_t muxWriteUs;
    uint64_t registerWriteUs;
    uint64_t registerReadbackUs;
    uint64_t drdyWaitUs;
    uint64_t sampleReadUs;
    uint64_t aggregationUs;
    uint64_t autorangeUs;
    uint64_t batteryUs;
    uint64_t adsCheckUs;
    uint64_t profileHits;
    uint64_t profileMisses;
    uint64_t bypassHits;
    uint64_t gainHits;
    uint64_t registerCacheHits;
    uint64_t registerWrites;
    uint64_t registerReadbacks;
    uint64_t singleSampleCells;
    uint64_t tripleSampleCells;
    uint64_t freshCells;
    uint64_t rawConversions;
    uint64_t autorangeAttempts;
    uint64_t profileInvalidations;
    uint64_t railFingerprintHits;
    uint64_t railFingerprintMisses;
    uint64_t railInvalidations;
} sensorarrayAdsPerformanceSummary_t;

typedef struct {
    sensorarrayFrame_t frame;
    sensorarrayTextPacket_t textPacket;
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
            sensorarrayMeasurementMode_t measurementMode;
            uint64_t measurementValidMask;
            uint64_t measurementFreshMask;
            uint64_t capValidMask;
            uint64_t errorMask;
            uint64_t warnMask;
            uint64_t frameDurationUs;
            uint32_t gainChangeCount;
            uint32_t overrangeCount;
            uint32_t autorangeAttemptCount;
            uint32_t drdyTimeoutCount;
            uint32_t staleCount;
            uint32_t spiErrorCount;
            uint32_t statusErrorCount;
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
            bool adsFrame;
        } frameError;
        struct {
            sensorarrayCommandType_t type;
            uint32_t value;
        } command;
        struct {
            uint16_t length;
            char bytes[SENSORARRAY_ASYNC_EVENT_TEXT_MAX];
        } text;
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
    sensorarrayAdsPerformanceSummary_t adsPerformance;
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
    uint8_t activeRows;
    sensorarrayUsbSinkStats_t usbStatsStart;
    sensorarrayNetSinkStats_t netStatsStart;
} sensorarrayAsyncLogSummary_t;

typedef struct {
    sensorarrayAsyncLogSummary_t summary;
    sensorarrayFrame_t frame;
    sensorarrayTextPacket_t framePacket;
    sensorarrayTextPacket_t summaryPacket;
    sensorarrayTextPacket_t adsPacket;
} sensorarrayAsyncLogWorkspace_t;

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
static uint32_t s_summaryTruncated;
static uint32_t s_initialFreeHeap;
/*
 * Owned exclusively by sensorarrayLogTask. No producer, callback, or other
 * task may read or write this workspace directly. Keeping the complete frame
 * and wire packets here removes their overlapping lifetime from the task
 * stack; individual fields are reset at their real lifecycle boundaries.
 */
static sensorarrayAsyncLogWorkspace_t s_asyncLogWorkspace;

static uint64_t sensorarrayAsyncLogElapsedPositiveUs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    return elapsedUs > 0 ? (uint64_t)elapsedUs : 0u;
}

static size_t sensorarrayAsyncLogTextAppend(char *buffer,
                                            size_t bufferSize,
                                            size_t position,
                                            const char *format,
                                            ...)
{
    if (!buffer || bufferSize == 0u || position >= bufferSize || !format) {
        return position;
    }
    va_list args;
    va_start(args, format);
    int written = vsnprintf(&buffer[position], bufferSize - position, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= bufferSize - position) {
        return bufferSize;
    }
    return position + (size_t)written;
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
                                        sensorarrayTextPacket_t *outTextPacket,
                                        uint64_t *outMeasureFrameUs,
                                        int64_t *outPublishedUs,
                                        uint32_t *outQueueDepth)
{
    if (!outFrame || !outTextPacket || !outMeasureFrameUs ||
        !outPublishedUs || !outQueueDepth) {
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
    *outQueueDepth = s_pendingCount;
    s_frameSlotState[slot] = 3u; /* exclusively owned by the consumer */
    portEXIT_CRITICAL(&s_asyncLogMux);

    /* Copy the two large snapshots after ownership has transferred and after
     * leaving the critical section. Producers cannot reuse state=3 slots. */
    *outFrame = s_frameSlots[slot].frame;
    *outTextPacket = s_frameSlots[slot].textPacket;
    *outMeasureFrameUs = s_frameSlots[slot].measureFrameUs;
    *outPublishedUs = s_frameSlots[slot].publishedUs;

    portENTER_CRITICAL(&s_asyncLogMux);
    if (s_frameSlotState[slot] == 3u) {
        s_frameSlotState[slot] = 0u;
    }
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
        if (event->data.frameError.adsFrame) {
            const char *stage = event->data.frameError.measurementValidMask == 0u ?
                "all_invalid" : "io_error";
            printf("ADSFRAME,stage=%s,mode=%s,seq=%lu,valid=0x%016llX,error=0x%016llX,fresh=0x%016llX,err=0x%lx,gainChanges=%lu,overrange=%lu,attempts=%lu,timeout=%lu,stale=%lu,spi=%lu,status=%lu,durationUs=%llu,async=1\n",
                   stage,
                   sensorarrayMeasurementModeName(
                       event->data.frameError.measurementMode),
                   (unsigned long)event->sequence,
                   (unsigned long long)event->data.frameError.measurementValidMask,
                   (unsigned long long)event->data.frameError.errorMask,
                   (unsigned long long)event->data.frameError.measurementFreshMask,
                   (unsigned long)event->data.frameError.readErr,
                   (unsigned long)event->data.frameError.gainChangeCount,
                   (unsigned long)event->data.frameError.overrangeCount,
                   (unsigned long)event->data.frameError.autorangeAttemptCount,
                   (unsigned long)event->data.frameError.drdyTimeoutCount,
                   (unsigned long)event->data.frameError.staleCount,
                   (unsigned long)event->data.frameError.spiErrorCount,
                   (unsigned long)event->data.frameError.statusErrorCount,
                   (unsigned long long)event->data.frameError.frameDurationUs);
        } else if (event->data.frameError.capValidMask == 0u) {
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
    case SENSORARRAY_ASYNC_LOG_EVENT_COMMAND_APPLIED:
        printf("E,fr=%lu,type=cmd,name=%s,value=%lu,state=applied\n",
               (unsigned long)event->sequence,
               sensorarrayCommandMailboxTypeName(event->data.command.type),
               (unsigned long)event->data.command.value);
        break;
    case SENSORARRAY_ASYNC_LOG_EVENT_TEXT:
        (void)fwrite(event->data.text.bytes,
                     1u,
                     event->data.text.length,
                     stdout);
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
    sensorarrayUsbSinkStats_t usbStats = {0};
    sensorarrayNetSinkStats_t netStats = {0};
    sensorarrayUsbSinkGetStats(&usbStats);
    sensorarrayNetGetSinkStats(&netStats);
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
        .usbStatsStart = usbStats,
        .netStatsStart = netStats,
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
    sensorarrayUsbSinkStats_t usbStats = {0};
    sensorarrayNetSinkStats_t netStats = {0};
    sensorarrayUsbSinkGetStats(&usbStats);
    sensorarrayNetGetSinkStats(&netStats);
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
        .usbStatsStart = usbStats,
        .netStatsStart = netStats,
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
    summary->activeRows = frame->activeRows ? frame->activeRows : SENSORARRAY_MATRIX_ROWS;
    summary->rowStepTotalUs += frame->rowStepUsAvg * summary->activeRows;
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
    uint8_t expectedRowMask = (uint8_t)((1u << summary->activeRows) - 1u);
    if (frame->rowFreshMask != expectedRowMask) {
        summary->rowFreshMaskBad++;
    }
    if (frame->primaryFreshMask != expectedRowMask) {
        summary->primaryFreshMaskBad++;
    }
    if (frame->secondaryFreshMask != expectedRowMask) {
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
    if (frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ||
        frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE) {
        sensorarrayAdsPerformanceSummary_t *adsPerformance =
            &summary->adsPerformance;
        const sensorarrayMeasurementPayload_t *measurement = &frame->measurement;
        adsPerformance->mode = measurement->mode;
        adsPerformance->frames++;
        adsPerformance->precisionFrames += measurement->precisionFrame ? 1u : 0u;
        adsPerformance->frameUs += measurement->frameDurationUs;
        if (measurement->frameDurationUs > adsPerformance->frameUsMax) {
            adsPerformance->frameUsMax = measurement->frameDurationUs;
        }
#define SENSORARRAY_ADS_PERFORMANCE_ADD(target, source) \
        adsPerformance->target += measurement->source
        SENSORARRAY_ADS_PERFORMANCE_ADD(rowRouteUs, rowRouteUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(muxWriteUs, muxWriteUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(registerWriteUs, registerWriteUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(registerReadbackUs, registerReadbackUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(drdyWaitUs, drdyWaitUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(sampleReadUs, sampleReadUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(aggregationUs, aggregationUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(autorangeUs, autorangeUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(batteryUs, batteryUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(adsCheckUs, adsCheckUs);
        SENSORARRAY_ADS_PERFORMANCE_ADD(profileHits, profileHitCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(profileMisses, profileMissCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(bypassHits, bypassHitCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(gainHits, gainHitCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(registerCacheHits, registerCacheHitCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(registerWrites, registerWriteCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(registerReadbacks, registerReadbackCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(singleSampleCells, singleSampleCellCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(tripleSampleCells, tripleSampleCellCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(freshCells, freshCellCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(rawConversions, rawConversionCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(autorangeAttempts, autorangeAttemptCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(profileInvalidations,
                                         profileInvalidationCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(railFingerprintHits,
                                         railFingerprintHitCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(railFingerprintMisses,
                                         railFingerprintMissCount);
        SENSORARRAY_ADS_PERFORMANCE_ADD(railInvalidations,
                                         railInvalidationCount);
#undef SENSORARRAY_ADS_PERFORMANCE_ADD
    }
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

static const char *sensorarrayBatteryReasonName(sensorarrayBatteryInvalidReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_BATTERY_INVALID_DISABLED:
        return "disabled";
    case SENSORARRAY_BATTERY_INVALID_NOT_DUE:
        return "not_due";
    case SENSORARRAY_BATTERY_INVALID_DEFERRED:
        return "deferred";
    case SENSORARRAY_BATTERY_INVALID_CAL:
        return "cal";
    case SENSORARRAY_BATTERY_INVALID_RAIL:
        return "rail_invalid";
    case SENSORARRAY_BATTERY_INVALID_ZERO:
        return "zero";
    case SENSORARRAY_BATTERY_INVALID_ADC:
        return "adc_fail";
    case SENSORARRAY_BATTERY_INVALID_ADC_TIMEOUT:
        return "adc_timeout";
    case SENSORARRAY_BATTERY_INVALID_ADC_STALE:
        return "adc_stale";
    case SENSORARRAY_BATTERY_INVALID_ADC_STATUS_ERROR:
        return "adc_status_error";
    case SENSORARRAY_BATTERY_INVALID_SPI_ERROR:
        return "spi_error";
    case SENSORARRAY_BATTERY_INVALID_RESTORE_FAILED:
        return "restore_failed";
    case SENSORARRAY_BATTERY_INVALID_VBIAS:
        return "vbias_invalid";
    case SENSORARRAY_BATTERY_INVALID_DIV:
        return "divider_invalid";
    case SENSORARRAY_BATTERY_INVALID_NO_AINCOM_GND_REFERENCE:
        return "no_aincom_gnd_reference";
    case SENSORARRAY_BATTERY_INVALID_REFERENCE_INVALID:
        return "reference_invalid";
    case SENSORARRAY_BATTERY_INVALID_ABSENT_OR_OPEN:
        return "absent_or_open";
    case SENSORARRAY_BATTERY_INVALID_RANGE_ERROR:
        return "range_error";
    case SENSORARRAY_BATTERY_INVALID_UNSTABLE:
        return "unstable";
    case SENSORARRAY_BATTERY_INVALID_SATURATED:
        return "saturated";
    case SENSORARRAY_BATTERY_INVALID_OUT_OF_RANGE:
        return "out_of_range";
    case SENSORARRAY_BATTERY_INVALID_OVERFLOW:
        return "overflow";
    case SENSORARRAY_BATTERY_INVALID_NONE:
        return "ok";
    case SENSORARRAY_BATTERY_INVALID_UNKNOWN:
    default:
        return "unk";
    }
}

static void sensorarrayAsyncLogPrintCompactSummary(sensorarrayAsyncLogSummary_t *summary)
{
    if (!summary || !summary->active) {
        return;
    }

    uint32_t every = (uint32_t)SENSORARRAY_CFG_LOG_PERIOD_FRAMES;
    if (every == 0u) {
        every = 50u;
    }
    if (sensorarrayCommandMailboxAdsDebugEnabled() &&
        summary->adsPerformance.frames != 0u) {
        every = 1u;
    }
    uint64_t frameCount = summary->processedFrames;
    if (frameCount < every) {
        return;
    }

    sensorarrayAsyncLogSharedStats_t sharedStats = sensorarrayAsyncLogReadStats();
    sensorarrayUsbSinkStats_t usbStats = {0};
    sensorarrayNetSinkStats_t netStats = {0};
    sensorarrayTransportStats_t transportStats = {0};
    sensorarrayTransportTaskStackStats_t transportTaskStacks = {0};
    sensorarrayBleStats_t bleStats = {0};
    sensorarrayUsbSinkGetStats(&usbStats);
    sensorarrayNetGetSinkStats(&netStats);
    sensorarrayTransportGetStats(&transportStats);
    sensorarrayTransportGetTaskStackStats(&transportTaskStacks);
    sensorarrayBleGetStats(&bleStats);

    uint64_t captureIntervalUs = summary->frameStartIntervalCount ?
        summary->frameStartIntervalTotalUs / summary->frameStartIntervalCount : 0u;
    uint64_t emitIntervalUs = summary->emitIntervalCount ?
        summary->emitIntervalTotalUs / summary->emitIntervalCount : 0u;
    uint64_t captureFpsX100 = captureIntervalUs ? 100000000ull / captureIntervalUs : 0u;
    uint64_t emitFpsX100 = emitIntervalUs ? 100000000ull / emitIntervalUs : 0u;
    uint8_t activeRows = summary->activeRows ? summary->activeRows : SENSORARRAY_MATRIX_ROWS;
    uint64_t rowCount = frameCount * activeRows;
    const sensorarrayFdcFrameTelemetry_t *telemetry = &summary->telemetry;
    uint64_t readyWaitUs = rowCount ? telemetry->readyWaitUs / (rowCount * 2u) : 0u;
    uint64_t dataReadUs = rowCount ? telemetry->dataReadUs / (rowCount * 2u) : 0u;
    uint64_t rowStepUs = rowCount ? summary->rowStepTotalUs / rowCount : 0u;
    uint64_t attributedUs = rowCount ?
        (telemetry->routeUs + telemetry->settleUs + telemetry->cacheCompareUs +
         telemetry->cacheApplyI2cUs + telemetry->mergeUs) / rowCount : 0u;
    uint64_t coordinatorUs = rowStepUs > attributedUs ? rowStepUs - attributedUs : 0u;
    uint64_t validRate = frameCount ?
        (summary->freshCells * 100u) /
            (frameCount * (uint64_t)activeRows * SENSORARRAY_MATRIX_COLS) : 0u;
    uint64_t invalidFrames = frameCount > summary->freshFrames ?
        frameCount - summary->freshFrames : 0u;

    int64_t nowUs = esp_timer_get_time();
    uint64_t windowUs = nowUs > summary->windowStartUs ?
        (uint64_t)(nowUs - summary->windowStartUs) : 1u;
    uint64_t usbPackets = usbStats.sentPackets - summary->usbStatsStart.sentPackets;
    uint64_t blePackets = netStats.bleSentPackets - summary->netStatsStart.bleSentPackets;
    uint64_t wifiPackets = netStats.wifiSentPackets - summary->netStatsStart.wifiSentPackets;
    uint64_t usbFpsX10 = (usbPackets * 10000000ull) / windowUs;
    uint64_t bleFpsX10 = (blePackets * 10000000ull) / windowUs;
    uint64_t wifiFpsX10 = (wifiPackets * 10000000ull) / windowUs;
    uint64_t usbDrop = usbStats.droppedPackets - summary->usbStatsStart.droppedPackets;
    uint64_t bleDrop = netStats.bleDroppedPackets - summary->netStatsStart.bleDroppedPackets;
    uint64_t wifiDrop = netStats.wifiDroppedPackets - summary->netStatsStart.wifiDroppedPackets;
    uint64_t usbKb = (usbStats.sentBytes - summary->usbStatsStart.sentBytes) / 1024u;
    uint64_t bleKb = (netStats.bleSentBytes - summary->netStatsStart.bleSentBytes) / 1024u;
    uint64_t wifiKb = (netStats.wifiSentBytes - summary->netStatsStart.wifiSentBytes) / 1024u;
    uint64_t usbBlocked = usbStats.blockedCount - summary->usbStatsStart.blockedCount;
    uint64_t bleBlocked = netStats.bleBlockedCount - summary->netStatsStart.bleBlockedCount;
    uint64_t wifiBlocked = netStats.wifiBlockedCount - summary->netStatsStart.wifiBlockedCount;
    uint64_t textBusDrop = sharedStats.droppedOutputFrames - summary->droppedFrameStart;
    uint64_t eventRingDrop = sharedStats.droppedEventLogs - summary->droppedEventStart;
    uint64_t outputDrop = usbDrop + bleDrop + wifiDrop;
    uint64_t usbWrites = usbPackets ? usbPackets : 1u;
    uint64_t usbWriteUs = usbStats.writeUsTotal - summary->usbStatsStart.writeUsTotal;
    uint64_t outputAvgMs = (usbWriteUs / usbWrites) / 1000u;
    uint64_t outputMaxMs = usbStats.writeUsMax / 1000u;
    uint32_t queueCurrent = usbStats.queueDepth > netStats.queueDepth ?
        usbStats.queueDepth : netStats.queueDepth;
    uint32_t queueMax = usbStats.queueDepthMax > netStats.queueDepthMax ?
        usbStats.queueDepthMax : netStats.queueDepthMax;

    const sensorarrayAdsGapSnapshot_t *ads = &summary->adsGap;
    uint32_t zeroStdUv = ads->zeroResidualStdUv;
    if (zeroStdUv == 0u && summary->adsOffsetCount > 1u) {
        zeroStdUv = (uint32_t)sqrt(summary->adsOffsetM2 /
                                  (double)(summary->adsOffsetCount - 1u));
    }
    uint32_t adsJobsRunDelta = ads->jobsRun - summary->adsGapStart.jobsRun;
    uint32_t adsJobsSkipDelta = ads->jobsSkip - summary->adsGapStart.jobsSkip;
    const char *batteryState = !ads->batteryFresh ? "stale" :
                                (ads->batteryValid ? "present" : "unk");

    sensorarrayTextPacket_t *packet = &s_asyncLogWorkspace.summaryPacket;
    packet->sequence = summary->seqEnd;
    packet->length = 0u;
    size_t position = sensorarrayAsyncLogTextAppend(
        packet->data,
        sizeof(packet->data),
        0u,
        "SF50,seq=%lu-%lu,n=%llu,rows=%u,cfps=%llu.%02llu,efps=%llu.%02llu,ofps=%llu.%01llu/%llu.%01llu/%llu.%01llu,bad=%llu/%llu/%llu,drop=0/%llu/%llu,q=%lu/%lu\n",
        (unsigned long)summary->seqStart,
        (unsigned long)summary->seqEnd,
        (unsigned long long)frameCount,
        (unsigned)activeRows,
        (unsigned long long)(captureFpsX100 / 100u),
        (unsigned long long)(captureFpsX100 % 100u),
        (unsigned long long)(emitFpsX100 / 100u),
        (unsigned long long)(emitFpsX100 % 100u),
        (unsigned long long)(usbFpsX10 / 10u),
        (unsigned long long)(usbFpsX10 % 10u),
        (unsigned long long)(bleFpsX10 / 10u),
        (unsigned long long)(bleFpsX10 % 10u),
        (unsigned long long)(wifiFpsX10 / 10u),
        (unsigned long long)(wifiFpsX10 % 10u),
        (unsigned long long)summary->staleFrames,
        (unsigned long long)summary->mixedFrames,
        (unsigned long long)invalidFrames,
        (unsigned long long)textBusDrop,
        (unsigned long long)outputDrop,
        (unsigned long)queueCurrent,
        (unsigned long)queueMax);
    position = sensorarrayAsyncLogTextAppend(
        packet->data,
        sizeof(packet->data),
        position,
        "TR50,r=%u,fu=%llu,rau=%llu,rmu=%llu,rt=%llu,wt=%llu,rp=%llu,rs=%llu,co=%llu,ag=%llu,agn=%lu,ags=%lu,agf=%u\n",
        (unsigned)activeRows,
        (unsigned long long)captureIntervalUs,
        (unsigned long long)rowStepUs,
        (unsigned long long)summary->rowStepMaxUs,
        (unsigned long long)(rowCount ? telemetry->routeUs / rowCount : 0u),
        (unsigned long long)readyWaitUs,
        (unsigned long long)dataReadUs,
        (unsigned long long)(rowCount ? telemetry->dataReadUs / (rowCount * 2u) : 0u),
        (unsigned long long)coordinatorUs,
        (unsigned long long)(rowCount ? ads->dmaReadUs / rowCount : 0u),
        (unsigned long)adsJobsRunDelta,
        (unsigned long)adsJobsSkipDelta,
        ads->fallbackToBoundary ? 1u : 0u);
    sensorarrayTextPacket_t *adsPacket = &s_asyncLogWorkspace.adsPacket;
    adsPacket->sequence = summary->seqEnd;
    adsPacket->length = 0u;
    size_t adsPosition = 0u;
    const sensorarrayAdsPerformanceSummary_t *adsPerformance =
        &summary->adsPerformance;
    if (adsPerformance->frames != 0u) {
        uint64_t adsFrames = adsPerformance->frames;
        uint64_t attemptsX100 = adsPerformance->freshCells ?
            (adsPerformance->autorangeAttempts * 100u) /
                adsPerformance->freshCells : 0u;
        adsPosition = sensorarrayAsyncLogTextAppend(
            adsPacket->data,
            sizeof(adsPacket->data),
            adsPosition,
            "ADS50,mode=%s,n=%llu,frameUs=%llu/%llu,attemptsPerCell=%llu.%02llu,rawConversions=%llu,profileHit=%llu,profileMiss=%llu,bypassHit=%llu,gainHit=%llu,registerCacheHit=%llu,registerWrites=%llu,registerReadbacks=%llu,singleSampleCells=%llu,tripleSampleCells=%llu,precisionFrame=%u,precisionFrames=%llu,freshCells=%llu,profileInvalidations=%llu,railFingerprintHit=%llu,railFingerprintMiss=%llu,railInvalidations=%llu\n",
            sensorarrayMeasurementModeName(adsPerformance->mode),
            (unsigned long long)adsFrames,
            (unsigned long long)(adsPerformance->frameUs / adsFrames),
            (unsigned long long)adsPerformance->frameUsMax,
            (unsigned long long)(attemptsX100 / 100u),
            (unsigned long long)(attemptsX100 % 100u),
            (unsigned long long)adsPerformance->rawConversions,
            (unsigned long long)adsPerformance->profileHits,
            (unsigned long long)adsPerformance->profileMisses,
            (unsigned long long)adsPerformance->bypassHits,
            (unsigned long long)adsPerformance->gainHits,
            (unsigned long long)adsPerformance->registerCacheHits,
            (unsigned long long)adsPerformance->registerWrites,
            (unsigned long long)adsPerformance->registerReadbacks,
            (unsigned long long)adsPerformance->singleSampleCells,
            (unsigned long long)adsPerformance->tripleSampleCells,
            adsPerformance->precisionFrames != 0u ? 1u : 0u,
            (unsigned long long)adsPerformance->precisionFrames,
            (unsigned long long)adsPerformance->freshCells,
            (unsigned long long)adsPerformance->profileInvalidations,
            (unsigned long long)adsPerformance->railFingerprintHits,
            (unsigned long long)adsPerformance->railFingerprintMisses,
            (unsigned long long)adsPerformance->railInvalidations);
        adsPosition = sensorarrayAsyncLogTextAppend(
            adsPacket->data,
            sizeof(adsPacket->data),
            adsPosition,
            "ADST50,mode=%s,n=%llu,rowRouteUs=%llu,muxWriteUs=%llu,registerWriteUs=%llu,registerReadbackUs=%llu,drdyWaitUs=%llu,sampleReadUs=%llu,aggregationUs=%llu,autorangeUs=%llu,batteryUs=%llu,adsCheckUs=%llu\n",
            sensorarrayMeasurementModeName(adsPerformance->mode),
            (unsigned long long)adsFrames,
            (unsigned long long)(adsPerformance->rowRouteUs / adsFrames),
            (unsigned long long)(adsPerformance->muxWriteUs / adsFrames),
            (unsigned long long)(adsPerformance->registerWriteUs / adsFrames),
            (unsigned long long)(adsPerformance->registerReadbackUs / adsFrames),
            (unsigned long long)(adsPerformance->drdyWaitUs / adsFrames),
            (unsigned long long)(adsPerformance->sampleReadUs / adsFrames),
            (unsigned long long)(adsPerformance->aggregationUs / adsFrames),
            (unsigned long long)(adsPerformance->autorangeUs / adsFrames),
            (unsigned long long)(adsPerformance->batteryUs / adsFrames),
            (unsigned long long)(adsPerformance->adsCheckUs / adsFrames));
    }
    adsPosition = sensorarrayAsyncLogTextAppend(
        adsPacket->data,
        sizeof(adsPacket->data),
        adsPosition,
        "AB50,bt=%ld,valid=%u,br=%s,bs=%s,ageMs=%lu,periodMs=%lu,due=%u,run=%lu,validRun=%lu,invalidRun=%lu,skip=%lu,defer=%lu,boundary=%lu,restoreFail=%lu,retry=%lu/%lu,unstable=%lu,timeout=%lu,spreadRaw=%lu,spreadMaxRaw=%lu,sampleUs=%lu/%lu,a8d=%ld",
        ads->batteryValid ? (long)ads->batteryMv : -1L,
        ads->batteryValid ? 1u : 0u,
        sensorarrayBatteryReasonName(ads->batteryInvalidReason),
        batteryState,
        (unsigned long)ads->batteryAgeMs,
        (unsigned long)ads->batteryPeriodMs,
        ads->batteryDue ? 1u : 0u,
        (unsigned long)ads->batteryRunCount,
        (unsigned long)ads->batteryValidRunCount,
        (unsigned long)ads->batteryInvalidRunCount,
        (unsigned long)ads->batterySkipCount,
        (unsigned long)ads->batteryDeferCount,
        (unsigned long)ads->batteryBoundaryCount,
        (unsigned long)ads->batteryRestoreFailureCount,
        (unsigned long)ads->batteryLastRetryCount,
        (unsigned long)ads->batteryRetryCount,
        (unsigned long)ads->batteryUnstableCount,
        (unsigned long)ads->batteryTimeoutCount,
        (unsigned long)ads->batterySpreadRaw,
        (unsigned long)ads->batterySpreadRawMaximum,
        (unsigned long)ads->batterySampleUsAverage,
        (unsigned long)ads->batterySampleUsMaximum,
        (long)ads->ain8DiffUv);
    if (ads->aincomGndValid && ads->ain8GndValid) {
        adsPosition = sensorarrayAsyncLogTextAppend(
            adsPacket->data,
            sizeof(adsPacket->data),
            adsPosition,
            ",ac=%ld,a8g=%ld",
            (long)ads->aincomGndUv,
            (long)ads->ain8GndUv);
    } else {
        adsPosition = sensorarrayAsyncLogTextAppend(
            adsPacket->data, sizeof(adsPacket->data), adsPosition,
            ",ac=na,a8g=na");
    }
    adsPosition = sensorarrayAsyncLogTextAppend(
        adsPacket->data, sizeof(adsPacket->data), adsPosition,
        ",rail=%ld,rv=%u,rs=%s,re=%ld,age=%lu,z=%ld/%lu,fresh=%u,status=0x%02X,dg=%lu,chip=%u,j=%lu/%lu",
        (long)ads->railUv,
        ads->railValid ? 1u : 0u,
        sensorarrayAdsRailStatusName(ads->railStatus),
        (long)ads->railErrorUv,
        (unsigned long)ads->railAgeFrames,
        (long)ads->zeroResidualUv,
        (unsigned long)zeroStdUv,
        ads->adcFresh ? 1u : 0u,
        (unsigned)ads->adcStatus,
        (unsigned long)ads->drdyGenerationDelta,
        (unsigned)ads->chip,
        (unsigned long)adsJobsRunDelta,
        (unsigned long)adsJobsSkipDelta);
    adsPosition = sensorarrayAsyncLogTextAppend(
        adsPacket->data,
        sizeof(adsPacket->data),
        adsPosition,
        ",ae=%lu/%lu/%lu/%lu/%lu\n",
        (unsigned long)ads->spiErrorCount,
        (unsigned long)ads->drdyTimeoutCount,
        (unsigned long)ads->adcStaleCount,
        (unsigned long)ads->adcStatusErrorCount,
        (unsigned long)adsJobsSkipDelta);
    position = sensorarrayAsyncLogTextAppend(
        packet->data,
        sizeof(packet->data),
        position,
        "OT50,st=%s,tx=%s,out=%llu/%llu,q=%lu/%lu/%lu,ofps=%llu.%01llu/%llu.%01llu/%llu.%01llu,"
        "drop=%llu/%llu/%llu,kb=%llu/%llu/%llu,blk=%llu/%llu/%llu,wdt=0\n",
        sensorarrayTransportStreamName(sensorarrayTransportGetStream()),
        sensorarrayTransportTxModeName(sensorarrayTransportGetTxMode()),
        (unsigned long long)outputAvgMs,
        (unsigned long long)outputMaxMs,
        (unsigned long)queueCurrent,
        (unsigned long)queueMax,
        (unsigned long)transportStats.queueDrop,
        (unsigned long long)(usbFpsX10 / 10u),
        (unsigned long long)(usbFpsX10 % 10u),
        (unsigned long long)(bleFpsX10 / 10u),
        (unsigned long long)(bleFpsX10 % 10u),
        (unsigned long long)(wifiFpsX10 / 10u),
        (unsigned long long)(wifiFpsX10 % 10u),
        (unsigned long long)usbDrop,
        (unsigned long long)bleDrop,
        (unsigned long long)wifiDrop,
        (unsigned long long)usbKb,
        (unsigned long long)bleKb,
        (unsigned long long)wifiKb,
        (unsigned long long)usbBlocked,
        (unsigned long long)bleBlocked,
        (unsigned long long)wifiBlocked);
    position = sensorarrayAsyncLogTextAppend(
        packet->data,
        sizeof(packet->data),
        position,
        "BL50,conn=%u,sub=%u%u%u,mtu=%u,phy=%u/%u,mode=%s,mq=%lu,ms=%lu,md=%lu,sentD=%lu,sentL=%lu,dropD=%lu,dropL=%lu,dropC=%lu,ctrlRetry=%lu,ctrlExhaust=%lu,fs=%lu,fe=%lu,cg=%lu,stale=%lu,conf=%lu,tiny=%lu\n",
        bleStats.connected ? 1u : 0u,
        sensorarrayBleIsSubscribed(SENSORARRAY_BLE_CH_CTRL) ? 1u : 0u,
        sensorarrayBleIsSubscribed(SENSORARRAY_BLE_CH_DATA) ? 1u : 0u,
        sensorarrayBleIsSubscribed(SENSORARRAY_BLE_CH_LOG) ? 1u : 0u,
        (unsigned)bleStats.mtu,
        (unsigned)bleStats.txPhy,
        (unsigned)bleStats.rxPhy,
        sensorarrayBleTxModeName(sensorarrayBleGetTxMode()),
        (unsigned long)bleStats.messageQueued,
        (unsigned long)bleStats.messageSent,
        (unsigned long)bleStats.messageDropped,
        (unsigned long)bleStats.sent[SENSORARRAY_BLE_CH_DATA],
        (unsigned long)bleStats.sent[SENSORARRAY_BLE_CH_LOG],
        (unsigned long)bleStats.dropped[SENSORARRAY_BLE_CH_DATA],
        (unsigned long)bleStats.dropped[SENSORARRAY_BLE_CH_LOG],
        (unsigned long)bleStats.dropped[SENSORARRAY_BLE_CH_CTRL],
        (unsigned long)bleStats.controlTxRetry,
        (unsigned long)bleStats.controlTxRetryExhausted,
        (unsigned long)bleStats.fragmentSent,
        (unsigned long)bleStats.fragmentError,
        (unsigned long)bleStats.congestedCount,
        (unsigned long)bleStats.txConnectionStaleDrop,
        (unsigned long)bleStats.staleConfirmation,
        (unsigned long)bleStats.tinyTailCount);
    position = sensorarrayAsyncLogTextAppend(
        packet->data,
        sizeof(packet->data),
        position,
        "I2C50,p=0/1,a=%02X/%02X,ok=%llu,nack=%lu,to=%lu,rec=%lu,freq=na,bus=OK,rv=%llu,ed=%llu\n",
        (unsigned)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
        (unsigned)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
        (unsigned long long)validRate,
        (unsigned long)telemetry->i2cNackCount,
        (unsigned long)telemetry->i2cTimeoutCount,
        (unsigned long)telemetry->i2cRecoveryCount,
        (unsigned long long)textBusDrop,
        (unsigned long long)eventRingDrop);

    uint32_t logMinimumRemainingBytes =
        (uint32_t)uxTaskGetStackHighWaterMark(NULL) *
        (uint32_t)sizeof(StackType_t);
    position = sensorarrayAsyncLogTextAppend(
        packet->data,
        sizeof(packet->data),
        position,
        "STK50,unit=bytes,seq=%lu,log=%u/%lu,transport=%lu/%lu,usb=%lu/%lu,bleTx=%lu/%lu,bleCtrl=%lu/%lu,serialCtrl=%lu/%lu,heap=%lu/%lu/%lu,tSlot=%lu/%lu,ta=%lu,ts=%lu,tr=%lu,tq=%lu/%lu,bSlot=%lu/%lu,ba=%lu,bs=%lu,br=%lu,bc=%lu,bf=%lu,bg=%lu,bconf=%lu,trunc=%lu\n",
        (unsigned long)summary->seqEnd,
        (unsigned)CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK,
        (unsigned long)logMinimumRemainingBytes,
        (unsigned long)transportTaskStacks.transportConfiguredBytes,
        (unsigned long)transportTaskStacks.transportMinimumRemainingBytes,
        (unsigned long)usbStats.taskConfiguredBytes,
        (unsigned long)usbStats.taskMinimumRemainingBytes,
        (unsigned long)bleStats.txTaskConfiguredBytes,
        (unsigned long)bleStats.txTaskMinimumRemainingBytes,
        (unsigned long)bleStats.ctrlTaskConfiguredBytes,
        (unsigned long)bleStats.ctrlTaskMinimumRemainingBytes,
        (unsigned long)transportTaskStacks.serialCtrlConfiguredBytes,
        (unsigned long)transportTaskStacks.serialCtrlMinimumRemainingBytes,
        (unsigned long)s_initialFreeHeap,
        (unsigned long)esp_get_free_heap_size(),
        (unsigned long)esp_get_minimum_free_heap_size(),
        (unsigned long)transportStats.transportSlotUsed,
        (unsigned long)transportStats.transportSlotHighWater,
        (unsigned long)transportStats.transportSlotAllocFail,
        (unsigned long)transportStats.transportStaleDescriptor,
        (unsigned long)transportStats.transportSlotReleaseMismatch,
        (unsigned long)transportStats.queueDropData,
        (unsigned long)transportStats.queueDropLog,
        (unsigned long)bleStats.txSlotUsed,
        (unsigned long)bleStats.txSlotHighWater,
        (unsigned long)bleStats.txSlotAllocFail,
        (unsigned long)bleStats.txSlotStaleGenerationDrop,
        (unsigned long)bleStats.txSlotReleaseMismatch,
        (unsigned long)bleStats.txCrcMismatch,
        (unsigned long)bleStats.fragmentError,
        (unsigned long)bleStats.txConnectionStaleDrop,
        (unsigned long)bleStats.staleConfirmation,
        (unsigned long)s_summaryTruncated);

    if (position < sizeof(packet->data) && position <= UINT16_MAX) {
        packet->length = (uint16_t)position;
        (void)sensorarrayUsbSinkPublish(packet);
        (void)sensorarrayNetLogPublish(packet);
    } else {
        s_summaryTruncated++;
        printf("LOGTRUNC,packet=summary,seq=%lu,max=%u,count=%lu\n",
               (unsigned long)summary->seqEnd,
               (unsigned)sizeof(packet->data),
               (unsigned long)s_summaryTruncated);
    }
    if (adsPosition != 0u && adsPosition < sizeof(adsPacket->data) &&
        adsPosition <= UINT16_MAX) {
        adsPacket->length = (uint16_t)adsPosition;
        (void)sensorarrayUsbSinkPublish(adsPacket);
        (void)sensorarrayNetLogPublish(adsPacket);
    } else if (adsPosition != 0u) {
        s_summaryTruncated++;
        printf("LOGTRUNC,packet=ads,seq=%lu,max=%u,count=%lu\n",
               (unsigned long)summary->seqEnd,
               (unsigned)sizeof(adsPacket->data),
               (unsigned long)s_summaryTruncated);
    }
    sensorarrayAsyncLogSummarySetBaseline(summary, &sharedStats);
}

static void sensorarrayAsyncLogMaybePrintSummary(sensorarrayAsyncLogSummary_t *summary)
{
    /* Compact summaries preserve the established low-volume wire contract.
     * ADSDBG deliberately reduces their aggregation window to one frame; the
     * validator enables that mode only long enough to prove it is active, then
     * disables it before collecting performance or full-frame evidence. */
    sensorarrayAsyncLogPrintCompactSummary(summary);
    return;
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

    printf("LOG20,s0=%lu,s1=%lu,n=%llu,measureFps=%llu.%02llu,outputFps=%llu.%02llu,frameAgeAvgUs=%llu,frameAgeMaxUs=%llu,outputQueueDepth=%lu,qDepthMax=%lu,droppedOutputFrames=%llu,droppedEventLogs=%llu,outUsAvg=%llu,outUsMax=%llu,measureFrameUsAvg=%llu,measureFrameUsMax=%llu,stackHighWaterBytes=%lu\n",
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
           (unsigned long)((uint64_t)uxTaskGetStackHighWaterMark(NULL) *
                           sizeof(StackType_t)));

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
    printf("ADS20,start=%d,drdy=%d,init=%u,type=%s,id=0x%02X,adc=%s,dma=%u,rate=%lu,ain9OffsetUv=%ld,ain9StdUv=%.1f,ain8RawUv=%ld,battMv=%ld,battValid=%u,drdyTimeout=%lu,spiErr=%lu,diagAgeFrames=%lu,gapJobsRun=%lu,gapJobsSkip=%lu,gapOverrun=%lu\n",
           CONFIG_SENSORARRAY_ADS_START_GPIO,
           CONFIG_SENSORARRAY_ADS_DRDY_GPIO,
           ads->initialized ? 1u : 0u,
           ads->chip == 1262u ? "ADS1262" :
               (ads->chip == 1263u ? "ADS1263" : "unknown"),
           ads->id,
           ads->chip ? "ADC1" : "unknown",
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

#if 0
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
#endif

    sensorarrayAsyncLogSummarySetBaseline(summary, &stats);
}

static void sensorarrayAsyncLogTask(void *arg)
{
    (void)arg;
    printf("TASKCORE,name=output_hub,core=%d,expected=%d\n",
           (int)xPortGetCoreID(),
           CONFIG_SENSORARRAY_LOG_TASK_CORE);
    sensorarrayAsyncLogWorkspace_t *workspace = &s_asyncLogWorkspace;
    sensorarrayFrame_t *frame = &workspace->frame;
    sensorarrayTextPacket_t *textPacket = &workspace->framePacket;
    uint32_t framesSinceIdleYield = 0u;
    int64_t lastOutputEmitUs = 0;

    while (true) {
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000u));
        sensorarrayAsyncLogDrainEvents();

        uint64_t measureFrameUs = 0u;
        int64_t publishedUs = 0;
        uint32_t queueDepth = 0u;
        while (sensorarrayAsyncLogPopFrame(frame,
                                           textPacket,
                                           &measureFrameUs,
                                           &publishedUs,
                                           &queueDepth)) {
            sensorarrayAsyncLogDrainEvents();

            int64_t outputStartUs = esp_timer_get_time();
            uint64_t frameAgeUs = (publishedUs > 0 && outputStartUs > publishedUs) ?
                (uint64_t)(outputStartUs - publishedUs) : 0u;
            bool emitted = frame->freshFrame || CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG;
            uint32_t outputFpsCap = sensorarrayCommandMailboxGetOutputFpsCap();
            if (emitted && outputFpsCap > 0u) {
                int64_t minIntervalUs = 1000000LL / (int64_t)outputFpsCap;
                if (lastOutputEmitUs > 0 &&
                    outputStartUs - lastOutputEmitUs < minIntervalUs) {
                    emitted = false;
                }
            }
            uint64_t outputUs = 0u;
            if (emitted) {
                frame->emitUs = (uint64_t)outputStartUs;
                /* Serial data is a throttled debug/fallback sink. The queue is
                 * non-blocking and network data remains the primary path. */
                (void)sensorarrayNetTextPublish(textPacket, true);
                if ((frame->sequence % 10u) == 0u) {
                    (void)sensorarrayUsbSinkPublish(textPacket);
                }
                if (sensorarrayCommandMailboxTraceEnabled()) {
                    printf("E,fr=%lu,type=trace,age=%llu,q=%lu\n",
                           (unsigned long)frame->sequence,
                           (unsigned long long)frameAgeUs,
                           (unsigned long)queueDepth);
                }
                outputUs = sensorarrayAsyncLogElapsedPositiveUs(outputStartUs);
                lastOutputEmitUs = outputStartUs;
            }

            sensorarrayAsyncLogUpdateSummary(&workspace->summary,
                                             frame,
                                             measureFrameUs,
                                             frameAgeUs,
                                             outputUs,
                                             queueDepth,
                                             emitted);
            sensorarrayAsyncLogMaybePrintSummary(&workspace->summary);
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
    s_asyncLogWorkspace.summary.active = false;
    s_asyncLogWorkspace.framePacket.length = 0u;
    s_asyncLogWorkspace.summaryPacket.length = 0u;
    s_asyncLogWorkspace.adsPacket.length = 0u;
    s_summaryTruncated = 0u;
    s_initialFreeHeap = esp_get_free_heap_size();

    esp_err_t usbErr = sensorarrayUsbSinkInit();
    if (usbErr != ESP_OK) {
        return usbErr;
    }

    s_eventQueue = xQueueCreateStatic(SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT,
                                      sizeof(sensorarrayAsyncLogEvent_t),
                                      s_eventQueueStorage,
                                      &s_eventQueueStruct);
    if (!s_eventQueue) {
        return ESP_ERR_NO_MEM;
    }

    int logTaskCore = CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE >= 0 ?
        CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE : CONFIG_SENSORARRAY_COMM_TASK_CORE;
    BaseType_t taskOk = xTaskCreatePinnedToCore(sensorarrayAsyncLogTask,
                                                "sensorarrayLogTask",
                                                CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK,
                                                NULL,
                                                CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY,
                                                &s_logTaskHandle,
                                                logTaskCore);
    if (taskOk != pdPASS || !s_logTaskHandle) {
        s_eventQueue = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_asyncLogStarted = true;
    sensorarrayTransportMemoryDiagnostics_t transportMemory = {0};
    sensorarrayTransportGetMemoryDiagnostics(&transportMemory);
    printf("APP_LOG_INIT,mode=async_domains,frameFormat=ascii_pf6,frameSlots=%u,eventRingLen=%u,summaryEvery=%u,taskPrio=%u,taskCore=%d,dropOldFrames=%u,stackBytes=%u,workspaceBytes=%u,summaryBytes=%u,frameBytes=%u,textPacketBytes=%u,transportLegacyItemBytes=%lu,transportSlotBytes=%lu,transportDescriptorBytes=%lu\n",
           (unsigned)SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT,
           (unsigned)SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT,
           (unsigned)CONFIG_SENSORARRAY_ASYNC_LOG_SUMMARY_EVERY_N_FRAMES,
           (unsigned)CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY,
           (int)CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE,
           CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES ? 1u : 0u,
           (unsigned)CONFIG_SENSORARRAY_ASYNC_LOG_TASK_STACK,
           (unsigned)sizeof(sensorarrayAsyncLogWorkspace_t),
           (unsigned)sizeof(sensorarrayAsyncLogSummary_t),
           (unsigned)sizeof(sensorarrayFrame_t),
           (unsigned)sizeof(sensorarrayTextPacket_t),
           (unsigned long)transportMemory.legacyItemBytes,
           (unsigned long)transportMemory.payloadSlotBytes,
           (unsigned long)transportMemory.descriptorBytes);
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
    uint8_t activeRows = frame->activeRows ? frame->activeRows : SENSORARRAY_MATRIX_ROWS;
    uint8_t expectedRowMask = (uint8_t)((1u << activeRows) - 1u);
    s_sharedStats.rowFreshMaskBad += frame->rowFreshMask != expectedRowMask ? 1u : 0u;
    s_sharedStats.primaryFreshMaskBad += frame->primaryFreshMask != expectedRowMask ? 1u : 0u;
    s_sharedStats.secondaryFreshMaskBad += frame->secondaryFreshMask != expectedRowMask ? 1u : 0u;
    s_sharedStats.freshCells += frame->freshCount;
    s_sharedStats.physicalSweepTotalUs += frame->physicalSweepUs;
    if (frame->physicalSweepUs > s_sharedStats.physicalSweepMaxUs) {
        s_sharedStats.physicalSweepMaxUs = frame->physicalSweepUs;
    }
    s_sharedStats.rowStepTotalUs += frame->rowStepUsAvg * activeRows;
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
    sensorarrayOutputCongestionDecision_t congestionDecision =
        sensorarrayOutputCongestionDecide(
            slot != UINT8_MAX,
            s_pendingCount != 0u,
            CONFIG_SENSORARRAY_ASYNC_LOG_DROP_OLD_FRAMES != 0);
    if (congestionDecision ==
        SENSORARRAY_OUTPUT_CONGESTION_RECLAIM_OLDEST) {
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
        s_frameSlots[slot].frame = *frame;
        s_frameSlots[slot].measureFrameUs = measureFrameUs;
        s_frameSlots[slot].publishedUs = publishedUs;
        esp_err_t textErr = sensorarrayTextProtocolBuildFrame(
            frame,
            &s_frameSlots[slot].textPacket);
        if (textErr != ESP_OK) {
            portENTER_CRITICAL(&s_asyncLogMux);
            s_frameSlotState[slot] = 0u;
            s_sharedStats.droppedOutputFrames++;
            portEXIT_CRITICAL(&s_asyncLogMux);
            return textErr;
        }
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
    bool adsFrame = frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ||
                    frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    sensorarrayAsyncLogEvent_t event = {
        .type = SENSORARRAY_ASYNC_LOG_EVENT_FRAME_ERROR,
        .sequence = frame->sequence,
        .data.frameError = {
            .readErr = readErr,
            .measurementMode = frame->measurement.mode,
            .measurementValidMask = frame->measurement.validMask,
            .measurementFreshMask = frame->measurement.freshMask,
            .capValidMask = frame->capValidMask,
            .errorMask = adsFrame ? frame->measurement.errorMask : frame->errorMask,
            .warnMask = frame->warnMask,
            .frameDurationUs = frame->measurement.frameDurationUs,
            .gainChangeCount = frame->measurement.gainChangeCount,
            .overrangeCount = frame->measurement.overrangeCount,
            .autorangeAttemptCount = frame->measurement.autorangeAttemptCount,
            .drdyTimeoutCount = frame->measurement.drdyTimeoutCount,
            .staleCount = frame->measurement.staleCount,
            .spiErrorCount = frame->measurement.spiErrorCount,
            .statusErrorCount = frame->measurement.statusErrorCount,
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
            .adsFrame = adsFrame,
        },
    };
    return sensorarrayAsyncLogPublishEvent(&event);
}

esp_err_t sensorarrayAsyncLogPublishCommandApplied(uint32_t sequence,
                                                   const sensorarrayCommand_t *command)
{
    if (!command) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayAsyncLogEvent_t event = {
        .type = SENSORARRAY_ASYNC_LOG_EVENT_COMMAND_APPLIED,
        .sequence = sequence,
        .data.command = {
            .type = command->type,
            .value = command->value,
        },
    };
    return sensorarrayAsyncLogPublishEvent(&event);
}

esp_err_t sensorarrayAsyncLogPublishTextEvent(const char *text, size_t length)
{
    if (!text || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayAsyncLogEvent_t event = {
        .type = SENSORARRAY_ASYNC_LOG_EVENT_TEXT,
    };
    if (length > sizeof(event.data.text.bytes)) {
        length = sizeof(event.data.text.bytes);
    }
    memcpy(event.data.text.bytes, text, length);
    event.data.text.length = (uint16_t)length;
    return sensorarrayAsyncLogPublishEvent(&event);
}
