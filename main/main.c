#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "boardSupport.h"
#include "tmuxSwitch.h"

#include "sensorarrayAdsMatrix.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayCommandMailbox.h"
#include "sensorarrayFdcMatrix.h"
#include "sensorarrayFdcRescue.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayFrame.h"
#include "sensorarrayAsyncLog.h"
#include "sensorarrayAcqEvent.h"
#include "sensorarrayFrameOutput.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayMixedRow.h"
#include "sensorarrayNetStatus.h"
#include "sensorarrayScanConfig.h"
#include "sensorarrayScanPlan.h"
#include "sensorarrayTransport.h"
#include "sensorarrayTypes.h"

#define printf sensorarrayAcqEventPrintf

#ifndef CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT
#define CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_FULL_SWEEP_REQUEST_COOLDOWN_MS
#define CONFIG_SENSORARRAY_FDC_FULL_SWEEP_REQUEST_COOLDOWN_MS 5000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_MAX_CONSECUTIVE_FULL_SWEEP_FAILS
#define CONFIG_SENSORARRAY_FDC_MAX_CONSECUTIVE_FULL_SWEEP_FAILS 3
#endif
#ifndef CONFIG_SENSORARRAY_FDC_DIAG_DUMP_REGS
#define CONFIG_SENSORARRAY_FDC_DIAG_DUMP_REGS 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_DIAG_DUMP_INTERVAL_MS
#define CONFIG_SENSORARRAY_FDC_DIAG_DUMP_INTERVAL_MS 5000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_DIAG_DUMP_SKIP_OFFLINE_BUS
#define CONFIG_SENSORARRAY_FDC_DIAG_DUMP_SKIP_OFFLINE_BUS 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED
#define CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_BOOT_ALLOW_DEGRADED
#define CONFIG_SENSORARRAY_FDC_BOOT_ALLOW_DEGRADED 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_BOOT_MIN_VALID_CELLS
#define CONFIG_SENSORARRAY_FDC_BOOT_MIN_VALID_CELLS 48
#endif
#ifndef CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE
#define CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE 1
#endif
#ifndef CONFIG_BOARD_I2C_RUNTIME_FALLBACK_ERROR_FRAMES
#define CONFIG_BOARD_I2C_RUNTIME_FALLBACK_ERROR_FRAMES 3
#endif
#ifndef CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG
#define CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG 0
#endif

typedef enum {
    SENSORARRAY_RUNTIME_MODE_FDC_MATRIX = 0,
    SENSORARRAY_RUNTIME_MODE_ADS_MATRIX,
    SENSORARRAY_RUNTIME_MODE_MIXED_ROW,
} sensorarrayRuntimeMode_t;

typedef struct {
    sensorarrayRuntimeMode_t runtimeMode;
    sensorarrayState_t state;
    sensorarrayScanPlan_t scanPlan;
    sensorarrayFrame_t frame;
    sensorarrayFdcMatrixEngine_t fdcEngine;
    sensorarrayAdsMatrixEngine_t adsEngine;
    sensorarrayFdcRescueContext_t fdcRescue;

    bool primaryAddrValid;
    bool secondaryAddrValid;
    uint8_t requestedFdcChannels;
    bool fdcBootSweepOk;
    bool fdcDiagnosticMode;
    bool fdcDegradedMode;
    sensorarrayFdcBootSummary_t fdcBootSummary;
    uint32_t fdcFrameCounter;
    uint32_t failedRescueCount;
    uint32_t rescueEpoch;
    int64_t lastFullRescueTimeUs;
    bool rescueRunning;
    bool asyncLogReady;
    bool legacySyncOutput;
    uint32_t runtimeI2cErrorStreak;
} sensorarrayAppContext_t;

static sensorarrayAppContext_t s_appContext;
static int64_t s_lastDiagnosticDumpUs __attribute__((unused));
static esp_err_t sensorarrayInitSystem(sensorarrayAppContext_t *ctx);

#define SENSORARRAY_BOOT_BREADCRUMB_MAGIC 0x53414252u
#define SENSORARRAY_BOOT_BREADCRUMB_VERSION 1u
#define SENSORARRAY_BOOT_BREADCRUMB_STAGE_MAX 32u

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t bootCount;
    uint32_t resetReason;
    char lastStage[SENSORARRAY_BOOT_BREADCRUMB_STAGE_MAX];
    int32_t lastErr;
    uint32_t lastFrameSeq;
    uint32_t minFreeHeap;
} sensorarrayBootBreadcrumb_t;

RTC_NOINIT_ATTR static sensorarrayBootBreadcrumb_t s_bootBreadcrumb;

static const char *sensorarrayResetReasonName(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:
        return "poweron";
    case ESP_RST_EXT:
        return "external";
    case ESP_RST_SW:
        return "software";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "int_wdt";
    case ESP_RST_TASK_WDT:
        return "task_wdt";
    case ESP_RST_WDT:
        return "wdt";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_SDIO:
        return "sdio";
    default:
        return "unknown";
    }
}

static void sensorarrayBootBreadcrumbSetStage(const char *stage,
                                              esp_err_t err,
                                              const sensorarrayAppContext_t *ctx)
{
    if (s_bootBreadcrumb.magic != SENSORARRAY_BOOT_BREADCRUMB_MAGIC ||
        s_bootBreadcrumb.version != SENSORARRAY_BOOT_BREADCRUMB_VERSION) {
        return;
    }

    snprintf(s_bootBreadcrumb.lastStage,
             sizeof(s_bootBreadcrumb.lastStage),
             "%s",
             stage ? stage : "unknown");
    s_bootBreadcrumb.lastErr = (int32_t)err;
    s_bootBreadcrumb.lastFrameSeq = ctx ? ctx->frame.sequence : 0u;
    uint32_t freeHeap = esp_get_free_heap_size();
    if (s_bootBreadcrumb.minFreeHeap == 0u || freeHeap < s_bootBreadcrumb.minFreeHeap) {
        s_bootBreadcrumb.minFreeHeap = freeHeap;
    }
}

static void sensorarrayBootBreadcrumbStart(void)
{
    sensorarrayBootBreadcrumb_t previous = s_bootBreadcrumb;
    bool previousValid =
        previous.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC &&
        previous.version == SENSORARRAY_BOOT_BREADCRUMB_VERSION;
    esp_reset_reason_t reason = esp_reset_reason();

    printf("RST,reason=%s,code=%ld,boot=%lu,prevValid=%u,prevStage=%s,prevErr=0x%lx,prevSeq=%lu,prevHeap=%lu\n",
           sensorarrayResetReasonName(reason),
           (long)reason,
           (unsigned long)(previousValid ? previous.bootCount + 1u : 1u),
           previousValid ? 1u : 0u,
           previousValid ? previous.lastStage : "none",
           previousValid ? (unsigned long)(uint32_t)previous.lastErr : 0u,
           previousValid ? (unsigned long)previous.lastFrameSeq : 0u,
           previousValid ? (unsigned long)previous.minFreeHeap : 0u);

    memset(&s_bootBreadcrumb, 0, sizeof(s_bootBreadcrumb));
    s_bootBreadcrumb.magic = SENSORARRAY_BOOT_BREADCRUMB_MAGIC;
    s_bootBreadcrumb.version = SENSORARRAY_BOOT_BREADCRUMB_VERSION;
    s_bootBreadcrumb.bootCount = previousValid ? previous.bootCount + 1u : 1u;
    s_bootBreadcrumb.resetReason = (uint32_t)reason;
    s_bootBreadcrumb.minFreeHeap = esp_get_free_heap_size();
    sensorarrayBootBreadcrumbSetStage("app_main", ESP_OK, NULL);
}

static const char *sensorarrayAppFdcBootQualityName(sensorarrayFdcBootQuality_t quality)
{
    switch (quality) {
    case SENSORARRAY_FDC_BOOT_QUALITY_OK:
        return "ok";
    case SENSORARRAY_FDC_BOOT_QUALITY_DEGRADED:
        return "degraded";
    case SENSORARRAY_FDC_BOOT_QUALITY_FAIL:
    default:
        return "fail";
    }
}

static void sensorarrayLogRuntimeMemoryDiag(const char *stage,
                                            const sensorarrayAppContext_t *ctx)
{
    printf("APP_MEM,stage=%s,ctx=%p,ctxSize=%u,stackHighWaterWords=%u,freeHeap=%u,minFreeHeap=%u\n",
           stage ? stage : SENSORARRAY_NA,
           (const void *)ctx,
           (unsigned)sizeof(sensorarrayAppContext_t),
           (unsigned)uxTaskGetStackHighWaterMark(NULL),
           (unsigned)esp_get_free_heap_size(),
           (unsigned)esp_get_minimum_free_heap_size());
}

static void sensorarrayLogStackHighWater(const char *stage)
{
    printf("APP_STACK,stage=%s,freeWords=%lu\n",
           stage ? stage : SENSORARRAY_NA,
           (unsigned long)uxTaskGetStackHighWaterMark(NULL));
}

static uint32_t sensorarrayFdcFramePeriodMs(void)
{
    uint32_t captureFpsCap = sensorarrayCommandMailboxGetCaptureFpsCap();
    if (captureFpsCap == 0u) {
        return 0u;
    }
    uint32_t periodMs = (1000u + captureFpsCap - 1u) / captureFpsCap;
    return periodMs == 0u ? 1u : periodMs;
}

static uint32_t sensorarrayFdcReferenceFramePeriodMs(void)
{
    uint32_t periodMs = (uint32_t)((SENSORARRAY_CFG_FRAME_PERIOD_US + 999u) / 1000u);
    return periodMs == 0u ? 1u : periodMs;
}

static void sensorarrayDelayFramePeriodSince(sensorarrayAppContext_t *ctx,
                                             int64_t frameStartUs,
                                             uint32_t sequence)
{
    uint32_t periodMs = sensorarrayFdcFramePeriodMs();
    if (periodMs == 0u) {
        int64_t referenceUs = (int64_t)sensorarrayFdcReferenceFramePeriodMs() * 1000LL;
        int64_t elapsedUs = esp_timer_get_time() - frameStartUs;
        if (elapsedUs > (int64_t)CONFIG_SENSORARRAY_FDC_OVERRUN_HARD_US &&
            ctx && ctx->asyncLogReady) {
            (void)sensorarrayAsyncLogPublishOverrun(sequence, elapsedUs, referenceUs);
        }
        return;
    }
    int64_t periodUs = (int64_t)periodMs * 1000LL;
    int64_t elapsedUs = esp_timer_get_time() - frameStartUs;
    int64_t remainingUs = periodUs - elapsedUs;
    if (remainingUs > 0) {
        uint32_t delayMs = (uint32_t)((remainingUs + 999LL) / 1000LL);
        vTaskDelay(pdMS_TO_TICKS(delayMs == 0u ? 1u : delayMs));
        return;
    }

    if (elapsedUs <= (int64_t)CONFIG_SENSORARRAY_FDC_OVERRUN_HARD_US) {
        return;
    }
    if (ctx && ctx->asyncLogReady) {
        (void)sensorarrayAsyncLogPublishOverrun(sequence, elapsedUs, periodUs);
    } else {
        printf("OV,s=%lu,fu=%lld,pu=%lld,o=1\n",
               (unsigned long)sequence,
               (long long)elapsedUs,
               (long long)periodUs);
    }
}

static bool sensorarrayFrameRawAllZero(const sensorarrayFrame_t *frame)
{
    return frame &&
           frame->freshCount == SENSORARRAY_MATRIX_CELL_COUNT &&
           frame->hardwareZeroRawCount == SENSORARRAY_MATRIX_CELL_COUNT &&
           frame->zeroBeforeReadyCount == 0u &&
           frame->notReadyCount == 0u;
}

static bool sensorarrayFdcWaveDebugEnabled(void)
{
    return CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROUTE_ONLY ||
           CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW_HOLD ||
           CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_PRIMARY_ONLY ||
           CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_SECONDARY_ONLY ||
           CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_SINGLE_CHANNEL;
}

static const char *sensorarrayFdcWaveDebugModeName(void)
{
    if (CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROUTE_ONLY) {
        return "route_only";
    }
    if (CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW_HOLD) {
        return "row_hold";
    }
    if (CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_PRIMARY_ONLY) {
        return "primary_only";
    }
    if (CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_SECONDARY_ONLY) {
        return "secondary_only";
    }
    if (CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_SINGLE_CHANNEL) {
        return "single_channel";
    }
    return "normal";
}

static esp_err_t sensorarrayFdcWaveDebugSetSleep(sensorarrayFdcDeviceState_t *fdcState,
                                                 bool sleepEnabled)
{
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_OK;
    }
    return sleepEnabled ?
        Fdc2214CapEnterSleepWriteOnly(fdcState->handle, fdcState->configReg) :
        Fdc2214CapExitSleepWriteOnly(fdcState->handle, fdcState->configReg);
}

static esp_err_t sensorarrayFdcWaveDebugReadDevice(sensorarrayFdcDeviceState_t *fdcState,
                                                   sensorarrayFdcDeviceId_t devId,
                                                   bool singleChannel)
{
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_OK;
    }

    sensorarrayMeasureDebugSetReadWindow(devId, true);
    esp_err_t err;
    if (singleChannel) {
        Fdc2214CapSample_t sample = {0};
        err = Fdc2214CapReadSampleRelaxed(
            fdcState->handle,
            (Fdc2214CapChannel_t)CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_CHANNEL,
            &sample);
    } else {
        Fdc2214CapFastChannelSample_t samples[4] = {0};
        err = Fdc2214CapReadDataBurst4(fdcState->handle, samples);
    }
    sensorarrayMeasureDebugSetReadWindow(devId, false);
    return err;
}

static void sensorarrayFdcMapVerifyDebug(void)
{
    if (!CONFIG_SENSORARRAY_FDC_MAP_VERIFY_DEBUG) {
        return;
    }

    uint32_t failureCount = 0u;
    for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
        const sensorarrayFdcDLineMap_t *map = sensorarrayBoardMapFindFdcByDLine(dLine);
        sensorarrayFdcDeviceId_t expectedDev = (dLine <= 4u) ?
            SENSORARRAY_FDC_DEV_PRIMARY : SENSORARRAY_FDC_DEV_SECONDARY;
        uint8_t expectedChannel = (uint8_t)((dLine - 1u) % 4u);
        bool ok = map && map->devId == expectedDev && (uint8_t)map->channel == expectedChannel;
        if (!ok) {
            failureCount++;
        }
        printf("MAP_VERIFY,d=%u,expectedDev=%s,expectedCh=%u,actualDev=%s,actualCh=%d,label=%s,ok=%u\n",
               (unsigned)dLine,
               expectedDev == SENSORARRAY_FDC_DEV_PRIMARY ? "primary" : "secondary",
               (unsigned)expectedChannel,
               map ? (map->devId == SENSORARRAY_FDC_DEV_PRIMARY ? "primary" : "secondary") : "missing",
               map ? (int)map->channel : -1,
               (map && map->mapLabel) ? map->mapLabel : SENSORARRAY_NA,
               ok ? 1u : 0u);
    }
    printf("MAP_VERIFY,summary=1,failures=%lu,result=%s,physicalSinglePointCheckStillRequired=1\n",
           (unsigned long)failureCount,
           failureCount == 0u ? "pass" : "fail");
}

static void sensorarrayRunFdcWaveDebugLoop(sensorarrayAppContext_t *ctx)
{
    const char *mode = sensorarrayFdcWaveDebugModeName();
    sensorarrayFdcDeviceState_t *primary = &ctx->state.fdcPrimary;
    sensorarrayFdcDeviceState_t *secondary = &ctx->state.fdcSecondary;
    bool readPrimary = CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW_HOLD ||
                       CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_PRIMARY_ONLY;
    bool readSecondary = CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW_HOLD ||
                         CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_SECONDARY_ONLY;
    bool singleChannel = CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_SINGLE_CHANNEL;
    bool singleSecondary = CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_SINGLE_CHANNEL_SECONDARY;
    if (singleChannel) {
        readPrimary = !singleSecondary;
        readSecondary = singleSecondary;
    }

    esp_err_t firstErr = sensorarrayMeasurePrepareFdcMatrixPath(&ctx->state, "wave_debug");
    if (CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROUTE_ONLY) {
        esp_err_t primaryErr = sensorarrayFdcWaveDebugSetSleep(primary, true);
        esp_err_t secondaryErr = sensorarrayFdcWaveDebugSetSleep(secondary, true);
        if (firstErr == ESP_OK && primaryErr != ESP_OK) {
            firstErr = primaryErr;
        }
        if (firstErr == ESP_OK && secondaryErr != ESP_OK) {
            firstErr = secondaryErr;
        }
    } else {
        esp_err_t primaryErr = sensorarrayFdcWaveDebugSetSleep(primary, !readPrimary);
        esp_err_t secondaryErr = sensorarrayFdcWaveDebugSetSleep(secondary, !readSecondary);
        if (firstErr == ESP_OK && primaryErr != ESP_OK) {
            firstErr = primaryErr;
        }
        if (firstErr == ESP_OK && secondaryErr != ESP_OK) {
            firstErr = secondaryErr;
        }
    }

    if (singleChannel) {
        sensorarrayFdcDeviceState_t *selected = singleSecondary ? secondary : primary;
        if (selected->ready && selected->handle) {
            esp_err_t singleErr = Fdc2214CapSetSingleChannelMode(
                selected->handle,
                (Fdc2214CapChannel_t)CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_CHANNEL);
            if (firstErr == ESP_OK && singleErr != ESP_OK) {
                firstErr = singleErr;
            }
        }
    }

    sensorarrayMeasureDebugTimingGpioPrepare();
    printf("WAVE_DEBUG,stage=start,mode=%s,row=%u,singleDevice=%s,singleChannel=%u,stepMs=%u,formalFrames=disabled,err=0x%lx\n",
           mode,
           (unsigned)CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW,
           singleSecondary ? "secondary" : "primary",
           (unsigned)CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_CHANNEL,
           (unsigned)CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_STEP_MS,
           (unsigned long)firstErr);

    uint32_t cycleCount = 0u;
    uint32_t readOkCount = 0u;
    uint32_t readErrorCount = 0u;
    esp_err_t lastErr = firstErr;
    while (true) {
        uint8_t firstRow = CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW_HOLD ?
            (uint8_t)CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW : 1u;
        uint8_t lastRow = CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW_HOLD ? firstRow : SENSORARRAY_MATRIX_ROWS;
        sensorarrayMeasureDebugPulseFrameStrobe();
        for (uint8_t row = firstRow; row <= lastRow; ++row) {
            esp_err_t rowErr = tmuxSwitchSelectRow((uint8_t)(row - 1u));
            sensorarrayMeasureDebugPulseRowStrobe();
            if (rowErr != ESP_OK) {
                lastErr = rowErr;
                readErrorCount++;
            }

            vTaskDelay(pdMS_TO_TICKS(CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_STEP_MS));
            if (readPrimary) {
                esp_err_t readErr = sensorarrayFdcWaveDebugReadDevice(primary,
                                                                      SENSORARRAY_FDC_DEV_PRIMARY,
                                                                      singleChannel);
                if (readErr == ESP_OK) {
                    readOkCount++;
                } else {
                    lastErr = readErr;
                    readErrorCount++;
                }
            }
            if (readSecondary) {
                esp_err_t readErr = sensorarrayFdcWaveDebugReadDevice(secondary,
                                                                      SENSORARRAY_FDC_DEV_SECONDARY,
                                                                      singleChannel);
                if (readErr == ESP_OK) {
                    readOkCount++;
                } else {
                    lastErr = readErr;
                    readErrorCount++;
                }
            }
        }

        cycleCount++;
        uint32_t logEvery = CONFIG_SENSORARRAY_FDC_WAVE_DEBUG_ROW_HOLD ? 100u : 20u;
        if ((cycleCount % logEvery) == 0u) {
            printf("WAVE_DEBUG,mode=%s,cycles=%lu,rowFirst=%u,rowLast=%u,readOk=%lu,readErr=%lu,lastErr=0x%lx\n",
                   mode,
                   (unsigned long)cycleCount,
                   (unsigned)firstRow,
                   (unsigned)lastRow,
                   (unsigned long)readOkCount,
                   (unsigned long)readErrorCount,
                   (unsigned long)lastErr);
        }
    }
}

static void sensorarrayApplyTmuxDefaults(sensorarrayState_t *state)
{
    if (!state || !state->tmuxReady) {
        return;
    }

    esp_err_t tmuxErr = tmuxSwitchSelectRow(0);
    if (tmuxErr == ESP_OK) {
        tmuxErr = sensorarrayMeasureSetSelaPath(state,
                                                SENSORARRAY_SELA_ROUTE_ADS1263,
                                                SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                "init_default",
                                                "tmux_defaults");
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmux1134SelectSelBLevel(false);
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = sensorarrayMeasureSetSwPhysicalLevel(state,
                                                       SENSORARRAY_SW_PHYSICAL_LOW,
                                                       "init_default");
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmux1134SetEnLogicalState(true);
    }

    sensorarrayLogStartup("tmux_defaults",
                          tmuxErr,
                          (tmuxErr == ESP_OK) ? "ok" : "set_failed",
                          (int32_t)(tmuxErr == ESP_OK));
}

static bool sensorarrayAppFdcIdMatches(uint16_t manufacturerId, uint16_t deviceId)
{
    return manufacturerId == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID &&
           deviceId == SENSORARRAY_FDC_EXPECTED_DEVICE_ID;
}

static esp_err_t sensorarrayProbeFdcIdsAtClock(sensorarrayFdcDeviceState_t *fdcState,
                                               uint32_t clockHz,
                                               uint32_t attempts,
                                               uint32_t *outGoodCount,
                                               uint32_t *outBadCount,
                                               uint16_t *outLastManufacturerId,
                                               uint16_t *outLastDeviceId,
                                               const char **outReason)
{
    if (outGoodCount) {
        *outGoodCount = 0u;
    }
    if (outBadCount) {
        *outBadCount = 0u;
    }
    if (outLastManufacturerId) {
        *outLastManufacturerId = 0u;
    }
    if (outLastDeviceId) {
        *outLastDeviceId = 0u;
    }
    if (outReason) {
        *outReason = "unknown";
    }
    if (!fdcState || !fdcState->i2cCtx || clockHz == 0u || attempts == 0u) {
        if (outReason) {
            *outReason = "invalid_arg";
        }
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = boardSupportSetI2cFrequency(fdcState->i2cCtx, clockHz);
    if (err != ESP_OK) {
        if (outReason) {
            *outReason = "bus_reconfigure_failed";
        }
        return err;
    }

    uint32_t goodCount = 0u;
    uint32_t badCount = 0u;
    esp_err_t firstErr = ESP_OK;
    uint16_t lastManufacturerId = 0u;
    uint16_t lastDeviceId = 0u;
    for (uint32_t i = 0u; i < attempts; ++i) {
        uint16_t manufacturerId = 0u;
        uint16_t deviceId = 0u;
        err = sensorarrayBringupReadFdcIdsRaw(fdcState->i2cCtx,
                                              fdcState->i2cAddr,
                                              &manufacturerId,
                                              &deviceId);
        lastManufacturerId = manufacturerId;
        lastDeviceId = deviceId;
        if (err == ESP_OK && sensorarrayAppFdcIdMatches(manufacturerId, deviceId)) {
            goodCount++;
        } else {
            badCount++;
            if (firstErr == ESP_OK) {
                firstErr = (err != ESP_OK) ? err : ESP_FAIL;
            }
        }
    }

    if (outGoodCount) {
        *outGoodCount = goodCount;
    }
    if (outBadCount) {
        *outBadCount = badCount;
    }
    if (outLastManufacturerId) {
        *outLastManufacturerId = lastManufacturerId;
    }
    if (outLastDeviceId) {
        *outLastDeviceId = lastDeviceId;
    }
    if (goodCount == attempts) {
        if (outReason) {
            *outReason = "ok";
        }
        return ESP_OK;
    }
    if (outReason) {
        *outReason = (firstErr == ESP_OK || firstErr == ESP_FAIL) ? "id_mismatch" :
            (firstErr == ESP_ERR_TIMEOUT) ? "timeout" : "id_read_failed";
    }
    return (firstErr != ESP_OK) ? firstErr : ESP_FAIL;
}

static void sensorarrayProbeAndLockFdcI2cClock(sensorarrayFdcDeviceState_t *fdcState,
                                               const char *mapLabel)
{
    if (!fdcState || !fdcState->i2cCtx) {
        return;
    }
    BoardSupportI2cBusInfo_t busInfo = {0};
    bool secondary = fdcState->i2cCtx == boardSupportGetI2c1Ctx();
    (void)boardSupportGetI2cBusInfo(secondary, &busInfo);
    int bus = (int)busInfo.Port;

    if (!CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE) {
        printf("ICLK,stage=disabled,bus=%d,addr=0x%02X,hz=%lu,map=%s\n",
               bus,
               fdcState->i2cAddr,
               (unsigned long)busInfo.FrequencyHz,
               mapLabel ? mapLabel : SENSORARRAY_NA);
        return;
    }

    static const uint32_t clockCandidates[] = {350000u, 337500u, 325000u, 300000u};
    const uint32_t attempts = 8u;
    esp_err_t lastErr = ESP_FAIL;
    const char *lastReason = "not_probed";
    for (size_t i = 0u; i < sizeof(clockCandidates) / sizeof(clockCandidates[0]); ++i) {
        uint32_t hz = clockCandidates[i];
        uint32_t goodCount = 0u;
        uint32_t badCount = 0u;
        uint16_t manufacturerId = 0u;
        uint16_t deviceId = 0u;
        const char *reason = "unknown";
        esp_err_t err = sensorarrayProbeFdcIdsAtClock(fdcState,
                                                      hz,
                                                      attempts,
                                                      &goodCount,
                                                      &badCount,
                                                      &manufacturerId,
                                                      &deviceId,
                                                      &reason);
        printf("ICLK,stage=probe,bus=%d,addr=0x%02X,hz=%lu,attempts=%lu,good=%lu,bad=%lu,err=0x%lx,idMfg=0x%04X,idDev=0x%04X,result=%s,map=%s\n",
               bus,
               fdcState->i2cAddr,
               (unsigned long)hz,
               (unsigned long)attempts,
               (unsigned long)goodCount,
               (unsigned long)badCount,
               (unsigned long)err,
               manufacturerId,
               deviceId,
               (err == ESP_OK) ? "ok" : reason,
               mapLabel ? mapLabel : SENSORARRAY_NA);
        if (err == ESP_OK) {
            printf("ICLK,stage=locked,bus=%d,addr=0x%02X,hz=%lu,attempts=%lu,map=%s\n",
                   bus,
                   fdcState->i2cAddr,
                   (unsigned long)hz,
                   (unsigned long)attempts,
                   mapLabel ? mapLabel : SENSORARRAY_NA);
            if (fdcState->handle) {
                Fdc2214CapCoreRegs_t regs = {0};
                esp_err_t verifyErr = Fdc2214CapReadCoreRegs(fdcState->handle, &regs);
                printf("ICLK,stage=verify,bus=%d,addr=0x%02X,hz=%lu,err=0x%lx,status=0x%04X,statusConfig=0x%04X,config=0x%04X,muxConfig=0x%04X,map=%s\n",
                       bus,
                       fdcState->i2cAddr,
                       (unsigned long)hz,
                       (unsigned long)verifyErr,
                       regs.Status,
                       regs.StatusConfig,
                       regs.Config,
                       regs.MuxConfig,
                       mapLabel ? mapLabel : SENSORARRAY_NA);
            }
            return;
        }
        lastErr = err;
        lastReason = reason;
        if ((i + 1u) < (sizeof(clockCandidates) / sizeof(clockCandidates[0]))) {
            printf("ICLK,stage=fallback,bus=%d,addr=0x%02X,from=%lu,to=%lu,reason=%s,map=%s\n",
                   bus,
                   fdcState->i2cAddr,
                   (unsigned long)hz,
                   (unsigned long)clockCandidates[i + 1u],
                   reason,
                   mapLabel ? mapLabel : SENSORARRAY_NA);
        }
    }

    printf("ICLK,stage=failed,bus=%d,addr=0x%02X,err=0x%lx,reason=%s,action=continue_init_at_last_frequency,map=%s\n",
           bus,
           fdcState->i2cAddr,
           (unsigned long)lastErr,
           lastReason,
           mapLabel ? mapLabel : SENSORARRAY_NA);
}

static void sensorarraySweepFdcI2cClockUnderLoad(sensorarrayFdcDeviceState_t *fdcState,
                                                 const char *mapLabel)
{
    if (!CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE ||
        !fdcState || !fdcState->i2cCtx || !fdcState->handle) {
        return;
    }

    const uint32_t startHz = 400000u;
    const uint32_t minHz = 300000u;
    const uint32_t stepHz = 5000u;
    const uint32_t marginHz = 10000u;
    const uint32_t attempts = 3u;
    uint32_t maxStableHz = 0u;
    BoardSupportI2cBusInfo_t busInfo = {0};
    bool secondary = fdcState->i2cCtx == boardSupportGetI2c1Ctx();
    (void)boardSupportGetI2cBusInfo(secondary, &busInfo);

    for (uint32_t testHz = startHz; testHz >= minHz; testHz -= stepHz) {
        esp_err_t setErr = boardSupportSetI2cFrequency(fdcState->i2cCtx, testHz);
        uint32_t idOk = 0u;
        uint32_t statusOk = 0u;
        uint32_t orderedOk = 0u;
        uint32_t burstOk = 0u;
        Fdc2214CapResetI2cStats(fdcState->handle);
        esp_err_t firstErr = setErr;
        for (uint32_t attempt = 0u; attempt < attempts && setErr == ESP_OK; ++attempt) {
            uint16_t manufacturerId = 0u;
            uint16_t deviceId = 0u;
            esp_err_t err = Fdc2214CapReadId(fdcState->handle,
                                             &manufacturerId,
                                             &deviceId);
            if (err == ESP_OK && sensorarrayAppFdcIdMatches(manufacturerId, deviceId)) {
                idOk++;
            } else if (firstErr == ESP_OK) {
                firstErr = err != ESP_OK ? err : ESP_ERR_INVALID_RESPONSE;
            }

            Fdc2214CapStatus_t status = {0};
            err = Fdc2214CapReadStatus(fdcState->handle, &status);
            if (err == ESP_OK) {
                statusOk++;
            } else if (firstErr == ESP_OK) {
                firstErr = err;
            }

            Fdc2214CapFastChannelSample_t ordered[4] = {0};
            err = Fdc2214CapReadDataOrdered4(fdcState->handle, ordered);
            if (err == ESP_OK) {
                orderedOk++;
            } else if (firstErr == ESP_OK) {
                firstErr = err;
            }

            if (Fdc2214CapDataBurstSupported(fdcState->handle)) {
                Fdc2214CapFastChannelSample_t burst[4] = {0};
                err = Fdc2214CapReadDataBurst4(fdcState->handle, burst);
                if (err == ESP_OK) {
                    burstOk++;
                } else if (firstErr == ESP_OK) {
                    firstErr = err;
                }
            }
        }
        Fdc2214CapI2cStats_t stats = {0};
        Fdc2214CapGetI2cStats(fdcState->handle, &stats);
        bool burstRequired = Fdc2214CapDataBurstSupported(fdcState->handle);
        bool ok = setErr == ESP_OK && firstErr == ESP_OK &&
                  idOk == attempts && statusOk == attempts && orderedOk == attempts &&
                  (!burstRequired || burstOk == attempts) &&
                  stats.nackCount == 0u && stats.timeoutCount == 0u;
        const char *reason = setErr != ESP_OK ? "bus_reconfigure_failed" :
            (idOk != attempts ? "id_failed" :
             statusOk != attempts ? "status_failed" :
             orderedOk != attempts ? "ordered_failed" :
             (burstRequired && burstOk != attempts) ? "burst_failed" :
             stats.nackCount != 0u ? "nack" :
             stats.timeoutCount != 0u ? "timeout" : "ok");
        printf("I2C_SWEEP,bus=%d,testHz=%lu,ok=%u,idOk=%lu,statusOk=%lu,orderedOk=%lu,burstOk=%lu,nack=%lu,timeout=%lu,recover=%lu,reason=%s,map=%s\n",
               (int)busInfo.Port,
               (unsigned long)testHz,
               ok ? 1u : 0u,
               (unsigned long)idOk,
               (unsigned long)statusOk,
               (unsigned long)orderedOk,
               (unsigned long)burstOk,
               (unsigned long)stats.nackCount,
               (unsigned long)stats.timeoutCount,
               (unsigned long)stats.recoveryCount,
               reason,
               mapLabel ? mapLabel : SENSORARRAY_NA);
        if (ok) {
            maxStableHz = testHz;
            break;
        }
        if (testHz == minHz) {
            break;
        }
    }

    uint32_t selectedHz = maxStableHz > (minHz + marginHz) ?
        (maxStableHz - marginHz) : minHz;
    const char *selectedReason = maxStableHz == 0u ?
        "no_stable_workload_use_conservative" :
        (maxStableHz <= minHz ? "stable_only_at_safe_floor" : "max_stable_minus_margin");
    esp_err_t selectedErr = boardSupportSetI2cFrequency(fdcState->i2cCtx, selectedHz);
    printf("I2C_SELECTED,bus=%d,hz=%lu,maxStableHz=%lu,marginKHz=10,err=0x%lx,reason=%s,map=%s\n",
           (int)busInfo.Port,
           (unsigned long)selectedHz,
           (unsigned long)maxStableHz,
           (unsigned long)selectedErr,
           selectedReason,
           mapLabel ? mapLabel : SENSORARRAY_NA);
}

static void sensorarrayInitFdcDevice(sensorarrayFdcDeviceState_t *fdcState,
                                     bool addressValid,
                                     uint8_t requestedChannels,
                                     const char *mapLabel)
{
    if (!fdcState) {
        return;
    }
    if (!addressValid) {
        sensorarrayLogStartupFdc("fdc_init",
                                 fdcState,
                                 ESP_ERR_INVALID_ARG,
                                 "skip_invalid_addr_config",
                                 (int32_t)fdcState->i2cAddr,
                                 false,
                                 0,
                                 0,
                                 mapLabel);
        return;
    }
    if (!fdcState->i2cCtx) {
        sensorarrayLogStartupFdc("fdc_init",
                                 fdcState,
                                 ESP_ERR_NOT_SUPPORTED,
                                 "skip_i2c_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 mapLabel);
        return;
    }

    sensorarrayProbeAndLockFdcI2cClock(fdcState, mapLabel);
    sensorarrayBringupProbeFdcBus(fdcState);

    sensorarrayFdcInitDiag_t diag = {0};
    esp_err_t err = sensorarrayBringupInitFdcDevice(fdcState->i2cCtx,
                                                    fdcState->i2cAddr,
                                                    requestedChannels,
                                                    &fdcState->handle,
                                                    &diag);
    sensorarrayBringupApplyFdcInitResult(fdcState, fdcState->handle, err, &diag);
    sensorarrayLogStartupFdc("fdc_init",
                             fdcState,
                             err,
                             diag.status,
                             (err == ESP_OK) ? (int32_t)requestedChannels : diag.detail,
                             diag.haveIds,
                             diag.manufacturerId,
                             diag.deviceId,
                             mapLabel);
    if (err == ESP_OK && fdcState->handle) {
        Fdc2214CapBurstProbeResult_t burstProbe = {0};
        esp_err_t burstErr = Fdc2214CapProbeDataBurst4(fdcState->handle,
                                                       5u,
                                                       &burstProbe);
        printf("BURST_PROBE,device=%s,supported=%u,trials=%lu,mismatch=%lu,err=0x%lx,reason=%s,map=%s\n",
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               burstProbe.supported ? 1u : 0u,
               (unsigned long)burstProbe.trials,
               (unsigned long)burstProbe.mismatchCount,
               (unsigned long)burstErr,
               burstProbe.reason ? burstProbe.reason : "unknown",
               mapLabel ? mapLabel : SENSORARRAY_NA);
        sensorarraySweepFdcI2cClockUnderLoad(fdcState, mapLabel);
        Fdc2214CapSequenceProbeResult_t sequenceProbe = {0};
        esp_err_t sequenceErr = ESP_ERR_NOT_SUPPORTED;
        if (CONFIG_SENSORARRAY_FDC_PRECISION_SAFE_SEQUENCE_ENABLE) {
            sequenceErr = Fdc2214CapProbeDataSequence4(fdcState->handle,
                                                       5u,
                                                       &sequenceProbe);
        } else {
            sequenceProbe.selectedMode = FDC_DATA_READ_MODE_ORDERED8;
            sequenceProbe.reason = "config_disabled";
        }
        printf("SEQ_PROBE,device=%s,seqProbeOk=%u,dataReadMode=%s,trials=%lu,fixedMismatch=%lu,dataMismatch=%lu,testedMask=0x%lX,okMask=0x%lX,seqRegsPerTxn=%u,seqTxnPerRow=%u,selectedUs=%lu,err=0x%lx,reason=%s,map=%s\n",
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               sequenceProbe.supported ? 1u : 0u,
               Fdc2214CapDataReadModeName(Fdc2214CapDataReadMode(fdcState->handle)),
               (unsigned long)sequenceProbe.trials,
               (unsigned long)sequenceProbe.fixedRegMismatchCount,
               (unsigned long)sequenceProbe.dataMismatchCount,
               (unsigned long)sequenceProbe.testedModeMask,
               (unsigned long)sequenceProbe.modeOkMask,
               (unsigned)Fdc2214CapDataReadModeRegsPerTransaction(
                   Fdc2214CapDataReadMode(fdcState->handle)),
               (unsigned)Fdc2214CapDataReadModeTransactionsPerRow(
                   Fdc2214CapDataReadMode(fdcState->handle)),
               (unsigned long)sequenceProbe.selectedElapsedUs,
               (unsigned long)sequenceErr,
               sequenceProbe.reason ? sequenceProbe.reason : "unknown",
               mapLabel ? mapLabel : SENSORARRAY_NA);
        uint16_t rcount[4] = {0};
        uint16_t settle[4] = {0};
        uint16_t clockDiv[4] = {0};
        uint16_t drive[4] = {0};
        uint16_t statusConfig = 0u;
        uint16_t config = 0u;
        uint16_t muxConfig = 0u;
        esp_err_t profileErr = ESP_OK;
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            esp_err_t readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                           (uint8_t)(0x08u + ch),
                                                           &rcount[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
            readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 (uint8_t)(0x10u + ch),
                                                 &settle[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
            readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 (uint8_t)(0x14u + ch),
                                                 &clockDiv[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
            readErr = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 (uint8_t)(0x1Eu + ch),
                                                 &drive[ch]);
            if (profileErr == ESP_OK && readErr != ESP_OK) {
                profileErr = readErr;
            }
        }
        (void)Fdc2214CapReadRawRegisters(fdcState->handle, 0x19u, &statusConfig);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle, 0x1Au, &config);
        (void)Fdc2214CapReadRawRegisters(fdcState->handle, 0x1Bu, &muxConfig);
        printf("FDC_PROFILE,device=%s,name=precision_safe_fast,conversionProfile=%s,runtimeRcount=0x%04X,startupRcount=[%04X,%04X,%04X,%04X],settle=[%04X,%04X,%04X,%04X],clockDiv=[%04X,%04X,%04X,%04X],drive=[%04X,%04X,%04X,%04X],config=0x%04X,mux=0x%04X,statusConfig=0x%04X,precisionAffecting=0,err=0x%lx\n",
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               CONFIG_SENSORARRAY_FDC_RUNTIME_PROFILE_NAME,
               (unsigned)CONFIG_SENSORARRAY_FDC_RUNTIME_RCOUNT,
               rcount[0], rcount[1], rcount[2], rcount[3],
               settle[0], settle[1], settle[2], settle[3],
               clockDiv[0], clockDiv[1], clockDiv[2], clockDiv[3],
               drive[0], drive[1], drive[2], drive[3],
               config,
               muxConfig,
               statusConfig,
               (unsigned long)profileErr);
        static bool experimentalProfileLogged = false;
        if (!experimentalProfileLogged) {
            experimentalProfileLogged = true;
            printf("FDC_PROFILE,name=experimental_precision_affecting_profile,precisionAffecting=1,enabled=%u\n",
                   CONFIG_SENSORARRAY_FDC_FORMAL_FAST_PROFILE_ENABLE ? 1u : 0u);
        }
    }
}

static void sensorarrayLogFdcParallelCfg(void)
{
    BoardSupportI2cBusInfo_t primaryBus = {0};
    BoardSupportI2cBusInfo_t secondaryBus = {0};
    (void)boardSupportGetI2cBusInfo(false, &primaryBus);
    (void)boardSupportGetI2cBusInfo(true, &secondaryBus);
    bool primaryAvailable = primaryBus.Enabled && boardSupportGetI2cCtx();
    bool secondaryAvailable = secondaryBus.Enabled && boardSupportGetI2c1Ctx();
    bool sameBus = primaryAvailable &&
                   secondaryAvailable &&
                   primaryBus.Port == secondaryBus.Port;
    bool configEnabled = CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ != 0;
    bool enabled = configEnabled &&
                   primaryAvailable &&
                   secondaryAvailable &&
                   !sameBus;
    const char *reason = !configEnabled ? "config_disabled" :
        (!primaryAvailable || !secondaryAvailable) ? "bus_unavailable" :
        sameBus ? "same_bus" :
        "dual_bus_enabled";
    if (configEnabled && !enabled) {
        printf("FDC_PARALLEL_WARN,reason=config_enabled_but_worker_not_available,detail=%s\n",
               reason);
    }
    printf("FDC_PARALLEL_CFG,enabled=%u,primaryBus=%d,secondaryBus=%d,sameBus=%u,primaryCore=%d,secondaryCore=%d,workerMode=%s,reason=%s,scanCore=%d,logCore=%d,workerPrio=%d,logPrio=%d\n",
           enabled ? 1u : 0u,
           primaryAvailable ? (int)primaryBus.Port : -1,
           secondaryAvailable ? (int)secondaryBus.Port : -1,
           sameBus ? 1u : 0u,
           CONFIG_SENSORARRAY_FDC_PRIMARY_WORKER_TASK_CORE,
           CONFIG_SENSORARRAY_FDC_SECONDARY_WORKER_TASK_CORE,
           enabled ? "parallel" : "serial",
           reason,
           CONFIG_SENSORARRAY_SCAN_TASK_CORE,
           CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE,
           CONFIG_SENSORARRAY_FDC_WORKER_TASK_PRIO,
           CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY);
}

static esp_err_t sensorarrayInitRuntime(sensorarrayAppContext_t *ctx)
{
    sensorarrayLogRuntimeMemoryDiag("runtime_entry", ctx);
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayLogDbgExtraReset();
    sensorarrayLogRuntimeMemoryDiag("runtime_before_clear", ctx);
    *ctx = (sensorarrayAppContext_t){0};
    sensorarrayLogRuntimeMemoryDiag("runtime_after_clear", ctx);
    sensorarrayLogSetAdsState(false, false);
    sensorarrayFastSpeedSetEnabled(false);
    ctx->runtimeMode = SENSORARRAY_RUNTIME_MODE_FDC_MATRIX;

    esp_err_t mailboxErr = sensorarrayCommandMailboxInit();
    if (mailboxErr != ESP_OK) {
        return mailboxErr;
    }

    ctx->requestedFdcChannels =
        sensorarrayBringupNormalizeFdcChannels((uint8_t)CONFIG_FDC2214CAP_CHANNELS);
    if (ctx->requestedFdcChannels < SENSORARRAY_FDC_REQUIRED_CHANNELS) {
        ctx->requestedFdcChannels = SENSORARRAY_FDC_REQUIRED_CHANNELS;
    }
    ctx->state.fdcConfiguredChannels = ctx->requestedFdcChannels;

    sensorarrayBringupResetFdcState(&ctx->state.fdcPrimary,
                                    "primary_fdc2214",
                                    (uint8_t)(CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR & 0xFFu));
    sensorarrayBringupResetFdcState(&ctx->state.fdcSecondary,
                                    "secondary_fdc2214",
                                    (uint8_t)(CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR & 0xFFu));

    ctx->primaryAddrValid =
        sensorarrayBringupParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                                          &ctx->state.fdcPrimary.i2cAddr);
    ctx->secondaryAddrValid =
        sensorarrayBringupParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
                                          &ctx->state.fdcSecondary.i2cAddr);

    sensorarrayLogStartup("app", ESP_OK, "fdc_matrix_start", 0);
    sensorarrayLogStartup("fdc_channels", ESP_OK, "autoscan_ch0_ch3",
                          (int32_t)ctx->requestedFdcChannels);
    sensorarrayLogStartupFdc("fdc_cfg",
                             &ctx->state.fdcPrimary,
                             ctx->primaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             ctx->primaryAddrValid ? "configured" : "invalid_addr_config",
                             (int32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                             false,
                             0,
                             0,
                             "D1..D4_primary_ch0..ch3");
    sensorarrayLogStartupFdc("fdc_cfg",
                             &ctx->state.fdcSecondary,
                             ctx->secondaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             ctx->secondaryAddrValid ? "configured" : "invalid_addr_config",
                             (int32_t)ctx->state.fdcSecondary.i2cAddr,
                             false,
                             0,
                             0,
                             "D5..D8_secondary_ch0..ch3");
    return ESP_OK;
}

static void sensorarrayApplyPendingCommands(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }

    sensorarrayCommand_t command;
    while (sensorarrayCommandMailboxTryReceive(&command)) {
        switch (command.type) {
        case SENSORARRAY_COMMAND_CALIBRATE_ZERO:
            sensorarrayAdsGapRequestCalibration(true, false);
            break;
        case SENSORARRAY_COMMAND_CALIBRATE_RAIL:
            sensorarrayAdsGapRequestCalibration(false, true);
            break;
        case SENSORARRAY_COMMAND_CALIBRATE_ALL:
            sensorarrayAdsGapRequestCalibration(true, true);
            break;
        case SENSORARRAY_COMMAND_ADS_GAP_MODE:
            sensorarrayAdsGapSetMode((sensorarrayAdsGapMode_t)command.value);
            break;
        case SENSORARRAY_COMMAND_BLE_CAP_PERIOD:
        case SENSORARRAY_COMMAND_TRACE_ENABLE:
        case SENSORARRAY_COMMAND_CAPTURE_FPS_CAP:
        case SENSORARRAY_COMMAND_OUTPUT_FPS_CAP:
            break;
        default:
            continue;
        }

        sensorarrayCommandMailboxCommit(&command);
        if (ctx->asyncLogReady) {
            (void)sensorarrayAsyncLogPublishCommandApplied(ctx->frame.sequence, &command);
        }
    }
}

static esp_err_t sensorarrayRuntimeQueryCommand(const char *command,
                                                char *response,
                                                size_t responseSize,
                                                void *context)
{
    sensorarrayAppContext_t *ctx = (sensorarrayAppContext_t *)context;
    uint32_t frameSequence = ctx ? ctx->frame.sequence : 0u;
    if (!command || !response || responseSize == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (strcmp(command, "BAT?") == 0) {
        return sensorarrayAdsGapFormatBattery(response, responseSize, frameSequence) > 0u ?
            ESP_OK : ESP_FAIL;
    }
    if (strcmp(command, "BATD") == 0 ||
        strcmp(command, "BATD=VBIAS_ON") == 0 ||
        strcmp(command, "BATD,MODE=VBIAS_ON") == 0) {
        sensorarrayAdsGapRequestBatteryDiagnostic();
        snprintf(response, responseSize, "ACK,cmd=BATD,mode=vbias_on,status=queued\n");
        return ESP_OK;
    }
    if (strcmp(command, "RAIL?") == 0) {
        return sensorarrayAdsGapFormatRail(response, responseSize, frameSequence) > 0u ?
            ESP_OK : ESP_FAIL;
    }
    if (strcmp(command, "ADS?") == 0) {
        return sensorarrayAdsGapFormatAds(response, responseSize) > 0u ?
            ESP_OK : ESP_FAIL;
    }
    if (strcmp(command, "FPS?") == 0) {
        uint32_t captureCap = sensorarrayCommandMailboxGetCaptureFpsCap();
        uint32_t outputCap = sensorarrayCommandMailboxGetOutputFpsCap();
        snprintf(response,
                 responseSize,
                 "FPS,cfcap=%lu,ofcap=%lu,adsgap=%s\n",
                 (unsigned long)captureCap,
                 (unsigned long)outputCap,
                 sensorarrayAdsGapModeName(sensorarrayAdsGapGetMode()));
        return ESP_OK;
    }
    if (strcmp(command, "ADSDBG=1") == 0 || strcmp(command, "ADSDBG=0") == 0) {
        snprintf(response, responseSize, "ACK,cmd=ADSDBG,v=%c\n",
                 command[7]);
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t sensorarrayInitBoardAndRouting(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = boardSupportInit();
    ctx->state.boardReady = (err == ESP_OK);
    sensorarrayLogStartup("board", err,
                          ctx->state.boardReady ? "ok" : "init_failed",
                          (int32_t)ctx->state.boardReady);
    if (err != ESP_OK) {
        printf("APP_INIT_FATAL,stage=board_support,err=%ld,action=stop_init\n",
               (long)err);
        return err;
    }

    err = tmuxSwitchInit();
    ctx->state.tmuxReady = (err == ESP_OK);
    sensorarrayLogStartup("tmux", err,
                          ctx->state.tmuxReady ? "ok" : "init_failed",
                          (int32_t)ctx->state.tmuxReady);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayBoardMapAudit();
    sensorarrayApplyTmuxDefaults(&ctx->state);
    return ESP_OK;
}

static esp_err_t sensorarrayInitFrontends(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayBringupInitAds(&ctx->state);
    ctx->state.adsReady = (err == ESP_OK);
    ctx->state.adsRefReady = false;
    sensorarrayLogStartup("ads", err,
                          ctx->state.adsReady ? "ok_ref_off" : "init_failed",
                          (int32_t)ctx->state.adsReady);
    sensorarrayLogSetAdsState(ctx->state.adsReady, ctx->state.adsRefReady);

    if (ctx->state.boardReady) {
        ctx->state.fdcPrimary.i2cCtx = boardSupportGetI2cCtx();
        ctx->state.fdcSecondary.i2cCtx = boardSupportGetI2c1Ctx();
        sensorarrayInitFdcDevice(&ctx->state.fdcPrimary,
                                 ctx->primaryAddrValid,
                                 ctx->requestedFdcChannels,
                                 "D1..D4_primary_ch0..ch3");
        sensorarrayInitFdcDevice(&ctx->state.fdcSecondary,
                                 ctx->secondaryAddrValid,
                                 ctx->requestedFdcChannels,
                                 "D5..D8_secondary_ch0..ch3");
        BoardSupportI2cBusInfo_t bus0Info = {0};
        BoardSupportI2cBusInfo_t bus1Info = {0};
        (void)boardSupportGetI2cBusInfo(false, &bus0Info);
        (void)boardSupportGetI2cBusInfo(true, &bus1Info);
        printf("I2C_SELECTED,bus0Hz=%lu,bus1Hz=%lu,marginKHz=10,reason=post_handle_real_load_sweep\n",
               (unsigned long)bus0Info.FrequencyHz,
               (unsigned long)bus1Info.FrequencyHz);
        sensorarrayLogFdcParallelCfg();
    } else {
        sensorarrayLogStartupFdc("fdc_init",
                                 &ctx->state.fdcPrimary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D1..D4_primary_ch0..ch3");
        sensorarrayLogStartupFdc("fdc_init",
                                 &ctx->state.fdcSecondary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D5..D8_secondary_ch0..ch3");
    }

    err = sensorarrayFdcMatrixEngineInit(&ctx->fdcEngine, &ctx->state);
    if (err != ESP_OK) {
        sensorarrayLogStartup("fdc_matrix_engine", err, "init_failed", (int32_t)err);
        return err;
    }
    err = sensorarrayAdsMatrixEngineInit(&ctx->adsEngine, &ctx->state);
    if (err != ESP_OK) {
        sensorarrayLogStartup("ads_matrix_engine", err, "init_failed", (int32_t)err);
        return err;
    }
    esp_err_t adsGapErr = sensorarrayAdsGapInit(&ctx->state);
    if (adsGapErr == ESP_OK) {
        sensorarrayTransportSetRuntimeQueryCallback(sensorarrayRuntimeQueryCommand, ctx);
    }
    printf("ADS_PINMAP,start=%d,drdy=%d,cs=%d,sclk=%d,din=%d,dout=%d,core=%d,gapInit=0x%lx,dma=%u\n",
           CONFIG_SENSORARRAY_ADS_START_GPIO,
           CONFIG_SENSORARRAY_ADS_DRDY_GPIO,
           CONFIG_BOARD_ADS126X_CS_GPIO,
           CONFIG_BOARD_SPI_SCLK_GPIO,
           CONFIG_BOARD_SPI_MOSI_GPIO,
           CONFIG_BOARD_SPI_MISO_GPIO,
           (int)xPortGetCoreID(),
           (unsigned long)adsGapErr,
           ctx->state.ads.spiDmaCapable ? 1u : 0u);
    sensorarrayFdcRescueReset(&ctx->fdcRescue);
    return ESP_OK;
}

static void sensorarrayInitAsyncLogging(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->asyncLogReady = false;
    ctx->legacySyncOutput = CONFIG_SENSORARRAY_ASYNC_LOG_LEGACY_SYNC_OUTPUT != 0;
    if (!CONFIG_SENSORARRAY_ASYNC_LOG_ENABLE) {
        printf("APP_LOG_INIT,mode=async_disabled,legacySync=%u\n",
               ctx->legacySyncOutput ? 1u : 0u);
        return;
    }

    esp_err_t err = sensorarrayAsyncLogInit();
    if (err == ESP_OK) {
        ctx->asyncLogReady = true;
        ctx->legacySyncOutput = false;
        return;
    }

    ctx->legacySyncOutput = true;
    printf("APP_LOG_INIT_FAIL,err=0x%lx,action=legacy_sync_output,warning=printf_blocks_measurement\n",
           (unsigned long)err);
}

static void sensorarrayBuildDefaultScanPlan(sensorarrayAppContext_t *ctx)
{
    if (ctx) {
        sensorarrayScanPlanBuildDefaultFdcMatrix(&ctx->scanPlan);
    }
}

static esp_err_t sensorarrayRunBootCalibration(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    bool requireDual = CONFIG_SENSORARRAY_REQUIRE_DUAL_FDC_FOR_BOOT != 0;
    bool fallbackAllowed = !requireDual;
    printf("APP_FDC_BOOT,stage=begin,primaryReady=%d,secondaryReady=%d,primaryAddr=0x%02X,secondaryAddr=0x%02X,requireDual=%u,bootSweepRequired=%u,fallbackAllowed=%u\n",
           ctx->state.fdcPrimary.ready ? 1 : 0,
           ctx->state.fdcSecondary.ready ? 1 : 0,
           ctx->state.fdcPrimary.i2cAddr,
           ctx->state.fdcSecondary.i2cAddr,
           requireDual ? 1u : 0u,
           CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED ? 1u : 0u,
           fallbackAllowed ? 1u : 0u);
    sensorarrayLogStackHighWater("boot_sweep_before");
    if (!ctx->state.fdcPrimary.ready) {
        ctx->fdcBootSummary = (sensorarrayFdcBootSummary_t){
            .transportErr = ESP_ERR_INVALID_STATE,
            .quality = SENSORARRAY_FDC_BOOT_QUALITY_FAIL,
            .failedCellCount = SENSORARRAY_MATRIX_CELL_COUNT,
            .failedRowMask = 0xFFu,
            .reason = "primary_not_ready",
        };
        ctx->fdcBootSweepOk = false;
        ctx->fdcDegradedMode = false;
        ctx->fdcDiagnosticMode = true;
        sensorarrayFdcMatrixEngineSetDiagnosticMode(&ctx->fdcEngine, true);
        printf("APP_FDC,stage=boot_sweep_return,err=0x%lx,ok=0\n",
               (unsigned long)ESP_ERR_INVALID_STATE);
        printf("FDC_FATAL,stage=boot,reason=primary_not_ready,err=0x%lx\n",
               (unsigned long)ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }
    if (!ctx->state.fdcSecondary.ready) {
        ctx->fdcBootSummary = (sensorarrayFdcBootSummary_t){
            .transportErr = ESP_ERR_INVALID_STATE,
            .quality = requireDual ? SENSORARRAY_FDC_BOOT_QUALITY_FAIL : SENSORARRAY_FDC_BOOT_QUALITY_DEGRADED,
            .validCellCount = 0u,
            .failedCellCount = SENSORARRAY_MATRIX_CELL_COUNT,
            .failedRowMask = 0xFFu,
            .degraded = !requireDual,
            .reason = requireDual ? "secondary_not_ready_require_dual" : "secondary_unavailable_primary_only",
        };
        ctx->fdcBootSweepOk = false;
        ctx->fdcDegradedMode = !requireDual;
        ctx->fdcDiagnosticMode = requireDual;
        sensorarrayFdcMatrixEngineSetDiagnosticMode(&ctx->fdcEngine, requireDual);
        if (requireDual) {
            printf("FDC_FATAL,stage=boot,reason=secondary_not_ready_require_dual,err=0x%lx\n",
                   (unsigned long)ESP_ERR_INVALID_STATE);
            return ESP_ERR_INVALID_STATE;
        }
        printf("FDC_BUS_WARN,secondaryReady=0,action=primary_only_fallback,d5_d8=device_missing\n");
        printf("APP_FDC,stage=boot_sweep_skip,reason=secondary_unavailable,primaryReady=1,secondaryReady=0,action=primary_only,quality=%s\n",
               sensorarrayAppFdcBootQualityName(ctx->fdcBootSummary.quality));
        return ESP_OK;
    }

    sensorarrayFdcBootSummary_t bootSummary = {0};
    esp_err_t bootErr = sensorarrayFdcMatrixEngineRunBootSweep(&ctx->fdcEngine,
                                                               &ctx->scanPlan,
                                                               &bootSummary);
    sensorarrayLogStackHighWater("boot_sweep_after");
    ctx->fdcBootSummary = bootSummary;
    ctx->fdcBootSweepOk = (bootErr == ESP_OK &&
                           bootSummary.quality == SENSORARRAY_FDC_BOOT_QUALITY_OK);
    ctx->fdcDegradedMode = (bootSummary.quality == SENSORARRAY_FDC_BOOT_QUALITY_DEGRADED);
    ctx->fdcDiagnosticMode =
        (bootErr != ESP_OK) ||
        (CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED &&
         bootSummary.quality != SENSORARRAY_FDC_BOOT_QUALITY_OK);
    sensorarrayFdcMatrixEngineSetDiagnosticMode(&ctx->fdcEngine, ctx->fdcDiagnosticMode);
    printf("APP_FDC,stage=boot_sweep_return,err=0x%lx,ok=%d,quality=%s,validCount=%u,failCount=%u,cacheFilledCount=%u,required=%u,degraded=%u,diagnostic=%u,reason=%s\n",
           (unsigned long)bootErr,
           ctx->fdcBootSweepOk ? 1 : 0,
           sensorarrayAppFdcBootQualityName(bootSummary.quality),
           (unsigned)bootSummary.validCellCount,
           (unsigned)bootSummary.failedCellCount,
           (unsigned)bootSummary.cacheFilledCellCount,
           (unsigned)CONFIG_SENSORARRAY_FDC_BOOT_MIN_VALID_CELLS,
           ctx->fdcDegradedMode ? 1u : 0u,
           ctx->fdcDiagnosticMode ? 1u : 0u,
           bootSummary.reason ? bootSummary.reason : SENSORARRAY_NA);
    sensorarrayLogStartup("fdc_boot_sweep",
                          bootErr,
                          ctx->fdcBootSweepOk ? "ok" :
                          (ctx->fdcDegradedMode ? "degraded" : "failed"),
                          (int32_t)bootErr);
    if (ctx->fdcDiagnosticMode && CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED) {
        printf("FDC_FATAL,stage=boot,reason=boot_quality_not_ok,err=0x%lx,quality=%s,validCount=%u,required=%u,summaryReason=%s\n",
               (unsigned long)bootErr,
               sensorarrayAppFdcBootQualityName(bootSummary.quality),
               (unsigned)bootSummary.validCellCount,
               (unsigned)CONFIG_SENSORARRAY_FDC_BOOT_MIN_VALID_CELLS,
               bootSummary.reason ? bootSummary.reason : SENSORARRAY_NA);
    } else if (bootErr != ESP_OK) {
        ctx->fdcDiagnosticMode = false;
        sensorarrayFdcMatrixEngineSetDiagnosticMode(&ctx->fdcEngine, false);
        printf("FDC_BOOT,stage=warning,reason=boot_sweep_failed_not_required,err=0x%lx\n",
               (unsigned long)bootErr);
    }
    return bootErr;
}

static esp_err_t sensorarrayRunOneFrame(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayScanConfigApplyResult_t rowsApply = {0};
    sensorarrayScanConfigApplyPendingAtFrameBoundary(&rowsApply);
    if (ctx->runtimeMode == SENSORARRAY_RUNTIME_MODE_MIXED_ROW) {
        sensorarrayScanPlanBuildMixedExample(&ctx->scanPlan);
    } else {
        sensorarrayScanPlanBuildDefaultFdcMatrix(&ctx->scanPlan);
    }

    esp_err_t err = ESP_ERR_INVALID_STATE;
    switch (ctx->runtimeMode) {
    case SENSORARRAY_RUNTIME_MODE_FDC_MATRIX:
        err = sensorarrayFdcMatrixEngineReadFrame(&ctx->fdcEngine,
                                                  &ctx->scanPlan,
                                                  &ctx->frame);
        break;
    case SENSORARRAY_RUNTIME_MODE_ADS_MATRIX:
        err = sensorarrayAdsMatrixEngineReadFrame(&ctx->adsEngine,
                                                  &ctx->scanPlan,
                                                  &ctx->frame);
        break;
    case SENSORARRAY_RUNTIME_MODE_MIXED_ROW:
        err = sensorarrayMixedRowEngineReadFrame(&ctx->fdcEngine,
                                                 &ctx->adsEngine,
                                                 &ctx->scanPlan,
                                                 &ctx->frame);
        break;
    default:
        err = ESP_ERR_INVALID_STATE;
        break;
    }
    if (rowsApply.applied) {
        printf("RAPP,id=%lu,seq=%lu,old=%u,new=%u,gen=%lu,status=applied\n",
               (unsigned long)rowsApply.requestId,
               (unsigned long)ctx->frame.sequence,
               (unsigned)rowsApply.oldRows,
               (unsigned)rowsApply.newRows,
               (unsigned long)rowsApply.generation);
    }
    return err;
}

static void sensorarrayRuntimeRescueTick(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }
    if (ctx->runtimeMode == SENSORARRAY_RUNTIME_MODE_FDC_MATRIX ||
        ctx->runtimeMode == SENSORARRAY_RUNTIME_MODE_MIXED_ROW) {
        (void)sensorarrayFdcRescueTick(&ctx->fdcEngine, &ctx->frame, &ctx->fdcRescue);
    }
}

static void sensorarrayRuntimeI2cFallbackTick(sensorarrayAppContext_t *ctx)
{
    if (!ctx || !CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE) {
        return;
    }
    bool frameHasI2cError =
        ctx->frame.i2cErrorCount != 0u ||
        ctx->frame.firstReadErr == ESP_ERR_TIMEOUT ||
        ctx->frame.firstReadErr == ESP_FAIL;
    if (!frameHasI2cError) {
        ctx->runtimeI2cErrorStreak = 0u;
        return;
    }

    if (ctx->runtimeI2cErrorStreak < UINT32_MAX) {
        ctx->runtimeI2cErrorStreak++;
    }
    uint32_t threshold = (uint32_t)CONFIG_BOARD_I2C_RUNTIME_FALLBACK_ERROR_FRAMES;
    if (threshold == 0u) {
        threshold = 3u;
    }
    if (ctx->runtimeI2cErrorStreak < threshold) {
        return;
    }

    printf("ICLK,stage=runtime_fallback_begin,streak=%lu,threshold=%lu,seq=%lu,i2cErrorCount=%u,firstReadErr=0x%lx\n",
           (unsigned long)ctx->runtimeI2cErrorStreak,
           (unsigned long)threshold,
           (unsigned long)ctx->frame.sequence,
           (unsigned)ctx->frame.i2cErrorCount,
           (unsigned long)ctx->frame.firstReadErr);
    sensorarrayProbeAndLockFdcI2cClock(&ctx->state.fdcPrimary, "runtime_primary_reverify");
    sensorarrayProbeAndLockFdcI2cClock(&ctx->state.fdcSecondary, "runtime_secondary_reverify");
    ctx->runtimeI2cErrorStreak = 0u;
}

static void sensorarrayRunQueuedFullSweep(sensorarrayAppContext_t *ctx)
{
    if (!ctx || !sensorarrayFdcSweepConsumeForceFullSweepAll()) {
        return;
    }

    uint32_t maxFails = (uint32_t)CONFIG_SENSORARRAY_FDC_MAX_CONSECUTIVE_FULL_SWEEP_FAILS;
    if (maxFails != 0u && ctx->failedRescueCount >= maxFails) {
        ctx->fdcDiagnosticMode = true;
        sensorarrayFdcMatrixEngineSetDiagnosticMode(&ctx->fdcEngine, true);
        printf("FDC_RESCUE,stage=disabled,reason=max_failed_rescue,failedRescueCount=%lu,maxFailedRescue=%lu\n",
               (unsigned long)ctx->failedRescueCount,
               (unsigned long)maxFails);
        return;
    }

    int64_t nowUs = esp_timer_get_time();
    int64_t cooldownUs = (int64_t)CONFIG_SENSORARRAY_FDC_FULL_SWEEP_REQUEST_COOLDOWN_MS * 1000LL;
    bool cooldownElapsed = ctx->lastFullRescueTimeUs == 0 ||
                           cooldownUs <= 0 ||
                           (nowUs - ctx->lastFullRescueTimeUs) >= cooldownUs;
    if (ctx->rescueRunning) {
        printf("FDC_RESCUE,stage=skip,reason=already_running,failedRescueCount=%lu\n",
               (unsigned long)ctx->failedRescueCount);
        return;
    }
    if (!cooldownElapsed) {
        printf("FDC_RESCUE,stage=skip,reason=cooldown,remainingUs=%lld,failedRescueCount=%lu\n",
               (long long)(cooldownUs - (nowUs - ctx->lastFullRescueTimeUs)),
               (unsigned long)ctx->failedRescueCount);
        return;
    }

    uint32_t epoch = ++ctx->rescueEpoch;
    ctx->rescueRunning = true;
    int64_t rescueStartUs = esp_timer_get_time();
    sensorarrayLogRuntimeMemoryDiag("before_full_rescue", ctx);
    sensorarrayLogStackHighWater("full_rescue_before");
    printf("FDC_RESCUE,stage=begin,reason=queued_full_sweep_all,epoch=%lu,failedRescueCount=%lu\n",
           (unsigned long)epoch,
           (unsigned long)ctx->failedRescueCount);
    esp_err_t err = sensorarrayFdcMatrixEngineRunFullRescue(&ctx->fdcEngine,
                                                            "queued_full_sweep_all");
    ctx->lastFullRescueTimeUs = esp_timer_get_time();
    ctx->rescueRunning = false;
    sensorarrayLogStackHighWater("full_rescue_after");
    sensorarrayLogRuntimeMemoryDiag("after_full_rescue", ctx);
    int64_t durationUs = ctx->lastFullRescueTimeUs - rescueStartUs;
    if (err == ESP_OK) {
        ctx->failedRescueCount = 0u;
    } else {
        ctx->failedRescueCount++;
    }
    printf("FDC_RESCUE,stage=end,reason=queued_full_sweep_all,epoch=%lu,err=0x%lx,durationUs=%lld,failedRescueCount=%lu\n",
           (unsigned long)epoch,
           (unsigned long)err,
           (long long)durationUs,
           (unsigned long)ctx->failedRescueCount);
}

static void sensorarrayRunDiagnosticTick(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }
    BoardSupportI2cBusInfo_t primaryBus = {0};
    BoardSupportI2cBusInfo_t secondaryBus = {0};
    (void)boardSupportGetI2cBusInfo(false, &primaryBus);
    (void)boardSupportGetI2cBusInfo(true, &secondaryBus);
    printf("MATRIXFDC_DIAG,stage=diagnostic_mode,bootOk=%d,failedRescue=%lu,primaryReady=%d,secondaryReady=%d,primaryOffline=%u,secondaryOffline=%u\n",
           ctx->fdcBootSweepOk ? 1 : 0,
           (unsigned long)ctx->failedRescueCount,
           ctx->state.fdcPrimary.ready ? 1 : 0,
           ctx->state.fdcSecondary.ready ? 1 : 0,
           primaryBus.Offline ? 1u : 0u,
           secondaryBus.Offline ? 1u : 0u);
#if CONFIG_SENSORARRAY_FDC_DIAG_DUMP_REGS
    int64_t nowUs = esp_timer_get_time();
    int64_t intervalUs = (int64_t)CONFIG_SENSORARRAY_FDC_DIAG_DUMP_INTERVAL_MS * 1000LL;
    bool intervalElapsed = s_lastDiagnosticDumpUs == 0 ||
                           intervalUs <= 0 ||
                           (nowUs - s_lastDiagnosticDumpUs) >= intervalUs;
    bool skipOffline = CONFIG_SENSORARRAY_FDC_DIAG_DUMP_SKIP_OFFLINE_BUS != 0;
    if (intervalElapsed && (!skipOffline || (!primaryBus.Offline && !secondaryBus.Offline))) {
        esp_err_t dumpErr = sensorarrayFdcSweepDumpAllDeviceRegs(&ctx->state,
                                                                 "diagnostic_mode",
                                                                 "diagnostic_retry");
        s_lastDiagnosticDumpUs = nowUs;
        printf("MATRIXFDC_DIAG,stage=dump_done,err=0x%lx\n",
               (unsigned long)dumpErr);
    } else if (intervalElapsed) {
        printf("MATRIXFDC_DIAG,stage=dump_skip,reason=offline_bus,primaryOffline=%u,secondaryOffline=%u\n",
               primaryBus.Offline ? 1u : 0u,
               secondaryBus.Offline ? 1u : 0u);
        s_lastDiagnosticDumpUs = nowUs;
    }
#endif
    vTaskDelay(pdMS_TO_TICKS(1000u));
}

static void sensorarrayRunMainLoop(sensorarrayAppContext_t *ctx)
{
    sensorarrayBootBreadcrumbSetStage("main_loop", ESP_OK, ctx);
    while (true) {
        /* CommandMailbox is drained only here, before a new frame begins. No
         * BLE callback or Core0 task can mutate acquisition state mid-row. */
        sensorarrayApplyPendingCommands(ctx);
        sensorarrayRunQueuedFullSweep(ctx);

        if (CONFIG_SENSORARRAY_RUNTIME_PERIODIC_DIAG_ENABLE &&
            (ctx->fdcFrameCounter % 100u) == 0u) {
            sensorarrayLogStackHighWater("fdc_matrix_loop");
            sensorarrayLogRuntimeMemoryDiag("main_loop_100", ctx);
        }
        if (ctx->fdcDiagnosticMode ||
            sensorarrayFdcMatrixEngineDiagnosticMode(&ctx->fdcEngine)) {
            sensorarrayBootBreadcrumbSetStage("diagnostic_tick", ESP_OK, ctx);
            sensorarrayRunDiagnosticTick(ctx);
            continue;
        }

        int64_t frameStartUs = esp_timer_get_time();
        sensorarrayBootBreadcrumbSetStage("frame_read", ESP_OK, ctx);
        esp_err_t err = sensorarrayRunOneFrame(ctx);
        sensorarrayBootBreadcrumbSetStage("frame_done", err, ctx);
        uint64_t measureFrameUs = (uint64_t)(esp_timer_get_time() - frameStartUs);
        ctx->fdcFrameCounter++;

        bool allInvalid = ctx->frame.capValidMask == 0u;
        if (allInvalid) {
            bool rawAllZero = sensorarrayFrameRawAllZero(&ctx->frame);
            if (ctx->asyncLogReady) {
                (void)sensorarrayAsyncLogPublishFrameError(&ctx->frame,
                                                           err,
                                                           rawAllZero,
                                                           ctx->fdcBootSweepOk);
            } else {
                uint8_t invalidSentinelCount =
                    (uint8_t)(ctx->frame.activeRows * SENSORARRAY_MATRIX_COLS -
                              ctx->frame.validCount);
                printf("MATRIXFDC_DIAG,stage=all_invalid_frame,seq=%lu,errorMask=0x%016llX,readErr=0x%lx,bootOk=%u,freshCount=%u,hardwareZeroRawCount=%u,notReadyCount=%u,zeroBeforeReadyCount=%u,zeroAfterDrdyCount=%u,i2cErrorCount=%u,unreadWithoutDrdyCount=%u,softInvalidCount=%u,hardInvalidCount=%u,staleUnreadDrainCount=%u,invalidSentinelCount=%u,rawAllZero=%u\n",
                       (unsigned long)ctx->frame.sequence,
                       (unsigned long long)ctx->frame.errorMask,
                       (unsigned long)err,
                       ctx->fdcBootSweepOk ? 1u : 0u,
                       (unsigned)ctx->frame.freshCount,
                       (unsigned)ctx->frame.hardwareZeroRawCount,
                       (unsigned)ctx->frame.notReadyCount,
                       (unsigned)ctx->frame.zeroBeforeReadyCount,
                       (unsigned)ctx->frame.zeroAfterDrdyCount,
                       (unsigned)ctx->frame.i2cErrorCount,
                       (unsigned)ctx->frame.unreadWithoutDrdyCount,
                       (unsigned)ctx->frame.softInvalidCount,
                       (unsigned)ctx->frame.hardInvalidCount,
                       (unsigned)ctx->frame.staleUnreadDrainCount,
                       (unsigned)invalidSentinelCount,
                       rawAllZero ? 1u : 0u);
            }
        } else if (err != ESP_OK) {
            if (ctx->asyncLogReady) {
                (void)sensorarrayAsyncLogPublishFrameError(&ctx->frame,
                                                           err,
                                                           false,
                                                           ctx->fdcBootSweepOk);
            } else {
                ESP_LOGE("SensorArray",
                         "FRAME_ERROR,err=0x%lx,validMask=0x%016llX,errorMask=0x%016llX",
                         (unsigned long)err,
                         (unsigned long long)ctx->frame.capValidMask,
                         (unsigned long long)ctx->frame.errorMask);
            }
        }

        if (ctx->asyncLogReady) {
            (void)sensorarrayAsyncLogPublishFrameSnapshot(&ctx->frame, measureFrameUs);
        } else if (ctx->legacySyncOutput &&
                   (ctx->frame.freshFrame || CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG)) {
            (void)sensorarrayFrameOutputPrint(&ctx->frame);
        }
        sensorarrayRuntimeRescueTick(ctx);
        sensorarrayRuntimeI2cFallbackTick(ctx);
        sensorarrayDelayFramePeriodSince(ctx, frameStartUs, ctx->frame.sequence);
    }
}

static void sensorarrayScanTask(void *arg)
{
    sensorarrayAppContext_t *ctx = (sensorarrayAppContext_t *)arg;
    sensorarrayBootBreadcrumbSetStage("scan_task_start", ESP_OK, ctx);
    printf("TASKCORE,name=acquisition,core=%d,expected=%d,priority=%u,stackBytes=%u\n",
           (int)xPortGetCoreID(),
           CONFIG_SENSORARRAY_ADC_CORE,
           (unsigned)uxTaskPriorityGet(NULL),
           (unsigned)CONFIG_SENSORARRAY_SCAN_TASK_STACK);

    sensorarrayBootBreadcrumbSetStage("init_system", ESP_OK, ctx);
    esp_err_t initErr = sensorarrayInitSystem(ctx);
    if (initErr != ESP_OK) {
        sensorarrayBootBreadcrumbSetStage("init_failed", initErr, ctx);
        printf("APP_FATAL,stage=acquisition_init,err=%ld,action=safe_idle_no_restart\n",
               (long)initErr);
        for (;;) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    sensorarrayBootBreadcrumbSetStage("boot_sweep", ESP_OK, ctx);
    sensorarrayLogRuntimeMemoryDiag("before_boot_sweep", ctx);
    esp_err_t bootErr = sensorarrayRunBootCalibration(ctx);
    sensorarrayBootBreadcrumbSetStage("boot_sweep_done", bootErr, ctx);
    sensorarrayLogRuntimeMemoryDiag("after_boot_sweep", ctx);
    if (CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED &&
        (bootErr != ESP_OK || ctx->fdcBootSummary.quality != SENSORARRAY_FDC_BOOT_QUALITY_OK)) {
        ctx->fdcDiagnosticMode = true;
        sensorarrayFdcMatrixEngineSetDiagnosticMode(&ctx->fdcEngine, true);
    }

    sensorarrayFdcMapVerifyDebug();
    if (sensorarrayFdcWaveDebugEnabled()) {
        sensorarrayRunFdcWaveDebugLoop(ctx);
    }
    sensorarrayRunMainLoop(ctx);
    vTaskDelete(NULL);
}

static esp_err_t sensorarrayStartScanTask(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    TaskHandle_t scanTaskHandle = NULL;
    BaseType_t taskOk = xTaskCreatePinnedToCore(sensorarrayScanTask,
                                                "sensorarrayScanTask",
                                                CONFIG_SENSORARRAY_SCAN_TASK_STACK,
                                                ctx,
                                                CONFIG_SENSORARRAY_SCAN_TASK_PRIO,
                                                &scanTaskHandle,
                                                CONFIG_SENSORARRAY_SCAN_TASK_CORE);
    if (taskOk != pdPASS || !scanTaskHandle) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

static esp_err_t sensorarrayInitSystem(sensorarrayAppContext_t *ctx)
{
    esp_err_t err = sensorarrayInitRuntime(ctx);
    printf("APP_INIT,stage=runtime,err=%ld\n", (long)err);
    if (err != ESP_OK) {
        return err;
    }

    /* Bring up TextFrameBus/EventRing before hardware initialisation so Core 1
     * startup and runtime diagnostics can be handed to Core 0 immediately. */
    sensorarrayInitAsyncLogging(ctx);
    if (ctx->asyncLogReady) {
        boardSupportSetLogCallback(sensorarrayAsyncLogPublishTextEvent);
    }

    sensorarrayLogRuntimeMemoryDiag("before_board_support_init", ctx);
    err = sensorarrayInitBoardAndRouting(ctx);
    sensorarrayLogRuntimeMemoryDiag("after_board_support_init", ctx);
    printf("APP_INIT,stage=board_routing,err=%ld\n", (long)err);
    if (err != ESP_OK) {
        return err;
    }

    if (!ctx->state.boardReady) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayLogRuntimeMemoryDiag("before_frontends", ctx);
    err = sensorarrayInitFrontends(ctx);
    sensorarrayLogRuntimeMemoryDiag("after_frontends", ctx);
    printf("APP_INIT,stage=frontends,err=%ld\n", (long)err);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayBuildDefaultScanPlan(ctx);
    return ESP_OK;
}

void app_main(void)
{
    memset(&s_appContext, 0, sizeof(s_appContext));
    sensorarrayBootBreadcrumbStart();
    printf("COREMAP,acq=%d,misc=%d,fdcP=%d,fdcS=%d,ads=%d,log=%d,out=%d,net=%d\n",
           CONFIG_SENSORARRAY_ADC_CORE,
           CONFIG_SENSORARRAY_MISC_CORE,
           CONFIG_SENSORARRAY_FDC_PRIMARY_WORKER_TASK_CORE,
           CONFIG_SENSORARRAY_FDC_SECONDARY_WORKER_TASK_CORE,
           CONFIG_SENSORARRAY_ADS_WORKER_CORE,
           CONFIG_SENSORARRAY_LOG_TASK_CORE,
           CONFIG_SENSORARRAY_OUTPUT_TASK_CORE,
           CONFIG_SENSORARRAY_NET_TASK_CORE);

    /* BLE controller memory must be reserved before Wi-Fi, board drivers,
     * frame buses, and acquisition worker stacks consume internal RAM. */
    esp_err_t netErr = sensorarrayNetStatusInit();
    if (netErr != ESP_OK && netErr != ESP_ERR_NOT_SUPPORTED) {
        printf("NET_WARN,stage=ble_first_init,err=0x%lx,name=%s,action=continue_acquisition\n",
               (unsigned long)netErr, esp_err_to_name(netErr));
    }

    esp_err_t scanErr = sensorarrayStartScanTask(&s_appContext);
    if (scanErr != ESP_OK) {
        sensorarrayBootBreadcrumbSetStage("scan_task_create_failed", scanErr, &s_appContext);
        printf("APP_FATAL,stage=scan_task_create,err=0x%lx,action=safe_idle_no_restart\n",
               (unsigned long)scanErr);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000u));
        }
    }
    printf("APP_SCAN_TASK,stage=created,targetCore=%d,priority=%u,stackBytes=%u\n",
           CONFIG_SENSORARRAY_SCAN_TASK_CORE,
           (unsigned)CONFIG_SENSORARRAY_SCAN_TASK_PRIO,
           (unsigned)CONFIG_SENSORARRAY_SCAN_TASK_STACK);
}
