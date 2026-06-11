#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "boardSupport.h"
#include "tmuxSwitch.h"

#include "sensorarrayAdsMatrix.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFdcMatrix.h"
#include "sensorarrayFdcRescue.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayFrame.h"
#include "sensorarrayAsyncLog.h"
#include "sensorarrayFrameOutput.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayMixedRow.h"
#include "sensorarrayScanPlan.h"
#include "sensorarrayTypes.h"

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
    uint32_t periodMs = (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS;
    return periodMs == 0u ? 1u : periodMs;
}

static void sensorarrayDelayFramePeriodSince(sensorarrayAppContext_t *ctx,
                                             int64_t frameStartUs,
                                             uint32_t sequence)
{
    uint32_t periodMs = sensorarrayFdcFramePeriodMs();
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

    switch (ctx->runtimeMode) {
    case SENSORARRAY_RUNTIME_MODE_FDC_MATRIX:
        return sensorarrayFdcMatrixEngineReadFrame(&ctx->fdcEngine,
                                                   &ctx->scanPlan,
                                                   &ctx->frame);
    case SENSORARRAY_RUNTIME_MODE_ADS_MATRIX:
        return sensorarrayAdsMatrixEngineReadFrame(&ctx->adsEngine,
                                                   &ctx->scanPlan,
                                                   &ctx->frame);
    case SENSORARRAY_RUNTIME_MODE_MIXED_ROW:
        return sensorarrayMixedRowEngineReadFrame(&ctx->fdcEngine,
                                                  &ctx->adsEngine,
                                                  &ctx->scanPlan,
                                                  &ctx->frame);
    default:
        return ESP_ERR_INVALID_STATE;
    }
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
    while (true) {
        sensorarrayRunQueuedFullSweep(ctx);

        if ((ctx->fdcFrameCounter % 100u) == 0u) {
            sensorarrayLogStackHighWater("fdc_matrix_loop");
            sensorarrayLogRuntimeMemoryDiag("main_loop_100", ctx);
        }
        if (ctx->fdcDiagnosticMode ||
            sensorarrayFdcMatrixEngineDiagnosticMode(&ctx->fdcEngine)) {
            sensorarrayRunDiagnosticTick(ctx);
            continue;
        }

        int64_t frameStartUs = esp_timer_get_time();
        esp_err_t err = sensorarrayRunOneFrame(ctx);
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
                    (uint8_t)(SENSORARRAY_MATRIX_CELL_COUNT - ctx->frame.validCount);
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
        } else if (ctx->legacySyncOutput) {
            (void)sensorarrayFrameOutputPrint(&ctx->frame);
        }
        sensorarrayRuntimeRescueTick(ctx);
        sensorarrayRuntimeI2cFallbackTick(ctx);
        sensorarrayDelayFramePeriodSince(ctx, frameStartUs, ctx->frame.sequence);
    }
}

static esp_err_t sensorarrayInitSystem(sensorarrayAppContext_t *ctx)
{
    esp_err_t err = sensorarrayInitRuntime(ctx);
    printf("APP_INIT,stage=runtime,err=%ld\n", (long)err);
    if (err != ESP_OK) {
        return err;
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
    sensorarrayInitAsyncLogging(ctx);
    return ESP_OK;
}

void app_main(void)
{
    sensorarrayLogRuntimeMemoryDiag("app_main_entry_before_clear", &s_appContext);
    memset(&s_appContext, 0, sizeof(s_appContext));
    sensorarrayLogRuntimeMemoryDiag("app_main_after_clear", &s_appContext);

    esp_err_t initErr = sensorarrayInitSystem(&s_appContext);
    if (initErr != ESP_OK) {
        printf("APP_FATAL,stage=init,err=%ld,action=safe_idle_no_restart\n",
               (long)initErr);
        while (true) {
            printf("APP_FATAL,stage=idle,err=%ld,boardReady=%u,frontendsReady=%u,fdcDiagnosticMode=%u\n",
                   (long)initErr,
                   s_appContext.state.boardReady ? 1u : 0u,
                   (s_appContext.state.adsReady || s_appContext.state.fdcPrimary.ready || s_appContext.state.fdcSecondary.ready) ? 1u : 0u,
                   s_appContext.fdcDiagnosticMode ? 1u : 0u);
            vTaskDelay(pdMS_TO_TICKS(1000u));
        }
    }
    sensorarrayLogRuntimeMemoryDiag("before_boot_sweep", &s_appContext);
    esp_err_t bootErr = sensorarrayRunBootCalibration(&s_appContext);
    sensorarrayLogRuntimeMemoryDiag("after_boot_sweep", &s_appContext);
    if (CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED &&
        (bootErr != ESP_OK ||
         s_appContext.fdcBootSummary.quality != SENSORARRAY_FDC_BOOT_QUALITY_OK)) {
        s_appContext.fdcDiagnosticMode = true;
        sensorarrayFdcMatrixEngineSetDiagnosticMode(&s_appContext.fdcEngine, true);
    }

    sensorarrayRunMainLoop(&s_appContext);
}
