#include <stdbool.h>
#include <inttypes.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
#include "sensorarrayAdsFault.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayCommandMailbox.h"
#include "sensorarrayCalibration.h"
#include "sensorarrayEarlyRecovery.h"
#include "sensorarrayFdcMatrix.h"
#include "sensorarrayFdcRescue.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayFrame.h"
#include "sensorarrayFrameBuilder.h"
#include "sensorarrayAsyncLog.h"
#include "sensorarrayAcqEvent.h"
#include "sensorarrayFrameOutput.h"
#include "sensorarrayUsbSink.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayMeasurementMode.h"
#include "sensorarrayRowModeProfile.h"
#include "sensorarrayMeasurementSelfTest.h"
#include "sensorarrayMixedRow.h"
#include "sensorarrayNetStatus.h"
#include "sensorarrayScanConfig.h"
#include "sensorarrayScanPlan.h"
#include "sensorarrayRouteController.h"
#include "sensorarrayTransport.h"
#include "sensorarrayTextProtocol.h"
#include "sensorarrayTypes.h"
#include "esp_rtc_time.h"

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
    uint32_t sequence;
    uint8_t activeRows;
    sensorarrayMeasurementPayload_t payload;
} sensorarrayLastMeasurementSnapshot_t;

typedef struct {
    uint32_t sequence;
    sensorarrayMeasurementMode_t mode;
    sensorarrayMeasurementUnit_t unit;
    sensorarrayAdsReferenceSource_t referenceSource;
    int64_t valueFixed;
    int32_t rawCode;
    int32_t nodeUv;
    int32_t avssUv;
    int32_t matrixReferenceUv;
    uint32_t referenceResistorOhms;
    uint32_t railAgeFrames;
    uint8_t pgaGain;
    uint8_t errorReason;
    int8_t decimalScale;
    bool valid;
    bool fresh;
    bool railValid;
} sensorarrayMeasurementCellSnapshot_t;

typedef struct {
    sensorarrayRuntimeMode_t runtimeMode;
    sensorarrayState_t state;
    sensorarrayScanPlan_t scanPlan;
    sensorarrayFrame_t frame;
    /* Reusable mixed-mode segment frame owned by the Core 1 scan task.  The
     * frame is CPU-only scratch and must never sit on the acquisition stack. */
    sensorarrayFrame_t *mixedSegmentWorkspace;
    sensorarrayFdcMatrixEngine_t fdcEngine;
    sensorarrayAdsMatrixEngine_t adsEngine;
    sensorarrayMeasurementModeContext_t measurementMode;
    sensorarrayMeasurementRecovery_t adsRestoreRecovery;
    sensorarrayRowModeProfile_t rowModeProfile;
    sensorarrayRouteController_t routeController;
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
    uint32_t hostFrameSequence;
    uint32_t modeTransitionCount;
    uint32_t railCalibrationGeneration;
    bool pendingAdsCheck;
    uint32_t pendingAdsCheckRequestId;
    uint32_t pendingAdsCheckSamples;
    uint32_t pendingBatteryRequestId;
    bool pendingBatteryDiagnostic;
    TaskHandle_t scanTaskHandle;
    volatile uint32_t lastMeasurementVersion;
    sensorarrayLastMeasurementSnapshot_t lastMeasurement;
    volatile bool systemReady;
} sensorarrayAppContext_t;

static sensorarrayAppContext_t s_appContext;
static int64_t s_lastDiagnosticDumpUs __attribute__((unused));
static esp_err_t sensorarrayInitSystem(sensorarrayAppContext_t *ctx);
static bool sensorarrayCalibrationMatrixPayloadValid(const void *payload,
                                                     size_t payloadLength);
static void sensorarrayCalibrationLogOutcome(
    const char *tag,
    const char *state,
    const char *reason,
    const sensorarrayCalibrationStatus_t *status,
    esp_err_t err);
static const char *sensorarrayCalibrationLoadReason(
    const sensorarrayCalibrationStatus_t *status,
    esp_err_t err);

#define SENSORARRAY_PAYLOAD_TOO_LARGE_RATE_LIMIT_US (1000000LL)
static int64_t s_lastPayloadTooLargeDropUs;
static uint32_t s_payloadTooLargeDropCount;

#define SENSORARRAY_ROW_MODES_TERMINAL_TEXT_MAX 192u

#define SENSORARRAY_BOOT_BREADCRUMB_MAGIC 0x53414252u
#define SENSORARRAY_BOOT_BREADCRUMB_VERSION 2u
#define SENSORARRAY_BOOT_BREADCRUMB_STAGE_MAX 32u
#define SENSORARRAY_BOOT_RESTART_KIND_NONE 0u
#define SENSORARRAY_BOOT_RESTART_KIND_MANUAL 1u
#define SENSORARRAY_BOOT_RESTART_KIND_AUTO 2u
#define SENSORARRAY_BOOT_GUARD_NORMAL 0u
#define SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE 1u
#define SENSORARRAY_BOOT_AUTO_RESTART_MAX 3u
#define SENSORARRAY_BOOT_AUTO_RESTART_WINDOW_US (30ULL * 1000000ULL)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t bootId;
    uint32_t bootCount;
    uint32_t resetReason;
    char lastStage[SENSORARRAY_BOOT_BREADCRUMB_STAGE_MAX];
    int32_t lastErr;
    uint32_t lastFrameSeq;
    uint32_t minFreeHeap;
    char prevStage[SENSORARRAY_BOOT_BREADCRUMB_STAGE_MAX];
    int32_t prevErr;
    uint32_t prevHeap;
    uint32_t restartKind;
    uint32_t guardState;
    uint32_t autoRestartCount;
    uint64_t autoRestartWindowStartUs;
} sensorarrayBootBreadcrumb_t;

RTC_NOINIT_ATTR static sensorarrayBootBreadcrumb_t s_bootBreadcrumb;
static portMUX_TYPE s_bootBreadcrumbMux = portMUX_INITIALIZER_UNLOCKED;

#define SENSORARRAY_RECOVERY_LEVEL_NONE 0xFFu
#define SENSORARRAY_RECOVERY_LEVEL_SOFT 0u
#define SENSORARRAY_RECOVERY_LEVEL_FULL 1u
#define SENSORARRAY_RECOVERY_LEVEL_RESTART 2u
#define SENSORARRAY_RECOVERY_LEVEL_MAX 2u
#define SENSORARRAY_PROTOCOL_VERSION 1u
#define SENSORARRAY_ADS_RECOVERY_CHECK_SAMPLES 1u
#define SENSORARRAY_ADS_RECOVERY_ATTEMPT_DELAY_MS 20u

static portMUX_TYPE s_recoveryRequestMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_recoveryRequestLevel = SENSORARRAY_RECOVERY_LEVEL_NONE;
static volatile uint32_t s_recoveryRequestId;
static volatile uint32_t s_restartRequested;
static volatile uint32_t s_restartRequestId;
static uint32_t s_recoveryRequestSeq;

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
    case ESP_RST_USB:
        return "usb";
    case ESP_RST_JTAG:
        return "jtag";
    case ESP_RST_EFUSE:
        return "efuse";
    case ESP_RST_PWR_GLITCH:
        return "power_glitch";
    case ESP_RST_CPU_LOCKUP:
        return "cpu_lockup";
    default:
        return "unknown";
    }
}

static void sensorarrayBootBreadcrumbSetStage(const char *stage,
                                              esp_err_t err,
                                              const sensorarrayAppContext_t *ctx)
{
    portENTER_CRITICAL(&s_bootBreadcrumbMux);
    if (s_bootBreadcrumb.magic != SENSORARRAY_BOOT_BREADCRUMB_MAGIC ||
        s_bootBreadcrumb.version != SENSORARRAY_BOOT_BREADCRUMB_VERSION) {
        portEXIT_CRITICAL(&s_bootBreadcrumbMux);
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
    portEXIT_CRITICAL(&s_bootBreadcrumbMux);
}

static void sensorarrayBootBreadcrumbStart(void)
{
    sensorarrayBootBreadcrumb_t previous = s_bootBreadcrumb;
    bool previousValid =
        previous.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC &&
        previous.version == SENSORARRAY_BOOT_BREADCRUMB_VERSION;
    bool previousV1 =
        previous.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC &&
        previous.version == 1u;
    esp_reset_reason_t reason = esp_reset_reason();

    uint32_t nextBootId = previousValid ? previous.bootId + 1u : 1u;
    uint32_t nextBootCount = previousValid ? previous.bootCount + 1u :
        (previousV1 ? previous.bootCount + 1u : 1u);
    uint32_t guardState = previousValid ?
        previous.guardState : SENSORARRAY_BOOT_GUARD_NORMAL;
    uint32_t autoRestartCount = previousValid ? previous.autoRestartCount : 0u;
    uint64_t autoRestartWindowStartUs = previousValid ?
        previous.autoRestartWindowStartUs : 0u;
    if (reason == ESP_RST_POWERON) {
        guardState = SENSORARRAY_BOOT_GUARD_NORMAL;
        autoRestartCount = 0u;
        autoRestartWindowStartUs = 0u;
    }

    printf("RST,reason=%s,code=%ld,boot=%lu,bootId=%lu,prevValid=%u,prevKind=%s,prevStage=%s,prevErr=0x%lx,prevSeq=%lu,prevHeap=%lu,guard=%s,autoRestarts=%lu\n",
           sensorarrayResetReasonName(reason),
           (long)reason,
           (unsigned long)nextBootCount,
           (unsigned long)nextBootId,
           previousValid ? 1u : 0u,
           previousValid ?
               (previous.restartKind == SENSORARRAY_BOOT_RESTART_KIND_MANUAL ? "manual" :
                previous.restartKind == SENSORARRAY_BOOT_RESTART_KIND_AUTO ? "auto" : "none") :
               "none",
           previousValid ? previous.lastStage : "none",
           previousValid ? (unsigned long)(uint32_t)previous.lastErr : 0u,
           previousValid ? (unsigned long)previous.lastFrameSeq : 0u,
           previousValid ? (unsigned long)previous.minFreeHeap : 0u,
           guardState == SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE ?
               "recovery_safe" : "normal",
           (unsigned long)autoRestartCount);

    portENTER_CRITICAL(&s_bootBreadcrumbMux);
    memset(&s_bootBreadcrumb, 0, sizeof(s_bootBreadcrumb));
    s_bootBreadcrumb.magic = SENSORARRAY_BOOT_BREADCRUMB_MAGIC;
    s_bootBreadcrumb.version = SENSORARRAY_BOOT_BREADCRUMB_VERSION;
    s_bootBreadcrumb.bootId = nextBootId;
    s_bootBreadcrumb.bootCount = nextBootCount;
    s_bootBreadcrumb.resetReason = (uint32_t)reason;
    s_bootBreadcrumb.guardState = guardState;
    s_bootBreadcrumb.autoRestartCount = autoRestartCount;
    s_bootBreadcrumb.autoRestartWindowStartUs = autoRestartWindowStartUs;
    s_bootBreadcrumb.restartKind = SENSORARRAY_BOOT_RESTART_KIND_NONE;
    if (previousValid) {
        snprintf(s_bootBreadcrumb.prevStage,
                 sizeof(s_bootBreadcrumb.prevStage),
                 "%s",
                 previous.lastStage);
        s_bootBreadcrumb.prevErr = previous.lastErr;
        s_bootBreadcrumb.prevHeap = previous.minFreeHeap;
    }
    s_bootBreadcrumb.minFreeHeap = esp_get_free_heap_size();
    portEXIT_CRITICAL(&s_bootBreadcrumbMux);
    sensorarrayBootBreadcrumbSetStage("app_main", ESP_OK, NULL);
}

static bool sensorarrayBootBreadcrumbCopy(sensorarrayBootBreadcrumb_t *outCopy)
{
    if (!outCopy) {
        return false;
    }
    portENTER_CRITICAL(&s_bootBreadcrumbMux);
    *outCopy = s_bootBreadcrumb;
    portEXIT_CRITICAL(&s_bootBreadcrumbMux);
    return outCopy->magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC &&
           outCopy->version == SENSORARRAY_BOOT_BREADCRUMB_VERSION;
}

static void sensorarrayAdsFaultSink(const char *line,
                                    size_t length,
                                    void *context)
{
    (void)length;
    (void)context;
    printf("%s", line);
}

static void sensorarrayAdsFaultFillContext(sensorarrayAppContext_t *ctx,
                                           sensorarrayAdsFaultEvent_t *event)
{
    if (!ctx || !event) {
        return;
    }
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    if (sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode)) {
        event->modeGeneration = mode.generation;
        event->mode = sensorarrayMeasurementModeName(mode.activeMode);
    }
    sensorarrayRowModeProfile_t profile = {0};
    if (sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile)) {
        event->profileGeneration = profile.generation;
        event->rowGeneration = profile.generation;
        event->rowRequestId = profile.pending ?
            profile.pendingRequestId : profile.appliedRequestId;
    }
    sensorarrayRouteSnapshot_t route = {0};
    if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController, &route)) {
        event->route = route.safe ? "SAFE" :
            sensorarrayMeasurementModeName(route.mode);
    }
    sensorarrayAdsRegisterCache_t *registerCache =
        sensorarrayAdsMatrixEngineRegisterCache(&ctx->adsEngine);
    if (registerCache) {
        event->configGeneration = registerCache->generation;
    }
    event->drdyGeneration = ads126xAdcGetDrdyGeneration(&ctx->state.ads);
    if (ctx->frame.measurement.railValid) {
        event->railValid = true;
        event->railUv = ctx->frame.measurement.avddUv;
        event->reference = sensorarrayAdsReferenceSourceName(
            ctx->frame.measurement.referenceSource);
    }
}

static void sensorarrayEmitAdsFault(sensorarrayAppContext_t *ctx,
                                    sensorarrayAdsFaultEvent_t *event)
{
    if (!ctx || !event) {
        return;
    }
    sensorarrayBootBreadcrumb_t boot = {0};
    if (sensorarrayBootBreadcrumbCopy(&boot)) {
        event->bootId = boot.bootId;
        event->bootCount = boot.bootCount;
    }
    (void)sensorarrayAdsFaultEmit(
        event,
        (uint64_t)esp_timer_get_time(),
        sensorarrayAdsFaultSink,
        NULL);
}

static bool sensorarrayAdsRegisterCacheIsVerified(
    const sensorarrayAdsRegisterCache_t *cache)
{
    if (!cache || !cache->vrefValid || !cache->pgaModeValid ||
        !cache->adc1RunningValid) {
        return false;
    }
    for (uint8_t index = 0u; index < SENSORARRAY_ADS_REGISTER_COUNT; ++index) {
        const sensorarrayAdsRegisterShadowValue_t *entry =
            &cache->registers[index];
        if (!entry->valid ||
            entry->verifiedGeneration != cache->generation) {
            return false;
        }
    }
    return true;
}

static const char *sensorarrayAdsRecoveryOwner(
    const sensorarrayMeasurementRecovery_t *recovery)
{
    if (!recovery ||
        recovery->trigger == SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_BATTERY_RESTORE) {
        return sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_BATTERY);
    }
    return sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_CHECK);
}

static void sensorarrayEmitAdsRecoveryFault(
    sensorarrayAppContext_t *ctx,
    sensorarrayAdsFaultStage_t stage,
    uint32_t attempt,
    sensorarrayAdsFaultOutcome_t outcome,
    esp_err_t err,
    uint32_t sequence)
{
    if (!ctx) {
        return;
    }
    sensorarrayAdsFaultEvent_t fault = {
        .stage = stage,
        .err = err,
        .seq = sequence,
        .attempt = attempt,
        .outcome = outcome,
        .owner = sensorarrayAdsRecoveryOwner(&ctx->adsRestoreRecovery),
    };
    sensorarrayAdsFaultFillContext(ctx, &fault);
    fault.mode = sensorarrayMeasurementModeName(
        ctx->adsRestoreRecovery.resumeMode);
    sensorarrayEmitAdsFault(ctx, &fault);
}

static bool sensorarrayBeginAdsRestoreRecovery(
    sensorarrayAppContext_t *ctx,
    sensorarrayMeasurementRecoveryTrigger_t trigger,
    esp_err_t error,
    uint32_t triggerRequestId)
{
    if (!ctx) {
        return false;
    }
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    if (!sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        !sensorarrayMeasurementRecoveryStart(
            &ctx->adsRestoreRecovery,
            mode.activeMode,
            mode.appliedRequestId,
            trigger,
            (uint32_t)error,
            ctx->frame.sequence,
            triggerRequestId)) {
        return false;
    }
    sensorarrayMeasurementModeEnterRecovery(&ctx->measurementMode,
                                            (uint32_t)error);
    (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                              "ads_restore_recovery");
    sensorarrayAdsMatrixEngineInvalidateCaches(&ctx->adsEngine);
    (void)sensorarrayAdsMatrixEngineSetMode(
        &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
    sensorarrayEmitAdsRecoveryFault(
        ctx,
        SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_START,
        0u,
        SENSORARRAY_ADS_FAULT_OUTCOME_STARTED,
        error,
        ctx->frame.sequence);
    return true;
}

static void sensorarrayBootBreadcrumbMarkStableReady(void)
{
    portENTER_CRITICAL(&s_bootBreadcrumbMux);
    if (s_bootBreadcrumb.magic != SENSORARRAY_BOOT_BREADCRUMB_MAGIC ||
        s_bootBreadcrumb.version != SENSORARRAY_BOOT_BREADCRUMB_VERSION) {
        portEXIT_CRITICAL(&s_bootBreadcrumbMux);
        return;
    }
    s_bootBreadcrumb.guardState = SENSORARRAY_BOOT_GUARD_NORMAL;
    s_bootBreadcrumb.autoRestartCount = 0u;
    s_bootBreadcrumb.autoRestartWindowStartUs = 0u;
    portEXIT_CRITICAL(&s_bootBreadcrumbMux);
}

static bool sensorarrayBootPrepareRestart(uint32_t restartKind)
{
    bool allowed = true;
    uint64_t nowUs = esp_rtc_get_time_us();
    portENTER_CRITICAL(&s_bootBreadcrumbMux);
    if (restartKind == SENSORARRAY_BOOT_RESTART_KIND_AUTO) {
        if (s_bootBreadcrumb.guardState == SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE) {
            allowed = false;
        } else {
            uint64_t windowStartUs = s_bootBreadcrumb.autoRestartWindowStartUs;
            if (windowStartUs == 0u ||
                nowUs < windowStartUs ||
                (nowUs - windowStartUs) > SENSORARRAY_BOOT_AUTO_RESTART_WINDOW_US) {
                s_bootBreadcrumb.autoRestartWindowStartUs = nowUs;
                s_bootBreadcrumb.autoRestartCount = 0u;
            }
            if (s_bootBreadcrumb.autoRestartCount >= SENSORARRAY_BOOT_AUTO_RESTART_MAX) {
                s_bootBreadcrumb.guardState = SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE;
                allowed = false;
            } else {
                s_bootBreadcrumb.autoRestartCount++;
            }
        }
    }
    if (allowed) {
        s_bootBreadcrumb.restartKind = restartKind;
    }
    portEXIT_CRITICAL(&s_bootBreadcrumbMux);
    return allowed;
}

static void sensorarrayBootEmitRestartMarker(const char *marker)
{
    if (!marker || marker[0] == '\0') {
        return;
    }
    (void)fwrite(marker, 1u, strlen(marker), stdout);
    (void)fflush(stdout);
}

static void sensorarrayRecoveryPost(uint32_t level, uint32_t *outRequestId)
{
    portENTER_CRITICAL(&s_recoveryRequestMux);
    s_recoveryRequestSeq++;
    s_recoveryRequestLevel = level;
    s_recoveryRequestId = s_recoveryRequestSeq;
    if (outRequestId) {
        *outRequestId = s_recoveryRequestSeq;
    }
    portEXIT_CRITICAL(&s_recoveryRequestMux);
}

static bool sensorarrayRecoveryTake(uint32_t *outLevel, uint32_t *outRequestId)
{
    portENTER_CRITICAL(&s_recoveryRequestMux);
    uint32_t level = s_recoveryRequestLevel;
    uint32_t requestId = s_recoveryRequestId;
    s_recoveryRequestLevel = SENSORARRAY_RECOVERY_LEVEL_NONE;
    s_recoveryRequestId = 0u;
    portEXIT_CRITICAL(&s_recoveryRequestMux);
    if (outLevel) {
        *outLevel = level;
    }
    if (outRequestId) {
        *outRequestId = requestId;
    }
    return level != SENSORARRAY_RECOVERY_LEVEL_NONE;
}

static void sensorarrayRestartPost(uint32_t *outRequestId)
{
    portENTER_CRITICAL(&s_recoveryRequestMux);
    s_recoveryRequestSeq++;
    s_restartRequested = 1u;
    s_restartRequestId = s_recoveryRequestSeq;
    if (outRequestId) {
        *outRequestId = s_recoveryRequestSeq;
    }
    portEXIT_CRITICAL(&s_recoveryRequestMux);
}

static bool sensorarrayRestartTake(uint32_t *outRequestId)
{
    portENTER_CRITICAL(&s_recoveryRequestMux);
    bool pending = s_restartRequested != 0u;
    uint32_t requestId = s_restartRequestId;
    s_restartRequested = 0u;
    s_restartRequestId = 0u;
    portEXIT_CRITICAL(&s_recoveryRequestMux);
    if (outRequestId) {
        *outRequestId = requestId;
    }
    return pending;
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
    uint32_t freeHeap = esp_get_free_heap_size();
    uint32_t minFreeHeap = esp_get_minimum_free_heap_size();
    if (freeHeap >= 49152u && minFreeHeap >= 32768u) {
        return;
    }
    printf("APP_MEM_WARN,stage=%s,ctx=%p,ctxSize=%u,stackHighWaterBytes=%lu,freeHeap=%u,minFreeHeap=%u\n",
           stage ? stage : SENSORARRAY_NA,
           (const void *)ctx,
           (unsigned)sizeof(sensorarrayAppContext_t),
           (unsigned long)((uint64_t)uxTaskGetStackHighWaterMark(NULL) *
                           sizeof(StackType_t)),
           (unsigned)freeHeap,
           (unsigned)minFreeHeap);
}

static void sensorarrayLogRuntimeMemorySummary(const sensorarrayAppContext_t *ctx,
                                               uint32_t freeBefore,
                                               uint32_t minFreeBefore)
{
    uint32_t freeAfter = esp_get_free_heap_size();
    uint32_t minFreeAfter = esp_get_minimum_free_heap_size();
    uint32_t minFreeHeap = minFreeBefore < minFreeAfter ? minFreeBefore : minFreeAfter;
    printf("APP_MEM,stage=runtime,ctx=%p,ctxSize=%u,stackHighWaterBytes=%lu,freeBefore=%u,freeAfter=%u,minFreeHeap=%u,delta=%ld\n",
           (const void *)ctx,
           (unsigned)sizeof(sensorarrayAppContext_t),
           (unsigned long)((uint64_t)uxTaskGetStackHighWaterMark(NULL) *
                           sizeof(StackType_t)),
           (unsigned)freeBefore,
           (unsigned)freeAfter,
           (unsigned)minFreeHeap,
           (long)freeAfter - (long)freeBefore);
}

static void sensorarrayLogStackHighWater(const char *stage)
{
    printf("APP_STACK,stage=%s,freeBytes=%lu\n",
           stage ? stage : SENSORARRAY_NA,
           (unsigned long)((uint64_t)uxTaskGetStackHighWaterMark(NULL) *
                           sizeof(StackType_t)));
}

static uint32_t sensorarrayFrameTargetFps(sensorarrayMeasurementMode_t mode)
{
    (void)mode;
    /* Only the explicit runtime capture cap is allowed to pace Core 1.
     * VOLT/RES target values are performance budgets, never hidden sleeps. */
    return sensorarrayCommandMailboxGetCaptureFpsCap();
}

static uint32_t sensorarrayFrameBudgetFps(sensorarrayMeasurementMode_t mode)
{
    if (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
        return (uint32_t)CONFIG_SENSORARRAY_ADS_VOLT_TARGET_FPS;
    }
    return mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ?
        (uint32_t)CONFIG_SENSORARRAY_ADS_RES_TARGET_FPS :
        (uint32_t)CONFIG_SENSORARRAY_TARGET_FPS;
}

static uint32_t sensorarrayFramePeriodMs(sensorarrayMeasurementMode_t mode)
{
    uint32_t modeFps = sensorarrayFrameTargetFps(mode);
    if (modeFps == 0u) {
        return 0u;
    }
    uint32_t periodMs = (1000u + modeFps - 1u) / modeFps;
    return periodMs == 0u ? 1u : periodMs;
}

static uint32_t sensorarrayFdcReferenceFramePeriodMs(void)
{
    uint32_t periodMs = (uint32_t)((SENSORARRAY_CFG_FRAME_PERIOD_US + 999u) / 1000u);
    return periodMs == 0u ? 1u : periodMs;
}

static void sensorarrayDelayFramePeriodSince(sensorarrayAppContext_t *ctx,
                                             int64_t frameStartUs,
                                             uint32_t sequence,
                                             sensorarrayMeasurementMode_t mode)
{
    uint32_t periodMs = sensorarrayFramePeriodMs(mode);
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

    /* The pre-frontend default is passive: remove matrix REF first, then park
     * both TMUX1134 banks on the FDC/Cx side. FDC converters are not running
     * yet, so this avoids energising the resistive network during boot. */
    esp_err_t tmuxErr = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmuxSwitchSelectRow(0);
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = sensorarrayMeasureSetSelaPath(state,
                                                SENSORARRAY_SELA_ROUTE_FDC2214,
                                                SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                "init_safe",
                                                "tmux_defaults");
    }
    if (tmuxErr == ESP_OK) {
        bool fdcSelBLevel = true;
        (void)sensorarrayBoardMapFdcSelBLevel(&fdcSelBLevel);
        tmuxErr = tmux1134SelectSelBLevel(fdcSelBLevel);
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
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t freeBefore = esp_get_free_heap_size();
    uint32_t minFreeBefore = esp_get_minimum_free_heap_size();

    sensorarrayLogDbgExtraReset();
    if (ctx->mixedSegmentWorkspace != NULL) {
        heap_caps_free(ctx->mixedSegmentWorkspace);
        ctx->mixedSegmentWorkspace = NULL;
    }
    *ctx = (sensorarrayAppContext_t){0};
    /* The runtime reset owns the whole context, so restore the acquisition
     * task handle immediately afterwards for immutable status snapshots. */
    ctx->scanTaskHandle = xTaskGetCurrentTaskHandle();
    sensorarrayLogRuntimeMemorySummary(ctx, freeBefore, minFreeBefore);
    /* Mixed profiles would exceed the bounded acquisition stack if the full
     * per-group segment frame remained an automatic in the scan task.  Hold
     * one reusable CPU-only workspace instead, preferring PSRAM so internal
     * RAM stays reserved for tasks/stacks.  A missing workspace is an
     * explicit init failure and never silently falls back to a stack frame. */
    ctx->mixedSegmentWorkspace = heap_caps_aligned_alloc(
        8u,
        sizeof(sensorarrayFrame_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    bool mixedWorkspacePsram = ctx->mixedSegmentWorkspace != NULL;
    if (!mixedWorkspacePsram) {
        ctx->mixedSegmentWorkspace = heap_caps_aligned_alloc(
            8u, sizeof(sensorarrayFrame_t), MALLOC_CAP_8BIT);
    }
    if (ctx->mixedSegmentWorkspace == NULL) {
        printf("APP_MEM,stage=mixed_workspace,action=alloc_failed,size=%u,psramFree=%u,internalFree=%u,internalMin=%u,err=0x%lx\n",
               (unsigned)sizeof(sensorarrayFrame_t),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
               (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
               (unsigned long)ESP_ERR_NO_MEM);
        return ESP_ERR_NO_MEM;
    }
    printf("APP_MEM,stage=mixed_workspace,action=allocated,ptr=%p,size=%u,heap=%s,psramFree=%u,internalFree=%u\n",
           (const void *)ctx->mixedSegmentWorkspace,
           (unsigned)sizeof(sensorarrayFrame_t),
           mixedWorkspacePsram ? "psram" : "internal",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    sensorarrayLogSetAdsState(false, false);
    sensorarrayFastSpeedSetEnabled(false);
    ctx->runtimeMode = SENSORARRAY_RUNTIME_MODE_FDC_MATRIX;
    sensorarrayMeasurementModeInit(&ctx->measurementMode);
    sensorarrayMeasurementRecoveryInit(&ctx->adsRestoreRecovery);
    sensorarrayRowModeProfileInit(
        &ctx->rowModeProfile, SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE);
    sensorarrayMeasurementSelfTestResult_t selfTest = {0};
    if (!sensorarrayMeasurementSelfTestRun(&selfTest)) {
        printf("MSELF,passed=0,checks=%lu,line=%lu,action=safe_stop\n",
               (unsigned long)selfTest.checks,
               (unsigned long)selfTest.failureLine);
        return ESP_FAIL;
    }
    printf("MSELF,passed=1,checks=%lu\n", (unsigned long)selfTest.checks);
    uint32_t protocolChecks = 0u;
    if (!sensorarrayTextProtocolSelfTest(&ctx->frame, &protocolChecks)) {
        printf("PSELF,passed=0,checks=%lu,action=safe_stop\n",
               (unsigned long)protocolChecks);
        return ESP_FAIL;
    }
    printf("PSELF,passed=1,checks=%lu\n", (unsigned long)protocolChecks);

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

static bool sensorarrayBuildAdsRailSplit(sensorarrayAppContext_t *ctx,
                                         sensorarrayAdsRailSplit_t *outRail)
{
    if (!ctx || !outRail) {
        return false;
    }
    uint32_t nextSequence = ctx->hostFrameSequence + 1u;
    if (sensorarrayAdsGapCopyRailSplit(
            nextSequence,
            (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES,
            outRail)) {
        return true;
    }
    /* Mode application must not transiently release the clamp merely to
     * obtain a rail value. RES may measure it after its route owns the matrix;
     * VOLT requires an explicit fresh calibration before acceptance. */
    return false;
}

static esp_err_t sensorarrayRefreshVoltageRailIfNeeded(
    sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    if (!sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        mode.activeMode != SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ||
        mode.state == SENSORARRAY_MEASUREMENT_STATE_TRANSITION) {
        return ESP_OK;
    }

    sensorarrayAdsRailSplit_t rail = {0};
    if (sensorarrayBuildAdsRailSplit(ctx, &rail)) {
        return ESP_OK;
    }

    /* VOLT cannot refresh the rail from inside a live ADS frame: the monitor
     * shares REF/REFOUT with the matrix and must first enter the isolated
     * SAFE_RAIL_MONITOR route.  Re-apply the existing VOLT route at this
     * frame boundary without creating a user-visible mode transition (and
     * therefore without changing the MODE generation/request correlation). */
    uint64_t monitorTransitionUs = 0u;
    esp_err_t err = sensorarrayRouteControllerEnterSafeRailMonitor(
        &ctx->routeController, &monitorTransitionUs);
    if (err == ESP_OK) {
        err = sensorarrayAdsGapRefreshRailAtBoundary(
            &ctx->state, ctx->hostFrameSequence + 1u);
    }
    if (err == ESP_OK) {
        err = sensorarrayBuildAdsRailSplit(ctx, &rail) ?
            ESP_OK : ESP_ERR_INVALID_STATE;
    }
    uint64_t routeTransitionUs = 0u;
    if (err == ESP_OK) {
        err = sensorarrayRouteControllerApplyMode(
            &ctx->routeController,
            SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
            &rail,
            &routeTransitionUs);
    }
    if (err == ESP_OK) {
        err = sensorarrayAdsMatrixEngineSetMode(
            &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_VOLTAGE);
    }
    printf("RAIL_REFRESH,mode=VOLT,seq=%lu,err=0x%lx,monitorUs=%llu,routeUs=%llu\n",
           (unsigned long)(ctx->hostFrameSequence + 1u),
           (unsigned long)err,
           (unsigned long long)monitorTransitionUs,
           (unsigned long long)routeTransitionUs);
    return err;
}

static void sensorarrayPublishLastMeasurement(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }
    __atomic_add_fetch(&ctx->lastMeasurementVersion, 1u, __ATOMIC_RELEASE);
    ctx->lastMeasurement.sequence = ctx->frame.sequence;
    ctx->lastMeasurement.activeRows = ctx->frame.activeRows;
    ctx->lastMeasurement.payload = ctx->frame.measurement;
    __atomic_add_fetch(&ctx->lastMeasurementVersion, 1u, __ATOMIC_RELEASE);
}

static bool sensorarrayCopyLastMeasurementCell(
    const sensorarrayAppContext_t *ctx,
    uint8_t row,
    uint8_t dLine,
    sensorarrayMeasurementCellSnapshot_t *outSnapshot)
{
    if (!ctx || !outSnapshot || !sensorarrayMatrixIndexIsValid(row, dLine)) {
        return false;
    }
    size_t index = sensorarrayMatrixIndex(row, dLine);
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt) {
        uint32_t before = __atomic_load_n(&ctx->lastMeasurementVersion,
                                          __ATOMIC_ACQUIRE);
        if ((before & 1u) != 0u) {
            continue;
        }
        /* Copy only common metadata and the requested cell. This keeps the
         * Core 0 query stack bounded while the version check guarantees one
         * coherent Core 1 frame. */
        memset(outSnapshot, 0, sizeof(*outSnapshot));
        outSnapshot->sequence = ctx->lastMeasurement.sequence;
        outSnapshot->mode = ctx->lastMeasurement.payload.mode;
        outSnapshot->unit = ctx->lastMeasurement.payload.unit;
        outSnapshot->decimalScale = ctx->lastMeasurement.payload.decimalScale;
        outSnapshot->referenceSource =
            ctx->lastMeasurement.payload.referenceSource;
        outSnapshot->valueFixed =
            ctx->lastMeasurement.payload.valuesFixed[index];
        outSnapshot->rawCode = ctx->lastMeasurement.payload.rawCode[index];
        outSnapshot->nodeUv = ctx->lastMeasurement.payload.nodeUv[index];
        outSnapshot->pgaGain = ctx->lastMeasurement.payload.pgaGain[index];
        outSnapshot->errorReason =
            ctx->lastMeasurement.payload.errorReason[index];
        uint64_t bit = UINT64_C(1) << index;
        outSnapshot->valid =
            (ctx->lastMeasurement.payload.validMask & bit) != 0u;
        outSnapshot->fresh =
            (ctx->lastMeasurement.payload.freshMask & bit) != 0u;
        outSnapshot->avssUv = ctx->lastMeasurement.payload.avssUv;
        outSnapshot->matrixReferenceUv =
            ctx->lastMeasurement.payload.matrixReferenceUv;
        outSnapshot->referenceResistorOhms =
            ctx->lastMeasurement.payload.referenceResistorOhms;
        outSnapshot->railAgeFrames =
            ctx->lastMeasurement.payload.railAgeFrames;
        outSnapshot->railValid = ctx->lastMeasurement.payload.railValid;
        uint32_t after = __atomic_load_n(&ctx->lastMeasurementVersion,
                                         __ATOMIC_ACQUIRE);
        if (before == after && (after & 1u) == 0u) {
            return true;
        }
    }
    return false;
}

static esp_err_t sensorarrayApplyMeasurementMode(sensorarrayAppContext_t *ctx,
                                                 sensorarrayMeasurementMode_t targetMode,
                                                 uint32_t requestId,
                                                 bool emitEvent)
{
    if (!ctx || !sensorarrayMeasurementModeIsDataMode(targetMode)) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayMeasurementModeSnapshot_t before = {0};
    (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &before);
    if (!sensorarrayMeasurementModeAccept(&ctx->measurementMode, targetMode, requestId) ||
        !sensorarrayMeasurementModeBeginTransition(&ctx->measurementMode)) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayAdsRailSplit_t rail = {0};
    const sensorarrayAdsRailSplit_t *railPtr = NULL;
    esp_err_t err = ESP_OK;
    bool railAvailable = sensorarrayBuildAdsRailSplit(ctx, &rail);
    bool externalRail = sensorarrayAdsGapHasExternalRailCalibration();
    if (targetMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE &&
        (!railAvailable || !externalRail)) {
        /* VOLT no longer depends on a host RAILCFG. The monitor is measured in
         * a controller-owned, matrix-isolated SAFE_RAIL_MONITOR window and
         * the requested target route is restored before the transaction ends. */
        uint64_t railTransitionUs = 0u;
        err = sensorarrayRouteControllerEnterSafeRailMonitor(
            &ctx->routeController, &railTransitionUs);
        if (err == ESP_OK) {
            err = sensorarrayAdsGapRefreshRailAtBoundary(
                &ctx->state, ctx->hostFrameSequence + 1u);
        }
        if (err == ESP_OK) {
            railAvailable = sensorarrayBuildAdsRailSplit(ctx, &rail);
            externalRail = sensorarrayAdsGapHasExternalRailCalibration();
        }
        if (err == ESP_OK && (!railAvailable || (!externalRail && !rail.valid))) {
            err = ESP_ERR_INVALID_STATE;
        }
        if (err == ESP_OK && railAvailable) {
            railPtr = &rail;
        }
        if (err != ESP_OK) {
            (void)railTransitionUs;
        }
    } else if (railAvailable) {
        railPtr = &rail;
    }
    uint64_t transitionUs = 0u;
    if (err == ESP_OK) {
        err = sensorarrayRouteControllerApplyMode(&ctx->routeController,
                                                  targetMode,
                                                  railPtr,
                                                  &transitionUs);
    }
    uint32_t appliedSequence = ctx->hostFrameSequence + 1u;
    if (err == ESP_OK) {
        sensorarrayMeasurementMode_t adsMode =
            targetMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
                SENSORARRAY_MEASUREMENT_MODE_NONE : targetMode;
        err = sensorarrayAdsMatrixEngineSetMode(&ctx->adsEngine, adsMode);
    }
    if (err == ESP_OK) {
        bool changed = before.activeMode != targetMode;
        if (changed) {
            memset(&ctx->frame, 0, sizeof(ctx->frame));
            ctx->state.fdcAppliedRow[SENSORARRAY_FDC_DEV_PRIMARY].valid = false;
            ctx->state.fdcAppliedRow[SENSORARRAY_FDC_DEV_SECONDARY].valid = false;
            sensorarrayAdsMatrixEngineInvalidateGainCache(&ctx->adsEngine);
            sensorarrayFdcRescueReset(&ctx->fdcRescue);
        }
        if (!sensorarrayMeasurementModeCompleteTransition(&ctx->measurementMode,
                                                          appliedSequence,
                                                          transitionUs)) {
            err = ESP_ERR_INVALID_STATE;
        }
    }
    if (err != ESP_OK) {
        (void)sensorarrayAdsMatrixEngineSetMode(&ctx->adsEngine,
                                               SENSORARRAY_MEASUREMENT_MODE_NONE);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                  "mode_apply_failed");
        sensorarrayMeasurementModeFailTransition(&ctx->measurementMode,
                                                 (uint32_t)err,
                                                 transitionUs);
        if (emitEvent) {
            printf("MERR,id=%lu,old=%s,new=%s,seq=%lu,state=SAFE,err=0x%lx,transitionUs=%llu\n",
                   (unsigned long)requestId,
                   sensorarrayMeasurementModeName(before.activeMode),
                   sensorarrayMeasurementModeName(targetMode),
                   (unsigned long)appliedSequence,
                   (unsigned long)err,
                   (unsigned long long)transitionUs);
        }
        return err;
    }

    sensorarrayMeasurementModeSnapshot_t after = {0};
    (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &after);
    ctx->runtimeMode = targetMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
        SENSORARRAY_RUNTIME_MODE_FDC_MATRIX : SENSORARRAY_RUNTIME_MODE_ADS_MATRIX;
    ctx->modeTransitionCount++;
    if (emitEvent) {
        printf("MAPP,id=%lu,gen=%lu,old=%s,new=%s,seq=%lu,state=applied,transitionUs=%llu\n",
               (unsigned long)requestId,
               (unsigned long)after.generation,
               sensorarrayMeasurementModeName(before.activeMode),
               sensorarrayMeasurementModeName(targetMode),
               (unsigned long)appliedSequence,
               (unsigned long long)transitionUs);
    }
    return ESP_OK;
}

static void sensorarrayEmitRowModesTerminal(uint32_t requestId,
                                            const char *format,
                                            ...)
{
    char text[SENSORARRAY_ROW_MODES_TERMINAL_TEXT_MAX];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    if (written <= 0 || (size_t)written >= sizeof(text)) {
        printf("RMTERM_FMT,id=%lu,state=truncated\n",
               (unsigned long)requestId);
        /* Keep the accepted transaction terminable: replace an unprintable
         * terminal with a short diagnostic one instead of stranding the
         * reservation and guaranteed-text lane forever. */
        char fallback[SENSORARRAY_ROW_MODES_TERMINAL_TEXT_MAX];
        int fallbackLength = snprintf(
            fallback, sizeof(fallback),
            "RMERR,id=%lu,state=rejected,reason=terminal_too_long\n",
            (unsigned long)requestId);
        if (fallbackLength <= 0 || (size_t)fallbackLength >= sizeof(fallback)) {
            (void)sensorarrayCommandMailboxCancelRowModesAck(requestId);
            printf("RMTERM_FMT_FATAL,id=%lu,state=cancelled\n",
                   (unsigned long)requestId);
            return;
        }
        esp_err_t emitErr = sensorarrayCommandMailboxEmitRowModesTerminal(
            requestId, fallback, (size_t)fallbackLength);
        if (emitErr != ESP_OK) {
            printf("RMTERM_DROP,id=%lu,err=0x%lx,state=not_emitted\n",
                   (unsigned long)requestId,
                   (unsigned long)emitErr);
        }
        return;
    }
    esp_err_t emitErr = sensorarrayCommandMailboxEmitRowModesTerminal(
        requestId, text, (size_t)written);
    if (emitErr != ESP_OK) {
        printf("RMTERM_DROP,id=%lu,err=0x%lx,state=not_emitted\n",
               (unsigned long)requestId,
               (unsigned long)emitErr);
    }
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
        case SENSORARRAY_COMMAND_CALIBRATE_SAVE: {
            sensorarrayAdsMatrixCalibration_t calibration = {0};
            sensorarrayCalibrationStatus_t status = {0};
            if (!sensorarrayAdsMatrixEngineGetCalibration(&ctx->adsEngine,
                                                          &calibration)) {
                printf("CALSV,state=rejected,reason=engine_invalid\n");
                continue;
            }
            esp_err_t saveErr = sensorarrayCalibrationSave(
                &calibration,
                sizeof(calibration),
                &status);
            printf("CALSV,state=%s,reason=%s,source=%lu,schema=%lu,valid=%u,boardId=%08lX,hardwareRev=%lu,payloadLength=%lu,err=0x%lx\n",
                   saveErr == ESP_OK ? "saved" : "rejected",
                   saveErr == ESP_OK ? "persisted" : "nvs_error",
                   (unsigned long)status.source,
                   (unsigned long)status.schemaVersion,
                   status.valid ? 1u : 0u,
                   (unsigned long)status.boardId,
                   (unsigned long)status.hardwareRevision,
                   (unsigned long)status.payloadLength,
                   (unsigned long)saveErr);
            continue;
        }
        case SENSORARRAY_COMMAND_CALIBRATE_LOAD: {
            if (ctx->adsEngine.mode != SENSORARRAY_MEASUREMENT_MODE_NONE) {
                printf("CALLD,state=rejected,reason=not_safe\n");
                continue;
            }
            sensorarrayAdsMatrixCalibration_t calibration = {0};
            sensorarrayCalibrationStatus_t status = {0};
            esp_err_t loadErr = sensorarrayCalibrationLoad(
                sensorarrayCalibrationMatrixPayloadValid,
                &calibration,
                sizeof(calibration),
                NULL,
                &status);
            const char *state = "default";
            const char *reason = sensorarrayCalibrationLoadReason(&status,
                                                                  loadErr);
            if (loadErr == ESP_OK) {
                esp_err_t applyErr = sensorarrayAdsMatrixEngineSetCalibration(
                    &ctx->adsEngine,
                    &calibration);
                if (applyErr == ESP_OK) {
                    state = "applied";
                } else {
                    reason = "apply_failed";
                    loadErr = applyErr;
                }
            }
            sensorarrayCalibrationLogOutcome(
                "CALLD",
                state,
                reason,
                &status,
                loadErr);
            continue;
        }
        case SENSORARRAY_COMMAND_ADS_GAP_MODE:
            sensorarrayAdsGapSetMode((sensorarrayAdsGapMode_t)command.value);
            break;
        case SENSORARRAY_COMMAND_BLE_CAP_PERIOD:
        case SENSORARRAY_COMMAND_TRACE_ENABLE:
        case SENSORARRAY_COMMAND_CAPTURE_FPS_CAP:
        case SENSORARRAY_COMMAND_OUTPUT_FPS_CAP:
        case SENSORARRAY_COMMAND_ADS_DEBUG:
            break;
        case SENSORARRAY_COMMAND_ADS_CHECK:
            ctx->pendingAdsCheck = true;
            ctx->pendingAdsCheckRequestId = command.requestId;
            ctx->pendingAdsCheckSamples = command.value;
            break;
        case SENSORARRAY_COMMAND_BATTERY_NOW:
            sensorarrayAdsGapRequestBatteryNow();
            ctx->pendingBatteryRequestId = command.requestId;
            ctx->pendingBatteryDiagnostic = false;
            break;
        case SENSORARRAY_COMMAND_BATTERY_DIAGNOSTIC:
            sensorarrayAdsGapRequestBatteryDiagnostic();
            ctx->pendingBatteryRequestId = command.requestId;
            ctx->pendingBatteryDiagnostic = true;
            break;
        case SENSORARRAY_COMMAND_BATTERY_PERIOD: {
            bool enabled = command.signedValue != 0;
            bool applied = sensorarrayAdsGapConfigureBatteryPeriod(
                enabled, command.value);
            printf("BATPERIOD,id=%lu,enabled=%u,periodMs=%lu,status=%s\n",
                   (unsigned long)command.requestId,
                   enabled ? 1u : 0u,
                   (unsigned long)command.value,
                   applied ? "applied" : "rejected");
            break;
        }
        case SENSORARRAY_COMMAND_RES_SETTLE:
            printf("RESSETTLE,id=%lu,settleUs=%lu,status=%s\n",
                   (unsigned long)command.requestId,
                   (unsigned long)command.value,
                   sensorarrayRouteControllerSetRowSettleUs(
                       &ctx->routeController, command.value) ?
                       "applied" : "rejected");
            break;
        case SENSORARRAY_COMMAND_MEASUREMENT_MODE:
        {
            sensorarrayMeasurementMode_t requested =
                (sensorarrayMeasurementMode_t)command.value;
            sensorarrayRowModeProfile_t rowSnapshot = {0};
            if (sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &rowSnapshot) &&
                rowSnapshot.pending) {
                sensorarrayMeasurementModeSnapshot_t beforeMode = {0};
                (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode,
                                                             &beforeMode);
                printf("MERR,id=%lu,old=%s,new=%s,seq=%lu,state=rejected,err=0x%lx,transitionUs=0\n",
                       (unsigned long)command.requestId,
                       sensorarrayMeasurementModeName(beforeMode.activeMode),
                       sensorarrayMeasurementModeName(requested),
                       (unsigned long)(ctx->hostFrameSequence + 1u),
                       (unsigned long)ESP_ERR_INVALID_STATE);
                continue;
            }
            sensorarrayRouteSnapshot_t routeSnapshot = {0};
            if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController,
                                                       &routeSnapshot) &&
                routeSnapshot.fdcSdHigh) {
                sensorarrayMeasurementModeSnapshot_t beforeMode = {0};
                (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode,
                                                             &beforeMode);
                printf("MERR,id=%lu,old=%s,new=%s,seq=%lu,state=rejected,err=0x%lx,reason=fdc_restart_required,transitionUs=0\n",
                       (unsigned long)command.requestId,
                       sensorarrayMeasurementModeName(beforeMode.activeMode),
                       sensorarrayMeasurementModeName(requested),
                       (unsigned long)(ctx->hostFrameSequence + 1u),
                       (unsigned long)ESP_ERR_INVALID_STATE);
                continue;
            }
            esp_err_t modeErr = sensorarrayApplyMeasurementMode(
                ctx,
                requested,
                command.requestId,
                true);
            if (modeErr == ESP_OK) {
                sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
                for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
                    modes[row] = requested;
                }
                if (sensorarrayRowModeProfileAccept(&ctx->rowModeProfile,
                                                    modes,
                                                    command.requestId)) {
                    (void)sensorarrayRowModeProfileCompleteTransition(
                        &ctx->rowModeProfile,
                        ctx->hostFrameSequence + 1u,
                        0u);
                }
            }
            continue;
        }
        case SENSORARRAY_COMMAND_ROW_MODES: {
            sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
            if (!sensorarrayRowModeProfileParse(command.rowModes,
                                                SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH,
                                                modes)) {
                sensorarrayEmitRowModesTerminal(
                    command.requestId,
                    "RMERR,id=%lu,profile=%s,state=rejected,err=0x%lx,reason=profile\n",
                    (unsigned long)command.requestId,
                    command.rowModes,
                    (unsigned long)ESP_ERR_INVALID_ARG);
                break;
            }
            sensorarrayRouteSnapshot_t routeSnapshot = {0};
            if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController,
                                                       &routeSnapshot) &&
                routeSnapshot.fdcSdHigh) {
                sensorarrayEmitRowModesTerminal(
                    command.requestId,
                    "RMERR,id=%lu,profile=%s,state=rejected,err=0x%lx,reason=fdc_restart_required\n",
                    (unsigned long)command.requestId,
                    command.rowModes,
                    (unsigned long)ESP_ERR_INVALID_STATE);
                break;
            }
            if (!sensorarrayRowModeProfileAccept(&ctx->rowModeProfile,
                                                 modes,
                                                 command.requestId)) {
                sensorarrayEmitRowModesTerminal(
                    command.requestId,
                    "RMERR,id=%lu,profile=%s,state=rejected,err=0x%lx,reason=pending\n",
                    (unsigned long)command.requestId,
                    command.rowModes,
                    (unsigned long)ESP_ERR_INVALID_STATE);
            }
            break;
        }
        case SENSORARRAY_COMMAND_FDC_ISOLATE: {
            uint32_t appliedSequence = ctx->hostFrameSequence + 1u;
            if (command.value == 0u) {
                printf("FERR,id=%lu,seq=%lu,sd=low,state=rejected,reason=restart_required,restartRequired=1\n",
                       (unsigned long)command.requestId,
                       (unsigned long)appliedSequence);
                continue;
            }
            sensorarrayMeasurementModeSnapshot_t mode = {0};
            sensorarrayRowModeProfile_t profile = {0};
            sensorarrayRouteSnapshot_t route = {0};
            bool modeOk = sensorarrayMeasurementModeCopySnapshot(
                &ctx->measurementMode, &mode);
            bool profileOk = sensorarrayRowModeProfileCopy(
                &ctx->rowModeProfile, &profile);
            bool routeOk = sensorarrayRouteControllerCopySnapshot(
                &ctx->routeController, &route);
            bool routeConsistent =
                routeOk && !route.safe && route.mode == mode.activeMode;
            bool homogeneousAds =
                mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ||
                mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
            if (!modeOk || !profileOk || !routeConsistent || !homogeneousAds ||
                mode.pending ||
                mode.state == SENSORARRAY_MEASUREMENT_STATE_TRANSITION ||
                profile.pending ||
                !sensorarrayRowModeProfileIsHomogeneous(profile.modes)) {
                printf("FERR,id=%lu,seq=%lu,active=%s,state=rejected,reason=not_homogeneous_ads\n",
                       (unsigned long)command.requestId,
                       (unsigned long)appliedSequence,
                       sensorarrayMeasurementModeName(mode.activeMode));
                continue;
            }
            if (route.fdcSdHigh) {
                printf("FERR,id=%lu,seq=%lu,sd=high,state=rejected,reason=already_applied,restartRequired=1\n",
                       (unsigned long)command.requestId,
                       (unsigned long)appliedSequence);
                continue;
            }
            bool verified = false;
            esp_err_t err = sensorarrayRouteControllerForceFdcShutdown(
                &ctx->routeController, &verified);
            if (err == ESP_OK && verified) {
                printf("FAPP,id=%lu,seq=%lu,sd=high,verified=1,restartRequired=1,state=applied\n",
                       (unsigned long)command.requestId,
                       (unsigned long)appliedSequence);
            } else {
                const char *reason =
                    err == ESP_ERR_NOT_SUPPORTED ? "sd_gpio_unavailable" :
                    err == ESP_ERR_INVALID_RESPONSE ? "readback_failed" :
                    "sleep_verify_failed";
                printf("FERR,id=%lu,seq=%lu,sd=low,err=0x%lx,state=rejected,reason=%s\n",
                       (unsigned long)command.requestId,
                       (unsigned long)appliedSequence,
                       (unsigned long)err,
                       reason);
                continue;
            }
            break;
        }
        case SENSORARRAY_COMMAND_SET_RAIL_CALIBRATION: {
            uint32_t appliedSequence = ctx->hostFrameSequence + 1u;
            esp_err_t railErr = sensorarrayAdsGapSetExternalRailCalibration(
                command.signedValue,
                command.signedValue2,
                appliedSequence);
            if (railErr == ESP_OK) {
                ctx->railCalibrationGeneration++;
                sensorarrayAdsMatrixEngineInvalidateGainCache(&ctx->adsEngine);
                printf("RAPP,id=%lu,gen=%lu,seq=%lu,avdd=%ld,avss=%ld,source=external,state=applied\n",
                       (unsigned long)command.requestId,
                       (unsigned long)ctx->railCalibrationGeneration,
                       (unsigned long)appliedSequence,
                       (long)command.signedValue,
                       (long)command.signedValue2);
            } else {
                printf("RERR,id=%lu,seq=%lu,avdd=%ld,avss=%ld,err=0x%lx,state=rejected\n",
                       (unsigned long)command.requestId,
                       (unsigned long)appliedSequence,
                       (long)command.signedValue,
                       (long)command.signedValue2,
                       (unsigned long)railErr);
            }
            continue;
        }
        default:
            continue;
        }

        sensorarrayCommandMailboxCommit(&command);
        if (ctx->asyncLogReady) {
            (void)sensorarrayAsyncLogPublishCommandApplied(ctx->frame.sequence, &command);
        }
        if (command.type == SENSORARRAY_COMMAND_ADS_CHECK) {
            /* Preserve mailbox ordering and execute at most one intrusive ADS
             * transaction after each complete frame. Remaining requests stay
             * queued until the following frame boundary. */
            return;
        }
    }
}

static void sensorarrayRunPendingAdsCheck(sensorarrayAppContext_t *ctx)
{
    if (!ctx || !ctx->pendingAdsCheck) {
        return;
    }

    uint32_t requestId = ctx->pendingAdsCheckRequestId;
    uint32_t sampleCount = ctx->pendingAdsCheckSamples;
    ctx->pendingAdsCheck = false;
    ctx->pendingAdsCheckRequestId = 0u;
    ctx->pendingAdsCheckSamples = 0u;

    sensorarrayAdsActiveCheckResult_t result = {0};
    esp_err_t checkErr = sensorarrayAdsGapRunActiveCheck(
        &ctx->state,
        sensorarrayAdsMatrixEngineRegisterCache(&ctx->adsEngine),
        requestId,
        sampleCount,
        &result);
    ctx->frame.measurement.adsCheckUs = result.durationUs;

    char line[384];
    if (sensorarrayAdsGapFormatActiveCheck(&result, line, sizeof(line)) != 0u) {
        printf("%s", line);
    }
    if (sensorarrayAdsGapFormatActiveCheckTiming(&result, line, sizeof(line)) != 0u) {
        printf("%s", line);
    }

    if (!result.restoreOk) {
        if (sensorarrayBeginAdsRestoreRecovery(
                ctx,
                SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_ADS_RESTORE,
                checkErr,
                requestId)) {
            return;
        }
        sensorarrayAdsMatrixEngineInvalidateCaches(&ctx->adsEngine);
        (void)sensorarrayAdsMatrixEngineSetMode(
            &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
        sensorarrayMeasurementModeRecordRuntimeFault(
            &ctx->measurementMode, (uint32_t)checkErr);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                   "ads_check_restore_failed");
        sensorarrayAdsFaultEvent_t fault = {
            .stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK,
            .err = checkErr,
            .seq = ctx->frame.sequence,
            .outcome = SENSORARRAY_ADS_FAULT_OUTCOME_FAILED,
        };
        sensorarrayAdsFaultFillContext(ctx, &fault);
        fault.rowRequestId = requestId;
        fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_CHECK);
        fault.restoreExpectedValid = true;
        fault.restoreExpected = (int32_t)result.registers.power;
        sensorarrayEmitAdsFault(ctx, &fault);
        printf("MFAULT,source=ADSCHK,id=%lu,err=0x%lx,state=DEGRADED,route=SAFE\n",
               (unsigned long)requestId,
               (unsigned long)checkErr);
    }
}

static void sensorarrayRunBatteryBoundary(sensorarrayAppContext_t *ctx,
                                          sensorarrayMeasurementMode_t frameMode)
{
    if (!ctx) {
        return;
    }
    uint32_t batteryRequestId = ctx->pendingBatteryRequestId;
    uint32_t durationUs = 0u;
    bool ran = false;
    esp_err_t batteryErr = sensorarrayAdsGapRunBatteryAtBoundary(
        &ctx->state,
        ctx->frame.sequence,
        frameMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        &durationUs,
        &ran);
    ctx->frame.measurement.batteryUs = durationUs;
    if (ran && ctx->pendingBatteryRequestId != 0u) {
        printf("BAPP,id=%lu,cmd=%s,seq=%lu,err=0x%lx,durationUs=%lu,status=complete\n",
               (unsigned long)ctx->pendingBatteryRequestId,
               ctx->pendingBatteryDiagnostic ? "BATD" : "BATNOW",
               (unsigned long)ctx->frame.sequence,
               (unsigned long)batteryErr,
               (unsigned long)durationUs);
        ctx->pendingBatteryRequestId = 0u;
        ctx->pendingBatteryDiagnostic = false;
    }
    sensorarrayMeasurementRecoveryTrigger_t batteryRecoveryTrigger =
        sensorarrayMeasurementRecoveryTriggerForBattery(
            sensorarrayAdsGapConsumeRestoreFailure());
    if (batteryRecoveryTrigger != SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_NONE) {
        if (sensorarrayBeginAdsRestoreRecovery(
                ctx,
                batteryRecoveryTrigger,
                batteryErr,
                batteryRequestId)) {
            return;
        }
        sensorarrayAdsMatrixEngineInvalidateCaches(&ctx->adsEngine);
        (void)sensorarrayAdsMatrixEngineSetMode(
            &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
        sensorarrayMeasurementModeRecordRuntimeFault(
            &ctx->measurementMode, (uint32_t)batteryErr);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                   "battery_restore_failed");
        sensorarrayAdsFaultEvent_t fault = {
            .stage = SENSORARRAY_ADS_FAULT_STAGE_BATTERY_RESTORE,
            .err = batteryErr,
            .seq = ctx->frame.sequence,
            .outcome = SENSORARRAY_ADS_FAULT_OUTCOME_FAILED,
        };
        sensorarrayAdsFaultFillContext(ctx, &fault);
        fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_BATTERY);
        sensorarrayEmitAdsFault(ctx, &fault);
        printf("MFAULT,source=BATTERY,err=0x%lx,state=DEGRADED,route=SAFE\n",
               (unsigned long)batteryErr);
    }
}

static void sensorarrayFinishAdsRestoreRecovery(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }
    uint32_t error = ctx->adsRestoreRecovery.lastError;
    uint32_t attempt = ctx->adsRestoreRecovery.attempt;
    sensorarrayMeasurementModeRecordRuntimeFault(
        &ctx->measurementMode, error);
    (void)sensorarrayRouteControllerEnterSafe(
        &ctx->routeController, "ads_restore_recovery_failed");
    sensorarrayEmitAdsRecoveryFault(
        ctx,
        SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_FAILED,
        attempt,
        SENSORARRAY_ADS_FAULT_OUTCOME_FAILED,
        (esp_err_t)error,
        ctx->hostFrameSequence + 1u);
    if (ctx->adsRestoreRecovery.trigger ==
        SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_BATTERY_RESTORE) {
        printf("MFAULT,source=BATTERY,err=0x%lx,state=DEGRADED,route=SAFE,attempt=%lu\n",
               (unsigned long)error,
               (unsigned long)attempt);
    } else {
        printf("MFAULT,source=ADSCHK,id=%lu,err=0x%lx,state=DEGRADED,route=SAFE,attempt=%lu\n",
               (unsigned long)ctx->adsRestoreRecovery.triggerRequestId,
               (unsigned long)error,
               (unsigned long)attempt);
    }
}

static void sensorarrayRunAdsRestoreRecoveryAttempt(sensorarrayAppContext_t *ctx)
{
    if (!ctx ||
        !sensorarrayMeasurementRecoveryIsActive(&ctx->adsRestoreRecovery)) {
        return;
    }
    if (!sensorarrayMeasurementRecoveryBeginAttempt(&ctx->adsRestoreRecovery)) {
        sensorarrayFinishAdsRestoreRecovery(ctx);
        return;
    }
    uint32_t attempt = ctx->adsRestoreRecovery.attempt;
    uint32_t sequence = ctx->hostFrameSequence + 1u;

    memset(&ctx->frame, 0, sizeof(ctx->frame));
    ctx->state.fdcAppliedRow[SENSORARRAY_FDC_DEV_PRIMARY].valid = false;
    ctx->state.fdcAppliedRow[SENSORARRAY_FDC_DEV_SECONDARY].valid = false;
    sensorarrayAdsMatrixEngineInvalidateCaches(&ctx->adsEngine);
    sensorarrayFdcRescueReset(&ctx->fdcRescue);

    sensorarrayAdsRegisterCache_t *registerCache =
        sensorarrayAdsMatrixEngineRegisterCache(&ctx->adsEngine);
    uint32_t generationBefore = registerCache ? registerCache->generation : 0u;
    sensorarrayAdsActiveCheckResult_t check = {0};
    esp_err_t checkErr = sensorarrayAdsGapRunActiveCheck(
        &ctx->state,
        registerCache,
        ctx->adsRestoreRecovery.session,
        SENSORARRAY_ADS_RECOVERY_CHECK_SAMPLES,
        &check);
    uint32_t generationAfter = registerCache ? registerCache->generation : 0u;
    bool verified = check.ok && checkErr == ESP_OK &&
        generationAfter > generationBefore &&
        sensorarrayAdsRegisterCacheIsVerified(registerCache);
    esp_err_t attemptErr = verified ? ESP_OK :
        (checkErr != ESP_OK ? checkErr : ESP_ERR_INVALID_RESPONSE);

    if (verified) {
        esp_err_t resumeErr = sensorarrayApplyMeasurementMode(
            ctx,
            ctx->adsRestoreRecovery.resumeMode,
            ctx->adsRestoreRecovery.resumeRequestId,
            false);
        sensorarrayMeasurementRecoveryComplete(
            &ctx->adsRestoreRecovery,
            resumeErr == ESP_OK,
            (uint32_t)(resumeErr == ESP_OK ? ESP_OK : resumeErr));
        attemptErr = resumeErr;
        if (resumeErr == ESP_OK) {
            sensorarrayEmitAdsRecoveryFault(
                ctx,
                SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_RESUME,
                attempt,
                SENSORARRAY_ADS_FAULT_OUTCOME_RESUMED,
                ESP_OK,
                sequence);
            printf("MRECOVER,stage=resume,attempt=%lu,state=resumed,mode=%s,seq=%lu,err=0\n",
                   (unsigned long)attempt,
                   sensorarrayMeasurementModeName(
                       ctx->adsRestoreRecovery.resumeMode),
                   (unsigned long)sequence);
            return;
        }
    } else {
        sensorarrayMeasurementRecoveryComplete(
            &ctx->adsRestoreRecovery, false, (uint32_t)attemptErr);
    }

    if (sensorarrayMeasurementRecoveryIsTerminal(&ctx->adsRestoreRecovery)) {
        sensorarrayFinishAdsRestoreRecovery(ctx);
        return;
    }
    sensorarrayMeasurementModeEnterRecovery(
        &ctx->measurementMode, (uint32_t)attemptErr);
    sensorarrayEmitAdsRecoveryFault(
        ctx,
        SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_ATTEMPT,
        attempt,
        SENSORARRAY_ADS_FAULT_OUTCOME_ATTEMPT,
        attemptErr,
        sequence);
    vTaskDelay(pdMS_TO_TICKS(SENSORARRAY_ADS_RECOVERY_ATTEMPT_DELAY_MS));
}

static esp_err_t sensorarrayFormatBootReply(char *response,
                                            size_t responseSize,
                                            const sensorarrayAppContext_t *ctx)
{
    if (!response || responseSize == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayBootBreadcrumb_t boot = {0};
    if (!sensorarrayBootBreadcrumbCopy(&boot)) {
        snprintf(response, responseSize,
                 "ERR,cmd=BOOT,reason=breadcrumb_unavailable\n");
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t nowUs = esp_rtc_get_time_us();
    uint64_t windowAgeS = boot.autoRestartWindowStartUs != 0u &&
                          nowUs >= boot.autoRestartWindowStartUs ?
        (nowUs - boot.autoRestartWindowStartUs) / 1000000ULL : 0u;
    int written = snprintf(response,
             responseSize,
             "BOOT,boot=%lu,bootId=%lu,reset=%s,stage=%s,err=0x%lx,seq=%lu,heap=%lu,heapMin=%lu,prevStage=%s,prevErr=0x%lx,prevHeap=%lu,guard=%s,autoRestarts=%lu,windowAgeS=%llu,ready=%u\n",
             (unsigned long)boot.bootCount,
             (unsigned long)boot.bootId,
             sensorarrayResetReasonName((esp_reset_reason_t)boot.resetReason),
             boot.lastStage,
             (unsigned long)(uint32_t)boot.lastErr,
             (unsigned long)boot.lastFrameSeq,
             (unsigned long)esp_get_free_heap_size(),
             (unsigned long)boot.minFreeHeap,
             boot.prevStage[0] != '\0' ? boot.prevStage : "none",
             (unsigned long)(uint32_t)boot.prevErr,
             (unsigned long)boot.prevHeap,
             boot.guardState == SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE ?
                 "recovery_safe" : "normal",
             (unsigned long)boot.autoRestartCount,
             (unsigned long long)windowAgeS,
             ctx && ctx->systemReady ? 1u : 0u);
    if (written < 0 || (size_t)written >= responseSize) {
        (void)snprintf(response, responseSize,
                       "ERR,cmd=BOOT,reason=response_too_long\n");
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

static esp_err_t sensorarrayEarlyRuntimeQueryCommand(const char *command,
                                                     char *response,
                                                     size_t responseSize,
                                                     void *context)
{
    (void)context;
    if (!command || !response || responseSize == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t recoveryLevel = SENSORARRAY_RECOVERY_LEVEL_FULL;
    sensorarrayEarlyRecoveryKind_t kind =
        sensorarrayEarlyRecoveryParse(command, &recoveryLevel);
    switch (kind) {
    case SENSORARRAY_EARLY_KIND_BOOT_QUERY:
        return sensorarrayFormatBootReply(response, responseSize, NULL);
    case SENSORARRAY_EARLY_KIND_STATE_QUERY:
    case SENSORARRAY_EARLY_KIND_MODE_QUERY: {
        sensorarrayBootBreadcrumb_t boot = {0};
        (void)sensorarrayBootBreadcrumbCopy(&boot);
        snprintf(response,
                 responseSize,
                 "%s,ready=0,state=INIT,active=NONE,stage=%s,reason=acquisition_not_ready\n",
                 kind == SENSORARRAY_EARLY_KIND_STATE_QUERY ? "STATE" : "MODE",
                 boot.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC ?
                     boot.lastStage : "boot");
        return ESP_OK;
    }
    case SENSORARRAY_EARLY_KIND_READY_QUERY: {
        sensorarrayBootBreadcrumb_t boot = {0};
        (void)sensorarrayBootBreadcrumbCopy(&boot);
        snprintf(response,
                 responseSize,
                 "READY,ready=0,stage=%s,err=0x%lx,bootId=%lu,boot=%lu\n",
                 boot.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC ?
                     boot.lastStage : "unknown",
                 (unsigned long)(boot.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC ?
                     (uint32_t)boot.lastErr : 0u),
                 (unsigned long)boot.bootId,
                 (unsigned long)boot.bootCount);
        return ESP_OK;
    }
    case SENSORARRAY_EARLY_KIND_PROTO_QUERY:
        snprintf(response,
                 responseSize,
                 "PROTO,version=%u,wires=ascii,ctrlMax=%u,dataMax=%u,channels=CTRL/DATA/LOG/LIFECYCLE\n",
                 (unsigned)SENSORARRAY_PROTOCOL_VERSION,
                 (unsigned)SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX,
                 (unsigned)SENSORARRAY_TRANSPORT_TEXT_MAX);
        return ESP_OK;
    case SENSORARRAY_EARLY_KIND_RECOVER: {
        uint32_t requestId = 0u;
        sensorarrayRecoveryPost(recoveryLevel, &requestId);
        snprintf(response, responseSize,
                 "RACK,cmd=RECOVER,id=%lu,level=%u,state=accepted\n",
                 (unsigned long)requestId,
                 (unsigned)recoveryLevel);
        return ESP_OK;
    }
    case SENSORARRAY_EARLY_KIND_RECOVER_INVALID:
        snprintf(response, responseSize,
                 "ERR,cmd=RECOVER,reason=level,range=0-%u\n",
                 (unsigned)SENSORARRAY_RECOVERY_LEVEL_MAX);
        return ESP_ERR_INVALID_ARG;
    case SENSORARRAY_EARLY_KIND_RESTART: {
        uint32_t requestId = 0u;
        sensorarrayRestartPost(&requestId);
        snprintf(response, responseSize,
                 "RACK,cmd=RESTART,id=%lu,state=accepted\n",
                 (unsigned long)requestId);
        return ESP_OK;
    }
    case SENSORARRAY_EARLY_KIND_ROW_MODES_REJECT:
        snprintf(response, responseSize,
                 "ERR,cmd=ROWMODES,reason=not_ready\n");
        return ESP_ERR_INVALID_STATE;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

static void sensorarrayControlReplyPublished(const char *data, size_t length)
{
    if (!data || length == 0u) {
        return;
    }
    unsigned long requestId = 0u;
    if (sscanf(data, "RMACK,id=%lu", &requestId) == 1) {
        if (!sensorarrayCommandMailboxCommitRowModesAck(
                (uint32_t)requestId)) {
            printf("RMACK_TERM,id=%lu,state=unknown_reservation\n",
                   requestId);
        }
    }
}

static void sensorarrayControlReplyFailed(esp_err_t error,
                                          const char *data,
                                          size_t length)
{
    if (!data || length == 0u) {
        return;
    }
    unsigned long requestId = 0u;
    if (sscanf(data, "RMACK,id=%lu", &requestId) == 1) {
        if (sensorarrayCommandMailboxCancelRowModesAck((uint32_t)requestId)) {
            printf("RMACK_CANCEL,id=%lu,state=cancelled,reason=ack_publish_failed,err=0x%lx\n",
                   requestId,
                   (unsigned long)error);
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
    if (strcmp(command, "ROWMODES?") == 0) {
        sensorarrayRowModeProfile_t profile = {0};
        char active[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u] = {0};
        char pending[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u] = {0};
        if (!ctx || !sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile) ||
            !sensorarrayRowModeProfileFormat(profile.modes, active)) {
            snprintf(response, responseSize,
                     "ERR,cmd=ROWMODES,reason=snapshot_busy\n");
            return ESP_ERR_TIMEOUT;
        }
        const char *pendingText = "none";
        if (profile.pending && sensorarrayRowModeProfileFormat(profile.pendingModes, pending)) {
            pendingText = pending;
        }
        snprintf(response, responseSize,
                 "ROWMODES,active=%s,pending=%s,gen=%lu,rid=%lu,rows=%u,state=%s\n",
                 active,
                 pendingText,
                 (unsigned long)profile.generation,
                 (unsigned long)profile.appliedRequestId,
                 (unsigned)sensorarrayScanConfigGetActiveRows(),
                 sensorarrayRowModeProfileStateName(profile.state));
        return ESP_OK;
    }
    if (strcmp(command, "MODE?") == 0 || strcmp(command, "STATE?") == 0) {
        sensorarrayMeasurementModeSnapshot_t mode = {0};
        sensorarrayRowModeProfile_t rowProfile = {0};
        char profileText[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u] = {0};
        sensorarrayRouteSnapshot_t route = {0};
        bool modeOk = ctx && sensorarrayMeasurementModeCopySnapshot(
            &ctx->measurementMode, &mode);
        bool routeOk = ctx && sensorarrayRouteControllerCopySnapshot(
            &ctx->routeController, &route);
        bool profileOk = ctx && sensorarrayRowModeProfileCopy(
            &ctx->rowModeProfile, &rowProfile) &&
            sensorarrayRowModeProfileFormat(rowProfile.modes, profileText);
        if (!modeOk || !routeOk || !profileOk) {
            snprintf(response, responseSize,
                     "ERR,cmd=MODE,reason=snapshot_busy\n");
            return ESP_ERR_TIMEOUT;
        }
        int written = snprintf(response,
                 responseSize,
                 "MODE,state=%s,active=%s,pending=%s,pid=%lu,gen=%lu,rid=%lu,seq=%lu,layout=%s,profile=%s,route=%s,row=%u,sw=%s,source=%s,matrixRef=%s,intref=%u,vbias=%u,refmux=%02X,pga=%u,rail=%u,age=%lu,fdcPrimary=%s,fdcPrimaryVerified=%u,fdcSecondary=%s,fdcSecondaryVerified=%u,fdcSd=%s,fdcSdVerified=%u,fdcRestartRequired=%u,budgetFps=%lu,captureCap=%lu,transitionUs=%llu,transitions=%lu,heap=%lu,heapMin=%lu,stackWords=%lu,stackBytes=%lu,stackUnit=bytes\n",
                 sensorarrayMeasurementStateName(mode.state),
                 sensorarrayMeasurementModeName(mode.activeMode),
                 mode.pending ? sensorarrayMeasurementModeName(mode.pendingMode) : "NONE",
                 (unsigned long)mode.pendingRequestId,
                 (unsigned long)mode.generation,
                 (unsigned long)mode.appliedRequestId,
                 (unsigned long)mode.appliedFrameSequence,
                 sensorarrayRowModeProfileIsHomogeneous(rowProfile.modes) ?
                     "HOMOGENEOUS" : "MIXED",
                 profileText,
                 route.safe ? "SAFE" : sensorarrayMeasurementModeName(route.mode),
                 (unsigned)route.row,
                 route.profile.swPhysicalLevel == SENSORARRAY_SW_PHYSICAL_HIGH ?
                     "HIGH" : "LOW",
                 route.profile.swLogicalSource == TMUX1108_SOURCE_REF ?
                     "REF" : "GND",
                 sensorarrayBoardMapMatrixExcitationName(
                     route.profile.matrixExcitationEnabled),
                 (route.power & ADS126X_POWER_INTREF) != 0u ? 1u : 0u,
                 (route.power & ADS126X_POWER_VBIAS) != 0u ? 1u : 0u,
                 route.refmux,
                 (unsigned)route.pgaGain,
                 route.railValid ? 1u : 0u,
                 (unsigned long)route.railAgeFrames,
                 route.fdcPrimarySleeping ? "sleep" : "active",
                 route.fdcPrimaryVerified ? 1u : 0u,
                 route.fdcSecondarySleeping ? "sleep" : "active",
                 route.fdcSecondaryVerified ? 1u : 0u,
                 route.fdcSdHigh ? "high" : "low",
                 route.fdcSdVerified ? 1u : 0u,
                 route.fdcSdHigh ? 1u : 0u,
                 (unsigned long)sensorarrayFrameBudgetFps(mode.activeMode),
                 (unsigned long)sensorarrayFrameTargetFps(mode.activeMode),
                 (unsigned long long)route.transitionDurationUs,
                 (unsigned long)ctx->modeTransitionCount,
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size(),
                 (unsigned long)(ctx->scanTaskHandle ?
                     uxTaskGetStackHighWaterMark(ctx->scanTaskHandle) : 0u),
                 (unsigned long)(ctx->scanTaskHandle ?
                     uxTaskGetStackHighWaterMark(ctx->scanTaskHandle) *
                         sizeof(StackType_t) : 0u));
        if (written < 0 || (size_t)written >= responseSize) {
            (void)snprintf(response,
                           responseSize,
                           "ERR,cmd=MODE,reason=response_too_long\n");
            return ESP_ERR_INVALID_SIZE;
        }
        return ESP_OK;
    }
    if (strncmp(command, "ROWMODES=", 9u) == 0) {
        const char *profile = command + 9u;
        sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
        if (!sensorarrayRowModeProfileParse(profile, strlen(profile), modes)) {
            snprintf(response, responseSize,
                     "ERR,cmd=ROWMODES,reason=value,format=ROWMODES=<8 chars C|V|R>\n");
            return ESP_ERR_INVALID_ARG;
        }
        sensorarrayRowModeProfile_t current = {0};
        char oldProfile[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u] = {0};
        (void)sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &current);
        (void)sensorarrayRowModeProfileFormat(current.modes, oldProfile);
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostRowModes(profile, &requestId);
        if (postErr != ESP_OK) {
            snprintf(response, responseSize,
                     "ERR,cmd=ROWMODES,reason=%s,err=0x%lx\n",
                     postErr == ESP_ERR_NO_MEM ? "capacity" : "mailbox",
                     (unsigned long)postErr);
            return postErr;
        }
        snprintf(response, responseSize,
                 "RMACK,id=%lu,old=%s,new=%s,state=accepted\n",
                 (unsigned long)requestId, oldProfile, profile);
        return ESP_OK;
    }
    if (strncmp(command, "MODE=", 5u) == 0) {
        sensorarrayMeasurementMode_t requested = SENSORARRAY_MEASUREMENT_MODE_NONE;
        const char *value = command + 5u;
        if (strcmp(value, "CAP") == 0 || strcmp(value, "CAPACITANCE") == 0) {
            requested = SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
        } else if (strcmp(value, "VOLT") == 0 || strcmp(value, "VOLTAGE") == 0) {
            requested = SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
        } else if (strcmp(value, "RES") == 0 || strcmp(value, "RESISTANCE") == 0) {
            requested = SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
        } else {
            snprintf(response, responseSize,
                     "ERR,cmd=MODE,reason=value,allowed=CAP|VOLT|RES\n");
            return ESP_ERR_INVALID_ARG;
        }
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostMeasurementMode(
            requested, &requestId);
        if (postErr != ESP_OK) {
            snprintf(response, responseSize,
                     "ERR,cmd=MODE,mode=%s,reason=mailbox,err=0x%lx\n",
                     sensorarrayMeasurementModeName(requested),
                     (unsigned long)postErr);
            return postErr;
        }
        sensorarrayMeasurementModeSnapshot_t mode = {0};
        (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode);
        snprintf(response, responseSize,
                 "MACK,id=%lu,old=%s,new=%s,state=accepted\n",
                 (unsigned long)requestId,
                 sensorarrayMeasurementModeName(mode.activeMode),
                 sensorarrayMeasurementModeName(requested));
        return ESP_OK;
    }
    if (strncmp(command, "RAILCFG=", 8u) == 0) {
        int32_t avddUv = 0;
        int32_t avssUv = 0;
        char trailing = '\0';
        if (sscanf(command + 8u,
                   "%" SCNd32 ",%" SCNd32 "%c",
                   &avddUv,
                   &avssUv,
                   &trailing) != 2 || avddUv <= 0 || avssUv >= 0) {
            snprintf(response,
                     responseSize,
                     "ERR,cmd=RAILCFG,reason=value,format=RAILCFG=<AVDD_UV>,<negative_AVSS_UV>\n");
            return ESP_ERR_INVALID_ARG;
        }
        sensorarrayMeasurementModeSnapshot_t activeMode = {0};
        if (sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode,
                                                   &activeMode) &&
            activeMode.activeMode ==
                SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
            snprintf(response,
                     responseSize,
                     "ERR,cmd=RAILCFG,reason=apply_before_volt\n");
            return ESP_ERR_INVALID_STATE;
        }
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostRailCalibration(
            avddUv, avssUv, &requestId);
        if (postErr != ESP_OK) {
            snprintf(response,
                     responseSize,
                     "ERR,cmd=RAILCFG,reason=mailbox,err=0x%lx\n",
                     (unsigned long)postErr);
            return postErr;
        }
        snprintf(response,
                 responseSize,
                 "RACK,id=%lu,avdd=%ld,avss=%ld,source=external,state=accepted\n",
                 (unsigned long)requestId,
                 (long)avddUv,
                 (long)avssUv);
        return ESP_OK;
    }
    if (strncmp(command, "CELL?=S", 7u) == 0) {
        unsigned row = 0u;
        unsigned dLine = 0u;
        char trailing = '\0';
        if (sscanf(command + 6u, "S%uD%u%c", &row, &dLine, &trailing) != 2 ||
            row < 1u || row > 8u || dLine < 1u || dLine > 8u) {
            snprintf(response, responseSize,
                     "ERR,cmd=CELL,reason=coordinate,format=CELL?=S1D1\n");
            return ESP_ERR_INVALID_ARG;
        }
        sensorarrayMeasurementCellSnapshot_t cell = {0};
        if (!sensorarrayCopyLastMeasurementCell(ctx,
                                                (uint8_t)row,
                                                (uint8_t)dLine,
                                                &cell)) {
            snprintf(response, responseSize,
                     "ERR,cmd=CELL,reason=snapshot_busy\n");
            return ESP_ERR_TIMEOUT;
        }
        snprintf(response,
                 responseSize,
                 "CELL,seq=%lu,cell=S%uD%u,mode=%s,value=%lld,unit=%s,scale=%d,raw=%ld,nodeUv=%ld,vref=%ld,vss=%ld,rref=%lu,pga=%u,valid=%u,fresh=%u,error=%s,rail=%u,age=%lu,ref=%s\n",
                 (unsigned long)cell.sequence,
                 row,
                 dLine,
                 sensorarrayMeasurementModeName(cell.mode),
                 (long long)cell.valueFixed,
                 sensorarrayMeasurementUnitName(cell.unit),
                 (int)cell.decimalScale,
                 (long)cell.rawCode,
                 (long)cell.nodeUv,
                 (long)cell.matrixReferenceUv,
                 (long)cell.avssUv,
                 (unsigned long)cell.referenceResistorOhms,
                 (unsigned)cell.pgaGain,
                 cell.valid ? 1u : 0u,
                 cell.fresh ? 1u : 0u,
                 sensorarrayCellErrorName(
                     (sensorarrayCellError_t)cell.errorReason),
                 cell.railValid ? 1u : 0u,
                 (unsigned long)cell.railAgeFrames,
                 sensorarrayAdsReferenceSourceName(cell.referenceSource));
        return ESP_OK;
    }
    if (strcmp(command, "BAT?") == 0) {
        return sensorarrayAdsGapFormatBattery(response, responseSize, frameSequence) > 0u ?
            ESP_OK : ESP_FAIL;
    }
    if (strcmp(command, "RESSETTLE?") == 0) {
        snprintf(response, responseSize,
                 "RESSETTLE,settleUs=%lu,phases=2,source=runtime\n",
                 (unsigned long)sensorarrayRouteControllerGetRowSettleUs(
                     &ctx->routeController));
        return ESP_OK;
    }
    if (strncmp(command, "RESSETTLE=", 10u) == 0) {
        char *end = NULL;
        unsigned long parsed = strtoul(command + 10u, &end, 10);
        if (end == command + 10u || *end != '\0' || parsed > 10000u) {
            snprintf(response, responseSize,
                     "ERR,cmd=RESSETTLE,reason=range,max=10000\n");
            return ESP_ERR_INVALID_ARG;
        }
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostResSettle(
            (uint32_t)parsed, &requestId);
        if (postErr != ESP_OK) {
            return postErr;
        }
        snprintf(response, responseSize,
                 "ACK,cmd=RESSETTLE,id=%lu,settleUs=%lu,status=accepted\n",
                 (unsigned long)requestId,
                 parsed);
        return ESP_OK;
    }
    if (strcmp(command, "BATD") == 0 ||
        strcmp(command, "BATD=VBIAS_ON") == 0 ||
        strcmp(command, "BATD,MODE=VBIAS_ON") == 0) {
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostBatteryNow(
            true, &requestId);
        if (postErr != ESP_OK) {
            return postErr;
        }
        snprintf(response, responseSize,
                 "ACK,cmd=BATD,id=%lu,mode=vbias_on,status=accepted\n",
                 (unsigned long)requestId);
        return ESP_OK;
    }
    if (strcmp(command, "BATNOW") == 0) {
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostBatteryNow(
            false, &requestId);
        if (postErr != ESP_OK) {
            return postErr;
        }
        snprintf(response, responseSize,
                 "ACK,cmd=BATNOW,id=%lu,status=accepted\n",
                 (unsigned long)requestId);
        return ESP_OK;
    }
    if (strcmp(command, "BATPERIOD?") == 0) {
        sensorarrayAdsGapSnapshot_t battery = {0};
        sensorarrayAdsGapCopySnapshot(&battery, frameSequence);
        snprintf(response, responseSize,
                 "BATPERIOD,enabled=%u,periodMs=%lu,due=%u,ageMs=%lu\n",
                 battery.batteryEnabled ? 1u : 0u,
                 (unsigned long)battery.batteryPeriodMs,
                 battery.batteryDue ? 1u : 0u,
                 (unsigned long)battery.batteryAgeMs);
        return ESP_OK;
    }
    if (strncmp(command, "BATPERIOD=", 10u) == 0) {
        const char *value = command + 10u;
        bool enabled = true;
        uint32_t periodMs = 0u;
        if (strcmp(value, "OFF") == 0) {
            enabled = false;
        } else {
            if (strncmp(value, "ON,", 3u) == 0) {
                value += 3u;
            }
            char *end = NULL;
            unsigned long parsed = strtoul(value, &end, 10);
            if (end == value || *end != '\0' || parsed < 100u ||
                parsed > 600000u) {
                snprintf(response, responseSize,
                         "ERR,cmd=BATPERIOD,reason=period,range=100-600000\n");
                return ESP_ERR_INVALID_ARG;
            }
            periodMs = (uint32_t)parsed;
        }
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostBatteryPeriod(
            enabled, periodMs, &requestId);
        if (postErr != ESP_OK) {
            return postErr;
        }
        snprintf(response, responseSize,
                 "ACK,cmd=BATPERIOD,id=%lu,enabled=%u,periodMs=%lu,status=accepted\n",
                 (unsigned long)requestId,
                 enabled ? 1u : 0u,
                 (unsigned long)periodMs);
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
    if (strcmp(command, "ADSCHK") == 0 || strncmp(command, "ADSCHK=", 7u) == 0) {
        uint32_t sampleCount = 100u;
        if (command[6] == '=') {
            char *end = NULL;
            unsigned long parsed = strtoul(command + 7u, &end, 10);
            if (end == command + 7u || *end != '\0' || parsed < 1u || parsed > 1000u) {
                snprintf(response, responseSize,
                         "ERR,cmd=ADSCHK,reason=sample_count,range=1-1000\n");
                return ESP_ERR_INVALID_ARG;
            }
            sampleCount = (uint32_t)parsed;
        }
        uint32_t requestId = 0u;
        esp_err_t postErr = sensorarrayCommandMailboxPostAdsCheck(sampleCount,
                                                                  &requestId);
        if (postErr != ESP_OK) {
            snprintf(response, responseSize,
                     "ERR,cmd=ADSCHK,reason=mailbox,err=0x%lx\n",
                     (unsigned long)postErr);
            return postErr;
        }
        snprintf(response, responseSize,
                 "ACK,cmd=ADSCHK,id=%lu,samples=%lu,status=accepted\n",
                 (unsigned long)requestId,
                 (unsigned long)sampleCount);
        return ESP_OK;
    }
    if (strcmp(command, "CAL=SAVE") == 0 || strcmp(command, "CAL=LOAD") == 0) {
        esp_err_t postErr = sensorarrayCommandMailboxPostText(
            (const uint8_t *)command, strlen(command));
        if (postErr != ESP_OK) {
            return postErr;
        }
        snprintf(response, responseSize, "ACK,cmd=CAL,v=%s,status=accepted\n",
                 command[4] == 'S' ? "SAVE" : "LOAD");
        return ESP_OK;
    }
    if (strcmp(command, "CAL?") == 0) {
        sensorarrayCalibrationStatus_t status = {0};
        esp_err_t queryErr = sensorarrayCalibrationQuery(&status);
        if (queryErr != ESP_OK) {
            snprintf(response, responseSize,
                     "ERR,cmd=CAL,reason=query,err=0x%lx\n",
                     (unsigned long)queryErr);
            return queryErr;
        }
        snprintf(response,
                 responseSize,
                 "CAL,source=%lu,schema=%lu,valid=%u,boardId=%08lX,hardwareRev=%lu,payloadLength=%lu\n",
                 (unsigned long)status.source,
                 (unsigned long)status.schemaVersion,
                 status.valid ? 1u : 0u,
                 (unsigned long)status.boardId,
                 (unsigned long)status.hardwareRevision,
                 (unsigned long)status.payloadLength);
        return ESP_OK;
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
    if (strcmp(command, "ADSDBG?") == 0) {
        snprintf(response, responseSize, "ADSDBG,enabled=%u,summaryFrames=%u\n",
                 sensorarrayCommandMailboxAdsDebugEnabled() ? 1u : 0u,
                 sensorarrayCommandMailboxAdsDebugEnabled() ? 1u :
                     (unsigned)SENSORARRAY_CFG_LOG_PERIOD_FRAMES);
        return ESP_OK;
    }
    if (strcmp(command, "ADSDBG=1") == 0 || strcmp(command, "ADSDBG=0") == 0) {
        esp_err_t postErr = sensorarrayCommandMailboxPostText(
            (const uint8_t *)command, strlen(command));
        if (postErr != ESP_OK) {
            return postErr;
        }
        snprintf(response, responseSize, "ACK,cmd=ADSDBG,v=%c,status=accepted\n",
                 command[7]);
        return ESP_OK;
    }
    if (strcmp(command, "FDCISO?") == 0) {
        sensorarrayRouteSnapshot_t route = {0};
        if (!ctx || !sensorarrayRouteControllerCopySnapshot(
                         &ctx->routeController, &route)) {
            snprintf(response, responseSize,
                     "ERR,cmd=FDCISO,reason=snapshot_busy\n");
            return ESP_ERR_TIMEOUT;
        }
        snprintf(response, responseSize,
                 "FDCISO,sd=%s,verified=%u,restartRequired=%u\n",
                 route.fdcSdHigh ? "high" : "low",
                 route.fdcSdVerified ? 1u : 0u,
                 route.fdcSdHigh ? 1u : 0u);
        return ESP_OK;
    }
    if (strcmp(command, "FDCISO=ON") == 0) {
        sensorarrayRouteSnapshot_t route = {0};
        sensorarrayMeasurementModeSnapshot_t mode = {0};
        sensorarrayRowModeProfile_t profile = {0};
        bool routeOk = ctx && sensorarrayRouteControllerCopySnapshot(
                                  &ctx->routeController, &route);
        bool modeOk = ctx && sensorarrayMeasurementModeCopySnapshot(
                                 &ctx->measurementMode, &mode);
        bool profileOk = ctx && sensorarrayRowModeProfileCopy(
                                    &ctx->rowModeProfile, &profile);
        if (!routeOk || !modeOk || !profileOk) {
            snprintf(response, responseSize,
                     "ERR,cmd=FDCISO,reason=snapshot_busy\n");
            return ESP_ERR_TIMEOUT;
        }
        bool routeConsistent =
            routeOk && !route.safe && route.mode == mode.activeMode;
        if (route.fdcSdHigh) {
            snprintf(response, responseSize,
                     "FERR,cmd=FDCISO,sd=high,reason=already_applied,state=rejected,restartRequired=1\n");
            return ESP_ERR_INVALID_STATE;
        }
        bool homogeneousAds =
            mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ||
            mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
        if (!routeConsistent || !homogeneousAds || mode.pending ||
            mode.state == SENSORARRAY_MEASUREMENT_STATE_TRANSITION ||
            profile.pending ||
            !sensorarrayRowModeProfileIsHomogeneous(profile.modes)) {
            snprintf(response, responseSize,
                     "FERR,cmd=FDCISO,reason=not_homogeneous_ads,active=%s,state=rejected\n",
                     sensorarrayMeasurementModeName(mode.activeMode));
            return ESP_ERR_INVALID_STATE;
        }
        uint32_t requestId = 0u;
        esp_err_t postErr =
            sensorarrayCommandMailboxPostFdcIsolation(true, &requestId);
        if (postErr != ESP_OK) {
            snprintf(response, responseSize,
                     "ERR,cmd=FDCISO,reason=mailbox,err=0x%lx\n",
                     (unsigned long)postErr);
            return postErr;
        }
        snprintf(response, responseSize,
                 "FACK,id=%lu,sd=low,state=accepted,restartRequired=0\n",
                 (unsigned long)requestId);
        return ESP_OK;
    }
    if (strcmp(command, "FDCISO=OFF") == 0) {
        sensorarrayRouteSnapshot_t route = {0};
        if (!ctx || !sensorarrayRouteControllerCopySnapshot(
                         &ctx->routeController, &route)) {
            snprintf(response, responseSize,
                     "ERR,cmd=FDCISO,reason=snapshot_busy\n");
            return ESP_ERR_TIMEOUT;
        }
        if (route.fdcSdHigh) {
            snprintf(response, responseSize,
                     "FERR,cmd=FDCISO,sd=high,reason=restart_required,state=rejected,restartRequired=1\n");
            return ESP_ERR_INVALID_STATE;
        }
        snprintf(response, responseSize,
                 "FACK,cmd=FDCISO,sd=low,state=unchanged,restartRequired=0\n");
        return ESP_OK;
    }
    if (strcmp(command, "BOOT?") == 0) {
        return sensorarrayFormatBootReply(response, responseSize, ctx);
    }
    if (strcmp(command, "READY?") == 0) {
        sensorarrayBootBreadcrumb_t boot = {0};
        (void)sensorarrayBootBreadcrumbCopy(&boot);
        bool ready = ctx && ctx->systemReady;
        snprintf(response,
                 responseSize,
                 "READY,ready=%u,stage=%s,err=0x%lx,bootId=%lu,boot=%lu\n",
                 ready ? 1u : 0u,
                 boot.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC ?
                     boot.lastStage : "unknown",
                 (unsigned long)(boot.magic == SENSORARRAY_BOOT_BREADCRUMB_MAGIC ?
                     (uint32_t)boot.lastErr : 0u),
                 (unsigned long)boot.bootId,
                 (unsigned long)boot.bootCount);
        return ESP_OK;
    }
    if (strcmp(command, "PROTO?") == 0) {
        snprintf(response,
                 responseSize,
                 "PROTO,version=%u,wires=ascii,ctrlMax=%u,dataMax=%u,channels=CTRL/DATA/LOG/LIFECYCLE\n",
                 (unsigned)SENSORARRAY_PROTOCOL_VERSION,
                 (unsigned)SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX,
                 (unsigned)SENSORARRAY_TRANSPORT_TEXT_MAX);
        return ESP_OK;
    }
    if (strcmp(command, "BUILD?") == 0) {
        snprintf(response,
                 responseSize,
                 "BUILD,idf=%s,target=%s,project=SensorArray,proto=%u\n",
                 esp_get_idf_version(),
                 CONFIG_IDF_TARGET,
                 (unsigned)SENSORARRAY_PROTOCOL_VERSION);
        return ESP_OK;
    }
    if (strcmp(command, "PERF?") == 0) {
        sensorarrayAsyncLogStats_t asyncStats = {0};
        sensorarrayUsbSinkStats_t usbStats = {0};
        sensorarrayTransportStats_t transportStats = {0};
        sensorarrayAsyncLogGetStats(&asyncStats);
        sensorarrayUsbSinkGetStats(&usbStats);
        sensorarrayTransportGetStats(&transportStats);
        int written = snprintf(response,
                 responseSize,
                 "PERF,pub=%llu,fresh=%llu,stale=%llu,mixed=%llu,dropOut=%llu,dropEvent=%llu,startAvgUs=%llu,startN=%lu,usbSent=%lu,usbDrop=%lu,usbBlock=%lu,usbBytes=%llu,usbWriteMaxUs=%lu,usbQMax=%lu,usbStackMin=%lu,lifePub=%lu,lifeDrop=%lu,queueDrop=%lu,frames=%lu,heap=%lu,heapMin=%lu\n",
                 (unsigned long long)asyncStats.publishedFrames,
                 (unsigned long long)asyncStats.freshFrames,
                 (unsigned long long)asyncStats.staleFrames,
                 (unsigned long long)asyncStats.mixedFrames,
                 (unsigned long long)asyncStats.droppedOutputFrames,
                 (unsigned long long)asyncStats.droppedEventLogs,
                 (unsigned long long)asyncStats.frameStartIntervalAvgUs,
                 (unsigned long)asyncStats.frameStartIntervalCount,
                 (unsigned long)usbStats.sentPackets,
                 (unsigned long)usbStats.droppedPackets,
                 (unsigned long)usbStats.blockedCount,
                 (unsigned long long)usbStats.sentBytes,
                 (unsigned long)usbStats.writeUsMax,
                 (unsigned long)usbStats.queueDepthMax,
                 (unsigned long)usbStats.taskMinimumRemainingBytes,
                 (unsigned long)transportStats.lifecyclePublished,
                 (unsigned long)transportStats.lifecycleDropped,
                 (unsigned long)transportStats.queueDrop,
                 (unsigned long)(ctx ? ctx->fdcFrameCounter : 0u),
                 (unsigned long)esp_get_free_heap_size(),
                 (unsigned long)esp_get_minimum_free_heap_size());
        if (written < 0 || (size_t)written >= responseSize) {
            (void)snprintf(response, responseSize,
                           "ERR,cmd=PERF,reason=response_too_long\n");
            return ESP_ERR_INVALID_SIZE;
        }
        return ESP_OK;
    }
    if (strcmp(command, "USBSTREAM?") == 0) {
        sensorarrayUsbStreamProfile_t profile = sensorarrayUsbSinkGetStreamProfile();
        snprintf(response,
                 responseSize,
                 "USBSTREAM,v=%s,dataEvery=%lu,diagEvery=%lu\n",
                 sensorarrayUsbStreamModeName(profile.mode),
                 (unsigned long)profile.dataEvery,
                 (unsigned long)profile.diagEvery);
        return ESP_OK;
    }
    if (strcmp(command, "USBSTREAM=DEBUG") == 0 ||
        strcmp(command, "USBSTREAM=FULL") == 0) {
        sensorarrayUsbStreamMode_t mode = command[10] == 'D' ?
            SENSORARRAY_USB_STREAM_DEBUG : SENSORARRAY_USB_STREAM_FULL;
        esp_err_t setErr = sensorarrayUsbSinkSetStreamProfile(mode, 0u, 0u);
        if (setErr != ESP_OK) {
            snprintf(response,
                     responseSize,
                     "ERR,cmd=USBSTREAM,reason=apply,err=0x%lx\n",
                     (unsigned long)setErr);
            return setErr;
        }
        sensorarrayUsbStreamProfile_t profile = sensorarrayUsbSinkGetStreamProfile();
        snprintf(response,
                 responseSize,
                 "USBSTREAM,v=%s,dataEvery=%lu,diagEvery=%lu,state=applied\n",
                 sensorarrayUsbStreamModeName(profile.mode),
                 (unsigned long)profile.dataEvery,
                 (unsigned long)profile.diagEvery);
        return ESP_OK;
    }
    if (strcmp(command, "RECOVER") == 0 || strncmp(command, "RECOVER=", 8u) == 0) {
        uint32_t level = SENSORARRAY_RECOVERY_LEVEL_FULL;
        if (command[7] == '=') {
            char *end = NULL;
            unsigned long parsed = strtoul(command + 8u, &end, 10);
            if (end == command + 8u || *end != '\0' ||
                parsed > SENSORARRAY_RECOVERY_LEVEL_MAX) {
                snprintf(response, responseSize,
                         "ERR,cmd=RECOVER,reason=level,range=0-%u\n",
                         (unsigned)SENSORARRAY_RECOVERY_LEVEL_MAX);
                return ESP_ERR_INVALID_ARG;
            }
            level = (uint32_t)parsed;
        }
        if (!ctx || !ctx->systemReady) {
            snprintf(response, responseSize,
                     "ERR,cmd=RECOVER,reason=not_ready\n");
            return ESP_ERR_INVALID_STATE;
        }
        sensorarrayRouteSnapshot_t recoveryRoute = {0};
        if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController,
                                                   &recoveryRoute) &&
            recoveryRoute.fdcSdHigh) {
            snprintf(response, responseSize,
                     "ERR,cmd=RECOVER,reason=fdc_restart_required,restartRequired=1\n");
            return ESP_ERR_INVALID_STATE;
        }
        uint32_t requestId = 0u;
        sensorarrayRecoveryPost(level, &requestId);
        snprintf(response, responseSize,
                 "RACK,cmd=RECOVER,id=%lu,level=%u,state=accepted\n",
                 (unsigned long)requestId,
                 (unsigned)level);
        return ESP_OK;
    }
    if (strcmp(command, "RESTART") == 0) {
        uint32_t requestId = 0u;
        sensorarrayRestartPost(&requestId);
        snprintf(response, responseSize,
                 "RACK,cmd=RESTART,id=%lu,state=accepted\n",
                 (unsigned long)requestId);
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

static bool sensorarrayCalibrationMatrixPayloadValid(const void *payload,
                                                     size_t payloadLength)
{
    if (!payload || payloadLength != sizeof(sensorarrayAdsMatrixCalibration_t)) {
        return false;
    }
    return sensorarrayAdsMatrixCalibrationValid(
        (const sensorarrayAdsMatrixCalibration_t *)payload);
}

static void sensorarrayCalibrationLogOutcome(
    const char *tag,
    const char *state,
    const char *reason,
    const sensorarrayCalibrationStatus_t *status,
    esp_err_t err)
{
    if (!tag || !state || !reason || !status) {
        return;
    }
    printf("%s,state=%s,reason=%s,source=%lu,schema=%lu,valid=%u,boardId=%08lX,hardwareRev=%lu,payloadLength=%lu,err=0x%lx\n",
           tag,
           state,
           reason,
           (unsigned long)status->source,
           (unsigned long)status->schemaVersion,
           status->valid ? 1u : 0u,
           (unsigned long)status->boardId,
           (unsigned long)status->hardwareRevision,
           (unsigned long)status->payloadLength,
           (unsigned long)err);
}

static const char *sensorarrayCalibrationLoadReason(
    const sensorarrayCalibrationStatus_t *status,
    esp_err_t err)
{
    if (err == ESP_OK) {
        return "persisted";
    }
    if (err == ESP_ERR_NOT_FOUND) {
        return "missing";
    }
    if (status->hardwareRevision != 0u &&
        status->hardwareRevision != SENSORARRAY_CALIBRATION_HARDWARE_REVISION) {
        return "hardware_revision";
    }
    if (status->boardId != 0u &&
        status->boardId != SENSORARRAY_CALIBRATION_BOARD_ID) {
        return "board_id";
    }
    if (status->schemaVersion != 0u &&
        status->schemaVersion != SENSORARRAY_CALIBRATION_SCHEMA_VERSION) {
        return "schema_version";
    }
    return "invalid_record";
}

static void sensorarrayLoadPersistedMatrixCalibration(
    sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }
    sensorarrayAdsMatrixCalibration_t calibration = {0};
    sensorarrayCalibrationStatus_t status = {0};
    esp_err_t err = sensorarrayCalibrationLoad(
        sensorarrayCalibrationMatrixPayloadValid,
        &calibration,
        sizeof(calibration),
        NULL,
        &status);
    const char *state = "default";
    const char *reason = sensorarrayCalibrationLoadReason(&status, err);
    if (err == ESP_OK) {
        err = sensorarrayAdsMatrixEngineSetCalibration(&ctx->adsEngine,
                                                       &calibration);
        if (err == ESP_OK) {
            state = "loaded";
            reason = "persisted";
        } else {
            reason = "apply_failed";
        }
    }
    sensorarrayCalibrationLogOutcome("CALBOOT", state, reason, &status, err);
}

static esp_err_t sensorarrayInitFrontends(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    /* The shared FDC SD net must be driven low before either FDC2214 is
     * probed or configured.  The route controller owns this pin; a failed
     * prepare is logged here and keeps FDCISO rejected for this boot. */
    (void)sensorarrayRouteControllerPrepareFdcSdGpio();

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
    esp_err_t routeErr = sensorarrayRouteControllerInit(&ctx->routeController,
                                                        &ctx->state);
    if (routeErr != ESP_OK) {
        sensorarrayLogStartup("route_controller", routeErr, "init_failed",
                              (int32_t)routeErr);
        return routeErr;
    }
    sensorarrayAdsMatrixEngineBindRouteController(&ctx->adsEngine,
                                                   &ctx->routeController);
    sensorarrayLoadPersistedMatrixCalibration(ctx);
    sensorarrayAdsGapBindRegisterCache(
        sensorarrayAdsMatrixEngineRegisterCache(&ctx->adsEngine));
    esp_err_t adsGapErr = sensorarrayAdsGapInit(&ctx->state);
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
    esp_err_t defaultModeErr = sensorarrayApplyMeasurementMode(
        ctx, SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE, 0u, false);
    sensorarrayLogStartup("measurement_mode", defaultModeErr,
                          defaultModeErr == ESP_OK ? "cap_default" : "safe",
                          (int32_t)defaultModeErr);
    if (defaultModeErr != ESP_OK) {
        return defaultModeErr;
    }
    /* Install the acquisition command surface only after the default mode
     * has applied successfully.  From here on sensorarrayInitSystem() cannot
     * fail, so an accepted ROWMODES can never be stranded in a mailbox that
     * a failed-bring-up fatal loop would never drain. */
    sensorarrayTransportSetRuntimeQueryCallback(sensorarrayRuntimeQueryCommand, ctx);
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
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    sensorarrayRowModeProfile_t rowProfile = {0};
    if (!sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        !sensorarrayMeasurementModeIsDataMode(mode.activeMode) ||
        mode.state == SENSORARRAY_MEASUREMENT_STATE_TRANSITION ||
        !sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &rowProfile)) {
        sensorarrayFrameBuilderInitInvalid(&ctx->frame);
        return ESP_ERR_INVALID_STATE;
    }

    uint32_t frameSequence = ctx->hostFrameSequence + 1u;
    sensorarrayMeasurementMode_t effectiveModes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
    memcpy(effectiveModes,
           rowProfile.pending ? rowProfile.pendingModes : rowProfile.modes,
           sizeof(effectiveModes));
    bool mixedProfile = !sensorarrayRowModeProfileIsHomogeneous(effectiveModes);
    bool homogeneousProfileFaultTerminal = false;
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (mixedProfile) {
        bool voltageRowPresent = false;
        for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
            voltageRowPresent = voltageRowPresent ||
                effectiveModes[row] == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
        }
        if (voltageRowPresent && !sensorarrayAdsGapCopyRailSplit(
                frameSequence,
                (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES,
                &(sensorarrayAdsRailSplit_t){0})) {
            /* Mixed profiles can enter VOLT without a prior MODE=VOLT
             * transition. Acquire the rail in the isolated monitor route
             * before the first group owns the matrix. */
            uint64_t monitorTransitionUs = 0u;
            err = sensorarrayRouteControllerEnterSafeRailMonitor(
                &ctx->routeController, &monitorTransitionUs);
            if (err == ESP_OK) {
                err = sensorarrayAdsGapRefreshRailAtBoundary(
                    &ctx->state, frameSequence);
            }
            (void)monitorTransitionUs;
        } else {
            err = ESP_OK;
        }
        sensorarrayScanPlanBuildRowProfile(&ctx->scanPlan,
                                           effectiveModes,
                                           rowProfile.pending ? rowProfile.generation + 1u :
                                               rowProfile.generation,
                                           rowProfile.pending ? rowProfile.pendingRequestId :
                                               rowProfile.appliedRequestId);
        sensorarrayAdsMatrixEngineSetFrameSequenceHint(&ctx->adsEngine,
                                                       frameSequence);
        if (err == ESP_OK) {
            err = sensorarrayMixedRowEngineReadFrame(&ctx->fdcEngine,
                                                     &ctx->adsEngine,
                                                     &ctx->scanPlan,
                                                     &ctx->frame,
                                                     ctx->mixedSegmentWorkspace);
        } else {
            sensorarrayFrameBuilderInitInvalid(&ctx->frame);
            ctx->frame.mixedProfile = true;
        }
        ctx->runtimeMode = SENSORARRAY_RUNTIME_MODE_MIXED_ROW;
    } else {
        char profileText[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u] = {0};
        sensorarrayMeasurementMode_t targetMode = effectiveModes[0];
        bool homogeneousApplyOk = false;
        if (rowProfile.pending) {
            /* ROWMODES homogeneous is still a profile transaction, but it
             * uses the legacy fast path and does not emit user MAPP. The
             * terminal is deferred until the first frame proves the applied
             * route is readable, so an accepted request cannot end in both
             * RMAPP and MFAULT for the same frame. */
            err = sensorarrayApplyMeasurementMode(ctx,
                                                  targetMode,
                                                  rowProfile.pendingRequestId,
                                                  false);
            if (err != ESP_OK) {
                (void)sensorarrayRowModeProfileFormat(effectiveModes, profileText);
                sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile,
                                                        (uint32_t)err,
                                                        0u);
                /* The readback block is skipped on this path, so invalidate
                 * the frame now.  A stale freshFrame from the previous frame
                 * must not be republished under the new sequence number. */
                sensorarrayFrameBuilderInitInvalid(&ctx->frame);
                sensorarrayEmitRowModesTerminal(
                    rowProfile.pendingRequestId,
                    "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE\n",
                    (unsigned long)rowProfile.pendingRequestId,
                    (unsigned long)rowProfile.generation,
                    (unsigned long)frameSequence,
                    profileText,
                    (unsigned long)err);
                homogeneousProfileFaultTerminal = true;
            } else {
                homogeneousApplyOk = true;
            }
        }
        if (homogeneousApplyOk) {
            /* `mode` was snapshotted before this function applied the pending
             * homogeneous profile. Refresh it so the trailing frame metadata
             * (mode/generation/requestId) and the CAP unit/mask branch below
             * describe the actually applied mode, not the pre-apply one.
             * Non-pending MODE frames and mixed frames are unaffected. */
            (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode,
                                                         &mode);
        }
        if (err == ESP_OK || !rowProfile.pending) {
            if (targetMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
                sensorarrayScanPlanBuildDefaultFdcMatrix(&ctx->scanPlan);
                err = sensorarrayFdcMatrixEngineReadFrame(&ctx->fdcEngine,
                                                          &ctx->scanPlan,
                                                          &ctx->frame);
            } else {
                sensorarrayScanPlanBuildAdsMatrix(&ctx->scanPlan, targetMode);
                sensorarrayAdsMatrixEngineSetFrameSequenceHint(&ctx->adsEngine,
                                                               frameSequence);
                err = sensorarrayAdsMatrixEngineReadFrame(&ctx->adsEngine,
                                                          &ctx->scanPlan,
                                                          &ctx->frame);
            }
        }
        if (rowProfile.pending && homogeneousApplyOk) {
            /* The first acquisition/readback decides the transaction
             * terminal: RMAPP only on a fresh frame, RMERR when the applied
             * profile cannot produce one. */
            if (err != ESP_OK || !ctx->frame.freshFrame) {
                uint32_t readErr = err == ESP_OK ?
                    (uint32_t)ESP_ERR_INVALID_STATE : (uint32_t)err;
                sensorarrayMeasurementModeRecordRuntimeFault(
                    &ctx->measurementMode, readErr);
                (void)sensorarrayAdsMatrixEngineSetMode(
                    &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
                (void)sensorarrayRouteControllerEnterSafe(
                    &ctx->routeController, "rowmodes_readback_fault");
                (void)sensorarrayRowModeProfileFormat(effectiveModes, profileText);
                sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile,
                                                        readErr,
                                                        ctx->frame.physicalSweepUs);
                sensorarrayEmitRowModesTerminal(
                    rowProfile.pendingRequestId,
                    "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE\n",
                    (unsigned long)rowProfile.pendingRequestId,
                    (unsigned long)rowProfile.generation,
                    (unsigned long)frameSequence,
                    profileText,
                    (unsigned long)readErr);
                homogeneousProfileFaultTerminal = true;
            } else {
                (void)sensorarrayRowModeProfileFormat(effectiveModes, profileText);
                (void)sensorarrayRowModeProfileCompleteTransition(
                    &ctx->rowModeProfile, frameSequence, ctx->frame.physicalSweepUs);
                sensorarrayEmitRowModesTerminal(
                    rowProfile.pendingRequestId,
                    "RMAPP,id=%lu,gen=%lu,seq=%lu,profile=%s,state=applied\n",
                    (unsigned long)rowProfile.pendingRequestId,
                    (unsigned long)(rowProfile.generation + 1u),
                    (unsigned long)frameSequence,
                    profileText);
            }
        }
    }
    ctx->hostFrameSequence = frameSequence;
    ctx->frame.sequence = frameSequence;
    if (mixedProfile) {
        bool mixedProfileApplied = ctx->frame.freshFrame;
        bool mixedRequestFailed = rowProfile.pending && !mixedProfileApplied;
        uint32_t mixedFailureErr = err == ESP_OK ?
            (uint32_t)ESP_ERR_INVALID_STATE : (uint32_t)err;
        ctx->frame.rowProfileGeneration = rowProfile.pending ?
            (mixedProfileApplied ? rowProfile.generation + 1u : rowProfile.generation) :
            rowProfile.generation;
        ctx->frame.rowProfileRequestId = rowProfile.pending ? rowProfile.pendingRequestId :
            rowProfile.appliedRequestId;
        if (rowProfile.pending && mixedProfileApplied) {
            char appliedProfile[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u];
            (void)sensorarrayRowModeProfileFormat(effectiveModes,
                                                  appliedProfile);
            (void)sensorarrayRowModeProfileCompleteTransition(
                &ctx->rowModeProfile, frameSequence, ctx->frame.physicalSweepUs);
            sensorarrayEmitRowModesTerminal(
                rowProfile.pendingRequestId,
                "RMAPP,id=%lu,gen=%lu,seq=%lu,profile=%s,state=applied\n",
                (unsigned long)rowProfile.pendingRequestId,
                (unsigned long)(rowProfile.generation + 1u),
                (unsigned long)frameSequence,
                appliedProfile);
        } else if (mixedRequestFailed) {
            char rejectedProfile[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u];
            (void)sensorarrayRowModeProfileFormat(
                effectiveModes, rejectedProfile);
            sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile,
                                                    mixedFailureErr,
                                                    ctx->frame.physicalSweepUs);
            sensorarrayEmitRowModesTerminal(
                rowProfile.pendingRequestId,
                "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE\n",
                (unsigned long)rowProfile.pendingRequestId,
                (unsigned long)rowProfile.generation,
                (unsigned long)frameSequence,
                rejectedProfile,
                (unsigned long)mixedFailureErr);
        }
        if (!ctx->frame.freshFrame) {
            sensorarrayMeasurementModeRecordRuntimeFault(
                &ctx->measurementMode,
                mixedRequestFailed ? mixedFailureErr : (uint32_t)err);
            (void)sensorarrayRouteControllerEnterSafe(
                &ctx->routeController, "mixed_profile_fault");
            if (!mixedRequestFailed) {
                sensorarrayAdsFaultEvent_t fault = {
                    .stage = SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION,
                    .err = err,
                    .seq = frameSequence,
                };
                sensorarrayAdsFaultFillContext(ctx, &fault);
                fault.rowGeneration = rowProfile.generation;
                fault.rowRequestId = rowProfile.pending ?
                    rowProfile.pendingRequestId : rowProfile.appliedRequestId;
                fault.owner = sensorarrayAdsOwnerName(
                    SENSORARRAY_ADS_OWNER_MATRIX);
                sensorarrayEmitAdsFault(ctx, &fault);
                printf("MFAULT,mode=MIXED,seq=%lu,err=0x%lx,state=DEGRADED,route=SAFE\n",
                       (unsigned long)frameSequence,
                       (unsigned long)err);
            }
        }
    } else {
        ctx->frame.measurement.mode = mode.activeMode;
        ctx->frame.measurement.modeGeneration = mode.generation;
        ctx->frame.measurement.modeRequestId = mode.appliedRequestId;
    }
    if (!mixedProfile && mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
        ctx->frame.measurement.unit = SENSORARRAY_MEASUREMENT_UNIT_PF;
        ctx->frame.measurement.decimalScale = -6;
        ctx->frame.measurement.validMask = ctx->frame.capValidMask;
        ctx->frame.measurement.freshMask = ctx->frame.freshMask;
        ctx->frame.measurement.errorMask = ctx->frame.errorMask;
    } else if (!mixedProfile && !ctx->frame.freshFrame &&
               !homogeneousProfileFaultTerminal) {
        /* A coherent ADS frame may contain explicitly invalid cells. A false
         * frame freshness flag instead means route/rail ownership failed. */
        sensorarrayMeasurementModeRecordRuntimeFault(&ctx->measurementMode,
                                                      (uint32_t)err);
        (void)sensorarrayAdsMatrixEngineSetMode(
            &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                  "ads_frame_fault");
        sensorarrayAdsFaultEvent_t fault = {
            .stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE,
            .err = err,
            .seq = frameSequence,
        };
        sensorarrayAdsFaultFillContext(ctx, &fault);
        fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_MATRIX);
        sensorarrayEmitAdsFault(ctx, &fault);
        printf("MFAULT,mode=%s,seq=%lu,err=0x%lx,state=DEGRADED,route=SAFE\n",
               sensorarrayMeasurementModeName(mode.activeMode),
               (unsigned long)frameSequence,
               (unsigned long)err);
    }
    if (rowsApply.applied) {
        printf("RAPP,id=%lu,seq=%lu,old=%u,new=%u,gen=%lu,status=applied\n",
               (unsigned long)rowsApply.requestId,
               (unsigned long)ctx->frame.sequence,
               (unsigned)rowsApply.oldRows,
               (unsigned)rowsApply.newRows,
               (unsigned long)rowsApply.generation);
    }
    sensorarrayPublishLastMeasurement(ctx);
    return err;
}

static void sensorarrayRuntimeRescueTick(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    sensorarrayRowModeProfile_t profile = {0};
    if (sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) &&
        mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE &&
        sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile) &&
        sensorarrayRowModeProfileIsHomogeneous(profile.modes)) {
        (void)sensorarrayFdcRescueTick(&ctx->fdcEngine, &ctx->frame, &ctx->fdcRescue);
    }
}

static void sensorarrayRuntimeI2cFallbackTick(sensorarrayAppContext_t *ctx)
{
    if (!ctx || !CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE) {
        return;
    }
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    if (!sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        mode.activeMode != SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
        ctx->runtimeI2cErrorStreak = 0u;
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
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    if (!ctx ||
        !sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        mode.activeMode != SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ||
        !sensorarrayFdcSweepConsumeForceFullSweepAll()) {
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

static esp_err_t sensorarrayRunRecoveryLevel(sensorarrayAppContext_t *ctx,
                                             uint32_t level)
{
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    sensorarrayRowModeProfile_t profile = {0};
    if (!sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        !sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile) ||
        !sensorarrayMeasurementModeIsDataMode(mode.activeMode) ||
        mode.state == SENSORARRAY_MEASUREMENT_STATE_TRANSITION ||
        mode.pending || profile.pending ||
        !sensorarrayRowModeProfileIsHomogeneous(profile.modes)) {
        return ESP_ERR_INVALID_STATE;
    }

    if (level >= SENSORARRAY_RECOVERY_LEVEL_FULL) {
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                  "recover_full");
        sensorarrayProbeAndLockFdcI2cClock(&ctx->state.fdcPrimary,
                                           "recover_full");
        sensorarrayProbeAndLockFdcI2cClock(&ctx->state.fdcSecondary,
                                           "recover_full");
        if (mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
            (void)sensorarrayFdcMatrixEngineRunFullRescue(&ctx->fdcEngine,
                                                          "recover_full");
        }
        sensorarrayFdcRescueReset(&ctx->fdcRescue);
    }
    return sensorarrayApplyMeasurementMode(ctx, mode.activeMode, 0u, false);
}

static bool sensorarrayRunRecoveryBoundary(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return false;
    }

    uint32_t restartId = 0u;
    if (sensorarrayRestartTake(&restartId)) {
        char marker[96];
        (void)snprintf(marker, sizeof(marker),
                       "RAPP,cmd=RESTART,id=%lu,state=restarting,kind=manual\n",
                       (unsigned long)restartId);
        (void)sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_MANUAL);
        sensorarrayBootBreadcrumbSetStage("manual_restart", ESP_OK, ctx);
        sensorarrayBootEmitRestartMarker(marker);
        esp_restart();
        return true;
    }

    uint32_t level = SENSORARRAY_RECOVERY_LEVEL_NONE;
    uint32_t requestId = 0u;
    if (!sensorarrayRecoveryTake(&level, &requestId)) {
        return false;
    }

    sensorarrayRouteSnapshot_t recoveryRoute = {0};
    if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController,
                                               &recoveryRoute) &&
        recoveryRoute.fdcSdHigh) {
        /* FDC SD high has already put both FDC2214 frontends into shutdown.
         * Any FDC probe/recovery would touch the dead I2C path; CAP is only
         * recoverable through an explicit device restart. */
        printf("RERR,cmd=RECOVER,id=%lu,level=%u,state=rejected,reason=fdc_restart_required,restartRequired=1\n",
               (unsigned long)requestId,
               (unsigned)level);
        return true;
    }

    if (level == SENSORARRAY_RECOVERY_LEVEL_RESTART) {
        if (sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
            char marker[96];
            (void)snprintf(marker, sizeof(marker),
                           "RAPP,cmd=RECOVER,id=%lu,level=2,state=restarting,kind=auto\n",
                           (unsigned long)requestId);
            sensorarrayBootBreadcrumbSetStage("auto_restart", ESP_ERR_INVALID_STATE, ctx);
            sensorarrayBootEmitRestartMarker(marker);
            esp_restart();
        }
        sensorarrayBootBreadcrumbSetStage("recovery_safe_idle",
                                          ESP_ERR_INVALID_STATE,
                                          ctx);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                  "recovery_safe_idle");
        printf("RAPP,cmd=RECOVER,id=%lu,level=2,state=safe,reason=restart_guard\n",
               (unsigned long)requestId);
        return true;
    }

    esp_err_t recoverErr = sensorarrayRunRecoveryLevel(ctx, level);
    printf("RAPP,cmd=RECOVER,id=%lu,level=%u,state=%s,err=0x%lx\n",
           (unsigned long)requestId,
           (unsigned)level,
           recoverErr == ESP_OK ? "applied" : "rejected",
           (unsigned long)recoverErr);
    return true;
}

static void sensorarrayEmitPayloadTooLargeDrop(uint32_t sequence)
{
    int64_t nowUs = esp_timer_get_time();
    int64_t sinceUs = nowUs - s_lastPayloadTooLargeDropUs;
    if (s_lastPayloadTooLargeDropUs != 0 &&
        sinceUs >= 0 &&
        sinceUs < SENSORARRAY_PAYLOAD_TOO_LARGE_RATE_LIMIT_US) {
        return;
    }
    s_lastPayloadTooLargeDropUs = nowUs;
    if (s_payloadTooLargeDropCount < UINT32_MAX) {
        s_payloadTooLargeDropCount++;
    }
    char event[128];
    int written = snprintf(event,
                           sizeof(event),
                           "TXDROP,ch=0,seq=%lu,drop=%lu,reason=payload_too_large\n",
                           (unsigned long)sequence,
                           (unsigned long)s_payloadTooLargeDropCount);
    if (written > 0 && (size_t)written < sizeof(event)) {
        (void)sensorarrayAsyncLogPublishProtocolEvent(event, (size_t)written);
    }
}

static void sensorarrayRunMainLoop(sensorarrayAppContext_t *ctx)
{
    sensorarrayBootBreadcrumbSetStage("main_loop", ESP_OK, ctx);
    while (true) {
        if (sensorarrayRunRecoveryBoundary(ctx)) {
            continue;
        }
        if (sensorarrayMeasurementRecoveryIsActive(&ctx->adsRestoreRecovery)) {
            sensorarrayRunAdsRestoreRecoveryAttempt(ctx);
            continue;
        }
        /* CommandMailbox is drained only here, before a new frame begins. No
         * BLE callback or Core0 task can mutate acquisition state mid-row. */
        sensorarrayApplyPendingCommands(ctx);
        sensorarrayRunQueuedFullSweep(ctx);

        esp_err_t railRefreshErr = sensorarrayRefreshVoltageRailIfNeeded(ctx);
        if (railRefreshErr != ESP_OK) {
            sensorarrayMeasurementModeRecordRuntimeFault(
                &ctx->measurementMode, (uint32_t)railRefreshErr);
            (void)sensorarrayAdsMatrixEngineSetMode(
                &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
            (void)sensorarrayRouteControllerEnterSafe(
                &ctx->routeController, "volt_rail_refresh_failed");
            sensorarrayAdsFaultEvent_t fault = {
                .stage = SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR,
                .err = railRefreshErr,
                .seq = ctx->hostFrameSequence + 1u,
            };
            sensorarrayAdsFaultFillContext(ctx, &fault);
            fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_RAIL);
            sensorarrayEmitAdsFault(ctx, &fault);
            printf("MFAULT,mode=VOLT,seq=%lu,err=0x%lx,state=DEGRADED,route=SAFE\n",
                   (unsigned long)(ctx->hostFrameSequence + 1u),
                   (unsigned long)railRefreshErr);
            continue;
        }

        sensorarrayMeasurementModeSnapshot_t loopMode = {0};
        (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode,
                                                     &loopMode);
        if (!sensorarrayMeasurementModeIsDataMode(loopMode.activeMode) ||
            loopMode.state == SENSORARRAY_MEASUREMENT_STATE_TRANSITION) {
            /* SAFE/DEGRADED remains command-responsive without attempting to
             * fabricate ordinary frames or spin after a route/readback fault. */
            sensorarrayRowModeProfile_t stalledProfile = {0};
            if (sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &stalledProfile) &&
                stalledProfile.pending) {
                char stalledText[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u] = {0};
                (void)sensorarrayRowModeProfileFormat(stalledProfile.pendingModes,
                                                      stalledText);
                sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile,
                                                        (uint32_t)ESP_ERR_INVALID_STATE,
                                                        0u);
                sensorarrayEmitRowModesTerminal(
                    stalledProfile.pendingRequestId,
                    "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE\n",
                    (unsigned long)stalledProfile.pendingRequestId,
                    (unsigned long)stalledProfile.generation,
                    (unsigned long)(ctx->hostFrameSequence + 1u),
                    stalledText,
                    (unsigned long)ESP_ERR_INVALID_STATE);
            }
            vTaskDelay(pdMS_TO_TICKS(50u));
            continue;
        }
        bool capacitanceActive =
            loopMode.activeMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
        sensorarrayRowModeProfile_t loopProfile = {0};
        if (sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &loopProfile) &&
            !sensorarrayRowModeProfileIsHomogeneous(loopProfile.modes)) {
            capacitanceActive = false;
        }

        if (CONFIG_SENSORARRAY_RUNTIME_PERIODIC_DIAG_ENABLE &&
            (ctx->fdcFrameCounter % 100u) == 0u) {
            sensorarrayLogStackHighWater("fdc_matrix_loop");
            sensorarrayLogRuntimeMemoryDiag("main_loop_100", ctx);
        }
        if (capacitanceActive &&
            (ctx->fdcDiagnosticMode ||
             sensorarrayFdcMatrixEngineDiagnosticMode(&ctx->fdcEngine))) {
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

        sensorarrayRunBatteryBoundary(ctx, ctx->frame.measurement.mode);
        /* Intrusive ADS diagnostics run only after the complete acquisition
         * frame. The finished frame remains coherent and the next frame cannot
         * start until register restoration has been verified. */
        sensorarrayMeasurementModeSnapshot_t postBoundaryMode = {0};
        if (sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode,
                                                    &postBoundaryMode) &&
            sensorarrayMeasurementModeIsDataMode(postBoundaryMode.activeMode)) {
            sensorarrayRunPendingAdsCheck(ctx);
        }

        sensorarrayMeasurementMode_t frameMode = ctx->frame.measurement.mode;
        bool mixedFrame = ctx->frame.mixedProfile;
        bool adsFrame = frameMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ||
                        frameMode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
        uint64_t validMask = mixedFrame ?
            (ctx->frame.capValidMask | ctx->frame.measurement.validMask) :
            (adsFrame ? ctx->frame.measurement.validMask : ctx->frame.capValidMask);
        uint64_t errorMask = mixedFrame ?
            (ctx->frame.errorMask | ctx->frame.measurement.errorMask) :
            (adsFrame ? ctx->frame.measurement.errorMask : ctx->frame.errorMask);
        bool allInvalid = validMask == 0u;
        if (allInvalid) {
            bool rawAllZero = !adsFrame && !mixedFrame &&
                              sensorarrayFrameRawAllZero(&ctx->frame);
            if (ctx->asyncLogReady) {
                (void)sensorarrayAsyncLogPublishFrameError(&ctx->frame,
                                                           err,
                                                           rawAllZero,
                                                           ctx->fdcBootSweepOk);
            } else if (adsFrame || mixedFrame) {
                printf("ADSFRAME,stage=all_invalid,mode=%s,seq=%lu,valid=0x%016llX,error=0x%016llX,fresh=0x%016llX,err=0x%lx,timeout=%lu,stale=%lu,spi=%lu,status=%lu,durationUs=%llu\n",
                       sensorarrayMeasurementModeName(frameMode),
                       (unsigned long)ctx->frame.sequence,
                       (unsigned long long)validMask,
                       (unsigned long long)errorMask,
                       (unsigned long long)ctx->frame.measurement.freshMask,
                       (unsigned long)err,
                       (unsigned long)ctx->frame.measurement.drdyTimeoutCount,
                       (unsigned long)ctx->frame.measurement.staleCount,
                       (unsigned long)ctx->frame.measurement.spiErrorCount,
                       (unsigned long)ctx->frame.measurement.statusErrorCount,
                       (unsigned long long)ctx->frame.measurement.frameDurationUs);
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
            bool adsIoError = adsFrame &&
                (ctx->frame.measurement.drdyTimeoutCount != 0u ||
                 ctx->frame.measurement.staleCount != 0u ||
                 ctx->frame.measurement.spiErrorCount != 0u ||
                 ctx->frame.measurement.statusErrorCount != 0u);
            if (ctx->asyncLogReady) {
                if (!adsFrame || adsIoError) {
                    (void)sensorarrayAsyncLogPublishFrameError(&ctx->frame,
                                                               err,
                                                               false,
                                                               ctx->fdcBootSweepOk);
                }
            } else if (adsFrame) {
                printf("ADSFRAME,stage=partial,mode=%s,seq=%lu,valid=0x%016llX,error=0x%016llX,err=0x%lx,gainChanges=%lu,overrange=%lu,attempts=%lu,durationUs=%llu\n",
                       sensorarrayMeasurementModeName(frameMode),
                       (unsigned long)ctx->frame.sequence,
                       (unsigned long long)validMask,
                       (unsigned long long)errorMask,
                       (unsigned long)err,
                       (unsigned long)ctx->frame.measurement.gainChangeCount,
                       (unsigned long)ctx->frame.measurement.overrangeCount,
                       (unsigned long)ctx->frame.measurement.autorangeAttemptCount,
                       (unsigned long long)ctx->frame.measurement.frameDurationUs);
            } else {
                ESP_LOGE("SensorArray",
                         "FRAME_ERROR,err=0x%lx,validMask=0x%016llX,errorMask=0x%016llX",
                         (unsigned long)err,
                         (unsigned long long)validMask,
                         (unsigned long long)errorMask);
            }
        }

        if (ctx->asyncLogReady) {
            esp_err_t snapshotErr = sensorarrayAsyncLogPublishFrameSnapshot(
                &ctx->frame, measureFrameUs);
            if (snapshotErr == ESP_ERR_INVALID_SIZE) {
                sensorarrayEmitPayloadTooLargeDrop(ctx->frame.sequence);
            }
        } else if (ctx->legacySyncOutput &&
                   (ctx->frame.freshFrame || CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG)) {
            (void)sensorarrayFrameOutputPrint(&ctx->frame);
        }
        sensorarrayRuntimeRescueTick(ctx);
        sensorarrayRuntimeI2cFallbackTick(ctx);
        sensorarrayDelayFramePeriodSince(ctx,
                                         frameStartUs,
                                         ctx->frame.sequence,
                                         frameMode);
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
        if (sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
            sensorarrayBootBreadcrumbSetStage("auto_restart", initErr, ctx);
            sensorarrayBootEmitRestartMarker(
                "RAPP,cmd=RECOVER,id=0,level=2,state=restarting,kind=auto\n");
            esp_restart();
        }
        sensorarrayBootBreadcrumbSetStage("recovery_safe_idle", initErr, ctx);
        printf("APP_FATAL,stage=acquisition_init,err=%ld,action=recovery_safe_no_restart\n",
               (long)initErr);
        for (;;) {
            uint32_t restartId = 0u;
            if (sensorarrayRestartTake(&restartId)) {
                char marker[96];
                (void)snprintf(marker, sizeof(marker),
                               "RAPP,cmd=RESTART,id=%lu,state=restarting,kind=manual\n",
                               (unsigned long)restartId);
                (void)sensorarrayBootPrepareRestart(
                    SENSORARRAY_BOOT_RESTART_KIND_MANUAL);
                sensorarrayBootBreadcrumbSetStage("manual_restart", ESP_OK, ctx);
                sensorarrayBootEmitRestartMarker(marker);
                esp_restart();
            }
            uint32_t recoveryLevel = SENSORARRAY_RECOVERY_LEVEL_NONE;
            uint32_t recoveryId = 0u;
            if (sensorarrayRecoveryTake(&recoveryLevel, &recoveryId)) {
                if (recoveryLevel == SENSORARRAY_RECOVERY_LEVEL_RESTART &&
                    sensorarrayBootPrepareRestart(
                        SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
                    char marker[96];
                    (void)snprintf(marker, sizeof(marker),
                                   "RAPP,cmd=RECOVER,id=%lu,level=2,state=restarting,kind=auto\n",
                                   (unsigned long)recoveryId);
                    sensorarrayBootBreadcrumbSetStage("auto_restart",
                                                      ESP_ERR_INVALID_STATE,
                                                      ctx);
                    sensorarrayBootEmitRestartMarker(marker);
                    esp_restart();
                }
                printf("RAPP,cmd=RECOVER,id=%lu,level=%u,state=safe,reason=acquisition_init_failed\n",
                       (unsigned long)recoveryId,
                       (unsigned)recoveryLevel);
            }
            vTaskDelay(pdMS_TO_TICKS(250u));
        }
    }

    sensorarrayBootBreadcrumbSetStage("boot_sweep", ESP_OK, ctx);
    sensorarrayLogRuntimeMemoryDiag("before_boot_sweep", ctx);
    esp_err_t bootErr = sensorarrayRunBootCalibration(ctx);
    sensorarrayAsyncLogEnableAcquisition();
    if (ctx->asyncLogReady) {
        boardSupportSetLogCallback(sensorarrayAsyncLogPublishTextEvent);
    }
    sensorarrayBootBreadcrumbSetStage("boot_sweep_done", bootErr, ctx);
    sensorarrayLogRuntimeMemoryDiag("after_boot_sweep", ctx);
    if (CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED &&
        (bootErr != ESP_OK || ctx->fdcBootSummary.quality != SENSORARRAY_FDC_BOOT_QUALITY_OK)) {
        ctx->fdcDiagnosticMode = true;
        sensorarrayFdcMatrixEngineSetDiagnosticMode(&ctx->fdcEngine, true);
    }

    ctx->systemReady = initErr == ESP_OK && bootErr == ESP_OK;
    if (ctx->systemReady) {
        sensorarrayBootBreadcrumb_t boot = {0};
        char readyEvent[96];
        int readyLength = 0;
        if (sensorarrayBootBreadcrumbCopy(&boot)) {
            readyLength = snprintf(readyEvent,
                                   sizeof(readyEvent),
                                   "READY,bootId=%lu,boot=%lu,state=ok,quality=%s\n",
                                   (unsigned long)boot.bootId,
                                   (unsigned long)boot.bootCount,
                                   sensorarrayAppFdcBootQualityName(
                                       ctx->fdcBootSummary.quality));
        } else {
            readyLength = snprintf(readyEvent,
                                   sizeof(readyEvent),
                                   "READY,bootId=0,boot=0,state=ok,quality=%s\n",
                                   sensorarrayAppFdcBootQualityName(
                                       ctx->fdcBootSummary.quality));
        }
        if (readyLength > 0 && (size_t)readyLength < sizeof(readyEvent)) {
            (void)sensorarrayAsyncLogPublishProtocolEvent(
                readyEvent, (size_t)readyLength);
        }
        sensorarrayBootBreadcrumbMarkStableReady();
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
    ctx->scanTaskHandle = scanTaskHandle;
    return ESP_OK;
}

static void sensorarrayLogBootHeap(const char *stage)
{
    printf("APP_BOOT_MEM,stage=%s,internalFree=%u,internalLargest=%u,internalMin=%u\n",
           stage ? stage : SENSORARRAY_NA,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
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
    /* Keep board-support/I2C diagnostics on synchronous stdout during
     * hardware bring-up.  The acquisition-side event path remains
     * asynchronous; the board callback is installed after FDC boot
     * calibration so startup bus probing keeps the known-good timing. */

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
    sensorarrayBoardLogGpioIsrSummary();

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

    /* The transport is live as soon as sensorarrayNetStatusInit() runs, but
     * the full acquisition runtime callback is only installed after hardware
     * bring-up.  Keep the pre-acquisition window recovery-safe: BOOT/STATE
     * remain queryable and RECOVER/RESTART remain receivable without
     * touching measurement hardware.  The full callback replaces this one
     * from sensorarrayInitFrontends().  ROWMODES uses a two-phase acceptance:
     * the command is handed to Core 1 only after its RMACK publish succeeds,
     * while a failed publish cancels the reservation before it can apply. */
    sensorarrayTransportSetRuntimeQueryCallback(
        sensorarrayEarlyRuntimeQueryCommand, &s_appContext);
    sensorarrayTransportSetControlReplyPublishedCallback(
        sensorarrayControlReplyPublished);
    sensorarrayTransportSetControlReplyFailedCallback(
        sensorarrayControlReplyFailed);

    /* BLE controller memory must be reserved before Wi-Fi, board drivers,
     * frame buses, and acquisition worker stacks consume internal RAM. */
    esp_err_t netErr = sensorarrayNetStatusInit();
    if (netErr != ESP_OK && netErr != ESP_ERR_NOT_SUPPORTED) {
        printf("NET_WARN,stage=ble_first_init,err=0x%lx,name=%s,action=continue_acquisition\n",
               (unsigned long)netErr, esp_err_to_name(netErr));
    }
    sensorarrayLogBootHeap("after_net_init");

    /* Reserve the Core 0 output/logger tasks before the large Core 1
     * acquisition task.  The logger owns the bounded frame workspace and
     * must be present for STK50/full-HIL telemetry; creating it after the
     * acquisition stack can leave no suitable internal heap block even when
     * the aggregate free heap appears sufficient.  sensorarrayInitSystem()
     * calls the same initializer idempotently after its runtime reset. */
    sensorarrayInitAsyncLogging(&s_appContext);
    sensorarrayLogBootHeap("after_async_log_init");
    /* sensorarrayInitRuntime() deliberately resets the shared application
     * context in the acquisition task.  Keep the result of the pre-created
     * logger in a local value so that a fast Core 1 start cannot make Core 0
     * skip the USB sink initialization below. */
    bool asyncLogStartedBeforeScan = s_appContext.asyncLogReady;

    esp_err_t scanErr = sensorarrayStartScanTask(&s_appContext);
    if (scanErr != ESP_OK) {
        sensorarrayBootBreadcrumbSetStage("scan_task_create_failed", scanErr, &s_appContext);
        printf("APP_FATAL,stage=scan_task_create,err=0x%lx,action=safe_idle_no_restart\n",
               (unsigned long)scanErr);
        while (true) {
            uint32_t restartId = 0u;
            if (sensorarrayRestartTake(&restartId)) {
                char marker[96];
                (void)snprintf(marker, sizeof(marker),
                               "RAPP,cmd=RESTART,id=%lu,state=restarting,kind=manual\n",
                               (unsigned long)restartId);
                (void)sensorarrayBootPrepareRestart(
                    SENSORARRAY_BOOT_RESTART_KIND_MANUAL);
                sensorarrayBootBreadcrumbSetStage("manual_restart", ESP_OK, NULL);
                sensorarrayBootEmitRestartMarker(marker);
                esp_restart();
            }
            uint32_t recoveryLevel = SENSORARRAY_RECOVERY_LEVEL_NONE;
            uint32_t recoveryId = 0u;
            if (sensorarrayRecoveryTake(&recoveryLevel, &recoveryId)) {
                if (recoveryLevel == SENSORARRAY_RECOVERY_LEVEL_RESTART &&
                    sensorarrayBootPrepareRestart(
                        SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
                    char marker[96];
                    (void)snprintf(marker, sizeof(marker),
                                   "RAPP,cmd=RECOVER,id=%lu,level=2,state=restarting,kind=auto\n",
                                   (unsigned long)recoveryId);
                    sensorarrayBootBreadcrumbSetStage("auto_restart",
                                                      ESP_ERR_INVALID_STATE,
                                                      NULL);
                    sensorarrayBootEmitRestartMarker(marker);
                    esp_restart();
                }
                printf("RAPP,cmd=RECOVER,id=%lu,level=%u,state=safe,reason=scan_task_create_failed\n",
                       (unsigned long)recoveryId,
                       (unsigned)recoveryLevel);
            }
            vTaskDelay(pdMS_TO_TICKS(1000u));
        }
    }
    printf("APP_SCAN_TASK,stage=created,targetCore=%d,priority=%u,stackBytes=%u,stackMemory=internal\n",
           CONFIG_SENSORARRAY_SCAN_TASK_CORE,
           (unsigned)CONFIG_SENSORARRAY_SCAN_TASK_PRIO,
           (unsigned)CONFIG_SENSORARRAY_SCAN_TASK_STACK
    );
    sensorarrayLogBootHeap("after_scan_task_create");

    if (asyncLogStartedBeforeScan) {
        esp_err_t logSinkErr = sensorarrayAsyncLogStartUsbSink();
        if (logSinkErr != ESP_OK) {
            printf("APP_LOG_WARN,stage=usb_sink,err=0x%lx,action=continue_network_logging\n",
                   (unsigned long)logSinkErr);
        }
    }
    sensorarrayLogBootHeap("after_async_log_sink_init");
}
