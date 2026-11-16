#include "sensorarrayUsbOtg.h"
#include "control/sensorarrayUsbControl.h"
#include "control/sensorarrayLinkDiagnostics.h"
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
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "boardSupport.h"
#include "tmuxSwitch.h"

#include "sensorarrayAdsMatrix.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayAdsFault.h"
#include "sensorarrayFrontendStartup.h"
#include "sensorarrayFrontendPresence.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayCommandMailbox.h"
#include "sensorarrayCalibration.h"
#include "sensorarrayEarlyRecovery.h"
#include "sensorarrayRecoveryControl.h"
#include "sensorarrayRestartControl.h"
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
#include "sensorarraySerialIo.h"
#include "sensorarrayWorkBudget.h"
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
    bool hardwareQuarantineReported;
    bool frontendStartupReached;
    bool frontendFdcSdKnownLow;
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
static sensorarrayFrontendPresence_t s_frontendPresence;
static uint32_t s_frontendHold;
static uint32_t s_frontendCommandsReady;
static bool s_frontendRouteWaitEntered;
static bool s_frontendRestartAttempted;

static bool sensorarrayFrontendsHeld(void)
{
    return __atomic_load_n(&s_frontendHold, __ATOMIC_ACQUIRE) != 0u;
}

static esp_err_t sensorarrayFrontendRowsRequest(uint8_t rows,
    sensorarrayScanConfigRequestResult_t *outResult)
{
    if (!__atomic_load_n(&s_frontendCommandsReady, __ATOMIC_ACQUIRE) ||
        sensorarrayFrontendsHeld()) {
        return ESP_ERR_INVALID_STATE;
    }
    return sensorarrayCommandMailboxPostRows(rows, outResult);
}
static sensorarrayConfigIdentity_t sensorarrayPublishConfigAuthority(sensorarrayAppContext_t *ctx, uint32_t frameBoundary);
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

#define SENSORARRAY_ROW_MODES_TERMINAL_TEXT_MAX 384u

#define SENSORARRAY_BOOT_BREADCRUMB_MAGIC 0x53414252u
#define SENSORARRAY_BOOT_BREADCRUMB_VERSION 2u
#define SENSORARRAY_BOOT_BREADCRUMB_STAGE_MAX 32u
#define SENSORARRAY_BOOT_RESTART_KIND_NONE 0u
#define SENSORARRAY_BOOT_RESTART_KIND_MANUAL 1u
#define SENSORARRAY_BOOT_RESTART_KIND_AUTO 2u
#define SENSORARRAY_BOOT_GUARD_NORMAL 0u
#define SENSORARRAY_BOOT_GUARD_RECOVERY_SAFE 1u
#define SENSORARRAY_BOOT_AUTO_RESTART_MAX 3u
#define SENSORARRAY_BOOT_STABLE_PROGRESS_US (120ULL * 1000000ULL)
#define SENSORARRAY_BOOT_STABLE_PROGRESS_MAX_GAP_US (30ULL * 1000000ULL)
#define SENSORARRAY_BOOT_STABLE_PROGRESS_MIN_FRAMES 128u

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
#define SENSORARRAY_PROTOCOL_VERSION 2u
static uint64_t s_configSessionId;
#define SENSORARRAY_ADS_RECOVERY_CHECK_SAMPLES 1u
#define SENSORARRAY_ADS_RECOVERY_ATTEMPT_DELAY_MS 20u


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
    const sensorarrayConfigIdentity_t *identity = &ctx->frame.identity;
    bool frameContext =
        (event->stage == SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE ||
         event->stage == SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION) &&
        event->seq == ctx->frame.sequence && identity->revision != 0u &&
        identity->sessionId == s_configSessionId;
    event->profile = NULL; /* Unknown unless backed by persistent frame storage. */
    if (frameContext) {
        /* These are three independent configuration domains. In particular,
         * rowGen/rowReq are ROWS, never ROWMODES or an auxiliary request ID. */
        event->modeGeneration = identity->modeGeneration;
        event->mode = ctx->frame.mixedProfile ? "MIXED" :
            sensorarrayMeasurementModeName(identity->mode);
        event->profileGeneration = identity->profileGeneration;
        event->rowGeneration = identity->rowsGeneration;
        event->rowRequestId = identity->rowsRequestId;
        event->profile = identity->profile; /* Frame-owned, consumed synchronously. */
    } else {
        sensorarrayMeasurementModeSnapshot_t mode = {0};
        if (sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode)) {
            event->modeGeneration = mode.generation;
            event->mode = sensorarrayMeasurementModeName(mode.activeMode);
        }
        sensorarrayRowModeProfile_t profile = {0};
        if (sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile)) {
            event->profileGeneration = profile.generation;
        }
        sensorarrayFrameConfigSnapshot_t rows = sensorarrayScanConfigGetFrameSnapshot();
        event->rowGeneration = rows.generation;
        event->rowRequestId = rows.requestId;
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
        event->avddUv = ctx->frame.measurement.avddUv;
        event->avssUv = ctx->frame.measurement.avssUv;
        event->railSpanUv = (int32_t)((int64_t)event->avddUv - event->avssUv);
        event->railSource = sensorarrayAdsRailSourceName(ctx->frame.adsGap.railSource);
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
    sensorarrayAdsMatrixFaultContext_t captured = {0};
    if ((event->stage == SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE ||
         event->stage == SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION) &&
        sensorarrayAdsMatrixEngineCopyFaultContext(&ctx->adsEngine, &captured) &&
        captured.sequence == event->seq) {
        event->context = &captured;
        event->stage = captured.stage;
        event->err = captured.originalError;
    }
    (void)sensorarrayAdsFaultEmit(
        event,
        (uint64_t)esp_timer_get_time(),
        sensorarrayAdsFaultSink,
        NULL);
    event->context = NULL;
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
    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
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

static void sensorarrayBootBreadcrumbMarkStableProgress(bool healthyFrame)
{
    static uint64_t healthySinceUs;
    static uint64_t lastHealthyUs;
    static uint32_t healthyFrames;
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (!healthyFrame) {
        healthySinceUs = 0u;
        lastHealthyUs = 0u;
        healthyFrames = 0u;
        return;
    }
    /* This only qualifies a restart-budget reset. A long/slow/paused scan
     * does not become a fault, trigger a watchdog or alter capture pacing. */
    if (healthyFrames == 0u || nowUs < lastHealthyUs ||
        nowUs - lastHealthyUs > SENSORARRAY_BOOT_STABLE_PROGRESS_MAX_GAP_US) {
        healthySinceUs = nowUs;
        lastHealthyUs = nowUs;
        healthyFrames = 1u;
        return;
    }
    lastHealthyUs = nowUs;
    if (healthyFrames < UINT32_MAX) {
        healthyFrames++;
    }
    if (nowUs - healthySinceUs < SENSORARRAY_BOOT_STABLE_PROGRESS_US ||
        healthyFrames < SENSORARRAY_BOOT_STABLE_PROGRESS_MIN_FRAMES) {
        return;
    }
    healthySinceUs = nowUs;
    healthyFrames = 1u;
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
                nowUs < windowStartUs) {
                s_bootBreadcrumb.autoRestartWindowStartUs = nowUs;
            }
            /* A slow fault loop must not regain budget just by spending
             * 30 seconds booting. Only real healthy frame progress, power-on
             * or a power-on reset can clear the chain. */
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
    if (sensorarraySerialIoQueue(marker, strlen(marker), true) == ESP_OK) {
        /* Give the sole USB owner one bounded TX window. This does not
         * establish host receipt; session/BOOT queries recover lost markers. */
        vTaskDelay(pdMS_TO_TICKS(20u) + 1u);
    }
}

static bool sensorarrayRecoveryPost(uint32_t level, uint32_t *outRequestId)
{
    sensorarrayConfigAuthority_t authority = {0};
    bool known = sensorarrayConfigIdentityCopy(&authority);
    return sensorarrayRecoveryControlPost(level, &authority, known,
                                          outRequestId) == ESP_OK;
}

static bool sensorarrayRecoveryTake(uint32_t *outLevel, uint32_t *outRequestId)
{
    sensorarrayConfigAuthority_t authority = {0};
    bool known = sensorarrayConfigIdentityCopy(&authority);
    sensorarrayRecoverRequest_t request = {0};
    sensorarrayRecoverTake_t result = sensorarrayRecoveryControlTake(
        &authority, known, &request);
    if (result == SENSORARRAY_RECOVER_TAKE_STALE) {
        /* A stale request must not stop or relabel the newer configuration. */
        (void)sensorarrayRecoveryControlComplete(request.id,
            SENSORARRAY_RECOVER_REJECTED, "stale_config", ESP_ERR_INVALID_STATE,
            ESP_OK, &authority, known, false, false, false);
    }
    if (outLevel) { *outLevel = result == SENSORARRAY_RECOVER_TAKE_READY ?
        request.level : SENSORARRAY_RECOVERY_LEVEL_NONE; }
    if (outRequestId) { *outRequestId = request.id; }
    return result == SENSORARRAY_RECOVER_TAKE_READY;
}

static bool sensorarrayRestartPost(uint32_t *outRequestId)
{
    sensorarrayConfigAuthority_t authority = {0};
    bool known = sensorarrayConfigIdentityCopy(&authority);
    return sensorarrayRestartControlPost(&authority, known, outRequestId) == ESP_OK;
}

static bool sensorarrayRestartTake(uint32_t *outRequestId)
{
    return sensorarrayRestartControlTake(outRequestId);
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

    /* ADS has not been identified yet. Do not short its reset-default
     * INTREF with Q1. U5 has no enable control, so power-up analogue safety
     * requires hardware isolation; this is only a known GPIO request. */
    esp_err_t tmuxErr = tmuxSwitchSet1108Source(TMUX1108_SOURCE_REF);
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
    sensorarrayScanConfigSetRequestCallback(sensorarrayFrontendRowsRequest);

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
    sensorarrayRowModeProfile_t nextProfile = {0};
    if (sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &nextProfile) &&
        (nextProfile.pending || !sensorarrayRowModeProfileIsHomogeneous(nextProfile.modes))) {
        /* The pending profile's actual active rows decide rail requirements
         * in RunOneFrame. Do not refresh an outgoing VOLT configuration. */
        return ESP_OK;
    }

    sensorarrayAdsRailSplit_t rail = {0};
    if (sensorarrayBuildAdsRailSplit(ctx, &rail)) {
        return ESP_OK;
    }

    if (!sensorarrayBoardMapHasIsolatedRailMonitor()) {
        printf("RAIL_UNAVAILABLE,seq=%lu,capability=unavailable,reason=no_analog_isolation,action=external_calibration_required\n",
               (unsigned long)(ctx->hostFrameSequence + 1u));
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* A future hardware profile must provide a genuinely isolated monitor.
     * The current board rejects above without touching REFOUT.  Re-apply the existing VOLT route at this
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

/* 本轮新增: only the acquisition owner publishes this atomic authority. */
static sensorarrayConfigIdentity_t sensorarrayPublishConfigAuthority(sensorarrayAppContext_t *ctx, uint32_t frameBoundary)
{
    sensorarrayConfigIdentity_t identity = {.sessionId = s_configSessionId};
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    sensorarrayRowModeProfile_t profile = {0};
    sensorarrayBootBreadcrumb_t boot = {0};
    bool bootKnown = sensorarrayBootBreadcrumbCopy(&boot);
    if (bootKnown) { identity.bootId = boot.bootId; }
    if (!ctx || !sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        !sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile) ||
        !bootKnown) {
        return identity;
    }
    sensorarrayFrameConfigSnapshot_t rows = sensorarrayScanConfigGetFrameSnapshot();
    sensorarrayConfigIdentityCapture(s_configSessionId, boot.bootId, &mode, &rows,
                                     &profile, frameBoundary, &identity);
    sensorarrayConfigIdentityPublish(&identity, &mode, &profile, &rows);
    return identity;
}

static void sensorarrayCaptureFrameIdentity(sensorarrayAppContext_t *ctx,
    const sensorarrayMeasurementModeSnapshot_t *mode,
    const sensorarrayRowModeProfile_t *profile,
    uint32_t frameBoundary, sensorarrayConfigIdentity_t *outIdentity)
{
    sensorarrayRowModeProfile_t candidate = *profile;
    if (candidate.pending) {
        memcpy(candidate.modes, candidate.pendingModes, sizeof(candidate.modes));
        candidate.generation++;
        if (candidate.generation == 0u) { candidate.generation = 1u; }
        candidate.appliedRequestId = candidate.pendingRequestId;
        candidate.state = SENSORARRAY_ROW_PROFILE_STATE_APPLIED;
    }
    sensorarrayBootBreadcrumb_t boot = {0};
    if (sensorarrayBootBreadcrumbCopy(&boot)) {
        sensorarrayConfigIdentityCapture(s_configSessionId, boot.bootId, mode,
            &ctx->scanPlan.configSnapshot, &candidate, frameBoundary, outIdentity);
    }
}

static void sensorarrayEmitConfigTerminal(sensorarrayAppContext_t *ctx,
                                          uint32_t requestId, uint32_t frameBoundary, const char *format, ...);

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
    sensorarrayFdcSweepDiscardPendingForConfigChange();

    sensorarrayAdsRailSplit_t rail = {0};
    const sensorarrayAdsRailSplit_t *railPtr = NULL;
    esp_err_t err = ESP_OK;
    bool railAvailable = sensorarrayBuildAdsRailSplit(ctx, &rail);
    if (targetMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE && !railAvailable) {
        /* No transparent INTREF excursion is possible on circuit.pdf V1.0.
         * RAILCFG supplies independent external AVDD/AVSS measurements. */
        uint64_t railTransitionUs = 0u;
        err = sensorarrayRouteControllerEnterSafeRailMonitor(
            &ctx->routeController, &railTransitionUs);
        if (err == ESP_OK) {
            err = sensorarrayAdsGapRefreshRailAtBoundary(
                &ctx->state, ctx->hostFrameSequence + 1u);
        }
        if (err == ESP_OK) {
            railAvailable = sensorarrayBuildAdsRailSplit(ctx, &rail);
        }
        if (err == ESP_OK && (!railAvailable || !rail.valid)) {
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
        if (emitEvent || requestId == 0u) {
            (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
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
    if (!emitEvent && requestId == 0u) { (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u); }
    return ESP_OK;
}

static void sensorarrayEmitConfigTerminal(sensorarrayAppContext_t *ctx,
                                          uint32_t requestId, uint32_t frameBoundary,
                                          const char *format, ...)
{
    sensorarrayConfigIdentity_t id = sensorarrayPublishConfigAuthority(ctx, frameBoundary);
    char text[SENSORARRAY_ROW_MODES_TERMINAL_TEXT_MAX];
    va_list args;
    va_start(args, format);
    int written = vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    size_t length = written > 0 && (size_t)written < sizeof(text) ? (size_t)written : 0u;
    if (length > 0u && text[length - 1u] == '\n') { length--; }
    int identityLength = length > 0u ? snprintf(text + length, sizeof(text) - length,
        ",session=%016llX,bootId=%lu,rev=%lu,boundary=%lu,identityKnown=%u\n",
        (unsigned long long)id.sessionId, (unsigned long)id.bootId,
        (unsigned long)id.revision, (unsigned long)id.appliedFrameBoundary,
        id.revision != 0u ? 1u : 0u) : -1;
    if (identityLength <= 0 || (size_t)identityLength >= sizeof(text) - length) {
        /* Query is authoritative; formatting trouble must not invent a
         * rejection after hardware has successfully applied the request. */
        written = snprintf(text, sizeof(text),
            "%s,id=%lu,state=outcome_query_required,reason=terminal_too_long,session=%016llX,rev=%lu\n",
            strncmp(format, "RM", 2u) == 0 ? "RMERR" : (format[0] == 'R' ? "RERR" : "MERR"),
            (unsigned long)requestId, (unsigned long long)id.sessionId,
            (unsigned long)id.revision);
        length = written > 0 && (size_t)written < sizeof(text) ? (size_t)written : 0u;
    } else {
        length += (size_t)identityLength;
    }
    esp_err_t err = sensorarrayCommandMailboxEmitConfigTerminal(requestId, text, length);
    if (err != ESP_OK) {
        printf("CONFIG_TERM_DROP,id=%lu,err=0x%lx,query=CONFIG\n",
               (unsigned long)requestId, (unsigned long)err);
    }
}

static void sensorarrayRejectHeldCommand(sensorarrayAppContext_t *ctx,
                                         const sensorarrayCommand_t *command)
{
    const char *terminal = NULL;
    if (command->type == SENSORARRAY_COMMAND_ROWS) {
        sensorarrayScanConfigRecordRejected(command->requestId,
            (uint32_t)ESP_ERR_INVALID_STATE);
        terminal = "RERR";
    } else if (command->type == SENSORARRAY_COMMAND_MEASUREMENT_MODE) {
        sensorarrayMeasurementModeRejectRequest(&ctx->measurementMode,
            command->requestId, (uint32_t)ESP_ERR_INVALID_STATE);
        terminal = "MERR";
    } else if (command->type == SENSORARRAY_COMMAND_ROW_MODES) {
        sensorarrayRowModeProfileRejectRequest(&ctx->rowModeProfile,
            command->requestId, (uint32_t)ESP_ERR_INVALID_STATE);
        terminal = "RMERR";
    }
    if (terminal) {
        sensorarrayEmitConfigTerminal(ctx, command->requestId,
            ctx->hostFrameSequence + 1u,
            "%s,id=%lu,seq=%lu,state=rejected,err=0x%lx,reason=wait_frontends\n",
            terminal, (unsigned long)command->requestId,
            (unsigned long)(ctx->hostFrameSequence + 1u),
            (unsigned long)ESP_ERR_INVALID_STATE);
    } else {
        printf("CERR,cmd=%s,id=%lu,reason=wait_frontends,restartRequired=1\n",
            sensorarrayCommandMailboxTypeName(command->type),
            (unsigned long)command->requestId);
    }
}

static void sensorarrayApplyPendingCommands(sensorarrayAppContext_t *ctx)
{
    if (!ctx) {
        return;
    }

    sensorarrayCommand_t command;
    while (sensorarrayCommandMailboxTryReceive(&command)) {
        if (sensorarrayFrontendsHeld()) {
            sensorarrayRejectHeldCommand(ctx, &command);
            continue;
        }
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
        case SENSORARRAY_COMMAND_ROWS:
        {
            sensorarrayScanConfigApplyResult_t applied = {0};
            esp_err_t rowsErr = sensorarrayScanConfigStageAcceptedRows((uint8_t)command.value,
                                                                       command.requestId);
            if (rowsErr == ESP_OK) {
                sensorarrayScanConfigApplyPendingAtFrameBoundary(&applied);
                if (!applied.applied || applied.requestId != command.requestId) { rowsErr = ESP_ERR_INVALID_STATE; }
            }
            if (rowsErr == ESP_OK) {
                sensorarrayEmitConfigTerminal(ctx, command.requestId, ctx->hostFrameSequence + 1u,
                    "RAPP,id=%lu,seq=%lu,old=%u,new=%u,gen=%lu,status=applied\n",
                    (unsigned long)command.requestId, (unsigned long)(ctx->hostFrameSequence + 1u),
                    (unsigned)applied.oldRows, (unsigned)applied.newRows, (unsigned long)applied.generation);
            } else {
                sensorarrayScanConfigRecordRejected(command.requestId, (uint32_t)rowsErr);
                sensorarrayEmitConfigTerminal(ctx, command.requestId, ctx->hostFrameSequence + 1u,
                    "RERR,id=%lu,seq=%lu,status=rejected,err=0x%lx\n",
                    (unsigned long)command.requestId, (unsigned long)(ctx->hostFrameSequence + 1u),
                    (unsigned long)rowsErr);
            }
            break;
        }
        case SENSORARRAY_COMMAND_MEASUREMENT_MODE:
        {
            sensorarrayMeasurementMode_t requested = (sensorarrayMeasurementMode_t)command.value;
            sensorarrayMeasurementModeSnapshot_t before = {0};
            sensorarrayRowModeProfile_t profile = {0};
            sensorarrayRouteSnapshot_t route = {0};
            bool modeKnown = sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &before);
            bool profileKnown = sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile);
            bool routeKnown = sensorarrayRouteControllerCopySnapshot(&ctx->routeController, &route);
            esp_err_t modeErr = ESP_ERR_INVALID_STATE;
            sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
            for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) { modes[row] = requested; }
            if (!modeKnown || !profileKnown || !routeKnown || profile.pending || route.fdcSdHigh ||
                !sensorarrayRowModeProfileAccept(&ctx->rowModeProfile, modes, command.requestId)) {
                sensorarrayMeasurementModeRejectRequest(&ctx->measurementMode,
                    command.requestId, (uint32_t)modeErr);
            } else {
                modeErr = sensorarrayApplyMeasurementMode(ctx, requested, command.requestId, false);
                if (modeErr == ESP_OK) {
                    if (!sensorarrayRowModeProfileCompleteTransition(&ctx->rowModeProfile,
                            ctx->hostFrameSequence + 1u, 0u)) {
                        modeErr = ESP_ERR_INVALID_STATE;
                        sensorarrayMeasurementModeRecordRuntimeFault(&ctx->measurementMode, (uint32_t)modeErr);
                        sensorarrayMeasurementModeRejectRequest(&ctx->measurementMode, command.requestId, (uint32_t)modeErr);
                        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController, "mode_profile_commit");
                    }
                }
                if (modeErr != ESP_OK) {
                    sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile, (uint32_t)modeErr, 0u);
                }
            }
            sensorarrayMeasurementModeSnapshot_t after = {0};
            (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &after);
            if (modeErr == ESP_OK) {
                sensorarrayEmitConfigTerminal(ctx, command.requestId, ctx->hostFrameSequence + 1u,
                    "MAPP,id=%lu,gen=%lu,old=%s,new=%s,seq=%lu,state=applied,transitionUs=%llu\n",
                    (unsigned long)command.requestId, (unsigned long)after.generation,
                    sensorarrayMeasurementModeName(before.activeMode), sensorarrayMeasurementModeName(requested),
                    (unsigned long)after.appliedFrameSequence, (unsigned long long)after.transitionDurationUs);
            } else {
                sensorarrayEmitConfigTerminal(ctx, command.requestId, ctx->hostFrameSequence + 1u,
                    "MERR,id=%lu,old=%s,new=%s,seq=%lu,state=rejected,err=0x%lx,transitionUs=%llu\n",
                    (unsigned long)command.requestId, sensorarrayMeasurementModeName(before.activeMode),
                    sensorarrayMeasurementModeName(requested), (unsigned long)(ctx->hostFrameSequence + 1u),
                    (unsigned long)modeErr, (unsigned long long)after.transitionDurationUs);
            }
            continue;
        }
        case SENSORARRAY_COMMAND_ROW_MODES: {
            sensorarrayFdcSweepDiscardPendingForConfigChange();
            sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
            if (!sensorarrayRowModeProfileParse(command.rowModes,
                                                SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH,
                                                modes)) {
                sensorarrayRowModeProfileRejectRequest(&ctx->rowModeProfile,
                    command.requestId, (uint32_t)ESP_ERR_INVALID_ARG);
                sensorarrayEmitConfigTerminal(ctx,
                    command.requestId, ctx->hostFrameSequence + 1u,
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
                sensorarrayRowModeProfileRejectRequest(&ctx->rowModeProfile,
                    command.requestId, (uint32_t)ESP_ERR_INVALID_STATE);
                sensorarrayEmitConfigTerminal(ctx,
                    command.requestId, ctx->hostFrameSequence + 1u,
                    "RMERR,id=%lu,profile=%s,state=rejected,err=0x%lx,reason=fdc_restart_required\n",
                    (unsigned long)command.requestId,
                    command.rowModes,
                    (unsigned long)ESP_ERR_INVALID_STATE);
                break;
            }
            if (!sensorarrayRowModeProfileAccept(&ctx->rowModeProfile,
                                                 modes,
                                                 command.requestId)) {
                sensorarrayRowModeProfileRejectRequest(&ctx->rowModeProfile,
                    command.requestId, (uint32_t)ESP_ERR_INVALID_STATE);
                sensorarrayEmitConfigTerminal(ctx,
                    command.requestId, ctx->hostFrameSequence + 1u,
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

static bool sensorarrayFrontendOwnersQuiescent(const sensorarrayAppContext_t *ctx)
{
    /* Check boot publication first: no live driver fields may be read while
     * that task still owns them. All remaining callers are frame boundaries. */
    return ctx && sensorarrayFrontendStartupAdsOwnerQuiescent() &&
        !sensorarrayBringupAdsPresenceBusy() &&
        !sensorarrayBringupAdsPresenceQuarantined() &&
        sensorarrayMeasureFdcWorkersQuiescent() &&
        !sensorarrayMeasureFdcWorkersQuarantined() &&
        !ads126xAdcIsQuarantined(&ctx->state.ads) &&
        !ctx->state.ads.spiInFlight;
}

static void sensorarrayEnterFrontendHold(sensorarrayAppContext_t *ctx,
                                         const char *reason)
{
    __atomic_store_n(&s_frontendHold, 1u, __ATOMIC_RELEASE);
    __atomic_store_n(&s_frontendCommandsReady, 0u, __ATOMIC_RELEASE);
    sensorarrayFrontendPresenceLatchHold(&s_frontendPresence,
        (uint64_t)esp_timer_get_time());
    ctx->systemReady = false;
    sensorarrayBootBreadcrumbMarkStableProgress(false);
    if (s_frontendRouteWaitEntered ||
        !sensorarrayFrontendStartupAdsOwnerQuiescent() ||
        sensorarrayBringupAdsPresenceBusy() ||
        sensorarrayBringupAdsPresenceQuarantined()) {
        return;
    }
    /* No STOP, RESET, reference enable, or clamp command to an absent ADC.
     * Releasing Q1 prevents a returning default-on INTREF being clamped, but
     * U5 is still connected: the route is explicitly UNKNOWN. */
    esp_err_t routeErr = sensorarrayRouteControllerEnterFrontendWait(
        &ctx->routeController, &ctx->state);
    if (!sensorarrayFrontendOwnersQuiescent(ctx)) {
        /* The route helper published UNKNOWN without touching hardware.
         * Do not invalidate storage that a live DMA/worker may still use. */
        return;
    }
    s_frontendRouteWaitEntered = true;
    sensorarrayAdsMatrixEngineInvalidateCaches(&ctx->adsEngine);
    (void)sensorarrayAdsMatrixEngineSetMode(
        &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
    for (unsigned row = 0u; row < SENSORARRAY_MATRIX_ROWS; ++row) {
        for (unsigned col = 0u; col < SENSORARRAY_MATRIX_COLS; ++col) {
            ctx->state.fdcCellCache[row][col].valid = false;
        }
    }
    ctx->state.fdcAppliedRow[0].valid = false;
    ctx->state.fdcAppliedRow[1].valid = false;
    sensorarrayFdcSweepDiscardPendingForConfigChange();
    sensorarrayMeasurementRecoveryInit(&ctx->adsRestoreRecovery);
    ctx->pendingAdsCheck = false;
    ctx->pendingBatteryRequestId = 0u;
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    sensorarrayRowModeProfile_t profile = {0};
    (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode);
    (void)sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile);
    if (mode.pending) {
        sensorarrayMeasurementModeFailTransition(&ctx->measurementMode,
            (uint32_t)ESP_ERR_INVALID_STATE, 0u);
    }
    if (profile.pending) {
        sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile,
            (uint32_t)ESP_ERR_INVALID_STATE, 0u);
    }
    sensorarrayMeasurementModeRecordRuntimeFault(&ctx->measurementMode,
        (uint32_t)ESP_ERR_INVALID_STATE);
    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
    if (mode.pending) {
        sensorarrayEmitConfigTerminal(ctx, mode.pendingRequestId,
            ctx->hostFrameSequence + 1u,
            "MERR,id=%lu,state=rejected,err=0x%lx,reason=wait_frontends\n",
            (unsigned long)mode.pendingRequestId,
            (unsigned long)ESP_ERR_INVALID_STATE);
    }
    if (profile.pending) {
        sensorarrayEmitConfigTerminal(ctx, profile.pendingRequestId,
            ctx->hostFrameSequence + 1u,
            "RMERR,id=%lu,state=rejected,err=0x%lx,reason=wait_frontends\n",
            (unsigned long)profile.pendingRequestId,
            (unsigned long)ESP_ERR_INVALID_STATE);
    }
    sensorarrayBootBreadcrumbSetStage("wait_frontends", routeErr, ctx);
    sensorarrayTransportPublishRuntimeFault("MFAULT,source=FRONTEND_LOSS,seq=%lu,err=0x%lx,state=DEGRADED,route=UNKNOWN,reason=%s,gpioErr=0x%lx,physicalSafe=unknown,restartRequired=1\n",
        (unsigned long)(ctx->hostFrameSequence + 1u),
        (unsigned long)ESP_ERR_INVALID_STATE, reason,
        (unsigned long)(uint32_t)routeErr);
}

static bool sensorarrayRunFrontendPresenceBoundary(sensorarrayAppContext_t *ctx)
{
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    if (sensorarrayFrontendPresenceDue(&s_frontendPresence, nowUs)) {
        sensorarrayRouteSnapshot_t route = {0};
        bool sdKnownLow = ctx->frontendFdcSdKnownLow;
        if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController, &route) &&
            route.fdcSdVerified) {
            sdKnownLow = !route.fdcSdHigh;
        }
        sensorarrayFrontendPresenceSnapshot_t snapshot = {0};
        sensorarrayFrontendPresenceAction_t action = sensorarrayFrontendPresencePoll(
            &s_frontendPresence, &ctx->state, nowUs,
            sensorarrayFrontendOwnersQuiescent(ctx), !sdKnownLow, &snapshot);
        bool cleanupFailed = false;
        for (unsigned i = 0u; i < SENSORARRAY_FRONTEND_COUNT; ++i) {
            cleanupFailed |= snapshot.evidence.devices[i].cleanupError != ESP_OK;
        }
        if (action == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT ||
            sensorarrayFrontendPresenceIsWaiting(&s_frontendPresence) ||
            cleanupFailed) {
            sensorarrayEnterFrontendHold(ctx, cleanupFailed ?
                "presence_restore_failed" : "identity_unavailable_or_reset");
        }
        if (action != SENSORARRAY_FRONTEND_PRESENCE_NONE ||
            sensorarrayFrontendsHeld()) {
            printf("FRONTEND_PROBE,phase=%s,round=%lu,known=%u,present=%u,absentRun=%lu,presentRun=%lu,ownerFree=%u,lowSpeed=%u,startUs=%llu,endUs=%llu,physicalSafe=unknown\n",
                sensorarrayFrontendPresencePhaseName(snapshot.phase),
                (unsigned long)snapshot.probes, (unsigned)snapshot.knownMask,
                (unsigned)snapshot.presentMask, (unsigned long)snapshot.absentRun,
                (unsigned long)snapshot.presentRun,
                snapshot.evidence.ownerQuiescent ? 1u : 0u,
                snapshot.evidence.lowSpeed ? 1u : 0u,
                (unsigned long long)snapshot.evidence.startedUs,
                (unsigned long long)snapshot.evidence.endedUs);
        }
        if (snapshot.probes == 1u || action != SENSORARRAY_FRONTEND_PRESENCE_NONE ||
            sensorarrayFrontendsHeld()) {
            const sensorarrayFrontendPresenceDevice_t *d = snapshot.evidence.devices;
            printf("FRONTEND_CLOCKS,round=%lu,pProbeHz=%lu,pRestoredHz=%lu,sProbeHz=%lu,sRestoredHz=%lu,aProbeHz=%lu,aRestoredHz=%lu,unknownHz=0\n",
                (unsigned long)snapshot.probes, (unsigned long)d[0].clockHz,
                (unsigned long)d[0].restoredClockHz, (unsigned long)d[1].clockHz,
                (unsigned long)d[1].restoredClockHz, (unsigned long)d[2].clockHz,
                (unsigned long)d[2].restoredClockHz);
            printf("FRONTEND_IDS,round=%lu,pMfg1=%04X,pDev1=%04X,pMfg2=%04X,pDev2=%04X,pReadMask=0x%X,pErr=0x%lX,pCleanup=0x%lX,sMfg1=%04X,sDev1=%04X,sMfg2=%04X,sDev2=%04X,sReadMask=0x%X,sErr=0x%lX,sCleanup=0x%lX\n",
                (unsigned long)snapshot.probes,
                (unsigned)d[0].manufacturerId, (unsigned)d[0].deviceId,
                (unsigned)d[0].manufacturerIdSecond, (unsigned)d[0].deviceIdSecond,
                (unsigned)d[0].idReadMask, (unsigned long)(uint32_t)d[0].error,
                (unsigned long)(uint32_t)d[0].cleanupError,
                (unsigned)d[1].manufacturerId, (unsigned)d[1].deviceId,
                (unsigned)d[1].manufacturerIdSecond, (unsigned)d[1].deviceIdSecond,
                (unsigned)d[1].idReadMask, (unsigned long)(uint32_t)d[1].error,
                (unsigned long)(uint32_t)d[1].cleanupError);
        }
        if (action == SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART &&
            !s_frontendRestartAttempted) {
            s_frontendRestartAttempted = true;
            if (sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
                sensorarrayBootBreadcrumbSetStage("frontends_restored", ESP_OK, ctx);
                sensorarrayBootEmitRestartMarker(
                    "RAPP,cmd=FRONTENDS,id=0,state=restarting,reason=stable_identity_recovered\n");
                sensorarrayUsbOtgRestart(false);
            } else {
                printf("FRONTEND_WAIT,reason=restart_budget_exhausted,route=UNKNOWN,physicalSafe=unknown,query=FRONTENDS\n");
            }
        }
    }
    if (sensorarrayFrontendsHeld()) {
        sensorarrayBootBreadcrumbMarkStableProgress(false);
        sensorarrayApplyPendingCommands(ctx);
        return true;
    }
    return false;
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
        sensorarrayAdsFaultEvent_t fault = {
            .stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK,
            .err = checkErr,
            .seq = ctx->frame.sequence,
            .outcome = SENSORARRAY_ADS_FAULT_OUTCOME_FAILED,
        };
        sensorarrayAdsFaultFillContext(ctx, &fault);
        fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_CHECK);
        fault.restoreExpectedValid = true;
        fault.restoreExpected = (int32_t)result.registers.power;
        sensorarrayAdsMatrixEngineInvalidateCaches(&ctx->adsEngine);
        (void)sensorarrayAdsMatrixEngineSetMode(
            &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
        sensorarrayMeasurementModeRecordRuntimeFault(
            &ctx->measurementMode, (uint32_t)checkErr);
        (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                   "ads_check_restore_failed");
        sensorarrayEmitAdsFault(ctx, &fault);
        sensorarrayTransportPublishRuntimeFault("MFAULT,source=ADSCHK,id=%lu,err=0x%lx,state=DEGRADED,route=SAFE_REQUESTED\n",
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
        sensorarrayAdsFaultEvent_t fault = {
            .stage = SENSORARRAY_ADS_FAULT_STAGE_BATTERY_RESTORE,
            .err = batteryErr,
            .seq = ctx->frame.sequence,
            .outcome = SENSORARRAY_ADS_FAULT_OUTCOME_FAILED,
        };
        sensorarrayAdsFaultFillContext(ctx, &fault);
        fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_BATTERY);
        sensorarrayAdsMatrixEngineInvalidateCaches(&ctx->adsEngine);
        (void)sensorarrayAdsMatrixEngineSetMode(
            &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
        sensorarrayMeasurementModeRecordRuntimeFault(
            &ctx->measurementMode, (uint32_t)batteryErr);
        (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                   "battery_restore_failed");
        sensorarrayEmitAdsFault(ctx, &fault);
        sensorarrayTransportPublishRuntimeFault("MFAULT,source=BATTERY,err=0x%lx,state=DEGRADED,route=SAFE_REQUESTED\n",
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
    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
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
        sensorarrayTransportPublishRuntimeFault("MFAULT,source=BATTERY,err=0x%lx,state=DEGRADED,route=SAFE_REQUESTED,attempt=%lu\n",
               (unsigned long)error,
               (unsigned long)attempt);
    } else {
        sensorarrayTransportPublishRuntimeFault("MFAULT,source=ADSCHK,id=%lu,err=0x%lx,state=DEGRADED,route=SAFE_REQUESTED,attempt=%lu\n",
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
    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
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

static esp_err_t sensorarrayFormatFrontendReply(char *response, size_t responseSize)
{
    sensorarrayFrontendPresenceSnapshot_t s = {0};
    (void)sensorarrayFrontendPresenceCopySnapshot(&s_frontendPresence, &s);
    const sensorarrayFrontendPresenceDevice_t *p = s.evidence.devices;
    int written = snprintf(response, responseSize,
        "FRONTENDS,phase=%s,hold=%u,probes=%lu,absentRun=%lu,presentRun=%lu,known=%u,present=%u,primary=%s,secondary=%s,ads=%s,pErr=0x%lx,sErr=0x%lx,aErr=0x%lx,pCleanup=0x%lx,sCleanup=0x%lx,aCleanup=0x%lx,aHz=%lu,aRestoreHz=%lu,ownerFree=%u,sdInhibited=%u,lowSpeed=%u,quarantine=%u,nextUs=%llu,beginUs=%llu,endUs=%llu,physicalSafe=unknown\n",
        s.initialized ? sensorarrayFrontendPresencePhaseName(s.phase) : "idle",
        sensorarrayFrontendsHeld() ? 1u : 0u,
        (unsigned long)s.probes, (unsigned long)s.absentRun,
        (unsigned long)s.presentRun, (unsigned)s.knownMask, (unsigned)s.presentMask,
        sensorarrayFrontendPresenceLinkName(p[0].link),
        sensorarrayFrontendPresenceLinkName(p[1].link),
        sensorarrayFrontendPresenceLinkName(p[2].link),
        (unsigned long)(uint32_t)p[0].error,
        (unsigned long)(uint32_t)p[1].error,
        (unsigned long)(uint32_t)p[2].error,
        (unsigned long)(uint32_t)p[0].cleanupError,
        (unsigned long)(uint32_t)p[1].cleanupError,
        (unsigned long)(uint32_t)p[2].cleanupError,
        (unsigned long)p[2].clockHz, (unsigned long)p[2].restoredClockHz,
        s.evidence.ownerQuiescent ? 1u : 0u,
        s.evidence.fdcShutdownHigh ? 1u : 0u,
        s.evidence.lowSpeed ? 1u : 0u, s.evidence.quarantined ? 1u : 0u,
        (unsigned long long)s.nextProbeUs,
        (unsigned long long)s.evidence.startedUs,
        (unsigned long long)s.evidence.endedUs);
    return written > 0 && (size_t)written < responseSize ?
        ESP_OK : ESP_ERR_INVALID_SIZE;
}

/* 本轮新增: separate bounded sink counters; these are local queue/driver
 * observations and never a statement that a PC received the bytes. */
static esp_err_t sensorarrayFormatSerialReply(char *response, size_t responseSize)
{
    if (!response || responseSize == 0u) { return ESP_ERR_INVALID_ARG; }
    sensorarraySerialIoStats_t serial = {0};
    sensorarraySerialIoGetStats(&serial);
    int n = snprintf(response, responseSize,
        "SERIAL,normalSubmitted=%lu,prioritySubmitted=%lu,dataSubmitted=%lu,logSubmitted=%lu,dataRejected=%lu,logRejected=%lu,normalExpired=%lu,priorityExpired=%lu,priorityRejected=%lu,driverShortWrite=%lu,driverWriteError=%lu,txYield=%lu,maxQueueAgeUs=%lu,driverTxBufferBytes=%lu,driverAcceptedBytes=%llu,partialWrite=%lu\n",
        (unsigned long)serial.normalSubmitted, (unsigned long)serial.prioritySubmitted,
        (unsigned long)serial.dataSubmitted, (unsigned long)serial.logSubmitted,
        (unsigned long)serial.dataRejected, (unsigned long)serial.logRejected,
        (unsigned long)serial.normalExpired, (unsigned long)serial.priorityExpired,
        (unsigned long)serial.priorityRejected, (unsigned long)serial.driverShortWrite,
        (unsigned long)serial.driverWriteError, (unsigned long)serial.txYield,
        (unsigned long)serial.maxQueueAgeUs, (unsigned long)serial.driverTxBufferBytes,
        (unsigned long long)serial.driverAcceptedBytes, (unsigned long)serial.partialWrite);
    if (n <= 0 || (size_t)n >= responseSize) { response[0] = '\0'; return ESP_ERR_INVALID_SIZE; }
    return ESP_OK;
}

/* 本轮新增: real CPU0 idle progress and bounded worker checkpoints. These
 * independently sampled counters are neither CPU load nor configuration state. */
static esp_err_t sensorarrayFormatSchedulerReply(char *response, size_t responseSize)
{
    if (!response || responseSize == 0u) { return ESP_ERR_INVALID_ARG; }
    sensorarrayWorkBudgetSnapshot_t snapshot = {0};
    sensorarrayWorkBudgetCopySnapshot(&snapshot);
    char age[11], gap[11];
    if (!snapshot.idleKnown || snapshot.idleAgeTicks == UINT32_MAX) {
        memcpy(age, "unknown", sizeof("unknown"));
    } else {
        (void)snprintf(age, sizeof(age), "%lu", (unsigned long)snapshot.idleAgeTicks);
    }
    if (!snapshot.idleKnown || snapshot.maxIdleGapTicks == UINT32_MAX) {
        memcpy(gap, "unknown", sizeof("unknown"));
    } else {
        (void)snprintf(gap, sizeof(gap), "%lu", (unsigned long)snapshot.maxIdleGapTicks);
    }
    const sensorarrayWorkBudgetWorkerSnapshot_t *w = snapshot.workers;
    int n = snprintf(response, responseSize,
        "SCHED,initError=0x%lX,registered=%u,idleKnown=%u,tickHz=%lu,budgetTicks=%lu,maxUnits=%lu,graceUnits=%lu,backstopTicks=%lu,quantumTicks=%lu,idleGen=%lu,idleAgeTicks=%s,maxIdleGapTicks=%s,bt=%lu/%lu/%lu,tr=%lu/%lu/%lu,log=%lu/%lu/%lu,ser=%lu/%lu/%lu,bc=%lu/%lu/%lu,sc=%lu/%lu/%lu,wc=%lu/%lu/%lu\n",
        (unsigned long)(uint32_t)snapshot.initError, snapshot.registered ? 1u : 0u,
        snapshot.idleKnown ? 1u : 0u, (unsigned long)snapshot.tickHz,
        (unsigned long)snapshot.idleBudgetTicks, (unsigned long)snapshot.maxWorkUnits,
        (unsigned long)snapshot.graceUnits, (unsigned long)snapshot.backstopTicks,
        (unsigned long)snapshot.localQuantumTicks,
        (unsigned long)snapshot.idleGeneration, age, gap,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_BLE_TX].yields,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_BLE_TX].maxBurstUnits,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_TRANSPORT].checkpoints,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_TRANSPORT].yields,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_TRANSPORT].maxBurstUnits,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_ASYNC_LOG].checkpoints,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_ASYNC_LOG].yields,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_ASYNC_LOG].maxBurstUnits,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_SERIAL_TX].checkpoints,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_SERIAL_TX].yields,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_SERIAL_TX].maxBurstUnits,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_BLE_CTRL].checkpoints,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_BLE_CTRL].yields,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_BLE_CTRL].maxBurstUnits,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_SERIAL_CTRL].checkpoints,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_SERIAL_CTRL].yields,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_SERIAL_CTRL].maxBurstUnits,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_WIFI_CTRL].checkpoints,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_WIFI_CTRL].yields,
        (unsigned long)w[SENSORARRAY_WORK_BUDGET_WIFI_CTRL].maxBurstUnits);
    if (n <= 0 || (size_t)n >= responseSize) { response[0] = '\0'; return ESP_ERR_INVALID_SIZE; }
    return ESP_OK;
}

/* 本轮新增: BUILD needs all 32 descriptor bytes. The panic-cache getter
 * esp_app_get_elf_sha256 is capped by CONFIG_APP_RETRIEVE_LEN_ELF_SHA even
 * when the caller supplies a 65-byte destination. This runs in query task
 * context, with the application descriptor available through normal cache. */
static esp_err_t sensorarrayFormatBuildReply(char *response, size_t responseSize)
{
    if (!response || responseSize == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    response[0] = '\0';
    const esp_app_desc_t *app = esp_app_get_description();
    if (!app) {
        return ESP_ERR_INVALID_STATE;
    }
    static const char hex[] = "0123456789abcdef";
    char elfSha[sizeof(app->app_elf_sha256) * 2u + 1u];
    for (size_t i = 0; i < sizeof(app->app_elf_sha256); ++i) {
        elfSha[2u * i] = hex[app->app_elf_sha256[i] >> 4u];
        elfSha[2u * i + 1u] = hex[app->app_elf_sha256[i] & 0x0fu];
    }
    elfSha[sizeof(elfSha) - 1u] = '\0';
    int length = snprintf(response, responseSize,
        "BUILD,idf=%s,target=%s,project=SensorArray,proto=%u,version=%.32s,elfSha256=%s\n",
        esp_get_idf_version(), CONFIG_IDF_TARGET,
        (unsigned)SENSORARRAY_PROTOCOL_VERSION, app->version, elfSha);
    if (length <= 0 || (size_t)length >= responseSize) {
        response[0] = '\0';
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
    if (strcmp(command, "RECOVERY?") == 0) {
        return sensorarrayRecoveryControlFormat(response, responseSize);
    }
    if (strcmp(command, "RESTART?") == 0) {
        return sensorarrayRestartControlFormat(response, responseSize);
    }
    if (strcmp(command, "BLELINK?") == 0 || strcmp(command, "BLETX?") == 0) {
        return sensorarrayLinkDiagnosticsFormat(command, response, responseSize);
    }
    if (sensorarrayUsbControlHandles(command)) {
        return sensorarrayUsbControlHandle(command, response, responseSize);
    }
    if (strcmp(command, "SERIAL?") == 0) {
        return sensorarrayFormatSerialReply(response, responseSize);
    }
    if (strcmp(command, "SCHED?") == 0) {
        return sensorarrayFormatSchedulerReply(response, responseSize);
    }
    if (strcmp(command, "SPI?") == 0) {
        return sensorarrayFrontendStartupFormatSpi(response, responseSize);
    }
    if (strcmp(command, "FRONTENDS?") == 0) {
        return sensorarrayFormatFrontendReply(response, responseSize);
    }
    if (strcmp(command, "BUILD?") == 0) {
        return sensorarrayFormatBuildReply(response, responseSize);
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
                 "%s,ready=0,state=%s,active=NONE,route=UNKNOWN,stage=%s,reason=acquisition_not_ready\n",
                 kind == SENSORARRAY_EARLY_KIND_STATE_QUERY ? "STATE" : "MODE",
                 sensorarrayFrontendsHeld() ? "DEGRADED" : "INIT",
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
                 "PROTO,version=%u,wires=ascii,ctrlMax=%u,dataMax=%u,channels=CTRL/DATA/LOG/LIFECYCLE,caps=config_identity_v1/config_query_v1/session64/boot_spi_qual_v1/frontend_presence_v1/recovery_query_v1/restart_query_v1/%s/work_budget_v1/work_quantum_v2,legacyGen=tag_domain,railMonitor=unavailable,adcCrc=crc8\n",
                 (unsigned)SENSORARRAY_PROTOCOL_VERSION,
                 (unsigned)SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX,
                 (unsigned)SENSORARRAY_TRANSPORT_TEXT_MAX,
                 sensorarrayUsbControlCapabilities());
        return ESP_OK;
    case SENSORARRAY_EARLY_KIND_RECOVER: {
        uint32_t requestId = 0u;
        if (!sensorarrayRecoveryPost(recoveryLevel, &requestId)) {
            snprintf(response, responseSize, "ERR,cmd=RECOVER,reason=pending\n");
            return ESP_ERR_INVALID_STATE;
        }
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
        if (!sensorarrayRestartPost(&requestId)) {
            snprintf(response, responseSize, "ERR,cmd=RESTART,reason=pending\n");
            return ESP_ERR_INVALID_STATE;
        }
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
    if (sscanf(data, "RACK,cmd=RECOVER,id=%lu", &requestId) == 1) {
        (void)sensorarrayRecoveryControlAck((uint32_t)requestId, ESP_OK);
        return;
    }
    if (sscanf(data, "RACK,cmd=RESTART,id=%lu", &requestId) == 1) {
        (void)sensorarrayRestartControlAck((uint32_t)requestId, ESP_OK);
        return;
    }
    if (sscanf(data, "RMACK,id=%lu", &requestId) == 1 ||
        sscanf(data, "MACK,id=%lu", &requestId) == 1 ||
        sscanf(data, "RCMD,id=%lu", &requestId) == 1) {
        if (!sensorarrayCommandMailboxCommitConfigAck(
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
    if (sscanf(data, "RACK,cmd=RECOVER,id=%lu", &requestId) == 1) {
        (void)sensorarrayRecoveryControlAck((uint32_t)requestId, error);
        return;
    }
    if (sscanf(data, "RACK,cmd=RESTART,id=%lu", &requestId) == 1) {
        (void)sensorarrayRestartControlAck((uint32_t)requestId, error);
        return;
    }
    if (sscanf(data, "RMACK,id=%lu", &requestId) == 1 ||
        sscanf(data, "MACK,id=%lu", &requestId) == 1 ||
        sscanf(data, "RCMD,id=%lu", &requestId) == 1) {
        if (sensorarrayCommandMailboxCancelConfigAck((uint32_t)requestId)) {
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
    if (strcmp(command, "RECOVERY?") == 0) {
        return sensorarrayRecoveryControlFormat(response, responseSize);
    }
    if (strcmp(command, "RESTART?") == 0) {
        return sensorarrayRestartControlFormat(response, responseSize);
    }
    if (strcmp(command, "BLELINK?") == 0 || strcmp(command, "BLETX?") == 0) {
        return sensorarrayLinkDiagnosticsFormat(command, response, responseSize);
    }
    if (sensorarrayUsbControlHandles(command)) {
        return sensorarrayUsbControlHandle(command, response, responseSize);
    }
    if (strcmp(command, "SERIAL?") == 0) {
        return sensorarrayFormatSerialReply(response, responseSize);
    }
    if (strcmp(command, "SCHED?") == 0) {
        return sensorarrayFormatSchedulerReply(response, responseSize);
    }
    if (strcmp(command, "SPI?") == 0) {
        return sensorarrayFrontendStartupFormatSpi(response, responseSize);
    }
    if (strcmp(command, "FRONTENDS?") == 0) {
        return sensorarrayFormatFrontendReply(response, responseSize);
    }
    if (sensorarrayFrontendsHeld() && !strchr(command, '?') &&
        strcmp(command, "RESTART") != 0 && strcmp(command, "RECOVER=2") != 0 &&
        strcmp(command, "USBSTREAM=DEBUG") != 0 &&
        strcmp(command, "USBSTREAM=FULL") != 0) {
        snprintf(response, responseSize,
            "ERR,cmd=FRONTENDS,reason=wait_frontends,restartRequired=1\n");
        return ESP_ERR_INVALID_STATE;
    }
    if (strcmp(command, "ROUTE?") == 0) {
        sensorarrayRouteSnapshot_t route = {0};
        if (!ctx || !sensorarrayRouteControllerCopySnapshot(&ctx->routeController, &route)) {
            snprintf(response, responseSize, "ERR,cmd=ROUTE,reason=snapshot_busy\n");
            return ESP_ERR_TIMEOUT;
        }
        int written = snprintf(response, responseSize,
            "ROUTE,requested=%s,mode=%s,applied=%u,known=%u,verified=%u,safe=%u,gpioReadback=%u,adsReadback=%u,analogIsolationVerified=%u,physicalSafe=unknown,originalError=0x%lX,cleanupError=0x%lX,stage=%s,railMonitor=unavailable\n",
            sensorarrayMeasurementModeName(route.requestedMode),
            sensorarrayMeasurementModeName(route.mode),
            route.applied ? 1u : 0u, route.known ? 1u : 0u,
            route.verified ? 1u : 0u, route.safe ? 1u : 0u,
            route.gpioReadbackValid ? 1u : 0u, route.adsReadbackValid ? 1u : 0u,
            route.analogIsolationVerified ? 1u : 0u,
            (unsigned long)route.lastError, (unsigned long)route.cleanupError,
            route.failureStage ? route.failureStage : "none");
        return written > 0 && (size_t)written < responseSize ? ESP_OK : ESP_ERR_INVALID_SIZE;
    }
    if (strcmp(command, "CONFIG?") == 0) {
        return sensorarrayConfigIdentityFormat(response, responseSize);
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
                 "PROTO,version=%u,wires=ascii,ctrlMax=%u,dataMax=%u,channels=CTRL/DATA/LOG/LIFECYCLE,caps=config_identity_v1/config_query_v1/session64/boot_spi_qual_v1/frontend_presence_v1/recovery_query_v1/restart_query_v1/%s/work_budget_v1/work_quantum_v2,legacyGen=tag_domain,railMonitor=unavailable,adcCrc=crc8\n",
                 (unsigned)SENSORARRAY_PROTOCOL_VERSION,
                 (unsigned)SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX,
                 (unsigned)SENSORARRAY_TRANSPORT_TEXT_MAX,
                 sensorarrayUsbControlCapabilities());
        return ESP_OK;
    }
    if (strcmp(command, "BUILD?") == 0) {
        return sensorarrayFormatBuildReply(response, responseSize);
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
                 "PERF,pub=%llu,fresh=%llu,stale=%llu,mixed=%llu,dropOut=%llu,dropEvent=%llu,startAvgUs=%llu,startN=%lu,usbStage=queued,usbSent=%lu,usbDrop=%lu,usbBlock=%lu,usbBytes=%llu,usbWriteMaxUs=%lu,usbQMax=%lu,usbStackMin=%lu,lifePub=%lu,lifeDrop=%lu,queueDrop=%lu,frames=%lu,heap=%lu,heapMin=%lu\n",
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
        sensorarraySerialIoStats_t stats = {0};
        sensorarraySerialIoGetStats(&stats);
        int written = snprintf(response,
                 responseSize,
                 "USBSTREAM,v=%s,dataEvery=%lu,diagEvery=%lu,sinkQueued=%lu,sinkDropped=%lu,priorityQueued=%lu,driverAcceptedBytes=%llu,rxBytes=%llu,partialWrite=%lu,timeout=%lu,driverDrained=%lu,rxOversize=%lu,lineDropped=%lu,lineOversize=%lu,discardBytes=%llu,lastRxUs=%lld,lastTxUs=%lld,driverInstalled=%u,sofPresent=%u,stdioBound=%u,initError=0x%lx,txStackBytes=%lu,txStackMinBytes=%lu,bufferMemory=%u,bufferBytes=%lu,enumerated=unknown\n",
                 sensorarrayUsbStreamModeName(profile.mode),
                 (unsigned long)profile.dataEvery,
                 (unsigned long)profile.diagEvery,
                 (unsigned long)stats.sinkQueued,
                 (unsigned long)stats.sinkDropped,
                 (unsigned long)stats.priorityQueued,
                 (unsigned long long)stats.driverAcceptedBytes,
                 (unsigned long long)stats.rxBytes,
                 (unsigned long)stats.partialWrite,
                 (unsigned long)stats.timeout,
                 (unsigned long)stats.driverDrained,
                 (unsigned long)stats.rxOversize,
                 (unsigned long)stats.lineDropped,
                 (unsigned long)stats.lineOversize,
                 (unsigned long long)stats.discardBytes,
                 (long long)stats.lastRxUs,
                 (long long)stats.lastTxUs,
                 stats.driverInstalled ? 1u : 0u,
                 stats.sofPresent ? 1u : 0u,
                 stats.stdioBound ? 1u : 0u,
                 (unsigned long)stats.initError,
                 (unsigned long)stats.taskStackConfiguredBytes,
                 (unsigned long)stats.taskStackMinBytes,
                 (unsigned)stats.normalBufferMemory,
                 (unsigned long)stats.normalBufferBytes);
        if (written < 0 || (size_t)written >= responseSize) {
            snprintf(response, responseSize, "ERR,cmd=USBSTREAM,reason=response_too_long\n");
            return ESP_ERR_INVALID_SIZE;
        }
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
        if (!ctx || (!ctx->systemReady && !sensorarrayFrontendsHeld())) {
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
        if (!sensorarrayRecoveryPost(level, &requestId)) {
            snprintf(response, responseSize, "ERR,cmd=RECOVER,reason=pending\n");
            return ESP_ERR_INVALID_STATE;
        }
        snprintf(response, responseSize,
                 "RACK,cmd=RECOVER,id=%lu,level=%u,state=accepted\n",
                 (unsigned long)requestId,
                 (unsigned)level);
        return ESP_OK;
    }
    if (strcmp(command, "RESTART") == 0) {
        uint32_t requestId = 0u;
        if (!sensorarrayRestartPost(&requestId)) {
            snprintf(response, responseSize, "ERR,cmd=RESTART,reason=pending\n");
            return ESP_ERR_INVALID_STATE;
        }
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
    ctx->frontendFdcSdKnownLow =
        sensorarrayRouteControllerPrepareFdcSdGpio() == ESP_OK;

    ctx->frontendStartupReached = true;
    esp_err_t err = sensorarrayFrontendStartupRun(&ctx->state,
        ctx->primaryAddrValid, ctx->secondaryAddrValid, ctx->requestedFdcChannels);
    if (err != ESP_OK) {
        return err;
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
    sensorarrayBootBreadcrumb_t adsBoot = {0};
    if (sensorarrayBootBreadcrumbCopy(&adsBoot)) {
        sensorarrayAdsGapBindBootIdentity(adsBoot.bootId, adsBoot.bootCount);
    }
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
    __atomic_store_n(&s_frontendCommandsReady, 1u, __ATOMIC_RELEASE);
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
    sensorarrayConfigIdentity_t frameIdentity = {0};
    sensorarrayMeasurementMode_t effectiveModes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
    memcpy(effectiveModes,
           rowProfile.pending ? rowProfile.pendingModes : rowProfile.modes,
           sizeof(effectiveModes));
    bool mixedProfile = !sensorarrayRowModeProfileIsHomogeneous(effectiveModes);
    bool homogeneousProfileFaultTerminal = false;
    esp_err_t err = ESP_ERR_INVALID_STATE;
    if (mixedProfile) {
        sensorarrayScanPlanBuildRowProfile(&ctx->scanPlan,
                                           effectiveModes,
                                           rowProfile.pending ? (rowProfile.generation == UINT32_MAX ? 1u : rowProfile.generation + 1u) :
                                               rowProfile.generation,
                                           rowProfile.pending ? rowProfile.pendingRequestId :
                                               rowProfile.appliedRequestId);
        sensorarrayAdsMatrixEngineSetFrameSequenceHint(&ctx->adsEngine,
                                                       frameSequence);
        sensorarrayCaptureFrameIdentity(ctx, &mode, &rowProfile, frameSequence, &frameIdentity);
        /* Mixed engine prepares all active rows' rail dependency before its
         * first segment; an outgoing MODE=VOLT does not own that decision. */
        err = sensorarrayMixedRowEngineReadFrame(&ctx->fdcEngine,
                                                 &ctx->adsEngine,
                                                 &ctx->scanPlan,
                                                 &ctx->frame,
                                                 ctx->mixedSegmentWorkspace);
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
                sensorarrayEmitConfigTerminal(ctx,
                    rowProfile.pendingRequestId, frameSequence,
                    "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE_REQUESTED\n",
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
                sensorarrayCaptureFrameIdentity(ctx, &mode, &rowProfile, frameSequence, &frameIdentity);
                err = sensorarrayFdcMatrixEngineReadFrame(&ctx->fdcEngine,
                                                          &ctx->scanPlan,
                                                          &ctx->frame);
            } else {
                sensorarrayScanPlanBuildAdsMatrix(&ctx->scanPlan, targetMode);
                sensorarrayAdsMatrixEngineSetFrameSequenceHint(&ctx->adsEngine,
                                                               frameSequence);
                sensorarrayCaptureFrameIdentity(ctx, &mode, &rowProfile, frameSequence, &frameIdentity);
                err = sensorarrayAdsMatrixEngineReadFrame(&ctx->adsEngine,
                                                          &ctx->scanPlan,
                                                          &ctx->frame);
            }
        }
        if (rowProfile.pending && homogeneousApplyOk) {
            /* Applied configuration is independent of a rejected ADC wire
             * sample. The explicit disposition excludes state/owner faults;
             * it never turns an X02 cell into a fresh or valid acquisition. */
            if (!ctx->frame.boundedCrcFailure &&
                (err != ESP_OK || !ctx->frame.freshFrame)) {
                uint32_t readErr = err == ESP_OK ?
                    (uint32_t)ESP_ERR_INVALID_STATE : (uint32_t)err;
                sensorarrayMeasurementModeRecordRuntimeFault(
                    &ctx->measurementMode, readErr);
    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
                (void)sensorarrayAdsMatrixEngineSetMode(
                    &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
                (void)sensorarrayRouteControllerEnterSafe(
                    &ctx->routeController, "rowmodes_readback_fault");
                (void)sensorarrayRowModeProfileFormat(effectiveModes, profileText);
                sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile,
                                                        readErr,
                                                        ctx->frame.physicalSweepUs);
                sensorarrayEmitConfigTerminal(ctx,
                    rowProfile.pendingRequestId, frameSequence,
                    "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE_REQUESTED\n",
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
                sensorarrayEmitConfigTerminal(ctx,
                    rowProfile.pendingRequestId, frameSequence,
                    "RMAPP,id=%lu,gen=%lu,seq=%lu,profile=%s,state=applied\n",
                    (unsigned long)rowProfile.pendingRequestId,
                    (unsigned long)((rowProfile.generation == UINT32_MAX ? 1u : rowProfile.generation + 1u)),
                    (unsigned long)frameSequence,
                    profileText);
            }
        }
    }
    /* Engine initializers clear frames; restore only the start-of-frame identity. */
    ctx->frame.identity = frameIdentity;
    ctx->hostFrameSequence = frameSequence;
    ctx->frame.sequence = frameSequence;
    if (mixedProfile) {
        bool mixedProfileApplied = ctx->frame.freshFrame || ctx->frame.boundedCrcFailure;
        bool mixedRequestFailed = rowProfile.pending && !mixedProfileApplied;
        uint32_t mixedFailureErr = err == ESP_OK ?
            (uint32_t)ESP_ERR_INVALID_STATE : (uint32_t)err;
        ctx->frame.rowProfileGeneration = rowProfile.pending ?
            (mixedProfileApplied ? (rowProfile.generation == UINT32_MAX ? 1u : rowProfile.generation + 1u) : rowProfile.generation) :
            rowProfile.generation;
        ctx->frame.rowProfileRequestId = rowProfile.pending ? rowProfile.pendingRequestId :
            rowProfile.appliedRequestId;
        if (rowProfile.pending && mixedProfileApplied) {
            char appliedProfile[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u];
            (void)sensorarrayRowModeProfileFormat(effectiveModes,
                                                  appliedProfile);
            (void)sensorarrayRowModeProfileCompleteTransition(
                &ctx->rowModeProfile, frameSequence, ctx->frame.physicalSweepUs);
            sensorarrayEmitConfigTerminal(ctx,
                rowProfile.pendingRequestId, frameSequence,
                "RMAPP,id=%lu,gen=%lu,seq=%lu,profile=%s,state=applied\n",
                (unsigned long)rowProfile.pendingRequestId,
                (unsigned long)((rowProfile.generation == UINT32_MAX ? 1u : rowProfile.generation + 1u)),
                (unsigned long)frameSequence,
                appliedProfile);
        } else if (mixedRequestFailed) {
            char rejectedProfile[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u];
            (void)sensorarrayRowModeProfileFormat(
                effectiveModes, rejectedProfile);
            sensorarrayRowModeProfileFailTransition(&ctx->rowModeProfile,
                                                    mixedFailureErr,
                                                    ctx->frame.physicalSweepUs);
            sensorarrayMeasurementModeRecordRuntimeFault(&ctx->measurementMode,
                                                         mixedFailureErr);
            (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                      "mixed_profile_fault");
            sensorarrayEmitConfigTerminal(ctx,
                rowProfile.pendingRequestId, frameSequence,
                "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE_REQUESTED\n",
                (unsigned long)rowProfile.pendingRequestId,
                (unsigned long)rowProfile.generation,
                (unsigned long)frameSequence,
                rejectedProfile,
                (unsigned long)mixedFailureErr);
        }
        if (!ctx->frame.freshFrame && !ctx->frame.boundedCrcFailure) {
            if (!mixedRequestFailed) {
                sensorarrayAdsFaultEvent_t fault = {
                    .stage = SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION,
                    .err = err,
                    .seq = frameSequence,
                };
                sensorarrayAdsFaultFillContext(ctx, &fault);
                fault.owner = sensorarrayAdsOwnerName(
                    SENSORARRAY_ADS_OWNER_MATRIX);
                sensorarrayMeasurementModeRecordRuntimeFault(
                    &ctx->measurementMode, (uint32_t)err);
                (void)sensorarrayRouteControllerEnterSafe(
                    &ctx->routeController, "mixed_profile_fault");
                (void)sensorarrayPublishConfigAuthority(ctx, frameSequence);
                sensorarrayEmitAdsFault(ctx, &fault);
                sensorarrayTransportPublishRuntimeFault("MFAULT,mode=MIXED,seq=%lu,err=0x%lx,state=DEGRADED,route=SAFE_REQUESTED\n",
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
               !ctx->frame.boundedCrcFailure &&
               !homogeneousProfileFaultTerminal) {
        /* Missing samples alone do not prove route failure. Only the ADS
         * engine may explicitly permit a bounded sample-CRC partial frame. */
        sensorarrayAdsFaultEvent_t fault = {
            .stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE,
            .err = err,
            .seq = frameSequence,
        };
        sensorarrayAdsFaultFillContext(ctx, &fault);
        fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_MATRIX);
        sensorarrayMeasurementModeRecordRuntimeFault(&ctx->measurementMode,
                                                      (uint32_t)err);
        (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
        (void)sensorarrayAdsMatrixEngineSetMode(
            &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
        (void)sensorarrayRouteControllerEnterSafe(&ctx->routeController,
                                                  "ads_frame_fault");
        sensorarrayEmitAdsFault(ctx, &fault);
        sensorarrayTransportPublishRuntimeFault("MFAULT,mode=%s,seq=%lu,err=0x%lx,state=DEGRADED,route=SAFE_REQUESTED\n",
               sensorarrayMeasurementModeName(mode.activeMode),
               (unsigned long)frameSequence,
               (unsigned long)err);
    }
    (void)sensorarrayPublishConfigAuthority(ctx, frameSequence);
    if (rowsApply.applied) {
        printf("RAPP,id=%lu,seq=%lu,old=%u,new=%u,gen=%lu,status=applied,session=%016llX,bootId=%lu,rev=%lu,boundary=%lu\n",
               (unsigned long)rowsApply.requestId,
               (unsigned long)ctx->frame.sequence,
               (unsigned)rowsApply.oldRows,
               (unsigned)rowsApply.newRows,
               (unsigned long)rowsApply.generation,
               (unsigned long long)frameIdentity.sessionId, (unsigned long)frameIdentity.bootId,
               (unsigned long)frameIdentity.revision, (unsigned long)frameIdentity.appliedFrameBoundary);
    }
    sensorarrayPublishLastMeasurement(ctx);
    return err;
}

static void sensorarrayRuntimeRescueTick(sensorarrayAppContext_t *ctx)
{
    if (!ctx || !sensorarrayMeasureFdcWorkersQuiescent() ||
        sensorarrayMeasureFdcWorkersQuarantined()) {
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
    if (!ctx || !CONFIG_BOARD_I2C_AUTO_FALLBACK_ENABLE ||
        !sensorarrayMeasureFdcWorkersQuiescent() ||
        sensorarrayMeasureFdcWorkersQuarantined()) {
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
    sensorarrayFrontendStartupRecheckFdcClock(&ctx->state.fdcPrimary, "runtime_primary_reverify");
    sensorarrayFrontendStartupRecheckFdcClock(&ctx->state.fdcSecondary, "runtime_secondary_reverify");
    ctx->runtimeI2cErrorStreak = 0u;
}

static void sensorarrayRunQueuedFullSweep(sensorarrayAppContext_t *ctx)
{
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    sensorarrayRowModeProfile_t profile = {0};
    /* Consume at this boundary even if the current mode cannot run FDC.
     * Never carry a request through VOLT/RES/MIXED into a later CAP epoch. */
    if (!ctx || !sensorarrayFdcSweepConsumeForceFullSweepAll()) {
        return;
    }
    if (
        !sensorarrayMeasureFdcWorkersQuiescent() ||
        sensorarrayMeasureFdcWorkersQuarantined() ||
        !sensorarrayRowModeProfileCopy(&ctx->rowModeProfile, &profile) ||
        !sensorarrayRowModeProfileIsHomogeneous(profile.modes) ||
        !sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode, &mode) ||
        mode.activeMode != SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
        printf("FDC_SWEEP_REQUEST,stage=discard,reason=ineligible_mode_or_owner\n");
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

/* 本轮新增: explicit RECOVER results are cached before priority publication.
 * Route verification below is software/register evidence, never physical safety. */
static void sensorarrayCompleteRecoveryRequest(sensorarrayAppContext_t *ctx,
    uint32_t requestId, sensorarrayRecoverState_t outcome, const char *reason,
    esp_err_t originalError, esp_err_t cleanupError)
{
    sensorarrayConfigAuthority_t authority = {0};
    bool known = sensorarrayConfigIdentityCopy(&authority);
    sensorarrayRouteSnapshot_t route = {0};
    bool routeCopied = ctx && sensorarrayRouteControllerCopySnapshot(
        &ctx->routeController, &route);
    bool cleanupOk = cleanupError == ESP_OK;
    (void)sensorarrayRecoveryControlComplete(requestId, outcome, reason,
        originalError, cleanupError, &authority, known,
        routeCopied && cleanupOk && route.applied,
        routeCopied && cleanupOk && route.known,
        routeCopied && cleanupOk && route.verified);
}

/* 本轮新增: a failed recovery cannot leave CAP active and implicitly rebuild
 * its route next frame. Preserve the first cleanup failure, even if a later
 * SAFE action succeeds. An explicit later MODE/RESTART can recover. */
static esp_err_t sensorarrayStopAfterRecoveryFailure(sensorarrayAppContext_t *ctx,
    esp_err_t originalError, esp_err_t cleanupError, const char *reason)
{
    if (!ctx) { return cleanupError == ESP_OK ? ESP_ERR_INVALID_ARG : cleanupError; }
    sensorarrayMeasurementModeRecordRuntimeFault(&ctx->measurementMode,
        (uint32_t)originalError);
    if (ctx->adsRestoreRecovery.active) {
        ctx->adsRestoreRecovery.active = false;
        ctx->adsRestoreRecovery.terminal = true;
        ctx->adsRestoreRecovery.outcome = SENSORARRAY_MEASUREMENT_RECOVERY_OUTCOME_FAILED;
        ctx->adsRestoreRecovery.lastError = (uint32_t)originalError;
    }
    esp_err_t err = sensorarrayAdsMatrixEngineSetMode(
        &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
    if (cleanupError == ESP_OK && err != ESP_OK) { cleanupError = err; }
    err = sensorarrayRouteControllerEnterSafe(&ctx->routeController, reason);
    if (cleanupError == ESP_OK && err != ESP_OK) { cleanupError = err; }
    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
    sensorarrayBootBreadcrumbMarkStableProgress(false);
    return cleanupError;
}

static esp_err_t sensorarrayRunRecoveryLevel(sensorarrayAppContext_t *ctx,
                                             uint32_t level,
                                             bool *outHardwareTouched,
                                             esp_err_t *outCleanupError)
{
    *outHardwareTouched = false;
    *outCleanupError = ESP_OK;
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayMeasureFdcWorkersQuiescent() ||
        sensorarrayMeasureFdcWorkersQuarantined() ||
        ads126xAdcIsQuarantined(&ctx->state.ads)) {
        return ESP_ERR_INVALID_STATE;
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

    *outHardwareTouched = true;
    if (level >= SENSORARRAY_RECOVERY_LEVEL_FULL) {
        esp_err_t safeErr = sensorarrayRouteControllerEnterSafe(
            &ctx->routeController, "recover_full");
        if (safeErr != ESP_OK) {
            return safeErr;
        }
        if (mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
            sensorarrayFrontendStartupRecheckFdcClock(&ctx->state.fdcPrimary,
                                               "recover_full");
            sensorarrayFrontendStartupRecheckFdcClock(&ctx->state.fdcSecondary,
                                               "recover_full");
            esp_err_t rescueErr = sensorarrayFdcMatrixEngineRunFullRescue(
                &ctx->fdcEngine, "recover_full");
            if (rescueErr != ESP_OK) { return rescueErr; }
        }
        sensorarrayFdcRescueReset(&ctx->fdcRescue);
    }
    esp_err_t err = sensorarrayApplyMeasurementMode(ctx, mode.activeMode, 0u, false);
    if (err != ESP_OK) {
        sensorarrayRouteSnapshot_t route = {0};
        if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController, &route)) {
            *outCleanupError = (esp_err_t)route.cleanupError;
            /* ApplyMeasurementMode already attempted SAFE. Its final failed
             * SAFE readback is cleanup evidence, not a replacement first error. */
            if (*outCleanupError == ESP_OK && !route.safe && route.lastError != 0u) {
                *outCleanupError = (esp_err_t)route.lastError;
            }
        }
    }
    return err;
}

/* 本轮新增: independent manual-reset escape path. It never borrows the
 * config/RECOVER terminal slot or enters a frontend hardware lock. */
static void sensorarrayPerformManualRestart(sensorarrayAppContext_t *ctx,
                                            uint32_t requestId)
{
    bool romDownload = sensorarrayRestartControlIsRom(requestId);
    (void)sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_MANUAL);
    sensorarrayBootBreadcrumbSetStage("manual_restart", ESP_OK, ctx);
    sensorarrayConfigAuthority_t authority = {0};
    bool known = sensorarrayConfigIdentityCopy(&authority);
    (void)sensorarrayRestartControlPublish(requestId, &authority, known);
    /* Bounded opportunity for local priority publication, not a host ACK. */
    vTaskDelay(pdMS_TO_TICKS(20u) + 1u);
    sensorarrayUsbOtgRestart(romDownload);
}

static bool sensorarrayRunRecoveryBoundary(sensorarrayAppContext_t *ctx)
{
    if (!ctx) { return false; }
    uint32_t restartId = 0u;
    if (sensorarrayRestartTake(&restartId)) {
        sensorarrayPerformManualRestart(ctx, restartId);
        return true;
    }
    uint32_t level = SENSORARRAY_RECOVERY_LEVEL_NONE;
    uint32_t requestId = 0u;
    if (!sensorarrayRecoveryTake(&level, &requestId)) { return false; }
    if (sensorarrayFrontendsHeld() && level != SENSORARRAY_RECOVERY_LEVEL_RESTART) {
        sensorarrayCompleteRecoveryRequest(ctx, requestId, SENSORARRAY_RECOVER_REJECTED,
            "wait_frontends", ESP_ERR_INVALID_STATE, ESP_OK);
        return true;
    }
    sensorarrayRouteSnapshot_t recoveryRoute = {0};
    if (sensorarrayRouteControllerCopySnapshot(&ctx->routeController, &recoveryRoute) &&
        recoveryRoute.fdcSdHigh) {
        sensorarrayCompleteRecoveryRequest(ctx, requestId, SENSORARRAY_RECOVER_REJECTED,
            "fdc_restart_required", ESP_ERR_INVALID_STATE, ESP_OK);
        return true;
    }
    if (level == SENSORARRAY_RECOVERY_LEVEL_RESTART) {
        if (sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
            sensorarrayBootBreadcrumbSetStage("auto_restart", ESP_OK, ctx);
            sensorarrayCompleteRecoveryRequest(ctx, requestId, SENSORARRAY_RECOVER_RESTARTING,
                "auto_restart", ESP_OK, ESP_OK);
            /* A bounded opportunity for the reserved terminal to drain. This
             * is not host acknowledgment; a new session supersedes old IDs. */
            vTaskDelay(pdMS_TO_TICKS(20u) + 1u);
            sensorarrayUsbOtgRestart(false);
            return true;
        }
        esp_err_t originalError = ESP_ERR_INVALID_STATE;
        esp_err_t cleanupError = sensorarrayStopAfterRecoveryFailure(ctx,
            originalError, ESP_OK, "recovery_restart_guard");
        sensorarrayBootBreadcrumbSetStage("recovery_safe_idle", originalError, ctx);
        sensorarrayCompleteRecoveryRequest(ctx, requestId, SENSORARRAY_RECOVER_REJECTED,
            "restart_guard", originalError, cleanupError);
        return true;
    }
    bool hardwareTouched = false;
    esp_err_t cleanupError = ESP_OK;
    esp_err_t recoverErr = sensorarrayRunRecoveryLevel(ctx, level,
        &hardwareTouched, &cleanupError);
    if (recoverErr != ESP_OK && hardwareTouched) {
        cleanupError = sensorarrayStopAfterRecoveryFailure(ctx, recoverErr,
            cleanupError, "recovery_failed");
    }
    sensorarrayCompleteRecoveryRequest(ctx, requestId,
        recoverErr == ESP_OK ? SENSORARRAY_RECOVER_APPLIED : SENSORARRAY_RECOVER_REJECTED,
        recoverErr == ESP_OK ? "recovered" : "recovery_failed", recoverErr, cleanupError);
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
        bool frontendHeld = sensorarrayRunFrontendPresenceBoundary(ctx);
        if (sensorarrayRunRecoveryBoundary(ctx)) {
            continue;
        }
        if (sensorarrayMeasureFdcWorkersQuarantined() ||
            ads126xAdcIsQuarantined(&ctx->state.ads) ||
            sensorarrayBringupAdsPresenceQuarantined()) {
            sensorarrayBootBreadcrumbMarkStableProgress(false);
            /* 本轮新增: a timeout is not cancellation. Retain every worker/
             * DMA object, reject hardware transitions, and use the existing
             * RTC restart budget. Budget exhaustion is command-responsive
             * DEGRADED, never a claim that the physical route is safe. */
            sensorarrayApplyPendingCommands(ctx);
            if (!ctx->hardwareQuarantineReported) {
                ctx->hardwareQuarantineReported = true;
                sensorarrayMeasurementModeRecordRuntimeFault(
                    &ctx->measurementMode, (uint32_t)ESP_ERR_INVALID_STATE);
    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
                sensorarrayTransportPublishRuntimeFault("MFAULT,source=OWNER_QUARANTINED,seq=%lu,err=0x%lx,state=DEGRADED,route=UNKNOWN,fdc=%u,spi=%u,presence=%u,physicalSafe=unknown\n",
                       (unsigned long)(ctx->hostFrameSequence + 1u),
                       (unsigned long)ESP_ERR_INVALID_STATE,
                       sensorarrayMeasureFdcWorkersQuarantined() ? 1u : 0u,
                       ads126xAdcIsQuarantined(&ctx->state.ads) ? 1u : 0u,
                       sensorarrayBringupAdsPresenceQuarantined() ? 1u : 0u);
                sensorarrayBootBreadcrumbSetStage("owner_quarantined",
                                                   ESP_ERR_INVALID_STATE, ctx);
                if (sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
                    sensorarrayUsbOtgRestart(false);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(50u));
            continue;
        }
        if (frontendHeld) {
            vTaskDelay(pdMS_TO_TICKS(50u));
            continue;
        }
        if (sensorarrayMeasurementRecoveryIsActive(&ctx->adsRestoreRecovery)) {
            sensorarrayBootBreadcrumbMarkStableProgress(false);
            sensorarrayRunAdsRestoreRecoveryAttempt(ctx);
            continue;
        }
        /* CommandMailbox is drained only here, before a new frame begins. No
         * BLE callback or Core0 task can mutate acquisition state mid-row. */
        sensorarrayApplyPendingCommands(ctx);
        sensorarrayRunQueuedFullSweep(ctx);

        esp_err_t railRefreshErr = sensorarrayRefreshVoltageRailIfNeeded(ctx);
        if (railRefreshErr != ESP_OK) {
            sensorarrayBootBreadcrumbMarkStableProgress(false);
            sensorarrayAdsFaultEvent_t fault = {
                .stage = SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR,
                .err = railRefreshErr,
                .seq = ctx->hostFrameSequence + 1u,
            };
            sensorarrayAdsFaultFillContext(ctx, &fault);
            fault.owner = sensorarrayAdsOwnerName(SENSORARRAY_ADS_OWNER_RAIL);
            sensorarrayMeasurementModeRecordRuntimeFault(
                &ctx->measurementMode, (uint32_t)railRefreshErr);
            (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
            (void)sensorarrayAdsMatrixEngineSetMode(
                &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
            (void)sensorarrayRouteControllerEnterSafe(
                &ctx->routeController, "volt_rail_refresh_failed");
            sensorarrayEmitAdsFault(ctx, &fault);
            sensorarrayTransportPublishRuntimeFault("MFAULT,mode=VOLT,seq=%lu,err=0x%lx,state=DEGRADED,route=SAFE_REQUESTED\n",
                   (unsigned long)(ctx->hostFrameSequence + 1u),
                   (unsigned long)railRefreshErr);
            continue;
        }

        sensorarrayMeasurementModeSnapshot_t loopMode = {0};
        (void)sensorarrayMeasurementModeCopySnapshot(&ctx->measurementMode,
                                                     &loopMode);
        if (!sensorarrayMeasurementModeIsDataMode(loopMode.activeMode) ||
            loopMode.state == SENSORARRAY_MEASUREMENT_STATE_TRANSITION) {
            sensorarrayBootBreadcrumbMarkStableProgress(false);
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
                sensorarrayEmitConfigTerminal(ctx,
                    stalledProfile.pendingRequestId, ctx->hostFrameSequence + 1u,
                    "RMERR,id=%lu,gen=%lu,seq=%lu,profile=%s,err=0x%lx,state=rejected,route=SAFE_REQUESTED\n",
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
            (loopProfile.pending || !sensorarrayRowModeProfileIsHomogeneous(loopProfile.modes))) {
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
            sensorarrayBootBreadcrumbMarkStableProgress(false);
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
            esp_err_t auxiliaryErr = sensorarrayAdsGapRunPendingAtBoundary(
                &ctx->state, ctx->frame.sequence);
            if (sensorarrayAdsGapConsumeRestoreFailure()) {
                esp_err_t restoreError = auxiliaryErr != ESP_OK ?
                    auxiliaryErr : ESP_ERR_INVALID_RESPONSE;
                if (!sensorarrayBeginAdsRestoreRecovery(ctx,
                    SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_ADS_RESTORE,
                    restoreError, 0u)) {
                    sensorarrayMeasurementModeRecordRuntimeFault(
                        &ctx->measurementMode, (uint32_t)restoreError);
                    (void)sensorarrayAdsMatrixEngineSetMode(
                        &ctx->adsEngine, SENSORARRAY_MEASUREMENT_MODE_NONE);
                    esp_err_t cleanupError = sensorarrayRouteControllerEnterSafe(
                        &ctx->routeController, "auxiliary_restore_failed");
                    (void)sensorarrayPublishConfigAuthority(ctx, ctx->hostFrameSequence + 1u);
                    sensorarrayTransportPublishRuntimeFault("MFAULT,source=AUX_RESTORE,seq=%lu,err=0x%lx,cleanupError=0x%lx,state=DEGRADED,route=SAFE_REQUESTED\n",
                           (unsigned long)ctx->hostFrameSequence,
                           (unsigned long)restoreError, (unsigned long)cleanupError);
                }
            } else {
                sensorarrayRunPendingAdsCheck(ctx);
            }
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
                   (ctx->frame.freshFrame || ctx->frame.boundedCrcFailure ||
                    CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG)) {
            (void)sensorarrayFrameOutputPrint(&ctx->frame);
        }
        sensorarrayBootBreadcrumbMarkStableProgress(
            err == ESP_OK && ctx->frame.freshFrame && validMask != 0u);
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
        bool frontendWait = ctx->frontendStartupReached &&
            initErr != ESP_ERR_NO_MEM && sensorarrayFrontendOwnersQuiescent(ctx);
        if (frontendWait) {
            /* An absent ADC at cold boot is not a reason to consume every
             * restart token before the frontend supply has returned. */
            sensorarrayEnterFrontendHold(ctx, "startup_unqualified");
        } else if (sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
            sensorarrayBootBreadcrumbSetStage("auto_restart", initErr, ctx);
            sensorarrayBootEmitRestartMarker(
                "RAPP,cmd=RECOVER,id=0,level=2,state=restarting,kind=auto\n");
            sensorarrayUsbOtgRestart(false);
        }
        if (!frontendWait) {
            sensorarrayBootBreadcrumbSetStage("recovery_safe_idle", initErr, ctx);
        }
        printf("APP_FATAL,stage=acquisition_init,err=%ld,action=%s\n",
               (long)initErr, frontendWait ? "wait_frontends" : "recovery_safe_no_restart");
        for (;;) {
            if (!frontendWait && ctx->frontendStartupReached &&
                initErr != ESP_ERR_NO_MEM && sensorarrayFrontendOwnersQuiescent(ctx)) {
                /* A late boot completion releases ownership, but never
                 * turns the failed startup into READY or reuses its job. */
                frontendWait = true;
                sensorarrayEnterFrontendHold(ctx, "late_boot_owner_release");
            }
            if (frontendWait) {
                (void)sensorarrayRunFrontendPresenceBoundary(ctx);
                if (!ctx->hardwareQuarantineReported &&
                    (sensorarrayMeasureFdcWorkersQuarantined() ||
                     ads126xAdcIsQuarantined(&ctx->state.ads) ||
                     sensorarrayBringupAdsPresenceQuarantined())) {
                    ctx->hardwareQuarantineReported = true;
                    sensorarrayBootBreadcrumbSetStage("presence_quarantined",
                                                       ESP_ERR_INVALID_STATE, ctx);
                    if (sensorarrayBootPrepareRestart(SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
                        sensorarrayBootEmitRestartMarker(
                            "RAPP,cmd=RECOVER,id=0,level=2,state=restarting,reason=presence_quarantined\n");
                        sensorarrayUsbOtgRestart(false);
                    }
                }
            }
            uint32_t restartId = 0u;
            if (sensorarrayRestartTake(&restartId)) {
                sensorarrayPerformManualRestart(ctx, restartId);
            }
            uint32_t recoveryLevel = SENSORARRAY_RECOVERY_LEVEL_NONE;
            uint32_t recoveryId = 0u;
            if (sensorarrayRecoveryTake(&recoveryLevel, &recoveryId)) {
                if (recoveryLevel == SENSORARRAY_RECOVERY_LEVEL_RESTART &&
                    sensorarrayBootPrepareRestart(
                        SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
                    sensorarrayBootBreadcrumbSetStage("auto_restart",
                                                      ESP_ERR_INVALID_STATE,
                                                      ctx);
                    sensorarrayCompleteRecoveryRequest(&s_appContext, recoveryId,
                        SENSORARRAY_RECOVER_RESTARTING, "auto_restart", ESP_OK, ESP_OK);
                    vTaskDelay(pdMS_TO_TICKS(20u) + 1u);
                    sensorarrayUsbOtgRestart(false);
                }
                esp_err_t cleanupError = sensorarrayStopAfterRecoveryFailure(ctx,
                    ESP_ERR_INVALID_STATE, ESP_OK, "recovery_init_failed");
                sensorarrayCompleteRecoveryRequest(ctx, recoveryId,
                    SENSORARRAY_RECOVER_REJECTED,
                    recoveryLevel == SENSORARRAY_RECOVERY_LEVEL_RESTART ?
                        "restart_guard" : "acquisition_init_failed",
                    ESP_ERR_INVALID_STATE, cleanupError);
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
        /* READY is startup, not proof that the acquisition is making progress. */
        sensorarrayBootBreadcrumbMarkStableProgress(false);
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
    sensorarrayFrontendPresenceInit(&s_frontendPresence,
        (uint64_t)esp_timer_get_time());
    sensorarrayBootBreadcrumbStart();
    esp_fill_random(&s_configSessionId, sizeof(s_configSessionId));
    sensorarrayRecoveryControlInit(s_configSessionId, s_bootBreadcrumb.bootId);
    sensorarrayRestartControlInit(s_configSessionId, s_bootBreadcrumb.bootId);
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
                sensorarrayPerformManualRestart(&s_appContext, restartId);
            }
            uint32_t recoveryLevel = SENSORARRAY_RECOVERY_LEVEL_NONE;
            uint32_t recoveryId = 0u;
            if (sensorarrayRecoveryTake(&recoveryLevel, &recoveryId)) {
                if (recoveryLevel == SENSORARRAY_RECOVERY_LEVEL_RESTART &&
                    sensorarrayBootPrepareRestart(
                        SENSORARRAY_BOOT_RESTART_KIND_AUTO)) {
                    sensorarrayBootBreadcrumbSetStage("auto_restart",
                                                      ESP_ERR_INVALID_STATE,
                                                      NULL);
                    sensorarrayCompleteRecoveryRequest(&s_appContext, recoveryId,
                        SENSORARRAY_RECOVER_RESTARTING, "auto_restart", ESP_OK, ESP_OK);
                    vTaskDelay(pdMS_TO_TICKS(20u) + 1u);
                    sensorarrayUsbOtgRestart(false);
                }
                esp_err_t cleanupError = sensorarrayStopAfterRecoveryFailure(&s_appContext,
                    ESP_ERR_NO_MEM, ESP_OK, "recovery_no_scan_task");
                sensorarrayCompleteRecoveryRequest(&s_appContext, recoveryId,
                    SENSORARRAY_RECOVER_REJECTED,
                    recoveryLevel == SENSORARRAY_RECOVERY_LEVEL_RESTART ?
                        "restart_guard" : "scan_task_create_failed",
                    ESP_ERR_NO_MEM, cleanupError);
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
