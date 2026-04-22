#include "sensorarrayDebugSelftest.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "boardSupport.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"
#include "tmuxSwitch.h"

#define SENSORARRAY_S5D5_DRIVE_CURRENT_MASK FDC2214_CAP_DRIVE_CURRENT_MASK
#define SENSORARRAY_S5D5_DRIVE_STEP_MAX FDC2214_CAP_DRIVE_STEP_MAX
#define SENSORARRAY_S5D5_DRIVE_STEP_DEFAULT 31u
#define SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT 14u
#define SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK 0xC000u
#define SENSORARRAY_S5D5_CONFIG_SLEEP_MODE_EN_MASK 0x2000u
#define SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK 0x0040u
#define SENSORARRAY_S5D5_REG_STATUS 0x18u
#define SENSORARRAY_S5D5_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_S5D5_REG_CONFIG 0x1Au
#define SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0 0x1Eu
#define SENSORARRAY_S5D5_REG_DATA_MSB_BASE 0x00u
#define SENSORARRAY_S5D5_REG_DATA_LSB_BASE 0x01u
#define SENSORARRAY_S5D5_DATA_MSB_MASK 0x0FFFu
#define SENSORARRAY_S5D5_DATA_ERR_WD_MASK (1u << 13)
#define SENSORARRAY_S5D5_DATA_ERR_AW_MASK (1u << 12)
#define SENSORARRAY_S5D5_STEP_SETTLE_MS 350u
#define SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS 20u
#define SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS 500u
#define SENSORARRAY_S5D5_RECOVERY_FAIL_DEGRADED_THRESHOLD 3u
#define SENSORARRAY_S5D5_DEGRADED_MIN_BACKOFF_MS 2000u
#define SENSORARRAY_S5D5_RAW_SPAN_TOO_SMALL_PERMILLE 4u
#define SENSORARRAY_S5D5_RAW_SPAN_HEALTHY_LOW_MAX_PERMILLE 20u
#define SENSORARRAY_S5D5_RAW_SPAN_HEALTHY_MID_MAX_PERMILLE 120u
#define SENSORARRAY_S5D5_RAW_SPAN_HIGH_MAX_PERMILLE 280u

static const uint8_t SENSORARRAY_S5D5_DRIVE_STEP_SWEEP_TABLE[] = {
    20u, // 0xA000
    23u, // 0xB800
    24u, // 0xC000
    26u, // 0xD000
    28u, // 0xE000
    31u, // 0xF800
};

static const bool SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[] = {
    false,
    true,
};

void sensorarrayDebugRunAdsSelftestModeImpl(sensorarrayState_t *state)
{
    if (!state || !state->adsReady) {
        sensorarrayLogStartup("ads_selftest", ESP_ERR_INVALID_STATE, "skip_ads_unavailable", 0);
        sensorarrayDebugIdleForever("ads_unavailable");
        return;
    }

    uint8_t muxp = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXP & 0x0Fu);
    uint8_t muxn = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_MUXN & 0x0Fu);
    uint8_t refmux = (uint8_t)(CONFIG_SENSORARRAY_DEBUG_ADS_FORCE_REFMUX & 0xFFu);
    uint8_t retryCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_READ_RETRY_COUNT;
    uint8_t discardCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_DISCARD_COUNT;
    uint8_t sampleCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_ADS_SAMPLE_COUNT;
    bool drdyTimedOut = false;

    sensorarrayMeasureDumpAdsKeyRegisters(state, "pre_cfg");

    esp_err_t err = ads126xAdcConfigure(&state->ads, true, false, ADS126X_CRC_OFF, 1, 0);
    sensorarrayLogStartup("ads_selftest_cfg", err, (err == ESP_OK) ? "ok" : "configure_error", 0);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_cfg_fail");
        return;
    }

    sensorarrayMeasureDumpAdsKeyRegisters(state, "post_cfg");

    if (CONFIG_SENSORARRAY_DEBUG_ADS_STOP1_BEFORE_MUX && state->adsAdc1Running) {
        err = ads126xAdcStopAdc1(&state->ads);
        sensorarrayLogStartup("ads_selftest_stop1", err, (err == ESP_OK) ? "ok" : "stop_error", 0);
        if (err != ESP_OK) {
            sensorarrayDebugIdleForever("ads_stop_fail");
            return;
        }
        state->adsAdc1Running = false;
    }

    err = ads126xAdcSetRefMux(&state->ads, refmux);
    if (err == ESP_OK) {
        state->adsRefMux = refmux;
        state->adsRefMuxValid = true;
        sensorarrayLogDbgExtraSetRefMux(refmux);
    }
    sensorarrayLogStartup("ads_selftest_refmux", err, (err == ESP_OK) ? "ok" : "refmux_error", refmux);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_refmux_fail");
        return;
    }

    err = ads126xAdcSetInputMux(&state->ads, muxp, muxn);
    sensorarrayLogStartup("ads_selftest_inpmux",
                          err,
                          (err == ESP_OK) ? "ok" : "inpmux_error",
                          ((int32_t)muxp << 8) | (int32_t)muxn);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_inpmux_fail");
        return;
    }

    if (CONFIG_SENSORARRAY_DEBUG_ADS_SETTLE_AFTER_MUX_MS > 0) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_DEBUG_ADS_SETTLE_AFTER_MUX_MS));
    }

    if (CONFIG_SENSORARRAY_DEBUG_ADS_START1_BEFORE_READ || !state->adsAdc1Running) {
        err = ads126xAdcStartAdc1(&state->ads);
        sensorarrayLogStartup("ads_selftest_start1", err, (err == ESP_OK) ? "ok" : "start_error", 0);
        if (err != ESP_OK) {
            sensorarrayDebugIdleForever("ads_start_fail");
            return;
        }
        state->adsAdc1Running = true;
    }

    for (uint8_t i = 0; i < discardCount; ++i) {
        int32_t throwaway = 0;
        err = sensorarrayMeasureReadAdsRawWithRetry(state, &throwaway, retryCount, &drdyTimedOut, NULL);
        printf("DBGADSSELF,stage=discard,index=%u,raw=%ld,muxp=%u,muxn=%u,refmux=0x%02X,discardCount=%u,"
               "discardFirst=%u,drdyTimeout=%u,err=%ld,status=%s\n",
               (unsigned)i,
               (long)throwaway,
               (unsigned)muxp,
               (unsigned)muxn,
               refmux,
               (unsigned)discardCount,
               (discardCount > 0u) ? 1u : 0u,
               drdyTimedOut ? 1u : 0u,
               (long)err,
               (err == ESP_OK) ? "discard_ok" : "discard_error");
        if (err != ESP_OK) {
            sensorarrayDebugIdleForever("ads_discard_fail");
            return;
        }
    }

    for (uint8_t i = 0; i < sampleCount; ++i) {
        int32_t raw = 0;
        drdyTimedOut = false;
        err = sensorarrayMeasureReadAdsRawWithRetry(state, &raw, retryCount, &drdyTimedOut, NULL);
        int32_t uv = (err == ESP_OK) ? ads126xAdcRawToMicrovolts(&state->ads, raw) : 0;
        printf("DBGADSSELF,stage=sample,index=%u,raw=%ld,uv=%ld,muxp=%u,muxn=%u,refmux=0x%02X,discardCount=%u,"
               "discardFirst=%u,drdyTimeout=%u,err=%ld,status=%s\n",
               (unsigned)i,
               (long)raw,
               (long)uv,
               (unsigned)muxp,
               (unsigned)muxn,
               refmux,
               (unsigned)discardCount,
               (discardCount > 0u) ? 1u : 0u,
               drdyTimedOut ? 1u : 0u,
               (long)err,
               (err == ESP_OK) ? "sample_ok" : "sample_error");
    }

    sensorarrayMeasureDumpAdsKeyRegisters(state, "post_read");
    sensorarrayDebugIdleForever("ads_selftest_done");
}

void sensorarrayDebugRunFdcSelftestModeImpl(sensorarrayState_t *state)
{
    if (!state) {
        sensorarrayDebugIdleForever("fdc_state_null");
        return;
    }

    uint8_t dLine = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FDC_D_LINE;
    const sensorarrayFdcDLineMap_t *fdcMap = NULL;
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcStateForDLine(state, dLine, &fdcMap);

    if (!fdcMap || !fdcState || !fdcState->ready || !fdcState->handle) {
        sensorarrayLogStartup("fdc_selftest", ESP_ERR_INVALID_STATE, "fdc_not_ready", (int32_t)dLine);
        sensorarrayDebugIdleForever("fdc_unavailable");
        return;
    }

    bool discardFirst = (CONFIG_SENSORARRAY_DEBUG_FDC_DISCARD_FIRST != 0);
    uint8_t sampleCount = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FDC_SAMPLE_COUNT;
    if (sampleCount == 0u) {
        sampleCount = 1u;
    }

    uint32_t validCount = 0u;
    uint32_t i2cErrCount = 0u;
    uint32_t configUnknownCount = 0u;
    uint32_t stillSleepingCount = 0u;
    uint32_t notConvertingCount = 0u;
    uint32_t noUnreadCount = 0u;
    uint32_t zeroRawCount = 0u;
    uint32_t watchdogCount = 0u;
    uint32_t amplitudeCount = 0u;

    for (uint8_t i = 0; i < sampleCount; ++i) {
        bool doDiscard = discardFirst && (i == 0u);
        sensorarrayFdcReadDiag_t diag = {0};
        esp_err_t err = sensorarrayMeasureReadFdcSampleDiag(fdcState->handle,
                                                            fdcMap->channel,
                                                            doDiscard,
                                                            fdcState->haveIds,
                                                            fdcState->configVerified,
                                                            &diag);
        const char *statusName = sensorarrayMeasureFdcSampleStatusName(diag.statusCode);
        const char *refClockName = (diag.sample.RefClockSource == FDC2214_REF_CLOCK_EXTERNAL)
                                       ? "external_clkin"
                                       : "internal_oscillator";

        switch (diag.statusCode) {
        case SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID:
            validCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR:
            i2cErrCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN:
            configUnknownCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING:
            stillSleepingCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
            notConvertingCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
            noUnreadCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID:
            zeroRawCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT:
            watchdogCount++;
            break;
        case SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT:
            amplitudeCount++;
            break;
        default:
            configUnknownCount++;
            break;
        }

        printf("DBGFDCSELF,index=%u,dLine=%u,fdcDev=%s,channel=%u,discardFirst=%u,i2cOk=%u,idOk=%u,configOk=%u,"
               "converting=%u,unread=%u,sampleValid=%u,raw=%lu,wd=%u,amp=%u,statusReg=0x%04X,configReg=0x%04X,"
               "muxReg=0x%04X,refClock=%s,err=%ld,status=%s\n",
               (unsigned)i,
               (unsigned)dLine,
               fdcState->label ? fdcState->label : SENSORARRAY_NA,
               (unsigned)fdcMap->channel,
               doDiscard ? 1u : 0u,
               diag.i2cOk ? 1u : 0u,
               diag.idOk ? 1u : 0u,
               diag.configOk ? 1u : 0u,
               diag.converting ? 1u : 0u,
               diag.unreadConversionPresent ? 1u : 0u,
               diag.sampleValid ? 1u : 0u,
               (unsigned long)diag.sample.Raw28,
               diag.sample.ErrWatchdog ? 1u : 0u,
               diag.sample.ErrAmplitude ? 1u : 0u,
               diag.sample.StatusRaw,
               diag.sample.ConfigRaw,
               diag.sample.MuxRaw,
               refClockName,
               (long)err,
               statusName);
    }

    uint32_t invalidCount = i2cErrCount + configUnknownCount + stillSleepingCount + notConvertingCount + noUnreadCount +
                            zeroRawCount + watchdogCount + amplitudeCount;
    const char *rootCause = "sample_valid";
    if (i2cErrCount > 0u) {
        rootCause = "i2c_read_error";
    } else if (configUnknownCount > 0u) {
        rootCause = "config_unknown";
    } else if (stillSleepingCount > 0u) {
        rootCause = "device_still_sleeping";
    } else if (notConvertingCount > 0u) {
        rootCause = "not_converting";
    } else if (noUnreadCount > 0u) {
        rootCause = "no_unread_conversions";
    } else if (zeroRawCount > 0u) {
        rootCause = "raw_stuck_zero";
    } else if (watchdogCount > 0u) {
        rootCause = "watchdog_fault";
    } else if (amplitudeCount > 0u) {
        rootCause = "amplitude_fault";
    }

    printf("DBGFDCSELF_SUMMARY,dLine=%u,samples=%u,valid=%lu,invalid=%lu,i2cErr=%lu,configUnknown=%lu,stillSleeping=%lu,"
           "notConverting=%lu,noUnread=%lu,zeroRaw=%lu,watchdog=%lu,amplitude=%lu,rootCause=%s\n",
           (unsigned)dLine,
           (unsigned)sampleCount,
           (unsigned long)validCount,
           (unsigned long)invalidCount,
           (unsigned long)i2cErrCount,
           (unsigned long)configUnknownCount,
           (unsigned long)stillSleepingCount,
           (unsigned long)notConvertingCount,
           (unsigned long)noUnreadCount,
           (unsigned long)zeroRawCount,
           (unsigned long)watchdogCount,
           (unsigned long)amplitudeCount,
           rootCause);

    sensorarrayDebugIdleForever("fdc_selftest_done");
}

typedef enum {
    SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_BEGIN = 0,
    SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_OK,
    SENSORARRAY_CHECKPOINT_EVENT_ROUTE_APPLIED,
    SENSORARRAY_CHECKPOINT_EVENT_STEP_BEGIN,
    SENSORARRAY_CHECKPOINT_EVENT_SNAPSHOT_DONE,
    SENSORARRAY_CHECKPOINT_EVENT_WARNING,
} sensorarrayCheckpointEvent_t;

typedef struct {
    bool enabled;
    gpio_num_t gpio;
    uint32_t pulseWidthUs;
    uint32_t pulseGapUs;
} sensorarrayCheckpointGpio_t;

typedef struct {
    bool ctrlStateReadOk;
    bool commandMatch;
    bool gpioObservedMatch;
    bool gpioRouteVerified;
} sensorarrayS5d5RouteCheck_t;

typedef enum {
    SENSORARRAY_S5D5_ROUTE_GATE_OK = 0,
    SENSORARRAY_S5D5_ROUTE_GATE_GPIO_OK_ANALOG_UNVERIFIED,
    SENSORARRAY_S5D5_ROUTE_GATE_BLOCKED_BEFORE_SWEEP,
} sensorarrayS5d5RouteGateStatus_t;

typedef enum {
    SENSORARRAY_S5D5_FAULT_NONE = 0,
    SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED,
    SENSORARRAY_S5D5_FAULT_UNREAD_TIMEOUT_REPEATED,
    SENSORARRAY_S5D5_FAULT_WATCHDOG_FAULT_PERSISTENT,
    SENSORARRAY_S5D5_FAULT_AMPLITUDE_FAULT_PERSISTENT,
    SENSORARRAY_S5D5_FAULT_TRANSPORT_UNREADABLE_PERSISTENT,
    SENSORARRAY_S5D5_FAULT_RAW_SPAN_OR_HEALTH_GATE_FAILED,
    SENSORARRAY_S5D5_FAULT_READBACK_MISMATCH,
    SENSORARRAY_S5D5_FAULT_I2C_LINE_STUCK,
    SENSORARRAY_S5D5_FAULT_LOCK_APPLY_FAILED,
} sensorarrayS5d5FaultReason_t;

typedef struct {
    uint32_t recoveryCount;
    sensorarrayS5d5FaultReason_t lastFaultReason;
} sensorarrayS5d5RecoveryTracker_t;

static void sensorarrayDebugSelftestDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static const char *sensorarrayS5d5RouteGateStatusName(sensorarrayS5d5RouteGateStatus_t status)
{
    switch (status) {
    case SENSORARRAY_S5D5_ROUTE_GATE_OK:
        return "ok";
    case SENSORARRAY_S5D5_ROUTE_GATE_GPIO_OK_ANALOG_UNVERIFIED:
        return "gpio_ok_analog_unverified";
    case SENSORARRAY_S5D5_ROUTE_GATE_BLOCKED_BEFORE_SWEEP:
    default:
        return "blocked_before_sweep";
    }
}

static const char *sensorarrayS5d5FaultReasonName(sensorarrayS5d5FaultReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED:
        return "i2c_transaction_failed";
    case SENSORARRAY_S5D5_FAULT_UNREAD_TIMEOUT_REPEATED:
        return "unread_timeout_repeated";
    case SENSORARRAY_S5D5_FAULT_WATCHDOG_FAULT_PERSISTENT:
        return "watchdog_fault_persistent";
    case SENSORARRAY_S5D5_FAULT_AMPLITUDE_FAULT_PERSISTENT:
        return "amplitude_fault_persistent";
    case SENSORARRAY_S5D5_FAULT_TRANSPORT_UNREADABLE_PERSISTENT:
        return "transport_unreadable_persistent";
    case SENSORARRAY_S5D5_FAULT_RAW_SPAN_OR_HEALTH_GATE_FAILED:
        return "raw_span_or_health_gate_failed";
    case SENSORARRAY_S5D5_FAULT_READBACK_MISMATCH:
        return "readback_mismatch";
    case SENSORARRAY_S5D5_FAULT_I2C_LINE_STUCK:
        return "i2c_line_stuck";
    case SENSORARRAY_S5D5_FAULT_LOCK_APPLY_FAILED:
        return "lock_apply_failed";
    case SENSORARRAY_S5D5_FAULT_NONE:
    default:
        return "none";
    }
}

static bool sensorarrayS5d5FaultNeedsBusRecovery(sensorarrayS5d5FaultReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED:
    case SENSORARRAY_S5D5_FAULT_I2C_LINE_STUCK:
    case SENSORARRAY_S5D5_FAULT_LOCK_APPLY_FAILED:
        return true;
    case SENSORARRAY_S5D5_FAULT_UNREAD_TIMEOUT_REPEATED:
    case SENSORARRAY_S5D5_FAULT_WATCHDOG_FAULT_PERSISTENT:
    case SENSORARRAY_S5D5_FAULT_AMPLITUDE_FAULT_PERSISTENT:
    case SENSORARRAY_S5D5_FAULT_TRANSPORT_UNREADABLE_PERSISTENT:
    case SENSORARRAY_S5D5_FAULT_RAW_SPAN_OR_HEALTH_GATE_FAILED:
    case SENSORARRAY_S5D5_FAULT_READBACK_MISMATCH:
    case SENSORARRAY_S5D5_FAULT_NONE:
    default:
        return false;
    }
}

static const char *sensorarrayS5d5RecoveryTriggerClass(sensorarrayS5d5FaultReason_t reason, const char *detail)
{
    if (reason == SENSORARRAY_S5D5_FAULT_I2C_LINE_STUCK) {
        return "line_stuck";
    }
    if (reason == SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED ||
        reason == SENSORARRAY_S5D5_FAULT_LOCK_APPLY_FAILED) {
        if (detail && strstr(detail, "timeout")) {
            return "timeout";
        }
        return "transport_failure";
    }
    if (reason == SENSORARRAY_S5D5_FAULT_UNREAD_TIMEOUT_REPEATED ||
        reason == SENSORARRAY_S5D5_FAULT_WATCHDOG_FAULT_PERSISTENT ||
        reason == SENSORARRAY_S5D5_FAULT_AMPLITUDE_FAULT_PERSISTENT ||
        reason == SENSORARRAY_S5D5_FAULT_TRANSPORT_UNREADABLE_PERSISTENT ||
        reason == SENSORARRAY_S5D5_FAULT_RAW_SPAN_OR_HEALTH_GATE_FAILED) {
        return "analog_quality";
    }
    return "policy_mistake";
}

static int sensorarrayExpectedSwLevel(tmux1108Source_t source)
{
    int refLevel = CONFIG_TMUX1108_SW_REF_LEVEL ? 1 : 0;
    return (source == TMUX1108_SOURCE_REF) ? refLevel : (refLevel ? 0 : 1);
}

static void sensorarrayCheckpointPulse(const sensorarrayCheckpointGpio_t *checkpoint, uint32_t pulseCount)
{
    if (!checkpoint || !checkpoint->enabled || pulseCount == 0u) {
        return;
    }

    for (uint32_t i = 0u; i < pulseCount; ++i) {
        gpio_set_level(checkpoint->gpio, 1);
        esp_rom_delay_us(checkpoint->pulseWidthUs);
        gpio_set_level(checkpoint->gpio, 0);
        esp_rom_delay_us(checkpoint->pulseGapUs);
    }
}

static void sensorarrayCheckpointEmit(const sensorarrayCheckpointGpio_t *checkpoint, sensorarrayCheckpointEvent_t event)
{
    switch (event) {
    case SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_BEGIN:
        sensorarrayCheckpointPulse(checkpoint, 1u);
        break;
    case SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_OK:
        sensorarrayCheckpointPulse(checkpoint, 2u);
        break;
    case SENSORARRAY_CHECKPOINT_EVENT_ROUTE_APPLIED:
        sensorarrayCheckpointPulse(checkpoint, 3u);
        break;
    case SENSORARRAY_CHECKPOINT_EVENT_STEP_BEGIN:
        sensorarrayCheckpointPulse(checkpoint, 4u);
        break;
    case SENSORARRAY_CHECKPOINT_EVENT_SNAPSHOT_DONE:
        sensorarrayCheckpointPulse(checkpoint, 5u);
        break;
    case SENSORARRAY_CHECKPOINT_EVENT_WARNING:
        sensorarrayCheckpointPulse(checkpoint, 6u);
        break;
    default:
        break;
    }
}

static sensorarrayCheckpointGpio_t sensorarrayCheckpointInit(void)
{
    sensorarrayCheckpointGpio_t checkpoint = {
        .enabled = false,
        .gpio = GPIO_NUM_NC,
        .pulseWidthUs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_PULSE_US,
        .pulseGapUs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_PULSE_US,
    };

    if (CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO < 0 ||
        CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO >= (int)GPIO_NUM_MAX) {
        printf("DBGFDC_S5D5,stage=checkpoint,status=disabled,gpio=%d,reason=invalid_or_disabled\n",
               CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO);
        return checkpoint;
    }

    checkpoint.gpio = (gpio_num_t)CONFIG_SENSORARRAY_DEBUG_CHECKPOINT_GPIO;
    gpio_config_t ioCfg = {
        .pin_bit_mask = (1ULL << checkpoint.gpio),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&ioCfg);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=checkpoint,status=disabled,gpio=%d,err=%ld,reason=gpio_config_failed\n",
               (int)checkpoint.gpio,
               (long)err);
        return checkpoint;
    }

    gpio_set_level(checkpoint.gpio, 0);
    checkpoint.enabled = true;
    printf("DBGFDC_S5D5,stage=checkpoint,status=enabled,gpio=%d,pulseUs=%lu\n",
           (int)checkpoint.gpio,
           (unsigned long)checkpoint.pulseWidthUs);
    return checkpoint;
}

static sensorarrayS5d5RouteCheck_t sensorarrayVerifyS5d5Route(tmux1108Source_t swSource,
                                                               uint8_t rowIndex,
                                                               int selaWriteLevel,
                                                               bool selbWriteLevel)
{
    sensorarrayS5d5RouteCheck_t routeCheck = {
        .ctrlStateReadOk = false,
        .commandMatch = false,
        .gpioObservedMatch = false,
        .gpioRouteVerified = false,
    };

    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) != ESP_OK) {
        printf("DBGFDC_S5D5,stage=route_verify,ctrlStateRead=0,commandMatch=0,gpioObservedMatch=0,"
               "status=ctrl_state_unavailable,note=gpio_only_not_analog_conduction_proof\n");
        return routeCheck;
    }

    routeCheck.ctrlStateReadOk = true;
    int expectedA0 = (int)(rowIndex & 0x1u);
    int expectedA1 = (int)((rowIndex >> 1u) & 0x1u);
    int expectedA2 = (int)((rowIndex >> 2u) & 0x1u);
    int expectedSelB = selbWriteLevel ? 1 : 0;
    int expectedSw = sensorarrayExpectedSwLevel(swSource);

    bool rowCommandMatch = (ctrl.cmdRow == rowIndex) &&
                           (ctrl.cmdA0Level == expectedA0) &&
                           (ctrl.cmdA1Level == expectedA1) &&
                           (ctrl.cmdA2Level == expectedA2);
    bool selaCommandMatch = (ctrl.cmdSelaLevel == selaWriteLevel);
    bool selbCommandMatch = (ctrl.cmdSelbLevel == expectedSelB);
    bool swCommandMatch = (ctrl.cmdSource == swSource) && (ctrl.cmdSwLevel == expectedSw);
    routeCheck.commandMatch = rowCommandMatch && selaCommandMatch && selbCommandMatch && swCommandMatch;

    bool rowObservedMatch = (ctrl.obsA0Level == expectedA0) &&
                            (ctrl.obsA1Level == expectedA1) &&
                            (ctrl.obsA2Level == expectedA2);
    bool selaObservedMatch = (ctrl.obsSelaLevel == selaWriteLevel);
    bool selbObservedMatch = (ctrl.obsSelbLevel == expectedSelB);
    bool swObservedMatch = (ctrl.obsSwLevel == expectedSw);
    routeCheck.gpioObservedMatch = rowObservedMatch && selaObservedMatch && selbObservedMatch && swObservedMatch;
    routeCheck.gpioRouteVerified = routeCheck.commandMatch && routeCheck.gpioObservedMatch;

    printf("DBGFDC_S5D5,stage=route_verify,ctrlStateRead=1,row=%u,expectedA0=%d,expectedA1=%d,expectedA2=%d,"
           "expectedSELA=%d,expectedSELB=%d,expectedSW=%d,cmdA0=%d,cmdA1=%d,cmdA2=%d,cmdSELA=%d,cmdSELB=%d,"
           "cmdSW=%d,obsA0=%d,obsA1=%d,obsA2=%d,obsSELA=%d,obsSELB=%d,obsSW=%d,commandMatch=%u,gpioObservedMatch=%u,"
           "status=%s,note=gpio_only_not_analog_conduction_proof\n",
           (unsigned)rowIndex,
           expectedA0,
           expectedA1,
           expectedA2,
           selaWriteLevel,
           expectedSelB,
           expectedSw,
           ctrl.cmdA0Level,
           ctrl.cmdA1Level,
           ctrl.cmdA2Level,
           ctrl.cmdSelaLevel,
           ctrl.cmdSelbLevel,
           ctrl.cmdSwLevel,
           ctrl.obsA0Level,
           ctrl.obsA1Level,
           ctrl.obsA2Level,
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsSwLevel,
            routeCheck.commandMatch ? 1u : 0u,
            routeCheck.gpioObservedMatch ? 1u : 0u,
            routeCheck.gpioRouteVerified
                ? "route_gpio_match"
                : "warning_route_gpio_or_command_mismatch");
    return routeCheck;
}

static uint16_t sensorarrayS5d5DriveRawFromStep(uint8_t driveStep)
{
    uint16_t raw = 0u;
    if (!Fdc2214CapDriveStepToRaw(driveStep, &raw)) {
        return 0u;
    }
    return raw;
}

static uint8_t sensorarrayS5d5DriveStepFromRaw(uint16_t driveRaw, uint16_t *outMaskedRaw)
{
    uint8_t driveStep = 0u;
    uint16_t maskedRaw = 0u;
    (void)Fdc2214CapDriveRawToStep(driveRaw, &driveStep, &maskedRaw);
    if (outMaskedRaw) {
        *outMaskedRaw = maskedRaw;
    }
    return driveStep;
}

static void sensorarrayS5d5ResolveForcedDriveConfig(uint16_t configValue,
                                                    uint8_t *outDriveStep,
                                                    uint16_t *outDriveRawReq,
                                                    uint16_t *outDriveRawMasked,
                                                    bool *outLegacyRawInput,
                                                    bool *outReservedBitsMasked)
{
    uint8_t driveStep = SENSORARRAY_S5D5_DRIVE_STEP_DEFAULT;
    uint16_t driveRawReq = configValue;
    uint16_t driveRawMasked = 0u;
    bool legacyRawInput = false;
    bool reservedBitsMasked = false;

    if (configValue <= SENSORARRAY_S5D5_DRIVE_STEP_MAX) {
        driveStep = (uint8_t)configValue;
        driveRawMasked = sensorarrayS5d5DriveRawFromStep(driveStep);
        driveRawReq = driveRawMasked;
    } else {
        legacyRawInput = true;
        driveStep = sensorarrayS5d5DriveStepFromRaw(configValue, &driveRawMasked);
        reservedBitsMasked = (driveRawMasked != configValue);
    }

    if (outDriveStep) {
        *outDriveStep = driveStep;
    }
    if (outDriveRawReq) {
        *outDriveRawReq = driveRawReq;
    }
    if (outDriveRawMasked) {
        *outDriveRawMasked = driveRawMasked;
    }
    if (outLegacyRawInput) {
        *outLegacyRawInput = legacyRawInput;
    }
    if (outReservedBitsMasked) {
        *outReservedBitsMasked = reservedBitsMasked;
    }
}

static esp_err_t sensorarrayApplyS5d5DriveStep(Fdc2214CapDevice_t *dev,
                                                bool highCurrent,
                                                uint8_t driveStep,
                                                uint16_t *outDriveRawWritten)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }
    if (driveStep > SENSORARRAY_S5D5_DRIVE_STEP_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t configReg = 0u;
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_CONFIG, &configReg);
    if (err != ESP_OK) {
        return err;
    }

    configReg &= (uint16_t)~SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK;
    configReg |= (uint16_t)((uint16_t)FDC2214_CH0 << SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
    configReg &= (uint16_t)~SENSORARRAY_S5D5_CONFIG_SLEEP_MODE_EN_MASK;
    if (highCurrent) {
        configReg |= SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK;
    } else {
        configReg &= (uint16_t)~SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK;
    }

    err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_CONFIG, configReg);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t driveCurrentRaw = sensorarrayS5d5DriveRawFromStep(driveStep);
    if (outDriveRawWritten) {
        *outDriveRawWritten = driveCurrentRaw;
    }
    return Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, driveCurrentRaw);
}

typedef enum {
    SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL = 0,
    SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_LOW,
    SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_MID,
    SENSORARRAY_S5D5_RAW_SPAN_BAND_HIGH_NOISY,
    SENSORARRAY_S5D5_RAW_SPAN_BAND_ABSURD,
} sensorarrayS5d5RawSpanBand_t;

typedef enum {
    SENSORARRAY_S5D5_CANDIDATE_HEALTH_REJECTED = 0,
    SENSORARRAY_S5D5_CANDIDATE_HEALTH_DEGRADED = 1,
    SENSORARRAY_S5D5_CANDIDATE_HEALTH_MARGINAL = 2,
    SENSORARRAY_S5D5_CANDIDATE_HEALTH_MOSTLY_HEALTHY = 3,
    SENSORARRAY_S5D5_CANDIDATE_HEALTH_FULLY_HEALTHY = 4,
} sensorarrayS5d5CandidateHealthClass_t;

typedef struct {
    bool highCurrentReq;
    uint8_t driveStep;
    uint16_t driveCurrentReq;
    uint16_t driveCurrentNorm;
    bool configWriteOk;
    bool readbackValid;
    bool highCurrentReadback;
    uint16_t driveCurrentReadbackNorm;
    uint8_t activeChannelReadback;
    uint16_t statusRegReadback;
    uint16_t configRegReadback;
    uint16_t muxConfigReadback;
    uint32_t sampleCount;
    uint32_t i2cErrorCount;
    uint32_t transportReadableCount;
    uint32_t convertingCount;
    uint32_t unreadCount;
    uint32_t watchdogCount;
    uint32_t amplitudeCount;
    uint32_t nonZeroRawCount;
    uint32_t provisionalReadableCount;
    uint32_t healthReadableCount;
    uint32_t shouldCountForSweepCount;
    uint32_t sampleValidCount;
    uint32_t activeChannelMatchCount;
    uint32_t rawMin;
    uint32_t rawMax;
    uint64_t rawSum;
    uint32_t rawMean;
    uint32_t rawSpan;
    uint32_t rawSpanPermille;
    bool hasUnreadEvidence;
    bool hasWatchdogFault;
    bool hasAmplitudeFault;
    bool isConvertingStable;
    bool readbackActiveChannelMatch;
    bool readbackDriveMatch;
    bool readbackHighCurrentMatch;
    bool readbackMatches;
    bool analogRouteVerified;
    sensorarrayS5d5RawSpanBand_t rawSpanBand;
    sensorarrayS5d5CandidateHealthClass_t healthClass;
    bool passedValidityGate;
    const char *rejectReason;
    int32_t score;
    int32_t scoreHealth;
    int32_t scoreAmplitude;
    int32_t scoreStability;
    int32_t scoreWatchdog;
    int32_t scoreSpan;
    int32_t scoreReadback;
    int32_t scorePreference;
    bool selected;
    bool selectedFromValidPool;
} sensorarrayS5d5SweepCandidate_t;

typedef struct {
    double inductorValueUh;
    double fixedCapPf;
    double parasiticCapPf;
    bool enableCapComputation;
    bool enableNetCapOutput;
} sensorarrayS5d5CapComputationConfig_t;

typedef struct {
    uint32_t totalSamples;
    uint32_t goodSamples;
    uint32_t warningSamples;
    uint32_t amplitudeFaultSamples;
    uint32_t watchdogFaultSamples;
    uint32_t i2cErrorCount;
    uint32_t unreadTimeoutCount;
    uint32_t unreadPresentCount;
    uint32_t healthReadableCount;
    uint32_t nonConvertingCount;
    uint32_t nonZeroRawCount;
    uint32_t rawMin;
    uint32_t rawMax;
    uint64_t rawSum;
    uint32_t rawMean;
    uint32_t rawSpan;
    uint32_t rawSpanPermille;
    uint32_t freqSampleCount;
    uint32_t totalCapSampleCount;
    uint32_t netCapSampleCount;
    double freqHzSum;
    double totalCapPfSum;
    double netCapPfSum;
} sensorarrayS5d5LockedSummary_t;

typedef struct {
    bool valid;
    uint16_t statusReg;
    uint16_t configReg;
    uint16_t muxConfig;
    uint16_t driveReg;
    bool highCurrentReadback;
    uint16_t driveCurrentReadbackNorm;
    uint8_t activeChannelReadback;
    bool activeChannelMatch;
    bool matchesLockedCandidate;
} sensorarrayS5d5RuntimeReadback_t;

typedef struct {
    uint16_t statusRaw;
    bool converting;
    bool unreadPresent;
    bool errWatchdog;
    bool errAmplitude;
    uint32_t raw28;
    sensorarrayFdcSampleStatus_t statusCode;
} sensorarrayS5d5FastSample_t;

static const char *sensorarrayS5d5RawSpanBandName(sensorarrayS5d5RawSpanBand_t band)
{
    switch (band) {
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL:
        return "too_small";
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_LOW:
        return "healthy_low";
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_MID:
        return "healthy_mid";
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HIGH_NOISY:
        return "high_noisy";
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_ABSURD:
    default:
        return "absurd";
    }
}

static int32_t sensorarrayS5d5RawSpanBandRank(sensorarrayS5d5RawSpanBand_t band)
{
    switch (band) {
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_MID:
        return 4;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_LOW:
        return 3;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HIGH_NOISY:
        return 2;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_ABSURD:
        return 1;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL:
    default:
        return 0;
    }
}

static const char *sensorarrayS5d5CandidateHealthClassName(sensorarrayS5d5CandidateHealthClass_t healthClass)
{
    switch (healthClass) {
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_FULLY_HEALTHY:
        return "fully_healthy";
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_MOSTLY_HEALTHY:
        return "mostly_healthy";
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_MARGINAL:
        return "marginal";
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_DEGRADED:
        return "degraded";
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_REJECTED:
    default:
        return "rejected";
    }
}

static int32_t sensorarrayS5d5DrivePreferenceScore(uint8_t driveStep, bool highCurrentReq)
{
    int32_t delta = (int32_t)driveStep - 26;
    if (delta < 0) {
        delta = -delta;
    }
    int32_t score = 6 - (delta * 2);
    if (highCurrentReq) {
        score -= 1;
    }
    return score;
}

static sensorarrayS5d5RawSpanBand_t sensorarrayS5d5EvaluateRawSpanBand(const sensorarrayS5d5SweepCandidate_t *candidate)
{
    if (!candidate || candidate->nonZeroRawCount == 0u || candidate->rawMean == 0u) {
        return SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL;
    }
    if (candidate->rawSpanPermille < SENSORARRAY_S5D5_RAW_SPAN_TOO_SMALL_PERMILLE) {
        return SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL;
    }
    if (candidate->rawSpanPermille <= SENSORARRAY_S5D5_RAW_SPAN_HEALTHY_LOW_MAX_PERMILLE) {
        return SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_LOW;
    }
    if (candidate->rawSpanPermille <= SENSORARRAY_S5D5_RAW_SPAN_HEALTHY_MID_MAX_PERMILLE) {
        return SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_MID;
    }
    if (candidate->rawSpanPermille <= SENSORARRAY_S5D5_RAW_SPAN_HIGH_MAX_PERMILLE) {
        return SENSORARRAY_S5D5_RAW_SPAN_BAND_HIGH_NOISY;
    }
    return SENSORARRAY_S5D5_RAW_SPAN_BAND_ABSURD;
}

static sensorarrayS5d5CandidateHealthClass_t sensorarrayS5d5CandidateHealthClass(
    const sensorarrayS5d5SweepCandidate_t *candidate)
{
    if (!candidate || !candidate->passedValidityGate) {
        return SENSORARRAY_S5D5_CANDIDATE_HEALTH_REJECTED;
    }

    uint32_t denom = (candidate->sampleCount > 0u) ? candidate->sampleCount : 1u;
    uint32_t healthPermille = (uint32_t)((candidate->healthReadableCount * 1000u) / denom);
    bool faultsClean = (candidate->watchdogCount == 0u) && (candidate->amplitudeCount == 0u);
    bool spanHealthy = (candidate->rawSpanBand == SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_LOW) ||
                       (candidate->rawSpanBand == SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_MID);

    if (candidate->readbackMatches && faultsClean && spanHealthy && healthPermille >= 800u) {
        return SENSORARRAY_S5D5_CANDIDATE_HEALTH_FULLY_HEALTHY;
    }
    if (candidate->readbackMatches && spanHealthy && healthPermille >= 600u) {
        return SENSORARRAY_S5D5_CANDIDATE_HEALTH_MOSTLY_HEALTHY;
    }
    if (candidate->readbackActiveChannelMatch && healthPermille >= 350u) {
        return SENSORARRAY_S5D5_CANDIDATE_HEALTH_MARGINAL;
    }
    return SENSORARRAY_S5D5_CANDIDATE_HEALTH_DEGRADED;
}

static bool sensorarrayS5d5TryComputeCapacitance(const sensorarrayS5d5CapComputationConfig_t *capConfig,
                                                  double freqHz,
                                                  double *outTotalCapPf,
                                                  double *outNetCapPf,
                                                  bool *outHaveNetCapPf,
                                                  const char **outReason)
{
    if (outHaveNetCapPf) {
        *outHaveNetCapPf = false;
    }

    if (!outReason || !outTotalCapPf || !capConfig) {
        return false;
    }

    *outReason = "ok";
    if (!capConfig->enableCapComputation) {
        *outReason = "cap_computation_disabled";
        return false;
    }
    if (freqHz <= 0.0) {
        *outReason = "invalid_frequency";
        return false;
    }
    if (capConfig->inductorValueUh <= 0.0) {
        *outReason = "no_inductor_value";
        return false;
    }

    if (!sensorarrayMeasureFdcComputeCapacitancePf(freqHz, capConfig->inductorValueUh, outTotalCapPf)) {
        *outReason = "cap_compute_failed";
        return false;
    }

    if (capConfig->enableNetCapOutput && outNetCapPf) {
        // totalCapPf is LC-equivalent capacitance; netCapPf removes fixed + parasitic terms for debug delta view.
        *outNetCapPf = *outTotalCapPf - capConfig->fixedCapPf - capConfig->parasiticCapPf;
        if (outHaveNetCapPf) {
            *outHaveNetCapPf = true;
        }
    }

    return true;
}

static esp_err_t sensorarrayS5d5ReadKeyRegs(Fdc2214CapDevice_t *dev,
                                            uint16_t *outStatusReg,
                                            uint16_t *outConfigReg,
                                            uint16_t *outMuxConfig,
                                            uint16_t *outDriveCurrent)
{
    if (!dev || !outStatusReg || !outConfigReg || !outMuxConfig || !outDriveCurrent) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_STATUS, outStatusReg);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_CONFIG, outConfigReg);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, outMuxConfig);
    if (err != ESP_OK) {
        return err;
    }
    return Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, outDriveCurrent);
}

static uint8_t sensorarrayS5d5DataMsbRegForChannel(Fdc2214CapChannel_t channel)
{
    return (uint8_t)(SENSORARRAY_S5D5_REG_DATA_MSB_BASE + ((uint8_t)channel * 2u));
}

static esp_err_t sensorarrayS5d5ReadFastSample(Fdc2214CapDevice_t *dev,
                                               Fdc2214CapChannel_t channel,
                                               const Fdc2214CapStatus_t *statusHint,
                                               sensorarrayS5d5FastSample_t *outSample)
{
    if (!dev || !outSample || channel < FDC2214_CH0 || channel > FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }

    Fdc2214CapStatus_t status = {0};
    if (statusHint) {
        status = *statusHint;
    } else {
        esp_err_t statusErr = Fdc2214CapReadStatus(dev, &status);
        if (statusErr != ESP_OK) {
            return statusErr;
        }
    }

    uint8_t msbReg = sensorarrayS5d5DataMsbRegForChannel(channel);
    uint16_t dataMsb = 0u;
    uint16_t dataLsb = 0u;
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, msbReg, &dataMsb);
    if (err != ESP_OK) {
        return err;
    }
    err = Fdc2214CapReadRawRegisters(dev, (uint8_t)(msbReg + 1u), &dataLsb);
    if (err != ESP_OK) {
        return err;
    }

    outSample->statusRaw = status.Raw;
    outSample->converting = status.UnreadConversion[(uint8_t)channel] || status.DataReady;
    outSample->unreadPresent = status.UnreadConversion[(uint8_t)channel];
    outSample->raw28 = (((uint32_t)(dataMsb & SENSORARRAY_S5D5_DATA_MSB_MASK)) << 16) | (uint32_t)dataLsb;
    outSample->errWatchdog = ((dataMsb & SENSORARRAY_S5D5_DATA_ERR_WD_MASK) != 0u) || status.ErrWatchdog;
    outSample->errAmplitude = ((dataMsb & SENSORARRAY_S5D5_DATA_ERR_AW_MASK) != 0u) ||
                              status.ErrAmplitudeHigh ||
                              status.ErrAmplitudeLow;

    if (!outSample->converting) {
        outSample->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING;
    } else if (!outSample->unreadPresent) {
        outSample->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION;
    } else if (outSample->errWatchdog) {
        outSample->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT;
    } else if (outSample->errAmplitude) {
        outSample->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT;
    } else if (outSample->raw28 == 0u) {
        outSample->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID;
    } else {
        outSample->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID;
    }

    return ESP_OK;
}

static void sensorarrayS5d5StopAdsForIsolation(sensorarrayState_t *state)
{
    if (!state || !state->adsReady) {
        return;
    }
    if (!state->adsAdc1Running) {
        return;
    }

    esp_err_t stopErr = ads126xAdcStopAdc1(&state->ads);
    if (stopErr == ESP_OK) {
        state->adsAdc1Running = false;
    }
    printf("DBGFDC_S5D5,stage=ads_isolation,err=%ld,status=%s\n",
           (long)stopErr,
           (stopErr == ESP_OK) ? "ads_stopped" : "ads_stop_failed_continue");
}

static esp_err_t sensorarrayInitS5d5SecondaryFdc(sensorarrayState_t *state,
                                                 sensorarrayFdcDeviceState_t *fdcState,
                                                 sensorarrayFdcInitDiag_t *outDiag)
{
    if (!state || !fdcState || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayBringupInitFdcDiag(outDiag);
    fdcState->i2cCtx = boardSupportGetI2c1Ctx();
    fdcState->i2cAddr = SENSORARRAY_FDC_I2C_ADDR_LOW;

    if (!fdcState->i2cCtx) {
        outDiag->status = "i2c1_ctx_missing";
        return ESP_ERR_INVALID_STATE;
    }

    if (fdcState->handle) {
        (void)Fdc2214CapDestroy(fdcState->handle);
        fdcState->handle = NULL;
    }
    fdcState->ready = false;

    Fdc2214CapDevice_t *newHandle = NULL;
    esp_err_t err = sensorarrayBringupInitFdcSingleChannel(fdcState->i2cCtx,
                                                            fdcState->i2cAddr,
                                                            FDC2214_CH0,
                                                            &newHandle,
                                                            outDiag);
    sensorarrayBringupApplyFdcInitResult(fdcState, newHandle, err, outDiag);
    return err;
}

static void sensorarrayS5d5CandidateInit(sensorarrayS5d5SweepCandidate_t *candidate,
                                         bool highCurrentReq,
                                         uint8_t driveStep,
                                         uint16_t driveCurrentReq)
{
    if (!candidate) {
        return;
    }

    uint16_t driveCurrentNorm = sensorarrayS5d5DriveRawFromStep(driveStep);
    *candidate = (sensorarrayS5d5SweepCandidate_t){
        .highCurrentReq = highCurrentReq,
        .driveStep = driveStep,
        .driveCurrentReq = driveCurrentReq,
        .driveCurrentNorm = driveCurrentNorm,
        .configWriteOk = true,
        .rawMin = UINT_MAX,
        .rawSpanBand = SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL,
        .healthClass = SENSORARRAY_S5D5_CANDIDATE_HEALTH_REJECTED,
        .rejectReason = "none",
    };
}

static void sensorarrayS5d5CandidateIngestDiag(sensorarrayS5d5SweepCandidate_t *candidate,
                                               const sensorarrayFdcReadDiag_t *diag,
                                               Fdc2214CapChannel_t channel)
{
    if (!candidate || !diag) {
        return;
    }

    candidate->sampleCount++;
    if (!diag->i2cOk) {
        candidate->i2cErrorCount++;
        return;
    }

    if (diag->transportReadable) {
        candidate->transportReadableCount++;
    }
    if (diag->sample.Converting) {
        candidate->convertingCount++;
    }
    if (diag->sample.UnreadConversionPresent) {
        candidate->unreadCount++;
    }
    if (diag->sample.ErrWatchdog) {
        candidate->watchdogCount++;
    }
    if (diag->sample.ErrAmplitude) {
        candidate->amplitudeCount++;
    }
    if (diag->provisionalReadable) {
        candidate->provisionalReadableCount++;
    }
    if (diag->healthReadable) {
        candidate->healthReadableCount++;
    }
    if (diag->shouldCountForSweep) {
        candidate->shouldCountForSweepCount++;
    }
    if (diag->sampleValid) {
        candidate->sampleValidCount++;
    }
    if (diag->sample.ActiveChannel == channel) {
        candidate->activeChannelMatchCount++;
    }
    if (diag->sample.Raw28 != 0u) {
        candidate->nonZeroRawCount++;
        if (diag->sample.Raw28 < candidate->rawMin) {
            candidate->rawMin = diag->sample.Raw28;
        }
        if (diag->sample.Raw28 > candidate->rawMax) {
            candidate->rawMax = diag->sample.Raw28;
        }
        candidate->rawSum += diag->sample.Raw28;
    }
}

static int32_t sensorarrayS5d5FinalizeCandidate(sensorarrayS5d5SweepCandidate_t *candidate)
{
    if (!candidate) {
        return INT_MIN;
    }

    uint32_t denom = (candidate->sampleCount > 0u) ? candidate->sampleCount : 1u;
    uint32_t validTransportCount = candidate->transportReadableCount;
    if (candidate->nonZeroRawCount > 0u) {
        candidate->rawMean = (uint32_t)(candidate->rawSum / candidate->nonZeroRawCount);
        candidate->rawSpan = candidate->rawMax - candidate->rawMin;
        if (candidate->rawMean > 0u) {
            candidate->rawSpanPermille = (uint32_t)(((uint64_t)candidate->rawSpan * 1000ull) / candidate->rawMean);
        } else {
            candidate->rawSpanPermille = 0u;
        }
    } else {
        candidate->rawMin = 0u;
        candidate->rawMax = 0u;
        candidate->rawMean = 0u;
        candidate->rawSpan = 0u;
        candidate->rawSpanPermille = 0u;
    }

    candidate->hasUnreadEvidence = (candidate->unreadCount > 0u);
    candidate->hasWatchdogFault = (candidate->watchdogCount > 0u);
    candidate->hasAmplitudeFault = (candidate->amplitudeCount > 0u);
    candidate->isConvertingStable =
        (validTransportCount > 0u) && ((candidate->convertingCount * 100u) >= (validTransportCount * 80u));
    candidate->readbackActiveChannelMatch =
        candidate->readbackValid && (candidate->activeChannelReadback == (uint8_t)FDC2214_CH0);
    candidate->readbackDriveMatch = candidate->readbackValid &&
                                    (candidate->driveCurrentReadbackNorm == candidate->driveCurrentNorm);
    candidate->readbackHighCurrentMatch = candidate->readbackValid &&
                                          (candidate->highCurrentReadback == candidate->highCurrentReq);
    candidate->readbackMatches = candidate->readbackActiveChannelMatch &&
                                 candidate->readbackDriveMatch &&
                                 candidate->readbackHighCurrentMatch;
    candidate->rawSpanBand = sensorarrayS5d5EvaluateRawSpanBand(candidate);

    candidate->passedValidityGate = false;
    candidate->rejectReason = "none";
    if (!candidate->configWriteOk) {
        candidate->rejectReason = "config_write_failed";
    } else if (candidate->sampleCount == 0u) {
        candidate->rejectReason = "no_samples";
    } else if (candidate->i2cErrorCount >= candidate->sampleCount) {
        candidate->rejectReason = "all_samples_i2c_error";
    } else if (candidate->convertingCount == 0u) {
        candidate->rejectReason = "converting_count_zero";
    } else if (candidate->nonZeroRawCount == 0u) {
        candidate->rejectReason = "nonzero_raw_count_zero";
    } else if (!candidate->hasUnreadEvidence) {
        candidate->rejectReason = "no_unread_evidence";
    } else if (!candidate->readbackValid) {
        candidate->rejectReason = "readback_unavailable";
    } else if (!candidate->readbackActiveChannelMatch) {
        candidate->rejectReason = "active_channel_readback_mismatch";
    } else if (candidate->healthReadableCount == 0u) {
        candidate->rejectReason = "no_health_readable_samples";
    } else if (candidate->rawSpanBand == SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL) {
        candidate->rejectReason = "raw_span_too_small_low_information";
    } else if (validTransportCount > 0u && (candidate->watchdogCount * 3u) >= validTransportCount) {
        candidate->rejectReason = "watchdog_fault_persistent";
    } else if (validTransportCount > 0u && (candidate->amplitudeCount * 3u) >= validTransportCount) {
        candidate->rejectReason = "amplitude_fault_persistent";
    } else {
        candidate->passedValidityGate = true;
        candidate->rejectReason = "none";
    }

    candidate->healthClass = sensorarrayS5d5CandidateHealthClass(candidate);
    candidate->analogRouteVerified =
        candidate->passedValidityGate &&
        (candidate->healthClass == SENSORARRAY_S5D5_CANDIDATE_HEALTH_FULLY_HEALTHY ||
         candidate->healthClass == SENSORARRAY_S5D5_CANDIDATE_HEALTH_MOSTLY_HEALTHY);

    candidate->scoreHealth = (int32_t)((candidate->healthReadableCount * 60u) / denom);
    candidate->scoreAmplitude = -(int32_t)((candidate->amplitudeCount * 70u) / denom);
    candidate->scoreStability = (int32_t)((candidate->convertingCount * 22u) / denom);
    candidate->scoreStability += (int32_t)((candidate->unreadCount * 24u) / denom);
    candidate->scoreStability += (int32_t)((candidate->sampleValidCount * 18u) / denom);
    candidate->scoreStability -= (int32_t)((candidate->i2cErrorCount * 50u) / denom);
    candidate->scoreWatchdog = -(int32_t)((candidate->watchdogCount * 60u) / denom);
    switch (candidate->rawSpanBand) {
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_MID:
        candidate->scoreSpan = 26;
        break;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HEALTHY_LOW:
        candidate->scoreSpan = 12;
        break;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_HIGH_NOISY:
        candidate->scoreSpan = -18;
        break;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_ABSURD:
        candidate->scoreSpan = -50;
        break;
    case SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL:
    default:
        candidate->scoreSpan = -70;
        break;
    }
    candidate->scoreReadback = candidate->readbackMatches ? 48 : -20;
    if (!candidate->readbackActiveChannelMatch) {
        candidate->scoreReadback -= 60;
    }
    if (!candidate->readbackDriveMatch) {
        candidate->scoreReadback -= 12;
    }
    if (!candidate->readbackHighCurrentMatch) {
        candidate->scoreReadback -= 12;
    }
    candidate->scorePreference = sensorarrayS5d5DrivePreferenceScore(candidate->driveStep, candidate->highCurrentReq);

    int32_t healthClassBonus = 0;
    switch (candidate->healthClass) {
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_FULLY_HEALTHY:
        healthClassBonus = 24;
        break;
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_MOSTLY_HEALTHY:
        healthClassBonus = 12;
        break;
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_MARGINAL:
        healthClassBonus = -8;
        break;
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_DEGRADED:
        healthClassBonus = -24;
        break;
    case SENSORARRAY_S5D5_CANDIDATE_HEALTH_REJECTED:
    default:
        healthClassBonus = -40;
        break;
    }

    int32_t score = candidate->scoreHealth +
                    candidate->scoreAmplitude +
                    candidate->scoreStability +
                    candidate->scoreWatchdog +
                    candidate->scoreSpan +
                    candidate->scoreReadback +
                    candidate->scorePreference +
                    healthClassBonus;
    if (!candidate->passedValidityGate) {
        score -= 260;
    }

    candidate->score = score;
    return score;
}

static sensorarrayS5d5RouteGateStatus_t sensorarrayS5d5ProbeAnalogRouteGate(sensorarrayFdcDeviceState_t *fdcState,
                                                                             const sensorarrayFdcDLineMap_t *fdcMap,
                                                                             uint32_t sampleCount,
                                                                             bool discardFirst,
                                                                             uint32_t recoveryThreshold,
                                                                             uint8_t driveStep,
                                                                             uint16_t driveCurrentReq,
                                                                             sensorarrayS5d5SweepCandidate_t *outCandidate,
                                                                             bool *outNeedRecovery,
                                                                             sensorarrayS5d5FaultReason_t *outRecoveryFault,
                                                                             const char **outRecoveryReason,
                                                                             uint32_t *outRecoveryStreak)
{
    if (!fdcState || !fdcState->handle || !fdcMap || !outCandidate || !outNeedRecovery || !outRecoveryFault ||
        !outRecoveryReason || !outRecoveryStreak) {
        return SENSORARRAY_S5D5_ROUTE_GATE_BLOCKED_BEFORE_SWEEP;
    }

    if (sampleCount == 0u) {
        sampleCount = 1u;
    }
    if (recoveryThreshold == 0u) {
        recoveryThreshold = 1u;
    }

    *outNeedRecovery = false;
    *outRecoveryFault = SENSORARRAY_S5D5_FAULT_NONE;
    *outRecoveryReason = SENSORARRAY_NA;
    *outRecoveryStreak = 0u;

    sensorarrayS5d5SweepCandidate_t candidate = {0};
    sensorarrayS5d5CandidateInit(&candidate, false, driveStep, driveCurrentReq);

    uint16_t appliedDriveRaw = 0u;
    esp_err_t cfgErr =
        sensorarrayApplyS5d5DriveStep(fdcState->handle, candidate.highCurrentReq, candidate.driveStep, &appliedDriveRaw);
    candidate.driveCurrentNorm = appliedDriveRaw;
    if (cfgErr != ESP_OK) {
        candidate.configWriteOk = false;
        candidate.i2cErrorCount++;
        candidate.sampleCount = 1u;
        *outNeedRecovery = true;
        *outRecoveryFault = SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED;
        *outRecoveryReason = "route_gate_apply_drive_failed";
        *outRecoveryStreak = 1u;
        *outCandidate = candidate;
        printf("DBGFDC_S5D5,stage=route_analog_probe,result=apply_failed,driveStep=%u,driveRawReq=0x%04X,"
               "driveRawMasked=0x%04X,err=%ld\n",
               (unsigned)candidate.driveStep,
               candidate.driveCurrentReq,
               candidate.driveCurrentNorm,
               (long)cfgErr);
        return SENSORARRAY_S5D5_ROUTE_GATE_BLOCKED_BEFORE_SWEEP;
    }

    sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SETTLE_MS);

    if (discardFirst) {
        sensorarrayFdcReadDiag_t discardDiag = {0};
        esp_err_t discardErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(fdcState->handle,
                                                                           fdcMap->channel,
                                                                           false,
                                                                           fdcState->haveIds,
                                                                           fdcState->configVerified,
                                                                           &discardDiag);
        if (discardErr != ESP_OK) {
            candidate.i2cErrorCount++;
        }
    }

    uint32_t i2cStreak = 0u;

    for (uint32_t sampleIndex = 0u; sampleIndex < sampleCount; ++sampleIndex) {
        sensorarrayFdcReadDiag_t diag = {0};
        esp_err_t readErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(fdcState->handle,
                                                                        fdcMap->channel,
                                                                        false,
                                                                        fdcState->haveIds,
                                                                        fdcState->configVerified,
                                                                        &diag);
        if (readErr != ESP_OK) {
            candidate.sampleCount++;
            candidate.i2cErrorCount++;
            i2cStreak++;
        } else {
            sensorarrayS5d5CandidateIngestDiag(&candidate, &diag, fdcMap->channel);
            i2cStreak = 0u;
        }

        if (i2cStreak >= recoveryThreshold) {
            bool sclHigh = true;
            bool sdaHigh = true;
            esp_err_t lineErr = boardSupportI2cCheckLines(fdcState->i2cCtx, &sclHigh, &sdaHigh);
            *outNeedRecovery = true;
            if (lineErr == ESP_OK && (!sclHigh || !sdaHigh)) {
                *outRecoveryFault = SENSORARRAY_S5D5_FAULT_I2C_LINE_STUCK;
                *outRecoveryReason = "route_gate_i2c_line_stuck";
            } else {
                *outRecoveryFault = SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED;
                *outRecoveryReason = "route_gate_i2c_error_streak";
            }
            *outRecoveryStreak = i2cStreak;
            break;
        }
        sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
    }

    if (!*outNeedRecovery) {
        uint16_t statusReg = 0u;
        uint16_t configReg = 0u;
        uint16_t muxConfig = 0u;
        uint16_t driveReg = 0u;
        esp_err_t regsErr = sensorarrayS5d5ReadKeyRegs(fdcState->handle, &statusReg, &configReg, &muxConfig, &driveReg);
        if (regsErr == ESP_OK) {
            candidate.readbackValid = true;
            candidate.statusRegReadback = statusReg;
            candidate.configRegReadback = configReg;
            candidate.muxConfigReadback = muxConfig;
            candidate.highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
            candidate.driveCurrentReadbackNorm = (uint16_t)(driveReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
            candidate.activeChannelReadback =
                (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >> SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
        } else {
            candidate.i2cErrorCount++;
            *outNeedRecovery = true;
            *outRecoveryFault = SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED;
            *outRecoveryReason = "route_gate_readback_failed";
            *outRecoveryStreak = 1u;
        }
    }

    (void)sensorarrayS5d5FinalizeCandidate(&candidate);
    *outCandidate = candidate;

    sensorarrayS5d5RouteGateStatus_t routeGateStatus = candidate.analogRouteVerified
                                                            ? SENSORARRAY_S5D5_ROUTE_GATE_OK
                                                            : SENSORARRAY_S5D5_ROUTE_GATE_GPIO_OK_ANALOG_UNVERIFIED;
    if (*outNeedRecovery) {
        routeGateStatus = SENSORARRAY_S5D5_ROUTE_GATE_BLOCKED_BEFORE_SWEEP;
    }

    const char *probeAction = *outNeedRecovery ? "pre_sweep_block_hard_fault" : "continue_to_sweep";

    printf("DBGFDC_S5D5,stage=route_analog_probe,highCurrentReq=%u,driveStep=%u,driveCurrentReq=0x%04X,"
           "driveCurrentNorm=0x%04X,samples=%lu,i2cErr=%lu,"
           "transportReadable=%lu,healthReadable=%lu,unread=%lu,watchdog=%lu,amplitude=%lu,rawSpanPermille=%lu,"
           "rawSpanBand=%s,healthClass=%s,analogRouteVerified=%u,passedValidityGate=%u,rejectReason=%s,gate=%s,"
           "probeAction=%s,recoveryReason=%s\n",
           candidate.highCurrentReq ? 1u : 0u,
           (unsigned)candidate.driveStep,
           candidate.driveCurrentReq,
           candidate.driveCurrentNorm,
           (unsigned long)candidate.sampleCount,
           (unsigned long)candidate.i2cErrorCount,
           (unsigned long)candidate.transportReadableCount,
           (unsigned long)candidate.healthReadableCount,
           (unsigned long)candidate.unreadCount,
           (unsigned long)candidate.watchdogCount,
           (unsigned long)candidate.amplitudeCount,
           (unsigned long)candidate.rawSpanPermille,
           sensorarrayS5d5RawSpanBandName(candidate.rawSpanBand),
           sensorarrayS5d5CandidateHealthClassName(candidate.healthClass),
           candidate.analogRouteVerified ? 1u : 0u,
           candidate.passedValidityGate ? 1u : 0u,
           candidate.rejectReason ? candidate.rejectReason : SENSORARRAY_NA,
           sensorarrayS5d5RouteGateStatusName(routeGateStatus),
           probeAction,
           *outRecoveryReason ? *outRecoveryReason : SENSORARRAY_NA);

    return routeGateStatus;
}

static const char *sensorarrayS5d5CandidateStatus(const sensorarrayS5d5SweepCandidate_t *candidate, int32_t minScore)
{
    if (!candidate) {
        return "invalid";
    }
    if (!candidate->configWriteOk) {
        return "config_write_error";
    }
    if (!candidate->passedValidityGate) {
        return candidate->rejectReason ? candidate->rejectReason : "rejected";
    }
    if (!candidate->analogRouteVerified) {
        return "valid_but_analog_unhealthy";
    }
    if (candidate->score < minScore) {
        return "valid_but_below_min_score";
    }
    if (!candidate->readbackMatches) {
        return "valid_with_readback_warning";
    }
    return "fully_healthy_valid";
}

static bool sensorarrayS5d5CandidateBetter(const sensorarrayS5d5SweepCandidate_t *candidate,
                                           const sensorarrayS5d5SweepCandidate_t *best)
{
    if (!candidate) {
        return false;
    }
    if (!best) {
        return true;
    }

    if (candidate->passedValidityGate != best->passedValidityGate) {
        return candidate->passedValidityGate;
    }

    if (candidate->passedValidityGate) {
        if (candidate->readbackMatches != best->readbackMatches) {
            return candidate->readbackMatches;
        }
        if (candidate->healthClass != best->healthClass) {
            return candidate->healthClass > best->healthClass;
        }
        if (candidate->shouldCountForSweepCount != best->shouldCountForSweepCount) {
            return candidate->shouldCountForSweepCount > best->shouldCountForSweepCount;
        }
        if (candidate->unreadCount != best->unreadCount) {
            return candidate->unreadCount > best->unreadCount;
        }
        if (candidate->convertingCount != best->convertingCount) {
            return candidate->convertingCount > best->convertingCount;
        }
        if (candidate->watchdogCount != best->watchdogCount) {
            return candidate->watchdogCount < best->watchdogCount;
        }
        if (candidate->amplitudeCount != best->amplitudeCount) {
            return candidate->amplitudeCount < best->amplitudeCount;
        }
        if (sensorarrayS5d5RawSpanBandRank(candidate->rawSpanBand) !=
            sensorarrayS5d5RawSpanBandRank(best->rawSpanBand)) {
            return sensorarrayS5d5RawSpanBandRank(candidate->rawSpanBand) >
                   sensorarrayS5d5RawSpanBandRank(best->rawSpanBand);
        }
        if (candidate->i2cErrorCount != best->i2cErrorCount) {
            return candidate->i2cErrorCount < best->i2cErrorCount;
        }
        if (candidate->sampleValidCount != best->sampleValidCount) {
            return candidate->sampleValidCount > best->sampleValidCount;
        }
        if (candidate->score != best->score) {
            return candidate->score > best->score;
        }
        if (candidate->scorePreference != best->scorePreference) {
            return candidate->scorePreference > best->scorePreference;
        }
        if (candidate->highCurrentReq != best->highCurrentReq) {
            return !candidate->highCurrentReq && best->highCurrentReq;
        }
        return candidate->driveStep < best->driveStep;
    }

    // Fallback ranking when every candidate failed validity gate.
    if (candidate->configWriteOk != best->configWriteOk) {
        return candidate->configWriteOk;
    }
    if (candidate->readbackValid != best->readbackValid) {
        return candidate->readbackValid;
    }
    if (candidate->transportReadableCount != best->transportReadableCount) {
        return candidate->transportReadableCount > best->transportReadableCount;
    }
    if (candidate->convertingCount != best->convertingCount) {
        return candidate->convertingCount > best->convertingCount;
    }
    if (candidate->unreadCount != best->unreadCount) {
        return candidate->unreadCount > best->unreadCount;
    }
    if (candidate->healthReadableCount != best->healthReadableCount) {
        return candidate->healthReadableCount > best->healthReadableCount;
    }
    if (candidate->i2cErrorCount != best->i2cErrorCount) {
        return candidate->i2cErrorCount < best->i2cErrorCount;
    }
    if (candidate->watchdogCount != best->watchdogCount) {
        return candidate->watchdogCount < best->watchdogCount;
    }
    if (candidate->amplitudeCount != best->amplitudeCount) {
        return candidate->amplitudeCount < best->amplitudeCount;
    }
    if (sensorarrayS5d5RawSpanBandRank(candidate->rawSpanBand) !=
        sensorarrayS5d5RawSpanBandRank(best->rawSpanBand)) {
        return sensorarrayS5d5RawSpanBandRank(candidate->rawSpanBand) >
               sensorarrayS5d5RawSpanBandRank(best->rawSpanBand);
    }
    if (candidate->score != best->score) {
        return candidate->score > best->score;
    }
    if (candidate->scorePreference != best->scorePreference) {
        return candidate->scorePreference > best->scorePreference;
    }
    return candidate->driveStep < best->driveStep;
}

static const char *sensorarrayS5d5CandidateBetterReason(const sensorarrayS5d5SweepCandidate_t *candidate,
                                                        const sensorarrayS5d5SweepCandidate_t *best)
{
    if (!best) {
        return "first_candidate";
    }
    if (candidate->passedValidityGate != best->passedValidityGate) {
        return candidate->passedValidityGate ? "passed_validity_gate" : "fallback_pool";
    }
    if (candidate->passedValidityGate) {
        if (candidate->readbackMatches != best->readbackMatches) {
            return candidate->readbackMatches ? "readback_match_priority" : "readback_mismatch";
        }
        if (candidate->healthClass != best->healthClass) {
            return "health_class_better";
        }
        if (candidate->shouldCountForSweepCount != best->shouldCountForSweepCount) {
            return "health_readable_ratio_better";
        }
        if (candidate->unreadCount != best->unreadCount) {
            return "unread_stability_better";
        }
        if (candidate->convertingCount != best->convertingCount) {
            return "converting_stability_better";
        }
        if (candidate->watchdogCount != best->watchdogCount || candidate->amplitudeCount != best->amplitudeCount) {
            return "fault_count_lower";
        }
        if (sensorarrayS5d5RawSpanBandRank(candidate->rawSpanBand) !=
            sensorarrayS5d5RawSpanBandRank(best->rawSpanBand)) {
            return "raw_span_band_healthier";
        }
        if (candidate->i2cErrorCount != best->i2cErrorCount) {
            return "i2c_error_lower";
        }
        if (candidate->score != best->score) {
            return "score_higher";
        }
        return "tie_breaker";
    }

    if (candidate->configWriteOk != best->configWriteOk) {
        return "config_write_ok_priority";
    }
    if (candidate->transportReadableCount != best->transportReadableCount) {
        return "transport_readable_more";
    }
    if (candidate->healthReadableCount != best->healthReadableCount) {
        return "health_readable_more";
    }
    if (candidate->i2cErrorCount != best->i2cErrorCount) {
        return "i2c_error_lower";
    }
    if (candidate->score != best->score) {
        return "fallback_score_higher";
    }
    return "fallback_tie_breaker";
}

static bool sensorarrayS5d5SweepCandidates(sensorarrayFdcDeviceState_t *fdcState,
                                           const sensorarrayFdcDLineMap_t *fdcMap,
                                           uint32_t sweepSampleCount,
                                           bool discardFirst,
                                           bool forceSingleDrive,
                                           uint8_t forcedDriveStep,
                                           uint16_t forcedDriveCurrentReq,
                                           int32_t minScore,
                                           const sensorarrayCheckpointGpio_t *checkpoint,
                                           sensorarrayS5d5SweepCandidate_t *outBestCandidate,
                                           bool *outSelectedFromValidPool,
                                           bool *outNeedRecovery,
                                           sensorarrayS5d5FaultReason_t *outRecoveryFault,
                                           const char **outRecoveryReason,
                                           uint32_t *outRecoveryStreak)
{
    if (!fdcState || !fdcState->handle || !fdcMap || !outBestCandidate || !outSelectedFromValidPool ||
        !outNeedRecovery || !outRecoveryFault || !outRecoveryReason || !outRecoveryStreak) {
        return false;
    }

    *outSelectedFromValidPool = false;
    *outNeedRecovery = false;
    *outRecoveryFault = SENSORARRAY_S5D5_FAULT_NONE;
    *outRecoveryReason = SENSORARRAY_NA;
    *outRecoveryStreak = 0u;
    size_t highCount = sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE) /
                       sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[0]);
    size_t driveCount = forceSingleDrive
                            ? 1u
                            : (sizeof(SENSORARRAY_S5D5_DRIVE_STEP_SWEEP_TABLE) /
                               sizeof(SENSORARRAY_S5D5_DRIVE_STEP_SWEEP_TABLE[0]));
    uint16_t forcedDriveCurrentNorm = sensorarrayS5d5DriveRawFromStep(forcedDriveStep);
    printf("DBGFDC_S5D5,stage=sweep_begin,candidates=%u,samplesPerCandidate=%lu,forceSingleDrive=%u,"
           "forcedDriveStep=%u,forcedDriveRawReq=0x%04X,forcedDriveRawMasked=0x%04X\n",
           (unsigned)(highCount * driveCount),
           (unsigned long)sweepSampleCount,
           forceSingleDrive ? 1u : 0u,
           (unsigned)forcedDriveStep,
           forcedDriveCurrentReq,
           forcedDriveCurrentNorm);

    bool haveBestValid = false;
    bool haveBestFallback = false;
    sensorarrayS5d5SweepCandidate_t bestValidCandidate = {0};
    sensorarrayS5d5SweepCandidate_t bestFallbackCandidate = {0};
    uint32_t recoveryThreshold = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RECOVERY_REINIT_ERR_THRESHOLD;
    if (recoveryThreshold == 0u) {
        recoveryThreshold = 3u;
    }

    for (size_t highIndex = 0u; highIndex < highCount; ++highIndex) {
        bool highCurrentReq = SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[highIndex];
        for (size_t driveIndex = 0u; driveIndex < driveCount; ++driveIndex) {
            sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_STEP_BEGIN);

            sensorarrayS5d5SweepCandidate_t candidate = {0};
            uint8_t driveStep = forceSingleDrive ? forcedDriveStep : SENSORARRAY_S5D5_DRIVE_STEP_SWEEP_TABLE[driveIndex];
            uint16_t driveReqRaw = forceSingleDrive ? forcedDriveCurrentReq : sensorarrayS5d5DriveRawFromStep(driveStep);
            sensorarrayS5d5CandidateInit(&candidate, highCurrentReq, driveStep, driveReqRaw);

            uint16_t appliedDriveRaw = 0u;
            esp_err_t cfgErr =
                sensorarrayApplyS5d5DriveStep(fdcState->handle, candidate.highCurrentReq, candidate.driveStep, &appliedDriveRaw);
            candidate.driveCurrentNorm = appliedDriveRaw;
            if (cfgErr != ESP_OK) {
                candidate.configWriteOk = false;
                candidate.i2cErrorCount++;
                candidate.sampleCount = 1u;
            } else {
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SETTLE_MS);
            }

            if (cfgErr == ESP_OK && discardFirst) {
                sensorarrayFdcReadDiag_t discardDiag = {0};
                esp_err_t discardErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(fdcState->handle,
                                                                                   fdcMap->channel,
                                                                                   false,
                                                                                   fdcState->haveIds,
                                                                                   fdcState->configVerified,
                                                                                   &discardDiag);
                if (discardErr != ESP_OK) {
                    candidate.i2cErrorCount++;
                }
            }

            if (cfgErr == ESP_OK) {
                uint32_t i2cReadStreak = 0u;
                for (uint32_t sampleIndex = 0u; sampleIndex < sweepSampleCount; ++sampleIndex) {
                    sensorarrayFdcReadDiag_t diag = {0};
                    esp_err_t readErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(fdcState->handle,
                                                                                    fdcMap->channel,
                                                                                    false,
                                                                                    fdcState->haveIds,
                                                                                    fdcState->configVerified,
                                                                                    &diag);
                    if (readErr != ESP_OK) {
                        candidate.sampleCount++;
                        candidate.i2cErrorCount++;
                        i2cReadStreak++;
                    } else {
                        sensorarrayS5d5CandidateIngestDiag(&candidate, &diag, fdcMap->channel);
                        i2cReadStreak = 0u;
                    }
                    if (i2cReadStreak >= recoveryThreshold) {
                        bool sclHigh = true;
                        bool sdaHigh = true;
                        esp_err_t lineErr = boardSupportI2cCheckLines(fdcState->i2cCtx, &sclHigh, &sdaHigh);
                        *outNeedRecovery = true;
                        if (lineErr == ESP_OK && (!sclHigh || !sdaHigh)) {
                            *outRecoveryFault = SENSORARRAY_S5D5_FAULT_I2C_LINE_STUCK;
                            *outRecoveryReason = "sweep_i2c_line_stuck";
                        } else {
                            *outRecoveryFault = SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED;
                            *outRecoveryReason = "sweep_i2c_error_streak";
                        }
                        *outRecoveryStreak = i2cReadStreak;
                        printf("DBGFDC_S5D5,stage=sweep_abort,reason=%s,streak=%lu,lineCheckErr=%ld,sclHigh=%d,sdaHigh=%d\n",
                               *outRecoveryReason,
                               (unsigned long)i2cReadStreak,
                               (long)lineErr,
                               (lineErr == ESP_OK) ? (sclHigh ? 1 : 0) : -1,
                               (lineErr == ESP_OK) ? (sdaHigh ? 1 : 0) : -1);
                        break;
                    }
                    sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
                }
            }
            if (*outNeedRecovery) {
                return false;
            }

            if (cfgErr == ESP_OK) {
                uint16_t statusReg = 0u;
                uint16_t configReg = 0u;
                uint16_t muxConfig = 0u;
                uint16_t driveReg = 0u;
                esp_err_t regsErr =
                    sensorarrayS5d5ReadKeyRegs(fdcState->handle, &statusReg, &configReg, &muxConfig, &driveReg);
                if (regsErr == ESP_OK) {
                    candidate.readbackValid = true;
                    candidate.statusRegReadback = statusReg;
                    candidate.configRegReadback = configReg;
                    candidate.muxConfigReadback = muxConfig;
                    candidate.highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
                    candidate.driveCurrentReadbackNorm = (uint16_t)(driveReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
                    candidate.activeChannelReadback =
                        (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >>
                                  SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
                } else {
                    candidate.i2cErrorCount++;
                    *outNeedRecovery = true;
                    *outRecoveryFault = SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED;
                    *outRecoveryReason = "sweep_readback_failed";
                    *outRecoveryStreak = 1u;
                }
            }
            if (*outNeedRecovery) {
                return false;
            }

            (void)sensorarrayS5d5FinalizeCandidate(&candidate);
            const char *candidateStatus = sensorarrayS5d5CandidateStatus(&candidate, minScore);
            char scoreBreakdown[192] = {0};
            (void)snprintf(scoreBreakdown,
                           sizeof(scoreBreakdown),
                           "health=%ld,amp=%ld,stability=%ld,watchdog=%ld,span=%ld,readback=%ld,pref=%ld,total=%ld",
                           (long)candidate.scoreHealth,
                           (long)candidate.scoreAmplitude,
                           (long)candidate.scoreStability,
                           (long)candidate.scoreWatchdog,
                           (long)candidate.scoreSpan,
                           (long)candidate.scoreReadback,
                           (long)candidate.scorePreference,
                           (long)candidate.score);
            printf("DBGFDC_S5D5,stage=sweep_candidate,highCurrentReq=%u,highCurrentReadback=%u,driveStep=%u,"
                   "driveCurrentReq=0x%04X,driveCurrentNorm=0x%04X,driveCurrentReadback=0x%04X,"
                   "activeChannelReadback=%u,activeChannelReadbackMatch=%u,samples=%lu,"
                   "i2cErr=%lu,transportReadable=%lu,provisionalReadable=%lu,healthReadable=%lu,validSamples=%lu,"
                   "sweepCountable=%lu,convertingOk=%lu,unreadOk=%lu,watchdog=%lu,amplitude=%lu,nonZeroRaw=%lu,"
                   "rawMin=%lu,rawMax=%lu,rawAvg=%lu,rawSpan=%lu,rawSpanPermille=%lu,rawSpanBand=%s,statusReg=0x%04X,"
                   "configReg=0x%04X,muxConfig=0x%04X,isConfigWriteOk=%u,isConvertingStable=%u,hasUnreadEvidence=%u,"
                   "hasWatchdogFault=%u,hasAmplitudeFault=%u,readbackMatches=%u,healthClass=%s,analogRouteVerified=%u,"
                   "passedValidityGate=%u,rejectReason=%s,scoreBreakdown=%s,status=%s\n",
                   candidate.highCurrentReq ? 1u : 0u,
                   candidate.highCurrentReadback ? 1u : 0u,
                   (unsigned)candidate.driveStep,
                   candidate.driveCurrentReq,
                   candidate.driveCurrentNorm,
                   candidate.driveCurrentReadbackNorm,
                   (unsigned)candidate.activeChannelReadback,
                   candidate.readbackActiveChannelMatch ? 1u : 0u,
                   (unsigned long)candidate.sampleCount,
                   (unsigned long)candidate.i2cErrorCount,
                   (unsigned long)candidate.transportReadableCount,
                   (unsigned long)candidate.provisionalReadableCount,
                   (unsigned long)candidate.healthReadableCount,
                   (unsigned long)candidate.sampleValidCount,
                   (unsigned long)candidate.shouldCountForSweepCount,
                   (unsigned long)candidate.convertingCount,
                   (unsigned long)candidate.unreadCount,
                   (unsigned long)candidate.watchdogCount,
                   (unsigned long)candidate.amplitudeCount,
                   (unsigned long)candidate.nonZeroRawCount,
                   (unsigned long)candidate.rawMin,
                   (unsigned long)candidate.rawMax,
                   (unsigned long)candidate.rawMean,
                   (unsigned long)candidate.rawSpan,
                   (unsigned long)candidate.rawSpanPermille,
                   sensorarrayS5d5RawSpanBandName(candidate.rawSpanBand),
                   candidate.statusRegReadback,
                   candidate.configRegReadback,
                   candidate.muxConfigReadback,
                   candidate.configWriteOk ? 1u : 0u,
                   candidate.isConvertingStable ? 1u : 0u,
                   candidate.hasUnreadEvidence ? 1u : 0u,
                   candidate.hasWatchdogFault ? 1u : 0u,
                   candidate.hasAmplitudeFault ? 1u : 0u,
                   candidate.readbackMatches ? 1u : 0u,
                   sensorarrayS5d5CandidateHealthClassName(candidate.healthClass),
                   candidate.analogRouteVerified ? 1u : 0u,
                   candidate.passedValidityGate ? 1u : 0u,
                   candidate.rejectReason ? candidate.rejectReason : SENSORARRAY_NA,
                   scoreBreakdown,
                   candidateStatus);
            printf("DBGFDC_S5D5,stage=sweep_candidate_decision,highCurrentReq=%u,driveStep=%u,result=%s,"
                   "rejectReason=%s,status=%s\n",
                   candidate.highCurrentReq ? 1u : 0u,
                   (unsigned)candidate.driveStep,
                   candidate.passedValidityGate ? "candidate_valid" : "candidate_diagnostic_only",
                   candidate.rejectReason ? candidate.rejectReason : SENSORARRAY_NA,
                   candidateStatus);

            if (candidate.passedValidityGate) {
                if (!haveBestValid || sensorarrayS5d5CandidateBetter(&candidate, &bestValidCandidate)) {
                    const sensorarrayS5d5SweepCandidate_t *prev = haveBestValid ? &bestValidCandidate : NULL;
                    bestValidCandidate = candidate;
                    haveBestValid = true;
                    printf("DBGFDC_S5D5,stage=sweep_best_update,pool=valid,reason=%s,newScore=%ld,oldScore=%ld,"
                           "newDriveStep=%u,newDriveCurrentReq=0x%04X,newHighCurrentReq=%u\n",
                           sensorarrayS5d5CandidateBetterReason(&candidate, prev),
                           (long)candidate.score,
                           prev ? (long)prev->score : (long)INT_MIN,
                           (unsigned)candidate.driveStep,
                           candidate.driveCurrentReq,
                           candidate.highCurrentReq ? 1u : 0u);
                }
            } else if (!haveBestFallback || sensorarrayS5d5CandidateBetter(&candidate, &bestFallbackCandidate)) {
                const sensorarrayS5d5SweepCandidate_t *prev = haveBestFallback ? &bestFallbackCandidate : NULL;
                bestFallbackCandidate = candidate;
                haveBestFallback = true;
                printf("DBGFDC_S5D5,stage=sweep_best_update,pool=fallback,reason=%s,newScore=%ld,oldScore=%ld,"
                       "newDriveStep=%u,newDriveCurrentReq=0x%04X,newHighCurrentReq=%u\n",
                       sensorarrayS5d5CandidateBetterReason(&candidate, prev),
                       (long)candidate.score,
                       prev ? (long)prev->score : (long)INT_MIN,
                       (unsigned)candidate.driveStep,
                       candidate.driveCurrentReq,
                       candidate.highCurrentReq ? 1u : 0u);
            }
        }
    }

    if (!haveBestValid && !haveBestFallback) {
        return false;
    }

    sensorarrayS5d5SweepCandidate_t selectedCandidate = haveBestValid ? bestValidCandidate : bestFallbackCandidate;
    selectedCandidate.selected = true;
    selectedCandidate.selectedFromValidPool = haveBestValid;
    *outBestCandidate = selectedCandidate;
    *outSelectedFromValidPool = haveBestValid;

    char scoreBreakdown[192] = {0};
    (void)snprintf(scoreBreakdown,
                   sizeof(scoreBreakdown),
                   "health=%ld,amp=%ld,stability=%ld,watchdog=%ld,span=%ld,readback=%ld,pref=%ld,total=%ld",
                   (long)selectedCandidate.scoreHealth,
                   (long)selectedCandidate.scoreAmplitude,
                   (long)selectedCandidate.scoreStability,
                   (long)selectedCandidate.scoreWatchdog,
                   (long)selectedCandidate.scoreSpan,
                   (long)selectedCandidate.scoreReadback,
                   (long)selectedCandidate.scorePreference,
                   (long)selectedCandidate.score);

    printf("DBGFDC_S5D5,stage=sweep_selected,highCurrentReq=%u,driveStep=%u,driveCurrentReq=0x%04X,"
            "driveCurrentNorm=0x%04X,"
            "driveCurrentReadback=0x%04X,activeChannelReadback=%u,samples=%lu,i2cErr=%lu,convertingOk=%lu,unreadOk=%lu,"
           "watchdog=%lu,amplitude=%lu,nonZeroRaw=%lu,validSamples=%lu,healthReadable=%lu,rawMin=%lu,rawMax=%lu,"
           "rawAvg=%lu,rawSpan=%lu,rawSpanPermille=%lu,rawSpanBand=%s,healthClass=%s,analogRouteVerified=%u,"
           "passedValidityGate=%u,rejectReason=%s,selectionType=%s,scoreBreakdown=%s,status=%s\n",
           selectedCandidate.highCurrentReq ? 1u : 0u,
           (unsigned)selectedCandidate.driveStep,
           selectedCandidate.driveCurrentReq,
           selectedCandidate.driveCurrentNorm,
           selectedCandidate.driveCurrentReadbackNorm,
           (unsigned)selectedCandidate.activeChannelReadback,
           (unsigned long)selectedCandidate.sampleCount,
           (unsigned long)selectedCandidate.i2cErrorCount,
           (unsigned long)selectedCandidate.convertingCount,
           (unsigned long)selectedCandidate.unreadCount,
           (unsigned long)selectedCandidate.watchdogCount,
           (unsigned long)selectedCandidate.amplitudeCount,
           (unsigned long)selectedCandidate.nonZeroRawCount,
           (unsigned long)selectedCandidate.sampleValidCount,
           (unsigned long)selectedCandidate.healthReadableCount,
           (unsigned long)selectedCandidate.rawMin,
           (unsigned long)selectedCandidate.rawMax,
           (unsigned long)selectedCandidate.rawMean,
            (unsigned long)selectedCandidate.rawSpan,
           (unsigned long)selectedCandidate.rawSpanPermille,
           sensorarrayS5d5RawSpanBandName(selectedCandidate.rawSpanBand),
           sensorarrayS5d5CandidateHealthClassName(selectedCandidate.healthClass),
           selectedCandidate.analogRouteVerified ? 1u : 0u,
           selectedCandidate.passedValidityGate ? 1u : 0u,
           selectedCandidate.rejectReason ? selectedCandidate.rejectReason : SENSORARRAY_NA,
           haveBestValid ? "selected_best_valid_candidate" : "selected_fallback_candidate",
           scoreBreakdown,
           sensorarrayS5d5CandidateStatus(&selectedCandidate, minScore));
    return true;
}

static bool sensorarrayS5d5RunMathSelfCheck(void)
{
    const uint32_t raw28 = 8150000u;
    const uint32_t refClockHz = 40000000u;
    const uint16_t clockDividers = 0x2001u;

    double legacyFreqHz = sensorarrayMeasureFdcRawToFrequencyHz(raw28, refClockHz);
    Fdc2214CapClockDividerInfo_t clockInfo = {0};
    bool decodeOk = (Fdc2214CapDecodeClockDividers(clockDividers, &clockInfo) == ESP_OK);

    sensorarrayFdcFrequencyRestore_t restored = {0};
    bool restoreOk = decodeOk &&
                     sensorarrayMeasureFdcRestoreFrequencyWithClockInfo(raw28,
                                                                        refClockHz,
                                                                        SENSORARRAY_FDC_REF_CLOCK_QUALITY_NOMINAL_INTERNAL,
                                                                        &clockInfo,
                                                                        &restored);
    bool freqCasePass = restoreOk &&
                        (legacyFreqHz > 1.1e6) && (legacyFreqHz < 1.3e6) &&
                        (restored.restoredSensorFrequencyHz > 2.2e6) &&
                        (restored.restoredSensorFrequencyHz < 2.6e6);
    printf("DBGFDC_S5D5,stage=math_selfcheck_case1,clockDiv0Raw=0x%04X,raw=%lu,refClockHz=%lu,legacyFreqHz=%.3f,"
           "restoredSensorFreqHz=%.3f,result=%s\n",
           clockDividers,
           (unsigned long)raw28,
           (unsigned long)refClockHz,
           legacyFreqHz,
           restored.restoredSensorFrequencyHz,
           freqCasePass ? "pass" : "fail");

    double capLegacyPf = 0.0;
    double capRestoredPf = 0.0;
    bool capLegacyOk = sensorarrayMeasureFdcComputeCapacitancePf(legacyFreqHz, 18.0, &capLegacyPf);
    bool capRestoredOk = sensorarrayMeasureFdcComputeCapacitancePf(restored.restoredSensorFrequencyHz, 18.0, &capRestoredPf);
    double capRatio = (capLegacyOk && capRestoredOk && capLegacyPf > 0.0) ? (capRestoredPf / capLegacyPf) : 0.0;
    bool capCasePass = capLegacyOk && capRestoredOk && (capRatio > 0.20) && (capRatio < 0.30);
    printf("DBGFDC_S5D5,stage=math_selfcheck_case2,capAtLegacyPf=%.6f,capAtRestoredPf=%.6f,ratio=%.6f,result=%s\n",
           capLegacyPf,
           capRestoredPf,
           capRatio,
           capCasePass ? "pass" : "fail");

    sensorarrayS5d5SweepCandidate_t ampHealthy = {0};
    sensorarrayS5d5SweepCandidate_t ampFaulty = {0};
    sensorarrayS5d5CandidateInit(&ampHealthy, false, sensorarrayS5d5DriveStepFromRaw(0xC000u, NULL), 0xC000u);
    sensorarrayS5d5CandidateInit(&ampFaulty, false, sensorarrayS5d5DriveStepFromRaw(0xF800u, NULL), 0xF800u);

    ampHealthy.sampleCount = 6u;
    ampHealthy.transportReadableCount = 6u;
    ampHealthy.convertingCount = 6u;
    ampHealthy.unreadCount = 6u;
    ampHealthy.nonZeroRawCount = 6u;
    ampHealthy.provisionalReadableCount = 6u;
    ampHealthy.healthReadableCount = 6u;
    ampHealthy.shouldCountForSweepCount = 6u;
    ampHealthy.sampleValidCount = 6u;
    ampHealthy.activeChannelMatchCount = 6u;
    ampHealthy.rawMin = 9000000u;
    ampHealthy.rawMax = 9002500u;
    ampHealthy.rawSum = 54007500ull;
    ampHealthy.readbackValid = true;
    ampHealthy.highCurrentReadback = ampHealthy.highCurrentReq;
    ampHealthy.driveCurrentReadbackNorm = ampHealthy.driveCurrentNorm;
    ampHealthy.activeChannelReadback = (uint8_t)FDC2214_CH0;
    (void)sensorarrayS5d5FinalizeCandidate(&ampHealthy);

    ampFaulty.sampleCount = 6u;
    ampFaulty.transportReadableCount = 6u;
    ampFaulty.convertingCount = 6u;
    ampFaulty.unreadCount = 6u;
    ampFaulty.nonZeroRawCount = 6u;
    ampFaulty.provisionalReadableCount = 6u;
    ampFaulty.healthReadableCount = 2u;
    ampFaulty.shouldCountForSweepCount = 2u;
    ampFaulty.sampleValidCount = 2u;
    ampFaulty.activeChannelMatchCount = 6u;
    ampFaulty.amplitudeCount = 4u;
    ampFaulty.rawMin = 9000000u;
    ampFaulty.rawMax = 9000100u;
    ampFaulty.rawSum = 54000300ull;
    ampFaulty.readbackValid = true;
    ampFaulty.highCurrentReadback = ampFaulty.highCurrentReq;
    ampFaulty.driveCurrentReadbackNorm = ampFaulty.driveCurrentNorm;
    ampFaulty.activeChannelReadback = (uint8_t)FDC2214_CH0;
    (void)sensorarrayS5d5FinalizeCandidate(&ampFaulty);

    bool rankingCasePass = sensorarrayS5d5CandidateBetter(&ampHealthy, &ampFaulty) &&
                           !sensorarrayS5d5CandidateBetter(&ampFaulty, &ampHealthy);
    printf("DBGFDC_S5D5,stage=math_selfcheck_case3,healthyAmplitude=%lu,faultyAmplitude=%lu,healthyRawSpan=%lu,"
           "faultyRawSpan=%lu,healthyGate=%u,faultyGate=%u,result=%s\n",
           (unsigned long)ampHealthy.amplitudeCount,
           (unsigned long)ampFaulty.amplitudeCount,
           (unsigned long)ampHealthy.rawSpan,
           (unsigned long)ampFaulty.rawSpan,
           ampHealthy.passedValidityGate ? 1u : 0u,
           ampFaulty.passedValidityGate ? 1u : 0u,
           rankingCasePass ? "pass" : "fail");

    bool allPass = freqCasePass && capCasePass && rankingCasePass;
    printf("DBGFDC_S5D5,stage=math_selfcheck_summary,result=%s\n", allPass ? "pass" : "fail");
    return allPass;
}

static esp_err_t sensorarrayS5d5ApplyLockedCandidate(Fdc2214CapDevice_t *dev,
                                                     const sensorarrayS5d5SweepCandidate_t *candidate,
                                                     Fdc2214CapChannel_t channel)
{
    if (!dev || !candidate) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t appliedDriveRaw = 0u;
    esp_err_t err =
        sensorarrayApplyS5d5DriveStep(dev, candidate->highCurrentReq, candidate->driveStep, &appliedDriveRaw);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=lock_apply,highCurrentReq=%u,driveStep=%u,driveCurrentReq=0x%04X,"
               "driveCurrentNorm=0x%04X,err=%ld,status=apply_failed\n",
               candidate->highCurrentReq ? 1u : 0u,
               (unsigned)candidate->driveStep,
               candidate->driveCurrentReq,
               candidate->driveCurrentNorm,
               (long)err);
        return err;
    }

    sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SETTLE_MS);

    uint16_t statusReg = 0u;
    uint16_t configReg = 0u;
    uint16_t muxConfig = 0u;
    uint16_t driveReg = 0u;
    err = sensorarrayS5d5ReadKeyRegs(dev, &statusReg, &configReg, &muxConfig, &driveReg);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=lock_apply,highCurrentReq=%u,driveStep=%u,driveCurrentReq=0x%04X,"
               "driveCurrentNorm=0x%04X,err=%ld,status=readback_failed\n",
               candidate->highCurrentReq ? 1u : 0u,
               (unsigned)candidate->driveStep,
               candidate->driveCurrentReq,
               candidate->driveCurrentNorm,
               (long)err);
        return err;
    }

    bool highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    uint16_t driveCurrentReadbackNorm = (uint16_t)(driveReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    uint8_t activeChannelReadback =
        (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >> SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
    bool match = (highCurrentReadback == candidate->highCurrentReq) &&
                 (driveCurrentReadbackNorm == candidate->driveCurrentNorm) &&
                 (activeChannelReadback == (uint8_t)channel);

    printf("DBGFDC_S5D5,stage=lock_apply,highCurrentReq=%u,highCurrentReadback=%u,driveStep=%u,"
           "driveCurrentReq=0x%04X,driveCurrentNorm=0x%04X,driveAppliedRaw=0x%04X,driveCurrentReadback=0x%04X,"
           "activeChannelReadback=%u,statusReg=0x%04X,configReg=0x%04X,muxConfig=0x%04X,status=%s\n",
           candidate->highCurrentReq ? 1u : 0u,
           highCurrentReadback ? 1u : 0u,
           (unsigned)candidate->driveStep,
           candidate->driveCurrentReq,
           candidate->driveCurrentNorm,
           appliedDriveRaw,
           driveCurrentReadbackNorm,
           (unsigned)activeChannelReadback,
           statusReg,
           configReg,
           muxConfig,
           match ? "lock_applied" : "warning_readback_mismatch");
    return ESP_OK;
}

static esp_err_t sensorarrayS5d5WaitUnreadOrTimeout(Fdc2214CapDevice_t *dev,
                                                     Fdc2214CapChannel_t channel,
                                                     uint32_t timeoutMs,
                                                     uint32_t intervalMs,
                                                     bool verboseLog,
                                                     uint32_t warnLogStride,
                                                     uint32_t *ioWarnCounter,
                                                     bool *outReady,
                                                     Fdc2214CapStatus_t *outLastStatus,
                                                     uint32_t *outPollCount)
{
    if (!dev || !outReady || !outLastStatus || !outPollCount) {
        return ESP_ERR_INVALID_ARG;
    }

    *outReady = false;
    *outPollCount = 0u;
    *outLastStatus = (Fdc2214CapStatus_t){0};

    uint32_t elapsedMs = 0u;
    while (true) {
        esp_err_t err = Fdc2214CapReadStatus(dev, outLastStatus);
        (*outPollCount)++;
        if (err != ESP_OK) {
            if (verboseLog || sensorarrayLogRateLimited(ioWarnCounter, warnLogStride)) {
                printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,intervalMs=%lu,polls=%lu,result=i2c_error,err=%ld\n",
                       (unsigned long)timeoutMs,
                       (unsigned long)intervalMs,
                       (unsigned long)(*outPollCount),
                       (long)err);
            }
            return err;
        }

        bool unreadReady = outLastStatus->UnreadConversion[(uint8_t)channel];
        bool dataReady = outLastStatus->DataReady;
        if (unreadReady || dataReady) {
            *outReady = true;
            if (verboseLog) {
                printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,intervalMs=%lu,polls=%lu,statusReg=0x%04X,unread=%u,"
                       "dataReady=%u,result=ready\n",
                       (unsigned long)timeoutMs,
                       (unsigned long)intervalMs,
                       (unsigned long)(*outPollCount),
                       outLastStatus->Raw,
                       unreadReady ? 1u : 0u,
                       dataReady ? 1u : 0u);
            }
            return ESP_OK;
        }

        if (elapsedMs >= timeoutMs) {
            break;
        }

        sensorarrayDebugSelftestDelayMs(intervalMs);
        elapsedMs += intervalMs;
    }

    if (verboseLog || sensorarrayLogRateLimited(ioWarnCounter, warnLogStride)) {
        printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,intervalMs=%lu,polls=%lu,statusReg=0x%04X,unread=%u,"
               "dataReady=%u,result=timeout\n",
               (unsigned long)timeoutMs,
               (unsigned long)intervalMs,
               (unsigned long)(*outPollCount),
               outLastStatus->Raw,
               outLastStatus->UnreadConversion[(uint8_t)channel] ? 1u : 0u,
               outLastStatus->DataReady ? 1u : 0u);
    }
    return ESP_OK;
}

static void sensorarrayS5d5ResolveClockMetadata(const sensorarrayFdcDeviceState_t *fdcState,
                                                uint32_t *outRefClockHz,
                                                sensorarrayFdcRefClockQuality_t *outRefClockQuality,
                                                uint16_t *outClockDiv0Raw,
                                                Fdc2214CapClockDividerInfo_t *outClockDividerInfo,
                                                bool *outClockDividerInfoValid)
{
    if (outRefClockHz) {
        *outRefClockHz = SENSORARRAY_FDC_REF_CLOCK_HZ;
    }
    if (outRefClockQuality) {
        *outRefClockQuality = SENSORARRAY_FDC_REF_CLOCK_QUALITY_UNKNOWN;
    }
    if (outClockDiv0Raw) {
        *outClockDiv0Raw = SENSORARRAY_FDC_DEBUG_CLOCK_DIVIDERS_CH0;
    }
    if (outClockDividerInfo) {
        *outClockDividerInfo = (Fdc2214CapClockDividerInfo_t){0};
    }
    if (outClockDividerInfoValid) {
        *outClockDividerInfoValid = false;
    }

    if (!fdcState) {
        if (outClockDividerInfo && outClockDividerInfoValid &&
            Fdc2214CapDecodeClockDividers(SENSORARRAY_FDC_DEBUG_CLOCK_DIVIDERS_CH0, outClockDividerInfo) == ESP_OK) {
            *outClockDividerInfoValid = true;
        }
        return;
    }

    if (outRefClockHz && fdcState->refClockHz > 0u) {
        *outRefClockHz = fdcState->refClockHz;
    }
    if (outRefClockQuality) {
        *outRefClockQuality = fdcState->refClockQuality;
    }
    if (outClockDiv0Raw && fdcState->channel0ClockDividersRaw > 0u) {
        *outClockDiv0Raw = fdcState->channel0ClockDividersRaw;
    }

    if (!outClockDividerInfo || !outClockDividerInfoValid) {
        return;
    }

    if (fdcState->channel0ClockDividerValid) {
        *outClockDividerInfo = fdcState->channel0ClockDividerInfo;
        *outClockDividerInfoValid = true;
        return;
    }

    uint16_t rawClockDiv0 = (fdcState->channel0ClockDividersRaw > 0u)
                                ? fdcState->channel0ClockDividersRaw
                                : SENSORARRAY_FDC_DEBUG_CLOCK_DIVIDERS_CH0;
    if (Fdc2214CapDecodeClockDividers(rawClockDiv0, outClockDividerInfo) == ESP_OK) {
        *outClockDividerInfoValid = true;
    }
}

static esp_err_t sensorarrayS5d5DoRecoveryReinit(sensorarrayState_t *state,
                                                 sensorarrayFdcDeviceState_t *fdcState,
                                                 const sensorarrayCheckpointGpio_t *checkpoint,
                                                 const char *reason,
                                                 uint32_t streakCount,
                                                 uint32_t initSettleDelayMs)
{
    bool sclBefore = true;
    bool sdaBefore = true;
    bool sclAfter = true;
    bool sdaAfter = true;
    esp_err_t lineBeforeErr = ESP_ERR_INVALID_STATE;
    esp_err_t lineAfterErr = ESP_ERR_INVALID_STATE;
    esp_err_t busRecoverErr = ESP_ERR_INVALID_STATE;
    esp_err_t busReinitErr = ESP_ERR_INVALID_STATE;
    if (fdcState && fdcState->i2cCtx) {
        lineBeforeErr = boardSupportI2cCheckLines(fdcState->i2cCtx, &sclBefore, &sdaBefore);
        busRecoverErr = boardSupportI2cRecoverBus(fdcState->i2cCtx, reason ? reason : "s5d5_recovery");
        if (busRecoverErr != ESP_OK) {
            busReinitErr = boardSupportI2cReinit(fdcState->i2cCtx, reason ? reason : "s5d5_recovery_reinit_fallback");
        }
        lineAfterErr = boardSupportI2cCheckLines(fdcState->i2cCtx, &sclAfter, &sdaAfter);
    }
    printf("DBGFDC_S5D5,stage=recovery_bus,reason=%s,streak=%lu,lineBeforeErr=%ld,sclBefore=%d,sdaBefore=%d,"
           "recoverErr=%ld,reinitErr=%ld,lineAfterErr=%ld,sclAfter=%d,sdaAfter=%d,status=%s\n",
           reason ? reason : SENSORARRAY_NA,
           (unsigned long)streakCount,
           (long)lineBeforeErr,
           (lineBeforeErr == ESP_OK) ? (sclBefore ? 1 : 0) : -1,
           (lineBeforeErr == ESP_OK) ? (sdaBefore ? 1 : 0) : -1,
           (long)busRecoverErr,
           (long)busReinitErr,
           (long)lineAfterErr,
           (lineAfterErr == ESP_OK) ? (sclAfter ? 1 : 0) : -1,
           (lineAfterErr == ESP_OK) ? (sdaAfter ? 1 : 0) : -1,
           (busRecoverErr == ESP_OK || busReinitErr == ESP_OK) ? "bus_recovered_or_reinit_ok"
                                                                : "bus_recover_failed");
    if (busRecoverErr != ESP_OK && busReinitErr != ESP_OK) {
        return busReinitErr;
    }

    sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_BEGIN);
    sensorarrayFdcInitDiag_t initDiag = {0};
    esp_err_t err = sensorarrayInitS5d5SecondaryFdc(state, fdcState, &initDiag);
    if (err == ESP_OK) {
        sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_OK);
        sensorarrayDebugSelftestDelayMs(initSettleDelayMs);
    } else {
        sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_WARNING);
    }

    printf("DBGFDC_S5D5,stage=recovery_reinit,reason=%s,streak=%lu,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,"
           "detail=%ld,err=%ld,busRecoverErr=%ld,status=%s\n",
           reason ? reason : SENSORARRAY_NA,
           (unsigned long)streakCount,
           SENSORARRAY_FDC_I2C_ADDR_LOW,
           initDiag.manufacturerId,
           initDiag.deviceId,
           (long)initDiag.detail,
            (long)err,
           (long)busRecoverErr,
           initDiag.status ? initDiag.status : SENSORARRAY_NA);
    return err;
}

static esp_err_t sensorarrayS5d5HandleFaultAndRecover(sensorarrayState_t *state,
                                                      sensorarrayFdcDeviceState_t *fdcState,
                                                      const sensorarrayCheckpointGpio_t *checkpoint,
                                                      sensorarrayS5d5RecoveryTracker_t *tracker,
                                                      sensorarrayS5d5FaultReason_t faultReason,
                                                      const char *faultDetail,
                                                      uint32_t streakCount,
                                                      uint32_t maxRecoveryAttempts,
                                                      uint32_t recoveryCooldownMs,
                                                      uint32_t initSettleDelayMs)
{
    if (!tracker) {
        return ESP_ERR_INVALID_ARG;
    }

    tracker->lastFaultReason = faultReason;
    const char *reasonName = faultDetail ? faultDetail : sensorarrayS5d5FaultReasonName(faultReason);
    bool needsBusRecovery = sensorarrayS5d5FaultNeedsBusRecovery(faultReason);
    const char *triggerClass = sensorarrayS5d5RecoveryTriggerClass(faultReason, reasonName);
    if (maxRecoveryAttempts == 0u) {
        maxRecoveryAttempts = 1u;
    }

    if (!needsBusRecovery) {
        printf("DBGFDC_S5D5,stage=recovery_policy,action=analog_retry,trigger=%s,fault=%s,detail=%s,streak=%lu,"
               "busRecovery=%u,recoveryCount=%lu,maxAttempts=%lu,status=retry_without_bus_recovery\n",
               triggerClass,
               sensorarrayS5d5FaultReasonName(faultReason),
               reasonName ? reasonName : SENSORARRAY_NA,
               (unsigned long)streakCount,
               0u,
               (unsigned long)tracker->recoveryCount,
               (unsigned long)maxRecoveryAttempts);
        tracker->recoveryCount = 0u;
        return ESP_OK;
    }

    if (tracker->recoveryCount >= maxRecoveryAttempts) {
        printf("DBGFDC_S5D5,stage=recovery_gate,result=blocked,reason=max_attempts_reached,maxAttempts=%lu,"
               "recoveryCount=%lu,lastFault=%s,trigger=%s,cooldownMs=%lu\n",
               (unsigned long)maxRecoveryAttempts,
               (unsigned long)tracker->recoveryCount,
               sensorarrayS5d5FaultReasonName(tracker->lastFaultReason),
               triggerClass,
               (unsigned long)recoveryCooldownMs);
        if (recoveryCooldownMs > 0u) {
            sensorarrayDebugSelftestDelayMs(recoveryCooldownMs);
        }
        return ESP_ERR_INVALID_STATE;
    }

    tracker->recoveryCount++;
    esp_err_t sleepErr = ESP_ERR_INVALID_STATE;
    if (fdcState && fdcState->handle) {
        sleepErr = Fdc2214CapEnterSleep(fdcState->handle, fdcState->configReg);
    }
    printf("DBGFDC_S5D5,stage=recovery_policy,action=bus_recovery,trigger=%s,fault=%s,detail=%s,streak=%lu,"
           "recoveryCount=%lu,maxAttempts=%lu,sleepErr=%ld\n",
           triggerClass,
           sensorarrayS5d5FaultReasonName(faultReason),
           reasonName ? reasonName : SENSORARRAY_NA,
           (unsigned long)streakCount,
           (unsigned long)tracker->recoveryCount,
           (unsigned long)maxRecoveryAttempts,
           (long)sleepErr);

    esp_err_t recoverErr = sensorarrayS5d5DoRecoveryReinit(state,
                                                            fdcState,
                                                            checkpoint,
                                                            reasonName,
                                                            streakCount,
                                                            initSettleDelayMs);
    printf("DBGFDC_S5D5,stage=recovery_gate,result=%s,fault=%s,detail=%s,recoveryCount=%lu,maxAttempts=%lu,"
           "cooldownMs=%lu,trigger=%s,err=%ld\n",
           (recoverErr == ESP_OK) ? "recovered" : "failed",
           sensorarrayS5d5FaultReasonName(faultReason),
           reasonName ? reasonName : SENSORARRAY_NA,
           (unsigned long)tracker->recoveryCount,
           (unsigned long)maxRecoveryAttempts,
           (unsigned long)recoveryCooldownMs,
           triggerClass,
           (long)recoverErr);

    if (recoverErr == ESP_OK) {
        tracker->recoveryCount = 0u;
        return ESP_OK;
    }

    if (tracker->recoveryCount >= maxRecoveryAttempts && recoveryCooldownMs > 0u) {
        sensorarrayDebugSelftestDelayMs(recoveryCooldownMs);
    }
    return recoverErr;
}

void sensorarrayDebugRunS5d5CapFdcSecondaryModeImpl(sensorarrayState_t *state)
{
    if (!state || !state->boardReady || !state->tmuxReady) {
        printf("DBGFDC_S5D5,stage=mode_enter,status=state_not_ready\n");
        sensorarrayDebugIdleForever("s5d5_fdc_state_not_ready");
        return;
    }

    const sensorarrayRouteMap_t *route = sensorarrayBoardMapFindRoute(SENSORARRAY_S5,
                                                                       SENSORARRAY_D5,
                                                                       SENSORARRAY_PATH_CAPACITIVE);
    const sensorarrayFdcDLineMap_t *fdcMap = sensorarrayBoardMapFindFdcByDLine(SENSORARRAY_D5);
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, SENSORARRAY_FDC_DEV_SECONDARY);
    if (!route || !fdcMap || !fdcState) {
        printf("DBGFDC_S5D5,stage=mode_enter,status=route_or_fdc_state_missing\n");
        sensorarrayDebugIdleForever("s5d5_fdc_route_or_state_missing");
        return;
    }
    if (fdcMap->devId != SENSORARRAY_FDC_DEV_SECONDARY || fdcMap->channel != FDC2214_CH0) {
        printf("DBGFDC_S5D5,stage=mode_enter,status=fdc_map_mismatch,devId=%u,channel=%u\n",
               (unsigned)fdcMap->devId,
               (unsigned)fdcMap->channel);
        sensorarrayDebugIdleForever("s5d5_fdc_map_mismatch");
        return;
    }
    if (route->selaRoute != SENSORARRAY_SELA_ROUTE_FDC2214 || !route->selBLevel) {
        printf("DBGFDC_S5D5,stage=mode_enter,status=route_semantic_mismatch,"
               "note=expected_cap_route_sela_fdc_and_selb_high\n");
        sensorarrayDebugIdleForever("s5d5_fdc_route_semantic_mismatch");
        return;
    }

    int selaWriteLevel = -1;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(route->selaRoute, &selaWriteLevel)) {
        printf("DBGFDC_S5D5,stage=mode_enter,status=sela_route_invalid\n");
        sensorarrayDebugIdleForever("s5d5_fdc_sela_route_invalid");
        return;
    }

    tmux1108Source_t swSource = sensorarrayBoardMapDefaultSwSource(route);
    BoardSupportI2cBusInfo_t busInfo = {0};
    (void)boardSupportGetI2cBusInfo(true, &busInfo);
    if (!busInfo.Enabled ||
        busInfo.Port != 1 ||
        busInfo.SdaGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO ||
        busInfo.SclGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO) {
        printf("DBGFDC_S5D5,stage=mode_enter,status=secondary_i2c_bus_mismatch,enabled=%u,port=%d,sda=%d,scl=%d,"
               "expectedPort=1,expectedSda=%d,expectedScl=%d\n",
               busInfo.Enabled ? 1u : 0u,
               (int)busInfo.Port,
               busInfo.SdaGpio,
               busInfo.SclGpio,
               SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO,
               SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO);
        sensorarrayDebugIdleForever("s5d5_fdc_secondary_i2c_mismatch");
        return;
    }

    uint32_t sweepSampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_SWEEP_SAMPLE_COUNT;
    if (sweepSampleCount == 0u) {
        sweepSampleCount = 1u;
    }
    uint32_t lockedSampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOCKED_SAMPLE_COUNT;
    if (lockedSampleCount == 0u) {
        lockedSampleCount = 1u;
    }
    uint32_t loopDelayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOOP_DELAY_MS;
    if (loopDelayMs < 50u) {
        loopDelayMs = 50u;
    }
    bool verboseLog = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_VERBOSE_LOG != 0);
    uint32_t warnLogStride = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_WARN_LOG_STRIDE;
    if (warnLogStride == 0u) {
        warnLogStride = 1u;
    }
    uint32_t diagLogStride = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_DIAG_LOG_STRIDE;
    if (diagLogStride == 0u) {
        diagLogStride = 1u;
    }
    uint32_t unreadPollTimeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_UNREAD_POLL_TIMEOUT_MS;
    uint32_t unreadPollIntervalMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_UNREAD_POLL_INTERVAL_MS;
    if (unreadPollIntervalMs == 0u) {
        unreadPollIntervalMs = 1u;
    }
    if (unreadPollTimeoutMs < unreadPollIntervalMs) {
        unreadPollTimeoutMs = unreadPollIntervalMs;
    }
    uint32_t recoveryReinitThreshold =
        (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RECOVERY_REINIT_ERR_THRESHOLD;
    if (recoveryReinitThreshold == 0u) {
        recoveryReinitThreshold = 3u;
    }
    bool disableSweep = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_DISABLE_SWEEP != 0);
    bool forceSingleDrive = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FORCE_SINGLE_DRIVE != 0);
    bool sweepEnabled = !disableSweep;
    uint16_t forcedDriveConfigValue = (uint16_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FORCE_DRIVE_CURRENT;
    uint8_t forcedDriveStep = SENSORARRAY_S5D5_DRIVE_STEP_DEFAULT;
    uint16_t forcedDriveCurrentReq = 0u;
    uint16_t forcedDriveCurrentMasked = 0u;
    bool forcedDriveLegacyRawInput = false;
    bool forcedDriveReservedBitsMasked = false;
    sensorarrayS5d5ResolveForcedDriveConfig(forcedDriveConfigValue,
                                            &forcedDriveStep,
                                            &forcedDriveCurrentReq,
                                            &forcedDriveCurrentMasked,
                                            &forcedDriveLegacyRawInput,
                                            &forcedDriveReservedBitsMasked);
    uint32_t maxRecoveryAttempts = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_MAX_RECOVERY_ATTEMPTS;
    if (maxRecoveryAttempts == 0u) {
        maxRecoveryAttempts = 1u;
    }
    uint32_t recoveryCooldownMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RECOVERY_COOLDOWN_MS;
    uint32_t initSettleDelayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_INIT_SETTLE_DELAY_MS;
    uint32_t postResetDelayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_POST_RESET_DELAY_MS;
    int32_t lockMinScore = (int32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOCK_REQUIRE_MIN_SCORE;
    sensorarrayS5d5CapComputationConfig_t capConfig = {
        .inductorValueUh = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_INDUCTOR_UH,
        .fixedCapPf = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FIXED_CAP_PF,
        .parasiticCapPf = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_PARASITIC_CAP_PF,
        .enableCapComputation = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_ENABLE_CAP_COMPUTATION != 0),
        .enableNetCapOutput = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_ENABLE_NET_CAP_OUTPUT != 0),
    };
    bool discardFirst = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_DISCARD_FIRST != 0);
    sensorarrayCheckpointGpio_t checkpoint = sensorarrayCheckpointInit();

    if (forcedDriveLegacyRawInput || forcedDriveReservedBitsMasked) {
        printf("DBGFDC_S5D5,stage=drive_config_parse,configRaw=0x%04X,legacyRawInput=%u,reservedBitsMasked=%u,"
               "driveStep=%u,driveRawReq=0x%04X,driveRawMasked=0x%04X,status=normalized\n",
               forcedDriveConfigValue,
               forcedDriveLegacyRawInput ? 1u : 0u,
               forcedDriveReservedBitsMasked ? 1u : 0u,
               (unsigned)forcedDriveStep,
               forcedDriveCurrentReq,
               forcedDriveCurrentMasked);
    }

    printf("DBGFDC_S5D5,stage=target,mode=S5D5_CAP_FDC_SECONDARY,fdcDev=secondary_selb_side,i2cPort=1,"
           "sda=%d,scl=%d,i2cAddr=0x%02X,route=S5D5_CAP,channel=CH0,sweepSamples=%lu,lockedSamples=%lu,"
           "loopDelayMs=%lu,discardFirst=%u,recoveryThreshold=%lu,lockMinScore=%ld,verboseLog=%u,warnLogStride=%lu,"
           "diagLogStride=%lu,sweepEnabled=%u,disableSweep=%u,forceSingleDrive=%u,forcedDriveStep=%u,"
           "forcedDriveRawReq=0x%04X,forcedDriveRawMasked=0x%04X,maxRecoveryAttempts=%lu,"
           "recoveryCooldownMs=%lu,initSettleDelayMs=%lu,postResetDelayMs=%lu,enableCapComputation=%u,"
           "enableNetCapOutput=%u,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f\n",
           busInfo.SdaGpio,
           busInfo.SclGpio,
           SENSORARRAY_FDC_I2C_ADDR_LOW,
            (unsigned long)sweepSampleCount,
            (unsigned long)lockedSampleCount,
            (unsigned long)loopDelayMs,
           discardFirst ? 1u : 0u,
           (unsigned long)recoveryReinitThreshold,
           (long)lockMinScore,
           verboseLog ? 1u : 0u,
           (unsigned long)warnLogStride,
           (unsigned long)diagLogStride,
           sweepEnabled ? 1u : 0u,
           disableSweep ? 1u : 0u,
           forceSingleDrive ? 1u : 0u,
           (unsigned)forcedDriveStep,
           forcedDriveCurrentReq,
           forcedDriveCurrentMasked,
           (unsigned long)maxRecoveryAttempts,
           (unsigned long)recoveryCooldownMs,
           (unsigned long)initSettleDelayMs,
           (unsigned long)postResetDelayMs,
           capConfig.enableCapComputation ? 1u : 0u,
           capConfig.enableNetCapOutput ? 1u : 0u,
           capConfig.inductorValueUh,
           capConfig.fixedCapPf,
           capConfig.parasiticCapPf);
    printf("DBGFDC_S5D5,stage=sweep_policy,sweepEnabled=%u,reason=%s,forceSingleDrive=%u,forcedDriveStep=%u,"
           "forcedDriveRawReq=0x%04X,forcedDriveRawMasked=0x%04X\n",
           sweepEnabled ? 1u : 0u,
           sweepEnabled ? "normal_candidate_search_enabled" : "disabled_by_config",
           forceSingleDrive ? 1u : 0u,
           (unsigned)forcedDriveStep,
           forcedDriveCurrentReq,
           forcedDriveCurrentMasked);
    printf("DBGFDC_S5D5,stage=route_semantics,note=route_verification_split_into_gpio_and_analog_health\n");
    printf("DBGFDC_S5D5,stage=sweep_plan,highCurrent=0|1,driveStepList=20|23|24|26|28|31,"
           "driveRawList=0xA000|0xB800|0xC000|0xD000|0xE000|0xF800,stepSettleMs=%u,sampleGapMs=%u,"
           "unreadPollTimeoutMs=%lu,unreadPollIntervalMs=%lu,disableSweep=%u,forceSingleDrive=%u,"
           "forcedDriveStep=%u,forcedDriveRawReq=0x%04X,forcedDriveRawMasked=0x%04X,inductorUh=%.3f,"
           "fixedCapPf=%.3f,parasiticCapPf=%.3f\n",
           (unsigned)SENSORARRAY_S5D5_STEP_SETTLE_MS,
           (unsigned)SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS,
           (unsigned long)unreadPollTimeoutMs,
           (unsigned long)unreadPollIntervalMs,
           disableSweep ? 1u : 0u,
           forceSingleDrive ? 1u : 0u,
           (unsigned)forcedDriveStep,
           forcedDriveCurrentReq,
           forcedDriveCurrentMasked,
           capConfig.inductorValueUh,
           capConfig.fixedCapPf,
           capConfig.parasiticCapPf);

    bool mathSelfCheckOk = sensorarrayS5d5RunMathSelfCheck();
    if (!mathSelfCheckOk) {
        printf("DBGFDC_S5D5,stage=math_selfcheck,status=warning_continue\n");
    }

    if (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_START_DELAY_MS > 0) {
        sensorarrayDebugSelftestDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_START_DELAY_MS);
    }

    bool locked = false;
    bool modeFdcInitialized = false;
    uint32_t lockEpoch = 0u;
    uint32_t lockedSampleGlobalIndex = 0u;
    uint32_t i2cFailureStreak = 0u;
    uint32_t noUnreadStreak = 0u;
    uint32_t nonConvertingStreak = 0u;
    uint32_t readbackMismatchStreak = 0u;
    uint32_t readbackIoFailureStreak = 0u;
    uint32_t recoveryFailureStreak = 0u;
    bool degradedMode = false;
    uint32_t degradedWarnCounter = 0u;
    sensorarrayS5d5RecoveryTracker_t recoveryTracker = {
        .recoveryCount = 0u,
        .lastFaultReason = SENSORARRAY_S5D5_FAULT_NONE,
    };
    sensorarrayS5d5RuntimeReadback_t runtimeReadback = {0};
    sensorarrayS5d5SweepCandidate_t lockedCandidate = {0};

    while (true) {
        sensorarrayS5d5StopAdsForIsolation(state);
        if (degradedMode) {
            uint32_t degradedDelayMs = (loopDelayMs >= SENSORARRAY_S5D5_DEGRADED_MIN_BACKOFF_MS)
                                           ? loopDelayMs
                                           : SENSORARRAY_S5D5_DEGRADED_MIN_BACKOFF_MS;
            if (sensorarrayLogRateLimited(&degradedWarnCounter, warnLogStride)) {
                printf("DBGFDC_S5D5,stage=degraded_state,status=holdoff,recoveryFailureStreak=%lu,backoffMs=%lu\n",
                       (unsigned long)recoveryFailureStreak,
                       (unsigned long)degradedDelayMs);
            }
            sensorarrayDebugSelftestDelayMs(degradedDelayMs);
        }

        if (!modeFdcInitialized || !fdcState->ready || !fdcState->handle) {
            sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_BEGIN);
            sensorarrayFdcInitDiag_t initDiag = {0};
            esp_err_t initErr = sensorarrayInitS5d5SecondaryFdc(state, fdcState, &initDiag);
            if (initErr == ESP_OK) {
                sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_OK);
            } else {
                sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_WARNING);
            }
            printf("DBGFDC_S5D5,stage=fdc_init,fdcDev=secondary_selb_side,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,"
                   "transportOk=%u,idOk=%u,configReadbackOk=%u,converting=%u,unread=%u,watchdog=%u,amplitude=%u,"
                   "driveReq=0x%04X,driveApplied=0x%04X,detail=%ld,err=%ld,status=%s\n",
                   SENSORARRAY_FDC_I2C_ADDR_LOW,
                   initDiag.manufacturerId,
                   initDiag.deviceId,
                   initDiag.transportOk ? 1u : 0u,
                   initDiag.idOk ? 1u : 0u,
                   initDiag.configReadbackOk ? 1u : 0u,
                   initDiag.postInitConverting ? 1u : 0u,
                   initDiag.postInitUnreadPresent ? 1u : 0u,
                   initDiag.postInitStatusWatchdogFault ? 1u : 0u,
                   initDiag.postInitStatusAmplitudeFault ? 1u : 0u,
                   initDiag.channel0DriveCurrentReq,
                   initDiag.channel0DriveCurrentApplied,
                   (long)initDiag.detail,
                   (long)initErr,
                   initDiag.status ? initDiag.status : SENSORARRAY_NA);
            if (initErr != ESP_OK || !fdcState->ready || !fdcState->handle) {
                modeFdcInitialized = false;
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                continue;
            }
            sensorarrayDebugSelftestDelayMs(initSettleDelayMs);
            if (degradedMode) {
                printf("DBGFDC_S5D5,stage=degraded_state,status=exit,recoveryFailureStreak=%lu\n",
                       (unsigned long)recoveryFailureStreak);
            }
            degradedMode = false;
            degradedWarnCounter = 0u;
            recoveryFailureStreak = 0u;
            modeFdcInitialized = true;
            locked = false;
            runtimeReadback.valid = false;
        }

        if (!locked) {
            const char *routeMapLabel = SENSORARRAY_NA;
            esp_err_t routeErr = sensorarrayMeasureApplyRoute(state,
                                                              SENSORARRAY_S5,
                                                              SENSORARRAY_D5,
                                                              SENSORARRAY_PATH_CAPACITIVE,
                                                              swSource,
                                                              &routeMapLabel);
            printf("DBGFDC_S5D5,stage=route_apply,map=%s,err=%ld,status=%s\n",
                   routeMapLabel ? routeMapLabel : SENSORARRAY_NA,
                   (long)routeErr,
                   (routeErr == ESP_OK) ? "route_applied" : "route_apply_failed");
            if (routeErr != ESP_OK) {
                sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_WARNING);
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                continue;
            }
            sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_ROUTE_APPLIED);

            sensorarrayS5d5RouteCheck_t routeCheck = sensorarrayVerifyS5d5Route(swSource,
                                                                                 (uint8_t)(SENSORARRAY_S5 - 1u),
                                                                                 selaWriteLevel,
                                                                                 route->selBLevel);
            bool gpioRouteVerified = routeCheck.gpioRouteVerified;
            if (!gpioRouteVerified) {
                sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_WARNING);
                printf("DBGFDC_S5D5,stage=route_verify_status,gpioRouteVerified=0,analogRouteVerified=0,"
                       "commandMatch=%u,gpioObservedMatch=%u,status=gpio_not_verified\n",
                       routeCheck.commandMatch ? 1u : 0u,
                       routeCheck.gpioObservedMatch ? 1u : 0u);
                printf("DBGFDC_S5D5,stage=lock_failed,reason=gpio_route_not_verified,status=error\n");
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }
            printf("DBGFDC_S5D5,stage=route_verify_status,gpioRouteVerified=1,analogRouteVerified=0,status=gpio_only_verified\n");

            sensorarrayS5d5SweepCandidate_t bestCandidate = {0};
            bool selectedFromValidPool = false;
            sensorarrayS5d5SweepCandidate_t gateCandidate = {0};
            bool gateNeedRecovery = false;
            sensorarrayS5d5FaultReason_t gateRecoveryFault = SENSORARRAY_S5D5_FAULT_NONE;
            const char *gateRecoveryReason = SENSORARRAY_NA;
            uint32_t gateRecoveryStreak = 0u;
            uint32_t gateProbeSamples = (sweepSampleCount > 4u) ? 4u : sweepSampleCount;
            sensorarrayS5d5RouteGateStatus_t routeGateStatus =
                sensorarrayS5d5ProbeAnalogRouteGate(fdcState,
                                                    fdcMap,
                                                    gateProbeSamples,
                                                    discardFirst,
                                                    recoveryReinitThreshold,
                                                    forcedDriveStep,
                                                    forcedDriveCurrentReq,
                                                    &gateCandidate,
                                                    &gateNeedRecovery,
                                                    &gateRecoveryFault,
                                                    &gateRecoveryReason,
                                                    &gateRecoveryStreak);
            bool preSweepBlocked = (routeGateStatus == SENSORARRAY_S5D5_ROUTE_GATE_BLOCKED_BEFORE_SWEEP);
            printf("DBGFDC_S5D5,stage=route_gate,result=%s,gateStatus=%s,gpioRouteVerified=1,analogEvidence=%u,"
                   "probeRejectReason=%s,recoveryReason=%s,recoveryPending=%u\n",
                   preSweepBlocked ? "pre_sweep_block" : "probe_continue",
                   sensorarrayS5d5RouteGateStatusName(routeGateStatus),
                   gateCandidate.analogRouteVerified ? 1u : 0u,
                   gateCandidate.rejectReason ? gateCandidate.rejectReason : SENSORARRAY_NA,
                   gateRecoveryReason ? gateRecoveryReason : SENSORARRAY_NA,
                   gateNeedRecovery ? 1u : 0u);
            if (preSweepBlocked) {
                if (gateNeedRecovery) {
                    esp_err_t recoveryErr = sensorarrayS5d5HandleFaultAndRecover(state,
                                                                                  fdcState,
                                                                                  &checkpoint,
                                                                                  &recoveryTracker,
                                                                                  gateRecoveryFault,
                                                                                  gateRecoveryReason,
                                                                                  gateRecoveryStreak,
                                                                                  maxRecoveryAttempts,
                                                                                  recoveryCooldownMs,
                                                                                  initSettleDelayMs);
                    locked = false;
                    modeFdcInitialized = (recoveryErr == ESP_OK);
                    runtimeReadback.valid = false;
                    if (recoveryErr == ESP_OK) {
                        if (degradedMode) {
                            printf("DBGFDC_S5D5,stage=degraded_state,status=exit,recoveryFailureStreak=%lu\n",
                                   (unsigned long)recoveryFailureStreak);
                        }
                        degradedMode = false;
                        degradedWarnCounter = 0u;
                        recoveryFailureStreak = 0u;
                    } else {
                        recoveryFailureStreak++;
                        if (!degradedMode &&
                            recoveryFailureStreak >= SENSORARRAY_S5D5_RECOVERY_FAIL_DEGRADED_THRESHOLD) {
                            degradedMode = true;
                            printf("DBGFDC_S5D5,stage=degraded_state,status=enter,recoveryFailureStreak=%lu,reason=%s\n",
                                   (unsigned long)recoveryFailureStreak,
                                   gateRecoveryReason ? gateRecoveryReason : SENSORARRAY_NA);
                        }
                    }
                    sensorarrayDebugSelftestDelayMs(loopDelayMs);
                    continue;
                }
                printf("DBGFDC_S5D5,stage=lock_failed_after_sweep,reason=transport_failed_before_sweep,"
                       "status=pre_sweep_blocked_no_recovery_path\n");
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }

            bool haveBest = false;
            bool sweepNeedRecovery = false;
            sensorarrayS5d5FaultReason_t sweepRecoveryFault = SENSORARRAY_S5D5_FAULT_NONE;
            const char *sweepRecoveryReason = SENSORARRAY_NA;
            uint32_t sweepRecoveryStreak = 0u;
            if (disableSweep) {
                haveBest = true;
                bestCandidate = gateCandidate;
                selectedFromValidPool = bestCandidate.passedValidityGate;
                printf("DBGFDC_S5D5,stage=sweep_bypass,result=single_drive_mode,driveStep=%u,driveCurrentReq=0x%04X,"
                       "driveCurrent=0x%04X,score=%ld,healthClass=%s,analogRouteVerified=%u\n",
                       (unsigned)bestCandidate.driveStep,
                       bestCandidate.driveCurrentReq,
                       bestCandidate.driveCurrentNorm,
                       (long)bestCandidate.score,
                       sensorarrayS5d5CandidateHealthClassName(bestCandidate.healthClass),
                       bestCandidate.analogRouteVerified ? 1u : 0u);
            } else {
                haveBest = sensorarrayS5d5SweepCandidates(fdcState,
                                                          fdcMap,
                                                          sweepSampleCount,
                                                          discardFirst,
                                                          forceSingleDrive,
                                                          forcedDriveStep,
                                                          forcedDriveCurrentReq,
                                                          lockMinScore,
                                                          &checkpoint,
                                                          &bestCandidate,
                                                          &selectedFromValidPool,
                                                          &sweepNeedRecovery,
                                                          &sweepRecoveryFault,
                                                          &sweepRecoveryReason,
                                                          &sweepRecoveryStreak);
            }
            if (!haveBest && sweepNeedRecovery) {
                esp_err_t recoveryErr = sensorarrayS5d5HandleFaultAndRecover(state,
                                                                              fdcState,
                                                                              &checkpoint,
                                                                              &recoveryTracker,
                                                                              sweepRecoveryFault,
                                                                              sweepRecoveryReason,
                                                                              sweepRecoveryStreak,
                                                                              maxRecoveryAttempts,
                                                                              recoveryCooldownMs,
                                                                              initSettleDelayMs);
                locked = false;
                modeFdcInitialized = (recoveryErr == ESP_OK);
                runtimeReadback.valid = false;
                if (recoveryErr == ESP_OK) {
                    if (degradedMode) {
                        printf("DBGFDC_S5D5,stage=degraded_state,status=exit,recoveryFailureStreak=%lu\n",
                               (unsigned long)recoveryFailureStreak);
                    }
                    degradedMode = false;
                    degradedWarnCounter = 0u;
                    recoveryFailureStreak = 0u;
                } else {
                    recoveryFailureStreak++;
                    if (!degradedMode &&
                        recoveryFailureStreak >= SENSORARRAY_S5D5_RECOVERY_FAIL_DEGRADED_THRESHOLD) {
                        degradedMode = true;
                        printf("DBGFDC_S5D5,stage=degraded_state,status=enter,recoveryFailureStreak=%lu,reason=%s\n",
                               (unsigned long)recoveryFailureStreak,
                               sweepRecoveryReason ? sweepRecoveryReason : SENSORARRAY_NA);
                    }
                    printf("DBGFDC_S5D5,stage=lock_failed,reason=sweep_recovery_failed,recoveryReason=%s,err=%ld,"
                           "status=fail_fast\n",
                           sweepRecoveryReason ? sweepRecoveryReason : SENSORARRAY_NA,
                           (long)recoveryErr);
                    sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                }
                continue;
            }
            if (!haveBest) {
                printf("DBGFDC_S5D5,stage=lock_failed_after_sweep,reason=no_candidate_measured,bestScore=%ld,"
                       "minScore=%ld,status=error\n",
                       (long)INT_MIN,
                       (long)lockMinScore);
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }
            if (!selectedFromValidPool || !bestCandidate.passedValidityGate) {
                printf("DBGFDC_S5D5,stage=lock_failed_after_sweep,reason=no_candidate_met_minimum_health,"
                       "rejectReason=%s,bestScore=%ld,minScore=%ld,diagnosticDriveStep=%u,diagnosticDriveReq=0x%04X,"
                       "diagnosticDriveNorm=0x%04X,status=diagnostic_only\n",
                       bestCandidate.rejectReason ? bestCandidate.rejectReason : SENSORARRAY_NA,
                       (long)bestCandidate.score,
                       (long)lockMinScore,
                       (unsigned)bestCandidate.driveStep,
                       bestCandidate.driveCurrentReq,
                       bestCandidate.driveCurrentNorm);
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }
            bool bestAnalogRouteVerified = bestCandidate.analogRouteVerified;
            printf("DBGFDC_S5D5,stage=route_verify_status,gpioRouteVerified=1,analogRouteVerified=%u,status=%s\n",
                   bestAnalogRouteVerified ? 1u : 0u,
                   bestAnalogRouteVerified ? "fully_verified" : "gpio_ok_but_analog_unhealthy");
            if (!bestAnalogRouteVerified) {
                printf("DBGFDC_S5D5,stage=lock_failed_after_sweep,reason=candidate_analog_health_below_requirement,"
                       "healthClass=%s,rejectReason=%s,score=%ld,status=error\n",
                       sensorarrayS5d5CandidateHealthClassName(bestCandidate.healthClass),
                       bestCandidate.rejectReason ? bestCandidate.rejectReason : SENSORARRAY_NA,
                       (long)bestCandidate.score);
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }
            if (bestCandidate.score < lockMinScore) {
                printf("DBGFDC_S5D5,stage=lock_failed_after_sweep,reason=score_below_minimum,bestScore=%ld,"
                       "minScore=%ld,status=error\n",
                       (long)bestCandidate.score,
                       (long)lockMinScore);
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }

            esp_err_t lockErr = sensorarrayS5d5ApplyLockedCandidate(fdcState->handle, &bestCandidate, fdcMap->channel);
            if (lockErr != ESP_OK) {
                esp_err_t recoveryErr = sensorarrayS5d5HandleFaultAndRecover(state,
                                                                              fdcState,
                                                                              &checkpoint,
                                                                              &recoveryTracker,
                                                                              SENSORARRAY_S5D5_FAULT_LOCK_APPLY_FAILED,
                                                                              "lock_apply_failed",
                                                                              1u,
                                                                              maxRecoveryAttempts,
                                                                              recoveryCooldownMs,
                                                                              initSettleDelayMs);
                modeFdcInitialized = (recoveryErr == ESP_OK);
                runtimeReadback.valid = false;
                if (recoveryErr == ESP_OK) {
                    if (degradedMode) {
                        printf("DBGFDC_S5D5,stage=degraded_state,status=exit,recoveryFailureStreak=%lu\n",
                               (unsigned long)recoveryFailureStreak);
                    }
                    degradedMode = false;
                    degradedWarnCounter = 0u;
                    recoveryFailureStreak = 0u;
                } else {
                    recoveryFailureStreak++;
                    if (!degradedMode &&
                        recoveryFailureStreak >= SENSORARRAY_S5D5_RECOVERY_FAIL_DEGRADED_THRESHOLD) {
                        degradedMode = true;
                        printf("DBGFDC_S5D5,stage=degraded_state,status=enter,recoveryFailureStreak=%lu,reason=lock_apply_failed\n",
                               (unsigned long)recoveryFailureStreak);
                    }
                }
                locked = false;
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                continue;
            }

            uint32_t discardCountAfterLock = discardFirst ? 2u : 1u;
            for (uint32_t i = 0u; i < discardCountAfterLock; ++i) {
                sensorarrayFdcReadDiag_t throwawayDiag = {0};
                esp_err_t discardErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(fdcState->handle,
                                                                                   fdcMap->channel,
                                                                                   false,
                                                                                   fdcState->haveIds,
                                                                                   fdcState->configVerified,
                                                                                   &throwawayDiag);
                if (verboseLog || discardErr != ESP_OK) {
                    printf("DBGFDC_S5D5,stage=lock_apply,discardIndex=%lu,err=%ld,status=%s\n",
                           (unsigned long)i,
                           (long)discardErr,
                           (discardErr == ESP_OK) ? "discard_done" : "discard_read_error_continue");
                }
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
            }

            lockedCandidate = bestCandidate;
            runtimeReadback = (sensorarrayS5d5RuntimeReadback_t){
                .valid = bestCandidate.readbackValid,
                .statusReg = bestCandidate.statusRegReadback,
                .configReg = bestCandidate.configRegReadback,
                .muxConfig = bestCandidate.muxConfigReadback,
                .driveReg = bestCandidate.driveCurrentReadbackNorm,
                .highCurrentReadback = bestCandidate.highCurrentReadback,
                .driveCurrentReadbackNorm = bestCandidate.driveCurrentReadbackNorm,
                .activeChannelReadback = bestCandidate.activeChannelReadback,
                .activeChannelMatch = (bestCandidate.activeChannelReadback == (uint8_t)fdcMap->channel),
                .matchesLockedCandidate = bestCandidate.readbackMatches,
            };
            lockEpoch++;
            locked = true;
            i2cFailureStreak = 0u;
            noUnreadStreak = 0u;
            nonConvertingStreak = 0u;
            readbackMismatchStreak = 0u;
            readbackIoFailureStreak = 0u;
        }

        sensorarrayS5d5LockedSummary_t lockedSummary = {
            .rawMin = UINT_MAX,
        };
        bool needRecovery = false;
        sensorarrayS5d5FaultReason_t recoveryFault = SENSORARRAY_S5D5_FAULT_NONE;
        const char *recoveryReason = SENSORARRAY_NA;
        uint32_t recoveryStreakCount = 0u;
        uint32_t waitWarnLogCounter = 0u;
        uint32_t lockedWarnLogCounter = 0u;
        uint32_t watchdogFaultStreak = 0u;
        uint32_t amplitudeFaultStreak = 0u;
        uint32_t transportUnreadableStreak = 0u;

        for (uint32_t sampleIndex = 0u; sampleIndex < lockedSampleCount; ++sampleIndex) {
            Fdc2214CapStatus_t waitStatus = {0};
            bool waitReady = false;
            uint32_t waitPollCount = 0u;
            esp_err_t waitErr = sensorarrayS5d5WaitUnreadOrTimeout(fdcState->handle,
                                                                    fdcMap->channel,
                                                                    unreadPollTimeoutMs,
                                                                    unreadPollIntervalMs,
                                                                    verboseLog,
                                                                    warnLogStride,
                                                                    &waitWarnLogCounter,
                                                                    &waitReady,
                                                                    &waitStatus,
                                                                    &waitPollCount);

            sensorarrayS5d5FastSample_t sample = {0};
            esp_err_t readErr = ESP_OK;
            if (waitErr == ESP_OK) {
                readErr = sensorarrayS5d5ReadFastSample(fdcState->handle,
                                                        fdcMap->channel,
                                                        &waitStatus,
                                                        &sample);
            } else {
                readErr = waitErr;
            }

            lockedSummary.totalSamples++;
            if (!waitReady) {
                lockedSummary.unreadTimeoutCount++;
            }

            if (readErr != ESP_OK) {
                i2cFailureStreak++;
                noUnreadStreak = 0u;
                lockedSummary.i2cErrorCount++;
                lockedSummary.warningSamples++;
                const char *runtimeErrReason = (readErr == ESP_FAIL)
                                                   ? ((waitErr == ESP_OK) ? "readback_nack" : "unexpected_runtime_nack")
                                                   : "i2c_read_error";
                lockedSampleGlobalIndex++;
                if (verboseLog || sensorarrayLogRateLimited(&lockedWarnLogCounter, warnLogStride)) {
                    printf("DBGFDC_S5D5,stage=locked_warning,index=%lu,lockEpoch=%lu,reason=%s,err=%ld,"
                           "waitReady=%u,unread=%u,polls=%lu,i2cStreak=%lu\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           runtimeErrReason,
                           (long)readErr,
                           waitReady ? 1u : 0u,
                           waitStatus.UnreadConversion[(uint8_t)fdcMap->channel] ? 1u : 0u,
                           (unsigned long)waitPollCount,
                           (unsigned long)i2cFailureStreak);
                }
                if (i2cFailureStreak >= recoveryReinitThreshold && !needRecovery) {
                    needRecovery = true;
                    if (readErr == ESP_FAIL) {
                        recoveryReason = (waitErr == ESP_OK) ? "readback_nack_streak" : "unexpected_runtime_nack_streak";
                    } else {
                        recoveryReason = "i2c_error_streak";
                    }
                    recoveryFault = SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED;
                    recoveryStreakCount = i2cFailureStreak;
                }
                bool sclHigh = true;
                bool sdaHigh = true;
                esp_err_t lineErr = boardSupportI2cCheckLines(fdcState->i2cCtx, &sclHigh, &sdaHigh);
                if (!needRecovery && lineErr == ESP_OK && (!sclHigh || !sdaHigh)) {
                    needRecovery = true;
                    recoveryReason = "i2c_line_stuck";
                    recoveryFault = SENSORARRAY_S5D5_FAULT_I2C_LINE_STUCK;
                    recoveryStreakCount = i2cFailureStreak;
                }
                if (needRecovery) {
                    break;
                }
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
                continue;
            }

            i2cFailureStreak = 0u;

            bool doReadbackDiag = (sampleIndex == 0u) ||
                                  sensorarrayLogShouldPrint(lockedSampleGlobalIndex + 1u, diagLogStride);
            if (doReadbackDiag) {
                uint16_t statusReg = 0u;
                uint16_t configReg = 0u;
                uint16_t muxConfig = 0u;
                uint16_t driveReg = 0u;
                esp_err_t readbackErr =
                    sensorarrayS5d5ReadKeyRegs(fdcState->handle, &statusReg, &configReg, &muxConfig, &driveReg);
                if (readbackErr == ESP_OK) {
                    readbackIoFailureStreak = 0u;
                    bool highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
                    uint16_t driveCurrentReadbackNorm = (uint16_t)(driveReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
                    uint8_t activeChannelReadback =
                        (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >>
                                  SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
                    bool activeChannelMatch = (activeChannelReadback == (uint8_t)fdcMap->channel);
                    bool readbackMatchesLockedCandidate =
                        activeChannelMatch &&
                        (highCurrentReadback == lockedCandidate.highCurrentReq) &&
                        (driveCurrentReadbackNorm == lockedCandidate.driveCurrentNorm);

                    runtimeReadback.valid = true;
                    runtimeReadback.statusReg = statusReg;
                    runtimeReadback.configReg = configReg;
                    runtimeReadback.muxConfig = muxConfig;
                    runtimeReadback.driveReg = driveReg;
                    runtimeReadback.highCurrentReadback = highCurrentReadback;
                    runtimeReadback.driveCurrentReadbackNorm = driveCurrentReadbackNorm;
                    runtimeReadback.activeChannelReadback = activeChannelReadback;
                    runtimeReadback.activeChannelMatch = activeChannelMatch;
                    runtimeReadback.matchesLockedCandidate = readbackMatchesLockedCandidate;

                    if (verboseLog) {
                        printf("DBGFDC_S5D5,stage=locked_diag,index=%lu,lockEpoch=%lu,statusReg=0x%04X,configReg=0x%04X,"
                               "muxConfig=0x%04X,driveReg=0x%04X,activeChannelReadback=%u,match=%u\n",
                               (unsigned long)(lockedSampleGlobalIndex + 1u),
                               (unsigned long)lockEpoch,
                               statusReg,
                               configReg,
                               muxConfig,
                               driveReg,
                               (unsigned)activeChannelReadback,
                               readbackMatchesLockedCandidate ? 1u : 0u);
                    }
                } else {
                    readbackIoFailureStreak++;
                    runtimeReadback.valid = false;
                    lockedSummary.warningSamples++;
                    if (verboseLog || sensorarrayLogRateLimited(&lockedWarnLogCounter, warnLogStride)) {
                        printf("DBGFDC_S5D5,stage=locked_warning,index=%lu,lockEpoch=%lu,reason=%s,err=%ld,"
                               "readbackIoStreak=%lu\n",
                               (unsigned long)(lockedSampleGlobalIndex + 1u),
                               (unsigned long)lockEpoch,
                               (readbackErr == ESP_FAIL) ? "readback_nack" : "readback_i2c_error",
                               (long)readbackErr,
                               (unsigned long)readbackIoFailureStreak);
                    }
                    if (!needRecovery && readbackIoFailureStreak >= recoveryReinitThreshold) {
                        needRecovery = true;
                        recoveryReason = (readbackErr == ESP_FAIL) ? "readback_nack_streak"
                                                                   : "readback_i2c_error_streak";
                        recoveryFault = SENSORARRAY_S5D5_FAULT_I2C_TRANSACTION_FAILED;
                        recoveryStreakCount = readbackIoFailureStreak;
                    }
                }
            }
            if (needRecovery) {
                break;
            }

            bool activeChannelMatch = runtimeReadback.valid ? runtimeReadback.activeChannelMatch : true;
            if (runtimeReadback.valid && !runtimeReadback.matchesLockedCandidate) {
                readbackMismatchStreak++;
            } else if (runtimeReadback.valid) {
                readbackMismatchStreak = 0u;
            }

            if (!sample.converting) {
                nonConvertingStreak++;
                lockedSummary.nonConvertingCount++;
            } else {
                nonConvertingStreak = 0u;
            }
            if (!sample.unreadPresent) {
                noUnreadStreak++;
            } else {
                noUnreadStreak = 0u;
                lockedSummary.unreadPresentCount++;
            }
            bool transportReadableNow = sample.converting && (sample.raw28 != 0u);
            if (!transportReadableNow) {
                transportUnreadableStreak++;
            } else {
                transportUnreadableStreak = 0u;
            }
            if (sample.errWatchdog) {
                watchdogFaultStreak++;
            } else {
                watchdogFaultStreak = 0u;
            }
            if (sample.errAmplitude) {
                amplitudeFaultStreak++;
            } else {
                amplitudeFaultStreak = 0u;
            }

            if (nonConvertingStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "non_converting_streak";
                recoveryFault = SENSORARRAY_S5D5_FAULT_TRANSPORT_UNREADABLE_PERSISTENT;
                recoveryStreakCount = nonConvertingStreak;
            }
            if (transportUnreadableStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "transport_unreadable_streak";
                recoveryFault = SENSORARRAY_S5D5_FAULT_TRANSPORT_UNREADABLE_PERSISTENT;
                recoveryStreakCount = transportUnreadableStreak;
            }
            if (readbackMismatchStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "config_readback_mismatch";
                recoveryFault = SENSORARRAY_S5D5_FAULT_READBACK_MISMATCH;
                recoveryStreakCount = readbackMismatchStreak;
            }
            if (noUnreadStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "no_unread_streak";
                recoveryFault = SENSORARRAY_S5D5_FAULT_UNREAD_TIMEOUT_REPEATED;
                recoveryStreakCount = noUnreadStreak;
            }
            if (watchdogFaultStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "watchdog_fault_streak";
                recoveryFault = SENSORARRAY_S5D5_FAULT_WATCHDOG_FAULT_PERSISTENT;
                recoveryStreakCount = watchdogFaultStreak;
            }
            if (amplitudeFaultStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "amplitude_fault_streak";
                recoveryFault = SENSORARRAY_S5D5_FAULT_AMPLITUDE_FAULT_PERSISTENT;
                recoveryStreakCount = amplitudeFaultStreak;
            }

            bool sampleHealthReadable = sample.converting &&
                                        sample.unreadPresent &&
                                        !sample.errWatchdog &&
                                        !sample.errAmplitude &&
                                        (sample.raw28 != 0u);
            bool qualityGood = sampleHealthReadable && waitReady && activeChannelMatch;
            if (qualityGood) {
                lockedSummary.goodSamples++;
            } else {
                lockedSummary.warningSamples++;
                if (verboseLog || sensorarrayLogRateLimited(&lockedWarnLogCounter, warnLogStride)) {
                    printf("DBGFDC_S5D5,stage=locked_warning,index=%lu,lockEpoch=%lu,reason=sample_unhealthy,"
                           "converting=%u,unread=%u,watchdog=%u,amplitude=%u,activeChannelMatch=%u,sampleValid=%u\n",
                           (unsigned long)(lockedSampleGlobalIndex + 1u),
                           (unsigned long)lockEpoch,
                           sample.converting ? 1u : 0u,
                           sample.unreadPresent ? 1u : 0u,
                           sample.errWatchdog ? 1u : 0u,
                           sample.errAmplitude ? 1u : 0u,
                           activeChannelMatch ? 1u : 0u,
                           sampleHealthReadable ? 1u : 0u);
                }
            }
            if (sample.errAmplitude) {
                lockedSummary.amplitudeFaultSamples++;
            }
            if (sample.errWatchdog) {
                lockedSummary.watchdogFaultSamples++;
            }
            if (sampleHealthReadable) {
                lockedSummary.healthReadableCount++;
            }
            if (sample.raw28 != 0u) {
                lockedSummary.nonZeroRawCount++;
                if (sample.raw28 < lockedSummary.rawMin) {
                    lockedSummary.rawMin = sample.raw28;
                }
                if (sample.raw28 > lockedSummary.rawMax) {
                    lockedSummary.rawMax = sample.raw28;
                }
                lockedSummary.rawSum += sample.raw28;
            }

            sensorarrayFdcFrequencyRestore_t freqRestore = {0};
            bool haveRestoredFreq = sensorarrayMeasureFdcRestoreFrequency(fdcState, sample.raw28, &freqRestore);
            double restoredSensorFreqHz = haveRestoredFreq ? freqRestore.restoredSensorFrequencyHz : 0.0;
            if (haveRestoredFreq) {
                lockedSummary.freqHzSum += restoredSensorFreqHz;
                lockedSummary.freqSampleCount++;
            }

            double totalCapPf = 0.0;
            double netCapPf = 0.0;
            bool haveNetCapPf = false;
            const char *capReason = SENSORARRAY_NA;
            bool capGateOpen = qualityGood &&
                               lockedCandidate.analogRouteVerified &&
                               (!runtimeReadback.valid || runtimeReadback.matchesLockedCandidate) &&
                               !sample.errWatchdog &&
                               !sample.errAmplitude;
            bool haveTotalCapPf = false;
            if (capGateOpen) {
                haveTotalCapPf = sensorarrayS5d5TryComputeCapacitance(&capConfig,
                                                                       restoredSensorFreqHz,
                                                                       &totalCapPf,
                                                                       &netCapPf,
                                                                       &haveNetCapPf,
                                                                       &capReason);
            } else {
                capReason = "cap_gate_blocked";
                haveNetCapPf = false;
            }
            if (haveTotalCapPf) {
                lockedSummary.totalCapPfSum += totalCapPf;
                lockedSummary.totalCapSampleCount++;
                if (haveNetCapPf) {
                    lockedSummary.netCapPfSum += netCapPf;
                    lockedSummary.netCapSampleCount++;
                }
            }
            const char *statusName = sensorarrayMeasureFdcSampleStatusName(sample.statusCode);

            uint32_t refClockHzMeta = SENSORARRAY_FDC_REF_CLOCK_HZ;
            sensorarrayFdcRefClockQuality_t refClockQualityMeta = SENSORARRAY_FDC_REF_CLOCK_QUALITY_UNKNOWN;
            uint16_t clockDiv0RawMeta = SENSORARRAY_FDC_DEBUG_CLOCK_DIVIDERS_CH0;
            Fdc2214CapClockDividerInfo_t clockInfoMeta = {0};
            bool haveClockInfoMeta = false;
            sensorarrayS5d5ResolveClockMetadata(fdcState,
                                                &refClockHzMeta,
                                                &refClockQualityMeta,
                                                &clockDiv0RawMeta,
                                                &clockInfoMeta,
                                                &haveClockInfoMeta);

            char restoredFreqField[72] = {0};
            if (haveRestoredFreq) {
                (void)snprintf(restoredFreqField, sizeof(restoredFreqField), "%.3f", restoredSensorFreqHz);
            } else {
                (void)snprintf(restoredFreqField, sizeof(restoredFreqField), "na(reason=restore_failed)");
            }

            lockedSampleGlobalIndex++;
            bool emitLockedSampleLog = verboseLog || sensorarrayLogShouldPrint(lockedSampleGlobalIndex, warnLogStride);
            if (emitLockedSampleLog) {
                if (verboseLog) {
                if (haveTotalCapPf) {
                    if (haveNetCapPf) {
                        printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                               "raw=%lu,refClockHz=%lu,refClockQuality=%s,clockDiv0Raw=0x%04X,finSel=%u,frefDivider=%u,"
                               "restoredSensorFreqHz=%s,totalCapPf=%.3f,netCapPf=%.3f,inductorUh=%.3f,fixedCapPf=%.3f,"
                               "parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                               (unsigned long)lockedSampleGlobalIndex,
                               (unsigned long)lockEpoch,
                               lockedCandidate.highCurrentReq ? 1u : 0u,
                               lockedCandidate.driveCurrentNorm,
                               (unsigned long)sample.raw28,
                               (unsigned long)refClockHzMeta,
                               sensorarrayMeasureFdcRefClockQualityName(refClockQualityMeta),
                               clockDiv0RawMeta,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FinSel : 0u,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FrefDivider : 0u,
                               restoredFreqField,
                               totalCapPf,
                               netCapPf,
                               capConfig.inductorValueUh,
                               capConfig.fixedCapPf,
                               capConfig.parasiticCapPf,
                               sample.unreadPresent ? 1u : 0u,
                               sample.converting ? 1u : 0u,
                               sample.errWatchdog ? 1u : 0u,
                               sample.errAmplitude ? 1u : 0u,
                               qualityGood ? "good" : "warning",
                               statusName);
                    } else {
                        printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                               "raw=%lu,refClockHz=%lu,refClockQuality=%s,clockDiv0Raw=0x%04X,finSel=%u,frefDivider=%u,"
                               "restoredSensorFreqHz=%s,totalCapPf=%.3f,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,"
                               "unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                               (unsigned long)lockedSampleGlobalIndex,
                               (unsigned long)lockEpoch,
                               lockedCandidate.highCurrentReq ? 1u : 0u,
                               lockedCandidate.driveCurrentNorm,
                               (unsigned long)sample.raw28,
                               (unsigned long)refClockHzMeta,
                               sensorarrayMeasureFdcRefClockQualityName(refClockQualityMeta),
                               clockDiv0RawMeta,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FinSel : 0u,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FrefDivider : 0u,
                               restoredFreqField,
                               totalCapPf,
                               capConfig.inductorValueUh,
                               capConfig.fixedCapPf,
                               capConfig.parasiticCapPf,
                               sample.unreadPresent ? 1u : 0u,
                               sample.converting ? 1u : 0u,
                               sample.errWatchdog ? 1u : 0u,
                               sample.errAmplitude ? 1u : 0u,
                               qualityGood ? "good" : "warning",
                               statusName);
                    }
                } else {
                    if (capConfig.enableNetCapOutput) {
                        printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                               "raw=%lu,refClockHz=%lu,refClockQuality=%s,clockDiv0Raw=0x%04X,finSel=%u,frefDivider=%u,"
                               "restoredSensorFreqHz=%s,totalCapPf=na(reason=%s),netCapPf=na(reason=%s),inductorUh=%.3f,"
                               "fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,"
                               "status=%s\n",
                               (unsigned long)lockedSampleGlobalIndex,
                               (unsigned long)lockEpoch,
                               lockedCandidate.highCurrentReq ? 1u : 0u,
                               lockedCandidate.driveCurrentNorm,
                               (unsigned long)sample.raw28,
                               (unsigned long)refClockHzMeta,
                               sensorarrayMeasureFdcRefClockQualityName(refClockQualityMeta),
                               clockDiv0RawMeta,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FinSel : 0u,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FrefDivider : 0u,
                               restoredFreqField,
                               capReason,
                               capReason,
                               capConfig.inductorValueUh,
                               capConfig.fixedCapPf,
                               capConfig.parasiticCapPf,
                               sample.unreadPresent ? 1u : 0u,
                               sample.converting ? 1u : 0u,
                               sample.errWatchdog ? 1u : 0u,
                               sample.errAmplitude ? 1u : 0u,
                               qualityGood ? "good" : "warning",
                               statusName);
                    } else {
                        printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                               "raw=%lu,refClockHz=%lu,refClockQuality=%s,clockDiv0Raw=0x%04X,finSel=%u,frefDivider=%u,"
                               "restoredSensorFreqHz=%s,totalCapPf=na(reason=%s),inductorUh=%.3f,fixedCapPf=%.3f,"
                               "parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                               (unsigned long)lockedSampleGlobalIndex,
                               (unsigned long)lockEpoch,
                               lockedCandidate.highCurrentReq ? 1u : 0u,
                               lockedCandidate.driveCurrentNorm,
                               (unsigned long)sample.raw28,
                               (unsigned long)refClockHzMeta,
                               sensorarrayMeasureFdcRefClockQualityName(refClockQualityMeta),
                               clockDiv0RawMeta,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FinSel : 0u,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FrefDivider : 0u,
                               restoredFreqField,
                               capReason,
                               capConfig.inductorValueUh,
                               capConfig.fixedCapPf,
                               capConfig.parasiticCapPf,
                               sample.unreadPresent ? 1u : 0u,
                               sample.converting ? 1u : 0u,
                               sample.errWatchdog ? 1u : 0u,
                               sample.errAmplitude ? 1u : 0u,
                               qualityGood ? "good" : "warning",
                               statusName);
                    }
                }
                } else {
                    if (capConfig.enableNetCapOutput) {
                        printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                               "raw=%lu,refClockHz=%lu,refClockQuality=%s,clockDiv0Raw=0x%04X,finSel=%u,frefDivider=%u,"
                               "restoredSensorFreqHz=%s,totalCapPf=na(reason=%s),netCapPf=na(reason=%s),inductorUh=%.3f,"
                               "fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                               (unsigned long)lockedSampleGlobalIndex,
                               (unsigned long)lockEpoch,
                               lockedCandidate.highCurrentReq ? 1u : 0u,
                               lockedCandidate.driveCurrentNorm,
                               (unsigned long)sample.raw28,
                               (unsigned long)refClockHzMeta,
                               sensorarrayMeasureFdcRefClockQualityName(refClockQualityMeta),
                               clockDiv0RawMeta,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FinSel : 0u,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FrefDivider : 0u,
                               restoredFreqField,
                               capReason,
                               capReason,
                               capConfig.inductorValueUh,
                               capConfig.fixedCapPf,
                               capConfig.parasiticCapPf,
                               sample.unreadPresent ? 1u : 0u,
                               sample.converting ? 1u : 0u,
                               sample.errWatchdog ? 1u : 0u,
                               sample.errAmplitude ? 1u : 0u,
                               qualityGood ? "good" : "warning",
                               statusName);
                    } else {
                        printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                               "raw=%lu,refClockHz=%lu,refClockQuality=%s,clockDiv0Raw=0x%04X,finSel=%u,frefDivider=%u,"
                               "restoredSensorFreqHz=%s,totalCapPf=na(reason=%s),inductorUh=%.3f,fixedCapPf=%.3f,"
                               "parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                               (unsigned long)lockedSampleGlobalIndex,
                               (unsigned long)lockEpoch,
                               lockedCandidate.highCurrentReq ? 1u : 0u,
                               lockedCandidate.driveCurrentNorm,
                               (unsigned long)sample.raw28,
                               (unsigned long)refClockHzMeta,
                               sensorarrayMeasureFdcRefClockQualityName(refClockQualityMeta),
                               clockDiv0RawMeta,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FinSel : 0u,
                               haveClockInfoMeta ? (unsigned)clockInfoMeta.FrefDivider : 0u,
                               restoredFreqField,
                               capReason,
                               capConfig.inductorValueUh,
                               capConfig.fixedCapPf,
                               capConfig.parasiticCapPf,
                               sample.unreadPresent ? 1u : 0u,
                               sample.converting ? 1u : 0u,
                               sample.errWatchdog ? 1u : 0u,
                               sample.errAmplitude ? 1u : 0u,
                               qualityGood ? "good" : "warning",
                               statusName);
                    }
                }
            }
            if (needRecovery) {
                break;
            }
            sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
        }

        if (lockedSummary.nonZeroRawCount > 0u) {
            lockedSummary.rawMean = (uint32_t)(lockedSummary.rawSum / lockedSummary.nonZeroRawCount);
            lockedSummary.rawSpan = lockedSummary.rawMax - lockedSummary.rawMin;
            if (lockedSummary.rawMean > 0u) {
                lockedSummary.rawSpanPermille =
                    (uint32_t)(((uint64_t)lockedSummary.rawSpan * 1000ull) / lockedSummary.rawMean);
            }
        } else {
            lockedSummary.rawMin = 0u;
            lockedSummary.rawMax = 0u;
        }

        sensorarrayS5d5RawSpanBand_t lockedSpanBand = SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL;
        sensorarrayS5d5SweepCandidate_t lockedHealthProxy = {
            .nonZeroRawCount = lockedSummary.nonZeroRawCount,
            .rawMean = lockedSummary.rawMean,
            .rawSpanPermille = lockedSummary.rawSpanPermille,
        };
        lockedSpanBand = sensorarrayS5d5EvaluateRawSpanBand(&lockedHealthProxy);

        bool lockAnalogHealthy = (lockedSummary.totalSamples > 0u) &&
                                 (lockedSummary.healthReadableCount > 0u) &&
                                 ((lockedSummary.healthReadableCount * 100u) >= (lockedSummary.totalSamples * 50u)) &&
                                 ((lockedSummary.unreadPresentCount * 100u) >= (lockedSummary.totalSamples * 50u)) &&
                                 ((lockedSummary.amplitudeFaultSamples * 3u) < (lockedSummary.totalSamples * 2u)) &&
                                 ((lockedSummary.watchdogFaultSamples * 3u) < (lockedSummary.totalSamples * 2u)) &&
                                 (lockedSpanBand != SENSORARRAY_S5D5_RAW_SPAN_BAND_TOO_SMALL);
        if (!needRecovery && !lockAnalogHealthy) {
            needRecovery = true;
            recoveryReason = "lock_failed_after_selection";
            recoveryFault = SENSORARRAY_S5D5_FAULT_RAW_SPAN_OR_HEALTH_GATE_FAILED;
            recoveryStreakCount = (lockedSummary.totalSamples > lockedSummary.goodSamples)
                                      ? (lockedSummary.totalSamples - lockedSummary.goodSamples)
                                      : 1u;
        }

        bool haveAvgRestoredSensorFreqHz = (lockedSummary.freqSampleCount > 0u);
        bool haveAvgTotalCapPf = (lockedSummary.totalCapSampleCount > 0u);
        bool haveAvgNetCapPf = (lockedSummary.netCapSampleCount > 0u);
        double avgRestoredSensorFreqHz =
            haveAvgRestoredSensorFreqHz ? (lockedSummary.freqHzSum / (double)lockedSummary.freqSampleCount) : 0.0;
        double avgTotalCapPf =
            haveAvgTotalCapPf ? (lockedSummary.totalCapPfSum / (double)lockedSummary.totalCapSampleCount) : 0.0;
        double avgNetCapPf = haveAvgNetCapPf ? (lockedSummary.netCapPfSum / (double)lockedSummary.netCapSampleCount) : 0.0;

        const char *avgTotalCapReason = capConfig.enableCapComputation
                                            ? ((capConfig.inductorValueUh > 0.0) ? "no_valid_cap_sample"
                                                                                 : "no_inductor_value")
                                            : "cap_computation_disabled";
        const char *avgNetCapReason = capConfig.enableCapComputation
                                          ? ((capConfig.inductorValueUh > 0.0) ? "no_valid_net_cap_sample"
                                                                               : "no_inductor_value")
                                          : "cap_computation_disabled";

        uint32_t minGoodSamplesForCap = (lockedSampleCount >= 4u) ? 4u : lockedSampleCount;
        bool capSummaryGate = lockedCandidate.analogRouteVerified &&
                              lockAnalogHealthy &&
                              (lockedSummary.goodSamples >= minGoodSamplesForCap) &&
                              (lockedSummary.i2cErrorCount == 0u) &&
                              (lockedSummary.watchdogFaultSamples == 0u) &&
                              (lockedSummary.amplitudeFaultSamples == 0u);
        if (!capSummaryGate) {
            haveAvgTotalCapPf = false;
            haveAvgNetCapPf = false;
            avgTotalCapReason = "cap_gate_blocked";
            avgNetCapReason = "cap_gate_blocked";
            printf("DBGFDC_S5D5,stage=cap_gate,result=blocked,reason=gate_not_met,analogRouteVerified=%u,"
                   "lockAnalogHealthy=%u,goodSamples=%lu,minGoodSamples=%lu,i2cErr=%lu,watchdog=%lu,amplitude=%lu\n",
                   lockedCandidate.analogRouteVerified ? 1u : 0u,
                   lockAnalogHealthy ? 1u : 0u,
                   (unsigned long)lockedSummary.goodSamples,
                   (unsigned long)minGoodSamplesForCap,
                   (unsigned long)lockedSummary.i2cErrorCount,
                   (unsigned long)lockedSummary.watchdogFaultSamples,
                   (unsigned long)lockedSummary.amplitudeFaultSamples);
        }

        char avgRestoredSensorFreqHzField[64] = {0};
        char avgTotalCapPfField[64] = {0};
        char avgNetCapPfField[64] = {0};
        if (haveAvgRestoredSensorFreqHz) {
            (void)snprintf(avgRestoredSensorFreqHzField,
                           sizeof(avgRestoredSensorFreqHzField),
                           "%.3f",
                           avgRestoredSensorFreqHz);
        } else {
            (void)snprintf(avgRestoredSensorFreqHzField,
                           sizeof(avgRestoredSensorFreqHzField),
                           "na(reason=no_valid_frequency)");
        }
        if (haveAvgTotalCapPf) {
            (void)snprintf(avgTotalCapPfField, sizeof(avgTotalCapPfField), "%.3f", avgTotalCapPf);
        } else {
            (void)snprintf(avgTotalCapPfField, sizeof(avgTotalCapPfField), "na(reason=%s)", avgTotalCapReason);
        }
        if (haveAvgNetCapPf) {
            (void)snprintf(avgNetCapPfField, sizeof(avgNetCapPfField), "%.3f", avgNetCapPf);
        } else {
            (void)snprintf(avgNetCapPfField, sizeof(avgNetCapPfField), "na(reason=%s)", avgNetCapReason);
        }

        if (capConfig.enableNetCapOutput) {
            printf("DBGFDC_S5D5,stage=locked_summary,lockEpoch=%lu,samples=%lu,good=%lu,warning=%lu,"
                   "amplitudeFault=%lu,watchdogFault=%lu,i2cErr=%lu,waitTimeout=%lu,nonConverting=%lu,unreadPresent=%lu,"
                   "healthReadable=%lu,rawMin=%lu,rawMax=%lu,rawAvg=%lu,rawSpan=%lu,rawSpanPermille=%lu,rawSpanBand=%s,"
                   "avgRestoredSensorFreqHz=%s,"
                   "avgTotalCapPf=%s,avgNetCapPf=%s,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,"
                   "i2cStreak=%lu,noUnreadStreak=%lu,nonConvertingStreak=%lu,readbackMismatchStreak=%lu,status=%s\n",
                   (unsigned long)lockEpoch,
                   (unsigned long)lockedSummary.totalSamples,
                   (unsigned long)lockedSummary.goodSamples,
                   (unsigned long)lockedSummary.warningSamples,
                   (unsigned long)lockedSummary.amplitudeFaultSamples,
                   (unsigned long)lockedSummary.watchdogFaultSamples,
                   (unsigned long)lockedSummary.i2cErrorCount,
                   (unsigned long)lockedSummary.unreadTimeoutCount,
                   (unsigned long)lockedSummary.nonConvertingCount,
                   (unsigned long)lockedSummary.unreadPresentCount,
                   (unsigned long)lockedSummary.healthReadableCount,
                   (unsigned long)lockedSummary.rawMin,
                   (unsigned long)lockedSummary.rawMax,
                   (unsigned long)lockedSummary.rawMean,
                   (unsigned long)lockedSummary.rawSpan,
                   (unsigned long)lockedSummary.rawSpanPermille,
                   sensorarrayS5d5RawSpanBandName(lockedSpanBand),
                   avgRestoredSensorFreqHzField,
                   avgTotalCapPfField,
                   avgNetCapPfField,
                   capConfig.inductorValueUh,
                   capConfig.fixedCapPf,
                   capConfig.parasiticCapPf,
                   (unsigned long)i2cFailureStreak,
                   (unsigned long)noUnreadStreak,
                   (unsigned long)nonConvertingStreak,
                   (unsigned long)readbackMismatchStreak,
                   needRecovery ? "recovery_pending" : "locked_continue");
        } else {
            printf("DBGFDC_S5D5,stage=locked_summary,lockEpoch=%lu,samples=%lu,good=%lu,warning=%lu,"
                   "amplitudeFault=%lu,watchdogFault=%lu,i2cErr=%lu,waitTimeout=%lu,nonConverting=%lu,unreadPresent=%lu,"
                   "healthReadable=%lu,rawMin=%lu,rawMax=%lu,rawAvg=%lu,rawSpan=%lu,rawSpanPermille=%lu,rawSpanBand=%s,"
                   "avgRestoredSensorFreqHz=%s,avgTotalCapPf=%s,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,"
                   "i2cStreak=%lu,noUnreadStreak=%lu,nonConvertingStreak=%lu,readbackMismatchStreak=%lu,status=%s\n",
                   (unsigned long)lockEpoch,
                   (unsigned long)lockedSummary.totalSamples,
                   (unsigned long)lockedSummary.goodSamples,
                   (unsigned long)lockedSummary.warningSamples,
                   (unsigned long)lockedSummary.amplitudeFaultSamples,
                   (unsigned long)lockedSummary.watchdogFaultSamples,
                   (unsigned long)lockedSummary.i2cErrorCount,
                   (unsigned long)lockedSummary.unreadTimeoutCount,
                   (unsigned long)lockedSummary.nonConvertingCount,
                   (unsigned long)lockedSummary.unreadPresentCount,
                   (unsigned long)lockedSummary.healthReadableCount,
                   (unsigned long)lockedSummary.rawMin,
                   (unsigned long)lockedSummary.rawMax,
                   (unsigned long)lockedSummary.rawMean,
                   (unsigned long)lockedSummary.rawSpan,
                   (unsigned long)lockedSummary.rawSpanPermille,
                   sensorarrayS5d5RawSpanBandName(lockedSpanBand),
                   avgRestoredSensorFreqHzField,
                   avgTotalCapPfField,
                   capConfig.inductorValueUh,
                   capConfig.fixedCapPf,
                   capConfig.parasiticCapPf,
                   (unsigned long)i2cFailureStreak,
                   (unsigned long)noUnreadStreak,
                   (unsigned long)nonConvertingStreak,
                   (unsigned long)readbackMismatchStreak,
                   needRecovery ? "recovery_pending" : "locked_continue");
        }

        if (needRecovery) {
            if (recoveryReason && strcmp(recoveryReason, "lock_failed_after_selection") == 0) {
                printf("DBGFDC_S5D5,stage=lock_failed_after_selection,lockEpoch=%lu,reason=%s,good=%lu,warning=%lu,"
                       "healthReadable=%lu,rawSpanPermille=%lu,rawSpanBand=%s,status=rollback\n",
                       (unsigned long)lockEpoch,
                       recoveryReason,
                       (unsigned long)lockedSummary.goodSamples,
                       (unsigned long)lockedSummary.warningSamples,
                       (unsigned long)lockedSummary.healthReadableCount,
                       (unsigned long)lockedSummary.rawSpanPermille,
                       sensorarrayS5d5RawSpanBandName(lockedSpanBand));
            }
            esp_err_t recoveryErr = sensorarrayS5d5HandleFaultAndRecover(state,
                                                                          fdcState,
                                                                          &checkpoint,
                                                                          &recoveryTracker,
                                                                          recoveryFault,
                                                                          recoveryReason,
                                                                          recoveryStreakCount,
                                                                          maxRecoveryAttempts,
                                                                          recoveryCooldownMs,
                                                                          initSettleDelayMs);
            locked = false;
            runtimeReadback.valid = false;
            modeFdcInitialized = (recoveryErr == ESP_OK);
            if (recoveryErr == ESP_OK) {
                if (degradedMode) {
                    printf("DBGFDC_S5D5,stage=degraded_state,status=exit,recoveryFailureStreak=%lu\n",
                           (unsigned long)recoveryFailureStreak);
                }
                degradedMode = false;
                degradedWarnCounter = 0u;
                recoveryFailureStreak = 0u;
                i2cFailureStreak = 0u;
                noUnreadStreak = 0u;
                nonConvertingStreak = 0u;
                readbackMismatchStreak = 0u;
                readbackIoFailureStreak = 0u;
            } else {
                recoveryFailureStreak++;
                if (!degradedMode &&
                    recoveryFailureStreak >= SENSORARRAY_S5D5_RECOVERY_FAIL_DEGRADED_THRESHOLD) {
                    degradedMode = true;
                    printf("DBGFDC_S5D5,stage=degraded_state,status=enter,recoveryFailureStreak=%lu,reason=%s\n",
                           (unsigned long)recoveryFailureStreak,
                           recoveryReason ? recoveryReason : SENSORARRAY_NA);
                }
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
            }
            continue;
        }

        sensorarrayDebugSelftestDelayMs(loopDelayMs);
    }
}
