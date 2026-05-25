#include "sensorarrayDebugSelftest.h"

#include <limits.h>
#include <stdio.h>

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

#define SENSORARRAY_S5D5_DRIVE_CURRENT_MASK 0xF800u
#define SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT 14u
#define SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK 0xC000u
#define SENSORARRAY_S5D5_CONFIG_SLEEP_MODE_EN_MASK 0x2000u
#define SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK 0x0040u
#define SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK 0x0007u
#define SENSORARRAY_S5D5_REG_STATUS 0x18u
#define SENSORARRAY_S5D5_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_S5D5_REG_CONFIG 0x1Au
#define SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0 0x1Eu
#define SENSORARRAY_S5D5_STEP_SETTLE_MS 350u
#define SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS 20u
#define SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS 500u
#define SENSORARRAY_S5D5_RELOCK_FREQ_ABS_DELTA_HZ 500000.0
#define SENSORARRAY_S5D5_RELOCK_FREQ_REL_DELTA 0.10
#define SENSORARRAY_S5D5_RELOCK_FREQ_STREAK_THRESHOLD 3u
#define SENSORARRAY_S5D5_RELOCK_AMPLITUDE_STREAK_THRESHOLD 2u
#define SENSORARRAY_S5D5_RELOCK_DISCARD_SAMPLES 2u
#define SENSORARRAY_S5D5_SWEEP_PLAN_MAX 64u
#define SENSORARRAY_S5D5_FAST_SWEEP_PLAN_MAX 8u
#define SENSORARRAY_S5D5_MEDIUM_SWEEP_PLAN_MAX 24u
#define SENSORARRAY_S5D5_LOCKED_RAW_SPAN_MULTIPLIER 4u
#define SENSORARRAY_S5D5_LOCKED_RAW_SPAN_FLOOR 1024u

typedef struct {
    uint8_t code;
    uint32_t bandwidthHz;
    const char *name;
} sensorarrayS5d5DeglitchCandidate_t;

static const uint16_t SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[] = {
    0xA000u,
    0xB800u,
    0xC000u,
    0xD000u,
    0xE000u,
    0xF800u,
};

static const bool SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[] = {
    false,
    true,
};

static const sensorarrayS5d5DeglitchCandidate_t SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[] = {
    {0x1u, 1000000u, "1MHz"},
    {0x4u, 3300000u, "3p3MHz"},
    {0x5u, 10000000u, "10MHz"},
    {0x7u, 33000000u, "33MHz"},
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
} sensorarrayS5d5RouteCheck_t;

static void sensorarrayDebugSelftestDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
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
           (routeCheck.commandMatch && routeCheck.gpioObservedMatch)
               ? "route_gpio_match"
               : "warning_route_gpio_or_command_mismatch");
    return routeCheck;
}

static esp_err_t sensorarrayApplyS5d5DeglitchStep(Fdc2214CapDevice_t *dev, uint8_t deglitchCode)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (deglitchCode) {
    case 0x1u:
    case 0x4u:
    case 0x5u:
    case 0x7u:
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t muxConfigReg = 0u;
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, &muxConfigReg);
    if (err != ESP_OK) {
        return err;
    }

    muxConfigReg &= (uint16_t)~SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK;
    muxConfigReg |= (uint16_t)(deglitchCode & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK);
    return Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, muxConfigReg);
}

static esp_err_t sensorarrayApplyS5d5DriveStep(Fdc2214CapDevice_t *dev, bool highCurrent, uint16_t driveCurrentReq)
{
    if (!dev) {
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

    uint16_t driveCurrentNorm = (uint16_t)(driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    return Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, driveCurrentNorm);
}

typedef struct {
    uint8_t deglitchReq;
    uint32_t deglitchBandwidthHz;
    const char *deglitchName;
    bool highCurrentReq;
    uint16_t driveCurrentReq;
    uint16_t driveCurrentNorm;
    uint32_t sampleCount;
    uint32_t i2cErrorCount;
    uint32_t convertingCount;
    uint32_t unreadCount;
    uint32_t watchdogCount;
    uint32_t amplitudeCount;
    uint32_t nonZeroRawCount;
    uint32_t provisionalReadableCount;
    uint32_t activeChannelMatchCount;
    uint32_t rawMin;
    uint32_t rawMax;
    uint64_t rawSum;
    uint32_t rawMean;
    uint32_t rawSpan;
    int32_t score;
    bool hardReject;
    const char *rejectReason;
    const char *sourceMode;
    uint32_t elapsedMs;
    bool selected;
    bool readbackValid;
    bool deglitchReadbackValid;
    uint8_t deglitchReadback;
    bool deglitchBandwidthSufficient;
    double deglitchMarginRatio;
    bool avgFreqValid;
    double avgFreqHz;
    bool highCurrentReadback;
    uint16_t driveCurrentReadbackNorm;
    uint8_t activeChannelReadback;
    uint16_t statusRegReadback;
    uint16_t configRegReadback;
    uint16_t muxConfigReadback;
    uint16_t clockDividersReadback;
    uint8_t finSelCode;
    uint8_t finFactor;
    uint16_t frefDivider;
    uint32_t effectiveFclkHz;
    double effectiveFrefHz;
    bool clockDecodeValid;
    const char *clockDecodeStatus;
} sensorarrayS5d5SweepCandidate_t;

typedef struct {
    bool valid;
    bool bootFullSweepDone;

    uint8_t bestDeglitchReq;
    uint32_t bestDeglitchBandwidthHz;
    const char *bestDeglitchName;

    bool bestHighCurrentReq;
    uint16_t bestDriveCurrentReq;
    uint16_t bestDriveCurrentNorm;

    double bestFreqHz;
    uint32_t bestRawMean;
    uint32_t bestRawSpan;
    int32_t bestScore;

    bool normalCurrentWorked;
    bool highCurrentNeeded;

    bool deglitch1MHzBad;
    bool deglitch3p3MHzGood;
    bool deglitch10MHzGood;
    bool deglitch33MHzNeeded;

    uint32_t fullSweepCount;
    uint32_t fastSweepCount;
    uint32_t mediumSweepCount;
    uint32_t fullSweepFallbackCount;
} sensorarrayS5d5BootEnvProfile_t;

static sensorarrayS5d5BootEnvProfile_t s_s5d5BootEnvProfile = {0};

typedef struct {
    bool valid;

    uint8_t deglitchReq;
    uint32_t deglitchBandwidthHz;
    const char *deglitchName;

    bool highCurrentReq;
    uint16_t driveCurrentReq;
    uint16_t driveCurrentNorm;

    double baselineFreqHz;
    uint32_t baselineRawMean;
    uint32_t baselineRawSpan;

    uint32_t stableCount;
    uint32_t abnormalCount;
    uint32_t profileUseCount;
    uint32_t profileFailCount;
} sensorarrayS5d5FdcProfile_t;

static sensorarrayS5d5FdcProfile_t s_s5d5FdcProfile = {0};
static uint32_t s_s5d5RuntimeFastFailStreak = 0u;

typedef struct {
    const sensorarrayS5d5DeglitchCandidate_t *deglitch;
    bool highCurrentReq;
    uint16_t driveCurrentReq;
    const char *sourceMode;
} sensorarrayS5d5SweepPlanEntry_t;

typedef struct {
    sensorarrayS5d5SweepPlanEntry_t entries[SENSORARRAY_S5D5_SWEEP_PLAN_MAX];
    size_t count;
} sensorarrayS5d5SweepPlan_t;

typedef struct {
    uint32_t candidatesTried;
    bool normalCurrentWorked;
    bool highCurrentWorked;
    bool deglitchGood[4];
} sensorarrayS5d5SweepStats_t;

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
    uint32_t i2cErrorCount;
    uint32_t unreadTimeoutCount;
    uint32_t nonConvertingCount;
    uint32_t freqSampleCount;
    uint32_t totalCapSampleCount;
    uint32_t netCapSampleCount;
    uint32_t clockValidCount;
    uint32_t clockInvalidCount;
    uint32_t rawNonZeroCount;
    uint32_t rawMin;
    uint32_t rawMax;
    double freqHzSum;
    double totalCapPfSum;
    double netCapPfSum;
} sensorarrayS5d5LockedSummary_t;

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
                                            Fdc2214CapChannel_t ch,
                                            uint16_t *outStatusReg,
                                            uint16_t *outConfigReg,
                                            uint16_t *outMuxConfig,
                                            uint16_t *outClockDividers,
                                            uint16_t *outDriveCurrent)
{
    if (!dev || !outStatusReg || !outConfigReg || !outMuxConfig || !outClockDividers || !outDriveCurrent) {
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
    err = Fdc2214CapReadClockDividers(dev, ch, outClockDividers);
    if (err != ESP_OK) {
        return err;
    }
    return Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, outDriveCurrent);
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

static size_t sensorarrayS5d5DeglitchTableCount(void)
{
    return sizeof(SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE) / sizeof(SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[0]);
}

static size_t sensorarrayS5d5DriveTableCount(void)
{
    return sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE) / sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[0]);
}

static const sensorarrayS5d5DeglitchCandidate_t *sensorarrayS5d5FindDeglitchByCode(uint8_t deglitchReq)
{
    size_t deglitchCount = sensorarrayS5d5DeglitchTableCount();
    for (size_t i = 0u; i < deglitchCount; ++i) {
        if (SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i].code ==
            (uint8_t)(deglitchReq & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK)) {
            return &SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i];
        }
    }
    return NULL;
}

static int sensorarrayS5d5DeglitchIndexFromCode(uint8_t deglitchReq)
{
    size_t deglitchCount = sensorarrayS5d5DeglitchTableCount();
    for (size_t i = 0u; i < deglitchCount; ++i) {
        if (SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i].code ==
            (uint8_t)(deglitchReq & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK)) {
            return (int)i;
        }
    }
    return -1;
}

static int sensorarrayS5d5DriveIndexFromNorm(uint16_t driveCurrentNorm)
{
    uint16_t norm = (uint16_t)(driveCurrentNorm & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    size_t driveCount = sensorarrayS5d5DriveTableCount();
    for (size_t i = 0u; i < driveCount; ++i) {
        if ((SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[i] & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK) == norm) {
            return (int)i;
        }
    }
    return -1;
}

static int32_t sensorarrayS5d5DrivePreferenceRank(uint16_t driveCurrentNorm)
{
    switch (driveCurrentNorm & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK) {
    case 0xC000u:
        return 0;
    case 0xB800u:
        return 1;
    case 0xD000u:
        return 2;
    case 0xA000u:
        return 3;
    case 0xE000u:
        return 4;
    case 0xF800u:
        return 5;
    default:
        return 6;
    }
}

static bool sensorarrayS5d5PlanAppend(sensorarrayS5d5SweepPlan_t *plan,
                                      const sensorarrayS5d5DeglitchCandidate_t *deglitch,
                                      bool highCurrentReq,
                                      uint16_t driveCurrentReq,
                                      const char *sourceMode)
{
    if (!plan || !deglitch || plan->count >= SENSORARRAY_S5D5_SWEEP_PLAN_MAX) {
        return false;
    }

    uint16_t driveNorm = (uint16_t)(driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    for (size_t i = 0u; i < plan->count; ++i) {
        const sensorarrayS5d5SweepPlanEntry_t *entry = &plan->entries[i];
        if (entry->deglitch &&
            entry->deglitch->code == deglitch->code &&
            entry->highCurrentReq == highCurrentReq &&
            ((entry->driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK) == driveNorm)) {
            return true;
        }
    }

    plan->entries[plan->count++] = (sensorarrayS5d5SweepPlanEntry_t){
        .deglitch = deglitch,
        .highCurrentReq = highCurrentReq,
        .driveCurrentReq = driveNorm,
        .sourceMode = sourceMode ? sourceMode : SENSORARRAY_NA,
    };
    return true;
}

static void sensorarrayS5d5CandidateInit(sensorarrayS5d5SweepCandidate_t *candidate,
                                         const sensorarrayS5d5DeglitchCandidate_t *deglitch,
                                         bool highCurrentReq,
                                         uint16_t driveCurrentReq,
                                         const char *sourceMode)
{
    if (!candidate || !deglitch) {
        return;
    }

    *candidate = (sensorarrayS5d5SweepCandidate_t){
        .deglitchReq = deglitch->code,
        .deglitchBandwidthHz = deglitch->bandwidthHz,
        .deglitchName = deglitch->name,
        .highCurrentReq = highCurrentReq,
        .driveCurrentReq = driveCurrentReq,
        .driveCurrentNorm = (uint16_t)(driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .rawMin = UINT_MAX,
        .rejectReason = "not_finalized",
        .sourceMode = sourceMode ? sourceMode : SENSORARRAY_NA,
        .clockDecodeStatus = "not_read",
    };
}

static void sensorarrayS5d5CandidateSetClockReadback(sensorarrayS5d5SweepCandidate_t *candidate,
                                                     uint16_t clockDividers)
{
    if (!candidate) {
        return;
    }

    const char *clockStatus = "unknown";
    candidate->clockDividersReadback = clockDividers;
    candidate->effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz();
    candidate->clockDecodeValid = sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                                            &candidate->finSelCode,
                                                                            &candidate->finFactor,
                                                                            &candidate->frefDivider,
                                                                            &clockStatus);
    candidate->clockDecodeStatus = clockStatus ? clockStatus : SENSORARRAY_NA;
    candidate->effectiveFrefHz = (candidate->clockDecodeValid && candidate->frefDivider > 0u)
                                     ? ((double)candidate->effectiveFclkHz / (double)candidate->frefDivider)
                                     : 0.0;
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
    if (candidate->nonZeroRawCount > 0u) {
        candidate->rawMean = (uint32_t)(candidate->rawSum / candidate->nonZeroRawCount);
        candidate->rawSpan = candidate->rawMax - candidate->rawMin;
    } else {
        candidate->rawMin = 0u;
        candidate->rawMax = 0u;
        candidate->rawMean = 0u;
        candidate->rawSpan = 0u;
    }

    candidate->avgFreqValid = false;
    candidate->avgFreqHz = 0.0;
    candidate->deglitchBandwidthSufficient = false;
    candidate->deglitchMarginRatio = 0.0;
    if (candidate->rawMean > 0u && candidate->readbackValid) {
        sensorarrayFdcFrequencyDiag_t freqDiag = {0};
        if (sensorarrayMeasureFdcComputeFrequencyDiag(candidate->rawMean,
                                                      candidate->clockDividersReadback,
                                                      &freqDiag) &&
            freqDiag.valid) {
            candidate->avgFreqValid = true;
            candidate->avgFreqHz = freqDiag.freqHzCorrected;
        }
    }

    candidate->hardReject = false;
    candidate->rejectReason = "ok";
    if (candidate->avgFreqValid && candidate->avgFreqHz > 0.0 && candidate->deglitchBandwidthHz > 0u) {
        candidate->deglitchMarginRatio = (double)candidate->deglitchBandwidthHz / candidate->avgFreqHz;
        if (candidate->avgFreqHz >= ((double)candidate->deglitchBandwidthHz * 0.90)) {
            candidate->deglitchBandwidthSufficient = false;
        } else {
            candidate->deglitchBandwidthSufficient = true;
        }
    }

    int32_t score = 0;
    score += (int32_t)((candidate->convertingCount * 30u) / denom);
    score += (int32_t)((candidate->unreadCount * 15u) / denom);
    score += (int32_t)((candidate->provisionalReadableCount * 25u) / denom);
    score += (int32_t)((candidate->activeChannelMatchCount * 10u) / denom);

    if (candidate->readbackValid) {
        bool deglitchMatch = candidate->deglitchReadbackValid &&
                              (candidate->deglitchReadback ==
                               (uint8_t)(candidate->deglitchReq & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK));
        bool highCurrentMatch = (candidate->highCurrentReadback == candidate->highCurrentReq);
        bool driveMatch = (candidate->driveCurrentReadbackNorm == candidate->driveCurrentNorm);
        bool activeMatch = (candidate->activeChannelReadback == (uint8_t)FDC2214_CH0);
        score += (deglitchMatch && highCurrentMatch && driveMatch && activeMatch) ? 12 : -18;
    }

    if (candidate->rawMean > 0u) {
        uint64_t spanPermille = ((uint64_t)candidate->rawSpan * 1000ull) / candidate->rawMean;
        if (spanPermille <= 2ull) {
            score += 120;
        } else if (spanPermille <= 5ull) {
            score += 95;
        } else if (spanPermille <= 10ull) {
            score += 70;
        } else if (spanPermille <= 20ull) {
            score += 40;
        } else if (spanPermille <= 50ull) {
            score += 10;
        } else if (spanPermille <= 120ull) {
            score -= 30;
        } else {
            score -= 90;
        }
    }

    if (!candidate->highCurrentReq) {
        score += 80;
    } else {
        score -= 160;
    }

    switch (candidate->driveCurrentNorm & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK) {
    case 0xB800u:
    case 0xC000u:
    case 0xD000u:
        score += 70;
        break;
    case 0xA000u:
        score += 15;
        break;
    case 0xE000u:
        score -= 90;
        break;
    case 0xF800u:
        score -= 140;
        break;
    default:
        score -= 40;
        break;
    }

    switch (candidate->deglitchReq) {
    case 0x4u:
        score += 20;
        break;
    case 0x5u:
        score += 15;
        break;
    case 0x7u:
        score -= 15;
        break;
    case 0x1u:
    default:
        break;
    }

    if (candidate->amplitudeCount > 0u) {
        score -= (int32_t)(candidate->amplitudeCount * 45u);
    }
    if (candidate->sampleCount > 0u && (candidate->amplitudeCount * 2u) >= candidate->sampleCount) {
        score -= 160;
    }

    if (!candidate->deglitchBandwidthSufficient) {
        candidate->hardReject = true;
        candidate->rejectReason = candidate->avgFreqValid ? "deglitch_bandwidth_low" : "no_valid_avg_frequency";
        score -= 1000;
    } else if (candidate->i2cErrorCount > 0u) {
        candidate->hardReject = true;
        candidate->rejectReason = "i2c_error";
        score -= (int32_t)(500u * candidate->i2cErrorCount);
    } else if (candidate->watchdogCount > 0u) {
        candidate->hardReject = true;
        candidate->rejectReason = "watchdog_error";
        score -= (int32_t)(400u * candidate->watchdogCount);
    } else if (candidate->nonZeroRawCount < candidate->sampleCount) {
        candidate->hardReject = true;
        candidate->rejectReason = "raw_zero_or_missing";
        score -= 700;
    } else if (candidate->convertingCount == 0u) {
        candidate->hardReject = true;
        candidate->rejectReason = "not_converting";
        score -= 400;
    }

    candidate->score = score;
    return score;
}

static const char *sensorarrayS5d5CandidateStatus(const sensorarrayS5d5SweepCandidate_t *candidate, int32_t minScore)
{
    if (!candidate) {
        return "invalid";
    }
    if (candidate->hardReject) {
        return candidate->rejectReason ? candidate->rejectReason : "hard_reject";
    }
    if (candidate->sampleCount > 0u && candidate->i2cErrorCount >= candidate->sampleCount) {
        return "i2c_unstable";
    }
    if (candidate->convertingCount == 0u) {
        return "not_converting";
    }
    if (candidate->nonZeroRawCount == 0u) {
        return "raw_zero";
    }
    if (candidate->readbackValid && !candidate->clockDecodeValid) {
        return candidate->clockDecodeStatus ? candidate->clockDecodeStatus : "invalid_clock";
    }
    if (candidate->avgFreqValid && !candidate->deglitchBandwidthSufficient) {
        return "deglitch_bandwidth_low";
    }
    if (!candidate->avgFreqValid) {
        return "no_valid_avg_frequency";
    }
    if (candidate->score < minScore) {
        return "below_min_score";
    }
    return "working";
}

static bool sensorarrayS5d5CandidateGoodEnough(const sensorarrayS5d5SweepCandidate_t *candidate, int32_t minScore)
{
    return candidate &&
           !candidate->hardReject &&
           candidate->deglitchBandwidthSufficient &&
           candidate->avgFreqValid &&
           candidate->score >= minScore;
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

    if (candidate->hardReject != best->hardReject) {
        return !candidate->hardReject;
    }
    if (candidate->deglitchBandwidthSufficient != best->deglitchBandwidthSufficient) {
        return candidate->deglitchBandwidthSufficient;
    }

    int32_t scoreDelta = candidate->score - best->score;
    int32_t absScoreDelta = (scoreDelta < 0) ? -scoreDelta : scoreDelta;
    if (absScoreDelta > 50) {
        return candidate->score > best->score;
    }

    if (candidate->amplitudeCount != best->amplitudeCount) {
        return candidate->amplitudeCount < best->amplitudeCount;
    }
    if (candidate->highCurrentReq != best->highCurrentReq) {
        return !candidate->highCurrentReq;
    }
    if (candidate->rawSpan != best->rawSpan) {
        return candidate->rawSpan < best->rawSpan;
    }
    uint32_t candidateHardFaults = candidate->i2cErrorCount + candidate->watchdogCount;
    uint32_t bestHardFaults = best->i2cErrorCount + best->watchdogCount;
    if (candidateHardFaults != bestHardFaults) {
        return candidateHardFaults < bestHardFaults;
    }
    if (candidate->deglitchBandwidthHz != best->deglitchBandwidthHz) {
        return candidate->deglitchBandwidthHz < best->deglitchBandwidthHz;
    }
    if (candidate->score != best->score) {
        return candidate->score > best->score;
    }
    if (candidate->unreadCount != best->unreadCount) {
        return candidate->unreadCount > best->unreadCount;
    }
    return sensorarrayS5d5DrivePreferenceRank(candidate->driveCurrentNorm) <
           sensorarrayS5d5DrivePreferenceRank(best->driveCurrentNorm);
}

static void sensorarrayS5d5LogSweepCandidate(const char *stage,
                                             const sensorarrayS5d5SweepCandidate_t *candidate,
                                             int32_t minScore)
{
    if (!stage || !candidate) {
        return;
    }

    printf("DBGFDC_S5D5,stage=%s,source=%s,deglitchName=%s,deglitchReq=0x%X,"
           "deglitchBandwidthHz=%lu,deglitchReadbackValid=%u,deglitchReadback=0x%X,"
           "deglitchBandwidthOk=%u,deglitchMarginRatio=%.6f,avgFreqHz=%.3f,highCurrentReq=%u,"
           "highCurrentReadback=%u,driveCurrentReq=0x%04X,driveCurrentNorm=0x%04X,"
           "driveCurrentReadback=0x%04X,activeChannelReadback=%u,samples=%lu,i2cErr=%lu,"
           "convertingOk=%lu,unreadOk=%lu,watchdog=%lu,amplitude=%lu,nonZeroRaw=%lu,"
           "rawMin=%lu,rawMax=%lu,rawMean=%lu,rawSpan=%lu,statusReg=0x%04X,configReg=0x%04X,"
           "muxConfig=0x%04X,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,"
           "refClockSource=%s,effectiveFclkHz=%lu,effectiveFrefHz=%.3f,clockStatus=%s,"
           "score=%ld,hardReject=%u,rejectReason=%s,elapsedMs=%lu,status=%s\n",
           stage,
           candidate->sourceMode ? candidate->sourceMode : SENSORARRAY_NA,
           candidate->deglitchName ? candidate->deglitchName : SENSORARRAY_NA,
           (unsigned)candidate->deglitchReq,
           (unsigned long)candidate->deglitchBandwidthHz,
           candidate->deglitchReadbackValid ? 1u : 0u,
           (unsigned)candidate->deglitchReadback,
           candidate->deglitchBandwidthSufficient ? 1u : 0u,
           candidate->deglitchMarginRatio,
           candidate->avgFreqValid ? candidate->avgFreqHz : 0.0,
           candidate->highCurrentReq ? 1u : 0u,
           candidate->highCurrentReadback ? 1u : 0u,
           candidate->driveCurrentReq,
           candidate->driveCurrentNorm,
           candidate->driveCurrentReadbackNorm,
           (unsigned)candidate->activeChannelReadback,
           (unsigned long)candidate->sampleCount,
           (unsigned long)candidate->i2cErrorCount,
           (unsigned long)candidate->convertingCount,
           (unsigned long)candidate->unreadCount,
           (unsigned long)candidate->watchdogCount,
           (unsigned long)candidate->amplitudeCount,
           (unsigned long)candidate->nonZeroRawCount,
           (unsigned long)candidate->rawMin,
           (unsigned long)candidate->rawMax,
           (unsigned long)candidate->rawMean,
           (unsigned long)candidate->rawSpan,
           candidate->statusRegReadback,
           candidate->configRegReadback,
           candidate->muxConfigReadback,
           candidate->clockDividersReadback,
           (unsigned)candidate->finSelCode,
           (unsigned)candidate->finFactor,
           (unsigned)candidate->frefDivider,
           sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
           (unsigned long)candidate->effectiveFclkHz,
           candidate->effectiveFrefHz,
           candidate->clockDecodeStatus ? candidate->clockDecodeStatus : SENSORARRAY_NA,
           (long)candidate->score,
           candidate->hardReject ? 1u : 0u,
           candidate->rejectReason ? candidate->rejectReason : SENSORARRAY_NA,
           (unsigned long)candidate->elapsedMs,
           sensorarrayS5d5CandidateStatus(candidate, minScore));
}

static void sensorarrayS5d5UpdateSweepStats(sensorarrayS5d5SweepStats_t *stats,
                                            const sensorarrayS5d5SweepCandidate_t *candidate,
                                            int32_t minScore)
{
    if (!stats || !candidate) {
        return;
    }

    stats->candidatesTried++;
    if (!sensorarrayS5d5CandidateGoodEnough(candidate, minScore)) {
        return;
    }

    if (candidate->highCurrentReq) {
        stats->highCurrentWorked = true;
    } else {
        stats->normalCurrentWorked = true;
    }

    int deglitchIndex = sensorarrayS5d5DeglitchIndexFromCode(candidate->deglitchReq);
    if (deglitchIndex >= 0 && deglitchIndex < (int)(sizeof(stats->deglitchGood) / sizeof(stats->deglitchGood[0]))) {
        stats->deglitchGood[deglitchIndex] = true;
    }
}

static bool sensorarrayS5d5RunOneCandidate(sensorarrayFdcDeviceState_t *fdcState,
                                           const sensorarrayFdcDLineMap_t *fdcMap,
                                           const sensorarrayS5d5SweepPlanEntry_t *entry,
                                           uint32_t sampleCount,
                                           uint32_t settleMs,
                                           bool discardFirst,
                                           int32_t minScore,
                                           const sensorarrayCheckpointGpio_t *checkpoint,
                                           const char *candidateStage,
                                           bool verboseCandidateLog,
                                           sensorarrayS5d5SweepCandidate_t *outCandidate)
{
    if (!fdcState || !fdcState->handle || !fdcMap || !entry || !entry->deglitch || !outCandidate) {
        return false;
    }

    if (sampleCount == 0u) {
        sampleCount = 1u;
    }
    if (settleMs == 0u) {
        settleMs = 1u;
    }

    TickType_t startTick = xTaskGetTickCount();
    sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_STEP_BEGIN);

    sensorarrayS5d5SweepCandidate_t candidate = {0};
    sensorarrayS5d5CandidateInit(&candidate,
                                 entry->deglitch,
                                 entry->highCurrentReq,
                                 entry->driveCurrentReq,
                                 entry->sourceMode);

    esp_err_t cfgErr = sensorarrayApplyS5d5DeglitchStep(fdcState->handle, candidate.deglitchReq);
    if (cfgErr == ESP_OK) {
        cfgErr = sensorarrayApplyS5d5DriveStep(fdcState->handle,
                                               candidate.highCurrentReq,
                                               candidate.driveCurrentReq);
    }

    if (cfgErr != ESP_OK) {
        candidate.i2cErrorCount++;
        candidate.sampleCount = 1u;
        candidate.rawMin = 0u;
        candidate.rawMax = 0u;
        candidate.score = INT_MIN / 4;
        candidate.hardReject = true;
        candidate.rejectReason = "config_write_error";
        candidate.elapsedMs = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - startTick);
        *outCandidate = candidate;
        if (verboseCandidateLog) {
            sensorarrayS5d5LogSweepCandidate(candidateStage, outCandidate, minScore);
        }
        return true;
    }

    sensorarrayDebugSelftestDelayMs(settleMs);

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
        } else {
            sensorarrayS5d5CandidateIngestDiag(&candidate, &diag, fdcMap->channel);
        }
        sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
    }

    uint16_t statusReg = 0u;
    uint16_t configReg = 0u;
    uint16_t muxConfig = 0u;
    uint16_t clockDividers = 0u;
    uint16_t driveReg = 0u;
    esp_err_t regsErr = sensorarrayS5d5ReadKeyRegs(fdcState->handle,
                                                    fdcMap->channel,
                                                    &statusReg,
                                                    &configReg,
                                                    &muxConfig,
                                                    &clockDividers,
                                                    &driveReg);
    if (regsErr == ESP_OK) {
        candidate.readbackValid = true;
        candidate.statusRegReadback = statusReg;
        candidate.configRegReadback = configReg;
        candidate.muxConfigReadback = muxConfig;
        candidate.deglitchReadbackValid = true;
        candidate.deglitchReadback = (uint8_t)(muxConfig & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK);
        sensorarrayS5d5CandidateSetClockReadback(&candidate, clockDividers);
        candidate.highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
        candidate.driveCurrentReadbackNorm = (uint16_t)(driveReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
        candidate.activeChannelReadback =
            (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >>
                      SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
    } else {
        candidate.i2cErrorCount++;
    }

    (void)sensorarrayS5d5FinalizeCandidate(&candidate);
    candidate.elapsedMs = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount() - startTick);
    *outCandidate = candidate;
    if (verboseCandidateLog) {
        sensorarrayS5d5LogSweepCandidate(candidateStage, outCandidate, minScore);
    }
    return true;
}

static bool sensorarrayS5d5RunCandidatePlan(sensorarrayFdcDeviceState_t *fdcState,
                                            const sensorarrayFdcDLineMap_t *fdcMap,
                                            const sensorarrayS5d5SweepPlan_t *plan,
                                            uint32_t sampleCount,
                                            uint32_t settleMs,
                                            bool discardFirst,
                                            int32_t minScore,
                                            const sensorarrayCheckpointGpio_t *checkpoint,
                                            const char *candidateStage,
                                            const char *selectedStage,
                                            bool verboseCandidateLog,
                                            bool earlyAccept,
                                            sensorarrayS5d5SweepStats_t *outStats,
                                            sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    if (!fdcState || !fdcState->handle || !fdcMap || !plan || !outBestCandidate) {
        return false;
    }

    sensorarrayS5d5SweepStats_t localStats = {0};
    bool haveBest = false;
    sensorarrayS5d5SweepCandidate_t bestCandidate = {0};

    for (size_t i = 0u; i < plan->count; ++i) {
        sensorarrayS5d5SweepCandidate_t candidate = {0};
        if (!sensorarrayS5d5RunOneCandidate(fdcState,
                                            fdcMap,
                                            &plan->entries[i],
                                            sampleCount,
                                            settleMs,
                                            discardFirst,
                                            minScore,
                                            checkpoint,
                                            candidateStage,
                                            verboseCandidateLog,
                                            &candidate)) {
            continue;
        }

        sensorarrayS5d5UpdateSweepStats(&localStats, &candidate, minScore);
        if (!haveBest || sensorarrayS5d5CandidateBetter(&candidate, &bestCandidate)) {
            bestCandidate = candidate;
            haveBest = true;
        }
        if (earlyAccept && sensorarrayS5d5CandidateGoodEnough(&candidate, minScore)) {
            bestCandidate = candidate;
            haveBest = true;
            break;
        }
    }

    if (outStats) {
        *outStats = localStats;
    }
    if (!haveBest) {
        return false;
    }

    bestCandidate.selected = true;
    *outBestCandidate = bestCandidate;
    sensorarrayS5d5LogSweepCandidate(selectedStage, outBestCandidate, minScore);
    return sensorarrayS5d5CandidateGoodEnough(outBestCandidate, minScore);
}

static void sensorarrayS5d5BuildBootFullSweepPlan(sensorarrayS5d5SweepPlan_t *plan)
{
    if (!plan) {
        return;
    }

    *plan = (sensorarrayS5d5SweepPlan_t){0};
    size_t deglitchCount = sensorarrayS5d5DeglitchTableCount();
    size_t highCurrentCount = sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE) /
                              sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[0]);
    size_t driveCount = sensorarrayS5d5DriveTableCount();
    for (size_t deglitchIndex = 0u; deglitchIndex < deglitchCount; ++deglitchIndex) {
        for (size_t highIndex = 0u; highIndex < highCurrentCount; ++highIndex) {
            for (size_t driveIndex = 0u; driveIndex < driveCount; ++driveIndex) {
                (void)sensorarrayS5d5PlanAppend(plan,
                                                &SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[deglitchIndex],
                                                SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[highIndex],
                                                SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[driveIndex],
                                                "boot_full");
            }
        }
    }
}

static void sensorarrayS5d5BuildRuntimeFullFallbackPlan(sensorarrayS5d5SweepPlan_t *plan)
{
    if (!plan) {
        return;
    }

    static const uint16_t driveOrder[] = {
        0xB800u,
        0xC000u,
        0xD000u,
        0xA000u,
        0xE000u,
        0xF800u,
    };
    static const bool highOrder[] = {
        false,
        true,
    };

    *plan = (sensorarrayS5d5SweepPlan_t){0};
    size_t deglitchCount = sensorarrayS5d5DeglitchTableCount();
    for (size_t highIndex = 0u; highIndex < sizeof(highOrder) / sizeof(highOrder[0]); ++highIndex) {
        for (size_t deglitchIndex = 0u; deglitchIndex < deglitchCount; ++deglitchIndex) {
            for (size_t driveIndex = 0u; driveIndex < sizeof(driveOrder) / sizeof(driveOrder[0]); ++driveIndex) {
                (void)sensorarrayS5d5PlanAppend(plan,
                                                &SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[deglitchIndex],
                                                highOrder[highIndex],
                                                driveOrder[driveIndex],
                                                "runtime_full_fallback");
            }
        }
    }
}

static void sensorarrayS5d5UpdateCurrentProfileFromCandidate(const sensorarrayS5d5SweepCandidate_t *candidate)
{
    if (!candidate || !sensorarrayS5d5CandidateGoodEnough(candidate,
                                                          (int32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOCK_REQUIRE_MIN_SCORE)) {
        return;
    }

    uint32_t profileUseCount = s_s5d5FdcProfile.profileUseCount;
    uint32_t profileFailCount = s_s5d5FdcProfile.profileFailCount;
    uint32_t stableCount = s_s5d5FdcProfile.stableCount + 1u;
    s_s5d5FdcProfile = (sensorarrayS5d5FdcProfile_t){
        .valid = true,
        .deglitchReq = candidate->deglitchReq,
        .deglitchBandwidthHz = candidate->deglitchBandwidthHz,
        .deglitchName = candidate->deglitchName,
        .highCurrentReq = candidate->highCurrentReq,
        .driveCurrentReq = candidate->driveCurrentReq,
        .driveCurrentNorm = candidate->driveCurrentNorm,
        .baselineFreqHz = candidate->avgFreqValid ? candidate->avgFreqHz : 0.0,
        .baselineRawMean = candidate->rawMean,
        .baselineRawSpan = candidate->rawSpan,
        .stableCount = stableCount,
        .abnormalCount = 0u,
        .profileUseCount = profileUseCount,
        .profileFailCount = profileFailCount,
    };
}

static void sensorarrayS5d5UpdateBootEnvProfileFromCandidate(const sensorarrayS5d5SweepCandidate_t *candidate,
                                                             const sensorarrayS5d5SweepStats_t *stats)
{
    if (!candidate || !sensorarrayS5d5CandidateGoodEnough(candidate,
                                                          (int32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOCK_REQUIRE_MIN_SCORE)) {
        s_s5d5BootEnvProfile.valid = false;
        return;
    }

    s_s5d5BootEnvProfile.valid = true;
    s_s5d5BootEnvProfile.bestDeglitchReq = candidate->deglitchReq;
    s_s5d5BootEnvProfile.bestDeglitchBandwidthHz = candidate->deglitchBandwidthHz;
    s_s5d5BootEnvProfile.bestDeglitchName = candidate->deglitchName;
    s_s5d5BootEnvProfile.bestHighCurrentReq = candidate->highCurrentReq;
    s_s5d5BootEnvProfile.bestDriveCurrentReq = candidate->driveCurrentReq;
    s_s5d5BootEnvProfile.bestDriveCurrentNorm = candidate->driveCurrentNorm;
    s_s5d5BootEnvProfile.bestFreqHz = candidate->avgFreqValid ? candidate->avgFreqHz : 0.0;
    s_s5d5BootEnvProfile.bestRawMean = candidate->rawMean;
    s_s5d5BootEnvProfile.bestRawSpan = candidate->rawSpan;
    s_s5d5BootEnvProfile.bestScore = candidate->score;

    if (stats) {
        s_s5d5BootEnvProfile.normalCurrentWorked = stats->normalCurrentWorked;
        s_s5d5BootEnvProfile.highCurrentNeeded = !stats->normalCurrentWorked && candidate->highCurrentReq;
        s_s5d5BootEnvProfile.deglitch1MHzBad = !stats->deglitchGood[0];
        s_s5d5BootEnvProfile.deglitch3p3MHzGood = stats->deglitchGood[1];
        s_s5d5BootEnvProfile.deglitch10MHzGood = stats->deglitchGood[2];
        s_s5d5BootEnvProfile.deglitch33MHzNeeded =
            candidate->deglitchReq == 0x7u ||
            (!stats->deglitchGood[1] && !stats->deglitchGood[2] && stats->deglitchGood[3]);
    } else {
        s_s5d5BootEnvProfile.normalCurrentWorked = !candidate->highCurrentReq;
        s_s5d5BootEnvProfile.highCurrentNeeded = candidate->highCurrentReq;
        s_s5d5BootEnvProfile.deglitch1MHzBad = candidate->deglitchReq != 0x1u;
        s_s5d5BootEnvProfile.deglitch3p3MHzGood = candidate->deglitchReq == 0x4u;
        s_s5d5BootEnvProfile.deglitch10MHzGood = candidate->deglitchReq == 0x5u;
        s_s5d5BootEnvProfile.deglitch33MHzNeeded = candidate->deglitchReq == 0x7u;
    }

    printf("DBGFDC_S5D5,stage=boot_env_profile_update,valid=%u,bestDeglitchName=%s,bestDeglitchReq=0x%X,"
           "bestHighCurrentReq=%u,bestDriveCurrent=0x%04X,bestFreqHz=%.3f,bestRawMean=%lu,bestRawSpan=%lu,"
           "bestScore=%ld,normalCurrentWorked=%u,highCurrentNeeded=%u,deglitch1MHzBad=%u,"
           "deglitch3p3MHzGood=%u,deglitch10MHzGood=%u,deglitch33MHzNeeded=%u\n",
           s_s5d5BootEnvProfile.valid ? 1u : 0u,
           s_s5d5BootEnvProfile.bestDeglitchName ? s_s5d5BootEnvProfile.bestDeglitchName : SENSORARRAY_NA,
           (unsigned)s_s5d5BootEnvProfile.bestDeglitchReq,
           s_s5d5BootEnvProfile.bestHighCurrentReq ? 1u : 0u,
           s_s5d5BootEnvProfile.bestDriveCurrentNorm,
           s_s5d5BootEnvProfile.bestFreqHz,
           (unsigned long)s_s5d5BootEnvProfile.bestRawMean,
           (unsigned long)s_s5d5BootEnvProfile.bestRawSpan,
           (long)s_s5d5BootEnvProfile.bestScore,
           s_s5d5BootEnvProfile.normalCurrentWorked ? 1u : 0u,
           s_s5d5BootEnvProfile.highCurrentNeeded ? 1u : 0u,
           s_s5d5BootEnvProfile.deglitch1MHzBad ? 1u : 0u,
           s_s5d5BootEnvProfile.deglitch3p3MHzGood ? 1u : 0u,
           s_s5d5BootEnvProfile.deglitch10MHzGood ? 1u : 0u,
           s_s5d5BootEnvProfile.deglitch33MHzNeeded ? 1u : 0u);
}

static bool sensorarrayS5d5RunBootFullSweep(sensorarrayFdcDeviceState_t *fdcState,
                                            const sensorarrayFdcDLineMap_t *fdcMap,
                                            bool discardFirst,
                                            int32_t minScore,
                                            const sensorarrayCheckpointGpio_t *checkpoint,
                                            sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    if (!fdcState || !fdcMap || !outBestCandidate) {
        return false;
    }
    if (s_s5d5BootEnvProfile.bootFullSweepDone) {
        return false;
    }

    sensorarrayS5d5SweepPlan_t plan = {0};
    sensorarrayS5d5BuildBootFullSweepPlan(&plan);
    uint32_t sampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_BOOT_FULL_SWEEP_SAMPLE_COUNT;
    if (sampleCount == 0u) {
        sampleCount = 1u;
    }

    printf("DBGFDC_S5D5,stage=boot_full_sweep_begin,candidates=%u,samplesPerCandidate=%lu,settleMs=%u\n",
           (unsigned)plan.count,
           (unsigned long)sampleCount,
           (unsigned)SENSORARRAY_S5D5_STEP_SETTLE_MS);

    sensorarrayS5d5SweepStats_t stats = {0};
    bool ok = sensorarrayS5d5RunCandidatePlan(fdcState,
                                              fdcMap,
                                              &plan,
                                              sampleCount,
                                              SENSORARRAY_S5D5_STEP_SETTLE_MS,
                                              discardFirst,
                                              minScore,
                                              checkpoint,
                                              "boot_full_sweep_candidate",
                                              "boot_full_sweep_selected",
                                              true,
                                              false,
                                              &stats,
                                              outBestCandidate);

    s_s5d5BootEnvProfile.bootFullSweepDone = true;
    s_s5d5BootEnvProfile.fullSweepCount++;
    if (ok) {
        sensorarrayS5d5UpdateBootEnvProfileFromCandidate(outBestCandidate, &stats);
        sensorarrayS5d5UpdateCurrentProfileFromCandidate(outBestCandidate);
    }

    printf("DBGFDC_S5D5,stage=boot_full_sweep_end,status=%s,bestScore=%ld,minScore=%ld,"
           "bootFullSweepDone=%u,normalCurrentWorked=%u,highCurrentNeeded=%u\n",
           ok ? "ok" : "failed",
           ok ? (long)outBestCandidate->score : (long)INT_MIN,
           (long)minScore,
           s_s5d5BootEnvProfile.bootFullSweepDone ? 1u : 0u,
           s_s5d5BootEnvProfile.normalCurrentWorked ? 1u : 0u,
           s_s5d5BootEnvProfile.highCurrentNeeded ? 1u : 0u);
    return ok;
}

static void sensorarrayS5d5BuildFastSweepPlanFromBootEnv(sensorarrayS5d5SweepPlan_t *plan)
{
    if (!plan) {
        return;
    }
    *plan = (sensorarrayS5d5SweepPlan_t){0};
    if (!s_s5d5BootEnvProfile.valid) {
        return;
    }

    const sensorarrayS5d5DeglitchCandidate_t *bestDeglitch =
        sensorarrayS5d5FindDeglitchByCode(s_s5d5BootEnvProfile.bestDeglitchReq);
    if (!bestDeglitch) {
        bestDeglitch = sensorarrayS5d5FindDeglitchByCode(0x4u);
    }
    if (!bestDeglitch) {
        return;
    }

    bool useHighCurrent = (!s_s5d5BootEnvProfile.normalCurrentWorked && s_s5d5BootEnvProfile.highCurrentNeeded);
    uint16_t bestDrive = s_s5d5BootEnvProfile.bestDriveCurrentNorm ?
                             s_s5d5BootEnvProfile.bestDriveCurrentNorm :
                             0xC000u;
    int bestDriveIndex = sensorarrayS5d5DriveIndexFromNorm(bestDrive);

    (void)sensorarrayS5d5PlanAppend(plan, bestDeglitch, useHighCurrent, bestDrive, "fast_boot_env_best");
    if (bestDriveIndex > 0 && plan->count < SENSORARRAY_S5D5_FAST_SWEEP_PLAN_MAX) {
        (void)sensorarrayS5d5PlanAppend(plan,
                                        bestDeglitch,
                                        useHighCurrent,
                                        SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[(size_t)bestDriveIndex - 1u],
                                        "fast_adjacent_drive");
    }
    if (bestDriveIndex >= 0 &&
        (size_t)bestDriveIndex + 1u < sensorarrayS5d5DriveTableCount() &&
        plan->count < SENSORARRAY_S5D5_FAST_SWEEP_PLAN_MAX) {
        (void)sensorarrayS5d5PlanAppend(plan,
                                        bestDeglitch,
                                        useHighCurrent,
                                        SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[(size_t)bestDriveIndex + 1u],
                                        "fast_adjacent_drive");
    }

    static const uint16_t preferredDriveFill[] = {
        0xB800u,
        0xC000u,
        0xD000u,
        0xA000u,
    };
    for (size_t i = 0u; i < sizeof(preferredDriveFill) / sizeof(preferredDriveFill[0]); ++i) {
        if (plan->count >= 4u || plan->count >= SENSORARRAY_S5D5_FAST_SWEEP_PLAN_MAX) {
            break;
        }
        (void)sensorarrayS5d5PlanAppend(plan,
                                        bestDeglitch,
                                        useHighCurrent,
                                        preferredDriveFill[i],
                                        "fast_preferred_drive_fill");
    }

    const sensorarrayS5d5DeglitchCandidate_t *adjacentDeglitch = NULL;
    switch (bestDeglitch->code) {
    case 0x4u:
        adjacentDeglitch = sensorarrayS5d5FindDeglitchByCode(0x5u);
        break;
    case 0x5u:
        adjacentDeglitch = sensorarrayS5d5FindDeglitchByCode(0x4u);
        break;
    case 0x1u:
        adjacentDeglitch = sensorarrayS5d5FindDeglitchByCode(0x4u);
        break;
    case 0x7u:
        adjacentDeglitch = sensorarrayS5d5FindDeglitchByCode(0x5u);
        break;
    default:
        adjacentDeglitch = sensorarrayS5d5FindDeglitchByCode(0x5u);
        break;
    }

    if (adjacentDeglitch && plan->count < SENSORARRAY_S5D5_FAST_SWEEP_PLAN_MAX) {
        (void)sensorarrayS5d5PlanAppend(plan, adjacentDeglitch, useHighCurrent, bestDrive, "fast_adjacent_deglitch");
    }
    if (adjacentDeglitch && bestDriveIndex >= 0 && plan->count < SENSORARRAY_S5D5_FAST_SWEEP_PLAN_MAX) {
        uint16_t nextDrive =
            ((size_t)bestDriveIndex + 1u < sensorarrayS5d5DriveTableCount())
                ? SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[(size_t)bestDriveIndex + 1u]
                : SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[(size_t)bestDriveIndex];
        (void)sensorarrayS5d5PlanAppend(plan, adjacentDeglitch, useHighCurrent, nextDrive, "fast_adjacent_pair");
    }
}

static bool sensorarrayS5d5RunFastSweepFromBootEnv(sensorarrayFdcDeviceState_t *fdcState,
                                                   const sensorarrayFdcDLineMap_t *fdcMap,
                                                   bool discardFirst,
                                                   int32_t minScore,
                                                   const sensorarrayCheckpointGpio_t *checkpoint,
                                                   sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    sensorarrayS5d5SweepPlan_t plan = {0};
    sensorarrayS5d5BuildFastSweepPlanFromBootEnv(&plan);
    printf("DBGFDC_S5D5,stage=fast_sweep_begin,source=boot_env,candidates=%u,bootValid=%u,"
           "normalCurrentWorked=%u,highCurrentNeeded=%u\n",
           (unsigned)plan.count,
           s_s5d5BootEnvProfile.valid ? 1u : 0u,
           s_s5d5BootEnvProfile.normalCurrentWorked ? 1u : 0u,
           s_s5d5BootEnvProfile.highCurrentNeeded ? 1u : 0u);
    if (plan.count == 0u) {
        printf("DBGFDC_S5D5,stage=fast_sweep_selected,source=boot_env,status=failed,reason=no_plan\n");
        return false;
    }

    uint32_t sampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FAST_SWEEP_SAMPLE_COUNT;
    uint32_t settleMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FAST_SWEEP_SETTLE_MS;
    s_s5d5BootEnvProfile.fastSweepCount++;
    bool ok = sensorarrayS5d5RunCandidatePlan(fdcState,
                                              fdcMap,
                                              &plan,
                                              sampleCount,
                                              settleMs,
                                              discardFirst,
                                              minScore,
                                              checkpoint,
                                              "fast_sweep_candidate",
                                              "fast_sweep_selected",
                                              true,
                                              true,
                                              NULL,
                                              outBestCandidate);
    if (!ok) {
        printf("DBGFDC_S5D5,stage=fast_sweep_selected,source=boot_env,status=failed,bestScore=%ld,minScore=%ld\n",
               outBestCandidate ? (long)outBestCandidate->score : (long)INT_MIN,
               (long)minScore);
    }
    return ok;
}

static void sensorarrayS5d5BuildMediumSweepPlanFromBootEnv(sensorarrayS5d5SweepPlan_t *plan)
{
    if (!plan) {
        return;
    }
    *plan = (sensorarrayS5d5SweepPlan_t){0};
    if (!s_s5d5BootEnvProfile.valid) {
        return;
    }

    bool useHighCurrent = (!s_s5d5BootEnvProfile.normalCurrentWorked && s_s5d5BootEnvProfile.highCurrentNeeded);
    uint8_t deglitchCodes[] = {
        s_s5d5BootEnvProfile.bestDeglitchReq,
        0x4u,
        0x5u,
    };
    static const uint16_t driveList[] = {
        0xA000u,
        0xB800u,
        0xC000u,
        0xD000u,
        0xE000u,
    };

    for (size_t d = 0u; d < sizeof(deglitchCodes) / sizeof(deglitchCodes[0]); ++d) {
        const sensorarrayS5d5DeglitchCandidate_t *deglitch = sensorarrayS5d5FindDeglitchByCode(deglitchCodes[d]);
        if (!deglitch) {
            continue;
        }
        for (size_t i = 0u; i < sizeof(driveList) / sizeof(driveList[0]); ++i) {
            if (plan->count >= SENSORARRAY_S5D5_MEDIUM_SWEEP_PLAN_MAX) {
                return;
            }
            (void)sensorarrayS5d5PlanAppend(plan, deglitch, useHighCurrent, driveList[i], "medium_boot_env");
        }
    }
}

static bool sensorarrayS5d5RunMediumSweepFromBootEnv(sensorarrayFdcDeviceState_t *fdcState,
                                                     const sensorarrayFdcDLineMap_t *fdcMap,
                                                     bool discardFirst,
                                                     int32_t minScore,
                                                     const sensorarrayCheckpointGpio_t *checkpoint,
                                                     sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    sensorarrayS5d5SweepPlan_t plan = {0};
    sensorarrayS5d5BuildMediumSweepPlanFromBootEnv(&plan);
    printf("DBGFDC_S5D5,stage=medium_sweep_begin,source=boot_env,candidates=%u,bootValid=%u,"
           "normalCurrentWorked=%u,highCurrentNeeded=%u\n",
           (unsigned)plan.count,
           s_s5d5BootEnvProfile.valid ? 1u : 0u,
           s_s5d5BootEnvProfile.normalCurrentWorked ? 1u : 0u,
           s_s5d5BootEnvProfile.highCurrentNeeded ? 1u : 0u);
    if (plan.count == 0u) {
        printf("DBGFDC_S5D5,stage=medium_sweep_selected,source=boot_env,status=failed,reason=no_plan\n");
        return false;
    }

    uint32_t sampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_MEDIUM_SWEEP_SAMPLE_COUNT;
    uint32_t settleMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_MEDIUM_SWEEP_SETTLE_MS;
    s_s5d5BootEnvProfile.mediumSweepCount++;
    bool ok = sensorarrayS5d5RunCandidatePlan(fdcState,
                                              fdcMap,
                                              &plan,
                                              sampleCount,
                                              settleMs,
                                              discardFirst,
                                              minScore,
                                              checkpoint,
                                              "medium_sweep_candidate",
                                              "medium_sweep_selected",
                                              true,
                                              true,
                                              NULL,
                                              outBestCandidate);
    if (!ok) {
        printf("DBGFDC_S5D5,stage=medium_sweep_selected,source=boot_env,status=failed,bestScore=%ld,minScore=%ld\n",
               outBestCandidate ? (long)outBestCandidate->score : (long)INT_MIN,
               (long)minScore);
    }
    return ok;
}

static bool sensorarrayS5d5RunFullSweepFallback(sensorarrayFdcDeviceState_t *fdcState,
                                                const sensorarrayFdcDLineMap_t *fdcMap,
                                                bool discardFirst,
                                                int32_t minScore,
                                                const sensorarrayCheckpointGpio_t *checkpoint,
                                                sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    sensorarrayS5d5SweepPlan_t plan = {0};
    sensorarrayS5d5BuildRuntimeFullFallbackPlan(&plan);
    uint32_t sampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_SWEEP_SAMPLE_COUNT;
    if (sampleCount == 0u) {
        sampleCount = 1u;
    }
    bool verbose = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RUNTIME_FULL_SWEEP_VERBOSE_LOG != 0);
    printf("DBGFDC_S5D5,stage=full_sweep_fallback_begin,candidates=%u,samplesPerCandidate=%lu,"
           "verbose=%u,highCurrentOrder=normal_first,status=begin\n",
           (unsigned)plan.count,
           (unsigned long)sampleCount,
           verbose ? 1u : 0u);

    sensorarrayS5d5SweepStats_t stats = {0};
    s_s5d5BootEnvProfile.fullSweepFallbackCount++;
    bool ok = sensorarrayS5d5RunCandidatePlan(fdcState,
                                              fdcMap,
                                              &plan,
                                              sampleCount,
                                              SENSORARRAY_S5D5_STEP_SETTLE_MS,
                                              discardFirst,
                                              minScore,
                                              checkpoint,
                                              "full_sweep_fallback_candidate",
                                              "full_sweep_fallback_selected",
                                              verbose,
                                              true,
                                              &stats,
                                              outBestCandidate);
    if (ok) {
        sensorarrayS5d5UpdateBootEnvProfileFromCandidate(outBestCandidate, &stats);
        sensorarrayS5d5UpdateCurrentProfileFromCandidate(outBestCandidate);
        s_s5d5RuntimeFastFailStreak = 0u;
    }
    printf("DBGFDC_S5D5,stage=full_sweep_fallback_end,status=%s,bestScore=%ld,minScore=%ld\n",
           ok ? "ok" : "failed",
           ok ? (long)outBestCandidate->score : (long)INT_MIN,
           (long)minScore);
    return ok;
}

static bool sensorarrayS5d5TryCachedProfile(sensorarrayFdcDeviceState_t *fdcState,
                                            const sensorarrayFdcDLineMap_t *fdcMap,
                                            bool discardFirst,
                                            int32_t minScore,
                                            const sensorarrayCheckpointGpio_t *checkpoint,
                                            sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    printf("DBGFDC_S5D5,stage=profile_retry_begin,valid=%u,useCount=%lu,failCount=%lu\n",
           s_s5d5FdcProfile.valid ? 1u : 0u,
           (unsigned long)s_s5d5FdcProfile.profileUseCount,
           (unsigned long)s_s5d5FdcProfile.profileFailCount);
    if (!s_s5d5FdcProfile.valid) {
        printf("DBGFDC_S5D5,stage=profile_retry_result,status=failed,reason=no_cached_profile\n");
        return false;
    }

    const sensorarrayS5d5DeglitchCandidate_t *deglitch =
        sensorarrayS5d5FindDeglitchByCode(s_s5d5FdcProfile.deglitchReq);
    if (!deglitch) {
        s_s5d5FdcProfile.profileFailCount++;
        printf("DBGFDC_S5D5,stage=profile_retry_result,status=failed,reason=bad_cached_deglitch\n");
        return false;
    }

    sensorarrayS5d5SweepPlan_t plan = {0};
    (void)sensorarrayS5d5PlanAppend(&plan,
                                    deglitch,
                                    s_s5d5FdcProfile.highCurrentReq,
                                    s_s5d5FdcProfile.driveCurrentNorm,
                                    "cached_working_profile");
    s_s5d5FdcProfile.profileUseCount++;
    bool ok = sensorarrayS5d5RunCandidatePlan(fdcState,
                                              fdcMap,
                                              &plan,
                                              (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_PROFILE_TRY_SAMPLE_COUNT,
                                              (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_PROFILE_SETTLE_MS,
                                              discardFirst,
                                              minScore,
                                              checkpoint,
                                              "profile_retry_candidate",
                                              "profile_retry_selected",
                                              true,
                                              true,
                                              NULL,
                                              outBestCandidate);
    if (!ok) {
        s_s5d5FdcProfile.profileFailCount++;
        printf("DBGFDC_S5D5,stage=profile_retry_result,status=failed,bestScore=%ld,minScore=%ld\n",
               outBestCandidate ? (long)outBestCandidate->score : (long)INT_MIN,
               (long)minScore);
    } else {
        printf("DBGFDC_S5D5,stage=profile_retry_result,status=ok,score=%ld\n",
               (long)outBestCandidate->score);
    }
    return ok;
}

static bool sensorarrayS5d5RecoverAfterRuntimeFault(sensorarrayFdcDeviceState_t *fdcState,
                                                    const sensorarrayFdcDLineMap_t *fdcMap,
                                                    bool discardFirst,
                                                    int32_t minScore,
                                                    const sensorarrayCheckpointGpio_t *checkpoint,
                                                    const char *reason,
                                                    sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    printf("DBGFDC_S5D5,stage=runtime_recovery_begin,reason=%s,profileValid=%u,bootProfileValid=%u,"
           "fastFailStreak=%lu\n",
           reason ? reason : SENSORARRAY_NA,
           s_s5d5FdcProfile.valid ? 1u : 0u,
           s_s5d5BootEnvProfile.valid ? 1u : 0u,
           (unsigned long)s_s5d5RuntimeFastFailStreak);

    if (sensorarrayS5d5TryCachedProfile(fdcState, fdcMap, discardFirst, minScore, checkpoint, outBestCandidate)) {
        s_s5d5RuntimeFastFailStreak = 0u;
        printf("DBGFDC_S5D5,stage=runtime_recovery_end,status=ok,method=cached_profile\n");
        return true;
    }

    bool fastTried = false;
    if (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RUNTIME_FAST_SWEEP_FIRST != 0) {
        fastTried = true;
        if (sensorarrayS5d5RunFastSweepFromBootEnv(fdcState,
                                                   fdcMap,
                                                   discardFirst,
                                                   minScore,
                                                   checkpoint,
                                                   outBestCandidate)) {
            s_s5d5RuntimeFastFailStreak = 0u;
            printf("DBGFDC_S5D5,stage=runtime_recovery_end,status=ok,method=fast_sweep\n");
            return true;
        }
        s_s5d5RuntimeFastFailStreak++;
    }

    if (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_MEDIUM_SWEEP_ENABLE != 0) {
        if (sensorarrayS5d5RunMediumSweepFromBootEnv(fdcState,
                                                     fdcMap,
                                                     discardFirst,
                                                     minScore,
                                                     checkpoint,
                                                     outBestCandidate)) {
            s_s5d5RuntimeFastFailStreak = 0u;
            printf("DBGFDC_S5D5,stage=runtime_recovery_end,status=ok,method=medium_sweep\n");
            return true;
        }
    }

    uint32_t fullFallbackAfterFastFails =
        (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FULL_FALLBACK_AFTER_FAST_FAILS;
    if (fullFallbackAfterFastFails == 0u) {
        fullFallbackAfterFastFails = 1u;
    }
    if (!fastTried) {
        s_s5d5RuntimeFastFailStreak = fullFallbackAfterFastFails;
    }
    if (s_s5d5RuntimeFastFailStreak < fullFallbackAfterFastFails) {
        printf("DBGFDC_S5D5,stage=runtime_recovery_end,status=failed,method=defer_full_fallback,"
               "fastFailStreak=%lu,required=%lu\n",
               (unsigned long)s_s5d5RuntimeFastFailStreak,
               (unsigned long)fullFallbackAfterFastFails);
        return false;
    }

    if (sensorarrayS5d5RunFullSweepFallback(fdcState,
                                            fdcMap,
                                            discardFirst,
                                            minScore,
                                            checkpoint,
                                            outBestCandidate)) {
        printf("DBGFDC_S5D5,stage=runtime_recovery_end,status=ok,method=full_sweep_fallback\n");
        return true;
    }

    printf("DBGFDC_S5D5,stage=runtime_recovery_end,status=failed,method=all_failed\n");
    return false;
}

static esp_err_t sensorarrayS5d5ApplyCandidateAndDiscard(sensorarrayFdcDeviceState_t *fdcState,
                                                         const sensorarrayS5d5SweepCandidate_t *candidate,
                                                         Fdc2214CapChannel_t channel,
                                                         uint32_t discardCount,
                                                         const char *applyStage)
{
    if (!fdcState || !fdcState->handle || !candidate) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *stage = applyStage ? applyStage : "lock_apply";
    esp_err_t err = sensorarrayApplyS5d5DeglitchStep(fdcState->handle, candidate->deglitchReq);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=%s,deglitchName=%s,deglitchReq=0x%X,deglitchBandwidthHz=%lu,"
               "highCurrentReq=%u,driveCurrentReq=0x%04X,err=%ld,status=deglitch_apply_failed\n",
               stage,
               candidate->deglitchName ? candidate->deglitchName : SENSORARRAY_NA,
               (unsigned)candidate->deglitchReq,
               (unsigned long)candidate->deglitchBandwidthHz,
               candidate->highCurrentReq ? 1u : 0u,
               candidate->driveCurrentReq,
               (long)err);
        return err;
    }

    sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SETTLE_MS);

    err = sensorarrayApplyS5d5DriveStep(fdcState->handle, candidate->highCurrentReq, candidate->driveCurrentReq);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=%s,deglitchName=%s,deglitchReq=0x%X,deglitchBandwidthHz=%lu,"
               "highCurrentReq=%u,driveCurrentReq=0x%04X,err=%ld,status=drive_apply_failed\n",
               stage,
               candidate->deglitchName ? candidate->deglitchName : SENSORARRAY_NA,
               (unsigned)candidate->deglitchReq,
               (unsigned long)candidate->deglitchBandwidthHz,
               candidate->highCurrentReq ? 1u : 0u,
               candidate->driveCurrentReq,
               (long)err);
        return err;
    }

    sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SETTLE_MS);

    uint16_t statusReg = 0u;
    uint16_t configReg = 0u;
    uint16_t muxConfig = 0u;
    uint16_t clockDividers = 0u;
    uint16_t driveReg = 0u;
    err = sensorarrayS5d5ReadKeyRegs(fdcState->handle, channel, &statusReg, &configReg, &muxConfig, &clockDividers, &driveReg);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=%s,deglitchName=%s,deglitchReq=0x%X,deglitchBandwidthHz=%lu,"
               "highCurrentReq=%u,driveCurrentReq=0x%04X,err=%ld,status=readback_failed\n",
               stage,
               candidate->deglitchName ? candidate->deglitchName : SENSORARRAY_NA,
               (unsigned)candidate->deglitchReq,
               (unsigned long)candidate->deglitchBandwidthHz,
               candidate->highCurrentReq ? 1u : 0u,
               candidate->driveCurrentReq,
               (long)err);
        return err;
    }

    bool highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    uint16_t driveCurrentReadbackNorm = (uint16_t)(driveReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    uint8_t deglitchReadback = (uint8_t)(muxConfig & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK);
    uint8_t activeChannelReadback =
        (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >> SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *clockStatus = "unknown";
    bool clockOk = sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                            &finSelCode,
                                                            &finFactor,
                                                            &frefDivider,
                                                            &clockStatus);
    double effectiveFrefHz =
        (clockOk && frefDivider > 0u) ? ((double)sensorarrayMeasureFdcEffectiveFclkHz() / (double)frefDivider) : 0.0;
    bool match = (deglitchReadback == (uint8_t)(candidate->deglitchReq & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK)) &&
                 (highCurrentReadback == candidate->highCurrentReq) &&
                 (driveCurrentReadbackNorm == candidate->driveCurrentNorm) &&
                 (activeChannelReadback == (uint8_t)channel);

    printf("DBGFDC_S5D5,stage=%s,deglitchName=%s,deglitchReq=0x%X,deglitchReadback=0x%X,"
           "deglitchBandwidthHz=%lu,deglitchMarginRatio=%.6f,highCurrentReq=%u,highCurrentReadback=%u,"
           "driveCurrentReq=0x%04X,driveCurrentNorm=0x%04X,driveCurrentReadback=0x%04X,"
           "activeChannelReadback=%u,statusReg=0x%04X,configReg=0x%04X,muxConfig=0x%04X,clockDiv=0x%04X,"
           "finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,effectiveFclkHz=%lu,"
           "effectiveFrefHz=%.3f,clockStatus=%s,status=%s\n",
           stage,
           candidate->deglitchName ? candidate->deglitchName : SENSORARRAY_NA,
           (unsigned)candidate->deglitchReq,
           (unsigned)deglitchReadback,
           (unsigned long)candidate->deglitchBandwidthHz,
           candidate->deglitchMarginRatio,
           candidate->highCurrentReq ? 1u : 0u,
           highCurrentReadback ? 1u : 0u,
           candidate->driveCurrentReq,
           candidate->driveCurrentNorm,
           driveCurrentReadbackNorm,
           (unsigned)activeChannelReadback,
           statusReg,
           configReg,
           muxConfig,
           clockDividers,
           (unsigned)finSelCode,
           (unsigned)finFactor,
           (unsigned)frefDivider,
           sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
           (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
           effectiveFrefHz,
           clockStatus ? clockStatus : SENSORARRAY_NA,
           match ? "lock_applied" : "warning_readback_mismatch");

    for (uint32_t i = 0u; i < discardCount; ++i) {
        sensorarrayFdcReadDiag_t throwawayDiag = {0};
        esp_err_t discardErr = sensorarrayMeasureReadFdcSampleDiag(fdcState->handle,
                                                                   channel,
                                                                   false,
                                                                   fdcState->haveIds,
                                                                   fdcState->configVerified,
                                                                   &throwawayDiag);
        printf("DBGFDC_S5D5,stage=%s,discardIndex=%lu,err=%ld,status=%s\n",
               stage,
               (unsigned long)i,
               (long)discardErr,
               (discardErr == ESP_OK) ? "discard_done" : "discard_read_error_continue");
        sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
    }

    return ESP_OK;
}

static esp_err_t sensorarrayS5d5WaitUnreadOrTimeout(Fdc2214CapDevice_t *dev,
                                                     Fdc2214CapChannel_t channel,
                                                     uint32_t timeoutMs,
                                                     uint32_t intervalMs,
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
            printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,intervalMs=%lu,polls=%lu,result=i2c_error,err=%ld\n",
                   (unsigned long)timeoutMs,
                   (unsigned long)intervalMs,
                   (unsigned long)(*outPollCount),
                   (long)err);
            return err;
        }

        bool unreadReady = outLastStatus->UnreadConversion[(uint8_t)channel];
        bool dataReady = outLastStatus->DataReady;
        if (unreadReady || dataReady) {
            *outReady = true;
            printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,intervalMs=%lu,polls=%lu,statusReg=0x%04X,unread=%u,"
                   "dataReady=%u,result=ready\n",
                   (unsigned long)timeoutMs,
                   (unsigned long)intervalMs,
                   (unsigned long)(*outPollCount),
                   outLastStatus->Raw,
                   unreadReady ? 1u : 0u,
                   dataReady ? 1u : 0u);
            return ESP_OK;
        }

        if (elapsedMs >= timeoutMs) {
            break;
        }

        sensorarrayDebugSelftestDelayMs(intervalMs);
        elapsedMs += intervalMs;
    }

    printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,intervalMs=%lu,polls=%lu,statusReg=0x%04X,unread=%u,"
           "dataReady=%u,result=timeout\n",
           (unsigned long)timeoutMs,
           (unsigned long)intervalMs,
           (unsigned long)(*outPollCount),
           outLastStatus->Raw,
           outLastStatus->UnreadConversion[(uint8_t)channel] ? 1u : 0u,
           outLastStatus->DataReady ? 1u : 0u);
    return ESP_OK;
}

static esp_err_t sensorarrayS5d5DoRecoveryReinit(sensorarrayState_t *state,
                                                 sensorarrayFdcDeviceState_t *fdcState,
                                                 const sensorarrayCheckpointGpio_t *checkpoint,
                                                 const char *reason,
                                                 uint32_t streakCount)
{
    sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_BEGIN);
    sensorarrayFdcInitDiag_t initDiag = {0};
    esp_err_t err = sensorarrayInitS5d5SecondaryFdc(state, fdcState, &initDiag);
    if (err == ESP_OK) {
        sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_FDC_INIT_OK);
    } else {
        sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_WARNING);
    }

    printf("DBGFDC_S5D5,stage=recovery_reinit,reason=%s,streak=%lu,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,"
           "detail=%ld,err=%ld,status=%s\n",
           reason ? reason : SENSORARRAY_NA,
           (unsigned long)streakCount,
           SENSORARRAY_FDC_I2C_ADDR_LOW,
           initDiag.manufacturerId,
           initDiag.deviceId,
           (long)initDiag.detail,
           (long)err,
           initDiag.status ? initDiag.status : SENSORARRAY_NA);
    return err;
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

    tmux1108Source_t swSource = TMUX1108_SOURCE_GND;
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

    uint32_t bootFullSweepSampleCount =
        (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_BOOT_FULL_SWEEP_SAMPLE_COUNT;
    if (bootFullSweepSampleCount == 0u) {
        bootFullSweepSampleCount = 1u;
    }
    uint32_t lockedSampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOCKED_SAMPLE_COUNT;
    if (lockedSampleCount == 0u) {
        lockedSampleCount = 1u;
    }
    uint32_t loopDelayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOOP_DELAY_MS;
    if (loopDelayMs < 50u) {
        loopDelayMs = 50u;
    }
    uint32_t unreadPollTimeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_UNREAD_POLL_TIMEOUT_MS;
    uint32_t unreadPollIntervalMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_UNREAD_POLL_INTERVAL_MS;
    if (unreadPollIntervalMs == 0u) {
        unreadPollIntervalMs = 1u;
    }
    if (unreadPollTimeoutMs < unreadPollIntervalMs) {
        unreadPollTimeoutMs = unreadPollIntervalMs;
    }
    uint32_t resweepBadStreak = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RESWEEP_BAD_STREAK;
    if (resweepBadStreak == 0u) {
        resweepBadStreak = 1u;
    }
    double resweepFreqDeltaHz = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RESWEEP_FREQ_DELTA_HZ;
    double resweepFreqDeltaRel =
        (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RESWEEP_FREQ_DELTA_PERCENT / 100.0;
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

    printf("DBGFDC_S5D5,stage=target,mode=S5D5_CAP_FDC_SECONDARY,fdcDev=secondary_selb_side,i2cPort=1,"
           "sda=%d,scl=%d,i2cAddr=0x%02X,route=S5D5_CAP,channel=CH0,bootFullSweepSamples=%lu,"
           "fastSweepSamples=%u,mediumSweepSamples=%u,lockedSamples=%lu,loopDelayMs=%lu,discardFirst=%u,"
           "resweepBadStreak=%lu,resweepFreqDeltaHz=%.3f,resweepFreqDeltaPercent=%u,lockMinScore=%ld,"
           "enableCapComputation=%u,enableNetCapOutput=%u,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f\n",
           busInfo.SdaGpio,
           busInfo.SclGpio,
           SENSORARRAY_FDC_I2C_ADDR_LOW,
           (unsigned long)bootFullSweepSampleCount,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FAST_SWEEP_SAMPLE_COUNT,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_MEDIUM_SWEEP_SAMPLE_COUNT,
           (unsigned long)lockedSampleCount,
           (unsigned long)loopDelayMs,
           discardFirst ? 1u : 0u,
           (unsigned long)resweepBadStreak,
           resweepFreqDeltaHz,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RESWEEP_FREQ_DELTA_PERCENT,
           (long)lockMinScore,
           capConfig.enableCapComputation ? 1u : 0u,
           capConfig.enableNetCapOutput ? 1u : 0u,
           capConfig.inductorValueUh,
           capConfig.fixedCapPf,
           capConfig.parasiticCapPf);
    printf("DBGFDC_S5D5,stage=route_semantics,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n");
    printf("DBGFDC_S5D5,stage=sweep_plan,deglitch=1MHz|3p3MHz|10MHz|33MHz,highCurrent=0|1,"
           "driveCurrentList=0xA000|0xB800|0xC000|0xD000|0xE000|0xF800,bootStepSettleMs=%u,"
           "profileSettleMs=%u,fastSettleMs=%u,mediumSettleMs=%u,sampleGapMs=%u,unreadPollTimeoutMs=%lu,"
           "unreadPollIntervalMs=%lu,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f\n",
           (unsigned)SENSORARRAY_S5D5_STEP_SETTLE_MS,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_PROFILE_SETTLE_MS,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FAST_SWEEP_SETTLE_MS,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_MEDIUM_SWEEP_SETTLE_MS,
           (unsigned)SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS,
           (unsigned long)unreadPollTimeoutMs,
           (unsigned long)unreadPollIntervalMs,
           capConfig.inductorValueUh,
           capConfig.fixedCapPf,
           capConfig.parasiticCapPf);

    sensorarrayS5d5StopAdsForIsolation(state);
    esp_err_t refPolicyErr = sensorarrayMeasureApplyRefPolicy(state,
                                                              "enter_ground_no_ref_mode",
                                                              "S5D5_CAP_FDC_SECONDARY",
                                                              SENSORARRAY_MATRIX_D_SOURCE_GND,
                                                              SENSORARRAY_ADS_INTREF_OFF,
                                                              SENSORARRAY_ADS_VBIAS_OFF,
                                                              "fdc_cap_no_ref");
    if (refPolicyErr != ESP_OK) {
        printf("DBGFDC_S5D5,stage=ref_policy,err=%ld,status=ref_policy_failed\n", (long)refPolicyErr);
        sensorarrayDebugIdleForever("s5d5_ref_policy_failed");
        return;
    }

    if (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_START_DELAY_MS > 0) {
        sensorarrayDebugSelftestDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_START_DELAY_MS);
    }

    bool locked = false;
    bool modeFdcInitialized = false;
    uint32_t lockEpoch = 0u;
    uint32_t lockedSampleGlobalIndex = 0u;
    uint32_t i2cFailureStreak = 0u;
    uint32_t nonConvertingStreak = 0u;
    uint32_t readbackMismatchStreak = 0u;
    uint32_t watchdogStreak = 0u;
    uint32_t relockFreqJumpStreak = 0u;
    uint32_t relockAmplitudeStreak = 0u;
    uint32_t runtimeAbnormalCount = 0u;
    bool haveLockedFreqHz = false;
    double lockedFreqHz = 0.0;
    sensorarrayS5d5SweepCandidate_t lockedCandidate = {0};

    while (true) {
        sensorarrayS5d5StopAdsForIsolation(state);

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
                   "detail=%ld,err=%ld,status=%s\n",
                   SENSORARRAY_FDC_I2C_ADDR_LOW,
                   initDiag.manufacturerId,
                   initDiag.deviceId,
                   (long)initDiag.detail,
                   (long)initErr,
                   initDiag.status ? initDiag.status : SENSORARRAY_NA);
            if (initErr != ESP_OK || !fdcState->ready || !fdcState->handle) {
                modeFdcInitialized = false;
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                continue;
            }
            modeFdcInitialized = true;
            locked = false;
            haveLockedFreqHz = false;
            lockedFreqHz = 0.0;
            relockFreqJumpStreak = 0u;
            relockAmplitudeStreak = 0u;
            runtimeAbnormalCount = 0u;
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
            if (!routeCheck.commandMatch || !routeCheck.gpioObservedMatch) {
                sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_WARNING);
                printf("DBGFDC_S5D5,stage=route_warn,commandMatch=%u,gpioObservedMatch=%u,"
                       "status=warning_continue\n",
                       routeCheck.commandMatch ? 1u : 0u,
                       routeCheck.gpioObservedMatch ? 1u : 0u);
            }

            sensorarrayS5d5SweepCandidate_t bestCandidate = {0};
            bool haveBest = false;
            const char *lockSource = "runtime_recovery";
            if ((CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_BOOT_FULL_SWEEP_ENABLE != 0) &&
                !s_s5d5BootEnvProfile.bootFullSweepDone) {
                lockSource = "boot_full_sweep";
                haveBest = sensorarrayS5d5RunBootFullSweep(fdcState,
                                                            fdcMap,
                                                            discardFirst,
                                                            lockMinScore,
                                                            &checkpoint,
                                                            &bestCandidate);
            } else {
                haveBest = sensorarrayS5d5RecoverAfterRuntimeFault(fdcState,
                                                                    fdcMap,
                                                                    discardFirst,
                                                                    lockMinScore,
                                                                    &checkpoint,
                                                                    "unlocked_runtime_lock",
                                                                    &bestCandidate);
            }

            if (!haveBest || !sensorarrayS5d5CandidateGoodEnough(&bestCandidate, lockMinScore)) {
                const char *reason = !haveBest ? "no_working_candidate" :
                                                  sensorarrayS5d5CandidateStatus(&bestCandidate, lockMinScore);
                printf("DBGFDC_S5D5,stage=lock_failed,source=%s,reason=%s,bestScore=%ld,minScore=%ld,"
                       "deglitchBandwidthOk=%u,hardReject=%u,status=error\n",
                       lockSource,
                       reason,
                       haveBest ? (long)bestCandidate.score : (long)INT_MIN,
                       (long)lockMinScore,
                       (haveBest && bestCandidate.deglitchBandwidthSufficient) ? 1u : 0u,
                       (haveBest && bestCandidate.hardReject) ? 1u : 0u);
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }

            uint32_t discardCountAfterLock = discardFirst ? 2u : 1u;
            esp_err_t lockErr = sensorarrayS5d5ApplyCandidateAndDiscard(fdcState,
                                                                        &bestCandidate,
                                                                        fdcMap->channel,
                                                                        discardCountAfterLock,
                                                                        "lock_apply");
            if (lockErr != ESP_OK) {
                (void)sensorarrayS5d5DoRecoveryReinit(state,
                                                      fdcState,
                                                      &checkpoint,
                                                      "lock_apply_failed",
                                                      1u);
                locked = false;
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                continue;
            }

            sensorarrayS5d5UpdateCurrentProfileFromCandidate(&bestCandidate);
            lockedCandidate = bestCandidate;
            lockEpoch++;
            locked = true;
            i2cFailureStreak = 0u;
            nonConvertingStreak = 0u;
            readbackMismatchStreak = 0u;
            watchdogStreak = 0u;
            relockFreqJumpStreak = 0u;
            relockAmplitudeStreak = 0u;
            runtimeAbnormalCount = 0u;
            haveLockedFreqHz = bestCandidate.avgFreqValid;
            lockedFreqHz = haveLockedFreqHz ? bestCandidate.avgFreqHz : 0.0;
            printf("DBGFDC_S5D5,stage=relock_freq_anchor,reason=initial_lock,lockEpoch=%lu,haveFreq=%u,"
                   "lockedFreqHz=%.3f,deglitchName=%s,deglitchReq=0x%X,driveCurrentReq=0x%04X,"
                   "highCurrentReq=%u,status=%s\n",
                   (unsigned long)lockEpoch,
                   haveLockedFreqHz ? 1u : 0u,
                   lockedFreqHz,
                   lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                   (unsigned)lockedCandidate.deglitchReq,
                   lockedCandidate.driveCurrentReq,
                   lockedCandidate.highCurrentReq ? 1u : 0u,
                   haveLockedFreqHz ? "anchor_set" : "anchor_pending");
        }

        sensorarrayS5d5LockedSummary_t lockedSummary = {0};
        lockedSummary.rawMin = UINT_MAX;
        bool needRecovery = false;
        const char *recoveryReason = SENSORARRAY_NA;
        uint32_t recoveryStreakCount = 0u;
        bool needRelock = false;
        const char *relockReason = SENSORARRAY_NA;
        double relockCurrentFreqHz = 0.0;
        double relockFreqDeltaHz = 0.0;
        double relockFreqRelDelta = 0.0;
        uint32_t relockTriggerStreak = 0u;

        for (uint32_t sampleIndex = 0u; sampleIndex < lockedSampleCount; ++sampleIndex) {
            Fdc2214CapStatus_t waitStatus = {0};
            bool waitReady = false;
            uint32_t waitPollCount = 0u;
            esp_err_t waitErr = sensorarrayS5d5WaitUnreadOrTimeout(fdcState->handle,
                                                                    fdcMap->channel,
                                                                    unreadPollTimeoutMs,
                                                                    unreadPollIntervalMs,
                                                                    &waitReady,
                                                                    &waitStatus,
                                                                    &waitPollCount);

            sensorarrayFdcReadDiag_t diag = {0};
            esp_err_t readErr = ESP_OK;
            if (waitErr == ESP_OK) {
                readErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(fdcState->handle,
                                                                     fdcMap->channel,
                                                                     false,
                                                                     fdcState->haveIds,
                                                                     fdcState->configVerified,
                                                                     &diag);
            } else {
                readErr = waitErr;
            }

            lockedSummary.totalSamples++;
            if (!waitReady) {
                lockedSummary.unreadTimeoutCount++;
            }

            if (readErr != ESP_OK) {
                i2cFailureStreak++;
                lockedSummary.i2cErrorCount++;
                lockedSummary.warningSamples++;
                lockedSampleGlobalIndex++;
                if (capConfig.enableNetCapOutput) {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "deglitchReq=0x%X,deglitchName=%s,deglitchBandwidthHz=%lu,deglitchReadback=na,"
                           "raw=0,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,"
                           "effectiveFclkHz=%lu,effectiveFrefHz=%.3f,freqHzBase=0.000,freqHz=0.000,"
                           "totalCapPf=na(reason=i2c_error),netCapPf=na(reason=i2c_error),"
                           "inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,converting=0,wd=0,aw=0,"
                           "sampleQuality=warning,status=i2c_read_error,clockStatus=sample_not_read\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned)lockedCandidate.deglitchReq,
                           lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                           (unsigned long)lockedCandidate.deglitchBandwidthHz,
                           0u,
                           0u,
                           0u,
                           0u,
                           sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
                           (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
                           0.0,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           waitStatus.UnreadConversion[(uint8_t)fdcMap->channel] ? 1u : 0u);
                } else {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "deglitchReq=0x%X,deglitchName=%s,deglitchBandwidthHz=%lu,deglitchReadback=na,"
                           "raw=0,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,"
                           "effectiveFclkHz=%lu,effectiveFrefHz=%.3f,freqHzBase=0.000,freqHz=0.000,"
                           "totalCapPf=na(reason=i2c_error),inductorUh=%.3f,fixedCapPf=%.3f,"
                           "parasiticCapPf=%.3f,unread=%u,converting=0,wd=0,aw=0,sampleQuality=warning,"
                           "status=i2c_read_error,clockStatus=sample_not_read\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned)lockedCandidate.deglitchReq,
                           lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                           (unsigned long)lockedCandidate.deglitchBandwidthHz,
                           0u,
                           0u,
                           0u,
                           0u,
                           sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
                           (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
                           0.0,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           waitStatus.UnreadConversion[(uint8_t)fdcMap->channel] ? 1u : 0u);
                }
                if (i2cFailureStreak >= resweepBadStreak && !needRecovery) {
                    needRecovery = true;
                    recoveryReason = "i2c_error_streak";
                    recoveryStreakCount = i2cFailureStreak;
                }
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
                continue;
            }

            i2cFailureStreak = 0u;
            bool activeChannelMatch = (diag.sample.ActiveChannel == fdcMap->channel);
            if (diag.sample.Raw28 != 0u) {
                lockedSummary.rawNonZeroCount++;
                if (diag.sample.Raw28 < lockedSummary.rawMin) {
                    lockedSummary.rawMin = diag.sample.Raw28;
                }
                if (diag.sample.Raw28 > lockedSummary.rawMax) {
                    lockedSummary.rawMax = diag.sample.Raw28;
                }
            }
            if (!diag.sample.Converting) {
                nonConvertingStreak++;
                lockedSummary.nonConvertingCount++;
            } else {
                nonConvertingStreak = 0u;
            }
            if (!activeChannelMatch) {
                readbackMismatchStreak++;
            } else {
                readbackMismatchStreak = 0u;
            }
            if (diag.sample.ErrWatchdog) {
                watchdogStreak++;
            } else {
                watchdogStreak = 0u;
            }

            if (nonConvertingStreak >= resweepBadStreak && !needRecovery) {
                needRecovery = true;
                recoveryReason = "non_converting_streak";
                recoveryStreakCount = nonConvertingStreak;
            }
            if (readbackMismatchStreak >= resweepBadStreak && !needRecovery) {
                needRecovery = true;
                recoveryReason = "config_readback_mismatch";
                recoveryStreakCount = readbackMismatchStreak;
            }
            if (watchdogStreak >= resweepBadStreak && !needRecovery) {
                needRecovery = true;
                recoveryReason = "watchdog_streak";
                recoveryStreakCount = watchdogStreak;
            }

            bool sampleQualityGood = diag.provisionalReadable &&
                                     waitReady &&
                                     diag.sample.UnreadConversionPresent &&
                                     !diag.sample.ErrWatchdog &&
                                     !diag.sample.ErrAmplitude &&
                                     activeChannelMatch;
            if (diag.sample.ErrAmplitude) {
                lockedSummary.amplitudeFaultSamples++;
            }

            uint16_t clockDividers = 0u;
            sensorarrayFdcFrequencyDiag_t freqDiag = {
                .refClockSource = sensorarrayMeasureFdcEffectiveRefClockSource(),
                .effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz(),
                .status = "clock_not_read",
            };
            esp_err_t clockErr = Fdc2214CapReadClockDividers(fdcState->handle, fdcMap->channel, &clockDividers);
            bool freqOk = false;
            if (clockErr == ESP_OK) {
                freqOk = sensorarrayMeasureFdcComputeFrequencyDiag(diag.sample.Raw28, clockDividers, &freqDiag);
            } else {
                freqDiag.clockDividers = clockDividers;
                freqDiag.freqHzBase = sensorarrayMeasureFdcRawToFrequencyHz(diag.sample.Raw28, freqDiag.effectiveFclkHz);
                freqDiag.valid = false;
                freqDiag.status = "clock_read_error";
            }

            if (freqOk && freqDiag.valid) {
                lockedSummary.clockValidCount++;
                lockedSummary.freqHzSum += freqDiag.freqHzCorrected;
                lockedSummary.freqSampleCount++;
            } else {
                lockedSummary.clockInvalidCount++;
            }

            double totalCapPf = 0.0;
            double netCapPf = 0.0;
            bool haveNetCapPf = false;
            const char *capReason = SENSORARRAY_NA;
            bool haveTotalCapPf = false;
            if (freqOk && freqDiag.valid) {
                haveTotalCapPf = sensorarrayS5d5TryComputeCapacitance(&capConfig,
                                                                       freqDiag.freqHzCorrected,
                                                                       &totalCapPf,
                                                                       &netCapPf,
                                                                       &haveNetCapPf,
                                                                       &capReason);
            } else {
                capReason = freqDiag.status ? freqDiag.status : "invalid_clock";
            }
            if (haveTotalCapPf) {
                lockedSummary.totalCapPfSum += totalCapPf;
                lockedSummary.totalCapSampleCount++;
                if (haveNetCapPf) {
                    lockedSummary.netCapPfSum += netCapPf;
                    lockedSummary.netCapSampleCount++;
                }
            }
            const char *statusName = sensorarrayMeasureFdcSampleStatusName(diag.statusCode);
            const char *clockStatus = freqDiag.status ? freqDiag.status : SENSORARRAY_NA;
            const char *sampleQuality = (!freqOk && diag.sample.Raw28 != 0u)
                                            ? "bad_clock"
                                            : (sampleQualityGood ? "good" : "warning");
            const char *statusField = freqOk ? statusName : clockStatus;
            uint8_t currentDeglitchReadback =
                (uint8_t)(diag.sample.MuxRaw & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK);

            if (diag.sample.ErrAmplitude) {
                relockAmplitudeStreak++;
            } else {
                relockAmplitudeStreak = 0u;
            }

            if (freqOk && freqDiag.valid) {
                if (!haveLockedFreqHz || lockedFreqHz <= 0.0) {
                    haveLockedFreqHz = true;
                    lockedFreqHz = freqDiag.freqHzCorrected;
                    relockFreqJumpStreak = 0u;
                    printf("DBGFDC_S5D5,stage=relock_freq_anchor,reason=first_locked_sample,lockEpoch=%lu,"
                           "haveFreq=1,lockedFreqHz=%.3f,deglitchName=%s,deglitchReq=0x%X,driveCurrentReq=0x%04X,"
                           "highCurrentReq=%u,status=anchor_set\n",
                           (unsigned long)lockEpoch,
                           lockedFreqHz,
                           lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                           (unsigned)lockedCandidate.deglitchReq,
                           lockedCandidate.driveCurrentReq,
                           lockedCandidate.highCurrentReq ? 1u : 0u);
                } else {
                    relockFreqDeltaHz = (freqDiag.freqHzCorrected >= lockedFreqHz)
                                            ? (freqDiag.freqHzCorrected - lockedFreqHz)
                                            : (lockedFreqHz - freqDiag.freqHzCorrected);
                    relockFreqRelDelta = (lockedFreqHz > 0.0) ? (relockFreqDeltaHz / lockedFreqHz) : 0.0;
                    if (relockFreqDeltaHz >= resweepFreqDeltaHz ||
                        relockFreqRelDelta >= resweepFreqDeltaRel) {
                        relockFreqJumpStreak++;
                    } else {
                        relockFreqJumpStreak = 0u;
                    }
                }
            } else {
                relockFreqJumpStreak = 0u;
                relockFreqDeltaHz = 0.0;
                relockFreqRelDelta = 0.0;
            }

            if (!needRelock &&
                (relockFreqJumpStreak >= resweepBadStreak ||
                 relockAmplitudeStreak >= resweepBadStreak)) {
                needRelock = true;
                relockReason = (relockAmplitudeStreak >= resweepBadStreak)
                                   ? "amplitude_fault_streak"
                                   : "freq_jump_streak";
                relockCurrentFreqHz = (freqOk && freqDiag.valid) ? freqDiag.freqHzCorrected : 0.0;
                relockTriggerStreak =
                    (relockAmplitudeStreak >= resweepBadStreak)
                        ? relockAmplitudeStreak
                        : relockFreqJumpStreak;
                printf("DBGFDC_S5D5,stage=relock_trigger,reason=%s,lockEpoch=%lu,index=%lu,streak=%lu,"
                       "lockedFreqHz=%.3f,currentFreqHz=%.3f,freqDeltaHz=%.3f,freqRelDelta=%.6f,"
                       "freqJumpStreak=%lu,amplitudeStreak=%lu,amplitudeFault=%u,deglitchName=%s,"
                       "deglitchReq=0x%X,deglitchReadback=0x%X,driveCurrentReq=0x%04X,driveCurrentNorm=0x%04X,"
                       "highCurrentReq=%u,status=relock_pending\n",
                       relockReason,
                       (unsigned long)lockEpoch,
                       (unsigned long)(lockedSampleGlobalIndex + 1u),
                       (unsigned long)relockTriggerStreak,
                       lockedFreqHz,
                       relockCurrentFreqHz,
                       relockFreqDeltaHz,
                       relockFreqRelDelta,
                       (unsigned long)relockFreqJumpStreak,
                       (unsigned long)relockAmplitudeStreak,
                       diag.sample.ErrAmplitude ? 1u : 0u,
                       lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                       (unsigned)lockedCandidate.deglitchReq,
                       (unsigned)currentDeglitchReadback,
                       lockedCandidate.driveCurrentReq,
                       lockedCandidate.driveCurrentNorm,
                       lockedCandidate.highCurrentReq ? 1u : 0u);
            }

            if (freqOk && sampleQualityGood) {
                lockedSummary.goodSamples++;
            } else {
                lockedSummary.warningSamples++;
            }

            lockedSampleGlobalIndex++;
            if (haveTotalCapPf) {
                if (haveNetCapPf) {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "deglitchReq=0x%X,deglitchName=%s,deglitchBandwidthHz=%lu,deglitchReadback=0x%X,"
                           "raw=%lu,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,"
                           "effectiveFclkHz=%lu,effectiveFrefHz=%.3f,freqHzBase=%.3f,freqHz=%.3f,totalCapPf=%.3f,"
                           "netCapPf=%.3f,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,"
                           "converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s,clockStatus=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned)lockedCandidate.deglitchReq,
                           lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                           (unsigned long)lockedCandidate.deglitchBandwidthHz,
                           (unsigned)currentDeglitchReadback,
                           (unsigned long)diag.sample.Raw28,
                           freqDiag.clockDividers,
                           (unsigned)freqDiag.finSelCode,
                           (unsigned)freqDiag.finFactor,
                           (unsigned)freqDiag.frefDivider,
                           sensorarrayMeasureFdcRefClockSourceName(freqDiag.refClockSource),
                           (unsigned long)freqDiag.effectiveFclkHz,
                           freqDiag.effectiveFrefHz,
                           freqDiag.freqHzBase,
                           freqDiag.freqHzCorrected,
                           totalCapPf,
                           netCapPf,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           sampleQuality,
                           statusField,
                           clockStatus);
                } else {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "deglitchReq=0x%X,deglitchName=%s,deglitchBandwidthHz=%lu,deglitchReadback=0x%X,"
                           "raw=%lu,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,"
                           "effectiveFclkHz=%lu,effectiveFrefHz=%.3f,freqHzBase=%.3f,freqHz=%.3f,totalCapPf=%.3f,"
                           "inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,"
                           "sampleQuality=%s,status=%s,clockStatus=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned)lockedCandidate.deglitchReq,
                           lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                           (unsigned long)lockedCandidate.deglitchBandwidthHz,
                           (unsigned)currentDeglitchReadback,
                           (unsigned long)diag.sample.Raw28,
                           freqDiag.clockDividers,
                           (unsigned)freqDiag.finSelCode,
                           (unsigned)freqDiag.finFactor,
                           (unsigned)freqDiag.frefDivider,
                           sensorarrayMeasureFdcRefClockSourceName(freqDiag.refClockSource),
                           (unsigned long)freqDiag.effectiveFclkHz,
                           freqDiag.effectiveFrefHz,
                           freqDiag.freqHzBase,
                           freqDiag.freqHzCorrected,
                           totalCapPf,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           sampleQuality,
                           statusField,
                           clockStatus);
                }
            } else {
                if (capConfig.enableNetCapOutput) {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "deglitchReq=0x%X,deglitchName=%s,deglitchBandwidthHz=%lu,deglitchReadback=0x%X,"
                           "raw=%lu,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,"
                           "effectiveFclkHz=%lu,effectiveFrefHz=%.3f,freqHzBase=%.3f,freqHz=%.3f,"
                           "totalCapPf=na(reason=%s),netCapPf=na(reason=%s),inductorUh=%.3f,fixedCapPf=%.3f,"
                           "parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s,"
                           "clockStatus=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned)lockedCandidate.deglitchReq,
                           lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                           (unsigned long)lockedCandidate.deglitchBandwidthHz,
                           (unsigned)currentDeglitchReadback,
                           (unsigned long)diag.sample.Raw28,
                           freqDiag.clockDividers,
                           (unsigned)freqDiag.finSelCode,
                           (unsigned)freqDiag.finFactor,
                           (unsigned)freqDiag.frefDivider,
                           sensorarrayMeasureFdcRefClockSourceName(freqDiag.refClockSource),
                           (unsigned long)freqDiag.effectiveFclkHz,
                           freqDiag.effectiveFrefHz,
                           freqDiag.freqHzBase,
                           freqDiag.freqHzCorrected,
                           capReason,
                           capReason,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           sampleQuality,
                           statusField,
                           clockStatus);
                } else {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "deglitchReq=0x%X,deglitchName=%s,deglitchBandwidthHz=%lu,deglitchReadback=0x%X,"
                           "raw=%lu,clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,"
                           "effectiveFclkHz=%lu,effectiveFrefHz=%.3f,freqHzBase=%.3f,freqHz=%.3f,"
                           "totalCapPf=na(reason=%s),inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,"
                           "unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s,clockStatus=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned)lockedCandidate.deglitchReq,
                           lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                           (unsigned long)lockedCandidate.deglitchBandwidthHz,
                           (unsigned)currentDeglitchReadback,
                           (unsigned long)diag.sample.Raw28,
                           freqDiag.clockDividers,
                           (unsigned)freqDiag.finSelCode,
                           (unsigned)freqDiag.finFactor,
                           (unsigned)freqDiag.frefDivider,
                           sensorarrayMeasureFdcRefClockSourceName(freqDiag.refClockSource),
                           (unsigned long)freqDiag.effectiveFclkHz,
                           freqDiag.effectiveFrefHz,
                           freqDiag.freqHzBase,
                           freqDiag.freqHzCorrected,
                           capReason,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           sampleQuality,
                           statusField,
                           clockStatus);
                }
            }
            if (needRelock) {
                break;
            }
            sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
        }

        bool haveAvgFreqHz = (lockedSummary.freqSampleCount > 0u);
        bool haveAvgTotalCapPf = (lockedSummary.totalCapSampleCount > 0u);
        bool haveAvgNetCapPf = (lockedSummary.netCapSampleCount > 0u);
        double avgFreqHz = haveAvgFreqHz ? (lockedSummary.freqHzSum / (double)lockedSummary.freqSampleCount) : 0.0;
        double avgTotalCapPf =
            haveAvgTotalCapPf ? (lockedSummary.totalCapPfSum / (double)lockedSummary.totalCapSampleCount) : 0.0;
        double avgNetCapPf = haveAvgNetCapPf ? (lockedSummary.netCapPfSum / (double)lockedSummary.netCapSampleCount) : 0.0;
        uint32_t lockedRawSpan =
            (lockedSummary.rawNonZeroCount > 0u && lockedSummary.rawMax >= lockedSummary.rawMin)
                ? (lockedSummary.rawMax - lockedSummary.rawMin)
                : 0u;
        uint32_t baselineRawSpan = s_s5d5FdcProfile.valid ? s_s5d5FdcProfile.baselineRawSpan : lockedCandidate.rawSpan;
        uint32_t rawSpanLimit =
            (baselineRawSpan * SENSORARRAY_S5D5_LOCKED_RAW_SPAN_MULTIPLIER) + SENSORARRAY_S5D5_LOCKED_RAW_SPAN_FLOOR;
        if (!needRelock && lockedRawSpan > rawSpanLimit && lockedRawSpan > SENSORARRAY_S5D5_LOCKED_RAW_SPAN_FLOOR) {
            needRelock = true;
            relockReason = "raw_span_too_large";
            relockTriggerStreak = runtimeAbnormalCount + 1u;
        }

        const char *avgTotalCapReason = capConfig.enableCapComputation
                                            ? ((capConfig.inductorValueUh > 0.0) ? "no_valid_cap_sample"
                                                                                 : "no_inductor_value")
                                            : "cap_computation_disabled";
        const char *avgNetCapReason = capConfig.enableCapComputation
                                          ? ((capConfig.inductorValueUh > 0.0) ? "no_valid_net_cap_sample"
                                                                               : "no_inductor_value")
                                          : "cap_computation_disabled";

        char avgFreqHzField[64] = {0};
        char avgTotalCapPfField[64] = {0};
        char avgNetCapPfField[64] = {0};
        if (haveAvgFreqHz) {
            (void)snprintf(avgFreqHzField, sizeof(avgFreqHzField), "%.3f", avgFreqHz);
        } else {
            (void)snprintf(avgFreqHzField, sizeof(avgFreqHzField), "na(reason=no_valid_frequency)");
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
                   "amplitudeFault=%lu,i2cErr=%lu,waitTimeout=%lu,nonConverting=%lu,clockValid=%lu,clockInvalid=%lu,"
                   "rawNonZero=%lu,rawSpan=%lu,rawSpanLimit=%lu,runtimeAbnormalCount=%lu,"
                   "refClockSource=%s,effectiveFclkHz=%lu,avgFreqHz=%s,"
                   "avgTotalCapPf=%s,avgNetCapPf=%s,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,"
                   "i2cStreak=%lu,nonConvertingStreak=%lu,readbackMismatchStreak=%lu,status=%s\n",
                   (unsigned long)lockEpoch,
                   (unsigned long)lockedSummary.totalSamples,
                   (unsigned long)lockedSummary.goodSamples,
                   (unsigned long)lockedSummary.warningSamples,
                   (unsigned long)lockedSummary.amplitudeFaultSamples,
                   (unsigned long)lockedSummary.i2cErrorCount,
                   (unsigned long)lockedSummary.unreadTimeoutCount,
                   (unsigned long)lockedSummary.nonConvertingCount,
                   (unsigned long)lockedSummary.clockValidCount,
                   (unsigned long)lockedSummary.clockInvalidCount,
                   (unsigned long)lockedSummary.rawNonZeroCount,
                   (unsigned long)lockedRawSpan,
                   (unsigned long)rawSpanLimit,
                   (unsigned long)runtimeAbnormalCount,
                   sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
                   (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
                   avgFreqHzField,
                   avgTotalCapPfField,
                   avgNetCapPfField,
                   capConfig.inductorValueUh,
                   capConfig.fixedCapPf,
                   capConfig.parasiticCapPf,
                   (unsigned long)i2cFailureStreak,
                   (unsigned long)nonConvertingStreak,
                   (unsigned long)readbackMismatchStreak,
                   needRecovery ? "recovery_pending" : (needRelock ? "relock_pending" : "locked_continue"));
        } else {
            printf("DBGFDC_S5D5,stage=locked_summary,lockEpoch=%lu,samples=%lu,good=%lu,warning=%lu,"
                   "amplitudeFault=%lu,i2cErr=%lu,waitTimeout=%lu,nonConverting=%lu,clockValid=%lu,clockInvalid=%lu,"
                   "rawNonZero=%lu,rawSpan=%lu,rawSpanLimit=%lu,runtimeAbnormalCount=%lu,"
                   "refClockSource=%s,effectiveFclkHz=%lu,avgFreqHz=%s,"
                   "avgTotalCapPf=%s,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,i2cStreak=%lu,"
                   "nonConvertingStreak=%lu,readbackMismatchStreak=%lu,status=%s\n",
                   (unsigned long)lockEpoch,
                   (unsigned long)lockedSummary.totalSamples,
                   (unsigned long)lockedSummary.goodSamples,
                   (unsigned long)lockedSummary.warningSamples,
                   (unsigned long)lockedSummary.amplitudeFaultSamples,
                   (unsigned long)lockedSummary.i2cErrorCount,
                   (unsigned long)lockedSummary.unreadTimeoutCount,
                   (unsigned long)lockedSummary.nonConvertingCount,
                   (unsigned long)lockedSummary.clockValidCount,
                   (unsigned long)lockedSummary.clockInvalidCount,
                   (unsigned long)lockedSummary.rawNonZeroCount,
                   (unsigned long)lockedRawSpan,
                   (unsigned long)rawSpanLimit,
                   (unsigned long)runtimeAbnormalCount,
                   sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
                   (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
                   avgFreqHzField,
                   avgTotalCapPfField,
                   capConfig.inductorValueUh,
                   capConfig.fixedCapPf,
                   capConfig.parasiticCapPf,
                   (unsigned long)i2cFailureStreak,
                   (unsigned long)nonConvertingStreak,
                   (unsigned long)readbackMismatchStreak,
                   needRecovery ? "recovery_pending" : (needRelock ? "relock_pending" : "locked_continue"));
        }

        if (needRecovery || needRelock) {
            const char *runtimeReason = needRecovery ? recoveryReason : relockReason;
            uint32_t triggerStreak = needRecovery ? recoveryStreakCount : relockTriggerStreak;
            runtimeAbnormalCount++;
            if (s_s5d5FdcProfile.valid) {
                s_s5d5FdcProfile.abnormalCount = runtimeAbnormalCount;
            }
            printf("DBGFDC_S5D5,stage=runtime_abnormal,reason=%s,lockEpoch=%lu,abnormalCount=%lu,"
                   "required=%lu,triggerStreak=%lu,lockedFreqHz=%.3f,currentFreqHz=%.3f,"
                   "freqDeltaHz=%.3f,freqRelDelta=%.6f,rawSpan=%lu,rawSpanLimit=%lu,status=%s\n",
                   runtimeReason ? runtimeReason : SENSORARRAY_NA,
                   (unsigned long)lockEpoch,
                   (unsigned long)runtimeAbnormalCount,
                   (unsigned long)resweepBadStreak,
                   (unsigned long)triggerStreak,
                   lockedFreqHz,
                   relockCurrentFreqHz,
                   relockFreqDeltaHz,
                   relockFreqRelDelta,
                   (unsigned long)lockedRawSpan,
                   (unsigned long)rawSpanLimit,
                   (runtimeAbnormalCount >= resweepBadStreak) ? "recovery_pending" : "streak_collecting");

            if (runtimeAbnormalCount < resweepBadStreak) {
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }

            sensorarrayS5d5StopAdsForIsolation(state);
            sensorarrayS5d5SweepCandidate_t recoveryCandidate = {0};
            bool recovered = sensorarrayS5d5RecoverAfterRuntimeFault(fdcState,
                                                                      fdcMap,
                                                                      discardFirst,
                                                                      lockMinScore,
                                                                      &checkpoint,
                                                                      runtimeReason,
                                                                      &recoveryCandidate);
            if (!recovered) {
                esp_err_t recoveryErr = sensorarrayS5d5DoRecoveryReinit(state,
                                                                         fdcState,
                                                                         &checkpoint,
                                                                         runtimeReason,
                                                                         runtimeAbnormalCount);
                locked = false;
                modeFdcInitialized = (recoveryErr == ESP_OK);
                haveLockedFreqHz = false;
                relockFreqJumpStreak = 0u;
                relockAmplitudeStreak = 0u;
                watchdogStreak = 0u;
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                continue;
            }

            esp_err_t relockApplyErr = sensorarrayS5d5ApplyCandidateAndDiscard(fdcState,
                                                                               &recoveryCandidate,
                                                                               fdcMap->channel,
                                                                               SENSORARRAY_S5D5_RELOCK_DISCARD_SAMPLES,
                                                                               "runtime_recovery_apply");
            if (relockApplyErr != ESP_OK) {
                printf("DBGFDC_S5D5,stage=runtime_recovery_apply_failed,reason=%s,err=%ld,deglitchName=%s,"
                       "deglitchReq=0x%X,driveCurrentReq=0x%04X,highCurrentReq=%u,status=failed\n",
                       runtimeReason ? runtimeReason : SENSORARRAY_NA,
                       (long)relockApplyErr,
                       recoveryCandidate.deglitchName ? recoveryCandidate.deglitchName : SENSORARRAY_NA,
                       (unsigned)recoveryCandidate.deglitchReq,
                       recoveryCandidate.driveCurrentReq,
                       recoveryCandidate.highCurrentReq ? 1u : 0u);
                locked = false;
                haveLockedFreqHz = false;
                relockFreqJumpStreak = 0u;
                relockAmplitudeStreak = 0u;
                watchdogStreak = 0u;
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
                continue;
            }

            sensorarrayS5d5UpdateCurrentProfileFromCandidate(&recoveryCandidate);
            lockedCandidate = recoveryCandidate;
            lockEpoch++;
            locked = true;
            i2cFailureStreak = 0u;
            nonConvertingStreak = 0u;
            readbackMismatchStreak = 0u;
            watchdogStreak = 0u;
            relockFreqJumpStreak = 0u;
            relockAmplitudeStreak = 0u;
            runtimeAbnormalCount = 0u;
            haveLockedFreqHz = recoveryCandidate.avgFreqValid;
            lockedFreqHz = haveLockedFreqHz ? recoveryCandidate.avgFreqHz : 0.0;
            printf("DBGFDC_S5D5,stage=relock_freq_anchor,reason=runtime_recovery,lockEpoch=%lu,haveFreq=%u,"
                   "lockedFreqHz=%.3f,deglitchName=%s,deglitchReq=0x%X,driveCurrentReq=0x%04X,"
                   "highCurrentReq=%u,status=%s\n",
                   (unsigned long)lockEpoch,
                   haveLockedFreqHz ? 1u : 0u,
                   lockedFreqHz,
                   lockedCandidate.deglitchName ? lockedCandidate.deglitchName : SENSORARRAY_NA,
                   (unsigned)lockedCandidate.deglitchReq,
                   lockedCandidate.driveCurrentReq,
                   lockedCandidate.highCurrentReq ? 1u : 0u,
                   haveLockedFreqHz ? "anchor_set" : "anchor_pending");
            continue;
        }

        runtimeAbnormalCount = 0u;
        if (s_s5d5FdcProfile.valid) {
            s_s5d5FdcProfile.abnormalCount = 0u;
        }

        sensorarrayDebugSelftestDelayMs(loopDelayMs);
    }
}
