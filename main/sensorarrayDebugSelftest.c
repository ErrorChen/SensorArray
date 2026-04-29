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
#define SENSORARRAY_S5D5_REG_STATUS 0x18u
#define SENSORARRAY_S5D5_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_S5D5_REG_CONFIG 0x1Au
#define SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0 0x1Eu
#define SENSORARRAY_S5D5_STEP_SETTLE_MS 350u
#define SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS 20u
#define SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS 500u

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
    return tmuxSwitch1108SourceToSwLevel(source);
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
    bool selected;
    bool readbackValid;
    bool highCurrentReadback;
    uint16_t driveCurrentReadbackNorm;
    uint8_t activeChannelReadback;
    uint16_t statusRegReadback;
    uint16_t configRegReadback;
    uint16_t muxConfigReadback;
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
    uint32_t i2cErrorCount;
    uint32_t unreadTimeoutCount;
    uint32_t nonConvertingCount;
    uint32_t freqSampleCount;
    uint32_t totalCapSampleCount;
    uint32_t netCapSampleCount;
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
                                         uint16_t driveCurrentReq)
{
    if (!candidate) {
        return;
    }

    *candidate = (sensorarrayS5d5SweepCandidate_t){
        .highCurrentReq = highCurrentReq,
        .driveCurrentReq = driveCurrentReq,
        .driveCurrentNorm = (uint16_t)(driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .rawMin = UINT_MAX,
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

    int32_t score = 0;
    score += (int32_t)((candidate->convertingCount * 40u) / denom);
    score += (int32_t)((candidate->unreadCount * 25u) / denom);
    score += (int32_t)((candidate->nonZeroRawCount * 30u) / denom);
    score += (int32_t)((candidate->provisionalReadableCount * 35u) / denom);
    score += (int32_t)((candidate->activeChannelMatchCount * 10u) / denom);
    score -= (int32_t)(candidate->i2cErrorCount * 60u);
    score -= (int32_t)(candidate->watchdogCount * 35u);
    score -= (int32_t)(candidate->amplitudeCount * 10u);

    if (candidate->readbackValid) {
        bool highCurrentMatch = (candidate->highCurrentReadback == candidate->highCurrentReq);
        bool driveMatch = (candidate->driveCurrentReadbackNorm == candidate->driveCurrentNorm);
        bool activeMatch = (candidate->activeChannelReadback == (uint8_t)FDC2214_CH0);
        score += (highCurrentMatch && driveMatch && activeMatch) ? 12 : -18;
    }

    if (candidate->convertingCount == 0u) {
        score -= 120;
    }
    if (candidate->nonZeroRawCount == 0u) {
        score -= 120;
    }

    if (candidate->rawMean > 0u) {
        uint64_t spanPermille = ((uint64_t)candidate->rawSpan * 1000ull) / candidate->rawMean;
        if (spanPermille <= 5ull) {
            score += 20;
        } else if (spanPermille <= 20ull) {
            score += 12;
        } else if (spanPermille <= 50ull) {
            score += 5;
        } else if (spanPermille <= 120ull) {
            score -= 8;
        } else {
            score -= 20;
        }
    }

    candidate->score = score;
    return score;
}

static const char *sensorarrayS5d5CandidateStatus(const sensorarrayS5d5SweepCandidate_t *candidate, int32_t minScore)
{
    if (!candidate) {
        return "invalid";
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
    if (candidate->score < minScore) {
        return "below_min_score";
    }
    return "working";
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

    if (candidate->score != best->score) {
        return candidate->score > best->score;
    }
    if (candidate->watchdogCount != best->watchdogCount) {
        return candidate->watchdogCount < best->watchdogCount;
    }
    if (candidate->unreadCount != best->unreadCount) {
        return candidate->unreadCount > best->unreadCount;
    }
    if (candidate->rawSpan != best->rawSpan) {
        return candidate->rawSpan < best->rawSpan;
    }
    if (candidate->i2cErrorCount != best->i2cErrorCount) {
        return candidate->i2cErrorCount < best->i2cErrorCount;
    }
    return candidate->driveCurrentReq > best->driveCurrentReq;
}

static bool sensorarrayS5d5SweepCandidates(sensorarrayFdcDeviceState_t *fdcState,
                                           const sensorarrayFdcDLineMap_t *fdcMap,
                                           uint32_t sweepSampleCount,
                                           bool discardFirst,
                                           int32_t minScore,
                                           const sensorarrayCheckpointGpio_t *checkpoint,
                                           sensorarrayS5d5SweepCandidate_t *outBestCandidate)
{
    if (!fdcState || !fdcState->handle || !fdcMap || !outBestCandidate) {
        return false;
    }

    printf("DBGFDC_S5D5,stage=sweep_begin,candidates=%u,samplesPerCandidate=%lu\n",
           (unsigned)((sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE) /
                       sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[0])) *
                      (sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE) /
                       sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[0]))),
           (unsigned long)sweepSampleCount);

    bool haveBest = false;
    sensorarrayS5d5SweepCandidate_t bestCandidate = {0};

    for (size_t highIndex = 0u;
         highIndex < (sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE) /
                      sizeof(SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[0]));
         ++highIndex) {
        bool highCurrentReq = SENSORARRAY_S5D5_HIGH_CURRENT_SWEEP_TABLE[highIndex];
        for (size_t driveIndex = 0u;
             driveIndex < (sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE) /
                           sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[0]));
             ++driveIndex) {
            sensorarrayCheckpointEmit(checkpoint, SENSORARRAY_CHECKPOINT_EVENT_STEP_BEGIN);

            sensorarrayS5d5SweepCandidate_t candidate = {0};
            sensorarrayS5d5CandidateInit(&candidate, highCurrentReq, SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[driveIndex]);

            esp_err_t cfgErr =
                sensorarrayApplyS5d5DriveStep(fdcState->handle, candidate.highCurrentReq, candidate.driveCurrentReq);
            if (cfgErr != ESP_OK) {
                candidate.i2cErrorCount++;
                candidate.sampleCount = 1u;
                candidate.score = INT_MIN / 4;
                printf("DBGFDC_S5D5,stage=sweep_candidate,highCurrentReq=%u,driveCurrentReq=0x%04X,samples=%lu,"
                       "i2cErr=%lu,convertingOk=%lu,unreadOk=%lu,watchdog=%lu,amplitude=%lu,nonZeroRaw=%lu,"
                       "rawMin=%lu,rawMax=%lu,rawSpan=%lu,score=%ld,status=config_write_error\n",
                       candidate.highCurrentReq ? 1u : 0u,
                       candidate.driveCurrentReq,
                       (unsigned long)candidate.sampleCount,
                       (unsigned long)candidate.i2cErrorCount,
                       (unsigned long)candidate.convertingCount,
                       (unsigned long)candidate.unreadCount,
                       (unsigned long)candidate.watchdogCount,
                       (unsigned long)candidate.amplitudeCount,
                       (unsigned long)candidate.nonZeroRawCount,
                       (unsigned long)candidate.rawMin,
                       (unsigned long)candidate.rawMax,
                       (unsigned long)candidate.rawSpan,
                       (long)candidate.score);
                continue;
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
                } else {
                    sensorarrayS5d5CandidateIngestDiag(&candidate, &diag, fdcMap->channel);
                }
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
            }

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
            }

            (void)sensorarrayS5d5FinalizeCandidate(&candidate);
            const char *candidateStatus = sensorarrayS5d5CandidateStatus(&candidate, minScore);
            printf("DBGFDC_S5D5,stage=sweep_candidate,highCurrentReq=%u,highCurrentReadback=%u,driveCurrentReq=0x%04X,"
                   "driveCurrentNorm=0x%04X,driveCurrentReadback=0x%04X,activeChannelReadback=%u,samples=%lu,"
                   "i2cErr=%lu,convertingOk=%lu,unreadOk=%lu,watchdog=%lu,amplitude=%lu,nonZeroRaw=%lu,"
                   "rawMin=%lu,rawMax=%lu,rawMean=%lu,rawSpan=%lu,statusReg=0x%04X,configReg=0x%04X,muxConfig=0x%04X,"
                   "score=%ld,status=%s\n",
                   candidate.highCurrentReq ? 1u : 0u,
                   candidate.highCurrentReadback ? 1u : 0u,
                   candidate.driveCurrentReq,
                   candidate.driveCurrentNorm,
                   candidate.driveCurrentReadbackNorm,
                   (unsigned)candidate.activeChannelReadback,
                   (unsigned long)candidate.sampleCount,
                   (unsigned long)candidate.i2cErrorCount,
                   (unsigned long)candidate.convertingCount,
                   (unsigned long)candidate.unreadCount,
                   (unsigned long)candidate.watchdogCount,
                   (unsigned long)candidate.amplitudeCount,
                   (unsigned long)candidate.nonZeroRawCount,
                   (unsigned long)candidate.rawMin,
                   (unsigned long)candidate.rawMax,
                   (unsigned long)candidate.rawMean,
                   (unsigned long)candidate.rawSpan,
                   candidate.statusRegReadback,
                   candidate.configRegReadback,
                   candidate.muxConfigReadback,
                   (long)candidate.score,
                   candidateStatus);

            if (!haveBest || sensorarrayS5d5CandidateBetter(&candidate, &bestCandidate)) {
                bestCandidate = candidate;
                haveBest = true;
            }
        }
    }

    if (!haveBest) {
        return false;
    }

    bestCandidate.selected = true;
    *outBestCandidate = bestCandidate;
    printf("DBGFDC_S5D5,stage=sweep_selected,highCurrentReq=%u,driveCurrentReq=0x%04X,driveCurrentNorm=0x%04X,"
           "driveCurrentReadback=0x%04X,activeChannelReadback=%u,samples=%lu,i2cErr=%lu,convertingOk=%lu,unreadOk=%lu,"
           "watchdog=%lu,amplitude=%lu,nonZeroRaw=%lu,rawMin=%lu,rawMax=%lu,rawSpan=%lu,score=%ld,status=%s\n",
           bestCandidate.highCurrentReq ? 1u : 0u,
           bestCandidate.driveCurrentReq,
           bestCandidate.driveCurrentNorm,
           bestCandidate.driveCurrentReadbackNorm,
           (unsigned)bestCandidate.activeChannelReadback,
           (unsigned long)bestCandidate.sampleCount,
           (unsigned long)bestCandidate.i2cErrorCount,
           (unsigned long)bestCandidate.convertingCount,
           (unsigned long)bestCandidate.unreadCount,
           (unsigned long)bestCandidate.watchdogCount,
           (unsigned long)bestCandidate.amplitudeCount,
           (unsigned long)bestCandidate.nonZeroRawCount,
           (unsigned long)bestCandidate.rawMin,
           (unsigned long)bestCandidate.rawMax,
           (unsigned long)bestCandidate.rawSpan,
           (long)bestCandidate.score,
           sensorarrayS5d5CandidateStatus(&bestCandidate, minScore));
    return true;
}

static esp_err_t sensorarrayS5d5ApplyLockedCandidate(Fdc2214CapDevice_t *dev,
                                                     const sensorarrayS5d5SweepCandidate_t *candidate,
                                                     Fdc2214CapChannel_t channel)
{
    if (!dev || !candidate) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayApplyS5d5DriveStep(dev, candidate->highCurrentReq, candidate->driveCurrentReq);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=lock_apply,highCurrentReq=%u,driveCurrentReq=0x%04X,err=%ld,status=apply_failed\n",
               candidate->highCurrentReq ? 1u : 0u,
               candidate->driveCurrentReq,
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
        printf("DBGFDC_S5D5,stage=lock_apply,highCurrentReq=%u,driveCurrentReq=0x%04X,err=%ld,status=readback_failed\n",
               candidate->highCurrentReq ? 1u : 0u,
               candidate->driveCurrentReq,
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

    printf("DBGFDC_S5D5,stage=lock_apply,highCurrentReq=%u,highCurrentReadback=%u,driveCurrentReq=0x%04X,"
           "driveCurrentNorm=0x%04X,driveCurrentReadback=0x%04X,activeChannelReadback=%u,statusReg=0x%04X,"
           "configReg=0x%04X,muxConfig=0x%04X,status=%s\n",
           candidate->highCurrentReq ? 1u : 0u,
           highCurrentReadback ? 1u : 0u,
           candidate->driveCurrentReq,
           candidate->driveCurrentNorm,
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
           "sda=%d,scl=%d,i2cAddr=0x%02X,route=S5D5_CAP,channel=CH0,sweepSamples=%lu,lockedSamples=%lu,"
           "loopDelayMs=%lu,discardFirst=%u,recoveryThreshold=%lu,lockMinScore=%ld,enableCapComputation=%u,"
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
           capConfig.enableCapComputation ? 1u : 0u,
           capConfig.enableNetCapOutput ? 1u : 0u,
           capConfig.inductorValueUh,
           capConfig.fixedCapPf,
           capConfig.parasiticCapPf);
    printf("DBGFDC_S5D5,stage=route_semantics,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n");
    printf("DBGFDC_S5D5,stage=sweep_plan,highCurrent=0|1,driveCurrentList=0xA000|0xB800|0xC000|0xD000|0xE000|0xF800,"
           "stepSettleMs=%u,sampleGapMs=%u,unreadPollTimeoutMs=%lu,unreadPollIntervalMs=%lu,inductorUh=%.3f,"
           "fixedCapPf=%.3f,parasiticCapPf=%.3f\n",
           (unsigned)SENSORARRAY_S5D5_STEP_SETTLE_MS,
           (unsigned)SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS,
           (unsigned long)unreadPollTimeoutMs,
           (unsigned long)unreadPollIntervalMs,
           capConfig.inductorValueUh,
           capConfig.fixedCapPf,
           capConfig.parasiticCapPf);

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
        }

        if (!locked) {
            const char *routeMapLabel = SENSORARRAY_NA;
            esp_err_t routeErr = sensorarrayMeasureApplyCapacitiveRoute(state,
                                                                        SENSORARRAY_S5,
                                                                        SENSORARRAY_D5,
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
            bool haveBest = sensorarrayS5d5SweepCandidates(fdcState,
                                                           fdcMap,
                                                           sweepSampleCount,
                                                           discardFirst,
                                                           lockMinScore,
                                                           &checkpoint,
                                                           &bestCandidate);
            if (!haveBest || bestCandidate.score < lockMinScore) {
                const char *reason = haveBest ? "score_below_min" : "no_working_candidate";
                printf("DBGFDC_S5D5,stage=lock_failed,reason=%s,bestScore=%ld,minScore=%ld,status=error\n",
                       reason,
                       haveBest ? (long)bestCandidate.score : (long)INT_MIN,
                       (long)lockMinScore);
                sensorarrayDebugSelftestDelayMs(loopDelayMs);
                continue;
            }

            esp_err_t lockErr = sensorarrayS5d5ApplyLockedCandidate(fdcState->handle, &bestCandidate, fdcMap->channel);
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

            uint32_t discardCountAfterLock = discardFirst ? 2u : 1u;
            for (uint32_t i = 0u; i < discardCountAfterLock; ++i) {
                sensorarrayFdcReadDiag_t throwawayDiag = {0};
                esp_err_t discardErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(fdcState->handle,
                                                                                   fdcMap->channel,
                                                                                   false,
                                                                                   fdcState->haveIds,
                                                                                   fdcState->configVerified,
                                                                                   &throwawayDiag);
                printf("DBGFDC_S5D5,stage=lock_apply,discardIndex=%lu,err=%ld,status=%s\n",
                       (unsigned long)i,
                       (long)discardErr,
                       (discardErr == ESP_OK) ? "discard_done" : "discard_read_error_continue");
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
            }

            lockedCandidate = bestCandidate;
            lockEpoch++;
            locked = true;
            i2cFailureStreak = 0u;
            nonConvertingStreak = 0u;
            readbackMismatchStreak = 0u;
        }

        sensorarrayS5d5LockedSummary_t lockedSummary = {0};
        bool needRecovery = false;
        const char *recoveryReason = SENSORARRAY_NA;
        uint32_t recoveryStreakCount = 0u;

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
                           "raw=0,freqHz=0.000,totalCapPf=na(reason=i2c_error),netCapPf=na(reason=i2c_error),"
                           "inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,converting=0,wd=0,aw=0,"
                           "sampleQuality=warning,status=i2c_read_error\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           waitStatus.UnreadConversion[(uint8_t)fdcMap->channel] ? 1u : 0u);
                } else {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "raw=0,freqHz=0.000,totalCapPf=na(reason=i2c_error),inductorUh=%.3f,fixedCapPf=%.3f,"
                           "parasiticCapPf=%.3f,unread=%u,converting=0,wd=0,aw=0,sampleQuality=warning,"
                           "status=i2c_read_error\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           waitStatus.UnreadConversion[(uint8_t)fdcMap->channel] ? 1u : 0u);
                }
                if (i2cFailureStreak >= recoveryReinitThreshold && !needRecovery) {
                    needRecovery = true;
                    recoveryReason = "i2c_error_streak";
                    recoveryStreakCount = i2cFailureStreak;
                }
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
                continue;
            }

            i2cFailureStreak = 0u;
            bool activeChannelMatch = (diag.sample.ActiveChannel == fdcMap->channel);
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

            if (nonConvertingStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "non_converting_streak";
                recoveryStreakCount = nonConvertingStreak;
            }
            if (readbackMismatchStreak >= recoveryReinitThreshold && !needRecovery) {
                needRecovery = true;
                recoveryReason = "config_readback_mismatch";
                recoveryStreakCount = readbackMismatchStreak;
            }

            bool qualityGood = diag.provisionalReadable &&
                               waitReady &&
                               diag.sample.UnreadConversionPresent &&
                               !diag.sample.ErrWatchdog &&
                               !diag.sample.ErrAmplitude &&
                               activeChannelMatch;
            if (qualityGood) {
                lockedSummary.goodSamples++;
            } else {
                lockedSummary.warningSamples++;
            }
            if (diag.sample.ErrAmplitude) {
                lockedSummary.amplitudeFaultSamples++;
            }

            uint32_t refClockHz = (fdcState->refClockHz > 0u) ? fdcState->refClockHz : SENSORARRAY_FDC_REF_CLOCK_HZ;
            double freqHz = sensorarrayMeasureFdcRawToFrequencyHz(diag.sample.Raw28, refClockHz);
            if (freqHz > 0.0) {
                lockedSummary.freqHzSum += freqHz;
                lockedSummary.freqSampleCount++;
            }

            double totalCapPf = 0.0;
            double netCapPf = 0.0;
            bool haveNetCapPf = false;
            const char *capReason = SENSORARRAY_NA;
            bool haveTotalCapPf = sensorarrayS5d5TryComputeCapacitance(&capConfig,
                                                                        freqHz,
                                                                        &totalCapPf,
                                                                        &netCapPf,
                                                                        &haveNetCapPf,
                                                                        &capReason);
            if (haveTotalCapPf) {
                lockedSummary.totalCapPfSum += totalCapPf;
                lockedSummary.totalCapSampleCount++;
                if (haveNetCapPf) {
                    lockedSummary.netCapPfSum += netCapPf;
                    lockedSummary.netCapSampleCount++;
                }
            }
            const char *statusName = sensorarrayMeasureFdcSampleStatusName(diag.statusCode);

            lockedSampleGlobalIndex++;
            if (haveTotalCapPf) {
                if (haveNetCapPf) {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "raw=%lu,freqHz=%.3f,totalCapPf=%.3f,netCapPf=%.3f,inductorUh=%.3f,fixedCapPf=%.3f,"
                           "parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned long)diag.sample.Raw28,
                           freqHz,
                           totalCapPf,
                           netCapPf,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           qualityGood ? "good" : "warning",
                           statusName);
                } else {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "raw=%lu,freqHz=%.3f,totalCapPf=%.3f,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,"
                           "unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned long)diag.sample.Raw28,
                           freqHz,
                           totalCapPf,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           qualityGood ? "good" : "warning",
                           statusName);
                }
            } else {
                if (capConfig.enableNetCapOutput) {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "raw=%lu,freqHz=%.3f,totalCapPf=na(reason=%s),netCapPf=na(reason=%s),inductorUh=%.3f,"
                           "fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,"
                           "status=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned long)diag.sample.Raw28,
                           freqHz,
                           capReason,
                           capReason,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           qualityGood ? "good" : "warning",
                           statusName);
                } else {
                    printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                           "raw=%lu,freqHz=%.3f,totalCapPf=na(reason=%s),inductorUh=%.3f,fixedCapPf=%.3f,"
                           "parasiticCapPf=%.3f,unread=%u,converting=%u,wd=%u,aw=%u,sampleQuality=%s,status=%s\n",
                           (unsigned long)lockedSampleGlobalIndex,
                           (unsigned long)lockEpoch,
                           lockedCandidate.highCurrentReq ? 1u : 0u,
                           lockedCandidate.driveCurrentNorm,
                           (unsigned long)diag.sample.Raw28,
                           freqHz,
                           capReason,
                           capConfig.inductorValueUh,
                           capConfig.fixedCapPf,
                           capConfig.parasiticCapPf,
                           diag.sample.UnreadConversionPresent ? 1u : 0u,
                           diag.sample.Converting ? 1u : 0u,
                           diag.sample.ErrWatchdog ? 1u : 0u,
                           diag.sample.ErrAmplitude ? 1u : 0u,
                           qualityGood ? "good" : "warning",
                           statusName);
                }
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
                   "amplitudeFault=%lu,i2cErr=%lu,waitTimeout=%lu,nonConverting=%lu,avgFreqHz=%s,"
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
                   avgFreqHzField,
                   avgTotalCapPfField,
                   avgNetCapPfField,
                   capConfig.inductorValueUh,
                   capConfig.fixedCapPf,
                   capConfig.parasiticCapPf,
                   (unsigned long)i2cFailureStreak,
                   (unsigned long)nonConvertingStreak,
                   (unsigned long)readbackMismatchStreak,
                   needRecovery ? "recovery_pending" : "locked_continue");
        } else {
            printf("DBGFDC_S5D5,stage=locked_summary,lockEpoch=%lu,samples=%lu,good=%lu,warning=%lu,"
                   "amplitudeFault=%lu,i2cErr=%lu,waitTimeout=%lu,nonConverting=%lu,avgFreqHz=%s,"
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
                   avgFreqHzField,
                   avgTotalCapPfField,
                   capConfig.inductorValueUh,
                   capConfig.fixedCapPf,
                   capConfig.parasiticCapPf,
                   (unsigned long)i2cFailureStreak,
                   (unsigned long)nonConvertingStreak,
                   (unsigned long)readbackMismatchStreak,
                   needRecovery ? "recovery_pending" : "locked_continue");
        }

        if (needRecovery) {
            esp_err_t recoveryErr = sensorarrayS5d5DoRecoveryReinit(state,
                                                                     fdcState,
                                                                     &checkpoint,
                                                                     recoveryReason,
                                                                     recoveryStreakCount);
            locked = false;
            modeFdcInitialized = (recoveryErr == ESP_OK);
            if (recoveryErr != ESP_OK) {
                sensorarrayDebugSelftestDelayMs(SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS);
            }
            continue;
        }

        sensorarrayDebugSelftestDelayMs(loopDelayMs);
    }
}
