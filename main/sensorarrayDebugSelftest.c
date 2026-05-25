#include "sensorarrayDebugSelftest.h"

#include <limits.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#if CONFIG_ESP_TASK_WDT_EN
#include "esp_task_wdt.h"
#endif
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
#define SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS 20u
#define SENSORARRAY_S5D5_ROUND_FAIL_DELAY_MS 500u
#define SENSORARRAY_S5D5_RELOCK_DISCARD_SAMPLES 2u
#define SENSORARRAY_S5D5_TOP_PROFILE_CACHE_COUNT 5u
#define SENSORARRAY_S5D5_BASELINE_FREQ_MARGIN 1.05
#define SENSORARRAY_S5D5_BASELINE_FAILED_LOW_DEGLITCH_TIMEOUT_MS 75u
#define SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ 0x5u
#define SENSORARRAY_S5D5_DEFAULT_DEGLITCH_BW_HZ 10000000u
#define SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ \
    ((uint16_t)(SENSORARRAY_FDC_DEBUG_DRIVE_CURRENT_CH0 & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK))
#define SENSORARRAY_S5D5_DIRECT_SAFE_DRIVE_CURRENT_REQ 0x7800u
#define SENSORARRAY_S5D5_DEFAULT_CONFIG_REG 0x1481u
#define SENSORARRAY_S5D5_DEFAULT_MUX_CONFIG_REG 0x020Du
#define SENSORARRAY_S5D5_DEFAULT_CLOCK_DIVIDERS_REG 0x2001u

typedef struct {
    uint8_t code;
    uint32_t bandwidthHz;
    const char *name;
} sensorarrayS5d5DeglitchCandidate_t;

static const uint16_t SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[] = {
    SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ,
    0xA000u,
    0xB800u,
    0xC000u,
    0xD000u,
    0xE000u,
    0xF800u,
};

static const sensorarrayS5d5DeglitchCandidate_t SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[] = {
    {0x1u, 1000000u, "1MHz"},
    {0x4u, 3300000u, "3p3MHz"},
    {0x5u, 10000000u, "10MHz"},
    {0x7u, 33000000u, "33MHz"},
};

static const sensorarrayS5d5DeglitchCandidate_t *sensorarrayS5d5FindDeglitchCandidate(uint8_t deglitchCode)
{
    size_t count = sizeof(SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE) /
                   sizeof(SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[0]);
    for (size_t i = 0u; i < count; ++i) {
        if (SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i].code ==
            (uint8_t)(deglitchCode & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK)) {
            return &SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i];
        }
    }
    return NULL;
}

static const char *sensorarrayS5d5DeglitchNameFromCode(uint8_t deglitchCode)
{
    const sensorarrayS5d5DeglitchCandidate_t *candidate =
        sensorarrayS5d5FindDeglitchCandidate(deglitchCode);
    return candidate ? candidate->name : SENSORARRAY_NA;
}

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

static uint32_t sensorarrayS5d5NowMs(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000LL);
}

static TickType_t sensorarrayS5d5MsToTicksAtLeastOne(uint32_t delayMs)
{
    TickType_t ticks = pdMS_TO_TICKS(delayMs);
    return (ticks == 0u) ? 1u : ticks;
}

static void sensorarrayS5d5ServiceScheduler(void)
{
#if CONFIG_ESP_TASK_WDT_EN
    esp_err_t wdtErr = esp_task_wdt_reset();
    if (wdtErr != ESP_OK && wdtErr != ESP_ERR_NOT_FOUND) {
        (void)wdtErr;
    }
#endif
    vTaskDelay(sensorarrayS5d5MsToTicksAtLeastOne(1u));
}

static void sensorarrayS5d5DelayCooperativeMs(uint32_t delayMs)
{
    uint32_t remainingMs = delayMs;
    while (remainingMs > 0u) {
        uint32_t chunkMs = (remainingMs > 10u) ? 10u : remainingMs;
        vTaskDelay(sensorarrayS5d5MsToTicksAtLeastOne(chunkMs));
        sensorarrayS5d5ServiceScheduler();
        remainingMs -= chunkMs;
    }
}

static void sensorarrayDebugSelftestDelayMs(uint32_t delayMs)
{
    sensorarrayS5d5DelayCooperativeMs(delayMs);
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
    const char *deglitchName = sensorarrayS5d5DeglitchNameFromCode(deglitchCode);
    printf("DBGFDC_S5D5,stage=apply_deglitch_begin,deglitch=%s,deglitchReq=0x%X\n",
           deglitchName,
           (unsigned)(deglitchCode & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK));

    if (!dev) {
        printf("DBGFDC_S5D5,stage=apply_deglitch_done,err=%ld,muxConfigReadback=0x%04X,status=invalid_arg\n",
               (long)ESP_ERR_INVALID_ARG,
               0u);
        return ESP_ERR_INVALID_ARG;
    }

    switch (deglitchCode) {
    case 0x1u:
    case 0x4u:
    case 0x5u:
    case 0x7u:
        break;
    default:
        printf("DBGFDC_S5D5,stage=apply_deglitch_done,err=%ld,muxConfigReadback=0x%04X,status=invalid_deglitch\n",
               (long)ESP_ERR_INVALID_ARG,
               0u);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t muxConfigReg = 0u;
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, &muxConfigReg);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=apply_deglitch_done,err=%ld,muxConfigReadback=0x%04X,status=read_failed\n",
               (long)err,
               muxConfigReg);
        return err;
    }

    muxConfigReg &= (uint16_t)~SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK;
    muxConfigReg |= (uint16_t)(deglitchCode & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK);
    err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, muxConfigReg);

    uint16_t muxConfigReadback = 0u;
    esp_err_t readbackErr = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, &muxConfigReadback);
    bool match = (readbackErr == ESP_OK) &&
                 ((muxConfigReadback & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK) ==
                  (deglitchCode & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK));
    esp_err_t logErr = (err != ESP_OK) ? err : readbackErr;
    const char *status = (err != ESP_OK) ? "write_failed" :
                         ((readbackErr != ESP_OK) ? "readback_failed" :
                          (match ? "applied" : "warning_readback_mismatch"));
    printf("DBGFDC_S5D5,stage=apply_deglitch_done,err=%ld,muxConfigReadback=0x%04X,status=%s\n",
           (long)logErr,
           muxConfigReadback,
           status);
    return (err != ESP_OK) ? err : readbackErr;
}

static esp_err_t sensorarrayApplyS5d5DriveStep(Fdc2214CapDevice_t *dev, bool highCurrent, uint16_t driveCurrentReq)
{
    uint16_t driveCurrentNorm = (uint16_t)(driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    printf("DBGFDC_S5D5,stage=apply_drive_begin,highCurrent=%u,driveReq=0x%04X,driveNorm=0x%04X\n",
           highCurrent ? 1u : 0u,
           driveCurrentReq,
           driveCurrentNorm);

    if (!dev) {
        printf("DBGFDC_S5D5,stage=apply_drive_done,err=%ld,configReadback=0x%04X,driveReadback=0x%04X,"
               "driveNorm=0x%04X,status=invalid_arg\n",
               (long)ESP_ERR_INVALID_ARG,
               0u,
               0u,
               driveCurrentNorm);
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t configReg = 0u;
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_CONFIG, &configReg);
    if (err != ESP_OK) {
        printf("DBGFDC_S5D5,stage=apply_drive_done,err=%ld,configReadback=0x%04X,driveReadback=0x%04X,"
               "driveNorm=0x%04X,status=config_read_failed\n",
               (long)err,
               configReg,
               0u,
               driveCurrentNorm);
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
        printf("DBGFDC_S5D5,stage=apply_drive_done,err=%ld,configReadback=0x%04X,driveReadback=0x%04X,"
               "driveNorm=0x%04X,status=config_write_failed\n",
               (long)err,
               configReg,
               0u,
               driveCurrentNorm);
        return err;
    }

    err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, driveCurrentNorm);

    uint16_t configReadback = 0u;
    uint16_t driveReadbackRaw = 0u;
    esp_err_t configReadbackErr = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_CONFIG, &configReadback);
    esp_err_t driveReadbackErr =
        Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, &driveReadbackRaw);
    uint16_t driveReadback = (uint16_t)(driveReadbackRaw & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    bool highCurrentReadback = (configReadback & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    bool match = (configReadbackErr == ESP_OK) &&
                 (driveReadbackErr == ESP_OK) &&
                 (highCurrentReadback == highCurrent) &&
                 (driveReadback == driveCurrentNorm);
    esp_err_t logErr = (err != ESP_OK) ? err :
                       ((configReadbackErr != ESP_OK) ? configReadbackErr : driveReadbackErr);
    const char *status = (err != ESP_OK) ? "drive_write_failed" :
                         ((configReadbackErr != ESP_OK || driveReadbackErr != ESP_OK)
                              ? "readback_failed"
                              : (match ? "applied" : "warning_readback_mismatch"));
    printf("DBGFDC_S5D5,stage=apply_drive_done,err=%ld,configReadback=0x%04X,driveReadback=0x%04X,"
           "driveNorm=0x%04X,status=%s\n",
           (long)logErr,
           configReadback,
           driveReadback,
           driveCurrentNorm,
           status);
    if (err != ESP_OK) {
        return err;
    }
    if (configReadbackErr != ESP_OK) {
        return configReadbackErr;
    }
    return driveReadbackErr;
}

static esp_err_t sensorarrayS5d5ApplyDirectSafeLock(Fdc2214CapDevice_t *dev)
{
    if (!dev) {
        printf("DBGFDC_S5D5,stage=direct_lock_failed,err=%ld,reason=invalid_dev\n",
               (long)ESP_ERR_INVALID_ARG);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t firstErr = ESP_OK;
    const uint16_t muxConfigReq = SENSORARRAY_S5D5_DEFAULT_MUX_CONFIG_REG;
    const uint16_t configReq =
        (uint16_t)((SENSORARRAY_S5D5_DEFAULT_CONFIG_REG &
                    (uint16_t)~(SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK |
                                SENSORARRAY_S5D5_CONFIG_SLEEP_MODE_EN_MASK |
                                SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK)) |
                   ((uint16_t)FDC2214_CH0 << SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT));
    const uint16_t driveReq =
        (uint16_t)(SENSORARRAY_S5D5_DIRECT_SAFE_DRIVE_CURRENT_REQ & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);

    printf("DBGFDC_S5D5,stage=direct_apply_deglitch_begin,code=0x%X,muxConfigReq=0x%04X\n",
           (unsigned)SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ,
           muxConfigReq);
    esp_err_t err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, muxConfigReq);
    uint16_t muxConfigReadback = 0u;
    esp_err_t readErr = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_MUX_CONFIG, &muxConfigReadback);
    if (err == ESP_OK && readErr != ESP_OK) {
        err = readErr;
    }
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    printf("DBGFDC_S5D5,stage=direct_apply_deglitch_done,err=%ld,muxConfig=0x%04X,deglitch=0x%X\n",
           (long)err,
           muxConfigReadback,
           (unsigned)(muxConfigReadback & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK));

    printf("DBGFDC_S5D5,stage=direct_apply_config_begin,configReq=0x%04X\n", configReq);
    err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_CONFIG, configReq);
    uint16_t configReadback = 0u;
    readErr = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_CONFIG, &configReadback);
    if (err == ESP_OK && readErr != ESP_OK) {
        err = readErr;
    }
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    printf("DBGFDC_S5D5,stage=direct_apply_config_done,err=%ld,config=0x%04X,activeChannel=%u,"
           "sleep=%u,highCurrent=%u\n",
           (long)err,
           configReadback,
           (unsigned)((configReadback & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >>
                      SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT),
           (configReadback & SENSORARRAY_S5D5_CONFIG_SLEEP_MODE_EN_MASK) ? 1u : 0u,
           (configReadback & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) ? 1u : 0u);

    printf("DBGFDC_S5D5,stage=direct_apply_drive_begin,drive=0x%04X\n", driveReq);
    err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, driveReq);
    uint16_t driveReadbackRaw = 0u;
    readErr = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_S5D5_REG_DRIVE_CURRENT_CH0, &driveReadbackRaw);
    uint16_t driveReadback = (uint16_t)(driveReadbackRaw & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    if (err == ESP_OK && readErr != ESP_OK) {
        err = readErr;
    }
    if (firstErr == ESP_OK && err != ESP_OK) {
        firstErr = err;
    }
    printf("DBGFDC_S5D5,stage=direct_apply_drive_done,err=%ld,driveReadback=0x%04X\n",
           (long)err,
           driveReadback);

    if (firstErr != ESP_OK) {
        printf("DBGFDC_S5D5,stage=direct_lock_failed,err=%ld,status=continue_to_locked_loop\n",
               (long)firstErr);
    }
    return firstErr;
}

typedef struct {
    bool valid;
    uint8_t deglitchReq;
    const char *deglitchName;
    uint32_t deglitchBandwidthHz;
    bool deglitchBandwidthOk;
    double deglitchMarginRatio;

    bool highCurrentReq;
    bool highCurrentReadback;
    uint16_t driveCurrentReq;
    uint16_t driveCurrentNorm;
    uint16_t driveCurrentReadback;

    uint8_t activeChannelReadback;
    uint16_t statusReg;
    uint16_t configReg;
    uint16_t muxConfigReg;
    uint16_t clockDividersReg;

    uint8_t finSelCode;
    uint8_t finFactor;
    uint16_t frefDivider;
    uint32_t effectiveFclkHz;
    double effectiveFrefHz;
    bool clockDecodeValid;
    const char *clockStatus;

    uint32_t samples;
    uint32_t i2cErr;
    uint32_t convertingOk;
    uint32_t unreadOk;
    uint32_t watchdog;
    uint32_t amplitude;
    uint32_t nonZeroRaw;
    uint32_t rawMin;
    uint32_t rawMax;
    uint64_t rawSum;
    uint32_t rawMean;
    uint32_t rawSpan;
    double avgFreqHz;

    int32_t score;
    bool working;
    const char *status;
} sensorarrayS5d5Profile_t;

typedef struct {
    bool bootSweepDone;
    bool haveBootBest;
    bool haveLastGood;
    sensorarrayS5d5Profile_t bootBest;
    sensorarrayS5d5Profile_t lastGood;
    sensorarrayS5d5Profile_t topProfiles[SENSORARRAY_S5D5_TOP_PROFILE_CACHE_COUNT];

    uint32_t lockEpoch;
    uint32_t sampleIndex;
    uint32_t samplesSinceRelock;
    uint32_t lastRelockMs;
    uint32_t lastFullSweepMs;

    bool haveFreqAnchor;
    double lockedFreqHz;
    uint32_t largeFreqJumpStreak;

    uint32_t i2cErrStreak;
    uint32_t waitTimeoutStreak;
    uint32_t nonConvertingStreak;
    uint32_t amplitudeFaultStreak;
    uint32_t readbackMismatchStreak;
} sensorarrayS5d5Runtime_t;

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
    double freqHzSum;
    double totalCapPfSum;
    double netCapPfSum;
} sensorarrayS5d5LockedSummary_t;

static void sensorarrayS5d5AddUniqueDrive(uint16_t *drives,
                                           uint32_t *count,
                                           uint32_t capacity,
                                           uint16_t drive);

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

static void sensorarrayS5d5MakeProfile(sensorarrayS5d5Profile_t *profile,
                                        const sensorarrayS5d5DeglitchCandidate_t *deglitch,
                                        bool highCurrentReq,
                                        uint16_t driveCurrentReq)
{
    if (!profile || !deglitch) {
        return;
    }

    *profile = (sensorarrayS5d5Profile_t){
        .valid = true,
        .deglitchReq = deglitch->code,
        .deglitchName = deglitch->name,
        .deglitchBandwidthHz = deglitch->bandwidthHz,
        .highCurrentReq = highCurrentReq,
        .driveCurrentReq = driveCurrentReq,
        .driveCurrentNorm = (uint16_t)(driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .rawMin = UINT_MAX,
        .clockStatus = "not_read",
        .status = "candidate",
    };
}

static void sensorarrayS5d5MakeDefaultSafeProfile(sensorarrayS5d5Profile_t *profile)
{
    if (!profile) {
        return;
    }

    *profile = (sensorarrayS5d5Profile_t){
        .valid = true,
        .deglitchReq = SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ,
        .deglitchName = "10MHz",
        .deglitchBandwidthHz = SENSORARRAY_S5D5_DEFAULT_DEGLITCH_BW_HZ,
        .deglitchBandwidthOk = true,
        .highCurrentReq = false,
        .driveCurrentReq = SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ,
        .driveCurrentNorm = (uint16_t)(SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ &
                                       SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .configReg = SENSORARRAY_S5D5_DEFAULT_CONFIG_REG,
        .muxConfigReg = SENSORARRAY_S5D5_DEFAULT_MUX_CONFIG_REG,
        .clockDividersReg = SENSORARRAY_S5D5_DEFAULT_CLOCK_DIVIDERS_REG,
        .rawMin = UINT_MAX,
        .clockStatus = "default_safe",
        .status = "default_safe",
    };
}

static void sensorarrayS5d5PrepareCapture(sensorarrayS5d5Profile_t *profile,
                                           const sensorarrayS5d5Profile_t *base)
{
    if (!profile || !base) {
        return;
    }

    *profile = (sensorarrayS5d5Profile_t){
        .valid = base->valid,
        .deglitchReq = base->deglitchReq,
        .deglitchName = base->deglitchName,
        .deglitchBandwidthHz = base->deglitchBandwidthHz,
        .highCurrentReq = base->highCurrentReq,
        .driveCurrentReq = base->driveCurrentReq,
        .driveCurrentNorm = (uint16_t)(base->driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .rawMin = UINT_MAX,
        .clockStatus = "not_read",
        .status = "capture_pending",
    };
}

static void sensorarrayS5d5FinalizeRawStats(sensorarrayS5d5Profile_t *profile)
{
    if (!profile) {
        return;
    }

    if (profile->nonZeroRaw > 0u) {
        profile->rawMean = (uint32_t)(profile->rawSum / profile->nonZeroRaw);
        profile->rawSpan = profile->rawMax - profile->rawMin;
    } else {
        profile->rawMin = 0u;
        profile->rawMax = 0u;
        profile->rawMean = 0u;
        profile->rawSpan = 0u;
    }
}

static void sensorarrayS5d5ProfileSetClockReadback(sensorarrayS5d5Profile_t *profile,
                                                   uint16_t clockDividers)
{
    if (!profile) {
        return;
    }

    const char *clockStatus = "unknown";
    profile->clockDividersReg = clockDividers;
    profile->effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz();
    profile->clockDecodeValid = sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                                          &profile->finSelCode,
                                                                          &profile->finFactor,
                                                                          &profile->frefDivider,
                                                                          &clockStatus);
    profile->clockStatus = clockStatus ? clockStatus : SENSORARRAY_NA;
    profile->effectiveFrefHz = (profile->clockDecodeValid && profile->frefDivider > 0u)
                                   ? ((double)profile->effectiveFclkHz / (double)profile->frefDivider)
                                   : 0.0;

    profile->avgFreqHz = 0.0;
    profile->deglitchBandwidthOk = false;
    profile->deglitchMarginRatio = 0.0;
    if (profile->rawMean > 0u && profile->clockDecodeValid) {
        sensorarrayFdcFrequencyDiag_t freqDiag = {0};
        if (sensorarrayMeasureFdcComputeFrequencyDiag(profile->rawMean, clockDividers, &freqDiag) && freqDiag.valid) {
            profile->avgFreqHz = freqDiag.freqHzCorrected;
            if (profile->deglitchBandwidthHz > 0u && profile->avgFreqHz > 0.0) {
                profile->deglitchMarginRatio = (double)profile->deglitchBandwidthHz / profile->avgFreqHz;
                profile->deglitchBandwidthOk = (profile->avgFreqHz < ((double)profile->deglitchBandwidthHz * 0.90));
            }
        }
    }
}

static bool sensorarrayS5d5CandidateIsWorking(const sensorarrayS5d5Profile_t *p)
{
    if (!p || !p->valid || p->samples == 0u) {
        return false;
    }

    uint32_t minOk = (p->samples > 0u) ? (p->samples - 1u) : 0u;
    if (p->i2cErr != 0u || p->watchdog != 0u) {
        return false;
    }
    if (p->nonZeroRaw != p->samples) {
        return false;
    }
    if (p->convertingOk < minOk || p->unreadOk < minOk) {
        return false;
    }
    if (!p->deglitchBandwidthOk || !p->clockDecodeValid) {
        return false;
    }
    if (p->amplitude > (p->samples / 2u)) {
        return false;
    }
    return true;
}

static int32_t sensorarrayS5d5ScoreCandidate(const sensorarrayS5d5Profile_t *p)
{
    if (!p) {
        return INT_MIN;
    }

    int32_t score = sensorarrayS5d5CandidateIsWorking(p) ? 300 : 0;
    uint32_t missingNonZero = (p->samples > p->nonZeroRaw) ? (p->samples - p->nonZeroRaw) : 0u;
    uint32_t missingConverting = (p->samples > p->convertingOk) ? (p->samples - p->convertingOk) : 0u;
    uint32_t missingUnread = (p->samples > p->unreadOk) ? (p->samples - p->unreadOk) : 0u;

    score -= (int32_t)(p->i2cErr * 1000u);
    score -= (int32_t)(p->watchdog * 1000u);
    score -= (int32_t)(missingNonZero * 500u);
    score -= (int32_t)((missingConverting + missingUnread) * 300u);
    score -= (int32_t)(p->amplitude * 80u);
    score += (int32_t)(p->convertingOk * 25u);
    score += (int32_t)(p->unreadOk * 20u);
    score += (int32_t)(p->nonZeroRaw * 25u);

    if (p->rawMean > 0u) {
        uint32_t spanPermille = (uint32_t)(((uint64_t)p->rawSpan * 1000ull) / p->rawMean);
        if (spanPermille <= 5u) {
            score += 35;
        } else if (spanPermille <= 20u) {
            score += 20;
        } else if (spanPermille <= 80u) {
            score += 5;
        } else {
            uint32_t penalty = spanPermille / 4u;
            if (penalty > 120u) {
                penalty = 120u;
            }
            score -= (int32_t)penalty;
        }
    }

    if (p->highCurrentReq) {
        score -= 25;
    }
    if (p->deglitchMarginRatio > 0.0) {
        if (p->deglitchMarginRatio < 1.15) {
            score -= 120;
        } else if (p->deglitchMarginRatio < 1.50) {
            score -= 40;
        } else if (p->deglitchMarginRatio > 12.0) {
            score -= 15;
        }
    } else {
        score -= 200;
    }
    if (p->deglitchReq == SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ) {
        score += 15;
    }
    if (p->driveCurrentNorm == SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ) {
        score += 10;
    }
    return score;
}

static const char *sensorarrayS5d5ProfileStatus(const sensorarrayS5d5Profile_t *p)
{
    if (!p || !p->valid) {
        return "invalid";
    }
    if (p->samples == 0u) {
        return "no_samples";
    }
    if (p->i2cErr != 0u) {
        return "i2c_or_timeout";
    }
    if (p->watchdog != 0u) {
        return "watchdog_fault";
    }
    if (p->nonZeroRaw != p->samples) {
        return "raw_zero_or_stale";
    }
    if (p->convertingOk < (p->samples - 1u)) {
        return "not_converting";
    }
    if (p->unreadOk < (p->samples - 1u)) {
        return "no_unread_conversions";
    }
    if (!p->clockDecodeValid) {
        return p->clockStatus ? p->clockStatus : "invalid_clock";
    }
    if (!p->deglitchBandwidthOk) {
        return "deglitch_bandwidth_low";
    }
    if (p->amplitude > (p->samples / 2u)) {
        return "amplitude_fault";
    }
    return sensorarrayS5d5CandidateIsWorking(p) ? "working" : "not_working";
}

static bool sensorarrayS5d5ProfileBetter(const sensorarrayS5d5Profile_t *candidate,
                                          const sensorarrayS5d5Profile_t *best)
{
    if (!candidate || !candidate->valid) {
        return false;
    }
    if (!best || !best->valid) {
        return true;
    }

    bool candidateWorking = sensorarrayS5d5CandidateIsWorking(candidate);
    bool bestWorking = sensorarrayS5d5CandidateIsWorking(best);
    if (candidateWorking != bestWorking) {
        return candidateWorking;
    }
    if (candidate->score != best->score) {
        return candidate->score > best->score;
    }
    if (candidate->amplitude != best->amplitude) {
        return candidate->amplitude < best->amplitude;
    }
    if (candidate->rawSpan != best->rawSpan) {
        return candidate->rawSpan < best->rawSpan;
    }
    if (candidate->highCurrentReq != best->highCurrentReq) {
        return !candidate->highCurrentReq;
    }
    if (candidate->deglitchBandwidthHz != best->deglitchBandwidthHz) {
        return candidate->deglitchBandwidthHz < best->deglitchBandwidthHz;
    }
    return candidate->driveCurrentNorm > best->driveCurrentNorm;
}

static void sensorarrayS5d5IngestDiag(sensorarrayS5d5Profile_t *profile,
                                      const sensorarrayFdcReadDiag_t *diag)
{
    if (!profile || !diag) {
        return;
    }

    profile->samples++;
    if (diag->err != ESP_OK || !diag->i2cOk) {
        profile->i2cErr++;
        return;
    }
    if (diag->sample.Converting) {
        profile->convertingOk++;
    }
    if (diag->sample.UnreadConversionPresent) {
        profile->unreadOk++;
    }
    if (diag->sample.ErrWatchdog) {
        profile->watchdog++;
    }
    if (diag->sample.ErrAmplitude) {
        profile->amplitude++;
    }
    if (diag->sample.Raw28 != 0u) {
        profile->nonZeroRaw++;
        if (diag->sample.Raw28 < profile->rawMin) {
            profile->rawMin = diag->sample.Raw28;
        }
        if (diag->sample.Raw28 > profile->rawMax) {
            profile->rawMax = diag->sample.Raw28;
        }
        profile->rawSum += diag->sample.Raw28;
    }
}

static esp_err_t sensorarrayS5d5ReadbackAndFinalizeProfile(Fdc2214CapDevice_t *dev,
                                                            uint8_t channel,
                                                            sensorarrayS5d5Profile_t *profile)
{
    if (!dev || !profile) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayS5d5FinalizeRawStats(profile);

    uint16_t statusReg = 0u;
    uint16_t configReg = 0u;
    uint16_t muxConfigReg = 0u;
    uint16_t clockDividersReg = 0u;
    uint16_t driveCurrentReg = 0u;
    esp_err_t err = sensorarrayS5d5ReadKeyRegs(dev,
                                               (Fdc2214CapChannel_t)channel,
                                               &statusReg,
                                               &configReg,
                                               &muxConfigReg,
                                               &clockDividersReg,
                                               &driveCurrentReg);
    if (err == ESP_OK) {
        profile->statusReg = statusReg;
        profile->configReg = configReg;
        profile->muxConfigReg = muxConfigReg;
        profile->highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
        profile->driveCurrentReadback = (uint16_t)(driveCurrentReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
        profile->activeChannelReadback =
            (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >>
                      SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT);
        sensorarrayS5d5ProfileSetClockReadback(profile, clockDividersReg);
    } else {
        profile->i2cErr++;
        profile->clockStatus = "readback_failed";
        profile->clockDecodeValid = false;
        profile->deglitchBandwidthOk = false;
    }

    profile->score = sensorarrayS5d5ScoreCandidate(profile);
    profile->working = sensorarrayS5d5CandidateIsWorking(profile);
    profile->status = sensorarrayS5d5ProfileStatus(profile);
    return err;
}

static esp_err_t sensorarrayS5d5ApplyProfile(Fdc2214CapDevice_t *dev,
                                             const sensorarrayS5d5Profile_t *profile,
                                             const char *stage)
{
    if (!dev || !profile || !profile->valid) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *source = stage ? stage : SENSORARRAY_NA;
    esp_err_t applyErr = sensorarrayApplyS5d5DeglitchStep(dev, profile->deglitchReq);
    if (applyErr == ESP_OK) {
        applyErr = sensorarrayApplyS5d5DriveStep(dev, profile->highCurrentReq, profile->driveCurrentReq);
    }

    uint16_t statusReg = 0u;
    uint16_t configReg = 0u;
    uint16_t muxConfigReg = 0u;
    uint16_t clockDividersReg = 0u;
    uint16_t driveCurrentReg = 0u;
    esp_err_t readErr = sensorarrayS5d5ReadKeyRegs(dev,
                                                   FDC2214_CH0,
                                                   &statusReg,
                                                   &configReg,
                                                   &muxConfigReg,
                                                   &clockDividersReg,
                                                   &driveCurrentReg);
    bool highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    uint16_t driveCurrentReadback = (uint16_t)(driveCurrentReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    uint8_t deglitchReadback = (uint8_t)(muxConfigReg & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK);
    bool match = (readErr == ESP_OK) &&
                 (deglitchReadback == (uint8_t)(profile->deglitchReq & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK)) &&
                 (highCurrentReadback == profile->highCurrentReq) &&
                 (driveCurrentReadback == profile->driveCurrentNorm);
    esp_err_t logErr = (applyErr != ESP_OK) ? applyErr : readErr;
    const char *status = (applyErr != ESP_OK) ? "apply_failed" :
                         ((readErr != ESP_OK) ? "readback_failed" : (match ? "applied" : "warning_readback_mismatch"));

    printf("DBGFDC_S5D5,stage=profile_apply,source=%s,deglitchName=%s,deglitchReq=0x%X,"
           "driveCurrentReq=0x%04X,driveCurrentReadback=0x%04X,highCurrentReq=%u,"
           "highCurrentReadback=%u,configReg=0x%04X,muxConfig=0x%04X,clockDiv=0x%04X,err=%ld,status=%s\n",
           source,
           profile->deglitchName ? profile->deglitchName : SENSORARRAY_NA,
           (unsigned)profile->deglitchReq,
           profile->driveCurrentReq,
           driveCurrentReadback,
           profile->highCurrentReq ? 1u : 0u,
           highCurrentReadback ? 1u : 0u,
           configReg,
           muxConfigReg,
           clockDividersReg,
           (long)logErr,
           status);

    return (applyErr != ESP_OK) ? applyErr : readErr;
}

static esp_err_t sensorarrayS5d5ReadOneSampleBounded(Fdc2214CapDevice_t *dev,
                                                     uint8_t channel,
                                                     uint32_t timeoutMs,
                                                     sensorarrayFdcReadDiag_t *outDiag)
{
    if (!dev || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    *outDiag = (sensorarrayFdcReadDiag_t){
        .err = ESP_ERR_TIMEOUT,
        .statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR,
    };

    uint32_t intervalMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_UNREAD_POLL_INTERVAL_MS;
    if (intervalMs == 0u) {
        intervalMs = 1u;
    }
    if (timeoutMs == 0u) {
        timeoutMs = intervalMs;
    }

    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    uint32_t polls = 0u;
    sensorarrayFdcReadDiag_t lastDiag = {0};
    while (esp_timer_get_time() < deadlineUs) {
        sensorarrayFdcReadDiag_t diag = {0};
        esp_err_t readErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(dev,
                                                                        (Fdc2214CapChannel_t)channel,
                                                                        false,
                                                                        true,
                                                                        true,
                                                                        &diag);
        polls++;
        lastDiag = diag;
        if (readErr != ESP_OK) {
            *outDiag = diag;
            outDiag->err = readErr;
            printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,polls=%lu,statusReg=0x%04X,unread=0,"
                   "result=i2c_error,err=%ld\n",
                   (unsigned long)timeoutMs,
                   (unsigned long)polls,
                   diag.sample.StatusRaw,
                   (long)readErr);
            return readErr;
        }

        bool unread = diag.sample.UnreadConversionPresent;
        bool dataReady = diag.sample.DataReady;
        bool terminalFailure = diag.sample.SleepModeEnabled ||
                               !diag.sample.Converting ||
                               diag.sample.ErrWatchdog ||
                               diag.sample.ErrAmplitude;
        bool readable = diag.provisionalReadable && unread;
        if (readable || unread || dataReady || terminalFailure) {
            *outDiag = diag;
            printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,polls=%lu,statusReg=0x%04X,unread=%u,"
                   "dataReady=%u,result=%s,err=%ld\n",
                   (unsigned long)timeoutMs,
                   (unsigned long)polls,
                   diag.sample.StatusRaw,
                   unread ? 1u : 0u,
                   dataReady ? 1u : 0u,
                   terminalFailure ? "terminal_sample_state" : "ready",
                   (long)readErr);
            return readErr;
        }

        uint32_t delayMs = intervalMs;
        int64_t remainingUs = deadlineUs - esp_timer_get_time();
        if (remainingUs <= 0) {
            break;
        }
        uint32_t remainingMs = (uint32_t)((remainingUs + 999LL) / 1000LL);
        if (delayMs > remainingMs) {
            delayMs = remainingMs;
        }
        if (delayMs == 0u) {
            delayMs = 1u;
        }
        sensorarrayS5d5DelayCooperativeMs(delayMs);
    }

    *outDiag = lastDiag;
    outDiag->err = ESP_ERR_TIMEOUT;
    outDiag->i2cOk = false;
    printf("DBGFDC_S5D5,stage=wait_unread,timeoutMs=%lu,polls=%lu,statusReg=0x%04X,unread=%u,"
           "dataReady=%u,result=timeout,err=%ld\n",
           (unsigned long)timeoutMs,
           (unsigned long)polls,
           lastDiag.sample.StatusRaw,
           lastDiag.sample.UnreadConversionPresent ? 1u : 0u,
           lastDiag.sample.DataReady ? 1u : 0u,
           (long)ESP_ERR_TIMEOUT);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t sensorarrayS5d5DiscardSamples(Fdc2214CapDevice_t *dev,
                                               uint8_t channel,
                                               uint32_t discardCount,
                                               uint32_t timeoutMs,
                                               const char *stage)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t firstErr = ESP_OK;
    const char *source = stage ? stage : SENSORARRAY_NA;
    for (uint32_t i = 0u; i < discardCount; ++i) {
        sensorarrayFdcReadDiag_t diag = {0};
        esp_err_t err = sensorarrayS5d5ReadOneSampleBounded(dev, channel, timeoutMs, &diag);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
        printf("DBGFDC_S5D5,stage=%s_discard,discardIndex=%lu,timeoutMs=%lu,raw=%lu,err=%ld,status=%s\n",
               source,
               (unsigned long)i,
               (unsigned long)timeoutMs,
               (unsigned long)diag.sample.Raw28,
               (long)err,
               (err == ESP_OK) ? "discard_done" : "discard_error_continue");
        sensorarrayS5d5ServiceScheduler();
    }
    return firstErr;
}

static esp_err_t sensorarrayS5d5CaptureCandidate(Fdc2214CapDevice_t *dev,
                                                 uint8_t channel,
                                                 const sensorarrayS5d5Profile_t *candidateProfile,
                                                 uint32_t settleMs,
                                                 uint32_t samplesPerCandidate,
                                                 uint32_t timeoutMsPerSample,
                                                 const char *stage,
                                                 uint32_t candidateIndex,
                                                 uint32_t candidateCount,
                                                 sensorarrayS5d5Profile_t *outResult)
{
    if (!dev || !candidateProfile || !outResult) {
        return ESP_ERR_INVALID_ARG;
    }

    const char *stageName = stage ? stage : "capture";
    sensorarrayS5d5Profile_t result = {0};
    sensorarrayS5d5PrepareCapture(&result, candidateProfile);
    int64_t candidateDeadlineUs = esp_timer_get_time() +
                                  ((int64_t)(settleMs + (samplesPerCandidate * timeoutMsPerSample) + 10u) *
                                   1000LL);

    printf("DBGFDC_S5D5,stage=sweep_step_begin,sweep=%s,index=%lu,deglitch=%s,deglitchReq=0x%X,"
           "highCurrent=%u,driveReq=0x%04X,deadlineMs=%lld\n",
           stageName,
           (unsigned long)candidateIndex,
           result.deglitchName ? result.deglitchName : SENSORARRAY_NA,
           (unsigned)result.deglitchReq,
           result.highCurrentReq ? 1u : 0u,
           result.driveCurrentReq,
           (long long)(candidateDeadlineUs / 1000LL));
    printf("DBGFDC_S5D5,stage=%s_candidate_begin,index=%lu,count=%lu,deglitchName=%s,deglitchReq=0x%X,"
           "deglitchBandwidthHz=%lu,driveCurrentReq=0x%04X,highCurrentReq=%u,settleMs=%lu,samples=%lu\n",
           stageName,
           (unsigned long)candidateIndex,
           (unsigned long)candidateCount,
           result.deglitchName ? result.deglitchName : SENSORARRAY_NA,
           (unsigned)result.deglitchReq,
           (unsigned long)result.deglitchBandwidthHz,
           result.driveCurrentReq,
           result.highCurrentReq ? 1u : 0u,
           (unsigned long)settleMs,
           (unsigned long)samplesPerCandidate);

    esp_err_t firstErr = sensorarrayS5d5ApplyProfile(dev, &result, stageName);
    printf("DBGFDC_S5D5,stage=sweep_write_done,sweep=%s,index=%lu,err=%ld,status=%s\n",
           stageName,
           (unsigned long)candidateIndex,
           (long)firstErr,
           (firstErr == ESP_OK) ? "write_done" : "write_failed");
    if (firstErr == ESP_OK) {
        sensorarrayS5d5DelayCooperativeMs(settleMs);
        printf("DBGFDC_S5D5,stage=sweep_sample_begin,sweep=%s,index=%lu,samples=%lu,timeoutMs=%lu\n",
               stageName,
               (unsigned long)candidateIndex,
               (unsigned long)samplesPerCandidate,
               (unsigned long)timeoutMsPerSample);
        uint32_t sampleTimeouts = 0u;
        for (uint32_t sampleIndex = 0u; sampleIndex < samplesPerCandidate; ++sampleIndex) {
            sensorarrayFdcReadDiag_t diag = {0};
            esp_err_t readErr = sensorarrayS5d5ReadOneSampleBounded(dev, channel, timeoutMsPerSample, &diag);
            if (readErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = readErr;
            }
            if (readErr != ESP_OK) {
                diag.err = readErr;
                diag.i2cOk = false;
            }
            if (readErr == ESP_ERR_TIMEOUT) {
                sampleTimeouts++;
                printf("DBGFDC_S5D5,stage=sweep_sample_timeout,sweep=%s,index=%lu,sampleIndex=%lu,"
                       "timeoutMs=%lu,err=%ld\n",
                       stageName,
                       (unsigned long)candidateIndex,
                       (unsigned long)sampleIndex,
                       (unsigned long)timeoutMsPerSample,
                       (long)readErr);
            }
            sensorarrayS5d5IngestDiag(&result, &diag);
            sensorarrayS5d5ServiceScheduler();
        }
        printf("DBGFDC_S5D5,stage=sweep_sample_done,sweep=%s,index=%lu,samples=%lu,timeouts=%lu,err=%ld\n",
               stageName,
               (unsigned long)candidateIndex,
               (unsigned long)result.samples,
               (unsigned long)sampleTimeouts,
               (long)firstErr);
    } else {
        result.samples = 1u;
        result.i2cErr = 1u;
        printf("DBGFDC_S5D5,stage=sweep_sample_begin,sweep=%s,index=%lu,samples=0,timeoutMs=%lu\n",
               stageName,
               (unsigned long)candidateIndex,
               (unsigned long)timeoutMsPerSample);
        printf("DBGFDC_S5D5,stage=sweep_sample_done,sweep=%s,index=%lu,samples=0,timeouts=0,err=%ld\n",
               stageName,
               (unsigned long)candidateIndex,
               (long)firstErr);
    }

    esp_err_t regsErr = sensorarrayS5d5ReadbackAndFinalizeProfile(dev, channel, &result);
    if (regsErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = regsErr;
    }
    if (firstErr != ESP_OK && result.working) {
        result.working = false;
        result.status = "capture_error";
    }
    *outResult = result;

    printf("DBGFDC_S5D5,stage=%s_candidate_end,index=%lu,err=%ld,working=%u,score=%ld,avgFreqHz=%.3f,"
           "rawMin=%lu,rawMax=%lu,rawMean=%lu,rawSpan=%lu,i2cErr=%lu,convertingOk=%lu,unreadOk=%lu,"
           "watchdog=%lu,amplitude=%lu,nonZeroRaw=%lu,configReg=0x%04X,muxConfig=0x%04X,status=%s\n",
           stageName,
           (unsigned long)candidateIndex,
           (long)firstErr,
           result.working ? 1u : 0u,
           (long)result.score,
           result.avgFreqHz,
           (unsigned long)result.rawMin,
           (unsigned long)result.rawMax,
           (unsigned long)result.rawMean,
           (unsigned long)result.rawSpan,
           (unsigned long)result.i2cErr,
           (unsigned long)result.convertingOk,
           (unsigned long)result.unreadOk,
           (unsigned long)result.watchdog,
           (unsigned long)result.amplitude,
           (unsigned long)result.nonZeroRaw,
           result.configReg,
           result.muxConfigReg,
           result.status ? result.status : SENSORARRAY_NA);
    printf("DBGFDC_S5D5,stage=candidate_result,sweep=%s,index=%lu,err=%ld,working=%u,score=%ld,"
           "status=%s,avgFreqHz=%.3f\n",
           stageName,
           (unsigned long)candidateIndex,
           (long)firstErr,
           result.working ? 1u : 0u,
           (long)result.score,
           result.status ? result.status : SENSORARRAY_NA,
           result.avgFreqHz);
    return firstErr;
}

static int sensorarrayS5d5InsertProfileSorted(sensorarrayS5d5Profile_t *profiles,
                                              size_t capacity,
                                              const sensorarrayS5d5Profile_t *candidate)
{
    if (!profiles || capacity == 0u || !sensorarrayS5d5CandidateIsWorking(candidate)) {
        return -1;
    }

    for (size_t i = 0u; i < capacity; ++i) {
        if (profiles[i].valid &&
            profiles[i].deglitchReq == candidate->deglitchReq &&
            profiles[i].driveCurrentNorm == candidate->driveCurrentNorm &&
            profiles[i].highCurrentReq == candidate->highCurrentReq) {
            if (sensorarrayS5d5ProfileBetter(candidate, &profiles[i])) {
                profiles[i] = *candidate;
            }
            while (i > 0u && sensorarrayS5d5ProfileBetter(&profiles[i], &profiles[i - 1u])) {
                sensorarrayS5d5Profile_t tmp = profiles[i - 1u];
                profiles[i - 1u] = profiles[i];
                profiles[i] = tmp;
                i--;
            }
            return (int)i;
        }
    }

    size_t insertAt = capacity;
    for (size_t i = 0u; i < capacity; ++i) {
        if (!profiles[i].valid || sensorarrayS5d5ProfileBetter(candidate, &profiles[i])) {
            insertAt = i;
            break;
        }
    }
    if (insertAt >= capacity) {
        return -1;
    }

    for (size_t i = capacity - 1u; i > insertAt; --i) {
        profiles[i] = profiles[i - 1u];
    }
    profiles[insertAt] = *candidate;
    return (int)insertAt;
}

static void sensorarrayS5d5InsertTopProfile(sensorarrayS5d5Runtime_t *runtime,
                                            const sensorarrayS5d5Profile_t *candidate)
{
    if (!runtime || !candidate) {
        return;
    }

    int rank = sensorarrayS5d5InsertProfileSorted(runtime->topProfiles,
                                                  SENSORARRAY_S5D5_TOP_PROFILE_CACHE_COUNT,
                                                  candidate);
    if (rank >= 0) {
        printf("DBGFDC_S5D5,stage=profile_cache_update,rank=%d,deglitchName=%s,deglitchReq=0x%X,"
               "driveCurrentReq=0x%04X,highCurrentReq=%u,score=%ld,avgFreqHz=%.3f,status=cached\n",
               rank,
               candidate->deglitchName ? candidate->deglitchName : SENSORARRAY_NA,
               (unsigned)candidate->deglitchReq,
               candidate->driveCurrentReq,
               candidate->highCurrentReq ? 1u : 0u,
               (long)candidate->score,
               candidate->avgFreqHz);
    }
}

static bool sensorarrayS5d5CaptureBaseline(Fdc2214CapDevice_t *dev,
                                            uint8_t channel,
                                            uint32_t sampleCount,
                                            uint32_t timeoutMs,
                                            sensorarrayS5d5Profile_t *outBaseline)
{
    printf("DBGFDC_S5D5,stage=baseline_enter,sampleCount=%lu,timeoutMs=%lu\n",
           (unsigned long)sampleCount,
           (unsigned long)timeoutMs);
    if (!dev || !outBaseline) {
        printf("DBGFDC_S5D5,stage=baseline_failed,reason=invalid_arg,err=%ld,samples=0\n",
               (long)ESP_ERR_INVALID_ARG);
        return false;
    }
    if (sampleCount == 0u) {
        sampleCount = 1u;
    }
    if (timeoutMs == 0u) {
        timeoutMs = 1u;
    }

    uint16_t statusReg = 0u;
    uint16_t configReg = 0u;
    uint16_t muxConfigReg = 0u;
    uint16_t clockDividersReg = 0u;
    uint16_t driveCurrentReg = 0u;
    printf("DBGFDC_S5D5,stage=baseline_key_regs_begin\n");
    esp_err_t regsErr = sensorarrayS5d5ReadKeyRegs(dev,
                                                   (Fdc2214CapChannel_t)channel,
                                                   &statusReg,
                                                   &configReg,
                                                   &muxConfigReg,
                                                   &clockDividersReg,
                                                   &driveCurrentReg);
    printf("DBGFDC_S5D5,stage=baseline_key_regs_done,err=%ld,statusReg=0x%04X,config=0x%04X,"
           "muxConfig=0x%04X,clockDiv=0x%04X,drive=0x%04X\n",
           (long)regsErr,
           statusReg,
           configReg,
           muxConfigReg,
           clockDividersReg,
           driveCurrentReg);
    if (regsErr != ESP_OK) {
        *outBaseline = (sensorarrayS5d5Profile_t){0};
        printf("DBGFDC_S5D5,stage=baseline_failed,reason=readback_failed,err=%ld,samples=0\n",
               (long)regsErr);
        return false;
    }

    uint8_t deglitchReq = (uint8_t)(muxConfigReg & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK);
    const sensorarrayS5d5DeglitchCandidate_t *deglitch =
        sensorarrayS5d5FindDeglitchCandidate(deglitchReq);
    sensorarrayS5d5Profile_t baseline = {
        .valid = true,
        .deglitchReq = deglitchReq,
        .deglitchName = deglitch ? deglitch->name : SENSORARRAY_NA,
        .deglitchBandwidthHz = deglitch ? deglitch->bandwidthHz : 0u,
        .highCurrentReq = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u,
        .highCurrentReadback = (configReg & SENSORARRAY_S5D5_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u,
        .driveCurrentReq = (uint16_t)(driveCurrentReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .driveCurrentNorm = (uint16_t)(driveCurrentReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .driveCurrentReadback = (uint16_t)(driveCurrentReg & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK),
        .activeChannelReadback =
            (uint8_t)((configReg & SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_MASK) >>
                      SENSORARRAY_S5D5_CONFIG_ACTIVE_CHAN_SHIFT),
        .statusReg = statusReg,
        .configReg = configReg,
        .muxConfigReg = muxConfigReg,
        .clockDividersReg = clockDividersReg,
        .rawMin = UINT_MAX,
        .clockStatus = "baseline_pending",
        .status = "baseline_pending",
    };
    sensorarrayS5d5ProfileSetClockReadback(&baseline, clockDividersReg);

    printf("DBGFDC_S5D5,stage=baseline_begin,samples=%lu,timeoutMs=%lu,muxConfig=0x%04X,deglitch=%s,"
           "driveReadback=0x%04X\n",
           (unsigned long)sampleCount,
           (unsigned long)timeoutMs,
           muxConfigReg,
           baseline.deglitchName ? baseline.deglitchName : SENSORARRAY_NA,
           baseline.driveCurrentReadback);

    esp_err_t firstErr = ESP_OK;
    uint32_t sampleTimeouts = 0u;
    for (uint32_t sampleIndex = 0u; sampleIndex < sampleCount; ++sampleIndex) {
        sensorarrayFdcReadDiag_t diag = {0};
        printf("DBGFDC_S5D5,stage=baseline_sample_begin,index=%lu\n",
               (unsigned long)sampleIndex);
        printf("DBGFDC_S5D5,stage=baseline_sample_call_diag_begin,index=%lu\n",
               (unsigned long)sampleIndex);
        esp_err_t sampleErr = sensorarrayMeasureReadFdcSampleDiagRelaxed(dev,
                                                                          (Fdc2214CapChannel_t)channel,
                                                                          false,
                                                                          true,
                                                                          true,
                                                                          &diag);
        printf("DBGFDC_S5D5,stage=baseline_sample_call_diag_done,index=%lu,err=%ld,i2cOk=%u,statusCode=%u\n",
               (unsigned long)sampleIndex,
               (long)sampleErr,
               diag.i2cOk ? 1u : 0u,
               (unsigned)diag.statusCode);
        if (sampleErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = sampleErr;
        }
        if (sampleErr != ESP_OK) {
            diag.err = sampleErr;
            diag.i2cOk = false;
        }
        if (sampleErr == ESP_ERR_TIMEOUT) {
            sampleTimeouts++;
            printf("DBGFDC_S5D5,stage=baseline_sample_timeout,index=%lu,timeoutMs=%lu,err=%ld\n",
                   (unsigned long)sampleIndex,
                   (unsigned long)timeoutMs,
                   (long)sampleErr);
        }
        sensorarrayS5d5IngestDiag(&baseline, &diag);
        sensorarrayS5d5ServiceScheduler();
    }

    esp_err_t finalizeErr = sensorarrayS5d5ReadbackAndFinalizeProfile(dev, channel, &baseline);
    if (finalizeErr != ESP_OK && firstErr == ESP_OK) {
        firstErr = finalizeErr;
    }
    *outBaseline = baseline;

    bool haveFreq = (baseline.avgFreqHz > 0.0) && baseline.clockDecodeValid && (baseline.nonZeroRaw > 0u);
    if (haveFreq) {
        printf("DBGFDC_S5D5,stage=baseline_freq,freqHz=%.3f,muxConfig=0x%04X,deglitch=%s,"
               "deglitchReq=0x%X,samples=%lu,rawMean=%lu,driveReadback=0x%04X\n",
               baseline.avgFreqHz,
               baseline.muxConfigReg,
               baseline.deglitchName ? baseline.deglitchName : SENSORARRAY_NA,
               (unsigned)baseline.deglitchReq,
               (unsigned long)baseline.samples,
               (unsigned long)baseline.rawMean,
               baseline.driveCurrentReadback);
        return true;
    }

    printf("DBGFDC_S5D5,stage=baseline_failed,reason=%s,err=%ld,samples=%lu,timeouts=%lu,"
           "muxConfig=0x%04X,deglitch=%s,rawMean=%lu,clockStatus=%s\n",
           baseline.status ? baseline.status : SENSORARRAY_NA,
           (long)firstErr,
           (unsigned long)baseline.samples,
           (unsigned long)sampleTimeouts,
           baseline.muxConfigReg,
           baseline.deglitchName ? baseline.deglitchName : SENSORARRAY_NA,
           (unsigned long)baseline.rawMean,
           baseline.clockStatus ? baseline.clockStatus : SENSORARRAY_NA);
    return false;
}

static void sensorarrayS5d5MakeFallbackProfile(sensorarrayS5d5Profile_t *profile,
                                               const sensorarrayS5d5Profile_t *baseline)
{
    if (!profile) {
        return;
    }

    sensorarrayS5d5MakeDefaultSafeProfile(profile);
    bool haveBaseline = baseline && baseline->valid;
    double baselineFreqHz = haveBaseline ? baseline->avgFreqHz : 0.0;
    bool useFallback10MHz = (CONFIG_SENSORARRAY_DEBUG_S5D5_FALLBACK_DEGLITCH_10MHZ != 0) &&
                            (baselineFreqHz <= 0.0 || baselineFreqHz > 3300000.0);
    uint8_t deglitchReq = useFallback10MHz ? SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ :
                                             (haveBaseline ? baseline->deglitchReq
                                                           : SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ);
    const sensorarrayS5d5DeglitchCandidate_t *deglitch =
        sensorarrayS5d5FindDeglitchCandidate(deglitchReq);
    profile->deglitchReq = deglitch ? deglitch->code : SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ;
    profile->deglitchName = deglitch ? deglitch->name : "10MHz";
    profile->deglitchBandwidthHz = deglitch ? deglitch->bandwidthHz : SENSORARRAY_S5D5_DEFAULT_DEGLITCH_BW_HZ;
    profile->deglitchBandwidthOk = true;
    profile->highCurrentReq = false;

    uint16_t drive = SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ;
    if (haveBaseline && baseline->driveCurrentReadback != 0u) {
        drive = baseline->driveCurrentReadback;
    } else if (haveBaseline && baseline->driveCurrentNorm != 0u) {
        drive = baseline->driveCurrentNorm;
    }
    profile->driveCurrentReq = (uint16_t)(drive & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    profile->driveCurrentNorm = (uint16_t)(drive & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    if (haveBaseline) {
        profile->configReg = baseline->configReg;
        profile->clockDividersReg = baseline->clockDividersReg;
        profile->muxConfigReg = (uint16_t)((baseline->muxConfigReg &
                                            (uint16_t)~SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK) |
                                           profile->deglitchReq);
        profile->avgFreqHz = baseline->avgFreqHz;
    }
    profile->status = "fallback_safe";
}

static uint32_t sensorarrayS5d5BuildDriveOrder(const sensorarrayS5d5Profile_t *baseline,
                                               uint16_t *drives,
                                               uint32_t capacity)
{
    if (!drives || capacity == 0u) {
        return 0u;
    }

    uint32_t count = 0u;
    uint16_t baselineDrive = (baseline && baseline->valid && baseline->driveCurrentReadback != 0u)
                                 ? baseline->driveCurrentReadback
                                 : SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ;
    sensorarrayS5d5AddUniqueDrive(drives, &count, capacity, baselineDrive);
    sensorarrayS5d5AddUniqueDrive(drives, &count, capacity, SENSORARRAY_S5D5_DEFAULT_DRIVE_CURRENT_REQ);

    size_t tableCount = sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE) /
                        sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[0]);
    for (size_t i = 0u; i < tableCount; ++i) {
        sensorarrayS5d5AddUniqueDrive(drives, &count, capacity, SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[i]);
    }
    return count;
}

static uint32_t sensorarrayS5d5BuildDeglitchOrder(const sensorarrayS5d5Profile_t *baseline,
                                                  const sensorarrayS5d5DeglitchCandidate_t **order,
                                                  uint32_t capacity)
{
    if (!order || capacity == 0u) {
        return 0u;
    }

    uint32_t count = 0u;
    bool baselineFreqValid = baseline && baseline->valid && baseline->avgFreqHz > 0.0;
    size_t tableCount = sizeof(SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE) /
                        sizeof(SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[0]);
    if (baselineFreqValid) {
        double minBandwidthHz = baseline->avgFreqHz * SENSORARRAY_S5D5_BASELINE_FREQ_MARGIN;
        for (size_t i = 0u; i < tableCount; ++i) {
            const sensorarrayS5d5DeglitchCandidate_t *candidate = &SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i];
            if ((CONFIG_SENSORARRAY_DEBUG_S5D5_SKIP_BELOW_BASELINE_DEGLITCH != 0) &&
                ((double)candidate->bandwidthHz < minBandwidthHz)) {
                printf("DBGFDC_S5D5,stage=sweep_skip_candidate,deglitch=%s,deglitchReq=0x%X,"
                       "bandwidthHz=%lu,baselineFreqHz=%.3f,reason=below_baseline_freq\n",
                       candidate->name,
                       (unsigned)candidate->code,
                       (unsigned long)candidate->bandwidthHz,
                       baseline->avgFreqHz);
                continue;
            }
            if (count < capacity) {
                order[count++] = candidate;
            }
        }
        return count;
    }

    uint8_t preferredCode = (baseline && baseline->valid)
                                ? baseline->deglitchReq
                                : SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ;
    const sensorarrayS5d5DeglitchCandidate_t *preferred =
        sensorarrayS5d5FindDeglitchCandidate(preferredCode);
    if (!preferred && (CONFIG_SENSORARRAY_DEBUG_S5D5_FALLBACK_DEGLITCH_10MHZ != 0)) {
        preferred = sensorarrayS5d5FindDeglitchCandidate(SENSORARRAY_S5D5_DEFAULT_DEGLITCH_REQ);
    }
    if (preferred && count < capacity) {
        order[count++] = preferred;
    }
    for (size_t i = 0u; i < tableCount; ++i) {
        const sensorarrayS5d5DeglitchCandidate_t *candidate = &SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i];
        if (preferred && candidate->code == preferred->code) {
            continue;
        }
        if (candidate->bandwidthHz >= SENSORARRAY_S5D5_DEFAULT_DEGLITCH_BW_HZ && count < capacity) {
            order[count++] = candidate;
        }
    }
    for (size_t i = 0u; i < tableCount; ++i) {
        const sensorarrayS5d5DeglitchCandidate_t *candidate = &SENSORARRAY_S5D5_DEGLITCH_SWEEP_TABLE[i];
        if (preferred && candidate->code == preferred->code) {
            continue;
        }
        if (candidate->bandwidthHz < SENSORARRAY_S5D5_DEFAULT_DEGLITCH_BW_HZ && count < capacity) {
            order[count++] = candidate;
        }
    }
    return count;
}

static bool sensorarrayS5d5RunFullSweepPasses(Fdc2214CapDevice_t *dev,
                                              uint8_t channel,
                                              sensorarrayS5d5Runtime_t *runtime,
                                              sensorarrayS5d5Profile_t *outBest,
                                              const sensorarrayS5d5Profile_t *baseline,
                                              const char *stageName,
                                              uint32_t budgetMs,
                                              uint32_t firstPassSettleMs,
                                              uint32_t firstPassSamples,
                                              uint32_t verifySettleMs,
                                              uint32_t verifySamples,
                                              uint32_t topVerifyCount)
{
    if (!dev || !runtime || !outBest || !stageName) {
        return false;
    }

    uint32_t startMs = sensorarrayS5d5NowMs();
    uint32_t timeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_UNREAD_POLL_TIMEOUT_MS;
    if (timeoutMs == 0u) {
        timeoutMs = 1u;
    }
    if (budgetMs == 0u) {
        budgetMs = 1u;
    }
    if (topVerifyCount == 0u) {
        topVerifyCount = 1u;
    }
    if (topVerifyCount > 8u) {
        topVerifyCount = 8u;
    }

    const sensorarrayS5d5DeglitchCandidate_t *deglitchOrder[4] = {0};
    uint16_t driveOrder[8] = {0};
    uint32_t deglitchCount = sensorarrayS5d5BuildDeglitchOrder(baseline,
                                                               deglitchOrder,
                                                               (uint32_t)(sizeof(deglitchOrder) /
                                                                          sizeof(deglitchOrder[0])));
    uint32_t driveCurrentCount = sensorarrayS5d5BuildDriveOrder(baseline,
                                                                driveOrder,
                                                                (uint32_t)(sizeof(driveOrder) /
                                                                           sizeof(driveOrder[0])));
    uint32_t candidateCount = deglitchCount * 2u * driveCurrentCount;
    double baselineFreqHz = (baseline && baseline->valid) ? baseline->avgFreqHz : 0.0;

    printf("DBGFDC_S5D5,stage=%s_begin,candidates=%lu,budgetMs=%lu,firstPassSamples=%lu,"
           "firstPassSettleMs=%lu,verifySamples=%lu,verifySettleMs=%lu,baselineFreqHz=%.3f\n",
           stageName,
           (unsigned long)candidateCount,
           (unsigned long)budgetMs,
           (unsigned long)firstPassSamples,
           (unsigned long)firstPassSettleMs,
           (unsigned long)verifySamples,
           (unsigned long)verifySettleMs,
           baselineFreqHz);

    sensorarrayS5d5Profile_t topVerify[8] = {0};
    sensorarrayS5d5Profile_t best = {0};
    bool haveBest = false;
    uint32_t tried = 0u;
    uint32_t working = 0u;
    uint32_t verified = 0u;
    const char *reason = "completed";

    if (deglitchCount == 0u || driveCurrentCount == 0u) {
        reason = "no_candidate_after_filter";
        goto sweep_summary;
    }

    for (uint32_t deglitchIndex = 0u; deglitchIndex < deglitchCount; ++deglitchIndex) {
        const sensorarrayS5d5DeglitchCandidate_t *deglitch = deglitchOrder[deglitchIndex];
        if (!deglitch) {
            continue;
        }

        bool baselineFreqValid = baseline && baseline->valid && baseline->avgFreqHz > 0.0;
        bool shortLowDeglitchTry = !baselineFreqValid &&
                                   (deglitch->bandwidthHz < SENSORARRAY_S5D5_DEFAULT_DEGLITCH_BW_HZ);
        uint32_t candidateTimeoutMs = timeoutMs;
        uint32_t candidateSettleMs = firstPassSettleMs;
        if (shortLowDeglitchTry) {
            if (candidateTimeoutMs > SENSORARRAY_S5D5_BASELINE_FAILED_LOW_DEGLITCH_TIMEOUT_MS) {
                candidateTimeoutMs = SENSORARRAY_S5D5_BASELINE_FAILED_LOW_DEGLITCH_TIMEOUT_MS;
            }
            if (candidateSettleMs > 20u) {
                candidateSettleMs = 20u;
            }
        }

        bool requestHighCurrentPass = false;
        for (uint32_t driveIndex = 0u; driveIndex < driveCurrentCount; ++driveIndex) {
            if ((uint32_t)(sensorarrayS5d5NowMs() - startMs) >= budgetMs) {
                reason = haveBest ? "time_budget_with_working_candidate" : "time_budget_no_working_candidate";
                goto sweep_summary;
            }

            sensorarrayS5d5Profile_t candidateProfile = {0};
            sensorarrayS5d5MakeProfile(&candidateProfile, deglitch, false, driveOrder[driveIndex]);
            sensorarrayS5d5Profile_t captured = {0};
            (void)sensorarrayS5d5CaptureCandidate(dev,
                                                   channel,
                                                   &candidateProfile,
                                                   candidateSettleMs,
                                                   firstPassSamples,
                                                   candidateTimeoutMs,
                                                   stageName,
                                                   tried,
                                                   candidateCount,
                                                   &captured);
            tried++;
            bool rawUnstable = (captured.rawMean > 0u) &&
                               (((uint64_t)captured.rawSpan * 1000ull) >
                                ((uint64_t)captured.rawMean * 50ull));
            if (captured.amplitude != 0u || rawUnstable ||
                (captured.samples != 0u && captured.nonZeroRaw < captured.samples)) {
                requestHighCurrentPass = true;
            }
            if (sensorarrayS5d5CandidateIsWorking(&captured)) {
                working++;
                sensorarrayS5d5InsertTopProfile(runtime, &captured);
                (void)sensorarrayS5d5InsertProfileSorted(topVerify, topVerifyCount, &captured);
                if (!haveBest || sensorarrayS5d5ProfileBetter(&captured, &best)) {
                    best = captured;
                    haveBest = true;
                }
            }
            sensorarrayS5d5ServiceScheduler();
        }

        if (!requestHighCurrentPass) {
            printf("DBGFDC_S5D5,stage=sweep_skip_candidate,deglitch=%s,deglitchReq=0x%X,highCurrent=1,"
                   "reason=normal_current_no_amplitude_or_raw_instability\n",
                   deglitch->name,
                   (unsigned)deglitch->code);
            continue;
        }

        for (uint32_t driveIndex = 0u; driveIndex < driveCurrentCount; ++driveIndex) {
            if ((uint32_t)(sensorarrayS5d5NowMs() - startMs) >= budgetMs) {
                reason = haveBest ? "time_budget_with_working_candidate" : "time_budget_no_working_candidate";
                goto sweep_summary;
            }

            sensorarrayS5d5Profile_t candidateProfile = {0};
            sensorarrayS5d5MakeProfile(&candidateProfile, deglitch, true, driveOrder[driveIndex]);
            sensorarrayS5d5Profile_t captured = {0};
            (void)sensorarrayS5d5CaptureCandidate(dev,
                                                   channel,
                                                   &candidateProfile,
                                                   candidateSettleMs,
                                                   firstPassSamples,
                                                   candidateTimeoutMs,
                                                   stageName,
                                                   tried,
                                                   candidateCount,
                                                   &captured);
            tried++;
            if (sensorarrayS5d5CandidateIsWorking(&captured)) {
                working++;
                sensorarrayS5d5InsertTopProfile(runtime, &captured);
                (void)sensorarrayS5d5InsertProfileSorted(topVerify, topVerifyCount, &captured);
                if (!haveBest || sensorarrayS5d5ProfileBetter(&captured, &best)) {
                    best = captured;
                    haveBest = true;
                }
            }
            sensorarrayS5d5ServiceScheduler();
        }
    }

    for (uint32_t i = 0u; i < topVerifyCount; ++i) {
        if (!topVerify[i].valid) {
            break;
        }
        if ((uint32_t)(sensorarrayS5d5NowMs() - startMs) >= budgetMs) {
            reason = haveBest ? "time_budget_with_working_candidate" : "time_budget_no_working_candidate";
            break;
        }

        printf("DBGFDC_S5D5,stage=%s_verify_begin,index=%lu,count=%lu,deglitchName=%s,driveCurrentReq=0x%04X,"
               "highCurrentReq=%u,score=%ld\n",
               stageName,
               (unsigned long)i,
               (unsigned long)topVerifyCount,
               topVerify[i].deglitchName ? topVerify[i].deglitchName : SENSORARRAY_NA,
               topVerify[i].driveCurrentReq,
               topVerify[i].highCurrentReq ? 1u : 0u,
               (long)topVerify[i].score);

        char verifyStage[48] = {0};
        (void)snprintf(verifyStage, sizeof(verifyStage), "%s_verify", stageName);
        sensorarrayS5d5Profile_t verifiedProfile = {0};
        (void)sensorarrayS5d5CaptureCandidate(dev,
                                               channel,
                                               &topVerify[i],
                                               verifySettleMs,
                                               verifySamples,
                                               timeoutMs,
                                               verifyStage,
                                               i,
                                               topVerifyCount,
                                               &verifiedProfile);
        verified++;
        bool verifiedWorking = sensorarrayS5d5CandidateIsWorking(&verifiedProfile);
        if (verifiedWorking) {
            sensorarrayS5d5InsertTopProfile(runtime, &verifiedProfile);
            if (!haveBest || sensorarrayS5d5ProfileBetter(&verifiedProfile, &best)) {
                best = verifiedProfile;
                haveBest = true;
            }
        }
        printf("DBGFDC_S5D5,stage=%s_verify_end,index=%lu,working=%u,score=%ld,status=%s\n",
               stageName,
               (unsigned long)i,
               verifiedWorking ? 1u : 0u,
               (long)verifiedProfile.score,
               verifiedProfile.status ? verifiedProfile.status : SENSORARRAY_NA);
        sensorarrayS5d5ServiceScheduler();
    }

sweep_summary:
    if (!haveBest) {
        reason = "no_working_candidate";
    }
    if (haveBest) {
        *outBest = best;
    }
    printf("DBGFDC_S5D5,stage=%s_summary,tried=%lu,working=%lu,verified=%lu,selected=%u,fallback=%u,"
           "reason=%s,bestScore=%ld,bestDeglitch=%s,bestDrive=0x%04X,bestHighCurrent=%u\n",
           stageName,
           (unsigned long)tried,
           (unsigned long)working,
           (unsigned long)verified,
           haveBest ? 1u : 0u,
           haveBest ? 0u : 1u,
           reason,
           haveBest ? (long)best.score : (long)INT_MIN,
           (haveBest && best.deglitchName) ? best.deglitchName : SENSORARRAY_NA,
           haveBest ? best.driveCurrentReq : 0u,
           (haveBest && best.highCurrentReq) ? 1u : 0u);
    if (haveBest) {
        printf("DBGFDC_S5D5,stage=sweep_selected,sweep=%s,deglitch=%s,deglitchReq=0x%X,"
               "driveReq=0x%04X,highCurrent=%u,score=%ld,avgFreqHz=%.3f\n",
               stageName,
               best.deglitchName ? best.deglitchName : SENSORARRAY_NA,
               (unsigned)best.deglitchReq,
               best.driveCurrentReq,
               best.highCurrentReq ? 1u : 0u,
               (long)best.score,
               best.avgFreqHz);
    } else {
        printf("DBGFDC_S5D5,stage=sweep_fallback,sweep=%s,reason=%s,status=no_selected_candidate\n",
               stageName,
               reason);
    }
    return haveBest;
}

static bool sensorarrayS5d5RunBootFullSweep(Fdc2214CapDevice_t *dev,
                                            uint8_t channel,
                                            sensorarrayS5d5Runtime_t *runtime,
                                            sensorarrayS5d5Profile_t *outBest,
                                            const sensorarrayS5d5Profile_t *baseline)
{
    bool ok = sensorarrayS5d5RunFullSweepPasses(dev,
                                                channel,
                                                runtime,
                                                outBest,
                                                baseline,
                                                "boot_full_sweep",
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_BOOT_SWEEP_BUDGET_MS,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_FAST_SETTLE_MS,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_FIRST_PASS_SAMPLES,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_VERIFY_SETTLE_MS,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_VERIFY_SAMPLES,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_TOP_VERIFY_COUNT);
    if (runtime) {
        runtime->bootSweepDone = true;
        runtime->lastFullSweepMs = sensorarrayS5d5NowMs();
        if (ok && outBest) {
            runtime->bootBest = *outBest;
            runtime->haveBootBest = true;
        }
    }
    return ok;
}

static bool sensorarrayS5d5TryProfileQuick(Fdc2214CapDevice_t *dev,
                                           uint8_t channel,
                                           const sensorarrayS5d5Profile_t *profile,
                                           sensorarrayS5d5Profile_t *outVerified,
                                           const char *reason)
{
    if (!dev || !profile || !profile->valid || !outVerified) {
        return false;
    }

    uint32_t timeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_UNREAD_POLL_TIMEOUT_MS;
    printf("DBGFDC_S5D5,stage=profile_quick_retry_begin,reason=%s,deglitchName=%s,deglitchReq=0x%X,"
           "driveCurrentReq=0x%04X,highCurrentReq=%u\n",
           reason ? reason : SENSORARRAY_NA,
           profile->deglitchName ? profile->deglitchName : SENSORARRAY_NA,
           (unsigned)profile->deglitchReq,
           profile->driveCurrentReq,
           profile->highCurrentReq ? 1u : 0u);

    esp_err_t err = sensorarrayS5d5ApplyProfile(dev, profile, "profile_quick_retry");
    if (err == ESP_OK) {
        (void)sensorarrayS5d5DiscardSamples(dev,
                                            channel,
                                            SENSORARRAY_S5D5_RELOCK_DISCARD_SAMPLES,
                                            timeoutMs,
                                            "profile_quick_retry");
    }

    sensorarrayS5d5Profile_t verified = {0};
    sensorarrayS5d5PrepareCapture(&verified, profile);
    if (err == ESP_OK) {
        for (uint32_t i = 0u; i < 3u; ++i) {
            sensorarrayFdcReadDiag_t diag = {0};
            esp_err_t readErr = sensorarrayS5d5ReadOneSampleBounded(dev, channel, timeoutMs, &diag);
            if (readErr != ESP_OK) {
                diag.err = readErr;
                diag.i2cOk = false;
            }
            sensorarrayS5d5IngestDiag(&verified, &diag);
            sensorarrayS5d5ServiceScheduler();
        }
    } else {
        verified.samples = 1u;
        verified.i2cErr = 1u;
    }
    esp_err_t regsErr = sensorarrayS5d5ReadbackAndFinalizeProfile(dev, channel, &verified);
    if (err == ESP_OK && regsErr != ESP_OK) {
        err = regsErr;
    }
    bool ok = (err == ESP_OK) && sensorarrayS5d5CandidateIsWorking(&verified);
    printf("DBGFDC_S5D5,stage=profile_quick_retry_result,reason=%s,ok=%u,err=%ld,working=%u,score=%ld,"
           "avgFreqHz=%.3f,status=%s\n",
           reason ? reason : SENSORARRAY_NA,
           ok ? 1u : 0u,
           (long)err,
           verified.working ? 1u : 0u,
           (long)verified.score,
           verified.avgFreqHz,
           verified.status ? verified.status : SENSORARRAY_NA);
    if (ok) {
        *outVerified = verified;
    }
    return ok;
}

static int sensorarrayS5d5FindDriveIndex(uint16_t driveCurrentNorm)
{
    size_t count = sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE) /
                   sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[0]);
    for (size_t i = 0u; i < count; ++i) {
        if ((SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[i] & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK) ==
            (driveCurrentNorm & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK)) {
            return (int)i;
        }
    }
    return -1;
}

static void sensorarrayS5d5AddUniqueDrive(uint16_t *drives,
                                           uint32_t *count,
                                           uint32_t capacity,
                                           uint16_t drive)
{
    if (!drives || !count || *count >= capacity) {
        return;
    }
    drive = (uint16_t)(drive & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
    for (uint32_t i = 0u; i < *count; ++i) {
        if (drives[i] == drive) {
            return;
        }
    }
    drives[*count] = drive;
    (*count)++;
}

static bool sensorarrayS5d5RunFastSweep(Fdc2214CapDevice_t *dev,
                                        uint8_t channel,
                                        sensorarrayS5d5Runtime_t *runtime,
                                        sensorarrayS5d5Profile_t *outBest,
                                        const char *reason)
{
    if (!dev || !runtime || !outBest) {
        return false;
    }

    sensorarrayS5d5Profile_t reference = {0};
    if (runtime->haveLastGood) {
        reference = runtime->lastGood;
    } else if (runtime->haveBootBest) {
        reference = runtime->bootBest;
    } else {
        sensorarrayS5d5MakeDefaultSafeProfile(&reference);
    }

    uint16_t drives[3] = {0};
    uint32_t driveCount = 0u;
    int currentIndex = sensorarrayS5d5FindDriveIndex(reference.driveCurrentNorm);
    sensorarrayS5d5AddUniqueDrive(drives, &driveCount, 3u, reference.driveCurrentNorm);
    if (currentIndex >= 0) {
        if (currentIndex > 0) {
            sensorarrayS5d5AddUniqueDrive(drives,
                                           &driveCount,
                                           3u,
                                           SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[currentIndex - 1]);
        }
        size_t tableCount = sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE) /
                            sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[0]);
        if ((size_t)(currentIndex + 1) < tableCount) {
            sensorarrayS5d5AddUniqueDrive(drives,
                                           &driveCount,
                                           3u,
                                           SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[currentIndex + 1]);
        }
    }

    printf("DBGFDC_S5D5,stage=fast_sweep_begin,reason=%s,deglitchName=%s,deglitchReq=0x%X,"
           "highCurrentReq=%u,driveCount=%lu,settleMs=%lu,samples=%lu\n",
           reason ? reason : SENSORARRAY_NA,
           reference.deglitchName ? reference.deglitchName : SENSORARRAY_NA,
           (unsigned)reference.deglitchReq,
           reference.highCurrentReq ? 1u : 0u,
           (unsigned long)driveCount,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_FAST_SWEEP_SETTLE_MS,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_FAST_SWEEP_SAMPLES);

    bool haveBest = false;
    sensorarrayS5d5Profile_t best = {0};
    uint32_t timeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_UNREAD_POLL_TIMEOUT_MS;
    for (uint32_t i = 0u; i < driveCount; ++i) {
        sensorarrayS5d5Profile_t candidate = reference;
        candidate.driveCurrentReq = drives[i];
        candidate.driveCurrentNorm = drives[i];
        candidate.valid = true;
        sensorarrayS5d5Profile_t captured = {0};
        (void)sensorarrayS5d5CaptureCandidate(dev,
                                               channel,
                                               &candidate,
                                               (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_FAST_SWEEP_SETTLE_MS,
                                               (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_FAST_SWEEP_SAMPLES,
                                               timeoutMs,
                                               "fast_sweep",
                                               i,
                                               driveCount,
                                               &captured);
        if (sensorarrayS5d5CandidateIsWorking(&captured)) {
            sensorarrayS5d5InsertTopProfile(runtime, &captured);
            if (!haveBest || sensorarrayS5d5ProfileBetter(&captured, &best)) {
                best = captured;
                haveBest = true;
            }
        }
        sensorarrayS5d5ServiceScheduler();
    }

    if (haveBest) {
        *outBest = best;
        printf("DBGFDC_S5D5,stage=fast_sweep_selected,reason=%s,deglitchName=%s,driveCurrentReq=0x%04X,"
               "highCurrentReq=%u,score=%ld,avgFreqHz=%.3f,status=selected\n",
               reason ? reason : SENSORARRAY_NA,
               best.deglitchName ? best.deglitchName : SENSORARRAY_NA,
               best.driveCurrentReq,
               best.highCurrentReq ? 1u : 0u,
               (long)best.score,
               best.avgFreqHz);
        return true;
    }

    printf("DBGFDC_S5D5,stage=fast_sweep_failed,reason=%s,status=no_working_candidate\n",
           reason ? reason : SENSORARRAY_NA);
    return false;
}

static bool sensorarrayS5d5RunMediumSweep(Fdc2214CapDevice_t *dev,
                                          uint8_t channel,
                                          sensorarrayS5d5Runtime_t *runtime,
                                          sensorarrayS5d5Profile_t *outBest,
                                          const char *reason)
{
    if (!dev || !runtime || !outBest) {
        return false;
    }

    sensorarrayS5d5Profile_t reference = {0};
    if (runtime->haveLastGood) {
        reference = runtime->lastGood;
    } else if (runtime->haveBootBest) {
        reference = runtime->bootBest;
    } else {
        sensorarrayS5d5MakeDefaultSafeProfile(&reference);
    }

    size_t driveCount = sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE) /
                        sizeof(SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[0]);
    printf("DBGFDC_S5D5,stage=medium_sweep_begin,reason=%s,deglitchName=%s,deglitchReq=0x%X,"
           "driveCount=%u,settleMs=%lu,samples=%lu\n",
           reason ? reason : SENSORARRAY_NA,
           reference.deglitchName ? reference.deglitchName : SENSORARRAY_NA,
           (unsigned)reference.deglitchReq,
           (unsigned)driveCount,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_MEDIUM_SWEEP_SETTLE_MS,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_MEDIUM_SWEEP_SAMPLES);

    bool haveBest = false;
    sensorarrayS5d5Profile_t best = {0};
    uint32_t timeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_UNREAD_POLL_TIMEOUT_MS;
    for (uint32_t highPass = 0u; highPass < 2u && !haveBest; ++highPass) {
        bool highCurrent = (highPass != 0u);
        for (size_t driveIndex = 0u; driveIndex < driveCount; ++driveIndex) {
            sensorarrayS5d5Profile_t candidate = reference;
            candidate.highCurrentReq = highCurrent;
            candidate.driveCurrentReq = SENSORARRAY_S5D5_DRIVE_CURRENT_SWEEP_TABLE[driveIndex];
            candidate.driveCurrentNorm = (uint16_t)(candidate.driveCurrentReq & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
            candidate.valid = true;
            sensorarrayS5d5Profile_t captured = {0};
            (void)sensorarrayS5d5CaptureCandidate(dev,
                                                   channel,
                                                   &candidate,
                                                   (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_MEDIUM_SWEEP_SETTLE_MS,
                                                   (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_MEDIUM_SWEEP_SAMPLES,
                                                   timeoutMs,
                                                   "medium_sweep",
                                                   (uint32_t)(highPass * driveCount + driveIndex),
                                                   (uint32_t)(2u * driveCount),
                                                   &captured);
            if (sensorarrayS5d5CandidateIsWorking(&captured)) {
                sensorarrayS5d5InsertTopProfile(runtime, &captured);
                if (!haveBest || sensorarrayS5d5ProfileBetter(&captured, &best)) {
                    best = captured;
                    haveBest = true;
                }
            }
            sensorarrayS5d5ServiceScheduler();
        }
    }

    if (haveBest) {
        *outBest = best;
        printf("DBGFDC_S5D5,stage=medium_sweep_selected,reason=%s,deglitchName=%s,driveCurrentReq=0x%04X,"
               "highCurrentReq=%u,score=%ld,avgFreqHz=%.3f,status=selected\n",
               reason ? reason : SENSORARRAY_NA,
               best.deglitchName ? best.deglitchName : SENSORARRAY_NA,
               best.driveCurrentReq,
               best.highCurrentReq ? 1u : 0u,
               (long)best.score,
               best.avgFreqHz);
        return true;
    }

    printf("DBGFDC_S5D5,stage=medium_sweep_failed,reason=%s,status=no_working_candidate\n",
           reason ? reason : SENSORARRAY_NA);
    return false;
}

static bool sensorarrayS5d5RunRuntimeFullSweepFallback(Fdc2214CapDevice_t *dev,
                                                       uint8_t channel,
                                                       sensorarrayS5d5Runtime_t *runtime,
                                                       sensorarrayS5d5Profile_t *outBest,
                                                       const char *reason)
{
    if (!dev || !runtime || !outBest) {
        return false;
    }

    uint32_t nowMs = sensorarrayS5d5NowMs();
    uint32_t cooldownMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_FULL_SWEEP_COOLDOWN_MS;
    uint32_t elapsedMs = nowMs - runtime->lastFullSweepMs;
    printf("DBGFDC_S5D5,stage=full_sweep_fallback_begin,reason=%s,cooldownMs=%lu,elapsedMs=%lu\n",
           reason ? reason : SENSORARRAY_NA,
           (unsigned long)cooldownMs,
           (unsigned long)elapsedMs);
    if (runtime->lastFullSweepMs != 0u && elapsedMs < cooldownMs) {
        printf("DBGFDC_S5D5,stage=full_sweep_fallback_summary,reason=%s,ok=0,status=cooldown_skip\n",
               reason ? reason : SENSORARRAY_NA);
        return false;
    }

    runtime->lastFullSweepMs = nowMs;
    bool ok = sensorarrayS5d5RunFullSweepPasses(dev,
                                                channel,
                                                runtime,
                                                outBest,
                                                runtime->haveLastGood ? &runtime->lastGood : NULL,
                                                "runtime_full_sweep",
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_BOOT_SWEEP_BUDGET_MS,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_FAST_SETTLE_MS,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_FIRST_PASS_SAMPLES,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_VERIFY_SETTLE_MS,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_VERIFY_SAMPLES,
                                                (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_TOP_VERIFY_COUNT);
    printf("DBGFDC_S5D5,stage=full_sweep_fallback_summary,reason=%s,ok=%u,status=%s\n",
           reason ? reason : SENSORARRAY_NA,
           ok ? 1u : 0u,
           ok ? "selected" : "failed");
    return ok;
}

static void sensorarrayS5d5ResetRuntimeFaultStreaks(sensorarrayS5d5Runtime_t *runtime)
{
    if (!runtime) {
        return;
    }
    runtime->largeFreqJumpStreak = 0u;
    runtime->i2cErrStreak = 0u;
    runtime->waitTimeoutStreak = 0u;
    runtime->nonConvertingStreak = 0u;
    runtime->amplitudeFaultStreak = 0u;
    runtime->readbackMismatchStreak = 0u;
}

static void sensorarrayS5d5UpdateFreqAnchor(sensorarrayS5d5Runtime_t *runtime,
                                            double currentFreqHz,
                                            bool sampleGood)
{
    if (!runtime || currentFreqHz <= 0.0) {
        return;
    }
    if (!runtime->haveFreqAnchor || runtime->lockedFreqHz <= 0.0) {
        runtime->haveFreqAnchor = true;
        runtime->lockedFreqHz = currentFreqHz;
        return;
    }
    if (sampleGood) {
        double alpha = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_GOOD_ANCHOR_ALPHA_PERMILLE / 1000.0;
        if (alpha < 0.0) {
            alpha = 0.0;
        } else if (alpha > 1.0) {
            alpha = 1.0;
        }
        runtime->lockedFreqHz = runtime->lockedFreqHz * (1.0 - alpha) + currentFreqHz * alpha;
    }
}

static bool sensorarrayS5d5ShouldRelock(sensorarrayS5d5Runtime_t *runtime,
                                        const sensorarrayFdcReadDiag_t *diag,
                                        double currentFreqHz,
                                        const char **outReason)
{
    if (!runtime) {
        return false;
    }

    const char *reason = "locked_continue";
    bool sampleTransportOk = diag && diag->err == ESP_OK && diag->i2cOk;
    bool sampleGood = sampleTransportOk &&
                      diag->provisionalReadable &&
                      diag->sample.UnreadConversionPresent &&
                      !diag->sample.ErrWatchdog &&
                      !diag->sample.ErrAmplitude;

    if (!diag) {
        runtime->i2cErrStreak++;
        reason = "diag_missing";
    } else if (diag->err == ESP_ERR_TIMEOUT) {
        runtime->waitTimeoutStreak++;
        runtime->i2cErrStreak = 0u;
    } else if (!sampleTransportOk) {
        runtime->i2cErrStreak++;
        runtime->waitTimeoutStreak = 0u;
    } else {
        runtime->i2cErrStreak = 0u;
        runtime->waitTimeoutStreak = 0u;
    }

    if (sampleTransportOk && !diag->sample.Converting) {
        runtime->nonConvertingStreak++;
    } else if (sampleTransportOk) {
        runtime->nonConvertingStreak = 0u;
    }

    if (sampleTransportOk && diag->sample.ErrAmplitude) {
        runtime->amplitudeFaultStreak++;
    } else if (sampleTransportOk) {
        runtime->amplitudeFaultStreak = 0u;
    }

    double absDeltaHz = 0.0;
    double relPermille = 0.0;
    if (currentFreqHz > 0.0 && runtime->haveFreqAnchor && runtime->lockedFreqHz > 0.0) {
        absDeltaHz = (currentFreqHz >= runtime->lockedFreqHz)
                         ? (currentFreqHz - runtime->lockedFreqHz)
                         : (runtime->lockedFreqHz - currentFreqHz);
        relPermille = (absDeltaHz / runtime->lockedFreqHz) * 1000.0;
        if (absDeltaHz > (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_RELOCK_FREQ_JUMP_ABS_HZ ||
            relPermille > (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_RELOCK_FREQ_JUMP_REL_PERMILLE) {
            runtime->largeFreqJumpStreak++;
        } else {
            runtime->largeFreqJumpStreak = 0u;
        }
    } else if (sampleGood) {
        runtime->largeFreqJumpStreak = 0u;
    }

    bool hardFault = false;
    bool allow = false;
    if (runtime->i2cErrStreak >= 3u) {
        reason = "i2c_error_streak";
        hardFault = true;
        allow = true;
    } else if (runtime->waitTimeoutStreak >= 3u) {
        reason = "wait_timeout_streak";
        hardFault = true;
        allow = true;
    } else if (runtime->nonConvertingStreak >= 3u) {
        reason = "non_converting_streak";
        hardFault = true;
        allow = true;
    } else if (runtime->amplitudeFaultStreak >= 3u) {
        reason = "amplitude_fault_streak";
        allow = true;
    } else if (runtime->readbackMismatchStreak >= 3u) {
        reason = "readback_mismatch_streak";
        allow = true;
    } else if (runtime->largeFreqJumpStreak >= (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_RELOCK_JUMP_STREAK) {
        reason = "freq_jump_streak";
        allow = true;
    }

    if (allow && !hardFault &&
        runtime->samplesSinceRelock < (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_RELOCK_COOLDOWN_SAMPLES) {
        printf("DBGFDC_S5D5,stage=relock_suppressed,reason=%s,cooldownSamples=%lu,samplesSinceRelock=%lu\n",
               reason,
               (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_RELOCK_COOLDOWN_SAMPLES,
               (unsigned long)runtime->samplesSinceRelock);
        allow = false;
    }

    printf("DBGFDC_S5D5,stage=relock_decision,reason=%s,allow=%u,freqHz=%.3f,anchorHz=%.3f,"
           "absDeltaHz=%.3f,relPermille=%.3f,jumpStreak=%lu,i2cStreak=%lu,timeoutStreak=%lu,"
           "nonConvertingStreak=%lu,amplitudeStreak=%lu,readbackMismatchStreak=%lu,samplesSinceRelock=%lu\n",
           reason,
           allow ? 1u : 0u,
           currentFreqHz,
           runtime->haveFreqAnchor ? runtime->lockedFreqHz : 0.0,
           absDeltaHz,
           relPermille,
           (unsigned long)runtime->largeFreqJumpStreak,
           (unsigned long)runtime->i2cErrStreak,
           (unsigned long)runtime->waitTimeoutStreak,
           (unsigned long)runtime->nonConvertingStreak,
           (unsigned long)runtime->amplitudeFaultStreak,
           (unsigned long)runtime->readbackMismatchStreak,
           (unsigned long)runtime->samplesSinceRelock);

    if (outReason) {
        *outReason = reason;
    }
    return allow;
}

static bool sensorarrayS5d5SelectRelockProfile(Fdc2214CapDevice_t *dev,
                                               uint8_t channel,
                                               sensorarrayS5d5Runtime_t *runtime,
                                               sensorarrayS5d5Profile_t *outProfile,
                                               const char *reason)
{
    if (!dev || !runtime || !outProfile) {
        return false;
    }

    if (runtime->haveLastGood &&
        sensorarrayS5d5TryProfileQuick(dev, channel, &runtime->lastGood, outProfile, "last_good")) {
        return true;
    }
    if (runtime->haveBootBest &&
        sensorarrayS5d5TryProfileQuick(dev, channel, &runtime->bootBest, outProfile, "boot_best")) {
        return true;
    }
    for (uint32_t i = 0u; i < SENSORARRAY_S5D5_TOP_PROFILE_CACHE_COUNT; ++i) {
        if (runtime->topProfiles[i].valid &&
            sensorarrayS5d5TryProfileQuick(dev, channel, &runtime->topProfiles[i], outProfile, "top_profile")) {
            return true;
        }
    }
    if (sensorarrayS5d5RunFastSweep(dev, channel, runtime, outProfile, reason)) {
        return true;
    }
    if (sensorarrayS5d5RunMediumSweep(dev, channel, runtime, outProfile, reason)) {
        return true;
    }
    return sensorarrayS5d5RunRuntimeFullSweepFallback(dev, channel, runtime, outProfile, reason);
}

static esp_err_t sensorarrayS5d5ApplyRuntimeLockProfile(Fdc2214CapDevice_t *dev,
                                                        uint8_t channel,
                                                        sensorarrayS5d5Runtime_t *runtime,
                                                        const sensorarrayS5d5Profile_t *profile,
                                                        const char *source)
{
    if (!dev || !runtime || !profile) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayS5d5ApplyProfile(dev, profile, source ? source : "relock_apply");
    if (err == ESP_OK) {
        (void)sensorarrayS5d5DiscardSamples(dev,
                                            channel,
                                            SENSORARRAY_S5D5_RELOCK_DISCARD_SAMPLES,
                                            (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_UNREAD_POLL_TIMEOUT_MS,
                                            source ? source : "relock_apply");
        runtime->lastGood = *profile;
        runtime->haveLastGood = true;
        runtime->lockEpoch++;
        runtime->samplesSinceRelock = 0u;
        runtime->lastRelockMs = sensorarrayS5d5NowMs();
        if (profile->avgFreqHz > 0.0) {
            runtime->haveFreqAnchor = true;
            runtime->lockedFreqHz = profile->avgFreqHz;
        }
        sensorarrayS5d5ResetRuntimeFaultStreaks(runtime);
    }

    printf("DBGFDC_S5D5,stage=relock_apply,source=%s,err=%ld,lockEpoch=%lu,deglitchName=%s,"
           "driveCurrentReq=0x%04X,highCurrentReq=%u,avgFreqHz=%.3f,status=%s\n",
           source ? source : SENSORARRAY_NA,
           (long)err,
           (unsigned long)runtime->lockEpoch,
           profile->deglitchName ? profile->deglitchName : SENSORARRAY_NA,
           profile->driveCurrentReq,
           profile->highCurrentReq ? 1u : 0u,
           profile->avgFreqHz,
           (err == ESP_OK) ? "applied" : "failed");
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

    uint32_t lockedSampleCount = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOCKED_SAMPLE_COUNT;
    if (lockedSampleCount == 0u) {
        lockedSampleCount = 1u;
    }
    uint32_t loopDelayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOOP_DELAY_MS;
    if (loopDelayMs < 10u) {
        loopDelayMs = 10u;
    }
    uint32_t unreadPollTimeoutMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_UNREAD_POLL_TIMEOUT_MS;
    if (unreadPollTimeoutMs == 0u) {
        unreadPollTimeoutMs = 1u;
    }
    uint32_t recoveryReinitThreshold =
        (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_RECOVERY_REINIT_ERR_THRESHOLD;
    if (recoveryReinitThreshold == 0u) {
        recoveryReinitThreshold = 3u;
    }

    sensorarrayS5d5CapComputationConfig_t capConfig = {
        .inductorValueUh = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_INDUCTOR_UH,
        .fixedCapPf = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_FIXED_CAP_PF,
        .parasiticCapPf = (double)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_PARASITIC_CAP_PF,
        .enableCapComputation = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_ENABLE_CAP_COMPUTATION != 0),
        .enableNetCapOutput = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_ENABLE_NET_CAP_OUTPUT != 0),
    };
    sensorarrayCheckpointGpio_t checkpoint = sensorarrayCheckpointInit();

    bool directOnlyMode = (CONFIG_SENSORARRAY_DEBUG_S5D5_DISABLE_ALL_SWEEP_AND_BASELINE != 0);
    bool forceDirectLock = (CONFIG_SENSORARRAY_DEBUG_S5D5_FORCE_DIRECT_10MHZ_LOCK != 0);
    printf("DBGFDC_S5D5,stage=target,mode=S5D5_CAP_FDC_SECONDARY,fdcDev=secondary_selb_side,i2cPort=1,"
           "sda=%d,scl=%d,i2cAddr=0x%02X,route=S5D5_CAP,channel=CH0,lockedSamples=%lu,loopDelayMs=%lu,"
           "bootFullSweepEnable=%u,bootBudgetMs=%lu,enableCapComputation=%u,enableNetCapOutput=%u,"
           "inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,directOnly=%u,forceDirect10MHz=%u\n",
           busInfo.SdaGpio,
           busInfo.SclGpio,
           SENSORARRAY_FDC_I2C_ADDR_LOW,
           (unsigned long)lockedSampleCount,
           (unsigned long)loopDelayMs,
           (CONFIG_SENSORARRAY_DEBUG_S5D5_BOOT_FULL_SWEEP_ENABLE != 0) ? 1u : 0u,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_S5D5_BOOT_SWEEP_BUDGET_MS,
           capConfig.enableCapComputation ? 1u : 0u,
           capConfig.enableNetCapOutput ? 1u : 0u,
           capConfig.inductorValueUh,
           capConfig.fixedCapPf,
           capConfig.parasiticCapPf,
           directOnlyMode ? 1u : 0u,
           forceDirectLock ? 1u : 0u);
    printf("DBGFDC_S5D5,stage=route_semantics,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n");
    printf("DBGFDC_S5D5,stage=sweep_plan,deglitch=baseline_filtered_1MHz|3p3MHz|10MHz|33MHz,"
           "highCurrent=0_then_conditional_1,driveCurrentList=0x7800|0xA000|0xB800|0xC000|0xD000|0xE000|0xF800,bootFirstSettleMs=%lu,"
           "bootVerifySettleMs=%lu,fastSettleMs=%lu,mediumSettleMs=%lu,unreadPollTimeoutMs=%lu\n",
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_FAST_SETTLE_MS,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_BOOT_FULL_SWEEP_VERIFY_SETTLE_MS,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_FAST_SWEEP_SETTLE_MS,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_MEDIUM_SWEEP_SETTLE_MS,
           (unsigned long)unreadPollTimeoutMs);

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
        sensorarrayDebugIdleForever("s5d5_fdc_init_failed");
        return;
    }

    const char *routeMapLabel = SENSORARRAY_NA;
    tmux1108Source_t swSource = TMUX1108_SOURCE_GND;
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
        sensorarrayDebugIdleForever("s5d5_route_apply_failed");
        return;
    }
    sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_ROUTE_APPLIED);

    sensorarrayS5d5RouteCheck_t routeCheck = sensorarrayVerifyS5d5Route(swSource,
                                                                         (uint8_t)(SENSORARRAY_S5 - 1u),
                                                                         selaWriteLevel,
                                                                         route->selBLevel);
    if (!routeCheck.commandMatch || !routeCheck.gpioObservedMatch) {
        sensorarrayCheckpointEmit(&checkpoint, SENSORARRAY_CHECKPOINT_EVENT_WARNING);
        sensorarrayDebugIdleForever("s5d5_route_verify_failed");
        return;
    }

    sensorarrayS5d5Runtime_t runtime = {0};
    sensorarrayS5d5Profile_t selectedProfile = {0};
    sensorarrayS5d5Profile_t baselineProfile = {0};
    bool baselineOk = false;
    bool bootSweepOk = false;
    bool directLockPath = directOnlyMode || forceDirectLock;
    const char *lockSource = directLockPath ? "direct_10mhz" : "boot_sweep";

    printf("DBGFDC_S5D5,stage=sweep_enter,bootFullSweepEnable=%u,bootBudgetMs=%lu,"
           "baselineSamples=%lu,skipBelowBaseline=%u,fallback10MHz=%u,directOnly=%u,forceDirect10MHz=%u,"
           "bootBaselineEnable=%u\n",
           (CONFIG_SENSORARRAY_DEBUG_S5D5_BOOT_FULL_SWEEP_ENABLE != 0) ? 1u : 0u,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_S5D5_BOOT_SWEEP_BUDGET_MS,
           (unsigned long)CONFIG_SENSORARRAY_DEBUG_S5D5_BASELINE_SAMPLE_COUNT,
           (CONFIG_SENSORARRAY_DEBUG_S5D5_SKIP_BELOW_BASELINE_DEGLITCH != 0) ? 1u : 0u,
           (CONFIG_SENSORARRAY_DEBUG_S5D5_FALLBACK_DEGLITCH_10MHZ != 0) ? 1u : 0u,
           directOnlyMode ? 1u : 0u,
           forceDirectLock ? 1u : 0u,
           (CONFIG_SENSORARRAY_DEBUG_S5D5_ENABLE_BOOT_BASELINE_READ != 0) ? 1u : 0u);

    if (directLockPath) {
        sensorarrayS5d5MakeDefaultSafeProfile(&selectedProfile);
        selectedProfile.driveCurrentReq = SENSORARRAY_S5D5_DIRECT_SAFE_DRIVE_CURRENT_REQ;
        selectedProfile.driveCurrentNorm =
            (uint16_t)(SENSORARRAY_S5D5_DIRECT_SAFE_DRIVE_CURRENT_REQ & SENSORARRAY_S5D5_DRIVE_CURRENT_MASK);
        printf("DBGFDC_S5D5,stage=direct_lock_enter,reason=avoid_boot_baseline_hang,deglitch=10MHz,"
               "drive=0x%04X\n",
               selectedProfile.driveCurrentNorm);
        esp_err_t lockErr = sensorarrayS5d5ApplyDirectSafeLock(fdcState->handle);
        printf("DBGFDC_S5D5,stage=direct_lock_done,err=%ld,status=%s\n",
               (long)lockErr,
               (lockErr == ESP_OK) ? "ok" : "failed_continue_to_locked_loop");
        runtime.bootSweepDone = true;
    } else {
        if (CONFIG_SENSORARRAY_DEBUG_S5D5_ENABLE_BOOT_BASELINE_READ != 0) {
            baselineOk = sensorarrayS5d5CaptureBaseline(fdcState->handle,
                                                        (uint8_t)fdcMap->channel,
                                                        (uint32_t)CONFIG_SENSORARRAY_DEBUG_S5D5_BASELINE_SAMPLE_COUNT,
                                                        unreadPollTimeoutMs,
                                                        &baselineProfile);
        } else {
            printf("DBGFDC_S5D5,stage=baseline_skipped,reason=disabled_by_default_after_sweep_enter_hang\n");
        }

        if (CONFIG_SENSORARRAY_DEBUG_S5D5_BOOT_FULL_SWEEP_ENABLE != 0) {
            bootSweepOk = sensorarrayS5d5RunBootFullSweep(fdcState->handle,
                                                           (uint8_t)fdcMap->channel,
                                                           &runtime,
                                                           &selectedProfile,
                                                           baselineProfile.valid ? &baselineProfile : NULL);
        } else {
            runtime.bootSweepDone = true;
            printf("DBGFDC_S5D5,stage=boot_full_sweep_skip,status=disabled\n");
        }

        if (!bootSweepOk) {
            sensorarrayS5d5MakeFallbackProfile(&selectedProfile, baselineProfile.valid ? &baselineProfile : NULL);
            printf("DBGFDC_S5D5,stage=sweep_fallback,reason=boot_sweep_failed_or_disabled,baselineOk=%u,"
                   "deglitch=%s,driveReq=0x%04X,status=select_safe_profile\n",
                   baselineOk ? 1u : 0u,
                   selectedProfile.deglitchName ? selectedProfile.deglitchName : SENSORARRAY_NA,
                   selectedProfile.driveCurrentReq);
            printf("DBGFDC_S5D5,stage=boot_profile_select,status=fallback_default,reason=boot_sweep_failed_or_disabled\n");
        } else {
            printf("DBGFDC_S5D5,stage=boot_profile_select,status=boot_sweep_best,deglitchName=%s,driveCurrentReq=0x%04X,"
                   "highCurrentReq=%u,score=%ld\n",
                   selectedProfile.deglitchName ? selectedProfile.deglitchName : SENSORARRAY_NA,
                   selectedProfile.driveCurrentReq,
                   selectedProfile.highCurrentReq ? 1u : 0u,
                   (long)selectedProfile.score);
        }

        esp_err_t applyErr = sensorarrayS5d5ApplyProfile(fdcState->handle, &selectedProfile, "boot_selected");
        if (applyErr != ESP_OK) {
            sensorarrayS5d5MakeFallbackProfile(&selectedProfile, baselineProfile.valid ? &baselineProfile : NULL);
            applyErr = sensorarrayS5d5ApplyProfile(fdcState->handle, &selectedProfile, "boot_default_fallback");
            printf("DBGFDC_S5D5,stage=sweep_fallback,reason=boot_selected_apply_failed,err=%ld,"
                   "deglitch=%s,driveReq=0x%04X,status=%s\n",
                   (long)applyErr,
                   selectedProfile.deglitchName ? selectedProfile.deglitchName : SENSORARRAY_NA,
                   selectedProfile.driveCurrentReq,
                   (applyErr == ESP_OK) ? "fallback_applied" : "fallback_apply_failed_continue");
        }
    }
    if (!directLockPath) {
        (void)sensorarrayS5d5DiscardSamples(fdcState->handle,
                                            (uint8_t)fdcMap->channel,
                                            SENSORARRAY_S5D5_RELOCK_DISCARD_SAMPLES,
                                            unreadPollTimeoutMs,
                                            "boot_lock");
    }

    runtime.haveLastGood = true;
    runtime.lastGood = selectedProfile;
    runtime.lockEpoch++;
    runtime.samplesSinceRelock = 0u;
    if (selectedProfile.avgFreqHz > 0.0) {
        runtime.haveFreqAnchor = true;
        runtime.lockedFreqHz = selectedProfile.avgFreqHz;
    }
    printf("DBGFDC_S5D5,stage=boot_lock_ready,lockEpoch=%lu,deglitchName=%s,driveCurrent=0x%04X,"
           "highCurrent=%u,freqAnchorHz=%.3f,status=enter_locked_loop\n",
           (unsigned long)runtime.lockEpoch,
           selectedProfile.deglitchName ? selectedProfile.deglitchName : SENSORARRAY_NA,
           selectedProfile.driveCurrentReq,
           selectedProfile.highCurrentReq ? 1u : 0u,
           runtime.haveFreqAnchor ? runtime.lockedFreqHz : 0.0);

    sensorarrayS5d5Profile_t lockedProfile = selectedProfile;
    uint32_t degradedApplyFailStreak = 0u;
    printf("DBGFDC_S5D5,stage=locked_loop_enter,source=%s\n", lockSource);
    while (true) {
        sensorarrayS5d5LockedSummary_t lockedSummary = {0};
        bool needRelock = false;
        const char *relockReason = SENSORARRAY_NA;

        for (uint32_t burstIndex = 0u; burstIndex < lockedSampleCount; ++burstIndex) {
            uint32_t nextSampleIndex = runtime.sampleIndex + 1u;
            printf("DBGFDC_S5D5,stage=locked_sample_begin,index=%lu\n",
                   (unsigned long)nextSampleIndex);
            sensorarrayFdcReadDiag_t diag = {0};
            printf("DBGFDC_S5D5,stage=locked_sample_diag_begin,index=%lu\n",
                   (unsigned long)nextSampleIndex);
            esp_err_t readErr = sensorarrayS5d5ReadOneSampleBounded(fdcState->handle,
                                                                    (uint8_t)fdcMap->channel,
                                                                    unreadPollTimeoutMs,
                                                                    &diag);
            printf("DBGFDC_S5D5,stage=locked_sample_diag_done,index=%lu,err=%ld,i2cOk=%u,status=%s,statusCode=%u\n",
                   (unsigned long)nextSampleIndex,
                   (long)readErr,
                   diag.i2cOk ? 1u : 0u,
                   sensorarrayMeasureFdcSampleStatusName(diag.statusCode),
                   (unsigned)diag.statusCode);
            if (readErr != ESP_OK) {
                diag.err = readErr;
                diag.i2cOk = false;
            }

            uint16_t clockDividers = 0u;
            sensorarrayFdcFrequencyDiag_t freqDiag = {
                .refClockSource = sensorarrayMeasureFdcEffectiveRefClockSource(),
                .effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz(),
                .status = (readErr == ESP_OK) ? "clock_not_read" : "sample_not_read",
            };
            bool freqOk = false;
            if (readErr == ESP_OK) {
                esp_err_t clockErr = Fdc2214CapReadClockDividers(fdcState->handle, fdcMap->channel, &clockDividers);
                if (clockErr == ESP_OK) {
                    freqOk = sensorarrayMeasureFdcComputeFrequencyDiag(diag.sample.Raw28, clockDividers, &freqDiag);
                } else {
                    freqDiag.clockDividers = clockDividers;
                    freqDiag.status = "clock_read_error";
                }
            }

            bool activeChannelMatch = (readErr == ESP_OK) && (diag.sample.ActiveChannel == fdcMap->channel);
            if (readErr == ESP_OK) {
                runtime.readbackMismatchStreak = activeChannelMatch ? 0u : (runtime.readbackMismatchStreak + 1u);
            }

            bool sampleGood = (readErr == ESP_OK) &&
                              freqOk &&
                              freqDiag.valid &&
                              diag.provisionalReadable &&
                              diag.sample.UnreadConversionPresent &&
                              !diag.sample.ErrWatchdog &&
                              !diag.sample.ErrAmplitude &&
                              activeChannelMatch;
            double currentFreqHz = (freqOk && freqDiag.valid) ? freqDiag.freqHzCorrected : 0.0;
            const char *decisionReason = SENSORARRAY_NA;
            needRelock = directOnlyMode
                             ? false
                             : sensorarrayS5d5ShouldRelock(&runtime, &diag, currentFreqHz, &decisionReason);
            if (needRelock) {
                relockReason = decisionReason;
            }

            lockedSummary.totalSamples++;
            if (sampleGood) {
                lockedSummary.goodSamples++;
                runtime.samplesSinceRelock++;
                lockedProfile.avgFreqHz = currentFreqHz;
                runtime.lastGood.avgFreqHz = currentFreqHz;
                sensorarrayS5d5UpdateFreqAnchor(&runtime, currentFreqHz, true);
            } else {
                lockedSummary.warningSamples++;
            }
            if (readErr != ESP_OK) {
                lockedSummary.i2cErrorCount++;
            }
            if (diag.err == ESP_ERR_TIMEOUT) {
                lockedSummary.unreadTimeoutCount++;
            }
            if (readErr == ESP_OK && !diag.sample.Converting) {
                lockedSummary.nonConvertingCount++;
            }
            if (readErr == ESP_OK && diag.sample.ErrAmplitude) {
                lockedSummary.amplitudeFaultSamples++;
            }
            if (freqOk && freqDiag.valid) {
                lockedSummary.clockValidCount++;
                lockedSummary.freqSampleCount++;
                lockedSummary.freqHzSum += freqDiag.freqHzCorrected;
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
                capReason = freqDiag.status ? freqDiag.status : "invalid_frequency";
            }
            if (haveTotalCapPf) {
                lockedSummary.totalCapSampleCount++;
                lockedSummary.totalCapPfSum += totalCapPf;
            }
            if (haveNetCapPf) {
                lockedSummary.netCapSampleCount++;
                lockedSummary.netCapPfSum += netCapPf;
            }

            char totalCapField[64] = {0};
            char netCapField[64] = {0};
            if (haveTotalCapPf) {
                (void)snprintf(totalCapField, sizeof(totalCapField), "%.3f", totalCapPf);
            } else {
                (void)snprintf(totalCapField, sizeof(totalCapField), "na(reason=%s)", capReason);
            }
            if (haveNetCapPf) {
                (void)snprintf(netCapField, sizeof(netCapField), "%.3f", netCapPf);
            } else {
                (void)snprintf(netCapField,
                               sizeof(netCapField),
                               "na(reason=%s)",
                               capConfig.enableNetCapOutput ? capReason : "net_cap_output_disabled");
            }

            runtime.sampleIndex++;
            uint8_t currentDeglitchReadback =
                (readErr == ESP_OK) ? (uint8_t)(diag.sample.MuxRaw & SENSORARRAY_S5D5_MUX_CONFIG_DEGLITCH_MASK) : 0u;
            const char *sampleQuality = sampleGood ? "good" : "warning";
            const char *statusField = (readErr == ESP_OK)
                                          ? (freqOk ? sensorarrayMeasureFdcSampleStatusName(diag.statusCode)
                                                    : (freqDiag.status ? freqDiag.status : SENSORARRAY_NA))
                                          : ((readErr == ESP_ERR_TIMEOUT) ? "wait_unread_timeout" : "i2c_read_error");
            printf("DBGFDC_S5D5,stage=locked_sample,index=%lu,lockEpoch=%lu,highCurrent=%u,driveCurrent=0x%04X,"
                   "deglitchReq=0x%X,deglitchName=%s,deglitchBandwidthHz=%lu,deglitchReadback=0x%X,raw=%lu,"
                   "clockDiv=0x%04X,finSelCode=%u,finFactor=%u,frefDivider=%u,refClockSource=%s,"
                   "effectiveFclkHz=%lu,effectiveFrefHz=%.3f,freqHzBase=%.3f,freqHz=%.3f,totalCapPf=%s,"
                   "netCapPf=%s,inductorUh=%.3f,fixedCapPf=%.3f,parasiticCapPf=%.3f,unread=%u,converting=%u,"
                   "wd=%u,aw=%u,i2cErr=%u,sampleQuality=%s,status=%s,clockStatus=%s\n",
                   (unsigned long)runtime.sampleIndex,
                   (unsigned long)runtime.lockEpoch,
                   lockedProfile.highCurrentReq ? 1u : 0u,
                   lockedProfile.driveCurrentNorm,
                   (unsigned)lockedProfile.deglitchReq,
                   lockedProfile.deglitchName ? lockedProfile.deglitchName : SENSORARRAY_NA,
                   (unsigned long)lockedProfile.deglitchBandwidthHz,
                   (unsigned)currentDeglitchReadback,
                   (unsigned long)diag.sample.Raw28,
                   freqOk ? freqDiag.clockDividers : clockDividers,
                   freqOk ? (unsigned)freqDiag.finSelCode : 0u,
                   freqOk ? (unsigned)freqDiag.finFactor : 0u,
                   freqOk ? (unsigned)freqDiag.frefDivider : 0u,
                   sensorarrayMeasureFdcRefClockSourceName(freqDiag.refClockSource),
                   (unsigned long)freqDiag.effectiveFclkHz,
                   freqOk ? freqDiag.effectiveFrefHz : 0.0,
                   freqOk ? freqDiag.freqHzBase : 0.0,
                   freqOk ? freqDiag.freqHzCorrected : 0.0,
                   totalCapField,
                   netCapField,
                   capConfig.inductorValueUh,
                   capConfig.fixedCapPf,
                   capConfig.parasiticCapPf,
                   (readErr == ESP_OK && diag.sample.UnreadConversionPresent) ? 1u : 0u,
                   (readErr == ESP_OK && diag.sample.Converting) ? 1u : 0u,
                   (readErr == ESP_OK && diag.sample.ErrWatchdog) ? 1u : 0u,
                   (readErr == ESP_OK && diag.sample.ErrAmplitude) ? 1u : 0u,
                   (readErr == ESP_OK) ? 0u : 1u,
                   sampleQuality,
                   statusField,
                   freqDiag.status ? freqDiag.status : SENSORARRAY_NA);

            if (needRelock) {
                break;
            }
            sensorarrayS5d5DelayCooperativeMs(SENSORARRAY_S5D5_STEP_SAMPLE_GAP_MS);
        }

        bool haveAvgFreqHz = (lockedSummary.freqSampleCount > 0u);
        bool haveAvgTotalCapPf = (lockedSummary.totalCapSampleCount > 0u);
        bool haveAvgNetCapPf = (lockedSummary.netCapSampleCount > 0u);
        double avgFreqHz = haveAvgFreqHz ? (lockedSummary.freqHzSum / (double)lockedSummary.freqSampleCount) : 0.0;
        double avgTotalCapPf =
            haveAvgTotalCapPf ? (lockedSummary.totalCapPfSum / (double)lockedSummary.totalCapSampleCount) : 0.0;
        double avgNetCapPf = haveAvgNetCapPf ? (lockedSummary.netCapPfSum / (double)lockedSummary.netCapSampleCount) : 0.0;
        char avgFreqField[64] = {0};
        char avgTotalCapField[64] = {0};
        char avgNetCapField[64] = {0};
        if (haveAvgFreqHz) {
            (void)snprintf(avgFreqField, sizeof(avgFreqField), "%.3f", avgFreqHz);
        } else {
            (void)snprintf(avgFreqField, sizeof(avgFreqField), "na(reason=no_valid_frequency)");
        }
        if (haveAvgTotalCapPf) {
            (void)snprintf(avgTotalCapField, sizeof(avgTotalCapField), "%.3f", avgTotalCapPf);
        } else {
            (void)snprintf(avgTotalCapField, sizeof(avgTotalCapField), "na(reason=no_valid_cap_sample)");
        }
        if (haveAvgNetCapPf) {
            (void)snprintf(avgNetCapField, sizeof(avgNetCapField), "%.3f", avgNetCapPf);
        } else {
            (void)snprintf(avgNetCapField, sizeof(avgNetCapField), "na(reason=no_valid_net_cap_sample)");
        }

        printf("DBGFDC_S5D5,stage=locked_summary,lockEpoch=%lu,samples=%lu,good=%lu,warning=%lu,"
               "amplitudeFault=%lu,i2cErr=%lu,waitTimeout=%lu,nonConverting=%lu,clockValid=%lu,clockInvalid=%lu,"
               "refClockSource=%s,effectiveFclkHz=%lu,avgFreqHz=%s,avgTotalCapPf=%s,avgNetCapPf=%s,"
               "i2cStreak=%lu,timeoutStreak=%lu,nonConvertingStreak=%lu,readbackMismatchStreak=%lu,"
               "samplesSinceRelock=%lu,status=%s\n",
               (unsigned long)runtime.lockEpoch,
               (unsigned long)lockedSummary.totalSamples,
               (unsigned long)lockedSummary.goodSamples,
               (unsigned long)lockedSummary.warningSamples,
               (unsigned long)lockedSummary.amplitudeFaultSamples,
               (unsigned long)lockedSummary.i2cErrorCount,
               (unsigned long)lockedSummary.unreadTimeoutCount,
               (unsigned long)lockedSummary.nonConvertingCount,
               (unsigned long)lockedSummary.clockValidCount,
               (unsigned long)lockedSummary.clockInvalidCount,
               sensorarrayMeasureFdcRefClockSourceName(sensorarrayMeasureFdcEffectiveRefClockSource()),
               (unsigned long)sensorarrayMeasureFdcEffectiveFclkHz(),
               avgFreqField,
               avgTotalCapField,
               avgNetCapField,
               (unsigned long)runtime.i2cErrStreak,
               (unsigned long)runtime.waitTimeoutStreak,
               (unsigned long)runtime.nonConvertingStreak,
               (unsigned long)runtime.readbackMismatchStreak,
               (unsigned long)runtime.samplesSinceRelock,
               needRelock ? "relock_pending" : "locked_continue");

        if (needRelock) {
            sensorarrayS5d5Profile_t relockProfile = {0};
            bool relockOk = sensorarrayS5d5SelectRelockProfile(fdcState->handle,
                                                                (uint8_t)fdcMap->channel,
                                                                &runtime,
                                                                &relockProfile,
                                                                relockReason);
            if (relockOk &&
                sensorarrayS5d5ApplyRuntimeLockProfile(fdcState->handle,
                                                        (uint8_t)fdcMap->channel,
                                                        &runtime,
                                                        &relockProfile,
                                                        "relock_apply") == ESP_OK) {
                lockedProfile = relockProfile;
                degradedApplyFailStreak = 0u;
                continue;
            }

            sensorarrayS5d5Profile_t fallbackProfile = {0};
            sensorarrayS5d5MakeDefaultSafeProfile(&fallbackProfile);
            esp_err_t fallbackErr = sensorarrayS5d5ApplyRuntimeLockProfile(fdcState->handle,
                                                                           (uint8_t)fdcMap->channel,
                                                                           &runtime,
                                                                           &fallbackProfile,
                                                                           "degraded_default_fallback");
            printf("DBGFDC_S5D5,stage=degraded_fallback,reason=%s,err=%ld,status=%s\n",
                   relockReason ? relockReason : SENSORARRAY_NA,
                   (long)fallbackErr,
                   (fallbackErr == ESP_OK) ? "continue_locked_loop" : "apply_failed");
            if (fallbackErr == ESP_OK) {
                lockedProfile = fallbackProfile;
                degradedApplyFailStreak = 0u;
            } else {
                degradedApplyFailStreak++;
                if (degradedApplyFailStreak >= recoveryReinitThreshold) {
                    sensorarrayDebugIdleForever("s5d5_degraded_fallback_apply_failed");
                    return;
                }
            }
            continue;
        }

        sensorarrayS5d5DelayCooperativeMs(loopDelayMs);
    }
}
