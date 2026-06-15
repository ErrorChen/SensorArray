#include "sensorarrayAdsGap.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayConfig.h"
#include "sensorarrayAcqEvent.h"

#define printf sensorarrayAcqEventPrintf

#define SENSORARRAY_ADS_MUX_ANALOG_MONITOR 0x0Cu
#define SENSORARRAY_ADS_INTERNAL_VREF_UV 2500000u
#define SENSORARRAY_ADS_RAIL_SCALE 4
#define SENSORARRAY_ADS_RAIL_MIN_UV 3500000
#define SENSORARRAY_ADS_RAIL_MAX_UV 6000000
#define SENSORARRAY_ADS_BATTERY_PERIOD_FRAMES 10u
#define SENSORARRAY_ADS_ADC2_DATA_RATE_800_SPS 3u
#define SENSORARRAY_ADS_ADC2_TIMEOUT_US 2500u

typedef enum {
    SENSORARRAY_ADS_JOB_NONE = 0,
    SENSORARRAY_ADS_JOB_BATTERY,
    SENSORARRAY_ADS_JOB_ZERO,
    SENSORARRAY_ADS_JOB_RAIL,
} sensorarrayAdsJob_t;

static sensorarrayAdsGapSnapshot_t s_snapshot;
static uint32_t s_slackSamples;
static uint64_t s_slackTotalUs;
static uint32_t s_lastSampleFrame;
static uint32_t s_lastJobFrame;
static uint32_t s_lastBatteryFrame;
static uint32_t s_lastZeroFrame;
static uint32_t s_lastRailFrame;
static bool s_forceZero;
static bool s_forceRail;
static uint32_t s_zeroSampleCount;
static double s_zeroMeanUv;
static double s_zeroM2Uv;

static uint32_t sensorarrayAdsGapRemainingUs(uint64_t deadlineUs)
{
    int64_t nowUs = esp_timer_get_time();
    if (deadlineUs <= (uint64_t)nowUs) {
        return 0u;
    }
    uint64_t remaining = deadlineUs - (uint64_t)nowUs;
    return remaining > UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}

static void sensorarrayAdsGapRecordSlack(uint32_t slackUs)
{
    s_slackTotalUs += slackUs;
    s_slackSamples++;
    s_snapshot.avgSlackUs = s_slackSamples ?
        (uint32_t)(s_slackTotalUs / s_slackSamples) : 0u;
    if (s_snapshot.minSlackUs == 0u || slackUs < s_snapshot.minSlackUs) {
        s_snapshot.minSlackUs = slackUs;
    }
}

static void sensorarrayAdsGapRecordOverrun(void)
{
    s_snapshot.overrunCount++;
    if (s_snapshot.guardUs < 5000u) {
        s_snapshot.guardUs += 100u;
    }
    if (s_snapshot.overrunCount >= 3u) {
        s_snapshot.fallbackToBoundary = true;
    }
}

static uint16_t sensorarrayAdsChipValue(const ads126xAdcHandle_t *ads)
{
    return ads && ads->deviceType == ADS126X_DEVICE_ADS1263 ? 1263u : 1262u;
}

static void sensorarrayAdsUpdateZero(int32_t zeroUv, uint32_t frameSequence)
{
    s_zeroSampleCount++;
    double delta = (double)zeroUv - s_zeroMeanUv;
    s_zeroMeanUv += delta / (double)s_zeroSampleCount;
    double delta2 = (double)zeroUv - s_zeroMeanUv;
    s_zeroM2Uv += delta * delta2;
    double variance = s_zeroSampleCount > 1u ?
        s_zeroM2Uv / (double)(s_zeroSampleCount - 1u) : 0.0;

    s_snapshot.ain9OffsetUv = zeroUv;
    s_snapshot.zeroResidualUv = (int32_t)llround(s_zeroMeanUv);
    s_snapshot.zeroResidualStdUv = (uint32_t)llround(sqrt(variance));
    s_snapshot.zeroValid = true;
    s_snapshot.zeroAgeFrames = 0u;
    s_lastZeroFrame = frameSequence;
}

static void sensorarrayAdsUpdateBatteryValidity(bool adcReadOk)
{
    s_snapshot.batteryValid = false;
    s_snapshot.batteryMv = -1;
    if (!s_snapshot.bootCalibrationDone) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_CAL;
        return;
    }
    if (!s_snapshot.vbiasEnabled) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_VBIAS;
        return;
    }
    if (!s_snapshot.railValid) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_RAIL;
        return;
    }
    if (!s_snapshot.zeroValid) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_ZERO;
        return;
    }
    if (!s_snapshot.vrefSynced) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_VREF;
        return;
    }
    if (CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM != 2 ||
        CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_DEN != 1) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_DIV;
        return;
    }
    if (!adcReadOk) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_ADC;
        return;
    }

    int64_t vcmGndUv = (int64_t)s_snapshot.railUv / 2LL -
                       (int64_t)CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV;
    int64_t batteryUv = 2LL * ((int64_t)s_snapshot.ain8RawUv -
                               (int64_t)s_snapshot.zeroResidualUv +
                               vcmGndUv);
    if (batteryUv < 0LL || batteryUv > (int64_t)INT32_MAX * 1000LL) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_OVERFLOW;
        return;
    }

    s_snapshot.batteryMv = (int32_t)(batteryUv / 1000LL);
    s_snapshot.batteryValid = true;
    s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_NONE;
}

static esp_err_t sensorarrayAdsBootReadUv(ads126xAdcHandle_t *ads,
                                          uint8_t muxp,
                                          uint8_t muxn,
                                          int32_t *outRaw,
                                          int32_t *outUv)
{
    return ads126xAdcReadSingleDiffUv(ads,
                                      muxp,
                                      muxn,
                                      true,
                                      1u,
                                      1u,
                                      outRaw,
                                      outUv,
                                      NULL);
}

static esp_err_t sensorarrayAdsReadAdc2Uv(ads126xAdcHandle_t *ads,
                                          uint8_t muxp,
                                          uint8_t muxn,
                                          int32_t *outRaw,
                                          int32_t *outUv)
{
    esp_err_t err = ads126xAdcSetAdc2InputMux(ads, muxp, muxn);
    if (err == ESP_OK) {
        err = ads126xAdcStartAdc2(ads);
    }
    uint32_t dmaReadUs = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadAdc2RawDma(ads,
                                       SENSORARRAY_ADS_ADC2_TIMEOUT_US,
                                       outRaw,
                                       NULL,
                                       &dmaReadUs);
        s_snapshot.dmaReadCount++;
        s_snapshot.dmaReadUs += dmaReadUs;
    }
    esp_err_t stopErr = ads126xAdcStopAdc2(ads);
    if (err == ESP_OK) {
        err = stopErr;
    }
    if (err == ESP_OK) {
        *outUv = ads126xAdcAdc2RawToMicrovolts(ads, *outRaw);
    }
    return err;
}

static esp_err_t sensorarrayAdsBootCalibration(sensorarrayState_t *state)
{
    ads126xAdcHandle_t *ads = &state->ads;
    s_snapshot.id = ads->idRegRaw;
    s_snapshot.devId = ads126xAdcGetDevId(ads);
    s_snapshot.revId = ads126xAdcGetRevId(ads);
    s_snapshot.chip = sensorarrayAdsChipValue(ads);
    s_snapshot.hasAdc2 = ads126xAdcHasAdc2(ads);
    if (ads->deviceType == ADS126X_DEVICE_AUTO) {
        (void)ads126xAdcProbeAdc2(ads, &s_snapshot.hasAdc2);
        if (s_snapshot.hasAdc2) {
            s_snapshot.chip = 1263u;
        }
    }
    s_snapshot.activeAdc = 1u;

    printf("ADSBOOT,id=%02X,dev=%u,rev=%u,chip=%u,a2=%u\n",
           s_snapshot.id,
           s_snapshot.devId,
           s_snapshot.revId,
           s_snapshot.chip,
           s_snapshot.hasAdc2 ? 1u : 0u);

    esp_err_t err = ads126xAdcConfigure(ads,
                                        true,
                                        false,
                                        ADS126X_CRC_OFF,
                                        1u,
                                        (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE);
    if (err == ESP_OK) {
        err = ads126xAdcSetVbiasEnabled(ads, true);
    }
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(50u));
        err = ads126xAdcSetRefMuxWithVref(ads,
                                          ADS126X_REFMUX_INTERNAL,
                                          SENSORARRAY_ADS_INTERNAL_VREF_UV);
    }

    int32_t railMonitorRaw = 0;
    int32_t railMonitorUv = 0;
    if (err == ESP_OK) {
        err = sensorarrayAdsBootReadUv(ads,
                                       SENSORARRAY_ADS_MUX_ANALOG_MONITOR,
                                       SENSORARRAY_ADS_MUX_ANALOG_MONITOR,
                                       &railMonitorRaw,
                                       &railMonitorUv);
    }
    if (err == ESP_OK) {
        int64_t railUv = (int64_t)railMonitorUv * SENSORARRAY_ADS_RAIL_SCALE;
        s_snapshot.railUv = railUv > INT32_MAX ? INT32_MAX : (int32_t)railUv;
        s_snapshot.railValid = s_snapshot.railUv >= SENSORARRAY_ADS_RAIL_MIN_UV &&
                               s_snapshot.railUv <= SENSORARRAY_ADS_RAIL_MAX_UV;
        if (!s_snapshot.railValid) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetRefMuxWithVref(ads,
                                          ADS126X_REFMUX_AVDD_AVSS,
                                          (uint32_t)s_snapshot.railUv);
        s_snapshot.vrefSynced = err == ESP_OK;
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetInputMux(ads,
                                    SENSORARRAY_ADS_MUX_AIN9,
                                    SENSORARRAY_ADS_MUX_AINCOM);
    }
    if (err == ESP_OK) {
        err = ads126xAdcSystemOffsetCal(ads);
    }
    if (err == ESP_OK) {
        err = ads126xAdcWaitDrdy(ads, ads->drdyTimeoutMs);
    }

    int32_t zeroRaw = 0;
    int32_t zeroUv = 0;
    if (err == ESP_OK) {
        err = sensorarrayAdsBootReadUv(ads,
                                       SENSORARRAY_ADS_MUX_AIN9,
                                       SENSORARRAY_ADS_MUX_AINCOM,
                                       &zeroRaw,
                                       &zeroUv);
    }
    if (err == ESP_OK) {
        s_snapshot.ain9OffsetRaw = zeroRaw;
        sensorarrayAdsUpdateZero(zeroUv, 0u);
    }

    uint8_t adc2OffsetCal[2] = {0};
    uint8_t adc2FullScaleCal[2] = {0};
    if (err == ESP_OK && s_snapshot.hasAdc2) {
        err = ads126xAdcSetAdc2Config(ads,
                                      SENSORARRAY_ADS_ADC2_DATA_RATE_800_SPS,
                                      ADS126X_ADC2_REF_AVDD_AVSS,
                                      0u,
                                      (uint32_t)s_snapshot.railUv);
    }
    if (err == ESP_OK && s_snapshot.hasAdc2) {
        err = ads126xAdcSetAdc2InputMux(ads,
                                        SENSORARRAY_ADS_MUX_AIN9,
                                        SENSORARRAY_ADS_MUX_AINCOM);
    }
    if (err == ESP_OK && s_snapshot.hasAdc2) {
        err = ads126xAdcStartAdc2(ads);
    }
    if (err == ESP_OK && s_snapshot.hasAdc2) {
        err = ads126xAdcSystemOffsetCalAdc2(ads);
    }
    if (err == ESP_OK && s_snapshot.hasAdc2) {
        err = ads126xAdcWaitDrdy(ads, ads->drdyTimeoutMs);
    }
    if (s_snapshot.hasAdc2) {
        esp_err_t stopErr = ads126xAdcStopAdc2(ads);
        if (err == ESP_OK) {
            err = stopErr;
        }
    }
    if (err == ESP_OK && s_snapshot.hasAdc2) {
        err = ads126xAdcReadAdc2CalibrationRegisters(ads,
                                                     adc2OffsetCal,
                                                     adc2FullScaleCal);
    }
    if (err == ESP_OK && s_snapshot.hasAdc2) {
        s_snapshot.activeAdc = 2u;
    }

    uint8_t offsetCal[3] = {0};
    uint8_t fullScaleCal[3] = {0};
    uint8_t power = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    if (err == ESP_OK) {
        err = ads126xAdcReadCalibrationRegisters(ads, offsetCal, fullScaleCal);
    }
    if (err == ESP_OK) {
        err = ads126xAdcReadCoreRegisters(ads,
                                          &power,
                                          NULL,
                                          NULL,
                                          &inpmux,
                                          &refmux);
    }
    s_snapshot.vbiasEnabled = (power & ADS126X_POWER_VBIAS) != 0u;
    s_snapshot.bootCalibrationDone = err == ESP_OK;

    printf("CALB,chip=%u,adc=%u,rail=%ld,z=%ld/%lu,of=%02X%02X%02X,fs=%02X%02X%02X,a2of=%02X%02X,a2fs=%02X%02X,pw=%02X,im=%02X,rm=%02X,ok=%u,err=0x%lx\n",
           s_snapshot.chip,
           s_snapshot.activeAdc,
           (long)(s_snapshot.railUv / 1000),
           (long)s_snapshot.zeroResidualUv,
           (unsigned long)s_snapshot.zeroResidualStdUv,
           offsetCal[2], offsetCal[1], offsetCal[0],
           fullScaleCal[2], fullScaleCal[1], fullScaleCal[0],
           adc2OffsetCal[1], adc2OffsetCal[0],
           adc2FullScaleCal[1], adc2FullScaleCal[0],
           power,
           inpmux,
           refmux,
           s_snapshot.bootCalibrationDone ? 1u : 0u,
           (unsigned long)err);
    return err;
}

esp_err_t sensorarrayAdsGapInit(sensorarrayState_t *state)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    s_snapshot.guardUs = (uint32_t)CONFIG_SENSORARRAY_ADS_GAP_GUARD_US;
    s_snapshot.rateCode = (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE;
    s_snapshot.minSlackUs = UINT32_MAX;
    s_snapshot.batteryMv = -1;
    s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_CAL;
    s_slackSamples = 0u;
    s_slackTotalUs = 0u;
    s_lastSampleFrame = 0u;
    s_lastJobFrame = UINT32_MAX;
    s_lastBatteryFrame = 0u;
    s_lastZeroFrame = 0u;
    s_lastRailFrame = 0u;
    s_zeroSampleCount = 0u;
    s_zeroMeanUv = 0.0;
    s_zeroM2Uv = 0.0;
    s_forceZero = false;
    s_forceRail = false;

    if (!state || !state->adsReady) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (xPortGetCoreID() != CONFIG_SENSORARRAY_ADS_WORKER_CORE) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_config_t startConfig = {
        .pin_bit_mask = 1ULL << (uint32_t)CONFIG_SENSORARRAY_ADS_START_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&startConfig);
    if (err != ESP_OK) {
        return err;
    }
    gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 0);

    err = sensorarrayAdsBootCalibration(state);
    if (err == ESP_OK) {
        err = ads126xAdcEnableDrdyNotification(&state->ads);
    }
    if (err != ESP_OK) {
        sensorarrayAdsUpdateBatteryValidity(false);
        return err;
    }

    state->adsRefMux = ADS126X_REFMUX_AVDD_AVSS;
    state->adsRefMuxValid = true;
    s_snapshot.initialized = true;
    s_snapshot.dmaCapable = state->ads.spiDmaCapable;
    sensorarrayAdsUpdateBatteryValidity(false);
    return ESP_OK;
}

static sensorarrayAdsJob_t sensorarrayAdsSelectJob(uint32_t frameSequence)
{
    if (frameSequence == s_lastJobFrame) {
        return SENSORARRAY_ADS_JOB_NONE;
    }
    if (s_forceRail) {
        return SENSORARRAY_ADS_JOB_RAIL;
    }
    if (s_forceZero) {
        return SENSORARRAY_ADS_JOB_ZERO;
    }
    if (frameSequence - s_lastRailFrame >=
        (uint32_t)CONFIG_SENSORARRAY_ADS_RAIL_PERIOD_FRAMES) {
        return SENSORARRAY_ADS_JOB_RAIL;
    }
    if (frameSequence - s_lastZeroFrame >= (uint32_t)CONFIG_SENSORARRAY_ADS_ZERO_PERIOD_FRAMES) {
        return SENSORARRAY_ADS_JOB_ZERO;
    }
    if (frameSequence - s_lastBatteryFrame >= SENSORARRAY_ADS_BATTERY_PERIOD_FRAMES) {
        return SENSORARRAY_ADS_JOB_BATTERY;
    }
    return SENSORARRAY_ADS_JOB_NONE;
}

static esp_err_t sensorarrayAdsReadDma(sensorarrayState_t *state,
                                       uint8_t muxp,
                                       uint8_t muxn,
                                       int32_t *outRaw,
                                       int32_t *outUv)
{
    esp_err_t err = ads126xAdcSetInputMuxFast(&state->ads, muxp, muxn);
    uint32_t dmaReadUs = 0u;
    if (err == ESP_OK) {
        gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 1);
        int32_t discardRaw = 0;
        err = ads126xAdcReadAdc1RawDma(&state->ads,
                                       (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
                                       &discardRaw,
                                       NULL,
                                       &dmaReadUs);
        s_snapshot.dmaReadCount++;
        s_snapshot.dmaReadUs += dmaReadUs;
        /* DRDY's high release pulse can be narrower than task-level polling.
         * After consuming the stale conversion, wait one complete ADC period
         * and issue DMA RDATA directly instead of trying to observe that pulse. */
        uint32_t conversionUs = ads126xAdcExpectedConversionPeriodUs(
            (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE);
        esp_rom_delay_us(conversionUs + 50u);
        dmaReadUs = 0u;
        if (err == ESP_OK) {
            err = ads126xAdcReadAdc1RawDmaReady(&state->ads,
                                                outRaw,
                                                NULL,
                                                &dmaReadUs);
        }
        gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 0);
        s_snapshot.dmaReadCount++;
        s_snapshot.dmaReadUs += dmaReadUs;
    }
    if (err == ESP_OK) {
        *outUv = ads126xAdcRawToMicrovolts(&state->ads, *outRaw);
    }
    return err;
}

static esp_err_t sensorarrayAdsReadPreferred(sensorarrayState_t *state,
                                             uint8_t muxp,
                                             uint8_t muxn,
                                             int32_t *outRaw,
                                             int32_t *outUv)
{
    if (s_snapshot.hasAdc2) {
        return sensorarrayAdsReadAdc2Uv(&state->ads, muxp, muxn, outRaw, outUv);
    }
    return sensorarrayAdsReadDma(state, muxp, muxn, outRaw, outUv);
}

static esp_err_t sensorarrayAdsRunJob(sensorarrayState_t *state,
                                      sensorarrayAdsJob_t job,
                                      uint32_t frameSequence)
{
    int32_t raw = 0;
    int32_t uv = 0;
    esp_err_t err = ESP_OK;
    if (job == SENSORARRAY_ADS_JOB_RAIL) {
        if (s_snapshot.hasAdc2) {
            err = ads126xAdcSetAdc2Config(&state->ads,
                                          SENSORARRAY_ADS_ADC2_DATA_RATE_800_SPS,
                                          ADS126X_ADC2_REF_INTERNAL,
                                          0u,
                                          SENSORARRAY_ADS_INTERNAL_VREF_UV);
        } else {
            err = ads126xAdcSetRefMuxWithVref(&state->ads,
                                              ADS126X_REFMUX_INTERNAL,
                                              SENSORARRAY_ADS_INTERNAL_VREF_UV);
        }
        if (err == ESP_OK && !s_snapshot.hasAdc2) {
            esp_rom_delay_us(1000u);
        }
        if (err == ESP_OK) {
            err = sensorarrayAdsReadPreferred(state,
                                              SENSORARRAY_ADS_MUX_ANALOG_MONITOR,
                                              SENSORARRAY_ADS_MUX_ANALOG_MONITOR,
                                              &raw,
                                              &uv);
        }
        if (err == ESP_OK) {
            int64_t railUv = (int64_t)uv * SENSORARRAY_ADS_RAIL_SCALE;
            s_snapshot.railUv = railUv > INT32_MAX ? INT32_MAX : (int32_t)railUv;
            s_snapshot.railValid = s_snapshot.railUv >= SENSORARRAY_ADS_RAIL_MIN_UV &&
                                   s_snapshot.railUv <= SENSORARRAY_ADS_RAIL_MAX_UV;
            if (!s_snapshot.railValid) {
                err = ESP_ERR_INVALID_RESPONSE;
            }
        }
        uint32_t restoreVref = s_snapshot.railValid ?
            (uint32_t)s_snapshot.railUv : SENSORARRAY_ADS_INTERNAL_VREF_UV;
        esp_err_t restoreErr = s_snapshot.hasAdc2 ?
            ads126xAdcSetAdc2Config(&state->ads,
                                     SENSORARRAY_ADS_ADC2_DATA_RATE_800_SPS,
                                     ADS126X_ADC2_REF_AVDD_AVSS,
                                     0u,
                                     restoreVref) :
            ads126xAdcSetRefMuxWithVref(&state->ads,
                                        ADS126X_REFMUX_AVDD_AVSS,
                                        restoreVref);
        s_snapshot.vrefSynced = restoreErr == ESP_OK && s_snapshot.railValid;
        if (err == ESP_OK) {
            err = restoreErr;
        }
        if (err == ESP_OK) {
            s_lastRailFrame = frameSequence;
            s_forceRail = false;
        }
    } else if (job == SENSORARRAY_ADS_JOB_ZERO) {
        err = sensorarrayAdsReadPreferred(state,
                                          SENSORARRAY_ADS_MUX_AIN9,
                                          SENSORARRAY_ADS_MUX_AINCOM,
                                          &raw,
                                          &uv);
        if (err == ESP_OK) {
            s_snapshot.ain9OffsetRaw = raw;
            sensorarrayAdsUpdateZero(uv, frameSequence);
            s_forceZero = false;
        }
    } else if (job == SENSORARRAY_ADS_JOB_BATTERY) {
        err = sensorarrayAdsReadPreferred(state,
                                          SENSORARRAY_ADS_MUX_AIN8,
                                          SENSORARRAY_ADS_MUX_AINCOM,
                                          &raw,
                                          &uv);
        if (err == ESP_OK) {
            s_snapshot.ain8Raw = raw;
            s_snapshot.ain8RawUv = uv;
            s_lastBatteryFrame = frameSequence;
        }
        sensorarrayAdsUpdateBatteryValidity(err == ESP_OK);
    }
    return err;
}

void sensorarrayAdsGapRequestCalibration(bool requestZero, bool requestRail)
{
    /* This function is called only by the Acquisition domain at a frame
     * boundary. The gap scheduler consumes the flags later on the same core,
     * so no cross-core lock is required here. */
    s_forceZero = s_forceZero || requestZero;
    s_forceRail = s_forceRail || requestRail;
}

void sensorarrayAdsGapTryRun(sensorarrayState_t *state,
                             uint64_t expectedFdcReadyUs,
                             uint32_t frameSequence,
                             uint8_t row)
{
    (void)row;
    s_snapshot.windows++;
    if (!CONFIG_SENSORARRAY_ADS_GAP_FILL_ENABLE || !state || !state->adsReady ||
        !s_snapshot.initialized || s_snapshot.fallbackToBoundary) {
        s_snapshot.jobsSkip++;
        return;
    }

    sensorarrayAdsJob_t job = sensorarrayAdsSelectJob(frameSequence);
    if (job == SENSORARRAY_ADS_JOB_NONE) {
        return;
    }
    uint32_t remainingBeforeUs = sensorarrayAdsGapRemainingUs(expectedFdcReadyUs);
    uint32_t admissionUs = (uint32_t)CONFIG_SENSORARRAY_ADS_JOB_WORST_CASE_US +
                           s_snapshot.guardUs;
    if (remainingBeforeUs <= admissionUs) {
        s_snapshot.jobsSkip++;
        return;
    }

    s_lastJobFrame = frameSequence;
    esp_err_t err = sensorarrayAdsRunJob(state, job, frameSequence);
    if (err == ESP_ERR_TIMEOUT) {
        s_snapshot.drdyTimeoutCount++;
    } else if (err != ESP_OK) {
        s_snapshot.spiErrorCount++;
    }
    if (err == ESP_OK) {
        s_snapshot.jobsRun++;
        s_snapshot.timestampUs = (uint64_t)esp_timer_get_time();
        s_lastSampleFrame = frameSequence;
    } else {
        s_snapshot.jobsSkip++;
    }

    uint32_t slackUs = sensorarrayAdsGapRemainingUs(expectedFdcReadyUs);
    sensorarrayAdsGapRecordSlack(slackUs);
    if (slackUs < s_snapshot.guardUs) {
        sensorarrayAdsGapRecordOverrun();
    }
}

void sensorarrayAdsGapCopySnapshot(sensorarrayAdsGapSnapshot_t *outSnapshot,
                                   uint32_t frameSequence)
{
    if (!outSnapshot) {
        return;
    }
    *outSnapshot = s_snapshot;
    outSnapshot->sampleAgeFrames =
        (s_lastSampleFrame != 0u && frameSequence >= s_lastSampleFrame) ?
        (frameSequence - s_lastSampleFrame) : UINT32_MAX;
    outSnapshot->zeroAgeFrames =
        frameSequence >= s_lastZeroFrame ? frameSequence - s_lastZeroFrame : UINT32_MAX;
    if (outSnapshot->minSlackUs == UINT32_MAX) {
        outSnapshot->minSlackUs = 0u;
    }
}
