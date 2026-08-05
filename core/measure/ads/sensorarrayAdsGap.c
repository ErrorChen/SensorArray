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
#include "tmuxSwitch.h"

#define printf sensorarrayAcqEventPrintf

#define SENSORARRAY_ADS_MUX_ANALOG_MONITOR 0x0Cu
#define SENSORARRAY_ADS_INTERNAL_VREF_UV 2500000u
#define SENSORARRAY_ADS_RAIL_SCALE 4
#define SENSORARRAY_ADS_RAIL_MIN_UV 3500000
#define SENSORARRAY_ADS_RAIL_MAX_UV 6000000
#define SENSORARRAY_ADS_BATTERY_PERIOD_FRAMES 10u
#define SENSORARRAY_ADS_ADC1_JOB_WORST_CASE_US 2500u
#define SENSORARRAY_ADS_DIAGNOSTIC_SETTLE_US 1000u
#define SENSORARRAY_ADS_CONVERSION_MARGIN_US 100u
#define SENSORARRAY_ADS_RAIL_REFERENCE_SETTLE_US 5000u
#define SENSORARRAY_ADS_RAIL_MONITOR_SETTLE_MS 1u
#define SENSORARRAY_ADS_RAIL_MONITOR_DISCARD_COUNT 4u
#define SENSORARRAY_ADS_ADC1_GAP_DISCARD_COUNT 1u
#define SENSORARRAY_ADS_BATTERY_VBIAS_SETTLE_US 1000u
#define SENSORARRAY_ADS_BATTERY_DISCARD_COUNT 1u
#define SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT 3u
#define SENSORARRAY_ADS_REG_MODE2 0x05u
#define SENSORARRAY_ADS_REG_INTERFACE 0x02u
#define SENSORARRAY_ADS_REG_INPMUX 0x06u
#define SENSORARRAY_ADS_REG_OFCAL0 0x07u
#define SENSORARRAY_ADS_REG_FSCAL0 0x0Au

typedef enum {
    SENSORARRAY_ADS_JOB_NONE = 0,
    SENSORARRAY_ADS_JOB_BATTERY,
    SENSORARRAY_ADS_JOB_ZERO,
    SENSORARRAY_ADS_JOB_RAIL,
} sensorarrayAdsJob_t;

typedef struct {
    uint8_t power;
    uint8_t interface;
    uint8_t mode2;
    uint8_t inpmux;
    uint8_t refmux;
    uint8_t offsetCal[3];
    uint8_t fullScaleCal[3];
    uint32_t vrefMicrovolts;
    uint8_t pgaGain;
    uint8_t dataRateDr;
    ads126xCrcMode_t crcMode;
    bool enableInternalRef;
    bool enableStatusByte;
} sensorarrayAdsSavedState_t;

typedef struct {
    int32_t lastGoodRailUv;
    uint32_t lastGoodFrame;
    uint16_t validStreak;
    uint16_t invalidStreak;
    int32_t lastRawRailUv;
    int32_t lastRailErrUv;
    bool usableForBattery;
    bool diagnosticValid;
} sensorarrayAdsRailState_t;

typedef struct {
    sensorarrayAdsRailSource_t source;
    int32_t externalAvddUv;
    int32_t externalAvssUv;
} sensorarrayAdsRailCalibration_t;

typedef struct {
    uint8_t powerBefore;
    uint8_t powerAfter;
    uint8_t refmuxBefore;
    uint8_t refmuxDuring;
    uint8_t refmuxAfter;
    uint8_t inpmuxBefore;
    uint8_t inpmuxAfter;
    uint8_t mode2Before;
    uint8_t mode2After;
    uint32_t vrefBeforeUv;
    uint32_t vrefDuringUv;
    uint32_t vrefAfterUv;
    bool restored;
} sensorarrayAdsRailDiag_t;

typedef struct {
    uint8_t powerBefore;
    uint8_t powerRequested;
    uint8_t powerDuring;
    uint8_t powerAfter;
    uint8_t inpmux;
    uint8_t refmux;
    uint32_t settleUs;
    uint8_t discardCount;
    int32_t raw;
    int32_t ain8DiffUv;
    int32_t zeroUv;
    int32_t aincomGndUv;
    int32_t ain8GndUv;
    int32_t batteryMv;
    uint8_t status;
    uint32_t generationDelta;
    uint8_t sampleCount;
    esp_err_t readErr;
    bool fresh;
    bool collision;
    const char *reason;
    const char *stateName;
} sensorarrayAdsBatteryDiag_t;

typedef struct {
    int32_t raw;
    int32_t uv;
    uint8_t status;
    uint32_t generationStart;
    uint32_t generationEnd;
    uint32_t readUs;
    bool fresh;
} sensorarrayAdsFreshSample_t;

static sensorarrayAdsGapSnapshot_t s_snapshot;
static sensorarrayAdsRailState_t s_railState;
static sensorarrayAdsRailCalibration_t s_railCalibration;
static const uint8_t s_adsNeutralOffsetCal[3] = {0x00u, 0x00u, 0x00u};
static const uint8_t s_adsUnityFullScaleCal[3] = {0x00u, 0x00u, 0x40u};
static sensorarrayAdsGapMode_t s_gapMode =
#if CONFIG_SENSORARRAY_ADS_GAP_MODE_OFF
    SENSORARRAY_ADS_GAP_MODE_OFF;
#elif CONFIG_SENSORARRAY_ADS_GAP_MODE_RAIL
    SENSORARRAY_ADS_GAP_MODE_RAIL;
#elif CONFIG_SENSORARRAY_ADS_GAP_MODE_BAT
    SENSORARRAY_ADS_GAP_MODE_BAT;
#elif CONFIG_SENSORARRAY_ADS_GAP_MODE_ZERO
    SENSORARRAY_ADS_GAP_MODE_ZERO;
#else
    SENSORARRAY_ADS_GAP_MODE_ON;
#endif
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
static bool s_forceBattery;

static bool sensorarrayAdsRailUsableAsReference(int64_t railUv)
{
    return railUv >= SENSORARRAY_ADS_RAIL_MIN_UV &&
           railUv <= SENSORARRAY_ADS_RAIL_MAX_UV;
}

static bool sensorarrayAdsRailLastGoodFresh(uint32_t frameSequence)
{
    if (s_railState.lastGoodFrame == UINT32_MAX || s_railState.lastGoodRailUv <= 0) {
        return false;
    }
    uint32_t age = frameSequence >= s_railState.lastGoodFrame ?
        (frameSequence - s_railState.lastGoodFrame) : UINT32_MAX;
    return age <= (uint32_t)CONFIG_SENSORARRAY_ADS_RAIL_LAST_GOOD_MAX_AGE_FRAMES;
}

static bool sensorarrayAdsInternalReferenceIsUnclamped(void)
{
    tmuxSwitchControlState_t control = {0};
    if (tmuxSwitchGetControlState(&control) != ESP_OK) {
        return false;
    }
    /* The schematic connects SW high to the gate of Q1, which clamps the
     * shared REF/REFOUT node to GND. Only the observed low state is safe for
     * enabling INTREF. Require both the logical command and GPIO readback. */
    return control.cmdSource == TMUX1108_SOURCE_REF &&
           control.cmdSwLevel == 0 && control.obsSwLevel == 0;
}

static bool sensorarrayAdsUpdateRail(int32_t rawCode,
                                     int32_t monitorUv,
                                     uint32_t frameSequence)
{
    /* ADS126x analog supply monitor outputs approximately (AVDD - AVSS) / 4.
     * It must be measured against the internal 2.5 V reference; using
     * AVDD/AVSS as the reference would make the rail measure itself. */
    int64_t railUv = (int64_t)monitorUv * SENSORARRAY_ADS_RAIL_SCALE;
    int64_t expectedUv = (int64_t)CONFIG_SENSORARRAY_ADS_AVDD_TO_GND_UV +
                         (int64_t)CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV;
    int64_t errorUv = railUv - expectedUv;
    if (errorUv < 0) {
        errorUv = -errorUv;
    }
    int32_t rawRailUv = railUv > INT32_MAX ? INT32_MAX :
        (railUv < INT32_MIN ? INT32_MIN : (int32_t)railUv);
    s_snapshot.railRawUv = rawRailUv;
    s_snapshot.railMonitorUv = monitorUv;
    s_snapshot.railMonitorRaw = rawCode;
    s_snapshot.railExpectedUv =
        expectedUv > INT32_MAX ? INT32_MAX : (int32_t)expectedUv;
    s_snapshot.railErrorUv = errorUv > INT32_MAX ? INT32_MAX : (int32_t)errorUv;
    bool samplePlausible = sensorarrayAdsRailUsableAsReference(railUv);
    bool sampleValid =
        samplePlausible &&
        errorUv <= CONFIG_SENSORARRAY_ADS_RAIL_EXPECTED_TOLERANCE_UV;
    s_snapshot.railValid = sampleValid;
    s_railCalibration.source = SENSORARRAY_ADS_RAIL_SOURCE_MONITOR;
    s_railCalibration.externalAvddUv = 0;
    s_railCalibration.externalAvssUv = 0;
    s_railState.lastRawRailUv = rawRailUv;
    s_railState.lastRailErrUv = s_snapshot.railErrorUv;
    s_railState.diagnosticValid = true;

    if (sampleValid) {
        s_railState.lastGoodRailUv = rawRailUv;
        s_railState.lastGoodFrame = frameSequence;
        if (s_railState.validStreak < UINT16_MAX) {
            s_railState.validStreak++;
        }
        s_railState.invalidStreak = 0u;
        s_railState.usableForBattery = true;
        s_snapshot.railUv = rawRailUv;
        s_snapshot.railLastGoodUv = rawRailUv;
        s_snapshot.railAgeFrames = 0u;
        s_snapshot.railStatus = SENSORARRAY_ADS_RAIL_STATUS_OK;
    } else {
        if (s_railState.invalidStreak < UINT16_MAX) {
            s_railState.invalidStreak++;
        }
        s_railState.validStreak = 0u;
        bool hasLastGood = s_railState.lastGoodFrame != UINT32_MAX &&
                           s_railState.lastGoodRailUv > 0;
        bool holdFresh = hasLastGood &&
                         (samplePlausible ||
                          (sensorarrayAdsRailLastGoodFresh(frameSequence) &&
                           s_railState.invalidStreak <
                               (uint16_t)CONFIG_SENSORARRAY_ADS_RAIL_INVALID_STREAK_LIMIT));
        s_railState.usableForBattery = holdFresh;
        if (holdFresh) {
            s_snapshot.railUv = s_railState.lastGoodRailUv;
            s_snapshot.railLastGoodUv = s_railState.lastGoodRailUv;
            s_snapshot.railAgeFrames = frameSequence >= s_railState.lastGoodFrame ?
                frameSequence - s_railState.lastGoodFrame : UINT32_MAX;
            s_snapshot.railStatus = SENSORARRAY_ADS_RAIL_STATUS_HOLD;
        } else {
            s_snapshot.railUv = rawRailUv;
            s_snapshot.railLastGoodUv = s_railState.lastGoodRailUv;
            s_snapshot.railAgeFrames = UINT32_MAX;
            s_snapshot.railStatus = SENSORARRAY_ADS_RAIL_STATUS_BAD;
            s_snapshot.aincomGndValid = false;
            s_snapshot.ain8GndValid = false;
            s_snapshot.batteryValid = false;
            s_snapshot.batteryMv = -1;
            s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_RAIL;
        }
    }
    s_snapshot.railValidStreak = s_railState.validStreak;
    s_snapshot.railInvalidStreak = s_railState.invalidStreak;
    s_snapshot.railUsableForBattery = s_railState.usableForBattery;
    return sampleValid;
}

static bool sensorarrayAdsCalculateAincomGnd(int32_t *outUv)
{
    if (outUv && s_snapshot.railUsableForBattery &&
        s_railCalibration.source ==
            SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION) {
        int64_t midpointUv = ((int64_t)s_railCalibration.externalAvddUv +
                              (int64_t)s_railCalibration.externalAvssUv) / 2LL;
        if (midpointUv >= INT32_MIN && midpointUv <= INT32_MAX) {
            *outUv = (int32_t)midpointUv;
            return true;
        }
    }
    int64_t avddUv = CONFIG_SENSORARRAY_ADS_AVDD_TO_GND_UV;
    int64_t avssMagnitudeUv = CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV;
    int64_t nominalRailUv = avddUv + avssMagnitudeUv;
    if (!outUv || nominalRailUv <= 0 || !s_snapshot.railUsableForBattery) {
        return false;
    }

    /* VBIAS is the AVDD/AVSS midpoint. Scale the nominal positive/negative
     * rail split by the validated measured rail before locating that midpoint
     * relative to GND. */
    int64_t midpointUv = (int64_t)s_snapshot.railUv *
                         (avddUv - avssMagnitudeUv) /
                         (2LL * nominalRailUv);
    if (midpointUv < INT32_MIN || midpointUv > INT32_MAX) {
        return false;
    }
    *outUv = (int32_t)midpointUv;
    return true;
}

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

static int32_t sensorarrayAdsMedian3I32(int32_t a, int32_t b, int32_t c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a)) {
        return b;
    }
    if ((b <= a && a <= c) || (c <= a && a <= b)) {
        return a;
    }
    return c;
}

static void sensorarrayAdsRecordFreshSample(const sensorarrayAdsFreshSample_t *sample,
                                            esp_err_t err)
{
    if (sample) {
        s_snapshot.adcStatus = sample->status;
        s_snapshot.adcFresh = sample->fresh;
        s_snapshot.drdyGenerationDelta =
            sample->generationEnd - sample->generationStart;
    } else {
        s_snapshot.adcFresh = false;
        s_snapshot.drdyGenerationDelta = 0u;
    }

    if (err == ESP_ERR_TIMEOUT) {
        s_snapshot.adcStaleCount++;
    } else if (err == ESP_ERR_INVALID_RESPONSE) {
        if (sample && !sample->fresh) {
            s_snapshot.adcStaleCount++;
        } else {
            s_snapshot.adcStatusErrorCount++;
        }
    }
}

static esp_err_t sensorarrayAdsReadFreshAdc1(ads126xAdcHandle_t *ads,
                                             sensorarrayAdsFreshSample_t *outSample)
{
    if (!ads || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayAdsFreshSample_t sample = {0};
    sample.generationStart = ads126xAdcGetDrdyGeneration(ads);
    esp_err_t err = ads126xAdcWaitDrdyGenerationUs(
        ads,
        sample.generationStart,
        (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
        &sample.generationEnd);
    if (err == ESP_OK) {
        if (ads->spiDmaCapable) {
            err = ads126xAdcReadAdc1RawDmaReady(ads,
                                                &sample.raw,
                                                &sample.status,
                                                &sample.readUs);
            s_snapshot.dmaReadCount++;
            s_snapshot.dmaReadUs += sample.readUs;
        } else {
            int64_t readStartUs = esp_timer_get_time();
            err = ads126xAdcReadAdc1Raw(ads, &sample.raw, &sample.status);
            int64_t readElapsedUs = esp_timer_get_time() - readStartUs;
            s_snapshot.pollReadCount++;
            if (readElapsedUs > 0) {
                s_snapshot.pollReadUs += (uint32_t)readElapsedUs;
            }
        }
    }
    if (err == ESP_OK) {
        sample.uv = ads126xAdcRawToMicrovolts(ads, sample.raw);
        sample.fresh = ads126xAdcStatusByteHasAdc1NewData(ads, sample.status);
        if (!sample.fresh) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }

    *outSample = sample;
    sensorarrayAdsRecordFreshSample(&sample, err);
    return err;
}

static const char *sensorarrayAdsBatteryReasonName(sensorarrayBatteryInvalidReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_BATTERY_INVALID_NONE:
        return "ok";
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
    case SENSORARRAY_BATTERY_INVALID_DIV:
        return "divider_invalid";
    case SENSORARRAY_BATTERY_INVALID_NO_AINCOM_GND_REFERENCE:
        return "no_aincom";
    case SENSORARRAY_BATTERY_INVALID_REFERENCE_INVALID:
        return "reference_invalid";
    case SENSORARRAY_BATTERY_INVALID_ABSENT_OR_OPEN:
        return "absent_or_open";
    case SENSORARRAY_BATTERY_INVALID_RANGE_ERROR:
        return "range_error";
    case SENSORARRAY_BATTERY_INVALID_UNSTABLE:
        return "unstable";
    case SENSORARRAY_BATTERY_INVALID_OUT_OF_RANGE:
        return "out_of_range";
    case SENSORARRAY_BATTERY_INVALID_OVERFLOW:
        return "overflow";
    case SENSORARRAY_BATTERY_INVALID_UNKNOWN:
    default:
        return "unk";
    }
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

static void sensorarrayAdsUpdateBatteryValidity(esp_err_t adcReadErr)
{
    s_snapshot.batteryValid = false;
    s_snapshot.aincomGndValid = false;
    s_snapshot.ain8GndValid = false;
    s_snapshot.batteryMv = -1;
    if (!s_snapshot.bootCalibrationDone) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_CAL;
        return;
    }
    if (CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM <= 0 ||
        CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_DEN <= 0) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_DIV;
        return;
    }
    if (adcReadErr == ESP_ERR_TIMEOUT) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_ADC_TIMEOUT;
        return;
    }
    if (adcReadErr == ESP_ERR_INVALID_RESPONSE) {
        s_snapshot.batteryInvalidReason = s_snapshot.adcFresh ?
            SENSORARRAY_BATTERY_INVALID_ADC_STATUS_ERROR :
            SENSORARRAY_BATTERY_INVALID_ADC_STALE;
        return;
    }
    if (adcReadErr != ESP_OK) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_ADC;
        return;
    }
    if (!s_snapshot.adcFresh) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_ADC_STALE;
        return;
    }
    if (!s_snapshot.vbiasEnabled) {
        s_snapshot.batteryInvalidReason =
            SENSORARRAY_BATTERY_INVALID_NO_AINCOM_GND_REFERENCE;
        return;
    }
    if (!s_snapshot.railUsableForBattery) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_RAIL;
        return;
    }
    if (!sensorarrayAdsCalculateAincomGnd(&s_snapshot.aincomGndUv)) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_REFERENCE_INVALID;
        return;
    }
    s_snapshot.aincomGndValid = true;
    int64_t ain8GndUv = (int64_t)s_snapshot.ain8DiffUv +
                        (int64_t)s_snapshot.aincomGndUv;
    if (ain8GndUv < INT32_MIN || ain8GndUv > INT32_MAX) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_OVERFLOW;
        return;
    }
    s_snapshot.ain8GndUv = (int32_t)ain8GndUv;
    s_snapshot.ain8GndValid = true;
    int64_t batteryUv = ain8GndUv *
                        (int64_t)CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM /
                        (int64_t)CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_DEN;
    if (batteryUv < 0LL || batteryUv > (int64_t)INT32_MAX * 1000LL) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_OVERFLOW;
        return;
    }

    s_snapshot.batteryMv = (int32_t)(batteryUv / 1000LL);
    if (s_snapshot.batteryMv < CONFIG_SENSORARRAY_ADS_BATTERY_MIN_MV) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_ABSENT_OR_OPEN;
        s_snapshot.batteryMv = -1;
        return;
    }
    if (s_snapshot.batteryMv > CONFIG_SENSORARRAY_ADS_BATTERY_MAX_MV) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_RANGE_ERROR;
        s_snapshot.batteryMv = -1;
        return;
    }
    s_snapshot.batteryValid = true;
    s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_NONE;
}

static esp_err_t sensorarrayAdsReadAdc2Uv(ads126xAdcHandle_t *ads,
                                          uint8_t muxp,
                                          uint8_t muxn,
                                          int32_t *outRaw,
                                          int32_t *outUv)
{
    esp_err_t err = ads126xAdcStopAdc2(ads);
    if (err == ESP_OK) {
        err = ads126xAdcSetAdc2InputMux(ads, muxp, muxn);
    }
    if (err == ESP_OK) {
        esp_rom_delay_us(SENSORARRAY_ADS_DIAGNOSTIC_SETTLE_US);
    }
    if (err == ESP_OK) {
        err = ads126xAdcStartAdc2(ads);
    }
    uint32_t dmaReadUs = 0u;
    if (err == ESP_OK) {
        uint32_t conversionUs =
            ads126xAdcAdc2ExpectedConversionPeriodUs(ads->adc2DataRate);
        esp_rom_delay_us(conversionUs + SENSORARRAY_ADS_CONVERSION_MARGIN_US);
        err = ads126xAdcReadAdc2RawDmaReady(ads,
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

static esp_err_t sensorarrayAdsSaveState(ads126xAdcHandle_t *ads,
                                         sensorarrayAdsSavedState_t *saved)
{
    if (!ads || !saved) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(saved, 0, sizeof(*saved));
    esp_err_t err = ads126xAdcReadCoreRegisters(ads,
                                                &saved->power,
                                                &saved->interface,
                                                &saved->mode2,
                                                &saved->inpmux,
                                                &saved->refmux);
    if (err == ESP_OK) {
        err = ads126xAdcReadCalibrationRegisters(ads,
                                                 saved->offsetCal,
                                                 saved->fullScaleCal);
    }
    if (err == ESP_OK) {
        saved->vrefMicrovolts = ads->vrefMicrovolts;
        saved->pgaGain = ads->pgaGain;
        saved->dataRateDr = ads->dataRateDr;
        saved->crcMode = ads->crcMode;
        saved->enableInternalRef = ads->enableInternalRef;
        saved->enableStatusByte = ads->enableStatusByte;
    }
    return err;
}

static esp_err_t sensorarrayAdsWriteCalibration(ads126xAdcHandle_t *ads,
                                                const uint8_t offsetCal[3],
                                                const uint8_t fullScaleCal[3])
{
    if (!ads || !offsetCal || !fullScaleCal) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcWriteRegisters(ads,
                                             SENSORARRAY_ADS_REG_OFCAL0,
                                             offsetCal,
                                             3u);
    if (err == ESP_OK) {
        err = ads126xAdcWriteRegisters(ads,
                                       SENSORARRAY_ADS_REG_FSCAL0,
                                       fullScaleCal,
                                       3u);
    }
    return err;
}

static esp_err_t sensorarrayAdsRestoreState(ads126xAdcHandle_t *ads,
                                            const sensorarrayAdsSavedState_t *saved)
{
    if (!ads || !saved || saved->vrefMicrovolts == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t firstErr = ads126xAdcWritePowerRegister(ads, saved->power);
    esp_err_t err = ads126xAdcWriteRegisters(ads,
                                             SENSORARRAY_ADS_REG_INTERFACE,
                                             &saved->interface,
                                             1u);
    if (firstErr == ESP_OK) {
        firstErr = err;
    }
    err = ads126xAdcWriteRegisters(ads,
                                   SENSORARRAY_ADS_REG_MODE2,
                                   &saved->mode2,
                                   1u);
    if (firstErr == ESP_OK) {
        firstErr = err;
    }
    err = ads126xAdcSetRefMuxWithVref(ads, saved->refmux, saved->vrefMicrovolts);
    if (firstErr == ESP_OK) {
        firstErr = err;
    }
    err = ads126xAdcWriteRegisters(ads,
                                   SENSORARRAY_ADS_REG_INPMUX,
                                   &saved->inpmux,
                                   1u);
    if (firstErr == ESP_OK) {
        firstErr = err;
    }
    err = sensorarrayAdsWriteCalibration(ads,
                                         saved->offsetCal,
                                         saved->fullScaleCal);
    if (firstErr == ESP_OK) {
        firstErr = err;
    }
    if (firstErr == ESP_OK) {
        ads->enableInternalRef = saved->enableInternalRef;
        ads->enableStatusByte = saved->enableStatusByte;
        ads->crcMode = saved->crcMode;
        ads->pgaGain = saved->pgaGain;
        ads->dataRateDr = saved->dataRateDr;
        ads->vrefMicrovolts = saved->vrefMicrovolts;
    }
    return firstErr;
}

static esp_err_t sensorarrayAdsReadRailMonitorUv(
    sensorarrayState_t *state,
    int32_t *outRaw,
    int32_t *outMonitorUv,
    sensorarrayAdsRailDiag_t *outDiag)
{
    if (!state || !outRaw || !outMonitorUv) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayAdsInternalReferenceIsUnclamped()) {
        /* Never enable REFOUT while Q1 is commanded or observed clamping the
         * same net. VOLT/CAP must use a fresh external rail calibration. */
        return ESP_ERR_INVALID_STATE;
    }

    ads126xAdcHandle_t *ads = &state->ads;
    sensorarrayAdsSavedState_t saved = {0};
    sensorarrayAdsRailDiag_t diag = {0};
    esp_err_t err = sensorarrayAdsSaveState(ads, &saved);
    if (err != ESP_OK) {
        return err;
    }
    diag.powerBefore = saved.power;
    diag.refmuxBefore = saved.refmux;
    diag.inpmuxBefore = saved.inpmux;
    diag.mode2Before = saved.mode2;
    diag.vrefBeforeUv = saved.vrefMicrovolts;

    if (ads126xAdcHasAdc2(ads)) {
        (void)ads126xAdcStopAdc2(ads);
    }
    (void)ads126xAdcStopAdc1(ads);
    err = ads126xAdcSetInternalReference(ads, true);
    if (err == ESP_OK) {
        err = ads126xAdcSetVbiasEnabled(ads, true);
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetRefMuxWithVref(ads,
                                          ADS126X_REFMUX_INTERNAL,
                                          SENSORARRAY_ADS_INTERNAL_VREF_UV);
    }
    if (err == ESP_OK) {
        /* The boot calibration path already lets the internal reference settle
         * before measuring the analog supply monitor. Runtime rail jobs switch
         * from AVDD/AVSS reference back to the internal 2.5 V reference, so
         * give the reference mux and monitor source their own small settling
         * window before consuming DRDY. Without this, the first periodic rail
         * sample can be biased high even after normal conversion discards. */
        esp_rom_delay_us(SENSORARRAY_ADS_RAIL_REFERENCE_SETTLE_US);
    }
    if (err == ESP_OK) {
        /* System-offset calibration is intentionally tied to the board's AIN9
         * zero path. The internal analog supply monitor is a different source,
         * so read it with neutral ADC1 calibration and restore the board
         * calibration before returning to AIN8/AIN9 jobs. */
        err = sensorarrayAdsWriteCalibration(ads,
                                             s_adsNeutralOffsetCal,
                                             s_adsUnityFullScaleCal);
    }
    if (err == ESP_OK) {
        diag.refmuxDuring = ADS126X_REFMUX_INTERNAL;
        diag.vrefDuringUv = SENSORARRAY_ADS_INTERNAL_VREF_UV;
        err = ads126xAdcSetInputMux(ads,
                                    SENSORARRAY_ADS_MUX_ANALOG_MONITOR,
                                    SENSORARRAY_ADS_MUX_ANALOG_MONITOR);
    }
    if (err == ESP_OK && SENSORARRAY_ADS_RAIL_MONITOR_SETTLE_MS > 0u) {
        vTaskDelay(pdMS_TO_TICKS(SENSORARRAY_ADS_RAIL_MONITOR_SETTLE_MS));
    }
    if (err == ESP_OK) {
        ads126xAdcClearDrdyNotifications(ads);
        err = ads126xAdcStartAdc1(ads);
    }
    if (err == ESP_OK) {
        uint8_t readCount = (uint8_t)(SENSORARRAY_ADS_RAIL_MONITOR_DISCARD_COUNT + 1u);
        sensorarrayAdsFreshSample_t sample = {0};
        for (uint8_t readIdx = 0u; readIdx < readCount; ++readIdx) {
            err = sensorarrayAdsReadFreshAdc1(ads, &sample);
            if (err != ESP_OK) {
                break;
            }
        }
        if (err == ESP_OK) {
            *outRaw = sample.raw;
            *outMonitorUv = sample.uv;
        }
    }
    (void)ads126xAdcStopAdc1(ads);

    esp_err_t restoreErr = sensorarrayAdsRestoreState(ads, &saved);
    if (restoreErr == ESP_OK) {
        restoreErr = ads126xAdcStartAdc1(ads);
    }
    diag.restored = restoreErr == ESP_OK;
    uint8_t powerAfter = 0u;
    uint8_t mode2After = 0u;
    uint8_t inpmuxAfter = 0u;
    uint8_t refmuxAfter = 0u;
    if (ads126xAdcReadCoreRegisters(ads,
                                    &powerAfter,
                                    NULL,
                                    &mode2After,
                                    &inpmuxAfter,
                                    &refmuxAfter) == ESP_OK) {
        diag.powerAfter = powerAfter;
        diag.mode2After = mode2After;
        diag.inpmuxAfter = inpmuxAfter;
        diag.refmuxAfter = refmuxAfter;
        diag.vrefAfterUv = ads->vrefMicrovolts;
    }
    if (outDiag) {
        *outDiag = diag;
    }
    if (restoreErr != ESP_OK) {
        s_snapshot.fallbackToBoundary = true;
        printf("ERR,mod=ads,code=rail_restore,ctx=power/refmux/inpmux/mode,act=gap_pause,seq=0,err=0x%lx\n",
               (unsigned long)restoreErr);
        return restoreErr;
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
                                        false,
                                        true,
                                        ADS126X_CRC_OFF,
                                        1u,
                                        (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE);
    if (err == ESP_OK) {
        err = ads126xAdcSetVbiasEnabled(ads, true);
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetRefMuxWithVref(ads,
                                          ADS126X_REFMUX_AVDD_AVSS,
                                          (uint32_t)(CONFIG_SENSORARRAY_ADS_AVDD_TO_GND_UV +
                                                     CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV));
    }
    if (err == ESP_OK) {
        err = sensorarrayAdsWriteCalibration(ads,
                                             s_adsNeutralOffsetCal,
                                             s_adsUnityFullScaleCal);
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetInputMux(ads,
                                    SENSORARRAY_ADS_MUX_AIN9,
                                    SENSORARRAY_ADS_MUX_AINCOM);
    }
    if (err == ESP_OK) {
        err = ads126xAdcClearResetFlag(ads);
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
    s_snapshot.vrefSynced = false;
    s_snapshot.bootCalibrationDone = err == ESP_OK;

    printf("CALB,chip=%u,adc=%u,rail=invalid,z=deferred,of=%02X%02X%02X,fs=%02X%02X%02X,pw=%02X,im=%02X,rm=%02X,intref=%u,vbias=%u,mode=passive_deferred,ok=%u,err=0x%lx\n",
           s_snapshot.chip,
           s_snapshot.activeAdc,
           offsetCal[2], offsetCal[1], offsetCal[0],
           fullScaleCal[2], fullScaleCal[1], fullScaleCal[0],
           power,
           inpmux,
           refmux,
           (power & ADS126X_POWER_INTREF) != 0u ? 1u : 0u,
           (power & ADS126X_POWER_VBIAS) != 0u ? 1u : 0u,
           s_snapshot.bootCalibrationDone ? 1u : 0u,
           (unsigned long)err);
    return err;
}

esp_err_t sensorarrayAdsGapInit(sensorarrayState_t *state)
{
    memset(&s_snapshot, 0, sizeof(s_snapshot));
    memset(&s_railState, 0, sizeof(s_railState));
    memset(&s_railCalibration, 0, sizeof(s_railCalibration));
    s_snapshot.guardUs = (uint32_t)CONFIG_SENSORARRAY_ADS_GAP_GUARD_US;
    s_snapshot.rateCode = (uint8_t)CONFIG_SENSORARRAY_ADS_DATA_RATE;
    s_snapshot.minSlackUs = UINT32_MAX;
    s_snapshot.railStatus = SENSORARRAY_ADS_RAIL_STATUS_BAD;
    s_snapshot.railAgeFrames = UINT32_MAX;
    s_snapshot.batteryMv = -1;
    s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_CAL;
    s_railState.lastGoodFrame = UINT32_MAX;
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
    s_forceBattery = false;

    if (!state || !state->adsReady) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (xPortGetCoreID() != CONFIG_SENSORARRAY_ADS_WORKER_CORE) {
        printf("ADSGAP,stage=init,core=%d,workerCore=%d,action=allow_startup_calibration\n",
               (int)xPortGetCoreID(),
               CONFIG_SENSORARRAY_ADS_WORKER_CORE);
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

    /*
     * Boot calibration already uses the same fresh-conversion helper as
     * runtime BATD/rail jobs. That helper waits for an advancing DRDY
     * generation, so the GPIO ISR must be owned by boardSupport before the
     * first calibration conversion is started.
     */
    err = ads126xAdcEnableDrdyNotification(&state->ads);
    if (err == ESP_OK) {
        err = sensorarrayAdsBootCalibration(state);
    }
    if (err != ESP_OK) {
        sensorarrayAdsUpdateBatteryValidity(err);
        return err;
    }

    state->adsRefMux = ADS126X_REFMUX_AVDD_AVSS;
    state->adsRefMuxValid = true;
    s_snapshot.initialized = true;
    s_snapshot.dmaCapable = state->ads.spiDmaCapable;
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_STATE);
    return ESP_OK;
}

esp_err_t sensorarrayAdsGapRefreshRailAtBoundary(sensorarrayState_t *state,
                                                 uint32_t frameSequence)
{
    if (!state || !state->adsReady || !s_snapshot.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    int32_t railMonitorRaw = 0;
    int32_t railMonitorUv = 0;
    sensorarrayAdsRailDiag_t railDiag = {0};
    esp_err_t err = sensorarrayAdsReadRailMonitorUv(state,
                                                    &railMonitorRaw,
                                                    &railMonitorUv,
                                                    &railDiag);
    if (err == ESP_OK) {
        (void)sensorarrayAdsUpdateRail(railMonitorRaw,
                                      railMonitorUv,
                                      frameSequence);
        s_lastRailFrame = frameSequence;
        s_lastSampleFrame = frameSequence;
        s_snapshot.timestampUs = (uint64_t)esp_timer_get_time();
        s_snapshot.vrefSynced = railDiag.restored;
        if (!s_snapshot.railValid ||
            s_snapshot.railStatus == SENSORARRAY_ADS_RAIL_STATUS_BAD) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
    }
    if (err == ESP_ERR_TIMEOUT) {
        s_snapshot.drdyTimeoutCount++;
    } else if (err != ESP_OK) {
        s_snapshot.spiErrorCount++;
    }
    return err;
}

esp_err_t sensorarrayAdsGapSetExternalRailCalibration(int32_t avddUv,
                                                       int32_t avssUv,
                                                       uint32_t frameSequence)
{
    if (!s_snapshot.initialized || avddUv <= 0 || avssUv >= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    int64_t railUv = (int64_t)avddUv - (int64_t)avssUv;
    if (!sensorarrayAdsRailUsableAsReference(railUv) || railUv > INT32_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    s_railCalibration = (sensorarrayAdsRailCalibration_t){
        .source = SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION,
        .externalAvddUv = avddUv,
        .externalAvssUv = avssUv,
    };
    s_railState.lastGoodRailUv = (int32_t)railUv;
    s_railState.lastGoodFrame = frameSequence;
    s_railState.validStreak = 1u;
    s_railState.invalidStreak = 0u;
    s_railState.usableForBattery = true;
    s_railState.diagnosticValid = true;
    s_snapshot.railUv = (int32_t)railUv;
    s_snapshot.railRawUv = (int32_t)railUv;
    s_snapshot.railLastGoodUv = (int32_t)railUv;
    s_snapshot.railExpectedUv = (int32_t)railUv;
    s_snapshot.railErrorUv = 0;
    s_snapshot.railMonitorRaw = 0;
    s_snapshot.railMonitorUv = 0;
    s_snapshot.railAgeFrames = 0u;
    s_snapshot.railValidStreak = 1u;
    s_snapshot.railInvalidStreak = 0u;
    s_snapshot.railUsableForBattery = true;
    s_snapshot.railValid = true;
    s_snapshot.railStatus = SENSORARRAY_ADS_RAIL_STATUS_OK;
    s_snapshot.timestampUs = (uint64_t)esp_timer_get_time();
    s_lastRailFrame = frameSequence;
    s_lastSampleFrame = frameSequence;
    return ESP_OK;
}

bool sensorarrayAdsGapCopyRailSplit(uint32_t frameSequence,
                                    uint32_t maximumAgeFrames,
                                    sensorarrayAdsRailSplit_t *outRail)
{
    if (!outRail) {
        return false;
    }
    sensorarrayAdsGapSnapshot_t snapshot = {0};
    sensorarrayAdsGapCopySnapshot(&snapshot, frameSequence);
    if (!snapshot.railValid || snapshot.railAgeFrames > maximumAgeFrames) {
        *outRail = (sensorarrayAdsRailSplit_t){0};
        return false;
    }
    if (s_railCalibration.source ==
        SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION) {
        int64_t totalUv = (int64_t)s_railCalibration.externalAvddUv -
                          (int64_t)s_railCalibration.externalAvssUv;
        int64_t midpointUv = ((int64_t)s_railCalibration.externalAvddUv +
                              (int64_t)s_railCalibration.externalAvssUv) / 2LL;
        if (!sensorarrayAdsRailUsableAsReference(totalUv) ||
            midpointUv < INT32_MIN || midpointUv > INT32_MAX) {
            *outRail = (sensorarrayAdsRailSplit_t){0};
            return false;
        }
        *outRail = (sensorarrayAdsRailSplit_t){
            .avddUv = s_railCalibration.externalAvddUv,
            .avssUv = s_railCalibration.externalAvssUv,
            .aincomUv = (int32_t)midpointUv,
            .ageFrames = snapshot.railAgeFrames,
            .valid = true,
        };
        return true;
    }

    sensorarrayAdsRailInput_t input = {
        .measuredRailUv = snapshot.railUv,
        .nominalAvddToGroundUv = CONFIG_SENSORARRAY_ADS_AVDD_TO_GND_UV,
        .nominalAvssToGroundUv = CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV,
        .ageFrames = snapshot.railAgeFrames,
        .maximumAgeFrames = maximumAgeFrames,
        .valid = snapshot.railValid,
    };
    return sensorarrayAdsMathSplitRail(&input, outRail);
}

static sensorarrayAdsJob_t sensorarrayAdsSelectJob(uint32_t frameSequence)
{
    if (frameSequence == s_lastJobFrame) {
        return SENSORARRAY_ADS_JOB_NONE;
    }
    if (s_forceRail) {
        return SENSORARRAY_ADS_JOB_RAIL;
    }
    if (s_forceBattery) {
        return SENSORARRAY_ADS_JOB_BATTERY;
    }
    if (s_forceZero) {
        return SENSORARRAY_ADS_JOB_ZERO;
    }
    if (s_gapMode == SENSORARRAY_ADS_GAP_MODE_OFF) {
        return SENSORARRAY_ADS_JOB_NONE;
    }
    if (s_gapMode == SENSORARRAY_ADS_GAP_MODE_RAIL) {
        return sensorarrayAdsInternalReferenceIsUnclamped() ?
            SENSORARRAY_ADS_JOB_RAIL : SENSORARRAY_ADS_JOB_NONE;
    }
    if (s_gapMode == SENSORARRAY_ADS_GAP_MODE_BAT) {
        return SENSORARRAY_ADS_JOB_BATTERY;
    }
    if (s_gapMode == SENSORARRAY_ADS_GAP_MODE_ZERO) {
        return SENSORARRAY_ADS_JOB_ZERO;
    }
    if (frameSequence - s_lastRailFrame >=
        (uint32_t)CONFIG_SENSORARRAY_ADS_RAIL_PERIOD_FRAMES) {
        if (sensorarrayAdsInternalReferenceIsUnclamped()) {
            return SENSORARRAY_ADS_JOB_RAIL;
        }
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
    if (err == ESP_OK) {
        /* Internal monitors and the external divider need more than one
         * 38.4-kSPS period after a mux change. Match the proven boot-read
         * settling interval, then require a new DRDY generation for each
         * conversion that is consumed. */
        esp_rom_delay_us(SENSORARRAY_ADS_DIAGNOSTIC_SETTLE_US);
        ads126xAdcClearDrdyNotifications(&state->ads);
        gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 1);
        sensorarrayAdsFreshSample_t sample = {0};
        uint8_t readCount = (uint8_t)(SENSORARRAY_ADS_ADC1_GAP_DISCARD_COUNT + 1u);
        for (uint8_t readIdx = 0u; readIdx < readCount; ++readIdx) {
            err = sensorarrayAdsReadFreshAdc1(&state->ads, &sample);
            if (err != ESP_OK) {
                break;
            }
        }
        gpio_set_level((gpio_num_t)CONFIG_SENSORARRAY_ADS_START_GPIO, 0);
        if (err == ESP_OK) {
            *outRaw = sample.raw;
            *outUv = sample.uv;
        }
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

static void sensorarrayAdsPrintBatteryDiag(const sensorarrayAdsBatteryDiag_t *diag)
{
    if (!diag) {
        return;
    }
    printf("BATD,mode=vbias_on,POWER=%02X/%02X/%02X/%02X,VBIAS=%u,INPMUX=%02X,REFMUX=%02X,"
           "settleUs=%lu,discardCount=%u,samples=%u,read=%s,state=%s,fresh=%u,status=0x%02X,dg=%lu,"
           "raw=%ld,a8dUv=%ld,zeroUv=%ld,acUv=%ld,a8gUv=%ld,batteryMv=%ld,collision=%u,err=0x%lx,reason=%s\n",
           diag->powerBefore,
           diag->powerRequested,
           diag->powerDuring,
           diag->powerAfter,
           (diag->powerDuring & ADS126X_POWER_VBIAS) != 0u ? 1u : 0u,
           diag->inpmux,
           diag->refmux,
           (unsigned long)diag->settleUs,
           (unsigned)diag->discardCount,
           (unsigned)diag->sampleCount,
           diag->readErr == ESP_OK ? "ok" : "fail",
           diag->stateName ? diag->stateName : "unk",
           diag->fresh ? 1u : 0u,
           (unsigned)diag->status,
           (unsigned long)diag->generationDelta,
           (long)diag->raw,
           (long)diag->ain8DiffUv,
           (long)diag->zeroUv,
           (long)diag->aincomGndUv,
           (long)diag->ain8GndUv,
           (long)diag->batteryMv,
           diag->collision ? 1u : 0u,
           (unsigned long)diag->readErr,
           diag->reason ? diag->reason : "unknown");
}

static esp_err_t sensorarrayAdsFinishBatteryTransaction(esp_err_t err,
                                                        sensorarrayAdsBatteryDiag_t *diag,
                                                        sensorarrayAdsBatteryDiag_t *outDiag)
{
    if (diag) {
        diag->readErr = err;
        if (outDiag) {
            *outDiag = *diag;
        }
    }
    return err;
}

static esp_err_t sensorarrayAdsReadBatteryTransaction(sensorarrayState_t *state,
                                                      int32_t *outRaw,
                                                      int32_t *outUv,
                                                      sensorarrayAdsBatteryDiag_t *outDiag)
{
    if (!state || !outRaw || !outUv || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    ads126xAdcHandle_t *ads = &state->ads;
    sensorarrayAdsBatteryDiag_t diag = {
        .settleUs = SENSORARRAY_ADS_BATTERY_VBIAS_SETTLE_US,
        .discardCount = SENSORARRAY_ADS_BATTERY_DISCARD_COUNT,
        .zeroUv = s_snapshot.zeroResidualUv,
        .batteryMv = -1,
        .reason = "begin",
    };

    esp_err_t err = ads126xAdcReadPowerRegister(ads, &diag.powerBefore);
    if (err != ESP_OK) {
        diag.reason = "power_before_read";
        return sensorarrayAdsFinishBatteryTransaction(err, &diag, outDiag);
    }

    if (ads126xAdcHasAdc2(ads)) {
        (void)ads126xAdcStopAdc2(ads);
    }
    if (state->adsAdc1Running) {
        err = ads126xAdcStopAdc1(ads);
        if (err != ESP_OK) {
            diag.reason = "stop_adc1";
            return sensorarrayAdsFinishBatteryTransaction(err, &diag, outDiag);
        }
        state->adsAdc1Running = false;
    }

    err = ads126xAdcSetVbiasEnabled(ads, true);
    if (err == ESP_OK) {
        err = ads126xAdcReadPowerRegister(ads, &diag.powerDuring);
    }
    diag.powerRequested = (uint8_t)(diag.powerBefore | ADS126X_POWER_VBIAS);
    s_snapshot.vbiasEnabled = (diag.powerDuring & ADS126X_POWER_VBIAS) != 0u;
    if (err != ESP_OK || !s_snapshot.vbiasEnabled) {
        diag.reason = (err == ESP_OK) ? "vbias_readback_off" : "vbias_on";
        err = (err == ESP_OK) ? ESP_ERR_INVALID_RESPONSE : err;
        return sensorarrayAdsFinishBatteryTransaction(err, &diag, outDiag);
    }

    esp_rom_delay_us(SENSORARRAY_ADS_BATTERY_VBIAS_SETTLE_US);

    uint32_t refUv = s_snapshot.railUv > 0 ?
        (uint32_t)s_snapshot.railUv :
        (uint32_t)(CONFIG_SENSORARRAY_ADS_AVDD_TO_GND_UV +
                   CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV);
    err = ads126xAdcSetRefMuxWithVref(ads, ADS126X_REFMUX_AVDD_AVSS, refUv);
    if (err == ESP_OK) {
        state->adsRefMux = ADS126X_REFMUX_AVDD_AVSS;
        state->adsRefMuxValid = true;
        err = ads126xAdcSetInputMux(ads, SENSORARRAY_ADS_MUX_AIN8, SENSORARRAY_ADS_MUX_AINCOM);
    }
    if (err == ESP_OK) {
        err = ads126xAdcReadCoreRegisters(ads,
                                          &diag.powerAfter,
                                          NULL,
                                          NULL,
                                          &diag.inpmux,
                                          &diag.refmux);
    }
    if (err != ESP_OK) {
        diag.reason = "mux_or_ref";
        return sensorarrayAdsFinishBatteryTransaction(err, &diag, outDiag);
    }

    ads126xAdcClearDrdyNotifications(ads);
    err = ads126xAdcStartAdc1(ads);
    if (err == ESP_OK) {
        state->adsAdc1Running = true;
        sensorarrayAdsFreshSample_t discard = {0};
        for (uint8_t readIndex = 0u; readIndex < SENSORARRAY_ADS_BATTERY_DISCARD_COUNT;
             ++readIndex) {
            err = sensorarrayAdsReadFreshAdc1(ads, &discard);
            if (err != ESP_OK) {
                break;
            }
        }
        sensorarrayAdsFreshSample_t samples[SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT] = {0};
        if (err == ESP_OK) {
            for (uint8_t sampleIndex = 0u; sampleIndex < SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT;
                 ++sampleIndex) {
                err = sensorarrayAdsReadFreshAdc1(ads, &samples[sampleIndex]);
                if (err != ESP_OK) {
                    break;
                }
                diag.sampleCount++;
            }
        }
        if (err == ESP_OK) {
            int32_t medianRaw = sensorarrayAdsMedian3I32(samples[0].raw,
                                                         samples[1].raw,
                                                         samples[2].raw);
            *outRaw = medianRaw;
            *outUv = ads126xAdcRawToMicrovolts(ads, medianRaw);
            diag.raw = medianRaw;
            diag.ain8DiffUv = *outUv;
            diag.status = samples[SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT - 1u].status;
            diag.fresh = samples[SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT - 1u].fresh;
            diag.generationDelta =
                samples[SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT - 1u].generationEnd -
                samples[SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT - 1u].generationStart;
        }
    }
    if (err != ESP_OK) {
        diag.reason = "fresh_conversion";
        return sensorarrayAdsFinishBatteryTransaction(err, &diag, outDiag);
    }

    int32_t aincomGndUv = 0;
    if (sensorarrayAdsCalculateAincomGnd(&aincomGndUv)) {
        int64_t ain8GndUv = (int64_t)(*outUv) +
                            (int64_t)aincomGndUv;
        if (ain8GndUv >= INT32_MIN && ain8GndUv <= INT32_MAX) {
            diag.aincomGndUv = aincomGndUv;
            diag.ain8GndUv = (int32_t)ain8GndUv;
            int64_t batteryUv = ain8GndUv *
                                (int64_t)CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM /
                                (int64_t)CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_DEN;
            if (batteryUv >= 0LL && batteryUv <= (int64_t)INT32_MAX * 1000LL) {
                diag.batteryMv = (int32_t)(batteryUv / 1000LL);
            }
        }
    }

    diag.reason = "ok";
    return sensorarrayAdsFinishBatteryTransaction(ESP_OK, &diag, outDiag);
}

static esp_err_t sensorarrayAdsRunJob(sensorarrayState_t *state,
                                      sensorarrayAdsJob_t job,
                                      uint32_t frameSequence)
{
    int32_t raw = 0;
    int32_t uv = 0;
    esp_err_t err = ESP_OK;
    if (job == SENSORARRAY_ADS_JOB_RAIL) {
        sensorarrayAdsRailDiag_t railDiag = {0};
        err = sensorarrayAdsReadRailMonitorUv(state, &raw, &uv, &railDiag);
        if (err == ESP_OK) {
            (void)sensorarrayAdsUpdateRail(raw, uv, frameSequence);
            s_snapshot.vrefSynced = railDiag.restored;
        }
        if (err == ESP_OK) {
            s_lastRailFrame = frameSequence;
        }
        /* An unsafe clamped-reference request is terminal, not a reason to
         * retry every frame and repeatedly attempt the same hardware action. */
        s_forceRail = false;
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
        sensorarrayAdsBatteryDiag_t diag = {0};
        err = sensorarrayAdsReadBatteryTransaction(state, &raw, &uv, &diag);
        if (err == ESP_OK) {
            s_snapshot.ain8Raw = raw;
            s_snapshot.ain8RawUv = uv;
            s_snapshot.ain8DiffUv = uv;
            s_lastBatteryFrame = frameSequence;
        }
        s_forceBattery = false;
        sensorarrayAdsUpdateBatteryValidity(err);
        diag.stateName = sensorarrayAdsBatteryReasonName(s_snapshot.batteryInvalidReason);
        sensorarrayAdsPrintBatteryDiag(&diag);
    }
    return err;
}

static bool sensorarrayAdsJobIsExplicitRequest(sensorarrayAdsJob_t job)
{
    switch (job) {
    case SENSORARRAY_ADS_JOB_RAIL:
        return s_forceRail;
    case SENSORARRAY_ADS_JOB_ZERO:
        return s_forceZero;
    case SENSORARRAY_ADS_JOB_BATTERY:
        return s_forceBattery;
    case SENSORARRAY_ADS_JOB_NONE:
    default:
        return false;
    }
}

void sensorarrayAdsGapRequestCalibration(bool requestZero, bool requestRail)
{
    /* This function is called only by the Acquisition domain at a frame
     * boundary. The gap scheduler consumes the flags later on the same core,
     * so no cross-core lock is required here. A previous rail/restore failure
     * may have asked the diagnostic path to fall back to a frame boundary, but
     * it must not permanently disable future explicit calibration requests. */
    if (requestZero || requestRail) {
        s_snapshot.fallbackToBoundary = false;
    }
    s_forceZero = s_forceZero || requestZero;
    s_forceRail = s_forceRail || requestRail;
}

void sensorarrayAdsGapRequestBatteryDiagnostic(void)
{
    s_snapshot.fallbackToBoundary = false;
    s_forceBattery = true;
}

void sensorarrayAdsGapSetMode(sensorarrayAdsGapMode_t mode)
{
    if (mode > SENSORARRAY_ADS_GAP_MODE_ZERO) {
        return;
    }
    s_gapMode = mode;
    if (mode != SENSORARRAY_ADS_GAP_MODE_OFF) {
        s_snapshot.fallbackToBoundary = false;
    }
}

sensorarrayAdsGapMode_t sensorarrayAdsGapGetMode(void)
{
    return s_gapMode;
}

const char *sensorarrayAdsGapModeName(sensorarrayAdsGapMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_ADS_GAP_MODE_OFF:
        return "off";
    case SENSORARRAY_ADS_GAP_MODE_RAIL:
        return "rail";
    case SENSORARRAY_ADS_GAP_MODE_BAT:
        return "bat";
    case SENSORARRAY_ADS_GAP_MODE_ZERO:
        return "zero";
    case SENSORARRAY_ADS_GAP_MODE_ON:
    default:
        return "on";
    }
}

const char *sensorarrayAdsRailStatusName(sensorarrayAdsRailStatus_t status)
{
    switch (status) {
    case SENSORARRAY_ADS_RAIL_STATUS_OK:
        return "ok";
    case SENSORARRAY_ADS_RAIL_STATUS_HOLD:
        return "hold";
    case SENSORARRAY_ADS_RAIL_STATUS_BAD:
    default:
        return "bad";
    }
}

const char *sensorarrayAdsRailSourceName(sensorarrayAdsRailSource_t source)
{
    switch (source) {
    case SENSORARRAY_ADS_RAIL_SOURCE_MONITOR:
        return "monitor";
    case SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION:
        return "external";
    case SENSORARRAY_ADS_RAIL_SOURCE_NONE:
    default:
        return "none";
    }
}

void sensorarrayAdsGapTryRun(sensorarrayState_t *state,
                             uint64_t expectedFdcReadyUs,
                             uint32_t frameSequence,
                             uint8_t row)
{
    (void)row;
    s_snapshot.windows++;
    if (!CONFIG_SENSORARRAY_ADS_GAP_FILL_ENABLE || !state || !state->adsReady ||
        !s_snapshot.initialized) {
        s_snapshot.jobsSkip++;
        return;
    }

    sensorarrayAdsJob_t job = sensorarrayAdsSelectJob(frameSequence);
    if (job == SENSORARRAY_ADS_JOB_NONE) {
        return;
    }
    bool explicitRequest = sensorarrayAdsJobIsExplicitRequest(job);
    uint32_t remainingBeforeUs = sensorarrayAdsGapRemainingUs(expectedFdcReadyUs);
    uint32_t jobWorstCaseUs = s_snapshot.hasAdc2 ?
        (uint32_t)CONFIG_SENSORARRAY_ADS_JOB_WORST_CASE_US :
        SENSORARRAY_ADS_ADC1_JOB_WORST_CASE_US;
    uint32_t admissionUs = jobWorstCaseUs + s_snapshot.guardUs;
    if (remainingBeforeUs <= admissionUs) {
        /*
         * Periodic ADS work is opportunistic and must not steal the FDC ready
         * window. Explicit commands such as BATD are different: the host has
         * asked for a diagnostic transaction and needs a terminal BATD result,
         * even on an 8-row frame where no gap is large enough. Run that single
         * forced job at the frame-boundary wait point and expose the timing
         * cost through fallbackToBoundary/overrun counters instead of silently
         * keeping the request queued forever.
         */
        if (!explicitRequest) {
            s_snapshot.jobsSkip++;
            return;
        }
        s_snapshot.fallbackToBoundary = true;
    }

    s_lastJobFrame = frameSequence;
    int64_t jobStartUs = esp_timer_get_time();
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
    uint32_t runUs = 0u;
    int64_t jobEndUs = esp_timer_get_time();
    if (jobEndUs > jobStartUs) {
        uint64_t elapsed = (uint64_t)(jobEndUs - jobStartUs);
        runUs = elapsed > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsed;
    }
    if (slackUs < s_snapshot.guardUs || runUs > jobWorstCaseUs + s_snapshot.guardUs) {
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
    if (s_railState.lastGoodFrame != UINT32_MAX &&
        frameSequence >= s_railState.lastGoodFrame) {
        outSnapshot->railAgeFrames = frameSequence - s_railState.lastGoodFrame;
    }
}

size_t sensorarrayAdsGapFormatBattery(char *buffer,
                                      size_t bufferSize,
                                      uint32_t frameSequence)
{
    if (!buffer || bufferSize == 0u) {
        return 0u;
    }
    sensorarrayAdsGapSnapshot_t snapshot = {0};
    sensorarrayAdsGapCopySnapshot(&snapshot, frameSequence);
    int written = snprintf(buffer,
                           bufferSize,
                           "ABAT,bt=%ld,br=%s,a8d=%ld,z=%ld,ac=%ld,a8g=%ld,rail=%ld,rv=%u,rs=%s,age=%lu,fresh=%u,status=0x%02X,dg=%lu\n",
                           snapshot.batteryValid ? (long)snapshot.batteryMv : -1L,
                           sensorarrayAdsBatteryReasonName(snapshot.batteryInvalidReason),
                           (long)snapshot.ain8DiffUv,
                           (long)snapshot.zeroResidualUv,
                           snapshot.aincomGndValid ? (long)snapshot.aincomGndUv : -1L,
                           snapshot.ain8GndValid ? (long)snapshot.ain8GndUv : -1L,
                           (long)snapshot.railUv,
                           snapshot.railValid ? 1u : 0u,
                           sensorarrayAdsRailStatusName(snapshot.railStatus),
                           (unsigned long)snapshot.railAgeFrames,
                           snapshot.adcFresh ? 1u : 0u,
                           (unsigned)snapshot.adcStatus,
                           (unsigned long)snapshot.drdyGenerationDelta);
    return written > 0 ? (size_t)written : 0u;
}

size_t sensorarrayAdsGapFormatRail(char *buffer,
                                   size_t bufferSize,
                                   uint32_t frameSequence)
{
    if (!buffer || bufferSize == 0u) {
        return 0u;
    }
    sensorarrayAdsGapSnapshot_t snapshot = {0};
    sensorarrayAdsGapCopySnapshot(&snapshot, frameSequence);
    sensorarrayAdsRailSplit_t split = {0};
    bool splitValid = sensorarrayAdsGapCopyRailSplit(
        frameSequence,
        (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES,
        &split);
    int written = snprintf(buffer,
                           bufferSize,
                           "ARL,src=%s,raw=%ld,mon=%ld,rail=%ld,avdd=%ld,avss=%ld,exp=%ld,err=%ld,rv=%u,rs=%s,age=%lu,ref=%s,pwr=%s,mux=%s\n",
                           sensorarrayAdsRailSourceName(s_railCalibration.source),
                           (long)snapshot.railMonitorRaw,
                           (long)snapshot.railMonitorUv,
                           (long)snapshot.railRawUv,
                           splitValid ? (long)split.avddUv : 0L,
                           splitValid ? (long)split.avssUv : 0L,
                           (long)snapshot.railExpectedUv,
                           (long)snapshot.railErrorUv,
                           snapshot.railValid ? 1u : 0u,
                           sensorarrayAdsRailStatusName(snapshot.railStatus),
                           (unsigned long)snapshot.railAgeFrames,
                           snapshot.vrefSynced ? "restored" : "unsynced",
                           snapshot.vrefSynced ? "restored" : "unsynced",
                           snapshot.vrefSynced ? "restored" : "unsynced");
    return written > 0 ? (size_t)written : 0u;
}

size_t sensorarrayAdsGapFormatAds(char *buffer, size_t bufferSize)
{
    if (!buffer || bufferSize == 0u) {
        return 0u;
    }
    int written = snprintf(buffer,
                           bufferSize,
                           "ADS,chip=%u,adc=%u,ref=%s,pwr=vbias:%u,mode=dr%u,gap=%s\n",
                           s_snapshot.chip ? (unsigned)s_snapshot.chip : 1262u,
                           s_snapshot.activeAdc ? (unsigned)s_snapshot.activeAdc : 1u,
                           s_snapshot.vrefSynced ? "synced" : "unsynced",
                           s_snapshot.vbiasEnabled ? 1u : 0u,
                           (unsigned)s_snapshot.rateCode,
                           sensorarrayAdsGapModeName(s_gapMode));
    return written > 0 ? (size_t)written : 0u;
}
