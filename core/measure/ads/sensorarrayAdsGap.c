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
#include "sensorarrayBatteryScheduler.h"
#include "tmuxSwitch.h"

#define printf sensorarrayAcqEventPrintf

#define SENSORARRAY_ADS_MUX_ANALOG_MONITOR 0x0Cu
#define SENSORARRAY_ADS_INTERNAL_VREF_UV 2500000u
#define SENSORARRAY_ADS_RAIL_SCALE 4
#define SENSORARRAY_ADS_RAIL_MIN_UV 3500000
#define SENSORARRAY_ADS_RAIL_MAX_UV 6000000
#define SENSORARRAY_ADS_ADC1_JOB_WORST_CASE_US 2500u
#define SENSORARRAY_ADS_DIAGNOSTIC_SETTLE_US 1000u
#define SENSORARRAY_ADS_CONVERSION_MARGIN_US 100u
#define SENSORARRAY_ADS_RAIL_REFERENCE_SETTLE_US 5000u
#define SENSORARRAY_ADS_RAIL_MONITOR_SETTLE_MS 1u
#define SENSORARRAY_ADS_RAIL_MONITOR_DISCARD_COUNT 4u
#define SENSORARRAY_ADS_ADC1_GAP_DISCARD_COUNT 1u
#define SENSORARRAY_ADS_BATTERY_DISCARD_COUNT 1u
#define SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT 3u
#define SENSORARRAY_ADS_BATTERY_FRESH_RETRY_COUNT 1u
#define SENSORARRAY_ADS_BATTERY_FIXED_OVERHEAD_BUDGET_US 1500u
#define SENSORARRAY_ADS_BATTERY_SATURATION_MARGIN 4096
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
    uint8_t id;
    uint8_t power;
    uint8_t interface;
    uint8_t mode0;
    uint8_t mode1;
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
    bool adc1WasRunning;
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
    uint32_t spreadRaw;
    uint8_t retryCount;
    uint32_t dataRateSps;
    uint8_t sampleCount;
    uint32_t shadowGenerationBefore;
    uint32_t sampleUs;
    esp_err_t readErr;
    bool fresh;
    bool referenceValid;
    bool saturated;
    bool unstable;
    bool collision;
    bool restoreOk;
    bool diagnostic;
    bool boundaryFallback;
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
static uint32_t s_lastZeroFrame;
static uint32_t s_lastRailFrame;
static bool s_forceZero;
static bool s_forceRail;
static uint32_t s_zeroSampleCount;
static double s_zeroMeanUv;
static double s_zeroM2Uv;
static sensorarrayAdsRegisterCache_t *s_registerCache;
static sensorarrayBatteryScheduler_t s_batteryScheduler;
static sensorarrayAdsBatteryDiag_t s_lastBatteryDiag;
static bool s_batteryDiagnosticRequested;
static bool s_batteryRestoreFailureLatched;

static void sensorarrayAdsPopulateRestoredRegisterCache(
    sensorarrayAdsRegisterCache_t *cache,
    const ads126xAdc1RegisterSnapshot_t *snapshot,
    uint32_t vrefUv,
    bool adc1Running);

static bool sensorarrayAdsSnapshotFromRegisterCache(
    const sensorarrayAdsRegisterCache_t *cache,
    const ads126xAdcHandle_t *ads,
    ads126xAdc1RegisterSnapshot_t *outSnapshot)
{
    if (!cache || !ads || !outSnapshot || !cache->vrefValid ||
        cache->vrefUv <= 0 || !cache->adc1RunningValid) {
        return false;
    }
    for (uint8_t index = 0u; index < SENSORARRAY_ADS_REGISTER_COUNT; ++index) {
        if (!cache->registers[index].valid) {
            return false;
        }
    }
    *outSnapshot = (ads126xAdc1RegisterSnapshot_t){
        .id = ads->idRegRaw,
        .power = cache->registers[SENSORARRAY_ADS_REGISTER_POWER].value,
        .interface = cache->registers[SENSORARRAY_ADS_REGISTER_INTERFACE].value,
        .mode0 = cache->registers[SENSORARRAY_ADS_REGISTER_MODE0].value,
        .mode1 = cache->registers[SENSORARRAY_ADS_REGISTER_MODE1].value,
        .mode2 = cache->registers[SENSORARRAY_ADS_REGISTER_MODE2].value,
        .inpmux = cache->registers[SENSORARRAY_ADS_REGISTER_INPMUX].value,
        .offsetCal = {
            cache->registers[SENSORARRAY_ADS_REGISTER_OFCAL0].value,
            cache->registers[SENSORARRAY_ADS_REGISTER_OFCAL1].value,
            cache->registers[SENSORARRAY_ADS_REGISTER_OFCAL2].value,
        },
        .fullScaleCal = {
            cache->registers[SENSORARRAY_ADS_REGISTER_FSCAL0].value,
            cache->registers[SENSORARRAY_ADS_REGISTER_FSCAL1].value,
            cache->registers[SENSORARRAY_ADS_REGISTER_FSCAL2].value,
        },
        .refmux = cache->registers[SENSORARRAY_ADS_REGISTER_REFMUX].value,
    };
    return true;
}

static bool sensorarrayAdsCoreRegistersMatch(
    const ads126xAdc1RegisterSnapshot_t *expected,
    uint8_t power,
    uint8_t mode2,
    uint8_t inpmux,
    uint8_t refmux)
{
    return expected && power == expected->power && mode2 == expected->mode2 &&
           inpmux == expected->inpmux && refmux == expected->refmux;
}

static esp_err_t sensorarrayAdsRestoreBatteryCoreState(
    ads126xAdcHandle_t *ads,
    const ads126xAdc1RegisterSnapshot_t *saved,
    uint32_t savedVrefUv,
    bool adc1WasRunning,
    uint8_t transactionPower,
    uint8_t transactionMode2,
    uint8_t transactionInpmux,
    uint8_t transactionRefmux,
    uint8_t *outPowerAfter)
{
    if (!ads || !saved || savedVrefUv == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = ads126xAdcStopAdc1(ads);
    if (err == ESP_OK && transactionPower != saved->power) {
        err = ads126xAdcWritePowerRegister(ads, saved->power);
    }
    if (err == ESP_OK && transactionMode2 != saved->mode2) {
        err = ads126xAdcSetMode2Fast(ads, saved->mode2);
    }
    if (err == ESP_OK && transactionInpmux != saved->inpmux) {
        err = ads126xAdcSetInputMux(ads,
                                    (uint8_t)(saved->inpmux >> 4u),
                                    (uint8_t)(saved->inpmux & 0x0Fu));
    }
    if (err == ESP_OK && transactionRefmux != saved->refmux) {
        err = ads126xAdcSetRefMuxWithVref(ads, saved->refmux, savedVrefUv);
    }
    if (err != ESP_OK) {
        return err;
    }

    uint8_t power = 0u;
    uint8_t mode2 = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    err = ads126xAdcReadCoreRegisters(ads, &power, NULL, &mode2,
                                      &inpmux, &refmux);
    if (err == ESP_OK && !sensorarrayAdsCoreRegistersMatch(
            saved, power, mode2, inpmux, refmux)) {
        /* The production board ties CS low. One retry prevents a transient
         * DOUT bit upset from being mistaken for a failed restore, while a
         * persistent mismatch still forces SAFE/DEGRADED. */
        err = ads126xAdcReadCoreRegisters(ads, &power, NULL, &mode2,
                                          &inpmux, &refmux);
    }
    if (err != ESP_OK || !sensorarrayAdsCoreRegistersMatch(
            saved, power, mode2, inpmux, refmux)) {
        return err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
    }
    ads->enableInternalRef = (saved->power & ADS126X_POWER_INTREF) != 0u;
    ads->vrefMicrovolts = savedVrefUv;
    if (outPowerAfter) {
        *outPowerAfter = power;
    }
    return adc1WasRunning ? ads126xAdcStartAdc1(ads) : ESP_OK;
}

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
    s_snapshot.railSource = SENSORARRAY_ADS_RAIL_SOURCE_MONITOR;
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
    if (!ads) {
        return 0u;
    }
    if (ads->deviceType == ADS126X_DEVICE_ADS1262) {
        return 1262u;
    }
    return ads->deviceType == ADS126X_DEVICE_ADS1263 ? 1263u : 0u;
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
                                             uint32_t timeoutUs,
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
        timeoutUs,
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

static esp_err_t sensorarrayAdsReadBatteryFreshAdc1(
    ads126xAdcHandle_t *ads,
    uint32_t timeoutUs,
    sensorarrayAdsFreshSample_t *outSample,
    uint8_t *inOutRetryCount)
{
    if (!ads || !outSample || !inOutRetryCount) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ESP_FAIL;
    for (uint8_t attempt = 0u;
         attempt <= SENSORARRAY_ADS_BATTERY_FRESH_RETRY_COUNT;
         ++attempt) {
        *outSample = (sensorarrayAdsFreshSample_t){0};
        err = sensorarrayAdsReadFreshAdc1(ads, timeoutUs, outSample);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        /* A timeout or a cleared ADC1-new-data bit may be retried only by
         * waiting for a later DRDY generation. Transport/status failures are
         * not hidden, and a failed retry still invalidates the battery job. */
        bool retryable = err == ESP_ERR_TIMEOUT ||
            (err == ESP_ERR_INVALID_RESPONSE && !outSample->fresh);
        if (!retryable || attempt == SENSORARRAY_ADS_BATTERY_FRESH_RETRY_COUNT) {
            return err;
        }
        if (*inOutRetryCount < UINT8_MAX) {
            (*inOutRetryCount)++;
        }
        ads126xAdcClearDrdyNotifications(ads);
    }
    return err;
}

static const char *sensorarrayAdsBatteryReasonName(sensorarrayBatteryInvalidReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_BATTERY_INVALID_NONE:
        return "ok";
    case SENSORARRAY_BATTERY_INVALID_DISABLED:
        return "disabled";
    case SENSORARRAY_BATTERY_INVALID_NOT_DUE:
        return "not_due";
    case SENSORARRAY_BATTERY_INVALID_DEFERRED:
        return "deferred";
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
    case SENSORARRAY_BATTERY_INVALID_SPI_ERROR:
        return "spi_error";
    case SENSORARRAY_BATTERY_INVALID_RESTORE_FAILED:
        return "restore_failed";
    case SENSORARRAY_BATTERY_INVALID_VBIAS:
        return "vbias_invalid";
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
    case SENSORARRAY_BATTERY_INVALID_SATURATED:
        return "saturated";
    case SENSORARRAY_BATTERY_INVALID_OUT_OF_RANGE:
        return "out_of_range";
    case SENSORARRAY_BATTERY_INVALID_OVERFLOW:
        return "overflow";
    case SENSORARRAY_BATTERY_INVALID_UNKNOWN:
    default:
        return "unk";
    }
}

static sensorarrayBatteryInvalidReason_t sensorarrayAdsBatteryMathReason(
    sensorarrayAdsBatteryMathError_t error)
{
    switch (error) {
    case SENSORARRAY_ADS_BATTERY_MATH_OK:
        return SENSORARRAY_BATTERY_INVALID_NONE;
    case SENSORARRAY_ADS_BATTERY_MATH_RESTORE_FAILED:
        return SENSORARRAY_BATTERY_INVALID_RESTORE_FAILED;
    case SENSORARRAY_ADS_BATTERY_MATH_ADC_TIMEOUT:
        return SENSORARRAY_BATTERY_INVALID_ADC_TIMEOUT;
    case SENSORARRAY_ADS_BATTERY_MATH_ADC_STALE:
        return SENSORARRAY_BATTERY_INVALID_ADC_STALE;
    case SENSORARRAY_ADS_BATTERY_MATH_ADC_STATUS:
        return SENSORARRAY_BATTERY_INVALID_ADC_STATUS_ERROR;
    case SENSORARRAY_ADS_BATTERY_MATH_SPI:
        return SENSORARRAY_BATTERY_INVALID_SPI_ERROR;
    case SENSORARRAY_ADS_BATTERY_MATH_SATURATED:
        return SENSORARRAY_BATTERY_INVALID_SATURATED;
    case SENSORARRAY_ADS_BATTERY_MATH_UNSTABLE:
        return SENSORARRAY_BATTERY_INVALID_UNSTABLE;
    case SENSORARRAY_ADS_BATTERY_MATH_VBIAS:
        return SENSORARRAY_BATTERY_INVALID_VBIAS;
    case SENSORARRAY_ADS_BATTERY_MATH_RAIL:
        return SENSORARRAY_BATTERY_INVALID_RAIL;
    case SENSORARRAY_ADS_BATTERY_MATH_REFERENCE:
        return SENSORARRAY_BATTERY_INVALID_REFERENCE_INVALID;
    case SENSORARRAY_ADS_BATTERY_MATH_DIVIDER:
        return SENSORARRAY_BATTERY_INVALID_DIV;
    case SENSORARRAY_ADS_BATTERY_MATH_NEGATIVE:
        return SENSORARRAY_BATTERY_INVALID_RANGE_ERROR;
    case SENSORARRAY_ADS_BATTERY_MATH_BELOW_MINIMUM:
        return SENSORARRAY_BATTERY_INVALID_ABSENT_OR_OPEN;
    case SENSORARRAY_ADS_BATTERY_MATH_ABOVE_MAXIMUM:
        return SENSORARRAY_BATTERY_INVALID_RANGE_ERROR;
    case SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW:
    default:
        return SENSORARRAY_BATTERY_INVALID_OVERFLOW;
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

static void sensorarrayAdsUpdateBatteryValidity(
    esp_err_t adcReadErr,
    const sensorarrayAdsBatteryDiag_t *diag)
{
    s_snapshot.batteryValid = false;
    s_snapshot.aincomGndValid = false;
    s_snapshot.ain8GndValid = false;
    s_snapshot.batteryMv = -1;
    if (!s_snapshot.bootCalibrationDone) {
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_CAL;
        return;
    }
    bool aincomValid = sensorarrayAdsCalculateAincomGnd(&s_snapshot.aincomGndUv);
    sensorarrayAdsBatteryMathInput_t input = {
        .ain8DifferentialUv = s_snapshot.ain8DiffUv,
        .aincomGroundUv = s_snapshot.aincomGndUv,
        .dividerNumerator = CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM,
        .dividerDenominator = CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_DEN,
        .calibrationScalePpm = CONFIG_SENSORARRAY_BATTERY_CALIBRATION_SCALE_PPM,
        .calibrationOffsetUv = CONFIG_SENSORARRAY_BATTERY_CALIBRATION_OFFSET_UV,
        .minimumMv = CONFIG_SENSORARRAY_ADS_BATTERY_MIN_MV,
        .maximumMv = CONFIG_SENSORARRAY_ADS_BATTERY_MAX_MV,
        .restoreOk = s_snapshot.batteryRestoreOk,
        .adcTransportOk = adcReadErr == ESP_OK ||
            adcReadErr == ESP_ERR_TIMEOUT || adcReadErr == ESP_ERR_INVALID_RESPONSE,
        .adcTimedOut = adcReadErr == ESP_ERR_TIMEOUT,
        .adcFresh = diag ? diag->fresh : s_snapshot.adcFresh,
        .adcStatusOk = adcReadErr != ESP_ERR_INVALID_RESPONSE,
        .adcSaturated = diag && diag->saturated,
        .adcUnstable = diag && diag->unstable,
        .vbiasConfirmed = s_snapshot.vbiasEnabled,
        .railValid = s_snapshot.railUsableForBattery && s_snapshot.railValid,
        .railFresh = s_snapshot.railStatus == SENSORARRAY_ADS_RAIL_STATUS_OK,
        .referenceValid = aincomValid && diag && diag->referenceValid,
    };
    sensorarrayAdsBatteryMathResult_t result =
        sensorarrayAdsMathBatteryVoltage(&input);
    s_snapshot.aincomGndValid = aincomValid;
    s_snapshot.ain8GndUv = result.ain8GroundUv;
    s_snapshot.ain8GndValid = result.ain8GroundValid;
    s_snapshot.batteryMv = result.batteryMv;
    s_snapshot.batteryValid = result.valid;
    s_snapshot.batteryInvalidReason = sensorarrayAdsBatteryMathReason(result.error);
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
    ads126xAdc1RegisterSnapshot_t snapshot = {0};
    esp_err_t err = ads126xAdcReadAdc1RegisterSnapshot(ads, &snapshot);
    if (err == ESP_OK) {
        saved->id = snapshot.id;
        saved->power = snapshot.power;
        saved->interface = snapshot.interface;
        saved->mode0 = snapshot.mode0;
        saved->mode1 = snapshot.mode1;
        saved->mode2 = snapshot.mode2;
        saved->inpmux = snapshot.inpmux;
        saved->refmux = snapshot.refmux;
        memcpy(saved->offsetCal, snapshot.offsetCal, sizeof(saved->offsetCal));
        memcpy(saved->fullScaleCal, snapshot.fullScaleCal,
               sizeof(saved->fullScaleCal));
        saved->vrefMicrovolts = ads->vrefMicrovolts;
        saved->pgaGain = ads->pgaGain;
        saved->dataRateDr = ads->dataRateDr;
        saved->crcMode = ads->crcMode;
        saved->enableInternalRef = ads->enableInternalRef;
        saved->enableStatusByte = ads->enableStatusByte;
        saved->adc1WasRunning = ads126xAdcIsAdc1Running(ads);
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
    ads126xAdc1RegisterSnapshot_t snapshot = {
        .id = saved->id,
        .power = saved->power,
        .interface = saved->interface,
        .mode0 = saved->mode0,
        .mode1 = saved->mode1,
        .mode2 = saved->mode2,
        .inpmux = saved->inpmux,
        .offsetCal = {saved->offsetCal[0], saved->offsetCal[1],
                      saved->offsetCal[2]},
        .fullScaleCal = {saved->fullScaleCal[0], saved->fullScaleCal[1],
                         saved->fullScaleCal[2]},
        .refmux = saved->refmux,
    };
    esp_err_t firstErr = ads126xAdcRestoreAdc1RegisterSnapshot(
        ads,
        &snapshot,
        saved->vrefMicrovolts,
        saved->adc1WasRunning);
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
            err = sensorarrayAdsReadFreshAdc1(
                ads, (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US, &sample);
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
    s_snapshot.batteryEnabled = CONFIG_SENSORARRAY_BATTERY_ENABLE != 0;
    s_snapshot.batteryPeriodMs = (uint32_t)CONFIG_SENSORARRAY_BATTERY_PERIOD_MS;
    s_snapshot.batteryAgeMs = UINT32_MAX;
    s_snapshot.batteryRestoreOk = true;
    s_snapshot.batteryInvalidReason = s_snapshot.batteryEnabled ?
        SENSORARRAY_BATTERY_INVALID_NOT_DUE :
        SENSORARRAY_BATTERY_INVALID_DISABLED;
    s_railState.lastGoodFrame = UINT32_MAX;
    s_slackSamples = 0u;
    s_slackTotalUs = 0u;
    s_lastSampleFrame = 0u;
    s_lastJobFrame = UINT32_MAX;
    s_lastZeroFrame = 0u;
    s_lastRailFrame = 0u;
    s_zeroSampleCount = 0u;
    s_zeroMeanUv = 0.0;
    s_zeroM2Uv = 0.0;
    s_forceZero = false;
    s_forceRail = false;
    memset(&s_lastBatteryDiag, 0, sizeof(s_lastBatteryDiag));
    s_batteryDiagnosticRequested = false;
    s_batteryRestoreFailureLatched = false;
    sensorarrayBatterySchedulerInit(
        &s_batteryScheduler,
        s_snapshot.batteryEnabled,
        (uint32_t)CONFIG_SENSORARRAY_BATTERY_PERIOD_MS,
        (uint32_t)CONFIG_SENSORARRAY_BATTERY_MAX_DEFER_MS,
        (uint64_t)esp_timer_get_time());

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
        sensorarrayAdsUpdateBatteryValidity(err, NULL);
        return err;
    }

    state->adsRefMux = ADS126X_REFMUX_AVDD_AVSS;
    state->adsRefMuxValid = true;
    s_snapshot.initialized = true;
    s_snapshot.dmaCapable = state->ads.spiDmaCapable;
    return ESP_OK;
}

void sensorarrayAdsGapBindRegisterCache(sensorarrayAdsRegisterCache_t *registerCache)
{
    s_registerCache = registerCache;
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
    s_snapshot.railSource = SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION;
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
    /* RAILCFG is an explicit board calibration, not a sampled monitor value.
     * Its AVDD/AVSS split remains the active configuration until the host
     * replaces it or a hardware/status fault invalidates the rail. Applying a
     * frame-age limit here made a perfectly valid external split expire after
     * 100 high-rate matrix frames and forced the route into SAFE. Measured and
     * last-good monitor rails still retain the strict age check. */
    bool externalCalibration = s_railCalibration.source ==
        SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION;
    if (!snapshot.railValid ||
        (!externalCalibration && snapshot.railAgeFrames > maximumAgeFrames)) {
        *outRail = (sensorarrayAdsRailSplit_t){0};
        return false;
    }
    if (externalCalibration) {
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
    if (sensorarrayBatterySchedulerIsDue(
            &s_batteryScheduler, (uint64_t)esp_timer_get_time())) {
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
        return SENSORARRAY_ADS_JOB_NONE;
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
            err = sensorarrayAdsReadFreshAdc1(
                &state->ads, (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
                &sample);
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
           "settleUs=%lu,dr=%lu,discardCount=%u,samples=%u,retry=%u,read=%s,state=%s,fresh=%u,status=0x%02X,dg=%lu,spreadRaw=%lu,"
           "raw=%ld,a8dUv=%ld,zeroUv=%ld,acUv=%ld,a8gUv=%ld,batteryMv=%ld,collision=%u,err=0x%lx,reason=%s\n",
           diag->powerBefore,
           diag->powerRequested,
           diag->powerDuring,
           diag->powerAfter,
           (diag->powerDuring & ADS126X_POWER_VBIAS) != 0u ? 1u : 0u,
           diag->inpmux,
           diag->refmux,
           (unsigned long)diag->settleUs,
           (unsigned long)diag->dataRateSps,
           (unsigned)diag->discardCount,
           (unsigned)diag->sampleCount,
           (unsigned)diag->retryCount,
           diag->readErr == ESP_OK ? "ok" : "fail",
           diag->stateName ? diag->stateName : "unk",
           diag->fresh ? 1u : 0u,
           (unsigned)diag->status,
           (unsigned long)diag->generationDelta,
           (unsigned long)diag->spreadRaw,
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
        .settleUs = (uint32_t)CONFIG_SENSORARRAY_BATTERY_VBIAS_SETTLE_US,
        .dataRateSps = ads126xAdcDataRateCodeToSps(
            (uint8_t)CONFIG_SENSORARRAY_BATTERY_DATA_RATE),
        .discardCount = SENSORARRAY_ADS_BATTERY_DISCARD_COUNT,
        .zeroUv = s_snapshot.zeroResidualUv,
        .batteryMv = -1,
        .reason = "begin",
    };
    int64_t transactionStartUs = esp_timer_get_time();
    ads126xAdc1RegisterSnapshot_t saved = {0};
    bool snapshotValid = false;
    bool snapshotFromShadow = false;
    bool transactionCoreKnown = false;
    bool adc1WasRunning = ads126xAdcIsAdc1Running(ads);
    bool adc2WasRunning = ads126xAdcIsAdc2Running(ads);
    uint32_t vrefBeforeUv = ads->vrefMicrovolts;
    uint8_t batteryMode2 = 0u;
    const uint8_t batteryInpmux = (uint8_t)(
        (SENSORARRAY_ADS_MUX_AIN8 << 4u) | SENSORARRAY_ADS_MUX_AINCOM);
    const uint8_t batteryRefmux = ADS126X_REFMUX_AVDD_AVSS;
    esp_err_t err = ESP_OK;

    if (!s_registerCache ||
        !sensorarrayAdsRegisterCacheAcquire(s_registerCache,
                                             SENSORARRAY_ADS_OWNER_BATTERY)) {
        diag.collision = true;
        diag.restoreOk = true;
        diag.reason = "owner_busy";
        err = ESP_ERR_INVALID_STATE;
        goto finish;
    }
    diag.shadowGenerationBefore = s_registerCache->generation;

    snapshotFromShadow = sensorarrayAdsSnapshotFromRegisterCache(
        s_registerCache, ads, &saved);
    if (snapshotFromShadow) {
        vrefBeforeUv = (uint32_t)s_registerCache->vrefUv;
    } else {
        err = ads126xAdcReadAdc1RegisterSnapshot(ads, &saved);
        if (err != ESP_OK) {
            diag.reason = "snapshot_read";
            sensorarrayAdsRegisterCacheInvalidate(s_registerCache);
            goto release;
        }
    }
    snapshotValid = true;
    diag.powerBefore = saved.power;

    if (ads126xAdcHasAdc2(ads)) {
        err = ads126xAdcStopAdc2(ads);
        if (err != ESP_OK) {
            diag.reason = "stop_adc2";
            goto restore;
        }
    }
    err = ads126xAdcStopAdc1(ads);
    state->adsAdc1Running = false;
    if (err != ESP_OK) {
        diag.reason = "stop_adc1";
        goto restore;
    }

    err = ads126xAdcSetVbiasEnabled(ads, true);
    if (err == ESP_OK) {
        err = ads126xAdcReadPowerRegister(ads, &diag.powerDuring);
    }
    diag.powerRequested = (uint8_t)(saved.power | ADS126X_POWER_VBIAS);
    s_snapshot.vbiasEnabled = err == ESP_OK &&
        (diag.powerDuring & ADS126X_POWER_VBIAS) != 0u;
    if (err != ESP_OK || !s_snapshot.vbiasEnabled) {
        diag.reason = err == ESP_OK ? "vbias_readback_off" : "vbias_on";
        if (err == ESP_OK) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
        goto restore;
    }
    sensorarrayAdsRegisterCacheNoteWrite(
        s_registerCache, SENSORARRAY_ADS_REGISTER_POWER,
        diag.powerDuring, true);

    uint32_t refUv = s_snapshot.railUv > 0 ?
        (uint32_t)s_snapshot.railUv : 0u;
    if (!s_snapshot.railUsableForBattery || refUv == 0u) {
        diag.reason = "rail_invalid";
        err = ESP_ERR_INVALID_STATE;
        goto restore;
    }
    err = ads126xAdcSetRefMuxWithVref(ads,
                                      batteryRefmux,
                                      refUv);
    if (err == ESP_OK &&
        !ads126xAdcBuildMode2(true,
                              1u,
                              (uint8_t)CONFIG_SENSORARRAY_BATTERY_DATA_RATE,
                              &batteryMode2)) {
        err = ESP_ERR_INVALID_ARG;
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetMode2Verified(ads, batteryMode2);
    }
    if (err == ESP_OK) {
        err = ads126xAdcSetInputMuxVerified(ads,
                                            SENSORARRAY_ADS_MUX_AIN8,
                                            SENSORARRAY_ADS_MUX_AINCOM);
    }
    if (err == ESP_OK) {
        err = ads126xAdcReadCoreRegisters(ads,
                                          NULL,
                                          NULL,
                                          NULL,
                                          &diag.inpmux,
                                          &diag.refmux);
    }
    if (err != ESP_OK || diag.inpmux !=
            (uint8_t)((SENSORARRAY_ADS_MUX_AIN8 << 4u) |
                      SENSORARRAY_ADS_MUX_AINCOM) ||
        diag.refmux != batteryRefmux) {
        diag.reason = "mux_or_ref_readback";
        if (err == ESP_OK) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
        goto restore;
    }
    diag.referenceValid = true;
    transactionCoreKnown = true;
    sensorarrayAdsRegisterCacheNoteWrite(
        s_registerCache, SENSORARRAY_ADS_REGISTER_REFMUX,
        batteryRefmux, true);
    sensorarrayAdsRegisterCacheNoteWrite(
        s_registerCache, SENSORARRAY_ADS_REGISTER_MODE2,
        batteryMode2, true);
    sensorarrayAdsRegisterCacheNoteWrite(
        s_registerCache, SENSORARRAY_ADS_REGISTER_INPMUX,
        batteryInpmux, true);
    s_registerCache->vrefValid = true;
    s_registerCache->vrefUv = (int32_t)refUv;
    s_registerCache->pgaModeValid = true;
    s_registerCache->inputMode = SENSORARRAY_ADS_INPUT_BYPASS;
    s_registerCache->gain = 1u;
    s_registerCache->adc1RunningValid = true;
    s_registerCache->adc1Running = false;

    if (diag.settleUs != 0u) {
        esp_rom_delay_us(diag.settleUs);
    }
    ads126xAdcClearDrdyNotifications(ads);
    err = ads126xAdcStartAdc1(ads);
    if (err != ESP_OK) {
        diag.reason = "start_adc1";
        goto restore;
    }
    state->adsAdc1Running = true;

    uint32_t batteryPeriodUs = ads126xAdcExpectedConversionPeriodUs(
        (uint8_t)CONFIG_SENSORARRAY_BATTERY_DATA_RATE);
    uint64_t batteryTimeout64 = (uint64_t)batteryPeriodUs * 3u + 500u;
    uint32_t batteryTimeoutUs = batteryTimeout64 > UINT32_MAX ?
        UINT32_MAX : (uint32_t)batteryTimeout64;
    if (batteryTimeoutUs < (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US) {
        batteryTimeoutUs = (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US;
    }

    sensorarrayAdsFreshSample_t discard = {0};
    for (uint8_t readIndex = 0u;
         readIndex < SENSORARRAY_ADS_BATTERY_DISCARD_COUNT;
         ++readIndex) {
        err = sensorarrayAdsReadBatteryFreshAdc1(
            ads, batteryTimeoutUs, &discard, &diag.retryCount);
        if (err != ESP_OK) {
            diag.reason = "discard_conversion";
            goto restore;
        }
    }

    sensorarrayAdsFreshSample_t samples[SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT] = {0};
    for (uint8_t sampleIndex = 0u;
         sampleIndex < SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT;
         ++sampleIndex) {
        err = sensorarrayAdsReadBatteryFreshAdc1(
            ads, batteryTimeoutUs, &samples[sampleIndex], &diag.retryCount);
        if (err != ESP_OK) {
            diag.reason = "fresh_conversion";
            goto restore;
        }
        diag.sampleCount++;
        diag.fresh = true;
        diag.status |= samples[sampleIndex].status;
        if ((samples[sampleIndex].status &
             (ADS126X_STATUS_REFERENCE_ALARM |
              ADS126X_STATUS_PGA_ALARM_MASK |
              ADS126X_STATUS_RESET_OCCURRED)) != 0u) {
            if ((samples[sampleIndex].status & ADS126X_STATUS_REFERENCE_ALARM) != 0u) {
                diag.referenceValid = false;
            }
            diag.reason = "status_alarm";
            err = ESP_ERR_INVALID_RESPONSE;
            goto restore;
        }
        if (samples[sampleIndex].raw >=
                INT32_MAX - SENSORARRAY_ADS_BATTERY_SATURATION_MARGIN ||
            samples[sampleIndex].raw <=
                INT32_MIN + SENSORARRAY_ADS_BATTERY_SATURATION_MARGIN) {
            diag.saturated = true;
            diag.reason = "saturated";
            err = ESP_ERR_INVALID_RESPONSE;
            goto restore;
        }
    }

    *outRaw = sensorarrayAdsMedian3I32(samples[0].raw,
                                       samples[1].raw,
                                       samples[2].raw);
    *outUv = ads126xAdcRawToMicrovolts(ads, *outRaw);
    int32_t minimumRaw = samples[0].raw;
    int32_t maximumRaw = samples[0].raw;
    for (uint8_t sampleIndex = 1u;
         sampleIndex < SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT;
         ++sampleIndex) {
        if (samples[sampleIndex].raw < minimumRaw) {
            minimumRaw = samples[sampleIndex].raw;
        }
        if (samples[sampleIndex].raw > maximumRaw) {
            maximumRaw = samples[sampleIndex].raw;
        }
    }
    uint64_t rawSpread = (uint64_t)((int64_t)maximumRaw - minimumRaw);
    diag.spreadRaw = rawSpread > UINT32_MAX ? UINT32_MAX : (uint32_t)rawSpread;
    diag.raw = *outRaw;
    diag.ain8DiffUv = *outUv;
    diag.fresh = true;
    diag.generationDelta =
        samples[SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT - 1u].generationEnd -
        samples[0].generationStart;
    if (rawSpread > (uint32_t)CONFIG_SENSORARRAY_BATTERY_MAX_SPREAD_RAW) {
        diag.unstable = true;
        diag.reason = "unstable";
        err = ESP_ERR_INVALID_RESPONSE;
        goto restore;
    }
    diag.reason = "ok";

restore:
    (void)ads126xAdcStopAdc1(ads);
    if (snapshotValid) {
        esp_err_t restoreErr = ESP_FAIL;
        if (snapshotFromShadow && transactionCoreKnown && err == ESP_OK) {
            restoreErr = sensorarrayAdsRestoreBatteryCoreState(
                ads,
                &saved,
                vrefBeforeUv,
                adc1WasRunning,
                diag.powerDuring,
                batteryMode2,
                batteryInpmux,
                batteryRefmux,
                &diag.powerAfter);
        } else {
            /* An invalid shadow or a failed/incomplete transaction takes the
             * heavier full-snapshot restore path. Reliability wins over gap
             * latency whenever the modified-register set is uncertain. */
            restoreErr = ads126xAdcRestoreAdc1RegisterSnapshot(
                ads, &saved, vrefBeforeUv, adc1WasRunning);
            if (restoreErr == ESP_OK) {
                (void)ads126xAdcReadPowerRegister(ads, &diag.powerAfter);
            }
        }
        if (restoreErr == ESP_OK && adc2WasRunning) {
            restoreErr = ads126xAdcStartAdc2(ads);
        }
        diag.restoreOk = restoreErr == ESP_OK;
        if (diag.restoreOk) {
            sensorarrayAdsPopulateRestoredRegisterCache(s_registerCache,
                                                         &saved,
                                                         vrefBeforeUv,
                                                         adc1WasRunning);
            state->adsRefMux = saved.refmux;
            state->adsRefMuxValid = true;
            state->adsAdc1Running = adc1WasRunning;
        } else {
            sensorarrayAdsRegisterCacheInvalidate(s_registerCache);
            state->adsRefMuxValid = false;
            state->adsAdc1Running = false;
            diag.reason = "restore_failed";
            err = restoreErr;
        }
    }

release:
    (void)sensorarrayAdsRegisterCacheRelease(s_registerCache,
                                              SENSORARRAY_ADS_OWNER_BATTERY);
finish:
    {
        int64_t transactionEndUs = esp_timer_get_time();
        uint64_t elapsedUs = transactionEndUs > transactionStartUs ?
            (uint64_t)(transactionEndUs - transactionStartUs) : 0u;
        diag.sampleUs = elapsedUs > UINT32_MAX ? UINT32_MAX : (uint32_t)elapsedUs;
    }
    diag.readErr = err;
    if (outDiag) {
        *outDiag = diag;
    }
    return err;
}

static esp_err_t sensorarrayAdsRunJob(sensorarrayState_t *state,
                                       sensorarrayAdsJob_t job,
                                       uint32_t frameSequence,
                                       bool boundaryFallback)
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
        diag.diagnostic = s_batteryDiagnosticRequested;
        diag.boundaryFallback = boundaryFallback;
        s_lastBatteryDiag = diag;
        if (diag.collision) {
            return err;
        }
        if (diag.fresh) {
            s_snapshot.ain8Raw = raw;
            s_snapshot.ain8RawUv = uv;
            s_snapshot.ain8DiffUv = uv;
        }
        s_snapshot.batteryTimestampUs = (uint64_t)esp_timer_get_time();
        s_snapshot.batterySampleUs = diag.sampleUs;
        s_snapshot.batterySampleCount = diag.sampleCount;
        s_snapshot.batteryShadowGeneration = diag.shadowGenerationBefore;
        s_snapshot.batterySpreadRaw = diag.spreadRaw;
        if (diag.spreadRaw > s_snapshot.batterySpreadRawMaximum) {
            s_snapshot.batterySpreadRawMaximum = diag.spreadRaw;
        }
        s_snapshot.batteryLastRetryCount = diag.retryCount;
        s_snapshot.batteryRetryCount += diag.retryCount;
        s_snapshot.batteryBoundaryFallback = boundaryFallback;
        s_snapshot.batteryRestoreOk = diag.restoreOk;
        s_snapshot.batteryFresh = diag.fresh;
        s_batteryRestoreFailureLatched = s_batteryRestoreFailureLatched ||
            !diag.restoreOk;
        sensorarrayAdsUpdateBatteryValidity(err, &diag);
        if (s_snapshot.batteryValid) {
            s_snapshot.batteryValidRunCount++;
        } else {
            s_snapshot.batteryInvalidRunCount++;
        }
        if (diag.unstable) {
            s_snapshot.batteryUnstableCount++;
        }
        if (err == ESP_ERR_TIMEOUT) {
            s_snapshot.batteryTimeoutCount++;
        }
        diag.aincomGndUv = s_snapshot.aincomGndUv;
        diag.ain8GndUv = s_snapshot.ain8GndUv;
        diag.batteryMv = s_snapshot.batteryMv;
        diag.stateName = sensorarrayAdsBatteryReasonName(s_snapshot.batteryInvalidReason);
        if (s_batteryDiagnosticRequested) {
            sensorarrayAdsPrintBatteryDiag(&diag);
        }
        s_batteryDiagnosticRequested = false;
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
        return s_batteryScheduler.forcePending;
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
    s_batteryDiagnosticRequested = true;
    sensorarrayBatterySchedulerRequestNow(
        &s_batteryScheduler, (uint64_t)esp_timer_get_time());
}

void sensorarrayAdsGapRequestBatteryNow(void)
{
    s_snapshot.fallbackToBoundary = false;
    sensorarrayBatterySchedulerRequestNow(
        &s_batteryScheduler, (uint64_t)esp_timer_get_time());
}

bool sensorarrayAdsGapConfigureBatteryPeriod(bool enabled, uint32_t periodMs)
{
    bool configured = sensorarrayBatterySchedulerConfigure(
        &s_batteryScheduler,
        enabled,
        periodMs,
        (uint64_t)esp_timer_get_time());
    if (!configured) {
        return false;
    }
    s_snapshot.batteryEnabled = enabled;
    if (enabled) {
        s_snapshot.batteryPeriodMs = periodMs;
        if (s_snapshot.batteryTimestampUs == 0u) {
            s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_NOT_DUE;
        }
    } else {
        s_snapshot.batteryValid = false;
        s_snapshot.batteryFresh = false;
        s_snapshot.batteryInvalidReason = SENSORARRAY_BATTERY_INVALID_DISABLED;
    }
    return true;
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
    if (job == SENSORARRAY_ADS_JOB_BATTERY) {
        uint32_t periodUs = ads126xAdcExpectedConversionPeriodUs(
            (uint8_t)CONFIG_SENSORARRAY_BATTERY_DATA_RATE);
        uint64_t estimatedUs =
            (uint64_t)CONFIG_SENSORARRAY_BATTERY_VBIAS_SETTLE_US +
            (uint64_t)(SENSORARRAY_ADS_BATTERY_DISCARD_COUNT +
                       SENSORARRAY_ADS_BATTERY_SAMPLE_COUNT) * periodUs +
            SENSORARRAY_ADS_BATTERY_FIXED_OVERHEAD_BUDGET_US;
        if (estimatedUs > UINT32_MAX) {
            estimatedUs = UINT32_MAX;
        }
        if ((uint32_t)estimatedUs > jobWorstCaseUs) {
            jobWorstCaseUs = (uint32_t)estimatedUs;
        }
        if (s_snapshot.batterySampleUs != 0u) {
            uint64_t observedBudgetUs =
                (uint64_t)s_snapshot.batterySampleUs + 250u;
            if (observedBudgetUs > UINT32_MAX) {
                observedBudgetUs = UINT32_MAX;
            }
            if ((uint32_t)observedBudgetUs > jobWorstCaseUs) {
                jobWorstCaseUs = (uint32_t)observedBudgetUs;
            }
        }
        sensorarrayBatteryDecision_t decision =
            sensorarrayBatterySchedulerEvaluateGap(
                &s_batteryScheduler,
                (uint64_t)esp_timer_get_time(),
                remainingBeforeUs,
                jobWorstCaseUs,
                s_snapshot.guardUs);
        if (decision != SENSORARRAY_BATTERY_DECISION_RUN_GAP) {
            if (s_snapshot.batteryTimestampUs == 0u) {
                s_snapshot.batteryInvalidReason =
                    decision == SENSORARRAY_BATTERY_DECISION_DISABLED ?
                        SENSORARRAY_BATTERY_INVALID_DISABLED :
                        SENSORARRAY_BATTERY_INVALID_DEFERRED;
            }
            return;
        }
    }
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
    esp_err_t err = sensorarrayAdsRunJob(state, job, frameSequence, false);
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
    if (job == SENSORARRAY_ADS_JOB_BATTERY && !s_lastBatteryDiag.collision) {
        sensorarrayBatterySchedulerRecordRun(
            &s_batteryScheduler,
            (uint64_t)jobEndUs,
            runUs,
            false,
            s_lastBatteryDiag.restoreOk);
    }
    if (slackUs < s_snapshot.guardUs || runUs > jobWorstCaseUs + s_snapshot.guardUs) {
        sensorarrayAdsGapRecordOverrun();
    }
}

esp_err_t sensorarrayAdsGapRunBatteryAtBoundary(sensorarrayState_t *state,
                                                uint32_t frameSequence,
                                                bool capacitanceMode,
                                                uint32_t *outDurationUs,
                                                bool *outRan)
{
    if (outDurationUs) {
        *outDurationUs = 0u;
    }
    if (outRan) {
        *outRan = false;
    }
    if (!state || !state->adsReady || !s_snapshot.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    sensorarrayBatteryDecision_t decision =
        sensorarrayBatterySchedulerEvaluateBoundary(
            &s_batteryScheduler, nowUs, capacitanceMode);
    s_snapshot.batteryDue = sensorarrayBatterySchedulerIsDue(
        &s_batteryScheduler, nowUs);
    if (decision != SENSORARRAY_BATTERY_DECISION_RUN_BOUNDARY) {
        if (s_snapshot.batteryTimestampUs == 0u) {
            s_snapshot.batteryInvalidReason =
                decision == SENSORARRAY_BATTERY_DECISION_DISABLED ?
                    SENSORARRAY_BATTERY_INVALID_DISABLED :
                (decision == SENSORARRAY_BATTERY_DECISION_DEFER ?
                    SENSORARRAY_BATTERY_INVALID_DEFERRED :
                    SENSORARRAY_BATTERY_INVALID_NOT_DUE);
        }
        return ESP_OK;
    }

    int64_t startUs = esp_timer_get_time();
    esp_err_t err = sensorarrayAdsRunJob(state,
                                          SENSORARRAY_ADS_JOB_BATTERY,
                                          frameSequence,
                                          true);
    int64_t completedUsSigned = esp_timer_get_time();
    uint64_t elapsedUs = completedUsSigned > startUs ?
        (uint64_t)(completedUsSigned - startUs) : 0u;
    uint32_t durationUs = elapsedUs > UINT32_MAX ? UINT32_MAX :
        (uint32_t)elapsedUs;
    if (outDurationUs) {
        *outDurationUs = durationUs;
    }
    if (outRan) {
        *outRan = !s_lastBatteryDiag.collision;
    }
    if (!s_lastBatteryDiag.collision) {
        sensorarrayBatterySchedulerRecordRun(
            &s_batteryScheduler,
            completedUsSigned > 0 ? (uint64_t)completedUsSigned : nowUs,
            durationUs,
            true,
            s_lastBatteryDiag.restoreOk);
    }
    return err;
}

bool sensorarrayAdsGapConsumeRestoreFailure(void)
{
    bool failed = s_batteryRestoreFailureLatched;
    s_batteryRestoreFailureLatched = false;
    return failed;
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
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    outSnapshot->batteryEnabled = s_batteryScheduler.enabled;
    outSnapshot->batteryPeriodMs = s_batteryScheduler.periodMs;
    outSnapshot->batteryAgeMs = sensorarrayBatterySchedulerAgeMs(
        &s_batteryScheduler, nowUs);
    outSnapshot->batteryFresh = outSnapshot->batteryFresh &&
        outSnapshot->batteryTimestampUs != 0u &&
        outSnapshot->batteryAgeMs <=
            (uint32_t)CONFIG_SENSORARRAY_BATTERY_MAX_AGE_MS;
    outSnapshot->batteryDue = sensorarrayBatterySchedulerIsDue(
        &s_batteryScheduler, nowUs);
    outSnapshot->batteryRunCount = s_batteryScheduler.runCount;
    outSnapshot->batterySkipCount = s_batteryScheduler.skipCount;
    outSnapshot->batteryDeferCount = s_batteryScheduler.deferCount;
    outSnapshot->batteryBoundaryCount = s_batteryScheduler.boundaryCount;
    outSnapshot->batteryRestoreFailureCount =
        s_batteryScheduler.restoreFailureCount;
    outSnapshot->batterySampleUsMaximum =
        s_batteryScheduler.sampleDurationMaximumUs;
    outSnapshot->batterySampleUsAverage = s_batteryScheduler.runCount != 0u ?
        (uint32_t)(s_batteryScheduler.sampleDurationTotalUs /
                   s_batteryScheduler.runCount) : 0u;
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
                           "ABAT,bt=%ld,valid=%u,fresh=%u,ageMs=%lu,periodMs=%lu,due=%u,run=%lu,validRun=%lu,invalidRun=%lu,skip=%lu,defer=%lu,boundary=%lu,restoreFail=%lu,retry=%lu/%lu,unstable=%lu,timeout=%lu,spreadRaw=%lu,spreadMaxRaw=%lu,raw=%ld,a8d=%ld,ac=%ld,a8g=%ld,ratio=%u/%u,rail=%ld,railState=%s,vbias=%u,samples=%lu,sampleUs=%lu,restore=%s,reason=%s\n",
                           snapshot.batteryValid ? (long)snapshot.batteryMv : -1L,
                           snapshot.batteryValid ? 1u : 0u,
                           snapshot.batteryFresh ? 1u : 0u,
                           (unsigned long)snapshot.batteryAgeMs,
                           (unsigned long)snapshot.batteryPeriodMs,
                           snapshot.batteryDue ? 1u : 0u,
                           (unsigned long)snapshot.batteryRunCount,
                           (unsigned long)snapshot.batteryValidRunCount,
                           (unsigned long)snapshot.batteryInvalidRunCount,
                           (unsigned long)snapshot.batterySkipCount,
                           (unsigned long)snapshot.batteryDeferCount,
                           (unsigned long)snapshot.batteryBoundaryCount,
                           (unsigned long)snapshot.batteryRestoreFailureCount,
                           (unsigned long)snapshot.batteryLastRetryCount,
                           (unsigned long)snapshot.batteryRetryCount,
                           (unsigned long)snapshot.batteryUnstableCount,
                           (unsigned long)snapshot.batteryTimeoutCount,
                           (unsigned long)snapshot.batterySpreadRaw,
                           (unsigned long)snapshot.batterySpreadRawMaximum,
                           (long)snapshot.ain8Raw,
                           (long)snapshot.ain8DiffUv,
                           snapshot.aincomGndValid ? (long)snapshot.aincomGndUv : -1L,
                           snapshot.ain8GndValid ? (long)snapshot.ain8GndUv : -1L,
                           (unsigned)CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_NUM,
                           (unsigned)CONFIG_SENSORARRAY_ADS_AIN8_BATTERY_DIVIDER_DEN,
                           (long)snapshot.railUv,
                           sensorarrayAdsRailStatusName(snapshot.railStatus),
                           snapshot.vbiasEnabled ? 1u : 0u,
                           (unsigned long)snapshot.batterySampleCount,
                           (unsigned long)snapshot.batterySampleUs,
                           snapshot.batteryRestoreOk ? "ok" : "fail",
                           sensorarrayAdsBatteryReasonName(snapshot.batteryInvalidReason));
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
    const char *chipName = s_snapshot.chip == 1262u ? "1262" :
        (s_snapshot.chip == 1263u ? "1263" : "unknown");
    bool valid = s_snapshot.initialized && s_snapshot.chip != 0u;
    int written = snprintf(buffer,
                           bufferSize,
                           "ADS,chip=%s,valid=%u,adc1=%u,adc2=%u,adc=%u,ref=%s,pwr=vbias:%u,mode=dr%u,gap=%s\n",
                           chipName,
                           valid ? 1u : 0u,
                           valid ? 1u : 0u,
                           valid && s_snapshot.hasAdc2 ? 1u : 0u,
                           valid ? (unsigned)s_snapshot.activeAdc : 0u,
                           s_snapshot.vrefSynced ? "synced" : "unsynced",
                           s_snapshot.vbiasEnabled ? 1u : 0u,
                           (unsigned)s_snapshot.rateCode,
                           sensorarrayAdsGapModeName(s_gapMode));
    return written > 0 ? (size_t)written : 0u;
}

static void sensorarrayAdsPopulateRestoredRegisterCache(
    sensorarrayAdsRegisterCache_t *cache,
    const ads126xAdc1RegisterSnapshot_t *snapshot,
    uint32_t vrefUv,
    bool adc1Running)
{
    if (!cache || !snapshot) {
        return;
    }
    sensorarrayAdsRegisterCacheInvalidate(cache);
    const uint8_t values[SENSORARRAY_ADS_REGISTER_COUNT] = {
        [SENSORARRAY_ADS_REGISTER_POWER] = snapshot->power,
        [SENSORARRAY_ADS_REGISTER_INTERFACE] = snapshot->interface,
        [SENSORARRAY_ADS_REGISTER_MODE0] = snapshot->mode0,
        [SENSORARRAY_ADS_REGISTER_MODE1] = snapshot->mode1,
        [SENSORARRAY_ADS_REGISTER_MODE2] = snapshot->mode2,
        [SENSORARRAY_ADS_REGISTER_INPMUX] = snapshot->inpmux,
        [SENSORARRAY_ADS_REGISTER_REFMUX] = snapshot->refmux,
        [SENSORARRAY_ADS_REGISTER_OFCAL0] = snapshot->offsetCal[0],
        [SENSORARRAY_ADS_REGISTER_OFCAL1] = snapshot->offsetCal[1],
        [SENSORARRAY_ADS_REGISTER_OFCAL2] = snapshot->offsetCal[2],
        [SENSORARRAY_ADS_REGISTER_FSCAL0] = snapshot->fullScaleCal[0],
        [SENSORARRAY_ADS_REGISTER_FSCAL1] = snapshot->fullScaleCal[1],
        [SENSORARRAY_ADS_REGISTER_FSCAL2] = snapshot->fullScaleCal[2],
    };
    for (uint8_t index = 0u; index < SENSORARRAY_ADS_REGISTER_COUNT; ++index) {
        (void)sensorarrayAdsRegisterCacheNoteReadback(
            cache, (sensorarrayAdsRegisterId_t)index, values[index]);
    }
    cache->vrefValid = vrefUv != 0u;
    cache->vrefUv = (int32_t)vrefUv;
    cache->pgaModeValid = true;
    cache->inputMode = ads126xAdcMode2PgaBypassed(snapshot->mode2) ?
        SENSORARRAY_ADS_INPUT_BYPASS : SENSORARRAY_ADS_INPUT_PGA;
    uint8_t gain = 1u;
    if (!ads126xAdcMode2PgaBypassed(snapshot->mode2)) {
        (void)ads126xAdcMode2DecodePgaGain(snapshot->mode2, &gain);
    }
    cache->gain = gain;
    cache->adc1RunningValid = true;
    cache->adc1Running = adc1Running;
}

esp_err_t sensorarrayAdsGapRunActiveCheck(
    sensorarrayState_t *state,
    sensorarrayAdsRegisterCache_t *registerCache,
    uint32_t requestId,
    uint32_t sampleCount,
    sensorarrayAdsActiveCheckResult_t *outResult)
{
    if (!state || !registerCache || !outResult || sampleCount < 1u ||
        sampleCount > 1000u) {
        return ESP_ERR_INVALID_ARG;
    }
    *outResult = (sensorarrayAdsActiveCheckResult_t){
        .requestId = requestId,
        .requestedSamples = sampleCount,
    };
    if (!sensorarrayAdsRegisterCacheAcquire(registerCache,
                                             SENSORARRAY_ADS_OWNER_CHECK)) {
        return ESP_ERR_INVALID_STATE;
    }

    ads126xAdcHandle_t *ads = &state->ads;
    bool adc1WasRunning = ads126xAdcIsAdc1Running(ads);
    bool adc2WasRunning = ads126xAdcIsAdc2Running(ads);
    uint32_t vrefBeforeUv = ads->vrefMicrovolts;
    int64_t checkStartUs = esp_timer_get_time();
    esp_err_t err = ads126xAdcReadAdc1RegisterSnapshot(ads,
                                                       &outResult->registers);
    if (err != ESP_OK) {
        outResult->spiErrors++;
        (void)sensorarrayAdsRegisterCacheRelease(registerCache,
                                                  SENSORARRAY_ADS_OWNER_CHECK);
        return err;
    }
    outResult->chip = sensorarrayAdsChipValue(ads);
    outResult->revision = ads126xAdcGetRevId(ads);
    outResult->adc1Available = outResult->chip == 1262u || outResult->chip == 1263u;
    outResult->adc2Available = outResult->chip == 1263u && ads126xAdcHasAdc2(ads);
    outResult->dataRateSps = ads126xAdcDataRateCodeToSps(
        outResult->registers.mode2 & 0x0Fu);
    outResult->vrefUv = vrefBeforeUv;
    outResult->pgaBypassed = ads126xAdcMode2PgaBypassed(
        outResult->registers.mode2);
    outResult->gain = 1u;
    if (!outResult->pgaBypassed &&
        !ads126xAdcMode2DecodePgaGain(outResult->registers.mode2,
                                      &outResult->gain)) {
        outResult->statusErrors++;
    }

    if (ads126xAdcHasAdc2(ads)) {
        err = ads126xAdcStopAdc2(ads);
    }
    if (err == ESP_OK) {
        err = ads126xAdcStopAdc1(ads);
    }
    if (err == ESP_OK) {
        ads126xAdcClearDrdyNotifications(ads);
        err = ads126xAdcStartAdc1(ads);
    }
    uint64_t periodTotalUs = 0u;
    uint32_t periodCount = 0u;
    uint32_t priorGeneration = ads126xAdcGetDrdyGeneration(ads);
    int64_t priorDrdyUs = 0;
    int32_t priorRaw = 0;
    bool priorRawValid = false;
    if (err != ESP_OK) {
        outResult->spiErrors++;
    }
    for (uint32_t index = 0u; err == ESP_OK && index < sampleCount; ++index) {
        uint32_t generation = priorGeneration;
        esp_err_t waitErr = ads126xAdcWaitDrdyGenerationUs(
            ads,
            priorGeneration,
            (uint32_t)CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US,
            &generation);
        int64_t drdyUs = esp_timer_get_time();
        if (waitErr != ESP_OK) {
            if (waitErr == ESP_ERR_TIMEOUT) {
                outResult->drdyTimeouts++;
            } else {
                outResult->spiErrors++;
            }
            continue;
        }
        if (generation == priorGeneration) {
            outResult->staleSamples++;
            continue;
        }
        priorGeneration = generation;
        if (priorDrdyUs != 0 && drdyUs > priorDrdyUs) {
            uint64_t periodUs64 = (uint64_t)(drdyUs - priorDrdyUs);
            uint32_t periodUs = periodUs64 > UINT32_MAX ?
                UINT32_MAX : (uint32_t)periodUs64;
            if (outResult->periodMinUs == 0u || periodUs < outResult->periodMinUs) {
                outResult->periodMinUs = periodUs;
            }
            if (periodUs > outResult->periodMaxUs) {
                outResult->periodMaxUs = periodUs;
            }
            periodTotalUs += periodUs;
            periodCount++;
        }
        priorDrdyUs = drdyUs;

        int32_t raw = 0;
        uint8_t status = 0u;
        esp_err_t readErr = ads126xAdcReadAdc1RawDmaReady(
            ads, &raw, &status, NULL);
        if (readErr != ESP_OK) {
            outResult->spiErrors++;
            continue;
        }
        if (!ads126xAdcStatusByteHasAdc1NewData(ads, status)) {
            outResult->staleSamples++;
            continue;
        }
        outResult->freshSamples++;
        if (priorRawValid && raw != priorRaw) {
            outResult->changedSamples++;
        }
        priorRaw = raw;
        priorRawValid = true;
        if ((status & (ADS126X_STATUS_REFERENCE_ALARM |
                       ADS126X_STATUS_PGA_ALARM_MASK)) != 0u) {
            outResult->statusErrors++;
        }
        if ((status & ADS126X_STATUS_RESET_OCCURRED) != 0u) {
            outResult->resetFlags++;
        }
    }
    outResult->periodAverageUs = periodCount ?
        (uint32_t)(periodTotalUs / periodCount) : 0u;
    (void)ads126xAdcStopAdc1(ads);
    esp_err_t restoreErr = ads126xAdcRestoreAdc1RegisterSnapshot(
        ads,
        &outResult->registers,
        vrefBeforeUv,
        adc1WasRunning);
    if (restoreErr == ESP_OK && adc2WasRunning) {
        restoreErr = ads126xAdcStartAdc2(ads);
    }
    outResult->restoreOk = restoreErr == ESP_OK;
    state->adsAdc1Running = outResult->restoreOk && adc1WasRunning;
    if (outResult->restoreOk) {
        sensorarrayAdsPopulateRestoredRegisterCache(registerCache,
                                                     &outResult->registers,
                                                     vrefBeforeUv,
                                                     adc1WasRunning);
    } else {
        sensorarrayAdsRegisterCacheInvalidate(registerCache);
    }
    (void)sensorarrayAdsRegisterCacheRelease(registerCache,
                                              SENSORARRAY_ADS_OWNER_CHECK);
    int64_t checkEndUs = esp_timer_get_time();
    outResult->durationUs = checkEndUs > checkStartUs ?
        (uint64_t)(checkEndUs - checkStartUs) : 0u;
    outResult->ok = outResult->chip != 0u && outResult->adc1Available &&
        outResult->freshSamples == sampleCount && outResult->spiErrors == 0u &&
        outResult->drdyTimeouts == 0u && outResult->staleSamples == 0u &&
        outResult->statusErrors == 0u && outResult->resetFlags == 0u &&
        outResult->restoreOk;
    return outResult->ok ? ESP_OK :
        (restoreErr != ESP_OK ? restoreErr : ESP_ERR_INVALID_RESPONSE);
}

static const char *sensorarrayAdsFilterName(uint8_t mode1)
{
    switch ((mode1 >> 5u) & 0x07u) {
    case 0u: return "sinc1";
    case 1u: return "sinc2";
    case 2u: return "sinc3";
    case 3u: return "sinc4";
    case 4u: return "fir";
    default: return "reserved";
    }
}

static uint32_t sensorarrayAdsDelayCodeUs(uint8_t mode0)
{
    static const uint32_t delaysUs[12] = {
        0u, 9u, 17u, 35u, 69u, 139u, 278u, 555u,
        1100u, 2200u, 4400u, 8800u,
    };
    uint8_t code = mode0 & 0x0Fu;
    return code < 12u ? delaysUs[code] : UINT32_MAX;
}

static const char *sensorarrayAdsReferenceName(uint8_t refmux)
{
    if (refmux == ADS126X_REFMUX_INTERNAL) {
        return "internal";
    }
    return refmux == ADS126X_REFMUX_AVDD_AVSS ? "avdd-avss" : "external";
}

size_t sensorarrayAdsGapFormatActiveCheck(
    const sensorarrayAdsActiveCheckResult_t *result,
    char *buffer,
    size_t bufferSize)
{
    if (!result || !buffer || bufferSize == 0u) {
        return 0u;
    }
    const char *chip = result->chip == 1262u ? "1262" :
        (result->chip == 1263u ? "1263" : "unknown");
    char pga[12];
    if (result->pgaBypassed) {
        snprintf(pga, sizeof(pga), "bypass");
    } else {
        snprintf(pga, sizeof(pga), "%u", (unsigned)result->gain);
    }
    int written = snprintf(
        buffer,
        bufferSize,
        "ADSCHK,id=%lu,ok=%u,chip=%s,idreg=0x%02X,rev=%u,adc1=%u,adc2=%u,power=0x%02X,interface=0x%02X,mode0=0x%02X,mode1=0x%02X,mode2=0x%02X,inpmux=0x%02X,refmux=0x%02X,ofcal=%02X%02X%02X,fscal=%02X%02X%02X,dr=%lu,filter=%s,chop=%u,delayUs=%lu,pga=%s,gain=%u,reference=%s,vbias=%u\n",
        (unsigned long)result->requestId,
        result->ok ? 1u : 0u,
        chip,
        (unsigned)result->registers.id,
        (unsigned)result->revision,
        result->adc1Available ? 1u : 0u,
        result->adc2Available ? 1u : 0u,
        (unsigned)result->registers.power,
        (unsigned)result->registers.interface,
        (unsigned)result->registers.mode0,
        (unsigned)result->registers.mode1,
        (unsigned)result->registers.mode2,
        (unsigned)result->registers.inpmux,
        (unsigned)result->registers.refmux,
        (unsigned)result->registers.offsetCal[2],
        (unsigned)result->registers.offsetCal[1],
        (unsigned)result->registers.offsetCal[0],
        (unsigned)result->registers.fullScaleCal[2],
        (unsigned)result->registers.fullScaleCal[1],
        (unsigned)result->registers.fullScaleCal[0],
        (unsigned long)result->dataRateSps,
        sensorarrayAdsFilterName(result->registers.mode1),
        (result->registers.mode0 & 0x30u) != 0u ? 1u : 0u,
        (unsigned long)sensorarrayAdsDelayCodeUs(result->registers.mode0),
        pga,
        (unsigned)result->gain,
        sensorarrayAdsReferenceName(result->registers.refmux),
        (result->registers.power & ADS126X_POWER_VBIAS) != 0u ? 1u : 0u);
    return written > 0 ? (size_t)written : 0u;
}

size_t sensorarrayAdsGapFormatActiveCheckTiming(
    const sensorarrayAdsActiveCheckResult_t *result,
    char *buffer,
    size_t bufferSize)
{
    if (!result || !buffer || bufferSize == 0u) {
        return 0u;
    }
    int written = snprintf(
        buffer,
        bufferSize,
        "ADSCHKSTAT,id=%lu,samples=%lu,fresh=%lu,changed=%lu,periodMinUs=%lu,periodAvgUs=%lu,periodMaxUs=%lu,spi=%lu,timeout=%lu,stale=%lu,statusErr=%lu,reset=%lu,restore=%s,durationUs=%llu\n",
        (unsigned long)result->requestId,
        (unsigned long)result->requestedSamples,
        (unsigned long)result->freshSamples,
        (unsigned long)result->changedSamples,
        (unsigned long)result->periodMinUs,
        (unsigned long)result->periodAverageUs,
        (unsigned long)result->periodMaxUs,
        (unsigned long)result->spiErrors,
        (unsigned long)result->drdyTimeouts,
        (unsigned long)result->staleSamples,
        (unsigned long)result->statusErrors,
        (unsigned long)result->resetFlags,
        result->restoreOk ? "ok" : "fail",
        (unsigned long long)result->durationUs);
    return written > 0 ? (size_t)written : 0u;
}
