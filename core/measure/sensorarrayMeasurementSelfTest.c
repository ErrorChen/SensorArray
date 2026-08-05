#include "sensorarrayMeasurementSelfTest.h"

#include <limits.h>

#include "sensorarrayAdsAutoRange.h"
#include "sensorarrayAdsMath.h"
#include "sensorarrayMeasurementMode.h"
#include "sensorarrayRoutePolicy.h"

#define SENSORARRAY_SELF_CHECK(result, condition)          \
    do {                                                    \
        (result)->checks++;                                 \
        if (!(condition)) {                                 \
            (result)->failureLine = (uint32_t)__LINE__;     \
            return false;                                   \
        }                                                   \
    } while (0)

bool sensorarrayMeasurementSelfTestRun(
    sensorarrayMeasurementSelfTestResult_t *outResult)
{
    if (!outResult) {
        return false;
    }
    *outResult = (sensorarrayMeasurementSelfTestResult_t){0};

    sensorarrayMeasurementModeContext_t modeContext;
    sensorarrayMeasurementModeInit(&modeContext);
    sensorarrayMeasurementModeSnapshot_t mode = {0};
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementModeCopySnapshot(&modeContext, &mode));
    SENSORARRAY_SELF_CHECK(outResult,
        mode.state == SENSORARRAY_MEASUREMENT_STATE_SAFE);
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementModeAccept(&modeContext,
                                          SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                          41u));
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementModeBeginTransition(&modeContext));
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementModeCompleteTransition(&modeContext, 9u, 27u));
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementModeCopySnapshot(&modeContext, &mode) &&
        mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE &&
        mode.generation == 1u && mode.appliedRequestId == 41u);
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementModeAccept(&modeContext,
                                          SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                          42u) &&
        sensorarrayMeasurementModeBeginTransition(&modeContext));
    sensorarrayMeasurementModeFailTransition(&modeContext, 0x44u, 31u);
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementModeCopySnapshot(&modeContext, &mode) &&
        mode.state == SENSORARRAY_MEASUREMENT_STATE_SAFE &&
        mode.activeMode == SENSORARRAY_MEASUREMENT_MODE_NONE &&
        mode.lastError == 0x44u);
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayMeasurementCellCount(1u) == 8u &&
        sensorarrayMeasurementCellCount(2u) == 16u &&
        sensorarrayMeasurementCellCount(4u) == 32u &&
        sensorarrayMeasurementCellCount(8u) == 64u &&
        sensorarrayMeasurementCellCount(0u) == 0u);

    sensorarrayRouteExpectedControl_t expectedRoute = {
        .logicalSource = 0,
        .swLevel = 1,
        .selaLevel = 0,
        .selbLevel = 0,
    };
    sensorarrayRouteObservedControl_t observedRoute = {
        .commandedSource = 0,
        .commandedSwLevel = 1,
        .observedSwLevel = 1,
        .commandedSelaLevel = 0,
        .observedSelaLevel = 0,
        .commandedSelbLevel = 0,
        .observedSelbLevel = 0,
    };
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayRouteControlReadbackMatches(&expectedRoute, &observedRoute));
    observedRoute.observedSwLevel = 0;
    SENSORARRAY_SELF_CHECK(outResult,
        !sensorarrayRouteControlReadbackMatches(&expectedRoute, &observedRoute));

    sensorarrayAdsRailInput_t railInput = {
        .measuredRailUv = 5200000,
        .nominalAvddToGroundUv = 3400000,
        .nominalAvssToGroundUv = 1800000,
        .maximumAgeFrames = 4u,
        .valid = true,
    };
    sensorarrayAdsRailSplit_t rail = {0};
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathSplitRail(&railInput, &rail) &&
        rail.avddUv == 3400000 && rail.avssUv == -1800000 &&
        rail.aincomUv == 800000);
    railInput.ageFrames = 5u;
    SENSORARRAY_SELF_CHECK(outResult,
        !sensorarrayAdsMathSplitRail(&railInput, &rail));
    railInput.ageFrames = 0u;
    railInput.valid = false;
    SENSORARRAY_SELF_CHECK(outResult,
        !sensorarrayAdsMathSplitRail(&railInput, &rail));
    railInput.valid = true;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathSplitRail(&railInput, &rail));
    sensorarrayAdsVoltageLimits_t voltageLimits = {
        .minimumUv = -1800000,
        .maximumUv = 3400000,
    };
    int32_t nodeUv = 0;
    sensorarrayCellError_t voltageError = SENSORARRAY_CELL_ERROR_NONE;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathVoltageFromDifferential(-700000, &rail,
                                                   &voltageLimits,
                                                   &nodeUv,
                                                   &voltageError) &&
        nodeUv == 100000);
    SENSORARRAY_SELF_CHECK(outResult,
        !sensorarrayAdsMathVoltageFromDifferential(INT32_MAX, &rail,
                                                    &voltageLimits,
                                                    &nodeUv,
                                                    &voltageError) &&
        (voltageError == SENSORARRAY_CELL_ERROR_OVERFLOW ||
         voltageError == SENSORARRAY_CELL_ERROR_RANGE));

    sensorarrayAdsResistanceConfig_t resistanceConfig = {
        .referenceResistorOhms = 10000u,
        .minimumOhms = 1u,
        .maximumOhms = 100000000u,
        .shortThresholdOhms = 10u,
        .openDenominatorUv = 1000u,
    };
    sensorarrayAdsResistanceResult_t resistance =
        sensorarrayAdsMathResistanceDivider(700000, -550000, -1800000,
                                             &resistanceConfig);
    SENSORARRAY_SELF_CHECK(outResult,
        resistance.valid && resistance.resistanceMilliohms == 10000000LL);
    resistance = sensorarrayAdsMathResistanceDivider(700000, -1799500,
                                                      -1800000,
                                                      &resistanceConfig);
    SENSORARRAY_SELF_CHECK(outResult,
        !resistance.valid && resistance.open &&
        resistance.error == SENSORARRAY_CELL_ERROR_OPEN);
    resistance = sensorarrayAdsMathResistanceDivider(700000, 699900,
                                                      -1800000,
                                                      &resistanceConfig);
    SENSORARRAY_SELF_CHECK(outResult,
        !resistance.valid && resistance.shorted &&
        resistance.error == SENSORARRAY_CELL_ERROR_SHORT);
    resistance = sensorarrayAdsMathResistanceDivider(700000, 800000,
                                                      -1800000,
                                                      &resistanceConfig);
    SENSORARRAY_SELF_CHECK(outResult,
        !resistance.valid && resistance.error == SENSORARRAY_CELL_ERROR_NEGATIVE);
    resistanceConfig.maximumOhms = 1000u;
    resistance = sensorarrayAdsMathResistanceDivider(700000, -550000,
                                                      -1800000,
                                                      &resistanceConfig);
    SENSORARRAY_SELF_CHECK(outResult,
        !resistance.valid && resistance.error == SENSORARRAY_CELL_ERROR_RANGE);
    resistanceConfig.referenceResistorOhms = 0u;
    resistance = sensorarrayAdsMathResistanceDivider(700000, -550000,
                                                      -1800000,
                                                      &resistanceConfig);
    SENSORARRAY_SELF_CHECK(outResult,
        !resistance.valid &&
        resistance.error == SENSORARRAY_CELL_ERROR_REFERENCE_INVALID);
    resistanceConfig.referenceResistorOhms = UINT32_MAX;
    resistanceConfig.maximumOhms = UINT32_MAX;
    resistanceConfig.openDenominatorUv = 0u;
    resistance = sensorarrayAdsMathResistanceDivider(INT32_MAX,
                                                      INT32_MIN + 1,
                                                      INT32_MIN,
                                                      &resistanceConfig);
    SENSORARRAY_SELF_CHECK(outResult,
        !resistance.valid && resistance.error == SENSORARRAY_CELL_ERROR_OVERFLOW);

    int32_t stableValues[] = {100, 103, 101, 102, 99};
    int32_t median = 0;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathSamplesStable(stableValues, 5u, 4u, &median) &&
        median == 101);
    int32_t capacitiveTransient[] = {100, 1000, -200, 700, 20};
    SENSORARRAY_SELF_CHECK(outResult,
        !sensorarrayAdsMathSamplesStable(capacitiveTransient, 5u, 50u,
                                         &median));
    sensorarrayAdsSampleHealth_t health = {
        .transportOk = true,
        .generationAdvanced = true,
        .statusNewData = true,
        .commonModeSafe = true,
    };
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_NONE);
    health.drdyTimedOut = true;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT);
    health.drdyTimedOut = false;
    health.generationAdvanced = false;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_STALE);
    health.generationAdvanced = true;
    health.resetOccurred = true;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_READBACK);
    health.resetOccurred = false;
    health.referenceAlarm = true;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_REFERENCE_ALARM);
    health.referenceAlarm = false;
    health.pgaHighAlarm = true;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE);
    health.pgaHighAlarm = false;
    health.pgaDifferentialAlarm = true;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_PGA_DIFFERENTIAL);
    health.pgaDifferentialAlarm = false;
    health.transportOk = false;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_SPI);
    health.transportOk = true;
    health.statusNewData = false;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_STALE);
    health.statusNewData = true;
    health.fullScaleSaturated = true;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_SATURATED);
    health.fullScaleSaturated = false;
    health.commonModeSafe = false;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsMathClassifySampleHealth(&health) ==
            SENSORARRAY_CELL_ERROR_COMMON_MODE);

    sensorarrayAdsAutoRangeConfig_t rangeConfig = {
        .increaseBelowPermille = 250u,
        .decreaseAbovePermille = 850u,
        .saturationPermille = 980u,
        .maximumAttempts = 6u,
        .pgaRailMarginUv = 300000,
    };
    sensorarrayAdsAutoRangeInput_t rangeInput = {
        .rawCode = INT32_MAX / 10,
        .positiveInputUv = 100000,
        .negativeInputUv = 0,
        .avddUv = 3400000,
        .avssUv = -1800000,
        .currentGain = 1u,
        .allowIncrease = true,
    };
    sensorarrayAdsAutoRangeDecision_t range = sensorarrayAdsAutoRangeDecide(
        &rangeConfig, &rangeInput);
    SENSORARRAY_SELF_CHECK(outResult,
        range.action == SENSORARRAY_ADS_AUTORANGE_INCREASE &&
        range.nextGain == 2u);
    rangeInput.currentGain = 8u;
    rangeInput.rawCode = (int32_t)((int64_t)INT32_MAX * 900LL / 1000LL);
    range = sensorarrayAdsAutoRangeDecide(&rangeConfig, &rangeInput);
    SENSORARRAY_SELF_CHECK(outResult,
        range.action == SENSORARRAY_ADS_AUTORANGE_DECREASE &&
        range.nextGain == 4u);
    rangeInput.currentGain = 2u;
    rangeInput.rawCode = INT32_MAX / 2;
    range = sensorarrayAdsAutoRangeDecide(&rangeConfig, &rangeInput);
    SENSORARRAY_SELF_CHECK(outResult,
        range.action == SENSORARRAY_ADS_AUTORANGE_KEEP);
    rangeInput.currentGain = 1u;
    rangeInput.positiveInputUv = 3350000;
    range = sensorarrayAdsAutoRangeDecide(&rangeConfig, &rangeInput);
    SENSORARRAY_SELF_CHECK(outResult,
        range.action == SENSORARRAY_ADS_AUTORANGE_FAIL &&
        range.error == SENSORARRAY_CELL_ERROR_COMMON_MODE);
    rangeInput = (sensorarrayAdsAutoRangeInput_t){
        .rawCode = INT32_MAX / 2,
        .positiveInputUv = 100000,
        .negativeInputUv = 0,
        .avddUv = 3400000,
        .avssUv = -1800000,
        .currentGain = 2u,
        .attempt = rangeConfig.maximumAttempts,
        .allowIncrease = true,
    };
    range = sensorarrayAdsAutoRangeDecide(&rangeConfig, &rangeInput);
    SENSORARRAY_SELF_CHECK(outResult,
        range.action == SENSORARRAY_ADS_AUTORANGE_FAIL &&
        range.error == SENSORARRAY_CELL_ERROR_AUTORANGE);
    rangeInput.attempt = 0u;
    rangeInput.pgaDifferentialAlarm = true;
    range = sensorarrayAdsAutoRangeDecide(&rangeConfig, &rangeInput);
    SENSORARRAY_SELF_CHECK(outResult,
        range.action == SENSORARRAY_ADS_AUTORANGE_DECREASE &&
        range.nextGain == 1u);
    rangeInput.currentGain = 1u;
    rangeInput.pgaDifferentialAlarm = false;
    rangeInput.pgaLowAlarm = true;
    range = sensorarrayAdsAutoRangeDecide(&rangeConfig, &rangeInput);
    SENSORARRAY_SELF_CHECK(outResult,
        range.action == SENSORARRAY_ADS_AUTORANGE_FAIL &&
        range.error == SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE);

    sensorarrayAdsGainCache_t gainCache;
    sensorarrayAdsGainCacheInit(&gainCache);
    sensorarrayAdsGainCacheStore(&gainCache,
                                 SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                 3u,
                                 8u);
    uint8_t gain = 0u;
    SENSORARRAY_SELF_CHECK(outResult,
        sensorarrayAdsGainCacheGet(&gainCache,
                                   SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                   3u,
                                   &gain) && gain == 8u);
    sensorarrayAdsGainCacheInvalidate(&gainCache);
    SENSORARRAY_SELF_CHECK(outResult,
        !sensorarrayAdsGainCacheGet(&gainCache,
                                    SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                    3u,
                                    &gain));
    return true;
}
