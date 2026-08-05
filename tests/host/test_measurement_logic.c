#include <limits.h>
#include <stdint.h>
#include <stdio.h>

#include "sensorarrayAdsAutoRange.h"
#include "sensorarrayAdsMath.h"
#include "sensorarrayMeasurementMode.h"
#include "sensorarrayOutputPolicy.h"
#include "sensorarrayRoutePolicy.h"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL,line=%d,condition=%s\n", __LINE__, #condition); \
            return 1;                                                           \
        }                                                                       \
    } while (0)

static sensorarrayAdsAutoRangeConfig_t testRangeConfig(void)
{
    return (sensorarrayAdsAutoRangeConfig_t){
        .increaseBelowPermille = 250u,
        .decreaseAbovePermille = 850u,
        .saturationPermille = 980u,
        .maximumAttempts = 6u,
        .pgaRailMarginUv = 300000,
    };
}

static sensorarrayAdsAutoRangeInput_t testRangeInput(void)
{
    return (sensorarrayAdsAutoRangeInput_t){
        .positiveInputUv = 100000,
        .negativeInputUv = 0,
        .avddUv = 3400000,
        .avssUv = -1800000,
        .currentGain = 1u,
        .allowIncrease = true,
    };
}

static int testModeState(void)
{
    sensorarrayMeasurementModeContext_t context;
    sensorarrayMeasurementModeInit(&context);
    sensorarrayMeasurementModeSnapshot_t snapshot = {0};
    CHECK(sensorarrayMeasurementModeCopySnapshot(&context, &snapshot));
    CHECK(snapshot.state == SENSORARRAY_MEASUREMENT_STATE_SAFE);
    CHECK(sensorarrayMeasurementModeAccept(
        &context, SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE, 7u));
    CHECK(sensorarrayMeasurementModeBeginTransition(&context));
    CHECK(sensorarrayMeasurementModeCompleteTransition(&context, 11u, 123u));
    CHECK(sensorarrayMeasurementModeCopySnapshot(&context, &snapshot));
    CHECK(snapshot.activeMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE);
    CHECK(snapshot.generation == 1u && snapshot.appliedRequestId == 7u);
    CHECK(snapshot.appliedFrameSequence == 11u && !snapshot.pending);

    CHECK(sensorarrayMeasurementModeAccept(
        &context, SENSORARRAY_MEASUREMENT_MODE_RESISTANCE, 8u));
    CHECK(sensorarrayMeasurementModeBeginTransition(&context));
    sensorarrayMeasurementModeFailTransition(&context, 0x55u, 99u);
    CHECK(sensorarrayMeasurementModeCopySnapshot(&context, &snapshot));
    CHECK(snapshot.state == SENSORARRAY_MEASUREMENT_STATE_SAFE);
    CHECK(snapshot.activeMode == SENSORARRAY_MEASUREMENT_MODE_NONE);
    CHECK(snapshot.lastError == 0x55u && !snapshot.pending);

    CHECK(sensorarrayMeasurementCellCount(1u) == 8u);
    CHECK(sensorarrayMeasurementCellCount(2u) == 16u);
    CHECK(sensorarrayMeasurementCellCount(4u) == 32u);
    CHECK(sensorarrayMeasurementCellCount(8u) == 64u);
    CHECK(sensorarrayMeasurementCellCount(0u) == 0u);
    return 0;
}

static int testVoltageAndRailMath(void)
{
    sensorarrayAdsRailInput_t input = {
        .measuredRailUv = 5200000,
        .nominalAvddToGroundUv = 3400000,
        .nominalAvssToGroundUv = 1800000,
        .maximumAgeFrames = 4u,
        .valid = true,
    };
    sensorarrayAdsRailSplit_t rail = {0};
    CHECK(sensorarrayAdsMathSplitRail(&input, &rail));
    CHECK(rail.avddUv == 3400000 && rail.avssUv == -1800000);
    CHECK(rail.aincomUv == 800000);
    sensorarrayAdsVoltageLimits_t limits = {
        .minimumUv = -1800000,
        .maximumUv = 3400000,
    };
    int32_t nodeUv = 0;
    sensorarrayCellError_t error = SENSORARRAY_CELL_ERROR_NONE;
    CHECK(sensorarrayAdsMathVoltageFromDifferential(-700000, &rail, &limits,
                                                     &nodeUv, &error));
    CHECK(nodeUv == 100000 && error == SENSORARRAY_CELL_ERROR_NONE);
    CHECK(!sensorarrayAdsMathVoltageFromDifferential(INT32_MAX, &rail, &limits,
                                                      &nodeUv, &error));
    CHECK(error == SENSORARRAY_CELL_ERROR_OVERFLOW ||
          error == SENSORARRAY_CELL_ERROR_RANGE);
    input.ageFrames = 5u;
    CHECK(!sensorarrayAdsMathSplitRail(&input, &rail));
    input.ageFrames = 0u;
    input.valid = false;
    CHECK(!sensorarrayAdsMathSplitRail(&input, &rail));
    return 0;
}

static int testResistanceMath(void)
{
    sensorarrayAdsResistanceConfig_t config = {
        .referenceResistorOhms = 10000u,
        .minimumOhms = 1u,
        .maximumOhms = 100000000u,
        .shortThresholdOhms = 10u,
        .openDenominatorUv = 1000u,
    };
    sensorarrayAdsResistanceResult_t result = sensorarrayAdsMathResistanceDivider(
        700000, -550000, -1800000, &config);
    CHECK(result.valid && result.resistanceMilliohms == 10000000LL);

    result = sensorarrayAdsMathResistanceDivider(700000, -1799500, -1800000, &config);
    CHECK(!result.valid && result.open && result.error == SENSORARRAY_CELL_ERROR_OPEN);
    result = sensorarrayAdsMathResistanceDivider(700000, 699900, -1800000, &config);
    CHECK(!result.valid && result.shorted && result.error == SENSORARRAY_CELL_ERROR_SHORT);
    result = sensorarrayAdsMathResistanceDivider(700000, 800000, -1800000, &config);
    CHECK(!result.valid && result.error == SENSORARRAY_CELL_ERROR_NEGATIVE);

    config.maximumOhms = 1000u;
    result = sensorarrayAdsMathResistanceDivider(700000, -550000, -1800000, &config);
    CHECK(!result.valid && result.error == SENSORARRAY_CELL_ERROR_RANGE);
    config.referenceResistorOhms = UINT32_MAX;
    config.maximumOhms = UINT32_MAX;
    config.openDenominatorUv = 0u;
    result = sensorarrayAdsMathResistanceDivider(INT32_MAX, INT32_MIN + 1,
                                                  INT32_MIN, &config);
    CHECK(!result.valid && result.error == SENSORARRAY_CELL_ERROR_OVERFLOW);
    return 0;
}

static int testStabilityAndInjectedErrors(void)
{
    int32_t stable[] = {100, 103, 101, 102, 99};
    int32_t median = 0;
    CHECK(sensorarrayAdsMathSamplesStable(stable, 5u, 4u, &median));
    CHECK(median == 101);
    int32_t capacitiveTransient[] = {100, 1000, -200, 700, 20};
    CHECK(!sensorarrayAdsMathSamplesStable(capacitiveTransient, 5u, 50u, &median));

    sensorarrayAdsSampleHealth_t health = {
        .transportOk = true,
        .generationAdvanced = true,
        .statusNewData = true,
        .commonModeSafe = true,
    };
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) == SENSORARRAY_CELL_ERROR_NONE);
    health.drdyTimedOut = true;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT);
    health.drdyTimedOut = false;
    health.generationAdvanced = false;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) == SENSORARRAY_CELL_ERROR_STALE);
    health.generationAdvanced = true;
    health.resetOccurred = true;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) == SENSORARRAY_CELL_ERROR_READBACK);
    health.resetOccurred = false;
    health.referenceAlarm = true;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_REFERENCE_ALARM);
    health.referenceAlarm = false;
    health.pgaHighAlarm = true;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE);
    health.pgaHighAlarm = false;
    health.transportOk = false;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_SPI);
    health.transportOk = true;
    health.statusNewData = false;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_STALE);
    health.statusNewData = true;
    health.fullScaleSaturated = true;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_SATURATED);
    health.fullScaleSaturated = false;
    health.commonModeSafe = false;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_COMMON_MODE);
    return 0;
}

static int testOutputCongestionPolicy(void)
{
    CHECK(sensorarrayOutputCongestionDecide(true, false, true) ==
          SENSORARRAY_OUTPUT_CONGESTION_USE_FREE_SLOT);
    CHECK(sensorarrayOutputCongestionDecide(false, true, true) ==
          SENSORARRAY_OUTPUT_CONGESTION_RECLAIM_OLDEST);
    CHECK(sensorarrayOutputCongestionDecide(false, true, false) ==
          SENSORARRAY_OUTPUT_CONGESTION_DROP_INCOMING);
    CHECK(sensorarrayOutputCongestionDecide(false, false, true) ==
          SENSORARRAY_OUTPUT_CONGESTION_DROP_INCOMING);
    return 0;
}

static int testRouteReadbackMismatch(void)
{
    sensorarrayRouteExpectedControl_t expected = {
        .logicalSource = 1,
        .swLevel = 0,
        .selaLevel = 0,
        .selbLevel = 0,
    };
    sensorarrayRouteObservedControl_t observed = {
        .commandedSource = 1,
        .commandedSwLevel = 0,
        .observedSwLevel = 0,
        .commandedSelaLevel = 0,
        .observedSelaLevel = 0,
        .commandedSelbLevel = 0,
        .observedSelbLevel = 0,
    };
    CHECK(sensorarrayRouteControlReadbackMatches(&expected, &observed));
    observed.observedSelbLevel = 1;
    CHECK(!sensorarrayRouteControlReadbackMatches(&expected, &observed));
    return 0;
}

static int testAutoRangeAndCache(void)
{
    sensorarrayAdsAutoRangeConfig_t config = testRangeConfig();
    sensorarrayAdsAutoRangeInput_t input = testRangeInput();
    input.rawCode = INT32_MAX / 10;
    sensorarrayAdsAutoRangeDecision_t decision = sensorarrayAdsAutoRangeDecide(
        &config, &input);
    CHECK(decision.action == SENSORARRAY_ADS_AUTORANGE_INCREASE);
    CHECK(decision.nextGain == 2u);

    input.currentGain = 8u;
    input.rawCode = (int32_t)((int64_t)INT32_MAX * 900LL / 1000LL);
    decision = sensorarrayAdsAutoRangeDecide(&config, &input);
    CHECK(decision.action == SENSORARRAY_ADS_AUTORANGE_DECREASE);
    CHECK(decision.nextGain == 4u);

    input.currentGain = 2u;
    input.rawCode = INT32_MAX / 2;
    decision = sensorarrayAdsAutoRangeDecide(&config, &input);
    CHECK(decision.action == SENSORARRAY_ADS_AUTORANGE_KEEP);

    input.pgaDifferentialAlarm = true;
    decision = sensorarrayAdsAutoRangeDecide(&config, &input);
    CHECK(decision.action == SENSORARRAY_ADS_AUTORANGE_DECREASE);
    input.pgaDifferentialAlarm = false;
    input.currentGain = 1u;
    input.positiveInputUv = 3350000;
    decision = sensorarrayAdsAutoRangeDecide(&config, &input);
    CHECK(decision.action == SENSORARRAY_ADS_AUTORANGE_FAIL);
    CHECK(decision.error == SENSORARRAY_CELL_ERROR_COMMON_MODE);

    input = testRangeInput();
    input.attempt = config.maximumAttempts;
    decision = sensorarrayAdsAutoRangeDecide(&config, &input);
    CHECK(decision.action == SENSORARRAY_ADS_AUTORANGE_FAIL);

    sensorarrayAdsGainCache_t cache;
    sensorarrayAdsGainCacheInit(&cache);
    sensorarrayAdsGainCacheStore(&cache, SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                 3u, 8u);
    uint8_t gain = 0u;
    CHECK(sensorarrayAdsGainCacheGet(&cache,
                                     SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                     3u, &gain));
    CHECK(gain == 8u);
    CHECK(!sensorarrayAdsGainCacheGet(&cache,
                                      SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                      3u, &gain));
    uint32_t generation = cache.generation;
    sensorarrayAdsGainCacheInvalidate(&cache);
    CHECK(cache.generation == generation + 1u);
    CHECK(!sensorarrayAdsGainCacheGet(&cache,
                                      SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                      3u, &gain));
    sensorarrayAdsGainCacheStore(&cache, SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                 3u, 16u);
    sensorarrayAdsGainCacheNoteOverrange(&cache,
                                         SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                         3u);
    CHECK(sensorarrayAdsGainCacheGet(&cache,
                                     SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                     3u, &gain));
    sensorarrayAdsGainCacheNoteOverrange(&cache,
                                         SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                         3u);
    CHECK(!sensorarrayAdsGainCacheGet(&cache,
                                      SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                      3u, &gain));
    return 0;
}

int main(void)
{
    CHECK(testModeState() == 0);
    CHECK(testVoltageAndRailMath() == 0);
    CHECK(testResistanceMath() == 0);
    CHECK(testStabilityAndInjectedErrors() == 0);
    CHECK(testAutoRangeAndCache() == 0);
    CHECK(testOutputCongestionPolicy() == 0);
    CHECK(testRouteReadbackMismatch() == 0);
    puts("MEASUREMENT_LOGIC_TESTS,passed=1");
    return 0;
}
