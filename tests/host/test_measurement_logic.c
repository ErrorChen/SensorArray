#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensorarrayAdsAutoRange.h"
#include "sensorarrayAdsCache.h"
#include "sensorarrayAdsFault.h"
#include "sensorarrayAdsMath.h"
#include "sensorarrayBatteryScheduler.h"
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

    sensorarrayMeasurementModeEnterRecovery(&context, 0x66u);
    CHECK(sensorarrayMeasurementModeCopySnapshot(&context, &snapshot));
    CHECK(snapshot.state == SENSORARRAY_MEASUREMENT_STATE_RECOVERY);
    CHECK(snapshot.activeMode == SENSORARRAY_MEASUREMENT_MODE_NONE);
    CHECK(snapshot.lastError == 0x66u && !snapshot.pending);

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

static sensorarrayAdsResistanceConfig_t testHighZConfig(void)
{
    return (sensorarrayAdsResistanceConfig_t){
        .referenceResistorOhms = 10000u,
        .minimumOhms = 1u,
        .maximumOhms = 100000000u,
        .shortThresholdOhms = 10u,
        .openDenominatorUv = 1000u,
        .openConfirmMarginUv = 3000u,
    };
}

static sensorarrayAdsHighZCandidateInput_t testHighZCandidateInput(void)
{
    return (sensorarrayAdsHighZCandidateInput_t){
        .nodeUv = -1000000,
        .avssUv = -1800000,
        .rawCode = 0,
        .magnitude = 0u,
        .saturationLimit = 2000000000u,
        .openDenominatorUv = 1000u,
        .openConfirmMarginUv = 3000u,
    };
}

static int testHighZMath(void)
{
    sensorarrayAdsResistanceConfig_t config = testHighZConfig();
    const int32_t avssUv = -1800000;
    int32_t median = 123;

    CHECK(!sensorarrayAdsMathHighZOpenCandidate(NULL));
    sensorarrayAdsHighZCandidateInput_t candidate = testHighZCandidateInput();
    CHECK(!sensorarrayAdsMathHighZOpenCandidate(&candidate));
    candidate.nodeUv = avssUv + 4000;
    CHECK(sensorarrayAdsMathHighZOpenCandidate(&candidate));
    candidate.nodeUv = avssUv + 4001;
    CHECK(!sensorarrayAdsMathHighZOpenCandidate(&candidate));
    candidate = testHighZCandidateInput();
    candidate.nodeUv = avssUv - 100;
    CHECK(sensorarrayAdsMathHighZOpenCandidate(&candidate));
    candidate = testHighZCandidateInput();
    candidate.magnitude = candidate.saturationLimit;
    CHECK(sensorarrayAdsMathHighZOpenCandidate(&candidate));
    candidate = testHighZCandidateInput();
    candidate.rawCode = INT32_MIN;
    CHECK(sensorarrayAdsMathHighZOpenCandidate(&candidate));
    candidate = testHighZCandidateInput();
    candidate.nodeUv = avssUv + 100000;
    CHECK(!sensorarrayAdsMathHighZOpenCandidate(&candidate));

    CHECK(sensorarrayAdsMathConfirmOpenSet(
              NULL, 3u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE);
    CHECK(median == 0);
    int32_t empty[1] = {avssUv};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              empty, 0u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE);
    int32_t tooMany[10] = {0};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              tooMany, 10u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE);
    int32_t one[1] = {avssUv};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              one, 1u, avssUv, NULL, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE);

    int32_t atOpenLimit[3] = {avssUv + 1000, avssUv + 2000, avssUv + 3000};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              atOpenLimit, 3u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z);
    int32_t nearOpen[3] = {avssUv + 3000, avssUv + 3002, avssUv + 3001};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              nearOpen, 3u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z);
    CHECK(median == avssUv + 3001);
    int32_t finiteStable[3] = {avssUv + 5000, avssUv + 5002, avssUv + 5001};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              finiteStable, 3u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_NONE);
    CHECK(median == avssUv + 5001);
    int32_t negative[3] = {avssUv - 500, avssUv - 500, avssUv - 500};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              negative, 3u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z);
    int32_t unstable[3] = {avssUv + 5000, avssUv + 9000, avssUv + 5000};
    CHECK(sensorarrayAdsMathConfirmOpenSet(
              unstable, 3u, avssUv, &config, 100u, &median) ==
          SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE);

    sensorarrayAdsResistanceResult_t result = sensorarrayAdsMathResistanceDivider(
        700000, avssUv - 100, avssUv, &config);
    CHECK(!result.valid && result.open &&
          result.error == SENSORARRAY_CELL_ERROR_OPEN &&
          result.openSemantic == SENSORARRAY_ADS_OPEN_SEMANTIC_RAW);
    result = sensorarrayAdsMathResistanceDivider(
        700000, avssUv + 1001, avssUv, &config);
    CHECK(result.valid && result.error == SENSORARRAY_CELL_ERROR_NONE &&
          result.openSemantic == SENSORARRAY_ADS_OPEN_SEMANTIC_NONE);

    CHECK(!sensorarrayAdsMathConfigurationGenerationCurrent(1u, 2u));
    CHECK(sensorarrayAdsMathConfigurationGenerationCurrent(2u, 2u));
    CHECK(!sensorarrayAdsMathConfigurationGenerationCurrent(1u, 0u));
    CHECK(!sensorarrayAdsMathConfigurationGenerationCurrent(0u, 1u));
    sensorarrayAdsSampleHealth_t health = {
        .transportOk = true,
        .generationAdvanced = true,
        .statusNewData = true,
        .commonModeSafe = true,
        .configurationGeneration = 4u,
        .expectedConfigurationGeneration = 5u,
    };
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_STALE);
    health.configurationGeneration = 5u;
    CHECK(sensorarrayAdsMathClassifySampleHealth(&health) ==
          SENSORARRAY_CELL_ERROR_NONE);

    const uint8_t triggerAlarm = 0x01u | 0x10u;
    uint8_t cleanConfirmations[3] = {0u, 0u, 0u};
    CHECK(sensorarrayAdsMathCombineStatusBytes(
              triggerAlarm, cleanConfirmations, 3u) == triggerAlarm);
    CHECK(sensorarrayAdsMathCombineStatusBytes(0xA5u, NULL, 3u) == 0xA5u);
    uint8_t alarmConfirmations[3] = {0x02u, 0x00u, 0x04u};
    CHECK(sensorarrayAdsMathCombineStatusBytes(
              0x01u, alarmConfirmations, 3u) == 0x07u);

    uint64_t latch = sensorarrayAdsMathHighZLatchUpdate(
        0u, 3u, SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED);
    CHECK(latch == (UINT64_C(1) << 3u));
    latch = sensorarrayAdsMathHighZLatchUpdate(
        latch, 3u, SENSORARRAY_ADS_OPEN_SEMANTIC_NONE);
    CHECK(latch == 0u);
    latch = sensorarrayAdsMathHighZLatchUpdate(
        0u, 63u, SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED);
    CHECK(latch == (UINT64_C(1) << 63u));
    CHECK(sensorarrayAdsMathHighZLatchUpdate(
              latch, 63u, SENSORARRAY_ADS_OPEN_SEMANTIC_RAW) == latch);
    CHECK(sensorarrayAdsMathHighZLatchUpdate(
              0u, 64u, SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED) == 0u);

    sensorarrayAdsOpenConfirm_t decision = sensorarrayAdsMathConfirmOpenSet(
        atOpenLimit, 3u, avssUv, &config, 100u, &median);
    CHECK(decision == SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z);
    uint64_t recoveryLatch = sensorarrayAdsMathHighZLatchUpdate(
        0u, 2u,
        decision == SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z ?
            SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED :
            SENSORARRAY_ADS_OPEN_SEMANTIC_NONE);
    CHECK((recoveryLatch & (UINT64_C(1) << 2u)) != 0u);
    decision = sensorarrayAdsMathConfirmOpenSet(
        finiteStable, 3u, avssUv, &config, 100u, &median);
    CHECK(decision == SENSORARRAY_ADS_OPEN_CONFIRM_NONE);
    recoveryLatch = sensorarrayAdsMathHighZLatchUpdate(
        recoveryLatch, 2u, SENSORARRAY_ADS_OPEN_SEMANTIC_NONE);
    CHECK(recoveryLatch == 0u);
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

    return 0;
}

static int testProfileAndValueCaches(void)
{
    sensorarrayAdsProfileCache_t profiles;
    sensorarrayAdsProfileCacheInit(&profiles);
    CHECK(sensorarrayAdsProfileCacheStore(&profiles,
                                          SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                          3u,
                                          SENSORARRAY_ADS_INPUT_BYPASS,
                                          1u,
                                          11u));
    sensorarrayAdsCellProfile_t profile = {0};
    CHECK(sensorarrayAdsProfileCacheGet(&profiles,
                                        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                        3u,
                                        &profile));
    CHECK(profile.inputMode == SENSORARRAY_ADS_INPUT_BYPASS);
    CHECK(profile.gain == 1u && profile.lastVerifiedFrame == 11u);
    CHECK(!sensorarrayAdsProfileCacheGet(&profiles,
                                         SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                         3u,
                                         &profile));
    CHECK(!sensorarrayAdsProfileCacheGet(&profiles,
                                         SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                         4u,
                                         &profile));

    CHECK(sensorarrayAdsProfileCacheStore(&profiles,
                                          SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                          3u,
                                          SENSORARRAY_ADS_INPUT_PGA,
                                          16u,
                                          12u));
    sensorarrayAdsProfileCacheNoteFailure(&profiles,
                                          SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                          3u,
                                          true,
                                          2u);
    CHECK(sensorarrayAdsProfileCacheGet(&profiles,
                                        SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                        3u,
                                        &profile));
    sensorarrayAdsProfileCacheNoteFailure(&profiles,
                                          SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                          3u,
                                          true,
                                          2u);
    CHECK(!sensorarrayAdsProfileCacheGet(&profiles,
                                         SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
                                         3u,
                                         &profile));
    uint32_t profileGeneration = profiles.generation;
    sensorarrayAdsProfileCacheInvalidate(&profiles);
    CHECK(profiles.generation == profileGeneration + 1u);
    CHECK(!sensorarrayAdsProfileCacheGet(&profiles,
                                         SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                         3u,
                                         &profile));

    sensorarrayAdsValueCache_t values;
    sensorarrayAdsValueCacheInit(&values);
    sensorarrayAdsCellValueCache_t *cell = sensorarrayAdsValueCacheGet(
        &values, SENSORARRAY_MEASUREMENT_MODE_VOLTAGE, 3u);
    CHECK(cell != NULL && !cell->valid);
    CHECK(!sensorarrayAdsValueCacheShouldFastAccept(cell, 1000, 8u, 4u, 2u,
                                                    true, true, true,
                                                    false, false));
    sensorarrayAdsValueCacheObserve(cell, 1000, 100, 100, 3u, 1u, true);
    sensorarrayAdsValueCacheObserve(cell, 1002, 102, 102, 5u, 2u, true);
    CHECK(cell->noiseEstimateRaw == 4u);
    CHECK(sensorarrayAdsValueCacheShouldFastAccept(cell, 1010, 8u, 4u, 2u,
                                                   true, true, true,
                                                   false, false));
    CHECK(!sensorarrayAdsValueCacheShouldFastAccept(cell, 1100, 8u, 4u, 2u,
                                                    true, true, true,
                                                    false, false));
    CHECK(!sensorarrayAdsValueCacheShouldFastAccept(cell, 1010, 8u, 4u, 2u,
                                                    true, true, true,
                                                    true, false));
    CHECK(!sensorarrayAdsValueCacheShouldFastAccept(cell, 1010, 8u, 4u, 2u,
                                                    true, false, true,
                                                    false, false));
    sensorarrayAdsValueCacheAgeFrame(&values,
                                     SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
                                     8u);
    CHECK(cell->ageFrames == 1u);
    cell->noiseEstimateRaw = 0x40000000u;
    cell->lastRaw = 1000;
    cell->valid = true;
    cell->stableStreak = 4u;
    sensorarrayAdsValueCacheObserve(cell, 1001, 103, 103, 1u, 3u, false);
    CHECK(cell->stableStreak == 5u);
    return 0;
}

static int testRailFingerprint(void)
{
    sensorarrayAdsRailFingerprint_t rail;
    sensorarrayAdsRailFingerprintInit(&rail);
    sensorarrayAdsRailInvalidationReason_t reason = SENSORARRAY_ADS_RAIL_INVALIDATION_NONE;
    CHECK(sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5200000,
                                              3400000, -1800000, 1u, 0u, 1u,
                                              25000u, 5000u, false, &reason));
    CHECK(reason == SENSORARRAY_ADS_RAIL_INVALIDATION_INITIAL);
    CHECK(!sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5224000,
                                               3400000, -1800000, 1u, 0u, 1u,
                                               25000u, 5000u, false, &reason));
    CHECK(reason == SENSORARRAY_ADS_RAIL_INVALIDATION_NONE);
    CHECK(!sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5224000,
                                               3400001, -1800001, 1u, 0u, 1u,
                                               25000u, 5000u, false, &reason));
    CHECK(sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5226001,
                                              3400000, -1800000, 1u, 0u, 1u,
                                              25000u, 5000u, false, &reason));
    CHECK(reason == SENSORARRAY_ADS_RAIL_INVALIDATION_DELTA);
    CHECK(!sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5224000,
                                               3400000, -1800000, 1u, 0u, 1u,
                                               25000u, 5000u, false, &reason));
    CHECK(sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5224000,
                                              3400000, -1800000, 2u, 0u, 1u,
                                              25000u, 5000u, false, &reason));
    CHECK(reason == SENSORARRAY_ADS_RAIL_INVALIDATION_SOURCE);
    CHECK(sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5224000,
                                              3400000, -1800000, 2u, 0u, 2u,
                                              25000u, 5000u, false, &reason));
    CHECK(reason == SENSORARRAY_ADS_RAIL_INVALIDATION_CALIBRATION);
    CHECK(sensorarrayAdsRailFingerprintUpdate(&rail, true, true, 5224000,
                                              3400000, -1800000, 2u, 0u, 2u,
                                              25000u, 5000u, true, &reason));
    CHECK(reason == SENSORARRAY_ADS_RAIL_INVALIDATION_RESET);
    CHECK(rail.hitCount >= 2u && rail.missCount >= 5u);
    return 0;
}

static sensorarrayAdsBatteryMathInput_t testBatteryMathInput(void)
{
    return (sensorarrayAdsBatteryMathInput_t){
        .ain8DifferentialUv = 1200000,
        .aincomGroundUv = 800000,
        .dividerNumerator = 2,
        .dividerDenominator = 1,
        .calibrationScalePpm = 1000000,
        .minimumMv = 2500,
        .maximumMv = 5000,
        .restoreOk = true,
        .adcTransportOk = true,
        .adcFresh = true,
        .adcStatusOk = true,
        .vbiasConfirmed = true,
        .railValid = true,
        .railFresh = true,
        .referenceValid = true,
    };
}

static int testBatteryMath(void)
{
    sensorarrayAdsBatteryMathInput_t input = testBatteryMathInput();
    sensorarrayAdsBatteryMathResult_t result =
        sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(result.valid && result.ain8GroundValid);
    CHECK(result.ain8GroundUv == 2000000 && result.batteryMv == 4000);

    input.dividerNumerator = 3;
    input.dividerDenominator = 2;
    input.minimumMv = 0;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(result.valid && result.batteryMv == 3000);

    /* The same AIN8 differential works with a negative or positive rail
     * midpoint; the caller supplies the physically verified AINCOM-to-GND. */
    input = testBatteryMathInput();
    input.ain8DifferentialUv = 2100000;
    input.aincomGroundUv = -100000;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(result.valid && result.ain8GroundUv == 2000000);
    input.ain8DifferentialUv = 1900000;
    input.aincomGroundUv = 100000;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(result.valid && result.ain8GroundUv == 2000000);

    input = testBatteryMathInput();
    input.railFresh = false;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid && result.error == SENSORARRAY_ADS_BATTERY_MATH_RAIL);
    input.adcTransportOk = false;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid && result.error == SENSORARRAY_ADS_BATTERY_MATH_RAIL);
    input.railFresh = true;
    input.adcTransportOk = true;
    input.railValid = false;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid && result.error == SENSORARRAY_ADS_BATTERY_MATH_RAIL);

    input = testBatteryMathInput();
    input.dividerDenominator = 0;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid && result.error == SENSORARRAY_ADS_BATTERY_MATH_DIVIDER);
    input = testBatteryMathInput();
    input.ain8DifferentialUv = -900000;
    input.aincomGroundUv = 0;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid && result.error == SENSORARRAY_ADS_BATTERY_MATH_NEGATIVE);
    input = testBatteryMathInput();
    input.ain8DifferentialUv = INT32_MAX;
    input.aincomGroundUv = INT32_MAX;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid && result.error == SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW);

    input = testBatteryMathInput();
    input.ain8DifferentialUv = 100000;
    input.aincomGroundUv = 800000;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid &&
          result.error == SENSORARRAY_ADS_BATTERY_MATH_BELOW_MINIMUM);
    input = testBatteryMathInput();
    input.ain8DifferentialUv = 2000000;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid &&
          result.error == SENSORARRAY_ADS_BATTERY_MATH_ABOVE_MAXIMUM);

    input = testBatteryMathInput();
    input.vbiasConfirmed = false;
    CHECK(sensorarrayAdsMathBatteryVoltage(&input).error ==
          SENSORARRAY_ADS_BATTERY_MATH_VBIAS);
    input = testBatteryMathInput();
    input.adcFresh = false;
    CHECK(sensorarrayAdsMathBatteryVoltage(&input).error ==
          SENSORARRAY_ADS_BATTERY_MATH_ADC_STALE);
    input = testBatteryMathInput();
    input.adcTimedOut = true;
    CHECK(sensorarrayAdsMathBatteryVoltage(&input).error ==
          SENSORARRAY_ADS_BATTERY_MATH_ADC_TIMEOUT);
    input = testBatteryMathInput();
    input.adcStatusOk = false;
    CHECK(sensorarrayAdsMathBatteryVoltage(&input).error ==
          SENSORARRAY_ADS_BATTERY_MATH_ADC_STATUS);
    input = testBatteryMathInput();
    input.restoreOk = false;
    CHECK(sensorarrayAdsMathBatteryVoltage(&input).error ==
          SENSORARRAY_ADS_BATTERY_MATH_RESTORE_FAILED);
    input = testBatteryMathInput();
    input.adcSaturated = true;
    CHECK(sensorarrayAdsMathBatteryVoltage(&input).error ==
          SENSORARRAY_ADS_BATTERY_MATH_SATURATED);
    input = testBatteryMathInput();
    input.adcUnstable = true;
    CHECK(sensorarrayAdsMathBatteryVoltage(&input).error ==
          SENSORARRAY_ADS_BATTERY_MATH_UNSTABLE);
    return 0;
}

static int testRegisterShadowAndOwnership(void)
{
    sensorarrayAdsRegisterCache_t registers;
    sensorarrayAdsRegisterCacheInit(&registers);
    CHECK(sensorarrayAdsRegisterCacheAcquire(&registers,
                                             SENSORARRAY_ADS_OWNER_MATRIX));
    CHECK(!sensorarrayAdsRegisterCacheAcquire(&registers,
                                              SENSORARRAY_ADS_OWNER_BATTERY));
    CHECK(sensorarrayAdsRegisterCacheNeedsWrite(&registers,
                                                SENSORARRAY_ADS_REGISTER_MODE2,
                                                0x09u));
    sensorarrayAdsRegisterCacheNoteWrite(&registers,
                                         SENSORARRAY_ADS_REGISTER_MODE2,
                                         0x09u,
                                         false);
    CHECK(!sensorarrayAdsRegisterCacheNeedsWrite(&registers,
                                                 SENSORARRAY_ADS_REGISTER_MODE2,
                                                 0x09u));
    CHECK(sensorarrayAdsRegisterCacheNeedsWrite(&registers,
                                                SENSORARRAY_ADS_REGISTER_MODE2,
                                                0x08u));
    CHECK(sensorarrayAdsRegisterCacheNoteReadback(&registers,
                                                  SENSORARRAY_ADS_REGISTER_MODE2,
                                                  0x09u));
    CHECK(!sensorarrayAdsRegisterCacheNoteReadback(&registers,
                                                   SENSORARRAY_ADS_REGISTER_MODE2,
                                                   0x08u));
    CHECK(sensorarrayAdsRegisterCacheRelease(&registers,
                                             SENSORARRAY_ADS_OWNER_MATRIX));
    CHECK(sensorarrayAdsRegisterCacheAcquire(&registers,
                                             SENSORARRAY_ADS_OWNER_BATTERY));
    uint32_t generation = registers.generation;
    sensorarrayAdsRegisterCacheInvalidate(&registers);
    CHECK(registers.generation == generation + 1u);
    CHECK(sensorarrayAdsRegisterCacheNeedsWrite(&registers,
                                                SENSORARRAY_ADS_REGISTER_MODE2,
                                                0x09u));
    CHECK(sensorarrayAdsRegisterCacheRelease(&registers,
                                             SENSORARRAY_ADS_OWNER_BATTERY));
    return 0;
}

static int testBatteryTimeScheduler(void)
{
    sensorarrayBatteryScheduler_t scheduler;
    sensorarrayBatterySchedulerInit(&scheduler, true, 1000u, 3000u, 0u);
    CHECK(!sensorarrayBatterySchedulerIsDue(&scheduler, 999999u));
    CHECK(sensorarrayBatterySchedulerIsDue(&scheduler, 1000000u));
    CHECK(sensorarrayBatterySchedulerEvaluateGap(&scheduler,
                                                  1000000u,
                                                  2000u,
                                                  2500u,
                                                  200u) ==
          SENSORARRAY_BATTERY_DECISION_DEFER);
    CHECK(sensorarrayBatterySchedulerEvaluateGap(&scheduler,
                                                  1000100u,
                                                  3000u,
                                                  2500u,
                                                  200u) ==
          SENSORARRAY_BATTERY_DECISION_RUN_GAP);
    sensorarrayBatterySchedulerRecordRun(&scheduler,
                                          1003000u,
                                          2900u,
                                          false,
                                          true);
    CHECK(!sensorarrayBatterySchedulerIsDue(&scheduler, 1999999u));
    CHECK(sensorarrayBatterySchedulerIsDue(&scheduler, 2000000u));

    /* A 7 ms transaction must not shift every subsequent 1 Hz deadline.
     * The old completion+period policy lost roughly four samples over a
     * 120-second hardware dwell. */
    sensorarrayBatteryScheduler_t noDrift;
    sensorarrayBatterySchedulerInit(&noDrift, true, 1000u, 3000u, 0u);
    for (uint64_t second = 1u; second <= 120u; ++second) {
        uint64_t dueUs = second * 1000000u;
        CHECK(!sensorarrayBatterySchedulerIsDue(&noDrift, dueUs - 1u));
        CHECK(sensorarrayBatterySchedulerIsDue(&noDrift, dueUs));
        sensorarrayBatterySchedulerRecordRun(&noDrift,
                                              dueUs + 7000u,
                                              7000u,
                                              true,
                                              true);
    }
    CHECK(noDrift.runCount == 120u);
    CHECK(noDrift.nextDueUs == 121000000u);

    /* CAP gives a due job one real conversion gap first. If the measured job
     * plus guard cannot fit, it runs at that frame's complete boundary rather
     * than missing three 1 Hz schedule slots. */
    sensorarrayBatteryScheduler_t capFallback;
    sensorarrayBatterySchedulerInit(&capFallback, true, 1000u, 3000u, 0u);
    CHECK(sensorarrayBatterySchedulerEvaluateGap(&capFallback,
                                                  1000000u,
                                                  3000u,
                                                  5500u,
                                                  500u) ==
          SENSORARRAY_BATTERY_DECISION_DEFER);
    CHECK(sensorarrayBatterySchedulerEvaluateBoundary(&capFallback,
                                                       1000100u,
                                                       true) ==
          SENSORARRAY_BATTERY_DECISION_RUN_BOUNDARY);
    sensorarrayBatterySchedulerRecordRun(&capFallback,
                                          1007000u,
                                          6900u,
                                          true,
                                          true);
    CHECK(capFallback.boundaryCount == 1u);
    CHECK(!capFallback.gapDeferred);
    CHECK(capFallback.nextDueUs == 2000000u);

    /* Calling the policy at a 3-FPS or 100-FPS cadence cannot change the
     * wall-clock due time: frame count is deliberately absent from its API. */
    sensorarrayBatteryScheduler_t slow;
    sensorarrayBatteryScheduler_t fast;
    sensorarrayBatterySchedulerInit(&slow, true, 1000u, 3000u, 0u);
    sensorarrayBatterySchedulerInit(&fast, true, 1000u, 3000u, 0u);
    CHECK(!sensorarrayBatterySchedulerIsDue(&slow, 333333u));
    CHECK(!sensorarrayBatterySchedulerIsDue(&slow, 666666u));
    for (uint64_t nowUs = 10000u; nowUs < 1000000u; nowUs += 10000u) {
        CHECK(!sensorarrayBatterySchedulerIsDue(&fast, nowUs));
    }
    CHECK(sensorarrayBatterySchedulerIsDue(&slow, 1000000u));
    CHECK(sensorarrayBatterySchedulerIsDue(&fast, 1000000u));

    CHECK(sensorarrayBatterySchedulerEvaluateBoundary(&scheduler,
                                                       4999999u,
                                                       true) ==
          SENSORARRAY_BATTERY_DECISION_DEFER);
    CHECK(sensorarrayBatterySchedulerEvaluateBoundary(&scheduler,
                                                       5003000u,
                                                       true) ==
          SENSORARRAY_BATTERY_DECISION_RUN_BOUNDARY);

    CHECK(sensorarrayBatterySchedulerConfigure(&scheduler,
                                                false,
                                                0u,
                                                6000000u));
    CHECK(sensorarrayBatterySchedulerEvaluateBoundary(&scheduler,
                                                       7000000u,
                                                       false) ==
          SENSORARRAY_BATTERY_DECISION_DISABLED);
    sensorarrayBatterySchedulerRequestNow(&scheduler, 7000000u);
    CHECK(sensorarrayBatterySchedulerEvaluateGap(&scheduler,
                                                  7000000u,
                                                  10000u,
                                                  2500u,
                                                  200u) ==
          SENSORARRAY_BATTERY_DECISION_DEFER);
    CHECK(sensorarrayBatterySchedulerEvaluateBoundary(&scheduler,
                                                       7000000u,
                                                       false) ==
          SENSORARRAY_BATTERY_DECISION_RUN_BOUNDARY);
    sensorarrayBatterySchedulerRecordRun(&scheduler,
                                          7003000u,
                                          3000u,
                                          true,
                                          false);
    CHECK(scheduler.boundaryCount == 1u);
    CHECK(scheduler.restoreFailureCount == 1u);
    CHECK(sensorarrayBatterySchedulerAgeMs(&scheduler, 8003000u) == 1000u);
    return 0;
}

typedef struct {
    char text[512];
    size_t length;
    unsigned calls;
} sensorarrayAdsFaultTestSink_t;

static void sensorarrayAdsFaultTestSink(const char *line,
                                        size_t length,
                                        void *context)
{
    sensorarrayAdsFaultTestSink_t *sink =
        (sensorarrayAdsFaultTestSink_t *)context;
    if (!sink) {
        return;
    }
    sink->calls++;
    if (sink->length == 0u && length < sizeof(sink->text)) {
        memcpy(sink->text, line, length);
        sink->text[length] = '\0';
        sink->length = length;
    }
}

static int testAdsFaultStageNames(void)
{
    static const char *const expected[] = {
        "MATRIX_ROUTE",
        "MATRIX_READ",
        "MATRIX_READBACK",
        "MATRIX_DRDY",
        "BATTERY_GAP",
        "BATTERY_RESTORE",
        "RAIL_MONITOR",
        "PROFILE_TRANSITION",
        "RECOVERY_START",
        "RECOVERY_ATTEMPT",
        "RECOVERY_RESUME",
        "RECOVERY_FAILED",
        "UNKNOWN",
    };
    for (size_t index = 0u;
         index < SENSORARRAY_ADS_FAULT_STAGE_COUNT;
         ++index) {
        CHECK(strcmp(sensorarrayAdsFaultStageName(
                         (sensorarrayAdsFaultStage_t)index),
                     expected[index]) == 0);
    }
    CHECK(strcmp(sensorarrayAdsFaultStageName(
                     (sensorarrayAdsFaultStage_t)99u),
                 "UNKNOWN") == 0);
    return 0;
}

static int testAdsFaultFormatRequiredFields(void)
{
    sensorarrayAdsFaultEvent_t event = {
        .stage = SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK,
        .err = ESP_ERR_INVALID_RESPONSE,
        .bootId = 7u,
        .bootCount = 3u,
        .seq = 42u,
        .modeGeneration = 2u,
        .profileGeneration = 3u,
        .rowGeneration = 4u,
        .rowRequestId = 5u,
        .mode = "RESISTANCE",
        .profile = "VVVVVVRR",
        .route = "SAFE",
        .owner = "MATRIX",
        .drdyGeneration = 6u,
        .configGeneration = 8u,
        .railUv = 5200000,
        .reference = "internal",
        .restoreExpected = 1,
        .restoreActual = 2,
        .attempt = 2u,
        .outcome = SENSORARRAY_ADS_FAULT_OUTCOME_RESUMED,
        .railValid = true,
        .restoreExpectedValid = true,
        .restoreActualValid = true,
    };
    char line[SENSORARRAY_ADS_FAULT_LINE_MAX + 1u];
    size_t length = sensorarrayAdsFaultFormat(&event, line, sizeof(line));
    CHECK(length > 0u);
    CHECK(length <= SENSORARRAY_ADS_FAULT_LINE_MAX);
    CHECK(length == strlen(line));
    CHECK(line[length - 1u] == '\n');
    CHECK(strstr(line, "ADSFAULT,stage=MATRIX_READBACK,") != NULL);
    CHECK(strstr(line, "err=0x10c") != NULL);
    CHECK(strstr(line, "boot=3,bootId=7") != NULL);
    CHECK(strstr(line, "seq=42") != NULL);
    CHECK(strstr(line, "mode=RESISTANCE,modeGen=2") != NULL);
    CHECK(strstr(line, "profileGen=3") != NULL);
    CHECK(strstr(line, "rowGen=4,rowReq=5") != NULL);
    CHECK(strstr(line, "profile=VVVVVVRR") != NULL);
    CHECK(strstr(line, "route=SAFE,owner=MATRIX") != NULL);
    CHECK(strstr(line, "drdyGen=6,cfgGen=8") != NULL);
    CHECK(strstr(line, "rail=5200000,ref=internal") != NULL);
    CHECK(strstr(line, "restoreExp=1,restoreAct=2") != NULL);
    CHECK(strstr(line, "attempt=2,outcome=resumed") != NULL);
    return 0;
}

static int testAdsFaultTruncationAndBound(void)
{
    sensorarrayAdsFaultEvent_t event = {
        .stage = SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR,
        .err = ESP_ERR_TIMEOUT,
        .seq = 1u,
        .mode = "a-very-long-mode-name-that-must-not-blow-the-line",
        .profile = "123456789012345678901234567890",
        .route = "a-very-long-route-name-that-must-not-blow-the-line",
        .owner = "a-very-long-owner-name-that-must-not-blow-the-line",
        .reference = "a-very-long-reference-name-that-must-not-blow-the-line",
    };
    char small[8];
    CHECK(sensorarrayAdsFaultFormat(&event, small, sizeof(small)) == 0u);
    CHECK(small[0] == '\0');

    char line[SENSORARRAY_ADS_FAULT_LINE_MAX + 1u];
    size_t length = sensorarrayAdsFaultFormat(&event, line, sizeof(line));
    CHECK(length > 0u);
    CHECK(length <= SENSORARRAY_ADS_FAULT_LINE_MAX);
    CHECK(length == strlen(line));
    CHECK(strstr(line, "profile=12345678") != NULL);
    return 0;
}

static int testAdsFaultOnePerFault(void)
{
    sensorarrayAdsFaultEvent_t event = {
        .stage = SENSORARRAY_ADS_FAULT_STAGE_BATTERY_RESTORE,
        .err = ESP_FAIL,
        .seq = 99u,
        .owner = "BATTERY",
    };
    sensorarrayAdsFaultTestSink_t sink = {0};
    size_t length = sensorarrayAdsFaultEmit(
        &event, 1000000u, sensorarrayAdsFaultTestSink, &sink);
    CHECK(length > 0u);
    CHECK(sink.calls == 1u);
    CHECK(sink.length == length);
    CHECK(strncmp(sink.text, "ADSFAULT,stage=BATTERY_RESTORE,", 31u) == 0);
    CHECK(sink.text[sink.length - 1u] == '\n');

    size_t suppressed = sensorarrayAdsFaultEmit(
        &event, 1000001u, sensorarrayAdsFaultTestSink, &sink);
    CHECK(suppressed == 0u);
    CHECK(sink.calls == 1u);
    return 0;
}

static int testMeasurementRecoverySuccessOnLaterAttempt(void)
{
    sensorarrayMeasurementRecovery_t recovery;
    sensorarrayMeasurementRecoveryInit(&recovery);
    CHECK(recovery.maximumAttempts == SENSORARRAY_MEASUREMENT_RECOVERY_MAX_ATTEMPTS);
    CHECK(!sensorarrayMeasurementRecoveryIsActive(&recovery));
    CHECK(!sensorarrayMeasurementRecoveryIsTerminal(&recovery));

    CHECK(sensorarrayMeasurementRecoveryStart(
        &recovery,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        71u,
        SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_ADS_RESTORE,
        0x102u,
        9u,
        70u));
    CHECK(sensorarrayMeasurementRecoveryIsActive(&recovery));
    CHECK(!sensorarrayMeasurementRecoveryIsTerminal(&recovery));
    CHECK(recovery.resumeMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE);
    CHECK(recovery.resumeRequestId == 71u);
    CHECK(recovery.triggerRequestId == 70u);
    CHECK(recovery.triggerError == 0x102u);
    CHECK(recovery.outcome == SENSORARRAY_MEASUREMENT_RECOVERY_OUTCOME_STARTED);

    CHECK(sensorarrayMeasurementRecoveryBeginAttempt(&recovery));
    CHECK(recovery.attempt == 1u);
    sensorarrayMeasurementRecoveryComplete(&recovery, false, 0x103u);
    CHECK(sensorarrayMeasurementRecoveryIsActive(&recovery));
    CHECK(!sensorarrayMeasurementRecoveryIsTerminal(&recovery));
    CHECK(recovery.outcome == SENSORARRAY_MEASUREMENT_RECOVERY_OUTCOME_ATTEMPT);

    CHECK(sensorarrayMeasurementRecoveryBeginAttempt(&recovery));
    CHECK(recovery.attempt == 2u);
    sensorarrayMeasurementRecoveryComplete(&recovery, true, ESP_OK);
    CHECK(!sensorarrayMeasurementRecoveryIsActive(&recovery));
    CHECK(!sensorarrayMeasurementRecoveryIsTerminal(&recovery));
    CHECK(recovery.completedAttempts == 2u);
    CHECK(recovery.outcome == SENSORARRAY_MEASUREMENT_RECOVERY_OUTCOME_RESUMED);
    CHECK(recovery.resumeMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE);
    return 0;
}

static int testMeasurementRecoveryExhaustion(void)
{
    sensorarrayMeasurementRecovery_t recovery;
    sensorarrayMeasurementRecoveryInit(&recovery);
    CHECK(sensorarrayMeasurementRecoveryStart(
        &recovery,
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        0u,
        SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_BATTERY_RESTORE,
        0x104u,
        5u,
        6u));
    CHECK(!sensorarrayMeasurementRecoveryStart(
        &recovery,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        0u,
        SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_ADS_RESTORE,
        0u,
        0u,
        0u));

    for (uint32_t attempt = 1u;
         attempt <= SENSORARRAY_MEASUREMENT_RECOVERY_MAX_ATTEMPTS;
         ++attempt) {
        CHECK(sensorarrayMeasurementRecoveryBeginAttempt(&recovery));
        CHECK(recovery.attempt == attempt);
        sensorarrayMeasurementRecoveryComplete(
            &recovery, false, 0x200u + attempt);
    }
    CHECK(!sensorarrayMeasurementRecoveryIsActive(&recovery));
    CHECK(sensorarrayMeasurementRecoveryIsTerminal(&recovery));
    CHECK(recovery.completedAttempts ==
          SENSORARRAY_MEASUREMENT_RECOVERY_MAX_ATTEMPTS);
    CHECK(recovery.outcome == SENSORARRAY_MEASUREMENT_RECOVERY_OUTCOME_FAILED);
    CHECK(recovery.lastError ==
          0x200u + SENSORARRAY_MEASUREMENT_RECOVERY_MAX_ATTEMPTS);
    CHECK(!sensorarrayMeasurementRecoveryBeginAttempt(&recovery));
    CHECK(sensorarrayMeasurementRecoveryIsTerminal(&recovery));
    return 0;
}

static int testMeasurementRecoveryIgnoresOrdinaryInvalidBattery(void)
{
    CHECK(sensorarrayMeasurementRecoveryTriggerForBattery(true) ==
          SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_BATTERY_RESTORE);
    CHECK(sensorarrayMeasurementRecoveryTriggerForBattery(false) ==
          SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_NONE);

    sensorarrayMeasurementRecovery_t recovery;
    sensorarrayMeasurementRecoveryInit(&recovery);
    CHECK(!sensorarrayMeasurementRecoveryStart(
        &recovery,
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        0u,
        SENSORARRAY_MEASUREMENT_RECOVERY_TRIGGER_NONE,
        ESP_ERR_INVALID_RESPONSE,
        1u,
        0u));
    CHECK(!sensorarrayMeasurementRecoveryIsActive(&recovery));
    CHECK(!sensorarrayMeasurementRecoveryIsTerminal(&recovery));
    CHECK(!sensorarrayMeasurementRecoveryBeginAttempt(&recovery));

    sensorarrayAdsBatteryMathInput_t input = testBatteryMathInput();
    input.ain8DifferentialUv = 100000;
    input.aincomGroundUv = 800000;
    sensorarrayAdsBatteryMathResult_t result =
        sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid);
    CHECK(result.error != SENSORARRAY_ADS_BATTERY_MATH_RESTORE_FAILED);
    input.restoreOk = false;
    result = sensorarrayAdsMathBatteryVoltage(&input);
    CHECK(!result.valid);
    CHECK(result.error == SENSORARRAY_ADS_BATTERY_MATH_RESTORE_FAILED);
    return 0;
}

int main(void)
{
    CHECK(testModeState() == 0);
    CHECK(testVoltageAndRailMath() == 0);
    CHECK(testResistanceMath() == 0);
    CHECK(testStabilityAndInjectedErrors() == 0);
    CHECK(testHighZMath() == 0);
    CHECK(testAutoRangeAndCache() == 0);
    CHECK(testProfileAndValueCaches() == 0);
    CHECK(testRailFingerprint() == 0);
    CHECK(testRegisterShadowAndOwnership() == 0);
    CHECK(testBatteryTimeScheduler() == 0);
    CHECK(testBatteryMath() == 0);
    CHECK(testOutputCongestionPolicy() == 0);
    CHECK(testRouteReadbackMismatch() == 0);
    CHECK(testAdsFaultStageNames() == 0);
    CHECK(testAdsFaultFormatRequiredFields() == 0);
    CHECK(testAdsFaultTruncationAndBound() == 0);
    CHECK(testAdsFaultOnePerFault() == 0);
    CHECK(testMeasurementRecoverySuccessOnLaterAttempt() == 0);
    CHECK(testMeasurementRecoveryExhaustion() == 0);
    CHECK(testMeasurementRecoveryIgnoresOrdinaryInvalidBattery() == 0);
    puts("MEASUREMENT_LOGIC_TESTS,passed=1");
    return 0;
}
