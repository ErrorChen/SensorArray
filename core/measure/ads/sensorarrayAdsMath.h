#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensorarrayMeasurementMode.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t measuredRailUv;
    int32_t nominalAvddToGroundUv;
    int32_t nominalAvssToGroundUv;
    uint32_t ageFrames;
    uint32_t maximumAgeFrames;
    bool valid;
} sensorarrayAdsRailInput_t;

typedef struct {
    int32_t avddUv;
    int32_t avssUv;
    int32_t aincomUv;
    uint32_t ageFrames;
    bool valid;
} sensorarrayAdsRailSplit_t;

typedef struct {
    int32_t minimumUv;
    int32_t maximumUv;
} sensorarrayAdsVoltageLimits_t;

typedef struct {
    uint32_t referenceResistorOhms;
    int64_t pathOffsetMilliohms;
    uint32_t minimumOhms;
    uint32_t maximumOhms;
    uint32_t shortThresholdOhms;
    uint32_t openDenominatorUv;
    uint32_t openConfirmMarginUv;
} sensorarrayAdsResistanceConfig_t;

typedef enum {
    SENSORARRAY_ADS_OPEN_SEMANTIC_NONE = 0,
    SENSORARRAY_ADS_OPEN_SEMANTIC_RAW,
    SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED,
} sensorarrayAdsOpenSemantic_t;

typedef enum {
    SENSORARRAY_ADS_OPEN_CONFIRM_NONE = 0,
    SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z,
    SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE,
} sensorarrayAdsOpenConfirm_t;

typedef struct {
    int64_t resistanceMilliohms;
    int64_t numeratorUv;
    int64_t denominatorUv;
    sensorarrayCellError_t error;
    sensorarrayAdsOpenSemantic_t openSemantic;
    uint32_t configurationGeneration;
    bool valid;
    bool open;
    bool shorted;
} sensorarrayAdsResistanceResult_t;

typedef struct {
    bool transportOk;
    bool drdyTimedOut;
    bool generationAdvanced;
    bool statusNewData;
    bool resetOccurred;
    bool referenceAlarm;
    bool pgaLowAlarm;
    bool pgaHighAlarm;
    bool pgaDifferentialAlarm;
    bool fullScaleSaturated;
    bool commonModeSafe;
    uint32_t configurationGeneration;
    uint32_t expectedConfigurationGeneration;
} sensorarrayAdsSampleHealth_t;

typedef struct {
    int32_t nodeUv;
    int32_t avssUv;
    int32_t rawCode;
    uint64_t magnitude;
    uint64_t saturationLimit;
    uint32_t openDenominatorUv;
    uint32_t openConfirmMarginUv;
} sensorarrayAdsHighZCandidateInput_t;

typedef enum {
    SENSORARRAY_ADS_BATTERY_MATH_OK = 0,
    SENSORARRAY_ADS_BATTERY_MATH_RESTORE_FAILED,
    SENSORARRAY_ADS_BATTERY_MATH_ADC_TIMEOUT,
    SENSORARRAY_ADS_BATTERY_MATH_ADC_STALE,
    SENSORARRAY_ADS_BATTERY_MATH_ADC_STATUS,
    SENSORARRAY_ADS_BATTERY_MATH_SPI,
    SENSORARRAY_ADS_BATTERY_MATH_SATURATED,
    SENSORARRAY_ADS_BATTERY_MATH_UNSTABLE,
    SENSORARRAY_ADS_BATTERY_MATH_VBIAS,
    SENSORARRAY_ADS_BATTERY_MATH_RAIL,
    SENSORARRAY_ADS_BATTERY_MATH_REFERENCE,
    SENSORARRAY_ADS_BATTERY_MATH_DIVIDER,
    SENSORARRAY_ADS_BATTERY_MATH_NEGATIVE,
    SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW,
    SENSORARRAY_ADS_BATTERY_MATH_BELOW_MINIMUM,
    SENSORARRAY_ADS_BATTERY_MATH_ABOVE_MAXIMUM,
} sensorarrayAdsBatteryMathError_t;

typedef struct {
    int32_t ain8DifferentialUv;
    int32_t aincomGroundUv;
    int32_t dividerNumerator;
    int32_t dividerDenominator;
    int32_t calibrationScalePpm;
    int32_t calibrationOffsetUv;
    int32_t minimumMv;
    int32_t maximumMv;
    bool restoreOk;
    bool adcTransportOk;
    bool adcTimedOut;
    bool adcFresh;
    bool adcStatusOk;
    bool adcSaturated;
    bool adcUnstable;
    bool vbiasConfirmed;
    bool railValid;
    bool railFresh;
    bool referenceValid;
} sensorarrayAdsBatteryMathInput_t;

typedef struct {
    int32_t ain8GroundUv;
    int32_t batteryMv;
    sensorarrayAdsBatteryMathError_t error;
    bool ain8GroundValid;
    bool valid;
} sensorarrayAdsBatteryMathResult_t;

bool sensorarrayAdsMathSplitRail(const sensorarrayAdsRailInput_t *input,
                                 sensorarrayAdsRailSplit_t *outSplit);
bool sensorarrayAdsMathVoltageFromDifferential(int32_t differentialUv,
                                               const sensorarrayAdsRailSplit_t *rail,
                                               const sensorarrayAdsVoltageLimits_t *limits,
                                               int32_t *outNodeUv,
                                               sensorarrayCellError_t *outError);
sensorarrayAdsResistanceResult_t sensorarrayAdsMathResistanceDivider(
    int32_t matrixReferenceUv,
    int32_t nodeUv,
    int32_t avssUv,
    const sensorarrayAdsResistanceConfig_t *config);
bool sensorarrayAdsMathSamplesStable(const int32_t *samples,
                                     size_t sampleCount,
                                     uint32_t maximumSpreadUv,
                                     int32_t *outMedianUv);
bool sensorarrayAdsMathConfigurationGenerationCurrent(
    uint32_t sampleConfigurationGeneration,
    uint32_t currentConfigurationGeneration);
/* High-Z denominator rule shared by divider, candidate, and confirmation:
 * nodeUv - avssUv <= limit is open-range, including negative denominators.
 * A denominator at or below the open limit is never a finite resistance. */
bool sensorarrayAdsMathHighZOpenCandidate(
    const sensorarrayAdsHighZCandidateInput_t *input);
sensorarrayAdsOpenConfirm_t sensorarrayAdsMathConfirmOpenSet(
    const int32_t *nodeUvSamples,
    size_t sampleCount,
    int32_t avssUv,
    const sensorarrayAdsResistanceConfig_t *config,
    uint32_t maximumSpreadUv,
    int32_t *outMedianNodeUv);
uint8_t sensorarrayAdsMathCombineStatusBytes(
    uint8_t primaryStatus,
    const uint8_t *additionalStatus,
    size_t additionalCount);
uint64_t sensorarrayAdsMathHighZLatchUpdate(
    uint64_t latchMask,
    size_t cellIndex,
    sensorarrayAdsOpenSemantic_t openSemantic);
sensorarrayCellError_t sensorarrayAdsMathClassifySampleHealth(
    const sensorarrayAdsSampleHealth_t *health);
sensorarrayAdsBatteryMathResult_t sensorarrayAdsMathBatteryVoltage(
    const sensorarrayAdsBatteryMathInput_t *input);

#ifdef __cplusplus
}
#endif
