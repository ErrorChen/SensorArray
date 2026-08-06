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
} sensorarrayAdsResistanceConfig_t;

typedef struct {
    int64_t resistanceMilliohms;
    int64_t numeratorUv;
    int64_t denominatorUv;
    sensorarrayCellError_t error;
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
} sensorarrayAdsSampleHealth_t;

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
sensorarrayCellError_t sensorarrayAdsMathClassifySampleHealth(
    const sensorarrayAdsSampleHealth_t *health);
sensorarrayAdsBatteryMathResult_t sensorarrayAdsMathBatteryVoltage(
    const sensorarrayAdsBatteryMathInput_t *input);

#ifdef __cplusplus
}
#endif
