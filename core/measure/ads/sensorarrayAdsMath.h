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

#ifdef __cplusplus
}
#endif
