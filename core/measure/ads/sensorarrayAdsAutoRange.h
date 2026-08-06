#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensorarrayMeasurementMode.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_ADS_GAIN_COUNT 6u

typedef enum {
    SENSORARRAY_ADS_AUTORANGE_KEEP = 0,
    SENSORARRAY_ADS_AUTORANGE_INCREASE,
    SENSORARRAY_ADS_AUTORANGE_DECREASE,
    SENSORARRAY_ADS_AUTORANGE_FAIL,
} sensorarrayAdsAutoRangeAction_t;

typedef struct {
    uint16_t increaseBelowPermille;
    uint16_t decreaseAbovePermille;
    uint16_t saturationPermille;
    uint8_t maximumAttempts;
    int32_t pgaRailMarginUv;
} sensorarrayAdsAutoRangeConfig_t;

typedef struct {
    int32_t rawCode;
    int32_t positiveInputUv;
    int32_t negativeInputUv;
    int32_t avddUv;
    int32_t avssUv;
    uint8_t currentGain;
    uint8_t attempt;
    bool referenceAlarm;
    bool pgaLowAlarm;
    bool pgaHighAlarm;
    bool pgaDifferentialAlarm;
    bool allowIncrease;
} sensorarrayAdsAutoRangeInput_t;

typedef struct {
    sensorarrayAdsAutoRangeAction_t action;
    sensorarrayCellError_t error;
    uint8_t nextGain;
    uint16_t utilisationPermille;
    bool commonModeSafe;
} sensorarrayAdsAutoRangeDecision_t;

bool sensorarrayAdsAutoRangeGainSupported(uint8_t gain);
uint8_t sensorarrayAdsAutoRangeLowerGain(uint8_t gain);
uint8_t sensorarrayAdsAutoRangeHigherGain(uint8_t gain);
bool sensorarrayAdsAutoRangeCommonModeSafe(int32_t positiveInputUv,
                                          int32_t negativeInputUv,
                                          int32_t avddUv,
                                          int32_t avssUv,
                                          uint8_t gain,
                                          int32_t railMarginUv);
sensorarrayAdsAutoRangeDecision_t sensorarrayAdsAutoRangeDecide(
    const sensorarrayAdsAutoRangeConfig_t *config,
    const sensorarrayAdsAutoRangeInput_t *input);

#ifdef __cplusplus
}
#endif
