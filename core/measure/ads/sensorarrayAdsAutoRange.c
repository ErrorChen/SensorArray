#include "sensorarrayAdsAutoRange.h"

#include <limits.h>
#include <string.h>

static const uint8_t s_sensorarrayAdsGains[SENSORARRAY_ADS_GAIN_COUNT] = {
    1u, 2u, 4u, 8u, 16u, 32u,
};

static int sensorarrayAdsAutoRangeGainIndex(uint8_t gain)
{
    for (uint8_t index = 0u; index < SENSORARRAY_ADS_GAIN_COUNT; ++index) {
        if (s_sensorarrayAdsGains[index] == gain) {
            return (int)index;
        }
    }
    return -1;
}

static int sensorarrayAdsGainCacheModeIndex(sensorarrayMeasurementMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE:
        return 0;
    case SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:
        return 1;
    case SENSORARRAY_MEASUREMENT_MODE_RESISTANCE:
        return 2;
    case SENSORARRAY_MEASUREMENT_MODE_NONE:
    default:
        return -1;
    }
}

bool sensorarrayAdsAutoRangeGainSupported(uint8_t gain)
{
    return sensorarrayAdsAutoRangeGainIndex(gain) >= 0;
}

uint8_t sensorarrayAdsAutoRangeLowerGain(uint8_t gain)
{
    int index = sensorarrayAdsAutoRangeGainIndex(gain);
    return index > 0 ? s_sensorarrayAdsGains[index - 1] : 1u;
}

uint8_t sensorarrayAdsAutoRangeHigherGain(uint8_t gain)
{
    int index = sensorarrayAdsAutoRangeGainIndex(gain);
    return index >= 0 && index + 1 < (int)SENSORARRAY_ADS_GAIN_COUNT ?
        s_sensorarrayAdsGains[index + 1] : 32u;
}

bool sensorarrayAdsAutoRangeCommonModeSafe(int32_t positiveInputUv,
                                          int32_t negativeInputUv,
                                          int32_t avddUv,
                                          int32_t avssUv,
                                          uint8_t gain,
                                          int32_t railMarginUv)
{
    if (!sensorarrayAdsAutoRangeGainSupported(gain) || avddUv <= avssUv ||
        railMarginUv < 0) {
        return false;
    }
    int64_t differentialUv = (int64_t)positiveInputUv - negativeInputUv;
    if (differentialUv < 0) {
        differentialUv = -differentialUv;
    }
    int64_t gainHeadroomUv = differentialUv * (int64_t)(gain - 1u) / 2LL;
    int64_t lowerLimitUv = (int64_t)avssUv + railMarginUv + gainHeadroomUv;
    int64_t upperLimitUv = (int64_t)avddUv - railMarginUv - gainHeadroomUv;
    return (int64_t)positiveInputUv > lowerLimitUv &&
           (int64_t)positiveInputUv < upperLimitUv &&
           (int64_t)negativeInputUv > lowerLimitUv &&
           (int64_t)negativeInputUv < upperLimitUv;
}

static uint16_t sensorarrayAdsAutoRangeUtilisationPermille(int32_t rawCode)
{
    uint64_t magnitude = rawCode == INT32_MIN ?
        (uint64_t)INT32_MAX + 1u : (uint64_t)(rawCode < 0 ? -rawCode : rawCode);
    uint64_t permille = (magnitude * 1000u + (uint64_t)INT32_MAX / 2u) /
                        (uint64_t)INT32_MAX;
    return permille > UINT16_MAX ? UINT16_MAX : (uint16_t)permille;
}

sensorarrayAdsAutoRangeDecision_t sensorarrayAdsAutoRangeDecide(
    const sensorarrayAdsAutoRangeConfig_t *config,
    const sensorarrayAdsAutoRangeInput_t *input)
{
    sensorarrayAdsAutoRangeDecision_t decision = {
        .action = SENSORARRAY_ADS_AUTORANGE_FAIL,
        .error = SENSORARRAY_CELL_ERROR_AUTORANGE,
        .nextGain = 1u,
    };
    if (!config || !input || !sensorarrayAdsAutoRangeGainSupported(input->currentGain) ||
        config->increaseBelowPermille >= config->decreaseAbovePermille ||
        config->decreaseAbovePermille > config->saturationPermille ||
        config->saturationPermille > 1000u || config->maximumAttempts == 0u) {
        return decision;
    }
    decision.nextGain = input->currentGain;
    decision.utilisationPermille = sensorarrayAdsAutoRangeUtilisationPermille(input->rawCode);
    decision.commonModeSafe = sensorarrayAdsAutoRangeCommonModeSafe(
        input->positiveInputUv,
        input->negativeInputUv,
        input->avddUv,
        input->avssUv,
        input->currentGain,
        config->pgaRailMarginUv);

    if (input->attempt >= config->maximumAttempts) {
        return decision;
    }
    if (input->referenceAlarm) {
        decision.error = SENSORARRAY_CELL_ERROR_REFERENCE_ALARM;
        return decision;
    }

    bool pgaAbsoluteAlarm = input->pgaLowAlarm || input->pgaHighAlarm;
    bool mustDecrease = pgaAbsoluteAlarm || input->pgaDifferentialAlarm ||
                        decision.utilisationPermille >= config->decreaseAbovePermille ||
                        !decision.commonModeSafe;
    if (mustDecrease) {
        if (input->currentGain == 1u) {
            decision.error = pgaAbsoluteAlarm ? SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE :
                (input->pgaDifferentialAlarm ? SENSORARRAY_CELL_ERROR_PGA_DIFFERENTIAL :
                 (!decision.commonModeSafe ? SENSORARRAY_CELL_ERROR_COMMON_MODE :
                                             SENSORARRAY_CELL_ERROR_SATURATED));
            return decision;
        }
        decision.action = SENSORARRAY_ADS_AUTORANGE_DECREASE;
        decision.nextGain = sensorarrayAdsAutoRangeLowerGain(input->currentGain);
        decision.error = SENSORARRAY_CELL_ERROR_NONE;
        return decision;
    }

    if (input->allowIncrease &&
        decision.utilisationPermille < config->increaseBelowPermille &&
        input->currentGain < 32u) {
        uint8_t higherGain = sensorarrayAdsAutoRangeHigherGain(input->currentGain);
        if (sensorarrayAdsAutoRangeCommonModeSafe(input->positiveInputUv,
                                                  input->negativeInputUv,
                                                  input->avddUv,
                                                  input->avssUv,
                                                  higherGain,
                                                  config->pgaRailMarginUv)) {
            decision.action = SENSORARRAY_ADS_AUTORANGE_INCREASE;
            decision.nextGain = higherGain;
            decision.error = SENSORARRAY_CELL_ERROR_NONE;
            return decision;
        }
    }

    if (decision.utilisationPermille >= config->saturationPermille) {
        decision.error = SENSORARRAY_CELL_ERROR_SATURATED;
        return decision;
    }
    decision.action = SENSORARRAY_ADS_AUTORANGE_KEEP;
    decision.error = SENSORARRAY_CELL_ERROR_NONE;
    return decision;
}

void sensorarrayAdsGainCacheInit(sensorarrayAdsGainCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache, 0, sizeof(*cache));
    cache->generation = 1u;
}

void sensorarrayAdsGainCacheInvalidate(sensorarrayAdsGainCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache->valid, 0, sizeof(cache->valid));
    memset(cache->overrangeStreak, 0, sizeof(cache->overrangeStreak));
    cache->generation++;
}

bool sensorarrayAdsGainCacheGet(const sensorarrayAdsGainCache_t *cache,
                                sensorarrayMeasurementMode_t mode,
                                uint8_t cellIndex,
                                uint8_t *outGain)
{
    int modeIndex = sensorarrayAdsGainCacheModeIndex(mode);
    if (!cache || !outGain || modeIndex < 0 ||
        cellIndex >= SENSORARRAY_MEASUREMENT_MAX_CELLS ||
        cache->valid[modeIndex][cellIndex] == 0u) {
        return false;
    }
    uint8_t gain = cache->gain[modeIndex][cellIndex];
    if (!sensorarrayAdsAutoRangeGainSupported(gain)) {
        return false;
    }
    *outGain = gain;
    return true;
}

void sensorarrayAdsGainCacheStore(sensorarrayAdsGainCache_t *cache,
                                  sensorarrayMeasurementMode_t mode,
                                  uint8_t cellIndex,
                                  uint8_t gain)
{
    int modeIndex = sensorarrayAdsGainCacheModeIndex(mode);
    if (!cache || modeIndex < 0 || cellIndex >= SENSORARRAY_MEASUREMENT_MAX_CELLS ||
        !sensorarrayAdsAutoRangeGainSupported(gain)) {
        return;
    }
    cache->gain[modeIndex][cellIndex] = gain;
    cache->valid[modeIndex][cellIndex] = 1u;
    cache->overrangeStreak[modeIndex][cellIndex] = 0u;
}

void sensorarrayAdsGainCacheNoteOverrange(sensorarrayAdsGainCache_t *cache,
                                         sensorarrayMeasurementMode_t mode,
                                         uint8_t cellIndex)
{
    int modeIndex = sensorarrayAdsGainCacheModeIndex(mode);
    if (!cache || modeIndex < 0 || cellIndex >= SENSORARRAY_MEASUREMENT_MAX_CELLS) {
        return;
    }
    uint8_t *streak = &cache->overrangeStreak[modeIndex][cellIndex];
    if (*streak < UINT8_MAX) {
        (*streak)++;
    }
    if (*streak >= 2u) {
        cache->valid[modeIndex][cellIndex] = 0u;
    }
}
