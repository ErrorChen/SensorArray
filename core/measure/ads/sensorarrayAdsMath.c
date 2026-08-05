#include "sensorarrayAdsMath.h"

#include <limits.h>

static int32_t sensorarrayAdsMathMedianInPlace(int32_t *values, size_t count)
{
    for (size_t index = 1u; index < count; ++index) {
        int32_t value = values[index];
        size_t cursor = index;
        while (cursor > 0u && values[cursor - 1u] > value) {
            values[cursor] = values[cursor - 1u];
            cursor--;
        }
        values[cursor] = value;
    }
    if ((count & 1u) != 0u) {
        return values[count / 2u];
    }
    int64_t pair = (int64_t)values[count / 2u - 1u] + values[count / 2u];
    return (int32_t)(pair / 2LL);
}

bool sensorarrayAdsMathSplitRail(const sensorarrayAdsRailInput_t *input,
                                 sensorarrayAdsRailSplit_t *outSplit)
{
    if (!input || !outSplit) {
        return false;
    }
    *outSplit = (sensorarrayAdsRailSplit_t){
        .ageFrames = input->ageFrames,
    };
    int64_t nominalRailUv = (int64_t)input->nominalAvddToGroundUv +
                            (int64_t)input->nominalAvssToGroundUv;
    if (!input->valid || input->measuredRailUv <= 0 || nominalRailUv <= 0 ||
        input->nominalAvddToGroundUv <= 0 || input->nominalAvssToGroundUv <= 0 ||
        input->ageFrames > input->maximumAgeFrames) {
        return false;
    }

    int64_t avddUv = ((int64_t)input->measuredRailUv *
                      input->nominalAvddToGroundUv + nominalRailUv / 2LL) /
                     nominalRailUv;
    int64_t avssMagnitudeUv = (int64_t)input->measuredRailUv - avddUv;
    int64_t avssUv = -avssMagnitudeUv;
    int64_t aincomUv = (avddUv + avssUv) / 2LL;
    if (avddUv > INT32_MAX || avssUv < INT32_MIN ||
        aincomUv < INT32_MIN || aincomUv > INT32_MAX) {
        return false;
    }
    outSplit->avddUv = (int32_t)avddUv;
    outSplit->avssUv = (int32_t)avssUv;
    outSplit->aincomUv = (int32_t)aincomUv;
    outSplit->valid = true;
    return true;
}

bool sensorarrayAdsMathVoltageFromDifferential(int32_t differentialUv,
                                               const sensorarrayAdsRailSplit_t *rail,
                                               const sensorarrayAdsVoltageLimits_t *limits,
                                               int32_t *outNodeUv,
                                               sensorarrayCellError_t *outError)
{
    if (outError) {
        *outError = SENSORARRAY_CELL_ERROR_NONE;
    }
    if (!rail || !rail->valid || !limits || !outNodeUv) {
        if (outError) {
            *outError = SENSORARRAY_CELL_ERROR_RAIL_INVALID;
        }
        return false;
    }
    int64_t nodeUv = (int64_t)differentialUv + rail->aincomUv;
    if (nodeUv < INT32_MIN || nodeUv > INT32_MAX) {
        if (outError) {
            *outError = SENSORARRAY_CELL_ERROR_OVERFLOW;
        }
        return false;
    }
    if (nodeUv < limits->minimumUv || nodeUv > limits->maximumUv) {
        if (outError) {
            *outError = SENSORARRAY_CELL_ERROR_RANGE;
        }
        return false;
    }
    *outNodeUv = (int32_t)nodeUv;
    return true;
}

sensorarrayAdsResistanceResult_t sensorarrayAdsMathResistanceDivider(
    int32_t matrixReferenceUv,
    int32_t nodeUv,
    int32_t avssUv,
    const sensorarrayAdsResistanceConfig_t *config)
{
    sensorarrayAdsResistanceResult_t result = {
        .resistanceMilliohms = SENSORARRAY_MEASUREMENT_INVALID_FIXED,
        .error = SENSORARRAY_CELL_ERROR_REFERENCE_INVALID,
    };
    if (!config || config->referenceResistorOhms == 0u ||
        config->maximumOhms == 0u || config->minimumOhms > config->maximumOhms ||
        matrixReferenceUv <= avssUv) {
        return result;
    }

    result.numeratorUv = (int64_t)matrixReferenceUv - nodeUv;
    result.denominatorUv = (int64_t)nodeUv - avssUv;
    if (result.denominatorUv <= (int64_t)config->openDenominatorUv) {
        result.error = SENSORARRAY_CELL_ERROR_OPEN;
        result.open = true;
        return result;
    }
    if (result.numeratorUv < 0 || result.denominatorUv < 0) {
        result.error = SENSORARRAY_CELL_ERROR_NEGATIVE;
        return result;
    }

    int64_t referenceMilliohms = (int64_t)config->referenceResistorOhms * 1000LL;
    if (result.numeratorUv != 0 &&
        referenceMilliohms > INT64_MAX / result.numeratorUv) {
        result.error = SENSORARRAY_CELL_ERROR_OVERFLOW;
        return result;
    }
    int64_t resistanceMilliohms =
        (referenceMilliohms * result.numeratorUv) / result.denominatorUv;
    if (config->pathOffsetMilliohms > 0 &&
        resistanceMilliohms < config->pathOffsetMilliohms) {
        result.error = SENSORARRAY_CELL_ERROR_NEGATIVE;
        return result;
    }
    resistanceMilliohms -= config->pathOffsetMilliohms;

    int64_t shortLimit = (int64_t)config->shortThresholdOhms * 1000LL;
    int64_t minimum = (int64_t)config->minimumOhms * 1000LL;
    int64_t maximum = (int64_t)config->maximumOhms * 1000LL;
    if (resistanceMilliohms <= shortLimit || resistanceMilliohms < minimum) {
        result.error = SENSORARRAY_CELL_ERROR_SHORT;
        result.shorted = true;
        return result;
    }
    if (resistanceMilliohms > maximum) {
        result.error = SENSORARRAY_CELL_ERROR_RANGE;
        return result;
    }

    result.resistanceMilliohms = resistanceMilliohms;
    result.error = SENSORARRAY_CELL_ERROR_NONE;
    result.valid = true;
    return result;
}

bool sensorarrayAdsMathSamplesStable(const int32_t *samples,
                                     size_t sampleCount,
                                     uint32_t maximumSpreadUv,
                                     int32_t *outMedianUv)
{
    if (!samples || !outMedianUv || sampleCount == 0u || sampleCount > 9u) {
        return false;
    }
    int32_t sorted[9];
    int32_t minimum = samples[0];
    int32_t maximum = samples[0];
    for (size_t index = 0u; index < sampleCount; ++index) {
        sorted[index] = samples[index];
        if (samples[index] < minimum) {
            minimum = samples[index];
        }
        if (samples[index] > maximum) {
            maximum = samples[index];
        }
    }
    *outMedianUv = sensorarrayAdsMathMedianInPlace(sorted, sampleCount);
    int64_t spread = (int64_t)maximum - minimum;
    return spread >= 0 && (uint64_t)spread <= maximumSpreadUv;
}

sensorarrayCellError_t sensorarrayAdsMathClassifySampleHealth(
    const sensorarrayAdsSampleHealth_t *health)
{
    if (!health || !health->transportOk) {
        return SENSORARRAY_CELL_ERROR_SPI;
    }
    if (health->drdyTimedOut) {
        return SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT;
    }
    if (!health->generationAdvanced || !health->statusNewData) {
        return SENSORARRAY_CELL_ERROR_STALE;
    }
    if (health->resetOccurred) {
        return SENSORARRAY_CELL_ERROR_READBACK;
    }
    if (health->referenceAlarm) {
        return SENSORARRAY_CELL_ERROR_REFERENCE_ALARM;
    }
    if (health->pgaLowAlarm || health->pgaHighAlarm) {
        return SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE;
    }
    if (health->pgaDifferentialAlarm) {
        return SENSORARRAY_CELL_ERROR_PGA_DIFFERENTIAL;
    }
    if (health->fullScaleSaturated) {
        return SENSORARRAY_CELL_ERROR_SATURATED;
    }
    if (!health->commonModeSafe) {
        return SENSORARRAY_CELL_ERROR_COMMON_MODE;
    }
    return SENSORARRAY_CELL_ERROR_NONE;
}
