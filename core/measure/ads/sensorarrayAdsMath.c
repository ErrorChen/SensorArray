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
    /* Signed boundary: a denominator at or below the open limit, including a
     * negative value (node below AVSS), is an open/high-Z cell. */
    if (result.denominatorUv <= (int64_t)config->openDenominatorUv) {
        result.error = SENSORARRAY_CELL_ERROR_OPEN;
        result.open = true;
        result.openSemantic = SENSORARRAY_ADS_OPEN_SEMANTIC_RAW;
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

bool sensorarrayAdsMathConfigurationGenerationCurrent(
    uint32_t sampleConfigurationGeneration,
    uint32_t currentConfigurationGeneration)
{
    return currentConfigurationGeneration != 0u &&
           sampleConfigurationGeneration == currentConfigurationGeneration;
}

bool sensorarrayAdsMathHighZOpenCandidate(
    const sensorarrayAdsHighZCandidateInput_t *input)
{
    if (!input) {
        return false;
    }
    int64_t denominatorUv = (int64_t)input->nodeUv - (int64_t)input->avssUv;
    int64_t nearOpenLimitUv = (int64_t)input->openDenominatorUv +
                              (int64_t)input->openConfirmMarginUv;
    bool nearOpenBoundary = denominatorUv <= nearOpenLimitUv;
    bool rawSentinel = input->rawCode == INT32_MIN;
    bool saturationBoundary = input->magnitude >= input->saturationLimit;
    return nearOpenBoundary || rawSentinel || saturationBoundary;
}

sensorarrayAdsOpenConfirm_t sensorarrayAdsMathConfirmOpenSet(
    const int32_t *nodeUvSamples,
    size_t sampleCount,
    int32_t avssUv,
    const sensorarrayAdsResistanceConfig_t *config,
    uint32_t maximumSpreadUv,
    int32_t *outMedianNodeUv)
{
    if (outMedianNodeUv) {
        *outMedianNodeUv = 0;
    }
    if (!nodeUvSamples || !config || sampleCount == 0u || sampleCount > 9u) {
        return SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE;
    }

    int64_t openLimitUv = (int64_t)config->openDenominatorUv;
    int64_t nearOpenLimitUv = openLimitUv + (int64_t)config->openConfirmMarginUv;
    for (size_t index = 0u; index < sampleCount; ++index) {
        int64_t denominatorUv = (int64_t)nodeUvSamples[index] - avssUv;
        if (denominatorUv <= openLimitUv) {
            return SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z;
        }
    }

    int32_t medianNodeUv = 0;
    bool stable = sensorarrayAdsMathSamplesStable(nodeUvSamples,
                                                   sampleCount,
                                                   maximumSpreadUv,
                                                   &medianNodeUv);
    int64_t medianDenominatorUv = (int64_t)medianNodeUv - avssUv;
    if (outMedianNodeUv) {
        *outMedianNodeUv = medianNodeUv;
    }
    if (medianDenominatorUv <= nearOpenLimitUv) {
        return SENSORARRAY_ADS_OPEN_CONFIRM_HIGH_Z;
    }
    return stable ? SENSORARRAY_ADS_OPEN_CONFIRM_NONE :
                    SENSORARRAY_ADS_OPEN_CONFIRM_UNSTABLE;
}

uint8_t sensorarrayAdsMathCombineStatusBytes(
    uint8_t primaryStatus,
    const uint8_t *additionalStatus,
    size_t additionalCount)
{
    uint8_t combined = primaryStatus;
    if (additionalStatus) {
        for (size_t index = 0u; index < additionalCount; ++index) {
            combined |= additionalStatus[index];
        }
    }
    return combined;
}

uint64_t sensorarrayAdsMathHighZLatchUpdate(
    uint64_t latchMask,
    size_t cellIndex,
    sensorarrayAdsOpenSemantic_t openSemantic)
{
    if (cellIndex >= 64u) {
        return latchMask;
    }
    uint64_t bit = UINT64_C(1) << cellIndex;
    if (openSemantic == SENSORARRAY_ADS_OPEN_SEMANTIC_HIGH_Z_CONFIRMED) {
        return latchMask | bit;
    }
    if (openSemantic == SENSORARRAY_ADS_OPEN_SEMANTIC_NONE) {
        return latchMask & ~bit;
    }
    return latchMask;
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
    if (!health->generationAdvanced || !health->statusNewData ||
        (health->expectedConfigurationGeneration != 0u &&
         health->configurationGeneration !=
             health->expectedConfigurationGeneration)) {
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

sensorarrayAdsBatteryMathResult_t sensorarrayAdsMathBatteryVoltage(
    const sensorarrayAdsBatteryMathInput_t *input)
{
    sensorarrayAdsBatteryMathResult_t result = {
        .batteryMv = -1,
        .error = SENSORARRAY_ADS_BATTERY_MATH_REFERENCE,
    };
    if (!input) {
        return result;
    }
    if (!input->restoreOk) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_RESTORE_FAILED;
        return result;
    }
    if (input->adcTimedOut) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_ADC_TIMEOUT;
        return result;
    }
    /* VBIAS and the rail split are transaction admission prerequisites, not
     * conversion results. Check them before generic transport/freshness flags
     * so a deliberately skipped conversion reports rail_invalid or
     * vbias_invalid instead of manufacturing a misleading spi_error. */
    if (!input->vbiasConfirmed) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_VBIAS;
        return result;
    }
    if (!input->railValid || !input->railFresh) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_RAIL;
        return result;
    }
    if (!input->adcTransportOk) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_SPI;
        return result;
    }
    if (!input->adcFresh) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_ADC_STALE;
        return result;
    }
    if (input->adcSaturated) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_SATURATED;
        return result;
    }
    if (input->adcUnstable) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_UNSTABLE;
        return result;
    }
    if (!input->adcStatusOk) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_ADC_STATUS;
        return result;
    }
    if (!input->referenceValid) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_REFERENCE;
        return result;
    }
    if (input->dividerNumerator <= 0 || input->dividerDenominator <= 0 ||
        input->calibrationScalePpm <= 0 || input->minimumMv < 0 ||
        input->maximumMv < input->minimumMv) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_DIVIDER;
        return result;
    }

    int64_t ain8GroundUv = (int64_t)input->ain8DifferentialUv +
                           (int64_t)input->aincomGroundUv;
    if (ain8GroundUv < 0) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_NEGATIVE;
        return result;
    }
    if (ain8GroundUv > INT32_MAX) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW;
        return result;
    }
    result.ain8GroundUv = (int32_t)ain8GroundUv;
    result.ain8GroundValid = true;
    if (ain8GroundUv != 0 &&
        (int64_t)input->dividerNumerator > INT64_MAX / ain8GroundUv) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW;
        return result;
    }
    int64_t batteryUv = (ain8GroundUv * input->dividerNumerator) /
                        input->dividerDenominator;
    if (batteryUv != 0 &&
        (int64_t)input->calibrationScalePpm > INT64_MAX / batteryUv) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW;
        return result;
    }
    batteryUv = (batteryUv * input->calibrationScalePpm) / 1000000LL;
    if ((input->calibrationOffsetUv > 0 &&
         batteryUv > INT64_MAX - input->calibrationOffsetUv) ||
        (input->calibrationOffsetUv < 0 &&
         batteryUv < INT64_MIN - input->calibrationOffsetUv)) {
        result.error = SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW;
        return result;
    }
    batteryUv += input->calibrationOffsetUv;
    if (batteryUv < 0 || batteryUv > (int64_t)INT32_MAX * 1000LL) {
        result.error = batteryUv < 0 ? SENSORARRAY_ADS_BATTERY_MATH_NEGATIVE :
                                      SENSORARRAY_ADS_BATTERY_MATH_OVERFLOW;
        return result;
    }
    result.batteryMv = (int32_t)(batteryUv / 1000LL);
    if (result.batteryMv < input->minimumMv) {
        result.batteryMv = -1;
        result.error = SENSORARRAY_ADS_BATTERY_MATH_BELOW_MINIMUM;
        return result;
    }
    if (result.batteryMv > input->maximumMv) {
        result.batteryMv = -1;
        result.error = SENSORARRAY_ADS_BATTERY_MATH_ABOVE_MAXIMUM;
        return result;
    }
    result.valid = true;
    result.error = SENSORARRAY_ADS_BATTERY_MATH_OK;
    return result;
}
