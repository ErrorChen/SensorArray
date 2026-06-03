#include "sensorarrayFdcReady.h"

#include "sensorarrayConfig.h"
#include "sensorarrayMeasure.h"

static uint32_t sensorarrayFdcReadyCeilDivU64(uint64_t numerator, uint64_t denominator)
{
    if (denominator == 0u) {
        return 0u;
    }
    return (uint32_t)((numerator + denominator - 1u) / denominator);
}

static uint32_t sensorarrayFdcReadyClamp(uint32_t value, uint32_t minValue, uint32_t maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

uint32_t sensorarrayFdcEstimateAutoscanReadyTimeoutUs(const sensorarrayFdcRowCache_t *rowCache,
                                                      uint8_t requiredUnreadMask,
                                                      uint32_t *outEstimatedRoundUs)
{
    uint64_t estimatedRoundUs = 0u;
    uint32_t fclkHz = (rowCache && rowCache->effectiveFclkHz != 0u) ?
        rowCache->effectiveFclkHz :
        sensorarrayMeasureFdcEffectiveFclkHz();

    requiredUnreadMask &= 0x0Fu;
    if (requiredUnreadMask == 0u) {
        requiredUnreadMask = 0x0Fu;
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if ((requiredUnreadMask & (uint8_t)(1u << ch)) == 0u) {
            continue;
        }
        uint16_t rCount = (rowCache && rowCache->rCount[ch] != 0u) ?
            rowCache->rCount[ch] :
            SENSORARRAY_FDC_RCOUNT;
        uint16_t settleCount = (rowCache && rowCache->settleCount[ch] != 0u) ?
            rowCache->settleCount[ch] :
            SENSORARRAY_FDC_SETTLECOUNT;
        uint16_t clockDiv = (rowCache && rowCache->clockDiv[ch] != 0u) ?
            rowCache->clockDiv[ch] :
            SENSORARRAY_FDC_CLOCK_DIVIDERS;
        double frefDivider = sensorarrayMeasureFdcFrefDividerFromClockDiv(clockDiv);
        uint32_t frefHz = (frefDivider > 0.0) ? (uint32_t)((double)fclkHz / frefDivider) : fclkHz;
        if (frefHz == 0u) {
            frefHz = SENSORARRAY_FDC_REF_CLOCK_HZ;
        }

        uint32_t settleUs = sensorarrayFdcReadyCeilDivU64((uint64_t)settleCount * 16u * 1000000u,
                                                          frefHz);
        uint32_t convUs = sensorarrayFdcReadyCeilDivU64((uint64_t)rCount * 16u * 1000000u,
                                                        frefHz);
        uint32_t switchUs = 2u;
        estimatedRoundUs += (uint64_t)settleUs + convUs + switchUs;
    }

    if (estimatedRoundUs > UINT32_MAX) {
        estimatedRoundUs = UINT32_MAX;
    }
    if (outEstimatedRoundUs) {
        *outEstimatedRoundUs = (uint32_t)estimatedRoundUs;
    }
    uint64_t timeoutUs = (estimatedRoundUs * 2u) + 3000u;
    if (timeoutUs > UINT32_MAX) {
        timeoutUs = UINT32_MAX;
    }
    return sensorarrayFdcReadyClamp((uint32_t)timeoutUs, 25000u, 80000u);
}

