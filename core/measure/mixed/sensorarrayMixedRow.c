#include "sensorarrayMixedRow.h"

#include <string.h>

#include "esp_timer.h"

#include "sensorarrayAdsGap.h"
#include "sensorarrayFrameBuilder.h"

static void sensorarrayMixedMergeSegment(sensorarrayFrame_t *target,
                                         const sensorarrayFrame_t *segment,
                                         uint8_t rowMask,
                                         sensorarrayMeasurementMode_t mode)
{
    if (!target || !segment) {
        return;
    }
    for (uint8_t row = 1u; row <= target->activeRows; ++row) {
        uint8_t rowBit = (uint8_t)(1u << (row - 1u));
        if ((rowMask & rowBit) == 0u) {
            continue;
        }
        for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
            size_t index = sensorarrayMatrixIndex(row, dLine);
            if (mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
                target->freqHz[index] = segment->freqHz[index];
                target->capTotalPf[index] = segment->capTotalPf[index];
                target->raw28[index] = segment->raw28[index];
                target->clockDividers[index] = segment->clockDividers[index];
                target->driveCurrent[index] = segment->driveCurrent[index];
                target->deglitchCode[index] = segment->deglitchCode[index];
                target->effectiveFclkHz[index] = segment->effectiveFclkHz[index];
            } else {
                target->measurement.valuesFixed[index] =
                    segment->measurement.valuesFixed[index];
                target->measurement.rawCode[index] =
                    segment->measurement.rawCode[index];
                target->measurement.nodeUv[index] =
                    segment->measurement.nodeUv[index];
                target->measurement.pgaGain[index] =
                    segment->measurement.pgaGain[index];
                target->measurement.errorReason[index] =
                    segment->measurement.errorReason[index];
            }
            uint64_t bit = UINT64_C(1) << index;
            if (mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
                target->capValidMask = (target->capValidMask & ~bit) |
                    (segment->capValidMask & bit);
                target->validMask = (target->validMask & ~bit) |
                    (segment->validMask & bit);
                target->freshMask = (target->freshMask & ~bit) |
                    (segment->freshMask & bit);
                target->warnMask = (target->warnMask & ~bit) |
                    (segment->warnMask & bit);
                target->errorMask = (target->errorMask & ~bit) |
                    (segment->errorMask & bit);
            } else {
                target->measurement.validMask = (target->measurement.validMask & ~bit) |
                    (segment->measurement.validMask & bit);
                target->measurement.freshMask = (target->measurement.freshMask & ~bit) |
                    (segment->measurement.freshMask & bit);
                target->measurement.errorMask = (target->measurement.errorMask & ~bit) |
                    (segment->measurement.errorMask & bit);
                target->measurement.openMask = (target->measurement.openMask & ~bit) |
                    (segment->measurement.openMask & bit);
                target->measurement.shortMask = (target->measurement.shortMask & ~bit) |
                    (segment->measurement.shortMask & bit);
                target->measurement.unstableMask = (target->measurement.unstableMask & ~bit) |
                    (segment->measurement.unstableMask & bit);
                target->measurement.saturatedMask = (target->measurement.saturatedMask & ~bit) |
                    (segment->measurement.saturatedMask & bit);
            }
        }
        if ((segment->rowFreshMask & rowBit) != 0u) {
            target->rowFreshMask |= rowBit;
        }
        if ((segment->primaryFreshMask & rowBit) != 0u) {
            target->primaryFreshMask |= rowBit;
        }
        if ((segment->secondaryFreshMask & rowBit) != 0u) {
            target->secondaryFreshMask |= rowBit;
        }
        target->rowEpoch[row - 1u] = segment->rowEpoch[row - 1u];
        target->primaryEpoch[row - 1u] = segment->primaryEpoch[row - 1u];
        target->secondaryEpoch[row - 1u] = segment->secondaryEpoch[row - 1u];
        target->rowRouteSetUs[row - 1u] = segment->rowRouteSetUs[row - 1u];
        target->rowReadyUs[row - 1u] = segment->rowReadyUs[row - 1u];
        target->rowReadDoneUs[row - 1u] = segment->rowReadDoneUs[row - 1u];
        target->rowMergeDoneUs[row - 1u] = segment->rowMergeDoneUs[row - 1u];
    }
    if (segment->frameEndUs > target->frameEndUs) {
        target->frameEndUs = segment->frameEndUs;
    }
    if (segment->physicalSweepUs > target->physicalSweepUs) {
        target->physicalSweepUs = segment->physicalSweepUs;
    }
    target->telemetry.routeUs += segment->telemetry.routeUs;
    target->telemetry.settleUs += segment->telemetry.settleUs;
    target->measurement.frameDurationUs += segment->measurement.frameDurationUs;
    target->measurement.rowRouteUs += segment->measurement.rowRouteUs;
}

static uint32_t sensorarrayMixedCountCells(uint64_t mask, uint8_t activeRows)
{
    uint64_t activeMask = (activeRows * SENSORARRAY_MATRIX_COLS) == 64u ?
        UINT64_MAX : ((UINT64_C(1) << (activeRows * SENSORARRAY_MATRIX_COLS)) - 1u);
    mask &= activeMask;
    uint32_t count = 0u;
    while (mask != 0u) {
        mask &= mask - 1u;
        count++;
    }
    return count;
}

esp_err_t sensorarrayMixedRowEngineReadFrame(sensorarrayFdcMatrixEngine_t *fdcEngine,
                                             sensorarrayAdsMatrixEngine_t *adsEngine,
                                             const sensorarrayScanPlan_t *plan,
                                             sensorarrayFrame_t *frame)
{
    if (!fdcEngine || !adsEngine || !plan || !frame || plan->rowCount < 1u ||
        plan->rowCount > SENSORARRAY_MATRIX_ROWS) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFrameBuilderInitInvalid(frame);
    frame->configSnapshot = plan->configSnapshot;
    frame->activeRows = plan->rowCount;
    frame->mixedProfile = true;
    frame->rowProfileGeneration = plan->rowProfileGeneration;
    frame->rowProfileRequestId = plan->rowProfileRequestId;
    frame->timestampUs = (uint64_t)esp_timer_get_time();
    frame->frameStartUs = frame->timestampUs;
    frame->sequence = 0u;

    esp_err_t firstErr = ESP_OK;
    const sensorarrayMeasurementMode_t modes[] = {
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
    };
    for (size_t group = 0u; group < sizeof(modes) / sizeof(modes[0]); ++group) {
        sensorarrayMeasurementMode_t mode = modes[group];
        uint8_t rowMask = 0u;
        for (uint8_t row = 0u; row < plan->rowCount; ++row) {
            if (plan->rowModes[row] == mode) {
                rowMask |= (uint8_t)(1u << row);
            }
        }
        if (rowMask == 0u) {
            continue;
        }

        sensorarrayAdsRailSplit_t rail = {0};
        const sensorarrayAdsRailSplit_t *railPtr = NULL;
        if (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
            (void)sensorarrayAdsGapCopyRailSplit(
                adsEngine->frameSequenceHint,
                (uint32_t)CONFIG_SENSORARRAY_ADS_MATRIX_RAIL_MAX_AGE_FRAMES,
                &rail);
            railPtr = rail.valid ? &rail : NULL;
        }
        uint64_t transitionUs = 0u;
        esp_err_t err = sensorarrayRouteControllerApplyMode(
            adsEngine->routeController, mode, railPtr, &transitionUs);
        if (err == ESP_OK) {
            err = sensorarrayAdsMatrixEngineSetMode(
                adsEngine, mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
                    SENSORARRAY_MEASUREMENT_MODE_NONE : mode);
        }
        if (err != ESP_OK) {
            if (firstErr == ESP_OK) {
                firstErr = err;
            }
            continue;
        }

        sensorarrayFrame_t segment;
        sensorarrayScanPlan_t segmentPlan = *plan;
        /* The engines keep the full matrix index space but only visit the
         * physical rows assigned to this mode. This prevents a CAP/VOLT/RES
         * route from sampling rows owned by another profile. */
        segmentPlan.configSnapshot.rowMask = rowMask;
        esp_err_t readErr = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
            sensorarrayFdcMatrixEngineReadFrame(fdcEngine, &segmentPlan, &segment) :
            sensorarrayAdsMatrixEngineReadFrame(adsEngine, &segmentPlan, &segment);
        if (readErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = readErr;
        }
        sensorarrayMixedMergeSegment(frame, &segment, rowMask, mode);
        for (uint8_t row = 0u; row < plan->rowCount; ++row) {
            if ((rowMask & (uint8_t)(1u << row)) == 0u) {
                continue;
            }
            frame->rowMode[row] = mode;
            frame->rowUnit[row] = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
                SENSORARRAY_MEASUREMENT_UNIT_PF :
                (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ?
                    SENSORARRAY_MEASUREMENT_UNIT_VOLT : SENSORARRAY_MEASUREMENT_UNIT_OHM);
            frame->rowScale[row] = mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ? -3 : -6;
        }
    }
    frame->validCount = (uint8_t)sensorarrayMixedCountCells(
        frame->capValidMask | frame->measurement.validMask, frame->activeRows);
    frame->freshCount = (uint8_t)sensorarrayMixedCountCells(
        frame->freshMask | frame->measurement.freshMask, frame->activeRows);
    uint32_t cells = (uint32_t)frame->activeRows * SENSORARRAY_MATRIX_COLS;
    uint64_t expectedMask = cells == 64u ? UINT64_MAX : ((UINT64_C(1) << cells) - 1u);
    uint64_t freshMask = frame->freshMask | frame->measurement.freshMask;
    uint64_t validMask = frame->capValidMask | frame->measurement.validMask;
    frame->freshFrame = freshMask == expectedMask;
    frame->stale = !frame->freshFrame || validMask != expectedMask;
    frame->measurement.mode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    frame->frameEndUs = (uint64_t)esp_timer_get_time();
    frame->physicalSweepUs = frame->frameEndUs - frame->frameStartUs;
    return firstErr;
}
