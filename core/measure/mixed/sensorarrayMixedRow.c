#include "sensorarrayMixedRow.h"

#include <string.h>

#include "esp_timer.h"

#include "sensorarrayAdsGap.h"
#include "sensorarrayFrameBuilder.h"

static uint64_t sensorarrayMixedExpectedMask(uint8_t activeRows)
{
    uint32_t cells = (uint32_t)activeRows * SENSORARRAY_MATRIX_COLS;
    return cells >= SENSORARRAY_MATRIX_CELL_COUNT ? UINT64_MAX :
        ((UINT64_C(1) << cells) - 1u);
}

static void sensorarrayMixedMergeSegment(sensorarrayFrame_t *target,
                                         const sensorarrayFrame_t *segment,
                                         uint8_t rowMask,
                                         sensorarrayMeasurementMode_t mode)
{
    if (!target || !segment) {
        return;
    }
    bool capacitance = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
    for (uint8_t row = 1u; row <= target->activeRows; ++row) {
        uint8_t rowBit = (uint8_t)(1u << (row - 1u));
        if ((rowMask & rowBit) == 0u) {
            continue;
        }
        for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
            size_t index = sensorarrayMatrixIndex(row, dLine);
            uint64_t bit = UINT64_C(1) << index;
            bool acquired = capacitance ?
                (segment->freshMask & bit) != 0u :
                (segment->measurement.freshMask & bit) != 0u;
            if (!acquired) {
                /* A failed or partial child segment must not fold stale or
                 * uninitialised payload into the merged frame. Keep the
                 * target's invalid sentinels and record the acquisition
                 * failure without claiming electrical validity. */
                if (capacitance) {
                    target->validMask &= ~bit;
                    target->capValidMask &= ~bit;
                    target->freshMask &= ~bit;
                    target->warnMask &= ~bit;
                    target->errorMask |= bit;
                } else {
                    target->measurement.validMask &= ~bit;
                    target->measurement.freshMask &= ~bit;
                    target->measurement.errorMask |= bit;
                    target->measurement.openMask =
                        (target->measurement.openMask & ~bit) |
                        (segment->measurement.openMask & bit);
                    target->measurement.shortMask =
                        (target->measurement.shortMask & ~bit) |
                        (segment->measurement.shortMask & bit);
                    target->measurement.unstableMask =
                        (target->measurement.unstableMask & ~bit) |
                        (segment->measurement.unstableMask & bit);
                    target->measurement.saturatedMask =
                        (target->measurement.saturatedMask & ~bit) |
                        (segment->measurement.saturatedMask & bit);
                    if ((segment->measurement.errorMask & bit) != 0u &&
                        segment->measurement.errorReason[index] !=
                            SENSORARRAY_CELL_ERROR_UNSUPPORTED) {
                        target->measurement.errorReason[index] =
                            segment->measurement.errorReason[index];
                    } else {
                        target->measurement.errorReason[index] =
                            SENSORARRAY_CELL_ERROR_STALE;
                    }
                    target->validMask &= ~bit;
                    target->freshMask &= ~bit;
                    target->errorMask |= bit;
                }
                target->acquiredMask &= ~bit;
                continue;
            }
            if (capacitance) {
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
            if (capacitance) {
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
                /* Mirror the payload masks so frame-wide consumers that read
                 * the top-level masks stay coherent with the VOLT/RES rows. */
                target->validMask = (target->validMask & ~bit) |
                    (segment->validMask & bit);
                target->freshMask = (target->freshMask & ~bit) |
                    (segment->freshMask & bit);
                target->errorMask = (target->errorMask & ~bit) |
                    (segment->errorMask & bit);
            }
            target->acquiredMask |= bit;
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

static void sensorarrayMixedMergeSegmentTiming(sensorarrayFrame_t *target,
                                               const sensorarrayFrame_t *segment,
                                               sensorarrayMeasurementMode_t mode)
{
    if (!target || !segment) {
        return;
    }
    switch (mode) {
    case SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE:
        target->capStartUs = segment->capStartUs;
        target->capEndUs = segment->capEndUs;
        break;
    case SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:
        target->voltStartUs = segment->voltStartUs;
        target->voltEndUs = segment->voltEndUs;
        break;
    case SENSORARRAY_MEASUREMENT_MODE_RESISTANCE:
        target->resStartUs = segment->resStartUs;
        target->resEndUs = segment->resEndUs;
        break;
    default:
        break;
    }
}

static uint32_t sensorarrayMixedCountCells(uint64_t mask, uint8_t activeRows)
{
    mask &= sensorarrayMixedExpectedMask(activeRows);
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
                                             sensorarrayFrame_t *frame,
                                             sensorarrayFrame_t *segmentWorkspace)
{
    if (!fdcEngine || !adsEngine || !plan || !frame || !segmentWorkspace ||
        plan->rowCount < 1u || plan->rowCount > SENSORARRAY_MATRIX_ROWS) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFrameBuilderInitInvalid(frame);
    frame->configSnapshot = plan->configSnapshot;
    frame->activeRows = plan->rowCount;
    frame->expectedMask = sensorarrayMixedExpectedMask(frame->activeRows);
    frame->mixedProfile = true;
    frame->rowProfileGeneration = plan->rowProfileGeneration;
    frame->rowProfileRequestId = plan->rowProfileRequestId;
    frame->timestampUs = (uint64_t)esp_timer_get_time();
    frame->frameStartUs = frame->timestampUs;
    frame->sequence = 0u;
    /* Keep the planned per-row output contract even when a mode group fails
     * to acquire: an active physical row must stay C/V/R so the formatter
     * emits it with explicit STALE/invalid cells instead of dropping the row
     * as N while the profile still planned that mode. */
    for (uint8_t row = 0u; row < plan->rowCount; ++row) {
        sensorarrayMeasurementMode_t rowMode = plan->rowModes[row];
        frame->rowMode[row] = rowMode;
        frame->rowUnit[row] = rowMode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
            SENSORARRAY_MEASUREMENT_UNIT_PF :
            (rowMode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ?
                SENSORARRAY_MEASUREMENT_UNIT_VOLT : SENSORARRAY_MEASUREMENT_UNIT_OHM);
        frame->rowScale[row] = rowMode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ? -3 : -6;
    }

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
        /* The caller owns one reusable workspace for every mode group.  The
         * complete frame must never be a scan-task stack automatic: a mixed
         * profile would otherwise push the acquisition stack past its canary.
         * Reinitialise it fully before each group so a failed or partial
         * child segment can never fold stale data from a previous group. */
        sensorarrayFrameBuilderInitInvalid(segmentWorkspace);
        sensorarrayScanPlan_t segmentPlan = *plan;
        /* The engines keep the full matrix index space but only visit the
         * physical rows assigned to this mode. This prevents a CAP/VOLT/RES
         * route from sampling rows owned by another profile. */
        segmentPlan.configSnapshot.rowMask = rowMask;
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
            /* Merge the still-invalid segment so every cell of this planned
             * group carries an explicit unacquired error instead of relying
             * on the invalid-frame sentinel masks. */
            sensorarrayMixedMergeSegment(frame, segmentWorkspace, rowMask, mode);
            continue;
        }

        esp_err_t readErr = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
            sensorarrayFdcMatrixEngineReadFrame(fdcEngine, &segmentPlan,
                                                segmentWorkspace) :
            sensorarrayAdsMatrixEngineReadFrame(adsEngine, &segmentPlan,
                                                segmentWorkspace);
        if (readErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = readErr;
        }
        sensorarrayMixedMergeSegment(frame, segmentWorkspace, rowMask, mode);
        if (readErr == ESP_OK) {
            sensorarrayMixedMergeSegmentTiming(frame, segmentWorkspace, mode);
        }
    }
    frame->validCount = (uint8_t)sensorarrayMixedCountCells(
        frame->capValidMask | frame->measurement.validMask, frame->activeRows);
    frame->freshCount = (uint8_t)sensorarrayMixedCountCells(
        frame->freshMask | frame->measurement.freshMask, frame->activeRows);
    /* Freshness is acquisition completeness. A cell can be acquired but
     * electrically invalid (OPEN/SHORT/RANGE/SATURATED), which is reported
     * through validMask/errorMask and must not make the sweep stale.
     * Group/route failures remain distinct in the returned firstErr. */
    frame->freshFrame = sensorarrayFrameBuilderAcquisitionComplete(
        frame->acquiredMask, frame->expectedMask);
    frame->stale = !frame->freshFrame;
    frame->measurement.mode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    frame->frameEndUs = (uint64_t)esp_timer_get_time();
    frame->physicalSweepUs = frame->frameEndUs - frame->frameStartUs;
    frame->maxSkewUs = sensorarrayFrameBuilderMaxGroupSkewUs(
        frame->capStartUs, frame->capEndUs,
        frame->voltStartUs, frame->voltEndUs,
        frame->resStartUs, frame->resEndUs);
    return firstErr;
}
