#include "sensorarrayFdcRescue.h"

#include <stdio.h>

#include "sensorarrayFdcSweep.h"

void sensorarrayFdcRescueReset(sensorarrayFdcRescueContext_t *ctx)
{
    if (ctx) {
        *ctx = (sensorarrayFdcRescueContext_t){0};
    }
}

static uint8_t sensorarrayFdcRescueFirstFailedRow(const sensorarrayFrame_t *frame)
{
    if (!frame) {
        return 0u;
    }
    if (frame->firstBadRow != 0u) {
        return frame->firstBadRow;
    }
    for (uint8_t row = 1u; row <= SENSORARRAY_MATRIX_ROWS; ++row) {
        uint64_t rowMask = 0xFFull << ((row - 1u) * SENSORARRAY_MATRIX_COLS);
        if ((frame->capValidMask & rowMask) != rowMask) {
            return row;
        }
    }
    return 0u;
}

esp_err_t sensorarrayFdcRescueTick(sensorarrayFdcMatrixEngine_t *engine,
                                   const sensorarrayFrame_t *frame,
                                   sensorarrayFdcRescueContext_t *ctx)
{
    if (!engine || !engine->state || !frame || !ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->capValidMask != 0u) {
        ctx->allInvalidSequence = 0u;
        return ESP_OK;
    }

    ctx->allInvalidSequence++;
    bool statusReadyRejectedFault =
        frame->diagReadyButRejectedCount != 0u ||
        frame->actualDataReadSkippedDespiteStatusReadyCount != 0u ||
        frame->intbMissButStatusReadyCount != 0u;
    bool waitBudgetFault = frame->waitBudgetTooShortCount != 0u;
    bool runtimeReadyFault =
        frame->notReadyCount >= SENSORARRAY_MATRIX_CELL_COUNT ||
        frame->zeroBeforeReadyCount != 0u ||
        frame->unreadWithoutDrdyCount != 0u;
    bool zeroAfterDrdyFault = frame->zeroAfterDrdyCount != 0u;
    const char *reason = statusReadyRejectedFault ? "all_invalid_due_to_status_ready_rejected" :
        waitBudgetFault ? "all_invalid_due_to_wait_budget_too_short" :
        runtimeReadyFault ? "all_invalid_due_to_not_ready" :
        zeroAfterDrdyFault ? "all_invalid_due_to_zero_after_drdy" :
        (frame->i2cErrorCount != 0u ? "all_invalid_due_to_i2c" :
         (frame->freshCount == 0u ? "no_unread_after_poll" :
          "all_invalid_after_read"));
    const char *action = "none";
    const char *suppressedReason = "none";
    bool queuedFullSweep = false;
    bool fullSweepSuppressed = false;
    esp_err_t err = ESP_OK;

    if (statusReadyRejectedFault || waitBudgetFault) {
        action = "status_fallback_degraded";
        fullSweepSuppressed = true;
        suppressedReason = statusReadyRejectedFault ?
            "status_ready_rejected_not_a_sweep_fault" :
            "wait_budget_too_short_not_a_sweep_fault";
    } else if (runtimeReadyFault) {
        if (ctx->allInvalidSequence == 1u) {
            action = "force_exit_sleep_restore_autoscan";
            esp_err_t primaryErr = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                                      SENSORARRAY_FDC_DEV_PRIMARY,
                                                                      "runtime_ready_epoch_fault_1");
            esp_err_t secondaryErr = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                                        SENSORARRAY_FDC_DEV_SECONDARY,
                                                                        "runtime_ready_epoch_fault_1");
            err = (primaryErr != ESP_OK) ? primaryErr : secondaryErr;
            fullSweepSuppressed = true;
            suppressedReason = "first_runtime_ready_fault_restore";
        } else if (ctx->allInvalidSequence == 2u) {
            action = "soft_resync_force_cache";
            engine->state->fdcAppliedRow[SENSORARRAY_FDC_DEV_PRIMARY].dirty = true;
            engine->state->fdcAppliedRow[SENSORARRAY_FDC_DEV_SECONDARY].dirty = true;
            err = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                     SENSORARRAY_FDC_DEV_PRIMARY,
                                                     "runtime_ready_epoch_fault_repeat");
            if (err == ESP_OK) {
                err = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                         SENSORARRAY_FDC_DEV_SECONDARY,
                                                         "runtime_ready_epoch_fault_repeat");
            }
            fullSweepSuppressed = true;
            suppressedReason = "second_runtime_ready_fault_soft_resync";
        } else {
            action = "enqueue_full_sweep";
            err = sensorarrayFdcSweepRequestForceFullSweepAll();
            queuedFullSweep = err == ESP_OK;
        }
    } else if (ctx->allInvalidSequence == 1u) {
        action = "force_exit_sleep_restore_autoscan";
        esp_err_t primaryErr = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                                  SENSORARRAY_FDC_DEV_PRIMARY,
                                                                  "all_invalid_sequence_1");
        esp_err_t secondaryErr = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                                    SENSORARRAY_FDC_DEV_SECONDARY,
                                                                    "all_invalid_sequence_1");
        err = (primaryErr != ESP_OK) ? primaryErr : secondaryErr;
        fullSweepSuppressed = true;
        suppressedReason = "first_all_invalid_restore";
    } else if (ctx->allInvalidSequence == 2u) {
        action = "soft_resync_force_cache";
        engine->state->fdcAppliedRow[SENSORARRAY_FDC_DEV_PRIMARY].dirty = true;
        engine->state->fdcAppliedRow[SENSORARRAY_FDC_DEV_SECONDARY].dirty = true;
        err = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                 SENSORARRAY_FDC_DEV_PRIMARY,
                                                 "all_invalid_sequence_2");
        if (err == ESP_OK) {
            err = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                     SENSORARRAY_FDC_DEV_SECONDARY,
                                                     "all_invalid_sequence_2");
        }
        fullSweepSuppressed = true;
        suppressedReason = "second_all_invalid_soft_resync";
    } else {
        action = "enqueue_full_sweep";
        err = sensorarrayFdcSweepRequestForceFullSweepAll();
        queuedFullSweep = err == ESP_OK;
    }

    printf("FDC_RESCUE,stage=tick,allInvalidSequence=%lu,freshCount=%u,capValidCount=%u,notReadyCount=%u,zeroBeforeReadyCount=%u,zeroAfterDrdyCount=%u,i2cErrorCount=%u,unreadWithoutDrdyCount=%u,diagReadyButRejectedCount=%u,intbMissButStatusReadyCount=%u,statusFallbackAcceptedCount=%u,waitBudgetTooShortCount=%u,levelLowButEdgeMissCount=%u,actualDataReadSkippedDespiteStatusReadyCount=%u,firstFailedRow=%u,firstFailedDevice=%u,reason=%s,action=%s,result=%s,fullSweepSuppressed=%u,suppressedReason=%s,queuedFullSweep=%u,err=0x%lx\n",
           (unsigned long)ctx->allInvalidSequence,
           (unsigned)frame->freshCount,
           (unsigned)frame->validCount,
           (unsigned)frame->notReadyCount,
           (unsigned)frame->zeroBeforeReadyCount,
           (unsigned)frame->zeroAfterDrdyCount,
           (unsigned)frame->i2cErrorCount,
           (unsigned)frame->unreadWithoutDrdyCount,
           (unsigned)frame->diagReadyButRejectedCount,
           (unsigned)frame->intbMissButStatusReadyCount,
           (unsigned)frame->statusFallbackAcceptedCount,
           (unsigned)frame->waitBudgetTooShortCount,
           (unsigned)frame->levelLowButEdgeMissCount,
           (unsigned)frame->actualDataReadSkippedDespiteStatusReadyCount,
           (unsigned)sensorarrayFdcRescueFirstFailedRow(frame),
           (unsigned)frame->firstBadDevice,
           reason,
           action,
           (err == ESP_OK) ? "ok" : "failed",
           fullSweepSuppressed ? 1u : 0u,
           suppressedReason,
           queuedFullSweep ? 1u : 0u,
           (unsigned long)err);

    return err;
}
