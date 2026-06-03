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
    const char *reason = frame->freshCount == 0u ?
        "no_unread_after_poll" :
        "all_invalid_after_read";
    const char *action = "none";
    esp_err_t err = ESP_OK;

    if (ctx->allInvalidSequence == 1u) {
        action = "force_exit_sleep_restore_autoscan";
        esp_err_t primaryErr = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                                  SENSORARRAY_FDC_DEV_PRIMARY,
                                                                  "all_invalid_sequence_1");
        esp_err_t secondaryErr = sensorarrayFdcSweepRestoreAutoscan(engine->state,
                                                                    SENSORARRAY_FDC_DEV_SECONDARY,
                                                                    "all_invalid_sequence_1");
        err = (primaryErr != ESP_OK) ? primaryErr : secondaryErr;
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
    } else {
        action = "enqueue_full_sweep";
        err = sensorarrayFdcSweepRequestForceFullSweepAll();
    }

    printf("FDC_RESCUE,stage=tick,allInvalidSequence=%lu,freshCount=%u,capValidCount=%u,firstFailedRow=%u,firstFailedDevice=%u,reason=%s,action=%s,result=%s,err=0x%lx\n",
           (unsigned long)ctx->allInvalidSequence,
           (unsigned)frame->freshCount,
           (unsigned)frame->validCount,
           (unsigned)sensorarrayFdcRescueFirstFailedRow(frame),
           (unsigned)frame->firstBadDevice,
           reason,
           action,
           (err == ESP_OK) ? "ok" : "failed",
           (unsigned long)err);

    return err;
}

