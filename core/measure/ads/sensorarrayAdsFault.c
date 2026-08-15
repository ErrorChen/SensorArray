#include "sensorarrayAdsFault.h"

#include <stdio.h>
#include <string.h>

static const char *const s_stageNames[SENSORARRAY_ADS_FAULT_STAGE_COUNT] = {
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE] = "MATRIX_ROUTE",
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READ] = "MATRIX_READ",
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK] = "MATRIX_READBACK",
    [SENSORARRAY_ADS_FAULT_STAGE_MATRIX_DRDY] = "MATRIX_DRDY",
    [SENSORARRAY_ADS_FAULT_STAGE_BATTERY_GAP] = "BATTERY_GAP",
    [SENSORARRAY_ADS_FAULT_STAGE_BATTERY_RESTORE] = "BATTERY_RESTORE",
    [SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR] = "RAIL_MONITOR",
    [SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION] = "PROFILE_TRANSITION",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_START] = "RECOVERY_START",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_ATTEMPT] = "RECOVERY_ATTEMPT",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_RESUME] = "RECOVERY_RESUME",
    [SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_FAILED] = "RECOVERY_FAILED",
    [SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN] = "UNKNOWN",
};

static const char *const s_outcomeNames[] = {
    [SENSORARRAY_ADS_FAULT_OUTCOME_NONE] = "none",
    [SENSORARRAY_ADS_FAULT_OUTCOME_STARTED] = "started",
    [SENSORARRAY_ADS_FAULT_OUTCOME_ATTEMPT] = "attempt",
    [SENSORARRAY_ADS_FAULT_OUTCOME_RESUMED] = "resumed",
    [SENSORARRAY_ADS_FAULT_OUTCOME_FAILED] = "failed",
};

#define SENSORARRAY_ADS_FAULT_OUTCOME_COUNT \
    (sizeof(s_outcomeNames) / sizeof(s_outcomeNames[0]))

static uint64_t s_lastEmitUs[SENSORARRAY_ADS_FAULT_STAGE_COUNT];

const char *sensorarrayAdsFaultStageName(sensorarrayAdsFaultStage_t stage)
{
    if (stage >= SENSORARRAY_ADS_FAULT_STAGE_COUNT) {
        return "UNKNOWN";
    }
    return s_stageNames[stage];
}

const char *sensorarrayAdsFaultOutcomeName(sensorarrayAdsFaultOutcome_t outcome)
{
    if (outcome < 0 || (size_t)outcome >= SENSORARRAY_ADS_FAULT_OUTCOME_COUNT) {
        return "none";
    }
    return s_outcomeNames[outcome];
}

static const char *sensorarrayAdsFaultText(const char *text)
{
    return text && text[0] != '\0' ? text : "unknown";
}

size_t sensorarrayAdsFaultFormat(const sensorarrayAdsFaultEvent_t *event,
                                 char *buffer,
                                 size_t bufferSize)
{
    if (!event || !buffer || bufferSize == 0u) {
        return 0u;
    }

    char line[SENSORARRAY_ADS_FAULT_LINE_MAX + 1u];
    int written = snprintf(
        line,
        sizeof(line),
        "ADSFAULT,stage=%s,err=0x%lx,boot=%lu,bootId=%lu,seq=%lu,mode=%.16s,modeGen=%lu,profileGen=%lu,rowGen=%lu,rowReq=%lu,profile=%.8s,route=%.16s,owner=%.16s,drdyGen=%lu,cfgGen=%lu,rail=%ld,ref=%.16s,restoreExp=%ld,restoreAct=%ld,attempt=%lu,outcome=%s\n",
        sensorarrayAdsFaultStageName(event->stage),
        (unsigned long)(uint32_t)event->err,
        (unsigned long)event->bootCount,
        (unsigned long)event->bootId,
        (unsigned long)event->seq,
        sensorarrayAdsFaultText(event->mode),
        (unsigned long)event->modeGeneration,
        (unsigned long)event->profileGeneration,
        (unsigned long)event->rowGeneration,
        (unsigned long)event->rowRequestId,
        sensorarrayAdsFaultText(event->profile),
        sensorarrayAdsFaultText(event->route),
        sensorarrayAdsFaultText(event->owner),
        (unsigned long)event->drdyGeneration,
        (unsigned long)event->configGeneration,
        (long)(event->railValid ? event->railUv : 0),
        sensorarrayAdsFaultText(event->reference),
        (long)(event->restoreExpectedValid ? event->restoreExpected : 0),
        (long)(event->restoreActualValid ? event->restoreActual : 0),
        (unsigned long)event->attempt,
        sensorarrayAdsFaultOutcomeName(event->outcome));
    if (written < 0 || (size_t)written > SENSORARRAY_ADS_FAULT_LINE_MAX) {
        buffer[0] = '\0';
        return 0u;
    }

    size_t length = (size_t)written;
    if (length + 1u > bufferSize) {
        buffer[0] = '\0';
        return 0u;
    }
    memcpy(buffer, line, length + 1u);
    return length;
}

size_t sensorarrayAdsFaultEmit(const sensorarrayAdsFaultEvent_t *event,
                               uint64_t nowUs,
                               sensorarrayAdsFaultSink_t sink,
                               void *context)
{
    if (!event || !sink) {
        return 0u;
    }

    size_t stageIndex = (size_t)event->stage;
    if (stageIndex >= SENSORARRAY_ADS_FAULT_STAGE_COUNT) {
        stageIndex = SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN;
    }
    bool recoveryStage =
        event->stage >= SENSORARRAY_ADS_FAULT_STAGE_RECOVERY_START &&
        event->stage < SENSORARRAY_ADS_FAULT_STAGE_UNKNOWN;
    if (!recoveryStage &&
        s_lastEmitUs[stageIndex] != 0u &&
        nowUs >= s_lastEmitUs[stageIndex] &&
        nowUs - s_lastEmitUs[stageIndex] <
            SENSORARRAY_ADS_FAULT_RATE_LIMIT_US) {
        return 0u;
    }

    char line[SENSORARRAY_ADS_FAULT_LINE_MAX + 1u];
    size_t length = sensorarrayAdsFaultFormat(event, line, sizeof(line));
    if (length == 0u) {
        return 0u;
    }
    s_lastEmitUs[stageIndex] = nowUs;
    sink(line, length, context);
    return length;
}
