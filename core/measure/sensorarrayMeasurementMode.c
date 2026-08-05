#include "sensorarrayMeasurementMode.h"

#include <string.h>

static void sensorarrayMeasurementModeWriteBegin(sensorarrayMeasurementModeContext_t *context)
{
    __atomic_add_fetch(&context->snapshotVersion, 1u, __ATOMIC_RELEASE);
}

static void sensorarrayMeasurementModeWriteEnd(sensorarrayMeasurementModeContext_t *context)
{
    __atomic_add_fetch(&context->snapshotVersion, 1u, __ATOMIC_RELEASE);
}

bool sensorarrayMeasurementModeIsDataMode(sensorarrayMeasurementMode_t mode)
{
    return mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ||
           mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ||
           mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
}

uint8_t sensorarrayMeasurementCellCount(uint8_t rows)
{
    return rows >= 1u && rows <= 8u ? (uint8_t)(rows * 8u) : 0u;
}

sensorarrayMeasurementState_t sensorarrayMeasurementStateForMode(
    sensorarrayMeasurementMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE:
        return SENSORARRAY_MEASUREMENT_STATE_CAPACITANCE;
    case SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:
        return SENSORARRAY_MEASUREMENT_STATE_VOLTAGE;
    case SENSORARRAY_MEASUREMENT_MODE_RESISTANCE:
        return SENSORARRAY_MEASUREMENT_STATE_RESISTANCE;
    case SENSORARRAY_MEASUREMENT_MODE_NONE:
    default:
        return SENSORARRAY_MEASUREMENT_STATE_SAFE;
    }
}

void sensorarrayMeasurementModeInit(sensorarrayMeasurementModeContext_t *context)
{
    if (!context) {
        return;
    }
    memset(context, 0, sizeof(*context));
    context->snapshot = (sensorarrayMeasurementModeSnapshot_t){
        .state = SENSORARRAY_MEASUREMENT_STATE_SAFE,
        .activeMode = SENSORARRAY_MEASUREMENT_MODE_NONE,
        .oldMode = SENSORARRAY_MEASUREMENT_MODE_NONE,
        .pendingMode = SENSORARRAY_MEASUREMENT_MODE_NONE,
    };
}

bool sensorarrayMeasurementModeAccept(sensorarrayMeasurementModeContext_t *context,
                                      sensorarrayMeasurementMode_t requestedMode,
                                      uint32_t requestId)
{
    if (!context || !sensorarrayMeasurementModeIsDataMode(requestedMode)) {
        return false;
    }
    sensorarrayMeasurementModeWriteBegin(context);
    context->snapshot.pendingMode = requestedMode;
    context->snapshot.pendingRequestId = requestId;
    context->snapshot.pending = true;
    sensorarrayMeasurementModeWriteEnd(context);
    return true;
}

bool sensorarrayMeasurementModeBeginTransition(sensorarrayMeasurementModeContext_t *context)
{
    if (!context || !context->snapshot.pending ||
        !sensorarrayMeasurementModeIsDataMode(context->snapshot.pendingMode)) {
        return false;
    }
    sensorarrayMeasurementModeWriteBegin(context);
    context->snapshot.oldMode = context->snapshot.activeMode;
    context->snapshot.state = SENSORARRAY_MEASUREMENT_STATE_TRANSITION;
    context->snapshot.lastError = 0u;
    sensorarrayMeasurementModeWriteEnd(context);
    return true;
}

bool sensorarrayMeasurementModeCompleteTransition(sensorarrayMeasurementModeContext_t *context,
                                                  uint32_t appliedFrameSequence,
                                                  uint64_t transitionDurationUs)
{
    if (!context || context->snapshot.state != SENSORARRAY_MEASUREMENT_STATE_TRANSITION ||
        !context->snapshot.pending) {
        return false;
    }
    sensorarrayMeasurementModeWriteBegin(context);
    context->snapshot.activeMode = context->snapshot.pendingMode;
    context->snapshot.state = sensorarrayMeasurementStateForMode(
        context->snapshot.activeMode);
    context->snapshot.appliedRequestId = context->snapshot.pendingRequestId;
    context->snapshot.appliedFrameSequence = appliedFrameSequence;
    context->snapshot.transitionDurationUs = transitionDurationUs;
    context->snapshot.generation++;
    context->snapshot.pending = false;
    context->snapshot.pendingMode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    context->snapshot.pendingRequestId = 0u;
    sensorarrayMeasurementModeWriteEnd(context);
    return true;
}

void sensorarrayMeasurementModeFailTransition(sensorarrayMeasurementModeContext_t *context,
                                              uint32_t errorCode,
                                              uint64_t transitionDurationUs)
{
    if (!context) {
        return;
    }
    sensorarrayMeasurementModeWriteBegin(context);
    context->snapshot.activeMode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    context->snapshot.state = SENSORARRAY_MEASUREMENT_STATE_SAFE;
    context->snapshot.lastError = errorCode;
    context->snapshot.transitionDurationUs = transitionDurationUs;
    context->snapshot.appliedRequestId = context->snapshot.pendingRequestId;
    context->snapshot.pending = false;
    context->snapshot.pendingMode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    context->snapshot.pendingRequestId = 0u;
    sensorarrayMeasurementModeWriteEnd(context);
}

void sensorarrayMeasurementModeRecordRuntimeFault(sensorarrayMeasurementModeContext_t *context,
                                                  uint32_t errorCode)
{
    if (!context) {
        return;
    }
    sensorarrayMeasurementModeWriteBegin(context);
    context->snapshot.oldMode = context->snapshot.activeMode;
    context->snapshot.activeMode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    context->snapshot.state = SENSORARRAY_MEASUREMENT_STATE_DEGRADED;
    context->snapshot.lastError = errorCode;
    sensorarrayMeasurementModeWriteEnd(context);
}

bool sensorarrayMeasurementModeCopySnapshot(
    const sensorarrayMeasurementModeContext_t *context,
    sensorarrayMeasurementModeSnapshot_t *outSnapshot)
{
    if (!context || !outSnapshot) {
        return false;
    }
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt) {
        uint32_t before = __atomic_load_n(&context->snapshotVersion, __ATOMIC_ACQUIRE);
        if ((before & 1u) != 0u) {
            continue;
        }
        *outSnapshot = context->snapshot;
        uint32_t after = __atomic_load_n(&context->snapshotVersion, __ATOMIC_ACQUIRE);
        if (before == after && (after & 1u) == 0u) {
            return true;
        }
    }
    return false;
}

const char *sensorarrayMeasurementModeName(sensorarrayMeasurementMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE:
        return "CAP";
    case SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:
        return "VOLT";
    case SENSORARRAY_MEASUREMENT_MODE_RESISTANCE:
        return "RES";
    case SENSORARRAY_MEASUREMENT_MODE_NONE:
    default:
        return "NONE";
    }
}

const char *sensorarrayMeasurementStateName(sensorarrayMeasurementState_t state)
{
    switch (state) {
    case SENSORARRAY_MEASUREMENT_STATE_UNINITIALISED:
        return "UNINITIALISED";
    case SENSORARRAY_MEASUREMENT_STATE_SAFE:
        return "SAFE";
    case SENSORARRAY_MEASUREMENT_STATE_CAPACITANCE:
        return "CAPACITANCE";
    case SENSORARRAY_MEASUREMENT_STATE_VOLTAGE:
        return "VOLTAGE";
    case SENSORARRAY_MEASUREMENT_STATE_RESISTANCE:
        return "RESISTANCE";
    case SENSORARRAY_MEASUREMENT_STATE_TRANSITION:
        return "TRANSITION";
    case SENSORARRAY_MEASUREMENT_STATE_DEGRADED:
        return "DEGRADED";
    case SENSORARRAY_MEASUREMENT_STATE_FAULT:
    default:
        return "FAULT";
    }
}

const char *sensorarrayMeasurementUnitName(sensorarrayMeasurementUnit_t unit)
{
    switch (unit) {
    case SENSORARRAY_MEASUREMENT_UNIT_PF:
        return "pF";
    case SENSORARRAY_MEASUREMENT_UNIT_VOLT:
        return "V";
    case SENSORARRAY_MEASUREMENT_UNIT_OHM:
        return "ohm";
    case SENSORARRAY_MEASUREMENT_UNIT_NONE:
    default:
        return "none";
    }
}

const char *sensorarrayAdsReferenceSourceName(sensorarrayAdsReferenceSource_t source)
{
    switch (source) {
    case SENSORARRAY_ADS_REFERENCE_INTERNAL:
        return "INTREF";
    case SENSORARRAY_ADS_REFERENCE_AVDD_AVSS:
        return "AVDD_AVSS";
    case SENSORARRAY_ADS_REFERENCE_EXTERNAL:
        return "EXTERNAL";
    case SENSORARRAY_ADS_REFERENCE_NONE:
    default:
        return "NONE";
    }
}

const char *sensorarrayCellErrorName(sensorarrayCellError_t error)
{
    static const char *const names[] = {
        "ok", "route", "spi", "timeout", "stale", "ref_alarm",
        "pga_abs", "pga_diff", "saturated", "common_mode", "rail",
        "reference", "denominator", "open", "short", "negative",
        "range", "overflow", "unstable", "autorange", "unsupported",
        "readback",
    };
    size_t index = (size_t)error;
    return index < sizeof(names) / sizeof(names[0]) ? names[index] : "unknown";
}
