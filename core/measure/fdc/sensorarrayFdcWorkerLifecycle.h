#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Added in this repair: notifications are wakeups, never JOB/RUN semantics.
 * Release/acquire publishes all persistent slot storage across the two cores. */
typedef enum {
    SENSORARRAY_FDC_JOB_IDLE = 0,
    SENSORARRAY_FDC_JOB_POSTED,
    SENSORARRAY_FDC_JOB_ACKED,
    SENSORARRAY_FDC_JOB_RUNNING,
    SENSORARRAY_FDC_JOB_DONE,
    SENSORARRAY_FDC_JOB_CANCEL_REQUESTED,
    SENSORARRAY_FDC_JOB_CANCEL_ACKED,
    SENSORARRAY_FDC_JOB_RECLAIMED,
    SENSORARRAY_FDC_JOB_QUARANTINED,
} sensorarrayFdcJobPhase_t;

typedef struct {
    uint32_t phase;
    uint32_t released;
} sensorarrayFdcJobLifecycle_t;

static inline sensorarrayFdcJobPhase_t sensorarrayFdcJobPhase(
    const sensorarrayFdcJobLifecycle_t *life)
{
    return (sensorarrayFdcJobPhase_t)__atomic_load_n(&life->phase, __ATOMIC_ACQUIRE);
}

static inline bool sensorarrayFdcJobChange(sensorarrayFdcJobLifecycle_t *life,
    sensorarrayFdcJobPhase_t from, sensorarrayFdcJobPhase_t to)
{
    uint32_t expected = (uint32_t)from;
    return __atomic_compare_exchange_n(&life->phase, &expected, (uint32_t)to,
                                       false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

static inline bool sensorarrayFdcJobReleased(const sensorarrayFdcJobLifecycle_t *life)
{
    return __atomic_load_n(&life->released, __ATOMIC_ACQUIRE) != 0u ||
           sensorarrayFdcJobPhase(life) == SENSORARRAY_FDC_JOB_IDLE;
}

static inline bool sensorarrayFdcJobPost(sensorarrayFdcJobLifecycle_t *life)
{
    sensorarrayFdcJobPhase_t phase = sensorarrayFdcJobPhase(life);
    if (phase != SENSORARRAY_FDC_JOB_IDLE && phase != SENSORARRAY_FDC_JOB_RECLAIMED) {
        return false;
    }
    __atomic_store_n(&life->released, 0u, __ATOMIC_RELAXED);
    __atomic_store_n(&life->phase, SENSORARRAY_FDC_JOB_POSTED, __ATOMIC_RELEASE);
    return true;
}

static inline void sensorarrayFdcJobCancel(sensorarrayFdcJobLifecycle_t *life)
{
    sensorarrayFdcJobPhase_t phase = sensorarrayFdcJobPhase(life);
    while (phase == SENSORARRAY_FDC_JOB_POSTED || phase == SENSORARRAY_FDC_JOB_ACKED ||
           phase == SENSORARRAY_FDC_JOB_RUNNING) {
        if (sensorarrayFdcJobChange(life, phase, SENSORARRAY_FDC_JOB_CANCEL_REQUESTED)) {
            return;
        }
        phase = sensorarrayFdcJobPhase(life);
    }
}

static inline void sensorarrayFdcJobFinish(sensorarrayFdcJobLifecycle_t *life)
{
    sensorarrayFdcJobPhase_t phase = sensorarrayFdcJobPhase(life);
    while (phase != SENSORARRAY_FDC_JOB_QUARANTINED) {
        sensorarrayFdcJobPhase_t terminal = phase == SENSORARRAY_FDC_JOB_CANCEL_REQUESTED ?
            SENSORARRAY_FDC_JOB_CANCEL_ACKED : SENSORARRAY_FDC_JOB_DONE;
        if (sensorarrayFdcJobChange(life, phase, terminal)) {
            break;
        }
        phase = sensorarrayFdcJobPhase(life);
    }
    /* Last access to job/result/trace precedes this release publication. */
    __atomic_store_n(&life->released, 1u, __ATOMIC_RELEASE);
}

static inline bool sensorarrayFdcJobReclaim(sensorarrayFdcJobLifecycle_t *life)
{
    if (!sensorarrayFdcJobReleased(life)) {
        return false;
    }
    sensorarrayFdcJobPhase_t phase = sensorarrayFdcJobPhase(life);
    return (phase == SENSORARRAY_FDC_JOB_DONE || phase == SENSORARRAY_FDC_JOB_CANCEL_ACKED) &&
        sensorarrayFdcJobChange(life, phase, SENSORARRAY_FDC_JOB_RECLAIMED);
}

static inline void sensorarrayFdcJobQuarantine(sensorarrayFdcJobLifecycle_t *life)
{
    __atomic_store_n(&life->phase, SENSORARRAY_FDC_JOB_QUARANTINED, __ATOMIC_RELEASE);
}
