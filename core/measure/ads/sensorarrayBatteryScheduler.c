#include "sensorarrayBatteryScheduler.h"

#include <limits.h>
#include <stddef.h>

#define SENSORARRAY_BATTERY_PERIOD_MIN_MS 100u
#define SENSORARRAY_BATTERY_PERIOD_MAX_MS 600000u

static uint64_t sensorarrayBatteryMillisecondsToMicroseconds(uint32_t milliseconds)
{
    return (uint64_t)milliseconds * 1000ULL;
}

static uint64_t sensorarrayBatteryAddSaturating(uint64_t left, uint64_t right)
{
    return UINT64_MAX - left < right ? UINT64_MAX : left + right;
}

void sensorarrayBatterySchedulerInit(sensorarrayBatteryScheduler_t *scheduler,
                                     bool enabled,
                                     uint32_t periodMs,
                                     uint32_t maximumDeferMs,
                                     uint64_t nowUs)
{
    if (!scheduler) {
        return;
    }
    *scheduler = (sensorarrayBatteryScheduler_t){
        .initializedUs = nowUs,
        .lastRunUs = nowUs,
        .nextDueUs = enabled ? sensorarrayBatteryAddSaturating(
            nowUs, sensorarrayBatteryMillisecondsToMicroseconds(periodMs)) : 0u,
        .periodMs = periodMs,
        .maximumDeferMs = maximumDeferMs,
        .enabled = enabled,
    };
}

bool sensorarrayBatterySchedulerConfigure(sensorarrayBatteryScheduler_t *scheduler,
                                         bool enabled,
                                         uint32_t periodMs,
                                         uint64_t nowUs)
{
    if (!scheduler ||
        (enabled && (periodMs < SENSORARRAY_BATTERY_PERIOD_MIN_MS ||
                     periodMs > SENSORARRAY_BATTERY_PERIOD_MAX_MS))) {
        return false;
    }
    scheduler->enabled = enabled;
    if (enabled) {
        scheduler->periodMs = periodMs;
        /* A runtime period change starts a new wall-clock interval. This keeps
         * a shorter period from unexpectedly injecting a transaction into the
         * frame whose boundary accepted the command. */
        scheduler->nextDueUs = sensorarrayBatteryAddSaturating(
            nowUs, sensorarrayBatteryMillisecondsToMicroseconds(periodMs));
    } else {
        scheduler->nextDueUs = 0u;
    }
    scheduler->dueSinceUs = 0u;
    scheduler->forcePending = false;
    return true;
}

void sensorarrayBatterySchedulerRequestNow(sensorarrayBatteryScheduler_t *scheduler,
                                           uint64_t nowUs)
{
    if (!scheduler) {
        return;
    }
    scheduler->forcePending = true;
    if (scheduler->dueSinceUs == 0u) {
        scheduler->dueSinceUs = nowUs == 0u ? 1u : nowUs;
    }
}

bool sensorarrayBatterySchedulerIsDue(sensorarrayBatteryScheduler_t *scheduler,
                                      uint64_t nowUs)
{
    if (!scheduler) {
        return false;
    }
    bool due = scheduler->forcePending;
    if (!due && scheduler->enabled && scheduler->nextDueUs != 0u) {
        due = nowUs >= scheduler->nextDueUs;
    }
    return due;
}

static void sensorarrayBatterySchedulerMarkDue(
    sensorarrayBatteryScheduler_t *scheduler,
    uint64_t nowUs)
{
    if (scheduler && scheduler->dueSinceUs == 0u) {
        uint64_t dueUs = nowUs;
        if (!scheduler->forcePending && scheduler->enabled) {
            dueUs = scheduler->nextDueUs;
        }
        scheduler->dueSinceUs = dueUs == 0u ? 1u : dueUs;
    }
}

sensorarrayBatteryDecision_t sensorarrayBatterySchedulerEvaluateGap(
    sensorarrayBatteryScheduler_t *scheduler,
    uint64_t nowUs,
    uint32_t availableGapUs,
    uint32_t estimatedJobUs,
    uint32_t guardUs)
{
    if (!scheduler) {
        return SENSORARRAY_BATTERY_DECISION_DISABLED;
    }
    if (!scheduler->enabled && !scheduler->forcePending) {
        return SENSORARRAY_BATTERY_DECISION_DISABLED;
    }
    if (!sensorarrayBatterySchedulerIsDue(scheduler, nowUs)) {
        return SENSORARRAY_BATTERY_DECISION_NOT_DUE;
    }
    sensorarrayBatterySchedulerMarkDue(scheduler, nowUs);
    /* BATNOW is deliberately a complete-frame-boundary request. It never
     * consumes an FDC wait window even when that window is large enough. */
    if (scheduler->forcePending) {
        scheduler->deferCount++;
        return SENSORARRAY_BATTERY_DECISION_DEFER;
    }
    uint64_t admissionUs = (uint64_t)estimatedJobUs + (uint64_t)guardUs;
    if ((uint64_t)availableGapUs >= admissionUs) {
        return SENSORARRAY_BATTERY_DECISION_RUN_GAP;
    }
    scheduler->skipCount++;
    scheduler->deferCount++;
    return SENSORARRAY_BATTERY_DECISION_DEFER;
}

sensorarrayBatteryDecision_t sensorarrayBatterySchedulerEvaluateBoundary(
    sensorarrayBatteryScheduler_t *scheduler,
    uint64_t nowUs,
    bool capacitanceMode)
{
    if (!scheduler) {
        return SENSORARRAY_BATTERY_DECISION_DISABLED;
    }
    if (!scheduler->enabled && !scheduler->forcePending) {
        return SENSORARRAY_BATTERY_DECISION_DISABLED;
    }
    if (!sensorarrayBatterySchedulerIsDue(scheduler, nowUs)) {
        return SENSORARRAY_BATTERY_DECISION_NOT_DUE;
    }
    sensorarrayBatterySchedulerMarkDue(scheduler, nowUs);
    if (!capacitanceMode || scheduler->forcePending) {
        return SENSORARRAY_BATTERY_DECISION_RUN_BOUNDARY;
    }
    uint64_t deferredUs = nowUs >= scheduler->dueSinceUs ?
        nowUs - scheduler->dueSinceUs : 0u;
    if (deferredUs >=
        sensorarrayBatteryMillisecondsToMicroseconds(scheduler->maximumDeferMs)) {
        return SENSORARRAY_BATTERY_DECISION_RUN_BOUNDARY;
    }
    return SENSORARRAY_BATTERY_DECISION_DEFER;
}

void sensorarrayBatterySchedulerRecordRun(sensorarrayBatteryScheduler_t *scheduler,
                                          uint64_t completedUs,
                                          uint32_t durationUs,
                                          bool boundaryFallback,
                                          bool restoreOk)
{
    if (!scheduler) {
        return;
    }
    bool periodicDue = scheduler->enabled && scheduler->nextDueUs != 0u &&
                       completedUs >= scheduler->nextDueUs;
    scheduler->lastRunUs = completedUs;
    if (periodicDue) {
        /* Advance from the previous absolute deadline, not from completion.
         * Transaction time therefore cannot accumulate into multi-second
         * drift during a long 1 Hz dwell. Skip missed slots rather than
         * launching a burst of catch-up transactions. */
        uint64_t periodUs = sensorarrayBatteryMillisecondsToMicroseconds(
            scheduler->periodMs);
        uint64_t elapsedSinceDue = completedUs - scheduler->nextDueUs;
        uint64_t periodsToAdvance = elapsedSinceDue / periodUs + 1u;
        uint64_t advanceUs = periodsToAdvance > UINT64_MAX / periodUs ?
            UINT64_MAX : periodsToAdvance * periodUs;
        scheduler->nextDueUs = sensorarrayBatteryAddSaturating(
            scheduler->nextDueUs, advanceUs);
    } else if (scheduler->enabled && scheduler->nextDueUs == 0u) {
        scheduler->nextDueUs = sensorarrayBatteryAddSaturating(
            completedUs,
            sensorarrayBatteryMillisecondsToMicroseconds(scheduler->periodMs));
    }
    scheduler->dueSinceUs = 0u;
    scheduler->forcePending = false;
    scheduler->runCount++;
    if (boundaryFallback) {
        scheduler->boundaryCount++;
    }
    if (!restoreOk) {
        scheduler->restoreFailureCount++;
    }
    scheduler->sampleDurationTotalUs += durationUs;
    if (durationUs > scheduler->sampleDurationMaximumUs) {
        scheduler->sampleDurationMaximumUs = durationUs;
    }
}

uint32_t sensorarrayBatterySchedulerAgeMs(
    const sensorarrayBatteryScheduler_t *scheduler,
    uint64_t nowUs)
{
    if (!scheduler || scheduler->runCount == 0u || nowUs < scheduler->lastRunUs) {
        return UINT32_MAX;
    }
    uint64_t ageMs = (nowUs - scheduler->lastRunUs) / 1000ULL;
    return ageMs > UINT32_MAX ? UINT32_MAX : (uint32_t)ageMs;
}
