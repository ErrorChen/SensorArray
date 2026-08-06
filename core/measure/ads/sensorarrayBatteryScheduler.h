#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SENSORARRAY_BATTERY_DECISION_NOT_DUE = 0,
    SENSORARRAY_BATTERY_DECISION_DISABLED,
    SENSORARRAY_BATTERY_DECISION_DEFER,
    SENSORARRAY_BATTERY_DECISION_RUN_GAP,
    SENSORARRAY_BATTERY_DECISION_RUN_BOUNDARY,
} sensorarrayBatteryDecision_t;

typedef struct {
    uint64_t initializedUs;
    uint64_t lastRunUs;
    uint64_t nextDueUs;
    uint64_t dueSinceUs;
    uint64_t sampleDurationTotalUs;
    uint32_t periodMs;
    uint32_t maximumDeferMs;
    uint32_t runCount;
    uint32_t skipCount;
    uint32_t deferCount;
    uint32_t boundaryCount;
    uint32_t restoreFailureCount;
    uint32_t sampleDurationMaximumUs;
    bool enabled;
    bool forcePending;
    bool gapDeferred;
} sensorarrayBatteryScheduler_t;

void sensorarrayBatterySchedulerInit(sensorarrayBatteryScheduler_t *scheduler,
                                     bool enabled,
                                     uint32_t periodMs,
                                     uint32_t maximumDeferMs,
                                     uint64_t nowUs);
bool sensorarrayBatterySchedulerConfigure(sensorarrayBatteryScheduler_t *scheduler,
                                         bool enabled,
                                         uint32_t periodMs,
                                         uint64_t nowUs);
void sensorarrayBatterySchedulerRequestNow(sensorarrayBatteryScheduler_t *scheduler,
                                           uint64_t nowUs);
bool sensorarrayBatterySchedulerIsDue(sensorarrayBatteryScheduler_t *scheduler,
                                      uint64_t nowUs);
sensorarrayBatteryDecision_t sensorarrayBatterySchedulerEvaluateGap(
    sensorarrayBatteryScheduler_t *scheduler,
    uint64_t nowUs,
    uint32_t availableGapUs,
    uint32_t estimatedJobUs,
    uint32_t guardUs);
sensorarrayBatteryDecision_t sensorarrayBatterySchedulerEvaluateBoundary(
    sensorarrayBatteryScheduler_t *scheduler,
    uint64_t nowUs,
    bool capacitanceMode);
void sensorarrayBatterySchedulerRecordRun(sensorarrayBatteryScheduler_t *scheduler,
                                          uint64_t completedUs,
                                          uint32_t durationUs,
                                          bool boundaryFallback,
                                          bool restoreOk);
uint32_t sensorarrayBatterySchedulerAgeMs(
    const sensorarrayBatteryScheduler_t *scheduler,
    uint64_t nowUs);
