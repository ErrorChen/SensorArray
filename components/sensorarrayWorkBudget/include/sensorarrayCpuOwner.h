#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensorarrayWorkBudget.h"

/* 本轮新增: optional owner-only scheduled-runtime observation. These are
 * switch-out accumulators, including non-SDK work and possibly ISR time.
 * They cannot be subtracted from SDK wall time to obtain blocked time. */
typedef struct {
    bool enabled;
    bool available;
    bool sampleKnown;
    bool conflict;
    bool counterRegressed;
    bool clockRegressed;
    bool stale;
    bool ageKnown;
    uint32_t taskNumber;
    uint32_t conflictingTaskNumber;
    uint64_t runUs;
    uint64_t t0Us;
    uint64_t t1Us;
    uint64_t observedAtUs;
    uint64_t ageUs;
} sensorarrayCpuOwnerSnapshot_t;

/* Only the actual owner calls this at existing CPU0 work boundaries. No
 * allocation, new delay, task-handle retention, or stack-watermark scan.
 * At most one self-sample per role per second; a second observed task number
 * makes that role unavailable until reboot without changing its work budget. */
void sensorarrayCpuOwnerCheckpoint(sensorarrayWorkBudgetWorker_t worker);

/* Control readers only copy the published value. They never query a live task.
 * A stale historical sample is not evidence of zero current CPU consumption. */
bool sensorarrayCpuOwnerCopySnapshot(sensorarrayWorkBudgetWorker_t worker,
                                    sensorarrayCpuOwnerSnapshot_t *snapshot);

/* 本轮新增: strict CPUOWNER?0 through CPUOWNER?6. session is immutable for this
 * boot and is provided by main; taskNumber/counters cannot cross sessions. */
bool sensorarrayCpuOwnerHandles(const char *command);
esp_err_t sensorarrayCpuOwnerFormat(const char *command, uint64_t session,
                                    char *response, size_t size);
