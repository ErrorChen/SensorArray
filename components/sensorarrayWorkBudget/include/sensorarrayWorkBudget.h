#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Added in this reliability fix. Each worker has exactly one task writer. */
typedef enum {
    SENSORARRAY_WORK_BUDGET_BLE_TX = 0,
    SENSORARRAY_WORK_BUDGET_TRANSPORT,
    SENSORARRAY_WORK_BUDGET_ASYNC_LOG,
    SENSORARRAY_WORK_BUDGET_SERIAL_TX,
    SENSORARRAY_WORK_BUDGET_BLE_CTRL,
    SENSORARRAY_WORK_BUDGET_SERIAL_CTRL,
    SENSORARRAY_WORK_BUDGET_WIFI_CTRL,
    SENSORARRAY_WORK_BUDGET_WORKER_COUNT,
} sensorarrayWorkBudgetWorker_t;

typedef struct {
    uint32_t checkpoints;
    uint32_t yields;
    uint32_t maxBurstUnits;
} sensorarrayWorkBudgetWorkerSnapshot_t;

typedef struct {
    esp_err_t initError;
    bool registered;
    bool idleKnown;
    uint32_t tickHz;
    uint32_t idleBudgetTicks; /* Global real-idle threshold, now four ticks. */
    uint32_t localQuantumTicks; /* Per-task elapsed window, separate from idle. */
    uint32_t maxWorkUnits;
    uint32_t graceUnits; /* Legacy field; zero under work_quantum_v2. */
    uint32_t backstopTicks; /* Real-idle threshold bypasses the local quantum. */
    uint32_t idleGeneration;
    uint32_t idleAgeTicks; /* UINT32_MAX when no real idle callback was observed. */
    uint32_t maxIdleGapTicks; /* UINT32_MAX before real idle; includes open gap. */
    sensorarrayWorkBudgetWorkerSnapshot_t workers[SENSORARRAY_WORK_BUDGET_WORKER_COUNT];
} sensorarrayWorkBudgetSnapshot_t;

/* Idempotent and non-waiting. Verifies aligned internal DRAM storage before
 * registering the hook; invalid placement returns ESP_ERR_INVALID_STATE.
 * A failed hook registration remains observable;
 * Checkpoint still supplies a bounded fallback without inventing idle progress. */
esp_err_t sensorarrayWorkBudgetInit(void);

/* Call at a bounded work-unit boundary, outside hardware/queue/stdio locks.
 * CPU0 only. At most one vTaskDelay(1) per call, never waits for idle to run.
 * A retained packet/slot remains caller-owned. The caller's absolute deadline
 * must include this wait and must not be restarted after it. Returns true iff
 * this call requested that real RTOS block. Its local unit/time budget restarts
 * after its own block, but only the idle callback updates global idle evidence.
 * The quantum begins at the next checkpoint; a single caller operation may
 * overshoot it. No notifications consumed. */
bool sensorarrayWorkBudgetCheckpoint(sensorarrayWorkBudgetWorker_t worker);

/* Individually sampled diagnostic counters, not an atomic configuration or a
 * CPU-utilization measurement. All tick/counter arithmetic is modulo 2^32. */
void sensorarrayWorkBudgetCopySnapshot(sensorarrayWorkBudgetSnapshot_t *snapshot);

#ifdef __cplusplus
}
#endif
