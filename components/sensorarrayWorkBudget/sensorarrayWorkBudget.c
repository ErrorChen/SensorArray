#include "sensorarrayWorkBudget.h"

#include <stddef.h>
/* FreeRTOS must precede hooks.h: the latter includes portmacro.h, whose
 * multi-core definitions require the already established FreeRTOS config. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_memory_utils.h"
#include "esp_freertos_hooks.h"
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
#include "sensorarrayCpuOwner.h"
#endif

/* Two independent bounds: each task gets at most 32 checkpoint units or a
 * two-tick local quantum before a real block. Four ticks without actual CPU0
 * idle bypass that local budget. Resetting a task's local quantum after its
 * own block is not an idle observation and never updates the shared idle age.
 * This avoids treating time spent in other tasks as a permanent per-record
 * penalty. A quantum starts at the first boundary after a block; an individual
 * SDK call can overshoot it and is not bounded here. No acquisition FPS cap. */
#define SENSORARRAY_WORK_BUDGET_MAX_UNITS 32u
#define SENSORARRAY_WORK_BUDGET_QUANTUM_TICKS \
    ((pdMS_TO_TICKS(20u) > 0u) ? pdMS_TO_TICKS(20u) : 1u)
#define SENSORARRAY_WORK_BUDGET_BACKSTOP_TICKS (2u * SENSORARRAY_WORK_BUDGET_QUANTUM_TICKS)

_Static_assert(sizeof(TickType_t) == sizeof(uint32_t), "32-bit tick arithmetic required");
_Static_assert(portTICK_TYPE_IS_ATOMIC == 1, "idle observation must not take a tick lock");
#if !defined(ESP_PLATFORM)
_Static_assert(__atomic_always_lock_free(sizeof(uint32_t), 0), "idle counters must be lock-free");
#elif defined(__XTENSA__)
_Static_assert(XCHAL_HAVE_S32C1I == 1, "IDF internal-memory atomics require S32C1I");
#endif
_Static_assert(sizeof(esp_err_t) == sizeof(uint32_t), "all shared fields must be 32-bit");

typedef struct {
    uint32_t checkpoints;
    uint32_t yields;
    uint32_t maxBurstUnits;
    uint32_t quantumDue;
    uint32_t backstopDue;
    uint32_t unitsDue;
    /* Only this worker's task mutates these private scheduling fields. */
    uint32_t units;
    uint32_t quantumStartTick;
    bool quantumActive;
    uint32_t lastIdleGeneration;
} sensorarrayWorkBudgetWorkerState_t;

/* IDF's S32C1I PSRAM workaround deliberately makes GCC's generic lock-free
 * constant false. Its __atomic_{load,store}_4 wrappers dispatch non-EXTRAM
 * addresses to native MEMW/L32I/S32I helpers; EXTRAM would take a global lock.
 * Pin every shared object to internal DRAM, retain C atomic semantics, and
 * verify addresses before installing the idle callback. No workaround or
 * global compiler flag is disabled. All idle fields have one writer (IDLE0),
 * and each worker counter has one writer (its named task). */
static WORD_ALIGNED_ATTR DRAM_ATTR uint32_t s_initState; /* 0=not started, 1=registering, 2=finished */
static WORD_ALIGNED_ATTR DRAM_ATTR uint32_t s_initTick;
static WORD_ALIGNED_ATTR DRAM_ATTR esp_err_t s_initError = ESP_ERR_INVALID_STATE;
static WORD_ALIGNED_ATTR DRAM_ATTR uint32_t s_idleGeneration;
static WORD_ALIGNED_ATTR DRAM_ATTR uint32_t s_idleTick;
static WORD_ALIGNED_ATTR DRAM_ATTR uint32_t s_maxIdleGapTicks;
static WORD_ALIGNED_ATTR DRAM_ATTR sensorarrayWorkBudgetWorkerState_t
    s_workers[SENSORARRAY_WORK_BUDGET_WORKER_COUNT];
_Static_assert(_Alignof(sensorarrayWorkBudgetWorkerState_t) >= sizeof(uint32_t),
               "worker fields require aligned word access");

static bool sensorarrayWorkBudgetInternal(const void *address, size_t length)
{
    return length >= sizeof(uint32_t) && ((uintptr_t)address % sizeof(uint32_t)) == 0u &&
        /* Broader "internal" checks can also admit repurposed cache banks
         * inside the SDK atomic wrapper's EXTRAM interval. Require DRAM. */
        esp_ptr_in_dram(address) &&
        esp_ptr_in_dram((const uint8_t *)address + length - 1u);
}

static bool sensorarrayWorkBudgetStorageInternal(void)
{
    return sensorarrayWorkBudgetInternal(&s_initState, sizeof(s_initState)) &&
        sensorarrayWorkBudgetInternal(&s_initTick, sizeof(s_initTick)) &&
        sensorarrayWorkBudgetInternal(&s_initError, sizeof(s_initError)) &&
        sensorarrayWorkBudgetInternal(&s_idleGeneration, sizeof(s_idleGeneration)) &&
        sensorarrayWorkBudgetInternal(&s_idleTick, sizeof(s_idleTick)) &&
        sensorarrayWorkBudgetInternal(&s_maxIdleGapTicks, sizeof(s_maxIdleGapTicks)) &&
        sensorarrayWorkBudgetInternal(s_workers, sizeof(s_workers));
}

static uint32_t sensorarrayWorkBudgetLoad(const uint32_t *value)
{
    return __atomic_load_n(value, __ATOMIC_RELAXED);
}

/* ESP-IDF calls this from the CPU0 idle task. On ESP32-S3 the tick load is
 * atomic (no critical section); there is no log, allocation, lock, or WD feed.
 * Returning true allows IDF's idle hook to enter esp_cpu_wait_for_intr(). */
static bool sensorarrayWorkBudgetIdleHook(void)
{
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t generation = sensorarrayWorkBudgetLoad(&s_idleGeneration);
    uint32_t previous = sensorarrayWorkBudgetLoad(&s_idleTick);
    uint32_t gap = now - previous;
    if (generation != 0u && gap > sensorarrayWorkBudgetLoad(&s_maxIdleGapTicks)) {
        __atomic_store_n(&s_maxIdleGapTicks, gap, __ATOMIC_RELAXED);
    }
    __atomic_store_n(&s_idleTick, now, __ATOMIC_RELAXED);
    /* Reserve zero for "not observed", including the counter wrap. */
    generation++;
    if (generation == 0u) {
        generation = 1u;
    }
    __atomic_store_n(&s_idleGeneration, generation, __ATOMIC_RELEASE);
    return true;
}

esp_err_t sensorarrayWorkBudgetInit(void)
{
    if (!sensorarrayWorkBudgetStorageInternal()) {
        /* Do not touch potentially lock-backed atomics or register an unsafe
         * idle hook. CopySnapshot reports this explicit initialization error. */
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t expected = 0u;
    if (__atomic_compare_exchange_n(&s_initState, &expected, 1u, false,
                                    __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        __atomic_store_n(&s_initTick, (uint32_t)xTaskGetTickCount(), __ATOMIC_RELAXED);
        esp_err_t err = esp_register_freertos_idle_hook_for_cpu(
            sensorarrayWorkBudgetIdleHook, 0u);
        __atomic_store_n(&s_initError, err, __ATOMIC_RELAXED);
        __atomic_store_n(&s_initState, 2u, __ATOMIC_RELEASE);
        return err;
    }
    /* Do not spin on another caller registering a hook. Fallback checkpoints
     * are safe during that brief interval, and diagnostics expose the state. */
    return __atomic_load_n(&s_initState, __ATOMIC_ACQUIRE) == 2u ?
        __atomic_load_n(&s_initError, __ATOMIC_RELAXED) : ESP_ERR_INVALID_STATE;
}

bool sensorarrayWorkBudgetCheckpoint(sensorarrayWorkBudgetWorker_t worker)
{
    if ((unsigned)worker >= SENSORARRAY_WORK_BUDGET_WORKER_COUNT ||
        xPortGetCoreID() != 0) {
        return false;
    }
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    /* Optional owner self-sample only; OFF has no call, timer read or lock.
     * The observation neither adds a delay nor changes budget predicates. */
    sensorarrayCpuOwnerCheckpoint(worker);
#endif
    if (!sensorarrayWorkBudgetStorageInternal()) {
        /* Defensive fallback if a future linker/storage change violates the
         * internal-memory contract. No atomics are accessed on this path. */
        vTaskDelay(1u);
        return true;
    }
    sensorarrayWorkBudgetWorkerState_t *state = &s_workers[worker];
    __atomic_store_n(&state->checkpoints,
                     sensorarrayWorkBudgetLoad(&state->checkpoints) + 1u, __ATOMIC_RELAXED);
    uint32_t generation = __atomic_load_n(&s_idleGeneration, __ATOMIC_ACQUIRE);
    uint32_t reference = generation != 0u ? sensorarrayWorkBudgetLoad(&s_idleTick) :
        sensorarrayWorkBudgetLoad(&s_initTick);
    uint32_t now = (uint32_t)xTaskGetTickCount();
    if (generation != 0u && generation != state->lastIdleGeneration) {
        /* Real idle proves this task did not remain continuously ready. */
        state->units = 0u;
        state->quantumActive = false;
        state->lastIdleGeneration = generation;
    }
    if (!state->quantumActive) {
        state->quantumStartTick = now;
        state->quantumActive = true;
    }
    state->units++;
    if (state->units > sensorarrayWorkBudgetLoad(&state->maxBurstUnits)) {
        __atomic_store_n(&state->maxBurstUnits, state->units, __ATOMIC_RELAXED);
    }
    uint32_t idleAge = now - reference;
    const bool backstopDue = idleAge >= SENSORARRAY_WORK_BUDGET_BACKSTOP_TICKS;
    const bool quantumDue = now - state->quantumStartTick >= SENSORARRAY_WORK_BUDGET_QUANTUM_TICKS;
    const bool unitsDue = state->units >= SENSORARRAY_WORK_BUDGET_MAX_UNITS;
    bool due = backstopDue || quantumDue || unitsDue;
    if (!due) {
        return false;
    }
    /* Observe the original predicates, without changing their OR semantics.
     * Multiple reasons may hold at this same boundary. */
    if (quantumDue) {
        __atomic_store_n(&state->quantumDue, sensorarrayWorkBudgetLoad(&state->quantumDue) + 1u, __ATOMIC_RELAXED);
    }
    if (backstopDue) {
        __atomic_store_n(&state->backstopDue, sensorarrayWorkBudgetLoad(&state->backstopDue) + 1u, __ATOMIC_RELAXED);
    }
    if (unitsDue) {
        __atomic_store_n(&state->unitsDue, sensorarrayWorkBudgetLoad(&state->unitsDue) + 1u, __ATOMIC_RELAXED);
    }
    state->units = 0u;
    /* Start at the next boundary, after the caller resumes. The RTOS block
     * itself must not immediately exhaust the new local quantum. Shared idle
     * evidence stays unchanged, so the starvation backstop still wins. */
    state->quantumActive = false;
    __atomic_store_n(&state->yields,
                     sensorarrayWorkBudgetLoad(&state->yields) + 1u, __ATOMIC_RELAXED);
    /* One bounded block only. A broken hook or continuously busy SDK task
     * cannot trap CTRL/slots here; the next work unit and its deadline proceed. */
    vTaskDelay(1u);
    return true;
}

uint32_t sensorarrayWorkBudgetIdleGeneration(void)
{
    if (!sensorarrayWorkBudgetStorageInternal()) { return 0u; }
    return __atomic_load_n(&s_idleGeneration, __ATOMIC_ACQUIRE);
}

void sensorarrayWorkBudgetCopySnapshot(sensorarrayWorkBudgetSnapshot_t *snapshot)
{
    if (!snapshot) {
        return;
    }
    *snapshot = (sensorarrayWorkBudgetSnapshot_t){0};
    snapshot->initError = ESP_ERR_INVALID_STATE;
    snapshot->idleAgeTicks = UINT32_MAX;
    snapshot->maxIdleGapTicks = UINT32_MAX;
    if (!sensorarrayWorkBudgetStorageInternal()) {
        return;
    }
    uint32_t initState = __atomic_load_n(&s_initState, __ATOMIC_ACQUIRE);
    snapshot->initError = __atomic_load_n(&s_initError, __ATOMIC_RELAXED);
    snapshot->registered = initState == 2u && snapshot->initError == ESP_OK;
    snapshot->tickHz = configTICK_RATE_HZ;
    snapshot->idleBudgetTicks = SENSORARRAY_WORK_BUDGET_BACKSTOP_TICKS;
    snapshot->localQuantumTicks = SENSORARRAY_WORK_BUDGET_QUANTUM_TICKS;
    snapshot->maxWorkUnits = SENSORARRAY_WORK_BUDGET_MAX_UNITS;
    snapshot->graceUnits = 0u; /* Legacy field: no real-idle epoch grace. */
    snapshot->backstopTicks = SENSORARRAY_WORK_BUDGET_BACKSTOP_TICKS;
    snapshot->idleGeneration = __atomic_load_n(&s_idleGeneration, __ATOMIC_ACQUIRE);
    snapshot->idleKnown = snapshot->idleGeneration != 0u;
    uint32_t reference = snapshot->idleKnown ? sensorarrayWorkBudgetLoad(&s_idleTick) :
        sensorarrayWorkBudgetLoad(&s_initTick);
    /* Load the reference before time: CPU0 may update it while a Core1 query
     * runs. The reverse order can subtract a future tick and invent 2^32 age.
     * A concurrent newer idle may make this age conservative by one observed
     * interval; this diagnostic snapshot does not claim atomic field identity. */
    uint32_t now = (uint32_t)xTaskGetTickCount();
    uint32_t age = now - reference;
    snapshot->idleAgeTicks = snapshot->idleKnown ? age : UINT32_MAX;
    snapshot->maxIdleGapTicks = snapshot->idleKnown ?
        sensorarrayWorkBudgetLoad(&s_maxIdleGapTicks) : UINT32_MAX;
    if (snapshot->idleKnown && age > snapshot->maxIdleGapTicks) {
        snapshot->maxIdleGapTicks = age;
    }
    for (unsigned i = 0u; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) {
        snapshot->workers[i].checkpoints = sensorarrayWorkBudgetLoad(&s_workers[i].checkpoints);
        snapshot->workers[i].yields = sensorarrayWorkBudgetLoad(&s_workers[i].yields);
        snapshot->workers[i].maxBurstUnits = sensorarrayWorkBudgetLoad(&s_workers[i].maxBurstUnits);
        snapshot->workers[i].quantumDue = sensorarrayWorkBudgetLoad(&s_workers[i].quantumDue);
        snapshot->workers[i].backstopDue = sensorarrayWorkBudgetLoad(&s_workers[i].backstopDue);
        snapshot->workers[i].unitsDue = sensorarrayWorkBudgetLoad(&s_workers[i].unitsDue);
    }
}
