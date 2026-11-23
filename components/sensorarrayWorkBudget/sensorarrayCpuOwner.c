#include "sensorarrayCpuOwner.h"

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"

#define SENSORARRAY_CPU_OWNER_SAMPLE_US UINT64_C(1000000)
#define SENSORARRAY_CPU_OWNER_STALE_US UINT64_C(3000000)

#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_timer.h"

#if CONFIG_FREERTOS_SMP || CONFIG_FREERTOS_UNICORE || !CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS || \
    !CONFIG_FREERTOS_RUN_TIME_COUNTER_TYPE_U64 || \
    !CONFIG_FREERTOS_RUN_TIME_STATS_USING_ESP_TIMER
#error "CPU owner observation requires the dual-core IDF kernel, U64 runtime, and ESP_TIMER"
#endif
_Static_assert(sizeof(configRUN_TIME_COUNTER_TYPE) == sizeof(uint64_t),
               "runtime source and accumulator must retain 64 bits");
_Static_assert(sizeof(((TaskStatus_t *)0)->ulRunTimeCounter) == sizeof(uint64_t),
               "TaskStatus runtime must retain 64 bits");

typedef struct {
    bool sampling;
    sensorarrayCpuOwnerSnapshot_t value;
} sensorarrayCpuOwnerSlot_t;

/* Separate from the idle hook's 32-bit atomics. All U64 publication is under
 * this short task-context mux, never held across timer/kernel/format calls. */
static DRAM_ATTR portMUX_TYPE s_cpuOwnerMux = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR sensorarrayCpuOwnerSlot_t
    s_cpuOwners[SENSORARRAY_WORK_BUDGET_WORKER_COUNT];

void sensorarrayCpuOwnerCheckpoint(sensorarrayWorkBudgetWorker_t worker)
{
    if ((unsigned)worker >= SENSORARRAY_WORK_BUDGET_WORKER_COUNT ||
        xPortGetCoreID() != 0) { return; }
    uint64_t now = (uint64_t)esp_timer_get_time();
    sensorarrayCpuOwnerSlot_t *slot = &s_cpuOwners[worker];
    portENTER_CRITICAL(&s_cpuOwnerMux);
    /* A task cannot recursively enter this observer in production: the
     * unlocked portion only calls timer/kernel APIs, with no app callback.
     * An overlapping entry therefore violates the one-owner role contract.
     * Do not wait for that sampler or silently replace its pending sample. */
    if (slot->sampling) {
        slot->value.conflict = true;
    }
    bool due = !slot->sampling && !slot->value.conflict &&
        !slot->value.counterRegressed && !slot->value.clockRegressed;
    if (due && slot->value.sampleKnown) {
        if (now < slot->value.observedAtUs) {
            slot->value.clockRegressed = true;
            due = false;
        } else {
            due = now - slot->value.observedAtUs >= SENSORARRAY_CPU_OWNER_SAMPLE_US;
        }
    }
    if (due) { slot->sampling = true; }
    portEXIT_CRITICAL(&s_cpuOwnerMux);
    if (!due) { return; }

    TaskStatus_t status = {0};
    uint64_t t0 = (uint64_t)esp_timer_get_time();
    /* The dual-core IDF kernel copies U64 under the same kernel mux used by
     * switch-context accounting. NULL is the calling owner. pdFALSE avoids
     * stack scanning; eReady is
     * only a fast-path placeholder and is deliberately never published. */
    vTaskGetInfo(NULL, &status, pdFALSE, eReady);
    uint64_t t1 = (uint64_t)esp_timer_get_time();
    uint32_t taskNumber = (uint32_t)status.xTaskNumber;
    uint64_t runtime = (uint64_t)status.ulRunTimeCounter;

    portENTER_CRITICAL(&s_cpuOwnerMux);
    if (slot->value.sampleKnown && slot->value.taskNumber != taskNumber) {
        slot->value.conflict = true;
        slot->value.conflictingTaskNumber = taskNumber;
    } else if (slot->value.sampleKnown && runtime < slot->value.runUs) {
        slot->value.counterRegressed = true;
    } else if (t1 < t0) {
        slot->value.clockRegressed = true;
    } else {
        slot->value.sampleKnown = true;
        slot->value.taskNumber = taskNumber;
        slot->value.runUs = runtime;
        slot->value.t0Us = t0;
        slot->value.t1Us = t1;
        slot->value.observedAtUs = t1;
    }
    slot->sampling = false;
    portEXIT_CRITICAL(&s_cpuOwnerMux);
    /* No status pointer (name, stack, handle) outlives this local sample. */
}
#else
void sensorarrayCpuOwnerCheckpoint(sensorarrayWorkBudgetWorker_t worker)
{
    (void)worker;
}
#endif

bool sensorarrayCpuOwnerCopySnapshot(sensorarrayWorkBudgetWorker_t worker,
                                    sensorarrayCpuOwnerSnapshot_t *snapshot)
{
    if (!snapshot || (unsigned)worker >= SENSORARRAY_WORK_BUDGET_WORKER_COUNT) {
        return false;
    }
    *snapshot = (sensorarrayCpuOwnerSnapshot_t){.stale = true};
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    portENTER_CRITICAL(&s_cpuOwnerMux);
    *snapshot = s_cpuOwners[worker].value;
    portEXIT_CRITICAL(&s_cpuOwnerMux);
    snapshot->enabled = true;
    snapshot->available = snapshot->sampleKnown && !snapshot->conflict &&
        !snapshot->counterRegressed && !snapshot->clockRegressed;
    snapshot->stale = true;
    if (snapshot->sampleKnown) {
        /* Read time after copying: a concurrent newer publication cannot
         * produce an unsigned age from a sample in the reader's future. */
        uint64_t now = (uint64_t)esp_timer_get_time();
        snapshot->ageKnown = now >= snapshot->observedAtUs;
        if (snapshot->ageKnown) {
            snapshot->ageUs = now - snapshot->observedAtUs;
            snapshot->stale = snapshot->ageUs >= SENSORARRAY_CPU_OWNER_STALE_US;
        } else {
            /* Observation-only rejection, with no control-side mutation. */
            snapshot->clockRegressed = true;
            snapshot->available = false;
        }
    }
#endif
    return true;
}

bool sensorarrayCpuOwnerHandles(const char *command)
{
    return command && strncmp(command, "CPUOWNER?", 9u) == 0;
}

static const char *sensorarrayCpuOwnerReason(const sensorarrayCpuOwnerSnapshot_t *s)
{
    if (!s->enabled) { return "disabled"; }
    if (s->conflict) { return "writer_conflict"; }
    if (s->counterRegressed) { return "counter_regressed"; }
    if (s->clockRegressed) { return "clock_regressed"; }
    return s->sampleKnown ? "ok" : "not_observed";
}

static void sensorarrayCpuOwnerNumber(char text[21], bool known, uint64_t value)
{
    if (known) { (void)snprintf(text, 21u, "%llu", (unsigned long long)value); }
    else { (void)snprintf(text, 21u, "unknown"); }
}

esp_err_t sensorarrayCpuOwnerFormat(const char *command, uint64_t session,
                                    char *response, size_t size)
{
    if (!command || !response || size == 0u) { return ESP_ERR_INVALID_ARG; }
    if (!sensorarrayCpuOwnerHandles(command) || command[9] < '0' ||
        command[9] > '7' || command[10] != '\0') {
        int n = snprintf(response, size, "ERR,cmd=CPUOWNER,reason=role_0_to_7\n");
        return n > 0 && (size_t)n < size ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_SIZE;
    }
    unsigned role = (unsigned)(command[9] - '0');
    sensorarrayCpuOwnerSnapshot_t s;
    (void)sensorarrayCpuOwnerCopySnapshot((sensorarrayWorkBudgetWorker_t)role, &s);
    char task[21], run[21], t0[21], t1[21], observed[21], age[21];
    sensorarrayCpuOwnerNumber(task, s.sampleKnown, s.taskNumber);
    sensorarrayCpuOwnerNumber(run, s.available, s.runUs);
    sensorarrayCpuOwnerNumber(t0, s.sampleKnown, s.t0Us);
    sensorarrayCpuOwnerNumber(t1, s.sampleKnown, s.t1Us);
    sensorarrayCpuOwnerNumber(observed, s.sampleKnown, s.observedAtUs);
    sensorarrayCpuOwnerNumber(age, s.ageKnown, s.ageUs);
    int n = snprintf(response, size,
        "CPUOWNER,v=1,session=%016llX,role=%u,enabled=%u,available=%u,known=%u,conflict=%u,stale=%u,reason=%s,taskNumber=%s,runUs=%s,t0Us=%s,t1Us=%s,observedAtUs=%s,ageUs=%s,sampleMinUs=1000000,staleAfterUs=3000000,scope=switch_out,isr=may_include,sdk=not_isolated,blocked=unknown\n",
        (unsigned long long)session, role, s.enabled, s.available, s.sampleKnown,
        s.conflict, s.stale, sensorarrayCpuOwnerReason(&s), task, run, t0, t1,
        observed, age);
    return n > 0 && (size_t)n < size ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
