/* Independent synthetic scheduler; includes the unmodified source under test.
 * One modeled work unit ends at a real production checkpoint. Actual caller
 * boundaries differ by at most one unit; SDK execution is not bounded here. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayWorkBudget.c"

static uint32_t nowTick, delayCalls;
static esp_freertos_idle_cb_t hook;
static int core;
static bool registrationFails, scheduling;
static uint64_t modelUs, blockedUntil[SENSORARRAY_WORK_BUDGET_WORKER_COUNT];
static unsigned currentWorker;
TickType_t xTaskGetTickCount(void) { return nowTick; }
int xPortGetCoreID(void) { return core; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1u);
    delayCalls++;
    if (scheduling) {
        blockedUntil[currentWorker] = (modelUs / 10000u + 1u) * 10000u;
    } else { nowTick++; }
}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t cpu)
{
    assert(cpu == 0u);
    if (registrationFails) { return ESP_ERR_NO_MEM; }
    hook = cb; return ESP_OK;
}
static void reset(void)
{
    nowTick = delayCalls = 0u; core = 0; hook = NULL;
    registrationFails = scheduling = false;
    workBudgetTestStorageInternal = true;
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    memset(s_workers, 0, sizeof(s_workers));
    memset(blockedUntil, 0, sizeof(blockedUntil));
}
static void staleEpoch(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK);
    for (unsigned cycle = 0; cycle < 100u; ++cycle) {
        nowTick += 1u;
        /* Real callback observations, not a callback invented by vTaskDelay. */
        for (unsigned n = 0; n < 500u; ++n) { assert(hook()); }
        nowTick += 2u; /* Other higher-priority work before this task resumes. */
        for (unsigned unit = 0; unit < 4u; ++unit) {
            (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX);
        }
    }
    printf("{\"case\":\"stale_epoch_four_units\",\"checkpoints\":400,\"yields\":%u,\"idleCallbacks\":%u}\n",
           delayCalls, s_idleGeneration);
}
static void model(unsigned unitUs, unsigned phaseUs, unsigned sdkBusyUs)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(hook());
    scheduling = true; modelUs = phaseUs;
    uint64_t end = modelUs + 2000000u, nextSdk = 50000u;
    const unsigned priority[] = {5u, 5u, 7u, 4u, 6u, 4u, 5u};
    unsigned work[SENSORARRAY_WORK_BUDGET_WORKER_COUNT] = {0u};
    unsigned idleVisits = 0u, roundRobin = 0u;
    while (modelUs < end) {
        if (sdkBusyUs && modelUs >= nextSdk) {
            modelUs += sdkBusyUs; nextSdk += 50000u;
        }
        int selected = -1;
        for (unsigned n = 0; n < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++n) {
            unsigned i = (n + roundRobin) % SENSORARRAY_WORK_BUDGET_WORKER_COUNT;
            if (blockedUntil[i] <= modelUs &&
                (selected < 0 || priority[i] > priority[selected])) { selected = (int)i; }
        }
        if (selected < 0) {
            nowTick = (uint32_t)(modelUs / 10000u); assert(hook()); idleVisits++;
            uint64_t next = UINT64_MAX;
            for (unsigned i = 0; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) {
                if (blockedUntil[i] < next) { next = blockedUntil[i]; }
            }
            assert(next > modelUs); modelUs = next; continue;
        }
        currentWorker = (unsigned)selected;
        roundRobin = (currentWorker + 1u) % SENSORARRAY_WORK_BUDGET_WORKER_COUNT;
        modelUs += unitUs; nowTick = (uint32_t)(modelUs / 10000u);
        work[currentWorker]++;
        (void)sensorarrayWorkBudgetCheckpoint((sensorarrayWorkBudgetWorker_t)currentWorker);
    }
    sensorarrayWorkBudgetSnapshot_t snap;
    sensorarrayWorkBudgetCopySnapshot(&snap);
    printf("{\"case\":\"seven_ready\",\"unitUs\":%u,\"phaseUs\":%u,\"sdkBusyUsPer50ms\":%u,\"idleVisits\":%u,\"maxIdleGapTicks\":%u,\"yields\":%u,\"work\":[",
           unitUs, phaseUs, sdkBusyUs, idleVisits, snap.maxIdleGapTicks, delayCalls);
    for (unsigned i = 0; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) {
        assert(work[i] > 0u);
        printf("%s%u", i ? "," : "", work[i]);
    }
    puts("]}");
}
static void noProgress(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(hook());
    nowTick = 60u; /* No CPU0 application checkpoint or idle callback for 600ms. */
    core = 1;
    sensorarrayWorkBudgetSnapshot_t snapshot; sensorarrayWorkBudgetCopySnapshot(&snapshot);
    assert(snapshot.idleGeneration == 1u && snapshot.idleAgeTicks == 60u);
    core = 0;
    for (unsigned i = 0; i < 100u; ++i) {
        unsigned before = delayCalls;
        assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_CTRL));
        assert(delayCalls == before + 1u && s_idleGeneration == 1u);
    }
    puts("{\"case\":\"sdk_no_idle_cpu0_no_progress\",\"calls\":100,\"bounded_one_tick_returns\":100,\"fabricated_idle\":0}");
    reset(); registrationFails = true;
    assert(sensorarrayWorkBudgetInit() == ESP_ERR_NO_MEM); nowTick = 4u;
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_WIFI_CTRL));
    assert(!hook && !s_idleGeneration);
    puts("{\"case\":\"hook_failure\",\"bounded_one_tick_returns\":1,\"fabricated_idle\":0}");
}
int main(void)
{
    staleEpoch();
    for (unsigned phase = 0; phase < 2u; ++phase) {
        for (unsigned sdk = 0; sdk < 2u; ++sdk) {
            model(100u, phase * 9500u, sdk * 20000u);
            model(500u, phase * 9500u, sdk * 20000u);
            model(1000u, phase * 9500u, sdk * 20000u);
        }
    }
    noProgress();
    return 0;
}
