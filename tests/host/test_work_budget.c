/* Executes the production component; all time/idle events below are synthetic. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayWorkBudget.c"

static uint32_t nowTick, delayCalls, registrationCalls, largestDelay;
static int core;
static bool registrationFails, injectCrossCoreIdle, insideHook;
static esp_freertos_idle_cb_t registeredHook;
static bool schedulerModel;
static unsigned modelWorker;
static uint64_t modelUs, blockedUntil[SENSORARRAY_WORK_BUDGET_WORKER_COUNT];

TickType_t xTaskGetTickCount(void)
{
    uint32_t result = nowTick;
    /* Query read races with a CPU0 idle callback in the next tick. Returning
     * the old tick catches the former now-before-reference underflow. */
    if (injectCrossCoreIdle && !insideHook) {
        injectCrossCoreIdle = false;
        nowTick++;
        insideHook = true;
        assert(registeredHook && registeredHook());
        insideHook = false;
    }
    return result;
}
int xPortGetCoreID(void) { return core; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1u); /* A checkpoint never loops or asks for > one tick. */
    delayCalls++;
    if (ticks > largestDelay) { largestDelay = ticks; }
    if (schedulerModel) {
        blockedUntil[modelWorker] = ((modelUs / 10000u) + ticks) * 10000u;
    } else {
        nowTick += ticks;
    }
}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t cpu)
{
    registrationCalls++;
    assert(cpu == 0u && cb);
    if (registrationFails) { return ESP_ERR_NO_MEM; }
    registeredHook = cb;
    return ESP_OK;
}
static void reset(void)
{
    nowTick = delayCalls = registrationCalls = largestDelay = 0u;
    core = 0;
    registrationFails = injectCrossCoreIdle = insideHook = schedulerModel = false;
    registeredHook = NULL;
    workBudgetTestStorageInternal = true;
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    memset(s_workers, 0, sizeof(s_workers));
    memset(blockedUntil, 0, sizeof(blockedUntil));
}
static void testIdleAndBudget(void)
{
    reset();
    assert(sensorarrayWorkBudgetInit() == ESP_OK);
    assert(sensorarrayWorkBudgetInit() == ESP_OK && registrationCalls == 1u);
    sensorarrayWorkBudgetSnapshot_t snap;
    sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(snap.registered && !snap.idleKnown);
    assert(snap.idleAgeTicks == UINT32_MAX && snap.maxIdleGapTicks == UINT32_MAX);
    for (unsigned i = 0u; i < 31u; ++i) {
        assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_ASYNC_LOG));
    }
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_ASYNC_LOG));
    assert(delayCalls == 1u && largestDelay == 1u);
    assert(registeredHook());
    nowTick += 2u;
    unsigned before = delayCalls;
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX));
    assert(delayCalls == before + 1u); /* No idle occurs in delay: still returns. */
    sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(snap.tickHz == 100u && snap.idleBudgetTicks == 2u && snap.maxWorkUnits == 32u);
    assert(snap.idleKnown && snap.idleGeneration == 1u && snap.idleAgeTicks == 3u);
    assert(snap.maxIdleGapTicks == 3u && snap.workers[2].maxBurstUnits == 32u);
    core = 1;
    before = delayCalls;
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX));
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_WORKER_COUNT));
    assert(delayCalls == before); /* Does not cap Core1 acquisition. */
}
static void testFailureAndNoIdle(void)
{
    reset(); registrationFails = true;
    assert(sensorarrayWorkBudgetInit() == ESP_ERR_NO_MEM);
    assert(sensorarrayWorkBudgetInit() == ESP_ERR_NO_MEM && registrationCalls == 1u);
    nowTick = 3u;
    for (unsigned i = 0u; i < 100u; ++i) {
        unsigned before = delayCalls;
        assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
        assert(delayCalls == before + 1u);
    }
    sensorarrayWorkBudgetSnapshot_t snap;
    sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(!snap.registered && !snap.idleKnown && snap.idleGeneration == 0u);
    assert(snap.initError == ESP_ERR_NO_MEM && snap.maxIdleGapTicks == UINT32_MAX);
    /* No callback, mutex, WD reset or synthetic "idle" is used as fallback. */
    assert(!registeredHook);
}
static void testCrossCoreAndWrap(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK);
    nowTick = 7u; assert(registeredHook());
    core = 1;
    injectCrossCoreIdle = true;
    sensorarrayWorkBudgetSnapshot_t snap;
    sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(nowTick == 8u && snap.idleAgeTicks == 0u && snap.maxIdleGapTicks == 1u);
    assert(snap.idleGeneration == 1u); /* Deliberately non-atomic stats snapshot. */
    core = 0;
    nowTick = UINT32_MAX - 1u;
    s_idleTick = nowTick; s_idleGeneration = UINT32_MAX;
    s_maxIdleGapTicks = 0u;
    nowTick = 1u;
    assert(registeredHook());
    sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(snap.idleGeneration == 1u && snap.idleAgeTicks == 0u && snap.maxIdleGapTicks == 3u);
    nowTick = 3u;
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT));
    assert(delayCalls == 1u);
}
static void testUnsafeStorageRejected(void)
{
    reset(); workBudgetTestStorageInternal = false;
    assert(sensorarrayWorkBudgetInit() == ESP_ERR_INVALID_STATE);
    assert(registrationCalls == 0u && registeredHook == NULL);
    sensorarrayWorkBudgetSnapshot_t snap;
    sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(snap.initError == ESP_ERR_INVALID_STATE && !snap.registered && !snap.idleKnown);
    assert(snap.idleAgeTicks == UINT32_MAX && snap.maxIdleGapTicks == UINT32_MAX);
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_ASYNC_LOG));
    assert(delayCalls == 1u && s_workers[SENSORARRAY_WORK_BUDGET_ASYNC_LOG].checkpoints == 0u);
    /* Even a word-aligned internal region must cover its whole object. */
    workBudgetTestStorageInternal = true;
    uint32_t words[2] = {0u};
    assert(sensorarrayWorkBudgetInternal(words, sizeof(words)));
    assert(!sensorarrayWorkBudgetInternal((uint8_t *)words + 1u, sizeof(uint32_t)));
    assert(!sensorarrayWorkBudgetInternal(words, 1u));
}
static void testSevenContinuouslyReadyWorkers(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(registeredHook());
    schedulerModel = true; modelUs = 0u;
    unsigned work[SENSORARRAY_WORK_BUDGET_WORKER_COUNT] = {0u};
    const unsigned priority[SENSORARRAY_WORK_BUDGET_WORKER_COUNT] = {5u, 5u, 7u, 4u, 6u, 4u, 5u};
    unsigned roundRobin = 0u, idleVisits = 0u;
    while (modelUs < 2000000u) {
        int selected = -1;
        for (unsigned n = 0u; n < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++n) {
            unsigned i = (n + roundRobin) % SENSORARRAY_WORK_BUDGET_WORKER_COUNT;
            if (blockedUntil[i] <= modelUs &&
                (selected < 0 || priority[i] > priority[selected])) {
                selected = (int)i;
            }
        }
        if (selected < 0) {
            nowTick = (uint32_t)(modelUs / 10000u);
            assert(registeredHook()); idleVisits++;
            uint64_t next = UINT64_MAX;
            for (unsigned i = 0u; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) {
                if (blockedUntil[i] < next) { next = blockedUntil[i]; }
            }
            assert(next > modelUs); modelUs = next;
            continue;
        }
        modelWorker = (unsigned)selected;
        roundRobin = (modelWorker + 1u) % SENSORARRAY_WORK_BUDGET_WORKER_COUNT;
        /* Worst-case bounded work-unit cost for this deterministic model only.
         * Real SDK call latency remains a HIL measurement, not this assertion. */
        modelUs += 500u;
        nowTick = (uint32_t)(modelUs / 10000u);
        work[modelWorker]++;
        (void)sensorarrayWorkBudgetCheckpoint((sensorarrayWorkBudgetWorker_t)modelWorker);
    }
    sensorarrayWorkBudgetSnapshot_t snap;
    sensorarrayWorkBudgetCopySnapshot(&snap);
    for (unsigned i = 0u; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) { assert(work[i] > 0u && snap.workers[i].yields > 0u); }
    assert(idleVisits >= 50u && snap.maxIdleGapTicks <= 3u && largestDelay == 1u);
    printf("WORK_BUDGET_SCHEDULER_MODEL,tickHz=100,workers=7,unitUs=500,idleVisits=%u,maxIdleGapTicks=%u\n",
           idleVisits, snap.maxIdleGapTicks);
}
int main(void)
{
    testIdleAndBudget();
    testFailureAndNoIdle();
    testCrossCoreAndWrap();
    testUnsafeStorageRejected();
    testSevenContinuouslyReadyWorkers();
    puts("WORK_BUDGET_PASS,scenarios=5,production_component=1,hardware=NOT_RUN");
    return 0;
}
