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
    unsigned before = delayCalls;
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX));
    assert(delayCalls == before);
    nowTick += 2u; /* Local quantum, while the real-idle age is still < four. */
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX));
    assert(delayCalls == before + 1u); /* No idle occurs in delay: still returns. */
    sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(snap.tickHz == 100u && snap.idleBudgetTicks == 4u && snap.maxWorkUnits == 32u);
    assert(snap.graceUnits == 0u && snap.backstopTicks == 4u && snap.localQuantumTicks == 2u);
    assert(snap.idleKnown && snap.idleGeneration == 1u && snap.idleAgeTicks == 3u);
    assert(snap.maxIdleGapTicks == 3u && snap.workers[2].maxBurstUnits == 32u);
    core = 1;
    before = delayCalls;
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX));
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_WORKER_COUNT));
    assert(delayCalls == before); /* Does not cap Core1 acquisition. */
}
static void testDelayedWorkerFourUnitBurst(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK);
    unsigned yielded = 0u;
    for (unsigned i = 0u; i < 100u; ++i) {
        nowTick++;
        assert(registeredHook()); /* Actual callback observation, not a yield. */
        nowTick += 2u; /* Lower-priority worker resumes two ticks later. */
        for (unsigned unit = 0u; unit < 4u; ++unit) {
            yielded += sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX) ? 1u : 0u;
        }
    }
    /* The old actual component yielded 300/400 times here despite genuine
     * idle progress. The task has done no local two-tick quantum of work. */
    assert(yielded == 0u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_SERIAL_TX].checkpoints == 400u);
}
static void testFailureAndNoIdle(void)
{
    reset(); registrationFails = true;
    assert(sensorarrayWorkBudgetInit() == ESP_ERR_NO_MEM);
    assert(sensorarrayWorkBudgetInit() == ESP_ERR_NO_MEM && registrationCalls == 1u);
    nowTick = 2u;
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
    assert(delayCalls == 0u && s_idleGeneration == 0u); /* No fabricated idle. */
    nowTick = 4u; /* No idle and beyond the hard starvation backstop. */
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
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT));
    nowTick = 5u;
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
static void testOwnYieldRestartsLocalOnly(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(registeredHook());
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
    nowTick = 2u;
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
    assert(nowTick == 3u && s_idleGeneration == 1u && s_idleTick == 0u);
    /* Own block resets the local account, never the real-idle age. */
    for (unsigned i = 0u; i < 31u; ++i) {
        assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
    }
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
    assert(nowTick == 4u && s_idleGeneration == 1u && s_idleTick == 0u);
    for (unsigned i = 0u; i < 100u; ++i) {
        unsigned before = delayCalls;
        assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
        assert(delayCalls == before + 1u && s_idleGeneration == 1u);
    }
    /* A real idle restarts both evidence and the local work account. */
    assert(registeredHook()); nowTick += 2u;
    for (unsigned i = 0u; i < 4u; ++i) {
        assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_SERIAL_TX));
    }
}
static void testStarvationBackstopBypassesNewWorkerQuantum(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(registeredHook());
    nowTick = 4u;
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX));
    assert(delayCalls == 1u && !s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].quantumActive);
    assert(s_idleGeneration == 1u && s_idleTick == 0u);
}
static void testSevenContinuouslyReadyWorkers(unsigned unitUs, unsigned tickPhaseUs)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(registeredHook());
    schedulerModel = true; modelUs = tickPhaseUs;
    unsigned work[SENSORARRAY_WORK_BUDGET_WORKER_COUNT] = {0u};
    const unsigned priority[SENSORARRAY_WORK_BUDGET_WORKER_COUNT] = {5u, 5u, 7u, 4u, 6u, 4u, 5u};
    unsigned roundRobin = 0u, idleVisits = 0u;
    while (modelUs < tickPhaseUs + 2000000u) {
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
        modelUs += unitUs;
        nowTick = (uint32_t)(modelUs / 10000u);
        work[modelWorker]++;
        (void)sensorarrayWorkBudgetCheckpoint((sensorarrayWorkBudgetWorker_t)modelWorker);
    }
    sensorarrayWorkBudgetSnapshot_t snap;
    sensorarrayWorkBudgetCopySnapshot(&snap);
    for (unsigned i = 0u; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) { assert(work[i] > 0u && snap.workers[i].yields > 0u); }
    assert(idleVisits >= 20u && snap.maxIdleGapTicks <= 5u && largestDelay == 1u);
    printf("WORK_BUDGET_SCHEDULER_MODEL,tickHz=100,workers=7,unitUs=%u,phaseUs=%u,idleVisits=%u,maxIdleGapTicks=%u\n",
           unitUs, tickPhaseUs, idleVisits, snap.maxIdleGapTicks);
}
static void testFiniteSevenWorkerLoad(unsigned unitUs, unsigned phaseUs)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(registeredHook());
    schedulerModel = true; modelUs = phaseUs;
    uint64_t end = modelUs + 2000000u, nextSdk = phaseUs + 50000u, nextArrival = modelUs;
    const unsigned priority[] = {5u, 5u, 7u, 4u, 6u, 4u, 5u};
    unsigned work[SENSORARRAY_WORK_BUDGET_WORKER_COUNT] = {0u};
    unsigned pending[SENSORARRAY_WORK_BUDGET_WORKER_COUNT] = {0u};
    unsigned arrivals = 0u, idleVisits = 0u, roundRobin = 0u;
    while (modelUs < end) {
        if (modelUs >= nextSdk) { modelUs += 20000u; nextSdk += 50000u; }
        while (nextArrival <= modelUs && nextArrival < end) {
            arrivals++;
            for (unsigned i = 0; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) { pending[i]++; }
            nextArrival += 10000u;
        }
        int selected = -1;
        for (unsigned n = 0; n < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++n) {
            unsigned i = (n + roundRobin) % SENSORARRAY_WORK_BUDGET_WORKER_COUNT;
            if (pending[i] && blockedUntil[i] <= modelUs &&
                (selected < 0 || priority[i] > priority[selected])) { selected = (int)i; }
        }
        if (selected < 0) {
            nowTick = (uint32_t)(modelUs / 10000u); assert(registeredHook()); idleVisits++;
            uint64_t next = nextArrival < nextSdk ? nextArrival : nextSdk;
            for (unsigned i = 0; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) {
                if (pending[i] && blockedUntil[i] < next) { next = blockedUntil[i]; }
            }
            assert(next > modelUs); modelUs = next; continue;
        }
        modelWorker = (unsigned)selected;
        roundRobin = (modelWorker + 1u) % SENSORARRAY_WORK_BUDGET_WORKER_COUNT;
        modelUs += unitUs; nowTick = (uint32_t)(modelUs / 10000u);
        pending[modelWorker]--; work[modelWorker]++;
        (void)sensorarrayWorkBudgetCheckpoint((sensorarrayWorkBudgetWorker_t)modelWorker);
    }
    sensorarrayWorkBudgetSnapshot_t snap; sensorarrayWorkBudgetCopySnapshot(&snap);
    printf("{\"case\":\"finite_100Hz_7workers_sdk40percent\",\"unitUs\":%u,\"phaseUs\":%u,\"arrivalsPerWorker\":%u,\"idleVisits\":%u,\"maxIdleGapTicks\":%u,\"yields\":%u,\"work\":[",
           unitUs, phaseUs, arrivals, idleVisits, snap.maxIdleGapTicks, delayCalls);
    for (unsigned i = 0; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) { printf("%s%u", i ? "," : "", work[i]); }
    puts("]}");
    for (unsigned i = 0u; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) {
        assert(work[i] == arrivals && pending[i] == 0u); /* No artificial sink rate limit. */
    }
    assert(arrivals == 200u && idleVisits > 0u && snap.maxIdleGapTicks <= 4u);
    if (unitUs == 100u) { assert(delayCalls == 0u); } /* Old global-age gate added 273--351. */
}

static void testControlFloodMaxUnits(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(registeredHook());
    for (unsigned unit = 1u; unit <= 128u; ++unit) {
        bool yielded = sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_CTRL);
        assert(yielded == (unit % 32u == 0u));
    }
    assert(delayCalls == 4u && s_idleGeneration == 1u && s_idleTick == 0u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_CTRL].maxBurstUnits == 32u);
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_CTRL));
    assert(delayCalls == 5u); /* Global hard backstop now wins despite a fresh local budget. */
}
static void testNoCpu0OrIdleProgressIsNotFabricated(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK); assert(registeredHook());
    nowTick = 60u; core = 1; /* No CPU0 checkpoint ran in this synthetic 600ms gap. */
    sensorarrayWorkBudgetSnapshot_t snap; sensorarrayWorkBudgetCopySnapshot(&snap);
    assert(snap.idleGeneration == 1u && snap.idleAgeTicks == 60u && snap.maxIdleGapTicks == 60u);
    core = 0;
    for (unsigned call = 0u; call < 100u; ++call) {
        unsigned before = delayCalls;
        assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_WIFI_CTRL));
        assert(delayCalls == before + 1u && s_idleGeneration == 1u && s_idleTick == 0u);
    }
    /* Cannot force an unresponsive CPU or an independent SDK task to yield;
     * when called again, this API returns after one block, never fakes idle. */
}
static void testLocalQuantumAndGlobalAgeWrap(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK);
    nowTick = UINT32_MAX - 1u; assert(registeredHook());
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT));
    nowTick = 0u;
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT));
    assert(nowTick == 1u && s_idleTick == UINT32_MAX - 1u && s_idleGeneration == 1u);
    assert(!sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT));
    nowTick = 2u;
    assert(sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT));
    assert(delayCalls == 2u); /* Separate local elapsed and real-idle unsigned wrap. */
}

int main(void)
{
    testDelayedWorkerFourUnitBurst();
    testIdleAndBudget();
    testFailureAndNoIdle();
    testCrossCoreAndWrap();
    testUnsafeStorageRejected();
    testOwnYieldRestartsLocalOnly();
    testStarvationBackstopBypassesNewWorkerQuantum();
    testControlFloodMaxUnits();
    testNoCpu0OrIdleProgressIsNotFabricated();
    testLocalQuantumAndGlobalAgeWrap();
    for (unsigned phase = 0u; phase < 2u; ++phase) {
        testSevenContinuouslyReadyWorkers(100u, phase * 9500u);
        testSevenContinuouslyReadyWorkers(500u, phase * 9500u);
        testSevenContinuouslyReadyWorkers(1000u, phase * 9500u);
        testFiniteSevenWorkerLoad(100u, phase * 9500u);
        testFiniteSevenWorkerLoad(500u, phase * 9500u);
    }
    puts("WORK_BUDGET_PASS,scenarios=20,production_component=1,hardware=NOT_RUN");
    return 0;
}
