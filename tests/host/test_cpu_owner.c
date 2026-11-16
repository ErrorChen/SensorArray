/* Synthetic scheduler/clock, actual production observation and budget code. */
#include <assert.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayCpuOwner.c"
#include "sensorarrayWorkBudget.c"

static _Atomic uint64_t fakeUs;
static _Thread_local unsigned lockDepth;
static _Thread_local uint32_t fakeTaskNumber = 11u;
static _Thread_local uint64_t fakeRuntime;
static unsigned getInfoCalls, delayCalls, budgetTick;
static _Atomic unsigned timerCalls;
static bool nestedQuery, nestedWriter;
static int fakeCore;
static esp_freertos_idle_cb_t idleHook;
static const uint64_t session = UINT64_C(0x123456789ABCDEF0);

void testEnter(portMUX_TYPE *mux) { assert(pthread_mutex_lock(mux) == 0); lockDepth++; }
void testExit(portMUX_TYPE *mux) { assert(lockDepth); lockDepth--; assert(pthread_mutex_unlock(mux) == 0); }
int64_t esp_timer_get_time(void) { assert(lockDepth == 0u); timerCalls++; return (int64_t)atomic_load(&fakeUs); }
int xPortGetCoreID(void) { return fakeCore; }
TickType_t xTaskGetTickCount(void) { return budgetTick; }
void vTaskDelay(TickType_t ticks) { assert(lockDepth == 0u && ticks == 1u); delayCalls++; budgetTick += ticks; }
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t core)
{ assert(core == 0u); idleHook = cb; return ESP_OK; }

#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
void vTaskGetInfo(TaskHandle_t task, TaskStatus_t *status, int getStack, eTaskState state)
{
    assert(task == NULL && getStack == pdFALSE && state == eReady && lockDepth == 0u);
    getInfoCalls++;
    if (nestedQuery) {
        nestedQuery = false;
        sensorarrayCpuOwnerSnapshot_t s;
        assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
        assert(!s.sampleKnown); /* Publication is not half-written. */
    }
    if (nestedWriter) {
        nestedWriter = false;
        sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    }
    status->xTaskNumber = fakeTaskNumber;
    status->ulRunTimeCounter = fakeRuntime;
    status->xHandle = (void *)(uintptr_t)0xdead;
    status->pcTaskName = (char *)(uintptr_t)0xbeef;
}
#endif

static void reset(void)
{
    assert(lockDepth == 0u);
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    memset(s_cpuOwners, 0, sizeof(s_cpuOwners));
#endif
    atomic_store(&fakeUs, 0u);
    fakeTaskNumber = 11u; fakeRuntime = 0u; getInfoCalls = timerCalls = delayCalls = budgetTick = 0u;
    nestedQuery = nestedWriter = false; fakeCore = 0; idleHook = NULL;
    workBudgetTestStorageInternal = true;
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    memset(s_workers, 0, sizeof(s_workers));
}

static void testUnknownAndOff(void)
{
    reset();
    sensorarrayCpuOwnerSnapshot_t s;
    char reply[512];
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
    assert(!s.sampleKnown && !s.available && s.stale && !s.ageKnown);
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", session, reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "session=123456789ABCDEF0") && strstr(reply, "runUs=unknown"));
    assert(strstr(reply, "ageUs=unknown") && strstr(reply, "scope=switch_out"));
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    assert(s.enabled && strstr(reply, "reason=not_observed"));
#else
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    assert(!s.enabled && strstr(reply, "reason=disabled"));
    assert(getInfoCalls == 0 && timerCalls == 0);
#endif
    assert(!sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_WORKER_COUNT, &s));
    assert(!sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, NULL));
    assert(!sensorarrayCpuOwnerHandles(NULL) && !sensorarrayCpuOwnerHandles("CPUOWNER=0"));
    const char *bad[] = {"CPUOWNER?", "CPUOWNER?7", "CPUOWNER?00", "CPUOWNER?-1", "CPUOWNER?1x"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++) {
        assert(sensorarrayCpuOwnerFormat(bad[i], session, reply, sizeof(reply)) == ESP_ERR_INVALID_ARG);
    }
}

#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
static void testSparseStaleAndWide(void)
{
    reset(); nestedQuery = true;
    fakeRuntime = UINT64_C(0x100000012);
    atomic_store(&fakeUs, UINT64_C(0x100000000));
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    assert(getInfoCalls == 1u);
    sensorarrayCpuOwnerSnapshot_t s;
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
    assert(s.available && s.sampleKnown && !s.stale && s.runUs == fakeRuntime);
    assert(s.t0Us == UINT64_C(0x100000000) && s.t1Us == s.t0Us && s.observedAtUs == s.t1Us);
    atomic_fetch_add(&fakeUs, SENSORARRAY_CPU_OWNER_SAMPLE_US - 1u);
    for (unsigned i = 0; i < 64u; i++) { sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX); }
    assert(getInfoCalls == 1u);
    atomic_fetch_add(&fakeUs, 1u); fakeRuntime += UINT64_C(0x100000000);
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    assert(getInfoCalls == 2u);
    atomic_fetch_add(&fakeUs, SENSORARRAY_CPU_OWNER_STALE_US);
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
    assert(s.available && s.stale && s.ageUs == SENSORARRAY_CPU_OWNER_STALE_US);
    char reply[512]; assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", session, reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "runUs=8589934610") && strstr(reply, "stale=1") && strstr(reply, "blocked=unknown"));
    unsigned calls = getInfoCalls;
    fakeCore = 1; sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    fakeCore = 0; sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_WORKER_COUNT);
    assert(calls == getInfoCalls && delayCalls == 0u);
    atomic_store(&fakeUs, 1u);
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
    assert(!s.available && !s.ageKnown && s.clockRegressed && s.stale && s.sampleKnown);
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", session, reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "reason=clock_regressed") && strstr(reply, "ageUs=unknown") && strstr(reply, "runUs=unknown"));
}

static void testWriterConflictAndRegression(void)
{
    reset(); fakeRuntime = 100u;
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    atomic_store(&fakeUs, SENSORARRAY_CPU_OWNER_SAMPLE_US);
    fakeTaskNumber = 12u; fakeRuntime = 999u;
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    sensorarrayCpuOwnerSnapshot_t s;
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
    assert(s.conflict && !s.available && s.taskNumber == 11u && s.conflictingTaskNumber == 12u && s.runUs == 100u);
    unsigned calls = getInfoCalls;
    fakeTaskNumber = 11u; atomic_store(&fakeUs, UINT64_C(9000000));
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    assert(getInfoCalls == calls); /* Original owner cannot clear conflict. */
    char reply[512]; assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", session, reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "reason=writer_conflict") && strstr(reply, "runUs=unknown"));
    reset(); nestedWriter = true;
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s) && s.conflict && !s.available);
    assert(getInfoCalls == 1u); /* Overlapping writer cannot replace pending owner. */
    reset(); fakeRuntime = UINT64_C(0x100000005);
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    atomic_store(&fakeUs, SENSORARRAY_CPU_OWNER_SAMPLE_US); fakeRuntime = 4u;
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s) && s.counterRegressed && !s.available);
}

static void testCounterCrossing(void)
{
    reset(); fakeRuntime = (uint64_t)UINT32_MAX - 3u;
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    atomic_store(&fakeUs, SENSORARRAY_CPU_OWNER_SAMPLE_US);
    fakeRuntime += 10u;
    sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    sensorarrayCpuOwnerSnapshot_t s;
    assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
    assert(s.available && !s.counterRegressed && s.runUs == (uint64_t)UINT32_MAX + 7u);
}

static _Atomic bool writerDone;
static void *writer(void *unused)
{
    (void)unused; fakeTaskNumber = 77u;
    for (uint64_t i = 1u; i <= 2000u; i++) {
        atomic_store(&fakeUs, i * SENSORARRAY_CPU_OWNER_SAMPLE_US);
        fakeRuntime = UINT64_C(5000000000) + i;
        sensorarrayCpuOwnerCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    }
    atomic_store(&writerDone, true); return NULL;
}
static void testAtomicPublication(void)
{
    reset(); atomic_store(&writerDone, false);
    pthread_t thread; assert(pthread_create(&thread, NULL, writer, NULL) == 0);
    do {
        sensorarrayCpuOwnerSnapshot_t s;
        assert(sensorarrayCpuOwnerCopySnapshot(SENSORARRAY_WORK_BUDGET_BLE_TX, &s));
        if (s.sampleKnown) {
            assert(s.taskNumber == 77u && s.t0Us == s.t1Us && s.t1Us == s.observedAtUs);
            assert(s.runUs == UINT64_C(5000000000) + s.t0Us / SENSORARRAY_CPU_OWNER_SAMPLE_US);
        }
    } while (!atomic_load(&writerDone));
    assert(pthread_join(thread, NULL) == 0);
}
#endif

static void testBudgetUnchanged(void)
{
    reset(); assert(sensorarrayWorkBudgetInit() == ESP_OK);
    for (unsigned role = 0u; role < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; role++) {
        unsigned before = delayCalls;
        for (unsigned i = 0u; i < 31u; i++) {
            assert(!sensorarrayWorkBudgetCheckpoint((sensorarrayWorkBudgetWorker_t)role));
        }
        assert(sensorarrayWorkBudgetCheckpoint((sensorarrayWorkBudgetWorker_t)role));
        assert(delayCalls == before + 1u && idleHook && idleHook());
    }
    assert(delayCalls == SENSORARRAY_WORK_BUDGET_WORKER_COUNT);
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    assert(getInfoCalls == SENSORARRAY_WORK_BUDGET_WORKER_COUNT);
#else
    assert(getInfoCalls == 0u && timerCalls == 0u);
#endif
    printf("CPUOWNER_BUDGET,enabled=%u,roles=7,checkpoints=224,yields=%u\n",
        (unsigned)CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS, delayCalls);
}

/* ACTUAL_EARLY_DISPATCH */
/* ACTUAL_NORMAL_DISPATCH */
static void testDispatchAndCapacity(void)
{
    reset(); char response[512];
    for (unsigned role = 0; role < 7u; role++) {
        char command[16]; (void)snprintf(command, sizeof(command), "CPUOWNER?%u", role);
        assert(actualEarly(command, response, sizeof(response)) == ESP_OK);
        assert(actualNormal(command, response, sizeof(response)) == ESP_OK);
        assert(strstr(response, "session=123456789ABCDEF0"));
    }
    assert(actualEarly("CPUOWNER?7", response, sizeof(response)) == ESP_ERR_INVALID_ARG);
    assert(actualNormal("OTHER?", response, sizeof(response)) == ESP_ERR_NOT_FOUND);
    assert(getInfoCalls == 0u); /* Query never dereferences any live task. */
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    s_cpuOwners[0].value = (sensorarrayCpuOwnerSnapshot_t){.sampleKnown=true, .taskNumber=UINT32_MAX,
        .runUs=UINT64_MAX, .t0Us=UINT64_C(8000000000000000000), .t1Us=UINT64_C(8000000000000000000),
        .observedAtUs=UINT64_C(8000000000000000000)};
    atomic_store(&fakeUs, UINT64_MAX);
#endif
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", UINT64_MAX, response, sizeof(response)) == ESP_OK);
    size_t length = strlen(response);
    size_t ownerMaximum = length;
    assert(length < sizeof(response) && response[length-1] == '\n');
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", UINT64_MAX, response, length + 1u) == ESP_OK);
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", UINT64_MAX, response, length) == ESP_ERR_INVALID_SIZE);
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", UINT64_MAX, response, 1u) == ESP_ERR_INVALID_SIZE && !response[0]);
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", UINT64_MAX, response, 0u) == ESP_ERR_INVALID_ARG);
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    s_cpuOwners[0].value.counterRegressed = true;
    assert(sensorarrayCpuOwnerFormat("CPUOWNER?0", UINT64_MAX, response, sizeof(response)) == ESP_OK);
    if (strlen(response) > ownerMaximum) { ownerMaximum = strlen(response); }
    assert(ownerMaximum < sizeof(response));
#endif
    const char *caps[]={"usb_cdc_data_v1/usb_rom_handoff_v1","usb_vendor_bulk_v1/usb_rom_handoff_v1","serial_sink_v1/serial_classes_v1/usb_rom_handoff_v1"};
    size_t maximum=0u;
    for (unsigned i=0u;i<3u;i++) {
        int n=snprintf(response,sizeof(response),/* ACTUAL_PROTO */,2u,512u,2304u,caps[i]);
        assert(n>0 && n<(int)sizeof(response) && strstr(response,"/cpu_owner_v1,"));
        if ((size_t)n>maximum) { maximum=(size_t)n; }
    }
    printf("CPUOWNER_FORMAT,enabled=%u,maxOwnerBytes=%zu,maxProtoBytes=%zu,capacity=512\n",
        (unsigned)CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS, ownerMaximum, maximum);
}

int main(void)
{
    testUnknownAndOff(); testBudgetUnchanged();
#if CONFIG_SENSORARRAY_CPU_OWNER_DIAGNOSTICS
    testSparseStaleAndWide(); testWriterConflictAndRegression(); testCounterCrossing(); testAtomicPublication();
    printf("CPUOWNER_STORAGE,hostSlotBytes=%zu,hostTotalBytes=%zu,targetSize=NOT_RUN\n",
        sizeof(sensorarrayCpuOwnerSlot_t), sizeof(s_cpuOwners));
#endif
    testDispatchAndCapacity();
    puts("CPUOWNER_PASS,synthetic=1,productionModule=1,hardware=NOT_RUN");
    return 0;
}
