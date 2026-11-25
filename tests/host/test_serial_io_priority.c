/* Actual SerialIo TX/queue functions and WorkBudget in a synthetic fixed-
 * priority scheduler. No USB hardware, radio, CPU-usage or HIL FPS claim. */
#define _XOPEN_SOURCE 700
#include <assert.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <ucontext.h>
#include "sensorarrayWorkBudget.c"
#include "sensorarraySerialIo.h"
#include "serial_priority_config.h"
#include "sensorarrayUsbOtg.h"

#define portTICK_PERIOD_MS 10u
typedef int portMUX_TYPE;
typedef struct { unsigned count, read, write; uint8_t bytes[SERIAL_IO_QUEUE_COUNT]; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef void *TaskHandle_t;
static unsigned lockDepth;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(p) do { (void)(p); lockDepth++; } while (0)
#define portEXIT_CRITICAL(p) do { (void)(p); assert(lockDepth); lockDepth--; } while (0)

enum { TASK_SOURCE, TASK_SERIAL, TASK_BLE, TASK_TRANSPORT, TASK_CONTROL, TASK_COUNT };
typedef struct {
    ucontext_t context;
    uint8_t stack[65536];
    uint64_t readyAt;
    unsigned priority, notifications;
    bool waitingNotification;
} task_t;
static task_t tasks[TASK_COUNT];
static ucontext_t dispatcher;
static uint64_t nowUs, noHostUntil, stopUs;
static int currentTask;
static unsigned sourceFrames, controlCommands, bleWork, transportWork, driverWaits;
static unsigned sourceAccepted, controlAccepted, submittedAtCaptureEnd;
static uint64_t controlQueuedAt[40], maxControlDispatchUs, maxControlDeliveryUs;
static unsigned controlsCopied;
static unsigned idleVisits, maxQueueCount, driverCalls;
static bool handoffOnly, handoffHeld, handoffObservedHeld, handoffDone, handoffBleDone;
static uint64_t handoffReleaseUs;
static sensorarrayWorkBudgetWorker_t controlWorker;
static bool (*idleCallback)(void);
static char dataStorage[SERIAL_IO_NORMAL_SLOT_COUNT][SENSORARRAY_TRANSPORT_TEXT_MAX];
static char (*s_dataBytes)[SENSORARRAY_TRANSPORT_TEXT_MAX] = dataStorage;
static char s_priorityBytes[SERIAL_IO_PRIORITY_SLOT_COUNT][SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
#ifdef SERIAL_IO_CDC_DATA_SLOT_COUNT
static unsigned s_dataSlotCount = SERIAL_IO_DATA_SLOT_COUNT;
static uint32_t s_dataLifetimeUs = SERIAL_IO_NORMAL_LIFETIME_US;
static sensorarraySerialSlot_t s_smallSlots[SERIAL_IO_DATA_SLOT_COUNT + SERIAL_IO_PRIORITY_SLOT_COUNT + SERIAL_IO_LOG_SLOT_COUNT];
static sensorarraySerialSlot_t *s_dataSlots = s_smallSlots;
static bool s_inUse[SERIAL_IO_SLOT_COUNT];
#else
static uint16_t s_lengths[SERIAL_IO_SLOT_COUNT];
static int64_t s_enqueuedUs[SERIAL_IO_SLOT_COUNT], s_deadlineUs[SERIAL_IO_SLOT_COUNT];
#if SERIAL_TEST_HAS_EPOCH
static uint32_t s_slotEpoch[SERIAL_IO_SLOT_COUNT];
#endif
static bool s_inUse[SERIAL_IO_SLOT_COUNT];
#endif
static portMUX_TYPE s_poolMux, s_statsMux;
static StaticQueue_t queues[SERIAL_IO_LANE_COUNT];
static QueueHandle_t s_queues[SERIAL_IO_LANE_COUNT];
static TaskHandle_t s_task;
static sensorarraySerialIoStats_t s_stats;
static atomic_bool s_vfsReady;

static void reschedule(void)
{
    assert(lockDepth == 0u && currentTask >= 0);
    assert(swapcontext(&tasks[currentTask].context, &dispatcher) == 0);
}
static void blockUntil(uint64_t deadline)
{
    tasks[currentTask].readyAt = deadline;
    reschedule();
}
static void cpuWork(unsigned microseconds)
{
    nowUs += microseconds;
    reschedule();
}
TickType_t xTaskGetTickCount(void) { return (TickType_t)(nowUs / 10000u); }
int xPortGetCoreID(void) { return 0; }
void vTaskDelay(TickType_t ticks)
{
    assert(ticks == 1u && lockDepth == 0u);
    blockUntil((nowUs / 10000u + ticks) * 10000u);
}
esp_err_t esp_register_freertos_idle_hook_for_cpu(bool (*callback)(void), unsigned cpu)
{ assert(cpu == 0u); idleCallback = callback; return ESP_OK; }
static int64_t esp_timer_get_time(void) { return (int64_t)nowUs; }
static int xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks)
{
    assert(ticks == 0u);
    if (queue->count == SERIAL_IO_QUEUE_COUNT) { return 0; }
    queue->bytes[queue->write++ % SERIAL_IO_QUEUE_COUNT] = *(const uint8_t *)item;
    queue->count++;
    if (queue->count > maxQueueCount) { maxQueueCount = queue->count; }
    return pdTRUE;
}
static int xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks)
{
    assert(ticks == 0u);
    if (!queue->count) { return 0; }
    *(uint8_t *)item = queue->bytes[queue->read++ % SERIAL_IO_QUEUE_COUNT];
    queue->count--; return pdTRUE;
}
static int xTaskNotifyGive(TaskHandle_t task)
{
    task_t *target = task;
    target->notifications++;
    if (target->waitingNotification) {
        target->readyAt = nowUs;
        /* FreeRTOS queue/semaphore give preempts for a higher-priority ready
         * task. Equal priority is sliced at ticks, not at each handoff. */
        if (currentTask >= 0 && target->priority > tasks[currentTask].priority) {
            reschedule();
        }
    }
    return pdTRUE;
}
static uint32_t ulTaskNotifyTake(int clear, TickType_t ticks)
{
    assert(currentTask == TASK_SERIAL && clear == pdTRUE && ticks == portMAX_DELAY);
    task_t *task = &tasks[TASK_SERIAL];
    while (!task->notifications) {
        task->waitingNotification = true;
        blockUntil(UINT64_MAX);
    }
    task->waitingNotification = false;
    unsigned count = task->notifications; task->notifications = 0u;
    return count;
}
static int usb_serial_jtag_write_bytes(const void *data, size_t bytes, TickType_t waitTicks)
{
    assert(currentTask == TASK_SERIAL && lockDepth == 0u && data && bytes <= SERIAL_IO_DRIVER_TX_BYTES);
    assert(waitTicks <= 1u); driverCalls++;
    if (nowUs < noHostUntil) {
        if (waitTicks) {
            /* Worst case: the IDF function can wait one tick for tx_mux and
             * one for ring space. Only this context blocks, without a lock. */
            unsigned bleBefore = bleWork, controlBefore = controlCommands;
            driverWaits++; blockUntil(nowUs + 2u * waitTicks * 10000u);
            assert(bleWork > bleBefore || controlCommands > controlBefore);
        }
        return 0;
    }
    if (noHostUntil == 0u && bytes == 5u && memcmp(data, "CTRL\n", 5u) == 0) {
        assert(controlsCopied < controlAccepted);
        uint64_t age = nowUs - controlQueuedAt[controlsCopied++];
        if (age > maxControlDeliveryUs) { maxControlDeliveryUs = age; }
    }
    cpuWork(200u); /* Synthetic CPU copy/driver admission cost, not measured. */
    return (int)bytes;
}
static esp_err_t usb_serial_jtag_wait_tx_done(TickType_t ticks)
{ assert(ticks == 0u && currentTask == TASK_SERIAL); return ESP_OK; }

#include "serial_priority_source.h"

static void sourceTask(void)
{
    for (unsigned frame = 0u; frame < 200u; ++frame) {
        uint64_t release = (uint64_t)frame * 10000u;
        if (nowUs < release) { blockUntil(release); }
        sourceFrames++;
        sourceAccepted += sensorarraySerialIoQueueData("DATA\n", 5u) == ESP_OK;
        cpuWork(200u);
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_ASYNC_LOG);
    }
    blockUntil(UINT64_MAX); assert(false);
}
static void serialTask(void) { SERIAL_TEST_LEGACY_TX(NULL); assert(false); }
static void bleTask(void)
{
    if (handoffOnly) {
        assert(tasks[TASK_BLE].notifications > 0u);
        handoffObservedHeld = handoffHeld;
        bleWork++;
        cpuWork(7200u); /* Synthetic three-fragment service, not measured CPU. */
        handoffBleDone = true;
        blockUntil(UINT64_MAX); assert(false);
    }
    for (;;) {
        bleWork++; cpuWork(500u);
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    }
}
static void transportTask(void)
{
    if (handoffOnly) {
        handoffHeld = true;
        (void)xTaskNotifyGive(&tasks[TASK_BLE]);
        handoffHeld = false;
        handoffReleaseUs = nowUs;
        handoffDone = true;
        blockUntil(UINT64_MAX); assert(false);
    }
    for (;;) {
        transportWork++; cpuWork(500u);
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_TRANSPORT);
    }
}
static void controlTask(void)
{
    for (unsigned command = 0u; command < 40u; ++command) {
        uint64_t release = (uint64_t)command * 50000u;
        if (nowUs < release) { blockUntil(release); }
        uint64_t dispatch = nowUs - release;
        if (dispatch > maxControlDispatchUs) { maxControlDispatchUs = dispatch; }
        controlCommands++;
        if (sensorarraySerialQueueRecord("CTRL\n", 5u, SERIAL_IO_PRIORITY) == ESP_OK) {
            assert(controlAccepted < 40u);
            controlQueuedAt[controlAccepted++] = nowUs;
        }
        cpuWork(100u);
        (void)sensorarrayWorkBudgetCheckpoint(controlWorker);
    }
    blockUntil(UINT64_MAX); assert(false);
}
static void reset(unsigned serialPriority, uint64_t unavailableUntil, unsigned controlPriority)
{
    memset(tasks, 0, sizeof(tasks)); memset(queues, 0, sizeof(queues));

#ifdef SERIAL_IO_CDC_DATA_SLOT_COUNT
    memset(s_smallSlots, 0, sizeof(s_smallSlots)); memset(s_inUse, 0, sizeof(s_inUse));
#else
    memset(s_inUse, 0, sizeof(s_inUse));
#endif
 memset(&s_stats, 0, sizeof(s_stats));
    memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    nowUs = 0u; noHostUntil = unavailableUntil; stopUs = 2100000u;
    handoffOnly = handoffHeld = handoffObservedHeld = handoffDone = handoffBleDone = false;
    handoffReleaseUs = 0u;
    sourceFrames = controlCommands = bleWork = transportWork = driverWaits = 0u;
    sourceAccepted = controlAccepted = submittedAtCaptureEnd = 0u;
    memset(controlQueuedAt, 0, sizeof(controlQueuedAt));
    maxControlDispatchUs = maxControlDeliveryUs = 0u; controlsCopied = 0u;
    idleVisits = maxQueueCount = driverCalls = 0u; currentTask = -1;
    controlWorker = controlPriority == ACTUAL_SERIAL_CONTROL_PRIORITY ?
        SENSORARRAY_WORK_BUDGET_SERIAL_CTRL : SENSORARRAY_WORK_BUDGET_BLE_CTRL;
    atomic_store(&s_vfsReady, true); s_task = &tasks[TASK_SERIAL];
    for (unsigned i = 0u; i < SERIAL_IO_LANE_COUNT; ++i) { s_queues[i] = &queues[i]; }
    assert(sensorarrayWorkBudgetInit() == ESP_OK && idleCallback && idleCallback());
    void (*entries[])(void) = {sourceTask, serialTask, bleTask, transportTask, controlTask};
    unsigned priorities[] = {ACTUAL_LOG_PRIORITY, serialPriority, ACTUAL_BLE_PRIORITY, ACTUAL_TRANSPORT_PRIORITY, controlPriority};
    for (unsigned i = 0u; i < TASK_COUNT; ++i) {
        tasks[i].priority = priorities[i]; assert(getcontext(&tasks[i].context) == 0);
        tasks[i].context.uc_stack.ss_sp = tasks[i].stack;
        tasks[i].context.uc_stack.ss_size = sizeof(tasks[i].stack);
        tasks[i].context.uc_link = &dispatcher;
        makecontext(&tasks[i].context, entries[i], 0);
    }
}
static void run(unsigned serialPriority, uint64_t unavailableUntil, unsigned controlPriority)
{
    reset(serialPriority, unavailableUntil, controlPriority);
    int last = -1; uint32_t previousTick = 0u;
    while (nowUs < stopUs) {
        unsigned bestPriority = 0u;
        for (unsigned i = 0u; i < TASK_COUNT; ++i) {
            if (tasks[i].readyAt <= nowUs && tasks[i].priority > bestPriority) { bestPriority = tasks[i].priority; }
        }
        if (!bestPriority) {
            assert(idleCallback()); idleVisits++;
            uint64_t next = UINT64_MAX;
            for (unsigned i = 0u; i < TASK_COUNT; ++i) { if (tasks[i].readyAt < next) { next = tasks[i].readyAt; } }
            assert(next > nowUs); nowUs = next; continue;
        }
        int selected = -1; uint32_t tick = xTaskGetTickCount();
        /* Equal-priority tasks time-slice on ticks; no invented per-item RR. */
        if (last >= 0 && tick == previousTick && tasks[last].readyAt <= nowUs && tasks[last].priority == bestPriority) {
            selected = last;
        } else {
            for (unsigned n = 1u; n <= TASK_COUNT; ++n) {
                unsigned i = (unsigned)(last + (int)n) % TASK_COUNT;
                if (tasks[i].readyAt <= nowUs && tasks[i].priority == bestPriority) { selected = (int)i; break; }
            }
        }
        assert(selected >= 0); last = currentTask = selected; previousTick = tick;
        assert(swapcontext(&dispatcher, &tasks[selected].context) == 0);
        if (nowUs < 2000000u) { submittedAtCaptureEnd = s_stats.dataSubmitted; }
    }
    sensorarrayWorkBudgetSnapshot_t snap; sensorarrayWorkBudgetCopySnapshot(&snap);
    printf("SERIAL_PRIORITY_MODEL,priority=%u,ctrlPriority=%u,noHostUs=%llu,source=%u,accepted=%u,data=%u,dataAt2s=%u,rejected=%u,expired=%u,ctrl=%u,ctrlSubmitted=%u,bleWork=%u,trWork=%u,driverWaits=%u,idle=%u,maxGap=%u,queueMax=%u,ctrlDispatchMaxUs=%llu,ctrlDriverMaxUs=%llu\n",
           serialPriority, controlPriority, (unsigned long long)unavailableUntil, sourceFrames, sourceAccepted,
           s_stats.dataSubmitted, submittedAtCaptureEnd, s_stats.dataRejected, s_stats.normalExpired,
           controlCommands, s_stats.prioritySubmitted, bleWork, transportWork, driverWaits,
           idleVisits, snap.maxIdleGapTicks, maxQueueCount,
           (unsigned long long)maxControlDispatchUs, (unsigned long long)maxControlDeliveryUs);
    assert(sourceFrames == 200u && controlCommands == 40u);
    assert(bleWork > 0u && transportWork > 0u && idleVisits > 0u && snap.maxIdleGapTicks <= 6u);
    assert(s_stats.driverShortWrite == 0u && s_stats.driverWriteError == 0u && maxQueueCount <= 4u);
    unsigned dataRetained = 0u, controlRetained = 0u;
    #ifdef SERIAL_IO_CDC_DATA_SLOT_COUNT
    for (unsigned i=0u; i<s_dataSlotCount; i++) { dataRetained += s_inUse[i] ? 1u : 0u; }
    for (unsigned i=SERIAL_IO_PRIORITY_FIRST_SLOT; i<SERIAL_IO_LOG_FIRST_SLOT; i++) { controlRetained += s_inUse[i] ? 1u : 0u; }
#else
    for (unsigned i=0u; i<SERIAL_IO_DATA_SLOT_COUNT; i++) { dataRetained += s_inUse[i] ? 1u : 0u; }
    for (unsigned i=SERIAL_IO_DATA_SLOT_COUNT; i<SERIAL_IO_LOG_FIRST_SLOT; i++) { controlRetained += s_inUse[i] ? 1u : 0u; }
#endif
    assert(sourceAccepted + s_stats.dataRejected == sourceFrames);
    assert(sourceAccepted == s_stats.dataSubmitted + s_stats.normalExpired + dataRetained);
    assert(controlAccepted + s_stats.priorityRejected == controlCommands);
    assert(controlAccepted == s_stats.prioritySubmitted + s_stats.priorityExpired + controlRetained);
    assert(maxControlDispatchUs < SERIAL_IO_PRIORITY_LIFETIME_US);
    if (serialPriority == 4u && unavailableUntil == 0u) {
        assert(s_stats.dataRejected > 0u || s_stats.normalExpired > 0u);
    }
    /* Legacy USJ stays below BLE; the OTG-priority case shares the producer's
     * scheduling level. The real queue/TTL/WorkBudget bodies remain in use;
     * this synthetic scheduling result is not a HIL throughput acceptance. */
    if (unavailableUntil == 0u) {
        assert(s_stats.dataSubmitted > 0u);
        assert(s_stats.prioritySubmitted == 40u);
        assert(s_stats.priorityRejected == 0u && s_stats.priorityExpired == 0u);
        assert(controlsCopied == 40u && maxControlDeliveryUs < SERIAL_IO_PRIORITY_LIFETIME_US);
    }
    if (unavailableUntil == UINT64_MAX) {
        assert(driverWaits > 0u && s_stats.dataSubmitted == 0u && s_stats.normalExpired > 0u);
    }
    if (unavailableUntil == 200000u) {
        assert(driverWaits > 0u && s_stats.dataSubmitted > 0u && s_stats.prioritySubmitted > 0u);
    }
}
static void runHandoff(unsigned transportPriority, bool expectedHeld)
{
    reset(ACTUAL_OTG_PRIORITY, 0u, ACTUAL_CONTROL_PRIORITY);
    handoffOnly = true;
    for (unsigned i = 0; i < TASK_COUNT; i++) { tasks[i].readyAt = UINT64_MAX; }
    tasks[TASK_TRANSPORT].readyAt = 0u;
    tasks[TASK_TRANSPORT].priority = transportPriority;
    tasks[TASK_BLE].waitingNotification = true;
    unsigned switches = 0u;
    while (!handoffDone || !handoffBleDone) {
        int selected = -1;
        for (unsigned i = 0u; i < TASK_COUNT; i++) {
            if (tasks[i].readyAt <= nowUs &&
                (selected < 0 || tasks[i].priority > tasks[selected].priority)) {
                selected = (int)i;
            }
        }
        assert(selected >= 0 && ++switches <= 4u && nowUs <= 7200u);
        currentTask = selected;
        assert(swapcontext(&dispatcher, &tasks[selected].context) == 0);
    }
    assert(!handoffHeld && handoffObservedHeld == expectedHeld);
    assert(handoffReleaseUs == (expectedHeld ? 7200u : 0u));
    printf("HANDOFF_SCHEDULER_MODEL,transportPriority=%u,blePriority=%u,slotHeldDuringBle=%u,releaseUs=%llu,synthetic=1,hardware=NOT_RUN\n",
           transportPriority, ACTUAL_BLE_PRIORITY, handoffObservedHeld,
           (unsigned long long)handoffReleaseUs);
}
int main(void)
{
    setbuf(stdout, NULL);
    assert(ACTUAL_OUTPUT_CORE == 0 && ACTUAL_LOG_PRIORITY == 7u && ACTUAL_BLE_PRIORITY == 7u);
    assert(ACTUAL_CONTROL_PRIORITY == 8u && ACTUAL_CONTROL_PRIORITY > ACTUAL_BLE_PRIORITY);
    run(4u, 0u, ACTUAL_CONTROL_PRIORITY); /* Old priority counterexample retained. */
    run(6u, 0u, ACTUAL_CONTROL_PRIORITY);
    run(6u, UINT64_MAX, ACTUAL_CONTROL_PRIORITY);
    run(6u, 200000u, ACTUAL_CONTROL_PRIORITY);
    run(6u, 0u, ACTUAL_SERIAL_CONTROL_PRIORITY);
    run(6u, UINT64_MAX, ACTUAL_SERIAL_CONTROL_PRIORITY);
    assert(ACTUAL_SERIAL_PRIORITY == 6u && ACTUAL_SERIAL_PRIORITY < ACTUAL_BLE_PRIORITY);
    assert(ACTUAL_OTG_PRIORITY == ACTUAL_LOG_PRIORITY && ACTUAL_TRANSPORT_PRIORITY == ACTUAL_LOG_PRIORITY);
    run(ACTUAL_OTG_PRIORITY, 0u, ACTUAL_CONTROL_PRIORITY);
    run(ACTUAL_OTG_PRIORITY, UINT64_MAX, ACTUAL_CONTROL_PRIORITY);
    run(ACTUAL_OTG_PRIORITY, 200000u, ACTUAL_CONTROL_PRIORITY);
    run(ACTUAL_OTG_PRIORITY, 0u, ACTUAL_SERIAL_CONTROL_PRIORITY);
    run(ACTUAL_OTG_PRIORITY, UINT64_MAX, ACTUAL_SERIAL_CONTROL_PRIORITY);
    runHandoff(6u, true);
    runHandoff(ACTUAL_TRANSPORT_PRIORITY, false);
    printf("SERIAL_PRIORITY_PASS,scenarios=13,legacyPriority=%u,otgPriority=%u,actualFunctions=8,hardware=NOT_RUN\n",
           ACTUAL_SERIAL_PRIORITY, ACTUAL_OTG_PRIORITY);
    return 0;
}
