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
static unsigned idleVisits, maxQueueCount, driverCalls;
static sensorarrayWorkBudgetWorker_t controlWorker;
static bool (*idleCallback)(void);
static char dataStorage[SERIAL_IO_NORMAL_SLOT_COUNT][SENSORARRAY_TRANSPORT_TEXT_MAX];
static char (*s_dataBytes)[SENSORARRAY_TRANSPORT_TEXT_MAX] = dataStorage;
static char s_priorityBytes[SERIAL_IO_PRIORITY_SLOT_COUNT][SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
static uint16_t s_lengths[SERIAL_IO_SLOT_COUNT];
static int64_t s_enqueuedUs[SERIAL_IO_SLOT_COUNT], s_deadlineUs[SERIAL_IO_SLOT_COUNT];
static bool s_inUse[SERIAL_IO_SLOT_COUNT];
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
    if (target->waitingNotification) { target->readyAt = nowUs; }
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
static void serialTask(void) { sensorarraySerialTxTask(NULL); assert(false); }
static void bleTask(void)
{
    for (;;) {
        bleWork++; cpuWork(500u);
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
    }
}
static void transportTask(void)
{
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
        controlCommands++;
        controlAccepted += sensorarraySerialQueueRecord("CTRL\n", 5u, SERIAL_IO_PRIORITY) == ESP_OK;
        cpuWork(100u);
        (void)sensorarrayWorkBudgetCheckpoint(controlWorker);
    }
    blockUntil(UINT64_MAX); assert(false);
}
static void reset(unsigned serialPriority, uint64_t unavailableUntil, unsigned controlPriority)
{
    memset(tasks, 0, sizeof(tasks)); memset(queues, 0, sizeof(queues));
    memset(s_inUse, 0, sizeof(s_inUse)); memset(&s_stats, 0, sizeof(s_stats));
    memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    nowUs = 0u; noHostUntil = unavailableUntil; stopUs = 2100000u;
    sourceFrames = controlCommands = bleWork = transportWork = driverWaits = 0u;
    sourceAccepted = controlAccepted = submittedAtCaptureEnd = 0u;
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
    printf("SERIAL_PRIORITY_MODEL,priority=%u,ctrlPriority=%u,noHostUs=%llu,source=%u,accepted=%u,data=%u,dataAt2s=%u,rejected=%u,expired=%u,ctrl=%u,ctrlSubmitted=%u,bleWork=%u,trWork=%u,driverWaits=%u,idle=%u,maxGap=%u,queueMax=%u\n",
           serialPriority, controlPriority, (unsigned long long)unavailableUntil, sourceFrames, sourceAccepted,
           s_stats.dataSubmitted, submittedAtCaptureEnd, s_stats.dataRejected, s_stats.normalExpired,
           controlCommands, s_stats.prioritySubmitted, bleWork, transportWork, driverWaits,
           idleVisits, snap.maxIdleGapTicks, maxQueueCount);
    assert(sourceFrames == 200u && controlCommands == 40u);
    assert(bleWork > 0u && transportWork > 0u && idleVisits > 0u && snap.maxIdleGapTicks <= 6u);
    assert(s_stats.driverShortWrite == 0u && s_stats.driverWriteError == 0u && maxQueueCount <= 4u);
    if (serialPriority == 4u && unavailableUntil == 0u) {
        assert(s_stats.dataRejected > 0u || s_stats.normalExpired > 0u);
    }
    if (serialPriority == 6u && unavailableUntil == 0u) {
        assert(s_stats.dataSubmitted == 200u && s_stats.dataRejected == 0u && s_stats.normalExpired == 0u);
        assert(s_stats.prioritySubmitted == 40u);
    }
    if (unavailableUntil == UINT64_MAX) {
        assert(driverWaits > 0u && s_stats.dataSubmitted == 0u && s_stats.normalExpired > 0u);
    }
    if (unavailableUntil == 200000u) {
        assert(driverWaits > 0u && s_stats.dataSubmitted > 0u && s_stats.prioritySubmitted > 0u);
    }
}
int main(void)
{
    setbuf(stdout, NULL);
    assert(ACTUAL_OUTPUT_CORE == 0 && ACTUAL_LOG_PRIORITY == 7u && ACTUAL_BLE_PRIORITY == 5u);
    run(4u, 0u, ACTUAL_CONTROL_PRIORITY); /* Old priority counterexample retained. */
    run(6u, 0u, ACTUAL_CONTROL_PRIORITY);
    run(6u, UINT64_MAX, ACTUAL_CONTROL_PRIORITY);
    run(6u, 200000u, ACTUAL_CONTROL_PRIORITY);
    run(6u, 0u, ACTUAL_SERIAL_CONTROL_PRIORITY);
    run(6u, UINT64_MAX, ACTUAL_SERIAL_CONTROL_PRIORITY);
    assert(ACTUAL_SERIAL_PRIORITY == 6u); /* Bind the regression to the actual task creation. */
    printf("SERIAL_PRIORITY_PASS,scenarios=6,productionPriority=%u,actualFunctions=8,hardware=NOT_RUN\n", ACTUAL_SERIAL_PRIORITY);
    return 0;
}
