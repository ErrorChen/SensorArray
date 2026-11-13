/* Actual SerialIo OTG scheduling/slot bodies; synthetic endpoint boundary.
 * The separately tested TinyUSB component is not duplicated here. */
#include <assert.h>
#include <setjmp.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayWorkBudget.c"
#include "sensorarraySerialIo.h"
#include "serial_priority_config.h"

#define portTICK_PERIOD_MS 10u
typedef int portMUX_TYPE;
typedef struct { unsigned count, head, tail; uint8_t bytes[SERIAL_IO_QUEUE_COUNT]; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef void *TaskHandle_t;
static unsigned lockDepth;
#define portENTER_CRITICAL(p) do { (void)(p); assert(!lockDepth); lockDepth++; } while (0)
#define portEXIT_CRITICAL(p) do { (void)(p); assert(lockDepth == 1u); lockDepth--; } while (0)
static char dataBytes[SERIAL_IO_NORMAL_SLOT_COUNT][SENSORARRAY_TRANSPORT_TEXT_MAX];
static char (*s_dataBytes)[SENSORARRAY_TRANSPORT_TEXT_MAX] = dataBytes;
static char s_priorityBytes[SERIAL_IO_PRIORITY_SLOT_COUNT][SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
static uint16_t s_lengths[SERIAL_IO_SLOT_COUNT];
static int64_t s_enqueuedUs[SERIAL_IO_SLOT_COUNT], s_deadlineUs[SERIAL_IO_SLOT_COUNT];
static uint32_t s_slotEpoch[SERIAL_IO_SLOT_COUNT];
static bool s_inUse[SERIAL_IO_SLOT_COUNT];
static portMUX_TYPE s_poolMux, s_statsMux;
static StaticQueue_t queues[SERIAL_IO_LANE_COUNT];
static QueueHandle_t s_queues[SERIAL_IO_LANE_COUNT];
static TaskHandle_t s_task = (void *)1;
static sensorarraySerialIoStats_t s_stats;
static atomic_bool s_vfsReady, s_otgInitDone;
static atomic_int s_otgInitResult;

static uint64_t nowUs, readyAt[3], injectAt;
static uint32_t currentEpoch;
static unsigned polls, delays, waits, notifications, ownerInitCalls, writes, rxCalls, rxCommands;
static bool allowed[3], rxPending, injectDone, resetAfterWrite, resetDone;
static unsigned partialLane, invalidMode, produced, productionTarget;
static bool injectAfterWrite;
static int writeBudget[3];
static char wire[3][4096], oldWire[3][4096];
static size_t wireLength[3], oldWireLength[3];
static bool (*idleHook)(void);
static jmp_buf ownerEnd;

TickType_t xTaskGetTickCount(void) { return (TickType_t)(nowUs / 10000u); }
int xPortGetCoreID(void) { return 0; }
static int64_t esp_timer_get_time(void) { return (int64_t)nowUs; }
void vTaskDelay(TickType_t ticks)
{
    assert(!lockDepth && ticks == 1u); delays++; nowUs += 10000u;
    if (idleHook) { assert(idleHook()); }
}
esp_err_t esp_register_freertos_idle_hook_for_cpu(bool (*hook)(void), unsigned cpu)
{ assert(cpu == 0u); idleHook = hook; return ESP_OK; }
static int xQueueSend(QueueHandle_t queue, const void *item, TickType_t ticks)
{
    assert(!lockDepth && ticks == 0u);
    if (queue->count == SERIAL_IO_QUEUE_COUNT) { return 0; }
    queue->bytes[queue->tail++ % SERIAL_IO_QUEUE_COUNT] = *(const uint8_t *)item;
    queue->count++; return pdTRUE;
}
static int xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks)
{
    assert(!lockDepth && ticks == 0u);
    if (!queue->count) { return 0; }
    *(uint8_t *)item = queue->bytes[queue->head++ % SERIAL_IO_QUEUE_COUNT];
    queue->count--; return pdTRUE;
}
static int xTaskNotifyGive(TaskHandle_t task)
{ assert(task == s_task && !lockDepth); notifications++; return pdTRUE; }
static uint32_t ulTaskNotifyTake(int clear, TickType_t ticks)
{
    assert(!lockDepth && clear == pdTRUE && ticks == 1u); waits++;
    unsigned result = notifications; notifications = 0u;
    if (!result) { nowUs += 10000u; if (idleHook) { assert(idleHook()); } }
    return result;
}
static bool sensorarrayUsbOtgEnabled(void) { return true; }
static uint32_t sensorarrayUsbOtgEpoch(void) { return currentEpoch; }
static esp_err_t sensorarrayUsbOtgOwnerInit(void)
{ assert(!lockDepth && ownerInitCalls++ == 0u); return ESP_OK; }
static void sensorarrayUsbOtgOwnerPoll(void);
static bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane)
{
    assert(!lockDepth && lane < 3u);
    return allowed[lane] && (writeBudget[lane] != 0 || nowUs >= readyAt[lane]);
}
static int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t length)
{
    assert(!lockDepth && lane < 3u && data && length > 0u);
    writes++;
    if (invalidMode) { unsigned invalid = invalidMode; invalidMode = 0u; return invalid == 1u ? -1 : (int)length + 1; }
    if (nowUs >= readyAt[lane]) { writeBudget[lane] = -1; }
    if (writeBudget[lane] == 0) { return 0; }
    size_t count = length;
    if (writeBudget[lane] > 0 && count > (unsigned)writeBudget[lane]) { count = (unsigned)writeBudget[lane]; }
    assert(wireLength[lane] + count < sizeof(wire[lane]));
    memcpy(wire[lane] + wireLength[lane], data, count); wireLength[lane] += count;
    if (writeBudget[lane] > 0) { writeBudget[lane] -= (int)count; }
    nowUs += 100u;
    return (int)count;
}
static int sensorarrayUsbOtgRead(void *buffer, size_t size, uint32_t *epoch)
{
    assert(!lockDepth && size >= 8u && epoch); rxCalls++;
    if (!rxPending) { return 0; }
    memcpy(buffer, "CONFIG?\n", 8u); *epoch = currentEpoch; rxPending = false; return 8;
}
static bool usb_serial_jtag_is_driver_installed(void) { assert(false); return false; }
static int usb_serial_jtag_read_bytes(void *buffer, size_t size, TickType_t ticks)
{ (void)buffer; (void)size; (void)ticks; assert(false); return -1; }

#include "serial_otg_source.h"

static bool allReleased(void)
{
    for (unsigned i=0u; i<SERIAL_IO_SLOT_COUNT; i++) { if (s_inUse[i]) { return false; } }
    return true;
}
static void offer(unsigned lane, const char *text)
{
    esp_err_t result = lane == SERIAL_IO_DATA ? sensorarraySerialIoQueueData(text, strlen(text)) :
        sensorarraySerialIoQueue(text, strlen(text), lane == SERIAL_IO_PRIORITY);
    assert(result == ESP_OK);
}
static void sensorarrayUsbOtgOwnerPoll(void)
{
    assert(!lockDepth && ++polls < 20000u); nowUs += 100u;
    assert(nowUs < 3000000u);
    if (resetAfterWrite && writes && !resetDone) {
        resetDone = true; currentEpoch++;
        memcpy(oldWire, wire, sizeof(wire)); memcpy(oldWireLength, wireLength, sizeof(wireLength));
        memset(wire, 0, sizeof(wire)); memset(wireLength, 0, sizeof(wireLength));
        for (unsigned i=0u; i<3u; i++) { allowed[i] = true; writeBudget[i] = -1; readyAt[i] = 0u; }
        offer(SERIAL_IO_DATA, "NEW_DATA\n"); offer(SERIAL_IO_PRIORITY, "NEW_CTRL\n");
    }
    if (injectAt && nowUs >= injectAt && !injectDone) {
        injectDone = true; offer(partialLane, "NEW\n");
    }
    if (injectAfterWrite && writes && !injectDone) {
        injectDone = true; offer(SERIAL_IO_PRIORITY, "CTRL_AFTER\n");
    }
    if (productionTarget && produced < productionTarget && allReleased()) {
        offer(SERIAL_IO_DATA, "DATA\n"); produced++;
    }
    if (rxPending) {
        char command[64]; uint32_t epoch = 0u;
        assert(sensorarraySerialIoReadWithEpoch(command, sizeof(command), &epoch) == 8);
        assert(!memcmp(command, "CONFIG?\n", 8u) && epoch == currentEpoch);
        rxCommands++; offer(SERIAL_IO_PRIORITY, "CONFIG_ACK\n");
    }
    if (allReleased() && (!productionTarget || produced == productionTarget)) { longjmp(ownerEnd, 1); }
}
static void reset(void)
{
    memset(&s_stats, 0, sizeof(s_stats)); memset(queues, 0, sizeof(queues)); memset(s_inUse, 0, sizeof(s_inUse));
    memset(s_slotEpoch, 0, sizeof(s_slotEpoch)); memset(wire, 0, sizeof(wire)); memset(wireLength, 0, sizeof(wireLength));
    memset(oldWire, 0, sizeof(oldWire)); memset(oldWireLength, 0, sizeof(oldWireLength));
    memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    nowUs = injectAt = 0u; currentEpoch = 1u;
    polls = delays = waits = notifications = ownerInitCalls = writes = rxCalls = rxCommands = 0u;
    partialLane = invalidMode = produced = productionTarget = 0u;
    rxPending = injectDone = resetAfterWrite = resetDone = injectAfterWrite = false;
    for (unsigned i=0u; i<3u; i++) {
        s_queues[i] = &queues[i]; allowed[i] = true; readyAt[i] = 0u; writeBudget[i] = -1;
    }
    atomic_store(&s_vfsReady, true); atomic_store(&s_otgInitDone, false);
    atomic_store(&s_otgInitResult, ESP_ERR_INVALID_STATE);
    assert(sensorarrayWorkBudgetInit() == ESP_OK && idleHook && idleHook());
}
static void run(void)
{
    if (setjmp(ownerEnd) == 0) { sensorarraySerialOtgTxTask(NULL); }
    assert(!lockDepth && allReleased() && ownerInitCalls == 1u);
    assert(atomic_load(&s_otgInitDone) && atomic_load(&s_otgInitResult) == ESP_OK);
    for (unsigned i=0u; i<3u; i++) { assert(queues[i].count == 0u); }
}
static void testIndependentLanes(void)
{
    reset(); allowed[SERIAL_IO_LOG] = false;
    offer(SERIAL_IO_LOG, "UNREAD_DEBUG\n");
    for (unsigned i=0u; i<4u; i++) { offer(SERIAL_IO_DATA, "DATA\n"); offer(SERIAL_IO_PRIORITY, "CTRL\n"); }
    run(); assert(s_stats.dataSubmitted == 4u && s_stats.prioritySubmitted == 4u && s_stats.logSubmitted == 0u);
    assert(s_stats.normalExpired == 1u && s_stats.priorityExpired == 0u && wireLength[2] == 0u);
    assert(!memcmp(wire[0], "DATA\nDATA\nDATA\nDATA\n", 20u));
    reset(); allowed[SERIAL_IO_DATA] = false; rxPending = true;
    for (unsigned i=0u; i<4u; i++) { offer(SERIAL_IO_DATA, "UNREAD_DATA\n"); }
    offer(SERIAL_IO_PRIORITY, "CTRL\n"); offer(SERIAL_IO_LOG, "LOG\n");
    run(); assert(s_stats.dataSubmitted == 0u && s_stats.prioritySubmitted == 2u && s_stats.logSubmitted == 1u);
    assert(rxCalls == 1u && rxCommands == 1u && s_stats.normalExpired == 4u && wireLength[0] == 0u);
    assert(!strcmp(wire[1], "CTRL\nCONFIG_ACK\n"));
}
static void testPartialEachLane(void)
{
    for (unsigned lane=0u; lane<3u; lane++) {
        reset(); partialLane = lane; writeBudget[lane] = 3;
        uint64_t lifetime = lane == SERIAL_IO_PRIORITY ? SERIAL_IO_PRIORITY_LIFETIME_US : SERIAL_IO_NORMAL_LIFETIME_US;
        readyAt[lane] = lifetime + 50000u; injectAt = lifetime - 50000u;
        offer(lane, "ABCDEFGH");
        for (unsigned other=0u; other<3u; other++) { if (other != lane) { offer(other, "UNBLOCKED\n"); } }
        run(); assert(injectDone && !strcmp(wire[lane], "ABC\nNEW\n"));
        assert(s_stats.partialWrite == 1u && s_stats.driverShortWrite == 1u && s_stats.timeout == 1u);
        assert(s_stats.priorityExpired == (lane == SERIAL_IO_PRIORITY ? 1u : 0u));
        assert(s_stats.normalExpired == (lane == SERIAL_IO_PRIORITY ? 0u : 1u));
        for (unsigned other=0u; other<3u; other++) { if (other != lane) { assert(!strcmp(wire[other], "UNBLOCKED\n")); } }
    }
}
static void testEpochs(void)
{
    reset();
    offer(SERIAL_IO_DATA, "OLD_DATA\n"); offer(SERIAL_IO_LOG, "OLD_LOG\n"); offer(SERIAL_IO_PRIORITY, "OLD_CTRL\n");
    currentEpoch++;
    offer(SERIAL_IO_DATA, "NEW_DATA\n"); offer(SERIAL_IO_LOG, "NEW_LOG\n"); offer(SERIAL_IO_PRIORITY, "NEW_CTRL\n");
    run(); assert(!strcmp(wire[0], "NEW_DATA\n") && !strcmp(wire[1], "NEW_CTRL\n") && !strcmp(wire[2], "NEW_LOG\n"));
    assert(s_stats.timeout == 0u && s_stats.normalExpired == 2u && s_stats.priorityExpired == 1u);
    for (unsigned partial=0u; partial<2u; partial++) {
        reset(); resetAfterWrite = true; readyAt[0] = UINT64_MAX; writeBudget[0] = partial ? 3 : -1;
        offer(SERIAL_IO_DATA, "OLD_PREFIX"); run();
        assert(resetDone && oldWireLength[0] == (partial ? 3u : 10u));
        assert(!strcmp(wire[0], "NEW_DATA\n") && wire[0][0] != '\n');
        assert(!strcmp(wire[1], "NEW_CTRL\n") && s_stats.timeout == 0u);
        assert(s_stats.partialWrite == (partial ? 1u : 0u));
    }
}
static void testNoHostAndBudget(void)
{
    reset();
    for (unsigned lane=0u; lane<3u; lane++) {
        allowed[lane] = false;
        for (unsigned i=0u; i<4u; i++) { offer(lane, "EXPIRE\n"); }
    }
    run(); assert(writes == 0u && s_stats.normalExpired == 8u && s_stats.priorityExpired == 4u);
    assert(s_stats.timeout == 12u && s_stats.sinkDropped == 12u && s_stats.partialWrite == 0u);
    assert(nowUs <= SERIAL_IO_PRIORITY_LIFETIME_US + 2u * SERIAL_IO_TICK_US && waits > 0u);
    reset(); productionTarget = 100u; run();
    assert(produced == 100u && s_stats.dataSubmitted == 100u && s_stats.sinkDropped == 0u);
    assert(s_stats.txYield > 0u && delays > 0u && s_workers[SENSORARRAY_WORK_BUDGET_SERIAL_TX].checkpoints >= 100u);
    for (unsigned bad=1u; bad<=2u; bad++) {
        reset(); invalidMode = bad; offer(SERIAL_IO_DATA, "VALID\n"); run();
        assert(writes == 2u && s_stats.driverWriteError == 1u && s_stats.dataSubmitted == 1u);
        assert(s_stats.driverAcceptedBytes == 6u && !strcmp(wire[0], "VALID\n"));
    }
}
int main(void)
{
    testIndependentLanes(); testPartialEachLane(); testEpochs(); testNoHostAndBudget();
    puts("SERIAL_OTG_TX_PASS,groups=4,cases=12,productionFunctions=10,independentLanes=1,epochDrop=1,partialDelimiter=1,noHostBounded=1,cpuBudget=1,hardware=NOT_RUN");
    return 0;
}
