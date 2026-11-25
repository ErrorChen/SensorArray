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
typedef struct { unsigned count, head, tail, capacity; uint8_t bytes[SERIAL_IO_CDC_DATA_QUEUE_COUNT]; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef void *TaskHandle_t;
static unsigned lockDepth;
#define portENTER_CRITICAL(p) do { (void)(p); assert(!lockDepth); lockDepth++; } while (0)
#define portEXIT_CRITICAL(p) do { (void)(p); assert(lockDepth == 1u); lockDepth--; } while (0)
static char dataBytes[SERIAL_IO_CDC_NORMAL_SLOT_COUNT][SENSORARRAY_TRANSPORT_TEXT_MAX];
static char (*s_dataBytes)[SENSORARRAY_TRANSPORT_TEXT_MAX] = dataBytes;
static char s_priorityBytes[SERIAL_IO_PRIORITY_SLOT_COUNT][SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
static unsigned s_dataSlotCount = SERIAL_IO_DATA_SLOT_COUNT;
static uint32_t s_dataLifetimeUs = SERIAL_IO_NORMAL_LIFETIME_US;
static sensorarraySerialSlot_t s_smallSlots[SERIAL_IO_DATA_SLOT_COUNT + SERIAL_IO_PRIORITY_SLOT_COUNT + SERIAL_IO_LOG_SLOT_COUNT];
static sensorarraySerialSlot_t largeSlots[SERIAL_IO_CDC_DATA_SLOT_COUNT];
static sensorarraySerialSlot_t *s_dataSlots = s_smallSlots;
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
static unsigned timedProduced, timedTarget;
static uint64_t nextProductionUs;
static char timedExpected[300000];
static size_t timedExpectedLength;
static bool injectAfterWrite;
static bool cdcBackend;
static bool invalidAcceptPrefix;
static int writeBudget[3];
static char wire[3][300000], oldWire[3][300000];
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
    if (queue->count == queue->capacity) { return 0; }
    queue->bytes[queue->tail++ % queue->capacity] = *(const uint8_t *)item;
    queue->count++; return pdTRUE;
}
static int xQueueReceive(QueueHandle_t queue, void *item, TickType_t ticks)
{
    assert(!lockDepth && ticks == 0u);
    if (!queue->count) { return 0; }
    *(uint8_t *)item = queue->bytes[queue->head++ % queue->capacity];
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
static bool sensorarrayUsbOtgCdcEnabled(void) { return cdcBackend; }
static uint32_t sensorarrayUsbOtgEpoch(void) { return currentEpoch; }
static esp_err_t sensorarrayUsbOtgOwnerInit(void)
{ assert(!lockDepth && ownerInitCalls++ == 0u); return ESP_OK; }
static void sensorarrayUsbOtgOwnerPoll(void);
static bool sensorarrayUsbOtgOwnerCanWrite(unsigned lane)
{
    assert(!lockDepth && lane < 3u);
    if (cdcBackend && lane != SERIAL_IO_LOG) { lane = SERIAL_IO_DATA; }
    return allowed[lane] && (writeBudget[lane] != 0 || nowUs >= readyAt[lane]);
}
static int sensorarrayUsbOtgOwnerWrite(unsigned lane, const void *data, size_t length)
{
    assert(!lockDepth && lane < 3u && data && length > 0u);
    bool wholeData = cdcBackend && lane == SERIAL_IO_DATA;
    if (cdcBackend && lane != SERIAL_IO_LOG) { lane = SERIAL_IO_DATA; }
    writes++;
    if (invalidMode) {
        unsigned invalid = invalidMode; invalidMode = 0u;
        if (invalidAcceptPrefix) { assert(length>=3);memcpy(wire[lane]+wireLength[lane],data,3);wireLength[lane]+=3; }
        return invalid == 1u ? -1 : invalid == 3u ? 3 : (int)length + 1;
    }
    if (nowUs >= readyAt[lane]) { writeBudget[lane] = -1; }
    if (writeBudget[lane] == 0 || (wholeData && writeBudget[lane] > 0 && length > (unsigned)writeBudget[lane])) { return 0; }
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
    while (timedProduced < timedTarget && nowUs >= nextProductionUs) {
        char record[SENSORARRAY_TRANSPORT_TEXT_MAX];
        memset(record, 'A' + timedProduced % 26u, sizeof(record));
        record[sizeof(record) - 1u] = '\n';
        assert(sensorarraySerialIoQueueData(record, sizeof(record)) == ESP_OK);
        memcpy(timedExpected + timedExpectedLength, record, sizeof(record));
        timedExpectedLength += sizeof(record);
        timedProduced++; nextProductionUs += 10000u;
    }
    if (rxPending) {
        char command[64]; uint32_t epoch = 0u;
        assert(sensorarraySerialIoReadWithEpoch(command, sizeof(command), &epoch) == 8);
        assert(!memcmp(command, "CONFIG?\n", 8u) && epoch == currentEpoch);
        rxCommands++; offer(SERIAL_IO_PRIORITY, "CONFIG_ACK\n");
    }
    if (allReleased() && (!productionTarget || produced == productionTarget) && timedProduced == timedTarget) { longjmp(ownerEnd, 1); }
}
static void reset(void)
{
    memset(&s_stats, 0, sizeof(s_stats)); memset(queues, 0, sizeof(queues)); memset(s_inUse, 0, sizeof(s_inUse)); memset(s_smallSlots, 0, sizeof(s_smallSlots)); memset(largeSlots, 0, sizeof(largeSlots));
    s_dataSlotCount = SERIAL_IO_DATA_SLOT_COUNT; s_dataLifetimeUs = SERIAL_IO_NORMAL_LIFETIME_US; s_dataSlots = s_smallSlots; memset(wire, 0, sizeof(wire)); memset(wireLength, 0, sizeof(wireLength));
    memset(oldWire, 0, sizeof(oldWire)); memset(oldWireLength, 0, sizeof(oldWireLength));
    memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    nowUs = injectAt = 0u; currentEpoch = 1u;
    polls = delays = waits = notifications = ownerInitCalls = writes = rxCalls = rxCommands = 0u;
    partialLane = invalidMode = produced = productionTarget = 0u;
    timedProduced = timedTarget = 0u; nextProductionUs = 0u; timedExpectedLength = 0u;
    rxPending = injectDone = resetAfterWrite = resetDone = injectAfterWrite = cdcBackend = invalidAcceptPrefix = false;
    for (unsigned i=0u; i<3u; i++) {
        s_queues[i] = &queues[i]; queues[i].capacity = SERIAL_IO_QUEUE_COUNT; allowed[i] = true; readyAt[i] = 0u; writeBudget[i] = -1;
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
        reset(); invalidMode = bad; invalidAcceptPrefix=true;offer(SERIAL_IO_DATA, "INVALID\n");offer(SERIAL_IO_DATA,"VALID\n");run();
        assert(writes == 3u && s_stats.driverWriteError == 1u && s_stats.dataSubmitted == 1u && s_stats.sinkDropped==1u);
        assert(s_stats.driverAcceptedBytes == 7u && !strcmp(wire[0], "INV\nVALID\n"));
    }
}
static void testCdcRecordBoundaries(void)
{
    reset(); cdcBackend = true; allowed[SERIAL_IO_LOG] = false;
    offer(SERIAL_IO_LOG, "UNREAD_DEBUG\n");
    for (unsigned i=0; i<4; i++) { offer(SERIAL_IO_DATA,"DATA\n");offer(SERIAL_IO_PRIORITY,"CTRL\n"); }
    run();assert(!strcmp(wire[0],"CTRL\nCTRL\nCTRL\nCTRL\nDATA\nDATA\nDATA\nDATA\n"));
    assert(wireLength[1]==0&&wireLength[2]==0&&s_stats.normalExpired==1&&s_stats.prioritySubmitted==4&&s_stats.dataSubmitted==4);

    reset(); cdcBackend = true; writeBudget[0]=3;readyAt[0]=20000;injectAfterWrite=true;
    offer(SERIAL_IO_DATA,"ABCDEFGH\n");offer(SERIAL_IO_LOG,"DEBUG\n");run();
    assert(injectDone&&!strcmp(wire[0],"CTRL_AFTER\nABCDEFGH\n")&&!strcmp(wire[2],"DEBUG\n"));
    assert(s_stats.driverShortWrite==1&&s_stats.timeout==0&&s_stats.prioritySubmitted==1&&s_stats.dataSubmitted==1);

    reset(); cdcBackend = true;writeBudget[0]=3;readyAt[0]=20000;
    offer(SERIAL_IO_DATA,"DATA_AFTER\n");offer(SERIAL_IO_PRIORITY,"ABCDEFGH\n");run();
    assert(!strcmp(wire[0],"ABCDEFGH\nDATA_AFTER\n")&&s_stats.timeout==0);

    reset(); cdcBackend = true;writeBudget[0]=3;readyAt[0]=SERIAL_IO_NORMAL_LIFETIME_US+50000u;injectAfterWrite=true;
    offer(SERIAL_IO_DATA,"ABCDEFGH\n");offer(SERIAL_IO_LOG,"INDEPENDENT\n");run();
    assert(!strcmp(wire[0],"CTRL_AFTER\n")&&!strcmp(wire[2],"INDEPENDENT\n"));
    assert(s_stats.partialWrite==0&&s_stats.normalExpired==1&&s_stats.prioritySubmitted==1&&s_stats.timeout==1);

    reset();cdcBackend=true;writeBudget[0]=3;readyAt[0]=SERIAL_IO_PRIORITY_LIFETIME_US+50000u;
    injectAt=SERIAL_IO_PRIORITY_LIFETIME_US-50000u;partialLane=SERIAL_IO_DATA;
    offer(SERIAL_IO_PRIORITY,"ABCDEFGH\n");run();
    assert(!strcmp(wire[0],"ABC\nNEW\n")&&s_stats.priorityExpired==1&&s_stats.dataSubmitted==1&&s_stats.partialWrite==1);

    reset();cdcBackend=true;injectAfterWrite=true;offer(SERIAL_IO_DATA,"PREFIX_WITHOUT_LF");run();
    assert(!strcmp(wire[0],"PREFIX_WITHOUT_LF\nCTRL_AFTER\n")&&s_stats.partialWrite==0);
}
static void testCdcEpochAndBudget(void)
{
    reset();cdcBackend=true;
    offer(SERIAL_IO_DATA,"OLD_DATA\n");offer(SERIAL_IO_PRIORITY,"OLD_CTRL\n");offer(SERIAL_IO_LOG,"OLD_LOG\n");currentEpoch++;
    offer(SERIAL_IO_DATA,"NEW_DATA\n");offer(SERIAL_IO_PRIORITY,"NEW_CTRL\n");offer(SERIAL_IO_LOG,"NEW_LOG\n");run();
    assert(!strcmp(wire[0],"NEW_CTRL\nNEW_DATA\n")&&!strcmp(wire[2],"NEW_LOG\n"));
    assert(s_stats.timeout==0&&s_stats.normalExpired==2&&s_stats.priorityExpired==1);
    for (unsigned partial=0;partial<2;partial++) {
        reset();cdcBackend=true;resetAfterWrite=true;readyAt[0]=UINT64_MAX;writeBudget[0]=partial?3:-1;
        offer(SERIAL_IO_DATA,"OLD_PREFIX");run();
        assert(oldWireLength[0]==(partial?0u:10u)&&!strcmp(wire[0],"NEW_CTRL\nNEW_DATA\n")&&wire[0][0]!='\n');
        assert(s_stats.timeout==0&&s_stats.partialWrite==0u);
    }
    reset();cdcBackend=true;
    for(unsigned lane=0;lane<3;lane++){allowed[lane]=false;for(unsigned i=0;i<4;i++)offer(lane,"EXPIRE\n");}
    run();assert(writes==0&&s_stats.normalExpired==8&&s_stats.priorityExpired==4&&s_stats.timeout==12);
    assert(nowUs<=SERIAL_IO_PRIORITY_LIFETIME_US+2u*SERIAL_IO_TICK_US&&waits>0);
    reset();cdcBackend=true;productionTarget=100;run();
    assert(s_stats.dataSubmitted==100&&s_stats.sinkDropped==0&&s_stats.txYield>0&&delays>0);
    for (unsigned bad=1;bad<=2;bad++) {
        reset();cdcBackend=true;invalidMode=bad;invalidAcceptPrefix=true;injectAfterWrite=true;
        offer(SERIAL_IO_DATA,"UNKNOWN_PREFIX\n");run();
        assert(injectDone&&!strcmp(wire[0],"UNK\nCTRL_AFTER\n"));
        assert(s_stats.sinkDropped==1&&s_stats.driverWriteError==1&&s_stats.dataSubmitted==0&&s_stats.prioritySubmitted==1);
        assert(s_stats.driverAcceptedBytes==12&&s_stats.partialWrite==0&&s_stats.timeout==0);
    }
}
static void largeCdcPool(void)
{
    cdcBackend = true;
    s_dataSlotCount = SERIAL_IO_CDC_DATA_SLOT_COUNT;
    s_dataLifetimeUs = SERIAL_IO_CDC_DATA_LIFETIME_US;
    s_dataSlots = largeSlots;
    queues[SERIAL_IO_DATA].capacity = SERIAL_IO_CDC_DATA_QUEUE_COUNT;
}
static void testCdcMeasuredPauseBurst(void)
{
    const uint64_t pauses[] = {622000u, 800000u};
    for (unsigned trial = 0; trial < sizeof(pauses) / sizeof(pauses[0]); ++trial) {
        reset(); largeCdcPool(); timedTarget = 100u;
        writeBudget[0] = 0; readyAt[0] = pauses[trial];
        rxPending = true; offer(SERIAL_IO_LOG, "DEBUG_DURING_PAUSE\n");
        run();
        assert(timedProduced == 100u && s_stats.dataSubmitted == 100u);
        assert(s_stats.sinkDropped == 0u && s_stats.timeout == 0u && s_stats.partialWrite == 0u);
        assert(s_stats.prioritySubmitted == 1u && rxCommands == 1u);
        assert(!memcmp(wire[0], "CONFIG_ACK\n", 11u));
        assert(wireLength[0] == 11u + timedExpectedLength);
        assert(!memcmp(wire[0] + 11u, timedExpected, timedExpectedLength));
        assert(!strcmp(wire[2], "DEBUG_DURING_PAUSE\n") && s_stats.txYield > 0u);
        printf("SERIAL_CDC_BURST_PASS,pauseUs=%llu,sourceFps=100,records=100,recordBytes=%u,exactBytes=1,dataDrop=0,partial=0,expiry=0,controlFirst=1,debugIndependent=1,model=synthetic_no_hardware\n",
               (unsigned long long)pauses[trial], SENSORARRAY_TRANSPORT_TEXT_MAX);
    }
    /* FIFO has a positive prefix-sized gap, but no complete DATA record can
     * fit. The waiting unstarted record must not lock out a smaller CTRL. */
    reset(); largeCdcPool(); writeBudget[0] = 12; readyAt[0] = 622000u;
    offer(SERIAL_IO_DATA, "DATA_IS_LONGER_THAN_THE_FIFO_GAP\n");
    offer(SERIAL_IO_PRIORITY, "CTRL\n"); offer(SERIAL_IO_LOG, "LOG\n");
    run(); assert(!strcmp(wire[0], "CTRL\nDATA_IS_LONGER_THAN_THE_FIFO_GAP\n"));
    assert(s_stats.partialWrite == 0u && s_stats.driverShortWrite == 0u && s_stats.timeout == 0u);
    assert(!strcmp(wire[2], "LOG\n"));
}
static void testCdcBurstBounds(void)
{
    reset(); largeCdcPool();
    for (unsigned i = 0; i < SERIAL_IO_CDC_DATA_QUEUE_COUNT; ++i) { offer(SERIAL_IO_DATA, "EXPIRE\n"); }
    assert(sensorarraySerialIoQueueData("FULL\n", 5u) == ESP_ERR_TIMEOUT);
    assert(s_stats.dataRejected == 1u);
    for (unsigned lane = 0; lane < 3u; ++lane) { allowed[lane] = false; }
    offer(SERIAL_IO_PRIORITY, "CTRL\n"); offer(SERIAL_IO_LOG, "LOG\n");
    run(); assert(writes == 0u && s_stats.partialWrite == 0u);
    assert(s_stats.normalExpired == SERIAL_IO_CDC_DATA_QUEUE_COUNT + 1u && s_stats.priorityExpired == 1u);
    assert(nowUs <= SERIAL_IO_CDC_DATA_LIFETIME_US + 100000u && waits > 0u && delays > 0u);

    reset(); largeCdcPool(); writeBudget[0] = 3; readyAt[0] = 1100000u;
    offer(SERIAL_IO_DATA, "NO_PREFIX_BEFORE_EXPIRY\n");
    offer(SERIAL_IO_LOG, "INDEPENDENT\n");
    run(); assert(wireLength[0] == 0u && s_stats.dataSubmitted == 0u);
    assert(s_stats.normalExpired == 1u && s_stats.timeout == 1u && s_stats.partialWrite == 0u);
    assert(!strcmp(wire[2], "INDEPENDENT\n"));

    reset(); largeCdcPool();
    for (unsigned i = 0; i < 100u; ++i) { offer(SERIAL_IO_DATA, "OLD\n"); }
    currentEpoch++; run();
    assert(wireLength[0] == 0u && s_stats.normalExpired == 100u && s_stats.timeout == 0u);
    assert(s_stats.partialWrite == 0u);

    /* Defensive SDK-contract violation: positive short acceptance is counted
     * immediately, never kept alive to become a normal DATA prefix expiry. */
    reset(); largeCdcPool(); invalidMode = 3u; invalidAcceptPrefix = true;
    offer(SERIAL_IO_DATA, "UNKNOWN\n"); offer(SERIAL_IO_DATA, "NEXT\n");
    run(); assert(!strcmp(wire[0], "UNK\nNEXT\n"));
    assert(s_stats.driverWriteError == 1u && s_stats.driverShortWrite == 1u && s_stats.partialWrite == 1u);
    assert(s_stats.normalExpired == 0u && s_stats.timeout == 0u && s_stats.dataSubmitted == 1u);
    puts("SERIAL_CDC_BOUND_PASS,queue=100,overflowCounted=1,noHostBounded=1,noPrefixExpiry=1,epochDiscard=1,unexpectedPrefixCounted=1,model=synthetic_no_hardware");
}
int main(void)
{
    testIndependentLanes(); testPartialEachLane(); testEpochs(); testNoHostAndBudget();
    testCdcRecordBoundaries();testCdcEpochAndBudget();
    testCdcMeasuredPauseBurst(); testCdcBurstBounds();
    puts("SERIAL_OTG_TX_PASS,groups=8,cases=32,vendorCases=12,cdcCases=20,productionFunctions=11,independentDebug=1,sharedFormalRecordAtomic=1,epochDrop=1,partialDelimiter=1,unknownAcceptanceDelimiter=1,noHostBounded=1,cpuBudget=1,hardware=NOT_RUN");
    return 0;
}
