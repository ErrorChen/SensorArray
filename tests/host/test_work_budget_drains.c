/* Production task/drain bodies are inserted by tools/test_work_budget.py. */
#include <assert.h>
#include <arpa/inet.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayWorkBudget.c"
#include "sensorarrayUsbOtg.h"

#define SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT 2u
#define SENSORARRAY_TRANSPORT_DRAIN_BATCH_COUNT 8u
#define SENSORARRAY_TRANSPORT_SERIAL_COMMAND_MAX 96u
#define SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT 4u
#define SENSORARRAY_ASYNC_LOG_EVENT_QUEUE_COUNT 16u
#define SENSORARRAY_ASYNC_LOG_PROTOCOL_QUEUE_COUNT 8u
#define CONFIG_SENSORARRAY_LOG_TASK_CORE 0
#define CONFIG_SENSORARRAY_OUTPUT_ALLOW_NON_FRESH_DEBUG 0
#define SENSORARRAY_TRANSPORT_REPLY_SERIAL 1
#define SENSORARRAY_WIFI_CTRL_BUFFER 256u
#define SENSORARRAY_BLE_DATA_SERVICE_BURST 4u

typedef struct { unsigned source, token; } item_t;
typedef item_t sensorarrayBleTxDescriptor_t;
typedef item_t sensorarrayTransportDescriptor_t;
typedef item_t sensorarrayAsyncLogEvent_t;
typedef struct { unsigned connId, connectionGeneration; uint8_t data[8]; unsigned length; } sensorarrayBleControlCommand_t;
typedef void (*sensorarrayBleControlRxCallback_t)(const uint8_t *, size_t, void *);
typedef struct { int remaining; bool flood; unsigned source; } queue_t;
typedef queue_t *QueueHandle_t;
typedef void *TaskHandle_t;
typedef struct { int kind; } sensorarrayTransportReplyTarget_t;
static queue_t queues[9];
static QueueHandle_t s_eventQueue = &queues[0], s_protocolQueue = &queues[1];
static QueueHandle_t s_dataQueue = &queues[4], s_lifecycleQueue = &queues[5], s_logQueue = &queues[6];
static TaskHandle_t s_task = (void *)1, s_logTaskHandle = (void *)2;
static struct {
    QueueHandle_t ctrlTxQueue, txQueue, logTxQueue, ctrlRxQueue;
    void *txWake;
    struct { unsigned priorityDequeued, dataDequeued, logDequeued; } link;
    unsigned connId, connectionGeneration;
    struct { bool connected; unsigned controlRxStale; } stats;
    sensorarrayBleControlRxCallback_t controlCallback;
    void *controlContext;
} s_ble;
typedef struct { uint32_t ipv4; uint16_t port; } sensorarrayWifiPeer_t;
static struct {
    int ctrlSocket;
    struct { unsigned ctrlRx; } stats;
    void (*controlCallback)(const uint8_t *, size_t, const sensorarrayWifiPeer_t *, void *);
    void *controlContext;
} s_wifi;
static unsigned lockDepth, nowTick, delayCalls, notifications, takes;
static unsigned sent, released, owned, received[9], printed, published, consumed, commands, reads;
static unsigned sourceOrder[10000], orderLength, stopAfter, runKind;
static bool negativeRead, zeroRead, failSend, refillFrames;
static jmp_buf taskExit;
static esp_freertos_idle_cb_t idleHook;
static int s_bleMux, s_asyncLogMux;
#define portENTER_CRITICAL(mux) do { (void)(mux); lockDepth++; } while (0)
#define portEXIT_CRITICAL(mux) do { (void)(mux); assert(lockDepth); lockDepth--; } while (0)

static void order(unsigned source) { assert(orderLength < 10000u); sourceOrder[orderLength++] = source; }
TickType_t xTaskGetTickCount(void) { return nowTick; }
int xPortGetCoreID(void) { return 0; }
void vTaskDelay(TickType_t ticks)
{
    assert(lockDepth == 0u && ticks > 0u);
    nowTick += ticks; delayCalls++;
    /* No callback is faked here: lack of idle cannot trap a checkpoint. */
}
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t cpu)
{ assert(cpu == 0u); idleHook = cb; return ESP_OK; }
static int xTaskNotifyGive(TaskHandle_t task) { assert(task); notifications++; return pdTRUE; }
static int xSemaphoreTake(void *semaphore, TickType_t ticks)
{ assert(semaphore && ticks == portMAX_DELAY && !lockDepth); return pdTRUE; }
static void sensorarrayBleStatsIncrement(unsigned *counter) { (*counter)++; }
static uint32_t ulTaskNotifyTake(int clear, TickType_t ticks)
{
    assert(clear == pdTRUE && ticks > 0u && lockDepth == 0u); takes++;
    unsigned progress = runKind == 2u ? sent : (runKind == 3u ? printed : consumed);
    if (progress >= stopAfter) { longjmp(taskExit, 1); }
    return UINT32_MAX; /* Pure accumulated notification flood: never blocks. */
}
static int xQueueReceive(QueueHandle_t q, void *out, TickType_t wait)
{
    assert(q && lockDepth == 0u);
    if (runKind == 1u && sent >= stopAfter) { longjmp(taskExit, 1); }
    if (runKind == 5u && commands >= stopAfter) { longjmp(taskExit, 1); }
    if (!q->flood && q->remaining <= 0) {
        if (wait && wait != portMAX_DELAY) { nowTick += wait; }
        return 0;
    }
    if (!q->flood) { q->remaining--; }
    received[q->source]++;
    if (q == s_ble.ctrlRxQueue) {
        sensorarrayBleControlCommand_t command = {.connId=2u, .connectionGeneration=3u, .length=1u, .data={'?'}};
        *(sensorarrayBleControlCommand_t *)out = command;
    } else {
        *(item_t *)out = (item_t){.source=q->source, .token=received[q->source]};
        if (q == s_ble.ctrlTxQueue || q == s_ble.txQueue || q == s_ble.logTxQueue) { assert(owned == 0u); owned++; }
    }
    return pdTRUE;
}
static esp_err_t sensorarrayBleSendDescriptor(const sensorarrayBleTxDescriptor_t *desc)
{ assert(owned == 1u && lockDepth == 0u); sent++; order(desc->source); return failSend ? ESP_ERR_TIMEOUT : ESP_OK; }
static void sensorarrayBleReleaseTxDescriptor(const sensorarrayBleTxDescriptor_t *desc)
{ (void)desc; assert(owned == 1u); owned--; released++; }
static void sensorarrayTransportPrintNetInit(esp_err_t err) { assert(err == ESP_OK); }
static void sensorarrayTransportDrainGuaranteedText(void) { order(9u); }
static void sensorarrayTransportProcessDescriptor(sensorarrayTransportDescriptor_t *desc)
{ assert(lockDepth == 0u); sent++; released++; order(desc->source); }
static void sensorarrayTransportNotifyTask(void) { (void)xTaskNotifyGive(s_task); }
static void sensorarrayAsyncLogPrintEvent(sensorarrayAsyncLogEvent_t *event)
{ assert(lockDepth == 0u); printed++; order(event->source); assert(printed < 5000u); }
static void controlCallback(const uint8_t *data, size_t length, void *context)
{ assert(data && length && context == (void *)3 && lockDepth == 0u); commands++; }
static void wifiCallback(const uint8_t *data, size_t length, const sensorarrayWifiPeer_t *peer, void *context)
{
    assert(data && length == 7u && context == (void *)4 && lockDepth == 0u);
    assert(peer && peer->port == 3335u && peer->ipv4 == htonl(0xc0a80402u));
    commands++;
}
static ssize_t testRecvFrom(int socket, void *data, size_t capacity, int flags,
                            struct sockaddr *address, socklen_t *addressLength)
{
    assert(socket == 9 && capacity == 256u && flags == 0 && lockDepth == 0u);
    assert(*addressLength == sizeof(struct sockaddr_in));
    /* The task owns an endless socket service. The harness ends observation
     * after 80 completed calls; no claim that real socket error deletes it. */
    if (reads++ >= stopAfter) { longjmp(taskExit, 1); }
    if (negativeRead) { return -1; }
    if (zeroRead) { return 0; }
    struct sockaddr_in *peer = (struct sockaddr_in *)address;
    peer->sin_family = AF_INET;
    peer->sin_port = htons(3335u);
    peer->sin_addr.s_addr = htonl(0xc0a80402u);
    memcpy(data, "CONFIG?", 7u);
    return 7;
}
#define recvfrom testRecvFrom
static int sensorarraySerialIoReadWithEpoch(uint8_t *data, size_t length, uint32_t *epoch)
{
    assert(length == 64u && lockDepth == 0u);
    *epoch = 1u;
    if (reads++ >= stopAfter) { longjmp(taskExit, 1); }
    if (negativeRead) { return -1; }
    if (zeroRead) { return 0; }
    const char text[] = "MODE?\nROWS?\nCONFIG?\n";
    memcpy(data, text, sizeof(text) - 1u); return (int)sizeof(text) - 1;
}
static void sensorarraySerialIoNoteOversize(void) { assert(false); }
static esp_err_t sensorarrayTransportPublishControlReply(const sensorarrayTransportReplyTarget_t *target,
                                                        const char *text, size_t length)
{ assert(target && text && length); return ESP_OK; }
static esp_err_t sensorarrayTransportNormaliseSerialCommand(const char *line, size_t length,
                                                          char *command, size_t capacity, const char **reason)
{ assert(capacity > length); memcpy(command, line, length); command[length] = 0; *reason = "ok"; return ESP_OK; }
static esp_err_t sensorarrayTransportHandleControlCommand(const uint8_t *text, size_t length,
                                                         const sensorarrayTransportReplyTarget_t *target)
{ assert(text && length && target && lockDepth == 0u); commands++; return ESP_OK; }
static void sensorarrayTransportLogSerialCommandError(const uint8_t *text, size_t length, const char *reason)
{ (void)text; (void)length; (void)reason; assert(false); }

typedef struct { bool freshFrame, boundedCrcFailure; uint32_t sequence; uint64_t emitUs; } sensorarrayFrame_t;
typedef struct { uint32_t sequence; } sensorarrayTextPacket_t;
typedef struct { unsigned count; } summary_t;
typedef struct { sensorarrayFrame_t frame; sensorarrayTextPacket_t textPacket; uint64_t measureFrameUs; int64_t publishedUs; } slot_t;
typedef struct { sensorarrayFrame_t frame; sensorarrayTextPacket_t framePacket; summary_t summary; } sensorarrayAsyncLogWorkspace_t;
static slot_t slots[4], *s_frameSlots = slots;
static uint8_t s_frameSlotState[4], s_pendingSlots[4], s_pendingReadIndex, s_pendingCount;
static bool s_asyncLogUsbReady = true;
static sensorarrayAsyncLogWorkspace_t workspace, *s_asyncLogWorkspace = &workspace;
static uint32_t nextFrame;
static int64_t esp_timer_get_time(void) { return (int64_t)nowTick * 10000 + 1; }
static uint32_t sensorarrayCommandMailboxGetOutputFpsCap(void) { return 0u; }
static bool sensorarrayCommandMailboxTraceEnabled(void) { return false; }
static bool sensorarrayUsbSinkShouldEmitData(uint32_t sequence) { return sequence > 0u; }
static esp_err_t sensorarrayNetTextPublish(sensorarrayTextPacket_t *packet, bool data)
{ assert(packet->sequence > 0u && data && lockDepth == 0u); published++; return ESP_OK; }
static esp_err_t sensorarrayUsbSinkPublish(sensorarrayTextPacket_t *packet)
{ assert(packet->sequence > 0u && lockDepth == 0u); return ESP_OK; }
static uint64_t sensorarrayAsyncLogElapsedPositiveUs(int64_t start) { (void)start; return 1u; }
static void refillOneFrame(void)
{
    for (unsigned i = 0u; i < 4u; ++i) {
        if (s_frameSlotState[i] == 0u) {
            nextFrame++;
            slots[i] = (slot_t){.frame={.freshFrame=true, .sequence=nextFrame},
                              .textPacket={.sequence=nextFrame}, .measureFrameUs=123u, .publishedUs=1};
            s_frameSlotState[i] = 2u;
            s_pendingSlots[(s_pendingReadIndex + s_pendingCount) % 4u] = (uint8_t)i;
            s_pendingCount++; return;
        }
    }
    assert(false);
}
static void sensorarrayAsyncLogUpdateSummary(summary_t *summary, sensorarrayFrame_t *frame,
                                             uint64_t measure, uint64_t age, uint64_t output,
                                             uint32_t depth, bool emitted)
{
    (void)summary; (void)age; (void)output; (void)depth;
    assert(frame->sequence == consumed + 1u && measure == 123u && emitted);
    assert(lockDepth == 0u && workspace.framePacket.sequence == frame->sequence);
    for (unsigned i = 0u; i < 4u; ++i) { assert(s_frameSlotState[i] != 3u); }
    consumed++; assert(consumed < 5000u);
    if (refillFrames) { refillOneFrame(); }
}
static void sensorarrayAsyncLogMaybePrintSummary(summary_t *summary) { (void)summary; }

/* sensorarrayBleReceiveNext */
/* sensorarrayBleTxTask */
/* sensorarrayBleControlTask */
/* sensorarrayWifiControlTask */
/* sensorarrayTransportReceiveNext */
/* sensorarrayTransportTask */
/* sensorarrayTransportSerialControlTask */
/* sensorarrayAsyncLogPopFrame */
/* sensorarrayAsyncLogDrainEvents */
/* sensorarrayAsyncLogDrainProtocolEvents */
/* sensorarrayAsyncLogTask */

static void reset(void)
{
    memset(s_workers, 0, sizeof(s_workers));
    s_initState = s_initTick = s_idleGeneration = s_idleTick = s_maxIdleGapTicks = 0u;
    s_initError = ESP_ERR_INVALID_STATE;
    lockDepth = nowTick = delayCalls = notifications = takes = 0u;
    sent = released = owned = printed = published = consumed = commands = reads = 0u;
    orderLength = 0u; stopAfter = 80u; runKind = 0u;
    negativeRead = zeroRead = failSend = refillFrames = false;
    memset(received, 0, sizeof(received));
    memset(queues, 0, sizeof(queues));
    for (unsigned i = 0u; i < 9u; ++i) { queues[i].source = i; }
    memset(&s_ble, 0, sizeof(s_ble));
    s_ble.ctrlTxQueue = &queues[2]; s_ble.txQueue = &queues[3]; s_ble.ctrlRxQueue = &queues[7];
    s_ble.logTxQueue = &queues[8]; s_ble.txWake = (void *)1;
    s_ble.connId = 2u; s_ble.connectionGeneration = 3u; s_ble.stats.connected = true;
    s_ble.controlCallback = controlCallback; s_ble.controlContext = (void *)3;
    memset(&s_wifi, 0, sizeof(s_wifi));
    s_wifi.ctrlSocket = 9; s_wifi.controlCallback = wifiCallback; s_wifi.controlContext = (void *)4;
    memset(s_frameSlotState, 0, sizeof(s_frameSlotState));
    memset(&workspace, 0, sizeof(workspace));
    s_pendingReadIndex = s_pendingCount = 0u; nextFrame = 0u;
    assert(sensorarrayWorkBudgetInit() == ESP_OK);
}
static void testBle(bool control, bool failure)
{
    reset(); runKind = 1u; failSend = failure;
    queues[control ? 2 : 3].flood = true;
    if (control) { queues[3].flood = true; }
    if (setjmp(taskExit) == 0) { sensorarrayBleTxTask(NULL); }
    assert(sent == stopAfter && released == sent && owned == 0u && delayCalls > 0u);
    assert(received[control ? 2 : 3] == sent);
    if (control) { assert(received[3] == 0u); }
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_TX].checkpoints >= sent);
}
static void testTransport(bool lifecycle)
{
    reset(); runKind = 2u;
    queues[4].flood = queues[6].flood = true; queues[5].flood = lifecycle;
    if (setjmp(taskExit) == 0) { sensorarrayTransportTask(NULL); }
    assert(sent == stopAfter && released == sent &&
           notifications == stopAfter / SENSORARRAY_TRANSPORT_DRAIN_BATCH_COUNT);
    assert(delayCalls > 1u && takes == stopAfter / SENSORARRAY_TRANSPORT_DRAIN_BATCH_COUNT + 1u);
    for (unsigned i = 0u; i < sent; ++i) {
        assert(sourceOrder[2u * i] == 9u); /* Guaranteed terminal precedes data. */
        if (lifecycle) { assert(sourceOrder[2u * i + 1u] == 5u); }
    }
    if (lifecycle) { assert(received[4] == 0u && received[6] == 0u); }
    else { assert(received[4] > 0u && received[6] > 0u); }
}
static void testLog(bool frames)
{
    reset(); runKind = frames ? 4u : 3u;
    queues[0].flood = queues[1].flood = true;
    if (frames) { refillFrames = true; for (unsigned i = 0u; i < 4u; ++i) { refillOneFrame(); } }
    if (setjmp(taskExit) == 0) { sensorarrayAsyncLogTask(NULL); }
    assert(printed >= stopAfter && notifications > 0u && takes > 1u && delayCalls > 0u);
    for (unsigned i = 0u; i < 8u; ++i) { assert(sourceOrder[i] == 1u); }
    assert(received[0] > 0u && received[1] > 0u);
    if (frames) { assert(consumed == stopAfter && published == consumed); }
    else { assert(consumed == 0u); }
    assert(lockDepth == 0u);
}
static void testControl(bool ble, int readResult)
{
    reset(); runKind = ble ? 5u : 6u;
    if (ble) {
        queues[7].flood = true;
        if (setjmp(taskExit) == 0) { sensorarrayBleControlTask(NULL); }
        assert(commands == stopAfter && s_workers[SENSORARRAY_WORK_BUDGET_BLE_CTRL].checkpoints >= commands);
    } else {
        negativeRead = readResult < 0; zeroRead = readResult == 0;
        if (setjmp(taskExit) == 0) { sensorarrayTransportSerialControlTask(NULL); }
        assert(commands == (readResult > 0 ? 3u * stopAfter : 0u));
        if (readResult <= 0) { assert(delayCalls == stopAfter); } /* No double delay. */
    }
    assert(delayCalls > 0u && lockDepth == 0u);
}
static void testWifiControl(int readResult)
{
    reset(); runKind = 7u;
    negativeRead = readResult < 0; zeroRead = readResult == 0;
    if (setjmp(taskExit) == 0) { sensorarrayWifiControlTask(NULL); }
    assert(reads == stopAfter + 1u && commands == (readResult > 0 ? stopAfter : 0u));
    assert(s_wifi.stats.ctrlRx == commands && delayCalls > 0u && lockDepth == 0u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_WIFI_CTRL].checkpoints == stopAfter + 1u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_WIFI_CTRL].maxBurstUnits <= 32u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_BLE_CTRL].checkpoints == 0u);
    assert(s_workers[SENSORARRAY_WORK_BUDGET_SERIAL_CTRL].checkpoints == 0u);
}
int main(void)
{
    testBle(true, false); testBle(false, false); testBle(true, true); testBle(false, true);
    testTransport(true); testTransport(false);
    testLog(false); testLog(true);
    testControl(true, 1); testControl(false, 1); testControl(false, 0); testControl(false, -1);
    testWifiControl(1); testWifiControl(0); testWifiControl(-1);
    puts("WORK_BUDGET_DRAINS_PASS,scenarios=15,production_bodies=10,hardware=NOT_RUN");
    return 0;
}
