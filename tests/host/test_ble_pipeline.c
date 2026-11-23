/* A causal two-core arrival/one-core scheduling model. Production functions
 * are included without editing their bodies. Peripheral costs are synthetic;
 * neither these timings nor a PASS are HIL throughput or CPU measurements. */
#define _XOPEN_SOURCE 700
#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ucontext.h>
#include "pipeline_config.h"
#include "sensorarrayBle.h"
#include "sensorarrayBleWatchdog.h"
#include "sensorarrayTransport.h"
#include "sensorarrayTransportPool.h"
#include "sensorarrayTransportFrameMeta.h"
#include "sensorarrayTextProtocol.h"
#include "sensorarrayFrameBuilder.h"
#include "sensorarrayOutputPolicy.h"
#include "sensorarrayFramePublication.h"
#include "sensorarrayWorkBudget.c"

typedef int esp_gatt_if_t;
typedef int esp_gatt_status_t;
typedef uint8_t esp_bd_addr_t[6];
#define ESP_GATT_OK 0
#define ESP_GATT_ERROR 1
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT 1
#include "ble_types.h"
#include "transport_constants.h"
#include "async_types.h"

enum { ASYNC, TRANSPORT, BLE, CONTROL, BT, TASKS };
enum { Q_NONE, Q_TR_DATA, Q_TR_LOG, Q_TR_LIFE, Q_BLE_DATA, Q_BLE_LOG, Q_BLE_CTRL };
enum { BURST, SHORT, BT_PREEMPT, LONG_WAIT, REJECT, UNREAD, CONTROL_FLOW,
       SDK_BUSY, BT_SATURATED, NEVER_RETURNS, GUARDED_NEVER_RETURNS, SCENARIOS };
typedef struct {
    ucontext_t context;
    uint8_t stack[131072];
    uint64_t readyAt, order;
    unsigned priority, notifications;
    bool waitNotify;
} modelTask_t;
static modelTask_t tasks[TASKS];
static ucontext_t scheduler;
static int current = -1;
static bool sourceContext, inAdvance;
static uint64_t nowUs, readyOrder, sourceStart = 1000000u, nextSourceUs;
static unsigned scenario, sourceGoal, controlGoal, sourceCount, asyncPopped;
static unsigned transportQueued[3], transportRejected[3], transportProcessed[3];
static unsigned transportNoSink[3];
static unsigned bleQueued[3], bleStarted[3], bleFinished[3], sdkAccepted[3], receiverRecords[3];
static unsigned controlCalls, controlAccepted, controlRejected, sdkCalls, sdkErrors, btJobs;
static uint64_t controlAt[256], controlMaxDispatch, controlMaxLatency, poolHeldMaxUs;
static unsigned sdkWhilePoolHeld, idleVisits, sdkNeverOutstanding, modelSwitches;
static uint64_t nextGuardUs;
static unsigned guardPolls;
static bool trActive, bleActive;
static sensorarrayTransportDescriptor_t activeTransport;
static sensorarrayBleTxDescriptor_t activeBle;
static sensorarrayTextPacket_t expectedPackets[257];
static bool scienceAccepted[257];
unsigned pipelineLockDepth;
static esp_freertos_idle_cb_t idleHook;

static sensorarrayTransportPool_t s_pool;
static sensorarrayTransportStats_t g_sensorarrayTransportStats;
static portMUX_TYPE g_sensorarrayTransportStatsMux;
static StaticQueue_t trQueues[3];
static uint8_t trQueueBytes[3][2u * sizeof(sensorarrayTransportDescriptor_t)];
static QueueHandle_t s_dataQueue, s_logQueue, s_lifecycleQueue, s_protectedQueue;
static bool s_protectedRetryActive, s_lastBleFrameKnown;
static sensorarrayTransportDescriptor_t s_protectedRetry;
static sensorarrayTransportProtectionStats_t s_transportProtectionStats;
static bool s_protectedAdmissionKnown;
static sensorarrayTransportFrameIdentity_t s_protectedAdmissionIdentity;
static uint64_t s_lastProtectedAdmissionUs;
static uint32_t s_lastBleConnectionGeneration;
static sensorarrayTransportFrameIdentity_t s_lastBleFrame;
static TaskHandle_t s_task, s_logTaskHandle;
static sensorarrayAsyncLogFrameSlot_t asyncSlots[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static sensorarrayAsyncLogFrameSlot_t *s_frameSlots = asyncSlots;
static uint8_t s_frameSlotState[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static uint8_t s_pendingSlots[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static uint8_t s_pendingReadIndex, s_pendingCount;
static portMUX_TYPE s_asyncLogMux;
static sensorarrayAsyncLogSharedStats_t s_sharedStats;
static uint32_t s_frameSlotProtection[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static sensorarrayTransportFrameIdentity_t s_frameSlotIdentity[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static sensorarrayAsyncLogProtectionStats_t s_asyncProtectionStats;
static bool s_protectedIdentityKnown;
static sensorarrayTransportFrameIdentity_t s_protectedIdentity;
static int64_t s_lastProtectedPublishedUs;

static void reschedule(void);
static void blockUntil(uint64_t until);
static void consume(unsigned us);
static void advanceTo(uint64_t until);
static void queueObserved(QueueHandle_t queue, const void *value);
static bool sensorarrayAsyncLogIsRunning(void) { return true; }
bool sensorarrayTransportBleChannelEnabled(sensorarrayTransportChannel_t c) { (void)c; return true; }
bool sensorarrayTransportWifiSinkEnabled(void) { return false; }
bool sensorarrayWifiIsReady(void) { return false; }
bool sensorarrayWifiGetSendEpoch(bool control,uint32_t *epoch) { (void)control;if(epoch)*epoch=0;return false; }
bool sensorarrayWifiOwnerNeedsPoll(void) { return false; }
esp_err_t sensorarrayWifiSubmit(const sensorarrayWifiPublication_t *p) { (void)p;assert(!"Wi-Fi disabled in BLE pipeline");return ESP_ERR_INVALID_STATE; }
static void sensorarrayTransportWifiDrainTerminal(void) {}
sensorarrayTransportTxMode_t sensorarrayTransportGetTxMode(void) { return SENSORARRAY_TRANSPORT_TX_REL; }
static bool sensorarrayTransportFindFieldU32(const char *p, const char *field, uint32_t *out)
{ (void)p; (void)field; (void)out; assert(false); return false; }
static void sensorarrayTransportPrintNetInit(esp_err_t err) { assert(err == ESP_OK); }
/* This fixture injects control replies through the real BLE priority API.
 * There are no guaranteed terminal-mailbox events; that mailbox has separate
 * production regressions. No fake drain creates descriptors here. */
static void sensorarrayTransportDrainGuaranteedText(void) {}

TickType_t xTaskGetTickCount(void) { return (TickType_t)(nowUs / 10000u); }
int xPortGetCoreID(void) { return sourceContext ? 1 : 0; }
int64_t esp_timer_get_time(void) { return (int64_t)nowUs; }
TaskHandle_t xTaskGetCurrentTaskHandle(void) { assert(current >= 0); return &tasks[current]; }
UBaseType_t uxTaskPriorityGet(TaskHandle_t task) { return ((modelTask_t *)task)->priority; }
esp_err_t esp_register_freertos_idle_hook_for_cpu(esp_freertos_idle_cb_t cb, UBaseType_t cpu)
{ assert(cpu == 0u); idleHook = cb; return ESP_OK; }

static void wake(modelTask_t *task)
{
    if (task->readyAt > nowUs) { task->readyAt = nowUs; task->order = ++readyOrder; }
    if (current >= 0 && !sourceContext && !inAdvance && task->priority > tasks[current].priority) {
        reschedule();
    }
}

int xTaskNotifyGive(TaskHandle_t handle)
{
    modelTask_t *task = handle; assert(task && !pipelineLockDepth);
    task->notifications++;
    if (task->waitNotify) { wake(task); }
    return pdTRUE;
}
uint32_t ulTaskNotifyTake(int clear, TickType_t timeout)
{
    assert(clear == pdTRUE && !pipelineLockDepth && current >= 0);
    modelTask_t *task = &tasks[current];
    uint64_t end = timeout == portMAX_DELAY ? UINT64_MAX : nowUs + (uint64_t)timeout * 10000u;
    while (!task->notifications && nowUs < end) { task->waitNotify = true; blockUntil(end); }
    task->waitNotify = false;
    unsigned result = task->notifications; task->notifications = 0u;
    return result;
}
void vTaskDelay(TickType_t ticks)
{
    assert(current >= 0 && !pipelineLockDepth && ticks);
    blockUntil((nowUs / 10000u + ticks) * 10000u);
}
QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *storage, StaticQueue_t *queue)
{ *queue = (StaticQueue_t){.storage=storage,.capacity=count,.itemSize=size}; return queue; }
int xQueueSend(QueueHandle_t q, const void *item, TickType_t timeout)
{
    assert(q && !pipelineLockDepth && timeout == 0u);
    if (q->count == q->capacity) { return pdFALSE; }
    memcpy(q->storage + ((q->head + q->count) % q->capacity) * q->itemSize, item, q->itemSize);
    q->count++; queueObserved(q, item); return pdTRUE;
}
int xQueueSendToFront(QueueHandle_t q,const void *item,TickType_t timeout)
{
    assert(q && !pipelineLockDepth && !timeout);
    if(q->count==q->capacity)return pdFALSE;
    q->head=(q->head+q->capacity-1u)%q->capacity;
    memcpy(q->storage+q->head*q->itemSize,item,q->itemSize);q->count++;return pdTRUE;
}
int xQueueReceive(QueueHandle_t q, void *out, TickType_t timeout)
{
    assert(q && !pipelineLockDepth && timeout == 0u);
    if (!q->count) { return pdFALSE; }
    memcpy(out, q->storage + q->head * q->itemSize, q->itemSize);
    q->head = (q->head + 1u) % q->capacity; q->count--; return pdTRUE;
}
SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *s)
{ *s = (StaticSemaphore_t){.waiter=-1}; return s; }
int xSemaphoreGive(SemaphoreHandle_t s)
{
    assert(s && !pipelineLockDepth); s->token = 1u;
    if (s->waiter >= 0) { wake(&tasks[s->waiter]); }
    return pdTRUE;
}
int xSemaphoreTake(SemaphoreHandle_t s, TickType_t timeout)
{
    assert(s && !pipelineLockDepth && timeout == portMAX_DELAY && current == BLE);
    while (!s->token) { s->waiter = current; blockUntil(UINT64_MAX); }
    s->waiter = -1; s->token = 0u; return pdTRUE;
}
static esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t interface, uint16_t conn,
    uint16_t handle, uint16_t length, uint8_t *value, bool confirm);

#define MALLOC_CAP_SPIRAM 1u
#define MALLOC_CAP_8BIT 2u
static uint8_t blePayloadStorage[SENSORARRAY_BLE_PAYLOAD_STORAGE_BYTES];
static void *heap_caps_malloc(size_t bytes, uint32_t caps)
{
    assert(!pipelineLockDepth && bytes==sizeof blePayloadStorage && caps==(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    return blePayloadStorage;
}
#include "ble_functions.h"
#define sensorarrayBleSendDescriptor actualBleSendDescriptor
#include "ble_send.h"
#undef sensorarrayBleSendDescriptor
static esp_err_t sensorarrayBleSendDescriptor(const sensorarrayBleTxDescriptor_t *desc)
{
    assert(!bleActive); bleActive = true; activeBle = *desc;
    bleStarted[desc->channel]++;
    assert(bleStarted[desc->channel] <= bleQueued[desc->channel]);
    esp_err_t result = actualBleSendDescriptor(desc);
    assert(sensorarrayBleDescriptorCurrent(desc)); /* It is released only by owner after return. */
    bleFinished[desc->channel]++; bleActive = false;
    return result;
}
#include "ble_owner.h"
#include "transport_helpers.h"
#define sensorarrayTransportProcessDescriptor actualTransportProcessDescriptor
#include "transport_process.h"
#undef sensorarrayTransportProcessDescriptor
static void sensorarrayTransportProcessDescriptor(const sensorarrayTransportDescriptor_t *desc)
{
    assert(!trActive); trActive = true; activeTransport = *desc;
    uint64_t start = nowUs;
    transportProcessed[desc->channel]++;
    actualTransportProcessDescriptor(desc);
    assert(!s_pool.slots[desc->slotIndex].inUse ||
           s_pool.slots[desc->slotIndex].slotGeneration != desc->slotGeneration);
    if (nowUs - start > poolHeldMaxUs) { poolHeldMaxUs = nowUs - start; }
    trActive = false;
}
#include "transport_owner.h"
#include "async_functions.h"

static void queueObserved(QueueHandle_t q, const void *value)
{
    if (q->tag >= Q_BLE_DATA && q->tag <= Q_BLE_CTRL) {
        const sensorarrayBleTxDescriptor_t *desc = value;
        assert(sensorarrayBleDescriptorCurrent(desc));
        const uint8_t *payload = sensorarrayBleDescriptorData(desc);
        assert(payload && sensorarrayBleCrc32(payload, desc->length) == desc->crc);
        bleQueued[desc->channel]++;
    }
}

/* Preserve exact bytes handed to the SDK. Complete fragment assemblies must
 * equal the actual encoder output. SDK admission and peer-app receipt are
 * deliberately counted separately in UNREAD. */
typedef struct { uint32_t id, crc; unsigned next, count, length, used; uint8_t bytes[2304]; } assembly_t;
static assembly_t assemblies[3];
static void acceptedPayload(unsigned channel, const uint8_t *bytes, size_t length)
{
    sdkAccepted[channel]++;
    if (channel == SENSORARRAY_BLE_CH_DATA) {
        unsigned seq; assert(sscanf((const char *)bytes, "V,seq=%u,", &seq) == 1 && seq > 0 && seq <= sourceCount);
        assert(!scienceAccepted[seq]); scienceAccepted[seq] = true;
        assert(expectedPackets[seq].length == length);
        assert(memcmp(expectedPackets[seq].data, bytes, length) == 0);
    } else if (channel == SENSORARRAY_BLE_CH_CTRL) {
        unsigned id; assert(sscanf((const char *)bytes, "CTRL,id=%u", &id) == 1 && id < controlGoal);
        uint64_t latency = nowUs - controlAt[id];
        if (latency > controlMaxLatency) { controlMaxLatency = latency; }
    }
    if (scenario != UNREAD) { receiverRecords[channel]++; }
}
static void acceptSdkBytes(unsigned channel, const uint8_t *bytes, unsigned length)
{
    if (length < 2u || bytes[0] != 'G' || bytes[1] != ',') { acceptedPayload(channel, bytes, length); return; }
    char line[128]; const uint8_t *lf = memchr(bytes, '\n', length); assert(lf);
    size_t header = (size_t)(lf - bytes) + 1u; assert(header < sizeof(line));
    memcpy(line, bytes, header); line[header] = 0;
    char ch; unsigned mid,index,count,chunk,total,crc;
    assert(sscanf(line, "G,%c,%u,%u,%u,%u,%u,%x", &ch,&mid,&index,&count,&chunk,&total,&crc) == 7);
    assert(ch == sensorarrayBleEnvelopeChannel((sensorarrayBleChannel_t)channel));
    assert(header + chunk == length && total <= 2304u && index < count);
    assembly_t *a = &assemblies[channel];
    if (index == 0u) { *a = (assembly_t){.id=mid,.crc=crc,.count=count,.length=total}; }
    assert(a->id == mid && a->next == index && a->crc == crc && a->used + chunk <= a->length);
    memcpy(a->bytes + a->used, bytes + header, chunk); a->used += chunk; a->next++;
    if (a->next == a->count) {
        assert(a->used == a->length && sensorarrayBleCrc32(a->bytes,a->length) == a->crc);
        acceptedPayload(channel,a->bytes,a->length);
    }
}
static esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t interface, uint16_t conn,
    uint16_t handle, uint16_t length, uint8_t *value, bool confirm)
{
    assert(current == BLE && !pipelineLockDepth && bleActive && interface == 2 && conn == 3 && !confirm);
    assert(value && length <= 244u && sensorarrayBleDescriptorCurrent(&activeBle));
    assert(s_ble.link.sdkCallActive && s_ble.stats.txSlotUsed > 0u);
    sdkCalls++;
    if (trActive) {
        assert(sensorarrayTransportDescriptorCurrent(&activeTransport));
        sdkWhilePoolHeld++;
    }
    if (scenario == NEVER_RETURNS || scenario == GUARDED_NEVER_RETURNS) {
        sdkNeverOutstanding++;
        blockUntil(UINT64_MAX); /* No artificial SDK return/cancel at model deadline. */
        assert(false);
    }
    if (scenario == BT_PREEMPT || scenario == BT_SATURATED) {
        (void)xTaskNotifyGive(&tasks[BT]);
    } else if (scenario == LONG_WAIT) {
        blockUntil(nowUs + 15000u);
    }
    consume(scenario == SDK_BUSY ? 6000u : 100u);
    if (scenario == REJECT && sdkCalls % 3u == 0u) { sdkErrors++; return ESP_FAIL; }
    unsigned channel = handle == 20u ? SENSORARRAY_BLE_CH_DATA :
                       handle == 30u ? SENSORARRAY_BLE_CH_LOG : SENSORARRAY_BLE_CH_CTRL;
    assert(channel == (unsigned)activeBle.channel);
    acceptSdkBytes(channel, value, length);
    return ESP_OK;
}

static void publishSource(void)
{
    static sensorarrayFrame_t frame;
    sourceContext = true;
    sensorarrayFrameBuilderInitInvalid(&frame);
    frame.sequence = ++sourceCount; frame.timestampUs = nowUs; frame.frameStartUs = nowUs;
    frame.activeRows = 1; frame.freshFrame = true; frame.freshCount = frame.validCount = 8;
    frame.configSnapshot = (sensorarrayFrameConfigSnapshot_t){.rows=1,.cells=8,.rowMask=1,.generation=2,.requestId=1};
    frame.measurement.mode = SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
    frame.measurement.unit = SENSORARRAY_MEASUREMENT_UNIT_VOLT; frame.measurement.decimalScale = -6;
    frame.measurement.modeGeneration=4; frame.measurement.modeRequestId=3;
    frame.expectedMask=frame.acquiredMask=frame.freshMask=UINT64_C(255);
    frame.measurement.validMask=frame.measurement.freshMask=UINT64_C(255);
    frame.measurement.errorMask=frame.errorMask=0;
    frame.rowFreshMask=frame.primaryFreshMask=frame.secondaryFreshMask=1;
    frame.identity=(sensorarrayConfigIdentity_t){.sessionId=UINT64_C(0xAABBCCDDEE112233),.bootId=7,.revision=8,
        .modeGeneration=4,.modeRequestId=3,.rowsGeneration=2,.rowsRequestId=1,.profileGeneration=5,.profileRequestId=3};
    for(unsigned i=0;i<8;i++) { frame.measurement.valuesFixed[i]=50000+i;frame.measurement.pgaGain[i]=1; }
    assert(sensorarrayAsyncLogPublishFrameSnapshot(&frame, 9000u) == ESP_OK);
    unsigned slot = s_pendingSlots[(s_pendingReadIndex+s_pendingCount-1u)%SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
    assert(s_frameSlots[slot].frame.sequence == sourceCount);
    expectedPackets[sourceCount] = s_frameSlots[slot].textPacket;
    sourceContext = false;
}
static void advanceTo(uint64_t until)
{
    assert(!pipelineLockDepth && until >= nowUs);
    inAdvance=true;
    while (sourceCount < sourceGoal && nextSourceUs <= until) {
        nowUs=nextSourceUs; publishSource(); nextSourceUs += scenario == BURST ? 1000u : 10000u;
    }
    nowUs=until; inAdvance=false;
}
static void reschedule(void)
{
    assert(current >= 0 && !sourceContext && !pipelineLockDepth);
    assert(swapcontext(&tasks[current].context,&scheduler)==0);
}
static void blockUntil(uint64_t until) { tasks[current].readyAt=until; reschedule(); }
static bool higherReady(void)
{
    for(unsigned i=0;i<TASKS;i++) if(tasks[i].readyAt<=nowUs && tasks[i].priority>tasks[current].priority) return true;
    return false;
}
static void consume(unsigned us)
{
    assert(current>=0 && !pipelineLockDepth);
    while(us) {
        uint64_t tick=(nowUs/10000u+1u)*10000u;
        uint64_t end=nowUs+us, next=end<tick?end:tick;
        if(sourceCount<sourceGoal && nextSourceUs<next) next=nextSourceUs;
        for(unsigned i=0;i<TASKS;i++) if(tasks[i].readyAt>nowUs && tasks[i].readyAt<next) next=tasks[i].readyAt;
        assert(next>nowUs);
        unsigned used=(unsigned)(next-nowUs);advanceTo(next);us-=used;
        if(nowUs==tick || higherReady()) reschedule();
    }
}
static void outputBridge(void)
{
    /* The full AsyncLog formatter/task is not substituted with fake queue
     * ownership. Real publish/pop helpers and real Transport admission run;
     * the fanout/diagnostic execution cost alone is a 200us synthetic bridge.
     * This preexisting scheduler baseline intentionally keeps generic DATA
     * admission; typed source/transport protection is executed separately in
     * test_protected_frame_pipeline, not claimed by these 22 scenarios. */
    static sensorarrayFrame_t frame;
    static sensorarrayTextPacket_t packet;
    for(;;) {
        (void)ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        unsigned processed=0;
        uint64_t measure;int64_t published;uint32_t depth, protectionFlags;
        while(processed<SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT &&
              sensorarrayAsyncLogPopFrame(&frame,&packet,&measure,&published,&depth,&protectionFlags)) {
            assert(packet.sequence==frame.sequence && frame.identity.revision==8);
            assert(packet.length==expectedPackets[frame.sequence].length &&
                   memcmp(packet.data,expectedPackets[frame.sequence].data,packet.length)==0);
            asyncPopped++; bool queued=false;
            esp_err_t result=sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_DATA,packet.data,packet.length,&queued);
            if(queued) transportQueued[0]++;
            else if(result==ESP_OK) {
                assert(scenario==GUARDED_NEVER_RETURNS && s_ble.guard.quarantined);
                transportNoSink[0]++; /* Existing Queue API: no enabled network sink. */
            } else { transportRejected[0]++; }
            if(frame.sequence%10u==0u) {
                const char log[]="LOG,fixture=synthetic,source=bounded\n";
                queued=false;result=sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LOG,log,sizeof(log)-1,&queued);
                if(queued)transportQueued[1]++;
                else if(result==ESP_OK) {
                    assert(scenario==GUARDED_NEVER_RETURNS && s_ble.guard.quarantined);
                    transportNoSink[1]++;
                } else {transportRejected[1]++;}
            }
            consume(200u);processed++;
            (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_ASYNC_LOG);
        }
        if(processed==SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT)(void)xTaskNotifyGive(s_logTaskHandle);
    }
}
static void transportEntry(void) { sensorarrayTransportTask((void *)(intptr_t)ESP_OK);assert(false); }
static void bleEntry(void) { sensorarrayBleTxTask(NULL);assert(false); }
static void controlEntry(void)
{
    unsigned period=scenario==CONTROL_FLOW?10000u:50000u;
    for(unsigned i=0;i<controlGoal;i++) {
        uint64_t due=sourceStart+(uint64_t)i*period;
        if(nowUs<due)blockUntil(due);
        controlAt[i]=due;controlCalls++;
        if(nowUs-due>controlMaxDispatch)controlMaxDispatch=nowUs-due;
        char reply[48];int size=snprintf(reply,sizeof(reply),"CTRL,id=%u\n",i);assert(size>0);
        esp_err_t err=sensorarrayBleNotify(SENSORARRAY_BLE_CH_CTRL,(const uint8_t *)reply,(size_t)size);
        if(err==ESP_OK)controlAccepted++;else controlRejected++;
        consume(100u);
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_CTRL);
    }
    blockUntil(UINT64_MAX);assert(false);
}
static void btEntry(void)
{
    for(;;) {
        unsigned jobs=ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
        for(unsigned i=0;i<jobs;i++){btJobs++;consume(scenario==BT_SATURATED?4800u:1500u);}
    }
}
static esp_err_t initializeBleQueues(void)
{
#include "ble_init.h"
    return ESP_OK;
}
static void initialize(unsigned txPriority)
{
    assert(CONFIG_SENSORARRAY_SCAN_TASK_CORE==1 && CONFIG_SENSORARRAY_ASYNC_LOG_TASK_CORE==0);
    assert(CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN==4 && SENSORARRAY_ASYNC_LOG_NORMAL_FRAME_SLOT_COUNT==4 && SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT==6);
    sourceGoal=scenario==BURST?8:200;controlGoal=scenario==BURST?2:scenario==CONTROL_FLOW?200:40;
    nextSourceUs=sourceStart;
    assert(sensorarrayWorkBudgetInit()==ESP_OK && idleHook);
    void(*entries[])(void)={outputBridge,transportEntry,bleEntry,controlEntry,btEntry};
    unsigned priorities[]={CONFIG_SENSORARRAY_ASYNC_LOG_TASK_PRIORITY,PIPELINE_TRANSPORT_PRIORITY,txPriority,CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY,19};
    for(unsigned i=0;i<TASKS;i++) {
        tasks[i].priority=priorities[i];tasks[i].order=++readyOrder;
        assert(getcontext(&tasks[i].context)==0);
        tasks[i].context.uc_stack.ss_sp=tasks[i].stack;tasks[i].context.uc_stack.ss_size=sizeof(tasks[i].stack);
        tasks[i].context.uc_link=&scheduler;makecontext(&tasks[i].context,entries[i],0);
    }
    s_task=&tasks[TRANSPORT];s_logTaskHandle=&tasks[ASYNC];
    /* Fixture reset models a new boot; production never rebinds a live pool. */
    static char transportStorage[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    memset(&s_pool,0,sizeof(s_pool));
    assert(sensorarrayTransportPoolInit(&s_pool,transportStorage,sizeof(transportStorage)));
    s_dataQueue=xQueueCreateStatic(SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT,sizeof(sensorarrayTransportDescriptor_t),trQueueBytes[0],&trQueues[0]);
    s_logQueue=xQueueCreateStatic(SENSORARRAY_TRANSPORT_LOG_QUEUE_COUNT,sizeof(sensorarrayTransportDescriptor_t),trQueueBytes[1],&trQueues[1]);
    s_lifecycleQueue=xQueueCreateStatic(SENSORARRAY_TRANSPORT_LIFECYCLE_QUEUE_COUNT,sizeof(sensorarrayTransportDescriptor_t),trQueueBytes[2],&trQueues[2]);
    s_dataQueue->tag=Q_TR_DATA;s_logQueue->tag=Q_TR_LOG;s_lifecycleQueue->tag=Q_TR_LIFE;
    assert(sensorarrayBleInitPayloadStorage()==ESP_OK);
    assert(initializeBleQueues()==ESP_OK);
    s_ble.txQueue->tag=Q_BLE_DATA;s_ble.logTxQueue->tag=Q_BLE_LOG;s_ble.ctrlTxQueue->tag=Q_BLE_CTRL;
    s_ble.guard.ready=true; s_ble.guard.initError=ESP_OK;
    s_ble.stats.connected=s_ble.stats.gattReady=true;s_ble.stats.mtu=247;
    s_ble.connectionGeneration=7;s_ble.gattsIf=2;s_ble.connId=3;
    s_ble.cccd[0]=s_ble.cccd[1]=s_ble.cccd[2]=1;
    s_ble.handles[BLE_IDX_DATA_TX_VALUE]=20;s_ble.handles[BLE_IDX_LOG_TX_VALUE]=30;s_ble.handles[BLE_IDX_CTRL_TX_VALUE]=10;
    s_ble.txTask=&tasks[BLE];s_ble.controlTask=&tasks[CONTROL];
}
static unsigned outstandingBle(unsigned channel)
{
    unsigned count=0;
    for(unsigned i=0;i<CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN;i++)if(s_ble.dataSlots[i].inUse && s_ble.dataSlots[i].channel==channel)count++;
    for(unsigned i=0;i<SENSORARRAY_BLE_LOG_TX_SLOT_COUNT;i++)if(s_ble.logSlots[i].inUse && s_ble.logSlots[i].channel==channel)count++;
    for(unsigned i=0;i<SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT;i++)if(s_ble.ctrlSlots[i].inUse && s_ble.ctrlSlots[i].channel==channel)count++;
    return count;
}
int main(int argc,char **argv)
{
    setbuf(stdout,NULL);assert(argc==3);
    const char *names[]={"burst","short","bt_preempt","long_wait","reject","unread","control_flow",
                         "sdk_busy","bt_saturated","never_returns","guarded_never_returns"};
    for(scenario=0;scenario<SCENARIOS;scenario++)if(strcmp(names[scenario],argv[1])==0)break;
    assert(scenario<SCENARIOS);unsigned txPriority=(unsigned)strtoul(argv[2],NULL,10);assert(txPriority==7||txPriority==8);
    initialize(txPriority);
    nextGuardUs = SENSORARRAY_BLE_SEND_POLL_US;
    uint64_t stop=sourceStart+(uint64_t)sourceGoal*10000u+2000000u;
    while(nowUs<stop) {
        if (scenario == GUARDED_NEVER_RETURNS && nowUs >= nextGuardUs) {
            /* An independent simulated ESP timer dispatcher runs while the
             * BLE ucontext remains suspended inside the non-returning SDK.
             * Skip missed periods, matching the real timer registration. */
            assert(current == -1 && !pipelineLockDepth);
            sensorarrayBleObserveSendDeadline(); guardPolls++;
            nextGuardUs = nowUs + SENSORARRAY_BLE_SEND_POLL_US;
        }
        if(sourceCount<sourceGoal && nextSourceUs<=nowUs)advanceTo(nowUs);
        int selected=-1;
        for(unsigned i=0;i<TASKS;i++)if(tasks[i].readyAt<=nowUs &&
            (selected<0 || tasks[i].priority>tasks[selected].priority ||
             (tasks[i].priority==tasks[selected].priority && tasks[i].order<tasks[selected].order)))selected=(int)i;
        if(selected<0) {
            assert(idleHook());idleVisits++;
            /* IDLE keeps running between external events. Do not jump a
             *750ms startup sleep in one hook invocation and mislabel that
             *actually-idle interval as starvation. Tick events bound the
             *synthetic idle-hook cadence; no watchdog is fed by this model. */
            uint64_t next=(nowUs/10000u+1u)*10000u;
            if(next>stop)next=stop;
            if(sourceCount<sourceGoal && nextSourceUs<next)next=nextSourceUs;
            for(unsigned i=0;i<TASKS;i++)if(tasks[i].readyAt<next)next=tasks[i].readyAt;
            assert(next>nowUs);advanceTo(next);continue;
        }
        current=selected;tasks[selected].order=++readyOrder;modelSwitches++;
        assert(modelSwitches<100000u);
        assert(swapcontext(&scheduler,&tasks[selected].context)==0);current=-1;
    }
    bool bounded=scenario!=NEVER_RETURNS && scenario!=GUARDED_NEVER_RETURNS;
    sensorarrayWorkBudgetSnapshot_t budget;sensorarrayWorkBudgetCopySnapshot(&budget);
    /* All source arrivals occur independently of Core0 scheduling. Output
     * loss must be explained by the actual bounded slot/drop decisions. */
    assert(sourceCount==sourceGoal && s_sharedStats.publishedFrames==sourceGoal);
    assert(sourceGoal==asyncPopped+s_sharedStats.droppedOutputFrames+s_pendingCount);
    assert(asyncPopped==transportQueued[0]+transportRejected[0]+transportNoSink[0]);
    assert(transportQueued[0]==transportProcessed[0]+s_dataQueue->count+(trActive?1u:0u));
    assert(transportProcessed[0]==g_sensorarrayTransportStats.bleDataSent+g_sensorarrayTransportStats.bleDataDrop);
    /* DATA is effectively subscribed throughout this model. Actual ordinary
     * LOG admission rejection must reach transport as a counted drop, not a
     * fictional successful submit, and cannot consume SDK/owner capacity. */
    assert(g_sensorarrayTransportStats.bleLogSent==0u && sdkAccepted[1]==0u && bleQueued[1]==0u);
    assert(transportProcessed[1]==g_sensorarrayTransportStats.bleLogDrop);
    assert(s_ble.stats.logPolicyDrop<=s_ble.stats.dropped[SENSORARRAY_BLE_CH_LOG]);
    if(scenario!=GUARDED_NEVER_RETURNS) {
        assert(transportProcessed[1]==s_ble.stats.logPolicyDrop);
        assert(s_ble.stats.logPolicyDrop==s_ble.stats.dropped[SENSORARRAY_BLE_CH_LOG]);
    }
    assert(controlCalls==controlGoal && controlAccepted+controlRejected==controlCalls);
    assert(g_sensorarrayTransportStats.queueDropData==transportRejected[0]);
    assert(s_pool.stats.releaseMismatch==0 && s_ble.stats.txSlotReleaseMismatch==0 && s_ble.stats.txCrcMismatch==0);
    assert(idleVisits>0 && budget.maxIdleGapTicks<=6u);
    assert(controlMaxDispatch<500000u);
    for(unsigned ch=0;ch<3;ch++) {
        assert(bleStarted[ch]<=bleQueued[ch] && bleFinished[ch]<=bleStarted[ch]);
        assert(bleQueued[ch]==bleFinished[ch]+outstandingBle(ch));
        assert(s_ble.stats.sent[ch]==sdkAccepted[ch]);
    }
    if(bounded) {
        assert(asyncPopped>0 && s_pendingCount==0 && s_pool.stats.used==0 && !bleActive && !trActive);
        assert(s_ble.stats.txSlotUsed==0 && !s_ble.link.sdkCallActive);
        assert(controlMaxLatency<500000u); /* Never relaxed for candidate8. */
        if(scenario!=LONG_WAIT && scenario!=CONTROL_FLOW)assert(controlAccepted==controlGoal);
        if(scenario==UNREAD)assert(receiverRecords[0]==0 && sdkAccepted[0]>0);
        else assert(receiverRecords[0]==sdkAccepted[0]);
        if(scenario==SDK_BUSY || scenario==BT_SATURATED) {
            assert(budget.workers[SENSORARRAY_WORK_BUDGET_BLE_TX].yields>0);
            assert(s_ble.stats.dropped[0]+g_sensorarrayTransportStats.queueDropData+s_sharedStats.droppedOutputFrames>0);
        }
    } else {
        assert(sdkNeverOutstanding==1 && s_ble.link.sdkCallActive && bleActive);
        assert(bleFinished[activeBle.channel]<bleStarted[activeBle.channel]);
        assert(s_ble.stats.txSlotUsed>0 && sdkAccepted[0]==0 && sdkAccepted[2]==0);
        /* World inspection ends; blocked owner never returns, cancels or
         * releases its payload. Process exit is not firmware recovery. */
        if (scenario == GUARDED_NEVER_RETURNS) {
            assert(guardPolls > 1u && s_ble.guard.quarantined && s_ble.guard.callKnown);
            assert(s_ble.guard.callId == 1u && !s_ble.guard.completionKnown && sdkCalls == 1u);
            uint64_t detectedAge = s_ble.guard.detectedUs - s_ble.guard.startUs;
            assert(detectedAge >= SENSORARRAY_BLE_SEND_DEADLINE_US &&
                   detectedAge <= SENSORARRAY_BLE_SEND_DEADLINE_US + SENSORARRAY_BLE_SEND_POLL_US);
            assert(s_ble.guard.rejected > 0u && s_ble.stats.txConnectionStaleDrop == 0u);
            assert(sensorarrayBleDescriptorCurrent(&activeBle));
        }
    }
    printf("PIPELINE_RESULT,{\"scenario\":\"%s\",\"txPriority\":%u,\"status\":\"%s\",\"bounded\":%s,"
           "\"source\":%u,\"outputPopped\":%u,\"outputDrop\":%llu,\"transportQueued\":%u,\"transportDrop\":%u,"
           "\"bleAdmission\":%u,\"bleAdmissionReject\":%u,\"bleSdkComplete\":%u,\"bleDropTotal\":%u,"
           "\"ordinaryLogPolicyDrop\":%u,\"ordinaryLogDropTotal\":%u,\"transportLogRejected\":%u,"
           "\"peerReceived\":%u,\"sdkCalls\":%u,\"sdkErrors\":%u,\"btJobs\":%u,\"ctrlCalls\":%u,"
           "\"ctrlAccepted\":%u,\"ctrlRejected\":%u,\"ctrlSdkComplete\":%u,\"ctrlDispatchMaxUs\":%llu,\"ctrlMaxUs\":%llu,"
           "\"poolHeldMaxUs\":%llu,\"sdkWhilePoolHeld\":%u,\"poolHighWater\":%u,\"bleHighWater\":%u,"
           "\"idleVisits\":%u,\"maxIdleGapTicks\":%u,\"bleYields\":%u,\"bleOutstanding\":%u,\"sdkNeverOutstanding\":%u,"
           "\"sourcePeriodUs\":%u,\"syntheticSdkCpuUs\":%u,\"syntheticBtCpuUs\":%u,\"syntheticSdkWaitUs\":%u,"
           "\"syntheticOutputCpuUs\":200,\"controlDeadlineUs\":500000,\"controlDeadlineBasis\":\"local_SDK_admission_not_PC_receipt\","
           "\"guardTimerModeled\":%u,\"guardQuarantined\":%u,\"guardPolls\":%u,\"guardDetectedUs\":%llu,\"guardCallStartUs\":%llu,\"transportNoNetworkSink\":%u,"
           "\"performanceAcceptance\":\"NOT_ESTABLISHED\",\"hardware\":\"NOT_RUN\"}\n",
           names[scenario],txPriority,bounded?"MODEL_INVARIANTS_PASS":
               scenario==GUARDED_NEVER_RETURNS?"QUARANTINED_INFLIGHT_NOT_CANCELLED":"NOT_PROVABLY_BOUNDED",bounded?"true":"false",
           sourceCount,asyncPopped,(unsigned long long)s_sharedStats.droppedOutputFrames,transportQueued[0],transportRejected[0],
           g_sensorarrayTransportStats.bleDataSent,g_sensorarrayTransportStats.bleDataDrop,sdkAccepted[0],s_ble.stats.dropped[0],
           s_ble.stats.logPolicyDrop,s_ble.stats.dropped[SENSORARRAY_BLE_CH_LOG],g_sensorarrayTransportStats.bleLogDrop,
           receiverRecords[0],sdkCalls,sdkErrors,btJobs,controlCalls,controlAccepted,controlRejected,sdkAccepted[2],
           (unsigned long long)controlMaxDispatch,(unsigned long long)controlMaxLatency,(unsigned long long)poolHeldMaxUs,
           sdkWhilePoolHeld,s_pool.stats.highWater,s_ble.stats.txSlotHighWater,idleVisits,budget.maxIdleGapTicks,
           budget.workers[SENSORARRAY_WORK_BUDGET_BLE_TX].yields,s_ble.stats.txSlotUsed,sdkNeverOutstanding,
           scenario==BURST?1000u:10000u,scenario==SDK_BUSY?6000u:100u,
           scenario==BT_PREEMPT?1500u:scenario==BT_SATURATED?4800u:0u,scenario==LONG_WAIT?15000u:0u,
           scenario==GUARDED_NEVER_RETURNS,s_ble.guard.quarantined,guardPolls,
           (unsigned long long)s_ble.guard.detectedUs,(unsigned long long)s_ble.guard.startUs,transportNoSink[0]);
    return 0;
}
