/* Actual firmware bodies are extracted by test_protected_frame_pipeline.py.
 * Queue scheduling, clock and BLE admission are synthetic. No peer receipt. */
#include "common.h"
#include <stdio.h>
#include <string.h>
#include "sensorarrayBle.h"
#include "sensorarrayTransport.h"
#include "sensorarrayTransportPool.h"
#include "sensorarrayTransportFrameMeta.h"
#include "sensorarrayTextProtocol.h"
#include "sensorarrayFrameBuilder.h"
#include "sensorarrayFramePublication.h"
#include "sensorarrayOutputPolicy.h"
#include "protected_config.h"
#include "protected_types.h"
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT 1

unsigned pipelineLockDepth;
static int64_t nowUs;
static TaskHandle_t s_task, s_logTaskHandle;
static portMUX_TYPE s_asyncLogMux, g_sensorarrayTransportStatsMux;
static sensorarrayAsyncLogFrameSlot_t slots[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static sensorarrayAsyncLogFrameSlot_t *s_frameSlots = slots;
static uint8_t s_frameSlotState[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static uint32_t s_frameSlotProtection[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static sensorarrayTransportFrameIdentity_t s_frameSlotIdentity[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT];
static uint8_t s_pendingSlots[SENSORARRAY_ASYNC_LOG_FRAME_SLOT_COUNT], s_pendingReadIndex, s_pendingCount;
static sensorarrayAsyncLogSharedStats_t s_sharedStats;
static sensorarrayAsyncLogProtectionStats_t s_asyncProtectionStats;
static bool s_protectedIdentityKnown;
static sensorarrayTransportFrameIdentity_t s_protectedIdentity;
static int64_t s_lastProtectedPublishedUs;
static sensorarrayTransportPool_t s_pool;
static sensorarrayTransportStats_t g_sensorarrayTransportStats;
static sensorarrayTransportProtectionStats_t s_transportProtectionStats;
static bool s_protectedRetryActive, s_lastBleFrameKnown, s_protectedAdmissionKnown;
static sensorarrayTransportDescriptor_t s_protectedRetry;
static sensorarrayTransportFrameIdentity_t s_lastBleFrame, s_protectedAdmissionIdentity;
static uint64_t s_lastProtectedAdmissionUs;
static uint32_t s_lastBleConnectionGeneration;
static StaticQueue_t queues[4];
static uint8_t queueStorage[4][2u*sizeof(sensorarrayTransportDescriptor_t)];
static QueueHandle_t s_dataQueue, s_logQueue, s_lifecycleQueue, s_protectedQueue;
static bool shortMode, bleEnabled = true, wifiEnabled;
static esp_err_t admissionResult;
static unsigned bleCalls, normalCalls, lifeCalls, wifiCalls;
static sensorarrayBleFramePublication_t lastPublication;
static uint8_t lastFull[2305], lastNormal[2305], lastChosen[2305];
static size_t lastChosenLength;

int64_t esp_timer_get_time(void) { return nowUs; }
int xTaskNotifyGive(TaskHandle_t task) { (void)task; assert(!pipelineLockDepth); return pdTRUE; }
QueueHandle_t xQueueCreateStatic(unsigned n,unsigned size,uint8_t *storage,StaticQueue_t *q)
{ *q=(StaticQueue_t){.storage=storage,.capacity=n,.itemSize=size};return q; }
int xQueueSend(QueueHandle_t q,const void *item,TickType_t timeout)
{
    assert(q && !pipelineLockDepth && !timeout);
    if(q->count==q->capacity)return pdFALSE;
    memcpy(q->storage+((q->head+q->count)%q->capacity)*q->itemSize,item,q->itemSize);q->count++;return pdTRUE;
}
int xQueueReceive(QueueHandle_t q,void *out,TickType_t timeout)
{
    assert(q && !pipelineLockDepth && !timeout);
    if(!q->count)return pdFALSE;
    memcpy(out,q->storage+q->head*q->itemSize,q->itemSize);q->head=(q->head+1u)%q->capacity;q->count--;return pdTRUE;
}
static bool sensorarrayAsyncLogIsRunning(void) { return true; }
bool sensorarrayTransportBleChannelEnabled(sensorarrayTransportChannel_t channel) { (void)channel;return bleEnabled; }
bool sensorarrayTransportWifiSinkEnabled(void) { return wifiEnabled; }
bool sensorarrayWifiIsReady(void) { return wifiEnabled; }
esp_err_t sensorarrayWifiSend(sensorarrayWifiChannel_t channel,const uint8_t *data,size_t length)
{ (void)channel;assert(!pipelineLockDepth && data && length);wifiCalls++;return ESP_OK; }
sensorarrayTransportTxMode_t sensorarrayTransportGetTxMode(void)
{ return shortMode?SENSORARRAY_TRANSPORT_TX_SHORT:SENSORARRAY_TRANSPORT_TX_REL; }
static bool sensorarrayTransportFindFieldU32(const char *data,const char *field,uint32_t *out)
{ (void)data;(void)field;*out=0;return false; }
bool sensorarrayBleGetSendGeneration(sensorarrayBleChannel_t channel,uint32_t *generation)
{ (void)channel;*generation=7;return bleEnabled; }
bool sensorarrayBleIsCongested(void) { return false; }
esp_err_t sensorarrayBleNotifyFrameForGeneration(const sensorarrayBleFramePublication_t *p,uint32_t generation)
{
    assert(!pipelineLockDepth && generation==7 && p && p->fullData && p->normalData);
    assert(p->fullLength<=2304 && p->normalLength<=2304);
    bleCalls++;lastPublication=*p;
    memcpy(lastFull,p->fullData,p->fullLength);lastFull[p->fullLength]=0;
    memcpy(lastNormal,p->normalData,p->normalLength);lastNormal[p->normalLength]=0;
    lastChosenLength=p->protectionFlags?p->fullLength:p->normalLength;
    memcpy(lastChosen,p->protectionFlags?p->fullData:p->normalData,lastChosenLength);
    lastChosen[lastChosenLength]=0;
    return admissionResult;
}
esp_err_t sensorarrayBleNotifyForGeneration(sensorarrayBleChannel_t channel,const uint8_t *data,size_t n,uint32_t gen)
{ (void)channel;assert(data && n && gen==7 && !pipelineLockDepth);normalCalls++;return ESP_OK; }
esp_err_t sensorarrayBleNotifyLifecycleForGeneration(const uint8_t *data,size_t n,uint32_t gen)
{ assert(data && n && gen==7 && !pipelineLockDepth);lifeCalls++;return ESP_OK; }
#include "protected_async.h"
#include "protected_baseline.h"
#include "protected_transport.h"
#include "protected_net.h"

static void reset(void)
{
    assert(!pipelineLockDepth);
    memset(slots,0,sizeof slots);memset(s_frameSlotState,0,sizeof s_frameSlotState);
    memset(s_frameSlotProtection,0,sizeof s_frameSlotProtection);memset(s_frameSlotIdentity,0,sizeof s_frameSlotIdentity);
    memset(&s_sharedStats,0,sizeof s_sharedStats);memset(&s_asyncProtectionStats,0,sizeof s_asyncProtectionStats);
    s_pendingReadIndex=s_pendingCount=0;s_protectedIdentityKnown=false;s_lastProtectedPublishedUs=0;
    sensorarrayTransportPoolInit(&s_pool);memset(&g_sensorarrayTransportStats,0,sizeof g_sensorarrayTransportStats);
    memset(&s_transportProtectionStats,0,sizeof s_transportProtectionStats);
    s_protectedRetryActive=s_lastBleFrameKnown=s_protectedAdmissionKnown=false;s_lastProtectedAdmissionUs=0;
    s_dataQueue=xQueueCreateStatic(SENSORARRAY_TRANSPORT_DATA_QUEUE_COUNT,sizeof(sensorarrayTransportDescriptor_t),queueStorage[0],&queues[0]);
    s_logQueue=xQueueCreateStatic(SENSORARRAY_TRANSPORT_LOG_QUEUE_COUNT,sizeof(sensorarrayTransportDescriptor_t),queueStorage[1],&queues[1]);
    s_lifecycleQueue=xQueueCreateStatic(SENSORARRAY_TRANSPORT_LIFECYCLE_QUEUE_COUNT,sizeof(sensorarrayTransportDescriptor_t),queueStorage[2],&queues[2]);
    s_protectedQueue=xQueueCreateStatic(SENSORARRAY_TRANSPORT_PROTECTED_QUEUE_COUNT,sizeof(sensorarrayTransportDescriptor_t),queueStorage[3],&queues[3]);
    nowUs=1000000;bleEnabled=true;wifiEnabled=shortMode=false;admissionResult=ESP_OK;
    bleCalls=normalCalls=lifeCalls=wifiCalls=0;
}
static sensorarrayFrame_t makeFrame(uint32_t sequence,uint32_t revision)
{
    sensorarrayFrame_t f;sensorarrayFrameBuilderInitInvalid(&f);
    f.sequence=sequence;f.timestampUs=nowUs;f.frameStartUs=nowUs;f.activeRows=1;
    f.freshFrame=true;f.freshCount=f.validCount=7;
    f.configSnapshot=(sensorarrayFrameConfigSnapshot_t){.rows=1,.cells=8,.rowMask=1,.generation=2,.requestId=1};
    f.measurement.mode=SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
    f.measurement.unit=SENSORARRAY_MEASUREMENT_UNIT_VOLT;f.measurement.decimalScale=-6;
    f.measurement.modeGeneration=4;f.measurement.modeRequestId=3;
    f.expectedMask=UINT64_C(255);f.acquiredMask=f.freshMask=UINT64_C(254);
    f.measurement.validMask=f.measurement.freshMask=UINT64_C(254);
    f.measurement.errorMask=f.errorMask=1;f.measurement.errorReason[0]=SENSORARRAY_CELL_ERROR_SPI;
    f.rowFreshMask=f.primaryFreshMask=f.secondaryFreshMask=1;
    f.identity=(sensorarrayConfigIdentity_t){.sessionId=UINT64_C(0xAABBCCDDEE112233),.bootId=7,.revision=revision,
        .modeGeneration=4,.modeRequestId=3,.rowsGeneration=2,.rowsRequestId=1,.profileGeneration=5,.profileRequestId=3};
    for(unsigned i=0;i<8;i++){f.measurement.valuesFixed[i]=50000+i;f.measurement.pgaGain[i]=1;}
    return f;
}
static uint32_t pop(sensorarrayFrame_t *f,sensorarrayTextPacket_t *p)
{
    uint64_t measure;int64_t published;uint32_t depth,flags;
    assert(sensorarrayAsyncLogPopFrame(f,p,&measure,&published,&depth,&flags));assert(measure==9000);return flags;
}
static bool processNext(void)
{
    sensorarrayTransportDescriptor_t d;uint32_t budget=2;
    if(!sensorarrayTransportReceiveNext(&d,&budget))return false;
    sensorarrayTransportProcessDescriptor(&d);return true;
}
static void publish(sensorarrayFrame_t *f,uint32_t flags)
{
    sensorarrayTextPacket_t p;assert(sensorarrayTextProtocolBuildFrame(f,&p)==ESP_OK);
    assert(sensorarrayNetFramePublish(&p,f,flags)==ESP_OK);
}
static void baselineAndSourceTest(void)
{
    reset();sensorarrayFrame_t f;
    for(unsigned i=1;i<=12;i++){f=makeFrame(i,8);assert(baselinePublishFrameSnapshot(&f,9000)==ESP_OK);}
    assert(s_pendingCount==4 && slots[s_pendingSlots[s_pendingReadIndex]].frame.sequence==9);
    printf("PROTECTED_BASELINE,actual_old_source=first_seq1_evicted,ordinary_slots=4\n");
    reset();
    for(unsigned i=1;i<=12;i++){f=makeFrame(i,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);}
    sensorarrayTextPacket_t p;uint32_t flags=pop(&f,&p);assert(f.sequence==1 && flags==3);
    for(unsigned i=9;i<=12;i++){assert(pop(&f,&p)==0 && f.sequence==i);}
    assert(!s_pendingCount && s_sharedStats.droppedOutputFrames==7);
    assert(s_asyncProtectionStats.firstConfigQueued==1);
    /* FIFO is source order even with reservations; USB consumes this order. */
    nowUs+=1000000;f=makeFrame(13,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    assert(pop(&f,&p)==SENSORARRAY_FRAME_PROTECT_PERIODIC && f.sequence==13);
    /* Same-config periodic may use a second reserved slot, never replace the first. */
    reset();f=makeFrame(1,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    nowUs+=1000000;f=makeFrame(2,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    nowUs+=1000000;f=makeFrame(3,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    assert(pop(&f,&p)==3 && f.sequence==1);assert(pop(&f,&p)==2 && f.sequence==3);
    assert(s_asyncProtectionStats.superseded==1 && s_asyncProtectionStats.firstSuperseded==0);
    /* Real encoder rejection does not falsely retire the source obligation. */
    reset();f=makeFrame(1,8);f.measurement.valuesFixed[1]=INT64_MAX;
    assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_ERR_INVALID_SIZE);
    assert(!s_pendingCount && !s_protectedIdentityKnown && s_asyncProtectionStats.firstCandidateMiss==1);
    f=makeFrame(2,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    assert(pop(&f,&p)==3 && f.sequence==2);
    /* Both reserved slots have genuine different owners, never overwrite. */
    reset();s_frameSlotState[4]=3;s_frameSlotState[5]=2;
    memset(&slots[4],0x4a,sizeof slots[4]);memset(&slots[5],0x5b,sizeof slots[5]);
    sensorarrayAsyncLogFrameSlot_t held=slots[4],writing=slots[5];
    f=makeFrame(1,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    assert(s_asyncProtectionStats.firstCandidateMiss==1 && !s_protectedIdentityKnown);
    assert(!memcmp(&held,&slots[4],sizeof held) && !memcmp(&writing,&slots[5],sizeof writing));
    s_frameSlotState[5]=0;f=makeFrame(2,8);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    assert(pop(&f,&p)==0 && f.sequence==1);assert(pop(&f,&p)==3 && f.sequence==2);
    assert(!memcmp(&held,&slots[4],sizeof held));
    /* New revisions supersede queued ownership only; held old remains intact. */
    reset();s_frameSlotState[4]=3;memset(&slots[4],0x4a,sizeof slots[4]);held=slots[4];
    for(unsigned i=1;i<=50;i++){f=makeFrame(i,i);assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);}
    assert(s_asyncProtectionStats.superseded==49 && s_asyncProtectionStats.firstSuperseded==49);
    assert(pop(&f,&p)&SENSORARRAY_FRAME_PROTECT_FIRST_CONFIG);assert(f.sequence==50);
    assert(!memcmp(&held,&slots[4],sizeof held));
    printf("PROTECTED_SOURCE,reserved_first=PASS,FIFO=PASS,owned_and_writing=PASS,rapid_configs=50,miss_obligation=PASS\n");
}
static void byteIdentityTest(void)
{
    reset();shortMode=true;sensorarrayFrame_t f=makeFrame(1,8);sensorarrayTextPacket_t packet;
    assert(sensorarrayAsyncLogPublishFrameSnapshot(&f,9000)==ESP_OK);
    uint32_t flags=pop(&f,&packet);assert(sensorarrayNetFramePublish(&packet,&f,flags)==ESP_OK);
    /* Mutating caller after synchronous queue handoff cannot relabel bytes. */
    uint32_t savedSequence=f.sequence;f.sequence=99;f.identity.revision=99;
    assert(processNext());assert(bleCalls==1 && lastPublication.identity.sequence==savedSequence);
    assert(lastPublication.identity.revision==8 && lastPublication.identity.bootId==7);
    assert(lastPublication.identity.sessionId==UINT64_C(0xAABBCCDDEE112233));
    assert(lastChosenLength==packet.length && !memcmp(lastChosen,packet.data,packet.length));
    assert(lastPublication.normalLength==packet.length); /* Legacy SHORT applies to CAP only. */
    assert(strstr(packet.data,"X02"));
    printf("PROTECTED_FRAME_HEX,volt1_invalid,");for(size_t i=0;i<packet.length;i++){printf("%02x",(unsigned char)packet.data[i]);}puts("");
    f=makeFrame(2,9);f.measurement.mode=SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
    assert(sensorarrayTextProtocolBuildFrame(&f,&packet)==ESP_OK);publish(&f,1);assert(processNext());
    assert(lastChosenLength==packet.length && !memcmp(lastChosen,packet.data,packet.length));
    assert(lastPublication.normalLength<packet.length && !memcmp(lastNormal,"B20,",4));
    printf("PROTECTED_FRAME_HEX,cap1_full,");for(size_t i=0;i<packet.length;i++){printf("%02x",(unsigned char)packet.data[i]);}puts("");
    f.sequence=3;publish(&f,0);assert(processNext());
    assert(lastPublication.protectionFlags==0 && lastChosenLength==lastPublication.normalLength);
    assert(lastChosenLength<lastPublication.fullLength && !memcmp(lastChosen,"B20,",4));
    for(unsigned mixed=0;mixed<2;mixed++) {
        f=makeFrame(4+mixed,10+mixed);f.activeRows=8;
        f.configSnapshot.rows=8;f.configSnapshot.cells=64;f.configSnapshot.rowMask=255;
        f.expectedMask=UINT64_MAX;f.acquiredMask=f.freshMask=UINT64_MAX-1;
        f.measurement.validMask=f.measurement.freshMask=UINT64_MAX-1;
        f.measurement.mode=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
        f.measurement.unit=SENSORARRAY_MEASUREMENT_UNIT_OHM;f.measurement.decimalScale=-3;
        f.mixedProfile=mixed;f.rowProfileGeneration=5;f.rowProfileRequestId=3;
        for(unsigned i=0;i<64;i++)f.measurement.valuesFixed[i]=10000000+i;
        for(unsigned row=0;row<8;row++) {
            f.rowMode[row]=(sensorarrayMeasurementMode_t)(SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE+row%3);
            f.rowUnit[row]=row%3==0?SENSORARRAY_MEASUREMENT_UNIT_PF:row%3==1?SENSORARRAY_MEASUREMENT_UNIT_VOLT:SENSORARRAY_MEASUREMENT_UNIT_OHM;
            f.rowScale[row]=row%3==2?-3:-6;
        }
        assert(sensorarrayTextProtocolBuildFrame(&f,&packet)==ESP_OK);publish(&f,1);assert(processNext());
        assert(lastChosenLength==packet.length && !memcmp(lastChosen,packet.data,packet.length));
        printf("PROTECTED_FRAME_HEX,%s,",mixed?"mixed8_full":"res8_full");
        for(size_t i=0;i<packet.length;i++){printf("%02x",(unsigned char)packet.data[i]);}puts("");
    }
    /* Independent full 2304 capacity; synthetic maximum-size bytes, not a fake encoder fixture. */
    reset();char bytes[2304];for(unsigned i=0;i<sizeof bytes;i++)bytes[i]=(char)('A'+i%26);
    f=makeFrame(3,8);sensorarrayTransportFrameIdentity_t id=sensorarrayFramePublicationIdentity(&f);
    assert(sensorarrayTransportPublishFrame(bytes,sizeof bytes,&id,1)==ESP_OK);assert(processNext());
    assert(lastChosenLength==sizeof bytes && !memcmp(bytes,lastChosen,sizeof bytes));
    printf("PROTECTED_BYTES,real_encoder_crc_original=preserved,SHORT_ordinary=preview,SHORT_protected=full,max2304=PASS,identity=immutable\n");
}
static void transportTest(void)
{
    reset();sensorarrayFrame_t f=makeFrame(1,8);publish(&f,0);
    assert(s_transportProtectionStats.derivedProtection==1);assert(processNext());
    /* Legacy ordinary pool traffic cannot consume the two reserved capacities. */
    bool queued;for(unsigned i=0;i<2;i++)assert(sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_DATA,"D",1,&queued)==ESP_OK && queued);
    for(unsigned i=0;i<2;i++)assert(sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LOG,"L",1,&queued)==ESP_OK && queued);
    assert(sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE,"MAPP",4,&queued)==ESP_OK && queued);
    f=makeFrame(2,9);publish(&f,1);f=makeFrame(3,10);publish(&f,1);
    assert(s_pool.stats.used==7);assert(processNext() && lifeCalls==1 && bleCalls==1);
    assert(processNext() && lastPublication.identity.sequence==2);assert(processNext() && lastPublication.identity.sequence==3);
    while(processNext()){} assert(s_pool.stats.used==0);
    /* Protected overtakes normal; explicitly discard same-seq and older identity. */
    reset();f=makeFrame(1,8);publish(&f,1);assert(processNext());
    f=makeFrame(2,8);publish(&f,0);f=makeFrame(3,8);publish(&f,0);publish(&f,2);
    assert(processNext() && lastPublication.identity.sequence==3);unsigned calls=bleCalls;
    assert(processNext() && processNext());assert(bleCalls==calls && g_sensorarrayTransportStats.bleDataObsolete==2);
    /* Half-range ordering has an explicit wrap definition. */
    sensorarrayTransportFrameIdentity_t before=s_lastBleFrame,after=s_lastBleFrame;
    before.sequence=UINT32_MAX;after.sequence=0;
    assert(!sensorarrayTransportFrameOlderOrSame(&after,&before));
    assert(sensorarrayTransportFrameOlderOrSame(&before,&after));
    /* Cross-boot equal sequence is a new identity, never numerically ordered. */
    f=makeFrame(1,8);f.identity.bootId=8;f.identity.sessionId++;publish(&f,0);assert(processNext());
    assert(bleCalls==calls+1 && lastPublication.identity.bootId==8);
    /* Repeated NO_MEM retains exact pool owner with total deadline and Wi-Fi once. */
    reset();wifiEnabled=true;admissionResult=ESP_ERR_NO_MEM;f=makeFrame(1,8);publish(&f,1);
    assert(processNext() && s_protectedRetryActive && s_pool.stats.used==1 && wifiCalls==1);
    sensorarrayTransportDescriptor_t held=s_protectedRetry;
    assert(sensorarrayTransportWaitTicks()==1 && !processNext());nowUs+=10000;
    admissionResult=ESP_OK;assert(processNext());assert(!s_protectedRetryActive && !s_pool.stats.used && wifiCalls==1);
    assert(!sensorarrayTransportDescriptorCurrent(&held));
    reset();admissionResult=ESP_ERR_NO_MEM;f=makeFrame(1,8);publish(&f,1);assert(processNext());
    nowUs+=1000000;assert(sensorarrayTransportWaitTicks()==0 && processNext());
    assert(!s_pool.stats.used && !s_protectedRetryActive && s_transportProtectionStats.expired==1 && bleCalls==1);
    /* Non-busy failures do not become endless retries. */
    reset();admissionResult=ESP_ERR_INVALID_STATE;f=makeFrame(1,8);publish(&f,1);assert(processNext());
    assert(!s_pool.stats.used && !s_protectedRetryActive && s_transportProtectionStats.bleFailed==1);
    /* Owned retry plus pending latest can coexist, and rapid pending modes replace safely. */
    reset();admissionResult=ESP_ERR_NO_MEM;f=makeFrame(1,8);publish(&f,1);assert(processNext());held=s_protectedRetry;
    for(unsigned i=2;i<52;i++){f=makeFrame(i,i+8);publish(&f,1);assert(sensorarrayTransportDescriptorCurrent(&held));}
    assert(s_pool.stats.used==2 && s_transportProtectionStats.superseded==49);
    admissionResult=ESP_OK;assert(processNext() && lastPublication.identity.sequence==51);
    nowUs+=10000;assert(processNext());assert(s_transportProtectionStats.superseded==50 && !s_pool.stats.used);
    /* Both transport slots owned outside its pending queue cannot be reclaimed.
     * A missed first candidate leaves the next available typed frame protected. */
    reset();sensorarrayTransportDescriptor_t ownedSlots[2];
    for(unsigned i=0;i<2;i++)assert(sensorarrayTransportPoolAllocateProtected(&s_pool,2304,&ownedSlots[i]));
    f=makeFrame(1,8);sensorarrayTextPacket_t rejected;
    assert(sensorarrayTextProtocolBuildFrame(&f,&rejected)==ESP_OK);
    assert(sensorarrayNetFramePublish(&rejected,&f,0)==ESP_ERR_TIMEOUT);
    assert(s_transportProtectionStats.firstCandidateMiss==1 && !s_protectedAdmissionKnown);
    for(unsigned i=0;i<2;i++)assert(sensorarrayTransportDescriptorCurrent(&ownedSlots[i]));
    assert(sensorarrayTransportReleaseDescriptor(&ownedSlots[1]));f=makeFrame(2,8);publish(&f,0);
    assert(processNext() && (lastPublication.protectionFlags&1) && lastPublication.identity.sequence==2);
    assert(sensorarrayTransportDescriptorCurrent(&ownedSlots[0]));assert(sensorarrayTransportReleaseDescriptor(&ownedSlots[0]));
    /* Same-config first cannot be replaced by periodic; missed original is counted. */
    reset();f=makeFrame(1,8);publish(&f,1);sensorarrayTransportDescriptor_t owned;
    assert(xQueueReceive(s_protectedQueue,&owned,0)==pdTRUE);
    f=makeFrame(2,9);publish(&f,1);f=makeFrame(3,9);
    sensorarrayTextPacket_t p;assert(sensorarrayTextProtocolBuildFrame(&f,&p)==ESP_OK);
    assert(sensorarrayNetFramePublish(&p,&f,2)==ESP_ERR_TIMEOUT);
    assert(sensorarrayTransportDescriptorCurrent(&owned) && s_transportProtectionStats.admissionMiss==1);
    sensorarrayTransportProcessDescriptor(&owned);assert(processNext());
    nowUs+=1000000;f=makeFrame(4,9);publish(&f,0);assert(processNext() && lastPublication.protectionFlags==2);
    assert(!s_pool.stats.used && s_pool.stats.releaseMismatch==0);
    printf("PROTECTED_TRANSPORT,lifecycle_priority=PASS,independent_pool=7,stale_same_seq_drop=PASS,cross_boot=PASS,retry_deadline=PASS,rapid_configs=50,periodic_derived=PASS\n");
}

/* Independent synthetic adversarial schedule. Exit0 means the stated expired
 * owner retention was reproduced; this is not firmware acceptance PASS. */
static void expiredRetryUnderLifecycleBacklog(void)
{
    reset();
    admissionResult=ESP_ERR_NO_MEM;
    sensorarrayFrame_t f=makeFrame(1,8);
    publish(&f,1);
    assert(processNext() && s_protectedRetryActive);
    sensorarrayTransportDescriptor_t held=s_protectedRetry;
    const uint64_t originalDeadline=sensorarrayTransportDescriptorSlot(&held)->protectedDeadlineUs;
    for(unsigned i=0;i<50;i++) {
        nowUs+=100000;
        bool queued=false;
        assert(sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE,"MAPP",4,&queued)==ESP_OK && queued);
        assert(processNext());
    }
    assert((uint64_t)nowUs>originalDeadline);
    assert(s_protectedRetryActive && sensorarrayTransportDescriptorCurrent(&held));
    assert(s_transportProtectionStats.expired==0 && s_pool.stats.used==1);
    printf("BUG_REPRODUCED,case=expired_retry_behind_continuous_lifecycle,elapsedUs=5000000,deadlineUs=1000000,retainedOwner=1,expiryCount=%u,lifeProcessed=%u,noSdkCancellationClaim=1\n",s_transportProtectionStats.expired,lifeCalls);
    /* No dangling test owner: after priority backlog ends, normal owner
     * finally notices the already-expired deadline and reclaims its slot. */
    assert(processNext());
    assert(!s_protectedRetryActive && !s_pool.stats.used && s_transportProtectionStats.expired==1);
}

int main(void)
{
    setbuf(stdout,NULL);baselineAndSourceTest();byteIdentityTest();transportTest();expiredRetryUnderLifecycleBacklog();
    assert(!pipelineLockDepth);return 0;
}
