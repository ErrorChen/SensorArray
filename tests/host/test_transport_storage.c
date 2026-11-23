#define _POSIX_C_SOURCE 200809L
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "sensorarrayTransport.h"
#include "sensorarrayTransportPool.h"
#include "sensorarrayBle.h"

typedef unsigned BaseType_t;
typedef struct { unsigned count; size_t itemSize; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef void *TaskHandle_t;
#define pdPASS 1u
#define ESP_MAC_WIFI_SOFTAP 1u
#define MALLOC_CAP_SPIRAM 0x400u
#define MALLOC_CAP_8BIT 4u
#define ESP_ERR_NVS_NO_FREE_PAGES 0x110du
#define ESP_ERR_NVS_NEW_VERSION_FOUND 0x1110u
static pthread_mutex_t g_sensorarrayTransportStatsMux=PTHREAD_MUTEX_INITIALIZER;
static _Thread_local unsigned lockDepth;
#define portENTER_CRITICAL(m) do { assert(!lockDepth); assert(!pthread_mutex_lock(m)); lockDepth++; } while(0)
#define portEXIT_CRITICAL(m) do { assert(lockDepth==1); lockDepth--; assert(!pthread_mutex_unlock(m)); } while(0)
#include "storage_state.h"

static unsigned allocations, releases, queuesCreated, taskAttempts, tasksCreated;
static unsigned serialInits, nvsInits, nvsErases, bleInits, failTaskAt;
static esp_err_t serialResult, nvsResult, bleResult;
static bool failAllocation, holdAllocation, allocationEntered, allocationReleased;
static pthread_mutex_t allocatorMutex=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t allocatorCond=PTHREAD_COND_INITIALIZER;
static void *allocation;
static unsigned taskTokens[2];
static void unlocked(void) { assert(!lockDepth); }
static void sensorarrayTransportTask(void *arg) { (void)arg; }
static void sensorarrayTransportSerialControlTask(void *arg) { (void)arg; }
static void sensorarrayTransportNotifyTask(void) { }
static void sensorarrayTransportBleControl(const uint8_t *data,size_t length,void *context) { (void)data;(void)length;(void)context; }
static void sensorarrayTransportWifiControl(const uint8_t *data,size_t length,const sensorarrayWifiPeer_t *peer,void *context) { (void)data;(void)length;(void)peer;(void)context; }
static void sensorarrayTransportWifiCompleted(const sensorarrayWifiTxResult_t *result,void *context) { (void)result;(void)context; }
static void sensorarrayTransportBuildNames(const uint8_t mac[6]) { unlocked();assert(mac);strcpy(s_deviceName,"storage-host"); }
static esp_err_t sensorarrayWorkBudgetInit(void) { unlocked();return ESP_OK; }
static esp_err_t sensorarraySerialIoInit(void) { unlocked();serialInits++;return serialResult; }
static esp_err_t sensorarrayScanConfigInit(void) { unlocked();return ESP_OK; }
static esp_err_t esp_read_mac(uint8_t mac[6],unsigned type) { unlocked();assert(type==ESP_MAC_WIFI_SOFTAP);memset(mac,1,6);return ESP_OK; }
void sensorarrayTransportSetStream(sensorarrayTransportStream_t stream) { unlocked();assert(stream==SENSORARRAY_TRANSPORT_STREAM_AUTO); }
void sensorarrayTransportSetTxMode(sensorarrayTransportTxMode_t mode) { unlocked();assert(mode<=SENSORARRAY_TRANSPORT_TX_FULL); }
void sensorarrayTransportSetWifiMode(sensorarrayTransportWifiMode_t mode) { unlocked();assert(mode==SENSORARRAY_TRANSPORT_WIFI_OFF); }
static esp_err_t nvs_flash_init(void) { unlocked();nvsInits++;return nvsResult; }
esp_err_t nvs_flash_erase(void) { unlocked();nvsErases++;return ESP_OK; }
void sensorarrayBleSetControlRxCallback(sensorarrayBleControlRxCallback_t callback,void *ctx) { unlocked();assert(callback==sensorarrayTransportBleControl && !ctx); }
esp_err_t sensorarrayBleInit(const sensorarrayBleConfig_t *config) { unlocked();bleInits++;assert(config->preferredMtu==517u);return bleResult; }
void sensorarrayWifiSetControlRxCallback(sensorarrayWifiControlRxCallback_t callback,void *ctx) { unlocked();assert(callback==sensorarrayTransportWifiControl && !ctx); }
void sensorarrayWifiSetOwnerCallbacks(sensorarrayWifiTxResultCallback_t callback,void *ctx,sensorarrayWifiWakeCallback_t wake) { unlocked();assert(callback==sensorarrayTransportWifiCompleted && !ctx && wake==sensorarrayTransportNotifyTask); }
static void sensorarrayTransportGuaranteedTextSetNotify(void (*notify)(void)) { unlocked();assert(notify==sensorarrayTransportNotifyTask); }
static QueueHandle_t xQueueCreateStatic(unsigned count,size_t itemSize,uint8_t *data,StaticQueue_t *queue) {
    unlocked();assert(data && queue && itemSize==sizeof(sensorarrayTransportDescriptor_t));
    assert(!queue->count);queue->count=count;queue->itemSize=itemSize;queuesCreated++;return queue;
}
static BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *),const char *name,unsigned stack,void *arg,unsigned priority,TaskHandle_t *handle,int core) {
    unlocked();(void)arg;assert(name && stack && core==0 && handle && !*handle);
    taskAttempts++;
    if(taskAttempts==failTaskAt)return 0;
    unsigned index=entry==sensorarrayTransportTask?0u:1u;
    assert(index==0 || entry==sensorarrayTransportSerialControlTask);
    assert(priority==(index==0?7u:8u));
    *handle=&taskTokens[index];tasksCreated++;return pdPASS;
}
static void *heap_caps_malloc(size_t size,unsigned caps) {
    unlocked();assert(size==SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES);
    assert(caps==(MALLOC_CAP_SPIRAM|MALLOC_CAP_8BIT));allocations++;
    if(holdAllocation) {
        assert(!pthread_mutex_lock(&allocatorMutex));allocationEntered=true;
        assert(!pthread_cond_broadcast(&allocatorCond));
        while(!allocationReleased)assert(!pthread_cond_wait(&allocatorCond,&allocatorMutex));
        assert(!pthread_mutex_unlock(&allocatorMutex));
    }
    if(failAllocation)return NULL;
    assert(!allocation);allocation=malloc(size);assert(allocation);memset(allocation,0xcc,size);return allocation;
}
static void heap_caps_free(void *data) { unlocked();assert(data==allocation);free(data);allocation=NULL;releases++; }
#include "storage_functions.h"

static sensorarrayTransportMemoryDiagnostics_t diagnostics(void) {
    sensorarrayTransportMemoryDiagnostics_t d;sensorarrayTransportGetMemoryDiagnostics(&d);return d;
}
static void setup(void) {
    assert(!lockDepth && !s_initInProgress);free(allocation);allocation=NULL;
    memset(&s_pool,0,sizeof s_pool);s_payloadStorageState=0;s_payloadStorageError=ESP_ERR_INVALID_STATE;
    s_completedInitError=ESP_ERR_INVALID_STATE;s_started=false;
    s_dataQueue=s_logQueue=s_lifecycleQueue=s_protectedQueue=NULL;s_task=s_serialTask=NULL;
    memset(&s_dataQueueStruct,0,sizeof s_dataQueueStruct);memset(&s_logQueueStruct,0,sizeof s_logQueueStruct);
    memset(&s_lifecycleQueueStruct,0,sizeof s_lifecycleQueueStruct);memset(&s_protectedQueueStruct,0,sizeof s_protectedQueueStruct);
    allocations=releases=queuesCreated=taskAttempts=tasksCreated=serialInits=nvsInits=nvsErases=bleInits=failTaskAt=0;
    serialResult=nvsResult=bleResult=ESP_OK;failAllocation=holdAllocation=allocationEntered=allocationReleased=false;
}
static void checkControlAlive(void) { assert(s_started && s_task && s_serialTask && tasksCreated==2 && queuesCreated==4); }
static sensorarrayTransportDescriptor_t ownMaximum(void) {
    sensorarrayTransportDescriptor_t d;assert(sensorarrayTransportPoolAllocate(&s_pool,SENSORARRAY_TRANSPORT_CHANNEL_DATA,2304,&d));
    sensorarrayTransportPayloadSlot_t *slot=sensorarrayTransportPoolGetSlot(&s_pool,&d);
    memset(slot->data,'X',2304);slot->data[2304]=0;return d;
}
static void checkOwned(const sensorarrayTransportDescriptor_t *d,const sensorarrayTransportPool_t *held) {
    assert(memcmp(&s_pool,held,sizeof s_pool)==0 && sensorarrayTransportPoolValidate(&s_pool,d));
    for(unsigned i=0;i<2304;i++)assert(s_pool.slots[d->slotIndex].data[i]=='X');
    assert(s_pool.slots[d->slotIndex].data[2304]==0);
}
static void testSuccessAndRepeatedInit(void) {
    setup();sensorarrayTransportMemoryDiagnostics_t d=diagnostics();
    assert(!d.payloadStorageReady && d.payloadStorageError==ESP_ERR_INVALID_STATE && d.payloadExternalBytes==0);
    assert(sensorarrayTransportInit()==ESP_OK);checkControlAlive();
    d=diagnostics();assert(d.payloadStorageReady && d.payloadStorageError==ESP_OK);
    assert(d.payloadExternalBytes==16135 && d.payloadStorageBytes==16135);
    assert(d.payloadPoolBytes==sizeof s_pool.slots+16135 && d.payloadSlotBytes==sizeof s_pool.slots[0]+2305);
    assert(d.payloadInternalBytes>=sizeof s_pool && d.payloadInternalBytes<2048);
    sensorarrayTransportDescriptor_t owned=ownMaximum();sensorarrayTransportPool_t held=s_pool;
    assert(sensorarrayTransportInit()==ESP_OK);checkOwned(&owned,&held);
    assert(allocations==1 && releases==0 && queuesCreated==4 && taskAttempts==2 && nvsInits==1 && bleInits==1);
    assert(sensorarrayTransportPoolRelease(&s_pool,&owned));
}
static void testStickyAllocationFailure(void) {
    setup();failAllocation=true;assert(sensorarrayTransportInit()==ESP_ERR_NO_MEM);checkControlAlive();
    sensorarrayTransportMemoryDiagnostics_t d=diagnostics();
    assert(!d.payloadStorageReady && d.payloadStorageError==ESP_ERR_NO_MEM && d.payloadExternalBytes==0);
    assert(d.payloadPoolBytes==sizeof s_pool.slots);
    failAllocation=false;assert(sensorarrayTransportInit()==ESP_ERR_NO_MEM);checkControlAlive();
    assert(allocations==1 && releases==0 && !s_pool.storage && nvsInits==1 && bleInits==1);
    sensorarrayTransportDescriptor_t rejected;
    assert(!sensorarrayTransportPoolAllocate(&s_pool,SENSORARRAY_TRANSPORT_CHANNEL_LIFECYCLE,1,&rejected));
}
static void testPartialTaskStartup(void) {
    setup();failTaskAt=2;assert(sensorarrayTransportInit()==ESP_ERR_NO_MEM);
    assert(s_task && !s_serialTask && !s_started && queuesCreated==4);
    sensorarrayTransportDescriptor_t owned=ownMaximum();sensorarrayTransportPool_t held=s_pool;
    failTaskAt=0;assert(sensorarrayTransportInit()==ESP_OK);checkControlAlive();checkOwned(&owned,&held);
    assert(allocations==1 && releases==0 && taskAttempts==3 && queuesCreated==4);
}
static void testNvsPreserved(void) {
    const esp_err_t errors[]={ESP_ERR_NVS_NO_FREE_PAGES,ESP_ERR_NVS_NEW_VERSION_FOUND};
    for(unsigned i=0;i<2;i++) {
        setup();nvsResult=errors[i];assert(sensorarrayTransportInit()==errors[i]);checkControlAlive();
        assert(nvsInits==1 && nvsErases==0 && bleInits==0);
        nvsResult=ESP_OK;assert(sensorarrayTransportInit()==errors[i]);
        assert(nvsInits==1 && nvsErases==0 && bleInits==0 && allocations==1);
    }
    setup();bleResult=ESP_ERR_INVALID_RESPONSE;assert(sensorarrayTransportInit()==bleResult);checkControlAlive();
    bleResult=ESP_OK;assert(sensorarrayTransportInit()==ESP_ERR_INVALID_RESPONSE);assert(bleInits==1);
    setup();serialResult=ESP_FAIL;assert(sensorarrayTransportInit()==ESP_FAIL);checkControlAlive();
    serialResult=ESP_OK;assert(sensorarrayTransportInit()==ESP_OK);assert(serialInits==2 && allocations==1);
    setup();failAllocation=true;nvsResult=ESP_ERR_NVS_NEW_VERSION_FOUND;
    assert(sensorarrayTransportInit()==nvsResult);assert(sensorarrayTransportInit()==nvsResult);
    assert(diagnostics().payloadStorageError==ESP_ERR_NO_MEM && !nvsErases);
}
static void *initThread(void *arg) { *(esp_err_t *)arg=sensorarrayTransportInit();return NULL; }
static void testConcurrentInit(void) {
    setup();holdAllocation=true;pthread_t thread;esp_err_t result=ESP_FAIL;
    assert(!pthread_create(&thread,NULL,initThread,&result));
    assert(!pthread_mutex_lock(&allocatorMutex));
    struct timespec deadline;assert(!clock_gettime(CLOCK_REALTIME,&deadline));deadline.tv_sec+=2;
    while(!allocationEntered)assert(!pthread_cond_timedwait(&allocatorCond,&allocatorMutex,&deadline));
    assert(!pthread_mutex_unlock(&allocatorMutex));
    assert(sensorarrayTransportInit()==ESP_ERR_INVALID_STATE);
    sensorarrayTransportMemoryDiagnostics_t d=diagnostics();
    assert(!d.payloadStorageReady && d.payloadExternalBytes==0 && d.payloadStorageError==ESP_ERR_INVALID_STATE);
    assert(allocations==1 && queuesCreated==0);
    assert(!pthread_mutex_lock(&allocatorMutex));allocationReleased=true;assert(!pthread_cond_broadcast(&allocatorCond));
    assert(!pthread_mutex_unlock(&allocatorMutex));assert(!pthread_join(thread,NULL));
    assert(result==ESP_OK && allocations==1);checkControlAlive();
}
static void testNoRebindAfterPartialBinding(void) {
    setup();static char existing[SENSORARRAY_TRANSPORT_POOL_STORAGE_BYTES];
    assert(sensorarrayTransportPoolInit(&s_pool,existing,sizeof existing));
    sensorarrayTransportDescriptor_t owned=ownMaximum();sensorarrayTransportPool_t held=s_pool;
    assert(sensorarrayTransportInit()==ESP_ERR_INVALID_STATE);checkControlAlive();checkOwned(&owned,&held);
    assert(allocations==1 && releases==1 && !allocation && !diagnostics().payloadStorageReady);
    assert(sensorarrayTransportInit()==ESP_ERR_INVALID_STATE && allocations==1);checkOwned(&owned,&held);
}
int main(void) {
    testSuccessAndRepeatedInit();testStickyAllocationFailure();testPartialTaskStartup();
    testNvsPreserved();testConcurrentInit();testNoRebindAfterPartialBinding();setup();
    puts("TRANSPORT_STORAGE,groups=6,actual_init=1,psram_only=16135,repeat_owned=PASS,sticky_fail=PASS,parallel_claim=PASS,control_alive=PASS,nvs_no_erase=PASS,hardware=NOT_RUN");return 0;
}
