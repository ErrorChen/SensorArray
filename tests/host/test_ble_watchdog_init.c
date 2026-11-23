/* Real BLE Init/finish and timer source, synthetic ESP-IDF/RTOS boundary. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayBle.h"
#include "sensorarrayBleWatchdog.h"
#include "esp_timer.h"

typedef unsigned TickType_t;
typedef int portMUX_TYPE;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef void *TaskHandle_t;
typedef int esp_gatt_if_t;
typedef int esp_gatt_status_t;
typedef uint8_t esp_bd_addr_t[6];
typedef struct { unsigned marker; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef struct { unsigned marker; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
typedef int esp_power_level_t;
typedef struct { unsigned value; } esp_bt_controller_config_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define pdPASS 1
#define ESP_BT_MODE_CLASSIC_BT 1
#define ESP_BT_MODE_BLE 2
#define ESP_PWR_LVL_P9 9
#define ESP_BLE_PWR_TYPE_DEFAULT 0
#define ESP_BLE_PWR_TYPE_ADV 1
#define ESP_BLE_PWR_TYPE_SCAN 2
#define ESP_BLE_GAP_PHY_1M_PREF_MASK 1
#define ESP_BLE_GAP_PHY_2M_PREF_MASK 2
#define BT_CONTROLLER_INIT_CONFIG_DEFAULT() ((esp_bt_controller_config_t){0})
#define MALLOC_CAP_SPIRAM 1u
#define MALLOC_CAP_8BIT 2u
static unsigned lockDepth, creates, starts, queues, tasks, sdkCalls, observerCalls;
static unsigned allocations;
static bool failAllocation;
static unsigned failQueue, failTask, failSdk;
static esp_err_t createError, startError;
static bool reenter;
static esp_timer_create_args_t capturedTimer;
static const sensorarrayBleConfig_t configuration = {.deviceName="guard-test"};
#define portENTER_CRITICAL(p) do { (void)(p); assert(!lockDepth); lockDepth++; } while (0)
#define portEXIT_CRITICAL(p) do { (void)(p); assert(lockDepth == 1u); lockDepth--; } while (0)

/* PRODUCTION_TYPES */
/* PRODUCTION_INIT_STATE */
static struct { uint8_t before[16], data[SENSORARRAY_BLE_PAYLOAD_STORAGE_BYTES], after[16]; } payload;
static void *heap_caps_malloc(size_t size, uint32_t caps)
{
    assert(!lockDepth && !creates && !starts && !queues && !tasks && !sdkCalls);
    assert(size == sizeof payload.data && caps == (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    allocations++;
    return failAllocation ? NULL : payload.data;
}
static esp_err_t sensorarrayWorkBudgetInit(void) { assert(!lockDepth); return ESP_OK; }
static void sensorarrayBleResetLinkObservationLocked(void) { assert(lockDepth == 1u); }
static void sensorarrayBleBuildAdvData(void) { assert(!lockDepth); }
static void sensorarrayBleTxTask(void *arg) { (void)arg; }
static void sensorarrayBleControlTask(void *arg) { (void)arg; }
static void sensorarrayBleGapCallback(void) {}
static void sensorarrayBleGattCallback(void) {}
static void sensorarrayBleLogBoot(const char *stage, esp_err_t error) { (void)stage; (void)error; }
static void sensorarrayBleLogHeapDetail(const char *stage, const char *reason) { (void)stage; (void)reason; }
static const char *esp_err_to_name(esp_err_t err) { (void)err; return "synthetic"; }
static QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *storage, StaticQueue_t *queue)
{
    assert(!lockDepth && starts == 1u && startError == ESP_OK && count && size && storage);
    queues++; if (failQueue == queues) return NULL;
    queue->marker = 0xabc000u + queues; return queue;
}
static SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *semaphore)
{ assert(!lockDepth); semaphore->marker=0xdefu; return semaphore; }
static BaseType_t xTaskCreatePinnedToCore(void (*entry)(void *), const char *name,
    unsigned bytes, void *arg, unsigned priority, TaskHandle_t *task, unsigned core)
{
    assert(!lockDepth && starts == 1u && entry && name && bytes && !arg && priority && core == 0u);
    tasks++; if (failTask == tasks) return 0;
    *task=(void *)(uintptr_t)tasks; return pdPASS;
}
static esp_err_t sdkStep(void) { assert(!lockDepth && starts == 1u); sdkCalls++; return failSdk == sdkCalls ? ESP_FAIL : ESP_OK; }
static esp_err_t esp_bt_controller_mem_release(unsigned mode) { assert(mode==ESP_BT_MODE_CLASSIC_BT); return sdkStep(); }
static esp_err_t esp_bt_controller_init(esp_bt_controller_config_t *c) { assert(c); return sdkStep(); }
static esp_err_t esp_bt_controller_enable(unsigned mode) { assert(mode==ESP_BT_MODE_BLE); return sdkStep(); }
static esp_err_t esp_bluedroid_init(void) { return sdkStep(); }
static esp_err_t esp_bluedroid_enable(void) { return sdkStep(); }
static esp_err_t esp_ble_gap_register_callback(void (*cb)(void)) { assert(cb); return sdkStep(); }
static esp_err_t esp_ble_gatts_register_callback(void (*cb)(void)) { assert(cb); return sdkStep(); }
static esp_err_t esp_ble_gatts_app_register(unsigned id) { assert(id); return sdkStep(); }
static esp_err_t esp_ble_gatt_set_local_mtu(unsigned mtu) { assert(mtu == 247u); return sdkStep(); }
static esp_err_t esp_ble_tx_power_set(unsigned role, esp_power_level_t power) { (void)role; (void)power; return sdkStep(); }
static esp_err_t esp_ble_gap_set_preferred_default_phy(unsigned tx, unsigned rx) { assert(tx==rx); return sdkStep(); }

esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *out)
{
    assert(!lockDepth && args && out && args->dispatch_method == ESP_TIMER_TASK);
    assert(args->skip_unhandled_events && !strcmp(args->name, "bleSendGuard"));
    creates++; capturedTimer=*args;
    if (createError == ESP_OK) *out=(void *)0x1234;
    return createError;
}
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period)
{
    assert(!lockDepth && timer == (void *)0x1234 && period == 100000u);
    starts++;
    if (reenter) {
        assert(sensorarrayBleInit(&configuration) == ESP_ERR_INVALID_STATE);
        assert(creates == 1u && starts == 1u && queues == 0u && tasks == 0u);
    }
    return startError;
}
void sensorarrayBleObserveSendDeadline(void) { assert(!lockDepth); observerCalls++; }
#include "sensorarrayBleWatchdog.c"
/* PRODUCTION_INIT_FUNCTIONS */

static void newSyntheticBoot(void)
{
    /* Test fixture only: emulate separate boots. No reset/deinit API is added. */
    memset(&s_ble, 0, sizeof(s_ble));
    s_bleInitAttempted=s_initAttempted=false;
    s_bleInitResult=s_initResult=ESP_ERR_INVALID_STATE;
    s_sendTimer=NULL;
    lockDepth=creates=starts=queues=tasks=sdkCalls=observerCalls=0u;
    allocations=0u; failAllocation=false; memset(&payload, 0xa7, sizeof payload);
    failQueue=failTask=failSdk=0u; createError=startError=ESP_OK; reenter=false;
}
static void assertSticky(esp_err_t error)
{
    sensorarrayBleState_t before=s_ble;
    uint8_t payloadBefore[sizeof payload.data]; memcpy(payloadBefore, payload.data, sizeof payloadBefore);
    unsigned c=creates, s=starts, q=queues, t=tasks, b=sdkCalls, a=allocations;
    assert(sensorarrayBleInit(&configuration) == error);
    assert(!memcmp(&before, &s_ble, sizeof before));
    assert(c==creates && s==starts && q==queues && t==tasks && b==sdkCalls && a==allocations);
    assert(!memcmp(payloadBefore, payload.data, sizeof payloadBefore));
}
static void assertPayloadLayout(void)
{
    uint8_t *slots[SENSORARRAY_BLE_PAYLOAD_SLOT_COUNT]; unsigned count=0u;
    for (unsigned i=0u; i<CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN; i++) slots[count++]=s_ble.dataSlots[i].data;
    for (unsigned i=0u; i<SENSORARRAY_BLE_LOG_TX_SLOT_COUNT; i++) slots[count++]=s_ble.logSlots[i].data;
    for (unsigned i=0u; i<SENSORARRAY_BLE_PROTECTED_SLOT_COUNT; i++) slots[count++]=s_ble.protectedSlots[i].data;
    assert(count==8u && sizeof payload.data==18432u && allocations==1u && s_ble.payloadStorage==payload.data);
    assert(sizeof s_ble.ctrlSlots[0].data==512u && sizeof s_ble.txPacketScratch==512u);
    for (unsigned i=0u; i<count; i++) {
        assert(slots[i]==payload.data+i*SENSORARRAY_BLE_MESSAGE_VALUE_MAX);
        memset(slots[i], (int)(i+1u), SENSORARRAY_BLE_MESSAGE_VALUE_MAX);
    }
    for (unsigned i=0u; i<count; i++) for (unsigned j=0u; j<SENSORARRAY_BLE_MESSAGE_VALUE_MAX; j++)
        assert(slots[i][j]==i+1u);
    for (unsigned i=0u; i<16u; i++) assert(payload.before[i]==0xa7u && payload.after[i]==0xa7u);
}
int main(void)
{
    newSyntheticBoot(); assert(sensorarrayBleInit(NULL) == ESP_ERR_INVALID_ARG);
    assert(!s_bleInitAttempted && !creates && !allocations);
    failAllocation=true;
    assert(sensorarrayBleInit(&configuration)==ESP_ERR_NO_MEM && allocations==1u);
    assert(!creates && !starts && !queues && !tasks && !sdkCalls && !s_ble.payloadStorage);
    assert(!s_ble.guard.ready && s_ble.guard.initError==ESP_ERR_NO_MEM);
    for (unsigned i=0u; i<CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN; i++) assert(!s_ble.dataSlots[i].data);
    failAllocation=false; assertSticky(ESP_ERR_NO_MEM);

    newSyntheticBoot();
    createError=ESP_ERR_NO_MEM;
    assert(sensorarrayBleInit(&configuration) == ESP_ERR_NO_MEM);
    assert(creates==1u && !starts && !queues && !tasks && !sdkCalls && !s_ble.guard.ready);
    assert(s_ble.guard.initError==ESP_ERR_NO_MEM); assertSticky(ESP_ERR_NO_MEM);
    assert(sensorarrayBleWatchdogInit()==ESP_ERR_NO_MEM && creates==1u);

    newSyntheticBoot(); startError=ESP_FAIL;
    assert(sensorarrayBleInit(&configuration)==ESP_FAIL);
    assert(creates==1u && starts==1u && !queues && !tasks && s_sendTimer==(void *)0x1234);
    assert(!s_ble.guard.ready && s_ble.guard.initError==ESP_FAIL); assertSticky(ESP_FAIL);
    assert(sensorarrayBleWatchdogInit()==ESP_FAIL && creates==1u && starts==1u);

    for (unsigned phase=0u; phase<4u; ++phase) {
        newSyntheticBoot();
        if (phase==0u) failQueue=2u;
        if (phase==1u) failTask=1u;
        if (phase==2u) failTask=2u;
        if (phase==3u) failSdk=2u;
        esp_err_t expected=phase==3u ? ESP_FAIL : ESP_ERR_NO_MEM;
        assert(sensorarrayBleInit(&configuration)==expected && s_ble.guard.ready);
        assertSticky(expected);
        if (phase>=2u) assert(s_ble.txTask==(void *)1 && s_ble.txTaskStarted);
        if (phase==3u) assert(s_ble.controlTask==(void *)2 && s_ble.controlTaskStarted);
        capturedTimer.callback(capturedTimer.arg); assert(observerCalls==1u);
    }
    newSyntheticBoot(); reenter=true;
    assert(sensorarrayBleInit(&configuration)==ESP_OK && s_ble.initialized && s_ble.guard.ready);
    assert(creates==1u && starts==1u && queues==4u && tasks==2u && sdkCalls==9u);
    assertPayloadLayout();
    assertSticky(ESP_OK);
    s_ble.guard.quarantined=true; s_ble.guard.callId=17u; s_ble.guard.startUs=200u;
    assertSticky(ESP_OK); assert(s_ble.guard.quarantined && s_ble.guard.callId==17u);
    assert(sensorarrayBleWatchdogInit()==ESP_OK && creates==1u && starts==1u);
    capturedTimer.callback(capturedTimer.arg); assert(observerCalls==1u && !lockDepth);
    puts("BLE_GUARD_INIT_PASS,cases=8,timer_task=1,period_us=100000,skip_missed=1,partial_retry_blocked=1,concurrent_claim=1,psram_only=1,payload_slots=8,payload_bytes=18432,alloc_failure_sticky=1,hardware=NOT_RUN");
    return 0;
}
