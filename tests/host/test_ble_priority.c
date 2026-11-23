/* Production BLE bodies are inserted by test_ble_priority.py. No radio/HIL. */
#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayBle.h"
#include "sensorarrayBleWatchdog.h"
#include "mtu_reference.h"

typedef unsigned TickType_t;
typedef int portMUX_TYPE;
typedef int BaseType_t;
typedef unsigned UBaseType_t;
typedef void *TaskHandle_t;
typedef int esp_gatt_if_t;
typedef int esp_gatt_status_t;
typedef uint8_t esp_bd_addr_t[6];
typedef int esp_ble_gap_phy_mask_t;
typedef struct { uint8_t *storage; unsigned capacity, itemSize, count, head; } StaticQueue_t;
typedef StaticQueue_t *QueueHandle_t;
typedef struct { unsigned token; } StaticSemaphore_t;
typedef StaticSemaphore_t *SemaphoreHandle_t;
#define portMUX_INITIALIZER_UNLOCKED 0
#define pdTRUE 1
#define pdFALSE 0
#define portMAX_DELAY UINT32_MAX
#define pdMS_TO_TICKS(ms) ((ms) / 10u)
#define ESP_GATT_OK 0
#define ESP_GATT_ERROR 1
#define ESP_GATTS_CONF_EVT 1
#define ESP_GATTS_CONNECT_EVT 2
#define ESP_GATTS_DISCONNECT_EVT 3
#define ESP_GATTS_MTU_EVT 4
#define ESP_BT_STATUS_SUCCESS 0
#define ESP_BLE_GAP_PHY_1M_PREF_MASK 1
#define ESP_BLE_GAP_PHY_2M_PREF_MASK 2
#define ESP_BLE_GAP_PHY_OPTIONS_NO_PREF 0
#define SENSORARRAY_WORK_BUDGET_BLE_TX 0
typedef enum {
    ESP_GAP_BLE_READ_PHY_COMPLETE_EVT,
    ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT,
    ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT,
    ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT,
    ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT,
    TEST_OTHER_EVENT
} esp_gap_ble_cb_event_t;
typedef struct { esp_bd_addr_t bda; uint16_t min_int, max_int, latency, timeout; } esp_ble_conn_update_params_t;
typedef union {
    struct { int status; esp_bd_addr_t bda; uint8_t tx_phy, rx_phy; } read_phy, phy_update;
    struct { int status; esp_bd_addr_t bda; uint16_t min_int, max_int, latency, conn_int, timeout; } update_conn_params;
    struct { int status; } set_perf_phy;
    struct { int status; struct { uint16_t rx_len, tx_len; } params; } pkt_data_length_cmpl;
} esp_ble_gap_cb_param_t;
typedef union {
    struct { uint16_t conn_id, handle; esp_gatt_status_t status; } conf;
    struct { uint16_t conn_id; esp_bd_addr_t remote_bda; } connect;
    struct { uint16_t conn_id, mtu; } mtu;
} esp_ble_gatts_cb_param_t;

static unsigned lockDepth, checkpoints, wakeGives, idleTakes, delayCalls, queueInitCount, failQueueInit;
static unsigned failQueueSends, sdkCalls, sdkFailures, disconnectAfter, priorityOnSend, disableDataAfter;
static unsigned phyRequests, readRequests, connRequests, dleRequests;
static uint64_t nowUs, sdkDelayUs;
static uint32_t idleGeneration, sdkIdleAfter;
static bool changeIdleDuringSdk;
static esp_err_t phyReturn, fallbackReturn, readReturn, connReturn, dleReturn;
static bool wakeOnlyDuringConfirm, confirmSuccess, injectOnIdle, injectLogOnSend, enableDataOnSend;
static bool observeDuringSdk, observeDuringConfirm;
static bool resubscribeDuringSdk;
static unsigned typedCopyInjection;
static uint32_t notifications;
static unsigned confWakeGives;
static TaskHandle_t lastConfWakeTask;
static jmp_buf taskIdle;
static uint8_t sentBytes[65536];
static size_t sentLength;
static char sentChannels[256];
static uint16_t sentLengths[256];
static unsigned sentChannelCount;
static FILE *wireJournal;
static const char *wireCase;
static void journalWire(const char *type, char channel, const uint8_t *data, size_t length)
{
    if (!wireCase) return;
    assert(wireJournal);
    fprintf(wireJournal, "{\"type\":\"%s\",\"case\":\"%s\",\"channel\":\"%c\",\"hex\":\"", type, wireCase, channel);
    for (size_t i = 0; i < length; ++i) fprintf(wireJournal, "%02x", data[i]);
    fputs("\"}\n", wireJournal);
}
#define portENTER_CRITICAL(m) do { (void)(m); assert(lockDepth == 0u); lockDepth++; } while (0)
#define portEXIT_CRITICAL(m) do { (void)(m); assert(lockDepth == 1u); lockDepth--; } while (0)
static int64_t esp_timer_get_time(void) { return (int64_t)nowUs; }
static TickType_t xTaskGetTickCount(void) { return (TickType_t)(nowUs / 10000u); }
static TaskHandle_t xTaskGetCurrentTaskHandle(void) { return (void *)1; }
static UBaseType_t uxTaskPriorityGet(TaskHandle_t task) { assert(!lockDepth); return task == (void *)1 ? 7u : 8u; }
static bool sensorarrayWorkBudgetCheckpoint(unsigned worker)
{ assert(!lockDepth && worker == 0u); checkpoints++; return false; }
static uint32_t sensorarrayWorkBudgetIdleGeneration(void)
{ assert(!lockDepth); return idleGeneration; }
static QueueHandle_t xQueueCreateStatic(unsigned count, unsigned size, uint8_t *storage, StaticQueue_t *queue)
{
    queueInitCount++;
    if (failQueueInit && failQueueInit == queueInitCount) { return NULL; }
    *queue = (StaticQueue_t){.storage=storage, .capacity=count, .itemSize=size};
    return queue;
}
static int xQueueSend(QueueHandle_t queue, const void *value, TickType_t timeout)
{
    assert(queue && !lockDepth && timeout == 0u);
    if (failQueueSends) { failQueueSends--; return pdFALSE; }
    if (queue->count == queue->capacity) { return pdFALSE; }
    unsigned tail = (queue->head + queue->count) % queue->capacity;
    memcpy(queue->storage + tail * queue->itemSize, value, queue->itemSize);
    queue->count++;
    return pdTRUE;
}
static int xQueueReceive(QueueHandle_t queue, void *out, TickType_t timeout)
{
    assert(queue && !lockDepth && timeout == 0u);
    if (!queue->count) { return pdFALSE; }
    memcpy(out, queue->storage + queue->head * queue->itemSize, queue->itemSize);
    queue->head = (queue->head + 1u) % queue->capacity;
    queue->count--;
    return pdTRUE;
}
static SemaphoreHandle_t xSemaphoreCreateBinaryStatic(StaticSemaphore_t *semaphore)
{ semaphore->token = 0u; return semaphore; }
static int xSemaphoreGive(SemaphoreHandle_t semaphore)
{ assert(semaphore && !lockDepth); semaphore->token = 1u; wakeGives++; return pdTRUE; }
static int xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout);
static uint32_t ulTaskNotifyTake(int clear, TickType_t timeout);
static void xTaskNotifyGive(TaskHandle_t task)
{
    assert(!lockDepth && task != NULL); /* Callback must wake outside mux. */
    confWakeGives++; lastConfWakeTask = task; notifications++;
}
static void vTaskDelay(TickType_t ticks);
static esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t interface, uint16_t conn,
    uint16_t handle, uint16_t length, uint8_t *value, bool confirm);
static esp_err_t esp_ble_gap_set_preferred_phy(esp_bd_addr_t peer, unsigned all,
    esp_ble_gap_phy_mask_t tx, esp_ble_gap_phy_mask_t rx, unsigned options);
static esp_err_t esp_ble_gap_read_phy(esp_bd_addr_t peer);
static esp_err_t esp_ble_gap_update_conn_params(esp_ble_conn_update_params_t *params);
static esp_err_t esp_ble_gap_set_pkt_data_len(esp_bd_addr_t peer, uint16_t length);
static void sensorarrayBleDrainControlRxQueue(void) { assert(!lockDepth); }
static void sensorarrayBleStartAdvertising(void) { assert(!lockDepth); }
static void testTypedFrameCopy(void *destination, const void *source, size_t length);

/* PRODUCTION_TYPES */
#define MALLOC_CAP_SPIRAM 1u
#define MALLOC_CAP_8BIT 2u
static uint8_t payloadStorage[SENSORARRAY_BLE_PAYLOAD_STORAGE_BYTES];
static unsigned payloadAllocations;
static void *heap_caps_malloc(size_t bytes, uint32_t caps)
{
    assert(!lockDepth && bytes==sizeof payloadStorage && caps==(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    payloadAllocations++;
    return payloadStorage;
}
/* PRODUCTION_FUNCTIONS */

static void dispatchConf(const esp_ble_gatts_cb_param_t *param)
{
    switch (ESP_GATTS_CONF_EVT) {
        /* PRODUCTION_CONF_CASE */
    default: assert(false);
    }
}
static void dispatchConnection(unsigned event, const esp_ble_gatts_cb_param_t *param)
{
    const esp_gatt_if_t gattsIf = 2;
    switch (event) {
        /* PRODUCTION_CONNECTION_CASES */
    default: assert(false);
    }
}

static esp_err_t initializeQueues(void)
{
    /* PRODUCTION_QUEUE_INIT */
    return ESP_OK;
}
static int xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t timeout)
{
    assert(!lockDepth && timeout == portMAX_DELAY && semaphore == s_ble.txWake);
    idleTakes++;
    if (injectOnIdle) {
        injectOnIdle = false;
        assert(sensorarrayBleNotify(SENSORARRAY_BLE_CH_CTRL, (const uint8_t *)"CTRL\n", 5u) == ESP_OK);
    }
    if (semaphore->token) { semaphore->token = 0u; return pdTRUE; }
    assert(s_ble.txQueue->count == 0u && s_ble.logTxQueue->count == 0u && s_ble.ctrlTxQueue->count == 0u);
    longjmp(taskIdle, 1);
}
static uint32_t ulTaskNotifyTake(int clear, TickType_t timeout)
{
    assert(!lockDepth && clear == pdTRUE);
    if (timeout) {
        if (wakeOnlyDuringConfirm) { (void)xSemaphoreGive(s_ble.txWake); }
        if (confirmSuccess) {
            s_ble.confirmStatus = ESP_GATT_OK;
            s_ble.confirmPending = false;
            notifications++;
        }
        if (!notifications) { nowUs += (uint64_t)timeout * 10000u; }
        if (observeDuringConfirm) {
            sensorarrayBleObserveSendDeadline();
            assert(!s_ble.link.sdkCallActive && !s_ble.guard.quarantined);
        }
    }
    uint32_t result = notifications; notifications = 0u; return result;
}
static void vTaskDelay(TickType_t ticks)
{ assert(!lockDepth && ticks > 0u); nowUs += (uint64_t)ticks * 10000u; delayCalls++; }
static esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t interface, uint16_t conn,
    uint16_t handle, uint16_t length, uint8_t *value, bool confirm)
{
    (void)confirm;
    size_t maximum = s_ble.stats.mtu > 3u ? s_ble.stats.mtu - 3u : 20u;
    if (maximum > SENSORARRAY_BLE_ATTR_VALUE_MAX) maximum = SENSORARRAY_BLE_ATTR_VALUE_MAX;
    /* Keeps the original <=244 assertion at MTU247, and checks the actual
     * negotiated bound instead of fixing every synthetic peer to247. */
    assert(!lockDepth && interface == 2 && conn == 3 && value && length <= maximum);
    assert(s_ble.link.sdkCallActive && s_ble.link.sdkCallStartedUs == nowUs);
    assert(s_ble.link.sdkCallCount == s_ble.link.sdkCompletedCalls + 1u);
    assert(s_ble.stats.txSlotUsed > 0u); /* Caller still owns persistent payload. */
    sdkCalls++;
    if (observeDuringSdk) {
        uint8_t *persistentStorage = s_ble.payloadStorage;
        uint32_t slots = s_ble.stats.txSlotUsed;
        uint32_t queued = s_ble.txQueue->count + s_ble.ctrlTxQueue->count + s_ble.logTxQueue->count;
        uint64_t start = nowUs;
        nowUs = start + SENSORARRAY_BLE_SEND_DEADLINE_US - 1u;
        sensorarrayBleObserveSendDeadline(); assert(!s_ble.guard.quarantined);
        nowUs++;
        sensorarrayBleObserveSendDeadline();
        sensorarrayBleGuardSnapshot_t guard;
        sensorarrayBleGetGuardSnapshot(&guard);
        assert(guard.quarantined && guard.active && !guard.completionKnown && guard.callKnown);
        assert(guard.startUs == start && guard.detectedUs == nowUs && guard.ageUs == 500000u);
        assert(guard.callId == s_ble.link.sdkCallCount && guard.restartRequired);
        assert(sensorarrayBleNotify(SENSORARRAY_BLE_CH_DATA, (const uint8_t *)"DENIED\n", 7u) == ESP_ERR_INVALID_STATE);
        assert(s_ble.stats.txSlotUsed == slots && sdkCalls == 1u && s_ble.guard.rejected == 1u);
        assert(queued == s_ble.txQueue->count + s_ble.ctrlTxQueue->count + s_ble.logTxQueue->count);
        assert(s_ble.stats.txConnectionStaleDrop == 0u);
        assert(s_ble.payloadStorage == persistentStorage && payloadAllocations == 1u);
        for (unsigned i=0u; i<CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN; ++i) {
            sensorarrayBleDataSlot_t *slot=&s_ble.dataSlots[i];
            if (slot->inUse) assert(sensorarrayBleCrc32(slot->data,slot->length)==slot->crc);
        }
        nowUs += 300000u; sensorarrayBleObserveSendDeadline();
        assert(s_ble.guard.detectedUs == start + 500000u && s_ble.link.sdkCallActive);
    }
    nowUs += sdkDelayUs;
    if (changeIdleDuringSdk) { idleGeneration = sdkIdleAfter; }
    if (sdkCalls <= sdkFailures) { return ESP_FAIL; }
    assert(sentLength + length <= sizeof(sentBytes));
    memcpy(sentBytes + sentLength, value, length); sentLength += length;
    assert(sentChannelCount < sizeof(sentChannels));
    sentLengths[sentChannelCount] = length;
    sentChannels[sentChannelCount++] = handle == 20 ? 'D' : (handle == 30 ? 'L' : 'C');
    journalWire("notification", sentChannels[sentChannelCount - 1u], value, length);
    if (resubscribeDuringSdk) {
        resubscribeDuringSdk = false;
        portENTER_CRITICAL(&s_bleMux);
        s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u; sensorarrayBleFrameFlowSyncLocked();
        s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 1u; sensorarrayBleFrameFlowSyncLocked();
        portEXIT_CRITICAL(&s_bleMux);
    }
    if (injectLogOnSend) {
        injectLogOnSend = false;
        assert(sensorarrayBleNotify(SENSORARRAY_BLE_CH_LOG, (const uint8_t *)"LATELOG\n", 8u) == ESP_ERR_NOT_ALLOWED);
    }
    if (enableDataOnSend) { enableDataOnSend = false; s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 1u; }
    if (priorityOnSend && sdkCalls == priorityOnSend) {
        assert(sensorarrayBleNotify(SENSORARRAY_BLE_CH_CTRL, (const uint8_t *)"LATECTRL\n", 9u) == ESP_OK);
    }
    if (disconnectAfter && sdkCalls == disconnectAfter) {
        s_ble.stats.connected = false;
        s_ble.connectionGeneration++;
        sensorarrayBleDrainTxQueues();
    }
    if (disableDataAfter && sdkCalls == disableDataAfter) {
        s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
    }
    return ESP_OK;
}
static esp_err_t esp_ble_gap_set_preferred_phy(esp_bd_addr_t peer, unsigned all,
    esp_ble_gap_phy_mask_t tx, esp_ble_gap_phy_mask_t rx, unsigned options)
{
    assert(!lockDepth && !memcmp(peer, s_ble.remote, 6u) && all == 0u && options == 0u && tx == rx);
    phyRequests++;
    return phyRequests == 1u ? phyReturn : fallbackReturn;
}
static esp_err_t esp_ble_gap_read_phy(esp_bd_addr_t peer)
{ assert(!lockDepth && !memcmp(peer, s_ble.remote, 6u)); readRequests++; return readReturn; }
static esp_err_t esp_ble_gap_update_conn_params(esp_ble_conn_update_params_t *params)
{
    assert(!lockDepth && !memcmp(params->bda, s_ble.remote, 6u));
    assert(params->min_int == 6u && params->max_int == 6u && params->latency == 0u && params->timeout == 400u);
    connRequests++; return connReturn;
}
static esp_err_t esp_ble_gap_set_pkt_data_len(esp_bd_addr_t peer, uint16_t length)
{ assert(!lockDepth && !memcmp(peer, s_ble.remote, 6u) && length == 251u); dleRequests++; return dleReturn; }

static void reset(void)
{
    memset(&s_ble, 0, sizeof(s_ble));
    nowUs = 1000000u; sdkDelayUs = 0u;
    idleGeneration = 1u; sdkIdleAfter = 0u; changeIdleDuringSdk = false;
    lockDepth = checkpoints = wakeGives = idleTakes = delayCalls = queueInitCount = failQueueInit = 0u;
    failQueueSends = sdkCalls = sdkFailures = disconnectAfter = priorityOnSend = disableDataAfter = 0u;
    phyRequests = readRequests = connRequests = dleRequests = notifications = 0u;
    confWakeGives = 0u; lastConfWakeTask = NULL;
    phyReturn = fallbackReturn = readReturn = connReturn = dleReturn = ESP_OK;
    wakeOnlyDuringConfirm = confirmSuccess = injectOnIdle = injectLogOnSend = enableDataOnSend = false;
    observeDuringSdk = observeDuringConfirm = false;
    resubscribeDuringSdk = false;
    typedCopyInjection = 0u;
    sentLength = sentChannelCount = 0u;
    memset(sentBytes, 0, sizeof(sentBytes)); memset(sentChannels, 0, sizeof(sentChannels));
    memset(sentLengths, 0, sizeof(sentLengths));
    payloadAllocations=0u;
    assert(sensorarrayBleInitPayloadStorage()==ESP_OK && payloadAllocations==1u);
    assert(initializeQueues() == ESP_OK);
    s_ble.guard.ready = true; s_ble.guard.initError = ESP_OK;
    s_ble.stats.connected = s_ble.stats.gattReady = true; s_ble.stats.mtu = 247u;
    s_ble.connectionGeneration = 7u; s_ble.gattsIf = 2; s_ble.connId = 3;
    s_ble.cccd[0] = s_ble.cccd[1] = s_ble.cccd[2] = 1u;
    s_ble.handles[BLE_IDX_DATA_TX_VALUE] = 20u;
    s_ble.handles[BLE_IDX_LOG_TX_VALUE] = 30u;
    s_ble.handles[BLE_IDX_CTRL_TX_VALUE] = 10u;
    s_ble.txTask = (void *)1; s_ble.controlTask = (void *)2;
    s_ble.config.preferPhy2m = true;
    memset(s_ble.remote, 0x5a, sizeof(s_ble.remote));
    sensorarrayBleResetLinkObservationLocked();
}
static esp_err_t enqueue(sensorarrayBleChannel_t channel)
{
    const char *text = channel == SENSORARRAY_BLE_CH_DATA ? "DATA\n" :
        channel == SENSORARRAY_BLE_CH_LOG ? "LOG\n" : "CTRL\n";
    return sensorarrayBleNotifyForGeneration(channel, (const uint8_t *)text, strlen(text), 7u);
}
static void runToIdle(void)
{
    if (setjmp(taskIdle) == 0) { sensorarrayBleTxTask(NULL); }
    assert(lockDepth == 0u && s_ble.stats.txSlotUsed == 0u && !s_ble.link.sdkCallActive);
}
static void testLogDataAdmissionPolicy(void)
{
    /* The effective subscription includes TX mode: indication-only DATA is
     * inactive in FAST, active in SAFE. Queue emptiness is immaterial. */
    for (unsigned mode = 0u; mode < 2u; ++mode) {
        for (unsigned cccd = 0u; cccd < 4u; ++cccd) {
            reset(); s_ble.txMode = mode ? SENSORARRAY_BLE_TX_SAFE : SENSORARRAY_BLE_TX_FAST;
            s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = (uint16_t)cccd;
            bool active = (cccd & 1u) || (mode && (cccd & 2u));
            sensorarrayBleLogPolicySnapshot_t policy;
            sensorarrayBleGetLogPolicySnapshot(&policy);
            assert(policy.connectionGeneration == 7u && policy.connected && policy.txMode == s_ble.txMode);
            assert(policy.dataCccd == cccd && policy.dataActive == active && policy.ordinaryAllowed == !active);
            assert(policy.policyDrop == 0u && policy.droppedLog == 0u);
            assert(enqueue(SENSORARRAY_BLE_CH_LOG) == (active ? ESP_ERR_NOT_ALLOWED : ESP_OK));
            assert(s_ble.stats.logPolicyDrop == active && s_ble.stats.dropped[1] == active &&
                   s_ble.stats.messageDropped == active);
            assert(s_ble.stats.txSlotUsed == !active && s_ble.logTxQueue->count == !active);
            assert(s_ble.txQueue->count == 0u && sdkCalls == 0u);
            assert(s_ble.stats.txSlotAllocFail == 0u && s_ble.stats.fragmentError == 0u &&
                   s_ble.stats.txConnectionStaleDrop == 0u && s_ble.link.sdkErrors == 0u && s_ble.guard.rejected == 0u);
            /* These use the real reserved lifecycle and control paths even
             * when the ordinary LOG call above was denied. */
            const char fault[] = "MFAULT,seq=9,err=0x108\n";
            const char config[] = "CONFIG,rev=9\n";
            assert(sensorarrayBleNotifyLifecycleForGeneration((const uint8_t *)fault, sizeof fault - 1u, 7u) == ESP_OK);
            assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_CTRL, (const uint8_t *)config,
                                                    sizeof config - 1u, 7u) == ESP_OK);
            unsigned wakes = wakeGives;
            sensorarrayBleGetLogPolicySnapshot(NULL);
            sensorarrayBleGetLogPolicySnapshot(&policy);
            sensorarrayBleGetLogPolicySnapshot(&policy);
            assert(policy.policyDrop == active && policy.droppedLog == active && wakeGives == wakes && sdkCalls == 0u);
            runToIdle();
            assert(sentChannelCount == (active ? 2u : 3u) && !memcmp(sentChannels, active ? "LC" : "LCL", sentChannelCount));
            assert(!memcmp(sentBytes, fault, sizeof fault - 1u) &&
                   !memcmp(sentBytes + sizeof fault - 1u, config, sizeof config - 1u));
            assert(s_ble.stats.sent[1] == (active ? 1u : 2u) && s_ble.stats.sent[2] == 1u);
        }
    }
    /* Other admission failures keep their existing classification. */
    for (unsigned unavailable = 0u; unavailable < 4u; ++unavailable) {
        reset();
        if (unavailable == 0u) s_ble.stats.connected = false;
        if (unavailable == 1u) s_ble.stats.gattReady = false;
        if (unavailable == 2u) s_ble.guard.ready = false;
        if (unavailable == 3u) s_ble.cccd[SENSORARRAY_BLE_CH_LOG] = 0u;
        assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_INVALID_STATE);
        assert(s_ble.stats.logPolicyDrop == 0u && s_ble.stats.txSlotUsed == 0u && sdkCalls == 0u);
        sensorarrayBleLogPolicySnapshot_t policy;
        sensorarrayBleGetLogPolicySnapshot(&policy); assert(!policy.ordinaryAllowed);
    }
    /* Unsubscribe restores LOG-only operation; a reconnect does not reset a
     * boot counter, and stale submissions are not policy rejections. */
    reset(); assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_NOT_ALLOWED);
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_LOG, (const uint8_t *)"OLD\n", 4u, 6u) == ESP_ERR_INVALID_STATE);
    assert(s_ble.stats.logPolicyDrop == 1u && s_ble.stats.txConnectionStaleDrop == 1u);
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK); runToIdle();
    assert(s_ble.stats.sent[1] == 1u && s_ble.stats.logPolicyDrop == 1u);
    esp_ble_gatts_cb_param_t event = {.connect = {.conn_id = 3u}};
    dispatchConnection(ESP_GATTS_DISCONNECT_EVT, &event);
    memset(event.connect.remote_bda, 0x5a, sizeof event.connect.remote_bda);
    dispatchConnection(ESP_GATTS_CONNECT_EVT, &event);
    s_ble.cccd[SENSORARRAY_BLE_CH_LOG] = 1u;
    sensorarrayBleLogPolicySnapshot_t policy;
    sensorarrayBleGetLogPolicySnapshot(&policy);
    assert(policy.connectionGeneration == 9u && policy.policyDrop == 1u && !policy.dataActive && policy.ordinaryAllowed);
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_LOG, (const uint8_t *)"NEW\n", 4u, 9u) == ESP_OK);
    runToIdle(); assert(s_ble.stats.sent[1] == 2u);
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 1u;
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_INVALID_STATE);
    assert(s_ble.stats.logPolicyDrop == 1u && s_ble.stats.txConnectionStaleDrop == 2u);
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_LOG, (const uint8_t *)"NEW\n", 4u, 9u) == ESP_ERR_NOT_ALLOWED);
    assert(s_ble.stats.logPolicyDrop == 2u && sdkCalls == 2u);

    /* Turning DATA on between fragments cannot cancel the owned LOG or an
     * already accepted queued LOG. Both execute the actual send path. */
    reset(); s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
    uint8_t payload[600]; memset(payload, 'L', sizeof payload); payload[599] = '\n';
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_LOG, payload, sizeof payload, 7u) == ESP_OK);
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK); enableDataOnSend = true;
    runToIdle();
    assert(sentChannelCount == 4u && !memcmp(sentChannels, "LLLL", 4u));
    assert(s_ble.stats.sent[1] == 2u && s_ble.stats.logPolicyDrop == 0u && s_ble.stats.txSlotReleaseMismatch == 0u);
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_NOT_ALLOWED && s_ble.stats.logPolicyDrop == 1u);

    reset(); s_ble.stats.logPolicyDrop = s_ble.stats.dropped[1] = s_ble.stats.messageDropped = UINT32_MAX;
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_NOT_ALLOWED);
    sensorarrayBleGetLogPolicySnapshot(&policy);
    assert(policy.policyDrop == 0u && policy.droppedLog == 0u && s_ble.stats.messageDropped == 0u && sdkCalls == 0u);
    puts("BLE_LOG_POLICY_PASS,cccd_mode_cases=8,unavailable_cases=4,epoch_and_unsubscribe=1,critical_bypass=1,accepted_owner_complete=1,count_once_wrap=1,query_readonly=1,hardware=NOT_RUN");
}
/* Synthetic byte fixtures below exercise ownership/epoch decisions, not
 * scientific value validation. The separate golden case uses encoder bytes. */
static sensorarrayBleFramePublication_t typedPublication(uint32_t seq, uint32_t revision, uint32_t flags)
{
    static uint8_t full[600];
    static const uint8_t preview[]="B20,previewOnly=1\n";
    memset(full,'A',sizeof full); full[sizeof full-1u]='\n';
    return (sensorarrayBleFramePublication_t){.fullData=full,.fullLength=sizeof full,
        .normalData=preview,.normalLength=sizeof preview-1u,
        .identity={.sessionId=42u,.bootId=7u,.revision=revision,.sequence=seq},.protectionFlags=flags};
}
static void testTypedFrameCopy(void *destination, const void *source, size_t length)
{
    assert(!lockDepth);
    unsigned injection=typedCopyInjection;typedCopyInjection=0u;
    if(injection) {
        int writing=-1;
        for(unsigned i=0u;i<SENSORARRAY_BLE_PROTECTED_SLOT_COUNT;i++)
            if(destination==s_ble.protectedSlots[i].data)writing=(int)i;
        assert(writing>=0 && s_ble.protectedPhase[writing]==BLE_FRAME_WRITING);
        uint32_t generation=s_ble.protectedSlots[writing].generation;
        if(injection==1u) {
            sensorarrayBleFramePublication_t other=typedPublication(201u,2u,0u);
            assert(sensorarrayBleNotifyFrameForGeneration(&other,7u)==ESP_OK);
            other=typedPublication(202u,3u,0u);
            assert(sensorarrayBleNotifyFrameForGeneration(&other,7u)==ESP_OK);
            assert(s_ble.frameFlow.protectedSuperseded==1u);
        } else {
            esp_ble_gatts_cb_param_t event={.connect={.conn_id=3u}};
            dispatchConnection(ESP_GATTS_DISCONNECT_EVT,&event);
            memset(event.connect.remote_bda,0x5a,sizeof event.connect.remote_bda);
            dispatchConnection(ESP_GATTS_CONNECT_EVT,&event);
        }
        assert(s_ble.protectedPhase[writing]==BLE_FRAME_WRITING &&
               s_ble.protectedSlots[writing].generation==generation && s_ble.protectedSlots[writing].inUse);
    }
    memcpy(destination,source,length);
}
static void testFrameProtection(void)
{
    reset(); s_ble.stats.mtu=517u;
    char full[sizeof referenceVoltMtu+1u]; memcpy(full,referenceVoltMtu,sizeof referenceVoltMtu); full[sizeof referenceVoltMtu]='\0';
    sensorarrayBleFramePublication_t p=typedPublication(1u,1u,0u);
    unsigned sequence=0u,boot=0u,revision=0u; unsigned long long session=0u;
    assert(sscanf(strstr(full,"seq="),"seq=%u",&sequence)==1);
    assert(sscanf(strstr(full,"bootId="),"bootId=%u",&boot)==1);
    assert(sscanf(strstr(full,"rev="),"rev=%u",&revision)==1);
    assert(sscanf(strstr(full,"session="),"session=%llX",&session)==1);
    p.fullData=(const uint8_t *)full;p.fullLength=sizeof referenceVoltMtu;
    p.identity=(sensorarrayBleFrameIdentity_t){.sessionId=session,.bootId=boot,.revision=revision,.sequence=sequence};
    wireCase="golden-typed-first-full"; journalWire("expected",'D',p.fullData,p.fullLength);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(s_ble.protectedPhase[0]==BLE_FRAME_PENDING && s_ble.protectedDescriptors[0].protectedSlot && !s_ble.protectedDescriptors[0].ctrlSlot);
    assert(s_ble.protectedDescriptors[0].length>512u && s_ble.ctrlTxQueue->count==0u && s_ble.txQueue->count==0u);
    memset(full,'Z',sizeof referenceVoltMtu); /* Caller storage no longer owned. */
    runToIdle(); wireCase=NULL;
    assert(s_ble.frameFlow.protectedComplete==1u && !s_ble.frameFlow.firstPending && sdkCalls==2u);

    reset(); s_ble.stats.mtu=517u;
    for(unsigned i=0u;i<4u;i++) assert(enqueue(SENSORARRAY_BLE_CH_DATA)==ESP_OK);
    p=typedPublication(10u,1u,0u); assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL)==ESP_OK);
    runToIdle(); assert(sentChannels[0]=='C' && sentChannels[1]=='D' && sentLengths[1]==495u);
    assert(s_ble.frameFlow.protectedComplete==1u && s_ble.stats.txSlotReleaseMismatch==0u);
    p=typedPublication(11u,1u,0u); assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(s_ble.txQueue->count==1u && s_ble.protectedPhase[0]==BLE_FRAME_FREE);
    nowUs+=1000000u; /* A new periodic obligation is due, not merely requested. */
    p=typedPublication(12u,1u,SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    unsigned before=sdkCalls; runToIdle();
    assert(sdkCalls==before+2u && s_ble.frameFlow.staleNormal==1u && s_ble.frameFlow.floor.sequence==12u);
    p=typedPublication(11u,1u,0u); assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_ERR_INVALID_STATE);
    assert(s_ble.frameFlow.staleNormal==2u);

    reset(); s_ble.stats.mtu=517u;
    p=typedPublication(10u,1u,0u); assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    uint32_t firstGeneration=s_ble.protectedSlots[0].generation;
    nowUs+=1000000u;
    p=typedPublication(11u,1u,SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    nowUs+=1000000u;
    p=typedPublication(12u,1u,SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(s_ble.protectedSlots[0].generation==firstGeneration && s_ble.protectedMeta[0].identity.sequence==10u);
    assert(s_ble.protectedMeta[1].identity.sequence==12u && s_ble.frameFlow.protectedSuperseded==1u);
    sensorarrayBleTxDescriptor_t held[2]; assert(sensorarrayBleReceiveProtected(&held[0]));
    assert(sensorarrayBleReceiveProtected(&held[1]));
    nowUs+=1000000u; p=typedPublication(13u,2u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_ERR_NO_MEM);
    assert(s_ble.protectedPhase[0]==BLE_FRAME_OWNED && s_ble.protectedPhase[1]==BLE_FRAME_OWNED);
    assert(sensorarrayBleSendDescriptor(&held[0])==ESP_OK);sensorarrayBleReleaseTxDescriptor(&held[0]);
    assert(s_ble.frameFlow.firstPending && !s_ble.frameFlow.fullCompleteKnown); /* Older config cannot acknowledge revision2. */
    assert(sensorarrayBleSendDescriptor(&held[1])==ESP_OK);sensorarrayBleReleaseTxDescriptor(&held[1]);
    p=typedPublication(14u,2u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    runToIdle(); assert(!s_ble.frameFlow.firstPending && s_ble.frameFlow.floor.revision==2u);

    reset(); s_ble.stats.mtu=517u; p=typedPublication(20u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    resubscribeDuringSdk=true;runToIdle();
    assert(sdkCalls==1u && s_ble.frameFlow.firstPending && s_ble.frameFlow.protectedComplete==0u && !s_ble.frameFlow.fullCompleteKnown);
    p=typedPublication(21u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    runToIdle();assert(sdkCalls==3u && s_ble.frameFlow.protectedComplete==1u && !s_ble.frameFlow.firstPending);

    reset(); s_ble.stats.mtu=517u; sdkFailures=1u;
    p=typedPublication(30u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    runToIdle();assert(sdkCalls==1u && s_ble.stats.controlTxRetry==0u && s_ble.frameFlow.firstPending);
    p=typedPublication(31u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    runToIdle();assert(s_ble.frameFlow.protectedComplete==1u && !s_ble.frameFlow.firstPending);

    reset(); s_ble.stats.mtu=517u; observeDuringSdk=true;
    p=typedPublication(40u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    runToIdle();assert(sdkCalls==1u && s_ble.guard.quarantined && s_ble.frameFlow.protectedComplete==0u);
    p=typedPublication(41u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_ERR_INVALID_STATE);
    assert(sdkCalls==1u && s_ble.stats.txSlotReleaseMismatch==0u);

    reset();s_ble.stats.mtu=517u;
    uint8_t original[600];memset(original,'O',sizeof original);original[599]='\n';
    p=typedPublication(200u,1u,0u);p.fullData=original;typedCopyInjection=1u;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(s_ble.protectedMeta[0].identity.sequence==200u && s_ble.protectedSlots[0].data[0]=='O');
    assert(s_ble.protectedMeta[1].identity.sequence==202u);runToIdle();
    assert(!s_ble.frameFlow.firstPending && s_ble.frameFlow.floor.revision==3u);
    reset();s_ble.stats.mtu=517u;p=typedPublication(200u,1u,0u);typedCopyInjection=2u;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_ERR_INVALID_STATE);
    assert(s_ble.stats.txSlotUsed==0u && s_ble.stats.txSlotReleaseMismatch==0u && sdkCalls==0u);

    reset();s_ble.stats.mtu=517u;p=typedPublication(1u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    unsigned skips=0u;
    for(unsigned i=2u;i<420u;i++) {
        nowUs+=10000u;p=typedPublication(i,1u,0u);
        uint32_t drops=s_ble.stats.dropped[0],allocFailures=s_ble.stats.txSlotAllocFail;
        esp_err_t result=sensorarrayBleNotifyFrameForGeneration(&p,7u);
        if(result==ESP_ERR_NOT_ALLOWED) {
            skips++;assert(s_ble.stats.dropped[0]==drops && s_ble.stats.txSlotAllocFail==allocFailures);
        } else assert(result==ESP_OK || result==ESP_ERR_NO_MEM);
    }
    sensorarrayBleFlowSnapshot_t snapshot;
    sensorarrayBleGetFlowSnapshot(&snapshot);
    assert(snapshot.ordinaryRatePermille==950u && skips>0u && snapshot.intentionalSkip==skips);
    assert(snapshot.capacityRejected>0u && sdkCalls==2u && delayCalls==0u);
    uint32_t generations=snapshot.streamGeneration;
    sensorarrayBleGetFlowSnapshot(NULL);sensorarrayBleGetFlowSnapshot(&snapshot);
    assert(snapshot.streamGeneration==generations && snapshot.intentionalSkip==skips);
    runToIdle();assert(s_ble.stats.txSlotUsed==0u && s_ble.stats.txSlotReleaseMismatch==0u);

    reset();s_ble.stats.mtu=517u;s_ble.cccd[SENSORARRAY_BLE_CH_DATA]=2u;confirmSuccess=true;
    p=typedPublication(1u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_ERR_INVALID_STATE);
    sensorarrayBleSetTxMode(SENSORARRAY_BLE_TX_SAFE);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    sensorarrayBleGetFlowSnapshot(&snapshot);generations=snapshot.streamGeneration;
    assert(snapshot.lastCompleteKnown && snapshot.lastCompleteAgeKnown && !snapshot.firstPending && !snapshot.gapKnown);
    sensorarrayBleSetTxMode(SENSORARRAY_BLE_TX_FAST);sensorarrayBleSetTxMode(SENSORARRAY_BLE_TX_SAFE);
    p=typedPublication(2u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    sensorarrayBleGetFlowSnapshot(&snapshot);assert(snapshot.streamGeneration!=generations && snapshot.protectedComplete==2u);

    reset();s_ble.stats.mtu=517u;p=typedPublication(UINT32_MAX,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    nowUs+=1200000u;p=typedPublication(0u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    sensorarrayBleGetFlowSnapshot(&snapshot);
    assert(snapshot.lastSequence==0u && snapshot.gapKnown && snapshot.maxCompleteGapUs==1200000u && snapshot.lastCompleteAgeUs==0u);
    reset();s_ble.stats.mtu=517u;p=typedPublication(1000u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    p=typedPublication(1u,1u,0u);p.identity.bootId++;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.floor.bootId==8u && s_ble.frameFlow.floor.sequence==1u && s_ble.frameFlow.maxCompleteGapUs==0u);
    puts("BLE_FRAME_PROTECTION_PASS,groups=12,full_encoder_roundtrip=1,deep_copy=1,writing_races=2,independent_slots=2,owner_retained=1,stream_epoch=1,old_config_no_ack=1,ctrl_retry_unchanged=1,capacity_policy_integrated=1,fast_safe=1,wrap_boot=1,hardware=NOT_RUN");
}
static void testPeriodicCadence(void)
{
    /* Real typed admission / descriptor owner / whole SDK completion. The
     * source protects at 0.5, 1.5, ... seconds; the local fallback begins at
     * 1.0 seconds. Two-frame batches expose priority overtaking explicitly.
     * This deterministic local boundary is not an RF loss measurement. */
    bool expected=true;
    const unsigned phases[]={2u,10u,18u,UINT32_MAX};
    for (unsigned phase=0u;phase<sizeof phases/sizeof phases[0];phase++)
    for (unsigned preview=0u;preview<2u;preview++) {
        reset();s_ble.stats.mtu=517u;
        uint64_t start=nowUs;
        for (unsigned i=0u;i<200u;i++) {
            nowUs=start+(uint64_t)i*50000u;
            sensorarrayBleFramePublication_t p=typedPublication(i+1u,1u,
                i%20u==phases[phase] ? SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC : 0u);
            if (!preview) { p.normalData=p.fullData;p.normalLength=p.fullLength; }
            assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
            if (i%2u==0u || i==199u) {
                runToIdle();sentLength=sentChannelCount=0u;
            }
        }
        printf("BLE_PERIODIC_PHASE_OBSERVED,preview=%u,phase_tick=%u,offered=200,source_periodic=%u,protected=%u,whole=%u,stale=%u,hardware=NOT_RUN\n",
            preview,phases[phase],phase==3u ? 0u : 10u,s_ble.frameFlow.protectedComplete,s_ble.frameFlow.counters.wholeCompleted,s_ble.frameFlow.staleNormal);
        fflush(stdout);
        expected &= s_ble.frameFlow.protectedComplete==(preview ? 10u : 1u);
        expected &= s_ble.frameFlow.staleNormal==(preview ? 9u : 0u);
        expected &= s_ble.frameFlow.counters.wholeCompleted==(preview ? 10u : 200u);
        assert(!s_ble.frameFlow.firstPending && !s_ble.frameFlow.periodicPending && !s_ble.guard.quarantined);
    }
    assert(expected);
    puts("BLE_PERIODIC_PHASE_PASS,cases=8,source_lost_fallback=1,all_full_satisfies_periodic=1,preview_does_not=1,hardware=NOT_RUN");
}
static void testFullCompletionAnchor(void)
{
    reset();s_ble.stats.mtu=517u;sdkDelayUs=3700u;
    sensorarrayBleFramePublication_t p=typedPublication(1u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.fullCompleteKnown && s_ble.frameFlow.lastFullCompleteUs==1007400u);
    assert(s_ble.frameFlow.periodicDueUs==2007400u && s_ble.frameFlow.lastFullSequence==1u);
    /* Completion behind the newest offered frame still advances the anchor.
     * Waiting for equality with the latest offer would starve under load. */
    nowUs+=100000u;
    p=typedPublication(2u,1u,0u);p.normalData=p.fullData;p.normalLength=p.fullLength;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    sensorarrayBleTxDescriptor_t held;
    assert(sensorarrayBleReceiveNext(&held) && !held.protectedSlot);
    p.identity.sequence=3u;assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(sensorarrayBleSendDescriptor(&held)==ESP_OK);
    assert(s_ble.frameFlow.identity.sequence==3u && s_ble.frameFlow.lastFullSequence==2u);
    sensorarrayBleReleaseTxDescriptor(&held);runToIdle();
    uint64_t completed=s_ble.frameFlow.lastFullCompleteUs, due=s_ble.frameFlow.periodicDueUs;
    assert(s_ble.frameFlow.lastFullSequence==3u && due==completed+1000000u);
    nowUs+=100000u;p.identity.sequence=3u;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    p.identity.sequence=2u;assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.lastFullCompleteUs==completed && s_ble.frameFlow.periodicDueUs==due);
    /* Legacy raw success and previews provide no typed complete-bundle proof. */
    assert(enqueue(SENSORARRAY_BLE_CH_DATA)==ESP_OK);runToIdle();
    p=typedPublication(4u,1u,SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.periodicDueUs==due && s_ble.frameFlow.protectedComplete==1u);
    nowUs=due;p=typedPublication(5u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.protectedComplete==2u && s_ble.frameFlow.lastFullSequence==5u);
    assert(s_ble.frameFlow.periodicDueUs==nowUs+1000000u);

    /* Failed ordinary full data cannot postpone fallback. A failed protected
     * full send retains the same obligation for the next available frame. */
    reset();s_ble.stats.mtu=517u;p=typedPublication(1u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    due=s_ble.frameFlow.periodicDueUs;completed=s_ble.frameFlow.lastFullCompleteUs;
    nowUs+=500000u;sdkFailures=sdkCalls+1u;
    p=typedPublication(2u,1u,0u);p.normalData=p.fullData;p.normalLength=p.fullLength;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.lastFullCompleteUs==completed && s_ble.frameFlow.periodicDueUs==due);
    nowUs=due;sdkFailures=sdkCalls+1u;p=typedPublication(3u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.periodicPending && s_ble.frameFlow.lastFullCompleteUs==completed);
    assert(s_ble.frameFlow.protectedFailed==1u);
    p.identity.sequence=4u;assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(!s_ble.frameFlow.periodicPending && s_ble.frameFlow.protectedComplete==2u);

    /* Completing an older in-flight ordinary full frame can satisfy due, but
     * must neither cancel nor reuse an already accepted protected descriptor. */
    reset();s_ble.stats.mtu=517u;p=typedPublication(1u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    nowUs+=500000u;p=typedPublication(2u,1u,0u);p.normalData=p.fullData;p.normalLength=p.fullLength;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(sensorarrayBleReceiveNext(&held) && !held.protectedSlot);
    nowUs=s_ble.frameFlow.periodicDueUs;p=typedPublication(3u,1u,SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    uint32_t generation=s_ble.protectedSlots[0].generation;
    assert(s_ble.protectedPhase[0]==BLE_FRAME_PENDING && s_ble.frameFlow.periodicPending);
    assert(sensorarrayBleSendDescriptor(&held)==ESP_OK);sensorarrayBleReleaseTxDescriptor(&held);
    assert(!s_ble.frameFlow.periodicPending && s_ble.protectedPhase[0]==BLE_FRAME_PENDING &&
           s_ble.protectedSlots[0].generation==generation && s_ble.protectedSlots[0].inUse);
    runToIdle();assert(s_ble.frameFlow.protectedComplete==2u && s_ble.frameFlow.lastFullSequence==3u);

    /* An ordinary owner from the old configuration may return after the new
     * FIRST was accepted. Its successful bytes cannot move the new anchor. */
    reset();s_ble.stats.mtu=517u;p=typedPublication(1u,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    p=typedPublication(2u,1u,0u);p.normalData=p.fullData;p.normalLength=p.fullLength;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(sensorarrayBleReceiveNext(&held) && !held.protectedSlot);
    p=typedPublication(3u,2u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    due=s_ble.frameFlow.periodicDueUs;nowUs+=200000u;
    assert(sensorarrayBleSendDescriptor(&held)==ESP_OK);sensorarrayBleReleaseTxDescriptor(&held);
    assert(!s_ble.frameFlow.fullCompleteKnown && s_ble.frameFlow.firstPending && s_ble.frameFlow.periodicDueUs==due);
    runToIdle();assert(s_ble.frameFlow.fullCompleteKnown && s_ble.frameFlow.lastFullSequence==3u);

    /* Same config new stream and new config each retain independent FIRST;
     * recent full evidence cannot swallow either transition. */
    uint32_t stream=s_ble.frameFlow.streamGeneration;
    portENTER_CRITICAL(&s_bleMux);
    s_ble.cccd[0]=0u;sensorarrayBleFrameFlowSyncLocked();
    s_ble.cccd[0]=1u;sensorarrayBleFrameFlowSyncLocked();
    portEXIT_CRITICAL(&s_bleMux);
    assert(!s_ble.frameFlow.fullCompleteKnown && s_ble.frameFlow.streamGeneration!=stream);
    p.identity.sequence=4u;assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(s_ble.protectedMeta[0].ticket.protectionFlags&SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG);
    runToIdle();p.identity.sequence=5u;p.identity.revision++;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
    assert(!s_ble.frameFlow.fullCompleteKnown && s_ble.frameFlow.firstPending);
    runToIdle();assert(s_ble.frameFlow.fullCompleteKnown && !s_ble.frameFlow.firstPending);

    /* Serial sequence wrap is progress; timestamp regression or exactly half
     * range ambiguity cannot refresh the completion evidence. */
    reset();s_ble.stats.mtu=517u;p=typedPublication(UINT32_MAX,1u,0u);
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    nowUs+=100000u;p.identity.sequence=0u;p.normalData=p.fullData;p.normalLength=p.fullLength;
    assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.lastFullSequence==0u && s_ble.frameFlow.protectedComplete==1u);
    due=s_ble.frameFlow.periodicDueUs;completed=s_ble.frameFlow.lastFullCompleteUs;
    portENTER_CRITICAL(&s_bleMux);
    p.identity.sequence=1u;sensorarrayBleFrameFullCompletedLocked(&p.identity,completed-1u);
    p.identity.sequence=UINT32_C(0x80000000);sensorarrayBleFrameFullCompletedLocked(&p.identity,completed+1u);
    portEXIT_CRITICAL(&s_bleMux);
    assert(s_ble.frameFlow.periodicDueUs==due && s_ble.frameFlow.lastFullSequence==0u);
    /* Paused/idle time emits nothing; one real subsequent frame can recover.
     * A query does not turn elapsed time into fabricated scientific samples. */
    nowUs=completed+5000000u;sensorarrayBleFlowSnapshot_t snapshot;
    sensorarrayBleGetFlowSnapshot(&snapshot);
    assert(snapshot.protectedComplete==1u && !snapshot.periodicPending);
    p=typedPublication(1u,1u,0u);assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);runToIdle();
    assert(s_ble.frameFlow.protectedComplete==2u && s_ble.frameFlow.protectedDeadlineMissed==4u);
    portENTER_CRITICAL(&s_bleMux);
    p.identity.sequence=2u;sensorarrayBleFrameFullCompletedLocked(&p.identity,UINT64_MAX-500000u);
    portEXIT_CRITICAL(&s_bleMux);
    assert(s_ble.frameFlow.periodicDueUs==UINT64_MAX);
    puts("BLE_FULL_COMPLETION_ANCHOR_PASS,groups=7,local_completion_clock=1,old_duplicate_no_refresh=1,failed_no_refresh=1,pending_owner_retained=1,old_config_owner_no_refresh=1,stream_config_first=1,wrap_time_idle=1,hardware=NOT_RUN");
}
static void testPeriodicGoldenFull(void)
{
    for (unsigned flagged=0u;flagged<2u;flagged++) {
        reset();s_ble.stats.mtu=517u;
        sensorarrayBleFramePublication_t p=typedPublication(1u,1u,0u);
        for (unsigned after=0u;after<2u;after++) {
            p.fullData=after ? referenceCapAfter : referenceCapMtu;
            p.fullLength=after ? sizeof referenceCapAfter : sizeof referenceCapMtu;
            char text[SENSORARRAY_BLE_MESSAGE_VALUE_MAX+1u];
            assert(p.fullLength<sizeof text);memcpy(text,p.fullData,p.fullLength);text[p.fullLength]='\0';
            unsigned sequence,boot,revision;unsigned long long session;
            assert(sscanf(strstr(text,"seq="),"seq=%u",&sequence)==1);
            assert(sscanf(strstr(text,"bootId="),"bootId=%u",&boot)==1);
            assert(sscanf(strstr(text,"rev="),"rev=%u",&revision)==1);
            assert(sscanf(strstr(text,"session="),"session=%llX",&session)==1);
            p.identity=(sensorarrayBleFrameIdentity_t){.sessionId=session,.bootId=boot,.revision=revision,.sequence=sequence};
            if (after) {
                nowUs=s_ble.frameFlow.periodicDueUs;
                p.protectionFlags=flagged ? SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC : 0u;
                wireCase=flagged ? "golden-cap-periodic-short" : "golden-cap-periodic-fallback";
                journalWire("expected",'D',p.fullData,p.fullLength);
            }
            assert(sensorarrayBleNotifyFrameForGeneration(&p,7u)==ESP_OK);
            assert(s_ble.protectedPhase[0]==BLE_FRAME_PENDING && s_ble.protectedDescriptors[0].length==p.fullLength);
            assert(!memcmp(s_ble.protectedSlots[0].data,p.fullData,p.fullLength));
            runToIdle();wireCase=NULL;
        }
        assert(s_ble.frameFlow.protectedComplete==2u && s_ble.frameFlow.counters.wholeCompleted==2u);
    }
    puts("BLE_PERIODIC_GOLDEN_PASS,cases=2,original_encoder_bytes=1,short_full_and_missing_candidate_fallback=1,hardware=NOT_RUN");
}
static void testAdmissionAndPriority(void)
{
    reset();
    assert(CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN == 4u && SENSORARRAY_BLE_LOG_TX_SLOT_COUNT == 2u && SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT == 2u);
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK && enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_NO_MEM);
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 1u;
    for (unsigned i=0u; i<4u; i++) { assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); }
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_ERR_NO_MEM);
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK && enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK);
    assert(s_ble.stats.txSlotUsed == 8u);
    nowUs += 12345u; runToIdle();
    assert(sentChannelCount == 8u && !memcmp(sentChannels, "CCDDDDLL", 8u));
    assert(s_ble.stats.txSlotAllocFail == 2u && s_ble.stats.messageDropped == 2u);
    assert(s_ble.link.queueMaxAgeUs[0] == 12345u && s_ble.link.dataDequeued == 4u);
    assert(s_ble.link.logDequeued == 2u && s_ble.link.priorityDequeued == 2u);
    assert(checkpoints == 8u && wakeGives == 8u);
}
static void testDataBeforeBestEffortLog(void)
{
    reset();
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 1u;
    for (unsigned i=0u; i<4u; i++) { assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); }
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_NOT_ALLOWED);
    assert(s_ble.stats.dropped[SENSORARRAY_BLE_CH_LOG] == 1u && s_ble.stats.logPolicyDrop == 1u);
    for (unsigned i=0u; i<100u; i++) {
        sensorarrayBleTxDescriptor_t desc;
        if(i==20u || i==60u) {
            if(i==20u) assert(enqueue(SENSORARRAY_BLE_CH_CTRL)==ESP_OK);
            else assert(sensorarrayBleNotifyLifecycleForGeneration((const uint8_t *)"MAPP\n",5u,7u)==ESP_OK);
            assert(sensorarrayBleReceiveNext(&desc) && desc.ctrlSlot);
            assert(sensorarrayBleSendDescriptor(&desc)==ESP_OK);
            sensorarrayBleReleaseTxDescriptor(&desc);
        }
        assert(sensorarrayBleReceiveNext(&desc));
        assert(desc.channel == SENSORARRAY_BLE_CH_DATA && !desc.ctrlSlot);
        assert(sensorarrayBleSendDescriptor(&desc) == ESP_OK);
        sensorarrayBleReleaseTxDescriptor(&desc);
        assert(enqueue(desc.channel) == ESP_OK);
    }
    assert(s_ble.link.dataDequeued==100u && s_ble.link.logDequeued==0u &&
           s_ble.link.priorityDequeued==2u);
    runToIdle();
    assert(s_ble.link.dataDequeued==104u && s_ble.link.logDequeued==2u);
    assert(s_ble.stats.txSlotUsed==0u && s_ble.stats.txSlotReleaseMismatch==0u);
}
static void testLifecycleAndWake(void)
{
    reset(); s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK && enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 1u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK);
    assert(sensorarrayBleNotifyLifecycleForGeneration((const uint8_t *)"MAPP\n", 5u, 7u) == ESP_OK);
    runToIdle(); assert(!memcmp(sentBytes, "MAPP\nDATA\nLOG\nLOG\n", sentLength));
    assert(s_ble.link.priorityDequeued == 1u && s_ble.link.logDequeued == 2u);
    reset(); injectOnIdle = true; runToIdle();
    assert(sentChannelCount == 1u && sentChannels[0] == 'C' && nowUs == 1000000u);
    assert(notifications == 0u); /* Work wakeup never consumes confirmation slot. */
}
static void testPartialOwnership(void)
{
    uint8_t payload[600]; memset(payload, 'A', sizeof(payload)); payload[599] = '\n';
    reset(); priorityOnSend = 1u; injectLogOnSend = true;
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, payload, sizeof(payload), 7u) == ESP_OK);
    runToIdle(); assert(sentChannelCount == 4u && !memcmp(sentChannels, "DDDC", 4u));
    assert(s_ble.stats.sent[0] == 1u && s_ble.stats.sent[1] == 0u && s_ble.stats.sent[2] == 1u);
    assert(s_ble.stats.logPolicyDrop == 1u && s_ble.stats.dropped[1] == 1u);
    reset(); disconnectAfter = 1u;
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, payload, sizeof(payload), 7u) == ESP_OK);
    runToIdle(); assert(sdkCalls == 1u && s_ble.stats.sent[0] == 0u && s_ble.stats.dropped[0] == 1u);
    assert(s_ble.stats.txConnectionStaleDrop == 1u && s_ble.stats.txSlotReleaseMismatch == 0u);
    reset(); assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); s_ble.connectionGeneration++;
    runToIdle(); assert(sdkCalls == 0u && s_ble.stats.txSlotStaleGenerationDrop == 1u);
}
static void testFailuresAndConfirm(void)
{
    reset(); failQueueSends = 2u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_ERR_TIMEOUT && s_ble.stats.txSlotUsed == 0u);
    reset(); assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK);
    s_ble.dataSlots[0].data[0] ^= 1u;
    runToIdle(); assert(sdkCalls == 0u && s_ble.stats.txCrcMismatch == 1u);
    reset(); sdkFailures = 100u; sdkDelayUs = 300000u;
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK); runToIdle();
    assert(sdkCalls == 2u && delayCalls == 1u && nowUs == 1610000u);
    assert(s_ble.link.sdkErrors == 2u && s_ble.link.sdkMaxCallUs == 300000u);
    assert(s_ble.stats.controlTxRetryExhausted == 1u); /* SDK overrun observed, not cancellation. */
    reset(); s_ble.txMode = SENSORARRAY_BLE_TX_SAFE; s_ble.cccd[2] = 2u;
    wakeOnlyDuringConfirm = true;
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK); runToIdle();
    assert(sdkCalls == 1u && s_ble.stats.sent[2] == 0u && s_ble.stats.dropped[2] == 1u);
    assert(nowUs == 2000000u && s_ble.confirmWaitTask == NULL);
    reset(); s_ble.txMode = SENSORARRAY_BLE_TX_SAFE; s_ble.cccd[2] = 2u; confirmSuccess = true;
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK); runToIdle();
    assert(sdkCalls == 1u && s_ble.stats.sent[2] == 1u);
    reset();
    for (unsigned i=1u; i<=4u; i++) {
        queueInitCount = 0u; failQueueInit = i;
        assert(initializeQueues() == ESP_ERR_NO_MEM);
    }
}
static void testNegotiation(void)
{
    reset(); sensorarrayBleRequestLinkParameters();
    assert(phyRequests == 1u && readRequests == 1u && connRequests == 1u && dleRequests == 1u);
    assert(s_ble.link.phyRequestError == ESP_OK && s_ble.link.connRequestError == ESP_OK && s_ble.link.dataLengthRequestError == ESP_OK);
    assert(!s_ble.link.phyKnown && !s_ble.link.connParamsKnown && !s_ble.link.dataLengthObserved);
    esp_ble_gap_cb_param_t event = {0};
    memset(event.read_phy.bda, 0x5a, 6u); event.read_phy.tx_phy = event.read_phy.rx_phy = 1u;
    assert(sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_READ_PHY_COMPLETE_EVT, &event));
    assert(s_ble.link.phyKnown && s_ble.stats.txPhy == 1u);
    event.phy_update.tx_phy = event.phy_update.rx_phy = 2u;
    assert(sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT, &event));
    assert(s_ble.stats.txPhy == 2u && s_ble.stats.rxPhy == 2u && s_ble.link.phyUpdateEvents == 1u);
    event.phy_update.bda[0] = 0u; event.phy_update.tx_phy = 3u;
    assert(sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT, &event));
    assert(s_ble.stats.txPhy == 2u && s_ble.link.staleLinkEvents == 1u);
    memset(&event, 0, sizeof(event)); memset(event.update_conn_params.bda, 0x5a, 6u);
    event.update_conn_params.conn_int = 12u; event.update_conn_params.timeout = 400u;
    assert(sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT, &event));
    assert(s_ble.link.connParamsKnown && s_ble.link.connIntervalUnits == 12u && s_ble.link.supervisionTimeoutUnits == 400u);
    /* Requesting 6/6 never stamps that value over the central's actual 12. */
    event.update_conn_params.status = 0x28; event.update_conn_params.conn_int = 6u;
    sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT, &event);
    assert(s_ble.link.connUpdateStatus == 0x28u && s_ble.link.connIntervalUnits == 12u);
    event.update_conn_params.status = ESP_BT_STATUS_SUCCESS;
    sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT, &event);
    assert(s_ble.link.connParamsKnown && s_ble.link.connIntervalUnits == 6u);
    memset(&event, 0, sizeof(event)); event.pkt_data_length_cmpl.params.tx_len = 251u;
    event.pkt_data_length_cmpl.params.rx_len = 120u;
    assert(sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT, &event));
    assert(s_ble.link.dataLengthObserved && s_ble.link.dataLengthTx == 251u && !s_ble.link.dataLengthConnectionKnown);
    sensorarrayBleLinkSnapshot_t snapshot; sensorarrayBleGetLinkSnapshot(&snapshot);
    assert(snapshot.connected && snapshot.connectionGeneration == 7u && snapshot.mtu == 247u);
    assert(snapshot.txTaskPriority == 7u && snapshot.ctrlTaskPriority == 8u);
    s_ble.stats.connected = false; sensorarrayBleResetLinkObservationLocked();
    assert(sensorarrayBleObserveLinkEvent(ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT, &event));
    assert(!s_ble.link.dataLengthObserved && !s_ble.link.phyKnown && s_ble.stats.txPhy == 0u);
    assert(!sensorarrayBleObserveLinkEvent(TEST_OTHER_EVENT, &event));
    reset(); phyReturn = ESP_FAIL; fallbackReturn = ESP_ERR_NOT_SUPPORTED;
    readReturn = ESP_ERR_TIMEOUT; connReturn = ESP_ERR_NO_MEM; dleReturn = ESP_FAIL;
    sensorarrayBleRequestLinkParameters();
    assert(phyRequests == 2u && s_ble.link.phyRequestError == ESP_FAIL && s_ble.link.phyFallbackRequestError == ESP_ERR_NOT_SUPPORTED);
    assert(s_ble.link.phyReadRequestError == ESP_ERR_TIMEOUT && s_ble.link.connRequestError == ESP_ERR_NO_MEM && s_ble.link.dataLengthRequestError == ESP_FAIL);
    assert(!s_ble.link.phyKnown && s_ble.link.phyUpdateStatus == UINT16_MAX);
}
static void testSdkObservation(void)
{
    reset(); sdkDelayUs = 2500u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
    assert(s_ble.link.sdkCompletedCalls == 1u && s_ble.link.sdkWallUs == 2500u);
    assert(s_ble.link.sdkIdleUnchanged == 1u && s_ble.link.sdkIdleChanged == 0u && s_ble.link.sdkIdleUnknown == 0u);
    /* A long synthetic API interval includes no proof of which task ran. */
    sdkDelayUs = 20001u; changeIdleDuringSdk = true; sdkIdleAfter = 2u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
    assert(s_ble.link.sdkCompletedCalls == 2u && s_ble.link.sdkWallUs == 22501u);
    assert(s_ble.link.sdkIdleChanged == 1u && s_ble.link.sdkIdleUnchanged == 1u);
    idleGeneration = 0u; changeIdleDuringSdk = false; sdkDelayUs = 0u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
    assert(s_ble.link.sdkIdleUnknown == 1u && s_ble.link.sdkWallUs == 22501u);
    changeIdleDuringSdk = true; sdkIdleAfter = 7u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
    assert(s_ble.link.sdkIdleUnknown == 2u); /* Unknown-before/known-after remains unknown. */
    idleGeneration = UINT32_MAX; sdkIdleAfter = 1u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
    assert(s_ble.link.sdkIdleChanged == 2u); /* Epoch wrap is a real observed change. */
    sensorarrayBleLinkSnapshot_t snapshot; sensorarrayBleGetLinkSnapshot(&snapshot);
    assert(snapshot.sdkCompletedCalls == 5u && !snapshot.sdkCallActive);
    assert(snapshot.sdkIdleChanged + snapshot.sdkIdleUnchanged + snapshot.sdkIdleUnknown == snapshot.sdkCompletedCalls);
    sensorarrayBleResetLinkObservationLocked();
    assert(s_ble.link.sdkCompletedCalls == 5u && s_ble.link.sdkWallUs == 22501u); /* Boot totals. */

    reset(); sdkDelayUs = 300000u; sdkFailures = 100u;
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK); runToIdle();
    assert(s_ble.link.sdkCompletedCalls == 2u && s_ble.link.sdkWallUs == 600000u);
    assert(s_ble.link.sdkIdleUnchanged == 2u && s_ble.link.sdkErrors == 2u);
    assert(nowUs == 1610000u); /* Retry delay is outside the summed SDK intervals. */

    reset(); sdkDelayUs = 10u;
    s_ble.link.sdkWallUs = UINT64_MAX - 5u;
    s_ble.link.sdkCallCount = s_ble.link.sdkCompletedCalls = UINT32_MAX;
    s_ble.link.sdkIdleUnchanged = UINT32_MAX;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
    assert(s_ble.link.sdkWallUs == 4u && s_ble.link.sdkCallCount == 0u && s_ble.link.sdkCompletedCalls == 0u);
    assert(s_ble.link.sdkIdleUnchanged == 0u);
}
static void assertConfCounts(uint32_t ok, uint32_t error,
                             uint32_t matched, uint32_t unmatched)
{
    sensorarrayBleLinkSnapshot_t snapshot;
    sensorarrayBleGetLinkSnapshot(&snapshot);
    assert(snapshot.confOk == ok && snapshot.confError == error);
    assert(snapshot.confMatched == matched && snapshot.confUnmatched == unmatched);
    assert((uint32_t)(snapshot.confOk + snapshot.confError) ==
           (uint32_t)(snapshot.confMatched + snapshot.confUnmatched));
    assert(!lockDepth && s_ble.stats.fragmentError == 19u);
}
static void testConfObservation(void)
{
    esp_ble_gatts_cb_param_t event = {.conf = {.conn_id=3u, .handle=10u, .status=ESP_GATT_OK}};
    sensorarrayBleSendContext_t context = {.connectionGeneration=7u, .connId=3u};

    reset(); s_ble.stats.fragmentError = 19u;
    dispatchConf(&event); assertConfCounts(1u, 0u, 0u, 1u);
    event.conf.status = ESP_GATT_ERROR;
    dispatchConf(&event); assertConfCounts(1u, 1u, 0u, 2u);
    assert(confWakeGives == 0u && s_ble.stats.staleConfirmation == 0u);

    for (unsigned status=0u; status<2u; status++) {
        reset(); s_ble.stats.fragmentError = 19u;
        event.conf.status = status == 0u ? ESP_GATT_OK : ESP_GATT_ERROR;
        sensorarrayBlePrepareConfirmation(&context, 10u);
        dispatchConf(&event);
        assertConfCounts(status == 0u, status != 0u, 1u, 0u);
        assert(!s_ble.confirmPending && s_ble.confirmStatus == event.conf.status);
        assert(confWakeGives == 1u && lastConfWakeTask == (void *)1);
        assert(s_ble.stats.staleConfirmation == 0u);
        assert(sensorarrayBleAwaitConfirmation(&context, 10u) ==
               (status == 0u ? ESP_OK : ESP_FAIL));
        assert(s_ble.confirmWaitTask == NULL && nowUs == 1000000u);
        /* A duplicate/late callback remains observable without waking a
         * future waiter or calling it a FAST notification/peer receipt. */
        dispatchConf(&event);
        assertConfCounts(status == 0u ? 2u : 0u, status != 0u ? 2u : 0u, 1u, 1u);
        assert(confWakeGives == 1u && s_ble.stats.staleConfirmation == 0u);
    }

    for (unsigned mismatch=0u; mismatch<5u; mismatch++) {
        reset(); s_ble.stats.fragmentError = 19u;
        event.conf.status = ESP_GATT_ERROR; event.conf.conn_id = 3u; event.conf.handle = 10u;
        sensorarrayBlePrepareConfirmation(&context, 10u);
        if (mismatch == 0u) { s_ble.connectionGeneration++; }
        if (mismatch == 1u) { event.conf.conn_id++; }
        if (mismatch == 2u) { event.conf.handle++; }
        if (mismatch == 3u) { s_ble.confirmWaitTask = NULL; }
        if (mismatch == 4u) { s_ble.confirmPending = false; }
        bool pendingBefore = s_ble.confirmPending;
        dispatchConf(&event); assertConfCounts(0u, 1u, 0u, 1u);
        assert(confWakeGives == 0u && s_ble.confirmPending == pendingBefore);
        assert(s_ble.stats.staleConfirmation == (mismatch < 3u ? 1u : 0u));
    }

    reset(); s_ble.stats.fragmentError = 19u;
    event.conf.conn_id = 3u; event.conf.handle = 10u; event.conf.status = ESP_GATT_OK;
    dispatchConf(&event);
    s_ble.stats.connected = false;
    sensorarrayBleResetLinkObservationLocked();
    s_ble.connectionGeneration++;
    s_ble.stats.connected = true;
    sensorarrayBleResetLinkObservationLocked();
    event.conf.status = ESP_GATT_ERROR; dispatchConf(&event);
    assertConfCounts(1u, 1u, 0u, 2u); /* Boot totals survive link observation reset. */
    sensorarrayBleLinkSnapshot_t snapshot; sensorarrayBleGetLinkSnapshot(&snapshot);
    assert(snapshot.connectionGeneration == 8u); /* Query gen is current, not event epoch. */

    reset(); s_ble.stats.fragmentError = 19u;
    s_ble.link.confOk = s_ble.link.confError = UINT32_MAX;
    s_ble.link.confMatched = s_ble.link.confUnmatched = UINT32_MAX;
    sensorarrayBlePrepareConfirmation(&context, 10u);
    event.conf.status = ESP_GATT_OK; dispatchConf(&event);
    assertConfCounts(0u, UINT32_MAX, 0u, UINT32_MAX);
    event.conf.status = ESP_GATT_ERROR; dispatchConf(&event);
    assertConfCounts(0u, 0u, 0u, 0u);
    assert(confWakeGives == 1u && s_ble.stats.staleConfirmation == 0u);
}
static void testMtuPayloadBoundaries(void)
{
    uint8_t payload[2304]; memset(payload, 'A', sizeof payload);
    const size_t lengths[] = {511u, 512u, 513u, 598u, 2304u};
    const char channels[] = "DLC";
    for (unsigned channel = 0; channel < 3; ++channel) {
        for (size_t i = 0; i < sizeof lengths / sizeof *lengths; ++i) {
            reset(); s_ble.stats.mtu = 517u;
            if (channel == SENSORARRAY_BLE_CH_LOG) s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
            size_t length = lengths[i]; payload[length - 1u] = '\n';
            char label[48]; snprintf(label, sizeof label, "mtu517-%c-%u", channels[channel], (unsigned)length);
            wireCase = label;
            esp_err_t result = sensorarrayBleNotifyForGeneration(channel, payload, length, 7u);
            if (channel == SENSORARRAY_BLE_CH_CTRL && length > 512u) {
                assert(result == ESP_ERR_INVALID_SIZE && sdkCalls == 0u && s_ble.stats.txSlotUsed == 0u);
            } else {
                assert(result == ESP_OK);
                journalWire("expected", channels[channel], payload, length);
                runToIdle();
                unsigned expected = length <= 512u ? 1u : length <= 598u ? 2u : 5u;
                assert(sdkCalls == expected && sentChannelCount == expected && s_ble.stats.sent[channel] == 1u);
                assert(s_ble.stats.txCrcMismatch == 0u && s_ble.stats.fragmentError == 0u);
                if (length <= 512u) assert(sentLength == length && !memcmp(sentBytes, payload, length));
                else assert(!memcmp(sentBytes, "G,", 2u));
            }
            wireCase = NULL; payload[length - 1u] = 'A';
        }
    }
    /* Actual encoder bytes, passed through the actual Notify/owner/SendPayload
     * and formatter, then replayed by Python through the real receiver. */
    for (unsigned mtu = 247u; mtu <= 517u; mtu += 270u) {
        const uint8_t *frames[] = {referenceCapMtu, referenceVoltMtu};
        const size_t sizes[] = {sizeof referenceCapMtu, sizeof referenceVoltMtu};
        for (unsigned i = 0; i < 2; ++i) {
            reset(); s_ble.stats.mtu = (uint16_t)mtu;
            char label[48]; snprintf(label, sizeof label, "golden-%s-mtu%u", i ? "volt" : "cap", mtu);
            wireCase = label;
            journalWire("expected", 'D', frames[i], sizes[i]);
            assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, frames[i], sizes[i], 7u) == ESP_OK);
            runToIdle();
            assert(sdkCalls == (mtu == 247u ? (i ? 3u : 2u) : (i ? 2u : 1u)));
            wireCase = NULL;
        }
    }
}

/* Independent snprintf oracle for the proposed two-packet geometry. Production
 * uses bounded decimal-width arithmetic; this test searches formatted headers. */
static size_t formattedFirstChunk(size_t length, uint32_t messageId, uint32_t crc)
{
    for (size_t first = 495u; first != 0u; --first) {
        int header = snprintf(NULL, 0, "G,D,%lu,0,2,%u,%u,%08lX\n",
            (unsigned long)messageId, (unsigned)first, (unsigned)length,
            (unsigned long)crc);
        assert(header > 0);
        if ((size_t)header + first > 495u) continue;
        assert(first < length);
        size_t second = length - first;
        header = snprintf(NULL, 0, "G,D,%lu,1,2,%u,%u,%08lX\n",
            (unsigned long)messageId, (unsigned)second, (unsigned)length,
            (unsigned long)crc);
        return second >= 8u && (size_t)header + second <= 244u ?
            first : length / 2u + length % 2u;
    }
    assert(false);
    return 0u;
}

static void assertSentPayload(const uint8_t *payload, size_t length,
                              char channel, uint32_t messageId, size_t maximum)
{
    if (length <= maximum) {
        assert(sentChannelCount == 1u && sentLength == length &&
               !memcmp(sentBytes, payload, length));
        return;
    }
    uint32_t count = 0u, crc = sensorarrayBleCrc32(payload, length);
    assert(sensorarrayBlePlanFragments(length, maximum, channel, messageId, crc, &count));
    assert(sentChannelCount == count && sdkCalls == count);
    size_t cursor = 0u, payloadOffset = 0u;
    size_t first = channel == 'D' && maximum == 512u && count == 2u ?
        formattedFirstChunk(length, messageId, crc) : length / count + (length % count != 0u);
    for (uint32_t index = 0u; index < count; ++index) {
        char actualChannel = 0;
        unsigned mid, part, parts, chunk, total, actualCrc;
        int header = 0;
        assert(sscanf((char *)sentBytes + cursor, "G,%c,%u,%u,%u,%u,%u,%x%n",
            &actualChannel, &mid, &part, &parts, &chunk, &total, &actualCrc, &header) == 7);
        assert(sentBytes[cursor + header] == '\n'); header++;
        size_t expected = count == 2u ? (index == 0u ? first : length - first) :
            length / count + (index < length % count ? 1u : 0u);
        assert(actualChannel == channel && mid == messageId && part == index &&
               parts == count && total == length && actualCrc == crc && chunk == expected);
        assert((size_t)header + chunk == sentLengths[index] && sentLengths[index] <= maximum);
        assert(payloadOffset + chunk <= length && !memcmp(sentBytes + cursor + header,
                                                          payload + payloadOffset, chunk));
        cursor += sentLengths[index]; payloadOffset += chunk;
    }
    assert(cursor == sentLength && payloadOffset == length);
    if (count == 2u && first != length / 2u + length % 2u) {
        assert(sentLengths[0] == 495u && sentLengths[1] <= 244u && length - first >= 8u);
    }
}

static void testTwoFragmentPacking(void)
{
    const uint32_t ids[] = {1u, 9u, 10u, 99u, 100u, 999u, 1000u, 9999u, 10000u,
        99999u, 100000u, 999999u, 1000000u, 9999999u, 10000000u,
        99999999u, 100000000u, 999999999u, 1000000000u, UINT32_MAX};
    uint8_t payload[2304]; memset(payload, 'A', sizeof payload);
    unsigned geometry = 0u, sends = 0u;
    for (size_t length = 513u; length <= sizeof payload; ++length) {
        for (size_t i = 0u; i < sizeof ids / sizeof *ids; ++i) {
            uint32_t crc = UINT32_C(0xA50B10C2);
            size_t balanced = length / 2u + length % 2u;
            assert(sensorarrayBleTwoFragmentFirstChunk(length, 512u, 'D', ids[i], crc) ==
                   formattedFirstChunk(length, ids[i], crc));
            assert(sensorarrayBleTwoFragmentFirstChunk(length, 511u, 'D', ids[i], crc) == balanced);
            assert(sensorarrayBleTwoFragmentFirstChunk(length, 244u, 'D', ids[i], crc) == balanced);
            assert(sensorarrayBleTwoFragmentFirstChunk(length, 512u, 'C', ids[i], crc) == balanced);
            assert(sensorarrayBleTwoFragmentFirstChunk(length, 512u, 'L', ids[i], crc) == balanced);
            geometry++;
        }
        /* Every public fragmented length executes the real owner, slot, CRC,
         * planner, formatter and SDK-boundary path. IDs cycle decimal edges. */
        uint32_t id = ids[(length - 513u) % (sizeof ids / sizeof *ids)];
        reset(); s_ble.stats.mtu = 517u; s_ble.nextMessageId[0] = id - 1u;
        assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, payload, length, 7u) == ESP_OK);
        runToIdle(); assertSentPayload(payload, length, 'D', id, 512u); sends++;
        assert(s_ble.stats.sent[0] == 1u && s_ble.stats.txCrcMismatch == 0u &&
               s_ble.stats.fragmentError == 0u && s_ble.stats.tinyTailCount == 0u);
    }
    const unsigned mtus[] = {64u, 247u, 514u, 515u, 516u, 517u};
    const size_t lengths[] = {512u, 513u, 598u, 666u, 700u, 959u, 960u, 1000u, 2304u};
    for (size_t m = 0u; m < sizeof mtus / sizeof *mtus; ++m) {
        for (size_t l = 0u; l < sizeof lengths / sizeof *lengths; ++l) {
            for (unsigned channel = 0u; channel < 2u; ++channel) {
                reset(); s_ble.stats.mtu = (uint16_t)mtus[m];
                if (channel == SENSORARRAY_BLE_CH_LOG) s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
                s_ble.nextMessageId[channel] = UINT32_MAX - 1u;
                assert(sensorarrayBleNotifyForGeneration(channel, payload, lengths[l], 7u) == ESP_OK);
                runToIdle();
                size_t maximum = mtus[m] - 3u; if (maximum > 512u) maximum = 512u;
                assertSentPayload(payload, lengths[l], channel ? 'L' : 'D', UINT32_MAX, maximum);
                sends++;
            }
        }
    }
    /* Real encoder bytes at all decimal-width edges are replayed through the
     * Python receiver too; neither G CRC nor scientific K CRC is fabricated. */
    for (size_t i = 0u; i < sizeof ids / sizeof *ids; ++i) {
        reset(); s_ble.stats.mtu = 517u; s_ble.nextMessageId[0] = ids[i] - 1u;
        char label[64]; snprintf(label, sizeof label, "golden-volt-pack-%lu", (unsigned long)ids[i]);
        wireCase = label;
        journalWire("expected", 'D', referenceVoltMtu, sizeof referenceVoltMtu);
        assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, referenceVoltMtu,
                                                sizeof referenceVoltMtu, 7u) == ESP_OK);
        runToIdle(); assertSentPayload(referenceVoltMtu, sizeof referenceVoltMtu, 'D', ids[i], 512u);
        wireCase = NULL; sends++;
    }
    reset(); s_ble.stats.mtu = 517u; s_ble.nextMessageId[0] = UINT32_MAX;
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, payload, 598u, 7u) == ESP_OK);
    runToIdle(); assertSentPayload(payload, 598u, 'D', 1u, 512u); sends++;
    /* Cancellation or failure after admission never releases a live payload
     * early, sends a tail under a new connection, or calls a partial send whole. */
    for (unsigned scenario = 0u; scenario < 3u; ++scenario) {
        reset(); s_ble.stats.mtu = 517u;
        if (scenario == 0u) disconnectAfter = 1u;
        if (scenario == 1u) disableDataAfter = 1u;
        if (scenario == 2u) sdkFailures = 1u;
        assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, payload, 598u, 7u) == ESP_OK);
        runToIdle(); assert(sdkCalls == 1u && s_ble.stats.sent[0] == 0u && s_ble.stats.dropped[0] == 1u);
        assert(s_ble.stats.txSlotReleaseMismatch == 0u);
    }
    printf("BLE_PACKING_PASS,geometry=%u,actual_send=%u,cancellation=3,hardware=NOT_RUN\n", geometry, sends);
}

static void testMtuReconnect(void)
{
    reset(); s_ble.config.preferredMtu = TEST_APP_PREFERRED_MTU;
    assert(TEST_APP_PREFERRED_MTU == 517u);
    esp_ble_gatts_cb_param_t event = {.mtu = {.conn_id=3u, .mtu=517u}};
    dispatchConnection(ESP_GATTS_MTU_EVT, &event); assert(s_ble.stats.mtu == 517u);
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK);
    sensorarrayBleTxDescriptor_t held;
    assert(xQueueReceive(s_ble.txQueue, &held, 0) == pdTRUE);
    dispatchConnection(ESP_GATTS_DISCONNECT_EVT, &event);
    assert(!s_ble.stats.connected && s_ble.stats.mtu == 23u && s_ble.connectionGeneration == 8u);
    memset(&event, 0, sizeof event); event.connect.conn_id = 3u;
    memset(event.connect.remote_bda, 0x5a, 6u);
    dispatchConnection(ESP_GATTS_CONNECT_EVT, &event);
    assert(s_ble.stats.connected && s_ble.stats.mtu == 23u && s_ble.connectionGeneration == 9u);
    assert(s_ble.cccd[0] == 0u && s_ble.cccd[1] == 0u && s_ble.cccd[2] == 0u);
    assert(sensorarrayBleSendDescriptor(&held) == ESP_ERR_INVALID_STATE && sdkCalls == 0u);
    sensorarrayBleReleaseTxDescriptor(&held); assert(s_ble.stats.txSlotUsed == 0u);
    event.mtu.conn_id = 3u; event.mtu.mtu = 247u;
    dispatchConnection(ESP_GATTS_MTU_EVT, &event);
    assert(s_ble.stats.mtu == 247u && s_ble.config.preferredMtu == 517u);
    s_ble.cccd[0] = 1u;
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, referenceVoltMtu,
           sizeof referenceVoltMtu, 9u) == ESP_OK);
    runToIdle(); assert(sdkCalls == 3u);
}

static void testSendGuard(void)
{
    uint8_t payload[598]; memset(payload, 'A', sizeof payload);
    reset(); s_ble.stats.mtu = 517u; observeDuringSdk = true;
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 0u;
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
    s_ble.cccd[SENSORARRAY_BLE_CH_DATA] = 1u;
    /* The direct owner call lets the SDK stub observe two queued, untouched
     * slots while the first descriptor remains owned inside the SDK boundary. */
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA, payload, sizeof payload, 7u) == ESP_OK);
    sensorarrayBleTxDescriptor_t held;
    assert(xQueueReceive(s_ble.txQueue, &held, 0) == pdTRUE);
    uint8_t *persistentStorage=s_ble.payloadStorage;
    uint8_t *heldStorage=sensorarrayBleDescriptorData(&held);
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK);
    assert(sensorarrayBleSendDescriptor(&held) == ESP_ERR_TIMEOUT);
    assert(sdkCalls == 1u && s_ble.stats.txSlotUsed == 3u && s_ble.stats.sent[0] == 0u);
    sensorarrayBleGuardSnapshot_t guard;
    sensorarrayBleGetGuardSnapshot(&guard);
    assert(guard.quarantined && !guard.active && guard.completionKnown && guard.callId == 1u);
    assert(guard.startUs == 1000000u && guard.detectedUs == 1500000u);
    assert(s_ble.link.sdkAccepted == 1u && s_ble.link.sdkLastError == ESP_OK && s_ble.link.sdkCompletedCalls == 1u);
    assert(s_ble.payloadStorage==persistentStorage && sensorarrayBleDescriptorData(&held)==heldStorage);
    assert(!memcmp(heldStorage,payload,sizeof payload) && payloadAllocations==1u);
    sensorarrayBleReleaseTxDescriptor(&held); runToIdle();
    assert(sdkCalls == 1u && s_ble.stats.txSlotUsed == 0u && s_ble.stats.txConnectionStaleDrop == 0u);
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_ERR_INVALID_STATE);
    esp_ble_gatts_cb_param_t event = {.connect = {.conn_id=3u}};
    dispatchConnection(ESP_GATTS_DISCONNECT_EVT, &event);
    memset(event.connect.remote_bda, 0x5a, sizeof event.connect.remote_bda);
    dispatchConnection(ESP_GATTS_CONNECT_EVT, &event);
    s_ble.cccd[0] = 1u;
    sensorarrayBleGetGuardSnapshot(&guard);
    assert(guard.quarantined && guard.completionKnown && guard.callId == 1u &&
           guard.startUs == 1000000u && guard.detectedUs == 1500000u);
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_ERR_INVALID_STATE && sdkCalls == 1u);
    assert(s_ble.payloadStorage==persistentStorage && payloadAllocations==1u);

    for (unsigned late = 0u; late < 3u; ++late) {
        reset(); sdkDelayUs = late == 0u ? 499999u : 500000u;
        if (late == 2u) sdkFailures = 1u;
        assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
        sensorarrayBleGetGuardSnapshot(&guard);
        assert(guard.quarantined == (late != 0u) && !guard.active && guard.completionKnown);
        assert(sdkCalls == 1u && s_ble.stats.sent[0] == (late == 0u));
        assert(s_ble.link.sdkLastError == (late == 2u ? ESP_FAIL : ESP_OK));
        assert(s_ble.link.sdkAccepted == (late != 2u) && s_ble.link.sdkErrors == (late == 2u));
    }
    reset(); s_ble.link.sdkCallCount = s_ble.link.sdkCompletedCalls = UINT32_MAX;
    sdkDelayUs = 500000u;
    assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); runToIdle();
    sensorarrayBleGetGuardSnapshot(&guard);
    assert(guard.callKnown && guard.callId == 0u && guard.completionKnown && guard.quarantined);

    reset(); s_ble.txMode = SENSORARRAY_BLE_TX_SAFE; s_ble.cccd[2] = 2u;
    observeDuringConfirm = true;
    assert(enqueue(SENSORARRAY_BLE_CH_CTRL) == ESP_OK); runToIdle();
    assert(!s_ble.guard.quarantined && s_ble.confirmWaitTask == NULL && sdkCalls == 1u);
    /* Idle/no-host/paused send owner: elapsed uptime alone cannot quarantine. */
    reset(); s_ble.stats.connected = false; nowUs = UINT64_C(360000000000);
    sensorarrayBleObserveSendDeadline(); sensorarrayBleGetGuardSnapshot(&guard);
    assert(guard.ready && !guard.quarantined && !guard.callKnown && !guard.completionKnown && !guard.restartRequired);
    puts("BLE_GUARD_SEND_PASS,cases=8,deadline_us=500000,slot_retained_inflight=1,late_completion_frozen=1,reconnect_sticky=1,hardware=NOT_RUN");
}
static void testPersistentMaxPayload(void)
{
    reset();
    uint8_t input[SENSORARRAY_BLE_MESSAGE_VALUE_MAX], expected[sizeof input];
    for (unsigned i=0u; i<sizeof input; ++i) input[i]=(uint8_t)('A'+i%26u);
    input[sizeof input-1u]='\n'; memcpy(expected,input,sizeof input);
    assert(sizeof input==2304u && s_ble.payloadStorage==payloadStorage && payloadAllocations==1u);
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA,input,sizeof input,7u)==ESP_OK);
    sensorarrayBleTxDescriptor_t held;
    assert(xQueueReceive(s_ble.txQueue,&held,0)==pdTRUE);
    uint8_t *owned=sensorarrayBleDescriptorData(&held);
    assert(owned>=payloadStorage && owned+sizeof input<=payloadStorage+sizeof payloadStorage);
    memset(input,'Z',sizeof input); /* Caller may immediately reuse its source. */
    assert(!memcmp(owned,expected,sizeof expected));
    assert(sensorarrayBleSendDescriptor(&held)==ESP_OK);
    assertSentPayload(expected,sizeof expected,'D',1u,244u);
    assert(sensorarrayBleDescriptorData(&held)==owned && !memcmp(owned,expected,sizeof expected));
    sensorarrayBleReleaseTxDescriptor(&held);
    assert(s_ble.stats.txSlotUsed==0u && payloadAllocations==1u && s_ble.payloadStorage==payloadStorage);
    /* A corrupted persistent source is rejected by the dequeue CRC before
     * any new SDK call; retaining this also covers the scratch experiment. */
    assert(sensorarrayBleNotifyForGeneration(SENSORARRAY_BLE_CH_DATA,expected,sizeof expected,7u)==ESP_OK);
    assert(xQueueReceive(s_ble.txQueue,&held,0)==pdTRUE);
    owned=sensorarrayBleDescriptorData(&held); owned[17]^=1u;
    unsigned callsBefore=sdkCalls;
    assert(sensorarrayBleSendDescriptor(&held)==ESP_ERR_INVALID_CRC);
    assert(sdkCalls==callsBefore && s_ble.stats.txSlotUsed==1u);
    sensorarrayBleReleaseTxDescriptor(&held);
    puts("BLE_PERSISTENT_PAYLOAD_PASS,bytes=18432,slots=8,max_payload=2304,caller_reuse=1,whole_send=1,no_reallocation=1,hardware=NOT_RUN");
}
int main(int argc, char **argv)
{
    assert(argc == 2); wireJournal = fopen(argv[1], "wb"); assert(wireJournal);
    testLogDataAdmissionPolicy(); testFrameProtection(); testPeriodicCadence(); testFullCompletionAnchor(); testPeriodicGoldenFull(); testAdmissionAndPriority(); testDataBeforeBestEffortLog(); testLifecycleAndWake();
    testPartialOwnership(); testFailuresAndConfirm(); testNegotiation();
    testSdkObservation(); testConfObservation();
    testMtuPayloadBoundaries(); testMtuReconnect(); testTwoFragmentPacking(); testSendGuard(); testPersistentMaxPayload();
    assert(fclose(wireJournal) == 0);
    printf("BLE_PRIORITY_PASS,groups=18,dataSlots=%u,logSlots=%u,ctrlSlots=%u,stateBytes=%u,descriptorBytes=%u,hardware=NOT_RUN\n",
        CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN, SENSORARRAY_BLE_LOG_TX_SLOT_COUNT,
        SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT, (unsigned)sizeof(s_ble), (unsigned)sizeof(sensorarrayBleTxDescriptor_t));
    return 0;
}
