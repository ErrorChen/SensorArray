/* Production BLE bodies are inserted by test_ble_priority.py. No radio/HIL. */
#include <assert.h>
#include <setjmp.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayBle.h"

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

static unsigned lockDepth, checkpoints, wakeGives, idleTakes, delayCalls, queueInitCount, failQueueInit;
static unsigned failQueueSends, sdkCalls, sdkFailures, disconnectAfter, priorityOnSend;
static unsigned phyRequests, readRequests, connRequests, dleRequests;
static uint64_t nowUs, sdkDelayUs;
static uint32_t idleGeneration, sdkIdleAfter;
static bool changeIdleDuringSdk;
static esp_err_t phyReturn, fallbackReturn, readReturn, connReturn, dleReturn;
static bool wakeOnlyDuringConfirm, confirmSuccess, injectOnIdle, injectLogOnSend;
static uint32_t notifications;
static jmp_buf taskIdle;
static uint8_t sentBytes[65536];
static size_t sentLength;
static char sentChannels[256];
static unsigned sentChannelCount;
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
static void vTaskDelay(TickType_t ticks);
static esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t interface, uint16_t conn,
    uint16_t handle, uint16_t length, uint8_t *value, bool confirm);
static esp_err_t esp_ble_gap_set_preferred_phy(esp_bd_addr_t peer, unsigned all,
    esp_ble_gap_phy_mask_t tx, esp_ble_gap_phy_mask_t rx, unsigned options);
static esp_err_t esp_ble_gap_read_phy(esp_bd_addr_t peer);
static esp_err_t esp_ble_gap_update_conn_params(esp_ble_conn_update_params_t *params);
static esp_err_t esp_ble_gap_set_pkt_data_len(esp_bd_addr_t peer, uint16_t length);

/* PRODUCTION_TYPES */
/* PRODUCTION_FUNCTIONS */

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
    }
    uint32_t result = notifications; notifications = 0u; return result;
}
static void vTaskDelay(TickType_t ticks)
{ assert(!lockDepth && ticks > 0u); nowUs += (uint64_t)ticks * 10000u; delayCalls++; }
static esp_err_t esp_ble_gatts_send_indicate(esp_gatt_if_t interface, uint16_t conn,
    uint16_t handle, uint16_t length, uint8_t *value, bool confirm)
{
    (void)confirm;
    assert(!lockDepth && interface == 2 && conn == 3 && value && length <= 244u);
    assert(s_ble.link.sdkCallActive && s_ble.link.sdkCallStartedUs == nowUs);
    assert(s_ble.link.sdkCallCount == s_ble.link.sdkCompletedCalls + 1u);
    assert(s_ble.stats.txSlotUsed > 0u); /* Caller still owns persistent payload. */
    sdkCalls++; nowUs += sdkDelayUs;
    if (changeIdleDuringSdk) { idleGeneration = sdkIdleAfter; }
    if (sdkCalls <= sdkFailures) { return ESP_FAIL; }
    assert(sentLength + length <= sizeof(sentBytes));
    memcpy(sentBytes + sentLength, value, length); sentLength += length;
    assert(sentChannelCount < sizeof(sentChannels));
    sentChannels[sentChannelCount++] = handle == 20 ? 'D' : (handle == 30 ? 'L' : 'C');
    if (injectLogOnSend) {
        injectLogOnSend = false;
        assert(sensorarrayBleNotify(SENSORARRAY_BLE_CH_LOG, (const uint8_t *)"LATELOG\n", 8u) == ESP_OK);
    }
    if (priorityOnSend && sdkCalls == priorityOnSend) {
        assert(sensorarrayBleNotify(SENSORARRAY_BLE_CH_CTRL, (const uint8_t *)"LATECTRL\n", 9u) == ESP_OK);
    }
    if (disconnectAfter && sdkCalls == disconnectAfter) {
        s_ble.stats.connected = false;
        s_ble.connectionGeneration++;
        sensorarrayBleDrainTxQueues();
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
    failQueueSends = sdkCalls = sdkFailures = disconnectAfter = priorityOnSend = 0u;
    phyRequests = readRequests = connRequests = dleRequests = notifications = 0u;
    phyReturn = fallbackReturn = readReturn = connReturn = dleReturn = ESP_OK;
    wakeOnlyDuringConfirm = confirmSuccess = injectOnIdle = injectLogOnSend = false;
    sentLength = sentChannelCount = 0u;
    memset(sentBytes, 0, sizeof(sentBytes)); memset(sentChannels, 0, sizeof(sentChannels));
    assert(initializeQueues() == ESP_OK);
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
static void testAdmissionAndPriority(void)
{
    reset();
    assert(CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN == 4u && SENSORARRAY_BLE_LOG_TX_SLOT_COUNT == 2u && SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT == 2u);
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK && enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_ERR_NO_MEM);
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
static void testWeightedLog(void)
{
    reset();
    for (unsigned i=0u; i<4u; i++) { assert(enqueue(SENSORARRAY_BLE_CH_DATA) == ESP_OK); }
    assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
    uint32_t dataRun = 0u;
    for (unsigned i=0u; i<10u; i++) {
        sensorarrayBleTxDescriptor_t desc;
        assert(sensorarrayBleReceiveNext(&desc, &dataRun));
        assert(desc.channel == ((i == 4u || i == 9u) ? SENSORARRAY_BLE_CH_LOG : SENSORARRAY_BLE_CH_DATA));
        assert(sensorarrayBleSendDescriptor(&desc) == ESP_OK);
        sensorarrayBleReleaseTxDescriptor(&desc);
        assert(enqueue(desc.channel) == ESP_OK);
    }
    sensorarrayBleDrainTxQueues(); assert(s_ble.stats.txSlotUsed == 0u);
}
static void testLifecycleAndWake(void)
{
    reset(); assert(enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK && enqueue(SENSORARRAY_BLE_CH_LOG) == ESP_OK);
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
    runToIdle(); assert(sentChannelCount == 5u && !memcmp(sentChannels, "DDDCL", 5u));
    assert(s_ble.stats.sent[0] == 1u && s_ble.stats.sent[1] == 1u && s_ble.stats.sent[2] == 1u);
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
int main(void)
{
    testAdmissionAndPriority(); testWeightedLog(); testLifecycleAndWake();
    testPartialOwnership(); testFailuresAndConfirm(); testNegotiation();
    testSdkObservation();
    printf("BLE_PRIORITY_PASS,groups=7,dataSlots=%u,logSlots=%u,ctrlSlots=%u,stateBytes=%u,descriptorBytes=%u,hardware=NOT_RUN\n",
        CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN, SENSORARRAY_BLE_LOG_TX_SLOT_COUNT,
        SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT, (unsigned)sizeof(s_ble), (unsigned)sizeof(sensorarrayBleTxDescriptor_t));
    return 0;
}
