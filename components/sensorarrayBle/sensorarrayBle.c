#include "sensorarrayBle.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifndef CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN
#define CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN 4
#endif
#ifndef CONFIG_SENSORARRAY_BLE_TX_TASK_STACK
#define CONFIG_SENSORARRAY_BLE_TX_TASK_STACK 4096
#endif
#ifndef CONFIG_SENSORARRAY_BLE_TX_TASK_PRIORITY
#define CONFIG_SENSORARRAY_BLE_TX_TASK_PRIORITY 5
#endif
#ifndef CONFIG_SENSORARRAY_BLE_TX_TASK_CORE
#define CONFIG_SENSORARRAY_BLE_TX_TASK_CORE 0
#endif
#ifndef CONFIG_SENSORARRAY_BLE_TX_MODE_SAFE
#define CONFIG_SENSORARRAY_BLE_TX_MODE_SAFE 0
#endif

#define SENSORARRAY_BLE_APP_ID 0x53u
#define SENSORARRAY_BLE_SERVICE_UUID 0x00FFu
#define SENSORARRAY_BLE_CTRL_RX_UUID 0xFF10u
#define SENSORARRAY_BLE_CTRL_TX_UUID 0xFF11u
#define SENSORARRAY_BLE_DATA_TX_UUID 0xFF20u
#define SENSORARRAY_BLE_LOG_TX_UUID 0xFF30u
#define SENSORARRAY_BLE_ATTR_VALUE_MAX 512u
#define SENSORARRAY_BLE_MESSAGE_VALUE_MAX 1536u
#define SENSORARRAY_BLE_TINY_TAIL_MIN 8u
#define SENSORARRAY_BLE_CONF_TIMEOUT_MS 1000u

enum {
    BLE_IDX_SERVICE = 0,
    BLE_IDX_CTRL_RX_DECL,
    BLE_IDX_CTRL_RX_VALUE,
    BLE_IDX_CTRL_TX_DECL,
    BLE_IDX_CTRL_TX_VALUE,
    BLE_IDX_CTRL_TX_CCCD,
    BLE_IDX_DATA_TX_DECL,
    BLE_IDX_DATA_TX_VALUE,
    BLE_IDX_DATA_TX_CCCD,
    BLE_IDX_LOG_TX_DECL,
    BLE_IDX_LOG_TX_VALUE,
    BLE_IDX_LOG_TX_CCCD,
    BLE_IDX_COUNT,
};

typedef struct {
    sensorarrayBleChannel_t channel;
    uint16_t length;
    uint8_t data[SENSORARRAY_BLE_MESSAGE_VALUE_MAX];
} sensorarrayBleQueuedMessage_t;

typedef struct {
    sensorarrayBleConfig_t config;
    sensorarrayBleStats_t stats;
    bool initialized;
    bool txTaskStarted;
    bool subscribed[3];
    uint32_t nextMessageId[3];
    esp_gatt_if_t gattsIf;
    uint16_t connId;
    uint16_t handles[BLE_IDX_COUNT];
    QueueHandle_t txQueue;
    StaticQueue_t txQueueStruct;
    uint8_t txQueueStorage[CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN *
                           sizeof(sensorarrayBleQueuedMessage_t)];
    TaskHandle_t txTask;
    TaskHandle_t confirmWaitTask;
    sensorarrayBleTxMode_t txMode;
    esp_bd_addr_t remote;
    sensorarrayBleControlRxCallback_t controlCallback;
    void *controlContext;
} sensorarrayBleState_t;

static sensorarrayBleState_t s_ble;
static portMUX_TYPE s_bleMux = portMUX_INITIALIZER_UNLOCKED;

static uint16_t s_primaryServiceUuid = ESP_GATT_UUID_PRI_SERVICE;
static uint16_t s_characterDeclarationUuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint16_t s_cccdUuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static uint16_t s_serviceUuid = SENSORARRAY_BLE_SERVICE_UUID;
static uint16_t s_ctrlRxUuid = SENSORARRAY_BLE_CTRL_RX_UUID;
static uint16_t s_ctrlTxUuid = SENSORARRAY_BLE_CTRL_TX_UUID;
static uint16_t s_dataTxUuid = SENSORARRAY_BLE_DATA_TX_UUID;
static uint16_t s_logTxUuid = SENSORARRAY_BLE_LOG_TX_UUID;
static uint8_t s_ctrlRxProperties = ESP_GATT_CHAR_PROP_BIT_WRITE |
                                    ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
static uint8_t s_txProperties = ESP_GATT_CHAR_PROP_BIT_READ |
                                ESP_GATT_CHAR_PROP_BIT_NOTIFY |
                                ESP_GATT_CHAR_PROP_BIT_INDICATE;
static uint8_t s_emptyValue[] = {0};
static uint8_t s_initialCccd[] = {0, 0};

#define BLE_DECL(index, properties) \
    [index] = { \
        {ESP_GATT_AUTO_RSP}, \
        {ESP_UUID_LEN_16, (uint8_t *)&s_characterDeclarationUuid, ESP_GATT_PERM_READ, \
         sizeof(properties), sizeof(properties), &(properties)}, \
    }
#define BLE_VALUE(index, uuid, permissions) \
    [index] = { \
        {ESP_GATT_AUTO_RSP}, \
        {ESP_UUID_LEN_16, (uint8_t *)&(uuid), permissions, SENSORARRAY_BLE_ATTR_VALUE_MAX, \
         sizeof(s_emptyValue), s_emptyValue}, \
    }
#define BLE_CCCD(index) \
    [index] = { \
        {ESP_GATT_AUTO_RSP}, \
        {ESP_UUID_LEN_16, (uint8_t *)&s_cccdUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, \
         sizeof(s_initialCccd), sizeof(s_initialCccd), s_initialCccd}, \
    }

static const esp_gatts_attr_db_t s_gattDb[BLE_IDX_COUNT] = {
    [BLE_IDX_SERVICE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_primaryServiceUuid, ESP_GATT_PERM_READ,
         sizeof(s_serviceUuid), sizeof(s_serviceUuid), (uint8_t *)&s_serviceUuid},
    },
    BLE_DECL(BLE_IDX_CTRL_RX_DECL, s_ctrlRxProperties),
    BLE_VALUE(BLE_IDX_CTRL_RX_VALUE, s_ctrlRxUuid, ESP_GATT_PERM_WRITE),
    BLE_DECL(BLE_IDX_CTRL_TX_DECL, s_txProperties),
    BLE_VALUE(BLE_IDX_CTRL_TX_VALUE, s_ctrlTxUuid, ESP_GATT_PERM_READ),
    BLE_CCCD(BLE_IDX_CTRL_TX_CCCD),
    BLE_DECL(BLE_IDX_DATA_TX_DECL, s_txProperties),
    BLE_VALUE(BLE_IDX_DATA_TX_VALUE, s_dataTxUuid, ESP_GATT_PERM_READ),
    BLE_CCCD(BLE_IDX_DATA_TX_CCCD),
    BLE_DECL(BLE_IDX_LOG_TX_DECL, s_txProperties),
    BLE_VALUE(BLE_IDX_LOG_TX_VALUE, s_logTxUuid, ESP_GATT_PERM_READ),
    BLE_CCCD(BLE_IDX_LOG_TX_CCCD),
};

static uint8_t s_advRaw[31];
static uint8_t s_advRawLength;
static esp_ble_gap_ext_adv_t s_advSet[] = {{0u, 0u, 0u}};
static esp_ble_gap_ext_adv_params_t s_advParams = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
    .interval_min = 0x40,
    .interval_max = 0x80,
    .channel_map = ADV_CHNL_ALL,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
    .sid = 0,
    .scan_req_notif = false,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
};

static void sensorarrayBleLogBoot(const char *stage, esp_err_t err)
{
    printf("BLEBOOT,stage=%s,err=0x%lx,name=%s\n",
           stage,
           (unsigned long)err,
           esp_err_to_name(err));
}

void sensorarrayBleLogHeap(const char *stage)
{
    printf("M,stage=%s,ih=%u,il=%u,im=%u,dh=%u,dl=%u,h8=%u,l8=%u\n",
           stage ? stage : "unknown",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

static void sensorarrayBleBuildAdvData(void)
{
    size_t nameLength = strnlen(s_ble.config.deviceName, sizeof(s_ble.config.deviceName));
    if (nameLength > sizeof(s_advRaw) - 9u) {
        nameLength = sizeof(s_advRaw) - 9u;
    }
    const uint8_t prefix[] = {
        0x02, 0x01, 0x06,
        0x03, 0x03, (uint8_t)(SENSORARRAY_BLE_SERVICE_UUID & 0xFFu),
        (uint8_t)(SENSORARRAY_BLE_SERVICE_UUID >> 8u),
    };
    memcpy(s_advRaw, prefix, sizeof(prefix));
    s_advRaw[sizeof(prefix)] = (uint8_t)(nameLength + 1u);
    s_advRaw[sizeof(prefix) + 1u] = 0x09u;
    memcpy(&s_advRaw[sizeof(prefix) + 2u], s_ble.config.deviceName, nameLength);
    s_advRawLength = (uint8_t)(sizeof(prefix) + 2u + nameLength);
}

static int sensorarrayBleCccdChannel(uint16_t handle)
{
    if (handle == s_ble.handles[BLE_IDX_DATA_TX_CCCD]) {
        return SENSORARRAY_BLE_CH_DATA;
    }
    if (handle == s_ble.handles[BLE_IDX_LOG_TX_CCCD]) {
        return SENSORARRAY_BLE_CH_LOG;
    }
    if (handle == s_ble.handles[BLE_IDX_CTRL_TX_CCCD]) {
        return SENSORARRAY_BLE_CH_CTRL;
    }
    return -1;
}

static uint16_t sensorarrayBleValueHandle(sensorarrayBleChannel_t channel)
{
    switch (channel) {
    case SENSORARRAY_BLE_CH_DATA:
        return s_ble.handles[BLE_IDX_DATA_TX_VALUE];
    case SENSORARRAY_BLE_CH_LOG:
        return s_ble.handles[BLE_IDX_LOG_TX_VALUE];
    case SENSORARRAY_BLE_CH_CTRL:
        return s_ble.handles[BLE_IDX_CTRL_TX_VALUE];
    default:
        return 0u;
    }
}

static char sensorarrayBleEnvelopeChannel(sensorarrayBleChannel_t channel)
{
    switch (channel) {
    case SENSORARRAY_BLE_CH_DATA:
        return 'D';
    case SENSORARRAY_BLE_CH_LOG:
        return 'L';
    case SENSORARRAY_BLE_CH_CTRL:
    default:
        return 'C';
    }
}

static uint32_t sensorarrayBleCrc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0u; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

static void sensorarrayBleNoteDrop(sensorarrayBleChannel_t channel)
{
    if (channel <= SENSORARRAY_BLE_CH_CTRL) {
        s_ble.stats.dropped[channel]++;
    }
    s_ble.stats.messageDropped++;
}

static bool sensorarrayBleValidateAscii(sensorarrayBleChannel_t channel,
                                        const uint8_t *data,
                                        size_t length,
                                        uint32_t messageId,
                                        uint32_t fragmentIndex)
{
    if (!data) {
        return false;
    }
    for (size_t offset = 0u; offset < length; ++offset) {
        if (data[offset] > 0x7Fu) {
            printf("BLX,ch=%c,reason=nonascii,mid=%lu,frag=%lu,off=%lu,byte=%02X\n",
                   sensorarrayBleEnvelopeChannel(channel),
                   (unsigned long)messageId,
                   (unsigned long)fragmentIndex,
                   (unsigned long)offset,
                   (unsigned)data[offset]);
            return false;
        }
    }
    return true;
}

const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t mode)
{
    return mode == SENSORARRAY_BLE_TX_SAFE ? "safe" : "fast";
}

void sensorarrayBleSetTxMode(sensorarrayBleTxMode_t mode)
{
    if (mode > SENSORARRAY_BLE_TX_SAFE) {
        return;
    }
    portENTER_CRITICAL(&s_bleMux);
    s_ble.txMode = mode;
    portEXIT_CRITICAL(&s_bleMux);
}

sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void)
{
    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleTxMode_t mode = s_ble.txMode;
    portEXIT_CRITICAL(&s_bleMux);
    return mode;
}

static size_t sensorarrayBleMaxPayloadSize(size_t maximum,
                                           char envChannel,
                                           uint32_t messageId,
                                           uint32_t index,
                                           uint32_t count,
                                           size_t chunk,
                                           size_t messageLength,
                                           uint32_t crc)
{
    uint8_t header[SENSORARRAY_BLE_ATTR_VALUE_MAX];
    int headerLen = snprintf((char *)header,
                             sizeof(header),
                             "G,%c,%lu,%lu,%lu,%u,%u,%08lX\n",
                             envChannel,
                             (unsigned long)messageId,
                             (unsigned long)index,
                             (unsigned long)count,
                             (unsigned)chunk,
                             (unsigned)messageLength,
                             (unsigned long)crc);
    if (headerLen <= 0 || (size_t)headerLen >= maximum) {
        return 0u;
    }
    return maximum - (size_t)headerLen;
}

static bool sensorarrayBlePlanFragments(size_t length,
                                        size_t maximum,
                                        char envChannel,
                                        uint32_t messageId,
                                        uint32_t crc,
                                        uint32_t *outCount)
{
    if (!outCount || length == 0u || maximum == 0u) {
        return false;
    }
    for (uint32_t count = 1u; count <= length; ++count) {
        size_t base = length / count;
        size_t extra = length % count;
        if (base == 0u) {
            return false;
        }
        bool fits = true;
        for (uint32_t index = 0u; index < count; ++index) {
            size_t chunk = base + (index < extra ? 1u : 0u);
            size_t payloadMax = sensorarrayBleMaxPayloadSize(maximum,
                                                             envChannel,
                                                             messageId,
                                                             index,
                                                             count,
                                                             chunk,
                                                             length,
                                                             crc);
            if (payloadMax < chunk) {
                fits = false;
                break;
            }
        }
        if (fits) {
            *outCount = count;
            if (count > 1u && base < SENSORARRAY_BLE_TINY_TAIL_MIN) {
                s_ble.stats.tinyTailCount++;
            }
            return true;
        }
    }
    return false;
}

static esp_err_t sensorarrayBleSendFragment(sensorarrayBleChannel_t channel,
                                            char envChannel,
                                            uint32_t messageId,
                                            uint32_t fragmentIndex,
                                            uint32_t fragmentCount,
                                            const uint8_t *data,
                                            size_t chunk,
                                            size_t messageLength,
                                            uint32_t crc,
                                            bool confirm)
{
    uint16_t handle = sensorarrayBleValueHandle(channel);
    uint8_t packet[SENSORARRAY_BLE_ATTR_VALUE_MAX];
    int headerLen = snprintf((char *)packet,
                             sizeof(packet),
                             "G,%c,%lu,%lu,%lu,%u,%u,%08lX\n",
                             envChannel,
                             (unsigned long)messageId,
                             (unsigned long)fragmentIndex,
                             (unsigned long)fragmentCount,
                             (unsigned)chunk,
                             (unsigned)messageLength,
                             (unsigned long)crc);
    size_t maximum = s_ble.stats.mtu > 3u ? (size_t)s_ble.stats.mtu - 3u : 20u;
    if (maximum > SENSORARRAY_BLE_ATTR_VALUE_MAX) {
        maximum = SENSORARRAY_BLE_ATTR_VALUE_MAX;
    }
    if (headerLen <= 0 || (size_t)headerLen >= sizeof(packet) ||
        (size_t)headerLen + chunk > maximum) {
        s_ble.stats.fragmentError++;
        return ESP_ERR_INVALID_SIZE;
    }
    if (!sensorarrayBleValidateAscii(channel, data, chunk, messageId, fragmentIndex)) {
        s_ble.stats.fragmentError++;
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(packet + headerLen, data, chunk);
    if (confirm) {
        s_ble.confirmWaitTask = xTaskGetCurrentTaskHandle();
        (void)xTaskNotifyStateClear(s_ble.confirmWaitTask);
    }
    esp_err_t err = esp_ble_gatts_send_indicate(s_ble.gattsIf,
                                                s_ble.connId,
                                                handle,
                                                (uint16_t)((size_t)headerLen + chunk),
                                                packet,
                                                confirm);
    if (err != ESP_OK) {
        s_ble.stats.fragmentError++;
        if (confirm) {
            s_ble.confirmWaitTask = NULL;
        }
        return err;
    }
    if (confirm) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE,
                                             pdMS_TO_TICKS(SENSORARRAY_BLE_CONF_TIMEOUT_MS));
        s_ble.confirmWaitTask = NULL;
        if (notified == 0u) {
            s_ble.stats.fragmentError++;
            return ESP_ERR_TIMEOUT;
        }
    }
    s_ble.stats.fragmentSent++;
    return ESP_OK;
}

static esp_err_t sensorarrayBleSendMessage(const sensorarrayBleQueuedMessage_t *message)
{
    if (!message || message->channel > SENSORARRAY_BLE_CH_CTRL || message->length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ble.stats.gattReady || !s_ble.stats.connected ||
        !s_ble.subscribed[message->channel]) {
        sensorarrayBleNoteDrop(message->channel);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ble.stats.congested && sensorarrayBleGetTxMode() == SENSORARRAY_BLE_TX_FAST) {
        s_ble.stats.congestedCount++;
        sensorarrayBleNoteDrop(message->channel);
        return ESP_ERR_TIMEOUT;
    }
    size_t maximum = s_ble.stats.mtu > 3u ? (size_t)s_ble.stats.mtu - 3u : 20u;
    if (maximum > SENSORARRAY_BLE_ATTR_VALUE_MAX) {
        maximum = SENSORARRAY_BLE_ATTR_VALUE_MAX;
    }
    char envChannel = sensorarrayBleEnvelopeChannel(message->channel);
    uint32_t messageId = ++s_ble.nextMessageId[message->channel];
    if (messageId == 0u) {
        messageId = ++s_ble.nextMessageId[message->channel];
    }
    uint32_t crc = sensorarrayBleCrc32(message->data, message->length);
    if (!sensorarrayBleValidateAscii(message->channel,
                                     message->data,
                                     message->length,
                                     messageId,
                                     0u)) {
        sensorarrayBleNoteDrop(message->channel);
        return ESP_ERR_INVALID_ARG;
    }
    if (message->length <= maximum) {
        uint16_t handle = sensorarrayBleValueHandle(message->channel);
        bool confirm = sensorarrayBleGetTxMode() == SENSORARRAY_BLE_TX_SAFE;
        if (confirm) {
            s_ble.confirmWaitTask = xTaskGetCurrentTaskHandle();
            (void)xTaskNotifyStateClear(s_ble.confirmWaitTask);
        }
        esp_err_t err = esp_ble_gatts_send_indicate(s_ble.gattsIf,
                                                    s_ble.connId,
                                                    handle,
                                                    message->length,
                                                    (uint8_t *)message->data,
                                                    confirm);
        if (err == ESP_OK && confirm) {
            uint32_t notified = ulTaskNotifyTake(pdTRUE,
                                                 pdMS_TO_TICKS(SENSORARRAY_BLE_CONF_TIMEOUT_MS));
            s_ble.confirmWaitTask = NULL;
            if (notified == 0u) {
                err = ESP_ERR_TIMEOUT;
            }
        } else if (confirm) {
            s_ble.confirmWaitTask = NULL;
        }
        if (err == ESP_OK) {
            s_ble.stats.fragmentSent++;
            s_ble.stats.sent[message->channel]++;
            s_ble.stats.messageSent++;
        } else {
            s_ble.stats.fragmentError++;
            sensorarrayBleNoteDrop(message->channel);
        }
        return err;
    }

    uint32_t fragmentCount = 0u;
    if (!sensorarrayBlePlanFragments(message->length,
                                     maximum,
                                     envChannel,
                                     messageId,
                                     crc,
                                     &fragmentCount) ||
        fragmentCount == 0u) {
        sensorarrayBleNoteDrop(message->channel);
        return ESP_ERR_INVALID_SIZE;
    }

    size_t base = message->length / fragmentCount;
    size_t extra = message->length % fragmentCount;
    size_t offset = 0u;
    bool confirm = sensorarrayBleGetTxMode() == SENSORARRAY_BLE_TX_SAFE;
    for (uint32_t index = 0u; index < fragmentCount; ++index) {
        size_t chunk = base + (index < extra ? 1u : 0u);
        esp_err_t err = sensorarrayBleSendFragment(message->channel,
                                                   envChannel,
                                                   messageId,
                                                   index,
                                                   fragmentCount,
                                                   message->data + offset,
                                                   chunk,
                                                   message->length,
                                                   crc,
                                                   confirm);
        if (err != ESP_OK) {
            sensorarrayBleNoteDrop(message->channel);
            return err;
        }
        offset += chunk;
    }
    s_ble.stats.sent[message->channel]++;
    s_ble.stats.messageSent++;
    return ESP_OK;
}

static void sensorarrayBleTxTask(void *arg)
{
    (void)arg;
    sensorarrayBleQueuedMessage_t message;
    for (;;) {
        if (xQueueReceive(s_ble.txQueue, &message, portMAX_DELAY) == pdTRUE) {
            (void)sensorarrayBleSendMessage(&message);
        }
    }
}

static void sensorarrayBleStartAdvertising(void)
{
    esp_err_t err = esp_ble_gap_ext_adv_start(1u, s_advSet);
    if (err != ESP_OK) {
        s_ble.stats.initError = err;
        sensorarrayBleLogBoot("adv_start", err);
    }
}

static void sensorarrayBleGapCallback(esp_gap_ble_cb_event_t event,
                                      esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        if (param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS) {
            esp_err_t err = esp_ble_gap_config_ext_adv_data_raw(0u, s_advRawLength, s_advRaw);
            if (err != ESP_OK) {
                sensorarrayBleLogBoot("adv_data", err);
            }
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS) {
            sensorarrayBleStartAdvertising();
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT: {
        esp_err_t err = param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL;
        s_ble.stats.advertising = err == ESP_OK;
        sensorarrayBleLogBoot("adv_start", err);
        break;
    }
    case ESP_GAP_BLE_READ_PHY_COMPLETE_EVT:
        if (param->read_phy.status == ESP_BT_STATUS_SUCCESS) {
            s_ble.stats.txPhy = param->read_phy.tx_phy;
            s_ble.stats.rxPhy = param->read_phy.rx_phy;
        }
        break;
    default:
        break;
    }
}

static void sensorarrayBleGattCallback(esp_gatts_cb_event_t event,
                                       esp_gatt_if_t gattsIf,
                                       esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if (param->reg.status != ESP_GATT_OK) {
            s_ble.stats.initError = ESP_FAIL;
            sensorarrayBleLogBoot("app_register_async", ESP_FAIL);
            break;
        }
        s_ble.gattsIf = gattsIf;
        (void)esp_ble_gap_set_device_name(s_ble.config.deviceName);
        esp_err_t advErr = esp_ble_gap_ext_adv_set_params(0u, &s_advParams);
        if (advErr != ESP_OK) {
            s_ble.stats.initError = advErr;
            sensorarrayBleLogBoot("adv_params", advErr);
        }
        esp_err_t tableErr = esp_ble_gatts_create_attr_tab(s_gattDb, gattsIf, BLE_IDX_COUNT,
                                                           SENSORARRAY_BLE_APP_ID);
        if (tableErr != ESP_OK) {
            s_ble.stats.initError = tableErr;
            sensorarrayBleLogBoot("gatt_table", tableErr);
        }
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK &&
            param->add_attr_tab.num_handle == BLE_IDX_COUNT) {
            memcpy(s_ble.handles, param->add_attr_tab.handles, sizeof(s_ble.handles));
            esp_err_t err = esp_ble_gatts_start_service(s_ble.handles[BLE_IDX_SERVICE]);
            if (err != ESP_OK) {
                s_ble.stats.initError = err;
                sensorarrayBleLogBoot("gatt_start", err);
            }
        } else {
            s_ble.stats.initError = ESP_FAIL;
            sensorarrayBleLogBoot("gatt_table_async", ESP_FAIL);
        }
        break;
    case ESP_GATTS_START_EVT: {
        esp_err_t err = param->start.status == ESP_GATT_OK ? ESP_OK : ESP_FAIL;
        s_ble.stats.gattReady = err == ESP_OK;
        if (err != ESP_OK) {
            s_ble.stats.initError = err;
        }
        sensorarrayBleLogBoot("gatt_start", err);
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        s_ble.stats.connected = true;
        s_ble.stats.advertising = false;
        s_ble.stats.congested = false;
        memset(s_ble.subscribed, 0, sizeof(s_ble.subscribed));
        s_ble.connId = param->connect.conn_id;
        s_ble.gattsIf = gattsIf;
        memcpy(s_ble.remote, param->connect.remote_bda, sizeof(s_ble.remote));
        esp_ble_gap_phy_mask_t phy = s_ble.config.preferPhy2m ?
            ESP_BLE_GAP_PHY_2M_PREF_MASK : ESP_BLE_GAP_PHY_1M_PREF_MASK;
        esp_err_t phyErr = esp_ble_gap_set_preferred_phy(s_ble.remote, 0u, phy, phy,
                                                         ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
        if (phyErr != ESP_OK && s_ble.config.preferPhy2m) {
            printf("BLEBOOT,stage=phy_2m,err=0x%lx,name=%s,fallback=1M\n",
                   (unsigned long)phyErr, esp_err_to_name(phyErr));
            (void)esp_ble_gap_set_preferred_phy(s_ble.remote, 0u,
                                                ESP_BLE_GAP_PHY_1M_PREF_MASK,
                                                ESP_BLE_GAP_PHY_1M_PREF_MASK,
                                                ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
        }
        (void)esp_ble_gap_read_phy(s_ble.remote);
        esp_ble_conn_update_params_t conn = {0};
        memcpy(conn.bda, s_ble.remote, sizeof(conn.bda));
        conn.min_int = 0x06;
        conn.max_int = 0x0C;
        conn.latency = 0u;
        conn.timeout = 400u;
        (void)esp_ble_gap_update_conn_params(&conn);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        s_ble.stats.connected = false;
        s_ble.stats.congested = false;
        memset(s_ble.subscribed, 0, sizeof(s_ble.subscribed));
        sensorarrayBleStartAdvertising();
        break;
    case ESP_GATTS_MTU_EVT:
        s_ble.stats.mtu = param->mtu.mtu;
        break;
    case ESP_GATTS_WRITE_EVT: {
        int channel = sensorarrayBleCccdChannel(param->write.handle);
        if (channel >= 0 && param->write.len == 2u) {
            uint16_t value = (uint16_t)param->write.value[0] |
                             ((uint16_t)param->write.value[1] << 8u);
            s_ble.subscribed[channel] = (value & 0x0003u) != 0u;
        } else if (param->write.handle == s_ble.handles[BLE_IDX_CTRL_RX_VALUE] &&
                   s_ble.controlCallback) {
            s_ble.controlCallback(param->write.value, param->write.len, s_ble.controlContext);
        }
        break;
    }
    case ESP_GATTS_CONF_EVT:
        if (s_ble.confirmWaitTask) {
            xTaskNotifyGive(s_ble.confirmWaitTask);
        }
        break;
    case ESP_GATTS_CONGEST_EVT:
        s_ble.stats.congested = param->congest.congested;
        if (param->congest.congested) {
            s_ble.stats.congestedCount++;
        }
        break;
    default:
        break;
    }
}

static esp_power_level_t sensorarrayBleHighPower(void)
{
    return ESP_PWR_LVL_P9;
}

static esp_err_t sensorarrayBleStep(const char *stage, esp_err_t err)
{
    sensorarrayBleLogBoot(stage, err);
    if (err != ESP_OK) {
        s_ble.stats.initError = err;
        sensorarrayBleLogHeap(stage);
    }
    return err;
}

esp_err_t sensorarrayBleInit(const sensorarrayBleConfig_t *config)
{
    if (!config || config->deviceName[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ble.initialized) {
        return ESP_OK;
    }
    sensorarrayBleControlRxCallback_t controlCallback = s_ble.controlCallback;
    void *controlContext = s_ble.controlContext;
    memset(&s_ble, 0, sizeof(s_ble));
    s_ble.controlCallback = controlCallback;
    s_ble.controlContext = controlContext;
    s_ble.config = *config;
    s_ble.stats.mtu = 23u;
    s_ble.stats.txPhy = ESP_BLE_GAP_PHY_1M;
    s_ble.stats.rxPhy = ESP_BLE_GAP_PHY_1M;
    s_ble.txMode = CONFIG_SENSORARRAY_BLE_TX_MODE_SAFE ?
        SENSORARRAY_BLE_TX_SAFE : SENSORARRAY_BLE_TX_FAST;
    s_ble.txQueue = xQueueCreateStatic(CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN,
                                       sizeof(sensorarrayBleQueuedMessage_t),
                                       s_ble.txQueueStorage,
                                       &s_ble.txQueueStruct);
    if (!s_ble.txQueue) {
        s_ble.stats.initError = ESP_ERR_NO_MEM;
        return ESP_ERR_NO_MEM;
    }
    BaseType_t taskOk = xTaskCreatePinnedToCore(sensorarrayBleTxTask,
                                                "bleTx",
                                                CONFIG_SENSORARRAY_BLE_TX_TASK_STACK,
                                                NULL,
                                                CONFIG_SENSORARRAY_BLE_TX_TASK_PRIORITY,
                                                &s_ble.txTask,
                                                CONFIG_SENSORARRAY_BLE_TX_TASK_CORE);
    if (taskOk != pdPASS || !s_ble.txTask) {
        s_ble.stats.initError = ESP_ERR_NO_MEM;
        return ESP_ERR_NO_MEM;
    }
    s_ble.txTaskStarted = true;
    sensorarrayBleBuildAdvData();

    sensorarrayBleLogHeap("before_ble_classic_release");
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    if (sensorarrayBleStep("classic_release", err) != ESP_OK) {
        return err;
    }
    sensorarrayBleLogHeap("after_ble_classic_release");
    sensorarrayBleLogHeap("before_ble_controller_init");

    esp_bt_controller_config_t controller = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&controller);
    if (sensorarrayBleStep("controller_init", err) != ESP_OK) {
        sensorarrayBleLogHeap("ble_controller_init_fail");
        return err;
    }
    sensorarrayBleLogHeap("after_ble_controller_init");
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (sensorarrayBleStep("controller_enable", err) != ESP_OK) {
        return err;
    }
    s_ble.stats.controllerReady = true;
    sensorarrayBleLogHeap("after_ble_controller_enable");

    err = esp_bluedroid_init();
    if (sensorarrayBleStep("bluedroid_init", err) != ESP_OK) {
        return err;
    }
    err = esp_bluedroid_enable();
    if (sensorarrayBleStep("bluedroid_enable", err) != ESP_OK) {
        return err;
    }
    s_ble.stats.hostReady = true;
    sensorarrayBleLogHeap("after_bluedroid_enable");

    err = esp_ble_gap_register_callback(sensorarrayBleGapCallback);
    if (sensorarrayBleStep("gap_cb", err) != ESP_OK) {
        return err;
    }
    err = esp_ble_gatts_register_callback(sensorarrayBleGattCallback);
    if (sensorarrayBleStep("gatts_cb", err) != ESP_OK) {
        return err;
    }
    err = esp_ble_gatts_app_register(SENSORARRAY_BLE_APP_ID);
    if (sensorarrayBleStep("app_register", err) != ESP_OK) {
        return err;
    }
    err = esp_ble_gatt_set_local_mtu(config->preferredMtu ? config->preferredMtu : 247u);
    sensorarrayBleLogBoot("mtu", err);

    if (config->preferHighTxPower) {
        esp_power_level_t level = sensorarrayBleHighPower();
        (void)esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, level);
        (void)esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, level);
        (void)esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, level);
    }
    if (config->preferPhy2m) {
        esp_err_t phyErr = esp_ble_gap_set_preferred_default_phy(
            ESP_BLE_GAP_PHY_2M_PREF_MASK, ESP_BLE_GAP_PHY_2M_PREF_MASK);
        if (phyErr != ESP_OK) {
            printf("BLEBOOT,stage=phy_2m_default,err=0x%lx,name=%s,fallback=1M\n",
                   (unsigned long)phyErr, esp_err_to_name(phyErr));
            (void)esp_ble_gap_set_preferred_default_phy(ESP_BLE_GAP_PHY_1M_PREF_MASK,
                                                        ESP_BLE_GAP_PHY_1M_PREF_MASK);
        }
    }
    s_ble.initialized = true;
    return ESP_OK;
}

esp_err_t sensorarrayBleNotify(sensorarrayBleChannel_t channel,
                              const uint8_t *data,
                              size_t length)
{
    if (channel > SENSORARRAY_BLE_CH_CTRL || !data || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (length > SENSORARRAY_BLE_MESSAGE_VALUE_MAX) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_SIZE;
    }
    if (!s_ble.stats.gattReady || !s_ble.stats.connected || !s_ble.subscribed[channel]) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_STATE;
    }
    if (!s_ble.txQueue) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_STATE;
    }
    sensorarrayBleQueuedMessage_t message = {
        .channel = channel,
        .length = (uint16_t)length,
    };
    memcpy(message.data, data, length);
    if (xQueueSend(s_ble.txQueue, &message, 0) != pdTRUE) {
        sensorarrayBleQueuedMessage_t discarded;
        if (xQueueReceive(s_ble.txQueue, &discarded, 0) == pdTRUE) {
            sensorarrayBleNoteDrop(discarded.channel);
        }
        if (xQueueSend(s_ble.txQueue, &message, 0) != pdTRUE) {
            sensorarrayBleNoteDrop(channel);
            return ESP_ERR_TIMEOUT;
        }
    }
    s_ble.stats.messageQueued++;
    return ESP_OK;
}

void sensorarrayBleSetControlRxCallback(sensorarrayBleControlRxCallback_t callback,
                                       void *userContext)
{
    s_ble.controlCallback = callback;
    s_ble.controlContext = userContext;
}

bool sensorarrayBleIsReady(void)
{
    return s_ble.stats.controllerReady && s_ble.stats.hostReady &&
           s_ble.stats.gattReady && s_ble.stats.advertising;
}

bool sensorarrayBleIsConnected(void)
{
    return s_ble.stats.connected;
}

bool sensorarrayBleIsSubscribed(sensorarrayBleChannel_t channel)
{
    return channel <= SENSORARRAY_BLE_CH_CTRL && s_ble.subscribed[channel];
}

bool sensorarrayBleIsCongested(void)
{
    return s_ble.stats.congested;
}

void sensorarrayBleGetStats(sensorarrayBleStats_t *outStats)
{
    if (outStats) {
        portENTER_CRITICAL(&s_bleMux);
        *outStats = s_ble.stats;
        portEXIT_CRITICAL(&s_bleMux);
    }
}
