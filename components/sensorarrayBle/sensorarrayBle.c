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
#ifndef CONFIG_SENSORARRAY_BLE_CTRL_TASK_STACK
#define CONFIG_SENSORARRAY_BLE_CTRL_TASK_STACK 6144
#endif
#ifndef CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY
#define CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY 6
#endif
#ifndef CONFIG_SENSORARRAY_BLE_CTRL_TASK_CORE
#define CONFIG_SENSORARRAY_BLE_CTRL_TASK_CORE 0
#endif
#ifndef CONFIG_SENSORARRAY_BLE_CTRL_RX_QUEUE_LEN
#define CONFIG_SENSORARRAY_BLE_CTRL_RX_QUEUE_LEN 4
#endif

#define SENSORARRAY_BLE_APP_ID 0x53u
#define SENSORARRAY_BLE_SERVICE_UUID 0x00FFu
#define SENSORARRAY_BLE_CTRL_RX_UUID 0xFF10u
#define SENSORARRAY_BLE_CTRL_TX_UUID 0xFF11u
#define SENSORARRAY_BLE_DATA_TX_UUID 0xFF20u
#define SENSORARRAY_BLE_LOG_TX_UUID 0xFF30u
#define SENSORARRAY_BLE_ATTR_VALUE_MAX 512u
#define SENSORARRAY_BLE_MESSAGE_VALUE_MAX 1536u
#define SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX 256u
#define SENSORARRAY_BLE_CTRL_RX_MAX 128u
#define SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT 2u
#define SENSORARRAY_BLE_TINY_TAIL_MIN 8u
#define SENSORARRAY_BLE_CONF_TIMEOUT_MS 1000u
#define SENSORARRAY_BLE_HEAP_LOW_WATERMARK 49152u

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
    bool inUse;
    uint32_t generation;
    sensorarrayBleChannel_t channel;
    uint16_t length;
    uint32_t crc;
    uint8_t data[SENSORARRAY_BLE_MESSAGE_VALUE_MAX];
} sensorarrayBleDataSlot_t;

typedef struct {
    bool inUse;
    uint32_t generation;
    sensorarrayBleChannel_t channel;
    uint16_t length;
    uint32_t crc;
    uint8_t data[SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX];
} sensorarrayBleCtrlSlot_t;

typedef struct {
    uint8_t slotIndex;
    uint8_t ctrlSlot;
    sensorarrayBleChannel_t channel;
    uint16_t length;
    uint32_t generation;
    uint32_t connectionGeneration;
    uint32_t crc;
} sensorarrayBleTxDescriptor_t;

typedef struct {
    uint16_t length;
    uint16_t connId;
    uint32_t connectionGeneration;
    uint8_t data[SENSORARRAY_BLE_CTRL_RX_MAX];
} sensorarrayBleControlCommand_t;

_Static_assert(sizeof(sensorarrayBleControlCommand_t) <= 144u,
               "BLE RX command must stay a small BTC-task object");
_Static_assert(sizeof(sensorarrayBleTxDescriptor_t) <= 24u,
               "BLE TX queue must store descriptors, not whole payloads");

typedef struct {
    sensorarrayBleConfig_t config;
    sensorarrayBleStats_t stats;
    bool initialized;
    bool txTaskStarted;
    bool controlTaskStarted;
    bool bootSummaryPrinted;
    bool subscribed[3];
    uint32_t nextMessageId[3];
    uint32_t connectionGeneration;
    esp_gatt_if_t gattsIf;
    uint16_t connId;
    uint16_t handles[BLE_IDX_COUNT];
    QueueHandle_t txQueue;
    QueueHandle_t ctrlTxQueue;
    QueueHandle_t ctrlRxQueue;
    StaticQueue_t txQueueStruct;
    StaticQueue_t ctrlTxQueueStruct;
    StaticQueue_t ctrlRxQueueStruct;
    uint8_t txQueueStorage[CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN *
                           sizeof(sensorarrayBleTxDescriptor_t)];
    uint8_t ctrlTxQueueStorage[SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT *
                               sizeof(sensorarrayBleTxDescriptor_t)];
    uint8_t ctrlRxQueueStorage[CONFIG_SENSORARRAY_BLE_CTRL_RX_QUEUE_LEN *
                               sizeof(sensorarrayBleControlCommand_t)];
    sensorarrayBleDataSlot_t dataSlots[CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN];
    sensorarrayBleCtrlSlot_t ctrlSlots[SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT];
    TaskHandle_t txTask;
    TaskHandle_t controlTask;
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

static uint32_t sensorarrayBleNextGeneration(uint32_t value)
{
    value++;
    return value == 0u ? 1u : value;
}

static uint32_t sensorarrayBleCountUsedSlotsLocked(void)
{
    uint32_t used = 0u;
    for (uint8_t i = 0u; i < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN; ++i) {
        used += s_ble.dataSlots[i].inUse ? 1u : 0u;
    }
    for (uint8_t i = 0u; i < SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT; ++i) {
        used += s_ble.ctrlSlots[i].inUse ? 1u : 0u;
    }
    return used;
}

static void sensorarrayBleUpdateSlotUseLocked(void)
{
    uint32_t used = sensorarrayBleCountUsedSlotsLocked();
    s_ble.stats.txSlotUsed = used;
    if (used > s_ble.stats.txSlotHighWater) {
        s_ble.stats.txSlotHighWater = used;
    }
}

static bool sensorarrayBleAllocateTxDescriptor(sensorarrayBleChannel_t channel,
                                               size_t length,
                                               uint32_t crc,
                                               sensorarrayBleTxDescriptor_t *outDesc)
{
    if (!outDesc || channel > SENSORARRAY_BLE_CH_CTRL || length == 0u) {
        return false;
    }
    if (channel == SENSORARRAY_BLE_CH_CTRL &&
        length > SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX) {
        return false;
    }

    memset(outDesc, 0, sizeof(*outDesc));
    portENTER_CRITICAL(&s_bleMux);
    if (channel == SENSORARRAY_BLE_CH_CTRL) {
        for (uint8_t i = 0u; i < SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT; ++i) {
            sensorarrayBleCtrlSlot_t *slot = &s_ble.ctrlSlots[i];
            if (slot->inUse) {
                continue;
            }
            slot->inUse = true;
            slot->channel = channel;
            slot->length = (uint16_t)length;
            slot->crc = crc;
            slot->generation = sensorarrayBleNextGeneration(slot->generation);
            *outDesc = (sensorarrayBleTxDescriptor_t){
                .slotIndex = i,
                .ctrlSlot = 1u,
                .channel = channel,
                .length = (uint16_t)length,
                .generation = slot->generation,
                .connectionGeneration = s_ble.connectionGeneration,
                .crc = crc,
            };
            sensorarrayBleUpdateSlotUseLocked();
            portEXIT_CRITICAL(&s_bleMux);
            return true;
        }
    } else {
        for (uint8_t i = 0u; i < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN; ++i) {
            sensorarrayBleDataSlot_t *slot = &s_ble.dataSlots[i];
            if (slot->inUse) {
                continue;
            }
            slot->inUse = true;
            slot->channel = channel;
            slot->length = (uint16_t)length;
            slot->crc = crc;
            slot->generation = sensorarrayBleNextGeneration(slot->generation);
            *outDesc = (sensorarrayBleTxDescriptor_t){
                .slotIndex = i,
                .ctrlSlot = 0u,
                .channel = channel,
                .length = (uint16_t)length,
                .generation = slot->generation,
                .connectionGeneration = s_ble.connectionGeneration,
                .crc = crc,
            };
            sensorarrayBleUpdateSlotUseLocked();
            portEXIT_CRITICAL(&s_bleMux);
            return true;
        }
    }
    s_ble.stats.txSlotAllocFail++;
    portEXIT_CRITICAL(&s_bleMux);
    return false;
}

static uint8_t *sensorarrayBleDescriptorData(const sensorarrayBleTxDescriptor_t *desc)
{
    if (!desc) {
        return NULL;
    }
    if (desc->ctrlSlot) {
        return desc->slotIndex < SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT ?
            s_ble.ctrlSlots[desc->slotIndex].data : NULL;
    }
    return desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN ?
        s_ble.dataSlots[desc->slotIndex].data : NULL;
}

static bool sensorarrayBleDescriptorCurrent(const sensorarrayBleTxDescriptor_t *desc)
{
    if (!desc) {
        return false;
    }
    bool current = false;
    portENTER_CRITICAL(&s_bleMux);
    if (desc->ctrlSlot && desc->slotIndex < SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT) {
        const sensorarrayBleCtrlSlot_t *slot = &s_ble.ctrlSlots[desc->slotIndex];
        current = slot->inUse &&
                  slot->generation == desc->generation &&
                  slot->channel == desc->channel &&
                  slot->length == desc->length &&
                  slot->crc == desc->crc &&
                  desc->connectionGeneration == s_ble.connectionGeneration;
    } else if (!desc->ctrlSlot && desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN) {
        const sensorarrayBleDataSlot_t *slot = &s_ble.dataSlots[desc->slotIndex];
        current = slot->inUse &&
                  slot->generation == desc->generation &&
                  slot->channel == desc->channel &&
                  slot->length == desc->length &&
                  slot->crc == desc->crc &&
                  desc->connectionGeneration == s_ble.connectionGeneration;
    }
    portEXIT_CRITICAL(&s_bleMux);
    return current;
}

static void sensorarrayBleReleaseTxDescriptor(const sensorarrayBleTxDescriptor_t *desc)
{
    if (!desc) {
        return;
    }
    bool released = false;
    portENTER_CRITICAL(&s_bleMux);
    if (desc->ctrlSlot && desc->slotIndex < SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT) {
        sensorarrayBleCtrlSlot_t *slot = &s_ble.ctrlSlots[desc->slotIndex];
        if (slot->inUse && slot->generation == desc->generation) {
            slot->inUse = false;
            released = true;
        }
    } else if (!desc->ctrlSlot && desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN) {
        sensorarrayBleDataSlot_t *slot = &s_ble.dataSlots[desc->slotIndex];
        if (slot->inUse && slot->generation == desc->generation) {
            slot->inUse = false;
            released = true;
        }
    }
    if (!released) {
        s_ble.stats.txSlotReleaseMismatch++;
    }
    sensorarrayBleUpdateSlotUseLocked();
    portEXIT_CRITICAL(&s_bleMux);
}

static void sensorarrayBleLogHeapDetail(const char *stage, const char *reason)
{
    printf("M,stage=%s,reason=%s,ih=%u,il=%u,im=%u,dh=%u,dl=%u,h8=%u,l8=%u\n",
           stage ? stage : "unknown",
           reason ? reason : "diag",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

static void sensorarrayBleLogBoot(const char *stage, esp_err_t err)
{
    printf("BLEBOOT,stage=%s,err=0x%lx,name=%s\n",
           stage,
           (unsigned long)err,
           esp_err_to_name(err));
}

void sensorarrayBleLogHeap(const char *stage)
{
    uint32_t internalFree = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    if (internalFree < SENSORARRAY_BLE_HEAP_LOW_WATERMARK) {
        sensorarrayBleLogHeapDetail(stage, "low_heap");
    }
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

static esp_err_t sensorarrayBleSendPayload(sensorarrayBleChannel_t channel,
                                           const uint8_t *data,
                                           uint16_t length,
                                           uint32_t enqueueCrc)
{
    if (!data || channel > SENSORARRAY_BLE_CH_CTRL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_ble.stats.gattReady || !s_ble.stats.connected ||
        !s_ble.subscribed[channel]) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ble.stats.congested && sensorarrayBleGetTxMode() == SENSORARRAY_BLE_TX_FAST) {
        s_ble.stats.congestedCount++;
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_TIMEOUT;
    }
    size_t maximum = s_ble.stats.mtu > 3u ? (size_t)s_ble.stats.mtu - 3u : 20u;
    if (maximum > SENSORARRAY_BLE_ATTR_VALUE_MAX) {
        maximum = SENSORARRAY_BLE_ATTR_VALUE_MAX;
    }
    char envChannel = sensorarrayBleEnvelopeChannel(channel);
    uint32_t messageId = ++s_ble.nextMessageId[channel];
    if (messageId == 0u) {
        messageId = ++s_ble.nextMessageId[channel];
    }
    uint32_t crc = sensorarrayBleCrc32(data, length);
    if (crc != enqueueCrc) {
        s_ble.stats.txCrcMismatch++;
        printf("BLECORRUPT,ch=%c,mid=%lu,enqLen=%u,deqLen=%u,enqCrc=%08lX,deqCrc=%08lX\n",
               envChannel,
               (unsigned long)messageId,
               (unsigned)length,
               (unsigned)length,
               (unsigned long)enqueueCrc,
               (unsigned long)crc);
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_CRC;
    }
    if (!sensorarrayBleValidateAscii(channel,
                                     data,
                                     length,
                                     messageId,
                                     0u)) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_ARG;
    }
    if (length <= maximum) {
        uint16_t handle = sensorarrayBleValueHandle(channel);
        bool confirm = sensorarrayBleGetTxMode() == SENSORARRAY_BLE_TX_SAFE;
        if (confirm) {
            s_ble.confirmWaitTask = xTaskGetCurrentTaskHandle();
            (void)xTaskNotifyStateClear(s_ble.confirmWaitTask);
        }
        esp_err_t err = esp_ble_gatts_send_indicate(s_ble.gattsIf,
                                                    s_ble.connId,
                                                    handle,
                                                    length,
                                                    (uint8_t *)data,
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
            s_ble.stats.sent[channel]++;
            s_ble.stats.messageSent++;
        } else {
            s_ble.stats.fragmentError++;
            sensorarrayBleNoteDrop(channel);
        }
        return err;
    }

    uint32_t fragmentCount = 0u;
    if (!sensorarrayBlePlanFragments(length,
                                     maximum,
                                     envChannel,
                                     messageId,
                                     crc,
                                     &fragmentCount) ||
        fragmentCount == 0u) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_SIZE;
    }

    size_t base = length / fragmentCount;
    size_t extra = length % fragmentCount;
    size_t offset = 0u;
    bool confirm = sensorarrayBleGetTxMode() == SENSORARRAY_BLE_TX_SAFE;
    for (uint32_t index = 0u; index < fragmentCount; ++index) {
        size_t chunk = base + (index < extra ? 1u : 0u);
        esp_err_t err = sensorarrayBleSendFragment(channel,
                                                   envChannel,
                                                   messageId,
                                                   index,
                                                   fragmentCount,
                                                   data + offset,
                                                   chunk,
                                                   length,
                                                   crc,
                                                   confirm);
        if (err != ESP_OK) {
            sensorarrayBleNoteDrop(channel);
            return err;
        }
        offset += chunk;
    }
    s_ble.stats.sent[channel]++;
    s_ble.stats.messageSent++;
    return ESP_OK;
}

static esp_err_t sensorarrayBleSendDescriptor(const sensorarrayBleTxDescriptor_t *desc)
{
    if (!desc) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayBleDescriptorCurrent(desc)) {
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.txSlotStaleGenerationDrop++;
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleNoteDrop(desc->channel);
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t *data = sensorarrayBleDescriptorData(desc);
    if (!data) {
        sensorarrayBleNoteDrop(desc->channel);
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayBleSendPayload(desc->channel, data, desc->length, desc->crc);
}

static void sensorarrayBleTxTask(void *arg)
{
    (void)arg;
    sensorarrayBleTxDescriptor_t desc;
    for (;;) {
        if (s_ble.ctrlTxQueue &&
            xQueueReceive(s_ble.ctrlTxQueue, &desc, 0) == pdTRUE) {
            (void)sensorarrayBleSendDescriptor(&desc);
            sensorarrayBleReleaseTxDescriptor(&desc);
            continue;
        }
        if (xQueueReceive(s_ble.txQueue, &desc, pdMS_TO_TICKS(20u)) == pdTRUE) {
            (void)sensorarrayBleSendDescriptor(&desc);
            sensorarrayBleReleaseTxDescriptor(&desc);
        }
    }
}

static esp_err_t sensorarrayBleQueueTxDescriptor(sensorarrayBleTxDescriptor_t *desc)
{
    if (!desc) {
        return ESP_ERR_INVALID_ARG;
    }
    QueueHandle_t queue = desc->ctrlSlot ? s_ble.ctrlTxQueue : s_ble.txQueue;
    if (!queue) {
        sensorarrayBleReleaseTxDescriptor(desc);
        sensorarrayBleNoteDrop(desc->channel);
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(queue, desc, 0) == pdTRUE) {
        s_ble.stats.messageQueued++;
        return ESP_OK;
    }

    if (!desc->ctrlSlot) {
        sensorarrayBleTxDescriptor_t discarded;
        if (xQueueReceive(queue, &discarded, 0) == pdTRUE) {
            sensorarrayBleNoteDrop(discarded.channel);
            sensorarrayBleReleaseTxDescriptor(&discarded);
        }
        if (xQueueSend(queue, desc, 0) == pdTRUE) {
            s_ble.stats.messageQueued++;
            return ESP_OK;
        }
    }

    sensorarrayBleReleaseTxDescriptor(desc);
    sensorarrayBleNoteDrop(desc->channel);
    return ESP_ERR_TIMEOUT;
}

static void sensorarrayBleDrainTxQueues(void)
{
    sensorarrayBleTxDescriptor_t desc;
    while (s_ble.ctrlTxQueue &&
           xQueueReceive(s_ble.ctrlTxQueue, &desc, 0) == pdTRUE) {
        sensorarrayBleNoteDrop(desc.channel);
        sensorarrayBleReleaseTxDescriptor(&desc);
    }
    while (s_ble.txQueue &&
           xQueueReceive(s_ble.txQueue, &desc, 0) == pdTRUE) {
        sensorarrayBleNoteDrop(desc.channel);
        sensorarrayBleReleaseTxDescriptor(&desc);
    }
}

static void sensorarrayBleDrainControlRxQueue(void)
{
    sensorarrayBleControlCommand_t command;
    while (s_ble.ctrlRxQueue &&
           xQueueReceive(s_ble.ctrlRxQueue, &command, 0) == pdTRUE) {
        s_ble.stats.controlRxStale++;
    }
}

static void sensorarrayBleControlTask(void *arg)
{
    (void)arg;
    sensorarrayBleControlCommand_t command;
    for (;;) {
        if (xQueueReceive(s_ble.ctrlRxQueue, &command, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        bool current = false;
        sensorarrayBleControlRxCallback_t callback = NULL;
        void *callbackContext = NULL;
        portENTER_CRITICAL(&s_bleMux);
        current = s_ble.stats.connected &&
                  command.connId == s_ble.connId &&
                  command.connectionGeneration == s_ble.connectionGeneration;
        callback = s_ble.controlCallback;
        callbackContext = s_ble.controlContext;
        if (!current) {
            s_ble.stats.controlRxStale++;
        }
        portEXIT_CRITICAL(&s_bleMux);
        if (current && callback) {
            callback(command.data, command.length, callbackContext);
        }
    }
}

static void sensorarrayBleQueueControlCommand(const esp_ble_gatts_cb_param_t *param)
{
    if (!param || param->write.handle != s_ble.handles[BLE_IDX_CTRL_RX_VALUE]) {
        return;
    }
    if (!s_ble.ctrlRxQueue || !s_ble.controlCallback) {
        s_ble.stats.controlRxDropped++;
        return;
    }
    if (param->write.len == 0u || param->write.len > SENSORARRAY_BLE_CTRL_RX_MAX) {
        s_ble.stats.controlRxDropped++;
        printf("BLERXERR,src=ctrl,len=%u,max=%u,reason=length\n",
               (unsigned)param->write.len,
               (unsigned)SENSORARRAY_BLE_CTRL_RX_MAX);
        return;
    }
    sensorarrayBleControlCommand_t command;
    command.length = param->write.len;
    command.connId = param->write.conn_id;
    command.connectionGeneration = s_ble.connectionGeneration;
    memcpy(command.data, param->write.value, param->write.len);
    if (xQueueSend(s_ble.ctrlRxQueue, &command, 0) == pdTRUE) {
        s_ble.stats.controlRxQueued++;
    } else {
        s_ble.stats.controlRxDropped++;
        printf("BLERXERR,src=ctrl,len=%u,reason=rx_queue_full\n",
               (unsigned)param->write.len);
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

static void sensorarrayBleLogReadySummary(void)
{
    if (s_ble.bootSummaryPrinted ||
        !s_ble.stats.controllerReady ||
        !s_ble.stats.hostReady ||
        !s_ble.stats.gattReady ||
        !s_ble.stats.advertising) {
        return;
    }
    s_ble.bootSummaryPrinted = true;
    printf("BLEBOOT,status=ok,controller=%u,host=%u,gatt=%u,adv=%u,mtu=%u,phyReq=%s,txMode=%s,ctrlRxMax=%u,msgMax=%u\n",
           s_ble.stats.controllerReady ? 1u : 0u,
           s_ble.stats.hostReady ? 1u : 0u,
           s_ble.stats.gattReady ? 1u : 0u,
           s_ble.stats.advertising ? 1u : 0u,
           s_ble.config.preferredMtu ? s_ble.config.preferredMtu : s_ble.stats.mtu,
           s_ble.config.preferPhy2m ? "2M" : "1M",
           sensorarrayBleTxModeName(s_ble.txMode),
           (unsigned)SENSORARRAY_BLE_CTRL_RX_MAX,
           (unsigned)SENSORARRAY_BLE_MESSAGE_VALUE_MAX);
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
        } else {
            s_ble.stats.initError = ESP_FAIL;
            sensorarrayBleLogBoot("adv_params_async", ESP_FAIL);
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS) {
            sensorarrayBleStartAdvertising();
        } else {
            s_ble.stats.initError = ESP_FAIL;
            sensorarrayBleLogBoot("adv_data_async", ESP_FAIL);
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT: {
        esp_err_t err = param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL;
        s_ble.stats.advertising = err == ESP_OK;
        if (err == ESP_OK) {
            sensorarrayBleLogReadySummary();
        } else {
            s_ble.stats.initError = err;
            sensorarrayBleLogBoot("adv_start", err);
        }
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
            sensorarrayBleLogBoot("gatt_start", err);
        }
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        s_ble.stats.connected = true;
        s_ble.stats.advertising = false;
        s_ble.stats.congested = false;
        memset(s_ble.subscribed, 0, sizeof(s_ble.subscribed));
        s_ble.connId = param->connect.conn_id;
        s_ble.connectionGeneration = sensorarrayBleNextGeneration(s_ble.connectionGeneration);
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
        s_ble.connectionGeneration = sensorarrayBleNextGeneration(s_ble.connectionGeneration);
        memset(s_ble.subscribed, 0, sizeof(s_ble.subscribed));
        sensorarrayBleDrainControlRxQueue();
        sensorarrayBleDrainTxQueues();
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
            sensorarrayBleQueueControlCommand(param);
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
    if (err != ESP_OK) {
        s_ble.stats.initError = err;
        sensorarrayBleLogBoot(stage, err);
        sensorarrayBleLogHeapDetail(stage, "init_fail");
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
                                       sizeof(sensorarrayBleTxDescriptor_t),
                                       s_ble.txQueueStorage,
                                       &s_ble.txQueueStruct);
    s_ble.ctrlTxQueue = xQueueCreateStatic(SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT,
                                           sizeof(sensorarrayBleTxDescriptor_t),
                                           s_ble.ctrlTxQueueStorage,
                                           &s_ble.ctrlTxQueueStruct);
    s_ble.ctrlRxQueue = xQueueCreateStatic(CONFIG_SENSORARRAY_BLE_CTRL_RX_QUEUE_LEN,
                                           sizeof(sensorarrayBleControlCommand_t),
                                           s_ble.ctrlRxQueueStorage,
                                           &s_ble.ctrlRxQueueStruct);
    if (!s_ble.txQueue || !s_ble.ctrlTxQueue || !s_ble.ctrlRxQueue) {
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
    taskOk = xTaskCreatePinnedToCore(sensorarrayBleControlTask,
                                     "bleCtrl",
                                     CONFIG_SENSORARRAY_BLE_CTRL_TASK_STACK,
                                     NULL,
                                     CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY,
                                     &s_ble.controlTask,
                                     CONFIG_SENSORARRAY_BLE_CTRL_TASK_CORE);
    if (taskOk != pdPASS || !s_ble.controlTask) {
        s_ble.stats.initError = ESP_ERR_NO_MEM;
        return ESP_ERR_NO_MEM;
    }
    s_ble.controlTaskStarted = true;
    sensorarrayBleBuildAdvData();

    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    if (sensorarrayBleStep("classic_release", err) != ESP_OK) {
        return err;
    }

    esp_bt_controller_config_t controller = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&controller);
    if (sensorarrayBleStep("controller_init", err) != ESP_OK) {
        return err;
    }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (sensorarrayBleStep("controller_enable", err) != ESP_OK) {
        return err;
    }
    s_ble.stats.controllerReady = true;

    err = esp_bluedroid_init();
    if (sensorarrayBleStep("bluedroid_init", err) != ESP_OK) {
        return err;
    }
    err = esp_bluedroid_enable();
    if (sensorarrayBleStep("bluedroid_enable", err) != ESP_OK) {
        return err;
    }
    s_ble.stats.hostReady = true;

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
    if (err != ESP_OK) {
        sensorarrayBleLogBoot("mtu", err);
    }

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
    if (channel == SENSORARRAY_BLE_CH_CTRL &&
        length > SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_SIZE;
    }
    uint32_t crc = sensorarrayBleCrc32(data, length);
    sensorarrayBleTxDescriptor_t desc;
    if (!sensorarrayBleAllocateTxDescriptor(channel, length, crc, &desc)) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_TIMEOUT;
    }
    uint8_t *slotData = sensorarrayBleDescriptorData(&desc);
    if (!slotData) {
        sensorarrayBleReleaseTxDescriptor(&desc);
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_STATE;
    }
    memcpy(slotData, data, length);
    return sensorarrayBleQueueTxDescriptor(&desc);
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
