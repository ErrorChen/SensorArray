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
#define SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX 512u
#define SENSORARRAY_BLE_CTRL_RX_MAX 128u
#define SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT 2u
#define SENSORARRAY_BLE_LOG_TX_SLOT_COUNT 2u
#define SENSORARRAY_BLE_TX_DESCRIPTOR_QUEUE_COUNT \
    (CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN + SENSORARRAY_BLE_LOG_TX_SLOT_COUNT)
#define SENSORARRAY_BLE_TINY_TAIL_MIN 8u
#define SENSORARRAY_BLE_CONF_TIMEOUT_MS 1000u
#define SENSORARRAY_BLE_DEFAULT_ATT_MTU 23u
#define SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS 10u
#define SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS 500u
#define SENSORARRAY_BLE_HEAP_LOW_WATERMARK 49152u
#define SENSORARRAY_BLE_CCCD_NOTIFY_BIT 0x0001u
#define SENSORARRAY_BLE_CCCD_INDICATE_BIT 0x0002u
#define SENSORARRAY_BLE_CCCD_SUPPORTED_MASK \
    (SENSORARRAY_BLE_CCCD_NOTIFY_BIT | SENSORARRAY_BLE_CCCD_INDICATE_BIT)
#define SENSORARRAY_BLE_CCCD_ALLOWS_MODE(cccdValue, txModeValue) \
    ((((uint16_t)(cccdValue)) & \
      ((txModeValue) == SENSORARRAY_BLE_TX_SAFE ? \
           SENSORARRAY_BLE_CCCD_INDICATE_BIT : \
           SENSORARRAY_BLE_CCCD_NOTIFY_BIT)) != 0u)

_Static_assert(SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS > 0u &&
                   SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS >=
                       SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS,
               "BLE CTRL retry window must be finite and non-zero");

_Static_assert(!SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0000u,
                                                  SENSORARRAY_BLE_TX_FAST) &&
                   !SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0000u,
                                                     SENSORARRAY_BLE_TX_SAFE),
               "disabled CCCD must reject FAST and SAFE sends");
_Static_assert(SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0001u,
                                                 SENSORARRAY_BLE_TX_FAST) &&
                   !SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0001u,
                                                     SENSORARRAY_BLE_TX_SAFE),
               "notify-only CCCD must allow FAST only");
_Static_assert(!SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0002u,
                                                  SENSORARRAY_BLE_TX_FAST) &&
                   SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0002u,
                                                    SENSORARRAY_BLE_TX_SAFE),
               "indicate-only CCCD must allow SAFE only");
_Static_assert(SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0003u,
                                                 SENSORARRAY_BLE_TX_FAST) &&
                   SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0003u,
                                                    SENSORARRAY_BLE_TX_SAFE),
               "both CCCD bits must allow FAST and SAFE sends");

/*
 * Fragment header worst case (not including the terminating NUL):
 *
 * G,<channel>,<messageId>,<fragmentIndex>,<fragmentCount>,<chunkLength>,
 * <messageLength>,<crc>\n
 *
 * There are five decimal uint32_t values, one eight-digit CRC, one channel,
 * and the fixed "G"/comma/newline characters.  80 bytes therefore leaves
 * eleven bytes beyond the required 69 bytes including the terminating NUL.
 */
#define SENSORARRAY_BLE_U32_DECIMAL_MAX_CHARS 10u
#define SENSORARRAY_BLE_U32_HEX_MAX_CHARS 8u
#define SENSORARRAY_BLE_FRAGMENT_HEADER_FIXED_CHARS \
    (sizeof("G,,,,,,,\n") - 1u)
#define SENSORARRAY_BLE_FRAGMENT_HEADER_MAX_CHARS \
    (SENSORARRAY_BLE_FRAGMENT_HEADER_FIXED_CHARS + 1u + \
     (5u * SENSORARRAY_BLE_U32_DECIMAL_MAX_CHARS) + \
     SENSORARRAY_BLE_U32_HEX_MAX_CHARS)
#define SENSORARRAY_BLE_FRAGMENT_HEADER_BUFFER_SIZE 80u

_Static_assert(SENSORARRAY_BLE_FRAGMENT_HEADER_MAX_CHARS == 68u,
               "BLE fragment header bound calculation changed");
_Static_assert(SENSORARRAY_BLE_FRAGMENT_HEADER_BUFFER_SIZE >=
                   SENSORARRAY_BLE_FRAGMENT_HEADER_MAX_CHARS + 1u,
               "BLE fragment header buffer must include the terminating NUL");
_Static_assert(SENSORARRAY_BLE_FRAGMENT_HEADER_BUFFER_SIZE <
                   SENSORARRAY_BLE_ATTR_VALUE_MAX,
               "BLE fragment planner header must remain a small stack object");

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

/* Immutable connection data captured while holding s_bleMux.  A TX
 * descriptor may use only this connection; it must never re-read a newer
 * global connId after a disconnect/reconnect race. */
typedef struct {
    uint32_t connectionGeneration;
    esp_gatt_if_t gattsIf;
    uint16_t connId;
    uint16_t mtu;
    sensorarrayBleTxMode_t txMode;
    bool congested;
} sensorarrayBleSendContext_t;

_Static_assert(sizeof(sensorarrayBleControlCommand_t) <= 144u,
               "BLE RX command must stay a small BTC-task object");
_Static_assert(sizeof(sensorarrayBleTxDescriptor_t) <= 24u,
               "BLE TX queue must store descriptors, not whole payloads");
_Static_assert(SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX >= 512u,
               "BLE CTRL TX must carry the longest command response");
_Static_assert(SENSORARRAY_BLE_LOG_TX_SLOT_COUNT >= 2u,
               "BLE LOG pool must hold compact-summary and ADS messages");

typedef struct {
    sensorarrayBleConfig_t config;
    sensorarrayBleStats_t stats;
    bool initialized;
    bool txTaskStarted;
    bool controlTaskStarted;
    bool bootSummaryPrinted;
    uint16_t cccd[3];
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
    uint8_t txQueueStorage[SENSORARRAY_BLE_TX_DESCRIPTOR_QUEUE_COUNT *
                           sizeof(sensorarrayBleTxDescriptor_t)];
    uint8_t ctrlTxQueueStorage[SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT *
                               sizeof(sensorarrayBleTxDescriptor_t)];
    uint8_t ctrlRxQueueStorage[CONFIG_SENSORARRAY_BLE_CTRL_RX_QUEUE_LEN *
                               sizeof(sensorarrayBleControlCommand_t)];
    sensorarrayBleDataSlot_t dataSlots[CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN];
    sensorarrayBleDataSlot_t logSlots[SENSORARRAY_BLE_LOG_TX_SLOT_COUNT];
    sensorarrayBleCtrlSlot_t ctrlSlots[SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT];
    /*
     * Owned exclusively by sensorarrayBleTxTask.  The Bluedroid API deep-copies
     * indication/notification values before returning, so the single TX task
     * can safely reuse this buffer for the next fragment without keeping a
     * 512-byte automatic array on its stack.
     */
    uint8_t txPacketScratch[SENSORARRAY_BLE_ATTR_VALUE_MAX];
    TaskHandle_t txTask;
    TaskHandle_t controlTask;
    TaskHandle_t confirmWaitTask;
    uint32_t confirmConnectionGeneration;
    uint16_t confirmConnId;
    uint16_t confirmHandle;
    esp_gatt_status_t confirmStatus;
    bool confirmPending;
    sensorarrayBleTxMode_t txMode;
    esp_bd_addr_t remote;
    sensorarrayBleControlRxCallback_t controlCallback;
    void *controlContext;
} sensorarrayBleState_t;

_Static_assert(sizeof(((sensorarrayBleState_t *)0)->txPacketScratch) ==
                   SENSORARRAY_BLE_ATTR_VALUE_MAX,
               "BLE TX scratch must hold one maximum ATT value");

static sensorarrayBleState_t s_ble;
static portMUX_TYPE s_bleMux = portMUX_INITIALIZER_UNLOCKED;

/*
 * Statistics are observed by sensorarrayLogTask while producers, bleTx and
 * the Bluedroid callbacks can update them on either core.  Keep every shared
 * read/modify/write in this short critical section; callers must never hold
 * s_bleMux when invoking these helpers.  Payload copies, formatting and GATT
 * calls deliberately remain outside the critical section.
 */
static void sensorarrayBleStatsIncrement(uint32_t *counter)
{
    if (!counter) {
        return;
    }
    portENTER_CRITICAL(&s_bleMux);
    (*counter)++;
    portEXIT_CRITICAL(&s_bleMux);
}

static void sensorarrayBleStatsNoteDrop(sensorarrayBleChannel_t channel)
{
    portENTER_CRITICAL(&s_bleMux);
    if (channel <= SENSORARRAY_BLE_CH_CTRL) {
        s_ble.stats.dropped[channel]++;
    }
    s_ble.stats.messageDropped++;
    portEXIT_CRITICAL(&s_bleMux);
}

static void sensorarrayBleStatsNoteMessageSent(sensorarrayBleChannel_t channel,
                                               bool countFragment)
{
    portENTER_CRITICAL(&s_bleMux);
    if (countFragment) {
        s_ble.stats.fragmentSent++;
    }
    if (channel <= SENSORARRAY_BLE_CH_CTRL) {
        s_ble.stats.sent[channel]++;
    }
    s_ble.stats.messageSent++;
    portEXIT_CRITICAL(&s_bleMux);
}

static void sensorarrayBleStatsSetInitError(esp_err_t err)
{
    portENTER_CRITICAL(&s_bleMux);
    s_ble.stats.initError = err;
    portEXIT_CRITICAL(&s_bleMux);
}

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
    for (uint8_t i = 0u; i < SENSORARRAY_BLE_LOG_TX_SLOT_COUNT; ++i) {
        used += s_ble.logSlots[i].inUse ? 1u : 0u;
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

static esp_err_t sensorarrayBleAllocateTxDescriptor(
    sensorarrayBleChannel_t channel,
    size_t length,
    uint32_t crc,
    uint32_t expectedConnectionGeneration,
    sensorarrayBleTxDescriptor_t *outDesc)
{
    if (!outDesc || channel > SENSORARRAY_BLE_CH_CTRL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (channel == SENSORARRAY_BLE_CH_CTRL &&
        length > SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(outDesc, 0, sizeof(*outDesc));
    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleTxMode_t txMode = s_ble.txMode;
    uint32_t connectionGeneration = s_ble.connectionGeneration;
    bool sendable = s_ble.stats.gattReady &&
                    s_ble.stats.connected &&
                    SENSORARRAY_BLE_CCCD_ALLOWS_MODE(s_ble.cccd[channel], txMode) &&
                    (expectedConnectionGeneration == 0u ||
                     expectedConnectionGeneration == connectionGeneration);
    if (!sendable) {
        if (expectedConnectionGeneration != 0u &&
            expectedConnectionGeneration != connectionGeneration) {
            s_ble.stats.txConnectionStaleDrop++;
        }
        portEXIT_CRITICAL(&s_bleMux);
        return ESP_ERR_INVALID_STATE;
    }
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
                .connectionGeneration = connectionGeneration,
                .crc = crc,
            };
            sensorarrayBleUpdateSlotUseLocked();
            portEXIT_CRITICAL(&s_bleMux);
            return ESP_OK;
        }
    } else {
        /*
         * SAFE mode keeps a payload slot until every fragment indication is
         * confirmed.  DATA production is faster than SAFE transmission and
         * must not consume the storage needed by FF30.  Keep the original
         * DATA capacity intact and give LOG two independent bounded slots:
         * compact-summary and ADS packets are published back-to-back.  The
         * shared descriptor FIFO preserves message ordering, while CTRL keeps
         * its independent higher-priority queue.
         */
        sensorarrayBleDataSlot_t *slots =
            channel == SENSORARRAY_BLE_CH_DATA ?
                s_ble.dataSlots : s_ble.logSlots;
        uint8_t slotCount = channel == SENSORARRAY_BLE_CH_DATA ?
            CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN :
            SENSORARRAY_BLE_LOG_TX_SLOT_COUNT;
        for (uint8_t i = 0u; i < slotCount; ++i) {
            sensorarrayBleDataSlot_t *slot = &slots[i];
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
                .connectionGeneration = connectionGeneration,
                .crc = crc,
            };
            sensorarrayBleUpdateSlotUseLocked();
            portEXIT_CRITICAL(&s_bleMux);
            return ESP_OK;
        }
    }
    s_ble.stats.txSlotAllocFail++;
    portEXIT_CRITICAL(&s_bleMux);
    return ESP_ERR_NO_MEM;
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
    if (desc->channel == SENSORARRAY_BLE_CH_DATA) {
        return desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN ?
            s_ble.dataSlots[desc->slotIndex].data : NULL;
    }
    return desc->channel == SENSORARRAY_BLE_CH_LOG &&
           desc->slotIndex < SENSORARRAY_BLE_LOG_TX_SLOT_COUNT ?
        s_ble.logSlots[desc->slotIndex].data : NULL;
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
    } else if (!desc->ctrlSlot &&
               desc->channel == SENSORARRAY_BLE_CH_DATA &&
               desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN) {
        const sensorarrayBleDataSlot_t *slot = &s_ble.dataSlots[desc->slotIndex];
        current = slot->inUse &&
                  slot->generation == desc->generation &&
                  slot->channel == desc->channel &&
                  slot->length == desc->length &&
                  slot->crc == desc->crc &&
                  desc->connectionGeneration == s_ble.connectionGeneration;
    } else if (!desc->ctrlSlot &&
               desc->channel == SENSORARRAY_BLE_CH_LOG &&
               desc->slotIndex < SENSORARRAY_BLE_LOG_TX_SLOT_COUNT) {
        const sensorarrayBleDataSlot_t *slot = &s_ble.logSlots[desc->slotIndex];
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
    } else if (!desc->ctrlSlot &&
               desc->channel == SENSORARRAY_BLE_CH_DATA &&
               desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN) {
        sensorarrayBleDataSlot_t *slot = &s_ble.dataSlots[desc->slotIndex];
        if (slot->inUse && slot->generation == desc->generation) {
            slot->inUse = false;
            released = true;
        }
    } else if (!desc->ctrlSlot &&
               desc->channel == SENSORARRAY_BLE_CH_LOG &&
               desc->slotIndex < SENSORARRAY_BLE_LOG_TX_SLOT_COUNT) {
        sensorarrayBleDataSlot_t *slot = &s_ble.logSlots[desc->slotIndex];
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
    sensorarrayBleStatsNoteDrop(channel);
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

static bool sensorarrayBleSnapshotSendContext(
    sensorarrayBleChannel_t channel,
    uint32_t expectedConnectionGeneration,
    sensorarrayBleSendContext_t *outContext)
{
    if (channel > SENSORARRAY_BLE_CH_CTRL) {
        return false;
    }

    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleTxMode_t mode = s_ble.txMode;
    bool canSend = s_ble.stats.gattReady &&
                   s_ble.stats.connected &&
                   SENSORARRAY_BLE_CCCD_ALLOWS_MODE(s_ble.cccd[channel], mode) &&
                   (expectedConnectionGeneration == 0u ||
                    expectedConnectionGeneration == s_ble.connectionGeneration);
    if (canSend && outContext) {
        *outContext = (sensorarrayBleSendContext_t){
            .connectionGeneration = s_ble.connectionGeneration,
            .gattsIf = s_ble.gattsIf,
            .connId = s_ble.connId,
            .mtu = s_ble.stats.mtu,
            .txMode = mode,
            .congested = s_ble.stats.congested,
        };
    }
    portEXIT_CRITICAL(&s_bleMux);
    return canSend;
}

static bool sensorarrayBleSendContextMatches(
    sensorarrayBleChannel_t channel,
    const sensorarrayBleSendContext_t *context,
    bool *outCongested)
{
    if (!context || channel > SENSORARRAY_BLE_CH_CTRL) {
        return false;
    }
    portENTER_CRITICAL(&s_bleMux);
    bool current = s_ble.stats.gattReady &&
                   s_ble.stats.connected &&
                   s_ble.connectionGeneration == context->connectionGeneration &&
                   s_ble.gattsIf == context->gattsIf &&
                   s_ble.connId == context->connId &&
                   s_ble.txMode == context->txMode &&
                   SENSORARRAY_BLE_CCCD_ALLOWS_MODE(s_ble.cccd[channel],
                                                    context->txMode);
    if (outCongested) {
        *outCongested = s_ble.stats.congested;
    }
    portEXIT_CRITICAL(&s_bleMux);
    return current;
}

static bool sensorarrayBleSendContextCurrent(
    sensorarrayBleChannel_t channel,
    const sensorarrayBleSendContext_t *context)
{
    bool congested = false;
    return sensorarrayBleSendContextMatches(channel, context, &congested) &&
           !(congested && context->txMode == SENSORARRAY_BLE_TX_FAST);
}

bool sensorarrayBleCanSend(sensorarrayBleChannel_t channel)
{
    return sensorarrayBleSnapshotSendContext(channel, 0u, NULL);
}

bool sensorarrayBleGetSendGeneration(sensorarrayBleChannel_t channel,
                                     uint32_t *outConnectionGeneration)
{
    if (!outConnectionGeneration) {
        return false;
    }
    sensorarrayBleSendContext_t context = {0};
    if (!sensorarrayBleSnapshotSendContext(channel, 0u, &context)) {
        *outConnectionGeneration = 0u;
        return false;
    }
    *outConnectionGeneration = context.connectionGeneration;
    return true;
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
    char header[SENSORARRAY_BLE_FRAGMENT_HEADER_BUFFER_SIZE];
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
    if (headerLen <= 0 || (size_t)headerLen >= sizeof(header) ||
        (size_t)headerLen >= maximum) {
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
                sensorarrayBleStatsIncrement(&s_ble.stats.tinyTailCount);
            }
            return true;
        }
    }
    return false;
}

static void sensorarrayBlePrepareConfirmation(
    const sensorarrayBleSendContext_t *context,
    uint16_t handle)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    /* The TX task owns this notification slot.  Drain both state and value so
     * a confirmation that arrived after an earlier timeout cannot confirm the
     * next indication. */
    while (ulTaskNotifyTake(pdTRUE, 0) != 0u) {
    }
    portENTER_CRITICAL(&s_bleMux);
    if (s_ble.confirmWaitTask != NULL) {
        s_ble.stats.staleConfirmation++;
    }
    s_ble.confirmWaitTask = task;
    s_ble.confirmConnectionGeneration = context->connectionGeneration;
    s_ble.confirmConnId = context->connId;
    s_ble.confirmHandle = handle;
    s_ble.confirmStatus = ESP_GATT_ERROR;
    s_ble.confirmPending = true;
    portEXIT_CRITICAL(&s_bleMux);
}

static void sensorarrayBleClearConfirmation(TaskHandle_t task)
{
    portENTER_CRITICAL(&s_bleMux);
    if (s_ble.confirmWaitTask == task) {
        s_ble.confirmWaitTask = NULL;
        s_ble.confirmConnectionGeneration = 0u;
        s_ble.confirmConnId = 0u;
        s_ble.confirmHandle = 0u;
        s_ble.confirmStatus = ESP_GATT_ERROR;
        s_ble.confirmPending = false;
    }
    portEXIT_CRITICAL(&s_bleMux);
}

static esp_err_t sensorarrayBleAwaitConfirmation(
    const sensorarrayBleSendContext_t *context,
    uint16_t handle)
{
    TaskHandle_t task = xTaskGetCurrentTaskHandle();
    uint32_t notified = ulTaskNotifyTake(
        pdTRUE, pdMS_TO_TICKS(SENSORARRAY_BLE_CONF_TIMEOUT_MS));
    portENTER_CRITICAL(&s_bleMux);
    bool ownsWait = s_ble.confirmWaitTask == task &&
                    s_ble.confirmConnectionGeneration ==
                        context->connectionGeneration &&
                    s_ble.confirmConnId == context->connId &&
                    s_ble.confirmHandle == handle;
    esp_gatt_status_t status = s_ble.confirmStatus;
    bool connectionCurrent = s_ble.stats.connected &&
                             s_ble.connectionGeneration ==
                                 context->connectionGeneration &&
                             s_ble.connId == context->connId;
    if (s_ble.confirmWaitTask == task) {
        s_ble.confirmWaitTask = NULL;
        s_ble.confirmConnectionGeneration = 0u;
        s_ble.confirmConnId = 0u;
        s_ble.confirmHandle = 0u;
        s_ble.confirmStatus = ESP_GATT_ERROR;
        s_ble.confirmPending = false;
    }
    portEXIT_CRITICAL(&s_bleMux);
    if (notified == 0u) {
        return ESP_ERR_TIMEOUT;
    }
    if (!ownsWait || !connectionCurrent) {
        return ESP_ERR_INVALID_STATE;
    }
    return status == ESP_GATT_OK ? ESP_OK : ESP_FAIL;
}

static esp_err_t sensorarrayBleSendGattValue(
    const sensorarrayBleSendContext_t *context,
    sensorarrayBleChannel_t channel,
    uint16_t handle,
    uint16_t length,
    uint8_t *value,
    bool confirm)
{
    if (!context || !value || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t maximumAttempts = channel == SENSORARRAY_BLE_CH_CTRL ?
        (SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS /
         SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS) + 1u : 1u;
    esp_err_t lastError = ESP_FAIL;
    for (uint32_t attempt = 0u; attempt < maximumAttempts; ++attempt) {
        bool congested = false;
        if (!sensorarrayBleSendContextMatches(channel, context, &congested)) {
            sensorarrayBleStatsIncrement(&s_ble.stats.txConnectionStaleDrop);
            return ESP_ERR_INVALID_STATE;
        }

        if (!congested) {
            if (confirm) {
                sensorarrayBlePrepareConfirmation(context, handle);
            }
            lastError = esp_ble_gatts_send_indicate(context->gattsIf,
                                                    context->connId,
                                                    handle,
                                                    length,
                                                    value,
                                                    confirm);
            if (lastError == ESP_OK) {
                /* Once Bluedroid accepts an indication, never resend it: a
                 * missing confirmation is ambiguous and retrying could
                 * duplicate a control reply at the client. */
                return confirm ?
                    sensorarrayBleAwaitConfirmation(context, handle) : ESP_OK;
            }
            if (confirm) {
                sensorarrayBleClearConfirmation(xTaskGetCurrentTaskHandle());
            }
        }

        /* ESP-IDF's Bluedroid API returns ESP_FAIL before accepting the
         * value when L2CAP is congested or BTC cannot queue it.  Other error
         * classes are not transient for this descriptor and must not be
         * retried. */
        if (lastError != ESP_FAIL) {
            break;
        }
        if (attempt + 1u >= maximumAttempts) {
            if (channel == SENSORARRAY_BLE_CH_CTRL) {
                sensorarrayBleStatsIncrement(
                    &s_ble.stats.controlTxRetryExhausted);
            }
            break;
        }
        sensorarrayBleStatsIncrement(&s_ble.stats.controlTxRetry);
        vTaskDelay(pdMS_TO_TICKS(SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS));
    }
    return lastError;
}

static esp_err_t sensorarrayBleSendFragment(
                                            const sensorarrayBleSendContext_t *context,
                                            sensorarrayBleChannel_t channel,
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
    bool contextCurrent = channel == SENSORARRAY_BLE_CH_CTRL ?
        sensorarrayBleSendContextMatches(channel, context, NULL) :
        sensorarrayBleSendContextCurrent(channel, context);
    if (!context || !contextCurrent) {
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.txConnectionStaleDrop++;
        portEXIT_CRITICAL(&s_bleMux);
        return ESP_ERR_INVALID_STATE;
    }
    uint16_t handle = sensorarrayBleValueHandle(channel);
    uint8_t *packet = s_ble.txPacketScratch;
    int headerLen = snprintf((char *)packet,
                             SENSORARRAY_BLE_ATTR_VALUE_MAX,
                             "G,%c,%lu,%lu,%lu,%u,%u,%08lX\n",
                             envChannel,
                             (unsigned long)messageId,
                             (unsigned long)fragmentIndex,
                             (unsigned long)fragmentCount,
                             (unsigned)chunk,
                             (unsigned)messageLength,
                             (unsigned long)crc);
    size_t maximum = context->mtu > 3u ? (size_t)context->mtu - 3u : 20u;
    if (maximum > SENSORARRAY_BLE_ATTR_VALUE_MAX) {
        maximum = SENSORARRAY_BLE_ATTR_VALUE_MAX;
    }
    if (headerLen <= 0 ||
        (size_t)headerLen >= SENSORARRAY_BLE_ATTR_VALUE_MAX ||
        (size_t)headerLen + chunk > maximum) {
        sensorarrayBleStatsIncrement(&s_ble.stats.fragmentError);
        return ESP_ERR_INVALID_SIZE;
    }
    if (!sensorarrayBleValidateAscii(channel, data, chunk, messageId, fragmentIndex)) {
        sensorarrayBleStatsIncrement(&s_ble.stats.fragmentError);
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(packet + headerLen, data, chunk);
    esp_err_t err = sensorarrayBleSendGattValue(
        context,
        channel,
        handle,
        (uint16_t)((size_t)headerLen + chunk),
        packet,
        confirm);
    if (err != ESP_OK) {
        /* A CCCD disable or disconnect can race between the fragment-level
         * context check above and sensorarrayBleSendGattValue().  That helper
         * reports the lifecycle cancellation as ESP_ERR_INVALID_STATE and
         * already increments txConnectionStaleDrop.  It is an expected
         * bounded drop during unsubscribe/reconnect, not a malformed
         * fragment or GATT transmit failure. */
        if (err != ESP_ERR_INVALID_STATE) {
            sensorarrayBleStatsIncrement(&s_ble.stats.fragmentError);
        }
        return err;
    }
    sensorarrayBleStatsIncrement(&s_ble.stats.fragmentSent);
    return ESP_OK;
}

static esp_err_t sensorarrayBleSendPayload(sensorarrayBleChannel_t channel,
                                           const uint8_t *data,
                                           uint16_t length,
                                           uint32_t enqueueCrc,
                                           uint32_t expectedConnectionGeneration)
{
    if (!data || channel > SENSORARRAY_BLE_CH_CTRL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayBleSendContext_t context = {0};
    if (!sensorarrayBleSnapshotSendContext(channel,
                                           expectedConnectionGeneration,
                                           &context)) {
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.txConnectionStaleDrop++;
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_STATE;
    }
    if (context.congested && context.txMode == SENSORARRAY_BLE_TX_FAST &&
        channel != SENSORARRAY_BLE_CH_CTRL) {
        sensorarrayBleStatsIncrement(&s_ble.stats.congestedCount);
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_TIMEOUT;
    }
    size_t maximum = context.mtu > 3u ? (size_t)context.mtu - 3u : 20u;
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
        sensorarrayBleStatsIncrement(&s_ble.stats.txCrcMismatch);
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
        bool confirm = context.txMode == SENSORARRAY_BLE_TX_SAFE;
        esp_err_t err = sensorarrayBleSendGattValue(&context,
                                                    channel,
                                                    handle,
                                                    length,
                                                    (uint8_t *)data,
                                                    confirm);
        if (err == ESP_OK) {
            sensorarrayBleStatsNoteMessageSent(channel, true);
        } else {
            /* Keep lifecycle cancellation separate from true fragment/send
             * errors for the same reason as the fragmented path above. */
            if (err != ESP_ERR_INVALID_STATE) {
                sensorarrayBleStatsIncrement(&s_ble.stats.fragmentError);
            }
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
    bool confirm = context.txMode == SENSORARRAY_BLE_TX_SAFE;
    for (uint32_t index = 0u; index < fragmentCount; ++index) {
        size_t chunk = base + (index < extra ? 1u : 0u);
        esp_err_t err = sensorarrayBleSendFragment(&context,
                                                   channel,
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
    sensorarrayBleStatsNoteMessageSent(channel, false);
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
    return sensorarrayBleSendPayload(desc->channel,
                                     data,
                                     desc->length,
                                     desc->crc,
                                     desc->connectionGeneration);
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
        sensorarrayBleStatsIncrement(&s_ble.stats.messageQueued);
        return ESP_OK;
    }

    if (!desc->ctrlSlot) {
        sensorarrayBleTxDescriptor_t discarded;
        if (xQueueReceive(queue, &discarded, 0) == pdTRUE) {
            sensorarrayBleNoteDrop(discarded.channel);
            sensorarrayBleReleaseTxDescriptor(&discarded);
        }
        if (xQueueSend(queue, desc, 0) == pdTRUE) {
            sensorarrayBleStatsIncrement(&s_ble.stats.messageQueued);
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
        sensorarrayBleStatsIncrement(&s_ble.stats.controlRxStale);
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
    portENTER_CRITICAL(&s_bleMux);
    bool controlReady = s_ble.controlCallback != NULL;
    uint32_t connectionGeneration = s_ble.connectionGeneration;
    portEXIT_CRITICAL(&s_bleMux);
    if (!s_ble.ctrlRxQueue || !controlReady) {
        sensorarrayBleStatsIncrement(&s_ble.stats.controlRxDropped);
        return;
    }
    if (param->write.len == 0u || param->write.len > SENSORARRAY_BLE_CTRL_RX_MAX) {
        sensorarrayBleStatsIncrement(&s_ble.stats.controlRxDropped);
        printf("BLERXERR,src=ctrl,len=%u,max=%u,reason=length\n",
               (unsigned)param->write.len,
               (unsigned)SENSORARRAY_BLE_CTRL_RX_MAX);
        return;
    }
    sensorarrayBleControlCommand_t command;
    command.length = param->write.len;
    command.connId = param->write.conn_id;
    command.connectionGeneration = connectionGeneration;
    memcpy(command.data, param->write.value, param->write.len);
    if (xQueueSend(s_ble.ctrlRxQueue, &command, 0) == pdTRUE) {
        sensorarrayBleStatsIncrement(&s_ble.stats.controlRxQueued);
    } else {
        sensorarrayBleStatsIncrement(&s_ble.stats.controlRxDropped);
        printf("BLERXERR,src=ctrl,len=%u,reason=rx_queue_full\n",
               (unsigned)param->write.len);
    }
}

static void sensorarrayBleStartAdvertising(void)
{
    esp_err_t err = esp_ble_gap_ext_adv_start(1u, s_advSet);
    if (err != ESP_OK) {
        sensorarrayBleStatsSetInitError(err);
        sensorarrayBleLogBoot("adv_start", err);
    }
}

static void sensorarrayBleLogReadySummary(void)
{
    bool controllerReady;
    bool hostReady;
    bool gattReady;
    bool advertising;
    uint16_t mtu;
    uint16_t preferredMtu;
    bool preferPhy2m;
    sensorarrayBleTxMode_t txMode;

    portENTER_CRITICAL(&s_bleMux);
    controllerReady = s_ble.stats.controllerReady;
    hostReady = s_ble.stats.hostReady;
    gattReady = s_ble.stats.gattReady;
    advertising = s_ble.stats.advertising;
    if (s_ble.bootSummaryPrinted || !controllerReady || !hostReady ||
        !gattReady || !advertising) {
        portEXIT_CRITICAL(&s_bleMux);
        return;
    }
    s_ble.bootSummaryPrinted = true;
    mtu = s_ble.stats.mtu;
    preferredMtu = s_ble.config.preferredMtu;
    preferPhy2m = s_ble.config.preferPhy2m;
    txMode = s_ble.txMode;
    portEXIT_CRITICAL(&s_bleMux);

    printf("BLEBOOT,status=ok,controller=%u,host=%u,gatt=%u,adv=%u,mtu=%u,phyReq=%s,txMode=%s,ctrlRxMax=%u,msgMax=%u\n",
           controllerReady ? 1u : 0u,
           hostReady ? 1u : 0u,
           gattReady ? 1u : 0u,
           advertising ? 1u : 0u,
           preferredMtu ? preferredMtu : mtu,
           preferPhy2m ? "2M" : "1M",
           sensorarrayBleTxModeName(txMode),
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
            sensorarrayBleStatsSetInitError(ESP_FAIL);
            sensorarrayBleLogBoot("adv_params_async", ESP_FAIL);
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS) {
            sensorarrayBleStartAdvertising();
        } else {
            sensorarrayBleStatsSetInitError(ESP_FAIL);
            sensorarrayBleLogBoot("adv_data_async", ESP_FAIL);
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT: {
        esp_err_t err = param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS ? ESP_OK : ESP_FAIL;
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.advertising = err == ESP_OK;
        if (err != ESP_OK) {
            s_ble.stats.initError = err;
        }
        portEXIT_CRITICAL(&s_bleMux);
        if (err == ESP_OK) {
            sensorarrayBleLogReadySummary();
        } else {
            sensorarrayBleLogBoot("adv_start", err);
        }
        break;
    }
    case ESP_GAP_BLE_READ_PHY_COMPLETE_EVT:
        if (param->read_phy.status == ESP_BT_STATUS_SUCCESS) {
            portENTER_CRITICAL(&s_bleMux);
            s_ble.stats.txPhy = param->read_phy.tx_phy;
            s_ble.stats.rxPhy = param->read_phy.rx_phy;
            portEXIT_CRITICAL(&s_bleMux);
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
            sensorarrayBleStatsSetInitError(ESP_FAIL);
            sensorarrayBleLogBoot("app_register_async", ESP_FAIL);
            break;
        }
        portENTER_CRITICAL(&s_bleMux);
        s_ble.gattsIf = gattsIf;
        portEXIT_CRITICAL(&s_bleMux);
        (void)esp_ble_gap_set_device_name(s_ble.config.deviceName);
        esp_err_t advErr = esp_ble_gap_ext_adv_set_params(0u, &s_advParams);
        if (advErr != ESP_OK) {
            sensorarrayBleStatsSetInitError(advErr);
            sensorarrayBleLogBoot("adv_params", advErr);
        }
        esp_err_t tableErr = esp_ble_gatts_create_attr_tab(s_gattDb, gattsIf, BLE_IDX_COUNT,
                                                           SENSORARRAY_BLE_APP_ID);
        if (tableErr != ESP_OK) {
            sensorarrayBleStatsSetInitError(tableErr);
            sensorarrayBleLogBoot("gatt_table", tableErr);
        }
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK &&
            param->add_attr_tab.num_handle == BLE_IDX_COUNT) {
            memcpy(s_ble.handles, param->add_attr_tab.handles, sizeof(s_ble.handles));
            esp_err_t err = esp_ble_gatts_start_service(s_ble.handles[BLE_IDX_SERVICE]);
            if (err != ESP_OK) {
                sensorarrayBleStatsSetInitError(err);
                sensorarrayBleLogBoot("gatt_start", err);
            }
        } else {
            sensorarrayBleStatsSetInitError(ESP_FAIL);
            sensorarrayBleLogBoot("gatt_table_async", ESP_FAIL);
        }
        break;
    case ESP_GATTS_START_EVT: {
        esp_err_t err = param->start.status == ESP_GATT_OK ? ESP_OK : ESP_FAIL;
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.gattReady = err == ESP_OK;
        if (err != ESP_OK) {
            s_ble.stats.initError = err;
        }
        portEXIT_CRITICAL(&s_bleMux);
        if (err != ESP_OK) {
            sensorarrayBleLogBoot("gatt_start", err);
        }
        break;
    }
    case ESP_GATTS_CONNECT_EVT: {
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.connected = true;
        s_ble.stats.advertising = false;
        s_ble.stats.congested = false;
        /* ATT MTU is negotiated per connection.  Do not reuse the previous
         * peer's larger MTU before this connection receives MTU_EVT. */
        s_ble.stats.mtu = SENSORARRAY_BLE_DEFAULT_ATT_MTU;
        memset(s_ble.cccd, 0, sizeof(s_ble.cccd));
        s_ble.connId = param->connect.conn_id;
        s_ble.connectionGeneration = sensorarrayBleNextGeneration(s_ble.connectionGeneration);
        s_ble.gattsIf = gattsIf;
        portEXIT_CRITICAL(&s_bleMux);
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
    case ESP_GATTS_DISCONNECT_EVT: {
        TaskHandle_t confirmTask = NULL;
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.connected = false;
        s_ble.stats.congested = false;
        s_ble.stats.mtu = SENSORARRAY_BLE_DEFAULT_ATT_MTU;
        s_ble.connectionGeneration = sensorarrayBleNextGeneration(s_ble.connectionGeneration);
        memset(s_ble.cccd, 0, sizeof(s_ble.cccd));
        if (s_ble.confirmWaitTask != NULL) {
            s_ble.confirmStatus = ESP_GATT_ERROR;
            s_ble.confirmPending = false;
            confirmTask = s_ble.confirmWaitTask;
        }
        portEXIT_CRITICAL(&s_bleMux);
        if (confirmTask) {
            xTaskNotifyGive(confirmTask);
        }
        sensorarrayBleDrainControlRxQueue();
        sensorarrayBleDrainTxQueues();
        sensorarrayBleStartAdvertising();
        break;
    }
    case ESP_GATTS_MTU_EVT:
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.mtu = param->mtu.mtu;
        portEXIT_CRITICAL(&s_bleMux);
        break;
    case ESP_GATTS_WRITE_EVT: {
        int channel = sensorarrayBleCccdChannel(param->write.handle);
        if (channel >= 0 && param->write.len == 2u) {
            uint16_t value = (uint16_t)param->write.value[0] |
                             ((uint16_t)param->write.value[1] << 8u);
            portENTER_CRITICAL(&s_bleMux);
            s_ble.cccd[channel] = value & SENSORARRAY_BLE_CCCD_SUPPORTED_MASK;
            portEXIT_CRITICAL(&s_bleMux);
        } else if (param->write.handle == s_ble.handles[BLE_IDX_CTRL_RX_VALUE]) {
            sensorarrayBleQueueControlCommand(param);
        }
        break;
    }
    case ESP_GATTS_CONF_EVT: {
        TaskHandle_t confirmTask = NULL;
        portENTER_CRITICAL(&s_bleMux);
        if (s_ble.confirmPending && s_ble.confirmWaitTask != NULL) {
            bool matches = s_ble.confirmConnectionGeneration ==
                               s_ble.connectionGeneration &&
                           s_ble.confirmConnId == param->conf.conn_id &&
                           s_ble.confirmHandle == param->conf.handle;
            if (matches) {
                s_ble.confirmStatus = param->conf.status;
                s_ble.confirmPending = false;
                confirmTask = s_ble.confirmWaitTask;
            } else {
                s_ble.stats.staleConfirmation++;
            }
        }
        portEXIT_CRITICAL(&s_bleMux);
        if (confirmTask) {
            xTaskNotifyGive(confirmTask);
        }
        break;
    }
    case ESP_GATTS_CONGEST_EVT:
        portENTER_CRITICAL(&s_bleMux);
        if (s_ble.stats.connected &&
            param->congest.conn_id == s_ble.connId) {
            s_ble.stats.congested = param->congest.congested;
            if (param->congest.congested) {
                s_ble.stats.congestedCount++;
            }
        }
        portEXIT_CRITICAL(&s_bleMux);
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
        sensorarrayBleStatsSetInitError(err);
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
    s_ble.stats.mtu = SENSORARRAY_BLE_DEFAULT_ATT_MTU;
    s_ble.stats.txPhy = ESP_BLE_GAP_PHY_1M;
    s_ble.stats.rxPhy = ESP_BLE_GAP_PHY_1M;
    s_ble.stats.txTaskConfiguredBytes = CONFIG_SENSORARRAY_BLE_TX_TASK_STACK;
    s_ble.stats.ctrlTaskConfiguredBytes = CONFIG_SENSORARRAY_BLE_CTRL_TASK_STACK;
    s_ble.txMode = CONFIG_SENSORARRAY_BLE_TX_MODE_SAFE ?
        SENSORARRAY_BLE_TX_SAFE : SENSORARRAY_BLE_TX_FAST;
    s_ble.txQueue = xQueueCreateStatic(SENSORARRAY_BLE_TX_DESCRIPTOR_QUEUE_COUNT,
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
        sensorarrayBleStatsSetInitError(ESP_ERR_NO_MEM);
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
        sensorarrayBleStatsSetInitError(ESP_ERR_NO_MEM);
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
    portENTER_CRITICAL(&s_bleMux);
    s_ble.stats.controllerReady = true;
    portEXIT_CRITICAL(&s_bleMux);

    err = esp_bluedroid_init();
    if (sensorarrayBleStep("bluedroid_init", err) != ESP_OK) {
        return err;
    }
    err = esp_bluedroid_enable();
    if (sensorarrayBleStep("bluedroid_enable", err) != ESP_OK) {
        return err;
    }
    portENTER_CRITICAL(&s_bleMux);
    s_ble.stats.hostReady = true;
    portEXIT_CRITICAL(&s_bleMux);

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

static esp_err_t sensorarrayBleNotifyInternal(
    sensorarrayBleChannel_t channel,
    const uint8_t *data,
    size_t length,
    uint32_t expectedConnectionGeneration)
{
    if (channel > SENSORARRAY_BLE_CH_CTRL || !data || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (length > SENSORARRAY_BLE_MESSAGE_VALUE_MAX) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_SIZE;
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
    esp_err_t allocateErr = sensorarrayBleAllocateTxDescriptor(
        channel, length, crc, expectedConnectionGeneration, &desc);
    if (allocateErr != ESP_OK) {
        sensorarrayBleNoteDrop(channel);
        return allocateErr;
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

esp_err_t sensorarrayBleNotify(sensorarrayBleChannel_t channel,
                              const uint8_t *data,
                              size_t length)
{
    return sensorarrayBleNotifyInternal(channel, data, length, 0u);
}

esp_err_t sensorarrayBleNotifyForGeneration(
    sensorarrayBleChannel_t channel,
    const uint8_t *data,
    size_t length,
    uint32_t expectedConnectionGeneration)
{
    if (expectedConnectionGeneration == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    return sensorarrayBleNotifyInternal(channel,
                                        data,
                                        length,
                                        expectedConnectionGeneration);
}

void sensorarrayBleSetControlRxCallback(sensorarrayBleControlRxCallback_t callback,
                                       void *userContext)
{
    portENTER_CRITICAL(&s_bleMux);
    s_ble.controlCallback = callback;
    s_ble.controlContext = userContext;
    portEXIT_CRITICAL(&s_bleMux);
}

bool sensorarrayBleIsReady(void)
{
    portENTER_CRITICAL(&s_bleMux);
    bool ready = s_ble.stats.controllerReady && s_ble.stats.hostReady &&
                 s_ble.stats.gattReady && s_ble.stats.advertising;
    portEXIT_CRITICAL(&s_bleMux);
    return ready;
}

bool sensorarrayBleIsConnected(void)
{
    portENTER_CRITICAL(&s_bleMux);
    bool connected = s_ble.stats.connected;
    portEXIT_CRITICAL(&s_bleMux);
    return connected;
}

bool sensorarrayBleIsSubscribed(sensorarrayBleChannel_t channel)
{
    if (channel > SENSORARRAY_BLE_CH_CTRL) {
        return false;
    }
    portENTER_CRITICAL(&s_bleMux);
    bool subscribed = (s_ble.cccd[channel] &
                       SENSORARRAY_BLE_CCCD_SUPPORTED_MASK) != 0u;
    portEXIT_CRITICAL(&s_bleMux);
    return subscribed;
}

bool sensorarrayBleIsCongested(void)
{
    portENTER_CRITICAL(&s_bleMux);
    bool congested = s_ble.stats.congested;
    portEXIT_CRITICAL(&s_bleMux);
    return congested;
}

void sensorarrayBleGetStats(sensorarrayBleStats_t *outStats)
{
    if (!outStats) {
        return;
    }

    portENTER_CRITICAL(&s_bleMux);
    TaskHandle_t txTask = s_ble.txTask;
    TaskHandle_t controlTask = s_ble.controlTask;
    portEXIT_CRITICAL(&s_bleMux);

    /* FreeRTOS reports a count of StackType_t entries; convert it to bytes. */
    uint32_t txMinimumRemainingBytes = txTask ?
        (uint32_t)((uint64_t)uxTaskGetStackHighWaterMark(txTask) *
                   sizeof(StackType_t)) : 0u;
    uint32_t ctrlMinimumRemainingBytes = controlTask ?
        (uint32_t)((uint64_t)uxTaskGetStackHighWaterMark(controlTask) *
                   sizeof(StackType_t)) : 0u;

    portENTER_CRITICAL(&s_bleMux);
    s_ble.stats.txTaskConfiguredBytes = CONFIG_SENSORARRAY_BLE_TX_TASK_STACK;
    s_ble.stats.ctrlTaskConfiguredBytes = CONFIG_SENSORARRAY_BLE_CTRL_TASK_STACK;
    s_ble.stats.txTaskMinimumRemainingBytes = txMinimumRemainingBytes;
    s_ble.stats.ctrlTaskMinimumRemainingBytes = ctrlMinimumRemainingBytes;
    *outStats = s_ble.stats;
    portEXIT_CRITICAL(&s_bleMux);
}
