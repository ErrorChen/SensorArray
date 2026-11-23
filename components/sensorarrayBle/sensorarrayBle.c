#include "sensorarrayBle.h"
#include "sensorarrayBleWatchdog.h"
#include "sensorarrayBleOutputSchedule.h"
#include "sensorarrayWorkBudget.h"

#include <stdio.h>
#include <string.h>

#ifdef ESP_PLATFORM
#include "esp_rom_crc.h"
#endif

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifndef CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN
#define CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN 4
#endif
#ifndef CONFIG_SENSORARRAY_BLE_TX_TASK_STACK
#define CONFIG_SENSORARRAY_BLE_TX_TASK_STACK 4096
#endif
#ifndef CONFIG_SENSORARRAY_BLE_TX_TASK_PRIORITY
#define CONFIG_SENSORARRAY_BLE_TX_TASK_PRIORITY 7
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
#define CONFIG_SENSORARRAY_BLE_CTRL_TASK_PRIORITY 8
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
#define SENSORARRAY_BLE_PROTECTED_SLOT_COUNT 2u
#define SENSORARRAY_BLE_PAYLOAD_SLOT_COUNT \
    (CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN + SENSORARRAY_BLE_LOG_TX_SLOT_COUNT + \
     SENSORARRAY_BLE_PROTECTED_SLOT_COUNT)
#define SENSORARRAY_BLE_PAYLOAD_STORAGE_BYTES \
    (SENSORARRAY_BLE_PAYLOAD_SLOT_COUNT * SENSORARRAY_BLE_MESSAGE_VALUE_MAX)
#define SENSORARRAY_BLE_TINY_TAIL_MIN 8u
#define SENSORARRAY_BLE_CONF_TIMEOUT_MS 1000u
#define SENSORARRAY_BLE_DEFAULT_ATT_MTU 23u
#define SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS 10u
#define SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS 500u
#define SENSORARRAY_BLE_TX_FRAGMENT_BATCH_COUNT 4u
#define SENSORARRAY_BLE_HEAP_LOW_WATERMARK 49152u
#define SENSORARRAY_BLE_CCCD_NOTIFY_BIT 0x0001u
#define SENSORARRAY_BLE_CCCD_INDICATE_BIT 0x0002u
#define SENSORARRAY_BLE_CCCD_SUPPORTED_MASK \
    (SENSORARRAY_BLE_CCCD_NOTIFY_BIT | SENSORARRAY_BLE_CCCD_INDICATE_BIT)
#define SENSORARRAY_BLE_CCCD_ALLOWS_MODE(cccdValue, txModeValue) \
    ((((uint16_t)(cccdValue)) & \
      ((txModeValue) == SENSORARRAY_BLE_TX_SAFE ? \
           SENSORARRAY_BLE_CCCD_SUPPORTED_MASK : \
           SENSORARRAY_BLE_CCCD_NOTIFY_BIT)) != 0u)

#define SENSORARRAY_BLE_MODE_USES_CONFIRM(cccdValue, txModeValue) \
    ((txModeValue) == SENSORARRAY_BLE_TX_SAFE && \
     (((uint16_t)(cccdValue)) & SENSORARRAY_BLE_CCCD_INDICATE_BIT) != 0u)

_Static_assert(SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS > 0u &&
                   SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS >=
                       SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS,
               "BLE CTRL retry window must be finite and non-zero");
_Static_assert(SENSORARRAY_BLE_SEND_DEADLINE_US ==
                   SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS * 1000u,
               "BLE SDK guard must use the existing control retry envelope");

_Static_assert(!SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0000u,
                                                  SENSORARRAY_BLE_TX_FAST) &&
                   !SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0000u,
                                                     SENSORARRAY_BLE_TX_SAFE),
               "disabled CCCD must reject FAST and SAFE sends");
_Static_assert(SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0001u,
                                                 SENSORARRAY_BLE_TX_FAST) &&
                   SENSORARRAY_BLE_CCCD_ALLOWS_MODE(0x0001u,
                                                     SENSORARRAY_BLE_TX_SAFE) &&
                   !SENSORARRAY_BLE_MODE_USES_CONFIRM(0x0001u,
                                                      SENSORARRAY_BLE_TX_SAFE),
               "notify-only CCCD must allow FAST and SAFE fallback");
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
               "BLE fragment header limit must leave room for payload");

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
    /* CPU-only payload storage lives for the entire boot in explicit PSRAM.
     * Slot ownership and generation remain in internal RAM. */
    uint8_t *data;
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
    uint8_t protectedSlot;
    uint8_t typedFrame;
    sensorarrayBleChannel_t channel;
    uint16_t length;
    uint32_t generation;
    uint32_t connectionGeneration;
    uint32_t streamGeneration;
    uint32_t crc;
    uint64_t queuedAtUs;
} sensorarrayBleTxDescriptor_t;

enum { BLE_FRAME_FREE, BLE_FRAME_WRITING, BLE_FRAME_PENDING, BLE_FRAME_OWNED };
typedef struct {
    sensorarrayBleFrameIdentity_t identity;
    sensorarrayBleOutputTicket_t ticket;
    uint32_t periodicGeneration;
    bool completeBundle;
} sensorarrayBleFrameMeta_t;
typedef struct {
    bool active;
    bool identityKnown;
    bool firstPending;
    bool periodicPending;
    bool floorKnown;
    bool gapKnown;
    bool scheduleInitialized;
    bool fullCompleteKnown;
    uint32_t streamGeneration;
    uint32_t periodicGeneration;
    uint32_t lastFullSequence;
    uint64_t periodicDueUs;
    uint64_t lastFullCompleteUs;
    uint64_t lastProtectedCompleteUs;
    uint64_t maxCompleteGapUs;
    sensorarrayBleFrameIdentity_t identity;
    sensorarrayBleFrameIdentity_t floor;
    sensorarrayBleOutputSchedule_t schedule;
    sensorarrayBleOutputCounters_t counters;
    uint32_t protectedQueued;
    uint32_t protectedComplete;
    uint32_t protectedFailed;
    uint32_t protectedMissed;
    uint32_t protectedDeadlineMissed;
    uint32_t protectedSuperseded;
    uint32_t staleNormal;
    uint32_t intentionalSkip;
} sensorarrayBleFrameFlow_t;

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
    uint32_t streamGeneration;
    esp_gatt_if_t gattsIf;
    uint16_t connId;
    uint16_t mtu;
    sensorarrayBleTxMode_t txMode;
    bool confirm;
    bool congested;
    bool priority;
} sensorarrayBleSendContext_t;

_Static_assert(sizeof(sensorarrayBleControlCommand_t) <= 144u,
               "BLE RX command must stay a small BTC-task object");
_Static_assert(sizeof(sensorarrayBleTxDescriptor_t) <= 40u,
               "BLE TX queue must store descriptors, not whole payloads");
_Static_assert(SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX >= 512u,
               "BLE CTRL TX must carry the longest command response");
_Static_assert(SENSORARRAY_BLE_LOG_TX_SLOT_COUNT >= 2u,
               "BLE LOG pool must hold compact-summary and ADS messages");

typedef struct {
    sensorarrayBleConfig_t config;
    sensorarrayBleStats_t stats;
    sensorarrayBleLinkSnapshot_t link;
    sensorarrayBleGuardSnapshot_t guard;
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
    QueueHandle_t logTxQueue;
    QueueHandle_t ctrlTxQueue;
    QueueHandle_t ctrlRxQueue;
    StaticQueue_t txQueueStruct;
    StaticQueue_t logTxQueueStruct;
    StaticQueue_t ctrlTxQueueStruct;
    StaticQueue_t ctrlRxQueueStruct;
    uint8_t txQueueStorage[CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN *
                           sizeof(sensorarrayBleTxDescriptor_t)];
    uint8_t logTxQueueStorage[SENSORARRAY_BLE_LOG_TX_SLOT_COUNT *
                              sizeof(sensorarrayBleTxDescriptor_t)];
    uint8_t ctrlTxQueueStorage[SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT *
                               sizeof(sensorarrayBleTxDescriptor_t)];
    uint8_t ctrlRxQueueStorage[CONFIG_SENSORARRAY_BLE_CTRL_RX_QUEUE_LEN *
                               sizeof(sensorarrayBleControlCommand_t)];
    sensorarrayBleDataSlot_t dataSlots[CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN];
    sensorarrayBleDataSlot_t logSlots[SENSORARRAY_BLE_LOG_TX_SLOT_COUNT];
    sensorarrayBleCtrlSlot_t ctrlSlots[SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT];
    sensorarrayBleDataSlot_t protectedSlots[SENSORARRAY_BLE_PROTECTED_SLOT_COUNT];
    uint8_t *payloadStorage;
    sensorarrayBleTxDescriptor_t protectedDescriptors[SENSORARRAY_BLE_PROTECTED_SLOT_COUNT];
    uint8_t protectedPhase[SENSORARRAY_BLE_PROTECTED_SLOT_COUNT];
    sensorarrayBleFrameMeta_t protectedMeta[SENSORARRAY_BLE_PROTECTED_SLOT_COUNT];
    sensorarrayBleFrameMeta_t dataFrameMeta[CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN];
    sensorarrayBleFrameFlow_t frameFlow;
    bool ownerCapacityFailure;
    /* Work wakeups must not share the indication-confirmation notification.
     * Queue state is authoritative; the binary token only wakes an idle owner. */
    SemaphoreHandle_t txWake;
    StaticSemaphore_t txWakeStorage;
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

static sensorarrayBleState_t s_ble = {.guard = {.initError = ESP_ERR_INVALID_STATE}};
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
    for (uint8_t i = 0u; i < SENSORARRAY_BLE_PROTECTED_SLOT_COUNT; ++i) {
        used += s_ble.protectedSlots[i].inUse ? 1u : 0u;
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

/* Caller holds s_bleMux. Use the same effective CCCD/FAST/SAFE predicate as
 * DATA admission, not queue emptiness or an unsupported indication-only bit. */
static bool sensorarrayBleDataActiveLocked(void)
{
    return s_ble.guard.ready && !s_ble.guard.quarantined &&
           s_ble.stats.gattReady && s_ble.stats.connected &&
           SENSORARRAY_BLE_CCCD_ALLOWS_MODE(s_ble.cccd[SENSORARRAY_BLE_CH_DATA],
                                           s_ble.txMode);
}

static bool sensorarrayBleFrameSameBoot(const sensorarrayBleFrameIdentity_t *a,
                                       const sensorarrayBleFrameIdentity_t *b)
{
    return a->sessionId == b->sessionId && a->bootId == b->bootId;
}

static bool sensorarrayBleFrameSameConfig(const sensorarrayBleFrameIdentity_t *a,
                                         const sensorarrayBleFrameIdentity_t *b)
{
    return sensorarrayBleFrameSameBoot(a, b) && a->revision == b->revision;
}

/* Same-boot serial-number arithmetic; the exactly-half-range case is
 * unordered. Never infer ordering across boot/session identities. */
static bool sensorarrayBleFrameSequenceBefore(uint32_t a, uint32_t b)
{
    uint32_t distance = b - a;
    return distance != 0u && distance < UINT32_C(0x80000000);
}

static void sensorarrayBleFrameFlowSyncLocked(void)
{
    sensorarrayBleFrameFlow_t *flow = &s_ble.frameFlow;
    bool active = sensorarrayBleDataActiveLocked();
    if (active == flow->active) return;
    flow->active = active;
    flow->streamGeneration = sensorarrayBleNextGeneration(flow->streamGeneration);
    flow->identityKnown = false;
    flow->firstPending = true;
    flow->periodicPending = false;
    flow->periodicDueUs = 0u;
    flow->fullCompleteKnown = false;
    flow->periodicGeneration = sensorarrayBleNextGeneration(flow->periodicGeneration);
    flow->floorKnown = false;
    flow->maxCompleteGapUs = 0u;
    flow->gapKnown = false;
    if (flow->scheduleInitialized) {
        sensorarrayBleOutputObservation_t observation = {
            .epoch = flow->schedule.snapshot.epoch,
            .counters = flow->counters, .active = false, .valid = true,
        };
        (void)sensorarrayBleOutputScheduleObserve(&flow->schedule,
            (uint64_t)esp_timer_get_time(), &observation);
    }
}

static void sensorarrayBleFrameFlowObserveLocked(uint64_t nowUs)
{
    sensorarrayBleFrameFlow_t *flow = &s_ble.frameFlow;
    if (!flow->identityKnown) return;
    if (!flow->scheduleInitialized) {
        sensorarrayBleOutputScheduleInit(&flow->schedule, nowUs);
        flow->scheduleInitialized = true;
    }
    sensorarrayBleOutputObservation_t observation = {
        .epoch = {.sessionId = flow->identity.sessionId, .bootId = flow->identity.bootId,
            .connectionGeneration = s_ble.connectionGeneration,
            .streamGeneration = flow->streamGeneration},
        .counters = flow->counters, .active = flow->active, .valid = true,
    };
    (void)sensorarrayBleOutputScheduleObserve(&flow->schedule, nowUs, &observation);
}

static bool sensorarrayBleFrameProtectedRepresentedLocked(uint8_t flag)
{
    sensorarrayBleFrameFlow_t *flow = &s_ble.frameFlow;
    for (unsigned i = 0u; i < SENSORARRAY_BLE_PROTECTED_SLOT_COUNT; ++i) {
        sensorarrayBleFrameMeta_t *meta = &s_ble.protectedMeta[i];
        if (s_ble.protectedPhase[i] != BLE_FRAME_FREE &&
            s_ble.protectedDescriptors[i].streamGeneration == flow->streamGeneration &&
            sensorarrayBleFrameSameConfig(&meta->identity, &flow->identity) &&
            (meta->ticket.protectionFlags & flag) != 0u &&
            (flag != SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC ||
             meta->periodicGeneration == flow->periodicGeneration)) return true;
    }
    return false;
}

/* One local periodic obligation. Upstream PERIODIC flags reserve passage
 * through source/network queues; they do not create a second BLE cadence.
 * A recently completed typed full bundle already satisfies this obligation.
 * Without full completion, even unflagged input triggers the local fallback. */
static void sensorarrayBleFramePeriodicLocked(uint64_t nowUs)
{
    sensorarrayBleFrameFlow_t *flow = &s_ble.frameFlow;
    if (flow->periodicDueUs == 0u) flow->periodicDueUs =
        UINT64_MAX - nowUs < UINT64_C(1000000) ? UINT64_MAX : nowUs + UINT64_C(1000000);
    if (nowUs >= flow->periodicDueUs) {
        uint64_t periods = (nowUs - flow->periodicDueUs) / UINT64_C(1000000) + 1u;
        if (flow->periodicPending) flow->protectedDeadlineMissed++;
        flow->protectedDeadlineMissed += (uint32_t)(periods - 1u);
        flow->periodicGeneration += (uint32_t)periods;
        if (flow->periodicGeneration == 0u) flow->periodicGeneration = 1u;
        flow->periodicPending = true;
        flow->periodicDueUs = UINT64_MAX - nowUs < UINT64_C(1000000) ?
            UINT64_MAX : nowUs + (UINT64_C(1000000) -
                (nowUs - flow->periodicDueUs) % UINT64_C(1000000));
    }
}

/* Caller has verified a successful typed whole-bundle local completion in
 * the current connection/stream. FULL ordinary data counts; preview, raw
 * legacy messages and failed/partial sends never reach this helper. Keep
 * this evidence separate from BLESNAP's protected-only counters and clock. */
static void sensorarrayBleFrameFullCompletedLocked(
    const sensorarrayBleFrameIdentity_t *identity, uint64_t completedUs)
{
    sensorarrayBleFrameFlow_t *flow = &s_ble.frameFlow;
    if (!sensorarrayBleFrameSameConfig(identity, &flow->identity) ||
        (flow->fullCompleteKnown &&
         (completedUs < flow->lastFullCompleteUs ||
          !sensorarrayBleFrameSequenceBefore(flow->lastFullSequence, identity->sequence)))) return;
    flow->fullCompleteKnown = true;
    flow->lastFullSequence = identity->sequence;
    flow->lastFullCompleteUs = completedUs;
    flow->periodicDueUs = UINT64_MAX - completedUs < UINT64_C(1000000) ?
        UINT64_MAX : completedUs + UINT64_C(1000000);
    flow->periodicPending = false;
    /* Do not cancel or reclaim an already accepted protected descriptor.
     * Its owner retains the original identity, bytes and SDK-return fence. */
}

static esp_err_t sensorarrayBleAllocateTxDescriptor(
    sensorarrayBleChannel_t channel,
    size_t length,
    uint32_t crc,
    uint32_t expectedConnectionGeneration,
    bool priority,
    sensorarrayBleTxDescriptor_t *outDesc)
{
    if (!outDesc || channel > SENSORARRAY_BLE_CH_CTRL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (priority &&
        length > SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX) {
        return ESP_ERR_INVALID_SIZE;
    }

    memset(outDesc, 0, sizeof(*outDesc));
    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleFrameFlowSyncLocked();
    sensorarrayBleTxMode_t txMode = s_ble.txMode;
    uint32_t connectionGeneration = s_ble.connectionGeneration;
    bool sendable = s_ble.guard.ready && !s_ble.guard.quarantined &&
                    s_ble.stats.gattReady &&
                    s_ble.stats.connected &&
                    SENSORARRAY_BLE_CCCD_ALLOWS_MODE(s_ble.cccd[channel], txMode) &&
                    (expectedConnectionGeneration == 0u ||
                     expectedConnectionGeneration == connectionGeneration);
    if (!sendable) {
        if (!s_ble.guard.ready || s_ble.guard.quarantined) {
            s_ble.guard.rejected++;
        } else if (expectedConnectionGeneration != 0u &&
            expectedConnectionGeneration != connectionGeneration) {
            s_ble.stats.txConnectionStaleDrop++;
        }
        portEXIT_CRITICAL(&s_bleMux);
        return ESP_ERR_INVALID_STATE;
    }
    if (channel == SENSORARRAY_BLE_CH_LOG && !priority &&
        sensorarrayBleDataActiveLocked()) {
        /* A momentarily empty DATA queue does not establish spare link
         * capacity. Count this intentional ordinary-log policy separately
         * from slot exhaustion; never touch an already admitted owner. */
        s_ble.stats.logPolicyDrop++;
        s_ble.stats.dropped[SENSORARRAY_BLE_CH_LOG]++;
        s_ble.stats.messageDropped++;
        portEXIT_CRITICAL(&s_bleMux);
        return ESP_ERR_NOT_ALLOWED;
    }
    if (priority) {
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
                .streamGeneration = s_ble.frameFlow.streamGeneration,
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
         * per-channel descriptor queues preserve order within each channel;
         * ordinary LOG cannot sit in front of newly queued scientific DATA.
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
                .streamGeneration = s_ble.frameFlow.streamGeneration,
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
    if (desc->protectedSlot) {
        return desc->slotIndex < SENSORARRAY_BLE_PROTECTED_SLOT_COUNT ?
            s_ble.protectedSlots[desc->slotIndex].data : NULL;
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
    sensorarrayBleFrameFlowSyncLocked();
    if (desc->protectedSlot && !desc->ctrlSlot &&
        desc->slotIndex < SENSORARRAY_BLE_PROTECTED_SLOT_COUNT) {
        const sensorarrayBleDataSlot_t *slot = &s_ble.protectedSlots[desc->slotIndex];
        current = slot->inUse && slot->generation == desc->generation &&
            slot->channel == desc->channel && slot->length == desc->length && slot->crc == desc->crc &&
            desc->connectionGeneration == s_ble.connectionGeneration &&
            desc->streamGeneration == s_ble.frameFlow.streamGeneration;
    } else if (!desc->protectedSlot && desc->ctrlSlot && desc->slotIndex < SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT) {
        const sensorarrayBleCtrlSlot_t *slot = &s_ble.ctrlSlots[desc->slotIndex];
        current = slot->inUse &&
                  slot->generation == desc->generation &&
                  slot->channel == desc->channel &&
                  slot->length == desc->length &&
                  slot->crc == desc->crc &&
                  desc->connectionGeneration == s_ble.connectionGeneration;
    } else if (!desc->protectedSlot && !desc->ctrlSlot &&
               desc->channel == SENSORARRAY_BLE_CH_DATA &&
               desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN) {
        const sensorarrayBleDataSlot_t *slot = &s_ble.dataSlots[desc->slotIndex];
        current = slot->inUse &&
                  slot->generation == desc->generation &&
                  slot->channel == desc->channel &&
                  slot->length == desc->length &&
                  slot->crc == desc->crc &&
                  desc->connectionGeneration == s_ble.connectionGeneration &&
                  desc->streamGeneration == s_ble.frameFlow.streamGeneration;
    } else if (!desc->protectedSlot && !desc->ctrlSlot &&
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
    if (desc->protectedSlot && !desc->ctrlSlot &&
        desc->slotIndex < SENSORARRAY_BLE_PROTECTED_SLOT_COUNT) {
        sensorarrayBleDataSlot_t *slot = &s_ble.protectedSlots[desc->slotIndex];
        if (slot->inUse && slot->generation == desc->generation) {
            slot->inUse = false;
            s_ble.protectedPhase[desc->slotIndex] = BLE_FRAME_FREE;
            released = true;
        }
    } else if (!desc->protectedSlot && desc->ctrlSlot && desc->slotIndex < SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT) {
        sensorarrayBleCtrlSlot_t *slot = &s_ble.ctrlSlots[desc->slotIndex];
        if (slot->inUse && slot->generation == desc->generation) {
            slot->inUse = false;
            released = true;
        }
    } else if (!desc->protectedSlot && !desc->ctrlSlot &&
               desc->channel == SENSORARRAY_BLE_CH_DATA &&
               desc->slotIndex < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN) {
        sensorarrayBleDataSlot_t *slot = &s_ble.dataSlots[desc->slotIndex];
        if (slot->inUse && slot->generation == desc->generation) {
            slot->inUse = false;
            released = true;
        }
    } else if (!desc->protectedSlot && !desc->ctrlSlot &&
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
#ifdef ESP_PLATFORM
    /* New in this fix: ROM uses the same complemented IEEE CRC32 and a zero
     * initial seed. Keep size_t lengths explicit: production messages fit in
     * one uint32_t chunk, while wider host size_t must never truncate. */
    uint32_t crc = 0u;
    while (length != 0u) {
#if SIZE_MAX > UINT32_MAX
        const uint32_t chunk = length > UINT32_MAX ? UINT32_MAX : (uint32_t)length;
#else
        const uint32_t chunk = (uint32_t)length;
#endif
        crc = esp_rom_crc32_le(crc, data, chunk);
        data += chunk;
        length -= chunk;
    }
    return crc;
#else
    /* Independent portable bitwise reference, also used by host regressions. */
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0u; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            uint32_t mask = 0u - (crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
#endif
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
    sensorarrayBleFrameFlowSyncLocked();
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
    sensorarrayBleFrameFlowSyncLocked();
    sensorarrayBleTxMode_t mode = s_ble.txMode;
    uint16_t cccd = s_ble.cccd[channel];
    bool canSend = s_ble.guard.ready && !s_ble.guard.quarantined &&
                   s_ble.stats.gattReady &&
                   s_ble.stats.connected &&
                   SENSORARRAY_BLE_CCCD_ALLOWS_MODE(cccd, mode) &&
                   (expectedConnectionGeneration == 0u ||
                    expectedConnectionGeneration == s_ble.connectionGeneration);
    if (canSend && outContext) {
        *outContext = (sensorarrayBleSendContext_t){
            .connectionGeneration = s_ble.connectionGeneration,
            .streamGeneration = s_ble.frameFlow.streamGeneration,
            .gattsIf = s_ble.gattsIf,
            .connId = s_ble.connId,
            .mtu = s_ble.stats.mtu,
            .txMode = mode,
            .confirm = SENSORARRAY_BLE_MODE_USES_CONFIRM(cccd, mode),
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
    sensorarrayBleFrameFlowSyncLocked();
    uint16_t cccd = s_ble.cccd[channel];
    bool confirm = SENSORARRAY_BLE_MODE_USES_CONFIRM(cccd, s_ble.txMode);
    bool current = s_ble.guard.ready && !s_ble.guard.quarantined &&
                   s_ble.stats.gattReady &&
                   s_ble.stats.connected &&
                   s_ble.connectionGeneration == context->connectionGeneration &&
                   (channel != SENSORARRAY_BLE_CH_DATA ||
                    context->streamGeneration == s_ble.frameFlow.streamGeneration) &&
                   s_ble.gattsIf == context->gattsIf &&
                   s_ble.connId == context->connId &&
                   s_ble.txMode == context->txMode &&
                   confirm == context->confirm &&
                   SENSORARRAY_BLE_CCCD_ALLOWS_MODE(cccd, context->txMode);
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

/* New in this fix: count decimal characters without formatting trial headers.
 * The actual transmitted header still uses snprintf in SendFragment. */
static size_t sensorarrayBleDecimalChars(uint32_t value)
{
    size_t digits = 1u;
    while (value >= 10u) {
        value /= 10u;
        digits++;
    }
    return digits;
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
    (void)envChannel; /* %c is one byte for each C/D/L envelope. */
    (void)crc;        /* A uint32_t CRC is exactly eight %08lX digits. */
    /* NotifyInternal already limits accepted messages to 2304 bytes. Reject
     * unrepresentable future/internal inputs instead of narrowing size_t. */
#if SIZE_MAX > UINT32_MAX
    if (chunk > UINT32_MAX || messageLength > UINT32_MAX) {
        return 0u;
    }
#endif
    const size_t headerLen = SENSORARRAY_BLE_FRAGMENT_HEADER_FIXED_CHARS + 1u +
        SENSORARRAY_BLE_U32_HEX_MAX_CHARS +
        sensorarrayBleDecimalChars(messageId) +
        sensorarrayBleDecimalChars(index) +
        sensorarrayBleDecimalChars(count) +
        sensorarrayBleDecimalChars((uint32_t)chunk) +
        sensorarrayBleDecimalChars((uint32_t)messageLength);
    if (headerLen >= SENSORARRAY_BLE_FRAGMENT_HEADER_BUFFER_SIZE ||
        headerLen >= maximum) {
        return 0u;
    }
    return maximum - headerLen;
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

/* New in this fix: retain the planner's two notifications, but avoid splitting
 * an ordinary ~600-byte DATA record into two ~330-byte ATT values. With a
 * 251-byte LL payload, 495 + <=244 ATT bytes need three PDUs instead of four
 * (each notification also carries seven L2CAP/ATT bytes). This is a packing
 * target, not evidence of the peer's negotiated LL data length or delivery.
 * Other channels, ATT bounds and records that cannot meet both targets stay balanced.
 */
static size_t sensorarrayBleTwoFragmentFirstChunk(size_t length,
                                                  size_t maximum,
                                                  char envChannel,
                                                  uint32_t messageId,
                                                  uint32_t crc)
{
    const size_t balanced = length / 2u + length % 2u;
    const size_t firstValueMax = 495u;
    const size_t secondValueMax = 244u;
    if (envChannel != 'D' || maximum != 512u || length <= maximum) {
        return balanced;
    }

    /* Guess the largest possible chunk's decimal width. The resulting chunk
     * cannot have more digits; three bounded header calculations validate the
     * actual two chunks without a per-byte search or another SDK call. */
    size_t first = sensorarrayBleMaxPayloadSize(firstValueMax, envChannel,
        messageId, 0u, 2u, firstValueMax, length, crc);
    if (first == 0u || first >= length ||
        first > sensorarrayBleMaxPayloadSize(firstValueMax, envChannel,
            messageId, 0u, 2u, first, length, crc)) {
        return balanced;
    }
    size_t second = length - first;
    if (second < SENSORARRAY_BLE_TINY_TAIL_MIN ||
        second > sensorarrayBleMaxPayloadSize(secondValueMax, envChannel,
            messageId, 1u, 2u, second, length, crc)) {
        return balanced;
    }
    return first;
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
                             s_ble.connId == context->connId &&
                             (handle != s_ble.handles[BLE_IDX_DATA_TX_VALUE] ||
                              context->streamGeneration == s_ble.frameFlow.streamGeneration);
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

/* Caller holds s_bleMux. This only latches evidence/admission state: neither
 * the timer task nor a late-returning owner cancels or frees an SDK operation. */
static void sensorarrayBleObserveSendDeadlineLocked(uint64_t nowUs)
{
    if (s_ble.guard.ready && !s_ble.guard.quarantined && s_ble.link.sdkCallActive &&
        nowUs >= s_ble.link.sdkCallStartedUs &&
        nowUs - s_ble.link.sdkCallStartedUs >= SENSORARRAY_BLE_SEND_DEADLINE_US) {
        s_ble.guard.quarantined = true;
        s_ble.guard.callKnown = true;
        s_ble.guard.callId = s_ble.link.sdkCallCount;
        s_ble.guard.startUs = s_ble.link.sdkCallStartedUs;
        s_ble.guard.detectedUs = nowUs;
        s_ble.guard.completionKnown = false;
    }
}

void sensorarrayBleObserveSendDeadline(void)
{
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleObserveSendDeadlineLocked(nowUs);
    portEXIT_CRITICAL(&s_bleMux);
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

    const bool priority = context->priority || channel == SENSORARRAY_BLE_CH_CTRL;
    const uint64_t retryDeadlineUs = (uint64_t)esp_timer_get_time() +
        (uint64_t)SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS * 1000u;
    const uint32_t maximumAttempts = priority ?
        (SENSORARRAY_BLE_CTRL_TX_RETRY_TIMEOUT_MS /
         SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS) + 1u : 1u;
    esp_err_t lastError = ESP_FAIL;
    for (uint32_t attempt = 0u; attempt < maximumAttempts; ++attempt) {
        bool congested = false;
        if (!sensorarrayBleSendContextMatches(channel, context, &congested)) {
            portENTER_CRITICAL(&s_bleMux);
            if (!s_ble.guard.ready || s_ble.guard.quarantined) s_ble.guard.rejected++;
            else s_ble.stats.txConnectionStaleDrop++;
            portEXIT_CRITICAL(&s_bleMux);
            return ESP_ERR_INVALID_STATE;
        }

        if (!congested) {
            if (confirm) {
                sensorarrayBlePrepareConfirmation(context, handle);
            }
            uint32_t idleBefore = sensorarrayWorkBudgetIdleGeneration();
            uint64_t callStartedUs = (uint64_t)esp_timer_get_time();
            portENTER_CRITICAL(&s_bleMux);
            sensorarrayBleFrameFlowSyncLocked();
            /* Admission and the active mark are atomic against the observer.
             * This last admitted call retains its slot until the SDK returns,
             * including preemption between this mark and actual SDK entry. */
            bool callAdmitted = s_ble.guard.ready && !s_ble.guard.quarantined &&
                                !s_ble.link.sdkCallActive && s_ble.stats.connected && s_ble.stats.gattReady &&
                                context->connectionGeneration == s_ble.connectionGeneration &&
                                context->connId == s_ble.connId && context->gattsIf == s_ble.gattsIf &&
                                context->txMode == s_ble.txMode &&
                                context->confirm == SENSORARRAY_BLE_MODE_USES_CONFIRM(s_ble.cccd[channel], s_ble.txMode) &&
                                (channel != SENSORARRAY_BLE_CH_DATA ||
                                 context->streamGeneration == s_ble.frameFlow.streamGeneration) &&
                                SENSORARRAY_BLE_CCCD_ALLOWS_MODE(s_ble.cccd[channel], context->txMode);
            if (callAdmitted) {
                s_ble.link.sdkCallActive = true;
                s_ble.link.sdkCallStartedUs = callStartedUs;
                s_ble.link.sdkCallCount++;
                s_ble.guard.callKnown = true;
            } else {
                if (!s_ble.guard.ready || s_ble.guard.quarantined) s_ble.guard.rejected++;
                else s_ble.stats.txConnectionStaleDrop++;
            }
            portEXIT_CRITICAL(&s_bleMux);
            if (!callAdmitted) {
                if (confirm) sensorarrayBleClearConfirmation(xTaskGetCurrentTaskHandle());
                return ESP_ERR_INVALID_STATE;
            }
            lastError = esp_ble_gatts_send_indicate(context->gattsIf,
                                                    context->connId,
                                                    handle,
                                                    length,
                                                    value,
                                                    confirm);
            uint64_t callCompletedUs = (uint64_t)esp_timer_get_time();
            uint64_t callUs = callCompletedUs - callStartedUs;
            uint32_t idleAfter = sensorarrayWorkBudgetIdleGeneration();
            portENTER_CRITICAL(&s_bleMux);
            /* Also catch a late return between timer polls. */
            sensorarrayBleObserveSendDeadlineLocked(callCompletedUs);
            s_ble.link.sdkCallActive = false;
            s_ble.link.sdkCallCompletedUs = callCompletedUs;
            s_ble.link.sdkLastError = lastError;
            s_ble.link.sdkCompletedCalls++;
            s_ble.link.sdkWallUs += callUs;
            if (idleBefore == 0u || idleAfter == 0u) { s_ble.link.sdkIdleUnknown++; }
            else if (idleBefore != idleAfter) { s_ble.link.sdkIdleChanged++; }
            else { s_ble.link.sdkIdleUnchanged++; }
            if (callUs > s_ble.link.sdkMaxCallUs) {
                s_ble.link.sdkMaxCallUs = callUs > UINT32_MAX ? UINT32_MAX : (uint32_t)callUs;
            }
            if (lastError == ESP_OK) { s_ble.link.sdkAccepted++; }
            else { s_ble.link.sdkErrors++; }
            bool quarantined = s_ble.guard.quarantined;
            if (quarantined && s_ble.guard.callId == s_ble.link.sdkCallCount &&
                s_ble.guard.startUs == callStartedUs) {
                s_ble.guard.completionKnown = true;
            }
            portEXIT_CRITICAL(&s_bleMux);
            if (quarantined) {
                if (confirm) sensorarrayBleClearConfirmation(xTaskGetCurrentTaskHandle());
                return ESP_ERR_TIMEOUT;
            }
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
        } else if (channel == SENSORARRAY_BLE_CH_DATA && !priority) {
            portENTER_CRITICAL(&s_bleMux);
            s_ble.ownerCapacityFailure = true;
            portEXIT_CRITICAL(&s_bleMux);
        }

        /* ESP_FAIL can be rejected admission/congestion. The public IDF 5.5
         * implementation can itself wait indefinitely for BTC queue space:
         * this deadline bounds our retries, not that SDK call. Keep the slot
         * owned across it. The independent observer quarantines later calls;
         * it does not make this SDK call return or establish peer receipt. */
        if (lastError != ESP_FAIL) {
            break;
        }
        if (attempt + 1u >= maximumAttempts ||
            (uint64_t)esp_timer_get_time() +
                (uint64_t)SENSORARRAY_BLE_CTRL_TX_RETRY_INTERVAL_MS * 1000u >=
                retryDeadlineUs) {
            if (priority) {
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
    bool contextCurrent = context &&
        (context->priority || channel == SENSORARRAY_BLE_CH_CTRL) ?
        sensorarrayBleSendContextMatches(channel, context, NULL) :
        sensorarrayBleSendContextCurrent(channel, context);
    if (!context || !contextCurrent) {
        bool congested = false;
        bool capacity = context && sensorarrayBleSendContextMatches(channel, context, &congested) &&
            congested && context->txMode == SENSORARRAY_BLE_TX_FAST && !context->priority &&
            channel == SENSORARRAY_BLE_CH_DATA;
        portENTER_CRITICAL(&s_bleMux);
        if (capacity) s_ble.ownerCapacityFailure = true;
        else if (!s_ble.guard.ready || s_ble.guard.quarantined) s_ble.guard.rejected++;
        else s_ble.stats.txConnectionStaleDrop++;
        portEXIT_CRITICAL(&s_bleMux);
        return capacity ? ESP_ERR_TIMEOUT : ESP_ERR_INVALID_STATE;
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
                                           uint32_t expectedConnectionGeneration,
                                           bool priority,
                                           uint32_t expectedStreamGeneration)
{
    if (!data || channel > SENSORARRAY_BLE_CH_CTRL || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayBleSendContext_t context = {0};
    if (!sensorarrayBleSnapshotSendContext(channel,
                                           expectedConnectionGeneration,
                                           &context)) {
        portENTER_CRITICAL(&s_bleMux);
        if (!s_ble.guard.ready || s_ble.guard.quarantined) s_ble.guard.rejected++;
        else s_ble.stats.txConnectionStaleDrop++;
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_STATE;
    }
    if (channel == SENSORARRAY_BLE_CH_DATA &&
        context.streamGeneration != expectedStreamGeneration) {
        sensorarrayBleStatsIncrement(&s_ble.stats.txConnectionStaleDrop);
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_STATE;
    }
    context.priority = priority;
    if (context.congested && context.txMode == SENSORARRAY_BLE_TX_FAST &&
        channel != SENSORARRAY_BLE_CH_CTRL && !priority) {
        sensorarrayBleStatsIncrement(&s_ble.stats.congestedCount);
        portENTER_CRITICAL(&s_bleMux);
        s_ble.ownerCapacityFailure = true;
        portEXIT_CRITICAL(&s_bleMux);
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
        bool confirm = context.confirm;
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
    size_t firstChunk = fragmentCount == 2u ?
        sensorarrayBleTwoFragmentFirstChunk(length, maximum, envChannel,
                                            messageId, crc) : 0u;
    size_t offset = 0u;
    uint32_t fragmentsSinceCheckpoint = 0u;
    TickType_t checkpointTick = xTaskGetTickCount();
    bool confirm = context.confirm;
    for (uint32_t index = 0u; index < fragmentCount; ++index) {
        size_t chunk = base + (index < extra ? 1u : 0u);
        if (fragmentCount == 2u) {
            chunk = index == 0u ? firstChunk : length - firstChunk;
        }
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
        fragmentsSinceCheckpoint++;
        if (index + 1u < fragmentCount &&
            (fragmentsSinceCheckpoint >= SENSORARRAY_BLE_TX_FRAGMENT_BATCH_COUNT ||
             (TickType_t)(xTaskGetTickCount() - checkpointTick) >= 1u)) {
            /* Retain this descriptor/slot until all fragments finish. The
             * descriptor boundary already counts as a work unit: charging
             * every fast fragment again used the idle grace halfway through
             * ordinary three-fragment frames. Keep larger/slow messages
             * bounded by four fragments or one elapsed tick between checks.
             * Every fragment still checks its connection generation, even
             * inside a batch. No BLE mux or SDK lock is held at this point. */
            (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
            fragmentsSinceCheckpoint = 0u;
            checkpointTick = xTaskGetTickCount();
        }
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
    uint64_t ageUs = (uint64_t)esp_timer_get_time() - desc->queuedAtUs;
    sensorarrayBleFrameMeta_t frameMeta = {0};
    portENTER_CRITICAL(&s_bleMux);
    s_ble.ownerCapacityFailure = false;
    if (desc->typedFrame) {
        frameMeta = desc->protectedSlot ? s_ble.protectedMeta[desc->slotIndex] :
            s_ble.dataFrameMeta[desc->slotIndex];
        if (s_ble.frameFlow.floorKnown &&
            sensorarrayBleFrameSameBoot(&frameMeta.identity, &s_ble.frameFlow.floor) &&
            (frameMeta.identity.sequence == s_ble.frameFlow.floor.sequence ||
             sensorarrayBleFrameSequenceBefore(frameMeta.identity.sequence, s_ble.frameFlow.floor.sequence))) {
            if (desc->protectedSlot) s_ble.frameFlow.protectedSuperseded++;
            else s_ble.frameFlow.staleNormal++;
            portEXIT_CRITICAL(&s_bleMux);
            sensorarrayBleNoteDrop(desc->channel);
            return ESP_ERR_INVALID_STATE;
        }
    }
    if (ageUs > s_ble.link.queueMaxAgeUs[desc->channel]) {
        s_ble.link.queueMaxAgeUs[desc->channel] =
            ageUs > UINT32_MAX ? UINT32_MAX : (uint32_t)ageUs;
    }
    portEXIT_CRITICAL(&s_bleMux);
    esp_err_t result = sensorarrayBleSendPayload(desc->channel,
                                     data,
                                     desc->length,
                                     desc->crc,
                                     desc->connectionGeneration,
                                     desc->ctrlSlot != 0u,
                                     desc->streamGeneration);
    if (desc->typedFrame) {
        portENTER_CRITICAL(&s_bleMux);
        sensorarrayBleFrameFlowSyncLocked();
        /* Policy timestamps are sampled after acquiring its serialization
         * lock; another core may have recorded a later observation meanwhile. */
        uint64_t completedUs = (uint64_t)esp_timer_get_time();
        sensorarrayBleFrameFlow_t *flow = &s_ble.frameFlow;
        bool current = desc->connectionGeneration == s_ble.connectionGeneration &&
            desc->streamGeneration == flow->streamGeneration && flow->active && flow->identityKnown &&
            sensorarrayBleFrameSameBoot(&frameMeta.identity, &flow->identity);
        if (current) {
            if (result == ESP_OK && frameMeta.completeBundle) flow->counters.wholeCompleted++;
            else if (result != ESP_OK && s_ble.ownerCapacityFailure) flow->counters.capacityRejected++;
            else if (result != ESP_OK) flow->counters.otherFailures++;
            uint8_t acknowledged = frameMeta.completeBundle ?
                sensorarrayBleOutputScheduleNoteCompletion(&flow->schedule, completedUs,
                    &frameMeta.ticket, &frameMeta.ticket, result == ESP_OK) : 0u;
            if (desc->protectedSlot) {
                if (result == ESP_OK && acknowledged) {
                    flow->protectedComplete++;
                    if (flow->floorKnown && sensorarrayBleFrameSameConfig(&flow->floor, &frameMeta.identity) &&
                        sensorarrayBleFrameSameConfig(&flow->identity, &frameMeta.identity) &&
                        completedUs >= flow->lastProtectedCompleteUs) {
                        flow->gapKnown = true;
                        if (completedUs - flow->lastProtectedCompleteUs > flow->maxCompleteGapUs)
                            flow->maxCompleteGapUs = completedUs - flow->lastProtectedCompleteUs;
                    }
                    flow->lastProtectedCompleteUs = completedUs;
                    if (!flow->floorKnown || !sensorarrayBleFrameSameBoot(&flow->floor, &frameMeta.identity) ||
                        sensorarrayBleFrameSequenceBefore(flow->floor.sequence, frameMeta.identity.sequence)) {
                        flow->floor = frameMeta.identity;
                        flow->floorKnown = true;
                    }
                    if (sensorarrayBleFrameSameConfig(&frameMeta.identity, &flow->identity)) {
                        if (acknowledged & SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG) flow->firstPending = false;
                        if ((acknowledged & SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC) &&
                            frameMeta.periodicGeneration == flow->periodicGeneration) flow->periodicPending = false;
                    }
                } else {
                    flow->protectedFailed++;
                }
            }
            if (result == ESP_OK && frameMeta.completeBundle)
                sensorarrayBleFrameFullCompletedLocked(&frameMeta.identity, completedUs);
            sensorarrayBleFrameFlowObserveLocked(completedUs);
        } else if (desc->protectedSlot) {
            flow->protectedFailed++;
        }
        portEXIT_CRITICAL(&s_bleMux);
    }
    return result;
}

static bool sensorarrayBleReceiveProtected(sensorarrayBleTxDescriptor_t *desc)
{
    bool found = false;
    portENTER_CRITICAL(&s_bleMux);
    int selected = -1;
    for (unsigned i = 0u; i < SENSORARRAY_BLE_PROTECTED_SLOT_COUNT; ++i) {
        if (s_ble.protectedPhase[i] != BLE_FRAME_PENDING) continue;
        bool first = (s_ble.protectedMeta[i].ticket.protectionFlags & SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG) != 0u;
        bool selectedFirst = selected >= 0 &&
            (s_ble.protectedMeta[selected].ticket.protectionFlags & SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG) != 0u;
        if (selected < 0 || (first && !selectedFirst) ||
            (first == selectedFirst &&
             sensorarrayBleFrameSameBoot(&s_ble.protectedMeta[i].identity, &s_ble.protectedMeta[selected].identity) &&
             sensorarrayBleFrameSequenceBefore(s_ble.protectedMeta[i].identity.sequence,
                                               s_ble.protectedMeta[selected].identity.sequence))) selected = (int)i;
    }
    if (selected >= 0) {
        s_ble.protectedPhase[selected] = BLE_FRAME_OWNED;
        *desc = s_ble.protectedDescriptors[selected];
        s_ble.link.dataDequeued++;
        found = true;
    }
    portEXIT_CRITICAL(&s_bleMux);
    return found;
}

/* New in this repair: CTRL and lifecycle records retain reserved priority.
 * Ordinary diagnostic LOG is best effort and may remain queued while DATA is
 * backlogged; it must not force a scientific frame to wait every four records.
 * Existing bounded LOG slots count rejected admissions. Once selected, a
 * record retains ownership through its fragments; no in-flight preemption. */
static bool sensorarrayBleReceiveNext(sensorarrayBleTxDescriptor_t *desc)
{
    if (s_ble.ctrlTxQueue && xQueueReceive(s_ble.ctrlTxQueue, desc, 0) == pdTRUE) {
        sensorarrayBleStatsIncrement(&s_ble.link.priorityDequeued);
        return true;
    }
    if (sensorarrayBleReceiveProtected(desc)) return true;
    if (s_ble.txQueue && xQueueReceive(s_ble.txQueue, desc, 0) == pdTRUE) {
        sensorarrayBleStatsIncrement(&s_ble.link.dataDequeued);
        return true;
    }
    if (s_ble.logTxQueue && xQueueReceive(s_ble.logTxQueue, desc, 0) == pdTRUE) {
        sensorarrayBleStatsIncrement(&s_ble.link.logDequeued);
        return true;
    }
    return false;
}

static void sensorarrayBleTxTask(void *arg)
{
    (void)arg;
    sensorarrayBleTxDescriptor_t desc;
    for (;;) {
        if (!sensorarrayBleReceiveNext(&desc)) {
            /* An enqueue between the last queue check and this take leaves a
             * token. CTRL, DATA and LOG all wake this same owner immediately.
             * No work token is consumed by SAFE indication confirmation. */
            (void)xSemaphoreTake(s_ble.txWake, portMAX_DELAY);
            continue;
        }
        /* Successful notify, congestion, stale descriptors and short failures
         * all consume the same persistent budget. A CTRL flood cannot bypass
         * this checkpoint through the continue below. */
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_TX);
        (void)sensorarrayBleSendDescriptor(&desc);
        sensorarrayBleReleaseTxDescriptor(&desc);
    }
}

static esp_err_t sensorarrayBleQueueTxDescriptor(sensorarrayBleTxDescriptor_t *desc)
{
    if (!desc) {
        return ESP_ERR_INVALID_ARG;
    }
    QueueHandle_t queue = desc->ctrlSlot ? s_ble.ctrlTxQueue :
        (desc->channel == SENSORARRAY_BLE_CH_DATA ? s_ble.txQueue : s_ble.logTxQueue);
    if (!queue) {
        sensorarrayBleReleaseTxDescriptor(desc);
        sensorarrayBleNoteDrop(desc->channel);
        return ESP_ERR_INVALID_STATE;
    }
    desc->queuedAtUs = (uint64_t)esp_timer_get_time();
    if (xQueueSend(queue, desc, 0) == pdTRUE) {
        sensorarrayBleStatsIncrement(&s_ble.stats.messageQueued);
        (void)xSemaphoreGive(s_ble.txWake);
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
            (void)xSemaphoreGive(s_ble.txWake);
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
    while (sensorarrayBleReceiveProtected(&desc)) {
        sensorarrayBleNoteDrop(desc.channel);
        sensorarrayBleReleaseTxDescriptor(&desc);
    }
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
    while (s_ble.logTxQueue &&
           xQueueReceive(s_ble.logTxQueue, &desc, 0) == pdTRUE) {
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
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_BLE_CTRL);
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

/* Caller owns s_bleMux. Unknown is explicit until a completion is observed;
 * a preference request alone must never label the link as 2M/short-interval. */
static void sensorarrayBleResetLinkObservationLocked(void)
{
    s_ble.link.phyKnown = false;
    s_ble.link.connParamsKnown = false;
    s_ble.stats.txPhy = 0u;
    s_ble.stats.rxPhy = 0u;
    s_ble.link.txPhy = 0u;
    s_ble.link.rxPhy = 0u;
    s_ble.link.connIntervalUnits = 0u;
    s_ble.link.connLatency = 0u;
    s_ble.link.supervisionTimeoutUnits = 0u;
    s_ble.link.phyRequestError = ESP_ERR_INVALID_STATE;
    s_ble.link.phyFallbackRequestError = ESP_ERR_INVALID_STATE;
    s_ble.link.phyReadRequestError = ESP_ERR_INVALID_STATE;
    s_ble.link.connRequestError = ESP_ERR_INVALID_STATE;
    s_ble.link.dataLengthRequestError = ESP_ERR_INVALID_STATE;
    s_ble.link.phySetStatus = UINT16_MAX;
    s_ble.link.phyReadStatus = UINT16_MAX;
    s_ble.link.phyUpdateStatus = UINT16_MAX;
    s_ble.link.connUpdateStatus = UINT16_MAX;
    s_ble.link.dataLengthStatus = UINT16_MAX;
    s_ble.link.dataLengthTx = 0u;
    s_ble.link.dataLengthRx = 0u;
    s_ble.link.dataLengthObserved = false;
    s_ble.link.dataLengthConnectionKnown = false;
}

static void sensorarrayBleRequestLinkParameters(void)
{
    esp_bd_addr_t remote;
    portENTER_CRITICAL(&s_bleMux);
    memcpy(remote, s_ble.remote, sizeof(remote));
    bool prefer2m = s_ble.config.preferPhy2m;
    uint32_t generation = s_ble.connectionGeneration;
    portEXIT_CRITICAL(&s_bleMux);
    esp_ble_gap_phy_mask_t phy = prefer2m ?
        ESP_BLE_GAP_PHY_2M_PREF_MASK : ESP_BLE_GAP_PHY_1M_PREF_MASK;
    esp_err_t phyError = esp_ble_gap_set_preferred_phy(
        remote, 0u, phy, phy, ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
    esp_err_t fallbackError = ESP_ERR_INVALID_STATE;
    if (phyError != ESP_OK && prefer2m) {
        fallbackError = esp_ble_gap_set_preferred_phy(
            remote, 0u, ESP_BLE_GAP_PHY_1M_PREF_MASK,
            ESP_BLE_GAP_PHY_1M_PREF_MASK, ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
    }
    esp_err_t readError = esp_ble_gap_read_phy(remote);
    esp_ble_conn_update_params_t conn = {0};
    memcpy(conn.bda, remote, sizeof(conn.bda));
    conn.min_int = 0x06; /* Request 7.5 ms; only the actual callback confirms it. */
    conn.max_int = 0x06; /* A 15 ms maximum can no-op when already at 15 ms. */
    conn.latency = 0u;
    conn.timeout = 400u; /* 4 s supervision timeout. */
    esp_err_t connError = esp_ble_gap_update_conn_params(&conn);
    esp_err_t dataLengthError = esp_ble_gap_set_pkt_data_len(remote, 251u);
    portENTER_CRITICAL(&s_bleMux);
    if (s_ble.stats.connected && s_ble.connectionGeneration == generation) {
        s_ble.link.phyRequestError = phyError;
        s_ble.link.phyFallbackRequestError = fallbackError;
        s_ble.link.phyReadRequestError = readError;
        s_ble.link.connRequestError = connError;
        s_ble.link.dataLengthRequestError = dataLengthError;
    }
    portEXIT_CRITICAL(&s_bleMux);
}

/* New in this repair. GAP events with an address must match the active peer.
 * DLE/preferred-PHY-command completion events have no address in this SDK;
 * record them without claiming a negotiated current-connection identity. */
static bool sensorarrayBleObserveLinkEvent(esp_gap_ble_cb_event_t event,
                                           const esp_ble_gap_cb_param_t *param)
{
    if (!param) { return false; }
    const uint8_t *peer = NULL;
    switch (event) {
    case ESP_GAP_BLE_READ_PHY_COMPLETE_EVT: peer = param->read_phy.bda; break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT: peer = param->phy_update.bda; break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: peer = param->update_conn_params.bda; break;
    case ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT:
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT: break;
    default: return false;
    }
    portENTER_CRITICAL(&s_bleMux);
    if (!s_ble.stats.connected || (peer && memcmp(peer, s_ble.remote, sizeof(s_ble.remote)) != 0)) {
        s_ble.link.staleLinkEvents++;
        portEXIT_CRITICAL(&s_bleMux);
        return true;
    }
    switch (event) {
    case ESP_GAP_BLE_READ_PHY_COMPLETE_EVT:
        s_ble.link.phyReadStatus = (uint16_t)param->read_phy.status;
        if (param->read_phy.status == ESP_BT_STATUS_SUCCESS) {
            s_ble.link.phyKnown = true;
            s_ble.link.txPhy = s_ble.stats.txPhy = param->read_phy.tx_phy;
            s_ble.link.rxPhy = s_ble.stats.rxPhy = param->read_phy.rx_phy;
        }
        break;
    case ESP_GAP_BLE_SET_PREFERRED_PHY_COMPLETE_EVT:
        s_ble.link.phySetStatus = (uint16_t)param->set_perf_phy.status;
        break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        s_ble.link.phyUpdateEvents++;
        s_ble.link.phyUpdateStatus = (uint16_t)param->phy_update.status;
        if (param->phy_update.status == ESP_BT_STATUS_SUCCESS) {
            s_ble.link.phyKnown = true;
            s_ble.link.txPhy = s_ble.stats.txPhy = param->phy_update.tx_phy;
            s_ble.link.rxPhy = s_ble.stats.rxPhy = param->phy_update.rx_phy;
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        s_ble.link.connUpdateEvents++;
        s_ble.link.connUpdateStatus = (uint16_t)param->update_conn_params.status;
        if (param->update_conn_params.status == ESP_BT_STATUS_SUCCESS) {
            s_ble.link.connParamsKnown = true;
            s_ble.link.connIntervalUnits = param->update_conn_params.conn_int;
            s_ble.link.connLatency = param->update_conn_params.latency;
            s_ble.link.supervisionTimeoutUnits = param->update_conn_params.timeout;
        }
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        s_ble.link.dataLengthEvents++;
        s_ble.link.dataLengthStatus = (uint16_t)param->pkt_data_length_cmpl.status;
        s_ble.link.dataLengthObserved = true;
        s_ble.link.dataLengthConnectionKnown = false;
        s_ble.link.dataLengthTx = param->pkt_data_length_cmpl.params.tx_len;
        s_ble.link.dataLengthRx = param->pkt_data_length_cmpl.params.rx_len;
        break;
    default: break;
    }
    portEXIT_CRITICAL(&s_bleMux);
    return true;
}

static void sensorarrayBleGapCallback(esp_gap_ble_cb_event_t event,
                                      esp_ble_gap_cb_param_t *param)
{
    if (sensorarrayBleObserveLinkEvent(event, param)) { return; }
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
        memcpy(s_ble.remote, param->connect.remote_bda, sizeof(s_ble.remote));
        sensorarrayBleResetLinkObservationLocked();
        sensorarrayBleFrameFlowSyncLocked();
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleRequestLinkParameters();
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT: {
        TaskHandle_t confirmTask = NULL;
        portENTER_CRITICAL(&s_bleMux);
        s_ble.stats.connected = false;
        s_ble.stats.congested = false;
        s_ble.stats.mtu = SENSORARRAY_BLE_DEFAULT_ATT_MTU;
        s_ble.connectionGeneration = sensorarrayBleNextGeneration(s_ble.connectionGeneration);
        sensorarrayBleResetLinkObservationLocked();
        memset(s_ble.cccd, 0, sizeof(s_ble.cccd));
        sensorarrayBleFrameFlowSyncLocked();
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
            sensorarrayBleFrameFlowSyncLocked();
            portEXIT_CRITICAL(&s_bleMux);
        } else if (param->write.handle == s_ble.handles[BLE_IDX_CTRL_RX_VALUE]) {
            sensorarrayBleQueueControlCommand(param);
        }
        break;
    }
    case ESP_GATTS_CONF_EVT: {
        TaskHandle_t confirmTask = NULL;
        portENTER_CRITICAL(&s_bleMux);
        /* Count SDK events independently of the indication waiter. A FAST
         * notification can also produce a local CONF status; this is not a
         * peer/application receipt counter or proven connection attribution. */
        if (param->conf.status == ESP_GATT_OK) {
            s_ble.link.confOk++;
        } else {
            s_ble.link.confError++;
        }
        if (s_ble.confirmPending && s_ble.confirmWaitTask != NULL) {
            bool matches = s_ble.confirmConnectionGeneration ==
                               s_ble.connectionGeneration &&
                           s_ble.confirmConnId == param->conf.conn_id &&
                           s_ble.confirmHandle == param->conf.handle;
            if (matches) {
                s_ble.link.confMatched++;
                s_ble.confirmStatus = param->conf.status;
                s_ble.confirmPending = false;
                confirmTask = s_ble.confirmWaitTask;
            } else {
                s_ble.stats.staleConfirmation++;
            }
        }
        if (confirmTask == NULL) {
            s_ble.link.confUnmatched++;
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

/* Separate from s_ble: a failed startup may already own a timer/task/SDK
 * object. Never erase those references and retry creation in the same boot. */
static bool s_bleInitAttempted;
static esp_err_t s_bleInitResult = ESP_ERR_INVALID_STATE;

static esp_err_t sensorarrayBleFinishInit(esp_err_t error)
{
    portENTER_CRITICAL(&s_bleMux);
    s_bleInitResult = error;
    if (error == ESP_OK) s_ble.initialized = true;
    else s_ble.stats.initError = error;
    portEXIT_CRITICAL(&s_bleMux);
    return error;
}

/* New in this repair: one allocation, after the serialized init claim and
 * before any worker/SDK can own a slot. This is not a DMA buffer. Fragment
 * formatting still uses internal txPacketScratch; Bluedroid synchronously
 * copies GATT values. The allocation is retained until reboot even if later
 * initialization fails. Sticky init prevents live reset/reallocation, while
 * slot reuse still requires the existing producer/worker/SDK-return fences. */
static esp_err_t sensorarrayBleInitPayloadStorage(void)
{
    uint8_t *storage = heap_caps_malloc(SENSORARRAY_BLE_PAYLOAD_STORAGE_BYTES,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!storage) {
        return ESP_ERR_NO_MEM;
    }
    portENTER_CRITICAL(&s_bleMux);
    s_ble.payloadStorage = storage;
    size_t offset = 0u;
    for (unsigned i = 0u; i < CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN; ++i) {
        s_ble.dataSlots[i].data = storage + offset;
        offset += SENSORARRAY_BLE_MESSAGE_VALUE_MAX;
    }
    for (unsigned i = 0u; i < SENSORARRAY_BLE_LOG_TX_SLOT_COUNT; ++i) {
        s_ble.logSlots[i].data = storage + offset;
        offset += SENSORARRAY_BLE_MESSAGE_VALUE_MAX;
    }
    for (unsigned i = 0u; i < SENSORARRAY_BLE_PROTECTED_SLOT_COUNT; ++i) {
        s_ble.protectedSlots[i].data = storage + offset;
        offset += SENSORARRAY_BLE_MESSAGE_VALUE_MAX;
    }
    portEXIT_CRITICAL(&s_bleMux);
    return ESP_OK;
}

esp_err_t sensorarrayBleInit(const sensorarrayBleConfig_t *config)
{
    if (!config || config->deviceName[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_bleMux);
    if (s_bleInitAttempted) {
        esp_err_t result = s_bleInitResult;
        portEXIT_CRITICAL(&s_bleMux);
        return result;
    }
    s_bleInitAttempted = true;
    /* A concurrent caller observes in-progress, never a premature success. */
    s_bleInitResult = ESP_ERR_INVALID_STATE;
    portEXIT_CRITICAL(&s_bleMux);
    esp_err_t budgetErr = sensorarrayWorkBudgetInit();
    if (budgetErr != ESP_OK) {
        printf("WORKBUDGET,owner=ble_tx,err=0x%lx,fallback=bounded\n",
               (unsigned long)budgetErr);
    }
    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleControlRxCallback_t controlCallback = s_ble.controlCallback;
    void *controlContext = s_ble.controlContext;
    memset(&s_ble, 0, sizeof(s_ble));
    s_ble.controlCallback = controlCallback;
    s_ble.controlContext = controlContext;
    s_ble.config = *config;
    s_ble.stats.mtu = SENSORARRAY_BLE_DEFAULT_ATT_MTU;
    sensorarrayBleResetLinkObservationLocked();
    s_ble.stats.txTaskConfiguredBytes = CONFIG_SENSORARRAY_BLE_TX_TASK_STACK;
    s_ble.stats.ctrlTaskConfiguredBytes = CONFIG_SENSORARRAY_BLE_CTRL_TASK_STACK;
    s_ble.txMode = CONFIG_SENSORARRAY_BLE_TX_MODE_SAFE ?
        SENSORARRAY_BLE_TX_SAFE : SENSORARRAY_BLE_TX_FAST;
    s_ble.guard.initError = ESP_ERR_INVALID_STATE;
    portEXIT_CRITICAL(&s_bleMux);
    esp_err_t storageError = sensorarrayBleInitPayloadStorage();
    if (storageError != ESP_OK) {
        portENTER_CRITICAL(&s_bleMux);
        s_ble.guard.initError = storageError;
        portEXIT_CRITICAL(&s_bleMux);
        return sensorarrayBleFinishInit(storageError);
    }
    esp_err_t guardError = sensorarrayBleWatchdogInit();
    portENTER_CRITICAL(&s_bleMux);
    s_ble.guard.initError = guardError;
    s_ble.guard.ready = guardError == ESP_OK;
    portEXIT_CRITICAL(&s_bleMux);
    if (guardError != ESP_OK) {
        return sensorarrayBleFinishInit(guardError);
    }
    s_ble.txQueue = xQueueCreateStatic(CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN,
                                       sizeof(sensorarrayBleTxDescriptor_t),
                                       s_ble.txQueueStorage,
                                       &s_ble.txQueueStruct);
    s_ble.logTxQueue = xQueueCreateStatic(SENSORARRAY_BLE_LOG_TX_SLOT_COUNT,
                                          sizeof(sensorarrayBleTxDescriptor_t),
                                          s_ble.logTxQueueStorage,
                                          &s_ble.logTxQueueStruct);
    s_ble.ctrlTxQueue = xQueueCreateStatic(SENSORARRAY_BLE_CTRL_TX_SLOT_COUNT,
                                           sizeof(sensorarrayBleTxDescriptor_t),
                                           s_ble.ctrlTxQueueStorage,
                                           &s_ble.ctrlTxQueueStruct);
    s_ble.ctrlRxQueue = xQueueCreateStatic(CONFIG_SENSORARRAY_BLE_CTRL_RX_QUEUE_LEN,
                                           sizeof(sensorarrayBleControlCommand_t),
                                           s_ble.ctrlRxQueueStorage,
                                           &s_ble.ctrlRxQueueStruct);
    s_ble.txWake = xSemaphoreCreateBinaryStatic(&s_ble.txWakeStorage);
    if (!s_ble.txQueue || !s_ble.logTxQueue || !s_ble.ctrlTxQueue ||
        !s_ble.ctrlRxQueue || !s_ble.txWake) {
        return sensorarrayBleFinishInit(ESP_ERR_NO_MEM);
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
        return sensorarrayBleFinishInit(ESP_ERR_NO_MEM);
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
        return sensorarrayBleFinishInit(ESP_ERR_NO_MEM);
    }
    s_ble.controlTaskStarted = true;
    sensorarrayBleBuildAdvData();

    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err == ESP_ERR_INVALID_STATE) {
        err = ESP_OK;
    }
    if (sensorarrayBleStep("classic_release", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
    }

    esp_bt_controller_config_t controller = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&controller);
    if (sensorarrayBleStep("controller_init", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
    }
    err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (sensorarrayBleStep("controller_enable", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
    }
    portENTER_CRITICAL(&s_bleMux);
    s_ble.stats.controllerReady = true;
    portEXIT_CRITICAL(&s_bleMux);

    err = esp_bluedroid_init();
    if (sensorarrayBleStep("bluedroid_init", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
    }
    err = esp_bluedroid_enable();
    if (sensorarrayBleStep("bluedroid_enable", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
    }
    portENTER_CRITICAL(&s_bleMux);
    s_ble.stats.hostReady = true;
    portEXIT_CRITICAL(&s_bleMux);

    err = esp_ble_gap_register_callback(sensorarrayBleGapCallback);
    if (sensorarrayBleStep("gap_cb", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
    }
    err = esp_ble_gatts_register_callback(sensorarrayBleGattCallback);
    if (sensorarrayBleStep("gatts_cb", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
    }
    err = esp_ble_gatts_app_register(SENSORARRAY_BLE_APP_ID);
    if (sensorarrayBleStep("app_register", err) != ESP_OK) {
        return sensorarrayBleFinishInit(err);
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
    return sensorarrayBleFinishInit(ESP_OK);
}

static esp_err_t sensorarrayBleNotifyInternal(
    sensorarrayBleChannel_t channel,
    const uint8_t *data,
    size_t length,
    uint32_t expectedConnectionGeneration,
    bool priority)
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
    if (priority &&
        length > SENSORARRAY_BLE_CTRL_MESSAGE_VALUE_MAX) {
        sensorarrayBleNoteDrop(channel);
        return ESP_ERR_INVALID_SIZE;
    }
    uint32_t crc = sensorarrayBleCrc32(data, length);
    sensorarrayBleTxDescriptor_t desc;
    esp_err_t allocateErr = sensorarrayBleAllocateTxDescriptor(
        channel, length, crc, expectedConnectionGeneration, priority, &desc);
    if (allocateErr != ESP_OK) {
        /* Policy rejection already atomically counted its LOG drop. Returning
         * ESP_OK here would falsely label it accepted in transport statistics. */
        if (allocateErr != ESP_ERR_NOT_ALLOWED) { sensorarrayBleNoteDrop(channel); }
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
    return sensorarrayBleNotifyInternal(channel, data, length, 0u,
                                        channel == SENSORARRAY_BLE_CH_CTRL);
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
                                        expectedConnectionGeneration,
                                        channel == SENSORARRAY_BLE_CH_CTRL);
}

esp_err_t sensorarrayBleNotifyFrameForGeneration(
    const sensorarrayBleFramePublication_t *publication, uint32_t expectedConnectionGeneration)
{
    if (!publication || !publication->fullData || !publication->normalData || !expectedConnectionGeneration ||
        !publication->fullLength || !publication->normalLength ||
        publication->fullLength > SENSORARRAY_BLE_MESSAGE_VALUE_MAX ||
        publication->normalLength > SENSORARRAY_BLE_MESSAGE_VALUE_MAX) return ESP_ERR_INVALID_ARG;
    sensorarrayBleTxDescriptor_t desc = {0};
    sensorarrayBleDataSlot_t *slot = NULL;
    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleFrameFlowSyncLocked();
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    sensorarrayBleFrameFlow_t *flow = &s_ble.frameFlow;
    if (!flow->active || !s_ble.txQueue || expectedConnectionGeneration != s_ble.connectionGeneration) {
        if (!s_ble.guard.ready || s_ble.guard.quarantined) s_ble.guard.rejected++;
        else if (expectedConnectionGeneration != s_ble.connectionGeneration) s_ble.stats.txConnectionStaleDrop++;
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleNoteDrop(SENSORARRAY_BLE_CH_DATA);
        return ESP_ERR_INVALID_STATE;
    }
    const sensorarrayBleFrameIdentity_t *identity = &publication->identity;
    bool olderConfig = flow->identityKnown && sensorarrayBleFrameSameBoot(identity, &flow->identity) &&
        identity->revision != flow->identity.revision &&
        sensorarrayBleFrameSequenceBefore(identity->sequence, flow->identity.sequence);
    bool obsolete = flow->floorKnown && sensorarrayBleFrameSameBoot(identity, &flow->floor) &&
        (identity->sequence == flow->floor.sequence ||
         sensorarrayBleFrameSequenceBefore(identity->sequence, flow->floor.sequence));
    if (olderConfig || obsolete) {
        flow->staleNormal++;
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleNoteDrop(SENSORARRAY_BLE_CH_DATA);
        return ESP_ERR_INVALID_STATE;
    }
    if (!flow->identityKnown || !sensorarrayBleFrameSameConfig(identity, &flow->identity)) {
        flow->identity = *identity; flow->identityKnown = true; flow->firstPending = true;
        flow->fullCompleteKnown = false;
        flow->periodicPending = false;
        flow->periodicDueUs = 0u;
        flow->periodicGeneration = sensorarrayBleNextGeneration(flow->periodicGeneration);
        flow->maxCompleteGapUs = 0u;
        flow->gapKnown = false;
    } else if (sensorarrayBleFrameSequenceBefore(flow->identity.sequence, identity->sequence)) {
        flow->identity.sequence = identity->sequence;
    }
    sensorarrayBleFramePeriodicLocked(nowUs);
    uint8_t protection = 0u;
    if (flow->firstPending && !sensorarrayBleFrameProtectedRepresentedLocked(SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG))
        protection |= SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG;
    if (flow->periodicPending && !sensorarrayBleFrameProtectedRepresentedLocked(SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC))
        protection |= SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC;
    sensorarrayBleFrameFlowObserveLocked(nowUs);
    sensorarrayBleOutputEpoch_t epoch = {.sessionId=identity->sessionId,.bootId=identity->bootId,
        .connectionGeneration=s_ble.connectionGeneration,.streamGeneration=flow->streamGeneration};
    sensorarrayBleOutputFrameKey_t key = {.revision=identity->revision,.sequence=identity->sequence};
    sensorarrayBleOutputTicket_t ticket = sensorarrayBleOutputScheduleDecide(&flow->schedule,nowUs,&epoch,&key,protection);
    if (ticket.decision == SENSORARRAY_BLE_OUTPUT_SKIP_POLICY) {
        flow->intentionalSkip++;
        portEXIT_CRITICAL(&s_bleMux);
        return ESP_ERR_NOT_ALLOWED;
    }
    if (ticket.decision == SENSORARRAY_BLE_OUTPUT_NO_STREAM) {
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleNoteDrop(SENSORARRAY_BLE_CH_DATA);
        return ESP_ERR_INVALID_STATE;
    }
    flow->counters.admissionAttempts++;
    bool protectedFrame = ticket.decision == SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED;
    const uint8_t *source = protectedFrame ? publication->fullData : publication->normalData;
    size_t length = protectedFrame ? publication->fullLength : publication->normalLength;
    int index = -1;
    if (protectedFrame) {
        for (unsigned i=0u;i<SENSORARRAY_BLE_PROTECTED_SLOT_COUNT;i++)
            if (s_ble.protectedPhase[i]==BLE_FRAME_FREE) { index=(int)i; break; }
        if (index<0) for (unsigned i=0u;i<SENSORARRAY_BLE_PROTECTED_SLOT_COUNT;i++) {
            if (s_ble.protectedPhase[i]!=BLE_FRAME_PENDING) continue;
            bool sameFirst = (s_ble.protectedMeta[i].ticket.protectionFlags & SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG) &&
                sensorarrayBleFrameSameConfig(&s_ble.protectedMeta[i].identity,identity);
            if (sameFirst) continue;
            index=(int)i; flow->protectedSuperseded++;
            s_ble.stats.dropped[SENSORARRAY_BLE_CH_DATA]++; s_ble.stats.messageDropped++;
            break;
        }
        if (index>=0) slot=&s_ble.protectedSlots[index];
    } else for (unsigned i=0u;i<CONFIG_SENSORARRAY_BLE_TX_QUEUE_LEN;i++)
        if (!s_ble.dataSlots[i].inUse) { index=(int)i; slot=&s_ble.dataSlots[i]; break; }
    if (!slot) {
        flow->counters.capacityRejected++; s_ble.stats.txSlotAllocFail++;
        if (protectedFrame) flow->protectedMissed++;
        portEXIT_CRITICAL(&s_bleMux);
        sensorarrayBleNoteDrop(SENSORARRAY_BLE_CH_DATA);
        return ESP_ERR_NO_MEM;
    }
    slot->inUse=true; slot->generation=sensorarrayBleNextGeneration(slot->generation);
    slot->channel=SENSORARRAY_BLE_CH_DATA; slot->length=(uint16_t)length; slot->crc=0u;
    desc=(sensorarrayBleTxDescriptor_t){.slotIndex=(uint8_t)index,.protectedSlot=protectedFrame,.typedFrame=true,
        .channel=SENSORARRAY_BLE_CH_DATA,.length=(uint16_t)length,.generation=slot->generation,
        .connectionGeneration=s_ble.connectionGeneration,.streamGeneration=flow->streamGeneration,.queuedAtUs=nowUs};
    sensorarrayBleFrameMeta_t meta={.identity=*identity,.ticket=ticket,.periodicGeneration=flow->periodicGeneration,
        .completeBundle=protectedFrame || (publication->normalData==publication->fullData && publication->normalLength==publication->fullLength)};
    if (protectedFrame) {
        s_ble.protectedPhase[index]=BLE_FRAME_WRITING; s_ble.protectedDescriptors[index]=desc; s_ble.protectedMeta[index]=meta;
    } else s_ble.dataFrameMeta[index]=meta;
    sensorarrayBleUpdateSlotUseLocked();
    portEXIT_CRITICAL(&s_bleMux);
    /* No caller/preview pointer escapes. WRITING cannot be superseded, taken
     * by the worker or reclaimed by disconnect while copying outside mux. */
    uint32_t crc=sensorarrayBleCrc32(source,length);
    memcpy(slot->data,source,length);
    portENTER_CRITICAL(&s_bleMux);
    sensorarrayBleFrameFlowSyncLocked();
    bool current=slot->inUse && slot->generation==desc.generation &&
        desc.connectionGeneration==s_ble.connectionGeneration &&
        desc.streamGeneration==s_ble.frameFlow.streamGeneration && s_ble.frameFlow.active;
    if (current) {
        slot->crc=desc.crc=crc;
        if (protectedFrame) {
            s_ble.protectedDescriptors[index]=desc; s_ble.protectedPhase[index]=BLE_FRAME_PENDING;
            s_ble.frameFlow.protectedQueued++; s_ble.stats.messageQueued++;
        }
    } else s_ble.stats.txConnectionStaleDrop++;
    portEXIT_CRITICAL(&s_bleMux);
    if (!current) {
        sensorarrayBleReleaseTxDescriptor(&desc); sensorarrayBleNoteDrop(SENSORARRAY_BLE_CH_DATA);
        return ESP_ERR_INVALID_STATE;
    }
    if (protectedFrame) { (void)xSemaphoreGive(s_ble.txWake); return ESP_OK; }
    esp_err_t result=sensorarrayBleQueueTxDescriptor(&desc);
    if (result!=ESP_OK) {
        portENTER_CRITICAL(&s_bleMux);
        if (desc.connectionGeneration==s_ble.connectionGeneration && desc.streamGeneration==s_ble.frameFlow.streamGeneration) {
            if (result==ESP_ERR_TIMEOUT) s_ble.frameFlow.counters.capacityRejected++;
            else s_ble.frameFlow.counters.otherFailures++;
        }
        portEXIT_CRITICAL(&s_bleMux);
    }
    return result;
}

esp_err_t sensorarrayBleNotifyLifecycleForGeneration(
    const uint8_t *data, size_t length, uint32_t expectedConnectionGeneration)
{
    if (expectedConnectionGeneration == 0u) { return ESP_ERR_INVALID_ARG; }
    return sensorarrayBleNotifyInternal(SENSORARRAY_BLE_CH_LOG, data, length,
                                        expectedConnectionGeneration, true);
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

void sensorarrayBleGetLinkSnapshot(sensorarrayBleLinkSnapshot_t *outSnapshot)
{
    if (!outSnapshot) { return; }
    portENTER_CRITICAL(&s_bleMux);
    *outSnapshot = s_ble.link;
    outSnapshot->connectionGeneration = s_ble.connectionGeneration;
    outSnapshot->connected = s_ble.stats.connected;
    outSnapshot->mtu = s_ble.stats.mtu;
    TaskHandle_t txTask = s_ble.txTask;
    TaskHandle_t controlTask = s_ble.controlTask;
    portEXIT_CRITICAL(&s_bleMux);
    /* Priorities are task configuration, not part of negotiated link state;
     * reading the kernel outside our mux avoids a lock-order dependency. */
    outSnapshot->txTaskPriority = txTask ? (uint32_t)uxTaskPriorityGet(txTask) : 0u;
    outSnapshot->ctrlTaskPriority = controlTask ? (uint32_t)uxTaskPriorityGet(controlTask) : 0u;
}

void sensorarrayBleGetGuardSnapshot(sensorarrayBleGuardSnapshot_t *outSnapshot)
{
    if (!outSnapshot) return;
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    portENTER_CRITICAL(&s_bleMux);
    *outSnapshot = s_ble.guard;
    outSnapshot->active = s_ble.link.sdkCallActive;
    if (!outSnapshot->quarantined) {
        outSnapshot->callId = s_ble.link.sdkCallCount;
        outSnapshot->startUs = s_ble.link.sdkCallStartedUs;
        outSnapshot->completionKnown = outSnapshot->callKnown && !outSnapshot->active;
    }
    outSnapshot->ageUs = outSnapshot->callKnown && nowUs >= outSnapshot->startUs ?
        nowUs - outSnapshot->startUs : 0u;
    outSnapshot->deadlineUs = SENSORARRAY_BLE_SEND_DEADLINE_US;
    outSnapshot->pollUs = SENSORARRAY_BLE_SEND_POLL_US;
    outSnapshot->restartRequired = outSnapshot->quarantined ||
        (!outSnapshot->ready && outSnapshot->initError != ESP_OK);
    portEXIT_CRITICAL(&s_bleMux);
}

void sensorarrayBleGetLogPolicySnapshot(sensorarrayBleLogPolicySnapshot_t *outSnapshot)
{
    if (!outSnapshot) { return; }
    portENTER_CRITICAL(&s_bleMux);
    bool dataActive = sensorarrayBleDataActiveLocked();
    bool logSendable = s_ble.guard.ready && !s_ble.guard.quarantined &&
                       s_ble.stats.gattReady && s_ble.stats.connected &&
                       SENSORARRAY_BLE_CCCD_ALLOWS_MODE(s_ble.cccd[SENSORARRAY_BLE_CH_LOG],
                                                       s_ble.txMode);
    *outSnapshot = (sensorarrayBleLogPolicySnapshot_t){
        .connectionGeneration = s_ble.connectionGeneration,
        .connected = s_ble.stats.connected,
        .txMode = s_ble.txMode,
        .dataCccd = s_ble.cccd[SENSORARRAY_BLE_CH_DATA],
        .dataActive = dataActive,
        .ordinaryAllowed = logSendable && !dataActive,
        .policyDrop = s_ble.stats.logPolicyDrop,
        .droppedLog = s_ble.stats.dropped[SENSORARRAY_BLE_CH_LOG],
    };
    portEXIT_CRITICAL(&s_bleMux);
}

void sensorarrayBleGetFlowSnapshot(sensorarrayBleFlowSnapshot_t *outSnapshot)
{
    if (!outSnapshot) return;
    portENTER_CRITICAL(&s_bleMux);
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    const sensorarrayBleFrameFlow_t *f = &s_ble.frameFlow;
    const sensorarrayBleOutputScheduleSnapshot_t *p = &f->schedule.snapshot;
    bool active = sensorarrayBleDataActiveLocked();
    *outSnapshot = (sensorarrayBleFlowSnapshot_t){
        .connectionGeneration=s_ble.connectionGeneration,.streamGeneration=f->streamGeneration,
        .stage=!active ? SENSORARRAY_BLE_OUTPUT_INACTIVE :
            (f->scheduleInitialized && p->active ? (uint8_t)p->mode : SENSORARRAY_BLE_OUTPUT_FULL),
        .ordinaryRatePermille=f->scheduleInitialized && p->active ? p->ordinaryRatePermille : 1000u,
        .capacityRejectedPermille=p->capacityRejectedPermille,.pressureWindows=p->pressureWindows,
        .active=active,.evidenceKnown=active && p->active && p->evidenceKnown,
        .serviceKnown=active && p->active && p->serviceKnown,
        .localServiceMilliFps=p->localServiceMilliFps,.admissionAttempts=f->counters.admissionAttempts,
        .capacityRejected=f->counters.capacityRejected,.intentionalSkip=p->policySkipped,
        .probeSelected=p->probeSelected,.cleanEvidenceUs=p->cleanUs,
        .protectedQueued=f->protectedQueued,.protectedComplete=f->protectedComplete,.protectedFailed=f->protectedFailed,
        .protectedAdmissionMiss=f->protectedMissed,.protectedDeadlineMiss=f->protectedDeadlineMissed,
        .protectedSuperseded=f->protectedSuperseded,.staleNormal=f->staleNormal,
        .lastCompleteKnown=f->floorKnown,.lastCompleteAgeKnown=f->floorKnown && nowUs>=f->lastProtectedCompleteUs,
        .gapKnown=f->gapKnown,
        .firstPending=f->firstPending,.periodicPending=f->periodicPending,
        .lastSequence=f->floor.sequence,.lastRevision=f->floor.revision,
        .lastCompleteAgeUs=nowUs>=f->lastProtectedCompleteUs ? nowUs-f->lastProtectedCompleteUs : 0u,
        .maxCompleteGapUs=f->maxCompleteGapUs,
    };
    portEXIT_CRITICAL(&s_bleMux);
}
