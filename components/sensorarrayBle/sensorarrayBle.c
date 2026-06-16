#include "sensorarrayBle.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_heap_caps.h"

#define SENSORARRAY_BLE_APP_ID 0x53u
#define SENSORARRAY_BLE_SERVICE_UUID 0x00FFu
#define SENSORARRAY_BLE_CTRL_RX_UUID 0xFF10u
#define SENSORARRAY_BLE_CTRL_TX_UUID 0xFF11u
#define SENSORARRAY_BLE_DATA_TX_UUID 0xFF20u
#define SENSORARRAY_BLE_LOG_TX_UUID 0xFF30u
#define SENSORARRAY_BLE_ATTR_VALUE_MAX 512u

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
    sensorarrayBleConfig_t config;
    sensorarrayBleStats_t stats;
    bool initialized;
    bool subscribed[3];
    uint32_t nextMessageId[3];
    esp_gatt_if_t gattsIf;
    uint16_t connId;
    uint16_t handles[BLE_IDX_COUNT];
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
                                ESP_GATT_CHAR_PROP_BIT_NOTIFY;
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
            s_ble.subscribed[channel] = value == 1u;
        } else if (param->write.handle == s_ble.handles[BLE_IDX_CTRL_RX_VALUE] &&
                   s_ble.controlCallback) {
            s_ble.controlCallback(param->write.value, param->write.len, s_ble.controlContext);
        }
        break;
    }
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
    if (!s_ble.stats.gattReady || !s_ble.stats.connected || !s_ble.subscribed[channel]) {
        s_ble.stats.dropped[channel]++;
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ble.stats.congested) {
        s_ble.stats.dropped[channel]++;
        return ESP_ERR_TIMEOUT;
    }
    size_t maximum = s_ble.stats.mtu > 3u ? (size_t)s_ble.stats.mtu - 3u : 20u;
    if (maximum > SENSORARRAY_BLE_ATTR_VALUE_MAX) {
        maximum = SENSORARRAY_BLE_ATTR_VALUE_MAX;
    }
    size_t payloadMax = maximum > 64u ? maximum - 64u : 1u;
    uint32_t fragmentCount = (uint32_t)((length + payloadMax - 1u) / payloadMax);
    if (fragmentCount == 0u) {
        fragmentCount = 1u;
    }
    uint32_t messageId = ++s_ble.nextMessageId[channel];
    if (messageId == 0u) {
        messageId = ++s_ble.nextMessageId[channel];
    }
    uint32_t crc = sensorarrayBleCrc32(data, length);
    char envChannel = sensorarrayBleEnvelopeChannel(channel);
    uint16_t handle = sensorarrayBleValueHandle(channel);
    uint8_t packet[SENSORARRAY_BLE_ATTR_VALUE_MAX];
    uint32_t fragmentIndex = 0u;
    for (size_t offset = 0u; offset < length; ++fragmentIndex) {
        size_t chunk = length - offset;
        if (chunk > payloadMax) {
            chunk = payloadMax;
        }
        int headerLen = snprintf((char *)packet,
                                 sizeof(packet),
                                 "G,%c,%lu,%lu,%lu,%u,%08lX\n",
                                 envChannel,
                                 (unsigned long)messageId,
                                 (unsigned long)fragmentIndex,
                                 (unsigned long)fragmentCount,
                                 (unsigned)chunk,
                                 (unsigned long)crc);
        if (headerLen <= 0 || (size_t)headerLen >= sizeof(packet) ||
            (size_t)headerLen + chunk > maximum) {
            s_ble.stats.dropped[channel]++;
            return ESP_ERR_INVALID_SIZE;
        }
        memcpy(packet + headerLen, data + offset, chunk);
        esp_err_t err = esp_ble_gatts_send_indicate(s_ble.gattsIf, s_ble.connId, handle,
                                                     (uint16_t)((size_t)headerLen + chunk),
                                                     packet, false);
        if (err != ESP_OK) {
            s_ble.stats.dropped[channel]++;
            return err;
        }
        offset += chunk;
    }
    s_ble.stats.sent[channel]++;
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
