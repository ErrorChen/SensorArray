#include "sensorarrayNetStatus.h"

#include <stdio.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_event.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

#include "sensorarrayConfig.h"

#ifndef CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE
#define CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE 0
#endif
#ifndef CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD
#define CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD ""
#endif
#ifndef CONFIG_SENSORARRAY_WIFI_STATUS_UDP_PORT
#define CONFIG_SENSORARRAY_WIFI_STATUS_UDP_PORT 3333
#endif
#ifndef CONFIG_SENSORARRAY_WIFI_TX_POWER_DBM_X4
#define CONFIG_SENSORARRAY_WIFI_TX_POWER_DBM_X4 80
#endif
#ifndef CONFIG_SENSORARRAY_BLE_ENABLE
#define CONFIG_SENSORARRAY_BLE_ENABLE 0
#endif
#ifndef CONFIG_SENSORARRAY_BLE_USE_2M_PHY
#define CONFIG_SENSORARRAY_BLE_USE_2M_PHY 0
#endif
#ifndef CONFIG_SENSORARRAY_BLE_USE_CODED_PHY
#define CONFIG_SENSORARRAY_BLE_USE_CODED_PHY 0
#endif
#ifndef CONFIG_SENSORARRAY_BLE_HIGH_TX_POWER
#define CONFIG_SENSORARRAY_BLE_HIGH_TX_POWER 0
#endif
#ifndef CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL
#define CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL 9
#endif
#ifndef CONFIG_SENSORARRAY_BLE_STATUS_NOTIFY_ENABLE
#define CONFIG_SENSORARRAY_BLE_STATUS_NOTIFY_ENABLE 0
#endif
#ifndef CONFIG_SENSORARRAY_NET_TASK_STACK
#define CONFIG_SENSORARRAY_NET_TASK_STACK 8192
#endif
#ifndef CONFIG_SENSORARRAY_NET_TASK_PRIORITY
#define CONFIG_SENSORARRAY_NET_TASK_PRIORITY 5
#endif

#define SENSORARRAY_NET_QUEUE_LEN 2u
#define SENSORARRAY_NET_STATUS_TEXT_MAX 384u
#define SENSORARRAY_BLE_APP_ID 0x53u
#define SENSORARRAY_BLE_SERVICE_UUID 0x00FFu
#define SENSORARRAY_BLE_STATUS_UUID 0xFF01u
#define SENSORARRAY_BLE_NOTIFY_UUID 0xFF02u
#define SENSORARRAY_BLE_HANDLE_COUNT 6u

enum {
    BLE_IDX_SERVICE = 0,
    BLE_IDX_STATUS_DECL,
    BLE_IDX_STATUS_VALUE,
    BLE_IDX_NOTIFY_DECL,
    BLE_IDX_NOTIFY_VALUE,
    BLE_IDX_NOTIFY_CCCD,
};

typedef struct __attribute__((packed)) {
    uint8_t version;
    uint8_t flags;
    uint16_t physFpsX100;
    uint32_t sequence;
    int32_t batteryMv;
    int32_t ain9OffsetUv;
    uint32_t netDropCount;
} sensorarrayBleNotifyV1_t;

typedef struct {
    bool started;
    bool wifiReady;
    bool wifiStationConnected;
    bool bleReady;
    bool bleAdvertising;
    bool bleConnected;
    bool bleSubscribed;
    uint16_t bleConnId;
    uint16_t bleMtu;
    uint16_t bleHandles[SENSORARRAY_BLE_HANDLE_COUNT];
    esp_gatt_if_t bleGattIf;
    esp_bd_addr_t bleRemote;
    uint8_t bleTxPhy;
    uint8_t bleRxPhy;
    uint32_t publishDropCount;
    uint32_t udpSendOk;
    uint32_t udpSendDrop;
    uint32_t bleNotifyOk;
    uint32_t bleNotifyDrop;
    uint32_t errorCount;
    char deviceName[32];
    char statusText[SENSORARRAY_NET_STATUS_TEXT_MAX];
} sensorarrayNetState_t;

static sensorarrayNetState_t s_net;
static StaticQueue_t s_statusQueueStruct;
static uint8_t s_statusQueueStorage[SENSORARRAY_NET_QUEUE_LEN * sizeof(sensorarrayNetStatus_t)];
static QueueHandle_t s_statusQueue;
static TaskHandle_t s_netTask;

static uint16_t s_primaryServiceUuid = ESP_GATT_UUID_PRI_SERVICE;
static uint16_t s_characterDeclarationUuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint16_t s_cccdUuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static uint16_t s_serviceUuid = SENSORARRAY_BLE_SERVICE_UUID;
static uint16_t s_statusUuid = SENSORARRAY_BLE_STATUS_UUID;
static uint16_t s_notifyUuid = SENSORARRAY_BLE_NOTIFY_UUID;
static uint8_t s_statusProperties = ESP_GATT_CHAR_PROP_BIT_READ;
static uint8_t s_notifyProperties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t s_initialStatus[] = "SensorArray status pending";
static uint8_t s_initialNotify[sizeof(sensorarrayBleNotifyV1_t)];
static uint8_t s_initialCccd[] = {0x00, 0x00};

static const esp_gatts_attr_db_t s_gattDb[SENSORARRAY_BLE_HANDLE_COUNT] = {
    [BLE_IDX_SERVICE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_primaryServiceUuid, ESP_GATT_PERM_READ,
         sizeof(s_serviceUuid), sizeof(s_serviceUuid), (uint8_t *)&s_serviceUuid},
    },
    [BLE_IDX_STATUS_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_characterDeclarationUuid, ESP_GATT_PERM_READ,
         sizeof(s_statusProperties), sizeof(s_statusProperties), &s_statusProperties},
    },
    [BLE_IDX_STATUS_VALUE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_statusUuid, ESP_GATT_PERM_READ,
         SENSORARRAY_NET_STATUS_TEXT_MAX, sizeof(s_initialStatus) - 1u, s_initialStatus},
    },
    [BLE_IDX_NOTIFY_DECL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_characterDeclarationUuid, ESP_GATT_PERM_READ,
         sizeof(s_notifyProperties), sizeof(s_notifyProperties), &s_notifyProperties},
    },
    [BLE_IDX_NOTIFY_VALUE] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_notifyUuid, ESP_GATT_PERM_READ,
         sizeof(s_initialNotify), sizeof(s_initialNotify), s_initialNotify},
    },
    [BLE_IDX_NOTIFY_CCCD] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&s_cccdUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
         sizeof(s_initialCccd), sizeof(s_initialCccd), s_initialCccd},
    },
};

#if CONFIG_BT_BLE_42_FEATURES_SUPPORTED
static esp_ble_adv_params_t s_advParams = {
    .adv_int_min = 0x40,
    .adv_int_max = 0x80,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_adv_data_t s_advData = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x10,
    .max_interval = 0x20,
    .service_uuid_len = sizeof(s_serviceUuid),
    .p_service_uuid = (uint8_t *)&s_serviceUuid,
    .flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT,
};
#endif

#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
static uint8_t s_extAdvRaw[31];
static uint8_t s_extAdvRawLength;
static esp_ble_gap_ext_adv_t s_extAdvSet[] = {
    {0u, 0u, 0u},
};
static esp_ble_gap_ext_adv_params_t s_extAdvParams = {
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
#endif

static void sensorarrayNetBuildDeviceName(void)
{
    uint8_t mac[6] = {0};
    (void)esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_net.deviceName, sizeof(s_net.deviceName),
             "SensorArray_%02X%02X%02X", mac[3], mac[4], mac[5]);
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    size_t nameLength = strlen(s_net.deviceName);
    if (nameLength > sizeof(s_extAdvRaw) - 9u) {
        nameLength = sizeof(s_extAdvRaw) - 9u;
    }
    const uint8_t prefix[] = {
        0x02, 0x01, 0x06,
        0x03, 0x03, (uint8_t)(SENSORARRAY_BLE_SERVICE_UUID & 0xFFu),
        (uint8_t)(SENSORARRAY_BLE_SERVICE_UUID >> 8u),
    };
    memcpy(s_extAdvRaw, prefix, sizeof(prefix));
    s_extAdvRaw[sizeof(prefix)] = (uint8_t)(nameLength + 1u);
    s_extAdvRaw[sizeof(prefix) + 1u] = 0x09u;
    memcpy(&s_extAdvRaw[sizeof(prefix) + 2u], s_net.deviceName, nameLength);
    s_extAdvRawLength = (uint8_t)(sizeof(prefix) + 2u + nameLength);
#endif
}

static void sensorarrayWifiEvent(void *arg,
                                 esp_event_base_t eventBase,
                                 int32_t eventId,
                                 void *eventData)
{
    (void)arg;
    (void)eventBase;
    (void)eventData;
    if (eventId == WIFI_EVENT_AP_STACONNECTED) {
        s_net.wifiStationConnected = true;
    } else if (eventId == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_sta_list_t stations = {0};
        s_net.wifiStationConnected =
            esp_wifi_ap_get_sta_list(&stations) == ESP_OK && stations.num != 0u;
    }
}

static esp_err_t sensorarrayWifiInit(void)
{
#if CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if (!esp_netif_create_default_wifi_ap()) {
        return ESP_ERR_NO_MEM;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init);
    if (err != ESP_OK) {
        return err;
    }
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, sensorarrayWifiEvent, NULL);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t config = {0};
    size_t ssidLen = strnlen(s_net.deviceName, sizeof(config.ap.ssid));
    memcpy(config.ap.ssid, s_net.deviceName, ssidLen);
    config.ap.ssid_len = (uint8_t)ssidLen;
    const char *password = CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD;
    size_t passwordLen = strlen(password);
    if (passwordLen > sizeof(config.ap.password)) {
        passwordLen = sizeof(config.ap.password);
    }
    memcpy(config.ap.password, password, passwordLen);
    config.ap.channel = 1u;
    config.ap.max_connection = 4u;
    config.ap.authmode = passwordLen >= 8u ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &config);
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    if (err == ESP_OK) {
        if (esp_wifi_set_max_tx_power(CONFIG_SENSORARRAY_WIFI_TX_POWER_DBM_X4) != ESP_OK) {
            s_net.errorCount++;
        }
        s_net.wifiReady = true;
    }
    return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void sensorarrayBleGapCallback(esp_gap_ble_cb_event_t event,
                                      esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#if CONFIG_BT_BLE_42_FEATURES_SUPPORTED
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        if (param->adv_data_cmpl.status != ESP_BT_STATUS_SUCCESS ||
            esp_ble_gap_start_advertising(&s_advParams) != ESP_OK) {
            s_net.errorCount++;
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        s_net.bleAdvertising = param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS;
        if (!s_net.bleAdvertising) {
            s_net.errorCount++;
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        s_net.bleAdvertising = false;
        break;
#endif
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
        if (param->ext_adv_set_params.status == ESP_BT_STATUS_SUCCESS) {
            if (esp_ble_gap_config_ext_adv_data_raw(0u, s_extAdvRawLength, s_extAdvRaw) != ESP_OK) {
                s_net.errorCount++;
            }
        } else {
            s_net.errorCount++;
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
        if (param->ext_adv_data_set.status == ESP_BT_STATUS_SUCCESS) {
            if (esp_ble_gap_ext_adv_start(1u, s_extAdvSet) != ESP_OK) {
                s_net.errorCount++;
            }
        } else {
            s_net.errorCount++;
        }
        break;
    case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
        s_net.bleAdvertising = param->ext_adv_start.status == ESP_BT_STATUS_SUCCESS;
        if (!s_net.bleAdvertising) {
            s_net.errorCount++;
        }
        break;
    case ESP_GAP_BLE_READ_PHY_COMPLETE_EVT:
        if (param->read_phy.status == ESP_BT_STATUS_SUCCESS) {
            s_net.bleTxPhy = param->read_phy.tx_phy;
            s_net.bleRxPhy = param->read_phy.rx_phy;
        }
        break;
#endif
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
        s_net.bleGattIf = gattsIf;
        if (esp_ble_gap_set_device_name(s_net.deviceName) != ESP_OK) {
            s_net.errorCount++;
        }
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
        if (esp_ble_gap_ext_adv_set_params(0u, &s_extAdvParams) != ESP_OK) {
            s_net.errorCount++;
        }
#else
        if (esp_ble_gap_config_adv_data(&s_advData) != ESP_OK) {
            s_net.errorCount++;
        }
#endif
        if (esp_ble_gatts_create_attr_tab(s_gattDb, gattsIf,
                                          SENSORARRAY_BLE_HANDLE_COUNT,
                                          SENSORARRAY_BLE_APP_ID) != ESP_OK) {
            s_net.errorCount++;
        }
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status == ESP_GATT_OK &&
            param->add_attr_tab.num_handle == SENSORARRAY_BLE_HANDLE_COUNT) {
            memcpy(s_net.bleHandles, param->add_attr_tab.handles, sizeof(s_net.bleHandles));
            (void)esp_ble_gatts_start_service(s_net.bleHandles[BLE_IDX_SERVICE]);
            s_net.bleReady = true;
        } else {
            s_net.errorCount++;
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        s_net.bleConnected = true;
        s_net.bleSubscribed = false;
        s_net.bleConnId = param->connect.conn_id;
        s_net.bleGattIf = gattsIf;
        memcpy(s_net.bleRemote, param->connect.remote_bda, sizeof(esp_bd_addr_t));
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
        {
            esp_ble_gap_phy_mask_t mask = ESP_BLE_GAP_PHY_1M_PREF_MASK;
#if CONFIG_SENSORARRAY_BLE_USE_2M_PHY
            mask = ESP_BLE_GAP_PHY_2M_PREF_MASK;
#elif CONFIG_SENSORARRAY_BLE_USE_CODED_PHY
            mask = ESP_BLE_GAP_PHY_CODED_PREF_MASK;
#endif
            (void)esp_ble_gap_set_preferred_phy(s_net.bleRemote, 0u, mask, mask,
                                                ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
            (void)esp_ble_gap_read_phy(s_net.bleRemote);
        }
#endif
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        s_net.bleConnected = false;
        s_net.bleSubscribed = false;
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
        if (esp_ble_gap_ext_adv_start(1u, s_extAdvSet) != ESP_OK) {
            s_net.errorCount++;
        }
#else
        if (esp_ble_gap_start_advertising(&s_advParams) != ESP_OK) {
            s_net.errorCount++;
        }
#endif
        break;
    case ESP_GATTS_MTU_EVT:
        s_net.bleMtu = param->mtu.mtu;
        break;
    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == s_net.bleHandles[BLE_IDX_NOTIFY_CCCD] &&
            param->write.len == 2u) {
            uint16_t value = (uint16_t)param->write.value[0] |
                             ((uint16_t)param->write.value[1] << 8u);
            s_net.bleSubscribed = value == 1u;
        }
        break;
    default:
        break;
    }
}

#if CONFIG_SENSORARRAY_BLE_ENABLE
static esp_power_level_t sensorarrayBlePowerLevel(void)
{
#if CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= 18
    return ESP_PWR_LVL_P18;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= 15
    return ESP_PWR_LVL_P15;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= 12
    return ESP_PWR_LVL_P12;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= 9
    return ESP_PWR_LVL_P9;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= 6
    return ESP_PWR_LVL_P6;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= 3
    return ESP_PWR_LVL_P3;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= 0
    return ESP_PWR_LVL_N0;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= -3
    return ESP_PWR_LVL_N3;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= -6
    return ESP_PWR_LVL_N6;
#elif CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL >= -9
    return ESP_PWR_LVL_N9;
#else
    return ESP_PWR_LVL_N12;
#endif
}
#endif

static esp_err_t sensorarrayBleInit(void)
{
#if CONFIG_SENSORARRAY_BLE_ENABLE
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    esp_bt_controller_config_t config = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&config);
    if (err == ESP_OK) {
        err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    }
    if (err == ESP_OK) {
        err = esp_bluedroid_init();
    }
    if (err == ESP_OK) {
        err = esp_bluedroid_enable();
    }
    if (err == ESP_OK) {
        err = esp_ble_gap_register_callback(sensorarrayBleGapCallback);
    }
    if (err == ESP_OK) {
        err = esp_ble_gatts_register_callback(sensorarrayBleGattCallback);
    }
    if (err == ESP_OK) {
        err = esp_ble_gatts_app_register(SENSORARRAY_BLE_APP_ID);
    }
    if (err == ESP_OK) {
        (void)esp_ble_gatt_set_local_mtu(247u);
#if CONFIG_SENSORARRAY_BLE_HIGH_TX_POWER
        if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, sensorarrayBlePowerLevel()) != ESP_OK) {
            s_net.errorCount++;
        }
        if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, sensorarrayBlePowerLevel()) != ESP_OK) {
            s_net.errorCount++;
        }
#endif
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
        esp_ble_gap_phy_mask_t mask = ESP_BLE_GAP_PHY_1M_PREF_MASK;
#if CONFIG_SENSORARRAY_BLE_USE_2M_PHY
        mask = ESP_BLE_GAP_PHY_2M_PREF_MASK;
#elif CONFIG_SENSORARRAY_BLE_USE_CODED_PHY
        mask = ESP_BLE_GAP_PHY_CODED_PREF_MASK;
#endif
        (void)esp_ble_gap_set_preferred_default_phy(mask, mask);
#endif
    }
    return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static size_t sensorarrayNetFormatStatus(const sensorarrayNetStatus_t *status)
{
    int length = snprintf(
        s_net.statusText, sizeof(s_net.statusText),
        "SA1,seq=%lu,ts=%llu,phys=%lu.%02lu,fresh=%lu.%02lu,emit=%lu.%02lu,"
        "row=%lu,ready=%lu,read=%lu,coord=%lu,rf=%02X,pf=%02X,sf=%02X,"
        "stale=%u,mixed=%u,nack=%lu,to=%lu,rec=%lu,fb=%lu,se=%lu,direct=%lu.%02lu,"
        "prec=%u,stdA=%lu,stdM=%lu,ain8=%ld,batt=%ld,bv=%u,off=%ld,heap=%lu,min=%lu",
        (unsigned long)status->sequence,
        (unsigned long long)status->timestampUs,
        (unsigned long)(status->physFpsX100 / 100u),
        (unsigned long)(status->physFpsX100 % 100u),
        (unsigned long)(status->cellFreshFpsX100 / 100u),
        (unsigned long)(status->cellFreshFpsX100 % 100u),
        (unsigned long)(status->emitFpsX100 / 100u),
        (unsigned long)(status->emitFpsX100 % 100u),
        (unsigned long)status->rowStepUsAvg,
        (unsigned long)status->readyWaitUsAvg,
        (unsigned long)status->dataReadUsAvg,
        (unsigned long)status->coordinatorResidualUsAvg,
        status->rowFreshMask, status->primaryFreshMask, status->secondaryFreshMask,
        status->stale ? 1u : 0u, status->mixed ? 1u : 0u,
        (unsigned long)status->nack, (unsigned long)status->timeout,
        (unsigned long)status->recover, (unsigned long)status->sequenceFallbacks,
        (unsigned long)status->sequenceErrors,
        (unsigned long)(status->directValidRateX100 / 100u),
        (unsigned long)(status->directValidRateX100 % 100u),
        status->precisionPass ? 1u : 0u,
        (unsigned long)status->pfStdAvgNano, (unsigned long)status->pfStdMaxNano,
        (long)status->adsAin8RawUv, (long)status->batteryMv,
        status->batteryValid ? 1u : 0u, (long)status->adsAin9OffsetUv,
        (unsigned long)status->heapFree, (unsigned long)status->heapMin);
    if (length <= 0) {
        return 0u;
    }
    return (size_t)length < sizeof(s_net.statusText) ?
        (size_t)length : sizeof(s_net.statusText) - 1u;
}

static void sensorarrayNetSendWifi(const char *text, size_t length)
{
#if CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE
    if (!s_net.wifiReady || !s_net.wifiStationConnected || length == 0u) {
        return;
    }
    int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (fd < 0) {
        s_net.udpSendDrop++;
        return;
    }
    int broadcast = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
    struct sockaddr_in target = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_SENSORARRAY_WIFI_STATUS_UDP_PORT),
        .sin_addr.s_addr = inet_addr("192.168.4.255"),
    };
    int sent = sendto(fd, text, length, 0, (struct sockaddr *)&target, sizeof(target));
    close(fd);
    if (sent == (int)length) {
        s_net.udpSendOk++;
    } else {
        s_net.udpSendDrop++;
    }
#else
    (void)text;
    (void)length;
#endif
}

static void sensorarrayNetUpdateBle(const sensorarrayNetStatus_t *status,
                                    const char *text,
                                    size_t textLength)
{
#if CONFIG_SENSORARRAY_BLE_ENABLE
    if (!s_net.bleReady) {
        return;
    }
    uint16_t attrLength = (uint16_t)(textLength < SENSORARRAY_NET_STATUS_TEXT_MAX ?
                                     textLength : SENSORARRAY_NET_STATUS_TEXT_MAX - 1u);
    if (esp_ble_gatts_set_attr_value(s_net.bleHandles[BLE_IDX_STATUS_VALUE],
                                     attrLength, (const uint8_t *)text) != ESP_OK) {
        s_net.errorCount++;
    }

    sensorarrayBleNotifyV1_t notify = {
        .version = 1u,
        .flags = (status->precisionPass ? 1u : 0u) |
                 (status->stale ? 2u : 0u) |
                 (status->mixed ? 4u : 0u) |
                 (status->batteryValid ? 8u : 0u),
        .physFpsX100 = (uint16_t)(status->physFpsX100 > UINT16_MAX ?
                                  UINT16_MAX : status->physFpsX100),
        .sequence = status->sequence,
        .batteryMv = status->batteryMv,
        .ain9OffsetUv = status->adsAin9OffsetUv,
        .netDropCount = s_net.publishDropCount,
    };
    (void)esp_ble_gatts_set_attr_value(s_net.bleHandles[BLE_IDX_NOTIFY_VALUE],
                                       sizeof(notify), (const uint8_t *)&notify);
#if CONFIG_SENSORARRAY_BLE_STATUS_NOTIFY_ENABLE
    if (s_net.bleConnected && s_net.bleSubscribed) {
        esp_err_t err = esp_ble_gatts_send_indicate(s_net.bleGattIf,
                                                    s_net.bleConnId,
                                                    s_net.bleHandles[BLE_IDX_NOTIFY_VALUE],
                                                    sizeof(notify),
                                                    (uint8_t *)&notify,
                                                    false);
        if (err == ESP_OK) {
            s_net.bleNotifyOk++;
        } else {
            s_net.bleNotifyDrop++;
        }
    }
#endif
#else
    (void)status;
    (void)text;
    (void)textLength;
#endif
}

static const char *sensorarrayBlePhyName(uint8_t phy)
{
    switch (phy) {
    case ESP_BLE_GAP_PHY_2M:
        return "2M";
    case ESP_BLE_GAP_PHY_CODED:
        return "coded";
    case ESP_BLE_GAP_PHY_1M:
    default:
        return "1M";
    }
}

static void sensorarrayNetTask(void *arg)
{
    (void)arg;
    printf("TASKCORE,name=net,core=%d,expected=%d\n",
           (int)xPortGetCoreID(), CONFIG_SENSORARRAY_NET_TASK_CORE);
    sensorarrayNetBuildDeviceName();

    esp_err_t nvsErr = nvs_flash_init();
    if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvsErr = nvs_flash_erase();
        if (nvsErr == ESP_OK) {
            nvsErr = nvs_flash_init();
        }
    }
    esp_err_t wifiErr = nvsErr == ESP_OK ? sensorarrayWifiInit() : nvsErr;
    esp_err_t bleErr = nvsErr == ESP_OK ? sensorarrayBleInit() : nvsErr;
    printf("NET_INIT,name=%s,wifi=%u,wifiErr=0x%lx,ssid=%s,udpPort=%u,ble=%u,bleErr=0x%lx\n",
           s_net.deviceName, s_net.wifiReady ? 1u : 0u, (unsigned long)wifiErr,
           s_net.deviceName, (unsigned)CONFIG_SENSORARRAY_WIFI_STATUS_UDP_PORT,
           CONFIG_SENSORARRAY_BLE_ENABLE ? 1u : 0u, (unsigned long)bleErr);

    sensorarrayNetStatus_t status;
    while (true) {
        if (xQueueReceive(s_statusQueue, &status, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        while (xQueueReceive(s_statusQueue, &status, 0) == pdTRUE) {
            s_net.publishDropCount++;
        }
        size_t textLength = sensorarrayNetFormatStatus(&status);
        sensorarrayNetSendWifi(s_net.statusText, textLength);
        sensorarrayNetUpdateBle(&status, s_net.statusText, textLength);
        printf("NET20,enable=1,ap=%u,client=%u,ssid=%s,udpPort=%u,sendOk=%lu,sendDrop=%lu,statusDrop=%lu,err=%lu\n",
               s_net.wifiReady ? 1u : 0u, s_net.wifiStationConnected ? 1u : 0u,
               s_net.deviceName, (unsigned)CONFIG_SENSORARRAY_WIFI_STATUS_UDP_PORT,
               (unsigned long)s_net.udpSendOk, (unsigned long)s_net.udpSendDrop,
               (unsigned long)s_net.publishDropCount, (unsigned long)s_net.errorCount);
        printf("BLE20,adv=%u,conn=%u,sub=%u,mtu=%u,phyReq=%s,phyTx=%s,phyRx=%s,coded=%u,txPower=%d,notifyOk=%lu,notifyDrop=%lu,err=%lu\n",
               s_net.bleAdvertising ? 1u : 0u, s_net.bleConnected ? 1u : 0u,
               s_net.bleSubscribed ? 1u : 0u, (unsigned)s_net.bleMtu,
               CONFIG_SENSORARRAY_BLE_USE_2M_PHY ? "2M" :
                   (CONFIG_SENSORARRAY_BLE_USE_CODED_PHY ? "coded" : "1M"),
               sensorarrayBlePhyName(s_net.bleTxPhy), sensorarrayBlePhyName(s_net.bleRxPhy),
               CONFIG_SENSORARRAY_BLE_USE_CODED_PHY ? 1u : 0u,
               CONFIG_SENSORARRAY_BLE_HIGH_TX_POWER ? CONFIG_SENSORARRAY_BLE_TX_POWER_LEVEL : 0,
               (unsigned long)s_net.bleNotifyOk, (unsigned long)s_net.bleNotifyDrop,
               (unsigned long)s_net.errorCount);
    }
}

esp_err_t sensorarrayNetStatusInit(void)
{
#if !CONFIG_SENSORARRAY_NET_ENABLE
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (s_net.started) {
        return ESP_OK;
    }
    memset(&s_net, 0, sizeof(s_net));
    s_net.bleMtu = 23u;
    s_net.bleTxPhy = ESP_BLE_GAP_PHY_1M;
    s_net.bleRxPhy = ESP_BLE_GAP_PHY_1M;
    s_statusQueue = xQueueCreateStatic(SENSORARRAY_NET_QUEUE_LEN,
                                       sizeof(sensorarrayNetStatus_t),
                                       s_statusQueueStorage,
                                       &s_statusQueueStruct);
    if (!s_statusQueue) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(sensorarrayNetTask, "sensorarrayNet",
                                            CONFIG_SENSORARRAY_NET_TASK_STACK, NULL,
                                            CONFIG_SENSORARRAY_NET_TASK_PRIORITY,
                                            &s_netTask, CONFIG_SENSORARRAY_NET_TASK_CORE);
    if (ok != pdPASS) {
        s_statusQueue = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_net.started = true;
    return ESP_OK;
#endif
}

esp_err_t sensorarrayNetStatusPublish(const sensorarrayNetStatus_t *status)
{
#if !CONFIG_SENSORARRAY_NET_ENABLE
    (void)status;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!status || !s_net.started || !s_statusQueue) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xQueueSend(s_statusQueue, status, 0) == pdTRUE) {
        return ESP_OK;
    }
    sensorarrayNetStatus_t discarded;
    (void)xQueueReceive(s_statusQueue, &discarded, 0);
    s_net.publishDropCount++;
    return xQueueSend(s_statusQueue, status, 0) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
#endif
}
