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
#include "sensorarrayCommandMailbox.h"

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
#ifndef CONFIG_SENSORARRAY_BLE_CAP_TEXT_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_BLE_CAP_TEXT_EVERY_N_FRAMES 0
#endif

#define SENSORARRAY_NET_QUEUE_LEN 4u
#define SENSORARRAY_NET_STATUS_TEXT_MAX 512u
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

typedef struct {
    sensorarrayTextPacket_t packet;
    bool allowBle;
} sensorarrayNetTextItem_t;

typedef struct {
    bool started;
    bool wifiReady;
    bool wifiStationConnected;
    bool bleReady;
    bool bleAdvertising;
    bool bleConnected;
    bool bleSubscribed;
    bool bleCongested;
    uint16_t bleConnId;
    uint16_t bleMtu;
    uint16_t bleHandles[SENSORARRAY_BLE_HANDLE_COUNT];
    esp_gatt_if_t bleGattIf;
    esp_bd_addr_t bleRemote;
    uint8_t bleTxPhy;
    uint8_t bleRxPhy;
    int udpSocket;
    uint32_t publishDropCount;
    uint32_t udpSendOk;
    uint32_t udpSendDrop;
    uint32_t bleNotifyOk;
    uint32_t bleNotifyDrop;
    uint32_t errorCount;
    char deviceName[32];
    sensorarrayNetSinkStats_t sinkStats;
} sensorarrayNetState_t;

static sensorarrayNetState_t s_net;
static StaticQueue_t s_statusQueueStruct;
static uint8_t s_statusQueueStorage[SENSORARRAY_NET_QUEUE_LEN * sizeof(sensorarrayNetTextItem_t)];
static QueueHandle_t s_statusQueue;
static TaskHandle_t s_netTask;

static uint16_t s_primaryServiceUuid = ESP_GATT_UUID_PRI_SERVICE;
static uint16_t s_characterDeclarationUuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint16_t s_cccdUuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static uint16_t s_serviceUuid = SENSORARRAY_BLE_SERVICE_UUID;
static uint16_t s_statusUuid = SENSORARRAY_BLE_STATUS_UUID;
static uint16_t s_notifyUuid = SENSORARRAY_BLE_NOTIFY_UUID;
static uint8_t s_statusProperties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
static uint8_t s_notifyProperties = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t s_initialStatus[] = "SensorArray status pending";
static uint8_t s_initialNotify[] = "SensorArray text notify pending\n";
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
        {ESP_UUID_LEN_16, (uint8_t *)&s_statusUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
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
         SENSORARRAY_NET_STATUS_TEXT_MAX, sizeof(s_initialNotify) - 1u, s_initialNotify},
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

#if CONFIG_SENSORARRAY_BLE_ENABLE
static esp_power_level_t sensorarrayBlePowerLevel(void);
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
        esp_err_t powerErr = esp_wifi_set_max_tx_power(CONFIG_SENSORARRAY_WIFI_TX_POWER_DBM_X4);
        if (powerErr != ESP_OK) {
            s_net.errorCount++;
            printf("W,WIFI,txp=fail,err=0x%lx,fb=default\n", (unsigned long)powerErr);
        } else {
            printf("W,WIFI,txp=%d\n", CONFIG_SENSORARRAY_WIFI_TX_POWER_DBM_X4);
        }

        esp_err_t psErr = esp_wifi_set_ps(WIFI_PS_NONE);
        if (psErr != ESP_OK) {
            s_net.errorCount++;
            printf("W,WIFI,ps=none_fail,err=0x%lx,fb=default\n", (unsigned long)psErr);
        } else {
            printf("W,WIFI,ps=none\n");
        }

        esp_err_t protocolErr = esp_wifi_set_protocol(
            WIFI_IF_AP,
            WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
        if (protocolErr != ESP_OK) {
            s_net.errorCount++;
            printf("W,WIFI,rate=11N_FAIL,err=0x%lx,fb=default\n",
                   (unsigned long)protocolErr);
        }
        esp_err_t bandwidthErr = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40);
        if (bandwidthErr != ESP_OK) {
            printf("W,WIFI,rate=HT40_FAIL,err=0x%lx,fb=HT20\n",
                   (unsigned long)bandwidthErr);
            bandwidthErr = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
        } else {
            printf("W,WIFI,rate=HT40\n");
        }
        if (bandwidthErr != ESP_OK) {
            s_net.errorCount++;
            printf("W,WIFI,rate=HT20_FAIL,err=0x%lx,fb=wifi_disabled\n",
                   (unsigned long)bandwidthErr);
            return bandwidthErr;
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
        s_net.bleCongested = false;
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
        {
            esp_ble_conn_update_params_t connParams = {0};
            memcpy(connParams.bda, s_net.bleRemote, sizeof(esp_bd_addr_t));
            connParams.min_int = 0x06;
            connParams.max_int = 0x0C;
            connParams.latency = 0u;
            connParams.timeout = 400u;
            esp_err_t connErr = esp_ble_gap_update_conn_params(&connParams);
            if (connErr != ESP_OK) {
                printf("W,BLE,conn=fast_fail,err=0x%lx,fb=peer\n",
                       (unsigned long)connErr);
            }
        }
#if CONFIG_SENSORARRAY_BLE_HIGH_TX_POWER
        if (s_net.bleConnId <= 8u) {
            esp_ble_power_type_t powerType =
                (esp_ble_power_type_t)(ESP_BLE_PWR_TYPE_CONN_HDL0 + s_net.bleConnId);
            esp_err_t powerErr = esp_ble_tx_power_set(powerType, sensorarrayBlePowerLevel());
            if (powerErr != ESP_OK) {
                printf("W,BLE,txp=conn_fail,err=0x%lx,fb=default\n",
                       (unsigned long)powerErr);
            }
        }
#endif
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        s_net.bleConnected = false;
        s_net.bleSubscribed = false;
        s_net.bleCongested = false;
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
        } else if (param->write.handle == s_net.bleHandles[BLE_IDX_STATUS_VALUE]) {
            esp_err_t commandErr = sensorarrayCommandMailboxPostText(param->write.value,
                                                                      param->write.len);
            printf("E,type=cmd,state=%s,err=0x%lx\n",
                   commandErr == ESP_OK ? "queued" : "rejected",
                   (unsigned long)commandErr);
        }
        break;
    case ESP_GATTS_CONGEST_EVT:
        s_net.bleCongested = param->congest.congested;
        if (s_net.bleCongested) {
            s_net.bleNotifyDrop++;
            s_net.sinkStats.bleBlockedCount++;
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
        esp_power_level_t powerLevel = sensorarrayBlePowerLevel();
        esp_err_t defaultPowerErr = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, powerLevel);
        esp_err_t advPowerErr = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, powerLevel);
        esp_err_t scanPowerErr = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, powerLevel);
        if (defaultPowerErr != ESP_OK || advPowerErr != ESP_OK || scanPowerErr != ESP_OK) {
            s_net.errorCount++;
            printf("W,BLE,txp=max_fail,err=%lx/%lx/%lx,fb=controller_default\n",
                   (unsigned long)defaultPowerErr,
                   (unsigned long)advPowerErr,
                   (unsigned long)scanPowerErr);
        } else {
            printf("BLEBOOT,txp=max,mtu=247,phyReq=%s\n",
                   CONFIG_SENSORARRAY_BLE_USE_2M_PHY ? "2M" : "1M");
        }
#endif
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
        esp_ble_gap_phy_mask_t mask = ESP_BLE_GAP_PHY_1M_PREF_MASK;
#if CONFIG_SENSORARRAY_BLE_USE_2M_PHY
        mask = ESP_BLE_GAP_PHY_2M_PREF_MASK;
#elif CONFIG_SENSORARRAY_BLE_USE_CODED_PHY
        mask = ESP_BLE_GAP_PHY_CODED_PREF_MASK;
#endif
        esp_err_t phyErr = esp_ble_gap_set_preferred_default_phy(mask, mask);
        if (phyErr != ESP_OK && mask != ESP_BLE_GAP_PHY_1M_PREF_MASK) {
            printf("W,BLE,phy=2M_FAIL,err=0x%lx,fb=1M\n", (unsigned long)phyErr);
            (void)esp_ble_gap_set_preferred_default_phy(ESP_BLE_GAP_PHY_1M_PREF_MASK,
                                                        ESP_BLE_GAP_PHY_1M_PREF_MASK);
        }
#endif
    }
    return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void sensorarrayNetSendWifi(const sensorarrayTextPacket_t *packet)
{
#if CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE
    if (!packet || packet->length == 0u || !s_net.wifiReady || !s_net.wifiStationConnected) {
        return;
    }
    if (s_net.udpSocket < 0) {
        s_net.udpSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (s_net.udpSocket >= 0) {
            int broadcast = 1;
            (void)setsockopt(s_net.udpSocket, SOL_SOCKET, SO_BROADCAST,
                             &broadcast, sizeof(broadcast));
        }
    }
    if (s_net.udpSocket < 0) {
        s_net.udpSendDrop++;
        s_net.sinkStats.wifiDroppedPackets++;
        return;
    }

    struct sockaddr_in target = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_SENSORARRAY_WIFI_STATUS_UDP_PORT),
        .sin_addr.s_addr = inet_addr("192.168.4.255"),
    };
    int sent = sendto(s_net.udpSocket,
                      packet->data,
                      packet->length,
                      MSG_DONTWAIT,
                      (struct sockaddr *)&target,
                      sizeof(target));
    if (sent == (int)packet->length) {
        s_net.udpSendOk++;
        s_net.sinkStats.wifiSentPackets++;
        s_net.sinkStats.wifiSentBytes += (uint32_t)sent;
    } else {
        s_net.udpSendDrop++;
        s_net.sinkStats.wifiDroppedPackets++;
        s_net.sinkStats.wifiBlockedCount++;
    }
#else
    (void)packet;
#endif
}

static size_t sensorarrayBleChunkLength(const char *data, size_t remaining, size_t maximum)
{
    size_t length = remaining < maximum ? remaining : maximum;
    for (size_t index = length; index > 0u; --index) {
        if (data[index - 1u] == '\n') {
            return index;
        }
    }
    return length;
}

static void sensorarrayNetSendBle(const sensorarrayTextPacket_t *packet, bool allowBle)
{
#if CONFIG_SENSORARRAY_BLE_ENABLE && CONFIG_SENSORARRAY_BLE_STATUS_NOTIFY_ENABLE
    if (!packet || !allowBle || packet->length == 0u || !s_net.bleReady ||
        !s_net.bleConnected || !s_net.bleSubscribed) {
        return;
    }
    if (s_net.bleCongested) {
        s_net.bleNotifyDrop++;
        s_net.sinkStats.bleDroppedPackets++;
        s_net.sinkStats.bleBlockedCount++;
        return;
    }

    size_t maximum = s_net.bleMtu > 3u ? (size_t)(s_net.bleMtu - 3u) : 20u;
    if (maximum > SENSORARRAY_NET_STATUS_TEXT_MAX) {
        maximum = SENSORARRAY_NET_STATUS_TEXT_MAX;
    }
    size_t offset = 0u;
    bool complete = true;
    while (offset < packet->length) {
        size_t chunkLength = sensorarrayBleChunkLength(&packet->data[offset],
                                                       packet->length - offset,
                                                       maximum);
        esp_err_t attrErr = esp_ble_gatts_set_attr_value(
            s_net.bleHandles[BLE_IDX_NOTIFY_VALUE],
            (uint16_t)chunkLength,
            (const uint8_t *)&packet->data[offset]);
        esp_err_t notifyErr = attrErr == ESP_OK ?
            esp_ble_gatts_send_indicate(s_net.bleGattIf,
                                        s_net.bleConnId,
                                        s_net.bleHandles[BLE_IDX_NOTIFY_VALUE],
                                        (uint16_t)chunkLength,
                                        (uint8_t *)&packet->data[offset],
                                        false) : attrErr;
        if (notifyErr != ESP_OK) {
            complete = false;
            s_net.sinkStats.bleBlockedCount++;
            break;
        }
        s_net.sinkStats.bleSentBytes += chunkLength;
        offset += chunkLength;
    }
    if (complete) {
        s_net.bleNotifyOk++;
        s_net.sinkStats.bleSentPackets++;
    } else {
        s_net.bleNotifyDrop++;
        s_net.sinkStats.bleDroppedPackets++;
    }
#else
    (void)packet;
    (void)allowBle;
#endif
}

static void sensorarrayNetTask(void *arg)
{
    (void)arg;
    printf("TASKCORE,name=net_sink,core=%d,expected=%d\n",
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
    printf("NET_INIT,name=%s,wifi=%u,wifiErr=0x%lx,ssid=%s,udpPort=%u,ble=%u,bleErr=0x%lx,protocol=ascii\n",
           s_net.deviceName,
           s_net.wifiReady ? 1u : 0u,
           (unsigned long)wifiErr,
           s_net.deviceName,
           (unsigned)CONFIG_SENSORARRAY_WIFI_STATUS_UDP_PORT,
           CONFIG_SENSORARRAY_BLE_ENABLE ? 1u : 0u,
           (unsigned long)bleErr);

    sensorarrayNetTextItem_t item;
    while (true) {
        if (xQueueReceive(s_statusQueue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        while (xQueueReceive(s_statusQueue, &item, 0) == pdTRUE) {
            s_net.publishDropCount++;
        }
        s_net.sinkStats.queueDepth = uxQueueMessagesWaiting(s_statusQueue);
        sensorarrayNetSendWifi(&item.packet);
        sensorarrayNetSendBle(&item.packet, item.allowBle);
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
    s_net.udpSocket = -1;
    s_net.bleMtu = 23u;
    s_net.bleTxPhy = ESP_BLE_GAP_PHY_1M;
    s_net.bleRxPhy = ESP_BLE_GAP_PHY_1M;
    s_statusQueue = xQueueCreateStatic(SENSORARRAY_NET_QUEUE_LEN,
                                       sizeof(sensorarrayNetTextItem_t),
                                       s_statusQueueStorage,
                                       &s_statusQueueStruct);
    if (!s_statusQueue) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t ok = xTaskCreatePinnedToCore(sensorarrayNetTask,
                                            "sensorarrayNetSink",
                                            CONFIG_SENSORARRAY_NET_TASK_STACK,
                                            NULL,
                                            CONFIG_SENSORARRAY_NET_TASK_PRIORITY,
                                            &s_netTask,
                                            CONFIG_SENSORARRAY_NET_TASK_CORE);
    if (ok != pdPASS) {
        s_statusQueue = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_net.started = true;
    return ESP_OK;
#endif
}

esp_err_t sensorarrayNetTextPublish(const sensorarrayTextPacket_t *packet,
                                    bool allowBle)
{
#if !CONFIG_SENSORARRAY_NET_ENABLE
    (void)packet;
    (void)allowBle;
    return ESP_ERR_NOT_SUPPORTED;
#else
    if (!packet || packet->length == 0u || packet->length > SENSORARRAY_TEXT_PACKET_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_net.started || !s_statusQueue) {
        return ESP_ERR_INVALID_STATE;
    }
    sensorarrayNetTextItem_t item = {
        .packet = *packet,
        .allowBle = allowBle,
    };
    if (xQueueSend(s_statusQueue, &item, 0) != pdTRUE) {
        sensorarrayNetTextItem_t discarded;
        (void)xQueueReceive(s_statusQueue, &discarded, 0);
        s_net.publishDropCount++;
        s_net.sinkStats.wifiDroppedPackets++;
        if (allowBle) {
            s_net.sinkStats.bleDroppedPackets++;
        }
        if (xQueueSend(s_statusQueue, &item, 0) != pdTRUE) {
            return ESP_ERR_TIMEOUT;
        }
    }
    uint32_t depth = uxQueueMessagesWaiting(s_statusQueue);
    s_net.sinkStats.queueDepth = depth;
    if (depth > s_net.sinkStats.queueDepthMax) {
        s_net.sinkStats.queueDepthMax = depth;
    }
    return ESP_OK;
#endif
}

void sensorarrayNetGetSinkStats(sensorarrayNetSinkStats_t *outStats)
{
    if (!outStats) {
        return;
    }
    *outStats = s_net.sinkStats;
}
