#include "sensorarrayTransportInternal.h"

#include <stdio.h>
#include <string.h>

#include "esp_mac.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sensorarrayBle.h"
#include "sensorarrayScanConfig.h"
#include "sensorarrayWifi.h"

#ifndef CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT
#define CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT 0
#endif
#ifndef CONFIG_SENSORARRAY_OUTPUT_BLE_CAP_TEXT
#define CONFIG_SENSORARRAY_OUTPUT_BLE_CAP_TEXT 1
#endif
#ifndef CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE
#define CONFIG_SENSORARRAY_WIFI_SOFTAP_ENABLE 1
#endif
#ifndef CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD
#define CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD ""
#endif

#define SENSORARRAY_TRANSPORT_TEXT_MAX 1536u
#define SENSORARRAY_TRANSPORT_QUEUE_COUNT 3u
#define SENSORARRAY_CFG_OUTPUT_WIFI_TEXT (CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT != 0)
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT (CONFIG_SENSORARRAY_OUTPUT_BLE_CAP_TEXT != 0)

typedef struct {
    sensorarrayTransportChannel_t channel;
    uint16_t length;
    char data[SENSORARRAY_TRANSPORT_TEXT_MAX];
} sensorarrayTransportItem_t;

static StaticQueue_t s_queueStruct;
static uint8_t s_queueStorage[SENSORARRAY_TRANSPORT_QUEUE_COUNT *
                              sizeof(sensorarrayTransportItem_t)];
static QueueHandle_t s_queue;
static TaskHandle_t s_task;
static bool s_started;
static char s_deviceName[32];
static char s_mdnsName[32];

static void sensorarrayTransportBuildNames(const uint8_t mac[6])
{
    snprintf(s_deviceName, sizeof(s_deviceName), "CscArray_%02X%02X%02X",
             mac[3], mac[4], mac[5]);
    snprintf(s_mdnsName, sizeof(s_mdnsName), "cscarray-%02x%02x%02x.local",
             mac[3], mac[4], mac[5]);
}

static void sensorarrayTransportBuildWifiConfig(sensorarrayWifiConfig_t *wifiConfig)
{
    if (!wifiConfig) {
        return;
    }
    *wifiConfig = (sensorarrayWifiConfig_t){
        .profile = SENSORARRAY_WIFI_PROFILE_HIGH,
        .dataPort = 3333u,
        .logPort = 3334u,
        .ctrlPort = 3335u,
    };
    snprintf(wifiConfig->ssid, sizeof(wifiConfig->ssid), "%s", s_deviceName);
    snprintf(wifiConfig->password, sizeof(wifiConfig->password), "%s",
             CONFIG_SENSORARRAY_WIFI_SOFTAP_PASSWORD);
}

static esp_err_t sensorarrayTransportStartWifiAp(void)
{
    sensorarrayWifiConfig_t wifiConfig;
    sensorarrayTransportBuildWifiConfig(&wifiConfig);
    esp_err_t err = sensorarrayWifiInit(&wifiConfig);
    if (err != ESP_OK) {
        wifiConfig.profile = SENSORARRAY_WIFI_PROFILE_BLE_COMPAT;
        err = sensorarrayWifiInit(&wifiConfig);
    }
    return err;
}

static void sensorarrayTransportStatResult(sensorarrayTransportChannel_t channel,
                                           bool ble,
                                           esp_err_t err)
{
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    uint32_t *sent = NULL;
    uint32_t *drop = NULL;
    if (ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA) {
        sent = &g_sensorarrayTransportStats.bleDataSent;
        drop = &g_sensorarrayTransportStats.bleDataDrop;
    } else if (ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
        sent = &g_sensorarrayTransportStats.bleLogSent;
        drop = &g_sensorarrayTransportStats.bleLogDrop;
    } else if (!ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA) {
        sent = &g_sensorarrayTransportStats.wifiDataSent;
        drop = &g_sensorarrayTransportStats.wifiDataDrop;
    } else if (!ble && channel == SENSORARRAY_TRANSPORT_CHANNEL_LOG) {
        sent = &g_sensorarrayTransportStats.wifiLogSent;
        drop = &g_sensorarrayTransportStats.wifiLogDrop;
    }
    if (sent && drop) {
        if (err == ESP_OK) {
            (*sent)++;
        } else {
            (*drop)++;
        }
    }
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
}

static void sensorarrayTransportBleControl(const uint8_t *data, size_t length, void *context)
{
    (void)context;
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    g_sensorarrayTransportStats.bleCtrlRx++;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    sensorarrayTransportReplyTarget_t target = {.kind = SENSORARRAY_TRANSPORT_REPLY_BLE};
    (void)sensorarrayTransportHandleControlCommand(data, length, &target);
}

static void sensorarrayTransportWifiControl(const uint8_t *data,
                                            size_t length,
                                            const sensorarrayWifiPeer_t *peer,
                                            void *context)
{
    (void)context;
    portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
    g_sensorarrayTransportStats.wifiCtrlRx++;
    portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
    sensorarrayTransportReplyTarget_t target = {
        .kind = SENSORARRAY_TRANSPORT_REPLY_WIFI,
        .wifiPeer = *peer,
    };
    (void)sensorarrayTransportHandleControlCommand(data, length, &target);
}

static void sensorarrayTransportSerialControlTask(void *arg)
{
    (void)arg;
    char line[128];
    sensorarrayTransportReplyTarget_t target = {.kind = SENSORARRAY_TRANSPORT_REPLY_SERIAL};
    for (;;) {
        if (fgets(line, sizeof(line), stdin)) {
            (void)sensorarrayTransportHandleControlCommand((const uint8_t *)line,
                                                           strlen(line), &target);
        } else {
            clearerr(stdin);
            vTaskDelay(pdMS_TO_TICKS(10u));
        }
    }
}

static void sensorarrayTransportPrintNetInit(esp_err_t wifiErr)
{
    sensorarrayBleStats_t ble = {0};
    sensorarrayWifiStats_t wifi = {0};
    sensorarrayBleGetStats(&ble);
    sensorarrayWifiGetStats(&wifi);
    bool bleOk = ble.controllerReady && ble.hostReady && ble.gattReady && ble.advertising;
    printf("NET,ble=%s,ap=%s,mdns=%s,wifi=%s,wok=%u,werr=0x%lx,berr=0x%lx,bok=%u\n",
           s_deviceName,
           s_deviceName,
           s_mdnsName,
           sensorarrayTransportWifiModeName(sensorarrayTransportGetWifiMode()),
           wifi.ready ? 1u : 0u,
           (unsigned long)wifiErr,
           (unsigned long)ble.initError,
           bleOk ? 1u : 0u);
}

static void sensorarrayTransportTask(void *arg)
{
    esp_err_t wifiErr = (esp_err_t)(intptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(750u));
    sensorarrayTransportPrintNetInit(wifiErr);
    sensorarrayTransportItem_t item;
    while (xQueueReceive(s_queue, &item, portMAX_DELAY) == pdTRUE) {
        sensorarrayBleChannel_t bleChannel = item.channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA ?
            SENSORARRAY_BLE_CH_DATA : SENSORARRAY_BLE_CH_LOG;
        sensorarrayWifiChannel_t wifiChannel = item.channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA ?
            SENSORARRAY_WIFI_CH_DATA : SENSORARRAY_WIFI_CH_LOG;
        if (SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT && sensorarrayTransportBleSinkEnabled()) {
            esp_err_t bleErr = sensorarrayBleNotify(bleChannel,
                                                    (const uint8_t *)item.data,
                                                    item.length);
            sensorarrayTransportStatResult(item.channel, true, bleErr);
        }
        if (sensorarrayTransportWifiSinkEnabled() && sensorarrayWifiIsReady()) {
            esp_err_t sendWifiErr = sensorarrayWifiSend(wifiChannel,
                                                        (const uint8_t *)item.data,
                                                        item.length);
            sensorarrayTransportStatResult(item.channel, false, sendWifiErr);
        }
        if (sensorarrayBleIsCongested()) {
            portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
            g_sensorarrayTransportStats.bleCongested++;
            portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        }
    }
}

static esp_err_t sensorarrayTransportQueue(sensorarrayTransportChannel_t channel,
                                           const char *data,
                                           size_t length)
{
    if (!data || length == 0u || length > SENSORARRAY_TRANSPORT_TEXT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_queue) {
        return ESP_ERR_INVALID_STATE;
    }
    if ((!SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT || !sensorarrayTransportBleSinkEnabled()) &&
        !sensorarrayTransportWifiSinkEnabled()) {
        return ESP_OK;
    }
    sensorarrayTransportItem_t item = {.channel = channel, .length = (uint16_t)length};
    memcpy(item.data, data, length);
    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        g_sensorarrayTransportStats.queueDrop++;
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t sensorarrayTransportInit(void)
{
    if (s_started) {
        return ESP_OK;
    }
    (void)sensorarrayScanConfigInit();
    uint8_t mac[6] = {0};
    (void)esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    sensorarrayTransportBuildNames(mac);
    sensorarrayTransportSetStream(SENSORARRAY_TRANSPORT_STREAM_SER);
    sensorarrayTransportSetTxMode(SENSORARRAY_TRANSPORT_TX_REL);
    sensorarrayTransportSetWifiMode(SENSORARRAY_CFG_OUTPUT_WIFI_TEXT ?
                                    SENSORARRAY_TRANSPORT_WIFI_AP :
                                    SENSORARRAY_TRANSPORT_WIFI_OFF);

    esp_err_t nvsErr = nvs_flash_init();
    if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvsErr = nvs_flash_erase();
        if (nvsErr == ESP_OK) {
            nvsErr = nvs_flash_init();
        }
    }
    sensorarrayBleSetControlRxCallback(sensorarrayTransportBleControl, NULL);
    sensorarrayBleConfig_t bleConfig = {
        .enableBle5 = true,
        .preferPhy2m = true,
        .preferHighTxPower = true,
        .preferredMtu = 247u,
    };
    snprintf(bleConfig.deviceName, sizeof(bleConfig.deviceName), "%s", s_deviceName);
    esp_err_t bleErr = nvsErr == ESP_OK && SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT ?
        sensorarrayBleInit(&bleConfig) : nvsErr;

    sensorarrayBleLogHeap("before_wifi_init");
    sensorarrayWifiSetControlRxCallback(sensorarrayTransportWifiControl, NULL);
    esp_err_t wifiErr = nvsErr == ESP_OK && SENSORARRAY_CFG_OUTPUT_WIFI_TEXT ?
        sensorarrayTransportStartWifiAp() : ESP_OK;
    sensorarrayBleLogHeap("after_wifi_init");

    s_queue = xQueueCreateStatic(SENSORARRAY_TRANSPORT_QUEUE_COUNT,
                                 sizeof(sensorarrayTransportItem_t),
                                 s_queueStorage, &s_queueStruct);
    if (!s_queue) {
        return ESP_ERR_NO_MEM;
    }
    BaseType_t ok = xTaskCreate(sensorarrayTransportTask, "transport", 6144u,
                                (void *)(intptr_t)wifiErr, 5u, &s_task);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ok = xTaskCreate(sensorarrayTransportSerialControlTask, "serialCtrl", 3072u,
                     NULL, 4u, NULL);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    s_started = true;
    return bleErr == ESP_OK || wifiErr == ESP_OK ? ESP_OK : bleErr;
}

esp_err_t sensorarrayTransportApplyWifiMode(sensorarrayTransportWifiMode_t mode)
{
    if (mode > SENSORARRAY_TRANSPORT_WIFI_APSTA) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayTransportSetWifiMode(mode);
    if (mode == SENSORARRAY_TRANSPORT_WIFI_OFF) {
        return ESP_OK;
    }
    if (mode == SENSORARRAY_TRANSPORT_WIFI_AP) {
        return sensorarrayTransportStartWifiAp();
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sensorarrayTransportPublishData(const char *data, size_t length)
{
    return sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_DATA, data, length);
}

esp_err_t sensorarrayTransportPublishLog(const char *data, size_t length)
{
    return sensorarrayTransportQueue(SENSORARRAY_TRANSPORT_CHANNEL_LOG, data, length);
}

esp_err_t sensorarrayTransportPublishControlReply(
    const sensorarrayTransportReplyTarget_t *target,
    const char *data,
    size_t length)
{
    if (!target || !data || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err;
    if (target->kind == SENSORARRAY_TRANSPORT_REPLY_BLE) {
        err = sensorarrayBleNotify(SENSORARRAY_BLE_CH_CTRL, (const uint8_t *)data, length);
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        if (err == ESP_OK) {
            g_sensorarrayTransportStats.bleCtrlTx++;
        }
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return err;
    }
    if (target->kind == SENSORARRAY_TRANSPORT_REPLY_WIFI) {
        err = sensorarrayWifiSendControlReply(&target->wifiPeer,
                                              (const uint8_t *)data, length);
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        if (err == ESP_OK) {
            g_sensorarrayTransportStats.wifiCtrlTx++;
        }
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        return err;
    }
    size_t written = fwrite(data, 1u, length, stdout);
    fflush(stdout);
    return written == length ? ESP_OK : ESP_FAIL;
}
