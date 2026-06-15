#include "sensorarrayWifi.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#define SENSORARRAY_WIFI_CTRL_BUFFER 256u

typedef struct {
    sensorarrayWifiConfig_t config;
    sensorarrayWifiStats_t stats;
    int txSocket;
    int ctrlSocket;
    TaskHandle_t ctrlTask;
    sensorarrayWifiControlRxCallback_t controlCallback;
    void *controlContext;
    esp_netif_t *apNetif;
    bool driverInitialized;
    bool eventHandlerRegistered;
    bool initialized;
} sensorarrayWifiState_t;

static sensorarrayWifiState_t s_wifi = {.txSocket = -1, .ctrlSocket = -1};

static const char *sensorarrayWifiProfileName(sensorarrayWifiProfile_t profile)
{
    return profile == SENSORARRAY_WIFI_PROFILE_HIGH ? "HIGH" : "BLE_COMPAT";
}

static uint16_t sensorarrayWifiChannelPort(sensorarrayWifiChannel_t channel)
{
    if (channel == SENSORARRAY_WIFI_CH_DATA) {
        return s_wifi.config.dataPort;
    }
    if (channel == SENSORARRAY_WIFI_CH_LOG) {
        return s_wifi.config.logPort;
    }
    return s_wifi.config.ctrlPort;
}

static void sensorarrayWifiEvent(void *arg,
                                 esp_event_base_t base,
                                 int32_t eventId,
                                 void *eventData)
{
    (void)arg;
    (void)base;
    (void)eventData;
    if (eventId == WIFI_EVENT_AP_STACONNECTED) {
        s_wifi.stats.stationConnected = true;
    } else if (eventId == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_sta_list_t stations = {0};
        s_wifi.stats.stationConnected =
            esp_wifi_ap_get_sta_list(&stations) == ESP_OK && stations.num != 0u;
    }
}

static esp_err_t sensorarrayWifiApplyProfile(sensorarrayWifiProfile_t profile)
{
    esp_err_t err = esp_wifi_set_max_tx_power(80);
    if (err == ESP_OK) {
        err = esp_wifi_set_ps(WIFI_PS_NONE);
    }
    if (err == ESP_OK) {
        err = esp_wifi_set_protocol(WIFI_IF_AP,
                                    WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                    WIFI_PROTOCOL_11N);
    }
    if (err == ESP_OK) {
        err = esp_wifi_set_bandwidth(WIFI_IF_AP,
                                     profile == SENSORARRAY_WIFI_PROFILE_HIGH ?
                                     WIFI_BW_HT40 : WIFI_BW_HT20);
    }
    return err;
}

static void sensorarrayWifiCleanupAttempt(void)
{
    if (s_wifi.ctrlTask) {
        vTaskDelete(s_wifi.ctrlTask);
        s_wifi.ctrlTask = NULL;
    }
    if (s_wifi.ctrlSocket >= 0) {
        close(s_wifi.ctrlSocket);
        s_wifi.ctrlSocket = -1;
    }
    if (s_wifi.txSocket >= 0) {
        close(s_wifi.txSocket);
        s_wifi.txSocket = -1;
    }
    if (s_wifi.eventHandlerRegistered) {
        (void)esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                           sensorarrayWifiEvent);
        s_wifi.eventHandlerRegistered = false;
    }
    if (s_wifi.driverInitialized) {
        (void)esp_wifi_stop();
        (void)esp_wifi_deinit();
        s_wifi.driverInitialized = false;
    }
    if (s_wifi.apNetif) {
        esp_netif_destroy_default_wifi(s_wifi.apNetif);
        s_wifi.apNetif = NULL;
    }
    s_wifi.stats.ready = false;
    s_wifi.stats.stationConnected = false;
}

static void sensorarrayWifiConfigureInit(wifi_init_config_t *init,
                                         sensorarrayWifiProfile_t profile)
{
    if (profile == SENSORARRAY_WIFI_PROFILE_HIGH) {
        init->static_rx_buf_num = 8;
        init->dynamic_rx_buf_num = 16;
        init->dynamic_tx_buf_num = 16;
        init->rx_ba_win = 6;
        init->mgmt_sbuf_num = 12;
    } else {
        init->static_rx_buf_num = 6;
        init->dynamic_rx_buf_num = 8;
        init->dynamic_tx_buf_num = 8;
        init->rx_ba_win = 4;
        init->mgmt_sbuf_num = 8;
    }
}

static esp_err_t sensorarrayWifiOpenSockets(void)
{
    s_wifi.txSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_wifi.txSocket < 0) {
        return ESP_FAIL;
    }
    int enabled = 1;
    (void)setsockopt(s_wifi.txSocket, SOL_SOCKET, SO_BROADCAST, &enabled, sizeof(enabled));

    s_wifi.ctrlSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_wifi.ctrlSocket < 0) {
        return ESP_FAIL;
    }
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_port = htons(s_wifi.config.ctrlPort),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(s_wifi.ctrlSocket, (struct sockaddr *)&address, sizeof(address)) != 0) {
        return ESP_FAIL;
    }
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 250000};
    (void)setsockopt(s_wifi.ctrlSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    return ESP_OK;
}

static void sensorarrayWifiControlTask(void *arg)
{
    (void)arg;
    uint8_t buffer[SENSORARRAY_WIFI_CTRL_BUFFER];
    while (true) {
        struct sockaddr_in source = {0};
        socklen_t sourceLength = sizeof(source);
        int received = recvfrom(s_wifi.ctrlSocket, buffer, sizeof(buffer), 0,
                                (struct sockaddr *)&source, &sourceLength);
        if (received <= 0) {
            continue;
        }
        s_wifi.stats.ctrlRx++;
        if (s_wifi.controlCallback) {
            sensorarrayWifiPeer_t peer = {
                .ipv4 = source.sin_addr.s_addr,
                .port = ntohs(source.sin_port),
            };
            s_wifi.controlCallback(buffer, (size_t)received, &peer, s_wifi.controlContext);
        }
    }
}

esp_err_t sensorarrayWifiInit(const sensorarrayWifiConfig_t *config)
{
    if (!config || config->ssid[0] == '\0' || config->dataPort == 0u ||
        config->logPort == 0u || config->ctrlPort == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_wifi.initialized) {
        esp_err_t err = sensorarrayWifiApplyProfile(config->profile);
        if (err == ESP_OK) {
            s_wifi.config.profile = config->profile;
            s_wifi.stats.profile = config->profile;
        }
        return err;
    }

    s_wifi.config = *config;
    s_wifi.stats.profile = config->profile;
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        goto done;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        goto done;
    }
    s_wifi.apNetif = esp_netif_create_default_wifi_ap();
    if (!s_wifi.apNetif) {
        err = ESP_ERR_NO_MEM;
        goto done;
    }
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    sensorarrayWifiConfigureInit(&init, config->profile);
    err = esp_wifi_init(&init);
    if (err != ESP_OK) {
        goto done;
    }
    s_wifi.driverInitialized = true;
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, sensorarrayWifiEvent, NULL);
    if (err != ESP_OK) {
        goto done;
    }
    s_wifi.eventHandlerRegistered = true;

    wifi_config_t ap = {0};
    size_t ssidLength = strnlen(config->ssid, sizeof(ap.ap.ssid));
    memcpy(ap.ap.ssid, config->ssid, ssidLength);
    ap.ap.ssid_len = (uint8_t)ssidLength;
    size_t passwordLength = strnlen(config->password, sizeof(ap.ap.password));
    memcpy(ap.ap.password, config->password, passwordLength);
    ap.ap.channel = 1u;
    ap.ap.max_connection =
        config->profile == SENSORARRAY_WIFI_PROFILE_HIGH ? 4u : 1u;
    ap.ap.authmode = passwordLength >= 8u ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err == ESP_OK) {
        err = esp_wifi_set_config(WIFI_IF_AP, &ap);
    }
    if (err == ESP_OK) {
        err = esp_wifi_start();
    }
    if (err == ESP_OK) {
        err = sensorarrayWifiApplyProfile(config->profile);
    }
    if (err == ESP_OK) {
        err = sensorarrayWifiOpenSockets();
    }
    if (err == ESP_OK) {
        BaseType_t ok = xTaskCreate(sensorarrayWifiControlTask, "wifiCtrl", 4096u, NULL, 5u,
                                    &s_wifi.ctrlTask);
        if (ok != pdPASS) {
            err = ESP_ERR_NO_MEM;
        }
    }
    if (err == ESP_OK) {
        s_wifi.initialized = true;
        s_wifi.stats.ready = true;
    }

done:
    if (err != ESP_OK) {
        sensorarrayWifiCleanupAttempt();
    }
    s_wifi.stats.initError = err;
    printf("WIFIBOOT,profile=%s,ssid=%s,dataPort=%u,logPort=%u,ctrlPort=%u,"
           "err=0x%lx,name=%s\n",
           sensorarrayWifiProfileName(config->profile), config->ssid,
           (unsigned)config->dataPort, (unsigned)config->logPort,
           (unsigned)config->ctrlPort, (unsigned long)err, esp_err_to_name(err));
    return err;
}

esp_err_t sensorarrayWifiSend(sensorarrayWifiChannel_t channel,
                              const uint8_t *data,
                              size_t length)
{
    if (channel > SENSORARRAY_WIFI_CH_CTRL || !data || length == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_wifi.stats.ready || !s_wifi.stats.stationConnected || s_wifi.txSocket < 0) {
        s_wifi.stats.dropped[channel]++;
        return ESP_ERR_INVALID_STATE;
    }
    struct sockaddr_in target = {
        .sin_family = AF_INET,
        .sin_port = htons(sensorarrayWifiChannelPort(channel)),
        .sin_addr.s_addr = inet_addr("192.168.4.255"),
    };
    int sent = sendto(s_wifi.txSocket, data, length, MSG_DONTWAIT,
                      (struct sockaddr *)&target, sizeof(target));
    if (sent == (int)length) {
        s_wifi.stats.sent[channel]++;
        return ESP_OK;
    }
    s_wifi.stats.dropped[channel]++;
    return ESP_ERR_TIMEOUT;
}

esp_err_t sensorarrayWifiSendControlReply(const sensorarrayWifiPeer_t *peer,
                                          const uint8_t *data,
                                          size_t length)
{
    if (!peer || !data || length == 0u || s_wifi.ctrlSocket < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    struct sockaddr_in target = {
        .sin_family = AF_INET,
        .sin_port = htons(peer->port),
        .sin_addr.s_addr = peer->ipv4,
    };
    int sent = sendto(s_wifi.ctrlSocket, data, length, MSG_DONTWAIT,
                      (struct sockaddr *)&target, sizeof(target));
    if (sent == (int)length) {
        s_wifi.stats.ctrlTx++;
        s_wifi.stats.sent[SENSORARRAY_WIFI_CH_CTRL]++;
        return ESP_OK;
    }
    s_wifi.stats.dropped[SENSORARRAY_WIFI_CH_CTRL]++;
    return ESP_ERR_TIMEOUT;
}

void sensorarrayWifiSetControlRxCallback(sensorarrayWifiControlRxCallback_t callback,
                                        void *userContext)
{
    s_wifi.controlCallback = callback;
    s_wifi.controlContext = userContext;
}

void sensorarrayWifiGetStats(sensorarrayWifiStats_t *outStats)
{
    if (outStats) {
        *outStats = s_wifi.stats;
    }
}

bool sensorarrayWifiIsReady(void)
{
    return s_wifi.stats.ready;
}
