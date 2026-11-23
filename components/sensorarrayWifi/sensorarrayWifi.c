#include "sensorarrayWifiOwner.h"
#include "sensorarrayWorkBudget.h"

#include <errno.h>
#include <string.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "sdkconfig.h"

#ifndef CONFIG_SENSORARRAY_NET_TASK_CORE
#define CONFIG_SENSORARRAY_NET_TASK_CORE 0
#endif
#define SENSORARRAY_WIFI_CTRL_BUFFER 256u

typedef struct {
    sensorarrayWifiConfig_t config;
    int txSocket;
    int ctrlSocket;
    TaskHandle_t ctrlTask;
    esp_netif_t *apNetif;
    bool attachAttempted;
    bool driverInitialized;
    bool driverInitUncertain;
    bool started;
    bool eventHandlerRegistered;
} sensorarrayWifiState_t;

/* Only the persistent owner mutates backend resources. Once ctrlTask exists,
 * both socket and task are retained for this boot, including OFF/quarantine.
 * A blocked recvfrom cannot be cancelled by deleting its owner or socket. */
static sensorarrayWifiState_t s_wifi = {.txSocket = -1, .ctrlSocket = -1};

/* SDK init arguments must outlive every blocked/late init call. The sole
 * persistent owner writes them before entering SDK calls; no timeout/OFF
 * path frees or replaces them. Ordinary CPU data, kept in internal BSS. */
static struct {
    esp_netif_config_t netif;
    wifi_init_config_t driver;
    wifi_config_t ap;
} s_wifiInitScratch;

/* Record the real SDK return first. Guard revocation never means that the
 * SDK call was cancelled, nor does it erase resources actually acquired. */
#define WIFI_STEP(phase_, call_, success_) do { \
    if (!sensorarrayWifiOwnerBeginSdk(phase_)) { return ESP_ERR_INVALID_STATE; } \
    esp_err_t stepError = (call_); \
    if (stepError == ESP_OK) { success_; } \
    bool stepAllowed = sensorarrayWifiOwnerEndSdk(stepError, 0, 0); \
    if (stepError != ESP_OK) { return stepError; } \
    if (!stepAllowed) { return ESP_ERR_INVALID_STATE; } \
} while (0)

static void sensorarrayWifiEvent(void *arg, esp_event_base_t base,
    int32_t eventId, void *eventData)
{
    (void)arg; (void)base; (void)eventData;
    /* Application callback only records state. IDF's separately registered
     * default AP handlers still run in the SDK event thread. */
    if (eventId == WIFI_EVENT_AP_STACONNECTED) { sensorarrayWifiOwnerNoteStationEvent(true); }
    else if (eventId == WIFI_EVENT_AP_STADISCONNECTED) { sensorarrayWifiOwnerNoteStationEvent(false); }
}

esp_err_t sensorarrayWifiBackendApplyProfile(sensorarrayWifiProfile_t profile)
{
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_SET_POWER, esp_wifi_set_max_tx_power(80), (void)0);
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_SET_PS, esp_wifi_set_ps(WIFI_PS_NONE), (void)0);
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_SET_PROTOCOL,
        esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N), (void)0);
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_SET_BANDWIDTH,
        esp_wifi_set_bandwidth(WIFI_IF_AP, profile == SENSORARRAY_WIFI_PROFILE_HIGH ? WIFI_BW_HT40 : WIFI_BW_HT20),
        s_wifi.config.profile = profile);
    return ESP_OK;
}

static void sensorarrayWifiConfigureInit(wifi_init_config_t *init, sensorarrayWifiProfile_t profile)
{
    /* Existing memory/profile policy is unchanged. NO_MEM is an ordinary
     * failed request, not an assertion or an implicit fallback allocation. */
    if (profile == SENSORARRAY_WIFI_PROFILE_HIGH) {
        init->static_rx_buf_num = 8; init->dynamic_rx_buf_num = 16;
        init->dynamic_tx_buf_num = 16; init->rx_ba_win = 6; init->mgmt_sbuf_num = 12;
    } else {
        init->static_rx_buf_num = 6; init->dynamic_rx_buf_num = 8;
        init->dynamic_tx_buf_num = 8; init->rx_ba_win = 4; init->mgmt_sbuf_num = 8;
    }
}

/* Keep aggregate default construction outside the deeper SDK call chain,
 * even if this compiler materializes WIFI_INIT_CONFIG_DEFAULT on the stack. */
static void __attribute__((noinline))
sensorarrayWifiPrepareInitScratch(const sensorarrayWifiConfig_t *config)
{
    s_wifiInitScratch.netif = (esp_netif_config_t)ESP_NETIF_DEFAULT_WIFI_AP();
    s_wifiInitScratch.driver = (wifi_init_config_t)WIFI_INIT_CONFIG_DEFAULT();
    sensorarrayWifiConfigureInit(&s_wifiInitScratch.driver, config->profile);
    memset(&s_wifiInitScratch.ap, 0, sizeof(s_wifiInitScratch.ap));
    size_t ssidLength = strnlen(config->ssid, sizeof(s_wifiInitScratch.ap.ap.ssid));
    memcpy(s_wifiInitScratch.ap.ap.ssid, config->ssid, ssidLength);
    s_wifiInitScratch.ap.ap.ssid_len = (uint8_t)ssidLength;
    size_t passwordLength = strnlen(config->password, sizeof(s_wifiInitScratch.ap.ap.password));
    memcpy(s_wifiInitScratch.ap.ap.password, config->password, passwordLength);
    s_wifiInitScratch.ap.ap.channel = 1u;
    s_wifiInitScratch.ap.ap.max_connection = config->profile == SENSORARRAY_WIFI_PROFILE_HIGH ? 4u : 1u;
    s_wifiInitScratch.ap.ap.authmode = passwordLength >= 8u ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
}

static esp_err_t sensorarrayWifiSocketOption(int fd, int option, const void *value, socklen_t length)
{
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_SETSOCKOPT)) { return ESP_ERR_INVALID_STATE; }
    int result = setsockopt(fd, SOL_SOCKET, option, value, length);
    int savedErrno = result < 0 ? errno : 0;
    esp_err_t error = result == 0 ? ESP_OK : ESP_FAIL;
    bool allowed = sensorarrayWifiOwnerEndSdk(error, savedErrno, result);
    return error != ESP_OK ? error : allowed ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t sensorarrayWifiOpenSocket(int *fd)
{
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_SOCKET)) { return ESP_ERR_INVALID_STATE; }
    *fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    int savedErrno = *fd < 0 ? errno : 0;
    esp_err_t error = *fd >= 0 ? ESP_OK : ESP_FAIL;
    bool allowed = sensorarrayWifiOwnerEndSdk(error, savedErrno, *fd);
    return error != ESP_OK ? error : allowed ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t sensorarrayWifiOpenSockets(void)
{
    esp_err_t error = sensorarrayWifiOpenSocket(&s_wifi.txSocket);
    if (error != ESP_OK) { return error; }
    int enabled = 1;
    error = sensorarrayWifiSocketOption(s_wifi.txSocket, SO_BROADCAST, &enabled, sizeof(enabled));
    if (error != ESP_OK) { return error; }
    error = sensorarrayWifiOpenSocket(&s_wifi.ctrlSocket);
    if (error != ESP_OK) { return error; }
    struct sockaddr_in address = {.sin_family = AF_INET,
        .sin_port = htons(s_wifi.config.ctrlPort), .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_BIND)) { return ESP_ERR_INVALID_STATE; }
    int result = bind(s_wifi.ctrlSocket, (struct sockaddr *)&address, sizeof(address));
    int savedErrno = result < 0 ? errno : 0;
    error = result == 0 ? ESP_OK : ESP_FAIL;
    bool allowed = sensorarrayWifiOwnerEndSdk(error, savedErrno, result);
    if (error != ESP_OK || !allowed) { return error != ESP_OK ? error : ESP_ERR_INVALID_STATE; }
    struct timeval timeout = {.tv_sec = 0, .tv_usec = 250000};
    return sensorarrayWifiSocketOption(s_wifi.ctrlSocket, SO_RCVTIMEO, &timeout, sizeof(timeout));
}

static void sensorarrayWifiControlTask(void *arg)
{
    (void)arg;
    uint8_t buffer[SENSORARRAY_WIFI_CTRL_BUFFER];
    while (true) {
        (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_WIFI_CTRL);
        if (!sensorarrayWifiOwnerRxAllowed()) {
            /* Quarantine is sticky. No new receive/SDK operation; an earlier
             * recvfrom was allowed to return on this same persistent stack. */
            (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        struct sockaddr_in source = {0};
        socklen_t sourceLength = sizeof(source);
        int received = recvfrom(s_wifi.ctrlSocket, buffer, sizeof(buffer), 0,
            (struct sockaddr *)&source, &sourceLength);
        if (received <= 0) { continue; }
        sensorarrayWifiPeer_t peer = {.ipv4 = source.sin_addr.s_addr, .port = ntohs(source.sin_port)};
        sensorarrayWifiOwnerReceive(buffer, (size_t)received, &peer);
    }
}

esp_err_t sensorarrayWifiBackendInitialize(const sensorarrayWifiConfig_t *config)
{
    if (!config || s_wifi.ctrlTask || s_wifi.apNetif || s_wifi.driverInitialized) { return ESP_ERR_INVALID_STATE; }
    s_wifi.config = *config;
    sensorarrayWifiPrepareInitScratch(config);
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_NETIF_INIT)) { return ESP_ERR_INVALID_STATE; }
    esp_err_t error = esp_netif_init();
    bool allowed = sensorarrayWifiOwnerEndSdk(error, 0, 0);
    if ((error != ESP_OK && error != ESP_ERR_INVALID_STATE) || !allowed) {
        return error != ESP_OK ? error : ESP_ERR_INVALID_STATE;
    }
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_EVENT_LOOP)) { return ESP_ERR_INVALID_STATE; }
    error = esp_event_loop_create_default();
    allowed = sensorarrayWifiOwnerEndSdk(error, 0, 0);
    if ((error != ESP_OK && error != ESP_ERR_INVALID_STATE) || !allowed) {
        return error != ESP_OK ? error : ESP_ERR_INVALID_STATE;
    }
    /* Do not use esp_netif_create_default_wifi_ap: this IDF helper asserts on
     * allocation/attach/handler failure. Each official lower-level API is
     * checked, and partial ownership is retained for bounded cleanup. */
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_NETIF_NEW)) { return ESP_ERR_INVALID_STATE; }
    s_wifi.apNetif = esp_netif_new(&s_wifiInitScratch.netif);
    error = s_wifi.apNetif ? ESP_OK : ESP_ERR_NO_MEM;
    allowed = sensorarrayWifiOwnerEndSdk(error, 0, 0);
    if (error != ESP_OK || !allowed) { return error != ESP_OK ? error : ESP_ERR_INVALID_STATE; }
    /* IDF publishes its AP pointer before attach completes. Even failure must
     * use the checked clear-default API before releasing our netif. */
    s_wifi.attachAttempted = true;
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_NETIF_ATTACH, esp_netif_attach_wifi_ap(s_wifi.apNetif), (void)0);
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_DEFAULT_HANDLERS, esp_wifi_set_default_wifi_ap_handlers(), (void)0);
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_DRIVER_INIT)) { return ESP_ERR_INVALID_STATE; }
    error = esp_wifi_init(&s_wifiInitScratch.driver);
    s_wifi.driverInitialized = error == ESP_OK;
    /* IDF 5.5.5 returns the init error even if its private error-path deinit
     * also failed. Public deinit may then report NOT_INIT. There is no public
     * proof of reclamation: retain associated netif/handlers and quarantine. */
    s_wifi.driverInitUncertain = error != ESP_OK;
    allowed = sensorarrayWifiOwnerEndSdk(error, 0, 0);
    if (error != ESP_OK || !allowed) { return error != ESP_OK ? error : ESP_ERR_INVALID_STATE; }
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_EVENT_REGISTER,
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, sensorarrayWifiEvent, NULL),
        s_wifi.eventHandlerRegistered = true);
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_SET_MODE, esp_wifi_set_mode(WIFI_MODE_AP), (void)0);
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_SET_CONFIG, esp_wifi_set_config(WIFI_IF_AP, &s_wifiInitScratch.ap), (void)0);
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_START, esp_wifi_start(), s_wifi.started = true);
    error = sensorarrayWifiBackendApplyProfile(config->profile);
    if (error != ESP_OK) { return error; }
    error = sensorarrayWifiOpenSockets();
    if (error != ESP_OK) { return error; }
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_RX_TASK_CREATE)) { return ESP_ERR_INVALID_STATE; }
    /* This is the last resource creation. Once it succeeds no local teardown
     * can prove recvfrom has stopped; keep this task/socket until restart. */
    BaseType_t created = xTaskCreatePinnedToCore(sensorarrayWifiControlTask, "wifiCtrl", 4096u, NULL, 5u,
        &s_wifi.ctrlTask, CONFIG_SENSORARRAY_NET_TASK_CORE);
    error = created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
    allowed = sensorarrayWifiOwnerEndSdk(error, 0, 0);
    return error != ESP_OK ? error : allowed ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t sensorarrayWifiCloseSocket(int *fd)
{
    if (*fd < 0) { return ESP_OK; }
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_CLOSE)) { return ESP_ERR_INVALID_STATE; }
    int result = close(*fd);
    int savedErrno = result < 0 ? errno : 0;
    if (result == 0) { *fd = -1; }
    esp_err_t error = result == 0 ? ESP_OK : ESP_FAIL;
    bool allowed = sensorarrayWifiOwnerEndSdk(error, savedErrno, result);
    return error != ESP_OK ? error : allowed ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t sensorarrayWifiBackendCleanup(void)
{
    if (s_wifi.ctrlTask || s_wifi.driverInitUncertain) { return ESP_ERR_INVALID_STATE; }
    esp_err_t error = sensorarrayWifiCloseSocket(&s_wifi.ctrlSocket);
    if (error != ESP_OK) { return error; }
    error = sensorarrayWifiCloseSocket(&s_wifi.txSocket);
    if (error != ESP_OK) { return error; }
    if (s_wifi.eventHandlerRegistered) {
        WIFI_STEP(SENSORARRAY_WIFI_PHASE_EVENT_UNREGISTER,
            esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, sensorarrayWifiEvent),
            s_wifi.eventHandlerRegistered = false);
    }
    if (s_wifi.started) { WIFI_STEP(SENSORARRAY_WIFI_PHASE_STOP, esp_wifi_stop(), s_wifi.started = false); }
    if (s_wifi.driverInitialized) {
        WIFI_STEP(SENSORARRAY_WIFI_PHASE_DEINIT, esp_wifi_deinit(), s_wifi.driverInitialized = false);
    }
    if (s_wifi.apNetif && s_wifi.attachAttempted) {
        WIFI_STEP(SENSORARRAY_WIFI_PHASE_CLEAR_HANDLERS,
            esp_wifi_clear_default_wifi_driver_and_handlers(s_wifi.apNetif), s_wifi.attachAttempted = false);
    }
    if (s_wifi.apNetif) {
        if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_NETIF_DESTROY)) { return ESP_ERR_INVALID_STATE; }
        esp_netif_destroy(s_wifi.apNetif); /* void: actual API return, not a physical-off proof */
        s_wifi.apNetif = NULL;
        if (!sensorarrayWifiOwnerEndSdk(ESP_OK, 0, 0)) { return ESP_ERR_INVALID_STATE; }
    }
    return ESP_OK;
}

bool sensorarrayWifiBackendHasReader(void) { return s_wifi.ctrlTask != NULL; }

esp_err_t sensorarrayWifiBackendSend(const sensorarrayWifiPublication_t *p, int *returned, int *sdkErrno)
{
    int fd = p->unicast ? s_wifi.ctrlSocket : s_wifi.txSocket;
    if (fd < 0) { return ESP_ERR_INVALID_STATE; }
    uint16_t port = p->channel == SENSORARRAY_WIFI_CH_DATA ? s_wifi.config.dataPort :
        p->channel == SENSORARRAY_WIFI_CH_LOG ? s_wifi.config.logPort : s_wifi.config.ctrlPort;
    struct sockaddr_in target = {.sin_family = AF_INET,
        .sin_port = htons(p->unicast ? p->peer.port : port),
        .sin_addr.s_addr = p->unicast ? p->peer.ipv4 : htonl(UINT32_C(0xc0a804ff))};
    if (!sensorarrayWifiOwnerBeginSdk(SENSORARRAY_WIFI_PHASE_SEND)) { return ESP_ERR_INVALID_STATE; }
    *returned = sendto(fd, p->data, p->length, MSG_DONTWAIT, (struct sockaddr *)&target, sizeof(target));
    *sdkErrno = *returned < 0 ? errno : 0;
    esp_err_t error = *returned == (int)p->length ? ESP_OK : ESP_ERR_TIMEOUT;
    bool allowed = sensorarrayWifiOwnerEndSdk(error, *sdkErrno, *returned);
    return error != ESP_OK ? error : allowed ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t sensorarrayWifiBackendStationQuery(bool *connected)
{
    wifi_sta_list_t stations = {0};
    WIFI_STEP(SENSORARRAY_WIFI_PHASE_STATION_QUERY, esp_wifi_ap_get_sta_list(&stations), (void)0);
    *connected = stations.num != 0u;
    return ESP_OK;
}
