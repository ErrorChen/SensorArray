#include "sensorarrayTransportInternal.h"

#include <stdbool.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
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
#ifndef CONFIG_SENSORARRAY_OUTPUT_TASK_CORE
#define CONFIG_SENSORARRAY_OUTPUT_TASK_CORE 0
#endif
#ifndef CONFIG_SENSORARRAY_COMM_TASK_CORE
#define CONFIG_SENSORARRAY_COMM_TASK_CORE 0
#endif
#ifndef CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_SHORT
#define CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_SHORT 0
#endif
#ifndef CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_FULL
#define CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_FULL 0
#endif

#define SENSORARRAY_TRANSPORT_TEXT_MAX 1536u
#define SENSORARRAY_TRANSPORT_QUEUE_COUNT 3u
#define SENSORARRAY_TRANSPORT_SERIAL_COMMAND_MAX 96u
#define SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX 32u
#define SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX 48u
#define SENSORARRAY_CFG_OUTPUT_WIFI_TEXT (CONFIG_SENSORARRAY_OUTPUT_WIFI_TEXT != 0)
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT (CONFIG_SENSORARRAY_OUTPUT_BLE_CAP_TEXT != 0)

typedef struct {
    sensorarrayTransportChannel_t channel;
    uint16_t length;
    bool hasFrameMeta;
    uint32_t frameSeq;
    uint32_t generation;
    uint32_t requestId;
    uint8_t rows;
    uint8_t cells;
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
static bool s_serialCmdErrPrinted;
static uint32_t s_serialCmdErrSuppressed;
static const char *s_serialCmdErrLastReason = "none";

static void sensorarrayTransportLogSerialCommandError(const uint8_t *data,
                                                      size_t length,
                                                      const char *reason)
{
    if (!data || length == 0u) {
        return;
    }
    s_serialCmdErrLastReason = reason ? reason : "unknown";
    if (s_serialCmdErrPrinted) {
        s_serialCmdErrSuppressed++;
        if ((s_serialCmdErrSuppressed % 32u) == 0u) {
            printf("CMDERR_SUM,src=serial,count=%lu,lastReason=%s\n",
                   (unsigned long)s_serialCmdErrSuppressed,
                   s_serialCmdErrLastReason);
        }
        return;
    }
    s_serialCmdErrPrinted = true;
    char hex[(SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX * 2u) + 1u];
    char ascii[SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX + 1u];
    size_t hexLen = length < SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX ?
        length : SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX;
    for (size_t i = 0u; i < hexLen; ++i) {
        (void)snprintf(&hex[i * 2u], 3u, "%02X", data[i]);
    }
    hex[hexLen * 2u] = '\0';
    size_t asciiLen = length < SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX ?
        length : SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX;
    for (size_t i = 0u; i < asciiLen; ++i) {
        uint8_t ch = data[i];
        ascii[i] = (ch >= 0x20u && ch <= 0x7Eu) ? (char)ch : '.';
    }
    ascii[asciiLen] = '\0';
    printf("CMDERR,src=serial,len=%lu,hex=%s,ascii=%s,reason=%s,trunc=%u\n",
           (unsigned long)length,
           hex,
           ascii,
           s_serialCmdErrLastReason,
           length > hexLen ? 1u : 0u);
}

static esp_err_t sensorarrayTransportNormaliseSerialCommand(const char *input,
                                                            size_t inputLength,
                                                            char *out,
                                                            size_t outSize,
                                                            const char **outReason)
{
    if (!input || !out || outSize == 0u) {
        if (outReason) {
            *outReason = "invalid_arg";
        }
        return ESP_ERR_INVALID_ARG;
    }
    size_t write = 0u;
    for (size_t read = 0u; read < inputLength; ++read) {
        unsigned char ch = (unsigned char)input[read];
        if (ch == '\0') {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            break;
        }
        if (ch == 0x1Bu) {
            read++;
            if (read < inputLength && input[read] == '[') {
                while (read + 1u < inputLength) {
                    read++;
                    unsigned char seq = (unsigned char)input[read];
                    if (seq >= 0x40u && seq <= 0x7Eu) {
                        break;
                    }
                }
            }
            continue;
        }
        if (iscntrl(ch) && !isspace(ch)) {
            if (outReason) {
                *outReason = "control";
            }
            return ESP_ERR_INVALID_RESPONSE;
        }
        if (write + 1u >= outSize) {
            if (outReason) {
                *outReason = "too_long";
            }
            return ESP_ERR_INVALID_SIZE;
        }
        out[write++] = (char)ch;
    }
    while (write > 0u && isspace((unsigned char)out[write - 1u])) {
        write--;
    }
    out[write] = '\0';
    char *start = out;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    if (start != out) {
        memmove(out, start, strlen(start) + 1u);
        write = strlen(out);
    }
    if (write == 0u) {
        if (outReason) {
            *outReason = "empty";
        }
        return ESP_ERR_NOT_FOUND;
    }
    if (outReason) {
        *outReason = "ok";
    }
    return ESP_OK;
}

static bool sensorarrayTransportFindFieldU32(const char *line,
                                             const char *key,
                                             uint32_t *outValue)
{
    if (!line || !key || !outValue) {
        return false;
    }
    const char *match = strstr(line, key);
    if (!match) {
        return false;
    }
    match += strlen(key);
    char *end = NULL;
    unsigned long parsed = strtoul(match, &end, 10);
    if (end == match) {
        return false;
    }
    *outValue = (uint32_t)parsed;
    return true;
}

static bool sensorarrayTransportFindFieldHex8(const char *line,
                                              const char *key,
                                              uint32_t *outValue)
{
    if (!line || !key || !outValue) {
        return false;
    }
    const char *match = strstr(line, key);
    if (!match) {
        return false;
    }
    match += strlen(key);
    char *end = NULL;
    unsigned long parsed = strtoul(match, &end, 16);
    if (end == match) {
        return false;
    }
    *outValue = (uint32_t)parsed;
    return true;
}

static bool sensorarrayTransportBuildShortData(const char *data,
                                               size_t length,
                                               char *out,
                                               size_t outSize)
{
    if (!data || length == 0u || !out || outSize == 0u) {
        return false;
    }
    const char *firstLineEnd = memchr(data, '\n', length);
    if (!firstLineEnd || data[0] != 'C' || data[1] != ',') {
        return false;
    }
    char cLine[192];
    size_t cLineLength = (size_t)(firstLineEnd - data);
    if (cLineLength >= sizeof(cLine)) {
        cLineLength = sizeof(cLine) - 1u;
    }
    memcpy(cLine, data, cLineLength);
    cLine[cLineLength] = '\0';

    uint32_t seq = 0u;
    uint32_t rows = 0u;
    uint32_t rf = 0u;
    uint32_t pf = 0u;
    uint32_t sf = 0u;
    (void)sensorarrayTransportFindFieldU32(cLine, "seq=", &seq);
    (void)sensorarrayTransportFindFieldU32(cLine, "rows=", &rows);
    (void)sensorarrayTransportFindFieldHex8(cLine, "rf=", &rf);
    (void)sensorarrayTransportFindFieldHex8(cLine, "pf=", &pf);
    (void)sensorarrayTransportFindFieldHex8(cLine, "sf=", &sf);

    const char *bad = strstr(cLine, "bad=");
    char badText[24] = "na";
    if (bad) {
        bad += 4u;
        size_t index = 0u;
        while (bad[index] != '\0' && bad[index] != ',' && index + 1u < sizeof(badText)) {
            badText[index] = bad[index];
            index++;
        }
        badText[index] = '\0';
    }

    int32_t first = 0;
    int32_t last = 0;
    bool firstSet = false;
    const char *cursor = firstLineEnd + 1;
    const char *end = data + length;
    while (cursor < end) {
        const char *lineEnd = memchr(cursor, '\n', (size_t)(end - cursor));
        if (!lineEnd) {
            break;
        }
        if (cursor[0] == 'D') {
            const char *value = strchr(cursor, ',');
            while (value && value < lineEnd) {
                value++;
                char *valueEnd = NULL;
                long parsed = strtol(value, &valueEnd, 10);
                if (valueEnd == value || valueEnd > lineEnd) {
                    break;
                }
                if (!firstSet) {
                    first = (int32_t)parsed;
                    firstSet = true;
                }
                last = (int32_t)parsed;
                value = valueEnd < lineEnd ? strchr(valueEnd, ',') : NULL;
            }
        }
        cursor = lineEnd + 1;
    }

    int written = snprintf(out,
                           outSize,
                           "B20,seq=%lu,rows=%lu,cfps=na,efps=na,rf=%02lX,pf=%02lX,sf=%02lX,bad=%s,d0=%ld,dl=%ld\n",
                           (unsigned long)seq,
                           (unsigned long)rows,
                           (unsigned long)rf,
                           (unsigned long)pf,
                           (unsigned long)sf,
                           badText,
                           (long)first,
                           (long)last);
    return written > 0 && (size_t)written < outSize;
}

static void sensorarrayTransportFillFrameMeta(sensorarrayTransportItem_t *item)
{
    bool frameTag = item && item->length >= 2u && item->data[1] == ',' &&
                    (item->data[0] == 'C' || item->data[0] == 'V' ||
                     item->data[0] == 'R');
    if (!frameTag) {
        return;
    }

    uint32_t rows = 0u;
    uint32_t cells = 0u;
    uint32_t sequence = 0u;
    uint32_t generation = 0u;
    uint32_t requestId = 0u;
    if (!sensorarrayTransportFindFieldU32(item->data, "seq=", &sequence) ||
        !sensorarrayTransportFindFieldU32(item->data, "rows=", &rows) ||
        !sensorarrayTransportFindFieldU32(item->data, "cells=", &cells)) {
        return;
    }
    (void)sensorarrayTransportFindFieldU32(item->data, "gen=", &generation);
    (void)sensorarrayTransportFindFieldU32(item->data, "rid=", &requestId);
    if (rows < 1u || rows > 8u || cells != rows * 8u) {
        return;
    }

    item->hasFrameMeta = true;
    item->frameSeq = sequence;
    item->generation = generation;
    item->requestId = requestId;
    item->rows = (uint8_t)rows;
    item->cells = (uint8_t)cells;
}

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
    char command[SENSORARRAY_TRANSPORT_SERIAL_COMMAND_MAX];
    sensorarrayTransportReplyTarget_t target = {.kind = SENSORARRAY_TRANSPORT_REPLY_SERIAL};
    for (;;) {
        if (fgets(line, sizeof(line), stdin)) {
            const char *reason = "ok";
            size_t lineLength = strlen(line);
            esp_err_t normaliseErr = sensorarrayTransportNormaliseSerialCommand(
                line,
                lineLength,
                command,
                sizeof(command),
                &reason);
            if (normaliseErr == ESP_OK) {
                (void)sensorarrayTransportHandleControlCommand((const uint8_t *)command,
                                                               strlen(command), &target);
            } else if (normaliseErr != ESP_ERR_NOT_FOUND) {
                sensorarrayTransportLogSerialCommandError((const uint8_t *)line,
                                                          lineLength,
                                                          reason);
            }
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
            char shortData[192];
            const char *bleData = item.data;
            size_t bleLength = item.length;
            if (item.channel == SENSORARRAY_TRANSPORT_CHANNEL_DATA &&
                sensorarrayTransportGetTxMode() == SENSORARRAY_TRANSPORT_TX_SHORT &&
                sensorarrayTransportBuildShortData(item.data,
                                                   item.length,
                                                   shortData,
                                                   sizeof(shortData))) {
                bleData = shortData;
                bleLength = strlen(shortData);
            }
            esp_err_t bleErr = sensorarrayBleNotify(bleChannel,
                                                    (const uint8_t *)bleData,
                                                    bleLength);
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
    sensorarrayTransportFillFrameMeta(&item);
    if (xQueueSend(s_queue, &item, 0) != pdTRUE) {
        portENTER_CRITICAL(&g_sensorarrayTransportStatsMux);
        g_sensorarrayTransportStats.queueDrop++;
        portEXIT_CRITICAL(&g_sensorarrayTransportStatsMux);
        if (item.hasFrameMeta) {
            printf("TXDROP,ch=%u,seq=%lu,rows=%u,cells=%u,gen=%lu,rid=%lu,reason=transport_queue_full\n",
                   (unsigned)item.channel,
                   (unsigned long)item.frameSeq,
                   (unsigned)item.rows,
                   (unsigned)item.cells,
                   (unsigned long)item.generation,
                   (unsigned long)item.requestId);
        }
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
    sensorarrayTransportSetStream(SENSORARRAY_TRANSPORT_STREAM_AUTO);
#if CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_SHORT
    sensorarrayTransportSetTxMode(SENSORARRAY_TRANSPORT_TX_SHORT);
#elif CONFIG_SENSORARRAY_TRANSPORT_TX_DEFAULT_FULL
    sensorarrayTransportSetTxMode(SENSORARRAY_TRANSPORT_TX_FULL);
#else
    sensorarrayTransportSetTxMode(SENSORARRAY_TRANSPORT_TX_REL);
#endif
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
    BaseType_t ok = xTaskCreatePinnedToCore(sensorarrayTransportTask,
                                            "transport",
                                            6144u,
                                            (void *)(intptr_t)wifiErr,
                                            5u,
                                            &s_task,
                                            CONFIG_SENSORARRAY_OUTPUT_TASK_CORE);
    if (ok != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    ok = xTaskCreatePinnedToCore(sensorarrayTransportSerialControlTask,
                                 "serialCtrl",
                                 3072u,
                                 NULL,
                                 4u,
                                 NULL,
                                 CONFIG_SENSORARRAY_COMM_TASK_CORE);
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
