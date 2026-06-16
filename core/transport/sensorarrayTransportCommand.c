#include "sensorarrayTransportInternal.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "sensorarrayScanConfig.h"

static void sensorarrayTransportCommandNormalize(char *text)
{
    char *read = text;
    while (*read && isspace((unsigned char)*read)) {
        read++;
    }
    char *write = text;
    while (*read && *read != '\r' && *read != '\n') {
        *write++ = (char)toupper((unsigned char)*read++);
    }
    while (write > text && isspace((unsigned char)write[-1])) {
        write--;
    }
    *write = '\0';
}

static esp_err_t sensorarrayTransportCommandParse(
    const uint8_t *data,
    size_t length,
    char *text,
    size_t textSize)
{
    if (!data || length == 0u || !text || textSize == 0u || length >= textSize) {
        return ESP_ERR_INVALID_ARG;
    }
    memcpy(text, data, length);
    text[length] = '\0';
    sensorarrayTransportCommandNormalize(text);
    return ESP_OK;
}

static esp_err_t sensorarrayTransportHandleRuntimeCommand(const char *text,
                                                          char *response,
                                                          size_t responseSize)
{
    if (!text || !response || responseSize == 0u) {
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(text, "TX?") == 0) {
        snprintf(response, responseSize, "ACK,cmd=TX,v=%s\n",
                 sensorarrayTransportTxModeName(sensorarrayTransportGetTxMode()));
        return ESP_OK;
    }
    if (strcmp(text, "TX=REL") == 0 || strcmp(text, "TX=RT") == 0) {
        sensorarrayTransportTxMode_t mode =
            strcmp(text, "TX=RT") == 0 ? SENSORARRAY_TRANSPORT_TX_RT :
                                         SENSORARRAY_TRANSPORT_TX_REL;
        sensorarrayTransportSetTxMode(mode);
        snprintf(response, responseSize, "ACK,cmd=TX,v=%s\n",
                 sensorarrayTransportTxModeName(mode));
        return ESP_OK;
    }

    if (strcmp(text, "ST?") == 0) {
        snprintf(response, responseSize, "ACK,cmd=ST,v=%s\n",
                 sensorarrayTransportStreamName(sensorarrayTransportGetStream()));
        return ESP_OK;
    }
    if (strcmp(text, "ST=SER") == 0 || strcmp(text, "ST=BLE") == 0 ||
        strcmp(text, "ST=WIFI") == 0 || strcmp(text, "ST=ALL") == 0) {
        sensorarrayTransportStream_t stream = SENSORARRAY_TRANSPORT_STREAM_SER;
        if (strcmp(text, "ST=BLE") == 0) {
            stream = SENSORARRAY_TRANSPORT_STREAM_BLE;
        } else if (strcmp(text, "ST=WIFI") == 0) {
            stream = SENSORARRAY_TRANSPORT_STREAM_WIFI;
        } else if (strcmp(text, "ST=ALL") == 0) {
            stream = SENSORARRAY_TRANSPORT_STREAM_ALL;
        }
        sensorarrayTransportSetStream(stream);
        snprintf(response, responseSize, "ACK,cmd=ST,v=%s\n",
                 sensorarrayTransportStreamName(stream));
        return ESP_OK;
    }

    if (strcmp(text, "WIFI?") == 0) {
        snprintf(response, responseSize, "ACK,cmd=WIFI,v=%s,ok=%u\n",
                 sensorarrayTransportWifiModeName(sensorarrayTransportGetWifiMode()),
                 sensorarrayWifiIsReady() ? 1u : 0u);
        return ESP_OK;
    }
    if (strcmp(text, "WIFI=OFF") == 0 || strcmp(text, "WIFI=AP") == 0) {
        sensorarrayTransportWifiMode_t mode =
            strcmp(text, "WIFI=AP") == 0 ? SENSORARRAY_TRANSPORT_WIFI_AP :
                                           SENSORARRAY_TRANSPORT_WIFI_OFF;
        esp_err_t err = sensorarrayTransportApplyWifiMode(mode);
        if (err == ESP_OK) {
            snprintf(response, responseSize, "ACK,cmd=WIFI,v=%s,ok=%u\n",
                     sensorarrayTransportWifiModeName(mode), 1u);
        } else {
            snprintf(response, responseSize, "ERR,cmd=WIFI,v=%s,e=0x%lx\n",
                     sensorarrayTransportWifiModeName(mode), (unsigned long)err);
        }
        return err;
    }
    if (strcmp(text, "WIFI=STA") == 0 || strcmp(text, "WIFI=APSTA") == 0 ||
        strcmp(text, "WIFI_SCAN") == 0 ||
        strncmp(text, "WIFI_STA_SSID=", 14u) == 0 ||
        strncmp(text, "WIFI_STA_PASS=", 14u) == 0 ||
        strcmp(text, "WIFI_SAVE=1") == 0 ||
        strcmp(text, "WIFI_CONNECT=1") == 0 ||
        strcmp(text, "WIFI_FORGET=1") == 0) {
        snprintf(response, responseSize, "ERR,cmd=WIFI,reason=sta_nyi\n");
        return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t sensorarrayTransportHandleControlCommand(
    const uint8_t *data,
    size_t length,
    const sensorarrayTransportReplyTarget_t *replyTarget)
{
    if (!data || length == 0u || !replyTarget) {
        return ESP_ERR_INVALID_ARG;
    }
    char response[128];
    esp_err_t err = sensorarrayScanConfigHandleCommand((const char *)data, length,
                                                        response, sizeof(response));
    if (err == ESP_ERR_NOT_SUPPORTED) {
        char text[96];
        err = sensorarrayTransportCommandParse(data, length, text, sizeof(text));
        if (err == ESP_OK) {
            err = sensorarrayTransportHandleRuntimeCommand(text, response,
                                                           sizeof(response));
        }
    }
    if (err == ESP_ERR_NOT_SUPPORTED && g_sensorarrayTransportLegacyCallback) {
        err = g_sensorarrayTransportLegacyCallback(data, length,
                                                   g_sensorarrayTransportLegacyContext);
        snprintf(response, sizeof(response),
                 err == ESP_OK ? "ACK,cmd=LEGACY,state=queued\n" :
                                 "ERR,cmd=UNKNOWN,reason=unsupported\n");
    } else if (err == ESP_ERR_NOT_SUPPORTED) {
        snprintf(response, sizeof(response), "ERR,cmd=UNKNOWN,reason=unsupported\n");
    }
    esp_err_t replyErr = sensorarrayTransportPublishControlReply(replyTarget,
                                                                 response,
                                                                 strlen(response));
    return replyErr == ESP_OK ? err : replyErr;
}
