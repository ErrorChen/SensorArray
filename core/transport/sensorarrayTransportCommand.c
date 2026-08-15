#include "sensorarrayTransportInternal.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "sensorarrayBle.h"
#include "sensorarrayScanConfig.h"

#define SENSORARRAY_TRANSPORT_CMDERR_HEX_MAX 32u
#define SENSORARRAY_TRANSPORT_CMDERR_ASCII_MAX 48u

static bool s_serialUnknownPrinted;
static uint32_t s_serialUnknownSuppressed;

static bool sensorarrayTransportSerialUnknownShouldReply(const uint8_t *data,
                                                         size_t length,
                                                         const char *reason)
{
    if (!data || length == 0u) {
        return false;
    }
    if (s_serialUnknownPrinted) {
        s_serialUnknownSuppressed++;
        if ((s_serialUnknownSuppressed % 32u) == 0u) {
            printf("CMDERR_SUM,src=serial,count=%lu,lastReason=%s\n",
                   (unsigned long)s_serialUnknownSuppressed,
                   reason ? reason : "unsupported");
        }
        return false;
    }
    s_serialUnknownPrinted = true;

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
           reason ? reason : "unsupported",
           length > hexLen ? 1u : 0u);
    return true;
}

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
    if (strcmp(text, "TX=REL") == 0 || strcmp(text, "TX=RT") == 0 ||
        strcmp(text, "TX=SHORT") == 0 || strcmp(text, "TX=FULL") == 0) {
        sensorarrayTransportTxMode_t mode = SENSORARRAY_TRANSPORT_TX_REL;
        if (strcmp(text, "TX=SHORT") == 0 || strcmp(text, "TX=RT") == 0) {
            mode = SENSORARRAY_TRANSPORT_TX_SHORT;
        } else if (strcmp(text, "TX=FULL") == 0) {
            mode = SENSORARRAY_TRANSPORT_TX_FULL;
        }
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
    if (strcmp(text, "ST=AUTO") == 0 ||
        strcmp(text, "ST=SER") == 0 || strcmp(text, "ST=BLE") == 0 ||
        strcmp(text, "ST=WIFI") == 0 || strcmp(text, "ST=ALL") == 0) {
        sensorarrayTransportStream_t stream = SENSORARRAY_TRANSPORT_STREAM_AUTO;
        if (strcmp(text, "ST=SER") == 0) {
            stream = SENSORARRAY_TRANSPORT_STREAM_SER;
        } else if (strcmp(text, "ST=BLE") == 0) {
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

    if (strcmp(text, "BTX?") == 0) {
        snprintf(response, responseSize, "ACK,cmd=BTX,v=%s\n",
                 sensorarrayBleTxModeName(sensorarrayBleGetTxMode()));
        return ESP_OK;
    }
    if (strcmp(text, "BTX=FAST") == 0 || strcmp(text, "BTX=SAFE") == 0) {
        sensorarrayBleTxMode_t mode = strcmp(text, "BTX=SAFE") == 0 ?
            SENSORARRAY_BLE_TX_SAFE : SENSORARRAY_BLE_TX_FAST;
        sensorarrayBleSetTxMode(mode);
        snprintf(response, responseSize, "ACK,cmd=BTX,v=%s\n",
                 sensorarrayBleTxModeName(mode));
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

    if (g_sensorarrayTransportRuntimeQueryCallback) {
        return g_sensorarrayTransportRuntimeQueryCallback(
            text,
            response,
            responseSize,
            g_sensorarrayTransportRuntimeQueryContext);
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
    /* MODE?/STATE? intentionally report the complete immutable hardware
     * snapshot (matrix excitation, ADS reference, VBIAS and PGA separately). */
    /* STATE now includes independently verified FDC sleep state in addition
     * to the immutable ADS/route snapshot. Keep one bounded control response;
     * data-frame slots and their 1536-byte contract are unchanged. */
    char response[512] = {0};
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
    }
    bool replyUnsupported = true;
    if (err == ESP_ERR_NOT_SUPPORTED &&
        replyTarget->kind == SENSORARRAY_TRANSPORT_REPLY_SERIAL) {
        replyUnsupported = sensorarrayTransportSerialUnknownShouldReply(data,
                                                                        length,
                                                                        "unsupported");
    }
    if (!replyUnsupported) {
        return err;
    }
    if (err == ESP_ERR_NOT_SUPPORTED) {
        snprintf(response, sizeof(response), "ERR,cmd=UNKNOWN,reason=unsupported\n");
    }
    if (err != ESP_OK && response[0] == '\0') {
        if (err == ESP_FAIL) {
            printf("CTRLDROP,ch=ctrl,len=%lu,max=%u,reason=reply_too_large\n",
                   (unsigned long)SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX,
                   (unsigned)SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX);
            return ESP_ERR_INVALID_SIZE;
        }
        return err;
    }
    esp_err_t replyErr = sensorarrayTransportPublishControlReply(replyTarget,
                                                                 response,
                                                                 strlen(response));
    /* The published hook closes the ROWMODES ACK/terminal ordering barrier,
     * so it must only run when the ACK actually left the device.  A failed
     * publish reports through the separate failure hook instead of being
     * treated as an accepted, acknowledged transaction. */
    if (replyErr == ESP_OK && g_sensorarrayTransportControlReplyPublishedCallback) {
        g_sensorarrayTransportControlReplyPublishedCallback(response,
                                                             strlen(response));
    } else if (replyErr != ESP_OK &&
               g_sensorarrayTransportControlReplyFailedCallback) {
        g_sensorarrayTransportControlReplyFailedCallback(replyErr,
                                                         response,
                                                         strlen(response));
    }
    return replyErr == ESP_OK ? err : replyErr;
}
