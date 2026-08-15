#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sensorarrayBle.h"
#include "sensorarrayScanConfig.h"
#include "sensorarrayTransportInternal.h"
#include "sensorarrayWifi.h"

#define CHECK(condition)                                                        \
    do {                                                                        \
        if (!(condition)) {                                                     \
            fprintf(stderr, "FAIL,line=%d,condition=%s\n", __LINE__, #condition); \
            return 1;                                                           \
        }                                                                       \
    } while (0)

sensorarrayTransportLegacyCommandCallback_t g_sensorarrayTransportLegacyCallback;
void *g_sensorarrayTransportLegacyContext;
sensorarrayTransportRuntimeQueryCallback_t g_sensorarrayTransportRuntimeQueryCallback;
void *g_sensorarrayTransportRuntimeQueryContext;
sensorarrayTransportControlReplyPublishedCallback_t
    g_sensorarrayTransportControlReplyPublishedCallback;
sensorarrayTransportControlReplyFailedCallback_t
    g_sensorarrayTransportControlReplyFailedCallback;

static int s_publishCalls;
static int s_publishResult = ESP_OK;
static int s_publishedCallbackCalls;
static int s_failedCallbackCalls;
static esp_err_t s_lastFailedError;
static char s_lastPublished[SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX + 1u];
static size_t s_lastPublishedLength;

esp_err_t sensorarrayTransportPublishControlReply(
    const sensorarrayTransportReplyTarget_t *target,
    const char *data,
    size_t length)
{
    (void)target;
    s_publishCalls++;
    if (s_publishResult == ESP_OK && data && length < sizeof(s_lastPublished)) {
        memcpy(s_lastPublished, data, length);
        s_lastPublishedLength = length;
    }
    return s_publishResult;
}

static void onControlReplyPublished(const char *data, size_t length)
{
    (void)data;
    (void)length;
    s_publishedCallbackCalls++;
}

static void onControlReplyFailed(esp_err_t error,
                                 const char *data,
                                 size_t length)
{
    (void)data;
    (void)length;
    s_failedCallbackCalls++;
    s_lastFailedError = error;
}

void sensorarrayTransportSetTxMode(sensorarrayTransportTxMode_t mode)
{
    (void)mode;
}

sensorarrayTransportTxMode_t sensorarrayTransportGetTxMode(void)
{
    return SENSORARRAY_TRANSPORT_TX_REL;
}

const char *sensorarrayTransportTxModeName(sensorarrayTransportTxMode_t mode)
{
    (void)mode;
    return "REL";
}

void sensorarrayTransportSetStream(sensorarrayTransportStream_t stream)
{
    (void)stream;
}

sensorarrayTransportStream_t sensorarrayTransportGetStream(void)
{
    return SENSORARRAY_TRANSPORT_STREAM_AUTO;
}

const char *sensorarrayTransportStreamName(sensorarrayTransportStream_t stream)
{
    (void)stream;
    return "AUTO";
}

void sensorarrayTransportSetWifiMode(sensorarrayTransportWifiMode_t mode)
{
    (void)mode;
}

sensorarrayTransportWifiMode_t sensorarrayTransportGetWifiMode(void)
{
    return SENSORARRAY_TRANSPORT_WIFI_OFF;
}

const char *sensorarrayTransportWifiModeName(sensorarrayTransportWifiMode_t mode)
{
    (void)mode;
    return "OFF";
}

esp_err_t sensorarrayTransportApplyWifiMode(sensorarrayTransportWifiMode_t mode)
{
    (void)mode;
    return ESP_ERR_NOT_SUPPORTED;
}

bool sensorarrayWifiIsReady(void)
{
    return false;
}

const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t mode)
{
    (void)mode;
    return "FAST";
}

sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void)
{
    return SENSORARRAY_BLE_TX_FAST;
}

void sensorarrayBleSetTxMode(sensorarrayBleTxMode_t mode)
{
    (void)mode;
}

esp_err_t sensorarrayScanConfigHandleCommand(const char *command,
                                             size_t length,
                                             char *response,
                                             size_t responseSize)
{
    (void)command;
    (void)length;
    (void)response;
    (void)responseSize;
    return ESP_ERR_NOT_SUPPORTED;
}

static esp_err_t testRuntimeQuery(const char *command,
                                  char *response,
                                  size_t responseSize,
                                  void *context)
{
    (void)context;
    if (strcmp(command, "BAT?") == 0) {
        /* Mirrors sensorarrayRuntimeQueryCommand after the ADS formatter
         * rejects an over-long ABAT: the formatter clears the buffer and
         * returns 0, and main.c maps that to ESP_FAIL. */
        if (response && responseSize > 0u) {
            response[0] = '\0';
        }
        return ESP_FAIL;
    }
    if (strcmp(command, "FAKE?") == 0) {
        snprintf(response, responseSize, "FAKE,ok=1\n");
        return ESP_OK;
    }
    if (strcmp(command, "BUSY?") == 0) {
        if (response && responseSize > 0u) {
            response[0] = '\0';
        }
        return ESP_ERR_TIMEOUT;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

static int testTruncatedBatteryReplyIsDropped(void)
{
    sensorarrayTransportReplyTarget_t target = {
        .kind = SENSORARRAY_TRANSPORT_REPLY_SERIAL,
    };
    s_publishCalls = 0;
    s_publishResult = ESP_OK;
    esp_err_t err = sensorarrayTransportHandleControlCommand(
        (const uint8_t *)"BAT?", 4u, &target);
    CHECK(err == ESP_ERR_INVALID_SIZE);
    CHECK(s_publishCalls == 0);
    return 0;
}

static int testValidRuntimeReplyIsPublished(void)
{
    sensorarrayTransportReplyTarget_t target = {
        .kind = SENSORARRAY_TRANSPORT_REPLY_SERIAL,
    };
    const char expected[] = "FAKE,ok=1\n";
    s_publishCalls = 0;
    s_publishResult = ESP_OK;
    s_publishedCallbackCalls = 0;
    s_failedCallbackCalls = 0;
    s_lastPublishedLength = 0u;
    esp_err_t err = sensorarrayTransportHandleControlCommand(
        (const uint8_t *)"FAKE?", 5u, &target);
    CHECK(err == ESP_OK);
    CHECK(s_publishCalls == 1);
    CHECK(s_publishedCallbackCalls == 1);
    CHECK(s_failedCallbackCalls == 0);
    CHECK(s_lastPublishedLength == strlen(expected));
    CHECK(memcmp(s_lastPublished, expected, s_lastPublishedLength) == 0);
    return 0;
}

static int testNonReplyErrorIsNotPublished(void)
{
    sensorarrayTransportReplyTarget_t target = {
        .kind = SENSORARRAY_TRANSPORT_REPLY_SERIAL,
    };
    s_publishCalls = 0;
    s_publishResult = ESP_OK;
    esp_err_t err = sensorarrayTransportHandleControlCommand(
        (const uint8_t *)"BUSY?", 5u, &target);
    CHECK(err == ESP_ERR_TIMEOUT);
    CHECK(s_publishCalls == 0);
    return 0;
}

static int testFailedPublishInvokesFailedCallbackOnly(void)
{
    sensorarrayTransportReplyTarget_t target = {
        .kind = SENSORARRAY_TRANSPORT_REPLY_SERIAL,
    };
    s_publishCalls = 0;
    s_publishResult = ESP_FAIL;
    s_publishedCallbackCalls = 0;
    s_failedCallbackCalls = 0;
    s_lastFailedError = ESP_OK;
    esp_err_t err = sensorarrayTransportHandleControlCommand(
        (const uint8_t *)"FAKE?", 5u, &target);
    CHECK(err == ESP_FAIL);
    CHECK(s_publishCalls == 1);
    CHECK(s_publishedCallbackCalls == 0);
    CHECK(s_failedCallbackCalls == 1);
    CHECK(s_lastFailedError == ESP_FAIL);
    s_publishResult = ESP_OK;
    return 0;
}

int main(void)
{
    g_sensorarrayTransportRuntimeQueryCallback = testRuntimeQuery;
    g_sensorarrayTransportRuntimeQueryContext = NULL;
    g_sensorarrayTransportControlReplyPublishedCallback = onControlReplyPublished;
    g_sensorarrayTransportControlReplyFailedCallback = onControlReplyFailed;
    CHECK(testTruncatedBatteryReplyIsDropped() == 0);
    CHECK(testValidRuntimeReplyIsPublished() == 0);
    CHECK(testNonReplyErrorIsNotPublished() == 0);
    CHECK(testFailedPublishInvokesFailedCallbackOnly() == 0);
    printf("CTRL_REPLY_GUARD,passed=1\n");
    return 0;
}

#include "sensorarrayTransportCommand.c"
