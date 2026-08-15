#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensorarrayTransportChannels.h"
#include "sensorarrayWifi.h"

#define SENSORARRAY_TRANSPORT_TEXT_MAX 1536u
/* CTRL replies are capped at 512 bytes; oversized replies are rejected,
 * never truncated. */
#define SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX 512u

typedef enum {
    SENSORARRAY_TRANSPORT_REPLY_SERIAL = 0,
    SENSORARRAY_TRANSPORT_REPLY_BLE,
    SENSORARRAY_TRANSPORT_REPLY_WIFI,
} sensorarrayTransportReplyKind_t;

typedef enum {
    SENSORARRAY_TRANSPORT_TX_SHORT = 0,
    SENSORARRAY_TRANSPORT_TX_REL,
    SENSORARRAY_TRANSPORT_TX_FULL,
} sensorarrayTransportTxMode_t;

typedef enum {
    SENSORARRAY_TRANSPORT_STREAM_AUTO = 0u,
    SENSORARRAY_TRANSPORT_STREAM_SER = 1u << 0,
    SENSORARRAY_TRANSPORT_STREAM_BLE = 1u << 1,
    SENSORARRAY_TRANSPORT_STREAM_WIFI = 1u << 2,
    SENSORARRAY_TRANSPORT_STREAM_ALL = SENSORARRAY_TRANSPORT_STREAM_SER |
                                       SENSORARRAY_TRANSPORT_STREAM_BLE |
                                       SENSORARRAY_TRANSPORT_STREAM_WIFI,
} sensorarrayTransportStream_t;

typedef enum {
    SENSORARRAY_TRANSPORT_WIFI_OFF = 0,
    SENSORARRAY_TRANSPORT_WIFI_STA,
    SENSORARRAY_TRANSPORT_WIFI_AP,
    SENSORARRAY_TRANSPORT_WIFI_APSTA,
} sensorarrayTransportWifiMode_t;

typedef struct {
    sensorarrayTransportReplyKind_t kind;
    sensorarrayWifiPeer_t wifiPeer;
} sensorarrayTransportReplyTarget_t;

typedef struct {
    uint32_t serialDataSent;
    uint32_t serialDataDrop;
    uint32_t serialLogSent;
    uint32_t serialLogDrop;
    uint32_t wifiDataSent;
    uint32_t wifiDataDrop;
    uint32_t wifiLogSent;
    uint32_t wifiLogDrop;
    uint32_t wifiCtrlRx;
    uint32_t wifiCtrlTx;
    uint32_t bleDataSent;
    uint32_t bleDataDrop;
    uint32_t bleLogSent;
    uint32_t bleLogDrop;
    uint32_t bleCtrlRx;
    uint32_t bleCtrlTx;
    uint32_t bleCongested;
    uint32_t queueDrop;
    uint32_t queueDropData;
    uint32_t queueDropLog;
    uint32_t queueDropLifecycle;
    uint32_t lifecyclePublished;
    uint32_t lifecycleDropped;
    uint32_t transportSlotUsed;
    uint32_t transportSlotHighWater;
    uint32_t transportSlotAllocFail;
    uint32_t transportSlotReleaseMismatch;
    uint32_t transportStaleDescriptor;
    uint32_t blockCount;
} sensorarrayTransportStats_t;

typedef struct {
    uint32_t legacyItemBytes;
    uint32_t payloadSlotBytes;
    uint32_t descriptorBytes;
    uint32_t legacyQueueStorageBytes;
    uint32_t descriptorQueueStorageBytes;
    uint32_t payloadPoolBytes;
} sensorarrayTransportMemoryDiagnostics_t;

typedef struct {
    uint32_t transportConfiguredBytes;
    uint32_t transportMinimumRemainingBytes;
    uint32_t serialCtrlConfiguredBytes;
    uint32_t serialCtrlMinimumRemainingBytes;
} sensorarrayTransportTaskStackStats_t;

typedef esp_err_t (*sensorarrayTransportLegacyCommandCallback_t)(const uint8_t *data,
                                                                 size_t length,
                                                                 void *context);
typedef esp_err_t (*sensorarrayTransportRuntimeQueryCallback_t)(const char *command,
                                                                char *response,
                                                                size_t responseSize,
                                                                void *context);
typedef void (*sensorarrayTransportControlReplyPublishedCallback_t)(
    const char *data,
    size_t length);
typedef void (*sensorarrayTransportControlReplyFailedCallback_t)(
    esp_err_t error,
    const char *data,
    size_t length);

esp_err_t sensorarrayTransportInit(void);
esp_err_t sensorarrayTransportPublishData(const char *data, size_t length);
esp_err_t sensorarrayTransportPublishLog(const char *data, size_t length);
esp_err_t sensorarrayTransportPublishLifecycle(const char *data, size_t length);
esp_err_t sensorarrayTransportPublishControlReply(
    const sensorarrayTransportReplyTarget_t *target,
    const char *data,
    size_t length);
esp_err_t sensorarrayTransportHandleControlCommand(
    const uint8_t *data,
    size_t length,
    const sensorarrayTransportReplyTarget_t *replyTarget);
esp_err_t sensorarrayTransportApplyWifiMode(sensorarrayTransportWifiMode_t mode);
void sensorarrayTransportSetLegacyCommandCallback(
    sensorarrayTransportLegacyCommandCallback_t callback,
    void *context);
void sensorarrayTransportSetRuntimeQueryCallback(
    sensorarrayTransportRuntimeQueryCallback_t callback,
    void *context);
void sensorarrayTransportSetControlReplyPublishedCallback(
    sensorarrayTransportControlReplyPublishedCallback_t callback);
void sensorarrayTransportSetControlReplyFailedCallback(
    sensorarrayTransportControlReplyFailedCallback_t callback);
void sensorarrayTransportGetStats(sensorarrayTransportStats_t *outStats);
void sensorarrayTransportGetMemoryDiagnostics(
    sensorarrayTransportMemoryDiagnostics_t *outDiagnostics);
void sensorarrayTransportGetTaskStackStats(
    sensorarrayTransportTaskStackStats_t *outStats);
void sensorarrayTransportNoteSerialData(bool sent);
void sensorarrayTransportNoteSerialLog(bool sent);
void sensorarrayTransportSetTxMode(sensorarrayTransportTxMode_t mode);
sensorarrayTransportTxMode_t sensorarrayTransportGetTxMode(void);
void sensorarrayTransportSetStream(sensorarrayTransportStream_t stream);
sensorarrayTransportStream_t sensorarrayTransportGetStream(void);
void sensorarrayTransportSetWifiMode(sensorarrayTransportWifiMode_t mode);
sensorarrayTransportWifiMode_t sensorarrayTransportGetWifiMode(void);
bool sensorarrayTransportSerialSinkEnabled(void);
bool sensorarrayTransportBleSinkEnabled(void);
bool sensorarrayTransportBleChannelEnabled(sensorarrayTransportChannel_t channel);
bool sensorarrayTransportWifiSinkEnabled(void);
const char *sensorarrayTransportTxModeName(sensorarrayTransportTxMode_t mode);
const char *sensorarrayTransportStreamName(sensorarrayTransportStream_t stream);
const char *sensorarrayTransportWifiModeName(sensorarrayTransportWifiMode_t mode);
