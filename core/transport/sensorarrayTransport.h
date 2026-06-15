#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensorarrayTransportChannels.h"
#include "sensorarrayWifi.h"

typedef enum {
    SENSORARRAY_TRANSPORT_REPLY_SERIAL = 0,
    SENSORARRAY_TRANSPORT_REPLY_BLE,
    SENSORARRAY_TRANSPORT_REPLY_WIFI,
} sensorarrayTransportReplyKind_t;

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
    uint32_t blockCount;
} sensorarrayTransportStats_t;

typedef esp_err_t (*sensorarrayTransportLegacyCommandCallback_t)(const uint8_t *data,
                                                                 size_t length,
                                                                 void *context);

esp_err_t sensorarrayTransportInit(void);
esp_err_t sensorarrayTransportPublishData(const char *data, size_t length);
esp_err_t sensorarrayTransportPublishLog(const char *data, size_t length);
esp_err_t sensorarrayTransportPublishControlReply(
    const sensorarrayTransportReplyTarget_t *target,
    const char *data,
    size_t length);
esp_err_t sensorarrayTransportHandleControlCommand(
    const uint8_t *data,
    size_t length,
    const sensorarrayTransportReplyTarget_t *replyTarget);
void sensorarrayTransportSetLegacyCommandCallback(
    sensorarrayTransportLegacyCommandCallback_t callback,
    void *context);
void sensorarrayTransportGetStats(sensorarrayTransportStats_t *outStats);
void sensorarrayTransportNoteSerialData(bool sent);
void sensorarrayTransportNoteSerialLog(bool sent);
