#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SENSORARRAY_WIFI_CH_DATA = 0,
    SENSORARRAY_WIFI_CH_LOG = 1,
    SENSORARRAY_WIFI_CH_CTRL = 2,
} sensorarrayWifiChannel_t;

typedef enum {
    SENSORARRAY_WIFI_PROFILE_HIGH = 0,
    SENSORARRAY_WIFI_PROFILE_BLE_COMPAT = 1,
} sensorarrayWifiProfile_t;

typedef struct {
    uint32_t ipv4;
    uint16_t port;
} sensorarrayWifiPeer_t;

typedef void (*sensorarrayWifiControlRxCallback_t)(const uint8_t *data,
                                                   size_t length,
                                                   const sensorarrayWifiPeer_t *peer,
                                                   void *userContext);

typedef struct {
    char ssid[32];
    char password[64];
    sensorarrayWifiProfile_t profile;
    uint16_t dataPort;
    uint16_t logPort;
    uint16_t ctrlPort;
} sensorarrayWifiConfig_t;

typedef struct {
    bool ready;
    bool stationConnected;
    sensorarrayWifiProfile_t profile;
    uint32_t sent[3];
    uint32_t dropped[3];
    uint32_t ctrlRx;
    uint32_t ctrlTx;
    esp_err_t initError;
} sensorarrayWifiStats_t;

esp_err_t sensorarrayWifiInit(const sensorarrayWifiConfig_t *config);
esp_err_t sensorarrayWifiSend(sensorarrayWifiChannel_t channel,
                              const uint8_t *data,
                              size_t length);
esp_err_t sensorarrayWifiSendControlReply(const sensorarrayWifiPeer_t *peer,
                                          const uint8_t *data,
                                          size_t length);
void sensorarrayWifiSetControlRxCallback(sensorarrayWifiControlRxCallback_t callback,
                                        void *userContext);
void sensorarrayWifiGetStats(sensorarrayWifiStats_t *outStats);
bool sensorarrayWifiIsReady(void);
