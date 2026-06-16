#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORARRAY_BLE_CH_DATA = 0,
    SENSORARRAY_BLE_CH_LOG = 1,
    SENSORARRAY_BLE_CH_CTRL = 2,
} sensorarrayBleChannel_t;

typedef enum {
    SENSORARRAY_BLE_TX_FAST = 0,
    SENSORARRAY_BLE_TX_SAFE,
} sensorarrayBleTxMode_t;

typedef void (*sensorarrayBleControlRxCallback_t)(const uint8_t *data,
                                                  size_t length,
                                                  void *userContext);

typedef struct {
    char deviceName[32];
    bool enableBle5;
    bool preferPhy2m;
    bool preferHighTxPower;
    uint16_t preferredMtu;
} sensorarrayBleConfig_t;

typedef struct {
    bool controllerReady;
    bool hostReady;
    bool gattReady;
    bool advertising;
    bool connected;
    bool congested;
    uint16_t mtu;
    uint8_t txPhy;
    uint8_t rxPhy;
    uint32_t sent[3];
    uint32_t dropped[3];
    uint32_t messageQueued;
    uint32_t messageSent;
    uint32_t messageDropped;
    uint32_t fragmentSent;
    uint32_t fragmentError;
    uint32_t tinyTailCount;
    uint32_t congestedCount;
    esp_err_t initError;
} sensorarrayBleStats_t;

esp_err_t sensorarrayBleInit(const sensorarrayBleConfig_t *config);
esp_err_t sensorarrayBleNotify(sensorarrayBleChannel_t channel,
                              const uint8_t *data,
                              size_t length);
void sensorarrayBleSetControlRxCallback(sensorarrayBleControlRxCallback_t callback,
                                       void *userContext);
bool sensorarrayBleIsReady(void);
bool sensorarrayBleIsConnected(void);
bool sensorarrayBleIsSubscribed(sensorarrayBleChannel_t channel);
bool sensorarrayBleIsCongested(void);
void sensorarrayBleSetTxMode(sensorarrayBleTxMode_t mode);
sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void);
const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t mode);
void sensorarrayBleGetStats(sensorarrayBleStats_t *outStats);
void sensorarrayBleLogHeap(const char *stage);

#ifdef __cplusplus
}
#endif
