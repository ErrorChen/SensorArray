#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum complete DATA/LOG message accepted before ATT fragmentation. */
#define SENSORARRAY_BLE_MESSAGE_VALUE_MAX 1536u

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
    uint32_t controlRxQueued;
    uint32_t controlRxDropped;
    uint32_t controlRxStale;
    uint32_t txSlotUsed;
    uint32_t txSlotHighWater;
    uint32_t txSlotAllocFail;
    uint32_t txSlotReleaseMismatch;
    uint32_t txSlotStaleGenerationDrop;
    uint32_t txConnectionStaleDrop;
    uint32_t staleConfirmation;
    uint32_t txCrcMismatch;
    uint32_t controlTxRetry;
    uint32_t controlTxRetryExhausted;
    /* Configured and lifetime-minimum remaining stack sizes, all in bytes. */
    uint32_t txTaskConfiguredBytes;
    uint32_t txTaskMinimumRemainingBytes;
    uint32_t ctrlTaskConfiguredBytes;
    uint32_t ctrlTaskMinimumRemainingBytes;
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
bool sensorarrayBleCanSend(sensorarrayBleChannel_t channel);
/* Capture the active connection generation for a specific sendable channel.
 * A queued producer can later use NotifyForGeneration to guarantee that its
 * payload is never delivered to a different reconnecting client. */
bool sensorarrayBleGetSendGeneration(sensorarrayBleChannel_t channel,
                                     uint32_t *outConnectionGeneration);
esp_err_t sensorarrayBleNotifyForGeneration(
    sensorarrayBleChannel_t channel,
    const uint8_t *data,
    size_t length,
    uint32_t expectedConnectionGeneration);
bool sensorarrayBleIsCongested(void);
void sensorarrayBleSetTxMode(sensorarrayBleTxMode_t mode);
sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void);
const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t mode);
void sensorarrayBleGetStats(sensorarrayBleStats_t *outStats);
void sensorarrayBleLogHeap(const char *stage);

#ifdef __cplusplus
}
#endif
