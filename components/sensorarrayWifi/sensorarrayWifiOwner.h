#pragma once
#include "sensorarrayWifi.h"

/* 本轮新增 private boundary. Only the persistent owner invokes backend work.
 * RX is a separately isolated persistent reader and never borrows TX slots. */
esp_err_t sensorarrayWifiBackendInitialize(const sensorarrayWifiConfig_t *config);
esp_err_t sensorarrayWifiBackendCleanup(void);
esp_err_t sensorarrayWifiBackendApplyProfile(sensorarrayWifiProfile_t profile);
esp_err_t sensorarrayWifiBackendSend(const sensorarrayWifiPublication_t *publication,
    int *returnedBytes, int *sdkErrno);
esp_err_t sensorarrayWifiBackendStationQuery(bool *connected);
bool sensorarrayWifiBackendHasReader(void);

bool sensorarrayWifiOwnerBeginSdk(sensorarrayWifiPhase_t phase);
bool sensorarrayWifiOwnerEndSdk(esp_err_t error, int sdkErrno, int returnedBytes);
bool sensorarrayWifiOwnerMayRunSdk(void);
void sensorarrayWifiOwnerNoteStationEvent(bool connected);
void sensorarrayWifiOwnerReceive(const uint8_t *data, size_t length,
    const sensorarrayWifiPeer_t *peer);
bool sensorarrayWifiOwnerRxAllowed(void);
bool sensorarrayWifiOwnerProcessOnce(void);

