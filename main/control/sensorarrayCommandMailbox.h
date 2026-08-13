#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensorarrayMeasurementMode.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORARRAY_COMMAND_BLE_CAP_PERIOD = 0,
    SENSORARRAY_COMMAND_TRACE_ENABLE,
    SENSORARRAY_COMMAND_CALIBRATE_ZERO,
    SENSORARRAY_COMMAND_CALIBRATE_RAIL,
    SENSORARRAY_COMMAND_CALIBRATE_ALL,
    SENSORARRAY_COMMAND_ADS_GAP_MODE,
    SENSORARRAY_COMMAND_CAPTURE_FPS_CAP,
    SENSORARRAY_COMMAND_OUTPUT_FPS_CAP,
    SENSORARRAY_COMMAND_MEASUREMENT_MODE,
    SENSORARRAY_COMMAND_SET_RAIL_CALIBRATION,
    SENSORARRAY_COMMAND_ADS_CHECK,
    SENSORARRAY_COMMAND_BATTERY_NOW,
    SENSORARRAY_COMMAND_BATTERY_DIAGNOSTIC,
    SENSORARRAY_COMMAND_BATTERY_PERIOD,
    SENSORARRAY_COMMAND_RES_SETTLE,
    SENSORARRAY_COMMAND_ADS_DEBUG,
    SENSORARRAY_COMMAND_ROW_MODES,
} sensorarrayCommandType_t;

typedef struct {
    sensorarrayCommandType_t type;
    uint32_t value;
    uint32_t requestId;
    int32_t signedValue;
    int32_t signedValue2;
    char rowModes[9];
} sensorarrayCommand_t;

esp_err_t sensorarrayCommandMailboxInit(void);
esp_err_t sensorarrayCommandMailboxPostText(const uint8_t *text, size_t length);
esp_err_t sensorarrayCommandMailboxPostMeasurementMode(
    sensorarrayMeasurementMode_t mode,
    uint32_t *outRequestId);
esp_err_t sensorarrayCommandMailboxPostRowModes(const char *profile,
                                                uint32_t *outRequestId);
esp_err_t sensorarrayCommandMailboxPostRailCalibration(
    int32_t avddUv,
    int32_t avssUv,
    uint32_t *outRequestId);
esp_err_t sensorarrayCommandMailboxPostAdsCheck(uint32_t sampleCount,
                                                uint32_t *outRequestId);
esp_err_t sensorarrayCommandMailboxPostBatteryNow(bool diagnostic,
                                                  uint32_t *outRequestId);
esp_err_t sensorarrayCommandMailboxPostBatteryPeriod(bool enabled,
                                                     uint32_t periodMs,
                                                     uint32_t *outRequestId);
esp_err_t sensorarrayCommandMailboxPostResSettle(uint32_t settleUs,
                                                 uint32_t *outRequestId);
bool sensorarrayCommandMailboxTryReceive(sensorarrayCommand_t *outCommand);
void sensorarrayCommandMailboxCommit(const sensorarrayCommand_t *command);
uint32_t sensorarrayCommandMailboxGetBleCapPeriod(void);
bool sensorarrayCommandMailboxTraceEnabled(void);
bool sensorarrayCommandMailboxAdsDebugEnabled(void);
uint32_t sensorarrayCommandMailboxGetCaptureFpsCap(void);
uint32_t sensorarrayCommandMailboxGetOutputFpsCap(void);
const char *sensorarrayCommandMailboxTypeName(sensorarrayCommandType_t type);

#ifdef __cplusplus
}
#endif
