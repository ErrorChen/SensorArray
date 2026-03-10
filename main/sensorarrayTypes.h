#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "ads126xAdc.h"
#include "boardSupport.h"
#include "fdc2214Cap.h"
#include "tmuxSwitch.h"

typedef enum {
    SENSORARRAY_FDC_DEV_PRIMARY = 0,
    SENSORARRAY_FDC_DEV_SECONDARY = 1,
} sensorarrayFdcDeviceId_t;

typedef enum {
    SENSORARRAY_PATH_RESISTIVE = 0,
    SENSORARRAY_PATH_CAPACITIVE = 1,
} sensorarrayPath_t;

typedef enum {
    SENSORARRAY_DEBUG_PATH_RESISTIVE = 0,
    SENSORARRAY_DEBUG_PATH_CAPACITIVE,
    SENSORARRAY_DEBUG_PATH_VOLTAGE,
} sensorarrayDebugPath_t;

typedef enum {
    SENSORARRAY_DEBUG_MODE_ROUTE_IDLE = 0,
    SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE,
    SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE,
    SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP,
    SENSORARRAY_DEBUG_MODE_ADS_SELFTEST,
    SENSORARRAY_DEBUG_MODE_FDC_SELFTEST,
    SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR,
} sensorarrayDebugMode_t;

typedef struct {
    uint8_t sColumn;
    uint8_t dLine;
    sensorarrayPath_t path;
    bool selALevel;
    bool selBLevel;
    const char *mapLabel;
} sensorarrayRouteMap_t;

typedef struct {
    bool stopBeforeMuxChange;
    uint32_t settleAfterMuxMs;
    bool startEveryRead;
    uint8_t baseDiscardCount;
    uint8_t readRetryCount;
} sensorarrayAdsReadPolicy_t;

typedef struct {
    uint8_t sColumn;
    uint8_t dLine;
    sensorarrayDebugPath_t path;
    tmux1108Source_t swSource;
    bool selALevel;
    bool selBLevel;
    bool skipAdsRead;
    bool skipFdcRead;
    uint32_t delayAfterRowMs;
    uint32_t delayAfterSelAMs;
    uint32_t delayAfterSelBMs;
    uint32_t delayAfterSwMs;
    bool holdForever;
    uint32_t holdMs;
    const char *label;
} sensorarrayDebugFixedRoute_t;

typedef struct {
    uint8_t dLine;
    sensorarrayFdcDeviceId_t devId;
    Fdc2214CapChannel_t channel;
    const char *mapLabel;
} sensorarrayFdcDLineMap_t;

typedef struct {
    const char *label;
    const BoardSupportI2cCtx_t *i2cCtx;
    uint8_t i2cAddr;
    Fdc2214CapDevice_t *handle;
    bool ready;
    bool haveIds;
    uint16_t manufacturerId;
    uint16_t deviceId;
} sensorarrayFdcDeviceState_t;

typedef enum {
    SENSORARRAY_RES_CONVERT_OK = 0,
    SENSORARRAY_RES_CONVERT_SIGNED_INPUT,
    SENSORARRAY_RES_CONVERT_MODEL_INVALID,
} sensorarrayResConvertResult_t;

typedef struct {
    const char *status;
    bool haveIds;
    uint16_t manufacturerId;
    uint16_t deviceId;
    int32_t detail;
} sensorarrayFdcInitDiag_t;

typedef struct {
    uint8_t id;
    uint8_t power;
    uint8_t iface;
    uint8_t mode2;
    uint8_t inpmux;
    uint8_t refmux;
} sensorarrayAdsRegSnapshot_t;

typedef struct {
    spi_device_handle_t spiDevice;
    ads126xAdcHandle_t ads;
    bool adsReady;
    bool adsRefReady;
    bool adsAdc1Running;
    bool adsRefMuxValid;
    uint8_t adsRefMux;

    sensorarrayFdcDeviceState_t fdcPrimary;
    sensorarrayFdcDeviceState_t fdcSecondary;
    uint8_t fdcConfiguredChannels;

    bool boardReady;
    bool tmuxReady;
} sensorarrayState_t;
