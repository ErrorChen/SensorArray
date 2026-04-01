#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

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

/*
 * Logical destination selected by the SELA branch mux.
 * Enum values do not encode GPIO levels; the board-specific GPIO mapping lives
 * in main/sensorarrayBoardMap.c only.
 */
typedef enum {
    SENSORARRAY_SELA_ROUTE_ADS1263 = 0,
    SENSORARRAY_SELA_ROUTE_FDC2214 = 1,
} sensorarraySelaRoute_t;

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
    SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY,
    SENSORARRAY_DEBUG_MODE_FDC_I2C_DISCOVERY,
} sensorarrayDebugMode_t;

typedef struct {
    uint8_t sColumn;
    uint8_t dLine;
    sensorarrayPath_t path;
    sensorarraySelaRoute_t selaRoute; // Logical SELA target path, not a raw GPIO level.
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
    sensorarraySelaRoute_t selaRoute; // Logical SELA target path; GPIO level is derived centrally.
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
    bool configVerified;
    bool refClockKnown;
    Fdc2214CapRefClockSource_t refClockSource;
    uint32_t refClockHz;
    uint16_t statusConfigReg;
    uint16_t configReg;
    uint16_t muxConfigReg;
} sensorarrayFdcDeviceState_t;

typedef enum {
    SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR = 0,
    SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN,
    SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING,
    SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING,
    SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION,
    SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID,
    SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT,
    SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT,
    SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID,
} sensorarrayFdcSampleStatus_t;

typedef struct {
    esp_err_t err;
    Fdc2214CapSample_t sample;
    Fdc2214CapStatus_t status;
    Fdc2214CapCoreRegs_t coreRegs;
    bool i2cOk;
    bool idOk;
    bool configOk;
    bool converting;
    bool unreadConversionPresent;
    bool sampleValid;
    sensorarrayFdcSampleStatus_t statusCode;
} sensorarrayFdcReadDiag_t;

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
    bool configVerified;
    bool refClockKnown;
    Fdc2214CapRefClockSource_t refClockSource;
    uint32_t refClockHz;
    uint16_t statusConfigReg;
    uint16_t configReg;
    uint16_t muxConfigReg;
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
