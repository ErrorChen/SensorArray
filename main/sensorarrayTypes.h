#pragma once

#include <stdbool.h>
#include <stddef.h>
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
 * in core/board/sensorarrayBoardMap.c only.
 */
typedef enum {
    SENSORARRAY_SELA_ROUTE_ADS1263 = 0,
    SENSORARRAY_SELA_ROUTE_FDC2214 = 1,
} sensorarraySelaRoute_t;

typedef enum {
    SENSORARRAY_ROUTE_PATH_RESISTIVE = 0,
    SENSORARRAY_ROUTE_PATH_CAPACITIVE,
    SENSORARRAY_ROUTE_PATH_VOLTAGE,
} sensorarrayRoutePathKind_t;

#define SENSORARRAY_MATRIX_ROWS 8u
#define SENSORARRAY_MATRIX_COLS 8u
#define SENSORARRAY_MATRIX_CELL_COUNT (SENSORARRAY_MATRIX_ROWS * SENSORARRAY_MATRIX_COLS)

static inline size_t sensorarrayMatrixIndex(uint8_t sIndex, uint8_t dIndex)
{
    return ((size_t)(sIndex - 1U) * SENSORARRAY_MATRIX_COLS) + (size_t)(dIndex - 1U);
}

static inline bool sensorarrayMatrixIndexIsValid(uint8_t sIndex, uint8_t dIndex)
{
    return sIndex >= 1U && sIndex <= SENSORARRAY_MATRIX_ROWS &&
           dIndex >= 1U && dIndex <= SENSORARRAY_MATRIX_COLS &&
           sensorarrayMatrixIndex(sIndex, dIndex) < SENSORARRAY_MATRIX_CELL_COUNT;
}

typedef enum {
    SENSORARRAY_MATRIX_D_SOURCE_GND = 0,
    SENSORARRAY_MATRIX_D_SOURCE_REF,
} sensorarrayMatrixDSourcePolicy_t;

typedef enum {
    SENSORARRAY_ADS_INTREF_OFF = 0,
    SENSORARRAY_ADS_INTREF_ON,
    SENSORARRAY_ADS_INTREF_KEEP,
} sensorarrayAdsIntRefPolicy_t;

typedef enum {
    SENSORARRAY_ADS_VBIAS_OFF = 0,
    SENSORARRAY_ADS_VBIAS_ON,
    SENSORARRAY_ADS_VBIAS_KEEP,
} sensorarrayAdsVbiasPolicy_t;

typedef enum {
    SENSORARRAY_SW_PHYSICAL_LOW = 0,
    SENSORARRAY_SW_PHYSICAL_HIGH = 1,
} sensorarraySwPhysicalLevel_t;

typedef struct {
    uint64_t timestampUs;
    uint32_t sequence;
    double freqHz[SENSORARRAY_MATRIX_CELL_COUNT];
    double capTotalPf[SENSORARRAY_MATRIX_CELL_COUNT];
    uint32_t raw28[SENSORARRAY_MATRIX_CELL_COUNT];
    uint16_t clockDividers[SENSORARRAY_MATRIX_CELL_COUNT];
    uint16_t driveCurrent[SENSORARRAY_MATRIX_CELL_COUNT];
    uint8_t deglitchCode[SENSORARRAY_MATRIX_CELL_COUNT];
    uint32_t effectiveFclkHz[SENSORARRAY_MATRIX_CELL_COUNT];
    uint64_t validMask;
    uint64_t capValidMask;
    uint64_t warnMask;
    uint64_t errorMask;
} sensorarrayFdcMatrixFrame_t;

typedef struct {
    uint64_t frameUs;
    uint64_t rowAvgUs;
    uint64_t rowMinUs;
    uint64_t rowMaxUs;
    uint8_t slowRow;

    uint64_t pathEnsureUs;
    uint64_t cacheApplyUs;

    uint64_t applyBuildConfigUs;
    uint64_t applyChannelConfigWriteUs;
    uint64_t applyGlobalConfigWriteUs;
    uint64_t applyVerifyUs;
    uint64_t applyDelayUs;
    uint64_t applyReadyWaitUs;
    uint64_t applyMutexWaitUs;
    uint64_t applyLogUs;

    uint64_t discardUs;
    uint64_t waitReadyUs;
    uint64_t readUs;
    uint64_t emitUs;
    uint64_t capComputeUs;

    uint64_t sweepUs;
    uint32_t runtimeSweepCount;

    uint32_t i2cWriteCount;
    uint32_t i2cReadCount;
    uint32_t i2cVerifyReadCount;
    uint32_t i2cRetryCount;
    uint32_t i2cNackCount;
    uint32_t i2cTimeoutCount;
    uint32_t i2cRecoveryCount;

    uint32_t i2cBus0WriteCount;
    uint32_t i2cBus0ReadCount;
    uint32_t i2cBus0WriteBytes;
    uint32_t i2cBus0ReadBytes;
    uint64_t i2cBus0TotalUs;
    uint32_t i2cBus0RetryCount;
    uint32_t i2cBus0NackCount;
    uint32_t i2cBus0TimeoutCount;

    uint32_t i2cBus1WriteCount;
    uint32_t i2cBus1ReadCount;
    uint32_t i2cBus1WriteBytes;
    uint32_t i2cBus1ReadBytes;
    uint64_t i2cBus1TotalUs;
    uint32_t i2cBus1RetryCount;
    uint32_t i2cBus1NackCount;
    uint32_t i2cBus1TimeoutCount;

    uint32_t i2cFreqHz;
    uint64_t i2cEstimatedBits;
    uint64_t i2cEstimatedBusUs;
    uint64_t i2cMeasuredUs;
    int64_t i2cOverheadUs;
} sensorarrayFdcTimingSummary_t;

typedef struct {
    uint8_t row;
    uint64_t rowUs;
    uint64_t rowSelectUs;
    uint64_t analogSettleUs;

    uint64_t primaryTotalUs;
    uint64_t secondaryTotalUs;
    uint64_t parallelJoinWaitUs;

    uint64_t discardUs;
    uint64_t waitReadyUs;
    uint64_t readUs;

    uint8_t rowValidMask;
    uint8_t rowWarnMask;
    uint8_t rowErrorMask;
} sensorarrayFdcRowTiming_t;

typedef struct {
    uint8_t row;
    sensorarrayFdcDeviceId_t deviceId;

    uint64_t deviceUs;
    uint64_t applyUs;
    uint64_t applyBuildConfigUs;
    uint64_t channelConfigWriteUs;
    uint64_t globalConfigWriteUs;
    uint64_t verifyUs;
    uint64_t discardUs;
    uint64_t waitReadyUs;
    uint64_t readRawUs;

    uint32_t readyPollCount;
    uint32_t regWriteCount;
    uint32_t regReadCount;
    uint32_t verifyReadCount;
    uint32_t retryCount;
    uint32_t timeoutCount;
    uint32_t nackCount;
} sensorarrayFdcDeviceTiming_t;

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
    uint8_t dLine;
    sensorarrayFdcDeviceId_t devId;
    Fdc2214CapChannel_t channel;
    const char *mapLabel;
} sensorarrayFdcDLineMap_t;

typedef struct {
    uint8_t sColumn;
    uint8_t dLine;
    uint8_t matrixIndex;
    sensorarrayFdcDeviceId_t devId;
    uint8_t fdcChannel;
    const char *mapLabel;
} sensorarrayFdcCellTarget_t;

typedef enum {
    SENSORARRAY_FDC_CACHE_SOURCE_NONE = 0,
    SENSORARRAY_FDC_CACHE_SOURCE_BOOT_FULL,
    SENSORARRAY_FDC_CACHE_SOURCE_MANUAL_FULL,
    SENSORARRAY_FDC_CACHE_SOURCE_FAST_RESCUE,
    SENSORARRAY_FDC_CACHE_SOURCE_LAST_GOOD,
} sensorarrayFdcCacheSource_t;

typedef struct {
    bool valid;
    sensorarrayFdcCacheSource_t source;
    uint16_t rCount;
    uint16_t settleCount;
    uint16_t clockDiv;
    uint16_t driveCurrent;
    uint8_t deglitchCode;
    bool highCurrentObserved;
    uint32_t effectiveFclkHz;

    uint32_t lastRaw28;
    double lastFreqHz;

    uint32_t qualityScore;
    uint32_t generation;

    int64_t storedTimestampUs;
    int64_t lastAppliedTimestampUs;
    int64_t lastGoodTimestampUs;
    int64_t lastWarningTimestampUs;
    int64_t lastRescueTimestampUs;

    uint16_t consecutiveAmplitudeWarnings;
    uint16_t consecutiveErrors;
    uint16_t consecutiveNoUnread;
    uint16_t consecutiveZeroRaw;
    uint16_t consecutiveWatchdogFaults;
    uint16_t consecutiveI2cErrors;

    bool reapplyPending;
    bool rescuePending;
    char lastWarningReason[32];
    char lastRescueReason[32];
    uint8_t fastRescueFailCount;
} sensorarrayFdcCellConfigCache_t;

typedef struct {
    bool valid;
    bool autoscanConfigured;

    uint8_t row;
    uint8_t deviceId;

    uint8_t selectedDeglitch;
    uint16_t rCount[4];
    uint16_t settleCount[4];
    uint16_t clockDiv[4];
    uint16_t driveCurrent[4];

    uint32_t cacheGeneration[4];

    uint32_t applyCount;
    int64_t lastAppliedTimestampUs;

    bool dirty;
} sensorarrayFdcAppliedRowConfig_t;

typedef struct {
    bool valid;
    uint16_t selectedDriveCurrent;
    bool selectedHighCurrent;
    uint8_t selectedDeglitchCode;
    uint32_t selectedDeglitchBandwidthHz;
    uint16_t selectedClockDividers;
    uint32_t lastRaw28;
    double lastFrequencyHz;
    uint64_t lastValidTimestampUs;
    uint32_t consecutiveInvalid;
    uint32_t consecutiveAmplitudeFault;
    uint32_t consecutiveWatchdogFault;
    uint32_t consecutiveSaturated;
    uint32_t consecutiveZeroRaw;
    bool quickSweepPending;
    const char *quickSweepReason;
} sensorarrayFdcSweepProfile_t;

typedef struct {
    uint8_t sIndex;
    uint8_t dIndex;
    uint8_t fdcDevice;
    uint8_t fdcChannel;

    bool hasBootSweep;
    bool hasLastGood;
    bool lockValid;

    uint16_t bootDriveCurrent;
    Fdc2214CapDeglitch_t bootDeglitch;
    uint32_t bootRaw28;
    double bootFreqHz;
    double bootCapPf;
    int bootQualityScore;

    uint16_t lastGoodDriveCurrent;
    Fdc2214CapDeglitch_t lastGoodDeglitch;
    bool lastGoodHighCurrent;

    uint32_t lastGoodRaw28;
    double lastGoodFreqHz;
    uint64_t lastGoodTimestampUs;
    double lastGoodCapPf;
    int lastGoodQualityScore;

    uint16_t consecutiveFailCount;
    uint16_t consecutiveNoUnreadCount;
    uint16_t consecutiveStatusFaultCount;
    uint16_t consecutiveZeroRawCount;

    uint64_t lastFastSweepTimestampUs;
    uint64_t lastFullSweepTimestampUs;

    uint32_t lastSweepMs;
    uint8_t directFailCount;
    uint8_t fastSweepFailCount;
    uint8_t fullSweepFailCount;
    const char *lastFailReason;
} sensorarrayFdcCellCalibration_t;

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
    sensorarrayFdcSweepProfile_t sweepProfile[4];
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
    bool provisionalReadable;
    bool qualityDegraded;
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
    sensorarrayFdcCellConfigCache_t fdcCellCache[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    sensorarrayFdcAppliedRowConfig_t fdcAppliedRow[2];

    bool boardReady;
    bool tmuxReady;
} sensorarrayState_t;
