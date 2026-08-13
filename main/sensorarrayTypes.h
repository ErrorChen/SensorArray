#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayScanConfig.h"
#include "ads126xAdc.h"
#include "boardSupport.h"
#include "fdc2214Cap.h"
#include "tmuxSwitch.h"
#include "sensorarrayMeasurementMode.h"

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

typedef struct {
    sensorarrayMatrixDSourcePolicy_t matrixRef;
    sensorarrayAdsIntRefPolicy_t intRef;
    sensorarrayAdsVbiasPolicy_t vbias;
} sensorarrayAdsRoutePowerPolicy_t;

typedef enum {
    SENSORARRAY_SW_PHYSICAL_LOW = 0,
    SENSORARRAY_SW_PHYSICAL_HIGH = 1,
} sensorarraySwPhysicalLevel_t;

typedef struct {
    uint64_t routeUs;
    uint64_t settleUs;
    uint64_t cacheCompareUs;
    uint64_t cacheApplyI2cUs;
    uint64_t readyWaitUs;
    uint64_t statusReadUs;
    uint64_t dataReadUs;
    uint64_t mergeUs;
    uint64_t primaryWorkerRunUs;
    uint64_t secondaryWorkerRunUs;
    uint64_t workerStartSkewUs;
    uint64_t workerDoneSkewUs;
    uint64_t workerNotifyUs;
    uint64_t rowSleepBarrierUs;
    uint64_t workerPreReleaseUs;
    uint64_t workerSleepAckWaitUs;
    uint64_t workerStartGiveUs;
    uint64_t workerWaitPrimaryUs;
    uint64_t workerWaitSecondaryUs;
    uint64_t parentWaitBothUs;
    uint64_t workerJoinUs;
    uint64_t frameMaskUpdateUs;
    uint64_t frameBookkeepingUs;
    uint64_t frameQueueUs;

    uint32_t i2cBus0ReadCount;
    uint32_t i2cBus1ReadCount;
    uint32_t i2cBus0WriteCount;
    uint32_t i2cBus1WriteCount;
    uint32_t i2cBus0ReadBytes;
    uint32_t i2cBus1ReadBytes;
    uint32_t i2cBus0WriteBytes;
    uint32_t i2cBus1WriteBytes;
    uint64_t i2cBus0TotalUs;
    uint64_t i2cBus1TotalUs;
    uint32_t i2cBus0TransactionCount;
    uint32_t i2cBus1TransactionCount;
    uint32_t i2cOrderedDataReadCount;
    uint32_t i2cBurstDataReadCount;
    uint32_t i2cBurstFallbackCount;
    uint32_t i2cSequenceDataReadCount;
    uint32_t i2cSequenceTransactionCount;
    uint32_t i2cSequenceFallbackCount;
    uint32_t i2cSequenceErrorCount;
    uint32_t i2cNackCount;
    uint32_t i2cTimeoutCount;
    uint32_t i2cRecoveryCount;
    uint8_t dataReadMode;
    uint8_t sequenceRegsPerTransaction;
    uint8_t sequenceTransactionsPerRow;

    uint32_t directDataReadCount;
    uint32_t directDataFallbackCount;
    uint32_t directDataFallbackReasonMask;
    uint32_t statusReadCount;
    uint32_t statusSavedReadCount;

    uint32_t cacheCompareCount;
    uint32_t cacheDiffRows;
    uint32_t cacheWriteCount;
    uint32_t cacheRestartRows;
    uint32_t cacheSkipRows;
} sensorarrayFdcFrameTelemetry_t;

typedef enum {
    SENSORARRAY_BATTERY_INVALID_NONE = 0,
    SENSORARRAY_BATTERY_INVALID_DISABLED,
    SENSORARRAY_BATTERY_INVALID_NOT_DUE,
    SENSORARRAY_BATTERY_INVALID_DEFERRED,
    SENSORARRAY_BATTERY_INVALID_CAL,
    SENSORARRAY_BATTERY_INVALID_RAIL,
    SENSORARRAY_BATTERY_INVALID_ZERO,
    SENSORARRAY_BATTERY_INVALID_ADC,
    SENSORARRAY_BATTERY_INVALID_ADC_TIMEOUT,
    SENSORARRAY_BATTERY_INVALID_ADC_STALE,
    SENSORARRAY_BATTERY_INVALID_ADC_STATUS_ERROR,
    SENSORARRAY_BATTERY_INVALID_SPI_ERROR,
    SENSORARRAY_BATTERY_INVALID_RESTORE_FAILED,
    SENSORARRAY_BATTERY_INVALID_VBIAS,
    SENSORARRAY_BATTERY_INVALID_DIV,
    SENSORARRAY_BATTERY_INVALID_NO_AINCOM_GND_REFERENCE,
    SENSORARRAY_BATTERY_INVALID_REFERENCE_INVALID,
    SENSORARRAY_BATTERY_INVALID_ABSENT_OR_OPEN,
    SENSORARRAY_BATTERY_INVALID_RANGE_ERROR,
    SENSORARRAY_BATTERY_INVALID_UNSTABLE,
    SENSORARRAY_BATTERY_INVALID_SATURATED,
    SENSORARRAY_BATTERY_INVALID_OUT_OF_RANGE,
    SENSORARRAY_BATTERY_INVALID_OVERFLOW,
    SENSORARRAY_BATTERY_INVALID_UNKNOWN,
} sensorarrayBatteryInvalidReason_t;

typedef enum {
    SENSORARRAY_ADS_RAIL_STATUS_BAD = 0,
    SENSORARRAY_ADS_RAIL_STATUS_OK,
    SENSORARRAY_ADS_RAIL_STATUS_HOLD,
} sensorarrayAdsRailStatus_t;

typedef struct {
    uint64_t timestampUs;
    uint64_t batteryTimestampUs;
    uint64_t batteryLastGoodTimestampUs;
    int32_t ain9OffsetRaw;
    int32_t ain9OffsetUv;
    int32_t ain8Raw;
    int32_t ain8RawUv; /* Deprecated alias for ain8DiffUv. */
    int32_t ain8DiffUv; /* Raw ADC result: AIN8 - AINCOM, in uV. */
    int32_t aincomGndUv;
    int32_t ain8GndUv;
    int32_t batteryMv;
    int32_t batteryLastGoodMv;
    int32_t railUv;
    int32_t railRawUv;
    int32_t railMonitorUv;
    int32_t railMonitorRaw;
    int32_t railLastGoodUv;
    int32_t railExpectedUv;
    int32_t railErrorUv;
    int32_t zeroResidualUv;
    uint32_t zeroResidualStdUv;
    uint32_t drdyGenerationDelta;
    uint32_t adcStaleCount;
    uint32_t adcStatusErrorCount;
    uint32_t railAgeFrames;
    uint16_t railValidStreak;
    uint16_t railInvalidStreak;
    uint32_t windows;
    uint32_t jobsRun;
    uint32_t jobsSkip;
    uint32_t overrunCount;
    uint32_t drdyTimeoutCount;
    uint32_t spiErrorCount;
    uint32_t dmaReadCount;
    uint64_t dmaReadUs;
    uint32_t pollReadCount;
    uint64_t pollReadUs;
    uint32_t guardUs;
    uint32_t avgSlackUs;
    uint32_t minSlackUs;
    uint32_t sampleAgeFrames;
    uint32_t zeroAgeFrames;
    uint32_t batteryAgeMs;
    uint32_t batteryLastGoodAgeMs;
    uint32_t batteryLastGoodFrame;
    uint32_t batteryPeriodMs;
    uint32_t batterySampleUs;
    uint32_t batterySampleUsAverage;
    uint32_t batterySampleUsMaximum;
    uint32_t batteryRunCount;
    uint32_t batterySkipCount;
    uint32_t batteryDeferCount;
    uint32_t batteryBoundaryCount;
    uint32_t batteryRestoreFailureCount;
    uint32_t batterySampleCount;
    uint32_t batteryShadowGeneration;
    uint32_t batterySpreadRaw;
    uint32_t batterySpreadRawMaximum;
    uint32_t batteryValidRunCount;
    uint32_t batteryInvalidRunCount;
    uint32_t batteryRetryCount;
    uint32_t batteryLastRetryCount;
    uint32_t batteryUnstableCount;
    uint32_t batteryTimeoutCount;
    uint8_t id;
    uint8_t devId;
    uint8_t revId;
    uint16_t chip;
    uint8_t activeAdc;
    uint8_t rateCode;
    uint8_t adcStatus;
    uint8_t railSource;
    bool initialized;
    bool dmaCapable;
    bool adcFresh;
    bool batteryValid;
    bool batteryLastGoodValid;
    bool batteryLastGoodFresh;
    bool batteryEnabled;
    bool batteryFresh;
    bool batteryDue;
    bool batteryBoundaryFallback;
    bool batteryRestoreOk;
    bool aincomGndValid;
    bool ain8GndValid;
    bool hasAdc2;
    bool bootCalibrationDone;
    bool vbiasEnabled;
    bool railValid;
    bool railUsableForBattery;
    bool zeroValid;
    bool vrefSynced;
    sensorarrayAdsRailStatus_t railStatus;
    sensorarrayBatteryInvalidReason_t batteryInvalidReason;
    bool fallbackToBoundary;
} sensorarrayAdsGapSnapshot_t;

typedef struct {
    uint64_t timestampUs;
    uint32_t sequence;
    uint32_t physicalSweepId;
    uint64_t frameStartUs;
    uint64_t frameEndUs;
    uint64_t emitUs;
    uint64_t physicalSweepUs;
    uint64_t rowStepUsAvg;
    uint64_t rowStepUsMax;
    sensorarrayFrameConfigSnapshot_t configSnapshot;
    uint8_t activeRows;

    uint8_t rowFreshMask;
    uint8_t primaryFreshMask;
    uint8_t secondaryFreshMask;
    uint32_t rowEpoch[SENSORARRAY_MATRIX_ROWS];
    uint32_t primaryEpoch[SENSORARRAY_MATRIX_ROWS];
    uint32_t secondaryEpoch[SENSORARRAY_MATRIX_ROWS];
    uint32_t primaryCacheFingerprint[SENSORARRAY_MATRIX_ROWS];
    uint32_t secondaryCacheFingerprint[SENSORARRAY_MATRIX_ROWS];
    uint64_t rowRouteSetUs[SENSORARRAY_MATRIX_ROWS];
    uint64_t rowReadyUs[SENSORARRAY_MATRIX_ROWS];
    uint64_t rowReadDoneUs[SENSORARRAY_MATRIX_ROWS];
    uint64_t rowMergeDoneUs[SENSORARRAY_MATRIX_ROWS];
    bool mixedEpoch;
    bool stale;
    bool freshFrame;
    sensorarrayFdcFrameTelemetry_t telemetry;
    sensorarrayAdsGapSnapshot_t adsGap;
    /* Mixed row output keeps unit/scale attached to the physical S row. */
    sensorarrayMeasurementMode_t rowMode[SENSORARRAY_MATRIX_ROWS];
    sensorarrayMeasurementUnit_t rowUnit[SENSORARRAY_MATRIX_ROWS];
    int8_t rowScale[SENSORARRAY_MATRIX_ROWS];
    uint32_t rowProfileGeneration;
    uint32_t rowProfileRequestId;
    bool mixedProfile;
    /* Generic payload is populated only for VOLT/RES. CAP keeps the legacy
     * FDC fields and byte-compatible C/D/K formatter as the authority. */
    sensorarrayMeasurementPayload_t measurement;

    uint32_t fdcTheoryReadyUs;
    uint32_t fdcTheoryFrameReadyUs;
    uint32_t fdcTheorySwitchDelayUs;
    uint32_t fdcFrefHz;
    uint16_t fdcRcount;
    uint16_t fdcSettleCount;
    uint16_t fdcClockDividers;
    uint16_t fdcDriveCurrent;
    uint16_t fdcConfig;
    uint16_t fdcMuxConfig;
    uint8_t fdcDeglitch;
    bool fdcSensorActivateFullCurrent;
    bool fdcHighCurrentDrive;

    double freqHz[SENSORARRAY_MATRIX_CELL_COUNT];
    double capTotalPf[SENSORARRAY_MATRIX_CELL_COUNT];
    uint32_t raw28[SENSORARRAY_MATRIX_CELL_COUNT];
    uint16_t clockDividers[SENSORARRAY_MATRIX_CELL_COUNT];
    uint16_t driveCurrent[SENSORARRAY_MATRIX_CELL_COUNT];
    uint8_t deglitchCode[SENSORARRAY_MATRIX_CELL_COUNT];
    uint32_t effectiveFclkHz[SENSORARRAY_MATRIX_CELL_COUNT];
    uint64_t validMask;
    uint64_t capValidMask;
    uint64_t freshMask;
    uint64_t warnMask;
    uint64_t errorMask;
    uint8_t hardwareZeroRawCount;
    uint8_t placeholderZeroCount;
    uint8_t notReadyCount;
    uint8_t zeroBeforeReadyCount;
    uint8_t zeroAfterDrdyCount;
    uint8_t i2cErrorCount;
    uint8_t unreadWithoutDrdyCount;
    uint8_t softInvalidCount;
    uint8_t hardInvalidCount;
    uint8_t staleUnreadDrainCount;
    uint8_t diagReadyButRejectedCount;
    uint8_t intbMissButStatusReadyCount;
    uint8_t statusFallbackAcceptedCount;
    uint8_t waitBudgetTooShortCount;
    uint8_t levelLowButEdgeMissCount;
    uint8_t actualDataReadSkippedDespiteStatusReadyCount;
    uint8_t validCount;
    uint8_t freshCount;
    esp_err_t firstReadErr;
    uint8_t firstBadRow;
    uint8_t firstBadDevice;
    uint16_t firstBadStatus;
    uint8_t firstBadUnread;
} sensorarrayFdcMatrixFrame_t;

typedef struct {
    uint64_t frameUs;
    uint64_t rowAvgUs;
    uint64_t rowMinUs;
    uint64_t rowMaxUs;
    uint8_t slowRow;

    uint64_t pathEnsureUs;
    uint64_t cacheApplyUs;
    uint64_t cacheCompareUs;
    uint64_t cacheApplyI2cUs;
    uint64_t cacheApplyI2cMaxUs;

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
    uint64_t coordinatorMergeUs;
    uint64_t emitUs;
    uint64_t capComputeUs;

    uint64_t sleepBeforeRowSwitchUs;
    uint64_t rowSwitchWhileSleepingUs;
    uint64_t rowSettleUs;
    uint64_t diffApplyWhileSleepingUs;
    uint64_t sleepTotalUs;
    uint64_t sleepExitToIntbUs;
    uint64_t statusReadUs;
    uint64_t statusPrecheckUs;
    uint64_t dataReadUs;
    uint64_t intbWaitOnlyUs;
    uint64_t statusVerifyAfterIntbUs;
    uint64_t pollFallbackUs;
    uint64_t staleDrainUs;
    uint64_t primaryJobUs;
    uint64_t secondaryJobUs;
    uint64_t dualBusWaitUs;
    uint64_t dualBusSkewUs;
    uint64_t workerQueueSendUs;
    uint64_t workerSleepAckWaitUs;
    uint64_t workerStartGiveUs;
    uint64_t workerDoneWaitUs;
    uint64_t workerPreReleaseUs;
    uint64_t workerWaitPrimaryUs;
    uint64_t workerWaitSecondaryUs;
    uint64_t workerJoinUs;
    uint64_t frameMaskUpdateUs;
    uint64_t frameBookkeepingUs;
    uint64_t frameQueueUs;
    uint64_t workerLateDoneUs;
    uint64_t workerIdleAfterTimeoutUs;
    uint64_t primaryWorkerRunUs;
    uint64_t secondaryWorkerRunUs;
    uint64_t primaryWorkerRunMaxUs;
    uint64_t secondaryWorkerRunMaxUs;
    uint64_t workerStartSkewUs;
    uint64_t workerStartSkewMaxUs;
    uint64_t workerDoneSkewUs;
    uint64_t workerDoneSkewMaxUs;
    uint64_t waitOverlapUs;
    uint64_t readOverlapUs;
    uint64_t waitSpanUs;
    uint64_t readSpanUs;
    uint64_t readStartDeltaUsTotal;
    uint64_t primaryFirstI2cStartUs;
    uint64_t secondaryFirstI2cStartUs;
    int64_t primaryMinusSecondaryStartUs;
    int64_t primaryMinusSecondaryDoneUs;
    uint32_t wpOkRowCount;
    uint32_t wpAnomRowCount;
    uint32_t frNormalRowCount;
    uint32_t frAnomRowCount;
    uint32_t frVmFullCount;
    uint32_t frWarnNonzeroCount;
    uint32_t frErrorNonzeroCount;
    uint32_t frCacheMissNonzeroCount;
    uint32_t frTimeoutNonzeroCount;
    uint32_t frPartialCount;
    uint64_t serialFallbackUs;
    uint64_t repairPrimaryUs;
    uint64_t repairSecondaryUs;
    uint64_t outputFormatUs;
    uint64_t outputPrintfUs;
    uint64_t measureLockHeldUs;

    uint32_t cacheApplySkippedCount;
    uint32_t cacheApplyDirtyDeviceCount;
    uint32_t cacheApplyDiffWriteCount;
    uint32_t cacheApplyFullWriteCount;
    uint32_t cacheApplyNoDiffCount;
    uint32_t diffRcountWrites;
    uint32_t diffSettleWrites;
    uint32_t diffClockDivWrites;
    uint32_t diffDriveWrites;
    uint32_t diffMuxWrites;
    uint32_t diffStatusConfigWrites;
    uint32_t diffConfigWrites;
    uint32_t appliedFingerprintChanges;
    uint32_t cacheApplyRestartCount;

    uint32_t intbEdgeCountPrimary;
    uint32_t intbEdgeCountSecondary;
    uint32_t intbFalseEdgeCount;
    uint32_t intbTimeoutCount;
    uint32_t intbFallbackPollCount;
    uint32_t intbFreshDrdyCount;
    uint32_t preStatusReadyCount;
    uint32_t intbReadyCount;
    uint32_t lateStatusReadyCount;
    uint32_t trueTimeoutCount;
    uint32_t intbStaleBeforeClearCount;
    uint32_t readyFullCount;
    uint32_t recoveredAfterRetryCount;
    uint32_t readyPartialCount;
    uint32_t readyNoneCount;
    uint32_t transientUnreadNoDrdyCount;
    uint32_t staleUnreadDrainCount;
    uint32_t hardReadyTimeoutCount;
    uint32_t statusReadsBeforeIntbCount;
    uint32_t statusReadsPrecheckCount;
    uint32_t statusReadsAfterIntbCount;
    uint32_t statusReadsInFallbackCount;
    uint32_t statusReadSuppressedBeforeIntbCount;
    uint32_t noStatusPollWaitCount;
    uint32_t statusAfterIntbCount;
    uint32_t statusAfterTimeoutCount;
    uint32_t hardTimeoutStatusDiagCount;
    uint32_t intbActiveStatusMismatchCount;
    uint32_t suppressedRpCount;
    uint32_t internalWaitStateLeakCount;
    uint32_t unsafeUnreadNoDrdyCount;
    uint32_t drdyPartialUnreadCount;
    uint32_t drdyFullUnreadReadyCount;
    uint32_t statusReadErrCount;
    uint32_t statusReadCountPrimary;
    uint32_t statusReadCountSecondary;
    uint32_t deferredRepairRequestCount;
    uint32_t inlineRepairSuppressedCount;
    uint32_t unreadWithoutDrdyCount;
    uint32_t dataNotReadyCount;
    uint32_t softInvalidCount;
    uint32_t hardInvalidCount;
    uint32_t drainCount;
    uint32_t diagUnreadLikelyFreshCount;
    uint32_t diagUnreadLikelyStaleCount;
    uint32_t zeroBeforeReadyCount;
    uint32_t zeroAfterDrdyCount;
    uint32_t fallbackAttemptCount;
    uint32_t fallbackSuccessCount;
    uint32_t fallbackPartialCount;
    uint32_t fallbackFailCount;
    uint64_t fallbackSecondWaitUs;
    uint64_t fallbackSecondWaitMaxUs;
    uint32_t fallbackSecondWaitCount;
    uint32_t diagReadyButRejectedCount;
    uint32_t intbMissButStatusReadyCount;
    uint32_t statusFallbackAcceptedCount;
    uint32_t waitBudgetTooShortCount;
    uint32_t levelLowButEdgeMissCount;
    uint32_t actualDataReadSkippedDespiteStatusReadyCount;
    uint32_t directDataReadCount;
    uint32_t directDataFallbackCount;
    uint32_t directDataFallbackReasonMask;
    uint32_t statusSavedReadCount;
    uint32_t rowFullInvalidCount;
    uint32_t deviceFullInvalidCount;
    uint32_t workerTimeoutCount;
    uint32_t workerLateDoneCount;
    uint32_t workerLateGoodAcceptedCount;
    uint32_t staleResultDiscardedCount;
    uint32_t duplicateReadCount;
    uint64_t waitReadyUsPrimaryTotal;
    uint64_t waitReadyUsSecondaryTotal;
    uint64_t read4UsPrimaryTotal;
    uint64_t read4UsSecondaryTotal;
    uint64_t maxWaitReadyUs;
    uint64_t maxWaitReadyUsPrimary;
    uint64_t maxWaitReadyUsSecondary;
    uint64_t statusReadUsPrimaryTotal;
    uint64_t statusReadUsSecondaryTotal;
    uint64_t dataReadUsPrimaryTotal;
    uint64_t dataReadUsSecondaryTotal;
    uint32_t alreadyLowPrimaryCount;
    uint32_t alreadyLowSecondaryCount;
    uint64_t maxI2cReadUs;
    uint32_t sweepRequestCount;
    uint32_t sweepActuallyQueuedCount;

    uint32_t freshAmplitudeWarningCount;
    uint32_t staleAmplitudeWarningCount;
    uint32_t transientAmplitudeWarningCount;
    uint32_t warningReapplySuppressedCount;
    uint32_t warningFastSweepRequestedCount;
    uint32_t warningFastSweepSuppressedCooldownCount;
    uint32_t degradedCellCount;

    uint64_t sweepUs;
    uint32_t runtimeSweepCount;

    uint32_t i2cWriteCount;
    uint32_t i2cReadCount;
    uint32_t i2cVerifyReadCount;
    uint32_t i2cRetryCount;
    uint32_t i2cNackCount;
    uint32_t i2cTimeoutCount;
    uint32_t i2cRecoveryCount;
    uint32_t i2cOrderedDataReadCount;
    uint32_t i2cBurstDataReadCount;
    uint32_t i2cBurstProbeReadCount;
    uint32_t i2cBurstFallbackCount;
    uint32_t i2cSequenceDataReadCount;
    uint32_t i2cSequenceTransactionCount;
    uint32_t i2cSequenceFallbackCount;
    uint32_t i2cSequenceErrorCount;

    uint32_t i2cBus0WriteCount;
    uint32_t i2cBus0ReadCount;
    uint32_t i2cBus0WriteBytes;
    uint32_t i2cBus0ReadBytes;
    uint64_t i2cBus0TotalUs;
    uint32_t i2cBus0RetryCount;
    uint32_t i2cBus0NackCount;
    uint32_t i2cBus0TimeoutCount;
    uint64_t i2cBus0BusyWaitUs;
    uint32_t i2cBus0TransactionCount;

    uint32_t i2cBus1WriteCount;
    uint32_t i2cBus1ReadCount;
    uint32_t i2cBus1WriteBytes;
    uint32_t i2cBus1ReadBytes;
    uint64_t i2cBus1TotalUs;
    uint32_t i2cBus1RetryCount;
    uint32_t i2cBus1NackCount;
    uint32_t i2cBus1TimeoutCount;
    uint64_t i2cBus1BusyWaitUs;
    uint32_t i2cBus1TransactionCount;
    uint64_t i2cGlobalLockWaitUs;
    uint32_t i2cCrossBusSerializedCount;

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
    uint64_t routeSetTimestampUs;

    uint64_t primaryTotalUs;
    uint64_t secondaryTotalUs;
    uint64_t parallelJoinWaitUs;

    uint64_t discardUs;
    uint64_t waitReadyUs;
    uint64_t readUs;
    uint64_t coordinatorMergeUs;

    uint64_t sleepBeforeRowSwitchUs;
    uint64_t rowSwitchWhileSleepingUs;
    uint64_t rowSettleUs;
    uint64_t diffApplyWhileSleepingUs;
    uint64_t sleepTotalUs;
    uint64_t sleepExitToIntbUs;
    uint64_t primaryJobUs;
    uint64_t secondaryJobUs;
    uint64_t dualBusWaitUs;
    uint64_t dualBusSkewUs;
    uint64_t workerQueueSendUs;
    uint64_t workerSleepAckWaitUs;
    uint64_t workerStartGiveUs;
    uint64_t workerDoneWaitUs;
    uint64_t workerPreReleaseUs;
    uint64_t workerWaitPrimaryUs;
    uint64_t workerWaitSecondaryUs;
    uint64_t workerJoinUs;
    uint64_t frameMaskUpdateUs;
    uint64_t frameBookkeepingUs;
    uint64_t workerLateDoneUs;
    uint64_t workerIdleAfterTimeoutUs;
    uint64_t primaryWorkerRunUs;
    uint64_t secondaryWorkerRunUs;
    uint64_t workerStartSkewUs;
    uint64_t workerDoneSkewUs;
    uint64_t primaryFirstI2cStartUs;
    uint64_t secondaryFirstI2cStartUs;
    int32_t primaryMinusSecondaryStartUs;
    int32_t primaryMinusSecondaryDoneUs;
    bool wpNormal;
    uint64_t serialFallbackUs;
    uint64_t repairPrimaryUs;
    uint64_t repairSecondaryUs;
    uint64_t workerReleaseUs;
    uint64_t primarySleepExitStartUs;
    uint64_t primarySleepExitDoneUs;
    uint64_t secondarySleepExitStartUs;
    uint64_t secondarySleepExitDoneUs;
    uint64_t primaryReadyBeginUs;
    uint64_t secondaryReadyBeginUs;
    uint64_t primaryIntbSeenUs;
    uint64_t secondaryIntbSeenUs;
    uint64_t primaryStatusVerifyStartUs;
    uint64_t secondaryStatusVerifyStartUs;
    uint64_t primaryDrdyUs;
    uint64_t secondaryDrdyUs;
    uint64_t primaryReadStartUs;
    uint64_t secondaryReadStartUs;
    uint64_t primaryReadDoneUs;
    uint64_t secondaryReadDoneUs;
    uint64_t parallelSpanUs;
    uint64_t parallelSerialEquivalentUs;
    uint32_t parallelEfficiencyPct;
    uint32_t sleepExitStartDeltaUs;
    uint32_t readyBeginDeltaUs;
    uint32_t intbSeenDeltaUs;
    uint32_t statusVerifyStartDeltaUs;
    uint32_t drdyDeltaUs;
    uint32_t readStartDeltaUs;
    uint32_t workerTimeoutCount;
    uint32_t workerLateDoneCount;
    uint32_t workerLateGoodAcceptedCount;
    uint32_t staleResultDiscardedCount;
    uint32_t duplicateReadCount;
    uint32_t deferredRepairRequestCount;
    uint32_t inlineRepairSuppressedCount;

    uint8_t rowValidMask;
    uint8_t rowWarnMask;
    uint8_t rowErrorMask;
} sensorarrayFdcRowTiming_t;

typedef struct {
    uint8_t row;
    sensorarrayFdcDeviceId_t deviceId;

    uint64_t deviceUs;
    uint64_t applyUs;
    uint64_t cacheCompareUs;
    uint64_t cacheApplyI2cUs;
    uint64_t applyBuildConfigUs;
    uint64_t channelConfigWriteUs;
    uint64_t globalConfigWriteUs;
    uint64_t verifyUs;
    uint64_t discardUs;
    uint64_t waitReadyUs;
    uint64_t readRawUs;
    uint64_t sleepEnterUs;
    uint64_t sleepExitUs;
    uint64_t sleepExitToIntbUs;
    uint64_t statusReadUs;
    uint64_t statusPrecheckUs;
    uint64_t dataReadUs;
    uint64_t intbWaitOnlyUs;
    uint64_t statusVerifyAfterIntbUs;
    uint64_t pollFallbackUs;
    uint64_t staleDrainUs;
    uint64_t applyStartUs;
    uint64_t applyDoneUs;
    uint64_t sleepExitStartUs;
    uint64_t sleepExitDoneUs;
    uint64_t readyBeginUs;
    uint64_t intbSeenUs;
    uint64_t statusVerifyStartUs;
    uint64_t drdyUs;
    uint64_t readStartUs;
    uint64_t readDoneUs;

    uint32_t readyPollCount;
    uint32_t regWriteCount;
    uint32_t regReadCount;
    uint32_t verifyReadCount;
    uint32_t retryCount;
    uint32_t timeoutCount;
    uint32_t nackCount;

    uint32_t cacheDiffWriteCount;
    uint32_t cacheFullWriteCount;
    uint32_t cacheNoDiffCount;
    uint32_t diffRcountWrites;
    uint32_t diffSettleWrites;
    uint32_t diffClockDivWrites;
    uint32_t diffDriveWrites;
    uint32_t diffMuxWrites;
    uint32_t diffStatusConfigWrites;
    uint32_t diffConfigWrites;
    uint32_t appliedFingerprintChanged;
    uint32_t cacheApplyRestartCount;

    uint32_t intbEdgeCount;
    uint32_t intbFalseEdgeCount;
    uint32_t alreadyLowCount;
    uint32_t intbTimeoutCount;
    uint32_t intbFallbackPollCount;
    uint32_t intbFreshDrdyCount;
    uint32_t preStatusReadyCount;
    uint32_t intbReadyCount;
    uint32_t lateStatusReadyCount;
    uint32_t trueTimeoutCount;
    uint32_t readyFullCount;
    uint32_t recoveredAfterRetryCount;
    uint32_t readyPartialCount;
    uint32_t readyNoneCount;
    uint32_t transientUnreadNoDrdyCount;
    uint32_t staleUnreadDrainCount;
    uint32_t hardReadyTimeoutCount;
    uint32_t statusReadsBeforeIntbCount;
    uint32_t statusReadsPrecheckCount;
    uint32_t statusReadsAfterIntbCount;
    uint32_t statusReadsInFallbackCount;
    uint32_t statusReadSuppressedBeforeIntbCount;
    uint32_t noStatusPollWaitCount;
    uint32_t statusAfterIntbCount;
    uint32_t statusAfterTimeoutCount;
    uint32_t hardTimeoutStatusDiagCount;
    uint32_t intbActiveStatusMismatchCount;
    uint32_t suppressedRpCount;
    uint32_t internalWaitStateLeakCount;
    uint32_t unsafeUnreadNoDrdyCount;
    uint32_t drdyPartialUnreadCount;
    uint32_t drdyFullUnreadReadyCount;
    uint32_t statusReadErrCount;
    uint32_t unreadWithoutDrdyCount;
    uint32_t dataNotReadyCount;
    uint32_t softInvalidCount;
    uint32_t hardInvalidCount;
    uint32_t drainCount;
    uint32_t diagUnreadLikelyFreshCount;
    uint32_t diagUnreadLikelyStaleCount;
    uint32_t zeroBeforeReadyCount;
    uint32_t zeroAfterDrdyCount;
    uint32_t fallbackAttemptCount;
    uint32_t fallbackSuccessCount;
    uint32_t fallbackPartialCount;
    uint32_t fallbackFailCount;
    uint64_t fallbackSecondWaitUs;
    uint64_t fallbackSecondWaitMaxUs;
    uint32_t fallbackSecondWaitCount;
    uint32_t directDataReadCount;
    uint32_t directDataFallbackCount;
    uint32_t directDataFallbackReasonMask;
    uint32_t statusSavedReadCount;
    uint32_t deviceFullInvalidCount;
    uint64_t maxWaitReadyUs;
    uint64_t maxI2cReadUs;
} sensorarrayFdcDeviceTiming_t;

typedef struct {
    bool valid;
    bool sourceIsShadow;
    bool sourceIsReadback;

    uint16_t rCount[4];
    uint16_t settleCount[4];
    uint16_t clockDividers[4];
    uint16_t driveCurrent[4];
    uint8_t deglitchCode[4];
    uint32_t effectiveFclkHz[4];

    uint32_t chSettleUs[4];
    uint32_t chConvertUs[4];
    uint32_t chSwitchUs[4];
    uint32_t chTotalUs[4];

    uint32_t autoscanRoundUs;
    uint32_t expectedTimeoutUs;

    uint16_t muxConfig;
    uint16_t statusConfig;
    uint16_t config;
} sensorarrayFdcProfileSnapshot_t;

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
    uint16_t staleAmplitudeWarnings;
    uint16_t transientAmplitudeWarnings;
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
    uint32_t lastAppliedFingerprint;
    uint32_t lastReapplyFingerprint;
    uint32_t lastReapplyFrame;
    uint64_t warningSuppressedUntilUs;
    uint64_t lastFastSweepRequestUs;
    bool degraded;
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
    uint16_t muxConfig;
    uint16_t statusConfig;
    uint16_t configBaseWithoutSleepBit;
    uint32_t fingerprint;

    uint32_t cacheGeneration[4];

    uint32_t applyCount;
    int64_t lastAppliedTimestampUs;

    bool dirty;

    sensorarrayFdcProfileSnapshot_t profileSnapshot;
    sensorarrayFdcProfileSnapshot_t bootStableProfile;
    sensorarrayFdcProfileSnapshot_t formalFastProfile;
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
