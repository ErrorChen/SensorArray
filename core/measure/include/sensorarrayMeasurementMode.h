#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Measurement mode is intentionally independent from transport and ESP-IDF.
 * Core 0 may read a snapshot, but only the Core 1 acquisition owner mutates
 * the context and applies hardware routes.
 */
typedef enum {
    SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE = 0,
    SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
    SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
    SENSORARRAY_MEASUREMENT_MODE_NONE,
} sensorarrayMeasurementMode_t;

typedef enum {
    SENSORARRAY_MEASUREMENT_STATE_UNINITIALISED = 0,
    SENSORARRAY_MEASUREMENT_STATE_SAFE,
    SENSORARRAY_MEASUREMENT_STATE_CAPACITANCE,
    SENSORARRAY_MEASUREMENT_STATE_VOLTAGE,
    SENSORARRAY_MEASUREMENT_STATE_RESISTANCE,
    SENSORARRAY_MEASUREMENT_STATE_TRANSITION,
    SENSORARRAY_MEASUREMENT_STATE_DEGRADED,
    SENSORARRAY_MEASUREMENT_STATE_FAULT,
} sensorarrayMeasurementState_t;

typedef enum {
    SENSORARRAY_MEASUREMENT_UNIT_PF = 0,
    SENSORARRAY_MEASUREMENT_UNIT_VOLT,
    SENSORARRAY_MEASUREMENT_UNIT_OHM,
    SENSORARRAY_MEASUREMENT_UNIT_NONE,
} sensorarrayMeasurementUnit_t;

typedef enum {
    SENSORARRAY_ADS_REFERENCE_NONE = 0,
    SENSORARRAY_ADS_REFERENCE_INTERNAL,
    SENSORARRAY_ADS_REFERENCE_AVDD_AVSS,
    SENSORARRAY_ADS_REFERENCE_EXTERNAL,
} sensorarrayAdsReferenceSource_t;

typedef enum {
    SENSORARRAY_CELL_ERROR_NONE = 0,
    SENSORARRAY_CELL_ERROR_ROUTE = 1,
    SENSORARRAY_CELL_ERROR_SPI = 2,
    SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT = 3,
    SENSORARRAY_CELL_ERROR_STALE = 4,
    SENSORARRAY_CELL_ERROR_REFERENCE_ALARM = 5,
    SENSORARRAY_CELL_ERROR_PGA_ABSOLUTE = 6,
    SENSORARRAY_CELL_ERROR_PGA_DIFFERENTIAL = 7,
    SENSORARRAY_CELL_ERROR_SATURATED = 8,
    SENSORARRAY_CELL_ERROR_COMMON_MODE = 9,
    SENSORARRAY_CELL_ERROR_RAIL_INVALID = 10,
    SENSORARRAY_CELL_ERROR_REFERENCE_INVALID = 11,
    SENSORARRAY_CELL_ERROR_DIVIDE_NEAR_ZERO = 12,
    SENSORARRAY_CELL_ERROR_OPEN = 13,
    SENSORARRAY_CELL_ERROR_SHORT = 14,
    SENSORARRAY_CELL_ERROR_NEGATIVE = 15,
    SENSORARRAY_CELL_ERROR_RANGE = 16,
    SENSORARRAY_CELL_ERROR_OVERFLOW = 17,
    SENSORARRAY_CELL_ERROR_UNSTABLE = 18,
    SENSORARRAY_CELL_ERROR_AUTORANGE = 19,
    SENSORARRAY_CELL_ERROR_UNSUPPORTED = 20,
    SENSORARRAY_CELL_ERROR_READBACK = 21,
} sensorarrayCellError_t;

#define SENSORARRAY_MEASUREMENT_INVALID_FIXED INT64_MIN
#define SENSORARRAY_MEASUREMENT_MAX_CELLS 64u

typedef struct {
    sensorarrayMeasurementMode_t mode;
    sensorarrayMeasurementUnit_t unit;
    int8_t decimalScale;
    sensorarrayAdsReferenceSource_t referenceSource;
    uint32_t modeGeneration;
    uint32_t modeRequestId;
    int64_t valuesFixed[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    int32_t rawCode[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    int32_t nodeUv[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    uint8_t pgaGain[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    uint8_t errorReason[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    uint64_t validMask;
    uint64_t freshMask;
    uint64_t errorMask;
    uint64_t openMask;
    uint64_t shortMask;
    uint64_t unstableMask;
    uint64_t saturatedMask;
    uint32_t gainChangeCount;
    uint32_t overrangeCount;
    uint32_t autorangeAttemptCount;
    uint32_t autorangeFallbackCount;
    uint32_t ioRetryCount;
    uint32_t drdyTimeoutCount;
    uint32_t staleCount;
    uint32_t statusErrorCount;
    uint32_t spiErrorCount;
    uint32_t profileHitCount;
    uint32_t profileMissCount;
    uint32_t bypassHitCount;
    uint32_t gainHitCount;
    uint32_t registerCacheHitCount;
    uint32_t registerWriteCount;
    uint32_t registerReadbackCount;
    uint32_t singleSampleCellCount;
    uint32_t tripleSampleCellCount;
    uint32_t freshCellCount;
    uint32_t rawConversionCount;
    uint32_t profileInvalidationCount;
    uint32_t railFingerprintHitCount;
    uint32_t railFingerprintMissCount;
    uint32_t railInvalidationCount;
    uint8_t railInvalidationReason;
    bool precisionFrame;
    uint64_t frameDurationUs;
    uint64_t rowRouteUs;
    uint64_t muxWriteUs;
    uint64_t registerWriteUs;
    uint64_t registerReadbackUs;
    uint64_t drdyWaitUs;
    uint64_t sampleReadUs;
    uint64_t aggregationUs;
    uint64_t autorangeUs;
    uint64_t batteryUs;
    uint64_t adsCheckUs;
    uint64_t transitionDurationUs;
    int32_t avddUv;
    int32_t avssUv;
    int32_t matrixReferenceUv;
    uint32_t referenceResistorOhms;
    uint32_t railAgeFrames;
    bool railValid;
    bool referenceValid;
} sensorarrayMeasurementPayload_t;

typedef struct {
    sensorarrayMeasurementState_t state;
    sensorarrayMeasurementMode_t activeMode;
    sensorarrayMeasurementMode_t oldMode;
    sensorarrayMeasurementMode_t pendingMode;
    uint32_t pendingRequestId;
    uint32_t appliedRequestId;
    uint32_t generation;
    uint32_t appliedFrameSequence;
    uint32_t lastError;
    uint64_t transitionDurationUs;
    bool pending;
} sensorarrayMeasurementModeSnapshot_t;

typedef struct {
    volatile uint32_t snapshotVersion;
    sensorarrayMeasurementModeSnapshot_t snapshot;
} sensorarrayMeasurementModeContext_t;

void sensorarrayMeasurementModeInit(sensorarrayMeasurementModeContext_t *context);
bool sensorarrayMeasurementModeAccept(sensorarrayMeasurementModeContext_t *context,
                                      sensorarrayMeasurementMode_t requestedMode,
                                      uint32_t requestId);
bool sensorarrayMeasurementModeBeginTransition(sensorarrayMeasurementModeContext_t *context);
bool sensorarrayMeasurementModeCompleteTransition(sensorarrayMeasurementModeContext_t *context,
                                                  uint32_t appliedFrameSequence,
                                                  uint64_t transitionDurationUs);
void sensorarrayMeasurementModeFailTransition(sensorarrayMeasurementModeContext_t *context,
                                              uint32_t errorCode,
                                              uint64_t transitionDurationUs);
void sensorarrayMeasurementModeRecordRuntimeFault(sensorarrayMeasurementModeContext_t *context,
                                                  uint32_t errorCode);
bool sensorarrayMeasurementModeCopySnapshot(
    const sensorarrayMeasurementModeContext_t *context,
    sensorarrayMeasurementModeSnapshot_t *outSnapshot);

bool sensorarrayMeasurementModeIsDataMode(sensorarrayMeasurementMode_t mode);
uint8_t sensorarrayMeasurementCellCount(uint8_t rows);
sensorarrayMeasurementState_t sensorarrayMeasurementStateForMode(
    sensorarrayMeasurementMode_t mode);
const char *sensorarrayMeasurementModeName(sensorarrayMeasurementMode_t mode);
const char *sensorarrayMeasurementStateName(sensorarrayMeasurementState_t state);
const char *sensorarrayMeasurementUnitName(sensorarrayMeasurementUnit_t unit);
const char *sensorarrayAdsReferenceSourceName(sensorarrayAdsReferenceSource_t source);
const char *sensorarrayCellErrorName(sensorarrayCellError_t error);

#ifdef __cplusplus
}
#endif
