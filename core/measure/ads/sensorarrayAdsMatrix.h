#pragma once

#include "esp_err.h"

#include "sensorarrayFrame.h"
#include "sensorarrayScanPlan.h"
#include "sensorarrayTypes.h"
#include "sensorarrayAdsAutoRange.h"
#include "sensorarrayAdsCache.h"
#include "sensorarrayRouteController.h"

#define SENSORARRAY_ADS_MATRIX_CALIBRATION_VERSION 1u

typedef struct {
    uint32_t version;
    uint32_t referenceResistorOhms;
    uint32_t matrixReferenceSpanUv;
    int64_t globalPathOffsetMilliohms;
    int32_t bankOffsetMilliohms[2];
    int32_t cellOffsetMilliohms[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    uint64_t cellOffsetValidMask;
} sensorarrayAdsMatrixCalibration_t;

typedef struct {
    sensorarrayState_t *state;
    sensorarrayRouteController_t *routeController;
    sensorarrayMeasurementMode_t mode;
    sensorarrayAdsProfileCache_t profileCache;
    sensorarrayAdsValueCache_t valueCache;
    sensorarrayAdsRegisterCache_t registerCache;
    sensorarrayAdsRailFingerprint_t railFingerprint;
    sensorarrayAdsMatrixCalibration_t calibration;
    uint32_t calibrationGeneration;
    uint8_t transitionSensitiveFrames;
    uint32_t frameSequenceHint;
    uint32_t frameCount;
    /* Cells whose bounded re-route confirmation resolved to HIGH_Z_OPEN.
     * A latched cell cannot publish a transient finite resistance: its clean
     * samples must pass another bounded confirmation before the bit clears. */
    uint64_t highZOpenMask;
} sensorarrayAdsMatrixEngine_t;

esp_err_t sensorarrayAdsMatrixEngineInit(sensorarrayAdsMatrixEngine_t *engine,
                                         sensorarrayState_t *state);
esp_err_t sensorarrayAdsMatrixEngineReadFrame(sensorarrayAdsMatrixEngine_t *engine,
                                              const sensorarrayScanPlan_t *plan,
                                              sensorarrayFrame_t *frame);
void sensorarrayAdsMatrixEngineBindRouteController(
    sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayRouteController_t *routeController);
esp_err_t sensorarrayAdsMatrixEngineSetMode(sensorarrayAdsMatrixEngine_t *engine,
                                           sensorarrayMeasurementMode_t mode);
void sensorarrayAdsMatrixEngineSetFrameSequenceHint(sensorarrayAdsMatrixEngine_t *engine,
                                                    uint32_t sequence);
void sensorarrayAdsMatrixEngineInvalidateGainCache(sensorarrayAdsMatrixEngine_t *engine);
void sensorarrayAdsMatrixEngineInvalidateCaches(sensorarrayAdsMatrixEngine_t *engine);
sensorarrayAdsRegisterCache_t *sensorarrayAdsMatrixEngineRegisterCache(
    sensorarrayAdsMatrixEngine_t *engine);
bool sensorarrayAdsMatrixCalibrationValid(
    const sensorarrayAdsMatrixCalibration_t *calibration);
esp_err_t sensorarrayAdsMatrixEngineSetCalibration(
    sensorarrayAdsMatrixEngine_t *engine,
    const sensorarrayAdsMatrixCalibration_t *calibration);
bool sensorarrayAdsMatrixEngineGetCalibration(
    const sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayAdsMatrixCalibration_t *outCalibration);
