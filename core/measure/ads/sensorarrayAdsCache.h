#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sensorarrayMeasurementMode.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SENSORARRAY_ADS_INPUT_PGA = 0,
    SENSORARRAY_ADS_INPUT_BYPASS,
} sensorarrayAdsInputMode_t;

typedef struct {
    bool valid;
    sensorarrayAdsInputMode_t inputMode;
    uint8_t gain;
    uint8_t overrangeStreak;
    uint8_t stableStreak;
    uint8_t failureStreak;
    uint32_t generation;
    uint32_t lastVerifiedFrame;
} sensorarrayAdsCellProfile_t;

typedef struct {
    sensorarrayAdsCellProfile_t voltage[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    sensorarrayAdsCellProfile_t resistance[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    uint32_t generation;
    uint32_t invalidationCount;
} sensorarrayAdsProfileCache_t;

typedef struct {
    bool valid;
    int32_t lastRaw;
    int32_t lastNodeUv;
    int64_t lastValueFixed;
    uint32_t noiseEstimateRaw;
    uint32_t stableStreak;
    uint32_t unstableStreak;
    uint32_t ageFrames;
    uint32_t lastFullSampleFrame;
} sensorarrayAdsCellValueCache_t;

typedef struct {
    sensorarrayAdsCellValueCache_t voltage[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    sensorarrayAdsCellValueCache_t resistance[SENSORARRAY_MEASUREMENT_MAX_CELLS];
    uint32_t generation;
    uint32_t invalidationCount;
} sensorarrayAdsValueCache_t;

typedef enum {
    SENSORARRAY_ADS_RAIL_INVALIDATION_NONE = 0,
    SENSORARRAY_ADS_RAIL_INVALIDATION_INITIAL,
    SENSORARRAY_ADS_RAIL_INVALIDATION_DELTA,
    SENSORARRAY_ADS_RAIL_INVALIDATION_SOURCE,
    SENSORARRAY_ADS_RAIL_INVALIDATION_VALIDITY,
    SENSORARRAY_ADS_RAIL_INVALIDATION_STATUS,
    SENSORARRAY_ADS_RAIL_INVALIDATION_SPLIT,
    SENSORARRAY_ADS_RAIL_INVALIDATION_REFERENCE,
    SENSORARRAY_ADS_RAIL_INVALIDATION_CALIBRATION,
    SENSORARRAY_ADS_RAIL_INVALIDATION_RESET,
} sensorarrayAdsRailInvalidationReason_t;

typedef struct {
    bool valid;
    bool railValid;
    bool railStatusGood;
    bool invalidationArmed;
    int32_t railUv;
    int32_t avddUv;
    int32_t avssUv;
    uint8_t railSource;
    uint8_t referenceSource;
    uint32_t calibrationGeneration;
    uint32_t hitCount;
    uint32_t missCount;
    uint32_t invalidationCount;
    sensorarrayAdsRailInvalidationReason_t lastInvalidationReason;
} sensorarrayAdsRailFingerprint_t;

typedef enum {
    SENSORARRAY_ADS_REGISTER_POWER = 0,
    SENSORARRAY_ADS_REGISTER_INTERFACE,
    SENSORARRAY_ADS_REGISTER_MODE0,
    SENSORARRAY_ADS_REGISTER_MODE1,
    SENSORARRAY_ADS_REGISTER_MODE2,
    SENSORARRAY_ADS_REGISTER_INPMUX,
    SENSORARRAY_ADS_REGISTER_REFMUX,
    SENSORARRAY_ADS_REGISTER_OFCAL0,
    SENSORARRAY_ADS_REGISTER_OFCAL1,
    SENSORARRAY_ADS_REGISTER_OFCAL2,
    SENSORARRAY_ADS_REGISTER_FSCAL0,
    SENSORARRAY_ADS_REGISTER_FSCAL1,
    SENSORARRAY_ADS_REGISTER_FSCAL2,
    SENSORARRAY_ADS_REGISTER_COUNT,
} sensorarrayAdsRegisterId_t;

typedef struct {
    uint8_t value;
    bool valid;
    uint32_t verifiedGeneration;
} sensorarrayAdsRegisterShadowValue_t;

typedef enum {
    SENSORARRAY_ADS_OWNER_NONE = 0,
    SENSORARRAY_ADS_OWNER_MATRIX,
    SENSORARRAY_ADS_OWNER_BATTERY,
    SENSORARRAY_ADS_OWNER_CHECK,
    SENSORARRAY_ADS_OWNER_RAIL,
    SENSORARRAY_ADS_OWNER_ZERO,
} sensorarrayAdsOwner_t;

typedef struct {
    sensorarrayAdsRegisterShadowValue_t registers[SENSORARRAY_ADS_REGISTER_COUNT];
    bool vrefValid;
    int32_t vrefUv;
    bool pgaModeValid;
    sensorarrayAdsInputMode_t inputMode;
    uint8_t gain;
    bool adc1RunningValid;
    bool adc1Running;
    sensorarrayAdsOwner_t owner;
    uint32_t generation;
    uint32_t cacheHitCount;
    uint32_t writeCount;
    uint32_t readbackCount;
    uint32_t invalidationCount;
} sensorarrayAdsRegisterCache_t;

void sensorarrayAdsProfileCacheInit(sensorarrayAdsProfileCache_t *cache);
void sensorarrayAdsProfileCacheInvalidate(sensorarrayAdsProfileCache_t *cache);
void sensorarrayAdsProfileCacheInvalidateCell(sensorarrayAdsProfileCache_t *cache,
                                              sensorarrayMeasurementMode_t mode,
                                              uint8_t cellIndex);
bool sensorarrayAdsProfileCacheGet(sensorarrayAdsProfileCache_t *cache,
                                   sensorarrayMeasurementMode_t mode,
                                   uint8_t cellIndex,
                                   sensorarrayAdsCellProfile_t *outProfile);
bool sensorarrayAdsProfileCacheStore(sensorarrayAdsProfileCache_t *cache,
                                     sensorarrayMeasurementMode_t mode,
                                     uint8_t cellIndex,
                                     sensorarrayAdsInputMode_t inputMode,
                                     uint8_t gain,
                                     uint32_t frameSequence);
void sensorarrayAdsProfileCacheNoteSuccess(sensorarrayAdsProfileCache_t *cache,
                                           sensorarrayMeasurementMode_t mode,
                                           uint8_t cellIndex,
                                           uint32_t frameSequence);
void sensorarrayAdsProfileCacheNoteFailure(sensorarrayAdsProfileCache_t *cache,
                                           sensorarrayMeasurementMode_t mode,
                                           uint8_t cellIndex,
                                           bool overrange,
                                           uint8_t invalidationThreshold);

void sensorarrayAdsValueCacheInit(sensorarrayAdsValueCache_t *cache);
void sensorarrayAdsValueCacheInvalidate(sensorarrayAdsValueCache_t *cache);
sensorarrayAdsCellValueCache_t *sensorarrayAdsValueCacheGet(
    sensorarrayAdsValueCache_t *cache,
    sensorarrayMeasurementMode_t mode,
    uint8_t cellIndex);
uint32_t sensorarrayAdsValueCacheThresholdRaw(
    const sensorarrayAdsCellValueCache_t *cell,
    uint32_t minimumRawThreshold,
    uint32_t noiseMultiplier);
bool sensorarrayAdsValueCacheShouldFastAccept(
    const sensorarrayAdsCellValueCache_t *cell,
    int32_t currentRaw,
    uint32_t minimumRawThreshold,
    uint32_t noiseMultiplier,
    uint32_t minimumStableStreak,
    bool profileValid,
    bool statusClean,
    bool freshConversion,
    bool precisionFrame,
    bool transitionSensitive);
void sensorarrayAdsValueCacheObserve(sensorarrayAdsCellValueCache_t *cell,
                                     int32_t raw,
                                     int32_t nodeUv,
                                     int64_t valueFixed,
                                     uint32_t spreadRaw,
                                     uint32_t frameSequence,
                                     bool fullSample);
void sensorarrayAdsValueCacheAgeFrame(sensorarrayAdsValueCache_t *cache,
                                      sensorarrayMeasurementMode_t mode,
                                      uint8_t activeCellCount);

void sensorarrayAdsRailFingerprintInit(sensorarrayAdsRailFingerprint_t *fingerprint);
bool sensorarrayAdsRailFingerprintUpdate(
    sensorarrayAdsRailFingerprint_t *fingerprint,
    bool railValid,
    bool railStatusGood,
    int32_t railUv,
    int32_t avddUv,
    int32_t avssUv,
    uint8_t railSource,
    uint8_t referenceSource,
    uint32_t calibrationGeneration,
    uint32_t invalidationThresholdUv,
    uint32_t hysteresisUv,
    bool adsReset,
    sensorarrayAdsRailInvalidationReason_t *outReason);
const char *sensorarrayAdsRailInvalidationReasonName(
    sensorarrayAdsRailInvalidationReason_t reason);

void sensorarrayAdsRegisterCacheInit(sensorarrayAdsRegisterCache_t *cache);
void sensorarrayAdsRegisterCacheInvalidate(sensorarrayAdsRegisterCache_t *cache);
bool sensorarrayAdsRegisterCacheNeedsWrite(sensorarrayAdsRegisterCache_t *cache,
                                           sensorarrayAdsRegisterId_t registerId,
                                           uint8_t value);
void sensorarrayAdsRegisterCacheNoteWrite(sensorarrayAdsRegisterCache_t *cache,
                                          sensorarrayAdsRegisterId_t registerId,
                                          uint8_t value,
                                          bool verified);
bool sensorarrayAdsRegisterCacheNoteReadback(sensorarrayAdsRegisterCache_t *cache,
                                             sensorarrayAdsRegisterId_t registerId,
                                             uint8_t value);
bool sensorarrayAdsRegisterCacheAcquire(sensorarrayAdsRegisterCache_t *cache,
                                        sensorarrayAdsOwner_t owner);
bool sensorarrayAdsRegisterCacheRelease(sensorarrayAdsRegisterCache_t *cache,
                                        sensorarrayAdsOwner_t owner);
const char *sensorarrayAdsOwnerName(sensorarrayAdsOwner_t owner);

#ifdef __cplusplus
}
#endif
