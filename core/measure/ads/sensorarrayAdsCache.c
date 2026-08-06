#include "sensorarrayAdsCache.h"

#include <limits.h>
#include <string.h>

#include "sensorarrayAdsAutoRange.h"

static sensorarrayAdsCellProfile_t *sensorarrayAdsProfileCell(
    sensorarrayAdsProfileCache_t *cache,
    sensorarrayMeasurementMode_t mode,
    uint8_t cellIndex)
{
    if (!cache || cellIndex >= SENSORARRAY_MEASUREMENT_MAX_CELLS) {
        return NULL;
    }
    if (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
        return &cache->voltage[cellIndex];
    }
    if (mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE) {
        return &cache->resistance[cellIndex];
    }
    return NULL;
}

static uint32_t sensorarrayAdsAbsoluteDelta32(int32_t left, int32_t right)
{
    int64_t delta = (int64_t)left - (int64_t)right;
    if (delta < 0) {
        delta = -delta;
    }
    return delta > UINT32_MAX ? UINT32_MAX : (uint32_t)delta;
}

void sensorarrayAdsProfileCacheInit(sensorarrayAdsProfileCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache, 0, sizeof(*cache));
    cache->generation = 1u;
}

void sensorarrayAdsProfileCacheInvalidate(sensorarrayAdsProfileCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache->voltage, 0, sizeof(cache->voltage));
    memset(cache->resistance, 0, sizeof(cache->resistance));
    cache->generation++;
    cache->invalidationCount++;
}

void sensorarrayAdsProfileCacheInvalidateCell(sensorarrayAdsProfileCache_t *cache,
                                              sensorarrayMeasurementMode_t mode,
                                              uint8_t cellIndex)
{
    sensorarrayAdsCellProfile_t *cell = sensorarrayAdsProfileCell(cache, mode, cellIndex);
    if (!cell) {
        return;
    }
    memset(cell, 0, sizeof(*cell));
    cache->invalidationCount++;
}

bool sensorarrayAdsProfileCacheGet(sensorarrayAdsProfileCache_t *cache,
                                   sensorarrayMeasurementMode_t mode,
                                   uint8_t cellIndex,
                                   sensorarrayAdsCellProfile_t *outProfile)
{
    sensorarrayAdsCellProfile_t *cell = sensorarrayAdsProfileCell(cache, mode, cellIndex);
    if (!cell || !cell->valid || cell->generation != cache->generation || !outProfile) {
        return false;
    }
    *outProfile = *cell;
    return true;
}

bool sensorarrayAdsProfileCacheStore(sensorarrayAdsProfileCache_t *cache,
                                     sensorarrayMeasurementMode_t mode,
                                     uint8_t cellIndex,
                                     sensorarrayAdsInputMode_t inputMode,
                                     uint8_t gain,
                                     uint32_t frameSequence)
{
    sensorarrayAdsCellProfile_t *cell = sensorarrayAdsProfileCell(cache, mode, cellIndex);
    if (!cell || (inputMode != SENSORARRAY_ADS_INPUT_PGA &&
                  inputMode != SENSORARRAY_ADS_INPUT_BYPASS) ||
        !sensorarrayAdsAutoRangeGainSupported(gain)) {
        return false;
    }
    *cell = (sensorarrayAdsCellProfile_t){
        .valid = true,
        .inputMode = inputMode,
        .gain = gain,
        .stableStreak = 1u,
        .generation = cache->generation,
        .lastVerifiedFrame = frameSequence,
    };
    return true;
}

void sensorarrayAdsProfileCacheNoteSuccess(sensorarrayAdsProfileCache_t *cache,
                                           sensorarrayMeasurementMode_t mode,
                                           uint8_t cellIndex,
                                           uint32_t frameSequence)
{
    sensorarrayAdsCellProfile_t *cell = sensorarrayAdsProfileCell(cache, mode, cellIndex);
    if (!cell || !cell->valid || cell->generation != cache->generation) {
        return;
    }
    if (cell->stableStreak < UINT8_MAX) {
        cell->stableStreak++;
    }
    cell->overrangeStreak = 0u;
    cell->failureStreak = 0u;
    cell->lastVerifiedFrame = frameSequence;
}

void sensorarrayAdsProfileCacheNoteFailure(sensorarrayAdsProfileCache_t *cache,
                                           sensorarrayMeasurementMode_t mode,
                                           uint8_t cellIndex,
                                           bool overrange,
                                           uint8_t invalidationThreshold)
{
    sensorarrayAdsCellProfile_t *cell = sensorarrayAdsProfileCell(cache, mode, cellIndex);
    if (!cell || !cell->valid || cell->generation != cache->generation) {
        return;
    }
    if (cell->failureStreak < UINT8_MAX) {
        cell->failureStreak++;
    }
    if (overrange && cell->overrangeStreak < UINT8_MAX) {
        cell->overrangeStreak++;
    }
    cell->stableStreak = 0u;
    if (invalidationThreshold == 0u ||
        cell->failureStreak >= invalidationThreshold ||
        cell->overrangeStreak >= invalidationThreshold) {
        cell->valid = false;
        cache->invalidationCount++;
    }
}

void sensorarrayAdsValueCacheInit(sensorarrayAdsValueCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache, 0, sizeof(*cache));
    cache->generation = 1u;
}

void sensorarrayAdsValueCacheInvalidate(sensorarrayAdsValueCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache->voltage, 0, sizeof(cache->voltage));
    memset(cache->resistance, 0, sizeof(cache->resistance));
    cache->generation++;
    cache->invalidationCount++;
}

sensorarrayAdsCellValueCache_t *sensorarrayAdsValueCacheGet(
    sensorarrayAdsValueCache_t *cache,
    sensorarrayMeasurementMode_t mode,
    uint8_t cellIndex)
{
    if (!cache || cellIndex >= SENSORARRAY_MEASUREMENT_MAX_CELLS) {
        return NULL;
    }
    if (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
        return &cache->voltage[cellIndex];
    }
    if (mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE) {
        return &cache->resistance[cellIndex];
    }
    return NULL;
}

uint32_t sensorarrayAdsValueCacheThresholdRaw(
    const sensorarrayAdsCellValueCache_t *cell,
    uint32_t minimumRawThreshold,
    uint32_t noiseMultiplier)
{
    if (!cell || noiseMultiplier == 0u) {
        return minimumRawThreshold;
    }
    uint64_t dynamicThreshold = (uint64_t)cell->noiseEstimateRaw * noiseMultiplier;
    if (dynamicThreshold > UINT32_MAX) {
        dynamicThreshold = UINT32_MAX;
    }
    return dynamicThreshold > minimumRawThreshold ?
        (uint32_t)dynamicThreshold : minimumRawThreshold;
}

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
    bool transitionSensitive)
{
    if (!cell || !cell->valid || !profileValid || !statusClean || !freshConversion ||
        precisionFrame || transitionSensitive || cell->stableStreak < minimumStableStreak) {
        return false;
    }
    uint32_t threshold = sensorarrayAdsValueCacheThresholdRaw(
        cell, minimumRawThreshold, noiseMultiplier);
    return sensorarrayAdsAbsoluteDelta32(currentRaw, cell->lastRaw) <= threshold;
}

void sensorarrayAdsValueCacheObserve(sensorarrayAdsCellValueCache_t *cell,
                                     int32_t raw,
                                     int32_t nodeUv,
                                     int64_t valueFixed,
                                     uint32_t spreadRaw,
                                     uint32_t frameSequence,
                                     bool fullSample)
{
    if (!cell) {
        return;
    }
    uint32_t delta = cell->valid ? sensorarrayAdsAbsoluteDelta32(raw, cell->lastRaw) : 0u;
    uint32_t priorNoise = cell->noiseEstimateRaw;
    uint64_t stabilityLimit64 = priorNoise > 0u ?
        (uint64_t)priorNoise * 4u : spreadRaw;
    uint32_t stabilityLimit = stabilityLimit64 > UINT32_MAX ?
        UINT32_MAX : (uint32_t)stabilityLimit64;
    if (!cell->valid || delta <= stabilityLimit) {
        if (cell->stableStreak < UINT32_MAX) {
            cell->stableStreak++;
        }
        cell->unstableStreak = 0u;
    } else {
        cell->stableStreak = 0u;
        if (cell->unstableStreak < UINT32_MAX) {
            cell->unstableStreak++;
        }
    }
    if (fullSample) {
        cell->noiseEstimateRaw = priorNoise == 0u ? spreadRaw :
            (uint32_t)(((uint64_t)priorNoise * 3u + spreadRaw + 2u) / 4u);
        cell->lastFullSampleFrame = frameSequence;
    }
    cell->valid = true;
    cell->lastRaw = raw;
    cell->lastNodeUv = nodeUv;
    cell->lastValueFixed = valueFixed;
    cell->ageFrames = 0u;
}

void sensorarrayAdsValueCacheAgeFrame(sensorarrayAdsValueCache_t *cache,
                                      sensorarrayMeasurementMode_t mode,
                                      uint8_t activeCellCount)
{
    if (!cache) {
        return;
    }
    if (activeCellCount > SENSORARRAY_MEASUREMENT_MAX_CELLS) {
        activeCellCount = SENSORARRAY_MEASUREMENT_MAX_CELLS;
    }
    for (uint8_t index = 0u; index < activeCellCount; ++index) {
        sensorarrayAdsCellValueCache_t *cell = sensorarrayAdsValueCacheGet(cache, mode, index);
        if (cell && cell->valid && cell->ageFrames < UINT32_MAX) {
            cell->ageFrames++;
        }
    }
}

void sensorarrayAdsRailFingerprintInit(sensorarrayAdsRailFingerprint_t *fingerprint)
{
    if (!fingerprint) {
        return;
    }
    memset(fingerprint, 0, sizeof(*fingerprint));
    fingerprint->invalidationArmed = true;
}

static sensorarrayAdsRailInvalidationReason_t sensorarrayAdsRailFingerprintReason(
    const sensorarrayAdsRailFingerprint_t *fingerprint,
    bool railValid,
    bool railStatusGood,
    int32_t railUv,
    int32_t avddUv,
    int32_t avssUv,
    uint8_t railSource,
    uint8_t referenceSource,
    uint32_t calibrationGeneration,
    uint32_t invalidationThresholdUv,
    bool adsReset)
{
    if (adsReset) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_RESET;
    }
    if (!fingerprint->valid) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_INITIAL;
    }
    if (fingerprint->railValid != railValid) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_VALIDITY;
    }
    if (fingerprint->railStatusGood != railStatusGood) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_STATUS;
    }
    if (fingerprint->railSource != railSource) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_SOURCE;
    }
    if (fingerprint->referenceSource != referenceSource) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_REFERENCE;
    }
    if (fingerprint->calibrationGeneration != calibrationGeneration) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_CALIBRATION;
    }
    /* A monitored rail split inherits normal ADC noise. Treat only a
     * material split movement as a new electrical fingerprint; source and
     * calibration changes are handled independently above. */
    if (sensorarrayAdsAbsoluteDelta32(fingerprint->avddUv, avddUv) >
            invalidationThresholdUv ||
        sensorarrayAdsAbsoluteDelta32(fingerprint->avssUv, avssUv) >
            invalidationThresholdUv) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_SPLIT;
    }
    if (sensorarrayAdsAbsoluteDelta32(fingerprint->railUv, railUv) >
        invalidationThresholdUv) {
        return SENSORARRAY_ADS_RAIL_INVALIDATION_DELTA;
    }
    return SENSORARRAY_ADS_RAIL_INVALIDATION_NONE;
}

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
    sensorarrayAdsRailInvalidationReason_t *outReason)
{
    if (!fingerprint || invalidationThresholdUv == 0u ||
        hysteresisUv >= invalidationThresholdUv) {
        if (outReason) {
            *outReason = SENSORARRAY_ADS_RAIL_INVALIDATION_NONE;
        }
        return false;
    }
    sensorarrayAdsRailInvalidationReason_t reason = sensorarrayAdsRailFingerprintReason(
        fingerprint, railValid, railStatusGood, railUv, avddUv, avssUv, railSource,
        referenceSource, calibrationGeneration, invalidationThresholdUv, adsReset);
    bool invalidated = reason != SENSORARRAY_ADS_RAIL_INVALIDATION_NONE;
    if (reason == SENSORARRAY_ADS_RAIL_INVALIDATION_DELTA &&
        !fingerprint->invalidationArmed) {
        invalidated = false;
    }
    if (invalidated) {
        fingerprint->missCount++;
        fingerprint->invalidationCount++;
        fingerprint->lastInvalidationReason = reason;
        fingerprint->invalidationArmed = reason != SENSORARRAY_ADS_RAIL_INVALIDATION_DELTA;
    } else {
        fingerprint->hitCount++;
        uint32_t rearmThreshold = invalidationThresholdUv - hysteresisUv;
        if (sensorarrayAdsAbsoluteDelta32(fingerprint->railUv, railUv) <= rearmThreshold) {
            fingerprint->invalidationArmed = true;
        }
    }
    if (invalidated || !fingerprint->valid || !fingerprint->invalidationArmed) {
        fingerprint->railUv = railUv;
    }
    fingerprint->valid = true;
    fingerprint->railValid = railValid;
    fingerprint->railStatusGood = railStatusGood;
    fingerprint->avddUv = avddUv;
    fingerprint->avssUv = avssUv;
    fingerprint->railSource = railSource;
    fingerprint->referenceSource = referenceSource;
    fingerprint->calibrationGeneration = calibrationGeneration;
    if (outReason) {
        *outReason = invalidated ? reason : SENSORARRAY_ADS_RAIL_INVALIDATION_NONE;
    }
    return invalidated;
}

const char *sensorarrayAdsRailInvalidationReasonName(
    sensorarrayAdsRailInvalidationReason_t reason)
{
    switch (reason) {
    case SENSORARRAY_ADS_RAIL_INVALIDATION_INITIAL: return "initial";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_DELTA: return "delta";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_SOURCE: return "source";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_VALIDITY: return "validity";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_STATUS: return "status";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_SPLIT: return "split";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_REFERENCE: return "reference";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_CALIBRATION: return "calibration";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_RESET: return "reset";
    case SENSORARRAY_ADS_RAIL_INVALIDATION_NONE:
    default: return "none";
    }
}

void sensorarrayAdsRegisterCacheInit(sensorarrayAdsRegisterCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache, 0, sizeof(*cache));
    cache->generation = 1u;
}

void sensorarrayAdsRegisterCacheInvalidate(sensorarrayAdsRegisterCache_t *cache)
{
    if (!cache) {
        return;
    }
    memset(cache->registers, 0, sizeof(cache->registers));
    cache->vrefValid = false;
    cache->pgaModeValid = false;
    cache->adc1RunningValid = false;
    cache->generation++;
    cache->invalidationCount++;
}

bool sensorarrayAdsRegisterCacheNeedsWrite(sensorarrayAdsRegisterCache_t *cache,
                                           sensorarrayAdsRegisterId_t registerId,
                                           uint8_t value)
{
    if (!cache || registerId >= SENSORARRAY_ADS_REGISTER_COUNT) {
        return true;
    }
    sensorarrayAdsRegisterShadowValue_t *entry = &cache->registers[registerId];
    if (entry->valid && entry->value == value) {
        cache->cacheHitCount++;
        return false;
    }
    return true;
}

void sensorarrayAdsRegisterCacheNoteWrite(sensorarrayAdsRegisterCache_t *cache,
                                          sensorarrayAdsRegisterId_t registerId,
                                          uint8_t value,
                                          bool verified)
{
    if (!cache || registerId >= SENSORARRAY_ADS_REGISTER_COUNT) {
        return;
    }
    sensorarrayAdsRegisterShadowValue_t *entry = &cache->registers[registerId];
    entry->value = value;
    entry->valid = true;
    entry->verifiedGeneration = verified ? cache->generation : 0u;
    cache->writeCount++;
}

bool sensorarrayAdsRegisterCacheNoteReadback(sensorarrayAdsRegisterCache_t *cache,
                                             sensorarrayAdsRegisterId_t registerId,
                                             uint8_t value)
{
    if (!cache || registerId >= SENSORARRAY_ADS_REGISTER_COUNT) {
        return false;
    }
    sensorarrayAdsRegisterShadowValue_t *entry = &cache->registers[registerId];
    cache->readbackCount++;
    if (entry->valid && entry->value != value) {
        entry->valid = false;
        return false;
    }
    entry->value = value;
    entry->valid = true;
    entry->verifiedGeneration = cache->generation;
    return true;
}

bool sensorarrayAdsRegisterCacheAcquire(sensorarrayAdsRegisterCache_t *cache,
                                        sensorarrayAdsOwner_t owner)
{
    if (!cache || owner == SENSORARRAY_ADS_OWNER_NONE ||
        cache->owner != SENSORARRAY_ADS_OWNER_NONE) {
        return false;
    }
    cache->owner = owner;
    return true;
}

bool sensorarrayAdsRegisterCacheRelease(sensorarrayAdsRegisterCache_t *cache,
                                        sensorarrayAdsOwner_t owner)
{
    if (!cache || owner == SENSORARRAY_ADS_OWNER_NONE || cache->owner != owner) {
        return false;
    }
    cache->owner = SENSORARRAY_ADS_OWNER_NONE;
    return true;
}

const char *sensorarrayAdsOwnerName(sensorarrayAdsOwner_t owner)
{
    switch (owner) {
    case SENSORARRAY_ADS_OWNER_MATRIX: return "MATRIX";
    case SENSORARRAY_ADS_OWNER_BATTERY: return "BATTERY";
    case SENSORARRAY_ADS_OWNER_CHECK: return "ADSCHK";
    case SENSORARRAY_ADS_OWNER_RAIL: return "RAIL";
    case SENSORARRAY_ADS_OWNER_ZERO: return "ZERO";
    case SENSORARRAY_ADS_OWNER_NONE:
    default: return "NONE";
    }
}
