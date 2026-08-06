#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayAdsMath.h"
#include "sensorarrayAdsCache.h"
#include "sensorarrayTypes.h"

enum {
    SENSORARRAY_ACQ_RESOURCE_ROW_MUX = 1u << 0,
    SENSORARRAY_ACQ_RESOURCE_TMUX1134 = 1u << 1,
    SENSORARRAY_ACQ_RESOURCE_FDC_ROUTE = 1u << 2,
    SENSORARRAY_ACQ_RESOURCE_ADS_ROUTE = 1u << 3,
    SENSORARRAY_ACQ_RESOURCE_FDC_I2C0 = 1u << 4,
    SENSORARRAY_ACQ_RESOURCE_FDC_I2C1 = 1u << 5,
    SENSORARRAY_ACQ_RESOURCE_ADS_SPI = 1u << 6,
    SENSORARRAY_ACQ_RESOURCE_ADS_START = 1u << 7,
};

#define SENSORARRAY_FDC_ROW_RESOURCE_MASK \
    (SENSORARRAY_ACQ_RESOURCE_ROW_MUX | SENSORARRAY_ACQ_RESOURCE_TMUX1134 | \
     SENSORARRAY_ACQ_RESOURCE_FDC_ROUTE | SENSORARRAY_ACQ_RESOURCE_FDC_I2C0 | \
     SENSORARRAY_ACQ_RESOURCE_FDC_I2C1)

#define SENSORARRAY_ADS_DIRECT_RESOURCE_MASK \
    (SENSORARRAY_ACQ_RESOURCE_ADS_SPI | SENSORARRAY_ACQ_RESOURCE_ADS_START)

typedef enum {
    SENSORARRAY_ADS_GAP_MODE_OFF = 0,
    SENSORARRAY_ADS_GAP_MODE_ON,
    SENSORARRAY_ADS_GAP_MODE_RAIL,
    SENSORARRAY_ADS_GAP_MODE_BAT,
    SENSORARRAY_ADS_GAP_MODE_ZERO,
} sensorarrayAdsGapMode_t;

typedef enum {
    SENSORARRAY_ADS_RAIL_SOURCE_NONE = 0,
    SENSORARRAY_ADS_RAIL_SOURCE_MONITOR,
    SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION,
} sensorarrayAdsRailSource_t;

typedef struct {
    uint32_t requestId;
    ads126xAdc1RegisterSnapshot_t registers;
    uint16_t chip;
    uint8_t revision;
    bool adc1Available;
    bool adc2Available;
    uint32_t dataRateSps;
    uint32_t vrefUv;
    uint8_t gain;
    bool pgaBypassed;
    uint32_t requestedSamples;
    uint32_t freshSamples;
    uint32_t changedSamples;
    uint32_t periodMinUs;
    uint32_t periodAverageUs;
    uint32_t periodMaxUs;
    uint32_t spiErrors;
    uint32_t drdyTimeouts;
    uint32_t staleSamples;
    uint32_t statusErrors;
    uint32_t resetFlags;
    uint64_t durationUs;
    bool restoreOk;
    bool ok;
} sensorarrayAdsActiveCheckResult_t;

esp_err_t sensorarrayAdsGapInit(sensorarrayState_t *state);
void sensorarrayAdsGapBindRegisterCache(sensorarrayAdsRegisterCache_t *registerCache);
/* Core 1 only: refreshes the rail monitor between complete matrix frames. */
esp_err_t sensorarrayAdsGapRefreshRailAtBoundary(sensorarrayState_t *state,
                                                 uint32_t frameSequence);
/* Core 1 only: apply a volatile, externally measured AVDD/AVSS calibration at
 * a frame boundary. AVSS is signed and must be below GND. The value is not
 * persisted and remains subject to maximumAgeFrames. */
esp_err_t sensorarrayAdsGapSetExternalRailCalibration(int32_t avddUv,
                                                       int32_t avssUv,
                                                       uint32_t frameSequence);
bool sensorarrayAdsGapCopyRailSplit(uint32_t frameSequence,
                                    uint32_t maximumAgeFrames,
                                    sensorarrayAdsRailSplit_t *outRail);
void sensorarrayAdsGapTryRun(sensorarrayState_t *state,
                             uint64_t expectedFdcReadyUs,
                             uint32_t frameSequence,
                             uint8_t row);
void sensorarrayAdsGapCopySnapshot(sensorarrayAdsGapSnapshot_t *outSnapshot,
                                   uint32_t frameSequence);
void sensorarrayAdsGapRequestCalibration(bool requestZero, bool requestRail);
void sensorarrayAdsGapRequestBatteryDiagnostic(void);
void sensorarrayAdsGapRequestBatteryNow(void);
bool sensorarrayAdsGapConfigureBatteryPeriod(bool enabled, uint32_t periodMs);
esp_err_t sensorarrayAdsGapRunBatteryAtBoundary(sensorarrayState_t *state,
                                                uint32_t frameSequence,
                                                bool capacitanceMode,
                                                uint32_t *outDurationUs,
                                                bool *outRan);
bool sensorarrayAdsGapConsumeRestoreFailure(void);
esp_err_t sensorarrayAdsGapRunActiveCheck(
    sensorarrayState_t *state,
    sensorarrayAdsRegisterCache_t *registerCache,
    uint32_t requestId,
    uint32_t sampleCount,
    sensorarrayAdsActiveCheckResult_t *outResult);
size_t sensorarrayAdsGapFormatActiveCheck(
    const sensorarrayAdsActiveCheckResult_t *result,
    char *buffer,
    size_t bufferSize);
size_t sensorarrayAdsGapFormatActiveCheckTiming(
    const sensorarrayAdsActiveCheckResult_t *result,
    char *buffer,
    size_t bufferSize);
void sensorarrayAdsGapSetMode(sensorarrayAdsGapMode_t mode);
sensorarrayAdsGapMode_t sensorarrayAdsGapGetMode(void);
const char *sensorarrayAdsGapModeName(sensorarrayAdsGapMode_t mode);
const char *sensorarrayAdsRailStatusName(sensorarrayAdsRailStatus_t status);
const char *sensorarrayAdsRailSourceName(sensorarrayAdsRailSource_t source);
size_t sensorarrayAdsGapFormatBattery(char *buffer, size_t bufferSize, uint32_t frameSequence);
size_t sensorarrayAdsGapFormatRail(char *buffer, size_t bufferSize, uint32_t frameSequence);
size_t sensorarrayAdsGapFormatAds(char *buffer, size_t bufferSize);
