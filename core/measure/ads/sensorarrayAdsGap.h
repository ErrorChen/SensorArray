#pragma once

#include <stdint.h>

#include "esp_err.h"

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

esp_err_t sensorarrayAdsGapInit(sensorarrayState_t *state);
void sensorarrayAdsGapTryRun(sensorarrayState_t *state,
                             uint64_t expectedFdcReadyUs,
                             uint32_t frameSequence,
                             uint8_t row);
void sensorarrayAdsGapCopySnapshot(sensorarrayAdsGapSnapshot_t *outSnapshot,
                                   uint32_t frameSequence);
void sensorarrayAdsGapRequestCalibration(bool requestZero, bool requestRail);
