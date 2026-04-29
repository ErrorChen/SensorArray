#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "ads126xAdc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_VOLTAGE_SCAN_ROWS 8u
#define SENSORARRAY_VOLTAGE_SCAN_COLS 8u

typedef struct {
    uint32_t sequence;
    uint64_t timestampUs;
    uint32_t scanDurationUs;
    int32_t microvolts[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    int32_t raw[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    uint8_t gain[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    esp_err_t err[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
    bool clipped[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
} sensorarrayVoltageFrame_t;

esp_err_t sensorarrayVoltageScanApplyRouteFast(uint8_t sColumn,
                                               uint8_t dLine,
                                               uint32_t rowSettleUs,
                                               uint32_t pathSettleUs);

esp_err_t sensorarrayVoltageScanOneFrame(ads126xAdcHandle_t *ads,
                                         const uint8_t gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS]
                                                                [SENSORARRAY_VOLTAGE_SCAN_COLS],
                                         sensorarrayVoltageFrame_t *outFrame);

#ifdef __cplusplus
}
#endif
