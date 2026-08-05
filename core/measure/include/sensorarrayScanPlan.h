#pragma once

#include <stdint.h>

#include "sensorarrayScanConfig.h"
#include "sensorarrayMeasurementMode.h"

typedef enum {
    SENSORARRAY_CELL_OP_SKIP = 0,
    SENSORARRAY_CELL_OP_FDC_CAP,
    SENSORARRAY_CELL_OP_ADS_VOLTAGE,
    SENSORARRAY_CELL_OP_ADS_RESISTANCE,
    SENSORARRAY_CELL_OP_ADS_PIEZO,
} sensorarrayCellOpKind_t;

typedef struct {
    uint8_t row;
    uint8_t dLine;
    sensorarrayCellOpKind_t opKind;
} sensorarrayCellOp_t;

typedef struct {
    uint8_t row;
    sensorarrayCellOp_t cells[8];
    uint8_t cellCount;
} sensorarrayRowPlan_t;

typedef struct {
    sensorarrayRowPlan_t rows[8];
    uint8_t rowCount;
    sensorarrayFrameConfigSnapshot_t configSnapshot;
} sensorarrayScanPlan_t;

void sensorarrayScanPlanBuildDefaultFdcMatrix(sensorarrayScanPlan_t *plan);
void sensorarrayScanPlanBuildAdsMatrix(sensorarrayScanPlan_t *plan,
                                       sensorarrayMeasurementMode_t mode);
void sensorarrayScanPlanBuildMixedExample(sensorarrayScanPlan_t *plan);
