#include "sensorarrayScanPlan.h"

#include <string.h>

#include "sensorarrayScanConfig.h"
#include "sensorarrayTypes.h"

static void sensorarrayScanPlanBuildRows(sensorarrayScanPlan_t *plan,
                                         sensorarrayCellOpKind_t defaultOp)
{
    if (!plan) {
        return;
    }

    memset(plan, 0, sizeof(*plan));
    sensorarrayFrameConfigSnapshot_t snapshot = sensorarrayScanConfigGetFrameSnapshot();
    uint8_t activeRows = snapshot.rows;
    if (activeRows < 1u || activeRows > SENSORARRAY_MATRIX_ROWS) {
        activeRows = SENSORARRAY_MATRIX_ROWS;
        snapshot = (sensorarrayFrameConfigSnapshot_t){
            .rows = activeRows,
            .cells = SENSORARRAY_MATRIX_CELL_COUNT,
            .rowMask = 0xFFu,
            .generation = snapshot.generation,
            .requestId = snapshot.requestId,
        };
    }
    plan->rowCount = activeRows;
    plan->configSnapshot = snapshot;
    for (uint8_t row = 0u; row < SENSORARRAY_MATRIX_ROWS; ++row) {
        plan->rowModes[row] = defaultOp == SENSORARRAY_CELL_OP_FDC_CAP ?
            SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE :
            (defaultOp == SENSORARRAY_CELL_OP_ADS_VOLTAGE ?
                SENSORARRAY_MEASUREMENT_MODE_VOLTAGE :
                SENSORARRAY_MEASUREMENT_MODE_RESISTANCE);
    }
    for (uint8_t row = 1u; row <= activeRows; ++row) {
        sensorarrayRowPlan_t *rowPlan = &plan->rows[row - 1u];
        rowPlan->row = row;
        rowPlan->cellCount = SENSORARRAY_MATRIX_COLS;
        for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
            rowPlan->cells[dLine - 1u] = (sensorarrayCellOp_t){
                .row = row,
                .dLine = dLine,
                .opKind = defaultOp,
            };
        }
    }
}

void sensorarrayScanPlanBuildDefaultFdcMatrix(sensorarrayScanPlan_t *plan)
{
    sensorarrayScanPlanBuildRows(plan, SENSORARRAY_CELL_OP_FDC_CAP);
}

void sensorarrayScanPlanBuildAdsMatrix(sensorarrayScanPlan_t *plan,
                                       sensorarrayMeasurementMode_t mode)
{
    sensorarrayCellOpKind_t op = mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ?
        SENSORARRAY_CELL_OP_ADS_VOLTAGE : SENSORARRAY_CELL_OP_ADS_RESISTANCE;
    sensorarrayScanPlanBuildRows(plan, op);
}

void sensorarrayScanPlanBuildMixedExample(sensorarrayScanPlan_t *plan)
{
    static const sensorarrayMeasurementMode_t example[SENSORARRAY_MATRIX_ROWS] = {
        SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
    };
    sensorarrayScanPlanBuildRowProfile(plan, example, 0u, 0u);
}

void sensorarrayScanPlanBuildRowProfile(
    sensorarrayScanPlan_t *plan,
    const sensorarrayMeasurementMode_t rowModes[8],
    uint32_t profileGeneration,
    uint32_t profileRequestId)
{
    sensorarrayScanPlanBuildRows(plan, SENSORARRAY_CELL_OP_SKIP);
    if (!plan) {
        return;
    }
    if (!rowModes) {
        sensorarrayScanPlanBuildDefaultFdcMatrix(plan);
        return;
    }
    memcpy(plan->rowModes, rowModes, sizeof(plan->rowModes));
    plan->rowProfileGeneration = profileGeneration;
    plan->rowProfileRequestId = profileRequestId;
    for (uint8_t row = 1u; row <= plan->rowCount; ++row) {
        sensorarrayRowPlan_t *rowPlan = &plan->rows[row - 1u];
        sensorarrayCellOpKind_t op = rowModes[row - 1u] ==
            SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ? SENSORARRAY_CELL_OP_FDC_CAP :
            (rowModes[row - 1u] == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ?
                SENSORARRAY_CELL_OP_ADS_VOLTAGE : SENSORARRAY_CELL_OP_ADS_RESISTANCE);
        for (uint8_t cell = 0u; cell < SENSORARRAY_MATRIX_COLS; ++cell) {
            rowPlan->cells[cell].opKind = op;
        }
    }
}
