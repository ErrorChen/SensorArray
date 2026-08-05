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
    sensorarrayScanPlanBuildRows(plan, SENSORARRAY_CELL_OP_SKIP);
    if (!plan) {
        return;
    }

    for (uint8_t row = 1u; row <= plan->rowCount; ++row) {
        sensorarrayRowPlan_t *rowPlan = &plan->rows[row - 1u];
        rowPlan->cells[0].opKind = SENSORARRAY_CELL_OP_ADS_RESISTANCE;
        rowPlan->cells[1].opKind = SENSORARRAY_CELL_OP_ADS_RESISTANCE;
        rowPlan->cells[2].opKind = SENSORARRAY_CELL_OP_FDC_CAP;
        rowPlan->cells[3].opKind = SENSORARRAY_CELL_OP_FDC_CAP;
        rowPlan->cells[4].opKind = SENSORARRAY_CELL_OP_FDC_CAP;
        rowPlan->cells[5].opKind = SENSORARRAY_CELL_OP_FDC_CAP;
        rowPlan->cells[6].opKind = SENSORARRAY_CELL_OP_ADS_PIEZO;
        rowPlan->cells[7].opKind = SENSORARRAY_CELL_OP_FDC_CAP;
    }
}
