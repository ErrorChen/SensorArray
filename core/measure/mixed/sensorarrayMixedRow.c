#include "sensorarrayMixedRow.h"

esp_err_t sensorarrayMixedRowEngineReadFrame(sensorarrayFdcMatrixEngine_t *fdcEngine,
                                             sensorarrayAdsMatrixEngine_t *adsEngine,
                                             const sensorarrayScanPlan_t *plan,
                                             sensorarrayFrame_t *frame)
{
    (void)adsEngine;
    return sensorarrayFdcMatrixEngineReadFrame(fdcEngine, plan, frame);
}
