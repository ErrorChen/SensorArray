#pragma once

#include "esp_err.h"

#include "sensorarrayAdsMatrixEngine.h"
#include "sensorarrayFdcMatrixEngine.h"
#include "sensorarrayFrame.h"
#include "sensorarrayScanPlan.h"

esp_err_t sensorarrayMixedRowEngineReadFrame(sensorarrayFdcMatrixEngine_t *fdcEngine,
                                             sensorarrayAdsMatrixEngine_t *adsEngine,
                                             const sensorarrayScanPlan_t *plan,
                                             sensorarrayFrame_t *frame);

