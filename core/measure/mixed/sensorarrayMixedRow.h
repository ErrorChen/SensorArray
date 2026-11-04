#pragma once

#include "esp_err.h"

#include "sensorarrayAdsMatrix.h"
#include "sensorarrayFdcMatrix.h"
#include "sensorarrayFrame.h"
#include "sensorarrayScanPlan.h"

/* 本轮新增: complete-frame boundary only, before any FDC/ADS segment. */
esp_err_t sensorarrayMixedRowPrepareRailAtBoundary(
    sensorarrayAdsMatrixEngine_t *adsEngine, const sensorarrayScanPlan_t *plan,
    uint32_t frameSequence, esp_err_t *outCleanupError);

esp_err_t sensorarrayMixedRowEngineReadFrame(sensorarrayFdcMatrixEngine_t *fdcEngine,
                                             sensorarrayAdsMatrixEngine_t *adsEngine,
                                             const sensorarrayScanPlan_t *plan,
                                             sensorarrayFrame_t *frame,
                                             sensorarrayFrame_t *segmentWorkspace);
