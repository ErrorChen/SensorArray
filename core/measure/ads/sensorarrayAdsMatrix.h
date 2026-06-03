#pragma once

#include "esp_err.h"

#include "sensorarrayFrame.h"
#include "sensorarrayScanPlan.h"
#include "sensorarrayTypes.h"

typedef struct {
    sensorarrayState_t *state;
} sensorarrayAdsMatrixEngine_t;

esp_err_t sensorarrayAdsMatrixEngineInit(sensorarrayAdsMatrixEngine_t *engine,
                                         sensorarrayState_t *state);
esp_err_t sensorarrayAdsMatrixEngineReadFrame(sensorarrayAdsMatrixEngine_t *engine,
                                              const sensorarrayScanPlan_t *plan,
                                              sensorarrayFrame_t *frame);

