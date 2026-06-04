#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayFrame.h"
#include "sensorarrayScanPlan.h"
#include "sensorarrayTypes.h"
#include "sensorarrayFdcSweep.h"

typedef struct {
    sensorarrayState_t *state;
    bool bootSweepOk;
    bool diagnosticMode;
    uint32_t allInvalidSequence;
    sensorarrayFdcBootSummary_t bootSummary;
} sensorarrayFdcMatrixEngine_t;

esp_err_t sensorarrayFdcMatrixEngineInit(sensorarrayFdcMatrixEngine_t *engine,
                                         sensorarrayState_t *state);
esp_err_t sensorarrayFdcMatrixEngineRunBootSweep(sensorarrayFdcMatrixEngine_t *engine,
                                                 const sensorarrayScanPlan_t *plan,
                                                 sensorarrayFdcBootSummary_t *outSummary);
esp_err_t sensorarrayFdcMatrixEngineReadFrame(sensorarrayFdcMatrixEngine_t *engine,
                                              const sensorarrayScanPlan_t *plan,
                                              sensorarrayFrame_t *frame);
esp_err_t sensorarrayFdcMatrixEngineRunFullRescue(sensorarrayFdcMatrixEngine_t *engine,
                                                  const char *reason);
void sensorarrayFdcMatrixEngineSetDiagnosticMode(sensorarrayFdcMatrixEngine_t *engine,
                                                 bool enabled);
bool sensorarrayFdcMatrixEngineDiagnosticMode(const sensorarrayFdcMatrixEngine_t *engine);
const sensorarrayFdcBootSummary_t *sensorarrayFdcMatrixEngineBootSummary(const sensorarrayFdcMatrixEngine_t *engine);
sensorarrayState_t *sensorarrayFdcMatrixEngineState(sensorarrayFdcMatrixEngine_t *engine);
