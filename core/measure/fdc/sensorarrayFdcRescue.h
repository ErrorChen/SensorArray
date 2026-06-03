#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayFdcMatrixEngine.h"
#include "sensorarrayFrame.h"

typedef struct {
    uint32_t allInvalidSequence;
} sensorarrayFdcRescueContext_t;

void sensorarrayFdcRescueReset(sensorarrayFdcRescueContext_t *ctx);
esp_err_t sensorarrayFdcRescueTick(sensorarrayFdcMatrixEngine_t *engine,
                                   const sensorarrayFrame_t *frame,
                                   sensorarrayFdcRescueContext_t *ctx);

