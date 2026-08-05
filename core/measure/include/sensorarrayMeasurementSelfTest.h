#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t checks;
    uint32_t failureLine;
} sensorarrayMeasurementSelfTestResult_t;

/* Hardware-independent safety test. It is run once before any production
 * matrix route is applied, so a broken state transition or arithmetic guard
 * leaves startup in the passive safe route. */
bool sensorarrayMeasurementSelfTestRun(
    sensorarrayMeasurementSelfTestResult_t *outResult);
