#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTypes.h"

void sensorarrayDebugIdleForever(const char *reason);
void sensorarrayDebugHandleFatal(const char *reason, esp_err_t err, const char *stage, uint8_t sColumn, uint8_t dLine);

void sensorarrayDebugRunSelectedMode(sensorarrayState_t *state,
                                     const sensorarrayAdsReadPolicy_t *adsPolicy,
                                     sensorarrayDebugMode_t mode);
