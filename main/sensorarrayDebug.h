#pragma once

#include "sensorarrayTypes.h"

void sensorarrayDebugIdleForever(const char *reason);

void sensorarrayDebugRunSelectedMode(sensorarrayState_t *state,
                                     const sensorarrayAdsReadPolicy_t *adsPolicy,
                                     sensorarrayDebugMode_t mode);
