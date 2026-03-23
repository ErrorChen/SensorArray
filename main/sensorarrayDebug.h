#pragma once

#include "sensorarrayTypes.h"

void sensorarrayDebugIdleForever(const char *reason);

void sensorarrayDebugRunAdsS1D1OnlyMode(sensorarrayState_t *state,
                                        const sensorarrayAdsReadPolicy_t *adsPolicy);

void sensorarrayDebugRunS1D1StaticResistorDebug(sensorarrayState_t *state,
                                                const sensorarrayAdsReadPolicy_t *adsPolicy);

void sensorarrayDebugRunS1D1ForceAdsHoldMode(sensorarrayState_t *state,
                                             const sensorarrayAdsReadPolicy_t *adsPolicy);

void sensorarrayDebugRunSelectedMode(sensorarrayState_t *state,
                                     const sensorarrayAdsReadPolicy_t *adsPolicy);
