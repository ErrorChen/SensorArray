#pragma once

#include "sensorarrayTypes.h"

void sensorarrayDebugRunAdsS1D1OnlyModeImpl(sensorarrayState_t *state,
                                            const sensorarrayAdsReadPolicy_t *adsPolicy);

void sensorarrayDebugRunSingleResistorS1D1ModeImpl(sensorarrayState_t *state,
                                                   const sensorarrayAdsReadPolicy_t *adsPolicy);

void sensorarrayDebugRunS1D1StaticResistorDebugImpl(sensorarrayState_t *state,
                                                    const sensorarrayAdsReadPolicy_t *adsPolicy);

void sensorarrayDebugRunS1D1ForceAdsHoldModeImpl(sensorarrayState_t *state,
                                                 const sensorarrayAdsReadPolicy_t *adsPolicy);
