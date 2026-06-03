#pragma once

#include <stdint.h>

typedef struct {
    uint16_t rCount[4];
    uint16_t settleCount[4];
    uint16_t clockDiv[4];
    uint32_t effectiveFclkHz;
} sensorarrayFdcRowCache_t;

uint32_t sensorarrayFdcEstimateAutoscanReadyTimeoutUs(const sensorarrayFdcRowCache_t *rowCache,
                                                      uint8_t requiredUnreadMask,
                                                      uint32_t *outEstimatedRoundUs);

