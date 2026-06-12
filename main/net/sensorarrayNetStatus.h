#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t timestampUs;
    uint32_t sequence;
    uint32_t physFpsX100;
    uint32_t cellFreshFpsX100;
    uint32_t emitFpsX100;
    uint32_t rowStepUsAvg;
    uint32_t readyWaitUsAvg;
    uint32_t dataReadUsAvg;
    uint32_t coordinatorResidualUsAvg;
    uint32_t sequenceFallbacks;
    uint32_t sequenceErrors;
    uint32_t nack;
    uint32_t timeout;
    uint32_t recover;
    uint32_t directValidRateX100;
    uint32_t pfStdAvgNano;
    uint32_t pfStdMaxNano;
    int32_t adsAin8RawUv;
    int32_t batteryMv;
    int32_t adsAin9OffsetUv;
    uint32_t heapFree;
    uint32_t heapMin;
    uint32_t logStackHighWater;
    uint8_t rowFreshMask;
    uint8_t primaryFreshMask;
    uint8_t secondaryFreshMask;
    bool stale;
    bool mixed;
    bool precisionPass;
    bool batteryValid;
} sensorarrayNetStatus_t;

esp_err_t sensorarrayNetStatusInit(void);
esp_err_t sensorarrayNetStatusPublish(const sensorarrayNetStatus_t *status);

#ifdef __cplusplus
}
#endif
