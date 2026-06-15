#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTextProtocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t sentPackets;
    uint32_t droppedPackets;
    uint32_t blockedCount;
    uint64_t sentBytes;
    uint64_t writeUsTotal;
    uint32_t writeUsMax;
    uint32_t queueDepth;
    uint32_t queueDepthMax;
} sensorarrayUsbSinkStats_t;

esp_err_t sensorarrayUsbSinkInit(void);
esp_err_t sensorarrayUsbSinkPublish(const sensorarrayTextPacket_t *packet);
void sensorarrayUsbSinkGetStats(sensorarrayUsbSinkStats_t *outStats);

#ifdef __cplusplus
}
#endif
