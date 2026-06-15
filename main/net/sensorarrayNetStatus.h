#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTextProtocol.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t wifiSentPackets;
    uint32_t wifiDroppedPackets;
    uint32_t wifiBlockedCount;
    uint64_t wifiSentBytes;
    uint32_t bleSentPackets;
    uint32_t bleDroppedPackets;
    uint32_t bleBlockedCount;
    uint64_t bleSentBytes;
    uint32_t queueDepth;
    uint32_t queueDepthMax;
} sensorarrayNetSinkStats_t;

esp_err_t sensorarrayNetStatusInit(void);
esp_err_t sensorarrayNetTextPublish(const sensorarrayTextPacket_t *packet,
                                    bool allowBle);
void sensorarrayNetGetSinkStats(sensorarrayNetSinkStats_t *outStats);

#ifdef __cplusplus
}
#endif
