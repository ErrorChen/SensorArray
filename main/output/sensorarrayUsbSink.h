#pragma once

#include <stdint.h>

#include "esp_err.h"

#include "sensorarrayTextProtocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSORARRAY_USB_STREAM_DEBUG_DATA_EVERY 50u
#define SENSORARRAY_USB_STREAM_DIAG_EVERY 15u
#define SENSORARRAY_USB_STREAM_DATA_EVERY_MAX 10000u
#define SENSORARRAY_USB_STREAM_DIAG_EVERY_MAX 10000u

typedef enum {
    SENSORARRAY_USB_STREAM_DEBUG = 0,
    SENSORARRAY_USB_STREAM_FULL,
} sensorarrayUsbStreamMode_t;

typedef struct {
    sensorarrayUsbStreamMode_t mode;
    uint32_t dataEvery;
    uint32_t diagEvery;
} sensorarrayUsbStreamProfile_t;

typedef struct {
    uint32_t sentPackets;
    uint32_t droppedPackets;
    uint32_t blockedCount;
    uint64_t sentBytes;
    uint64_t writeUsTotal;
    uint32_t writeUsMax;
    uint32_t queueDepth;
    uint32_t queueDepthMax;
    uint32_t taskConfiguredBytes;
    uint32_t taskMinimumRemainingBytes;
} sensorarrayUsbSinkStats_t;

esp_err_t sensorarrayUsbSinkInit(void);
esp_err_t sensorarrayUsbSinkPublish(const sensorarrayTextPacket_t *packet);
void sensorarrayUsbSinkGetStats(sensorarrayUsbSinkStats_t *outStats);
esp_err_t sensorarrayUsbSinkSetStreamProfile(sensorarrayUsbStreamMode_t mode,
                                             uint32_t dataEvery,
                                             uint32_t diagEvery);
sensorarrayUsbStreamProfile_t sensorarrayUsbSinkGetStreamProfile(void);
bool sensorarrayUsbSinkShouldEmitData(uint32_t sequence);
const char *sensorarrayUsbStreamModeName(sensorarrayUsbStreamMode_t mode);

#ifdef __cplusplus
}
#endif
