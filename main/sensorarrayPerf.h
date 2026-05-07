#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t framesScanned;
    uint32_t framesQueued;
    uint32_t framesOutput;
    uint32_t framesDropped;
    uint32_t outputDecimatedFrames;
    uint32_t pointsScanned;
    uint32_t spiTransactionCount;
    uint32_t gainChangeCount;

    uint64_t scanTotalUs;
    uint32_t scanMaxUs;

    uint64_t routeTotalUs;
    uint64_t inpmuxWriteTotalUs;
    uint64_t drdyWaitTotalUs;
    uint64_t adcReadTotalUs;
    uint64_t spiTransferTotalUs;
    uint64_t queueSendTotalUs;
    uint64_t usbWriteTotalUs;

    uint32_t routeMaxUs;
    uint32_t inpmuxWriteMaxUs;
    uint32_t drdyWaitMaxUs;
    uint32_t adcReadMaxUs;
    uint32_t spiTransferMaxUs;
    uint32_t queueSendMaxUs;
    uint32_t usbWriteMaxUs;
} sensorarrayPerfCounters_t;

void sensorarrayPerfAddSample(uint64_t *totalUs, uint32_t *maxUs, uint32_t sampleUs);
uint32_t sensorarrayPerfAvgU32(uint64_t totalUs, uint32_t count);

#ifdef __cplusplus
}
#endif
