#include "sensorarrayPerf.h"

void sensorarrayPerfAddSample(uint64_t *totalUs, uint32_t *maxUs, uint32_t sampleUs)
{
    if (totalUs) {
        *totalUs += sampleUs;
    }
    if (maxUs && sampleUs > *maxUs) {
        *maxUs = sampleUs;
    }
}

uint32_t sensorarrayPerfAvgU32(uint64_t totalUs, uint32_t count)
{
    return (count == 0u) ? 0u : (uint32_t)(totalUs / (uint64_t)count);
}
