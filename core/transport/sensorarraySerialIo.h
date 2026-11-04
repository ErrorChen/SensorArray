#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

enum {
    SENSORARRAY_SERIAL_MEMORY_UNAVAILABLE = 0,
    SENSORARRAY_SERIAL_MEMORY_INTERNAL = 1,
    SENSORARRAY_SERIAL_MEMORY_PSRAM = 2,
};

/* Added in this repair: local queue/driver observations, never PC receipt. */
typedef struct {
    uint32_t sinkQueued;
    uint32_t sinkDropped;
    uint32_t priorityQueued;
    uint32_t partialWrite;
    uint32_t timeout;
    uint32_t driverDrained;
    uint32_t rxOversize;
    uint32_t lineDropped;
    uint32_t lineOversize;
    uint64_t discardBytes;
    uint32_t taskStackMinBytes;
    uint32_t taskStackConfiguredBytes;
    uint32_t normalBufferBytes;
    uint8_t normalBufferMemory;
    uint64_t driverAcceptedBytes;
    uint64_t rxBytes;
    int64_t lastRxUs;
    int64_t lastTxUs;
    bool driverInstalled;
    bool sofPresent;
    bool stdioBound;
    esp_err_t initError;
} sensorarraySerialIoStats_t;

esp_err_t sensorarraySerialIoInit(void);
esp_err_t sensorarraySerialIoQueue(const char *data, size_t length, bool priority);
int sensorarraySerialIoRead(void *buffer, size_t length);
void sensorarraySerialIoNoteOversize(void);
void sensorarraySerialIoGetStats(sensorarraySerialIoStats_t *outStats);
