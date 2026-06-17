#include "sensorarrayTextProtocol.h"

#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

static size_t sensorarrayTextAppend(char *buffer,
                                    size_t bufferSize,
                                    size_t position,
                                    const char *format,
                                    ...)
{
    if (!buffer || bufferSize == 0u || position >= bufferSize || !format) {
        return position;
    }

    va_list args;
    va_start(args, format);
    int written = vsnprintf(&buffer[position], bufferSize - position, format, args);
    va_end(args);
    if (written < 0 || (size_t)written >= (bufferSize - position)) {
        return bufferSize;
    }
    return position + (size_t)written;
}

static uint32_t sensorarrayTextCrc32(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t index = 0u; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            crc = (crc >> 1u) ^ ((crc & 1u) ? UINT32_C(0xEDB88320) : 0u);
        }
    }
    return ~crc;
}

static int32_t sensorarrayTextCapFixed(const sensorarrayFrame_t *frame, size_t cell)
{
    uint64_t bit = UINT64_C(1) << cell;
    double capPf = frame->capTotalPf[cell];
    if ((frame->capValidMask & bit) == 0u || !isfinite(capPf)) {
        return -1000000;
    }

    double scaled = capPf * 1000000.0;
    if (scaled > (double)INT32_MAX) {
        return INT32_MAX;
    }
    if (scaled < (double)INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)llround(scaled);
}

esp_err_t sensorarrayTextProtocolBuildCapFrame(const sensorarrayFrame_t *frame,
                                                sensorarrayTextPacket_t *outPacket)
{
    if (!frame || !outPacket) {
        return ESP_ERR_INVALID_ARG;
    }

    *outPacket = (sensorarrayTextPacket_t){
        .sequence = frame->sequence,
    };
    uint8_t rows = frame->activeRows >= 1u && frame->activeRows <= SENSORARRAY_MATRIX_ROWS ?
        frame->activeRows : SENSORARRAY_MATRIX_ROWS;
    uint32_t cellCount = (uint32_t)rows * SENSORARRAY_MATRIX_COLS;
    uint32_t invalidCount = cellCount - frame->validCount;
    size_t position = sensorarrayTextAppend(
        outPacket->data,
        sizeof(outPacket->data),
        0u,
        "C,seq=%lu,ts=%llu,rows=%u,cells=%lu,gen=%lu,rid=%lu,rf=%02X,pf=%02X,sf=%02X,"
        "bad=%u/%u/%lu,fmt=pf6,n=%lu\n",
        (unsigned long)frame->sequence,
        (unsigned long long)frame->timestampUs,
        (unsigned)rows,
        (unsigned long)cellCount,
        (unsigned long)frame->configSnapshot.generation,
        (unsigned long)frame->configSnapshot.requestId,
        (unsigned)frame->rowFreshMask,
        (unsigned)frame->primaryFreshMask,
        (unsigned)frame->secondaryFreshMask,
        frame->stale ? 1u : 0u,
        frame->mixedEpoch ? 1u : 0u,
        (unsigned long)invalidCount,
        (unsigned long)cellCount);

    size_t chunkCount = (cellCount + 15u) / 16u;
    for (size_t chunk = 0u; chunk < chunkCount && position < sizeof(outPacket->data); ++chunk) {
        position = sensorarrayTextAppend(outPacket->data,
                                         sizeof(outPacket->data),
                                         position,
                                         "D%u",
                                         (unsigned)chunk);
        size_t valuesInChunk = cellCount - chunk * 16u;
        if (valuesInChunk > 16u) {
            valuesInChunk = 16u;
        }
        for (size_t offset = 0u; offset < valuesInChunk &&
                                position < sizeof(outPacket->data); ++offset) {
            size_t cell = chunk * 16u + offset;
            position = sensorarrayTextAppend(outPacket->data,
                                             sizeof(outPacket->data),
                                             position,
                                             ",%ld",
                                             (long)sensorarrayTextCapFixed(frame, cell));
        }
        position = sensorarrayTextAppend(outPacket->data,
                                         sizeof(outPacket->data),
                                         position,
                                         "\n");
    }

    if (position >= sizeof(outPacket->data)) {
        outPacket->data[sizeof(outPacket->data) - 1u] = '\0';
        outPacket->length = 0u;
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t crc = sensorarrayTextCrc32((const uint8_t *)outPacket->data, position);
    position = sensorarrayTextAppend(outPacket->data,
                                     sizeof(outPacket->data),
                                     position,
                                     "K,seq=%lu,gen=%lu,rid=%lu,crc=%08lX\n",
                                     (unsigned long)frame->sequence,
                                     (unsigned long)frame->configSnapshot.generation,
                                     (unsigned long)frame->configSnapshot.requestId,
                                     (unsigned long)crc);
    if (position >= sizeof(outPacket->data) || position > UINT16_MAX) {
        outPacket->length = 0u;
        return ESP_ERR_INVALID_SIZE;
    }
    outPacket->length = (uint16_t)position;
    return ESP_OK;
}
