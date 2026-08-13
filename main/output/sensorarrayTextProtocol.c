#include "sensorarrayTextProtocol.h"

#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sensorarrayRowModeProfile.h"

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

static uint8_t sensorarrayTextMixedRowMask(uint64_t mask, uint8_t row)
{
    return (uint8_t)((mask >> ((size_t)(row - 1u) * SENSORARRAY_MATRIX_COLS)) & 0xFFu);
}

static char sensorarrayTextMixedModeChar(sensorarrayMeasurementMode_t mode)
{
    return mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ? 'C' :
        (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ? 'V' : 'R');
}

static int64_t sensorarrayTextMixedValue(const sensorarrayFrame_t *frame,
                                         sensorarrayMeasurementMode_t mode,
                                         size_t index)
{
    if (mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
        return sensorarrayTextCapFixed(frame, index);
    }
    uint64_t bit = UINT64_C(1) << index;
    return (frame->measurement.validMask & bit) != 0u ?
        frame->measurement.valuesFixed[index] : INT64_MIN;
}

esp_err_t sensorarrayTextProtocolBuildMixedFrame(
    const sensorarrayFrame_t *frame,
    sensorarrayTextPacket_t *outPacket)
{
    if (!frame || !outPacket || !frame->mixedProfile || frame->activeRows < 1u ||
        frame->activeRows > SENSORARRAY_MATRIX_ROWS) {
        return ESP_ERR_INVALID_ARG;
    }
    *outPacket = (sensorarrayTextPacket_t){.sequence = frame->sequence};
    char profile[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u];
    for (uint8_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        sensorarrayMeasurementMode_t mode = frame->rowMode[row];
        profile[row] = sensorarrayTextMixedModeChar(mode);
    }
    profile[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH] = '\0';
    size_t position = sensorarrayTextAppend(
        outPacket->data, sizeof(outPacket->data), 0u,
        "M,seq=%lu,ts=%llu,rows=%u,cells=%u,rgen=%lu,rrid=%lu,pgen=%lu,prid=%lu,profile=%s,fmt=mix1\n",
        (unsigned long)frame->sequence,
        (unsigned long long)frame->timestampUs,
        (unsigned)frame->activeRows,
        (unsigned)(frame->activeRows * SENSORARRAY_MATRIX_COLS),
        (unsigned long)frame->configSnapshot.generation,
        (unsigned long)frame->configSnapshot.requestId,
        (unsigned long)frame->rowProfileGeneration,
        (unsigned long)frame->rowProfileRequestId,
        profile);
    for (uint8_t row = 1u; row <= frame->activeRows && position < sizeof(outPacket->data); ++row) {
        sensorarrayMeasurementMode_t mode = frame->rowMode[row - 1u];
        uint64_t valid = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
            frame->capValidMask : frame->measurement.validMask;
        uint64_t fresh = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
            frame->freshMask : frame->measurement.freshMask;
        uint64_t errors = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
            frame->errorMask : frame->measurement.errorMask;
        const char *unit = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ? "pF" :
            (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ? "V" : "ohm");
        const char *fmt = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ? "pf6" :
            (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ? "uv-x" : "mohm-x");
        position = sensorarrayTextAppend(
            outPacket->data, sizeof(outPacket->data), position,
            "MR,s=%u,m=%c,unit=%s,scale=%d,valid=%02X,fresh=%02X,error=%02X,fmt=%s,D=",
            (unsigned)row, sensorarrayTextMixedModeChar(mode), unit,
            (int)frame->rowScale[row - 1u],
            (unsigned)sensorarrayTextMixedRowMask(valid, row),
            (unsigned)sensorarrayTextMixedRowMask(fresh, row),
            (unsigned)sensorarrayTextMixedRowMask(errors, row), fmt);
        for (uint8_t dLine = 1u; dLine <= SENSORARRAY_MATRIX_COLS; ++dLine) {
            size_t index = sensorarrayMatrixIndex(row, dLine);
            uint64_t bit = UINT64_C(1) << index;
            int64_t value = sensorarrayTextMixedValue(frame, mode, index);
            if ((valid & bit) == 0u || value == INT64_MIN) {
                uint8_t error = mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE ?
                    SENSORARRAY_CELL_ERROR_UNSUPPORTED : frame->measurement.errorReason[index];
                position = sensorarrayTextAppend(outPacket->data,
                                                 sizeof(outPacket->data), position,
                                                 dLine == 1u ? "X%02X" : ",X%02X",
                                                 (unsigned)error);
            } else {
                position = sensorarrayTextAppend(outPacket->data,
                                                 sizeof(outPacket->data), position,
                                                 dLine == 1u ? "%lld" : ",%lld",
                                                 (long long)value);
            }
        }
        position = sensorarrayTextAppend(outPacket->data,
                                         sizeof(outPacket->data), position, "\n");
    }
    if (position >= sizeof(outPacket->data)) {
        outPacket->length = 0u;
        return ESP_ERR_INVALID_SIZE;
    }
    uint32_t crc = sensorarrayTextCrc32((const uint8_t *)outPacket->data, position);
    position = sensorarrayTextAppend(outPacket->data, sizeof(outPacket->data), position,
                                     "K,seq=%lu,rgen=%lu,rrid=%lu,pgen=%lu,prid=%lu,crc=%08lX\n",
                                     (unsigned long)frame->sequence,
                                     (unsigned long)frame->configSnapshot.generation,
                                     (unsigned long)frame->configSnapshot.requestId,
                                     (unsigned long)frame->rowProfileGeneration,
                                     (unsigned long)frame->rowProfileRequestId,
                                     (unsigned long)crc);
    if (position >= sizeof(outPacket->data) || position > UINT16_MAX) {
        outPacket->length = 0u;
        return ESP_ERR_INVALID_SIZE;
    }
    outPacket->length = (uint16_t)position;
    return ESP_OK;
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

static uint32_t sensorarrayTextCountBits64(uint64_t value)
{
    uint32_t count = 0u;
    while (value != 0u) {
        value &= value - 1u;
        count++;
    }
    return count;
}

esp_err_t sensorarrayTextProtocolBuildMeasurementFrame(
    const sensorarrayFrame_t *frame,
    sensorarrayTextPacket_t *outPacket)
{
    if (!frame || !outPacket ||
        (frame->measurement.mode != SENSORARRAY_MEASUREMENT_MODE_VOLTAGE &&
         frame->measurement.mode != SENSORARRAY_MEASUREMENT_MODE_RESISTANCE)) {
        return ESP_ERR_INVALID_ARG;
    }

    *outPacket = (sensorarrayTextPacket_t){.sequence = frame->sequence};
    uint8_t rows = frame->activeRows >= 1u &&
                   frame->activeRows <= SENSORARRAY_MATRIX_ROWS ?
        frame->activeRows : SENSORARRAY_MATRIX_ROWS;
    uint32_t cellCount = (uint32_t)rows * SENSORARRAY_MATRIX_COLS;
    uint64_t activeMask = cellCount == 64u ? UINT64_MAX :
        ((UINT64_C(1) << cellCount) - 1u);
    uint64_t validMask = frame->measurement.validMask & activeMask;
    uint64_t freshMask = frame->measurement.freshMask & activeMask;
    uint64_t errorMask = frame->measurement.errorMask & activeMask;
    char tag = frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ?
        'V' : 'R';
    const char *format = frame->measurement.mode ==
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ? "uv" : "mohm";
    size_t position = sensorarrayTextAppend(
        outPacket->data,
        sizeof(outPacket->data),
        0u,
        "%c,seq=%lu,ts=%llu,rows=%u,cells=%lu,gen=%lu,rid=%lu,mode=%s,unit=%s,scale=%d,valid=%016llX,fresh=%016llX,error=%016llX,ref=%s,rail=%u,age=%lu,avdd=%ld,avss=%ld,vexc=%ld,rref=%lu,dur=%llu,tr=%llu,gc=%lu,ov=%lu,aa=%lu,fb=%lu,ir=%lu,to=%lu,st=%lu,spi=%lu,fmt=%s-x,n=%lu,bad=%lu\n",
        tag,
        (unsigned long)frame->sequence,
        (unsigned long long)frame->timestampUs,
        (unsigned)rows,
        (unsigned long)cellCount,
        (unsigned long)frame->measurement.modeGeneration,
        (unsigned long)frame->measurement.modeRequestId,
        sensorarrayMeasurementModeName(frame->measurement.mode),
        sensorarrayMeasurementUnitName(frame->measurement.unit),
        (int)frame->measurement.decimalScale,
        (unsigned long long)validMask,
        (unsigned long long)freshMask,
        (unsigned long long)errorMask,
        sensorarrayAdsReferenceSourceName(frame->measurement.referenceSource),
        frame->measurement.railValid ? 1u : 0u,
        (unsigned long)frame->measurement.railAgeFrames,
        (long)frame->measurement.avddUv,
        (long)frame->measurement.avssUv,
        (long)frame->measurement.matrixReferenceUv,
        (unsigned long)frame->measurement.referenceResistorOhms,
        (unsigned long long)frame->measurement.frameDurationUs,
        (unsigned long long)frame->measurement.transitionDurationUs,
        (unsigned long)frame->measurement.gainChangeCount,
        (unsigned long)frame->measurement.overrangeCount,
        (unsigned long)frame->measurement.autorangeAttemptCount,
        (unsigned long)frame->measurement.autorangeFallbackCount,
        (unsigned long)frame->measurement.ioRetryCount,
        (unsigned long)frame->measurement.drdyTimeoutCount,
        (unsigned long)frame->measurement.staleCount,
        (unsigned long)frame->measurement.spiErrorCount,
        format,
        (unsigned long)cellCount,
        (unsigned long)(cellCount - sensorarrayTextCountBits64(validMask)));

    size_t chunkCount = (cellCount + 15u) / 16u;
    for (size_t chunk = 0u; chunk < chunkCount &&
                           position < sizeof(outPacket->data); ++chunk) {
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
            uint64_t bit = UINT64_C(1) << cell;
            int64_t value = frame->measurement.valuesFixed[cell];
            if ((validMask & bit) == 0u ||
                value == SENSORARRAY_MEASUREMENT_INVALID_FIXED) {
                position = sensorarrayTextAppend(
                    outPacket->data,
                    sizeof(outPacket->data),
                    position,
                    ",X%02X",
                    (unsigned)frame->measurement.errorReason[cell]);
            } else {
                if (value < -SENSORARRAY_TEXT_MEASUREMENT_VALUE_MAX ||
                    value > SENSORARRAY_TEXT_MEASUREMENT_VALUE_MAX) {
                    outPacket->length = 0u;
                    return ESP_ERR_INVALID_SIZE;
                }
                position = sensorarrayTextAppend(outPacket->data,
                                                 sizeof(outPacket->data),
                                                 position,
                                                 ",%lld",
                                                 (long long)value);
            }
        }
        position = sensorarrayTextAppend(outPacket->data,
                                         sizeof(outPacket->data),
                                         position,
                                         "\n");
    }

    /* Two hex digits per cell keep the final PGA visible without spending a
     * second integer token per cell. Values are literal gains (01..20 hex). */
    for (size_t chunk = 0u; chunk < chunkCount &&
                           position < sizeof(outPacket->data); ++chunk) {
        position = sensorarrayTextAppend(outPacket->data,
                                         sizeof(outPacket->data),
                                         position,
                                         "P%u,",
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
                                             "%02X",
                                             (unsigned)frame->measurement.pgaGain[cell]);
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
                                     (unsigned long)frame->measurement.modeGeneration,
                                     (unsigned long)frame->measurement.modeRequestId,
                                     (unsigned long)crc);
    if (position >= sizeof(outPacket->data) || position > UINT16_MAX) {
        outPacket->length = 0u;
        return ESP_ERR_INVALID_SIZE;
    }
    outPacket->length = (uint16_t)position;
    return ESP_OK;
}

esp_err_t sensorarrayTextProtocolBuildFrame(const sensorarrayFrame_t *frame,
                                            sensorarrayTextPacket_t *outPacket)
{
    if (!frame || !outPacket) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frame->mixedProfile) {
        return sensorarrayTextProtocolBuildMixedFrame(frame, outPacket);
    }
    if (frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE ||
        frame->measurement.mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE) {
        return sensorarrayTextProtocolBuildMeasurementFrame(frame, outPacket);
    }
    return sensorarrayTextProtocolBuildCapFrame(frame, outPacket);
}

#define SENSORARRAY_PROTOCOL_CHECK(condition) \
    do {                                      \
        checks++;                             \
        if (!(condition)) {                   \
            if (outChecks) {                  \
                *outChecks = checks;          \
            }                                 \
            return false;                     \
        }                                     \
    } while (0)

bool sensorarrayTextProtocolSelfTest(sensorarrayFrame_t *scratchFrame,
                                     uint32_t *outChecks)
{
    if (!scratchFrame) {
        return false;
    }
    uint32_t checks = 0u;
    sensorarrayTextPacket_t packet;
    static const uint8_t crcGolden[] = "123456789";
    SENSORARRAY_PROTOCOL_CHECK(sensorarrayTextCrc32(
        crcGolden, sizeof(crcGolden) - 1u) == UINT32_C(0xCBF43926));

    memset(scratchFrame, 0, sizeof(*scratchFrame));
    scratchFrame->activeRows = 1u;
    scratchFrame->validCount = 8u;
    scratchFrame->capValidMask = UINT64_C(0xFF);
    for (size_t index = 0u; index < 8u; ++index) {
        scratchFrame->capTotalPf[index] = (double)index + 0.25;
    }
    SENSORARRAY_PROTOCOL_CHECK(
        sensorarrayTextProtocolBuildCapFrame(scratchFrame, &packet) == ESP_OK);
    SENSORARRAY_PROTOCOL_CHECK(packet.length > 0u &&
                               strncmp(packet.data, "C,", 2u) == 0 &&
                               strstr(packet.data, "rows=1,cells=8") != NULL &&
                               strstr(packet.data, "fmt=pf6") != NULL);

    const uint8_t rowCases[] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
    for (size_t rowCase = 0u; rowCase < sizeof(rowCases); ++rowCase) {
        uint8_t rows = rowCases[rowCase];
        uint8_t cells = sensorarrayMeasurementCellCount(rows);
        memset(scratchFrame, 0, sizeof(*scratchFrame));
        scratchFrame->activeRows = rows;
        scratchFrame->measurement.mode = SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
        scratchFrame->measurement.unit = SENSORARRAY_MEASUREMENT_UNIT_VOLT;
        scratchFrame->measurement.decimalScale = -6;
        scratchFrame->measurement.referenceSource =
            SENSORARRAY_ADS_REFERENCE_AVDD_AVSS;
        scratchFrame->measurement.modeGeneration = 2u;
        scratchFrame->measurement.modeRequestId = 7u;
        scratchFrame->measurement.railValid = true;
        scratchFrame->measurement.validMask = cells == 64u ? UINT64_MAX :
            ((UINT64_C(1) << cells) - 1u);
        scratchFrame->measurement.freshMask =
            scratchFrame->measurement.validMask;
        for (size_t index = 0u; index < cells; ++index) {
            scratchFrame->measurement.valuesFixed[index] = (int64_t)index - 4LL;
            scratchFrame->measurement.pgaGain[index] = 1u;
        }
        scratchFrame->measurement.validMask &= ~UINT64_C(1);
        scratchFrame->measurement.errorMask |= UINT64_C(1);
        scratchFrame->measurement.valuesFixed[0] =
            SENSORARRAY_MEASUREMENT_INVALID_FIXED;
        scratchFrame->measurement.errorReason[0] = SENSORARRAY_CELL_ERROR_OPEN;
        SENSORARRAY_PROTOCOL_CHECK(
            sensorarrayTextProtocolBuildMeasurementFrame(scratchFrame, &packet) ==
                ESP_OK);
        char expectedCells[32];
        (void)snprintf(expectedCells, sizeof(expectedCells),
                       "rows=%u,cells=%u", (unsigned)rows, (unsigned)cells);
        SENSORARRAY_PROTOCOL_CHECK(packet.length > 0u &&
                                   packet.length <= SENSORARRAY_TEXT_PACKET_MAX &&
                                   packet.data[0] == 'V' &&
                                   strstr(packet.data, expectedCells) != NULL &&
                                   strstr(packet.data, ",X0D") != NULL);
    }

    memset(scratchFrame, 0, sizeof(*scratchFrame));
    scratchFrame->activeRows = 8u;
    scratchFrame->measurement.mode = SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    scratchFrame->measurement.unit = SENSORARRAY_MEASUREMENT_UNIT_OHM;
    scratchFrame->measurement.decimalScale = -3;
    scratchFrame->measurement.referenceSource = SENSORARRAY_ADS_REFERENCE_INTERNAL;
    scratchFrame->measurement.railValid = true;
    scratchFrame->measurement.validMask = UINT64_MAX;
    scratchFrame->measurement.freshMask = UINT64_MAX;
    scratchFrame->measurement.frameDurationUs = UINT64_MAX;
    scratchFrame->measurement.transitionDurationUs = UINT64_MAX;
    scratchFrame->measurement.gainChangeCount = UINT32_MAX;
    scratchFrame->measurement.overrangeCount = UINT32_MAX;
    scratchFrame->measurement.autorangeAttemptCount = UINT32_MAX;
    scratchFrame->measurement.autorangeFallbackCount = UINT32_MAX;
    scratchFrame->measurement.ioRetryCount = UINT32_MAX;
    scratchFrame->measurement.drdyTimeoutCount = UINT32_MAX;
    scratchFrame->measurement.staleCount = UINT32_MAX;
    scratchFrame->measurement.spiErrorCount = UINT32_MAX;
    for (size_t index = 0u; index < SENSORARRAY_MEASUREMENT_MAX_CELLS; ++index) {
        scratchFrame->measurement.valuesFixed[index] =
            SENSORARRAY_TEXT_MEASUREMENT_VALUE_MAX;
        scratchFrame->measurement.pgaGain[index] = 32u;
    }
    SENSORARRAY_PROTOCOL_CHECK(
        sensorarrayTextProtocolBuildMeasurementFrame(scratchFrame, &packet) == ESP_OK);
    SENSORARRAY_PROTOCOL_CHECK(packet.length <=
                               SENSORARRAY_TEXT_MEASUREMENT_WORST_CASE);

    memset(scratchFrame, 0, sizeof(*scratchFrame));
    scratchFrame->sequence = 42u;
    scratchFrame->activeRows = SENSORARRAY_MATRIX_ROWS;
    scratchFrame->mixedProfile = true;
    scratchFrame->configSnapshot.generation = 3u;
    scratchFrame->configSnapshot.requestId = 4u;
    scratchFrame->rowProfileGeneration = 5u;
    scratchFrame->rowProfileRequestId = 6u;
    static const sensorarrayMeasurementMode_t mixedModes[8] = {
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
        SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,
        SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
    };
    for (uint8_t row = 0u; row < SENSORARRAY_MATRIX_ROWS; ++row) {
        scratchFrame->rowMode[row] = mixedModes[row];
        scratchFrame->rowScale[row] = mixedModes[row] ==
            SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ? -3 : -6;
        uint64_t rowMask = UINT64_C(0xFF) <<
            ((size_t)row * SENSORARRAY_MATRIX_COLS);
        if (mixedModes[row] == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
            scratchFrame->capValidMask |= rowMask;
            scratchFrame->freshMask |= rowMask;
            for (uint8_t cell = 0u; cell < SENSORARRAY_MATRIX_COLS; ++cell) {
                scratchFrame->capTotalPf[row * SENSORARRAY_MATRIX_COLS + cell] =
                    1.0 + (double)cell;
            }
        } else {
            scratchFrame->measurement.validMask |= rowMask;
            scratchFrame->measurement.freshMask |= rowMask;
            for (uint8_t cell = 0u; cell < SENSORARRAY_MATRIX_COLS; ++cell) {
                scratchFrame->measurement.valuesFixed[
                    row * SENSORARRAY_MATRIX_COLS + cell] =
                    (int64_t)(row * 100u + cell);
            }
        }
    }
    SENSORARRAY_PROTOCOL_CHECK(
        sensorarrayTextProtocolBuildMixedFrame(scratchFrame, &packet) == ESP_OK);
    SENSORARRAY_PROTOCOL_CHECK(packet.data[0] == 'M' &&
                               strstr(packet.data, "MR,s=8") != NULL &&
                               strstr(packet.data, "fmt=mix1") != NULL &&
                               strstr(packet.data, "K,seq=42") != NULL);
    memset(scratchFrame, 0, sizeof(*scratchFrame));
    if (outChecks) {
        *outChecks = checks;
    }
    return true;
}
