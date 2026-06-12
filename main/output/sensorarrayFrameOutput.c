#include "sensorarrayFrameOutput.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "esp_timer.h"
#include "sensorarrayConfig.h"
#include "sensorarrayMeasure.h"

#define SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF (-1.0)
#define SENSORARRAY_BINARY_V1_MAGIC UINT32_C(0x31464153)
#define SENSORARRAY_BINARY_V1_VERSION 1u
#define SENSORARRAY_BINARY_V1_HEADER_BYTES 80u
#define SENSORARRAY_BINARY_V1_PACKET_BYTES \
    (SENSORARRAY_BINARY_V1_HEADER_BYTES + (SENSORARRAY_MATRIX_CELL_COUNT * sizeof(float)) + sizeof(uint32_t))

_Static_assert(SENSORARRAY_BINARY_V1_PACKET_BYTES == 340u,
               "SAF1 packet layout changed; update the host protocol version");

#ifndef CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY
#define CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY 1
#endif
#ifndef CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY_EVERY_N_FRAMES 20
#endif
#ifndef CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY_ON_ANOMALY
#define CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY_ON_ANOMALY 1
#endif
#ifndef CONFIG_SENSORARRAY_LOG_FRAME_OUTPUT_LEGACY
#define CONFIG_SENSORARRAY_LOG_FRAME_OUTPUT_LEGACY 0
#endif
#ifndef CONFIG_SENSORARRAY_LOG_OMIT_DEFAULT_FIELDS
#define CONFIG_SENSORARRAY_LOG_OMIT_DEFAULT_FIELDS 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_CAP_PRINT_DECIMALS
#define CONFIG_SENSORARRAY_FDC_CAP_PRINT_DECIMALS 6
#endif
#ifndef CONFIG_SENSORARRAY_FDC_CAP_DECIMALS
#define CONFIG_SENSORARRAY_FDC_CAP_DECIMALS CONFIG_SENSORARRAY_FDC_CAP_PRINT_DECIMALS
#endif
#ifndef CONFIG_SENSORARRAY_OUTPUT_LEGACY_MATRIXFDC_CAP
#define CONFIG_SENSORARRAY_OUTPUT_LEGACY_MATRIXFDC_CAP 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_COMPACT_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_TIMING_COMPACT_EVERY_N_FRAMES 20
#endif

static void sensorarrayBinaryPutU16(uint8_t *buffer, size_t offset, uint16_t value)
{
    buffer[offset] = (uint8_t)(value & 0xFFu);
    buffer[offset + 1u] = (uint8_t)(value >> 8u);
}

static void sensorarrayBinaryPutU32(uint8_t *buffer, size_t offset, uint32_t value)
{
    for (size_t byte = 0u; byte < sizeof(value); ++byte) {
        buffer[offset + byte] = (uint8_t)(value >> (byte * 8u));
    }
}

static void sensorarrayBinaryPutU64(uint8_t *buffer, size_t offset, uint64_t value)
{
    for (size_t byte = 0u; byte < sizeof(value); ++byte) {
        buffer[offset + byte] = (uint8_t)(value >> (byte * 8u));
    }
}

static uint32_t sensorarrayBinaryCrc32(const uint8_t *data, size_t length)
{
    uint32_t crc = UINT32_MAX;
    for (size_t index = 0u; index < length; ++index) {
        crc ^= data[index];
        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            uint32_t reflectedPolynomial = (crc & 1u) ? UINT32_C(0xEDB88320) : 0u;
            crc = (crc >> 1u) ^ reflectedPolynomial;
        }
    }
    return ~crc;
}

static esp_err_t sensorarrayTransportSendFdcMatrixFrame(const sensorarrayFrame_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * The packet is assembled byte-by-byte instead of casting a packed C
     * structure.  This makes the wire layout independent of compiler padding,
     * target alignment, and the ESP32-S3 ABI.  All integer and float fields are
     * little-endian; ESP32-S3 float is IEEE-754 binary32.
     */
    uint8_t packet[SENSORARRAY_BINARY_V1_PACKET_BYTES] = {0};
    sensorarrayBinaryPutU32(packet, 0u, SENSORARRAY_BINARY_V1_MAGIC);
    packet[4] = SENSORARRAY_BINARY_V1_VERSION;
    packet[5] = (frame->freshFrame ? 1u : 0u) |
                (frame->stale ? 2u : 0u) |
                (frame->mixedEpoch ? 4u : 0u);
    sensorarrayBinaryPutU16(packet, 6u, SENSORARRAY_BINARY_V1_HEADER_BYTES);
    sensorarrayBinaryPutU16(packet, 8u, (uint16_t)sizeof(packet));
    packet[10] = SENSORARRAY_MATRIX_CELL_COUNT;
    packet[11] = SENSORARRAY_MATRIX_ROWS;
    sensorarrayBinaryPutU32(packet, 12u, frame->sequence);
    sensorarrayBinaryPutU32(packet, 16u, frame->physicalSweepId);
    sensorarrayBinaryPutU64(packet, 20u, frame->frameStartUs);
    sensorarrayBinaryPutU64(packet, 28u, frame->frameEndUs);
    sensorarrayBinaryPutU64(packet, 36u, frame->emitUs);
    packet[44] = frame->rowFreshMask;
    packet[45] = frame->primaryFreshMask;
    packet[46] = frame->secondaryFreshMask;
    sensorarrayBinaryPutU64(packet, 48u, frame->capValidMask);
    sensorarrayBinaryPutU64(packet, 56u, frame->freshMask);
    sensorarrayBinaryPutU64(packet, 64u, frame->warnMask);
    sensorarrayBinaryPutU64(packet, 72u, frame->errorMask);

    size_t payloadOffset = SENSORARRAY_BINARY_V1_HEADER_BYTES;
    for (size_t cell = 0u; cell < SENSORARRAY_MATRIX_CELL_COUNT; ++cell) {
        float capPf = (float)frame->capTotalPf[cell];
        uint32_t encoded = 0u;
        memcpy(&encoded, &capPf, sizeof(encoded));
        sensorarrayBinaryPutU32(packet, payloadOffset + (cell * sizeof(encoded)), encoded);
    }

    uint32_t crc = sensorarrayBinaryCrc32(packet, sizeof(packet) - sizeof(uint32_t));
    sensorarrayBinaryPutU32(packet, sizeof(packet) - sizeof(uint32_t), crc);
    size_t written = fwrite(packet, 1u, sizeof(packet), stdout);
    fflush(stdout);
    return written == sizeof(packet) ? ESP_OK : ESP_FAIL;
}

static size_t sensorarrayFrameOutputAppend(char *buffer,
                                           size_t bufferSize,
                                           size_t pos,
                                           const char *fmt,
                                           ...)
{
    if (!buffer || bufferSize == 0u || pos >= bufferSize) {
        return pos;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(&buffer[pos], bufferSize - pos, fmt, args);
    va_end(args);
    if (written < 0) {
        return pos;
    }
    size_t used = (size_t)written;
    if (used >= bufferSize - pos) {
        return bufferSize - 1u;
    }
    return pos + used;
}

static void sensorarrayFrameOutputPrintCapLine(const sensorarrayFrame_t *frame,
                                               bool partial,
                                               const char *frameQuality)
{
    static char line[4096];
    size_t pos = 0u;
    bool fullValid = frame->capValidMask == UINT64_MAX;
    bool allFresh = frame->freshMask == UINT64_MAX;
    bool noWarn = frame->warnMask == 0u;
    bool noError = frame->errorMask == 0u;
    const char *qualityToken = partial ? "P" : "F";
    pos = sensorarrayFrameOutputAppend(line,
                                       sizeof(line),
                                       pos,
                                       "%s,s=%lu,ps=%lu,t=%llu,fs=%llu,fe=%llu,emit=%llu,q=%s,rf=%02X,pfmask=%02X,sfmask=%02X,stale=%u,mixed=%u",
#if CONFIG_SENSORARRAY_OUTPUT_LEGACY_MATRIXFDC_CAP
                                       "MATRIXFDC_CAP",
#else
                                       "Cap",
#endif
                                       (unsigned long)frame->sequence,
                                       (unsigned long)frame->physicalSweepId,
                                       (unsigned long long)frame->timestampUs,
                                       (unsigned long long)frame->frameStartUs,
                                       (unsigned long long)frame->frameEndUs,
                                       (unsigned long long)frame->emitUs,
                                       qualityToken,
                                       (unsigned)frame->rowFreshMask,
                                       (unsigned)frame->primaryFreshMask,
                                       (unsigned)frame->secondaryFreshMask,
                                       frame->stale ? 1u : 0u,
                                       frame->mixedEpoch ? 1u : 0u);
    if (!CONFIG_SENSORARRAY_LOG_OMIT_DEFAULT_FIELDS || partial) {
        pos = sensorarrayFrameOutputAppend(line,
                                           sizeof(line),
                                           pos,
                                           ",pa=%u,cv=%s,fm=%s",
                                           partial ? 1u : 0u,
                                           fullValid ? "*" : "x",
                                           allFresh ? "*" : "x");
        if (!fullValid) {
            pos = sensorarrayFrameOutputAppend(line,
                                               sizeof(line),
                                               pos,
                                               ",cvx=%016llX",
                                               (unsigned long long)frame->capValidMask);
        }
        if (!allFresh) {
            pos = sensorarrayFrameOutputAppend(line,
                                               sizeof(line),
                                               pos,
                                               ",fmx=%016llX",
                                               (unsigned long long)frame->freshMask);
        }
    }
    if (!CONFIG_SENSORARRAY_LOG_OMIT_DEFAULT_FIELDS || !noWarn || !noError) {
        pos = sensorarrayFrameOutputAppend(line,
                                           sizeof(line),
                                           pos,
                                           ",wm=%s,em=%s",
                                           noWarn ? "0" : "x",
                                           noError ? "0" : "x");
        if (!noWarn) {
            pos = sensorarrayFrameOutputAppend(line,
                                               sizeof(line),
                                               pos,
                                               ",wmx=%016llX",
                                               (unsigned long long)frame->warnMask);
        }
        if (!noError) {
            pos = sensorarrayFrameOutputAppend(line,
                                               sizeof(line),
                                               pos,
                                               ",emx=%016llX",
                                               (unsigned long long)frame->errorMask);
        }
    }
    pos = sensorarrayFrameOutputAppend(line, sizeof(line), pos, ",pf=[");
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        pos = sensorarrayFrameOutputAppend(line,
                                           sizeof(line),
                                           pos,
                                           "%s%.*f",
                                           (i == 0u) ? "" : ",",
                                           CONFIG_SENSORARRAY_FDC_CAP_DECIMALS,
                                           frame->capTotalPf[i]);
    }
    (void)sensorarrayFrameOutputAppend(line, sizeof(line), pos, "]\n");
    printf("%s", line);
    (void)frameQuality;
}

static void sensorarrayFrameOutputPrintText(const sensorarrayFrame_t *frame, const char *tag)
{
#if !CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
    (void)tag;
#endif
    bool partial = !frame->freshFrame || frame->capValidMask != UINT64_MAX;
    const char *frameQuality = partial ? "partial" : "full";
    bool anomaly = partial || frame->warnMask != 0u || frame->errorMask != 0u ||
                   frame->notReadyCount != 0u ||
                   frame->zeroBeforeReadyCount != 0u ||
                   frame->zeroAfterDrdyCount != 0u ||
                   frame->i2cErrorCount != 0u ||
                   frame->unreadWithoutDrdyCount != 0u ||
                   frame->stale || frame->mixedEpoch;
#if CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY
    uint32_t summaryEvery = (uint32_t)CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY_EVERY_N_FRAMES;
    bool sampledSummary = summaryEvery != 0u &&
                          frame->sequence != 0u &&
                          (frame->sequence % summaryEvery) == 0u;
    if ((CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY_ON_ANOMALY && anomaly) || sampledSummary) {
        printf("S,s=%lu,ps=%lu,n=%lu,vc=%u,ic=%u,fc=%u,nr=%u,z0=%u,zd=%u,i2c=%u,uwd=%u,rf=%02X,pf=%02X,sf=%02X,stale=%u,mixed=%u,cv=%s,vm=%s,wm=%s,em=%s,fb=%u/%u\n",
               (unsigned long)frame->sequence,
               (unsigned long)frame->physicalSweepId,
               (unsigned long)(summaryEvery ? summaryEvery : 1u),
               (unsigned)frame->validCount,
               (unsigned)(SENSORARRAY_MATRIX_CELL_COUNT - frame->validCount),
               (unsigned)frame->freshCount,
               (unsigned)frame->notReadyCount,
               (unsigned)frame->zeroBeforeReadyCount,
               (unsigned)frame->zeroAfterDrdyCount,
               (unsigned)frame->i2cErrorCount,
               (unsigned)frame->unreadWithoutDrdyCount,
               (unsigned)frame->rowFreshMask,
               (unsigned)frame->primaryFreshMask,
               (unsigned)frame->secondaryFreshMask,
               frame->stale ? 1u : 0u,
               frame->mixedEpoch ? 1u : 0u,
               frame->capValidMask == UINT64_MAX ? "*" : "x",
               frame->validMask == UINT64_MAX ? "*" : "x",
               frame->warnMask == 0u ? "0" : "x",
               frame->errorMask == 0u ? "0" : "x",
               (unsigned)frame->firstBadRow,
               (unsigned)frame->firstBadDevice);
    }
#endif
#if CONFIG_SENSORARRAY_LOG_FRAME_OUTPUT_LEGACY
    printf("FDC_FRAME_OUTPUT,seq=%lu,frameQuality=%s,partial=%u,capValidMask=0x%016llX,errorMask=0x%016llX,invalidSentinel=%.6f\n",
           (unsigned long)frame->sequence,
           frameQuality,
           partial ? 1u : 0u,
           (unsigned long long)frame->capValidMask,
           (unsigned long long)frame->errorMask,
           SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF);
#endif
#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_CAP_TOTAL_PF || CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
    sensorarrayFrameOutputPrintCapLine(frame, partial, frameQuality);
#endif

#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_FREQ_HZ || CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
    printf("MATRIXFDC_FREQ,seq=%lu,timestampUs=%llu,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,freqHz=[",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.1f", (i == 0u) ? "" : ",", frame->freqHz[i]);
    }
    printf("]\n");
#endif

#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
    printf("%s_INLINE_DEBUG,seq=%lu,timestampUs=%llu,partial=%u,frameQuality=%s,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,capValidMask=0x%016llX,invalidSentinel=%.6f,freqHz=[",
           tag ? tag : "MATRIXFDC",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           partial ? 1u : 0u,
           frameQuality,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask,
           (unsigned long long)frame->capValidMask,
           SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.1f", (i == 0u) ? "" : ",", frame->freqHz[i]);
    }
    printf("],capTotalPf=[");
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.6f", (i == 0u) ? "" : ",", frame->capTotalPf[i]);
    }
    printf("]\n");
#endif

#if CONFIG_SENSORARRAY_FDC_RAW_DEBUG_LOG
    printf("DEBUGFDC_RAW,seq=%lu,timestampUs=%llu,raw28=[",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%lu", (i == 0u) ? "" : ",", (unsigned long)frame->raw28[i]);
    }
    printf("]\n");
#endif
}

esp_err_t sensorarrayFrameOutputPrint(const sensorarrayFrame_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensorarrayFastSpeedIsEnabled()) {
        return sensorarrayTransportSendFdcMatrixFrame(frame);
    }

    int64_t outputStartUs = esp_timer_get_time();
    sensorarrayFrameOutputPrintText(frame, "MATRIXFDC");
    uint64_t outputUs = (uint64_t)(esp_timer_get_time() - outputStartUs);
    uint32_t period = (uint32_t)CONFIG_SENSORARRAY_FDC_TIMING_COMPACT_EVERY_N_FRAMES;
    if (period != 0u && frame->sequence != 0u && (frame->sequence % period) == 0u) {
        printf("OT,s=%lu,op=%llu,lr=1,n=%lu\n",
               (unsigned long)frame->sequence,
               (unsigned long long)outputUs,
               (unsigned long)period);
    }
    return ESP_OK;
}
