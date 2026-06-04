#include "sensorarrayFrameOutput.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

#include "esp_timer.h"
#include "sensorarrayConfig.h"
#include "sensorarrayMeasure.h"

#define SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF (-1.0)

#ifndef CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY
#define CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY 1
#endif

static esp_err_t sensorarrayTransportSendFdcMatrixFrame(const sensorarrayFrame_t *frame)
{
    (void)frame;
    return ESP_ERR_NOT_SUPPORTED;
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
    pos = sensorarrayFrameOutputAppend(line,
                                       sizeof(line),
                                       pos,
                                       "MATRIXFDC_CAP,seq=%lu,timestampUs=%llu,partial=%u,frameQuality=%s,capValidMask=0x%016llX,freshMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,invalidSentinel=%.6f,capTotalPf=[",
                                       (unsigned long)frame->sequence,
                                       (unsigned long long)frame->timestampUs,
                                       partial ? 1u : 0u,
                                       frameQuality,
                                       (unsigned long long)frame->capValidMask,
                                       (unsigned long long)frame->freshMask,
                                       (unsigned long long)frame->warnMask,
                                       (unsigned long long)frame->errorMask,
                                       SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        pos = sensorarrayFrameOutputAppend(line,
                                           sizeof(line),
                                           pos,
                                           "%s%.6f",
                                           (i == 0u) ? "" : ",",
                                           frame->capTotalPf[i]);
    }
    (void)sensorarrayFrameOutputAppend(line, sizeof(line), pos, "]\n");
    printf("%s", line);
}

static void sensorarrayFrameOutputPrintText(const sensorarrayFrame_t *frame, const char *tag)
{
#if !CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
    (void)tag;
#endif
    bool partial = frame->capValidMask != UINT64_MAX;
    const char *frameQuality = partial ? "partial" : "full";
#if CONFIG_SENSORARRAY_LOG_FRAME_SUMMARY
    printf("FDC_FRAME_SUMMARY,seq=%lu,validCells=%u,invalidCells=%u,freshCells=%u,notReadyCells=%u,zeroBeforeReadyCells=%u,zeroAfterDrdyCells=%u,i2cErrorCells=%u,unreadWithoutDrdyCells=%u,capValidMask=0x%016llX,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,firstBadRow=%u,firstBadDevice=%u\n",
           (unsigned long)frame->sequence,
           (unsigned)frame->validCount,
           (unsigned)(SENSORARRAY_MATRIX_CELL_COUNT - frame->validCount),
           (unsigned)frame->freshCount,
           (unsigned)frame->notReadyCount,
           (unsigned)frame->zeroBeforeReadyCount,
           (unsigned)frame->zeroAfterDrdyCount,
           (unsigned)frame->i2cErrorCount,
           (unsigned)frame->unreadWithoutDrdyCount,
           (unsigned long long)frame->capValidMask,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask,
           (unsigned)frame->firstBadRow,
           (unsigned)frame->firstBadDevice);
#endif
    printf("FDC_FRAME_OUTPUT,seq=%lu,frameQuality=%s,partial=%u,capValidMask=0x%016llX,errorMask=0x%016llX,invalidSentinel=%.6f\n",
           (unsigned long)frame->sequence,
           frameQuality,
           partial ? 1u : 0u,
           (unsigned long long)frame->capValidMask,
           (unsigned long long)frame->errorMask,
           SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF);
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
    uint32_t period = (uint32_t)CONFIG_SENSORARRAY_FDC_SAMPLE_DEVICE_LOG_EVERY_N_FRAMES;
    if (period != 0u && frame->sequence != 0u && (frame->sequence % period) == 0u) {
        printf("FDC_OUTPUT_TIMING,seq=%lu,outputPrintfUs=%llu,lockReleased=1\n",
               (unsigned long)frame->sequence,
               (unsigned long long)outputUs);
    }
    return ESP_OK;
}
