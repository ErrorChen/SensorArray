// Split from core/measure/sensorarrayMeasure.c to keep measurement domains isolated.
static esp_err_t sensorarrayTransportSendFdcMatrixFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    (void)frame;
    return ESP_ERR_NOT_SUPPORTED;
}

static void sensorarrayFdcMatrixPrintFrame(const sensorarrayFdcMatrixFrame_t *frame, const char *tag)
{
#if !CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_INLINE_DEBUG
    (void)tag;
#endif
    bool partial = frame->capValidMask != UINT64_MAX;
    const char *frameQuality = partial ? "partial" : "full";
    printf("FDC_FRAME_OUTPUT,seq=%lu,frameQuality=%s,partial=%u,capValidMask=0x%016llX,errorMask=0x%016llX,invalidSentinel=%.6f\n",
           (unsigned long)frame->sequence,
           frameQuality,
           partial ? 1u : 0u,
           (unsigned long long)frame->capValidMask,
           (unsigned long long)frame->errorMask,
           SENSORARRAY_FDC_INVALID_CAP_SENTINEL_PF);
#if CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_CAP_TOTAL_PF || CONFIG_SENSORARRAY_FDC_TEXT_OUTPUT_BOTH_SEPARATE
    printf("MATRIXFDC_CAP,seq=%lu,timestampUs=%llu,partial=%u,frameQuality=%s,capValidMask=0x%016llX,freshMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,invalidSentinel=%.6f,capTotalPf=[",
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
        printf("%s%.6f", (i == 0u) ? "" : ",", frame->capTotalPf[i]);
    }
    printf("]\n");
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

esp_err_t sensorarrayFdcMatrixEmitFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensorarrayFastSpeedIsEnabled()) {
        return sensorarrayTransportSendFdcMatrixFrame(frame);
    }

    sensorarrayFdcMatrixPrintFrame(frame, "MATRIXFDC");
    return ESP_OK;
