#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "sensorarrayTypes.h"

/* 本轮新增: pure-value result owned by the persistent boot job. A completed
 * task/result handshake is required before its storage can be reused. */
typedef struct {
    bool qualified;
    bool restored;
    uint32_t selectedHz;
    uint32_t actualHz;
    esp_err_t error;
    esp_err_t cleanupError;
    uint64_t startUs;
    uint64_t endUs;
    uint32_t candidatesTested;
    uint32_t attemptedSamples;
    uint32_t validSamples;
    uint32_t crcValidEven;
    uint32_t crcValidOdd;
    uint32_t crcErrors;
    uint32_t statusErrors;
    uint32_t registerErrors;
    uint32_t drdyTimeouts;
} sensorarrayAdsSpiProbeResult_t;

/* 本轮新增: exclusive ADS boot owner, before matrix/route/Gap initialization.
 * Absolute monotonic deadline includes cleanup. Does not route the matrix,
 * enable INTREF, calibrate, or publish a rail/scientific measurement.
 * ESP_OK requires both qualification and complete saved-state restoration.
 * Timeout never cancels DMA: an unknown transfer remains quarantined. */
esp_err_t sensorarrayAdsSpiProbeRun(sensorarrayState_t *state,
    uint64_t absoluteDeadlineUs, sensorarrayAdsSpiProbeResult_t *outResult);
