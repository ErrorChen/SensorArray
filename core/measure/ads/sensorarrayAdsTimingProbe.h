#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ads126xAdc.h"

/* 本轮新增: explicitly diagnostic; never a matrix sample or a speed selection. */
#define SENSORARRAY_ADS_TIMING_GROUPS 4u
#define SENSORARRAY_ADS_TIMING_SAMPLES 200u
#define SENSORARRAY_ADS_TIMING_RECORDS_PER_GROUP 8u
#define SENSORARRAY_ADS_TIMING_BUDGET_US 2000000u
#define SENSORARRAY_ADS_TIMING_CLEANUP_US 500000u

typedef enum {
    SENSORARRAY_ADS_TIMING_PROBE_TIMING = 0,
    SENSORARRAY_ADS_TIMING_PROBE_TAIL = 1,
} sensorarrayAdsTimingProbeKind_t;
const char *sensorarrayAdsTimingProbeKindName(sensorarrayAdsTimingProbeKind_t kind);

typedef struct {
    uint32_t attempted;
    uint32_t valid;
    uint32_t crc;
    uint32_t even;
    uint32_t odd;
    uint32_t xorUp;
    uint32_t xorDown;
    uint32_t xorOther;
    uint32_t status;
    uint32_t freshness;
    uint32_t timeout;
} sensorarrayAdsTimingReadStats_t;

typedef struct {
    bool pulse;
    bool gpioTail;
    bool bypass;
    bool completed;
    uint32_t startFenceMissed;
    uint32_t registerErrors;
    uint32_t recordCount;
    uint32_t recordsOmitted;
    uint32_t configurationGeneration;
    uint64_t startedUs;
    uint64_t completedUs;
    esp_err_t terminalError;
    const char *reason;
    sensorarrayAdsTimingReadStats_t primary;
    sensorarrayAdsTimingReadStats_t held;
} sensorarrayAdsTimingGroup_t;

typedef struct {
    uint32_t attempt;
    uint32_t configurationGeneration;
    uint32_t generationBefore;
    uint32_t generationFence;
    uint32_t generationAfter;
    uint64_t observedUs;
    uint32_t startCallUs;
    uint32_t waitUs;
    uint32_t readUs;
    int32_t raw;
    esp_err_t error;
    uint8_t status;
    uint8_t computed;
    uint8_t received;
    uint8_t crcParity;
    uint8_t length;
    uint8_t bytes[6];
    bool held;
    bool rawValid;
    bool fresh;
    bool crcValid;
    ads126xAdcTailEvidence_t tail;
} sensorarrayAdsTimingRecord_t;

typedef struct {
    sensorarrayAdsTimingProbeKind_t kind;
    uint32_t requestId;
    uint32_t frameSequence;
    uint32_t requestedHz;
    uint32_t actualHz;
    uint32_t generationBefore;
    uint32_t generationAfter;
    uint32_t crcGoodBefore;
    uint32_t crcGoodAfter;
    uint32_t crcBadBefore;
    uint32_t crcBadAfter;
    uint64_t startedUs;
    uint64_t workCompletedUs;
    uint64_t completedUs;
    uint64_t deadlineUs;
    esp_err_t firstError;
    esp_err_t terminalError;
    esp_err_t cleanupError;
    const char *reason;
    bool mutated;
    bool savedKnown;
    bool restoreKnown;
    bool restoreVerified;
    bool completed;
    bool requiresIsolation;
    ads126xAdc1RegisterSnapshot_t saved;
    ads126xAdc1RegisterSnapshot_t restored;
    sensorarrayAdsTimingGroup_t groups[SENSORARRAY_ADS_TIMING_GROUPS];
    sensorarrayAdsTimingRecord_t records[SENSORARRAY_ADS_TIMING_GROUPS]
        [SENSORARRAY_ADS_TIMING_RECORDS_PER_GROUP];
} sensorarrayAdsTimingProbeResult_t;

/* Caller holds CHECK owner and a deadline ending before the cleanup reserve.
 * The saved profile must already satisfy INTREF off, VBIAS on, CRC+STATUS,
 * AVDD/AVSS reference. This module never changes POWER, route or SPI clock. */
esp_err_t sensorarrayAdsTimingProbeRun(ads126xAdcHandle_t *ads,
    uint64_t workDeadlineUs, sensorarrayAdsTimingProbeResult_t *result);

/* Immutable completed-result formatters. Every record says diagnostic=1.
 * Return zero with an empty buffer if the whole line does not fit. */
size_t sensorarrayAdsTimingProbeFormatSummary(
    const sensorarrayAdsTimingProbeResult_t *result, char *buffer, size_t size);
size_t sensorarrayAdsTimingProbeFormatGroup(
    const sensorarrayAdsTimingProbeResult_t *result, uint32_t group,
    char *buffer, size_t size);
size_t sensorarrayAdsTimingProbeFormatGroupTiming(
    const sensorarrayAdsTimingProbeResult_t *result, uint32_t group,
    char *buffer, size_t size);
size_t sensorarrayAdsTimingProbeFormatRegisters(
    const sensorarrayAdsTimingProbeResult_t *result, bool restored,
    char *buffer, size_t size);
size_t sensorarrayAdsTimingProbeFormatRecord(
    const sensorarrayAdsTimingProbeResult_t *result, uint32_t record,
    char *buffer, size_t size);
size_t sensorarrayAdsTimingProbeFormatTailRecord(
    const sensorarrayAdsTimingProbeResult_t *result, uint32_t record,
    char *buffer, size_t size);
/* 本轮新增: numeric GPIO configuration evidence; never analog isolation. */
size_t sensorarrayAdsTimingProbeFormatTailIoRecord(
    const sensorarrayAdsTimingProbeResult_t *result, uint32_t record,
    char *buffer, size_t size);
uint32_t sensorarrayAdsTimingProbeRecordCount(
    const sensorarrayAdsTimingProbeResult_t *result);
