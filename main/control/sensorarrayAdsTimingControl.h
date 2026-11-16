#pragma once

#include "sensorarrayAdsTimingProbe.h"
#include "sensorarrayConfigIdentity.h"

/* 本轮新增: one ACK-fenced, bounded diagnostic request and immutable last result.
 * Hardware is exclusively called by Core 1 at a complete-frame boundary. */
typedef struct {
    uint32_t id;
    sensorarrayConfigIdentity_t identity;
    uint64_t acceptedUs;
    uint64_t expiresUs;
} sensorarrayAdsTimingRequest_t;

typedef enum {
    SENSORARRAY_TIMING_TAKE_NONE,
    SENSORARRAY_TIMING_TAKE_READY,
    SENSORARRAY_TIMING_TAKE_STALE,
    SENSORARRAY_TIMING_TAKE_EXPIRED,
} sensorarrayAdsTimingTake_t;

void sensorarrayAdsTimingControlInit(uint64_t session, uint32_t bootId);
esp_err_t sensorarrayAdsTimingControlPost(const sensorarrayConfigAuthority_t *authority,
    uint64_t nowUs, uint32_t *outId);
bool sensorarrayAdsTimingControlAck(uint32_t id, esp_err_t error);
sensorarrayAdsTimingTake_t sensorarrayAdsTimingControlTake(uint32_t id,
    const sensorarrayConfigAuthority_t *authority, bool known, uint64_t nowUs,
    sensorarrayAdsTimingRequest_t *outRequest);
/* Core 1 only: never expose the mutable working result to query tasks. */
sensorarrayAdsTimingProbeResult_t *sensorarrayAdsTimingControlWorkResult(void);
esp_err_t sensorarrayAdsTimingControlComplete(uint32_t id, const char *outcome,
    const char *reason, esp_err_t originalError, esp_err_t cleanupError,
    const sensorarrayConfigAuthority_t *authority, bool known, bool keepResult);
void sensorarrayAdsTimingControlExpire(uint64_t nowUs,
    const sensorarrayConfigAuthority_t *authority, bool known);
bool sensorarrayAdsTimingControlHandles(const char *command);
esp_err_t sensorarrayAdsTimingControlFormat(const char *command,
    char *response, size_t responseSize);
