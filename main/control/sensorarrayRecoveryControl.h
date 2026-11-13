#pragma once

#include "sensorarrayConfigIdentity.h"

/* 本轮新增: one bounded, queryable RECOVER transaction. Hardware remains
 * exclusively owned by the acquisition task. No result survives a new boot. */
typedef enum {
    SENSORARRAY_RECOVER_IDLE = 0,
    SENSORARRAY_RECOVER_PENDING,
    SENSORARRAY_RECOVER_RUNNING,
    SENSORARRAY_RECOVER_APPLIED,
    SENSORARRAY_RECOVER_REJECTED,
    SENSORARRAY_RECOVER_RESTARTING,
} sensorarrayRecoverState_t;

typedef struct {
    uint32_t id;
    uint32_t level;
    bool identityKnown;
    sensorarrayConfigIdentity_t identity;
} sensorarrayRecoverRequest_t;

typedef enum {
    SENSORARRAY_RECOVER_TAKE_NONE = 0,
    SENSORARRAY_RECOVER_TAKE_READY,
    SENSORARRAY_RECOVER_TAKE_STALE,
} sensorarrayRecoverTake_t;

void sensorarrayRecoveryControlInit(uint64_t sessionId, uint32_t bootId);
esp_err_t sensorarrayRecoveryControlPost(uint32_t level,
    const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId);
bool sensorarrayRecoveryControlAck(uint32_t id, esp_err_t publishError);
sensorarrayRecoverTake_t sensorarrayRecoveryControlTake(
    const sensorarrayConfigAuthority_t *authority, bool known,
    sensorarrayRecoverRequest_t *outRequest);
esp_err_t sensorarrayRecoveryControlComplete(uint32_t id,
    sensorarrayRecoverState_t outcome, const char *reason,
    esp_err_t originalError, esp_err_t cleanupError,
    const sensorarrayConfigAuthority_t *authority, bool known,
    bool routeApplied, bool routeKnown, bool routeVerified);
esp_err_t sensorarrayRecoveryControlFormat(char *response, size_t responseSize);
