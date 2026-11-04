#pragma once

#include "sensorarrayScanConfig.h"
#include "sensorarrayMeasurementMode.h"
#include "sensorarrayRowModeProfile.h"

/* 本轮新增. Legacy gen/rid keep their original domains; this identity is
 * captured by the acquisition owner before sampling and copied with the frame. */
typedef struct {
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t revision;
    uint32_t appliedFrameBoundary;
    uint32_t modeGeneration;
    uint32_t modeRequestId;
    uint32_t rowsGeneration;
    uint32_t rowsRequestId;
    uint32_t profileGeneration;
    uint32_t profileRequestId;
    sensorarrayMeasurementState_t state;
    sensorarrayMeasurementMode_t mode;
    sensorarrayRowModeProfileState_t profileState;
    uint8_t rows;
    char profile[9];
} sensorarrayConfigIdentity_t;

typedef struct {
    sensorarrayConfigIdentity_t identity;
    uint32_t stateError;
    uint32_t rowsTerminalId;
    uint32_t rowsError;
    sensorarrayConfigOutcome_t rowsOutcome;
    uint32_t modeTerminalId;
    uint32_t modeError;
    sensorarrayConfigOutcome_t modeOutcome;
    uint32_t profileTerminalId;
    uint32_t profileError;
    sensorarrayConfigOutcome_t profileOutcome;
    bool available;
} sensorarrayConfigAuthority_t;

/* Capture does not claim the candidate was applied. Only the single Core 1
 * owner publishes, after applying the route/profile. Readers copy one atomic
 * authority object, never combine independent global snapshots. */
void sensorarrayConfigIdentityCapture(
    uint64_t sessionId, uint32_t bootId,
    const sensorarrayMeasurementModeSnapshot_t *mode,
    const sensorarrayFrameConfigSnapshot_t *rows,
    const sensorarrayRowModeProfile_t *profile,
    uint32_t frameBoundary,
    sensorarrayConfigIdentity_t *outIdentity);
void sensorarrayConfigIdentityPublish(
    const sensorarrayConfigIdentity_t *identity,
    const sensorarrayMeasurementModeSnapshot_t *mode,
    const sensorarrayRowModeProfile_t *profile,
    const sensorarrayFrameConfigSnapshot_t *rows);
bool sensorarrayConfigIdentityCopy(sensorarrayConfigAuthority_t *outSnapshot);
esp_err_t sensorarrayConfigIdentityFormat(char *response, size_t responseSize);
