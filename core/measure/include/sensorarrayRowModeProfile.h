#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "sensorarrayMeasurementMode.h"

#define SENSORARRAY_ROW_MODE_PROFILE_ROWS 8u
#define SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH 8u

typedef enum {
    SENSORARRAY_ROW_PROFILE_STATE_APPLIED = 0,
    SENSORARRAY_ROW_PROFILE_STATE_TRANSITION,
    SENSORARRAY_ROW_PROFILE_STATE_DEGRADED,
} sensorarrayRowModeProfileState_t;

typedef struct {
    sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
    sensorarrayMeasurementMode_t pendingModes[SENSORARRAY_ROW_MODE_PROFILE_ROWS];
    uint32_t pendingRequestId;
    uint32_t appliedRequestId;
    uint32_t generation;
    uint32_t appliedFrameSequence;
    uint32_t lastError;
    uint64_t transitionDurationUs;
    volatile uint32_t snapshotVersion;
    sensorarrayRowModeProfileState_t state;
    bool pending;
} sensorarrayRowModeProfile_t;

void sensorarrayRowModeProfileInit(sensorarrayRowModeProfile_t *profile,
                                   sensorarrayMeasurementMode_t defaultMode);
bool sensorarrayRowModeProfileParse(const char *text,
                                    size_t length,
                                    sensorarrayMeasurementMode_t outModes[SENSORARRAY_ROW_MODE_PROFILE_ROWS]);
bool sensorarrayRowModeProfileFormat(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS],
    char outText[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u]);
bool sensorarrayRowModeProfileAccept(sensorarrayRowModeProfile_t *profile,
                                     const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS],
                                     uint32_t requestId);
bool sensorarrayRowModeProfileBeginTransition(sensorarrayRowModeProfile_t *profile);
bool sensorarrayRowModeProfileCompleteTransition(sensorarrayRowModeProfile_t *profile,
                                                  uint32_t appliedFrameSequence,
                                                  uint64_t transitionDurationUs);
void sensorarrayRowModeProfileFailTransition(sensorarrayRowModeProfile_t *profile,
                                             uint32_t errorCode,
                                             uint64_t transitionDurationUs);
bool sensorarrayRowModeProfileCopy(const sensorarrayRowModeProfile_t *profile,
                                   sensorarrayRowModeProfile_t *outSnapshot);
bool sensorarrayRowModeProfileIsHomogeneous(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS]);
sensorarrayMeasurementMode_t sensorarrayRowModeProfileHomogeneousMode(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS]);
uint8_t sensorarrayRowModeProfileMaskForMode(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS],
    sensorarrayMeasurementMode_t mode);
const char *sensorarrayRowModeProfileStateName(sensorarrayRowModeProfileState_t state);

