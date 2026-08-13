#include "sensorarrayRowModeProfile.h"

#include <string.h>

static void sensorarrayRowModeProfileWriteBegin(sensorarrayRowModeProfile_t *profile)
{
    __atomic_add_fetch(&profile->snapshotVersion, 1u, __ATOMIC_RELEASE);
}

static void sensorarrayRowModeProfileWriteEnd(sensorarrayRowModeProfile_t *profile)
{
    __atomic_add_fetch(&profile->snapshotVersion, 1u, __ATOMIC_RELEASE);
}

static sensorarrayMeasurementMode_t sensorarrayRowModeFromChar(char value)
{
    switch (value) {
    case 'C': return SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE;
    case 'V': return SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
    case 'R': return SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    default: return SENSORARRAY_MEASUREMENT_MODE_NONE;
    }
}

static char sensorarrayRowModeToChar(sensorarrayMeasurementMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE: return 'C';
    case SENSORARRAY_MEASUREMENT_MODE_VOLTAGE: return 'V';
    case SENSORARRAY_MEASUREMENT_MODE_RESISTANCE: return 'R';
    default: return '?';
    }
}

void sensorarrayRowModeProfileInit(sensorarrayRowModeProfile_t *profile,
                                   sensorarrayMeasurementMode_t defaultMode)
{
    if (!profile || !sensorarrayMeasurementModeIsDataMode(defaultMode)) {
        return;
    }
    *profile = (sensorarrayRowModeProfile_t){
        .generation = 1u,
        .state = SENSORARRAY_ROW_PROFILE_STATE_APPLIED,
    };
    for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        profile->modes[row] = defaultMode;
        profile->pendingModes[row] = defaultMode;
    }
}

bool sensorarrayRowModeProfileParse(const char *text,
                                    size_t length,
                                    sensorarrayMeasurementMode_t outModes[SENSORARRAY_ROW_MODE_PROFILE_ROWS])
{
    if (!text || !outModes || length != SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH) {
        return false;
    }
    for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        sensorarrayMeasurementMode_t mode = sensorarrayRowModeFromChar(text[row]);
        if (!sensorarrayMeasurementModeIsDataMode(mode)) {
            return false;
        }
        outModes[row] = mode;
    }
    return true;
}

bool sensorarrayRowModeProfileFormat(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS],
    char outText[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH + 1u])
{
    if (!modes || !outText) {
        return false;
    }
    for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        outText[row] = sensorarrayRowModeToChar(modes[row]);
        if (outText[row] == '?') {
            outText[0] = '\0';
            return false;
        }
    }
    outText[SENSORARRAY_ROW_MODE_PROFILE_TEXT_LENGTH] = '\0';
    return true;
}

bool sensorarrayRowModeProfileAccept(sensorarrayRowModeProfile_t *profile,
                                     const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS],
                                     uint32_t requestId)
{
    if (!profile || !modes) {
        return false;
    }
    for (size_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        if (!sensorarrayMeasurementModeIsDataMode(modes[row])) {
            return false;
        }
    }
    sensorarrayRowModeProfileWriteBegin(profile);
    memcpy(profile->pendingModes, modes, sizeof(profile->pendingModes));
    profile->pendingRequestId = requestId;
    profile->pending = true;
    profile->state = SENSORARRAY_ROW_PROFILE_STATE_TRANSITION;
    profile->lastError = 0u;
    sensorarrayRowModeProfileWriteEnd(profile);
    return true;
}

bool sensorarrayRowModeProfileBeginTransition(sensorarrayRowModeProfile_t *profile)
{
    return profile && profile->pending &&
           profile->state == SENSORARRAY_ROW_PROFILE_STATE_TRANSITION;
}

bool sensorarrayRowModeProfileCompleteTransition(sensorarrayRowModeProfile_t *profile,
                                                  uint32_t appliedFrameSequence,
                                                  uint64_t transitionDurationUs)
{
    if (!sensorarrayRowModeProfileBeginTransition(profile)) {
        return false;
    }
    sensorarrayRowModeProfileWriteBegin(profile);
    memcpy(profile->modes, profile->pendingModes, sizeof(profile->modes));
    profile->appliedRequestId = profile->pendingRequestId;
    profile->appliedFrameSequence = appliedFrameSequence;
    profile->transitionDurationUs = transitionDurationUs;
    profile->generation++;
    if (profile->generation == 0u) {
        profile->generation = 1u;
    }
    profile->pending = false;
    profile->pendingRequestId = 0u;
    profile->state = SENSORARRAY_ROW_PROFILE_STATE_APPLIED;
    sensorarrayRowModeProfileWriteEnd(profile);
    return true;
}

void sensorarrayRowModeProfileFailTransition(sensorarrayRowModeProfile_t *profile,
                                             uint32_t errorCode,
                                             uint64_t transitionDurationUs)
{
    if (!profile) {
        return;
    }
    sensorarrayRowModeProfileWriteBegin(profile);
    profile->lastError = errorCode;
    profile->transitionDurationUs = transitionDurationUs;
    profile->appliedRequestId = profile->pendingRequestId;
    profile->pending = false;
    profile->pendingRequestId = 0u;
    profile->state = SENSORARRAY_ROW_PROFILE_STATE_DEGRADED;
    sensorarrayRowModeProfileWriteEnd(profile);
}

bool sensorarrayRowModeProfileCopy(const sensorarrayRowModeProfile_t *profile,
                                   sensorarrayRowModeProfile_t *outSnapshot)
{
    if (!profile || !outSnapshot) {
        return false;
    }
    for (uint8_t attempt = 0u; attempt < 8u; ++attempt) {
        uint32_t before = __atomic_load_n(&profile->snapshotVersion, __ATOMIC_ACQUIRE);
        if ((before & 1u) != 0u) {
            continue;
        }
        *outSnapshot = *profile;
        uint32_t after = __atomic_load_n(&profile->snapshotVersion, __ATOMIC_ACQUIRE);
        if (before == after && (after & 1u) == 0u) {
            return true;
        }
    }
    return false;
}

bool sensorarrayRowModeProfileIsHomogeneous(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS])
{
    if (!modes || !sensorarrayMeasurementModeIsDataMode(modes[0])) {
        return false;
    }
    for (size_t row = 1u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        if (modes[row] != modes[0]) {
            return false;
        }
    }
    return true;
}

sensorarrayMeasurementMode_t sensorarrayRowModeProfileHomogeneousMode(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS])
{
    return sensorarrayRowModeProfileIsHomogeneous(modes) ? modes[0] :
        SENSORARRAY_MEASUREMENT_MODE_NONE;
}

uint8_t sensorarrayRowModeProfileMaskForMode(
    const sensorarrayMeasurementMode_t modes[SENSORARRAY_ROW_MODE_PROFILE_ROWS],
    sensorarrayMeasurementMode_t mode)
{
    uint8_t mask = 0u;
    if (!modes || !sensorarrayMeasurementModeIsDataMode(mode)) {
        return mask;
    }
    for (uint8_t row = 0u; row < SENSORARRAY_ROW_MODE_PROFILE_ROWS; ++row) {
        if (modes[row] == mode) {
            mask |= (uint8_t)(1u << row);
        }
    }
    return mask;
}

const char *sensorarrayRowModeProfileStateName(sensorarrayRowModeProfileState_t state)
{
    switch (state) {
    case SENSORARRAY_ROW_PROFILE_STATE_APPLIED: return "applied";
    case SENSORARRAY_ROW_PROFILE_STATE_TRANSITION: return "transition";
    case SENSORARRAY_ROW_PROFILE_STATE_DEGRADED: return "degraded";
    default: return "unknown";
    }
}
