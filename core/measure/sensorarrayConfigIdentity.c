#include "sensorarrayConfigIdentity.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"

static portMUX_TYPE s_authorityMux = portMUX_INITIALIZER_UNLOCKED;
static sensorarrayConfigAuthority_t s_authority;

static bool sensorarrayConfigIdentitySame(const sensorarrayConfigIdentity_t *a,
                                          const sensorarrayConfigIdentity_t *b)
{
    return a->sessionId == b->sessionId && a->bootId == b->bootId &&
        a->modeGeneration == b->modeGeneration && a->modeRequestId == b->modeRequestId &&
        a->rowsGeneration == b->rowsGeneration && a->rowsRequestId == b->rowsRequestId &&
        a->profileGeneration == b->profileGeneration &&
        a->profileRequestId == b->profileRequestId && a->state == b->state &&
        a->mode == b->mode && a->profileState == b->profileState && a->rows == b->rows &&
        memcmp(a->profile, b->profile, sizeof(a->profile)) == 0;
}

void sensorarrayConfigIdentityCapture(
    uint64_t sessionId, uint32_t bootId,
    const sensorarrayMeasurementModeSnapshot_t *mode,
    const sensorarrayFrameConfigSnapshot_t *rows,
    const sensorarrayRowModeProfile_t *profile,
    uint32_t frameBoundary,
    sensorarrayConfigIdentity_t *outIdentity)
{
    if (!mode || !rows || !profile || !outIdentity) {
        return;
    }
    sensorarrayConfigIdentity_t next = {
        .sessionId = sessionId, .bootId = bootId, .appliedFrameBoundary = frameBoundary,
        .modeGeneration = mode->generation, .modeRequestId = mode->appliedRequestId,
        .rowsGeneration = rows->generation, .rowsRequestId = rows->requestId,
        .profileGeneration = profile->generation, .profileRequestId = profile->appliedRequestId,
        .state = mode->state, .mode = mode->activeMode,
        .profileState = profile->state, .rows = rows->rows,
    };
    (void)sensorarrayRowModeProfileFormat(profile->modes, next.profile);
    portENTER_CRITICAL(&s_authorityMux);
    if (s_authority.available && sensorarrayConfigIdentitySame(&next, &s_authority.identity)) {
        next.revision = s_authority.identity.revision;
        next.appliedFrameBoundary = s_authority.identity.appliedFrameBoundary;
    } else {
        next.revision = s_authority.available &&
            sessionId == s_authority.identity.sessionId ? s_authority.identity.revision + 1u : 1u;
        if (next.revision == 0u) {
            next.revision = 1u;
        }
    }
    portEXIT_CRITICAL(&s_authorityMux);
    *outIdentity = next;
}

void sensorarrayConfigIdentityPublish(
    const sensorarrayConfigIdentity_t *identity,
    const sensorarrayMeasurementModeSnapshot_t *mode,
    const sensorarrayRowModeProfile_t *profile,
    const sensorarrayFrameConfigSnapshot_t *rows)
{
    if (!identity || !mode || !profile || !rows) {
        return;
    }
    sensorarrayConfigAuthority_t next = {
        .identity = *identity,
        .stateError = mode->lastError,
        .rowsTerminalId = rows->lastRequestId, .rowsError = rows->lastError,
        .rowsOutcome = !rows->lastTerminal ? SENSORARRAY_CONFIG_OUTCOME_NONE :
            (rows->lastApplied ? SENSORARRAY_CONFIG_OUTCOME_APPLIED : SENSORARRAY_CONFIG_OUTCOME_REJECTED),
        .modeTerminalId = mode->lastRequestId, .modeError = mode->lastRequestError,
        .modeOutcome = mode->lastOutcome,
        .profileTerminalId = profile->lastRequestId, .profileError = profile->lastRequestError,
        .profileOutcome = profile->lastOutcome, .available = true,
    };
    portENTER_CRITICAL(&s_authorityMux);
    s_authority = next;
    portEXIT_CRITICAL(&s_authorityMux);
}

bool sensorarrayConfigIdentityCopy(sensorarrayConfigAuthority_t *outSnapshot)
{
    if (!outSnapshot) {
        return false;
    }
    portENTER_CRITICAL(&s_authorityMux);
    *outSnapshot = s_authority;
    portEXIT_CRITICAL(&s_authorityMux);
    return outSnapshot->available;
}

static const char *sensorarrayConfigOutcomeName(sensorarrayConfigOutcome_t outcome)
{
    return outcome == SENSORARRAY_CONFIG_OUTCOME_APPLIED ? "applied" :
        (outcome == SENSORARRAY_CONFIG_OUTCOME_REJECTED ? "rejected" : "none");
}

esp_err_t sensorarrayConfigIdentityFormat(char *response, size_t responseSize)
{
    if (!response || responseSize == 0u) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayConfigAuthority_t snapshot;
    if (!sensorarrayConfigIdentityCopy(&snapshot)) {
        snprintf(response, responseSize, "ERR,cmd=CONFIG,reason=not_published\n");
        return ESP_ERR_INVALID_STATE;
    }
    const sensorarrayConfigIdentity_t *id = &snapshot.identity;
    bool mixed = false;
    for (size_t row = 1u; row < 8u; ++row) {
        mixed = mixed || id->profile[row] != id->profile[0];
    }
    const char *modeName = mixed && sensorarrayMeasurementModeIsDataMode(id->mode) ?
        "MIXED" : sensorarrayMeasurementModeName(id->mode);
    int written = snprintf(response, responseSize,
        "CONFIG,session=%016llX,bootId=%lu,rev=%lu,boundary=%lu,state=%s,mode=%s,rows=%u,profile=%s,err=0x%lX,"
        "mgen=%lu,mrid=%lu,rgen=%lu,rrid=%lu,pgen=%lu,prid=%lu,"
        "mtermId=%lu,mterm=%s,merr=0x%lX,ptermId=%lu,pterm=%s,perr=0x%lX,rtermId=%lu,rterm=%s,rerr=0x%lX\n",
        (unsigned long long)id->sessionId, (unsigned long)id->bootId,
        (unsigned long)id->revision, (unsigned long)id->appliedFrameBoundary,
        sensorarrayMeasurementStateName(id->state), modeName,
        (unsigned)id->rows, id->profile, (unsigned long)snapshot.stateError,
        (unsigned long)id->modeGeneration, (unsigned long)id->modeRequestId,
        (unsigned long)id->rowsGeneration, (unsigned long)id->rowsRequestId,
        (unsigned long)id->profileGeneration, (unsigned long)id->profileRequestId,
        (unsigned long)snapshot.modeTerminalId, sensorarrayConfigOutcomeName(snapshot.modeOutcome),
        (unsigned long)snapshot.modeError, (unsigned long)snapshot.profileTerminalId,
        sensorarrayConfigOutcomeName(snapshot.profileOutcome), (unsigned long)snapshot.profileError,
        (unsigned long)snapshot.rowsTerminalId, sensorarrayConfigOutcomeName(snapshot.rowsOutcome),
        (unsigned long)snapshot.rowsError);
    if (written < 0 || (size_t)written >= responseSize) {
        response[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}
