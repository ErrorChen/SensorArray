#include "sensorarrayRecoveryControl.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "sensorarrayTransportGuaranteedText.h"

typedef struct {
    sensorarrayRecoverRequest_t request;
    sensorarrayRecoverState_t outcome;
    char reason[32];
    esp_err_t originalError;
    esp_err_t cleanupError;
    uint32_t resultRevision;
    uint32_t frameBoundary;
    bool resultKnown;
    bool routeApplied;
    bool routeKnown;
    bool routeVerified;
    bool terminalQueued;
} sensorarrayRecoverResult_t;

static portMUX_TYPE s_recoverMux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t nextId;
    bool busy;
    bool acknowledged;
    sensorarrayRecoverState_t phase;
    sensorarrayRecoverRequest_t current;
    sensorarrayRecoverResult_t last;
} s_recover;

static const char *sensorarrayRecoverStateName(sensorarrayRecoverState_t state)
{
    switch (state) {
    case SENSORARRAY_RECOVER_PENDING: return "pending";
    case SENSORARRAY_RECOVER_RUNNING: return "running";
    case SENSORARRAY_RECOVER_APPLIED: return "applied";
    case SENSORARRAY_RECOVER_REJECTED: return "rejected";
    case SENSORARRAY_RECOVER_RESTARTING: return "restarting";
    default: return "none";
    }
}

void sensorarrayRecoveryControlInit(uint64_t sessionId, uint32_t bootId)
{
    portENTER_CRITICAL(&s_recoverMux);
    memset(&s_recover, 0, sizeof(s_recover));
    s_recover.sessionId = sessionId;
    s_recover.bootId = bootId;
    portEXIT_CRITICAL(&s_recoverMux);
}

esp_err_t sensorarrayRecoveryControlPost(uint32_t level,
    const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId)
{
    if (level > 2u || (known && !authority)) { return ESP_ERR_INVALID_ARG; }
    /* Share the existing MODE/ROWS/ROWMODES reserved terminal lane. A request
     * is never ACKed without bounded storage for its eventual terminal. */
    esp_err_t err = sensorarrayTransportGuaranteedTextReserve();
    if (err != ESP_OK) { return err; }
    portENTER_CRITICAL(&s_recoverMux);
    if (s_recover.busy) {
        portEXIT_CRITICAL(&s_recoverMux);
        (void)sensorarrayTransportGuaranteedTextAbort();
        return ESP_ERR_INVALID_STATE;
    }
    s_recover.nextId++;
    if (s_recover.nextId == 0u) { s_recover.nextId = 1u; }
    s_recover.current = (sensorarrayRecoverRequest_t){
        .id = s_recover.nextId, .level = level, .identityKnown = known,
        .identity = authority ? authority->identity : (sensorarrayConfigIdentity_t){0},
    };
    if (!known) {
        s_recover.current.identity.sessionId = s_recover.sessionId;
        s_recover.current.identity.bootId = s_recover.bootId;
    }
    s_recover.busy = true;
    s_recover.acknowledged = false;
    s_recover.phase = SENSORARRAY_RECOVER_PENDING;
    if (outId) { *outId = s_recover.current.id; }
    portEXIT_CRITICAL(&s_recoverMux);
    return ESP_OK;
}

bool sensorarrayRecoveryControlAck(uint32_t id, esp_err_t publishError)
{
    portENTER_CRITICAL(&s_recoverMux);
    bool matches = s_recover.busy && s_recover.current.id == id &&
        s_recover.phase == SENSORARRAY_RECOVER_PENDING && !s_recover.acknowledged;
    if (matches && publishError == ESP_OK) {
        s_recover.acknowledged = true;
    } else if (matches) {
        s_recover.last = (sensorarrayRecoverResult_t){
            .request = s_recover.current, .outcome = SENSORARRAY_RECOVER_REJECTED,
            .originalError = publishError,
        };
        memcpy(s_recover.last.reason, "ack_publish_failed", sizeof("ack_publish_failed"));
        s_recover.busy = false;
        s_recover.phase = SENSORARRAY_RECOVER_IDLE;
    }
    portEXIT_CRITICAL(&s_recoverMux);
    if (matches) {
        if (publishError == ESP_OK) {
            (void)sensorarrayTransportGuaranteedTextRelease();
        } else {
            (void)sensorarrayTransportGuaranteedTextAbort();
        }
    }
    return matches;
}

sensorarrayRecoverTake_t sensorarrayRecoveryControlTake(
    const sensorarrayConfigAuthority_t *authority, bool known,
    sensorarrayRecoverRequest_t *outRequest)
{
    if (!outRequest || (known && !authority)) { return SENSORARRAY_RECOVER_TAKE_NONE; }
    portENTER_CRITICAL(&s_recoverMux);
    if (!s_recover.busy || !s_recover.acknowledged ||
        s_recover.phase != SENSORARRAY_RECOVER_PENDING) {
        portEXIT_CRITICAL(&s_recoverMux);
        return SENSORARRAY_RECOVER_TAKE_NONE;
    }
    *outRequest = s_recover.current;
    s_recover.phase = SENSORARRAY_RECOVER_RUNNING;
    portEXIT_CRITICAL(&s_recoverMux);
    const sensorarrayConfigIdentity_t *accepted = &outRequest->identity;
    const sensorarrayConfigIdentity_t *current = known ? &authority->identity : NULL;
    bool stale = outRequest->identityKnown != known || (known &&
        (accepted->sessionId != current->sessionId || accepted->bootId != current->bootId ||
         accepted->revision != current->revision ||
         accepted->modeGeneration != current->modeGeneration || accepted->modeRequestId != current->modeRequestId ||
         accepted->rowsGeneration != current->rowsGeneration || accepted->rowsRequestId != current->rowsRequestId ||
         accepted->profileGeneration != current->profileGeneration || accepted->profileRequestId != current->profileRequestId));
    return stale ? SENSORARRAY_RECOVER_TAKE_STALE : SENSORARRAY_RECOVER_TAKE_READY;
}

esp_err_t sensorarrayRecoveryControlComplete(uint32_t id,
    sensorarrayRecoverState_t outcome, const char *reason,
    esp_err_t originalError, esp_err_t cleanupError,
    const sensorarrayConfigAuthority_t *authority, bool known,
    bool routeApplied, bool routeKnown, bool routeVerified)
{
    if (outcome < SENSORARRAY_RECOVER_APPLIED || outcome > SENSORARRAY_RECOVER_RESTARTING ||
        !reason || strlen(reason) >= sizeof(s_recover.last.reason) || (known && !authority)) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayRecoverResult_t result = {
        .outcome = outcome, .originalError = originalError, .cleanupError = cleanupError,
        .resultKnown = known, .resultRevision = known ? authority->identity.revision : 0u,
        .frameBoundary = known ? authority->identity.appliedFrameBoundary : 0u,
        .routeApplied = routeApplied, .routeKnown = routeKnown, .routeVerified = routeVerified,
    };
    memcpy(result.reason, reason, strlen(reason) + 1u);
    portENTER_CRITICAL(&s_recoverMux);
    if (!s_recover.busy || s_recover.current.id != id || !s_recover.acknowledged ||
        s_recover.phase != SENSORARRAY_RECOVER_RUNNING) {
        portEXIT_CRITICAL(&s_recoverMux);
        return ESP_ERR_INVALID_STATE;
    }
    result.request = s_recover.current;
    s_recover.last = result; /* Queryable before attempting any delivery. */
    s_recover.busy = false;
    s_recover.phase = SENSORARRAY_RECOVER_IDLE;
    portEXIT_CRITICAL(&s_recoverMux);
    /* Preserve the established pre-execution RERR tags and their small
     * reason-specific fields. Other legacy recovery outcomes use RAPP. */
    const char *tag = "RAPP";
    char legacySuffix[32] = {0};
    if (strcmp(reason, "stale_config") == 0) {
        tag = "RERR";
        (void)snprintf(legacySuffix, sizeof(legacySuffix), ",currentRev=%lu",
                       (unsigned long)result.resultRevision);
    } else if (strcmp(reason, "wait_frontends") == 0) {
        tag = "RERR";
        memcpy(legacySuffix, ",route=UNKNOWN", sizeof(",route=UNKNOWN"));
    } else if (strcmp(reason, "fdc_restart_required") == 0) {
        tag = "RERR";
        memcpy(legacySuffix, ",restartRequired=1", sizeof(",restartRequired=1"));
    }
    char text[SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX];
    int n = snprintf(text, sizeof(text),
        "%s,cmd=RECOVER,id=%lu,level=%lu,state=%s,reason=%s,session=%016llX,bootId=%lu,acceptedKnown=%u,acceptedRev=%lu,resultKnown=%u,rev=%lu,boundary=%lu,err=0x%lX,cleanupError=0x%lX,routeApplied=%u,routeKnown=%u,routeVerified=%u,physicalSafe=unknown%s\n",
        tag, (unsigned long)id, (unsigned long)result.request.level, sensorarrayRecoverStateName(outcome), reason,
        (unsigned long long)result.request.identity.sessionId, (unsigned long)result.request.identity.bootId,
        result.request.identityKnown ? 1u : 0u, (unsigned long)result.request.identity.revision,
        known ? 1u : 0u, (unsigned long)result.resultRevision, (unsigned long)result.frameBoundary,
        (unsigned long)(uint32_t)originalError, (unsigned long)(uint32_t)cleanupError,
        routeApplied ? 1u : 0u, routeKnown ? 1u : 0u, routeVerified ? 1u : 0u, legacySuffix);
    esp_err_t err = n > 0 && (size_t)n < sizeof(text) ?
        sensorarrayTransportGuaranteedTextPublish(text, (size_t)n) : ESP_ERR_INVALID_SIZE;
    if (err != ESP_OK) { (void)sensorarrayTransportGuaranteedTextAbort(); }
    portENTER_CRITICAL(&s_recoverMux);
    if (s_recover.last.request.id == id) { s_recover.last.terminalQueued = err == ESP_OK; }
    portEXIT_CRITICAL(&s_recoverMux);
    return err;
}

esp_err_t sensorarrayRecoveryControlFormat(char *response, size_t responseSize)
{
    if (!response || responseSize == 0u) { return ESP_ERR_INVALID_ARG; }
    sensorarrayRecoverRequest_t current;
    sensorarrayRecoverResult_t last;
    sensorarrayRecoverState_t phase;
    uint64_t sessionId;
    uint32_t bootId;
    bool busy;
    portENTER_CRITICAL(&s_recoverMux);
    current = s_recover.current; last = s_recover.last; phase = s_recover.phase;
    sessionId = s_recover.sessionId; bootId = s_recover.bootId; busy = s_recover.busy;
    portEXIT_CRITICAL(&s_recoverMux);
    int n = snprintf(response, responseSize,
        "RECOVERY,session=%016llX,bootId=%lu,busy=%u,phase=%s,pendingId=%lu,pendingLevel=%lu,pendingKnown=%u,pendingRev=%lu,lastId=%lu,lastLevel=%lu,last=%s,reason=%s,acceptedKnown=%u,acceptedRev=%lu,resultKnown=%u,rev=%lu,boundary=%lu,originalError=0x%lX,cleanupError=0x%lX,routeApplied=%u,routeKnown=%u,routeVerified=%u,terminalQueued=%u,physicalSafe=unknown\n",
        (unsigned long long)sessionId, (unsigned long)bootId, busy ? 1u : 0u, sensorarrayRecoverStateName(phase),
        (unsigned long)(busy ? current.id : 0u), (unsigned long)(busy ? current.level : 0u),
        busy && current.identityKnown ? 1u : 0u, (unsigned long)(busy ? current.identity.revision : 0u),
        (unsigned long)last.request.id, (unsigned long)last.request.level, sensorarrayRecoverStateName(last.outcome),
        last.request.id ? last.reason : "none", last.request.identityKnown ? 1u : 0u,
        (unsigned long)last.request.identity.revision, last.resultKnown ? 1u : 0u,
        (unsigned long)last.resultRevision, (unsigned long)last.frameBoundary,
        (unsigned long)(uint32_t)last.originalError, (unsigned long)(uint32_t)last.cleanupError,
        last.routeApplied ? 1u : 0u, last.routeKnown ? 1u : 0u, last.routeVerified ? 1u : 0u,
        last.terminalQueued ? 1u : 0u);
    if (n <= 0 || (size_t)n >= responseSize) { response[0] = '\0'; return ESP_ERR_INVALID_SIZE; }
    return ESP_OK;
}
