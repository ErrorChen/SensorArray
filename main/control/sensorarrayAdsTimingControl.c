#include "sensorarrayAdsTimingControl.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "sensorarrayCommandMailbox.h"
#include "sensorarrayTransportGuaranteedText.h"

#define SENSORARRAY_TIMING_QUEUE_BUDGET_US 10000000u

typedef struct {
    sensorarrayAdsTimingRequest_t request;
    char outcome[12];
    char reason[40];
    esp_err_t originalError;
    esp_err_t cleanupError;
    uint32_t resultRevision;
    uint32_t boundary;
    bool resultKnown;
    bool resultAvailable;
    bool terminalQueued;
} sensorarrayAdsTimingLast_t;

static portMUX_TYPE s_timingMux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint64_t session;
    uint32_t bootId;
    bool busy;
    bool acknowledged;
    bool running;
    bool queryBusy;
    bool initStarted;
    bool storageKnown;
    bool storageReady;
    esp_err_t storageError;
    uint32_t storageBytes;
    sensorarrayAdsTimingRequest_t current;
    sensorarrayAdsTimingLast_t last;
} s_timing;
/* CPU-only diagnostic storage must not consume the internal heap needed for
 * acquisition stacks/DMA. One boot-time PSRAM allocation, no internal fallback
 * and no free/reuse while a worker or query can still hold a reference. */
typedef struct {
    sensorarrayAdsTimingProbeResult_t work;
    sensorarrayAdsTimingProbeResult_t result;
    sensorarrayAdsTimingProbeResult_t query;
} sensorarrayAdsTimingStorage_t;
static sensorarrayAdsTimingStorage_t *s_timingStorage;

void sensorarrayAdsTimingControlInit(uint64_t session, uint32_t bootId)
{
    portENTER_CRITICAL(&s_timingMux);
    if (s_timing.initStarted) { portEXIT_CRITICAL(&s_timingMux); return; }
    s_timing.initStarted = true;
    s_timing.session = session; s_timing.bootId = bootId;
    s_timing.storageError = ESP_ERR_INVALID_STATE;
    portEXIT_CRITICAL(&s_timingMux);
    /* Allocation never occurs in a critical section or hardware transaction. */
    sensorarrayAdsTimingStorage_t *storage = NULL;
    esp_err_t error = ESP_ERR_NOT_SUPPORTED;
#if CONFIG_SPIRAM
    storage = heap_caps_calloc(1u, sizeof(*storage), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    error = storage ? ESP_OK : ESP_ERR_NO_MEM;
#endif
    portENTER_CRITICAL(&s_timingMux);
    s_timingStorage = storage;
    s_timing.storageReady = storage != NULL;
    s_timing.storageBytes = storage ? (uint32_t)sizeof(*storage) : 0u;
    s_timing.storageError = error;
    s_timing.storageKnown = true;
    portEXIT_CRITICAL(&s_timingMux);
}

esp_err_t sensorarrayAdsTimingControlStorageError(void)
{
    portENTER_CRITICAL(&s_timingMux);
    esp_err_t error = s_timing.storageReady ? ESP_OK :
        (s_timing.storageKnown ? s_timing.storageError : ESP_ERR_INVALID_STATE);
    portEXIT_CRITICAL(&s_timingMux);
    return error;
}

esp_err_t sensorarrayAdsTimingControlPost(const sensorarrayConfigAuthority_t *authority,
    uint64_t nowUs, uint32_t *outId)
{
    return sensorarrayAdsTimingControlPostKind(authority, SENSORARRAY_ADS_TIMING_PROBE_TIMING,
        nowUs, outId);
}

esp_err_t sensorarrayAdsTimingControlPostKind(const sensorarrayConfigAuthority_t *authority,
    sensorarrayAdsTimingProbeKind_t kind, uint64_t nowUs, uint32_t *outId)
{
    if (!authority || !authority->available || !outId ||
        (kind != SENSORARRAY_ADS_TIMING_PROBE_TIMING && kind != SENSORARRAY_ADS_TIMING_PROBE_TAIL)) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_timingMux);
    bool busy = s_timing.busy;
    bool bootMatches = authority->identity.sessionId == s_timing.session &&
        authority->identity.bootId == s_timing.bootId;
    bool storageReady = s_timing.storageReady;
    esp_err_t storageError = s_timing.storageKnown ? s_timing.storageError : ESP_ERR_INVALID_STATE;
    portEXIT_CRITICAL(&s_timingMux);
    if (!storageReady) { return storageError; }
    if (busy || !bootMatches) { return ESP_ERR_INVALID_STATE; }
    uint32_t id = 0u;
    esp_err_t error = kind == SENSORARRAY_ADS_TIMING_PROBE_TAIL ?
        sensorarrayCommandMailboxPostAdsTail(SENSORARRAY_ADS_TIMING_SAMPLES, &id) :
        sensorarrayCommandMailboxPostAdsTiming(SENSORARRAY_ADS_TIMING_SAMPLES, &id);
    if (error != ESP_OK) { return error; }
    portENTER_CRITICAL(&s_timingMux);
    s_timing.current = (sensorarrayAdsTimingRequest_t){
        .id = id, .kind = kind, .identity = authority->identity, .acceptedUs = nowUs,
        .expiresUs = nowUs + SENSORARRAY_TIMING_QUEUE_BUDGET_US,
    };
    s_timing.busy = true; s_timing.acknowledged = false; s_timing.running = false;
    portEXIT_CRITICAL(&s_timingMux);
    *outId = id;
    return ESP_OK;
}

bool sensorarrayAdsTimingControlAck(uint32_t id, esp_err_t error)
{
    portENTER_CRITICAL(&s_timingMux);
    bool matches = s_timing.busy && s_timing.current.id == id &&
        !s_timing.acknowledged && !s_timing.running;
    if (matches && error == ESP_OK) {
        /* Set before enqueue: Core 1 may run immediately after the barrier. */
        s_timing.acknowledged = true;
    } else if (matches) {
        s_timing.last = (sensorarrayAdsTimingLast_t){
            .request = s_timing.current, .originalError = error,
        };
        memcpy(s_timing.last.outcome, "rejected", sizeof("rejected"));
        memcpy(s_timing.last.reason, "ack_publish_failed", sizeof("ack_publish_failed"));
        s_timing.busy = false;
    }
    portEXIT_CRITICAL(&s_timingMux);
    if (matches) {
        if (error == ESP_OK) {
            if (!sensorarrayCommandMailboxCommitConfigAck(id)) {
                (void)sensorarrayAdsTimingControlComplete(id, "rejected", "ack_commit_failed",
                    ESP_ERR_INVALID_STATE, ESP_OK, NULL, false, false);
            }
        } else {
            (void)sensorarrayCommandMailboxCancelConfigAck(id);
        }
    }
    return matches;
}

sensorarrayAdsTimingTake_t sensorarrayAdsTimingControlTake(uint32_t id,
    const sensorarrayConfigAuthority_t *authority, bool known, uint64_t nowUs,
    sensorarrayAdsTimingRequest_t *outRequest)
{
    if (!outRequest || (known && !authority)) { return SENSORARRAY_TIMING_TAKE_NONE; }
    portENTER_CRITICAL(&s_timingMux);
    if (!s_timing.storageReady || !s_timing.busy || !s_timing.acknowledged || s_timing.running || s_timing.current.id != id) {
        portEXIT_CRITICAL(&s_timingMux); return SENSORARRAY_TIMING_TAKE_NONE;
    }
    *outRequest = s_timing.current; s_timing.running = true;
    portEXIT_CRITICAL(&s_timingMux);
    if (nowUs >= outRequest->expiresUs) { return SENSORARRAY_TIMING_TAKE_EXPIRED; }
    const sensorarrayConfigIdentity_t *a = &outRequest->identity;
    const sensorarrayConfigIdentity_t *b = known ? &authority->identity : NULL;
    if (!known || !authority->available || a->sessionId != b->sessionId || a->bootId != b->bootId ||
        a->revision != b->revision || a->modeGeneration != b->modeGeneration || a->modeRequestId != b->modeRequestId ||
        a->rowsGeneration != b->rowsGeneration || a->rowsRequestId != b->rowsRequestId ||
        a->profileGeneration != b->profileGeneration || a->profileRequestId != b->profileRequestId) {
        return SENSORARRAY_TIMING_TAKE_STALE;
    }
    memset(&s_timingStorage->work, 0, sizeof(s_timingStorage->work));
    return SENSORARRAY_TIMING_TAKE_READY;
}

sensorarrayAdsTimingProbeResult_t *sensorarrayAdsTimingControlWorkResult(void)
{ return s_timingStorage ? &s_timingStorage->work : NULL; }

esp_err_t sensorarrayAdsTimingControlComplete(uint32_t id, const char *outcome,
    const char *reason, esp_err_t originalError, esp_err_t cleanupError,
    const sensorarrayConfigAuthority_t *authority, bool known, bool keepResult)
{
    if (!outcome || !reason || strlen(outcome) >= sizeof(s_timing.last.outcome) ||
        strlen(reason) >= sizeof(s_timing.last.reason) || (known && !authority)) { return ESP_ERR_INVALID_ARG; }
    sensorarrayAdsTimingLast_t last = {
        .originalError = originalError, .cleanupError = cleanupError,
        .resultKnown = known, .resultAvailable = keepResult,
        .resultRevision = known ? authority->identity.revision : 0u,
        .boundary = known ? authority->identity.appliedFrameBoundary : 0u,
    };
    memcpy(last.outcome, outcome, strlen(outcome) + 1u);
    memcpy(last.reason, reason, strlen(reason) + 1u);
    portENTER_CRITICAL(&s_timingMux);
    if (!s_timing.busy || s_timing.current.id != id || !s_timing.acknowledged ||
        (keepResult && !s_timing.storageReady)) {
        portEXIT_CRITICAL(&s_timingMux); return ESP_ERR_INVALID_STATE;
    }
    last.request = s_timing.current;
    if (keepResult) { s_timingStorage->result = s_timingStorage->work; }
    s_timing.last = last; s_timing.busy = false; s_timing.running = false;
    portEXIT_CRITICAL(&s_timingMux);
    char text[SENSORARRAY_TRANSPORT_GUARANTEED_TEXT_MAX];
    int n = snprintf(text, sizeof(text),
        "ADSTERM,id=%lu,state=%s,reason=%s,session=%016llX,bootId=%lu,acceptedRev=%lu,rev=%lu,boundary=%lu,originalError=0x%lX,cleanupError=0x%lX,result=%u,diagnostic=1,query=ADSTIMING,probe=%s\n",
        (unsigned long)id, outcome, reason, (unsigned long long)last.request.identity.sessionId,
        (unsigned long)last.request.identity.bootId, (unsigned long)last.request.identity.revision,
        (unsigned long)last.resultRevision, (unsigned long)last.boundary,
        (unsigned long)(uint32_t)originalError, (unsigned long)(uint32_t)cleanupError, keepResult ? 1u : 0u,
        sensorarrayAdsTimingProbeKindName(last.request.kind));
    esp_err_t error = n > 0 && (size_t)n < sizeof(text) ?
        sensorarrayCommandMailboxEmitConfigTerminal(id, text, (size_t)n) : ESP_ERR_INVALID_SIZE;
    if (error != ESP_OK) { (void)sensorarrayCommandMailboxCancelConfigAck(id); }
    portENTER_CRITICAL(&s_timingMux);
    if (s_timing.last.request.id == id) { s_timing.last.terminalQueued = error == ESP_OK; }
    portEXIT_CRITICAL(&s_timingMux);
    return error;
}

void sensorarrayAdsTimingControlExpire(uint64_t nowUs,
    const sensorarrayConfigAuthority_t *authority, bool known)
{
    portENTER_CRITICAL(&s_timingMux);
    uint32_t id = s_timing.busy && !s_timing.running && nowUs >= s_timing.current.expiresUs ? s_timing.current.id : 0u;
    bool acknowledged = s_timing.acknowledged;
    portEXIT_CRITICAL(&s_timingMux);
    if (id == 0u) { return; }
    if (!acknowledged) { (void)sensorarrayAdsTimingControlAck(id, ESP_ERR_TIMEOUT); }
    else { (void)sensorarrayAdsTimingControlComplete(id, "rejected", "queue_deadline", ESP_ERR_TIMEOUT,
                                                    ESP_OK, authority, known, false); }
}

bool sensorarrayAdsTimingControlHandles(const char *command)
{ return command && strncmp(command, "ADSTIMING?", 10u) == 0; }

static bool sensorarrayTimingParseUnsigned(const char **text, uint32_t *value)
{
    if (**text < '0' || **text > '9') { return false; }
    uint32_t parsed = 0u;
    while (**text >= '0' && **text <= '9') {
        uint32_t digit = (uint32_t)(**text - '0');
        if (parsed > (UINT32_MAX - digit) / 10u) { return false; }
        parsed = parsed * 10u + digit; ++*text;
    }
    *value = parsed; return true;
}

esp_err_t sensorarrayAdsTimingControlFormat(const char *command, char *response, size_t responseSize)
{
    if (!command || !response || responseSize == 0u) { return ESP_ERR_INVALID_ARG; }
    response[0] = '\0';
    if (strcmp(command, "ADSTIMING?") == 0) {
        portENTER_CRITICAL(&s_timingMux);
        sensorarrayAdsTimingLast_t last = s_timing.last;
        sensorarrayAdsTimingRequest_t current = s_timing.current;
        bool busy = s_timing.busy, running = s_timing.running, ack = s_timing.acknowledged;
        bool storageKnown = s_timing.storageKnown, storageReady = s_timing.storageReady;
        esp_err_t storageError = s_timing.storageError;
        uint32_t storageBytes = s_timing.storageBytes;
        uint64_t session = s_timing.session; uint32_t bootId = s_timing.bootId;
        portEXIT_CRITICAL(&s_timingMux);
        int n = snprintf(response, responseSize,
            "ADSTIMING,session=%016llX,bootId=%lu,busy=%u,phase=%s,ackLocal=%u,pendingId=%lu,pendingRev=%lu,lastId=%lu,last=%s,reason=%s,acceptedRev=%lu,resultKnown=%u,rev=%lu,boundary=%lu,originalError=0x%lX,cleanupError=0x%lX,result=%u,terminalQueued=%u,groups=4,perGroup=200,diagnostic=1,storageKnown=%u,storageReady=%u,storageBytes=%lu,storageError=0x%lX,pendingProbe=%s,lastProbe=%s\n",
            (unsigned long long)session, (unsigned long)bootId, busy ? 1u : 0u,
            busy ? (running ? "running" : "pending") : "idle", busy && ack ? 1u : 0u,
            (unsigned long)(busy ? current.id : 0u), (unsigned long)(busy ? current.identity.revision : 0u),
            (unsigned long)last.request.id, last.request.id ? last.outcome : "none", last.request.id ? last.reason : "none",
            (unsigned long)last.request.identity.revision, last.resultKnown ? 1u : 0u,
            (unsigned long)last.resultRevision, (unsigned long)last.boundary,
            (unsigned long)(uint32_t)last.originalError, (unsigned long)(uint32_t)last.cleanupError,
            last.resultAvailable ? 1u : 0u, last.terminalQueued ? 1u : 0u,
            storageKnown ? 1u : 0u, storageReady ? 1u : 0u,
            (unsigned long)storageBytes, (unsigned long)(uint32_t)storageError,
            busy ? sensorarrayAdsTimingProbeKindName(current.kind) : "none",
            last.request.id ? sensorarrayAdsTimingProbeKindName(last.request.kind) : "none");
        if (n <= 0 || (size_t)n >= responseSize) { response[0] = '\0'; return ESP_ERR_INVALID_SIZE; }
        return ESP_OK;
    }
    const char *number = NULL; unsigned kind = 0u;
    static const struct { const char *prefix; unsigned kind; } queries[] = {
        {"ADSTIMING?RESULT,", 1u}, {"ADSTIMING?GROUP,", 2u},
        {"ADSTIMING?RECORD,", 3u}, {"ADSTIMING?TIME,", 4u}, {"ADSTIMING?REG,", 5u},
        {"ADSTIMING?TAIL,", 6u},
    };
    for (size_t i = 0u; i < sizeof(queries) / sizeof(queries[0]); ++i) {
        size_t length = strlen(queries[i].prefix);
        if (strncmp(command, queries[i].prefix, length) == 0) {
            number = command + length; kind = queries[i].kind; break;
        }
    }
    uint32_t id = 0u, index = 0u;
    if (!number || !sensorarrayTimingParseUnsigned(&number, &id) || !id ||
        (kind != 1u && (*number++ != ',' || !sensorarrayTimingParseUnsigned(&number, &index))) || *number != '\0' ||
        ((kind == 2u || kind == 4u) && index >= SENSORARRAY_ADS_TIMING_GROUPS) || (kind == 5u && index > 1u)) {
        return ESP_ERR_INVALID_ARG;
    }
    portENTER_CRITICAL(&s_timingMux);
    if (!s_timing.storageReady || s_timing.queryBusy || !s_timing.last.resultAvailable || s_timing.last.request.id != id) {
        portEXIT_CRITICAL(&s_timingMux); return ESP_ERR_INVALID_STATE;
    }
    s_timing.queryBusy = true; s_timingStorage->query = s_timingStorage->result;
    portEXIT_CRITICAL(&s_timingMux);
    size_t length = kind == 1u ? sensorarrayAdsTimingProbeFormatSummary(&s_timingStorage->query, response, responseSize) :
        kind == 2u ? sensorarrayAdsTimingProbeFormatGroup(&s_timingStorage->query, index, response, responseSize) :
        kind == 3u ? sensorarrayAdsTimingProbeFormatRecord(&s_timingStorage->query, index, response, responseSize) :
        kind == 4u ? sensorarrayAdsTimingProbeFormatGroupTiming(&s_timingStorage->query, index, response, responseSize) :
        kind == 5u ? sensorarrayAdsTimingProbeFormatRegisters(&s_timingStorage->query, index != 0u, response, responseSize) :
                     sensorarrayAdsTimingProbeFormatTailRecord(&s_timingStorage->query, index, response, responseSize);
    portENTER_CRITICAL(&s_timingMux); s_timing.queryBusy = false; portEXIT_CRITICAL(&s_timingMux);
    return length ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
