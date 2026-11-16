#include <assert.h>
#include <stdarg.h>
#include <limits.h>

/* Reuse the actual mailbox/ScanConfig/guaranteed-text and bounded fake queue. */
#define sensorarrayCommandMailboxPostAdsTiming realMailboxPostAdsTiming
#define main existingMailboxTests
/* EXISTING_MAILBOX_TEST */
#undef main
#undef sensorarrayCommandMailboxPostAdsTiming
#include "sensorarrayAdsTimingControl.h"

static sensorarrayConfigAuthority_t authority;
static bool nestedPost;
static esp_err_t nestedError;
static uint32_t nestedId;
esp_err_t sensorarrayCommandMailboxPostAdsTiming(uint32_t n, uint32_t *id)
{
    esp_err_t error = realMailboxPostAdsTiming(n, id);
    if (error == ESP_OK && nestedPost) {
        nestedPost = false;
        nestedError = sensorarrayAdsTimingControlPost(&authority, 100u, &nestedId);
    }
    return error;
}
#include "sensorarrayAdsTimingControl.c"
/* PRODUCTION_FORMATTERS */

typedef struct { bool applied, known, verified; sensorarrayMeasurementMode_t mode; uint8_t power; } sensorarrayRouteSnapshot_t;
typedef struct { int unused; } sensorarrayState_t;
typedef struct {
    bool systemReady;
    sensorarrayState_t state;
    int adsEngine;
    sensorarrayMeasurementRecovery_t adsRestoreRecovery;
    sensorarrayRouteSnapshot_t routeController;
    uint32_t hostFrameSequence;
} sensorarrayAppContext_t;
typedef struct { unsigned stage, seq, outcome; esp_err_t err; const char *owner; } sensorarrayAdsFaultEvent_t;
#define ADS126X_POWER_INTREF 1u
#define SENSORARRAY_ADS_OWNER_CHECK 4u
#define SENSORARRAY_ADS_FAULT_STAGE_MATRIX_READBACK 1u
#define SENSORARRAY_ADS_FAULT_OUTCOME_FAILED 1u
static bool authorityKnown, held, quiescent, recoveryActive, routeKnown, restoreLatched;
static unsigned probeCalls, stopCalls, faultCalls, runtimeFaultCalls, fillBeforeStop;
static uint64_t nowUs;
static esp_err_t safeError;
static sensorarrayAdsTimingProbeResult_t injectedResult;
bool sensorarrayConfigIdentityCopy(sensorarrayConfigAuthority_t *out)
{ *out = authority; return authorityKnown; }
static int64_t esp_timer_get_time(void) { return (int64_t)nowUs; }
static bool sensorarrayFrontendsHeld(void) { return held; }
static bool sensorarrayFrontendOwnersQuiescent(const sensorarrayAppContext_t *ctx) { return ctx && quiescent; }
bool sensorarrayMeasurementRecoveryIsActive(const sensorarrayMeasurementRecovery_t *r) { (void)r; return recoveryActive; }
static bool sensorarrayAdsGapConsumeRestoreFailure(void) { bool value=restoreLatched;restoreLatched=false;return value; }
static bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteSnapshot_t *route, sensorarrayRouteSnapshot_t *out)
{ *out = *route; return routeKnown; }
static int *sensorarrayAdsMatrixEngineRegisterCache(int *engine) { return engine; }
static esp_err_t sensorarrayAdsGapRunTimingProbe(sensorarrayState_t *state, int *cache,
    uint32_t id, uint32_t seq, sensorarrayAdsTimingProbeResult_t *result)
{
    assert(state && cache && seq == 123u); probeCalls++;
    *result = injectedResult; result->requestId = id; result->frameSequence = seq;
    return result->firstError;
}
static void sensorarrayAdsFaultFillContext(sensorarrayAppContext_t *ctx, sensorarrayAdsFaultEvent_t *fault)
{ assert(ctx && fault); fillBeforeStop = stopCalls + 1u; }
static const char *sensorarrayAdsOwnerName(unsigned owner) { assert(owner == SENSORARRAY_ADS_OWNER_CHECK); return "CHECK"; }
static esp_err_t sensorarrayStopAfterRecoveryFailure(sensorarrayAppContext_t *ctx, esp_err_t original,
    esp_err_t cleanup, const char *reason)
{
    assert(ctx && original != ESP_OK && (strcmp(reason, "ads_timing_restore_failed") == 0 || strcmp(reason,"ads_timing_probe_fault") == 0));
    assert(fillBeforeStop == stopCalls + 1u); stopCalls++;
    authority.identity.state = SENSORARRAY_MEASUREMENT_STATE_DEGRADED;
    authority.identity.mode = SENSORARRAY_MEASUREMENT_MODE_NONE;
    authority.identity.revision++;
    ctx->routeController.verified = false;
    return cleanup != ESP_OK ? cleanup : safeError;
}
static void sensorarrayEmitAdsFault(sensorarrayAppContext_t *ctx, sensorarrayAdsFaultEvent_t *event)
{ assert(ctx && event->err != ESP_OK); faultCalls++; }
static void sensorarrayTransportPublishRuntimeFault(const char *format, ...)
{ assert(strstr(format, "source=ADSTIMING")); runtimeFaultCalls++; }
/* PRODUCTION_BOUNDARY */
static bool sensorarrayRecoveryControlAck(uint32_t id, esp_err_t error) { (void)id; (void)error; return false; }
static bool sensorarrayRestartControlAck(uint32_t id, esp_err_t error) { (void)id; (void)error; return false; }
/* PRODUCTION_CALLBACKS */
static esp_err_t actualTimingCommand(const char *command, char *response, size_t responseSize)
{
/* PRODUCTION_COMMAND */
    return ESP_ERR_NOT_SUPPORTED;
}

static char response[512];
static size_t maxState, maxTerminal;
static unsigned cases;
static void readState(void)
{
    assert(sensorarrayAdsTimingControlFormat("ADSTIMING?", response, sizeof(response)) == ESP_OK);
    if (strlen(response) > maxState) { maxState = strlen(response); }
}
static void drain(void)
{
    char text[384]; size_t length = 0;
    if (sensorarrayTransportGuaranteedTextTakeDrain(text, sizeof(text), &length)) {
        assert(strncmp(text, "ADSTERM,", 8) == 0);
        if (length > maxTerminal) { maxTerminal = length; }
    }
}
static void reset(sensorarrayAppContext_t *ctx)
{
    (void)sensorarrayTransportGuaranteedTextAbort();
    sensorarrayScanConfigEndConfigurationRequest(s_configurationRequestId);
    memset(&s_rowModesReservation, 0, sizeof(s_rowModesReservation));
    s_testQueueHead = s_testQueueCount = 0u;
    authority = (sensorarrayConfigAuthority_t){.available = true, .identity = {
        .sessionId = UINT64_C(0x123456789ABCDEF0), .bootId = 9u, .revision = 7u,
        .appliedFrameBoundary = 120u, .modeGeneration = 3u, .modeRequestId = 2u,
        .rowsGeneration = 5u, .rowsRequestId = 4u, .profileGeneration = 8u, .profileRequestId = 6u,
        .mode = SENSORARRAY_MEASUREMENT_MODE_VOLTAGE, .state = SENSORARRAY_MEASUREMENT_STATE_VOLTAGE}};
    sensorarrayAdsTimingControlInit(authority.identity.sessionId, authority.identity.bootId);
    memset(ctx, 0, sizeof(*ctx)); ctx->systemReady = true; ctx->hostFrameSequence = 123u;
    ctx->routeController = (sensorarrayRouteSnapshot_t){.applied = true, .known = true,
        .verified = true, .mode = SENSORARRAY_MEASUREMENT_MODE_VOLTAGE};
    injectedResult = (sensorarrayAdsTimingProbeResult_t){.completed = true, .restoreKnown = true,
        .restoreVerified = true, .savedKnown = true, .reason = "complete"};
    for (unsigned g = 0; g < 4; g++) {
        injectedResult.groups[g].completed = true; injectedResult.groups[g].reason = "complete";
        injectedResult.groups[g].pulse = (g & 1u) != 0; injectedResult.groups[g].bypass = g >= 2u;
        injectedResult.groups[g].primary.attempted = injectedResult.groups[g].primary.valid = 200u;
    }
    authorityKnown = routeKnown = quiescent = true;
    held = recoveryActive = nestedPost = restoreLatched = false; nowUs = 100u;
    probeCalls = stopCalls = faultCalls = runtimeFaultCalls = fillBeforeStop = 0u;
    safeError = ESP_OK;
}
static uint32_t accept(bool ack)
{
    assert(actualTimingCommand("ADSCHK=TIMING,200", response, sizeof(response)) == ESP_OK);
    unsigned long id = 0; assert(sscanf(response, "ACK,cmd=ADSCHK,timing=1,id=%lu", &id) == 1);
    if (ack) { sensorarrayControlReplyPublished(response, strlen(response)); }
    return (uint32_t)id;
}
static void run(sensorarrayAppContext_t *ctx, uint32_t id)
{
    sensorarrayCommand_t cmd;
    assert(sensorarrayCommandMailboxTryReceive(&cmd));
    assert(cmd.type == SENSORARRAY_COMMAND_ADS_TIMING && cmd.requestId == id);
    sensorarrayRunAdsTimingCommand(ctx, &cmd);
}
static void query(const char *kind, uint32_t id, unsigned index)
{
    char cmd[64];
    if (strcmp(kind, "RESULT") == 0) { snprintf(cmd, sizeof(cmd), "ADSTIMING?%s,%lu", kind, (unsigned long)id); }
    else { snprintf(cmd, sizeof(cmd), "ADSTIMING?%s,%lu,%u", kind, (unsigned long)id, index); }
    assert(sensorarrayAdsTimingControlFormat(cmd, response, sizeof(response)) == ESP_OK);
    assert(strstr(response, "diagnostic=1"));
}
static void testFenceAndArbitration(sensorarrayAppContext_t *ctx)
{
    reset(ctx); nestedPost = true; nestedId = 0u;
    uint32_t id = accept(false); assert(nestedError != ESP_OK && nestedId == 0);
    sensorarrayCommand_t cmd; assert(!sensorarrayCommandMailboxTryReceive(&cmd));
    uint32_t other = 0; assert(sensorarrayCommandMailboxPostMeasurementMode(SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE, &other) != ESP_OK);
    assert(!sensorarrayAdsTimingControlAck(id + 1u, ESP_OK));
    readState(); assert(strstr(response, "ackLocal=0") && strstr(response, "phase=pending"));
    assert(sensorarrayAdsTimingControlAck(id, ESP_OK)); run(ctx, id);
    assert(probeCalls == 1u); readState(); assert(strstr(response, "last=completed") && strstr(response, "result=1"));
    /* A lost terminal never needs another hardware request. */
    query("RESULT", id, 0); assert(strstr(response, "science=0")); drain();
    assert(!sensorarrayAdsTimingControlAck(id, ESP_OK));
    uint32_t next = accept(true); assert(next != id); run(ctx, next); drain(); cases++;

    reset(ctx); id = accept(false);
    sensorarrayControlReplyFailed(ESP_ERR_TIMEOUT, response, strlen(response));
    assert(!sensorarrayCommandMailboxTryReceive(&cmd)); readState();
    assert(strstr(response, "last=rejected") && strstr(response, "reason=ack_publish_failed"));
    assert(probeCalls == 0u); assert(!sensorarrayAdsTimingControlAck(id, ESP_OK));
    next = accept(true); run(ctx, next); drain(); cases++;

    reset(ctx); id = accept(true);
    /* Synthetic loss of the reserved local sink exercises cached recovery
     * even when terminal publication itself fails, without replaying ADC. */
    assert(sensorarrayTransportGuaranteedTextAbort() == ESP_OK);
    run(ctx, id); readState();
    assert(strstr(response, "last=completed") && strstr(response, "terminalQueued=0"));
    query("RESULT", id, 0); assert(probeCalls == 1u);
    next = accept(true); run(ctx, next); drain(); cases++;

    reset(ctx); char tiny[8]; assert(actualTimingCommand("ADSCHK=TIMING,200", tiny, sizeof(tiny)) == ESP_ERR_INVALID_SIZE);
    assert(tiny[0] == 0 && !sensorarrayCommandMailboxTryReceive(&cmd)); readState();
    assert(strstr(response, "ack_publish_failed"));
    assert(actualTimingCommand("ADSCHK=TIMING,201", response, sizeof(response)) == ESP_ERR_INVALID_ARG);
    assert(realMailboxPostAdsTiming(201, &id) == ESP_ERR_INVALID_ARG); cases++;
}
static void testAdmissionAndExpiry(sensorarrayAppContext_t *ctx)
{
    for (unsigned fail = 0; fail < 10; fail++) {
        reset(ctx); uint32_t id = accept(true);
        switch (fail) {
            case 0: authority.identity.revision++; break;
            case 1: authority.identity.sessionId++; break;
            case 2: authorityKnown = false; break;
            case 3: held = true; break;
            case 4: authority.identity.mode = SENSORARRAY_MEASUREMENT_MODE_RESISTANCE; break;
            case 5: quiescent = false; break;
            case 6: recoveryActive = true; break;
            case 7: routeKnown = false; break;
            case 8: ctx->routeController.power = ADS126X_POWER_INTREF; break;
            default: nowUs += SENSORARRAY_TIMING_QUEUE_BUDGET_US; break;
        }
        run(ctx, id); assert(probeCalls == 0u && stopCalls == 0u);
        readState(); assert(strstr(response, "last=rejected") && strstr(response, "result=0")); drain();
    }
    cases++;
    for (unsigned ack = 0; ack < 2; ack++) {
        reset(ctx); uint32_t id = accept(ack != 0);
        nowUs += SENSORARRAY_TIMING_QUEUE_BUDGET_US;
        sensorarrayAdsTimingControlExpire(nowUs, &authority, true);
        readState(); assert(strstr(response, "last=rejected") && strstr(response, "busy=0"));
        if (ack) { run(ctx, id); }
        assert(probeCalls == 0u); drain();
        uint32_t next = accept(true); run(ctx, next); assert(probeCalls == 1u); drain();
    }
    cases++;
}
static void testResultQueriesAndFailure(sensorarrayAppContext_t *ctx)
{
    reset(ctx);
    for (unsigned g = 0; g < 4; g++) {
        injectedResult.groups[g].recordCount = 8u;
        for (unsigned r = 0; r < 8; r++) {
            injectedResult.records[g][r].attempt = r + 1u;
            injectedResult.records[g][r].rawValid = true;
            injectedResult.records[g][r].fresh = true;
        }
    }
    uint32_t id = accept(true); run(ctx, id); drain();
    for (unsigned g = 0; g < 4; g++) { query("GROUP", id, g); query("TIME", id, g); }
    query("REG", id, 0); assert(strstr(response, "phase=saved,known=1"));
    query("REG", id, 1); assert(strstr(response, "phase=restored,known=1"));
    for (unsigned r = 0; r < 32; r++) { query("RECORD", id, r); assert(strstr(response, "wireKnown=0,bytes=unknown,computed=unknown")); }
    char cmd[64]; snprintf(cmd, sizeof(cmd), "ADSTIMING?RECORD,%lu,32", (unsigned long)id);
    assert(sensorarrayAdsTimingControlFormat(cmd, response, sizeof(response)) != ESP_OK && !response[0]);
    assert(sensorarrayAdsTimingControlFormat("ADSTIMING?RESULT,4294967296", response, sizeof(response)) == ESP_ERR_INVALID_ARG);
    assert(sensorarrayAdsTimingControlFormat("ADSTIMING?REG,1,2", response, sizeof(response)) == ESP_ERR_INVALID_ARG);
    uint32_t next = accept(true);
    query("RESULT", id, 0); /* Last completed result remains immutable while pending. */
    run(ctx, next); drain();
    snprintf(cmd, sizeof(cmd), "ADSTIMING?RESULT,%lu", (unsigned long)id);
    assert(sensorarrayAdsTimingControlFormat(cmd, response, sizeof(response)) == ESP_ERR_INVALID_STATE); cases++;

    reset(ctx); injectedResult.firstError = ESP_ERR_INVALID_CRC;
    id = accept(true); run(ctx, id); readState();
    assert(strstr(response, "last=completed") && strstr(response, "originalError=0x109"));
    assert(probeCalls == 1 && stopCalls == 0); drain(); cases++;

    reset(ctx); injectedResult.firstError = ESP_ERR_INVALID_CRC;
    injectedResult.terminalError = ESP_ERR_INVALID_RESPONSE;
    injectedResult.cleanupError = ESP_ERR_TIMEOUT;
    injectedResult.completed = injectedResult.restoreVerified = false;
    injectedResult.requiresIsolation = true; safeError = ESP_ERR_INVALID_STATE;
    id = accept(true); run(ctx, id); readState();
    assert(strstr(response, "last=failed") && strstr(response, "originalError=0x109") && strstr(response, "cleanupError=0x107"));
    assert(authority.identity.state == SENSORARRAY_MEASUREMENT_STATE_DEGRADED && stopCalls == 1 && faultCalls == 1 && runtimeFaultCalls == 1);
    query("RESULT", id, 0); assert(strstr(response, "first=0x109,terminal=0x108,cleanup=0x107")); drain(); cases++;

    reset(ctx); injectedResult.completed = injectedResult.restoreVerified = false;
    injectedResult.requiresIsolation = true; safeError = ESP_ERR_INVALID_RESPONSE;
    id = accept(true); run(ctx, id); readState();
    assert(strstr(response, "originalError=0x103") && strstr(response, "cleanupError=0x108")); drain(); cases++;

    reset(ctx); injectedResult.firstError = injectedResult.terminalError = ESP_ERR_INVALID_RESPONSE;
    injectedResult.requiresIsolation = true; injectedResult.reason = "status_alarm";
    id = accept(true); run(ctx, id); readState();
    assert(strstr(response, "reason=probe_fault_requires_isolation") && strstr(response, "cleanupError=0x0"));
    query("RESULT", id, 0); assert(strstr(response, "reason=status_alarm") && strstr(response, "restoreVerified=1"));
    drain(); cases++;

    reset(ctx); restoreLatched = true;
    id = accept(true); run(ctx, id); readState();
    assert(!restoreLatched && stopCalls == 1u && strstr(response, "cleanupError=0x103"));
    assert(strstr(response, "reason=restore_failed")); drain(); cases++;
}
static void testExtremaAndReset(sensorarrayAppContext_t *ctx)
{
    reset(ctx); authority.identity.bootId = UINT32_MAX; authority.identity.sessionId = UINT64_MAX;
    authority.identity.revision = authority.identity.appliedFrameBoundary = UINT32_MAX;
    sensorarrayAdsTimingControlInit(UINT64_MAX, UINT32_MAX);
    s_nextRequestId = UINT32_MAX; /* Existing shared allocator wraps through nonzero IDs. */
    uint32_t id = accept(true); assert(id != 0u);
    sensorarrayCommand_t command; assert(sensorarrayCommandMailboxTryReceive(&command) && command.requestId == id);
    sensorarrayAdsTimingRequest_t request;
    assert(sensorarrayAdsTimingControlTake(id, &authority, true, nowUs, &request) == SENSORARRAY_TIMING_TAKE_READY);
    assert(sensorarrayAdsTimingControlComplete(id, "rejected", "abcdefghijklmnopqrstuvwxyzabcdefghijklm",
        ESP_FAIL, ESP_FAIL, &authority, true, false) == ESP_OK);
    readState(); assert(strlen(response) < 512); drain();
    assert(maxTerminal < 384);
    uint32_t next = accept(true); assert(next == 1u); readState();
    assert(strstr(response,"pendingId=1,pendingRev=4294967295"));
    run(ctx,next); drain();
    char tiny[8]; assert(sensorarrayAdsTimingControlFormat("ADSTIMING?", tiny, sizeof(tiny)) == ESP_ERR_INVALID_SIZE && !tiny[0]);
    sensorarrayAdsTimingControlInit(1u, 1u); readState(); assert(strstr(response, "lastId=0,last=none")); cases++;
}
int main(void)
{
    assert(sensorarrayCommandMailboxInit() == ESP_OK);
    sensorarrayAppContext_t ctx;
    testFenceAndArbitration(&ctx); testAdmissionAndExpiry(&ctx);
    testResultQueriesAndFailure(&ctx); testExtremaAndReset(&ctx);
    printf("ADS_TIMING_CONTROL,passed=%u,admission_subcases=10,maxState=%zu,maxTerminal=%zu,fixture=synthetic\n", cases, maxState, maxTerminal);
    return 0;
}
