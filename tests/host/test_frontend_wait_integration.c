#define sensorarrayTransportPublishRuntimeFault printf /* Transport is synthetic in this main-logic harness. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_ERR_NO_MEM 0x101
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_INVALID_SIZE 0x104
#define ESP_ERR_NOT_SUPPORTED 0x106
#define SENSORARRAY_MATRIX_ROWS 8
#define SENSORARRAY_MATRIX_COLS 8
#define SENSORARRAY_MEASUREMENT_MODE_NONE 0
#define SENSORARRAY_BOOT_RESTART_KIND_AUTO 2
/* PRESENCE_TYPES */
typedef struct { bool quarantined, spiInFlight; } mockAds_t;
typedef struct { bool valid; } mockCache_t;
typedef struct {
    mockAds_t ads;
    mockCache_t fdcCellCache[8][8];
    mockCache_t fdcAppliedRow[2];
} sensorarrayState_t;
typedef struct { bool pending; uint32_t pendingRequestId; } sensorarrayMeasurementModeSnapshot_t;
typedef sensorarrayMeasurementModeSnapshot_t sensorarrayRowModeProfile_t;
typedef struct { bool fdcSdVerified, fdcSdHigh; } sensorarrayRouteSnapshot_t;
typedef struct {
    sensorarrayState_t state;
    sensorarrayMeasurementModeSnapshot_t measurementMode;
    sensorarrayRowModeProfile_t rowModeProfile;
    sensorarrayRouteSnapshot_t routeController;
    uint32_t hostFrameSequence;
    struct { uint32_t sequence; } frame;
    bool frontendFdcSdKnownLow, frontendStartupReached, systemReady;
    bool pendingAdsCheck;
    uint32_t pendingBatteryRequestId;
    int adsEngine, adsRestoreRecovery;
} sensorarrayAppContext_t;
typedef enum { SENSORARRAY_COMMAND_ROWS, SENSORARRAY_COMMAND_MEASUREMENT_MODE,
    SENSORARRAY_COMMAND_ROW_MODES, MOCK_AUX } sensorarrayCommandType_t;
typedef struct { sensorarrayCommandType_t type; uint32_t requestId; } sensorarrayCommand_t;
typedef struct { uint32_t requestId; } sensorarrayScanConfigRequestResult_t;
static sensorarrayFrontendPresence_t s_frontendPresence;
static uint32_t s_frontendHold, s_frontendCommandsReady;
static bool s_frontendRouteWaitEntered, s_frontendRestartAttempted;
static bool mockBootOwnerFree, mockFdcOwnerFree, mockFdcQuarantined;
static bool mockColdPresenceBusy, mockColdPresenceQuarantined;
static bool mockDue, mockBudgetAllowed, mockLastOwner, mockLastSdHigh;
static unsigned mockRoute, mockInvalidate, mockTerminals, mockRowsPost, mockBudget, mockRestart;
static unsigned mockDriverRead, mockPoll, mockDrain, mockFault, mockStableFalse;
static esp_err_t mockRouteError;
static uint64_t mockNowUs;
static char mockLastTerminal[384];
static sensorarrayFrontendPresenceAction_t mockAction;
static sensorarrayFrontendPresenceSnapshot_t mockNextSnapshot;
static int64_t esp_timer_get_time(void) { return (int64_t)mockNowUs; }
static bool sensorarrayFrontendStartupAdsOwnerQuiescent(void) { return mockBootOwnerFree; }
static bool sensorarrayBringupAdsPresenceBusy(void) { return mockColdPresenceBusy; }
static bool sensorarrayBringupAdsPresenceQuarantined(void) { return mockColdPresenceQuarantined; }
static bool sensorarrayMeasureFdcWorkersQuiescent(void) { return mockFdcOwnerFree; }
static bool sensorarrayMeasureFdcWorkersQuarantined(void) { return mockFdcQuarantined; }
static bool ads126xAdcIsQuarantined(const mockAds_t *ads) { mockDriverRead++; return ads->quarantined; }
static esp_err_t sensorarrayCommandMailboxPostRows(uint8_t rows, sensorarrayScanConfigRequestResult_t *out)
{ (void)out; assert(rows == 4); mockRowsPost++; return ESP_OK; }
static void sensorarrayScanConfigRecordRejected(uint32_t id, uint32_t error)
{ assert(id && error == ESP_ERR_INVALID_STATE); }
static void sensorarrayMeasurementModeRejectRequest(void *ctx, uint32_t id, uint32_t err)
{ (void)ctx; assert(id && err == ESP_ERR_INVALID_STATE); }
static void sensorarrayRowModeProfileRejectRequest(void *ctx, uint32_t id, uint32_t err)
{ (void)ctx; assert(id && err == ESP_ERR_INVALID_STATE); }
static void sensorarrayEmitConfigTerminal(sensorarrayAppContext_t *ctx, uint32_t id,
    uint32_t boundary, const char *format, ...)
{
    assert(ctx && id && boundary == ctx->hostFrameSequence + 1);
    va_list args; va_start(args, format);
    vsnprintf(mockLastTerminal, sizeof(mockLastTerminal), format, args);
    va_end(args); mockTerminals++;
}
static const char *sensorarrayCommandMailboxTypeName(sensorarrayCommandType_t type)
{ (void)type; return "AUX"; }
static void sensorarrayBootBreadcrumbMarkStableProgress(bool healthy)
{ assert(!healthy); mockStableFalse++; }
static esp_err_t sensorarrayRouteControllerEnterFrontendWait(void *route, sensorarrayState_t *state)
{ (void)route; (void)state; mockRoute++; return mockRouteError; }
static void sensorarrayAdsMatrixEngineInvalidateCaches(int *engine)
{ (void)engine; mockInvalidate++; }
static esp_err_t sensorarrayAdsMatrixEngineSetMode(int *engine, unsigned mode)
{ (void)engine; assert(mode == 0); return ESP_OK; }
static void sensorarrayFdcSweepDiscardPendingForConfigChange(void) { }
static void sensorarrayMeasurementRecoveryInit(int *recovery) { *recovery = 0; }
static bool sensorarrayMeasurementModeCopySnapshot(const sensorarrayMeasurementModeSnapshot_t *from,
    sensorarrayMeasurementModeSnapshot_t *to) { *to = *from; return true; }
static bool sensorarrayRowModeProfileCopy(const sensorarrayRowModeProfile_t *from,
    sensorarrayRowModeProfile_t *to) { *to = *from; return true; }
static void sensorarrayMeasurementModeFailTransition(sensorarrayMeasurementModeSnapshot_t *ctx,
    uint32_t err, uint64_t us) { assert(err && us == 0); ctx->pending = false; }
static void sensorarrayRowModeProfileFailTransition(sensorarrayRowModeProfile_t *ctx,
    uint32_t err, uint64_t us) { assert(err && us == 0); ctx->pending = false; }
static void sensorarrayMeasurementModeRecordRuntimeFault(void *ctx, uint32_t err)
{ (void)ctx; assert(err); mockFault++; }
static int sensorarrayPublishConfigAuthority(sensorarrayAppContext_t *ctx, uint32_t boundary)
{ assert(boundary == ctx->hostFrameSequence + 1); return 0; }
static void sensorarrayBootBreadcrumbSetStage(const char *stage, esp_err_t err, const sensorarrayAppContext_t *ctx)
{ (void)err; assert(stage && ctx); }
static bool sensorarrayFrontendPresenceDue(const sensorarrayFrontendPresence_t *p, uint64_t now)
{ (void)p; (void)now; return mockDue; }
static void sensorarrayFrontendPresenceLatchHold(sensorarrayFrontendPresence_t *p, uint64_t now)
{
    if (p->published.phase == SENSORARRAY_FRONTEND_PRESENCE_MONITOR) {
        p->published.phase = SENSORARRAY_FRONTEND_PRESENCE_SUSPECT;
        p->published.nextProbeUs = now;
    }
}
static bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteSnapshot_t *from,
    sensorarrayRouteSnapshot_t *to) { *to = *from; return true; }
static sensorarrayFrontendPresenceAction_t sensorarrayFrontendPresencePoll(
    sensorarrayFrontendPresence_t *p, sensorarrayState_t *state, uint64_t now,
    bool ownerFree, bool sdHigh, sensorarrayFrontendPresenceSnapshot_t *out)
{
    (void)state; (void)now;
    mockPoll++; mockLastOwner = ownerFree; mockLastSdHigh = sdHigh;
    p->published = mockNextSnapshot; *out = p->published; mockDue = false;
    return mockAction;
}
static bool sensorarrayFrontendPresenceIsWaiting(const sensorarrayFrontendPresence_t *p)
{ return p->published.initialized && p->published.phase != SENSORARRAY_FRONTEND_PRESENCE_MONITOR; }
static bool sensorarrayFrontendPresenceCopySnapshot(const sensorarrayFrontendPresence_t *p,
    sensorarrayFrontendPresenceSnapshot_t *out) { *out = p->published; return out->initialized; }
static const char *sensorarrayFrontendPresencePhaseName(sensorarrayFrontendPresencePhase_t phase)
{ (void)phase; return "restart_required"; }
static const char *sensorarrayFrontendPresenceLinkName(sensorarrayFrontendLink_t link)
{ (void)link; return "unavailable"; }
static bool sensorarrayBootPrepareRestart(unsigned kind)
{ assert(kind == SENSORARRAY_BOOT_RESTART_KIND_AUTO); mockBudget++; return mockBudgetAllowed; }
static void sensorarrayBootEmitRestartMarker(const char *marker) { assert(strstr(marker, "restarting")); }
static void sensorarrayUsbOtgRestart(bool rom) { (void)rom; mockRestart++; }
static bool sensorarrayUsbControlHandles(const char *cmd) { (void)cmd; return false; }
static esp_err_t sensorarrayUsbControlHandle(const char *cmd,char *text,size_t size) { (void)cmd;(void)text;(void)size;return ESP_ERR_NOT_SUPPORTED; }
static esp_err_t sensorarrayLinkDiagnosticsFormat(const char *cmd,char *text,size_t size) { int n=snprintf(text,size,"%.*s,mock=1\n",(int)strlen(cmd)-1,cmd);return n>0&&(size_t)n<size ? ESP_OK : ESP_ERR_INVALID_SIZE; }
static const uint64_t s_configSessionId = UINT64_C(0x123456789ABCDEF0);
static bool sensorarrayCpuOwnerHandles(const char *cmd) { return strncmp(cmd,"CPUOWNER?",9u)==0; }
static esp_err_t sensorarrayCpuOwnerFormat(const char *cmd,uint64_t session,char *text,size_t size)
{ (void)cmd; assert(session==s_configSessionId); int n=snprintf(text,size,"CPUOWNER,mock=1\n");return n>0&&(size_t)n<size ? ESP_OK : ESP_ERR_INVALID_SIZE; }
static bool sensorarrayLinkDiagnosticsHandles(const char *cmd) { return strcmp(cmd,"BLELINK?")==0 || strcmp(cmd,"BLETX?")==0 || strcmp(cmd,"BLESVC?")==0 || strcmp(cmd,"SCHEDWHY?")==0; }
static bool sensorarrayAdsTimingControlHandles(const char *cmd) { return strncmp(cmd,"ADSTIMING?",10)==0; }
static esp_err_t sensorarrayAdsTimingControlFormat(const char *cmd,char *text,size_t size) { (void)cmd;int n=snprintf(text,size,"ADSTIMING,mock=1\n");return n>0&&(size_t)n<size ? ESP_OK : ESP_ERR_INVALID_SIZE; }
static void sensorarrayApplyPendingCommands(sensorarrayAppContext_t *ctx)
{ assert(ctx); mockDrain++; }
static esp_err_t sensorarrayFrontendStartupFormatSpi(char *text, size_t capacity)
{ snprintf(text, capacity, "SPI,mock=1\n"); return ESP_OK; }
static esp_err_t sensorarrayFrontendStartupFormatSpiSource(char *text, size_t capacity)
{ snprintf(text, capacity, "SPISOURCE,mock=1\n"); return ESP_OK; }
/* Formatting has separate production-module extrema tests. These four
 * boundary stubs let this test execute the actual main dispatch while held. */
static unsigned mockDiagnosticQueryCalls[4];
static esp_err_t mockDiagnosticQuery(unsigned which, char *text, size_t capacity)
{
    static const char *tags[] = {"RECOVERY", "RESTART", "SERIAL", "SCHED"};
    assert(which < 4); mockDiagnosticQueryCalls[which]++;
    int n = snprintf(text, capacity, "%s,mock=1\n", tags[which]);
    return n > 0 && (size_t)n < capacity ? ESP_OK : ESP_ERR_INVALID_SIZE;
}
static esp_err_t sensorarrayRecoveryControlFormat(char *text, size_t capacity)
{ return mockDiagnosticQuery(0, text, capacity); }
static esp_err_t sensorarrayRestartControlFormat(char *text, size_t capacity)
{ return mockDiagnosticQuery(1, text, capacity); }
static esp_err_t sensorarrayFormatSerialReply(char *text, size_t capacity)
{ return mockDiagnosticQuery(2, text, capacity); }
static esp_err_t sensorarrayFormatSchedulerReply(char *text, size_t capacity)
{ return mockDiagnosticQuery(3, text, capacity); }
/* ACTUAL_FUNCTIONS */

static void reset(sensorarrayAppContext_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));
    for (unsigned r = 0; r < 8; ++r) for (unsigned c = 0; c < 8; ++c) {
        ctx->state.fdcCellCache[r][c].valid = true;
    }
    ctx->state.fdcAppliedRow[0].valid = ctx->state.fdcAppliedRow[1].valid = true;
    ctx->systemReady = true;
    ctx->frontendFdcSdKnownLow = true;
    s_frontendHold = s_frontendCommandsReady = 0;
    s_frontendRouteWaitEntered = s_frontendRestartAttempted = false;
    mockBootOwnerFree = mockFdcOwnerFree = true;
    mockFdcQuarantined = mockDue = mockBudgetAllowed = false;
    mockColdPresenceBusy = mockColdPresenceQuarantined = false;
    mockRoute = mockInvalidate = mockTerminals = mockRowsPost = mockBudget = mockRestart = 0;
    mockDriverRead = mockPoll = mockDrain = mockFault = mockStableFalse = 0;
    mockRouteError = ESP_OK;
    mockNowUs = 1000000;
    mockAction = SENSORARRAY_FRONTEND_PRESENCE_NONE;
    mockNextSnapshot = (sensorarrayFrontendPresenceSnapshot_t){ .initialized = true };
    s_frontendPresence.published = mockNextSnapshot;
}
static void testOwnershipAndHold(void)
{
    sensorarrayAppContext_t ctx;
    reset(&ctx);
    mockBootOwnerFree = false;
    assert(!sensorarrayFrontendOwnersQuiescent(&ctx) && mockDriverRead == 0);
    sensorarrayEnterFrontendHold(&ctx, "synthetic_boot_busy");
    assert(sensorarrayFrontendsHeld() && !ctx.systemReady);
    assert(mockRoute == 0 && mockInvalidate == 0);
    reset(&ctx);
    mockColdPresenceBusy = mockColdPresenceQuarantined = true;
    assert(!sensorarrayFrontendOwnersQuiescent(&ctx) && mockDriverRead == 0);
    sensorarrayEnterFrontendHold(&ctx, "synthetic_cold_dma_unknown");
    assert(sensorarrayFrontendsHeld() && mockRoute == 0 && mockInvalidate == 0);
    reset(&ctx);
    ctx.state.ads.quarantined = true;
    sensorarrayEnterFrontendHold(&ctx, "synthetic_dma_unknown");
    assert(mockRoute == 1 && mockInvalidate == 0 && !s_frontendRouteWaitEntered);
    reset(&ctx);
    ctx.measurementMode = (sensorarrayMeasurementModeSnapshot_t){true, 11};
    ctx.rowModeProfile = (sensorarrayRowModeProfile_t){true, 12};
    mockRouteError = ESP_FAIL;
    sensorarrayEnterFrontendHold(&ctx, "synthetic_gpio_failure");
    assert(mockRoute == 1 && mockInvalidate == 1 && mockFault == 1 && mockTerminals == 2);
    assert(!ctx.measurementMode.pending && !ctx.rowModeProfile.pending);
    assert(!ctx.state.fdcCellCache[7][7].valid && !ctx.state.fdcAppliedRow[1].valid);
    sensorarrayEnterFrontendHold(&ctx, "repeat");
    assert(mockRoute == 1 && sensorarrayFrontendsHeld());
}
static void testCommands(void)
{
    sensorarrayAppContext_t ctx;
    reset(&ctx);
    assert(sensorarrayFrontendRowsRequest(4, NULL) == ESP_ERR_INVALID_STATE);
    s_frontendCommandsReady = 1;
    assert(sensorarrayFrontendRowsRequest(4, NULL) == ESP_OK && mockRowsPost == 1);
    s_frontendHold = 1;
    assert(sensorarrayFrontendRowsRequest(4, NULL) == ESP_ERR_INVALID_STATE);
    for (unsigned type = 0; type < 3; ++type) {
        sensorarrayCommand_t command = {(sensorarrayCommandType_t)type, 25 + type};
        sensorarrayRejectHeldCommand(&ctx, &command);
        assert(strstr(mockLastTerminal, "reason=wait_frontends"));
    }
    assert(mockTerminals == 3);
    char reply[512];
    const char *rejected[] = {"MODE=CAP", "ROWMODES=CCCCCCCC", "ADSCHK=1", "CALRAIL"};
    for (unsigned i = 0; i < 4; ++i) {
        assert(sensorarrayRuntimeQueryCommand(rejected[i], reply, sizeof(reply), &ctx) == ESP_ERR_INVALID_STATE);
    }
    const char *newQueries[] = {"RECOVERY?", "RESTART?", "SERIAL?", "SCHED?"};
    const char *expected[] = {"RECOVERY,mock=1\n", "RESTART,mock=1\n", "SERIAL,mock=1\n", "SCHED,mock=1\n"};
    unsigned readsBefore = mockDriverRead, routesBefore = mockRoute;
    unsigned resetsBefore = mockRestart, terminalsBefore = mockTerminals, postsBefore = mockRowsPost;
    memset(mockDiagnosticQueryCalls, 0, sizeof(mockDiagnosticQueryCalls));
    for (unsigned i = 0; i < 4; ++i) {
        assert(sensorarrayRuntimeQueryCommand(newQueries[i], reply, sizeof(reply), &ctx) == ESP_OK);
        assert(strcmp(reply, expected[i]) == 0 && mockDiagnosticQueryCalls[i] == 1);
        assert(sensorarrayRuntimeQueryCommand(newQueries[i], reply, sizeof(reply), NULL) == ESP_OK);
        assert(strcmp(reply, expected[i]) == 0 && mockDiagnosticQueryCalls[i] == 2);
    }
    const char *timingQueries[] = {"ADSTIMING?", "BLESVC?", "SCHEDWHY?", "SPISOURCE?", "CPUOWNER?0", "CPUOWNER?6"};
    for (unsigned i = 0; i < sizeof(timingQueries)/sizeof(timingQueries[0]); ++i) {
        assert(sensorarrayRuntimeQueryCommand(timingQueries[i], reply, sizeof(reply), &ctx) == ESP_OK);
        assert(strstr(reply, ",mock=1\n"));
        assert(sensorarrayRuntimeQueryCommand(timingQueries[i], reply, sizeof(reply), NULL) == ESP_OK);
    }
    assert(mockDriverRead == readsBefore && mockRoute == routesBefore && mockRestart == resetsBefore);
    assert(mockTerminals == terminalsBefore && mockRowsPost == postsBefore && s_frontendHold == 1);
    puts("FRONTEND_HELD_QUERY_DISPATCH,queries=10,with_context_and_null=1,hardware_side_effects=0");
    const char *allowed[] = {"SPI?", "FRONTENDS?", "CONFIG?", "USBSTREAM?", "USBSTREAM=DEBUG", "RESTART", "RECOVER=2"};
    for (unsigned i = 0; i < 7; ++i) {
        assert(sensorarrayRuntimeQueryCommand(allowed[i], reply, sizeof(reply), &ctx) == ESP_OK);
    }
}
static void testBoundaryAndRestart(void)
{
    sensorarrayAppContext_t ctx;
    reset(&ctx);
    mockDue = true; mockAction = SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT;
    mockNextSnapshot.phase = SENSORARRAY_FRONTEND_PRESENCE_WAIT;
    assert(sensorarrayRunFrontendPresenceBoundary(&ctx));
    assert(mockPoll == 1 && mockLastOwner && !mockLastSdHigh && mockRoute == 1);
    assert(mockDrain == 1 && mockRestart == 0);
    mockDue = true; mockAction = SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART;
    mockNextSnapshot.phase = SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED;
    mockBudgetAllowed = true;
    assert(sensorarrayRunFrontendPresenceBoundary(&ctx));
    assert(mockBudget == 1 && mockRestart == 1);
    mockDue = true;
    assert(sensorarrayRunFrontendPresenceBoundary(&ctx));
    assert(mockBudget == 1 && mockRestart == 1);
    reset(&ctx);
    mockDue = true;
    mockNextSnapshot.evidence.devices[0].cleanupError = ESP_FAIL;
    assert(sensorarrayRunFrontendPresenceBoundary(&ctx));
    assert(mockRoute == 1 && mockInvalidate == 1); /* No failed-restore data. */
    reset(&ctx);
    mockBootOwnerFree = false; mockDue = true;
    assert(!sensorarrayRunFrontendPresenceBoundary(&ctx));
    assert(!mockLastOwner && mockDriverRead == 0 && mockRoute == 0);
    reset(&ctx);
    ctx.frontendFdcSdKnownLow = false; mockDue = true;
    assert(!sensorarrayRunFrontendPresenceBoundary(&ctx) && mockLastSdHigh);
}
static void testStartupFailureAndQuery(void)
{
    sensorarrayAppContext_t ctx;
    reset(&ctx); ctx.frontendStartupReached = true;
    mockBudgetAllowed = true;
    actualStartupFailure(&ctx, ESP_FAIL);
    assert(sensorarrayFrontendsHeld() && mockBudget == 0 && mockRestart == 0);
    assert(s_frontendPresence.published.phase == SENSORARRAY_FRONTEND_PRESENCE_SUSPECT);
    reset(&ctx); ctx.frontendStartupReached = true;
    mockBootOwnerFree = false; mockBudgetAllowed = true;
    actualStartupFailure(&ctx, ESP_FAIL);
    assert(!sensorarrayFrontendsHeld() && mockBudget == 1 && mockRestart == 1);
    reset(&ctx); ctx.frontendStartupReached = true; mockBudgetAllowed = true;
    actualStartupFailure(&ctx, ESP_ERR_NO_MEM);
    assert(!sensorarrayFrontendsHeld() && mockBudget == 1);
    reset(&ctx); ctx.frontendStartupReached = true; mockBudgetAllowed = true;
    mockColdPresenceBusy = mockColdPresenceQuarantined = true;
    actualStartupFailure(&ctx, ESP_FAIL);
    assert(!sensorarrayFrontendsHeld() && mockBudget == 1 && mockRestart == 1);
    sensorarrayFrontendPresenceSnapshot_t *s = &s_frontendPresence.published;
    s->probes = s->absentRun = s->presentRun = UINT32_MAX;
    s->knownMask = s->presentMask = UINT8_MAX;
    s->nextProbeUs = s->evidence.startedUs = s->evidence.endedUs = UINT64_MAX;
    for (unsigned i = 0; i < 3; ++i) {
        s->evidence.devices[i].error = s->evidence.devices[i].cleanupError = -1;
        s->evidence.devices[i].clockHz = s->evidence.devices[i].restoredClockHz = UINT32_MAX;
    }
    char reply[512];
    assert(sensorarrayFormatFrontendReply(reply, sizeof(reply)) == ESP_OK);
    printf("FRONTENDS_QUERY_MAX,length=%lu,capacity=512\n", (unsigned long)strlen(reply));
    assert(reply[strlen(reply) - 1] == '\n');
    assert(sensorarrayFormatFrontendReply(reply, 16) == ESP_ERR_INVALID_SIZE);
}
static void testIdLogBound(void)
{
    sensorarrayFrontendPresenceSnapshot_t snapshot = { .probes = UINT32_MAX };
    for (unsigned i = 0; i < 2; ++i) {
        sensorarrayFrontendPresenceDevice_t *device = &snapshot.evidence.devices[i];
        device->manufacturerId = device->deviceId = UINT16_MAX;
        device->manufacturerIdSecond = device->deviceIdSecond = UINT16_MAX;
        device->idReadMask = UINT8_MAX;
        device->error = device->cleanupError = -1;
    }
    const sensorarrayFrontendPresenceDevice_t *d = snapshot.evidence.devices;
    char line[384];
    /* ACTUAL_IDS_FORMAT */
    assert(written > 0 && (size_t)written < sizeof(line));
    assert(line[written - 1] == '\n');
    assert(strstr(line, "pMfg1=FFFF,pDev1=FFFF,pMfg2=FFFF,pDev2=FFFF,pReadMask=0xFF"));
    assert(strstr(line, "sMfg1=FFFF,sDev1=FFFF,sMfg2=FFFF,sDev2=FFFF,sReadMask=0xFF"));
    assert(strstr(line, "pErr=0xFFFFFFFF,pCleanup=0xFFFFFFFF"));
    assert(strstr(line, "sErr=0xFFFFFFFF,sCleanup=0xFFFFFFFF"));
    printf("FRONTEND_IDS_MAX,length=%d,capacity=384\n", written);
}
int main(void)
{
    testOwnershipAndHold(); testCommands(); testBoundaryAndRestart();
    testStartupFailureAndQuery();
    testIdLogBound();
    puts("frontend production main hold/lifecycle/query: PASS");
    return 0;
}
