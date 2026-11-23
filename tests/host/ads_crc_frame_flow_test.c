#define sensorarrayTransportPublishRuntimeFault printf /* Transport is synthetic in this main-logic harness. */
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayMixedRow.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayFrameBuilder.h"
#include "sensorarrayTextProtocol.h"

/* Only hardware/child-engine boundaries are synthetic. Main's entire frame
 * dispatcher, MixedRow, mode/profile identity, masks and text encoder execute. */
#define SENSORARRAY_ROW_MODES_TERMINAL_TEXT_MAX 384u
enum { SENSORARRAY_RUNTIME_MODE_FDC_MATRIX, SENSORARRAY_RUNTIME_MODE_ADS_MATRIX,
       SENSORARRAY_RUNTIME_MODE_MIXED_ROW };
typedef struct { uint32_t bootId; } sensorarrayBootBreadcrumb_t;
typedef struct {
    sensorarrayState_t state;
    sensorarrayMeasurementModeContext_t measurementMode;
    sensorarrayRowModeProfile_t rowModeProfile;
    sensorarrayFdcMatrixEngine_t fdcEngine;
    sensorarrayAdsMatrixEngine_t adsEngine;
    sensorarrayRouteController_t routeController;
    sensorarrayScanPlan_t scanPlan;
    sensorarrayFrame_t frame;
    sensorarrayFrame_t *mixedSegmentWorkspace;
    uint32_t hostFrameSequence;
    unsigned runtimeMode;
    bool asyncLogReady, legacySyncOutput;
} sensorarrayAppContext_t;
typedef enum { GOOD, SAMPLE_CRC, HARD_READ } outcome_t;
static sensorarrayAppContext_t ctx;
static sensorarrayFrame_t workspace;
static sensorarrayTextPacket_t outputPacket;
static outcome_t outcomes[3];
static uint64_t s_configSessionId = 100;
static unsigned childCalls[3], safeCalls, faultCalls, mfaults, terminalCalls;
static unsigned netCalls, usbCalls, legacyCalls, snapshotCalls, publishLastCalls;
static unsigned acquisitionCases;
static uint32_t outputCap;
static bool emitUsb;
static sensorarrayMeasurementMode_t failRoute;
static esp_err_t applyError;
static int64_t nowUs;
static char terminal[384];

int64_t esp_timer_get_time(void) { nowUs += 100; return nowUs; }
int sensorarrayAcqEventPrintf(const char *format, ...)
{
    char line[768];
    va_list args; va_start(args, format);
    int n = vsnprintf(line, sizeof(line), format, args); va_end(args);
    if (!strncmp(line, "MFAULT,", 7)) { mfaults++; }
    return n;
}
static bool sensorarrayBootBreadcrumbCopy(sensorarrayBootBreadcrumb_t *out)
{ out->bootId = 42; return true; }
const char *sensorarrayAdsOwnerName(sensorarrayAdsOwner_t owner)
{ (void)owner; return "MATRIX"; }
bool sensorarrayBoardMapHasIsolatedRailMonitor(void) { return false; }
bool sensorarrayAdsGapCopyRailSplit(uint32_t sequence, uint32_t age, sensorarrayAdsRailSplit_t *out)
{
    (void)sequence; (void)age;
    *out = (sensorarrayAdsRailSplit_t){.valid = true, .avddUv = 3350000, .avssUv = -1795000};
    return true; /* Legal fixed rail fixture; these cases never need a helper. */
}
void sensorarrayAdsGapCopyTransactionEvidence(sensorarrayAdsTransactionEvidence_t *rail,
    sensorarrayAdsTransactionEvidence_t *battery, sensorarrayAdsTransactionEvidence_t *check)
{
    if (rail) { memset(rail, 0, sizeof(*rail)); }
    if (battery) { memset(battery, 0, sizeof(*battery)); }
    if (check) { memset(check, 0, sizeof(*check)); }
}
esp_err_t sensorarrayAdsGapRefreshRailAtBoundary(sensorarrayState_t *state, uint32_t sequence)
{ (void)state; (void)sequence; assert(!"unexpected helper in CRC frame-flow fixture"); return ESP_FAIL; }
bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteController_t *route,
    sensorarrayRouteSnapshot_t *out) { *out = route->snapshot; return true; }
esp_err_t sensorarrayRouteControllerSelectRow(sensorarrayRouteController_t *route, uint8_t row)
{ route->snapshot.row = row; return ESP_OK; }
esp_err_t sensorarrayRouteControllerApplyMode(sensorarrayRouteController_t *route,
    sensorarrayMeasurementMode_t mode, const sensorarrayAdsRailSplit_t *rail, uint64_t *duration)
{
    (void)rail; *duration = 1;
    if (mode == failRoute) { return ESP_ERR_INVALID_STATE; }
    route->snapshot.mode = mode; return ESP_OK;
}
esp_err_t sensorarrayRouteControllerEnterSafeRailMonitor(sensorarrayRouteController_t *route, uint64_t *duration)
{ (void)route; (void)duration; assert(!"unexpected rail route"); return ESP_FAIL; }
esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteController_t *route, const char *reason)
{ (void)reason; safeCalls++; route->snapshot.safe = true; return ESP_OK; }
esp_err_t ads126xAdcWaitReferenceReady(ads126xAdcHandle_t *ads)
{ (void)ads; return ESP_OK; }
void sensorarrayAdsMatrixEngineInvalidateCaches(sensorarrayAdsMatrixEngine_t *engine)
{ (void)engine; }
void sensorarrayAdsMatrixEngineRecordMixedBoundaryFault(sensorarrayAdsMatrixEngine_t *engine,
    uint32_t sequence, esp_err_t original, esp_err_t cleanup)
{ (void)engine; (void)sequence; (void)original; (void)cleanup; assert(!"unexpected rail-boundary failure"); }
esp_err_t sensorarrayAdsMatrixEngineSetMode(sensorarrayAdsMatrixEngine_t *engine, sensorarrayMeasurementMode_t mode)
{ engine->mode = mode; return ESP_OK; }
void sensorarrayAdsMatrixEngineSetFrameSequenceHint(sensorarrayAdsMatrixEngine_t *engine, uint32_t sequence)
{ engine->frameSequenceHint = sequence; }

static esp_err_t acquire(const sensorarrayScanPlan_t *plan, sensorarrayFrame_t *frame,
    sensorarrayMeasurementMode_t mode)
{
    childCalls[mode]++;
    sensorarrayFrameBuilderInitInvalid(frame);
    uint64_t mask = sensorarrayFrameBuilderActiveCellMask(plan->configSnapshot.rowMask);
    frame->configSnapshot = plan->configSnapshot; frame->activeRows = plan->rowCount;
    frame->expectedMask = mask;
    if (outcomes[mode] == HARD_READ) { return ESP_ERR_TIMEOUT; }
    uint64_t rejected = outcomes[mode] == SAMPLE_CRC ? (mask & (~mask + 1u)) : 0u;
    uint64_t acquired = mask & ~rejected;
    frame->validMask = frame->freshMask = frame->acquiredMask = acquired;
    frame->errorMask = rejected;
    frame->freshFrame = rejected == 0u;
    frame->stale = !frame->freshFrame;
    frame->boundedCrcFailure = rejected != 0u;
    frame->measurement.mode = mode;
    frame->measurement.unit = mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ?
        SENSORARRAY_MEASUREMENT_UNIT_OHM : SENSORARRAY_MEASUREMENT_UNIT_VOLT;
    frame->measurement.decimalScale = mode == SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ? -3 : -6;
    frame->measurement.referenceSource = SENSORARRAY_ADS_REFERENCE_AVDD_AVSS;
    frame->measurement.railValid = true;
    if (mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) { frame->capValidMask = acquired; }
    else {
        frame->measurement.validMask = frame->measurement.freshMask = acquired;
        frame->measurement.errorMask = rejected;
    }
    for (unsigned i = 0; i < 64; ++i) {
        uint64_t bit = UINT64_C(1) << i;
        if (acquired & bit) {
            frame->capTotalPf[i] = 33.0;
            frame->measurement.valuesFixed[i] = 123456;
            frame->measurement.rawCode[i] = 12345;
            frame->measurement.pgaGain[i] = 1;
            frame->measurement.errorReason[i] = SENSORARRAY_CELL_ERROR_NONE;
        } else if (rejected & bit) {
            frame->measurement.errorReason[i] = SENSORARRAY_CELL_ERROR_SPI;
            /* A mixed merge must not copy stale payload from a masked child. */
            if (!plan->auxiliaryTransactionsAllowed) { frame->measurement.valuesFixed[i] = 987654321; }
        }
    }
    frame->frameStartUs = (uint64_t)esp_timer_get_time();
    frame->frameEndUs = frame->frameStartUs + 10;
    frame->physicalSweepUs = 10;
    if (mode == SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE) {
        frame->capStartUs = frame->frameStartUs; frame->capEndUs = frame->frameEndUs;
    } else if (mode == SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
        frame->voltStartUs = frame->frameStartUs; frame->voltEndUs = frame->frameEndUs;
    } else { frame->resStartUs = frame->frameStartUs; frame->resEndUs = frame->frameEndUs; }
    return rejected ? ESP_ERR_INVALID_CRC : ESP_OK;
}
esp_err_t sensorarrayFdcMatrixEngineReadFrame(sensorarrayFdcMatrixEngine_t *engine,
    const sensorarrayScanPlan_t *plan, sensorarrayFrame_t *frame)
{ (void)engine; return acquire(plan, frame, SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE); }
esp_err_t sensorarrayAdsMatrixEngineReadFrame(sensorarrayAdsMatrixEngine_t *engine,
    const sensorarrayScanPlan_t *plan, sensorarrayFrame_t *frame)
{ return acquire(plan, frame, engine->mode); }
static esp_err_t sensorarrayApplyMeasurementMode(sensorarrayAppContext_t *app,
    sensorarrayMeasurementMode_t mode, uint32_t id, bool emitEvent)
{
    (void)emitEvent;
    if (applyError != ESP_OK) { return applyError; }
    assert(sensorarrayMeasurementModeAccept(&app->measurementMode, mode, id));
    assert(sensorarrayMeasurementModeBeginTransition(&app->measurementMode));
    assert(sensorarrayMeasurementModeCompleteTransition(&app->measurementMode, app->hostFrameSequence + 1, 1));
    app->adsEngine.mode = mode;
    return ESP_OK;
}
static void sensorarrayAdsFaultFillContext(sensorarrayAppContext_t *app, sensorarrayAdsFaultEvent_t *fault)
{ (void)app; (void)fault; }
static void sensorarrayEmitAdsFault(sensorarrayAppContext_t *app, const sensorarrayAdsFaultEvent_t *fault)
{ (void)app; (void)fault; faultCalls++; }
static void sensorarrayPublishLastMeasurement(sensorarrayAppContext_t *app)
{ (void)app; publishLastCalls++; }
static esp_err_t sensorarrayCommandMailboxEmitConfigTerminal(uint32_t requestId, const char *data, size_t length)
{
    assert(requestId && length < sizeof(terminal));
    memcpy(terminal, data, length); terminal[length] = '\0'; terminalCalls++;
    return ESP_OK;
}
static uint32_t sensorarrayCommandMailboxGetOutputFpsCap(void) { return outputCap; }
static bool sensorarrayCommandMailboxTraceEnabled(void) { return false; }
static esp_err_t sensorarrayNetFramePublish(const sensorarrayTextPacket_t *packet, const sensorarrayFrame_t *frame, uint32_t flags)
{ (void)flags; assert(frame && packet->length); netCalls++; return ESP_OK; }
static bool sensorarrayUsbSinkShouldEmitData(uint32_t sequence) { assert(sequence); return emitUsb; }
static esp_err_t sensorarrayUsbSinkPublish(const sensorarrayTextPacket_t *packet)
{ assert(packet->length); usbCalls++; return ESP_OK; }
static uint64_t sensorarrayAsyncLogElapsedPositiveUs(int64_t started) { (void)started; return 1; }
static esp_err_t sensorarrayAsyncLogPublishFrameSnapshot(const sensorarrayFrame_t *frame, uint64_t measureFrameUs)
{ (void)frame; assert(measureFrameUs == 100); snapshotCalls++; return ESP_OK; }
static void sensorarrayEmitPayloadTooLargeDrop(uint32_t sequence) { (void)sequence; assert(!"unexpected oversize"); }
static esp_err_t sensorarrayFrameOutputPrint(const sensorarrayFrame_t *frame)
{ legacyCalls++; return sensorarrayTextProtocolBuildFrame(frame, &outputPacket); }

#define printf sensorarrayAcqEventPrintf
/* ACTUAL_FUNCTIONS */
static bool runAsyncProtected(sensorarrayFrame_t *frame, const sensorarrayTextPacket_t *textPacket,
    int64_t outputStartUs, int64_t lastOutputEmitUs, uint32_t protectionFlags)
{
    uint64_t frameAgeUs = 1;
    uint32_t queueDepth = 1;
/* ACTUAL_ASYNC_GATE */
    (void)outputUs; (void)lastOutputEmitUs;
    return emitted;
}
static bool runAsync(sensorarrayFrame_t *frame, const sensorarrayTextPacket_t *packet,
    int64_t start, int64_t last)
{ return runAsyncProtected(frame, packet, start, last, 0u); }
static void runLegacy(sensorarrayAppContext_t *ctx)
{
    uint64_t measureFrameUs = 100;
/* ACTUAL_LEGACY_GATE */
}
#undef printf

static void reset(const char *profile, bool pending, unsigned rows)
{
    acquisitionCases++;
    memset(&ctx, 0, sizeof(ctx)); memset(&workspace, 0, sizeof(workspace));
    memset(outcomes, 0, sizeof(outcomes)); memset(childCalls, 0, sizeof(childCalls));
    assert(sensorarrayScanConfigInit() == ESP_OK);
    sensorarrayScanConfigRequestResult_t request;
    assert(sensorarrayScanConfigRequestRows((uint8_t)rows, &request) == ESP_OK);
    sensorarrayScanConfigApplyPendingAtFrameBoundary(NULL);
    sensorarrayMeasurementModeInit(&ctx.measurementMode);
    sensorarrayRowModeProfileInit(&ctx.rowModeProfile, SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE);
    sensorarrayMeasurementMode_t modes[8];
    assert(sensorarrayRowModeProfileParse(profile, 8, modes));
    sensorarrayMeasurementMode_t baseMode = pending ? SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE : modes[0];
    assert(sensorarrayMeasurementModeAccept(&ctx.measurementMode, baseMode, 7));
    assert(sensorarrayMeasurementModeBeginTransition(&ctx.measurementMode));
    assert(sensorarrayMeasurementModeCompleteTransition(&ctx.measurementMode, 100, 1));
    assert(sensorarrayRowModeProfileAccept(&ctx.rowModeProfile, modes, 19));
    if (!pending) { assert(sensorarrayRowModeProfileCompleteTransition(&ctx.rowModeProfile, 100, 1)); }
    ctx.hostFrameSequence = 100;
    ctx.adsEngine.state = &ctx.state; ctx.adsEngine.routeController = &ctx.routeController;
    ctx.adsEngine.mode = baseMode; ctx.fdcEngine.state = &ctx.state;
    ctx.mixedSegmentWorkspace = &workspace;
    ctx.routeController.snapshot = (sensorarrayRouteSnapshot_t){.mode = baseMode,
        .row = 1, .known = true, .verified = true, .applied = true};
    ctx.legacySyncOutput = true;
    safeCalls = faultCalls = mfaults = terminalCalls = 0;
    netCalls = usbCalls = legacyCalls = snapshotCalls = publishLastCalls = 0;
    terminal[0] = '\0'; failRoute = SENSORARRAY_MEASUREMENT_MODE_NONE;
    applyError = ESP_OK; outputCap = 0; emitUsb = true; nowUs = 10000;
    s_configSessionId++;
    (void)sensorarrayPublishConfigAuthority(&ctx, 101);
}
static void assertCrcPoint(unsigned index)
{
    uint64_t bit = UINT64_C(1) << index;
    assert(!(ctx.frame.freshMask & bit) && !(ctx.frame.validMask & bit));
    assert(!(ctx.frame.measurement.freshMask & bit) && !(ctx.frame.measurement.validMask & bit));
    assert(!(ctx.frame.acquiredMask & bit));
    assert((ctx.frame.errorMask & bit) && (ctx.frame.measurement.errorMask & bit));
    assert(ctx.frame.measurement.errorReason[index] == SENSORARRAY_CELL_ERROR_SPI);
    assert(ctx.frame.measurement.valuesFixed[index] == SENSORARRAY_MEASUREMENT_INVALID_FIXED);
    assert(!ctx.frame.freshFrame && ctx.frame.stale);
}
static void assertOutput(bool permitted)
{
    uint64_t valid = ctx.frame.validMask, fresh = ctx.frame.freshMask, acquired = ctx.frame.acquiredMask;
    assert(sensorarrayTextProtocolBuildFrame(&ctx.frame, &outputPacket) == ESP_OK);
    assert(runAsync(&ctx.frame, &outputPacket, 1000000, 0) == permitted);
    assert(netCalls == (unsigned)permitted && usbCalls == (unsigned)permitted);
    runLegacy(&ctx); assert(legacyCalls == (unsigned)permitted);
    assert(ctx.frame.validMask == valid && ctx.frame.freshMask == fresh && ctx.frame.acquiredMask == acquired);
    if (permitted) { assert(strstr(outputPacket.data, "X02")); }
}
static void fixture(const char *label)
{
    printf("FRAME_HEX,%s,", label);
    for (unsigned i = 0; i < outputPacket.length; ++i) { printf("%02X", (unsigned char)outputPacket.data[i]); }
    putchar('\n');
}
static void testHomogeneous(void)
{
    for (unsigned rows = 1; rows <= 8; rows += 7) {
        reset("VVVVVVVV", true, rows); outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = SAMPLE_CRC;
        assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_INVALID_CRC);
        assert(ctx.frame.boundedCrcFailure); assertCrcPoint(0);
        assert(terminalCalls == 1 && !strncmp(terminal, "RMAPP,", 6));
        assert(ctx.rowModeProfile.appliedRequestId == 19 && !ctx.rowModeProfile.pending);
        assert(ctx.frame.identity.profileRequestId == 19 && ctx.frame.identity.rows == rows);
        assert(mfaults == 0 && faultCalls == 0 && safeCalls == 0);
        assertOutput(true); fixture(rows == 1 ? "volt1_partial_crc" : "volt8_partial_crc");
    }
    reset("VVVVVVVV", false, 8); outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = SAMPLE_CRC;
    assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_INVALID_CRC);
    assert(terminalCalls == 0 && mfaults == 0 && safeCalls == 0); assertOutput(true);
    /* A bounded CRC permit never overrides the ordinary output rate cap. */
    outputCap = 10; netCalls = usbCalls = 0;
    assert(!runAsync(&ctx.frame, &outputPacket, 1000010, 1000000));
    assert(netCalls == 0 && usbCalls == 0);
    assert(runAsyncProtected(&ctx.frame, &outputPacket, 1000010, 1000000, 1u));
    assert(netCalls == 1 && usbCalls == 1); /* protected full snapshot bypasses ordinary cap */
    netCalls = usbCalls = 0;
    outputCap = 0; emitUsb = false;
    assert(runAsync(&ctx.frame, &outputPacket, 2000000, 0));
    assert(netCalls == 1 && usbCalls == 0);
    reset("VVVVVVVV", true, 8); outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = HARD_READ;
    assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_TIMEOUT);
    assert(!ctx.frame.boundedCrcFailure && !strncmp(terminal, "RMERR,", 6));
    assert(ctx.rowModeProfile.appliedRequestId == 0 && safeCalls == 1 && mfaults == 0);
    assertOutput(false);
    reset("VVVVVVVV", false, 8); outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = HARD_READ;
    assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_TIMEOUT);
    assert(mfaults == 1 && faultCalls == 1 && safeCalls == 1 && terminalCalls == 0);
    assertOutput(false);
}
static void testMixed(void)
{
    reset("CVRCCCCC", true, 8); outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = SAMPLE_CRC;
    assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_INVALID_CRC);
    assert(ctx.frame.boundedCrcFailure); assertCrcPoint(8);
    assert(ctx.frame.acquiredMask == (UINT64_MAX & ~(UINT64_C(1) << 8)));
    assert(terminalCalls == 1 && !strncmp(terminal, "RMAPP,", 6));
    assert(ctx.rowModeProfile.appliedRequestId == 19 && mfaults == 0 && safeCalls == 0);
    assert(childCalls[0] == 1 && childCalls[1] == 1 && childCalls[2] == 1);
    assertOutput(true); fixture("mixed8_partial_crc");
    for (unsigned pending = 0; pending <= 1; ++pending) {
        reset("CVRCCCCC", pending != 0, 8);
        outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = SAMPLE_CRC;
        outcomes[SENSORARRAY_MEASUREMENT_MODE_RESISTANCE] = HARD_READ;
        /* firstErr remains CRC. The later hard read must revoke publication. */
        assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_INVALID_CRC);
        assert(!ctx.frame.boundedCrcFailure && !ctx.frame.freshFrame);
        assertCrcPoint(8);
        assert((ctx.frame.acquiredMask & (UINT64_C(0xff) << 16)) == 0);
        assert(ctx.frame.acquiredMask & 1); /* Fresh CAP cannot mask a later hard error. */
        assert(safeCalls == 1);
        assert(mfaults == (pending ? 0u : 1u));
        if (pending) { assert(terminalCalls == 1 && !strncmp(terminal, "RMERR,", 6)); }
        else { assert(terminalCalls == 0); }
        assertOutput(false);
    }
    reset("CVRCCCCC", true, 8); outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = SAMPLE_CRC;
    failRoute = SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_INVALID_CRC);
    assert(!ctx.frame.boundedCrcFailure && childCalls[2] == 0 && safeCalls == 1);
    assert(terminalCalls == 1 && !strncmp(terminal, "RMERR,", 6)); assertOutput(false);
    reset("CVRCCCCC", false, 8); outcomes[SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE] = HARD_READ;
    outcomes[SENSORARRAY_MEASUREMENT_MODE_VOLTAGE] = SAMPLE_CRC;
    assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_TIMEOUT);
    assert(!ctx.frame.boundedCrcFailure && childCalls[1] == 0 && childCalls[2] == 0);
    assert(mfaults == 1 && safeCalls == 1); assertOutput(false);
}
static void testStalePermitAndSnapshotGate(void)
{
    reset("VVVVVVVV", true, 8);
    ctx.frame.freshFrame = ctx.frame.boundedCrcFailure = true;
    ctx.frame.acquiredMask = UINT64_MAX;
    applyError = ESP_ERR_INVALID_STATE;
    assert(sensorarrayRunOneFrame(&ctx) == ESP_ERR_INVALID_STATE);
    assert(!ctx.frame.freshFrame && !ctx.frame.boundedCrcFailure && ctx.frame.acquiredMask == 0);
    assert(terminalCalls == 1 && !strncmp(terminal, "RMERR,", 6));
    ctx.asyncLogReady = true;
    runLegacy(&ctx); assert(snapshotCalls == 1 && legacyCalls == 0);
    ctx.asyncLogReady = false; runLegacy(&ctx); assert(legacyCalls == 0);
    assertOutput(false);
}
int main(void)
{
    testHomogeneous(); testMixed(); testStalePermitAndSnapshotGate();
    assert(acquisitionCases == 11);
    printf("CRC_FLOW_MAIN,acquisition_cases=%u,partial_invalid_masks=preserved,pending_terminal=checked,old_first_error=does_not_permit_hard,output_caps=preserved\n", acquisitionCases);
    return 0;
}
