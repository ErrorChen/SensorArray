/* Synthetic presence evidence only. Executes the complete production module. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayFrontendPresence.h"
#include "sensorarrayBringup.h"

static uint64_t nowUs;
static unsigned lockDepth, adsCalls, fdcCalls, frequencyCalls;
static unsigned failFrequencyCall;
static bool busLockTimeout, secondIdMismatch;
static esp_err_t fdcErrors[2];
static uint16_t fdcManufacturer[2], fdcDevice[2];
static BoardSupportI2cCtx_t buses[2];
static sensorarrayAdsPresenceResult_t adsResult;
static uint32_t callDelayUs;

void presenceTestEnter(void) { assert(lockDepth == 0u); lockDepth++; }
void presenceTestExit(void) { assert(lockDepth == 1u); lockDepth--; }
int64_t esp_timer_get_time(void) { return (int64_t)nowUs; }
bool ads126xAdcIsQuarantined(const ads126xAdcHandle_t *ads) { return ads && ads->quarantined; }

esp_err_t sensorarrayBringupProbeAdsPresence(sensorarrayState_t *state,
    uint64_t deadlineUs, sensorarrayAdsPresenceResult_t *out)
{
    (void)state;
    assert(lockDepth == 0u && deadlineUs > nowUs);
    ++adsCalls; nowUs += callDelayUs; *out = adsResult;
    return adsResult.error != ESP_OK ? adsResult.error : adsResult.cleanupError;
}

esp_err_t boardSupportSetI2cFrequency(const BoardSupportI2cCtx_t *bus, uint32_t hz)
{
    assert(lockDepth == 0u);
    ++frequencyCalls;
    if (frequencyCalls == failFrequencyCall) { return ESP_FAIL; }
    BoardSupportI2cCtx_t *mutableBus = (BoardSupportI2cCtx_t *)bus;
    mutableBus->FrequencyHz = hz; mutableBus->Installed = true; mutableBus->Offline = false;
    return ESP_OK;
}

esp_err_t boardSupportI2cReadReg16Sequence(void *userCtx, uint8_t addr,
    const uint8_t *regs, size_t count, uint16_t *values, uint32_t timeoutMs)
{
    BoardSupportI2cCtx_t *bus = userCtx;
    assert(lockDepth == 0u && bus->Installed && !bus->Offline);
    assert(count == 2u && regs[0] == 0xFEu && regs[1] == 0xFFu);
    assert(addr == (bus == &buses[0] ? 0x2Bu : 0x2Au));
    assert(timeoutMs > 0u && timeoutMs <= 20u);
    ++fdcCalls;
    if (busLockTimeout) { return ESP_ERR_TIMEOUT; }
    ++bus->TransactionCount;
    nowUs += callDelayUs;
    unsigned index = bus == &buses[0] ? 0u : 1u;
    if (fdcErrors[index] != ESP_OK) { return fdcErrors[index]; }
    values[0] = fdcManufacturer[index]; values[1] = fdcDevice[index];
    if (secondIdMismatch && bus->TransactionCount % 2u == 0u) { values[1] ^= 1u; }
    return ESP_OK;
}

static sensorarrayState_t setup(void)
{
    nowUs = 1000000u; lockDepth = adsCalls = fdcCalls = frequencyCalls = 0u;
    failFrequencyCall = 0u; busLockTimeout = secondIdMismatch = false;
    callDelayUs = 100u;
    for (unsigned i = 0u; i < 2u; ++i) {
        buses[i] = (BoardSupportI2cCtx_t) {.Enabled = true, .Installed = true, .FrequencyHz = 400000u};
        fdcErrors[i] = ESP_OK; fdcManufacturer[i] = 0x5449u; fdcDevice[i] = 0x3055u;
    }
    adsResult = (sensorarrayAdsPresenceResult_t) {.known = true, .present = true,
        .id = 3u, .power = 2u, .interface = 6u, .actualHz = 125000u,
        .restoredHz = 2000000u};
    sensorarrayState_t state = {0};
    state.fdcPrimary.i2cCtx = &buses[0]; state.fdcPrimary.i2cAddr = 0x2Bu;
    state.fdcSecondary.i2cCtx = &buses[1]; state.fdcSecondary.i2cAddr = 0x2Au;
    return state;
}

static sensorarrayFrontendPresenceAction_t poll(sensorarrayFrontendPresence_t *p,
    sensorarrayState_t *state, bool owner, bool shutdown)
{
    sensorarrayFrontendPresenceSnapshot_t before;
    assert(sensorarrayFrontendPresenceCopySnapshot(p, &before));
    if (nowUs < before.nextProbeUs) { nowUs = before.nextProbeUs; }
    return sensorarrayFrontendPresencePoll(p, state, nowUs, owner, shutdown, NULL);
}

static void allUnavailable(void)
{
    fdcErrors[0] = fdcErrors[1] = ESP_FAIL;
    adsResult.present = false; adsResult.id = adsResult.power = adsResult.interface = 0u;
}

static void allPresent(void)
{
    fdcErrors[0] = fdcErrors[1] = ESP_OK;
    adsResult.present = true; adsResult.id = 3u; adsResult.power = 2u; adsResult.interface = 6u;
}

static void testFirstSuspectAndStableRecovery(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs); allUnavailable();
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
    assert(sensorarrayFrontendPresenceIsWaiting(&p));
    assert(p.published.phase == SENSORARRAY_FRONTEND_PRESENCE_SUSPECT && p.published.absentRun == 1u);
    assert(p.published.nextProbeUs - p.published.evidence.endedUs == 1000000u);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.phase == SENSORARRAY_FRONTEND_PRESENCE_WAIT && p.published.absentRun == 3u);
    assert(p.published.nextProbeUs - p.published.evidence.endedUs == 2000000u);
    allPresent();
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART);
    unsigned calls = adsCalls + fdcCalls;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(adsCalls + fdcCalls == calls); /* Terminal request is not emitted repeatedly. */
}

static void testTransientLossStillNeedsRestart(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs); allUnavailable();
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
    allPresent();
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.phase == SENSORARRAY_FRONTEND_PRESENCE_SUSPECT);
    adsResult.known = false;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.presentRun == 0u);
    adsResult.known = true;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    adsResult.id = 0x23u;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.presentRun == 1u); /* Different ID starts a new stable run. */
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART);
}

static void testFlagsDoNotEstablishPhysicalPresence(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs);
    assert(!state.adsReady && !state.fdcPrimary.ready && !state.fdcSecondary.ready);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.presentMask == 7u && p.published.phase == SENSORARRAY_FRONTEND_PRESENCE_MONITOR);
    assert(frequencyCalls == 0u); /* Normal I2C probe preserves selected speed. */
    state.adsReady = state.fdcPrimary.ready = state.fdcSecondary.ready = true;
    allUnavailable();
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
}

static void testBusyShutdownAndQuarantineNeverProbe(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs);
    assert(poll(&p, &state, false, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(poll(&p, &state, true, true) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    state.ads.quarantined = true;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(adsCalls == 0u && fdcCalls == 0u && frequencyCalls == 0u);
    assert(p.published.absentRun == 0u && p.published.knownMask == 0u);
}

static void testPartialTimeoutAndWrongId(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs); allUnavailable();
    fdcErrors[1] = ESP_OK; fdcDevice[1] = 0xFFFFu;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.evidence.devices[1].link == SENSORARRAY_FRONTEND_LINK_UNKNOWN);
    fdcDevice[1] = 0x3055u; busLockTimeout = true;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.evidence.devices[0].link == SENSORARRAY_FRONTEND_LINK_BUSY);
    busLockTimeout = false; allPresent(); secondIdMismatch = true;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.knownMask != 7u);
}

static void testOfflineLowSpeedAndRestore(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs); buses[0].Offline = true;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(frequencyCalls == 0u && p.published.knownMask != 7u);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(frequencyCalls == 4u && !buses[0].Offline);
    assert(buses[0].FrequencyHz == 400000u && buses[1].FrequencyHz == 400000u);
    assert(p.published.presentMask == 7u);
    allUnavailable();
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
    failFrequencyCall = frequencyCalls + 2u; /* First bus restoration fails. */
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.evidence.devices[0].error == ESP_FAIL);
    assert(p.published.evidence.devices[0].cleanupError == ESP_FAIL);
    assert(p.published.evidence.devices[0].link == SENSORARRAY_FRONTEND_LINK_UNKNOWN);
}

static void testResetAndDeadline(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs); adsResult.power |= ADS126X_POWER_RESET;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE); /* Cold reset is normal. */
    state.adsReady = true;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
    assert(p.published.evidence.devices[2].power & ADS126X_POWER_RESET); /* No flag clearing. */
    state = setup(); sensorarrayFrontendPresenceInit(&p, nowUs); callDelayUs = 700000u;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.knownMask == 0u && p.published.absentRun == 0u);
    assert(fdcCalls == 0u); /* Expired round admits no later I/O. */
    unsigned calls = adsCalls;
    assert(sensorarrayFrontendPresencePoll(&p, &state, nowUs, true, false, NULL) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(adsCalls == calls); /* No catch-up spin. */
}

static void testCleanupAndStaleEvidenceCannotResume(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    sensorarrayFrontendPresenceEvidence_t evidence = p.published.evidence;
    nowUs = p.published.nextProbeUs;
    evidence.startedUs = nowUs; evidence.endedUs = nowUs + 1u;
    evidence.devices[2].cleanupError = ESP_FAIL;
    assert(sensorarrayFrontendPresenceObserve(&p, &evidence, nowUs + 1u) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
    assert(sensorarrayFrontendPresenceIsWaiting(&p) && p.published.presentMask == 3u);
    nowUs = p.published.nextProbeUs;
    evidence.startedUs = nowUs; evidence.endedUs = nowUs + 1u;
    assert(sensorarrayFrontendPresenceObserve(&p, &evidence, nowUs + 1u) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.presentRun == 0u);
    nowUs = p.published.nextProbeUs;
    evidence.devices[2].cleanupError = ESP_OK;
    /* Replaying the previous round despite a later observer timestamp is rejected. */
    assert(sensorarrayFrontendPresenceObserve(&p, &evidence, nowUs) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.presentRun == 0u && p.published.knownMask == 0u);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART);
}

static void testAdsCleanupPropagationAndQuarantineHold(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs);
    adsResult.cleanupError = ESP_ERR_INVALID_RESPONSE;
    adsResult.restoredHz = 0u;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
    const sensorarrayFrontendPresenceDevice_t *adc = &p.published.evidence.devices[2];
    assert(adc->error == ESP_OK && adc->cleanupError == ESP_ERR_INVALID_RESPONSE);
    assert(adc->clockHz == 125000u && adc->restoredClockHz == 0u);
    assert(p.published.presentMask == 3u && p.published.presentRun == 0u);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.presentRun == 0u);
    adsResult.cleanupError = ESP_OK; adsResult.restoredHz = 2000000u;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.evidence.devices[2].restoredClockHz == 2000000u);
    assert(p.published.evidence.devices[0].restoredClockHz == 400000u);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART);

    state = setup(); sensorarrayFrontendPresenceInit(&p, nowUs);
    adsResult.quarantined = true; adsResult.cleanupError = ESP_ERR_TIMEOUT;
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT);
    assert(p.published.phase == SENSORARRAY_FRONTEND_PRESENCE_SUSPECT);
    assert(p.published.knownMask == 0u && fdcCalls == 0u);
    assert(p.published.evidence.devices[2].error == ESP_OK);
    assert(p.published.evidence.devices[2].cleanupError == ESP_ERR_TIMEOUT);
}

static void testExplicitStartupHoldNeedsTwoFreshIds(void)
{
    sensorarrayState_t state = setup(); sensorarrayFrontendPresence_t p;
    sensorarrayFrontendPresenceInit(&p, nowUs);
    sensorarrayFrontendPresenceLatchHold(&p, nowUs);
    assert(sensorarrayFrontendPresenceIsWaiting(&p));
    assert(p.published.phase == SENSORARRAY_FRONTEND_PRESENCE_SUSPECT);
    assert(p.published.probes == 0u && p.published.absentRun == 0u);
    assert(p.published.knownMask == 0u && p.published.presentMask == 0u);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_NONE);
    assert(p.published.presentRun == 1u);
    uint64_t waitEnteredUs = p.published.waitEnteredUs;
    uint64_t nextProbeUs = p.published.nextProbeUs;
    sensorarrayFrontendPresenceLatchHold(&p, nowUs + 100u);
    assert(p.published.presentRun == 1u && p.published.waitEnteredUs == waitEnteredUs);
    assert(p.published.nextProbeUs == nextProbeUs);
    assert(poll(&p, &state, true, false) == SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART);
    sensorarrayFrontendPresenceLatchHold(&p, nowUs + 100u);
    assert(p.published.phase == SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED);
}

int main(void)
{
    testFirstSuspectAndStableRecovery();
    testTransientLossStillNeedsRestart();
    testFlagsDoNotEstablishPhysicalPresence();
    testBusyShutdownAndQuarantineNeverProbe();
    testPartialTimeoutAndWrongId();
    testOfflineLowSpeedAndRestore();
    testResetAndDeadline();
    testCleanupAndStaleEvidenceCannotResume();
    testAdsCleanupPropagationAndQuarantineHold();
    testExplicitStartupHoldNeedsTwoFreshIds();
    puts("FRONTEND_PRESENCE_TESTS,passed=10,source=complete_production_module,fixture=synthetic");
    return 0;
}
