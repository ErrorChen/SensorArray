#include "sensorarrayFrontendPresence.h"

#include <limits.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sensorarrayBringup.h"

#define FRONTEND_MONITOR_INTERVAL_US 1000000u
#define FRONTEND_WAIT_INTERVAL_US 2000000u
#define FRONTEND_PROBE_BUDGET_US 600000u
#define FRONTEND_I2C_LOW_HZ 100000u
#define FRONTEND_I2C_READ_TIMEOUT_MS 20u
#define FRONTEND_I2C_RECONFIG_ADMISSION_US 150000u
#define FRONTEND_FDC_MANUFACTURER_ID 0x5449u
#define FRONTEND_FDC_DEVICE_ID 0x3055u

/* Static rather than embedded: early queries may precede context Init(). */
static portMUX_TYPE s_presenceMux = portMUX_INITIALIZER_UNLOCKED;

static uint64_t frontendAddTime(uint64_t time, uint64_t interval)
{
    return time > UINT64_MAX - interval ? UINT64_MAX : time + interval;
}

static uint32_t frontendIncrement(uint32_t value)
{
    return value == UINT32_MAX ? value : value + 1u;
}

void sensorarrayFrontendPresenceInit(sensorarrayFrontendPresence_t *presence,
                                    uint64_t nowUs)
{
    if (!presence) { return; }
    portENTER_CRITICAL(&s_presenceMux);
    memset(presence, 0, sizeof(*presence));
    presence->published.initialized = true;
    presence->published.phase = SENSORARRAY_FRONTEND_PRESENCE_MONITOR;
    presence->published.nextProbeUs = nowUs;
    portEXIT_CRITICAL(&s_presenceMux);
}

void sensorarrayFrontendPresenceLatchHold(sensorarrayFrontendPresence_t *presence,
                                         uint64_t nowUs)
{
    if (!presence) { return; }
    portENTER_CRITICAL(&s_presenceMux);
    sensorarrayFrontendPresenceSnapshot_t *p = &presence->published;
    if (p->initialized && p->phase == SENSORARRAY_FRONTEND_PRESENCE_MONITOR) {
        p->phase = SENSORARRAY_FRONTEND_PRESENCE_SUSPECT;
        p->waitEnteredUs = nowUs;
        p->nextProbeUs = nowUs;
    }
    portEXIT_CRITICAL(&s_presenceMux);
}

bool sensorarrayFrontendPresenceCopySnapshot(const sensorarrayFrontendPresence_t *presence,
    sensorarrayFrontendPresenceSnapshot_t *outSnapshot)
{
    if (!presence || !outSnapshot) { return false; }
    portENTER_CRITICAL(&s_presenceMux);
    *outSnapshot = presence->published;
    portEXIT_CRITICAL(&s_presenceMux);
    return outSnapshot->initialized;
}

bool sensorarrayFrontendPresenceDue(const sensorarrayFrontendPresence_t *presence,
                                   uint64_t nowUs)
{
    sensorarrayFrontendPresenceSnapshot_t snapshot = {0};
    return sensorarrayFrontendPresenceCopySnapshot(presence, &snapshot) &&
        snapshot.phase != SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED &&
        nowUs >= snapshot.nextProbeUs;
}

bool sensorarrayFrontendPresenceIsWaiting(const sensorarrayFrontendPresence_t *presence)
{
    sensorarrayFrontendPresenceSnapshot_t snapshot = {0};
    return sensorarrayFrontendPresenceCopySnapshot(presence, &snapshot) &&
        snapshot.phase != SENSORARRAY_FRONTEND_PRESENCE_MONITOR;
}

const char *sensorarrayFrontendPresencePhaseName(sensorarrayFrontendPresencePhase_t phase)
{
    switch (phase) {
    case SENSORARRAY_FRONTEND_PRESENCE_MONITOR: return "monitor";
    case SENSORARRAY_FRONTEND_PRESENCE_SUSPECT: return "suspect";
    case SENSORARRAY_FRONTEND_PRESENCE_WAIT: return "wait_frontends";
    case SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED: return "restart_required";
    default: return "unknown";
    }
}

const char *sensorarrayFrontendPresenceLinkName(sensorarrayFrontendLink_t link)
{
    switch (link) {
    case SENSORARRAY_FRONTEND_LINK_PRESENT: return "present";
    case SENSORARRAY_FRONTEND_LINK_UNAVAILABLE: return "unavailable";
    case SENSORARRAY_FRONTEND_LINK_BUSY: return "busy";
    case SENSORARRAY_FRONTEND_LINK_INHIBITED: return "inhibited";
    default: return "unknown";
    }
}

sensorarrayFrontendPresenceAction_t sensorarrayFrontendPresenceObserve(
    sensorarrayFrontendPresence_t *presence,
    const sensorarrayFrontendPresenceEvidence_t *evidence, uint64_t nowUs)
{
    if (!presence || !evidence) { return SENSORARRAY_FRONTEND_PRESENCE_NONE; }
    portENTER_CRITICAL(&s_presenceMux);
    sensorarrayFrontendPresenceSnapshot_t *p = &presence->published;
    if (!p->initialized || nowUs < p->nextProbeUs ||
        p->phase == SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED) {
        portEXIT_CRITICAL(&s_presenceMux);
        return SENSORARRAY_FRONTEND_PRESENCE_NONE;
    }
    bool firstRound = p->probes == 0u;
    uint64_t previousEndUs = p->evidence.endedUs;
    p->probes = frontendIncrement(p->probes);
    p->evidence = *evidence;
    p->knownMask = p->presentMask = 0u;
    bool freshRound = evidence->startedUs <= evidence->endedUs &&
        evidence->endedUs <= nowUs && nowUs - evidence->endedUs <= FRONTEND_PROBE_BUDGET_US &&
        (firstRound || evidence->startedUs > previousEndUs);
    bool usable = freshRound && evidence->ownerQuiescent && !evidence->fdcShutdownHigh &&
        !evidence->quarantined;
    bool cleanupFailed = false;
    for (unsigned i = 0u; i < SENSORARRAY_FRONTEND_COUNT; ++i) {
        if (evidence->devices[i].cleanupError != ESP_OK) {
            /* Cleanup failure must still hold acquisition if the failed
             * operation also quarantined the bus. It cannot establish IDs. */
            cleanupFailed |= freshRound;
            continue;
        }
        if (!usable) { continue; }
        sensorarrayFrontendLink_t link = evidence->devices[i].link;
        if (link == SENSORARRAY_FRONTEND_LINK_PRESENT ||
            link == SENSORARRAY_FRONTEND_LINK_UNAVAILABLE) {
            p->knownMask |= (uint8_t)(1u << i);
        }
        if (link == SENSORARRAY_FRONTEND_LINK_PRESENT) {
            p->presentMask |= (uint8_t)(1u << i);
        }
    }
    bool absent = p->knownMask == SENSORARRAY_FRONTEND_ALL_MASK && p->presentMask == 0u;
    bool present = p->knownMask == SENSORARRAY_FRONTEND_ALL_MASK &&
        p->presentMask == SENSORARRAY_FRONTEND_ALL_MASK;
    bool adsResetObserved = usable && evidence->adsWasInitialized &&
        evidence->devices[SENSORARRAY_FRONTEND_ADS].link == SENSORARRAY_FRONTEND_LINK_PRESENT &&
        (evidence->devices[SENSORARRAY_FRONTEND_ADS].power & ADS126X_POWER_RESET) != 0u;
    p->absentRun = absent ? frontendIncrement(p->absentRun) : 0u;
    sensorarrayFrontendPresenceAction_t action = SENSORARRAY_FRONTEND_PRESENCE_NONE;
    if (p->phase == SENSORARRAY_FRONTEND_PRESENCE_MONITOR) {
        p->presentRun = 0u;
        if (absent || adsResetObserved || cleanupFailed) {
            p->phase = SENSORARRAY_FRONTEND_PRESENCE_SUSPECT;
            p->waitEnteredUs = nowUs;
            action = SENSORARRAY_FRONTEND_PRESENCE_ENTER_WAIT;
        }
    } else {
        if (p->absentRun >= 3u) { p->phase = SENSORARRAY_FRONTEND_PRESENCE_WAIT; }
        bool sameIds = true;
        for (unsigned i = 0u; i < SENSORARRAY_FRONTEND_COUNT; ++i) {
            if (presence->recoveryIds[i] != evidence->devices[i].deviceId) { sameIds = false; }
        }
        p->presentRun = present ? (sameIds ? frontendIncrement(p->presentRun) : 1u) : 0u;
        if (present) {
            for (unsigned i = 0u; i < SENSORARRAY_FRONTEND_COUNT; ++i) {
                presence->recoveryIds[i] = evidence->devices[i].deviceId;
            }
        }
        if (p->presentRun >= 2u) {
            p->phase = SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED;
            action = SENSORARRAY_FRONTEND_PRESENCE_REQUEST_RESTART;
        }
    }
    p->nextProbeUs = frontendAddTime(nowUs,
        p->phase == SENSORARRAY_FRONTEND_PRESENCE_WAIT ||
        p->phase == SENSORARRAY_FRONTEND_PRESENCE_RESTART_REQUIRED ?
            FRONTEND_WAIT_INTERVAL_US : FRONTEND_MONITOR_INTERVAL_US);
    portEXIT_CRITICAL(&s_presenceMux);
    return action;
}

static bool frontendBeforeDeadline(uint64_t deadlineUs, uint64_t admissionUs)
{
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    return deadlineUs > nowUs && deadlineUs - nowUs >= admissionUs;
}

static void frontendReadFdc(const sensorarrayFdcDeviceState_t *fdc,
    bool lowSpeed, uint64_t deadlineUs, sensorarrayFrontendPresenceDevice_t *out)
{
    const BoardSupportI2cCtx_t *bus = fdc->i2cCtx;
    out->error = ESP_ERR_INVALID_STATE;
    if (!bus || !bus->Enabled || !bus->FrequencyHz || fdc->i2cAddr > 0x7Fu) { return; }
    if (bus->Recovering) { out->link = SENSORARRAY_FRONTEND_LINK_BUSY; return; }
    uint32_t savedHz = bus->FrequencyHz;
    uint32_t probeHz = lowSpeed && savedHz > FRONTEND_I2C_LOW_HZ ?
        FRONTEND_I2C_LOW_HZ : savedHz;
    bool reconfigured = false;
    if (lowSpeed && (probeHz != savedHz || bus->Offline || !bus->Installed)) {
        if (!frontendBeforeDeadline(deadlineUs, FRONTEND_I2C_RECONFIG_ADMISSION_US)) {
            out->error = ESP_ERR_TIMEOUT; return;
        }
        reconfigured = true;
        out->error = boardSupportSetI2cFrequency(bus, probeHz);
        if (out->error != ESP_OK) { goto restore; }
    }
    if (bus->Offline || !bus->Installed) { goto restore; }
    const uint8_t idRegisters[2] = {0xFEu, 0xFFu}; /* Never read clear-on-read STATUS. */
    uint16_t first[2] = {0}, second[2] = {0};
    if (!frontendBeforeDeadline(deadlineUs, 4u * FRONTEND_I2C_READ_TIMEOUT_MS * 1000u)) {
        out->error = ESP_ERR_TIMEOUT; goto restore;
    }
    uint32_t transactionBefore = bus->TransactionCount;
    out->error = boardSupportI2cReadReg16Sequence((void *)bus, fdc->i2cAddr,
        idRegisters, 2u, first, FRONTEND_I2C_READ_TIMEOUT_MS);
    if (out->error == ESP_FAIL || out->error == ESP_ERR_TIMEOUT) {
        /* A completed NACK/timeout establishes communication unavailability,
         * not physical unplugging. Local invalid-state/lock errors do not. */
        out->link = bus->TransactionCount != transactionBefore ?
            SENSORARRAY_FRONTEND_LINK_UNAVAILABLE : SENSORARRAY_FRONTEND_LINK_BUSY;
        goto restore;
    }
    if (out->error != ESP_OK) { goto restore; }
    out->manufacturerId = first[0];
    out->deviceId = first[1];
    if (!frontendBeforeDeadline(deadlineUs, 2u * FRONTEND_I2C_READ_TIMEOUT_MS * 1000u)) {
        out->error = ESP_ERR_TIMEOUT; goto restore;
    }
    out->error = boardSupportI2cReadReg16Sequence((void *)bus, fdc->i2cAddr,
        idRegisters, 2u, second, FRONTEND_I2C_READ_TIMEOUT_MS);
    if (out->error == ESP_OK && first[0] == second[0] && first[1] == second[1] &&
        first[0] == FRONTEND_FDC_MANUFACTURER_ID && first[1] == FRONTEND_FDC_DEVICE_ID) {
        out->link = SENSORARRAY_FRONTEND_LINK_PRESENT;
    } else if (out->error == ESP_OK) {
        out->error = ESP_ERR_INVALID_RESPONSE;
    }
restore:
    out->clockHz = bus->FrequencyHz;
    if (reconfigured && (bus->FrequencyHz != savedHz || !bus->Installed || bus->Offline)) {
        if (!frontendBeforeDeadline(deadlineUs, 1u)) {
            out->cleanupError = ESP_ERR_TIMEOUT;
        } else {
            out->cleanupError = boardSupportSetI2cFrequency(bus, savedHz);
            if (out->cleanupError == ESP_OK &&
                (bus->FrequencyHz != savedHz || !bus->Installed || bus->Offline)) {
                out->cleanupError = ESP_ERR_INVALID_RESPONSE;
            }
        }
        if (out->cleanupError != ESP_OK) { out->link = SENSORARRAY_FRONTEND_LINK_UNKNOWN; }
    }
    if (out->cleanupError == ESP_OK && bus->Installed && !bus->Offline) {
        out->restoredClockHz = bus->FrequencyHz;
    }
}

sensorarrayFrontendPresenceAction_t sensorarrayFrontendPresencePoll(
    sensorarrayFrontendPresence_t *presence, sensorarrayState_t *state,
    uint64_t nowUs, bool ownerQuiescent, bool fdcSdHigh,
    sensorarrayFrontendPresenceSnapshot_t *outSnapshot)
{
    sensorarrayFrontendPresenceSnapshot_t previous = {0};
    if (!state || !sensorarrayFrontendPresenceCopySnapshot(presence, &previous) ||
        !sensorarrayFrontendPresenceDue(presence, nowUs)) {
        if (outSnapshot && presence) { (void)sensorarrayFrontendPresenceCopySnapshot(presence, outSnapshot); }
        return SENSORARRAY_FRONTEND_PRESENCE_NONE;
    }
    sensorarrayFrontendPresenceEvidence_t evidence = {
        .startedUs = (uint64_t)esp_timer_get_time(),
        .ownerQuiescent = ownerQuiescent,
        .fdcShutdownHigh = fdcSdHigh,
        .lowSpeed = previous.phase != SENSORARRAY_FRONTEND_PRESENCE_MONITOR ||
            (previous.probes != 0u && previous.presentMask != SENSORARRAY_FRONTEND_ALL_MASK),
        .quarantined = ads126xAdcIsQuarantined(&state->ads),
        .adsWasInitialized = state->adsReady,
    };
    uint64_t deadlineUs = frontendAddTime(evidence.startedUs, FRONTEND_PROBE_BUDGET_US);
    if (!ownerQuiescent || fdcSdHigh || evidence.quarantined) {
        for (unsigned i = 0u; i < SENSORARRAY_FRONTEND_COUNT; ++i) {
            evidence.devices[i].link = fdcSdHigh ? SENSORARRAY_FRONTEND_LINK_INHIBITED :
                SENSORARRAY_FRONTEND_LINK_BUSY;
            evidence.devices[i].error = ESP_ERR_INVALID_STATE;
        }
    } else {
        sensorarrayAdsPresenceResult_t ads = {0};
        esp_err_t adsError = sensorarrayBringupProbeAdsPresence(state, deadlineUs, &ads);
        sensorarrayFrontendPresenceDevice_t *adc = &evidence.devices[SENSORARRAY_FRONTEND_ADS];
        adc->error = ads.error;
        adc->cleanupError = ads.cleanupError;
        /* The helper returns cleanupError when its read succeeded. Preserve
         * that distinction instead of relabelling cleanup as the first error. */
        if (adc->error == ESP_OK && adc->cleanupError == ESP_OK && adsError != ESP_OK) {
            adc->error = adsError;
        }
        adc->deviceId = ads.id;
        adc->power = ads.power;
        adc->interface = ads.interface;
        adc->clockHz = ads.actualHz;
        adc->restoredClockHz = ads.restoredHz;
        adc->link = ads.busy || ads.quarantined ? SENSORARRAY_FRONTEND_LINK_BUSY :
            ads.known ? (ads.present ? SENSORARRAY_FRONTEND_LINK_PRESENT :
                SENSORARRAY_FRONTEND_LINK_UNAVAILABLE) : SENSORARRAY_FRONTEND_LINK_UNKNOWN;
        evidence.quarantined = ads.quarantined || ads126xAdcIsQuarantined(&state->ads);
        if (!evidence.quarantined && !ads.busy) {
            frontendReadFdc(&state->fdcPrimary, evidence.lowSpeed, deadlineUs,
                &evidence.devices[SENSORARRAY_FRONTEND_PRIMARY]);
            frontendReadFdc(&state->fdcSecondary, evidence.lowSpeed, deadlineUs,
                &evidence.devices[SENSORARRAY_FRONTEND_SECONDARY]);
        }
    }
    evidence.endedUs = (uint64_t)esp_timer_get_time();
    if (evidence.endedUs > deadlineUs) {
        for (unsigned i = 0u; i < SENSORARRAY_FRONTEND_COUNT; ++i) {
            evidence.devices[i].link = SENSORARRAY_FRONTEND_LINK_UNKNOWN;
            if (evidence.devices[i].error == ESP_OK) { evidence.devices[i].error = ESP_ERR_TIMEOUT; }
        }
    }
    uint64_t observedUs = evidence.endedUs > nowUs ? evidence.endedUs : nowUs;
    sensorarrayFrontendPresenceAction_t action =
        sensorarrayFrontendPresenceObserve(presence, &evidence, observedUs);
    if (outSnapshot) { (void)sensorarrayFrontendPresenceCopySnapshot(presence, outSnapshot); }
    return action;
}
