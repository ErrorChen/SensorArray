#include "sensorarrayBleOutputSchedule.h"

#include <string.h>

static bool epochEqual(const sensorarrayBleOutputEpoch_t *a,
                       const sensorarrayBleOutputEpoch_t *b)
{
    return a->sessionId == b->sessionId && a->bootId == b->bootId &&
           a->connectionGeneration == b->connectionGeneration &&
           a->streamGeneration == b->streamGeneration;
}

static void advanceGeneration(sensorarrayBleOutputSchedule_t *s)
{
    if (++s->snapshot.policyGeneration == 0u) ++s->snapshot.policyGeneration;
}

static void updateMode(sensorarrayBleOutputSchedule_t *s)
{
    s->snapshot.mode = !s->snapshot.active ? SENSORARRAY_BLE_OUTPUT_INACTIVE :
        s->snapshot.ordinaryRatePermille < SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE ?
            SENSORARRAY_BLE_OUTPUT_LIMITED :
        s->snapshot.evidenceKnown && s->snapshot.capacityRejectedPermille >= 100u ?
            SENSORARRAY_BLE_OUTPUT_PRESSURE : SENSORARRAY_BLE_OUTPUT_FULL;
}

static void resetEvidence(sensorarrayBleOutputSchedule_t *s)
{
    s->baselineKnown = false;
    s->snapshot.evidenceKnown = false;
    s->snapshot.serviceKnown = false;
    s->snapshot.pressureWindows = 0u;
    s->snapshot.cleanUs = 0u;
    s->cleanEvidenceKnown = false;
    updateMode(s);
}

static void pauseRecovery(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs)
{
    /* Missing data is not success. A short empty interval pauses accumulation;
     * only recent, actual clean evidence may survive it. */
    if (!s->cleanEvidenceKnown || nowUs - s->lastCleanEvidenceUs > SENSORARRAY_BLE_OUTPUT_CLEAN_TTL_US) {
        s->snapshot.cleanUs = 0u;
        s->cleanEvidenceKnown = false;
    }
}

static bool observeTime(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs)
{
    bool ordered = nowUs >= s->lastNowUs;
    if (ordered) pauseRecovery(s, nowUs);
    if (!ordered) {
        s->snapshot.timeResets++;
        advanceGeneration(s);
        resetEvidence(s);
        s->ordinaryCredit = 0u;
        s->ordinarySelectedKnown = false;
        s->downKnown = false;
    }
    s->lastNowUs = nowUs;
    return ordered;
}

static void setBaseline(sensorarrayBleOutputSchedule_t *s, uint64_t nowUs,
                        const sensorarrayBleOutputCounters_t *counters)
{
    s->baseline = *counters;
    s->windowStartedUs = nowUs;
    s->baselineKnown = true;
}

void sensorarrayBleOutputScheduleInit(sensorarrayBleOutputSchedule_t *s,
                                     uint64_t nowUs)
{
    if (!s) return;
    memset(s, 0, sizeof(*s));
    s->snapshot.ordinaryRatePermille = SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE;
    s->lastNowUs = nowUs;
}

bool sensorarrayBleOutputScheduleObserve(sensorarrayBleOutputSchedule_t *s,
                                        uint64_t nowUs,
                                        const sensorarrayBleOutputObservation_t *observation)
{
    if (!s) return false;
    (void)observeTime(s, nowUs);
    if (!observation) {
        s->snapshot.invalidObservations++;
        resetEvidence(s);
        return false;
    }
    if (!s->epochKnown || !epochEqual(&s->snapshot.epoch, &observation->epoch) ||
        s->snapshot.active != observation->active) {
        s->snapshot.epoch = observation->epoch;
        s->snapshot.active = observation->active;
        s->snapshot.ordinaryRatePermille = SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE;
        s->snapshot.epochChanges++;
        s->snapshot.lastAdjustmentUs = nowUs;
        s->epochKnown = true;
        s->ordinaryCredit = 0u;
        s->ordinarySelectedKnown = false;
        s->downKnown = false;
        advanceGeneration(s);
        resetEvidence(s);
    }
    if (!observation->active) {
        resetEvidence(s);
        return false;
    }
    if (!observation->valid) {
        s->snapshot.invalidObservations++;
        resetEvidence(s);
        return false;
    }
    if (!s->baselineKnown) {
        setBaseline(s, nowUs, &observation->counters);
        return false;
    }
    uint64_t elapsedUs = nowUs - s->windowStartedUs;
    if (elapsedUs < SENSORARRAY_BLE_OUTPUT_WINDOW_US) return false;

    /* Unsigned differences handle counter wrap. A reset without an epoch, a
     * missing interval or impossible delta must not manufacture evidence. */
    sensorarrayBleOutputCounters_t delta = {
        .admissionAttempts = observation->counters.admissionAttempts - s->baseline.admissionAttempts,
        .capacityRejected = observation->counters.capacityRejected - s->baseline.capacityRejected,
        .otherFailures = observation->counters.otherFailures - s->baseline.otherFailures,
        .wholeCompleted = observation->counters.wholeCompleted - s->baseline.wholeCompleted,
    };
    setBaseline(s, nowUs, &observation->counters);
    if (elapsedUs > SENSORARRAY_BLE_OUTPUT_MAX_WINDOW_US ||
        delta.admissionAttempts > SENSORARRAY_BLE_OUTPUT_MAX_WINDOW_EVENTS ||
        delta.capacityRejected > delta.admissionAttempts ||
        delta.otherFailures > SENSORARRAY_BLE_OUTPUT_MAX_WINDOW_EVENTS ||
        delta.wholeCompleted > SENSORARRAY_BLE_OUTPUT_MAX_WINDOW_EVENTS) {
        s->snapshot.invalidObservations++;
        resetEvidence(s);
        setBaseline(s, nowUs, &observation->counters);
        return false;
    }
    if (delta.admissionAttempts == 0u) {
        s->snapshot.noEvidenceWindows++;
        if (delta.otherFailures != 0u) resetEvidence(s);
        else {
            s->snapshot.evidenceKnown = false;
            s->snapshot.serviceKnown = false;
            s->snapshot.pressureWindows = 0u;
            pauseRecovery(s, nowUs);
            updateMode(s);
        }
        setBaseline(s, nowUs, &observation->counters);
        return false;
    }

    s->snapshot.validWindows++;
    s->snapshot.evidenceKnown = true;
    s->snapshot.capacityRejectedPermille = (uint16_t)(
        (uint64_t)delta.capacityRejected * 1000u / delta.admissionAttempts);
    s->snapshot.localServiceMilliFps = (uint32_t)(
        (uint64_t)delta.wholeCompleted * UINT64_C(1000000000) / elapsedUs);
    s->snapshot.serviceKnown = true;
    bool pressured = (uint64_t)delta.capacityRejected * 1000u >
                     (uint64_t)delta.admissionAttempts * 200u;
    bool clean = delta.capacityRejected == 0u && delta.otherFailures == 0u &&
                 delta.wholeCompleted != 0u;
    if (pressured) {
        s->snapshot.cleanUs = 0u;
        s->cleanEvidenceKnown = false;
        if (s->snapshot.pressureWindows < 3u) s->snapshot.pressureWindows++;
        if (s->snapshot.pressureWindows >= 3u &&
            (!s->downKnown || nowUs - s->lastDownUs >= SENSORARRAY_BLE_OUTPUT_DOWN_HOLD_US) &&
            s->snapshot.ordinaryRatePermille != 0u) {
            s->snapshot.ordinaryRatePermille -= SENSORARRAY_BLE_OUTPUT_STEP_PERMILLE;
            s->snapshot.pressureWindows = 0u;
            s->snapshot.downSteps++;
            s->snapshot.lastAdjustmentUs = nowUs;
            s->lastDownUs = nowUs;
            s->downKnown = true;
        }
    } else {
        s->snapshot.pressureWindows = 0u;
        if (clean && s->snapshot.ordinaryRatePermille < SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE) {
            /* Ten seconds of actual clean windows, not ten seconds of idle
             * or missing observations. The bounded interval prevents overflow. */
            pauseRecovery(s, nowUs);
            s->snapshot.cleanUs += elapsedUs;
            s->lastCleanEvidenceUs = nowUs;
            s->cleanEvidenceKnown = true;
            if (s->snapshot.cleanUs >= SENSORARRAY_BLE_OUTPUT_RECOVER_US) {
                s->snapshot.ordinaryRatePermille += SENSORARRAY_BLE_OUTPUT_STEP_PERMILLE;
                s->snapshot.cleanUs = 0u;
                s->cleanEvidenceKnown = false;
                s->snapshot.upSteps++;
                s->snapshot.lastAdjustmentUs = nowUs;
            }
        } else if (delta.capacityRejected != 0u || delta.otherFailures != 0u ||
                   s->snapshot.ordinaryRatePermille == SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE) {
            s->snapshot.cleanUs = 0u;
            s->cleanEvidenceKnown = false;
        } else {
            pauseRecovery(s, nowUs);
        }
    }
    updateMode(s);
    return true;
}

sensorarrayBleOutputTicket_t sensorarrayBleOutputScheduleDecide(
    sensorarrayBleOutputSchedule_t *s, uint64_t nowUs,
    const sensorarrayBleOutputEpoch_t *epoch,
    const sensorarrayBleOutputFrameKey_t *frame, uint8_t protectionFlags)
{
    sensorarrayBleOutputTicket_t ticket = {.decision = SENSORARRAY_BLE_OUTPUT_NO_STREAM};
    if (!s) return ticket;
    (void)observeTime(s, nowUs);
    if (!epoch || !frame || !s->snapshot.active ||
        !epochEqual(&s->snapshot.epoch, epoch)) {
        s->snapshot.inactiveFrames++;
        return ticket;
    }
    ticket.epoch = *epoch;
    ticket.frame = *frame;
    ticket.policyGeneration = s->snapshot.policyGeneration;
    ticket.protectionFlags = protectionFlags;
    s->snapshot.eligibleFrames++;
    if (protectionFlags != 0u) {
        ticket.decision = SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED;
        s->snapshot.selectedProtected++;
    } else {
        /* Fractional frame selection avoids timer quantization (e.g. 100Hz
         * offers with an 11ms deadline accidentally becoming 50Hz). This
         * performs no delay, has no catch-up burst and changes no capture.
         * Protected frames have their own external reservation and budget. */
        s->ordinaryCredit = (uint16_t)(s->ordinaryCredit + s->snapshot.ordinaryRatePermille);
        if (s->ordinaryCredit < SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE) {
            bool probeDue = !s->ordinarySelectedKnown ||
                nowUs - s->lastOrdinarySelectedUs >= SENSORARRAY_BLE_OUTPUT_PROBE_US;
            if (!probeDue) {
                ticket.decision = SENSORARRAY_BLE_OUTPUT_SKIP_POLICY;
                s->snapshot.policySkipped++;
                return ticket;
            }
            s->snapshot.probeSelected++;
            s->ordinaryCredit = 0u;
        } else {
            s->ordinaryCredit -= SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE;
        }
        s->lastOrdinarySelectedUs = nowUs;
        s->ordinarySelectedKnown = true;
        ticket.decision = SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY;
        s->snapshot.selectedOrdinary++;
    }
    if (++s->nextTicketId == 0u) ++s->nextTicketId;
    ticket.ticketId = s->nextTicketId;
    return ticket;
}

uint8_t sensorarrayBleOutputScheduleNoteCompletion(
    sensorarrayBleOutputSchedule_t *s, uint64_t nowUs,
    const sensorarrayBleOutputTicket_t *completed,
    const sensorarrayBleOutputTicket_t *pendingProtection,
    bool wholeFrameAccepted)
{
    if (!s) return 0u;
    (void)observeTime(s, nowUs);
    if (!completed || !s->snapshot.active || completed->ticketId == 0u ||
        (completed->decision != SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY &&
         completed->decision != SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED) ||
        completed->policyGeneration != s->snapshot.policyGeneration ||
        !epochEqual(&completed->epoch, &s->snapshot.epoch)) {
        s->snapshot.ignoredCompletions++;
        return 0u;
    }
    if (!wholeFrameAccepted) {
        s->snapshot.reportedFailed++;
        return 0u;
    }
    s->snapshot.reportedCompleted++;
    if (!pendingProtection || completed->decision != SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED ||
        pendingProtection->decision != SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED ||
        pendingProtection->ticketId != completed->ticketId ||
        pendingProtection->policyGeneration != completed->policyGeneration ||
        !epochEqual(&pendingProtection->epoch, &completed->epoch) ||
        pendingProtection->frame.revision != completed->frame.revision ||
        pendingProtection->frame.sequence != completed->frame.sequence) {
        return 0u;
    }
    uint8_t candidates = completed->protectionFlags & pendingProtection->protectionFlags;
    if (candidates) s->snapshot.protectedAckCandidates++;
    return candidates;
}

void sensorarrayBleOutputScheduleCopySnapshot(
    const sensorarrayBleOutputSchedule_t *s,
    sensorarrayBleOutputScheduleSnapshot_t *snapshot)
{
    if (s && snapshot) *snapshot = s->snapshot;
}
