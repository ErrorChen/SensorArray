#pragma once

#include <stdbool.h>
#include <stdint.h>

/* 本轮新增: deterministic, allocation-free BLE output policy. No payload,
 * queue, RTOS, clock, SDK or radio ownership. Callers serialize all calls and
 * supply monotonic microseconds. FULL never paces capture or ordinary output.
 * Local capacity evidence is not RSSI or peer/application loss evidence. */
#define SENSORARRAY_BLE_OUTPUT_WINDOW_US UINT64_C(1000000)
#define SENSORARRAY_BLE_OUTPUT_MAX_WINDOW_US UINT64_C(2000000)
#define SENSORARRAY_BLE_OUTPUT_DOWN_HOLD_US UINT64_C(3000000)
#define SENSORARRAY_BLE_OUTPUT_RECOVER_US UINT64_C(10000000)
#define SENSORARRAY_BLE_OUTPUT_CLEAN_TTL_US UINT64_C(3000000)
#define SENSORARRAY_BLE_OUTPUT_PROBE_US UINT64_C(1000000)
#define SENSORARRAY_BLE_OUTPUT_STEP_PERMILLE 50u
#define SENSORARRAY_BLE_OUTPUT_FULL_PERMILLE 1000u
/* An observation sanity bound, not a source/output frame-rate limit. */
#define SENSORARRAY_BLE_OUTPUT_MAX_WINDOW_EVENTS UINT32_C(1000000)

typedef struct {
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t connectionGeneration;
    /* Effective DATA false->true, including same-connection resubscribe.
     * Integration owns this generation; connectionGeneration alone is not
     * sufficient. Equality only: never order generations across boots. */
    uint32_t streamGeneration;
} sensorarrayBleOutputEpoch_t;

typedef struct {
    uint32_t revision;
    uint32_t sequence;
} sensorarrayBleOutputFrameKey_t;

enum {
    SENSORARRAY_BLE_OUTPUT_PROTECT_NONE = 0u,
    SENSORARRAY_BLE_OUTPUT_PROTECT_FIRST_CONFIG = 1u,
    SENSORARRAY_BLE_OUTPUT_PROTECT_PERIODIC = 2u,
};

typedef enum {
    SENSORARRAY_BLE_OUTPUT_INACTIVE = 0,
    SENSORARRAY_BLE_OUTPUT_FULL,
    SENSORARRAY_BLE_OUTPUT_PRESSURE,
    SENSORARRAY_BLE_OUTPUT_LIMITED,
} sensorarrayBleOutputMode_t;

typedef enum {
    SENSORARRAY_BLE_OUTPUT_SEND_ORDINARY = 0,
    SENSORARRAY_BLE_OUTPUT_SEND_PROTECTED,
    SENSORARRAY_BLE_OUTPUT_SKIP_POLICY,
    SENSORARRAY_BLE_OUTPUT_NO_STREAM,
} sensorarrayBleOutputDecision_t;

/* Cumulative modulo-32 counters, from one atomic snapshot and one epoch.
 * Attempts exclude intentional policy skips; capacityRejected counts only
 * local DATA slot/queue/congestion rejection. Other errors (CRC, stale epoch,
 * quarantine, invalid payload, SDK failure) must not be relabelled capacity.
 * wholeCompleted means the complete message was accepted locally, not that
 * the PC received it. A stalled/nonreturning SDK is the independent guard's
 * responsibility; this module cannot cancel it or restore its resources. */
typedef struct {
    uint32_t admissionAttempts;
    uint32_t capacityRejected;
    uint32_t otherFailures;
    uint32_t wholeCompleted;
} sensorarrayBleOutputCounters_t;

typedef struct {
    sensorarrayBleOutputEpoch_t epoch;
    sensorarrayBleOutputCounters_t counters;
    bool active;
    bool valid;
} sensorarrayBleOutputObservation_t;

/* Copy this POD with the external descriptor; it owns no referenced memory.
 * The external protection owner retains its own current pending ticket and
 * obligations. A new decision never means admission or successful completion. */
typedef struct {
    sensorarrayBleOutputEpoch_t epoch;
    sensorarrayBleOutputFrameKey_t frame;
    uint64_t ticketId; /* Nonzero; wrap skips zero; pair with policyGeneration. */
    uint32_t policyGeneration;
    uint8_t protectionFlags;
    sensorarrayBleOutputDecision_t decision;
} sensorarrayBleOutputTicket_t;

typedef struct {
    sensorarrayBleOutputEpoch_t epoch;
    sensorarrayBleOutputMode_t mode;
    uint32_t policyGeneration;
    uint16_t ordinaryRatePermille;
    uint16_t capacityRejectedPermille;
    uint8_t pressureWindows;
    bool active;
    bool evidenceKnown;
    bool serviceKnown;
    uint32_t localServiceMilliFps; /* Local whole-message completions only. */
    uint64_t cleanUs;
    uint64_t lastAdjustmentUs;
    /* Modulo-64 boot totals. Decisions, local completion, and peer receipt
     * remain separate; RSSI and peer loss are deliberately not estimated. */
    uint64_t eligibleFrames;
    uint64_t selectedOrdinary;
    /* Bounded recovery probes are included in selectedOrdinary and may make
     * actual ordinary output exceed its proportional rate. Selection does
     * not prove completion. At most one such selection per monotonic second. */
    uint64_t probeSelected;
    uint64_t selectedProtected;
    uint64_t policySkipped;
    uint64_t inactiveFrames;
    uint64_t validWindows;
    uint64_t noEvidenceWindows;
    uint64_t invalidObservations;
    uint64_t timeResets;
    uint64_t epochChanges;
    uint64_t downSteps;
    uint64_t upSteps;
    uint64_t reportedCompleted;
    uint64_t reportedFailed;
    uint64_t ignoredCompletions;
    uint64_t protectedAckCandidates;
} sensorarrayBleOutputScheduleSnapshot_t;

/* Public storage permits static allocation and host tests. Fields below the
 * snapshot are private state; do not mutate them after Init. */
typedef struct {
    sensorarrayBleOutputScheduleSnapshot_t snapshot;
    sensorarrayBleOutputCounters_t baseline;
    uint64_t windowStartedUs;
    uint64_t lastNowUs;
    uint64_t lastDownUs;
    uint64_t nextTicketId;
    uint64_t lastCleanEvidenceUs;
    uint64_t lastOrdinarySelectedUs;
    uint16_t ordinaryCredit;
    bool epochKnown;
    bool baselineKnown;
    bool downKnown;
    bool cleanEvidenceKnown;
    bool ordinarySelectedKnown;
} sensorarrayBleOutputSchedule_t;

void sensorarrayBleOutputScheduleInit(sensorarrayBleOutputSchedule_t *state,
                                     uint64_t nowUs);
/* Returns true only for a valid completed evidence window. Startup, inactive,
 * a short interval, no attempts, missing/invalid data and a >2s gap cannot
 * count as either pressure or recovery evidence. */
bool sensorarrayBleOutputScheduleObserve(sensorarrayBleOutputSchedule_t *state,
                                        uint64_t nowUs,
                                        const sensorarrayBleOutputObservation_t *observation);
/* Call once per offered complete encoder frame, before ordinary BLE admission.
 * Do not pass a shortened preview as a protected complete frame. Any nonzero
 * protectionFlags bypasses policy skipping; no-stream is still no-stream.
 * 1Hz and first-config obligations belong to the external protection queue. */
sensorarrayBleOutputTicket_t sensorarrayBleOutputScheduleDecide(
    sensorarrayBleOutputSchedule_t *state, uint64_t nowUs,
    const sensorarrayBleOutputEpoch_t *epoch,
    const sensorarrayBleOutputFrameKey_t *frame, uint8_t protectionFlags);
/* Report each real terminal completion once. This never clears an obligation.
 * Returns an acknowledgement candidate bitmask only for successful WHOLE
 * local completion in the current epoch matching the external pending ticket.
 * Partial/failed completion, replaced pending tickets and stale subscriptions
 * return zero. External owner must apply the result under its own same lock. */
uint8_t sensorarrayBleOutputScheduleNoteCompletion(
    sensorarrayBleOutputSchedule_t *state, uint64_t nowUs,
    const sensorarrayBleOutputTicket_t *completed,
    const sensorarrayBleOutputTicket_t *pendingProtection,
    bool wholeFrameAccepted);
void sensorarrayBleOutputScheduleCopySnapshot(
    const sensorarrayBleOutputSchedule_t *state,
    sensorarrayBleOutputScheduleSnapshot_t *snapshot);
