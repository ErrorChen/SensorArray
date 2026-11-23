#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum complete DATA/LOG message accepted before ATT fragmentation. */
#define SENSORARRAY_BLE_MESSAGE_VALUE_MAX 2304u

typedef enum {
    SENSORARRAY_BLE_CH_DATA = 0,
    SENSORARRAY_BLE_CH_LOG = 1,
    SENSORARRAY_BLE_CH_CTRL = 2,
} sensorarrayBleChannel_t;

typedef enum {
    SENSORARRAY_BLE_TX_FAST = 0,
    SENSORARRAY_BLE_TX_SAFE,
} sensorarrayBleTxMode_t;

typedef void (*sensorarrayBleControlRxCallback_t)(const uint8_t *data,
                                                  size_t length,
                                                  void *userContext);

typedef struct {
    char deviceName[32];
    bool enableBle5;
    bool preferPhy2m;
    bool preferHighTxPower;
    uint16_t preferredMtu;
} sensorarrayBleConfig_t;

typedef struct {
    bool controllerReady;
    bool hostReady;
    bool gattReady;
    bool advertising;
    bool connected;
    bool congested;
    uint16_t mtu;
    uint8_t txPhy;
    uint8_t rxPhy;
    uint32_t sent[3];
    uint32_t dropped[3];
    uint32_t messageQueued;
    uint32_t messageSent;
    uint32_t messageDropped;
    uint32_t fragmentSent;
    uint32_t fragmentError;
    uint32_t tinyTailCount;
    uint32_t congestedCount;
    uint32_t controlRxQueued;
    uint32_t controlRxDropped;
    uint32_t controlRxStale;
    uint32_t txSlotUsed;
    uint32_t txSlotHighWater;
    uint32_t txSlotAllocFail;
    uint32_t txSlotReleaseMismatch;
    uint32_t txSlotStaleGenerationDrop;
    uint32_t txConnectionStaleDrop;
    uint32_t staleConfirmation;
    uint32_t txCrcMismatch;
    uint32_t controlTxRetry;
    uint32_t controlTxRetryExhausted;
    /* Ordinary FF30 admissions suppressed while DATA is effectively subscribed.
     * Included once in dropped[LOG]/messageDropped; boot total, modulo-32-bit. */
    uint32_t logPolicyDrop;
    /* Configured and lifetime-minimum remaining stack sizes, all in bytes. */
    uint32_t txTaskConfiguredBytes;
    uint32_t txTaskMinimumRemainingBytes;
    uint32_t ctrlTaskConfiguredBytes;
    uint32_t ctrlTaskMinimumRemainingBytes;
    esp_err_t initError;
} sensorarrayBleStats_t;

/* New in this repair: one atomic, read-only observation of link negotiation
 * and the application TX owner. Request acceptance is not negotiation success;
 * SDK acceptance is not over-the-air or peer-application delivery. */
typedef struct {
    uint32_t connectionGeneration;
    bool connected;
    uint16_t mtu;
    bool phyKnown;
    bool connParamsKnown;
    uint8_t txPhy;
    uint8_t rxPhy;
    uint16_t connIntervalUnits; /* 1.25 ms, as delivered by ESP-IDF. */
    uint16_t connLatency;
    uint16_t supervisionTimeoutUnits; /* 10 ms. */
    esp_err_t phyRequestError;
    esp_err_t phyFallbackRequestError;
    esp_err_t phyReadRequestError;
    esp_err_t connRequestError;
    esp_err_t dataLengthRequestError;
    uint16_t phySetStatus;    /* UINT16_MAX means no completion observed. */
    uint16_t phyReadStatus;
    uint16_t phyUpdateStatus;
    uint16_t connUpdateStatus;
    uint16_t dataLengthStatus;
    uint16_t dataLengthTx;
    uint16_t dataLengthRx;
    bool dataLengthObserved;
    /* IDF 5.5's DLE completion lacks address/connection ID. Its values are
     * retained as un-attributed evidence, never asserted as this peer's state. */
    bool dataLengthConnectionKnown;
    uint32_t phyUpdateEvents;
    uint32_t connUpdateEvents;
    uint32_t dataLengthEvents;
    uint32_t staleLinkEvents;
    uint32_t dataDequeued;
    uint32_t logDequeued;
    uint32_t priorityDequeued;
    uint32_t queueMaxAgeUs[3]; /* DATA / LOG / CTRL, saturating maxima. */
    bool sdkCallActive;
    uint64_t sdkCallStartedUs;
    uint64_t sdkCallCompletedUs;
    uint32_t sdkCallCount;
    uint32_t sdkAccepted;
    uint32_t sdkErrors;
    uint32_t sdkMaxCallUs;
    /* Added observation only; boot totals, not reset on link reconnect.
     * Completed SDK calls include errors. Wall time includes preemption and
     * possible blocking, so neither it nor idle-epoch change proves CPU cost
     * or blocking. 32-bit counts/64-bit microsecond total wrap modulo width. */
    uint32_t sdkCompletedCalls;
    uint64_t sdkWallUs;
    uint32_t sdkIdleChanged;
    uint32_t sdkIdleUnchanged;
    uint32_t sdkIdleUnknown;
    esp_err_t sdkLastError;
    uint32_t txTaskPriority;
    uint32_t ctrlTaskPriority;
    /* Generic SDK CONF events, boot totals with modulo-32-bit wrap. Status
     * and waiter matching are independent classifications of every event.
     * A matching waiter predicate does not prove the callback's originating
     * connection epoch; conn_id/handle can be reused. Notifications may emit
     * local CONF status, so none of these counts proves peer receipt. */
    uint32_t confOk;
    uint32_t confError;
    uint32_t confMatched;
    uint32_t confUnmatched;
} sensorarrayBleLinkSnapshot_t;

/* New in this fix, queried with BLEGUARD? (v=1). Quarantine is sticky until an
 * explicit application restart. It stops admission, not an in-flight SDK call.
 * When quarantined, callId/start/detected/completionKnown describe the frozen
 * fault call even after a late SDK return or a connection epoch change. */
typedef struct {
    bool ready;
    esp_err_t initError;
    bool quarantined;
    bool callKnown;
    uint32_t callId; /* Modulo-32-bit; pair with startUs within this boot. */
    uint64_t startUs;
    uint64_t detectedUs;
    uint64_t ageUs; /* Time since start, not SDK CPU time or completion time. */
    uint32_t deadlineUs;
    uint32_t pollUs;
    bool active;
    bool completionKnown;
    bool restartRequired;
    uint32_t rejected; /* Guard-blocked admissions/attempts, modulo-32-bit. */
} sensorarrayBleGuardSnapshot_t;

/* New in this fix: BLELOG? v=1. The policy affects new ordinary LOG admissions
 * only. Reserved lifecycle/CTRL and already-owned records are unchanged. */
typedef struct {
    uint32_t connectionGeneration;
    bool connected;
    sensorarrayBleTxMode_t txMode;
    uint16_t dataCccd;
    bool dataActive;
    bool ordinaryAllowed;
    uint32_t policyDrop;
    uint32_t droppedLog;
} sensorarrayBleLogPolicySnapshot_t;

/* 本轮新增: typed encoder publication. All pointers are borrowed for this call
 * only; successful BLE admission deep-copies its selected complete payload.
 * fullData is always the original complete encoder bundle. normalData may be
 * a legacy preview; protected output always selects fullData. Upstream
 * protectionFlags describe source/network protection candidates. BLE first
 * protection is tied to its config/stream, and its one-second obligation is
 * satisfied by a sequence-advancing typed full bundle's local completion;
 * an upstream PERIODIC candidate may therefore enter the ordinary queue. */
enum {
    SENSORARRAY_BLE_FRAME_PROTECT_FIRST_CONFIG = 1u,
    SENSORARRAY_BLE_FRAME_PROTECT_PERIODIC = 2u,
};
typedef struct {
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t revision;
    uint32_t sequence;
} sensorarrayBleFrameIdentity_t;
typedef struct {
    const uint8_t *fullData;
    size_t fullLength;
    const uint8_t *normalData;
    size_t normalLength;
    sensorarrayBleFrameIdentity_t identity;
    uint32_t protectionFlags;
} sensorarrayBleFramePublication_t;

/* 本轮新增: atomic local output observation. Counters are modulo width;
 * local completion/selection never imply PC receipt. */
typedef struct {
    uint32_t connectionGeneration, streamGeneration;
    uint8_t stage;
    uint16_t ordinaryRatePermille, capacityRejectedPermille;
    uint8_t pressureWindows;
    bool active, evidenceKnown, serviceKnown;
    uint32_t localServiceMilliFps, admissionAttempts, capacityRejected;
    uint64_t intentionalSkip, probeSelected, cleanEvidenceUs;
    uint32_t protectedQueued, protectedComplete, protectedFailed;
    uint32_t protectedAdmissionMiss, protectedDeadlineMiss, protectedSuperseded, staleNormal;
    bool lastCompleteKnown, lastCompleteAgeKnown, gapKnown, firstPending, periodicPending;
    uint32_t lastSequence, lastRevision;
    uint64_t lastCompleteAgeUs, maxCompleteGapUs;
} sensorarrayBleFlowSnapshot_t;

esp_err_t sensorarrayBleInit(const sensorarrayBleConfig_t *config);
esp_err_t sensorarrayBleNotify(sensorarrayBleChannel_t channel,
                              const uint8_t *data,
                              size_t length);
void sensorarrayBleSetControlRxCallback(sensorarrayBleControlRxCallback_t callback,
                                       void *userContext);
bool sensorarrayBleIsReady(void);
bool sensorarrayBleIsConnected(void);
bool sensorarrayBleIsSubscribed(sensorarrayBleChannel_t channel);
bool sensorarrayBleCanSend(sensorarrayBleChannel_t channel);
/* Capture the active connection generation for a specific sendable channel.
 * A queued producer can later use NotifyForGeneration to guarantee that its
 * payload is never delivered to a different reconnecting client. */
bool sensorarrayBleGetSendGeneration(sensorarrayBleChannel_t channel,
                                     uint32_t *outConnectionGeneration);
esp_err_t sensorarrayBleNotifyForGeneration(
    sensorarrayBleChannel_t channel,
    const uint8_t *data,
    size_t length,
    uint32_t expectedConnectionGeneration);
esp_err_t sensorarrayBleNotifyFrameForGeneration(
    const sensorarrayBleFramePublication_t *publication,
    uint32_t expectedConnectionGeneration);
/* New in this repair: lifecycle text keeps FF30 framing but uses the existing
 * reserved <=512-byte CTRL pool/queue. Caller must retain/query its terminal
 * result if local admission fails; this is not an exactly-once transport. */
esp_err_t sensorarrayBleNotifyLifecycleForGeneration(
    const uint8_t *data, size_t length, uint32_t expectedConnectionGeneration);
bool sensorarrayBleIsCongested(void);
void sensorarrayBleSetTxMode(sensorarrayBleTxMode_t mode);
sensorarrayBleTxMode_t sensorarrayBleGetTxMode(void);
const char *sensorarrayBleTxModeName(sensorarrayBleTxMode_t mode);
void sensorarrayBleGetStats(sensorarrayBleStats_t *outStats);
void sensorarrayBleGetLinkSnapshot(sensorarrayBleLinkSnapshot_t *outSnapshot);
void sensorarrayBleGetGuardSnapshot(sensorarrayBleGuardSnapshot_t *outSnapshot);
void sensorarrayBleGetLogPolicySnapshot(sensorarrayBleLogPolicySnapshot_t *outSnapshot);
void sensorarrayBleGetFlowSnapshot(sensorarrayBleFlowSnapshot_t *outSnapshot);
void sensorarrayBleLogHeap(const char *stage);

#ifdef __cplusplus
}
#endif
