#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    SENSORARRAY_WIFI_CH_DATA = 0,
    SENSORARRAY_WIFI_CH_LOG = 1,
    SENSORARRAY_WIFI_CH_CTRL = 2,
} sensorarrayWifiChannel_t;

typedef enum {
    SENSORARRAY_WIFI_PROFILE_HIGH = 0,
    SENSORARRAY_WIFI_PROFILE_BLE_COMPAT = 1,
} sensorarrayWifiProfile_t;

typedef struct {
    uint32_t ipv4;
    uint16_t port;
} sensorarrayWifiPeer_t;

typedef void (*sensorarrayWifiControlRxCallback_t)(const uint8_t *data,
                                                   size_t length,
                                                   const sensorarrayWifiPeer_t *peer,
                                                   void *userContext);

typedef struct {
    char ssid[32];
    char password[64];
    sensorarrayWifiProfile_t profile;
    uint16_t dataPort;
    uint16_t logPort;
    uint16_t ctrlPort;
} sensorarrayWifiConfig_t;

typedef struct {
    bool ready;
    bool stationConnected;
    sensorarrayWifiProfile_t profile;
    uint32_t sent[3];
    uint32_t dropped[3];
    uint32_t ctrlRx;
    uint32_t ctrlTx;
    esp_err_t initError;
    uint32_t queued[3];
    uint32_t admissionRejected[3];
    uint32_t expired[3];
    uint32_t stale[3];
    uint32_t partial[3];
    uint64_t driverAcceptedBytes[3];
} sensorarrayWifiStats_t;

/* 本轮新增: all publication/request success means local admission only.
 * Wi-Fi SDK/lwIP execution belongs to a persistent isolated owner. */
#define SENSORARRAY_WIFI_DATA_MAX 2304u
#define SENSORARRAY_WIFI_CTRL_MAX 512u
#define SENSORARRAY_WIFI_SEND_DEADLINE_US UINT64_C(500000)
#define SENSORARRAY_WIFI_QUEUE_DEADLINE_US UINT64_C(1000000)
#define SENSORARRAY_WIFI_INIT_DEADLINE_US UINT64_C(5000000)
#define SENSORARRAY_WIFI_CLEANUP_DEADLINE_US UINT64_C(1000000)
#define SENSORARRAY_WIFI_OBSERVER_POLL_US UINT64_C(100000)

typedef enum {
    SENSORARRAY_WIFI_REQUEST_NONE = 0,
    SENSORARRAY_WIFI_REQUEST_PENDING,
    SENSORARRAY_WIFI_REQUEST_RUNNING,
    SENSORARRAY_WIFI_REQUEST_APPLIED,
    SENSORARRAY_WIFI_REQUEST_FAILED,
    SENSORARRAY_WIFI_REQUEST_SUPERSEDED,
    SENSORARRAY_WIFI_REQUEST_QUARANTINED,
} sensorarrayWifiRequestOutcome_t;

typedef struct {
    bool known;
    uint32_t id;
    uint32_t epoch;
    bool requestedAp;
    sensorarrayWifiRequestOutcome_t outcome;
    esp_err_t originalError;
    esp_err_t cleanupError;
    uint64_t acceptedAtUs;
    uint64_t completedAtUs;
    uint8_t terminalAttempts;
    bool terminalQueued;
    bool terminalPending;
    uint64_t terminalNextAttemptUs;
} sensorarrayWifiRequestResult_t;

typedef enum {
    SENSORARRAY_WIFI_PHASE_IDLE = 0,
    SENSORARRAY_WIFI_PHASE_NETIF_INIT,
    SENSORARRAY_WIFI_PHASE_EVENT_LOOP,
    SENSORARRAY_WIFI_PHASE_NETIF_NEW,
    SENSORARRAY_WIFI_PHASE_NETIF_ATTACH,
    SENSORARRAY_WIFI_PHASE_DEFAULT_HANDLERS,
    SENSORARRAY_WIFI_PHASE_DRIVER_INIT,
    SENSORARRAY_WIFI_PHASE_EVENT_REGISTER,
    SENSORARRAY_WIFI_PHASE_SET_MODE,
    SENSORARRAY_WIFI_PHASE_SET_CONFIG,
    SENSORARRAY_WIFI_PHASE_START,
    SENSORARRAY_WIFI_PHASE_SET_POWER,
    SENSORARRAY_WIFI_PHASE_SET_PS,
    SENSORARRAY_WIFI_PHASE_SET_PROTOCOL,
    SENSORARRAY_WIFI_PHASE_SET_BANDWIDTH,
    SENSORARRAY_WIFI_PHASE_SOCKET,
    SENSORARRAY_WIFI_PHASE_SETSOCKOPT,
    SENSORARRAY_WIFI_PHASE_BIND,
    SENSORARRAY_WIFI_PHASE_SEND,
    SENSORARRAY_WIFI_PHASE_STATION_QUERY,
    SENSORARRAY_WIFI_PHASE_CLOSE,
    SENSORARRAY_WIFI_PHASE_EVENT_UNREGISTER,
    SENSORARRAY_WIFI_PHASE_STOP,
    SENSORARRAY_WIFI_PHASE_DEINIT,
    SENSORARRAY_WIFI_PHASE_CLEAR_HANDLERS,
    SENSORARRAY_WIFI_PHASE_NETIF_DESTROY,
    SENSORARRAY_WIFI_PHASE_RX_TASK_CREATE,
} sensorarrayWifiPhase_t;

typedef struct {
    sensorarrayWifiChannel_t channel;
    bool lifecycle;
    bool unicast;
    sensorarrayWifiPeer_t peer;
    const uint8_t *data;
    size_t length;
    uint32_t expectedEpoch;
    uint64_t deadlineUs; /* zero: local queue TTL; otherwise min with it */
    bool identityKnown;
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t revision;
    uint32_t sequence;
} sensorarrayWifiPublication_t;

typedef struct {
    sensorarrayWifiChannel_t channel;
    bool lifecycle;
    bool fullLength;
    bool late;
    esp_err_t error;
    int sdkErrno;
    int returnedBytes;
    uint32_t epoch;
    size_t requestedBytes;
} sensorarrayWifiTxResult_t;

typedef struct {
    uint64_t sessionId;
    uint32_t bootId;
    bool bootKnown;
    bool startAllowed;
    bool resourceAttempted;
    bool resourceReady;
    esp_err_t resourceError;
    bool outputEnabled;
    bool backendReady;
    bool stationKnown;
    bool quarantined;
    bool ownerBusy;
    uint32_t epoch;
    uint32_t linkEpoch;
    uint32_t requestId;
    bool requestedAp;
    sensorarrayWifiRequestOutcome_t requestOutcome;
    bool sdkActive;
    sensorarrayWifiPhase_t phase;
    uint32_t callId;
    uint64_t startUs;
    uint64_t deadlineUs;
    uint64_t detectedUs;
    uint64_t ageUs;
    bool completionKnown;
    esp_err_t originalError;
    esp_err_t cleanupError;
    esp_err_t sdkError;
    int sdkErrno;
    int returnedBytes;
    uint32_t timeouts;
    uint32_t lateReturns;
    uint32_t heldSlots;
    uint32_t writingSlots;
    uint32_t terminalNoSink;
    uint32_t terminalEvicted;
    uint32_t stackMinimumBytes;
    bool stackKnown;
} sensorarrayWifiOwnerSnapshot_t;

typedef void (*sensorarrayWifiTxResultCallback_t)(const sensorarrayWifiTxResult_t *, void *);
typedef void (*sensorarrayWifiWakeCallback_t)(void);

void sensorarrayWifiConfigureBoot(uint64_t sessionId, uint32_t bootId);
void sensorarrayWifiAllowOwnerStart(void);
esp_err_t sensorarrayWifiRequestAp(const sensorarrayWifiConfig_t *config, uint32_t *outRequestId);
esp_err_t sensorarrayWifiRequestOff(uint32_t *outRequestId);
esp_err_t sensorarrayWifiSubmit(const sensorarrayWifiPublication_t *publication);
bool sensorarrayWifiGetSendEpoch(bool control, uint32_t *outEpoch);
void sensorarrayWifiSetOwnerCallbacks(sensorarrayWifiTxResultCallback_t result,
    void *context, sensorarrayWifiWakeCallback_t wake);
void sensorarrayWifiCopyOwnerSnapshot(sensorarrayWifiOwnerSnapshot_t *snapshot);
bool sensorarrayWifiCopyRequestResult(uint32_t id, sensorarrayWifiRequestResult_t *result);
bool sensorarrayWifiCopyPendingTerminal(sensorarrayWifiRequestResult_t *result);
void sensorarrayWifiNoteTerminalAttempt(uint32_t id, bool actuallyQueued);
bool sensorarrayWifiOwnerNeedsPoll(void);
void sensorarrayWifiObserveDeadline(void);
const char *sensorarrayWifiPhaseName(sensorarrayWifiPhase_t phase);
const char *sensorarrayWifiRequestOutcomeName(sensorarrayWifiRequestOutcome_t outcome);

void sensorarrayWifiSetControlRxCallback(sensorarrayWifiControlRxCallback_t callback,
                                        void *userContext);
void sensorarrayWifiGetStats(sensorarrayWifiStats_t *outStats);
bool sensorarrayWifiIsReady(void);
