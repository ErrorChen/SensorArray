#include "sensorarrayWifiOwner.h"
#include "sensorarrayWifiWatchdog.h"
#include "sensorarrayWorkBudget.h"

#include <string.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#ifndef CONFIG_SENSORARRAY_NET_TASK_CORE
#define CONFIG_SENSORARRAY_NET_TASK_CORE 0
#endif

#define WIFI_NORMAL_SLOTS 2u
#define WIFI_LIFECYCLE_SLOT 2u
#define WIFI_CTRL_SLOT 3u
#define WIFI_SLOT_COUNT 4u
#define WIFI_PAYLOAD_BYTES (3u * SENSORARRAY_WIFI_DATA_MAX + SENSORARRAY_WIFI_CTRL_MAX)
/* Actual PANIC evidence F9CE9DC1 identifies wifiOwner stack overflow at 4096.
 * The failing ELF used 1200B before SDK entry; unbuffered SDK logging can
 * nest 1888B of libc formatter frames. Keep a bounded internal stack budget;
 * job snapshots below remove avoidable nesting. Verify the real post-job
 * watermark on target; this is not a proof of every closed-SDK stack path. */
#define WIFI_OWNER_STACK_BYTES 6144u
#define WIFI_TERMINAL_ATTEMPTS 3u
#define WIFI_TERMINAL_RETRY_US UINT64_C(200000)

typedef enum { WIFI_FREE, WIFI_WRITING, WIFI_PENDING, WIFI_OWNED } wifiSlotState_t;
typedef struct {
    wifiSlotState_t state;
    uint32_t generation;
    uint32_t order;
    uint8_t *bytes;
    sensorarrayWifiPublication_t publication;
} wifiSlot_t;

typedef struct {
    sensorarrayWifiPublication_t publication;
    sensorarrayWifiRequestResult_t request;
    sensorarrayWifiConfig_t config;
    sensorarrayWifiConfig_t applied;
    bool backendReady;
} wifiExecution_t;

/* One persistent owner, one execution snapshot. Admission/OFF/observer never
 * mutate this storage. A new snapshot is taken only after the previous SDK
 * job actually returns and jobOwned/sdkActive are clear, not at timeout. */
static wifiExecution_t s_execution;

/* All metadata/locks stay internal. Only the persistent payload bytes use
 * PSRAM; sendto reads ordinary CPU memory, not an application DMA buffer. */
static portMUX_TYPE s_wifiMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_ownerTask;
static struct {
    uint64_t sessionId;
    uint32_t bootId;
    bool bootKnown;
    bool startAllowed;
    uint8_t resourceState; /* 0 absent, 1 claimed, 2 ready, 3 sticky failure */
    esp_err_t resourceError;
    uint8_t *payload;
    wifiSlot_t slots[WIFI_SLOT_COUNT];
    uint32_t nextOrder;
    uint32_t nextRequestId;
    uint32_t epoch;
    uint32_t linkEpoch;
    uint32_t latestRequestId;
    sensorarrayWifiRequestResult_t request;
    sensorarrayWifiRequestResult_t terminal[2];
    uint8_t terminalWrite;
    sensorarrayWifiConfig_t requestedConfig;
    sensorarrayWifiConfig_t appliedConfig;
    bool outputEnabled;
    bool backendReady;
    bool stationKnown;
    bool reconcileStation;
    bool quarantined;
    bool jobOwned;
    bool jobCleanup;
    uint32_t jobEpoch;
    uint32_t jobLinkEpoch;
    uint32_t jobRequestId;
    uint64_t jobDeadlineUs;
    bool sdkActive;
    sensorarrayWifiPhase_t phase;
    uint32_t callId;
    uint64_t callStartUs;
    uint64_t callDeadlineUs;
    uint64_t detectedUs;
    bool completionKnown;
    esp_err_t originalError;
    esp_err_t cleanupError;
    esp_err_t sdkError;
    int sdkErrno;
    int returnedBytes;
    uint32_t timeouts;
    uint32_t lateReturns;
    uint32_t terminalNoSink;
    uint32_t terminalEvicted;
    uint32_t stackMinimumBytes;
    bool stackKnown;
    sensorarrayWifiStats_t stats;
    sensorarrayWifiControlRxCallback_t control;
    void *controlContext;
    sensorarrayWifiTxResultCallback_t result;
    void *resultContext;
    sensorarrayWifiWakeCallback_t wake;
} s_owner = {.resourceError = ESP_ERR_INVALID_STATE, .epoch = 1u, .linkEpoch = 1u};

static uint64_t wifiNow(void) { return (uint64_t)esp_timer_get_time(); }
static uint64_t wifiAdd(uint64_t a, uint64_t b) { return UINT64_MAX - a < b ? UINT64_MAX : a + b; }
static uint32_t wifiNext(uint32_t value) { return ++value ? value : 1u; }
static bool wifiTerminal(sensorarrayWifiRequestOutcome_t outcome)
{
    return outcome >= SENSORARRAY_WIFI_REQUEST_APPLIED;
}

static void wifiWake(void)
{
    sensorarrayWifiWakeCallback_t callback;
    portENTER_CRITICAL(&s_wifiMux);
    callback = s_owner.wake;
    portEXIT_CRITICAL(&s_wifiMux);
    if (callback) { callback(); }
}

static void wifiNotifyOwner(void)
{
    if (s_ownerTask) { xTaskNotifyGive(s_ownerTask); }
}

static sensorarrayWifiRequestResult_t *wifiFindResultLocked(uint32_t id)
{
    if (id == 0u) { id = s_owner.latestRequestId; }
    if (s_owner.request.known && s_owner.request.id == id) { return &s_owner.request; }
    for (unsigned i = 0; i < 2u; ++i) {
        if (s_owner.terminal[i].known && s_owner.terminal[i].id == id) { return &s_owner.terminal[i]; }
    }
    return NULL;
}

static void wifiFinishRequestLocked(uint32_t id, sensorarrayWifiRequestOutcome_t outcome,
    esp_err_t error, esp_err_t cleanup, uint64_t now)
{
    /* Exactly one terminal commit. A late SDK result/OFF must not rewrite a
     * failed, superseded or quarantined request already in the result cache. */
    if (!s_owner.request.known || s_owner.request.id != id || wifiTerminal(s_owner.request.outcome)) { return; }
    sensorarrayWifiRequestResult_t done = s_owner.request;
    done.outcome = outcome;
    done.originalError = error;
    done.cleanupError = cleanup;
    done.completedAtUs = now;
    done.terminalPending = true;
    done.terminalNextAttemptUs = now;
    if (s_owner.terminal[s_owner.terminalWrite].known) { s_owner.terminalEvicted++; }
    s_owner.terminal[s_owner.terminalWrite] = done;
    s_owner.terminalWrite ^= 1u;
    s_owner.request.known = false;
}

static void wifiQuarantineLocked(esp_err_t error, uint64_t now)
{
    if (s_owner.quarantined) { return; }
    s_owner.quarantined = true;
    s_owner.outputEnabled = false;
    s_owner.detectedUs = now;
    if (s_owner.jobRequestId && !s_owner.jobCleanup && s_owner.cleanupError == ESP_OK) {
        s_owner.cleanupError = ESP_ERR_INVALID_STATE; /* cleanup was not established */
    }
    if (s_owner.originalError == ESP_OK) { s_owner.originalError = error; }
    if (s_owner.jobRequestId) {
        wifiFinishRequestLocked(s_owner.jobRequestId, SENSORARRAY_WIFI_REQUEST_QUARANTINED,
            s_owner.originalError, s_owner.cleanupError, now);
    }
}

static void wifiObserveLocked(uint64_t now)
{
    if (s_owner.sdkActive && !s_owner.quarantined &&
        (now < s_owner.callStartUs || now >= s_owner.callDeadlineUs)) {
        s_owner.timeouts++;
        if (s_owner.jobCleanup && s_owner.cleanupError == ESP_OK) { s_owner.cleanupError = ESP_ERR_TIMEOUT; }
        wifiQuarantineLocked(ESP_ERR_TIMEOUT, now);
    }
    /* A management request waiting behind finite-priority traffic also has
     * one absolute total budget. No SDK has to return for its logical result. */
    if (s_owner.request.known && !wifiTerminal(s_owner.request.outcome) &&
        now >= wifiAdd(s_owner.request.acceptedAtUs,
            SENSORARRAY_WIFI_INIT_DEADLINE_US + SENSORARRAY_WIFI_CLEANUP_DEADLINE_US)) {
        wifiFinishRequestLocked(s_owner.request.id, SENSORARRAY_WIFI_REQUEST_FAILED,
            ESP_ERR_TIMEOUT, ESP_OK, now);
        s_owner.outputEnabled = false;
    }
}

void sensorarrayWifiObserveDeadline(void)
{
    portENTER_CRITICAL(&s_wifiMux);
    uint64_t now = wifiNow();
    wifiObserveLocked(now);
    portEXIT_CRITICAL(&s_wifiMux);
    /* Timer callback: no notify, log, allocation, reclamation or SDK. */
}

bool sensorarrayWifiOwnerMayRunSdk(void)
{
    bool allowed;
    portENTER_CRITICAL(&s_wifiMux);
    allowed = s_owner.jobOwned && !s_owner.quarantined &&
        (s_owner.jobCleanup || s_owner.jobEpoch == s_owner.epoch) &&
        (s_owner.jobLinkEpoch == 0u || s_owner.jobLinkEpoch == s_owner.linkEpoch);
    portEXIT_CRITICAL(&s_wifiMux);
    return allowed && xTaskGetCurrentTaskHandle() == s_ownerTask;
}

bool sensorarrayWifiOwnerBeginSdk(sensorarrayWifiPhase_t phase)
{
    if (xTaskGetCurrentTaskHandle() != s_ownerTask) { return false; }
    portENTER_CRITICAL(&s_wifiMux);
    uint64_t now = wifiNow();
    bool allowed = s_owner.jobOwned && !s_owner.sdkActive && !s_owner.quarantined &&
        (s_owner.jobCleanup || s_owner.jobEpoch == s_owner.epoch) &&
        (s_owner.jobLinkEpoch == 0u || s_owner.jobLinkEpoch == s_owner.linkEpoch) && now < s_owner.jobDeadlineUs;
    if (allowed) {
        s_owner.phase = phase;
        s_owner.callId = wifiNext(s_owner.callId);
        s_owner.callStartUs = now;
        s_owner.callDeadlineUs = s_owner.jobDeadlineUs;
        s_owner.completionKnown = false;
        s_owner.sdkActive = true;
    }
    portEXIT_CRITICAL(&s_wifiMux);
    return allowed;
}

bool sensorarrayWifiOwnerEndSdk(esp_err_t error, int sdkErrno, int returnedBytes)
{
    portENTER_CRITICAL(&s_wifiMux);
    uint64_t now = wifiNow();
    /* Observe before clearing active: a call returning between timer ticks
     * cannot escape the deadline. Keep the actual result separate. */
    wifiObserveLocked(now);
    s_owner.sdkError = error;
    s_owner.sdkErrno = sdkErrno;
    s_owner.returnedBytes = returnedBytes;
    s_owner.completionKnown = true;
    s_owner.sdkActive = false;
    if (s_owner.quarantined) { s_owner.lateReturns++; }
    bool allowed = !s_owner.quarantined &&
        (s_owner.jobCleanup || s_owner.jobEpoch == s_owner.epoch) &&
        (s_owner.jobLinkEpoch == 0u || s_owner.jobLinkEpoch == s_owner.linkEpoch);
    portEXIT_CRITICAL(&s_wifiMux);
    return allowed;
}

static bool wifiSameConfig(const sensorarrayWifiConfig_t *a, const sensorarrayWifiConfig_t *b,
    bool includeProfile)
{
    return (!includeProfile || a->profile == b->profile) &&
        a->dataPort == b->dataPort && a->logPort == b->logPort && a->ctrlPort == b->ctrlPort &&
        memcmp(a->ssid, b->ssid, sizeof(a->ssid)) == 0 &&
        memcmp(a->password, b->password, sizeof(a->password)) == 0;
}

static void sensorarrayWifiOwnerTask(void *arg)
{
    (void)arg;
    while (true) {
        while (sensorarrayWifiOwnerProcessOnce()) {
            (void)sensorarrayWorkBudgetCheckpoint(SENSORARRAY_WORK_BUDGET_WIFI_OWNER);
            uint32_t remaining = (uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);
            portENTER_CRITICAL(&s_wifiMux);
            s_owner.stackMinimumBytes = remaining;
            s_owner.stackKnown = true;
            portEXIT_CRITICAL(&s_wifiMux);
        }
        (void)ulTaskNotifyTake(pdTRUE, portMAX_DELAY); /* idle only */
    }
}

static esp_err_t wifiPrepareResources(void)
{
    portENTER_CRITICAL(&s_wifiMux);
    if (s_owner.resourceState != 0u) {
        esp_err_t result = s_owner.resourceState == 2u ? ESP_OK :
            s_owner.resourceState == 1u ? ESP_ERR_INVALID_STATE : s_owner.resourceError;
        portEXIT_CRITICAL(&s_wifiMux);
        return result;
    }
    s_owner.resourceState = 1u;
    portEXIT_CRITICAL(&s_wifiMux);
    /* Only this claimed initializer may allocate/start objects. Never reset
     * live state or reallocate after any partial startup failure. */
    uint8_t *payload = heap_caps_malloc(WIFI_PAYLOAD_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    esp_err_t err = payload ? ESP_OK : ESP_ERR_NO_MEM;
    if (payload) {
        portENTER_CRITICAL(&s_wifiMux);
        s_owner.payload = payload;
        for (unsigned i = 0; i < WIFI_SLOT_COUNT; ++i) {
            s_owner.slots[i].bytes = payload + i * SENSORARRAY_WIFI_DATA_MAX;
        }
        portEXIT_CRITICAL(&s_wifiMux);
        err = sensorarrayWifiWatchdogInit();
    }
    if (err == ESP_OK) {
        (void)sensorarrayWorkBudgetInit();
        if (xTaskCreatePinnedToCore(sensorarrayWifiOwnerTask, "wifiOwner", WIFI_OWNER_STACK_BYTES,
            NULL, 5u, &s_ownerTask, CONFIG_SENSORARRAY_NET_TASK_CORE) != pdPASS) {
            err = ESP_ERR_NO_MEM;
        }
    }
    portENTER_CRITICAL(&s_wifiMux);
    s_owner.resourceError = err;
    s_owner.stats.initError = err;
    s_owner.resourceState = err == ESP_OK ? 2u : 3u;
    portEXIT_CRITICAL(&s_wifiMux);
    return err;
}

void sensorarrayWifiConfigureBoot(uint64_t sessionId, uint32_t bootId)
{
    portENTER_CRITICAL(&s_wifiMux);
    if (!s_owner.bootKnown && s_owner.resourceState == 0u && sessionId != 0u) {
        s_owner.sessionId = sessionId;
        s_owner.bootId = bootId;
        s_owner.bootKnown = true;
    }
    portEXIT_CRITICAL(&s_wifiMux);
}

void sensorarrayWifiAllowOwnerStart(void)
{
    portENTER_CRITICAL(&s_wifiMux);
    s_owner.startAllowed = true;
    portEXIT_CRITICAL(&s_wifiMux);
}

static uint32_t wifiNewRequestLocked(bool ap, uint64_t now)
{
    s_owner.nextRequestId = wifiNext(s_owner.nextRequestId);
    s_owner.latestRequestId = s_owner.nextRequestId;
    s_owner.request = (sensorarrayWifiRequestResult_t){
        .known = true, .id = s_owner.nextRequestId, .epoch = s_owner.epoch,
        .requestedAp = ap, .outcome = SENSORARRAY_WIFI_REQUEST_PENDING, .acceptedAtUs = now,
    };
    return s_owner.nextRequestId;
}

esp_err_t sensorarrayWifiRequestAp(const sensorarrayWifiConfig_t *config, uint32_t *outRequestId)
{
    if (outRequestId) { *outRequestId = 0u; }
    if (!config || config->ssid[0] == '\0' || (unsigned)config->profile > SENSORARRAY_WIFI_PROFILE_BLE_COMPAT ||
        !config->dataPort || !config->logPort || !config->ctrlPort) { return ESP_ERR_INVALID_ARG; }
    portENTER_CRITICAL(&s_wifiMux);
    uint32_t reservationEpoch = s_owner.epoch;
    bool allowed = s_owner.bootKnown && s_owner.startAllowed && !s_owner.quarantined &&
        !s_owner.jobOwned && !s_owner.sdkActive && !s_owner.request.known;
    portEXIT_CRITICAL(&s_wifiMux);
    if (!allowed) { return ESP_ERR_INVALID_STATE; }
    esp_err_t err = wifiPrepareResources();
    if (err != ESP_OK) { return err; }
    uint64_t now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    if (s_owner.quarantined || s_owner.jobOwned || s_owner.sdkActive || s_owner.request.known ||
        reservationEpoch != s_owner.epoch) {
        portEXIT_CRITICAL(&s_wifiMux);
        return ESP_ERR_INVALID_STATE;
    }
    bool same = s_owner.backendReady && wifiSameConfig(config, &s_owner.appliedConfig, true);
    if (!same) {
        s_owner.epoch = wifiNext(s_owner.epoch);
        s_owner.linkEpoch = wifiNext(s_owner.linkEpoch);
    }
    uint32_t id = wifiNewRequestLocked(true, now);
    s_owner.requestedConfig = *config;
    s_owner.outputEnabled = same;
    if (same) { wifiFinishRequestLocked(id, SENSORARRAY_WIFI_REQUEST_APPLIED, ESP_OK, ESP_OK, now); }
    if (outRequestId) { *outRequestId = id; }
    portEXIT_CRITICAL(&s_wifiMux);
    wifiNotifyOwner();
    wifiWake();
    return ESP_OK;
}

esp_err_t sensorarrayWifiRequestOff(uint32_t *outRequestId)
{
    uint64_t now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    if (!s_owner.bootKnown) {
        portEXIT_CRITICAL(&s_wifiMux);
        if (outRequestId) { *outRequestId = 0u; }
        return ESP_ERR_INVALID_STATE;
    }
    if (s_owner.request.known) {
        wifiFinishRequestLocked(s_owner.request.id, SENSORARRAY_WIFI_REQUEST_SUPERSEDED,
            ESP_OK, ESP_OK, now);
    }
    s_owner.outputEnabled = false;
    s_owner.epoch = wifiNext(s_owner.epoch);
    s_owner.linkEpoch = wifiNext(s_owner.linkEpoch);
    uint32_t id = wifiNewRequestLocked(false, now);
    wifiFinishRequestLocked(id, SENSORARRAY_WIFI_REQUEST_APPLIED, ESP_OK, ESP_OK, now);
    if (outRequestId) { *outRequestId = id; }
    /* Physical call/owner/slots and quarantine are deliberately untouched. */
    portEXIT_CRITICAL(&s_wifiMux);
    wifiNotifyOwner();
    wifiWake();
    return ESP_OK;
}

bool sensorarrayWifiGetRequestedAp(bool *requestedAp)
{
    if (!requestedAp) { return false; }
    portENTER_CRITICAL(&s_wifiMux);
    sensorarrayWifiRequestResult_t *request = wifiFindResultLocked(0u);
    bool known = request != NULL;
    *requestedAp = known && request->requestedAp;
    portEXIT_CRITICAL(&s_wifiMux);
    return known;
}

bool sensorarrayWifiGetSendEpoch(bool control, uint32_t *outEpoch)
{
    portENTER_CRITICAL(&s_wifiMux);
    bool available = s_owner.resourceState == 2u && s_owner.backendReady && !s_owner.quarantined &&
        (control || s_owner.outputEnabled);
    if (outEpoch) { *outEpoch = available ? s_owner.linkEpoch : 0u; }
    portEXIT_CRITICAL(&s_wifiMux);
    return available;
}

esp_err_t sensorarrayWifiSubmit(const sensorarrayWifiPublication_t *p)
{
    if (!p || (unsigned)p->channel > SENSORARRAY_WIFI_CH_CTRL || !p->data || !p->length ||
        p->length > (p->channel == SENSORARRAY_WIFI_CH_CTRL ? SENSORARRAY_WIFI_CTRL_MAX : SENSORARRAY_WIFI_DATA_MAX) ||
        (p->lifecycle && p->channel != SENSORARRAY_WIFI_CH_LOG) ||
        (p->unicast && (p->channel != SENSORARRAY_WIFI_CH_CTRL || !p->peer.port))) { return ESP_ERR_INVALID_ARG; }
    uint64_t now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    bool available = s_owner.resourceState == 2u && s_owner.backendReady && !s_owner.quarantined &&
        p->expectedEpoch != 0u && p->expectedEpoch == s_owner.linkEpoch &&
        (p->channel == SENSORARRAY_WIFI_CH_CTRL ||
            (s_owner.outputEnabled && s_owner.stationKnown && s_owner.stats.stationConnected));
    unsigned first = p->channel == SENSORARRAY_WIFI_CH_CTRL ? WIFI_CTRL_SLOT :
        p->lifecycle ? WIFI_LIFECYCLE_SLOT : 0u;
    unsigned end = first < WIFI_NORMAL_SLOTS ? WIFI_NORMAL_SLOTS : first + 1u;
    unsigned index = first;
    while (index < end && s_owner.slots[index].state != WIFI_FREE) { ++index; }
    if (!available || index == end) {
        s_owner.stats.admissionRejected[p->channel]++;
        s_owner.stats.dropped[p->channel]++;
        portEXIT_CRITICAL(&s_wifiMux);
        return available ? ESP_ERR_NO_MEM : ESP_ERR_INVALID_STATE;
    }
    wifiSlot_t *slot = &s_owner.slots[index];
    slot->state = WIFI_WRITING;
    slot->generation = wifiNext(slot->generation);
    uint32_t generation = slot->generation;
    uint8_t *bytes = slot->bytes;
    portEXIT_CRITICAL(&s_wifiMux);
    memcpy(bytes, p->data, p->length);
    sensorarrayWifiPublication_t publication = *p;
    publication.data = bytes;
    uint64_t localDeadline = wifiAdd(now, SENSORARRAY_WIFI_QUEUE_DEADLINE_US);
    publication.deadlineUs = p->deadlineUs && p->deadlineUs < localDeadline ? p->deadlineUs : localDeadline;
    portENTER_CRITICAL(&s_wifiMux);
    if (slot->state != WIFI_WRITING || slot->generation != generation || s_owner.quarantined ||
        publication.expectedEpoch != s_owner.linkEpoch ||
        (p->channel != SENSORARRAY_WIFI_CH_CTRL && !s_owner.outputEnabled)) {
        /* Only this producer owns this WRITING reservation. */
        if (slot->state == WIFI_WRITING && slot->generation == generation) { slot->state = WIFI_FREE; }
        s_owner.stats.admissionRejected[p->channel]++;
        s_owner.stats.dropped[p->channel]++;
        portEXIT_CRITICAL(&s_wifiMux);
        return ESP_ERR_INVALID_STATE;
    }
    slot->publication = publication;
    s_owner.nextOrder = wifiNext(s_owner.nextOrder);
    slot->order = s_owner.nextOrder;
    slot->state = WIFI_PENDING;
    s_owner.stats.queued[p->channel]++;
    portEXIT_CRITICAL(&s_wifiMux);
    wifiNotifyOwner();
    wifiWake();
    return ESP_OK;
}

static int wifiTakeSlotLocked(void)
{
    if (s_owner.slots[WIFI_CTRL_SLOT].state == WIFI_PENDING) { return WIFI_CTRL_SLOT; }
    if (s_owner.slots[WIFI_LIFECYCLE_SLOT].state == WIFI_PENDING) { return WIFI_LIFECYCLE_SLOT; }
    if (s_owner.request.known) { return -1; } /* management before normal */
    int selected = -1;
    for (unsigned i = 0; i < WIFI_NORMAL_SLOTS; ++i) {
        if (s_owner.slots[i].state == WIFI_PENDING && (selected < 0 ||
            (uint32_t)(s_owner.slots[selected].order - s_owner.slots[i].order) < UINT32_C(0x80000000))) {
            selected = (int)i;
        }
    }
    return selected;
}

static void wifiRunDatagram(unsigned index, const sensorarrayWifiPublication_t *publication)
{
    uint64_t now = wifiNow();
    sensorarrayWifiTxResult_t result = {.channel = publication->channel,
        .lifecycle = publication->lifecycle, .epoch = publication->expectedEpoch,
        .requestedBytes = publication->length, .returnedBytes = -1};
    portENTER_CRITICAL(&s_wifiMux);
    bool stale = publication->expectedEpoch != s_owner.linkEpoch ||
        (publication->channel != SENSORARRAY_WIFI_CH_CTRL && !s_owner.outputEnabled);
    bool expired = now >= publication->deadlineUs;
    s_owner.jobDeadlineUs = wifiAdd(now, SENSORARRAY_WIFI_SEND_DEADLINE_US);
    if (publication->deadlineUs < s_owner.jobDeadlineUs) { s_owner.jobDeadlineUs = publication->deadlineUs; }
    portEXIT_CRITICAL(&s_wifiMux);
    if (stale || expired) { result.error = ESP_ERR_TIMEOUT; }
    else {
        result.error = sensorarrayWifiBackendSend(publication, &result.returnedBytes, &result.sdkErrno);
        result.fullLength = result.returnedBytes == (int)publication->length;
    }
    portENTER_CRITICAL(&s_wifiMux);
    result.late = s_owner.quarantined;
    if (result.fullLength) {
        s_owner.stats.sent[publication->channel]++;
        s_owner.stats.driverAcceptedBytes[publication->channel] += publication->length;
        if (publication->channel == SENSORARRAY_WIFI_CH_CTRL) { s_owner.stats.ctrlTx++; }
    } else {
        s_owner.stats.dropped[publication->channel]++;
        if (expired) { s_owner.stats.expired[publication->channel]++; }
        if (stale) { s_owner.stats.stale[publication->channel]++; }
        if (result.returnedBytes > 0) {
            s_owner.stats.partial[publication->channel]++;
            s_owner.stats.driverAcceptedBytes[publication->channel] += (uint32_t)result.returnedBytes;
        }
    }
    /* Backend really returned (or was never entered). This is the only
     * point reclaiming the OWNED payload; the timer/OFF never does so. */
    s_owner.slots[index].state = WIFI_FREE;
    s_owner.jobOwned = false;
    sensorarrayWifiTxResultCallback_t callback = s_owner.result;
    void *context = s_owner.resultContext;
    portEXIT_CRITICAL(&s_wifiMux);
    if (callback) { callback(&result, context); }
    wifiWake();
}

static void wifiRunRequest(const wifiExecution_t *execution)
{
    const sensorarrayWifiRequestResult_t *request = &execution->request;
    const sensorarrayWifiConfig_t *config = &execution->config;
    bool backendReady = execution->backendReady;
    esp_err_t error;
    if (backendReady) {
        error = wifiSameConfig(config, &execution->applied, false) ?
            sensorarrayWifiBackendApplyProfile(config->profile) : ESP_ERR_NOT_SUPPORTED;
    } else { error = sensorarrayWifiBackendInitialize(config); }
    uint64_t now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    if (error != ESP_OK && s_owner.originalError == ESP_OK) { s_owner.originalError = error; }
    bool quarantined = s_owner.quarantined;
    bool needsCleanup = error != ESP_OK && !backendReady && !quarantined;
    if (needsCleanup) {
        s_owner.jobCleanup = true;
        uint64_t totalDeadline = wifiAdd(request->acceptedAtUs,
            SENSORARRAY_WIFI_INIT_DEADLINE_US + SENSORARRAY_WIFI_CLEANUP_DEADLINE_US);
        s_owner.jobDeadlineUs = wifiAdd(now, SENSORARRAY_WIFI_CLEANUP_DEADLINE_US);
        if (totalDeadline < s_owner.jobDeadlineUs) { s_owner.jobDeadlineUs = totalDeadline; }
    }
    portEXIT_CRITICAL(&s_wifiMux);
    esp_err_t cleanup = ESP_OK;
    if (needsCleanup) {
        cleanup = sensorarrayWifiBackendHasReader() ? ESP_ERR_INVALID_STATE : sensorarrayWifiBackendCleanup();
    }
    now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    if (s_owner.cleanupError == ESP_OK) { s_owner.cleanupError = cleanup; }
    s_owner.stats.initError = error;
    if (cleanup != ESP_OK || (backendReady && error != ESP_OK && error != ESP_ERR_NOT_SUPPORTED)) {
        wifiQuarantineLocked(error != ESP_OK ? error : cleanup, now);
    }
    if (error == ESP_OK) {
        s_owner.backendReady = true;
        s_owner.appliedConfig = *config;
        s_owner.stats.profile = config->profile;
    }
    if (!s_owner.quarantined && s_owner.request.known && s_owner.request.id == request->id &&
        request->epoch == s_owner.epoch) {
        s_owner.outputEnabled = error == ESP_OK;
        wifiFinishRequestLocked(request->id, error == ESP_OK ? SENSORARRAY_WIFI_REQUEST_APPLIED :
            SENSORARRAY_WIFI_REQUEST_FAILED, error, cleanup, now);
    }
    s_owner.jobOwned = false;
    s_owner.jobCleanup = false;
    portEXIT_CRITICAL(&s_wifiMux);
    wifiWake();
}

bool sensorarrayWifiOwnerProcessOnce(void)
{
    if (xTaskGetCurrentTaskHandle() != s_ownerTask) { return false; }
    uint64_t now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    wifiObserveLocked(now);
    if (s_owner.resourceState != 2u || s_owner.quarantined || s_owner.jobOwned || s_owner.sdkActive) {
        portEXIT_CRITICAL(&s_wifiMux); return false;
    }
    int index = wifiTakeSlotLocked();
    s_execution.request.known = false;
    bool station = false;
    if (index >= 0) {
        s_owner.slots[index].state = WIFI_OWNED;
        s_execution.publication = s_owner.slots[index].publication;
    } else if (s_owner.request.known) {
        s_owner.request.outcome = SENSORARRAY_WIFI_REQUEST_RUNNING;
        s_execution.request = s_owner.request;
        s_execution.config = s_owner.requestedConfig;
        s_execution.applied = s_owner.appliedConfig;
        s_execution.backendReady = s_owner.backendReady;
    } else if (s_owner.reconcileStation && s_owner.backendReady) {
        station = true;
        s_owner.reconcileStation = false;
    } else { portEXIT_CRITICAL(&s_wifiMux); return false; }
    s_owner.jobOwned = true;
    s_owner.jobCleanup = false;
    s_owner.jobEpoch = s_owner.epoch;
    s_owner.jobLinkEpoch = index >= 0 ? s_execution.publication.expectedEpoch : 0u;
    s_owner.jobRequestId = s_execution.request.known ? s_execution.request.id : 0u;
    s_owner.jobDeadlineUs = s_execution.request.known ? wifiAdd(s_execution.request.acceptedAtUs, SENSORARRAY_WIFI_INIT_DEADLINE_US) :
        wifiAdd(now, SENSORARRAY_WIFI_SEND_DEADLINE_US);
    s_owner.originalError = s_owner.cleanupError = ESP_OK;
    uint32_t stationEpoch = s_owner.linkEpoch;
    portEXIT_CRITICAL(&s_wifiMux);
    if (index >= 0) { wifiRunDatagram((unsigned)index, &s_execution.publication); }
    else if (s_execution.request.known) { wifiRunRequest(&s_execution); }
    else if (station) {
        bool connected = false;
        esp_err_t err = sensorarrayWifiBackendStationQuery(&connected);
        portENTER_CRITICAL(&s_wifiMux);
        if (!s_owner.quarantined && stationEpoch == s_owner.linkEpoch) {
            s_owner.stationKnown = err == ESP_OK;
            s_owner.stats.stationConnected = err == ESP_OK && connected;
        }
        s_owner.jobOwned = false;
        portEXIT_CRITICAL(&s_wifiMux);
    }
    return true;
}

void sensorarrayWifiOwnerNoteStationEvent(bool connected)
{
    portENTER_CRITICAL(&s_wifiMux);
    s_owner.linkEpoch = wifiNext(s_owner.linkEpoch);
    s_owner.stationKnown = connected;
    s_owner.stats.stationConnected = connected;
    s_owner.reconcileStation = !connected;
    portEXIT_CRITICAL(&s_wifiMux);
    wifiNotifyOwner();
}

void sensorarrayWifiSetControlRxCallback(sensorarrayWifiControlRxCallback_t callback, void *context)
{
    portENTER_CRITICAL(&s_wifiMux);
    s_owner.control = callback; s_owner.controlContext = context;
    portEXIT_CRITICAL(&s_wifiMux);
}

void sensorarrayWifiSetOwnerCallbacks(sensorarrayWifiTxResultCallback_t result, void *context,
    sensorarrayWifiWakeCallback_t wake)
{
    portENTER_CRITICAL(&s_wifiMux);
    s_owner.result = result; s_owner.resultContext = context; s_owner.wake = wake;
    portEXIT_CRITICAL(&s_wifiMux);
}

bool sensorarrayWifiOwnerRxAllowed(void)
{
    portENTER_CRITICAL(&s_wifiMux);
    bool allowed = !s_owner.quarantined;
    portEXIT_CRITICAL(&s_wifiMux);
    return allowed;
}

void sensorarrayWifiOwnerReceive(const uint8_t *data, size_t length, const sensorarrayWifiPeer_t *peer)
{
    portENTER_CRITICAL(&s_wifiMux);
    s_owner.stats.ctrlRx++;
    sensorarrayWifiControlRxCallback_t callback = s_owner.quarantined ? NULL : s_owner.control;
    void *context = s_owner.controlContext;
    portEXIT_CRITICAL(&s_wifiMux);
    if (callback) { callback(data, length, peer, context); }
}

void sensorarrayWifiGetStats(sensorarrayWifiStats_t *out)
{
    if (!out) { return; }
    portENTER_CRITICAL(&s_wifiMux);
    *out = s_owner.stats;
    out->ready = s_owner.outputEnabled && s_owner.backendReady && !s_owner.quarantined;
    portEXIT_CRITICAL(&s_wifiMux);
}

bool sensorarrayWifiIsReady(void)
{
    sensorarrayWifiStats_t stats;
    sensorarrayWifiGetStats(&stats);
    return stats.ready;
}

void sensorarrayWifiCopyOwnerSnapshot(sensorarrayWifiOwnerSnapshot_t *out)
{
    if (!out) { return; }
    portENTER_CRITICAL(&s_wifiMux);
    uint64_t now = wifiNow();
    sensorarrayWifiRequestResult_t *latest = wifiFindResultLocked(0u);
    *out = (sensorarrayWifiOwnerSnapshot_t){
        .sessionId = s_owner.sessionId, .bootId = s_owner.bootId, .bootKnown = s_owner.bootKnown,
        .startAllowed = s_owner.startAllowed, .resourceAttempted = s_owner.resourceState != 0u,
        .resourceReady = s_owner.resourceState == 2u, .resourceError = s_owner.resourceError,
        .outputEnabled = s_owner.outputEnabled, .backendReady = s_owner.backendReady,
        .stationKnown = s_owner.stationKnown, .quarantined = s_owner.quarantined,
        .ownerBusy = s_owner.jobOwned, .epoch = s_owner.epoch, .linkEpoch = s_owner.linkEpoch,
        .requestId = latest ? latest->id : 0u, .requestedAp = latest && latest->requestedAp,
        .requestOutcome = latest ? latest->outcome : SENSORARRAY_WIFI_REQUEST_NONE,
        .sdkActive = s_owner.sdkActive, .phase = s_owner.phase, .callId = s_owner.callId,
        .startUs = s_owner.callStartUs, .deadlineUs = s_owner.callDeadlineUs, .detectedUs = s_owner.detectedUs,
        .ageUs = s_owner.callId && now >= s_owner.callStartUs ? now - s_owner.callStartUs : 0u,
        .completionKnown = s_owner.completionKnown, .originalError = s_owner.originalError,
        .cleanupError = s_owner.cleanupError, .sdkError = s_owner.sdkError,
        .sdkErrno = s_owner.sdkErrno, .returnedBytes = s_owner.returnedBytes,
        .timeouts = s_owner.timeouts, .lateReturns = s_owner.lateReturns,
        .terminalNoSink = s_owner.terminalNoSink, .terminalEvicted = s_owner.terminalEvicted,
        .stackMinimumBytes = s_owner.stackMinimumBytes, .stackKnown = s_owner.stackKnown,
    };
    for (unsigned i = 0; i < WIFI_SLOT_COUNT; ++i) {
        out->heldSlots += s_owner.slots[i].state != WIFI_FREE;
        out->writingSlots += s_owner.slots[i].state == WIFI_WRITING;
    }
    portEXIT_CRITICAL(&s_wifiMux);
}

bool sensorarrayWifiCopyRequestResult(uint32_t id, sensorarrayWifiRequestResult_t *out)
{
    if (!out) { return false; }
    portENTER_CRITICAL(&s_wifiMux);
    sensorarrayWifiRequestResult_t *result = wifiFindResultLocked(id);
    *out = result ? *result : (sensorarrayWifiRequestResult_t){0};
    portEXIT_CRITICAL(&s_wifiMux);
    return out->known;
}

bool sensorarrayWifiCopyPendingTerminal(sensorarrayWifiRequestResult_t *out)
{
    if (!out) { return false; }
    uint64_t now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    bool found = false;
    for (unsigned n = 0; n < 2u; ++n) {
        sensorarrayWifiRequestResult_t *r = &s_owner.terminal[(s_owner.terminalWrite + n) & 1u];
        if (r->known && r->terminalPending && r->terminalAttempts < WIFI_TERMINAL_ATTEMPTS && now >= r->terminalNextAttemptUs) {
            *out = *r; found = true; break;
        }
    }
    portEXIT_CRITICAL(&s_wifiMux);
    return found;
}

void sensorarrayWifiNoteTerminalAttempt(uint32_t id, bool queued)
{
    uint64_t now = wifiNow();
    portENTER_CRITICAL(&s_wifiMux);
    sensorarrayWifiRequestResult_t *r = wifiFindResultLocked(id);
    if (r && wifiTerminal(r->outcome) && r->terminalPending) {
        r->terminalAttempts++;
        r->terminalQueued |= queued;
        r->terminalPending = !queued && r->terminalAttempts < WIFI_TERMINAL_ATTEMPTS;
        r->terminalNextAttemptUs = wifiAdd(now, WIFI_TERMINAL_RETRY_US);
        if (!queued) { s_owner.terminalNoSink++; }
    }
    portEXIT_CRITICAL(&s_wifiMux);
}

bool sensorarrayWifiOwnerNeedsPoll(void)
{
    portENTER_CRITICAL(&s_wifiMux);
    bool needed = s_owner.request.known || (s_owner.sdkActive && !s_owner.quarantined);
    for (unsigned i = 0; i < 2u; ++i) { needed |= s_owner.terminal[i].terminalPending; }
    portEXIT_CRITICAL(&s_wifiMux);
    return needed;
}

const char *sensorarrayWifiRequestOutcomeName(sensorarrayWifiRequestOutcome_t outcome)
{
    static const char *names[] = {"none", "pending", "running", "applied", "failed", "superseded", "quarantined"};
    return (unsigned)outcome < sizeof(names) / sizeof(names[0]) ? names[outcome] : "unknown";
}

const char *sensorarrayWifiPhaseName(sensorarrayWifiPhase_t phase)
{
    static const char *names[] = {"idle", "netif_init", "event_loop", "netif_new", "netif_attach", "handlers",
        "driver_init", "event_register", "set_mode", "set_config", "start", "set_power", "set_ps", "set_protocol",
        "set_bandwidth", "socket", "setsockopt", "bind", "send", "station_query", "close", "event_unregister",
        "stop", "deinit", "clear_handlers", "netif_destroy", "rx_task_create"};
    return (unsigned)phase < sizeof(names) / sizeof(names[0]) ? names[phase] : "unknown";
}
