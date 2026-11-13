#include "sensorarrayRestartControl.h"

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "sensorarrayTransport.h"
#include "sensorarraySerialIo.h"

typedef enum {
    SENSORARRAY_RESTART_NONE = 0,
    SENSORARRAY_RESTART_PENDING,
    SENSORARRAY_RESTART_RUNNING,
    SENSORARRAY_RESTART_REJECTED,
    SENSORARRAY_RESTART_RESTARTING,
} sensorarrayRestartPhase_t;

typedef struct {
    uint32_t id;
    uint32_t acceptedRevision;
    bool acceptedKnown;
    bool romDownload;
} sensorarrayRestartRequest_t;

typedef struct {
    sensorarrayRestartRequest_t request;
    sensorarrayRestartPhase_t outcome;
    uint32_t resultRevision;
    uint32_t frameBoundary;
    bool resultKnown;
    bool terminalQueued;
    bool serialQueued;
    bool networkQueued;
    esp_err_t serialError;
    esp_err_t networkError;
    esp_err_t originalError;
    esp_err_t publishError;
} sensorarrayRestartResult_t;

static portMUX_TYPE s_restartMux = portMUX_INITIALIZER_UNLOCKED;
static struct {
    uint64_t sessionId;
    uint32_t bootId;
    uint32_t nextId;
    bool busy;
    bool acknowledged;
    sensorarrayRestartPhase_t phase;
    sensorarrayRestartRequest_t current;
    sensorarrayRestartResult_t last;
} s_restart;

static const char *sensorarrayRestartPhaseName(sensorarrayRestartPhase_t phase)
{
    switch (phase) {
    case SENSORARRAY_RESTART_PENDING: return "pending";
    case SENSORARRAY_RESTART_RUNNING: return "running";
    case SENSORARRAY_RESTART_REJECTED: return "rejected";
    case SENSORARRAY_RESTART_RESTARTING: return "restarting";
    default: return "none";
    }
}

void sensorarrayRestartControlInit(uint64_t sessionId, uint32_t bootId)
{
    portENTER_CRITICAL(&s_restartMux);
    memset(&s_restart, 0, sizeof(s_restart));
    s_restart.sessionId = sessionId;
    s_restart.bootId = bootId;
    portEXIT_CRITICAL(&s_restartMux);
}

static esp_err_t sensorarrayRestartControlPostTarget(
    const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId, bool romDownload)
{
    if (known && !authority) { return ESP_ERR_INVALID_ARG; }
    portENTER_CRITICAL(&s_restartMux);
    if (s_restart.busy) {
        portEXIT_CRITICAL(&s_restartMux);
        return ESP_ERR_INVALID_STATE;
    }
    s_restart.nextId++;
    if (s_restart.nextId == 0u) { s_restart.nextId = 1u; }
    s_restart.current = (sensorarrayRestartRequest_t){
        .id = s_restart.nextId, .acceptedKnown = known, .romDownload = romDownload,
        .acceptedRevision = known ? authority->identity.revision : 0u,
    };
    s_restart.busy = true;
    s_restart.acknowledged = false;
    s_restart.phase = SENSORARRAY_RESTART_PENDING;
    if (outId) { *outId = s_restart.current.id; }
    portEXIT_CRITICAL(&s_restartMux);
    return ESP_OK;
}

esp_err_t sensorarrayRestartControlPost(
    const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId)
{ return sensorarrayRestartControlPostTarget(authority, known, outId, false); }

esp_err_t sensorarrayRestartControlPostRom(
    const sensorarrayConfigAuthority_t *authority, bool known, uint32_t *outId)
{ return sensorarrayRestartControlPostTarget(authority, known, outId, true); }

bool sensorarrayRestartControlIsRom(uint32_t id)
{
    portENTER_CRITICAL(&s_restartMux);
    bool isRom = s_restart.busy && s_restart.current.id == id &&
        s_restart.acknowledged && s_restart.phase == SENSORARRAY_RESTART_RUNNING &&
        s_restart.current.romDownload;
    portEXIT_CRITICAL(&s_restartMux);
    return isRom;
}

bool sensorarrayRestartControlAck(uint32_t id, esp_err_t publishError)
{
    portENTER_CRITICAL(&s_restartMux);
    bool matches = s_restart.busy && s_restart.current.id == id &&
        s_restart.phase == SENSORARRAY_RESTART_PENDING && !s_restart.acknowledged;
    if (matches && publishError == ESP_OK) {
        s_restart.acknowledged = true;
    } else if (matches) {
        s_restart.last = (sensorarrayRestartResult_t){
            .request = s_restart.current, .outcome = SENSORARRAY_RESTART_REJECTED,
            .originalError = publishError,
        };
        s_restart.busy = false;
        s_restart.phase = SENSORARRAY_RESTART_NONE;
    }
    portEXIT_CRITICAL(&s_restartMux);
    return matches;
}

bool sensorarrayRestartControlTake(uint32_t *outId)
{
    portENTER_CRITICAL(&s_restartMux);
    bool ready = s_restart.busy && s_restart.acknowledged &&
        s_restart.phase == SENSORARRAY_RESTART_PENDING;
    if (ready) { s_restart.phase = SENSORARRAY_RESTART_RUNNING; }
    if (outId) { *outId = ready ? s_restart.current.id : 0u; }
    portEXIT_CRITICAL(&s_restartMux);
    return ready;
}

esp_err_t sensorarrayRestartControlPublish(uint32_t id,
    const sensorarrayConfigAuthority_t *authority, bool known)
{
    if (known && !authority) { return ESP_ERR_INVALID_ARG; }
    portENTER_CRITICAL(&s_restartMux);
    if (!s_restart.busy || s_restart.current.id != id ||
        s_restart.phase != SENSORARRAY_RESTART_RUNNING) {
        portEXIT_CRITICAL(&s_restartMux);
        return ESP_ERR_INVALID_STATE;
    }
    s_restart.last = (sensorarrayRestartResult_t){
        .request = s_restart.current, .outcome = SENSORARRAY_RESTART_RESTARTING,
        .resultKnown = known, .resultRevision = known ? authority->identity.revision : 0u,
        .frameBoundary = known ? authority->identity.appliedFrameBoundary : 0u,
    };
    s_restart.phase = SENSORARRAY_RESTART_RESTARTING;
    /* Keep busy until reset: a late duplicate cannot trigger a second reset.
     * Cache intent before delivery; esp_restart does not establish PC receipt. */
    sensorarrayRestartResult_t result = s_restart.last;
    uint64_t sessionId = s_restart.sessionId;
    uint32_t bootId = s_restart.bootId;
    portEXIT_CRITICAL(&s_restartMux);
    char terminal[384];
    int n = snprintf(terminal, sizeof(terminal),
        "RAPP,cmd=RESTART,id=%lu,state=restarting,kind=manual,target=%s,session=%016llX,bootId=%lu,acceptedKnown=%u,acceptedRev=%lu,resultKnown=%u,rev=%lu,boundary=%lu\n",
        (unsigned long)id, result.request.romDownload ? "rom_download" : "application",
        (unsigned long long)sessionId, (unsigned long)bootId,
        result.request.acceptedKnown ? 1u : 0u, (unsigned long)result.request.acceptedRevision,
        known ? 1u : 0u, (unsigned long)result.resultRevision, (unsigned long)result.frameBoundary);
    esp_err_t serialError = ESP_ERR_INVALID_SIZE;
    esp_err_t networkError = ESP_ERR_INVALID_SIZE;
    bool networkQueued = false;
    if (n > 0 && (size_t)n < sizeof(terminal)) {
        /* The lifecycle transport owns BLE/Wi-Fi only. Always explicitly
         * admit the complete terminal to the sole Serial TX owner's priority
         * queue, including Serial-only sessions. Neither result is PC receipt. */
        serialError = sensorarraySerialIoQueue(terminal, (size_t)n, true);
        networkError = sensorarrayTransportPublishLifecycleObserved(
            terminal, (size_t)n, &networkQueued);
    }
    esp_err_t err = serialError != ESP_OK ? serialError : networkError;
    portENTER_CRITICAL(&s_restartMux);
    if (s_restart.last.request.id == id) {
        s_restart.last.serialQueued = serialError == ESP_OK;
        s_restart.last.networkQueued = networkQueued;
        s_restart.last.serialError = serialError;
        s_restart.last.networkError = networkError;
        s_restart.last.terminalQueued = serialError == ESP_OK || networkQueued;
        s_restart.last.publishError = err;
    }
    portEXIT_CRITICAL(&s_restartMux);
    return err;
}

esp_err_t sensorarrayRestartControlFormat(char *response, size_t responseSize)
{
    if (!response || responseSize == 0u) { return ESP_ERR_INVALID_ARG; }
    portENTER_CRITICAL(&s_restartMux);
    sensorarrayRestartRequest_t current = s_restart.current;
    sensorarrayRestartResult_t last = s_restart.last;
    sensorarrayRestartPhase_t phase = s_restart.phase;
    uint64_t sessionId = s_restart.sessionId;
    uint32_t bootId = s_restart.bootId;
    bool busy = s_restart.busy;
    portEXIT_CRITICAL(&s_restartMux);
    int n = snprintf(response, responseSize,
        "RESTART,session=%016llX,bootId=%lu,busy=%u,phase=%s,pendingId=%lu,pendingKnown=%u,pendingRev=%lu,lastId=%lu,last=%s,reason=%s,acceptedKnown=%u,acceptedRev=%lu,resultKnown=%u,rev=%lu,boundary=%lu,originalError=0x%lX,publishError=0x%lX,terminalQueued=%u,serialQueued=%u,serialError=0x%lX,netQueued=%u,netError=0x%lX,target=%s\n",
        (unsigned long long)sessionId, (unsigned long)bootId, busy ? 1u : 0u,
        sensorarrayRestartPhaseName(phase), (unsigned long)(busy ? current.id : 0u),
        busy && current.acceptedKnown ? 1u : 0u, (unsigned long)(busy ? current.acceptedRevision : 0u),
        (unsigned long)last.request.id, sensorarrayRestartPhaseName(last.outcome),
        last.outcome == SENSORARRAY_RESTART_REJECTED ? "ack_publish_failed" :
            (last.outcome == SENSORARRAY_RESTART_RESTARTING ? "manual_restart" : "none"),
        last.request.acceptedKnown ? 1u : 0u, (unsigned long)last.request.acceptedRevision,
        last.resultKnown ? 1u : 0u, (unsigned long)last.resultRevision, (unsigned long)last.frameBoundary,
        (unsigned long)(uint32_t)last.originalError, (unsigned long)(uint32_t)last.publishError,
        last.terminalQueued ? 1u : 0u, last.serialQueued ? 1u : 0u,
        (unsigned long)(uint32_t)last.serialError, last.networkQueued ? 1u : 0u,
        (unsigned long)(uint32_t)last.networkError,
        last.request.romDownload ? "rom_download" : "application");
    if (n <= 0 || (size_t)n >= responseSize) { response[0] = '\0'; return ESP_ERR_INVALID_SIZE; }
    return ESP_OK;
}
