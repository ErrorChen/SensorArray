#include "sensorarrayTransportWifiControl.h"
#include "sensorarrayTransport.h"
#include "sensorarraySerialIo.h"
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Explicitly bounded secondary capability page; PROTO only advertises caps2_v1. */
static esp_err_t wifiFormat(char *out, size_t size, const char *format, ...)
{
    if (!out || size == 0u) { return ESP_ERR_INVALID_ARG; }
    va_list args;
    va_start(args, format);
    int length = vsnprintf(out, size, format, args);
    va_end(args);
    if (length < 0 || (size_t)length >= size || (size_t)length > SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX) {
        out[0] = '\0';
        return ESP_ERR_INVALID_SIZE;
    }
    return ESP_OK;
}

static const char *wifiBackendName(const sensorarrayWifiOwnerSnapshot_t *s)
{
    return s->quarantined ? "quarantined" : s->backendReady ? "ready" :
        s->ownerBusy ? "initializing" : s->resourceAttempted && !s->resourceReady ? "resource_failed" : "inactive";
}

static bool wifiPending(sensorarrayWifiRequestOutcome_t state)
{
    return state == SENSORARRAY_WIFI_REQUEST_PENDING || state == SENSORARRAY_WIFI_REQUEST_RUNNING;
}

static void wifiU64(char *out, size_t size, bool known, uint64_t value)
{
    if (known) { (void)snprintf(out, size, "%" PRIu64, value); }
    else { (void)snprintf(out, size, "unknown"); }
}

static esp_err_t wifiFormatRequest(const char *prefix, const sensorarrayWifiOwnerSnapshot_t *s,
    uint32_t id, const sensorarrayWifiRequestResult_t *r, char *response, size_t size)
{
    if (!r->known) {
        return wifiFormat(response, size,
            "%s,v=1,session=%016" PRIx64 ",boot=%" PRIu32 ",bootKnown=%u,id=%" PRIu32
            ",known=0,outcome=unknown,reason=not_cached,scope=boot,peerReceipt=unknown\n",
            prefix, s->sessionId, s->bootId, s->bootKnown, id);
    }
    char completed[21];
    wifiU64(completed, sizeof(completed), !wifiPending(r->outcome), r->completedAtUs);
    return wifiFormat(response, size,
        "%s,v=1,session=%016" PRIx64 ",boot=%" PRIu32 ",bootKnown=%u,id=%" PRIu32
        ",known=1,epoch=%" PRIu32 ",requested=%s,outcome=%s,err=0x%08" PRIx32
        ",cleanupErr=0x%08" PRIx32 ",acceptedUs=%" PRIu64 ",completedUs=%s,terminalAttempts=%u"
        ",terminalQueued=%u,terminalPending=%u,scope=%s,peerReceipt=unknown\n",
        prefix, s->sessionId, s->bootId, s->bootKnown, r->id, r->epoch,
        r->requestedAp ? "AP" : "OFF", sensorarrayWifiRequestOutcomeName(r->outcome),
        (uint32_t)r->originalError, (uint32_t)r->cleanupError, r->acceptedAtUs, completed,
        (unsigned)r->terminalAttempts, r->terminalQueued, r->terminalPending,
        r->requestedAp ? "backend_and_gate" : "output_gate_only");
}

bool sensorarrayTransportWifiCommandHandles(const char *text)
{
    return text && (strcmp(text, "CAPS2?") == 0 || strcmp(text, "WIFI?") == 0 ||
        strcmp(text, "WIFI=AP") == 0 || strcmp(text, "WIFI=OFF") == 0 ||
        strcmp(text, "WIFIGUARD?") == 0 || strncmp(text, "WIFIREQ?", 8u) == 0 ||
        strncmp(text, "WIFIQ?", 6u) == 0);
}

esp_err_t sensorarrayTransportWifiCommand(const char *text, char *response, size_t size)
{
    if (!sensorarrayTransportWifiCommandHandles(text) || !response || size == 0u) { return ESP_ERR_INVALID_ARG; }
    if (strcmp(text, "CAPS2?") == 0) {
        return wifiFormat(response, size, "CAPS2,v=1,page=1,caps=wifi_owner_v1,next=none\n");
    }
    sensorarrayWifiOwnerSnapshot_t s;
    sensorarrayWifiCopyOwnerSnapshot(&s);
    if (strcmp(text, "WIFI?") == 0) {
        return wifiFormat(response, size,
            "ACK,cmd=WIFI,v=%s,ok=%u,ver=2,session=%016" PRIx64 ",boot=%" PRIu32
            ",bootKnown=%u,requested=%s,outputGate=%u,backend=%s,startAllowed=%u"
            ",resources=%u,initErr=0x%08" PRIx32 ",quarantined=%u,id=%" PRIu32
            ",epoch=%" PRIu32 ",linkEpoch=%" PRIu32 ",outcome=%s,pending=%u,physicalOff=unknown,peerReceipt=unknown\n",
            s.requestedAp ? "AP" : "OFF",
            s.requestId && s.requestOutcome == SENSORARRAY_WIFI_REQUEST_APPLIED &&
                (!s.requestedAp || (s.outputEnabled && s.backendReady && !s.quarantined)),
            s.sessionId, s.bootId, s.bootKnown, s.requestedAp ? "AP" : "OFF", s.outputEnabled,
            wifiBackendName(&s), s.startAllowed, s.resourceReady, (uint32_t)s.resourceError,
            s.quarantined, s.requestId, s.epoch, s.linkEpoch,
            sensorarrayWifiRequestOutcomeName(s.requestOutcome), wifiPending(s.requestOutcome));
    }
    if (strcmp(text, "WIFIGUARD?") == 0) {
        char start[21], deadline[21], age[21], detected[21];
        wifiU64(start, sizeof(start), s.callId != 0u, s.startUs);
        wifiU64(deadline, sizeof(deadline), s.callId != 0u, s.deadlineUs);
        wifiU64(age, sizeof(age), s.callId != 0u, s.ageUs);
        wifiU64(detected, sizeof(detected), s.quarantined, s.detectedUs);
        return wifiFormat(response, size,
            "WIFIGUARD,v=1,session=%016" PRIx64 ",boot=%" PRIu32 ",bootKnown=%u,quarantined=%u,active=%u"
            ",ownerBusy=%u,phase=%s,callId=%" PRIu32 ",startUs=%s,deadlineUs=%s,ageUs=%s,detectedUs=%s"
            ",completionKnown=%u,err=0x%08" PRIx32 ",cleanupErr=0x%08" PRIx32
            ",sdkErr=0x%08" PRIx32 ",errno=%d,returned=%d,held=%" PRIu32 ",writing=%" PRIu32
            ",timeouts=%" PRIu32 ",late=%" PRIu32 ",pollUs=100000,recovery=explicit_manual_restart_attempt\n",
            s.sessionId, s.bootId, s.bootKnown, s.quarantined, s.sdkActive, s.ownerBusy,
            sensorarrayWifiPhaseName(s.phase), s.callId, start, deadline, age, detected, s.completionKnown,
            (uint32_t)s.originalError, (uint32_t)s.cleanupError, (uint32_t)s.sdkError,
            s.sdkErrno, s.returnedBytes, s.heldSlots, s.writingSlots, s.timeouts, s.lateReturns);
    }
    if (strncmp(text, "WIFIREQ?", 8u) == 0) {
        uint32_t id = 0u;
        if (text[8]) {
            for (const char *p = text + 8u; *p; ++p) {
                if (*p < '0' || *p > '9' || id > (UINT32_MAX - (unsigned)(*p - '0')) / 10u) {
                    return wifiFormat(response, size, "ERR,cmd=WIFIREQ,reason=invalid_id\n") == ESP_OK ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_SIZE;
                }
                id = id * 10u + (unsigned)(*p - '0');
            }
            if (id == 0u) { return wifiFormat(response, size, "ERR,cmd=WIFIREQ,reason=invalid_id\n") == ESP_OK ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_SIZE; }
        } else { id = s.requestId; }
        sensorarrayWifiRequestResult_t result = {0};
        if (id) { (void)sensorarrayWifiCopyRequestResult(id, &result); }
        return wifiFormatRequest("WIFIREQ", &s, id, &result, response, size);
    }
    if (strncmp(text, "WIFIQ?", 6u) == 0) {
        if (!text[6] || text[7] || (text[6] != 'D' && text[6] != 'L' && text[6] != 'C')) {
            return wifiFormat(response, size, "ERR,cmd=WIFIQ,reason=channel_D_L_C_required\n") == ESP_OK ? ESP_ERR_INVALID_ARG : ESP_ERR_INVALID_SIZE;
        }
        unsigned channel = text[6] == 'D' ? SENSORARRAY_WIFI_CH_DATA : text[6] == 'L' ? SENSORARRAY_WIFI_CH_LOG : SENSORARRAY_WIFI_CH_CTRL;
        sensorarrayWifiStats_t stats;
        sensorarrayWifiGetStats(&stats);
        char stackBytes[21];
        /* Cached by the owner only after a real job returned. A stuck first
         * SDK call has no invented stack sample; queries never scan tasks. */
        wifiU64(stackBytes, sizeof(stackBytes), s.stackKnown, s.stackMinimumBytes);
        return wifiFormat(response, size,
            "WIFIQ,v=1,session=%016" PRIx64 ",boot=%" PRIu32 ",channel=%c,scope=boot,queued=%" PRIu32
            ",rejected=%" PRIu32 ",sent=%" PRIu32 ",dropped=%" PRIu32 ",expired=%" PRIu32
            ",stale=%" PRIu32 ",partial=%" PRIu32 ",driverAcceptedBytes=%" PRIu64
            ",terminalNoSink=%" PRIu32 ",terminalEvicted=%" PRIu32
            ",stackKnown=%u,stackMinBytes=%s,stackScope=owner_after_job,sentScope=full_sendto_return,peerReceipt=unknown\n",
            s.sessionId, s.bootId, text[6], stats.queued[channel], stats.admissionRejected[channel],
            stats.sent[channel], stats.dropped[channel], stats.expired[channel], stats.stale[channel],
            stats.partial[channel], stats.driverAcceptedBytes[channel], s.terminalNoSink, s.terminalEvicted,
            s.stackKnown, stackBytes);
    }
    bool ap = strcmp(text, "WIFI=AP") == 0;
    uint32_t id = 0u;
    esp_err_t error = sensorarrayTransportRequestWifiMode(ap ? SENSORARRAY_TRANSPORT_WIFI_AP : SENSORARRAY_TRANSPORT_WIFI_OFF, &id);
    sensorarrayWifiCopyOwnerSnapshot(&s);
    sensorarrayWifiRequestResult_t result = {0};
    if (id) { (void)sensorarrayWifiCopyRequestResult(id, &result); }
    esp_err_t formatted = wifiFormat(response, size,
        "%s,cmd=WIFI,v=%s,ok=%u,ver=2,accepted=%u,pending=%u,session=%016" PRIx64
        ",boot=%" PRIu32 ",bootKnown=%u,id=%" PRIu32 ",epoch=%" PRIu32
        ",outcome=%s,e=0x%08" PRIx32 ",scope=%s,peerReceipt=unknown\n",
        error == ESP_OK ? "ACK" : "ERR", ap ? "AP" : "OFF",
        result.known && result.outcome == SENSORARRAY_WIFI_REQUEST_APPLIED,
        error == ESP_OK, result.known && wifiPending(result.outcome), s.sessionId, s.bootId, s.bootKnown,
        id, result.known ? result.epoch : s.epoch,
        result.known ? sensorarrayWifiRequestOutcomeName(result.outcome) : "unknown", (uint32_t)error,
        ap ? "backend_and_gate" : "output_gate_only");
    return formatted != ESP_OK ? formatted : error;
}

void sensorarrayTransportWifiDrainTerminal(void)
{
    sensorarrayWifiRequestResult_t result;
    if (!sensorarrayWifiCopyPendingTerminal(&result)) { return; }
    sensorarrayWifiOwnerSnapshot_t snapshot;
    sensorarrayWifiCopyOwnerSnapshot(&snapshot);
    char text[SENSORARRAY_TRANSPORT_CTRL_TEXT_MAX];
    bool networkQueued = false;
    bool serialQueued = false;
    if (wifiFormatRequest("WTERM", &snapshot, result.id, &result, text, sizeof(text)) == ESP_OK) {
        size_t length = strlen(text);
        serialQueued = sensorarraySerialIoQueue(text, length, true) == ESP_OK;
        (void)sensorarrayTransportPublishLifecycleObserved(text, length, &networkQueued);
    }
    /* ESP_OK with no enabled network sink is a documented no-op, not a queued
     * terminal. Bounded attempts + the two-result cache remain queryable. */
    sensorarrayWifiNoteTerminalAttempt(result.id, serialQueued || networkQueued);
}
