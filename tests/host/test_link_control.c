#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayUsbControl.h"
#include "sensorarrayLinkDiagnostics.h"
#include "sensorarrayRestartControl.h"
#include "sensorarrayUsbOtg.h"
#include "sensorarraySerialIo.h"
#include "sensorarrayBle.h"
#include "sensorarrayTransport.h"
#include "sensorarrayWorkBudget.h"

static bool otgEnabled, cdcEnabled, romAvailable, authorityKnown;
static sensorarrayUsbOtgStats_t usbStats;
static sensorarraySerialIoStats_t serialStats;
static sensorarrayBleLinkSnapshot_t link;
static sensorarrayWorkBudgetSnapshot_t budget;
static sensorarrayConfigAuthority_t authority;
static unsigned serialTerminals, networkTerminals, debugMirrors, publishCount;
static char lastTerminal[512], publishOrder[8];
static esp_err_t usbResult, networkResult, debugResult;
bool sensorarrayUsbOtgEnabled(void) { return otgEnabled; }
bool sensorarrayUsbOtgCdcEnabled(void) { return otgEnabled && cdcEnabled; }
bool sensorarrayUsbOtgRomDownloadAvailable(void) { return romAvailable; }
void sensorarrayUsbOtgGetStats(sensorarrayUsbOtgStats_t *out) { *out = usbStats; }
void sensorarraySerialIoGetStats(sensorarraySerialIoStats_t *out) { *out = serialStats; }
void sensorarrayBleGetLinkSnapshot(sensorarrayBleLinkSnapshot_t *out) { *out = link; }
void sensorarrayWorkBudgetCopySnapshot(sensorarrayWorkBudgetSnapshot_t *out) { *out = budget; }
bool sensorarrayConfigIdentityCopy(sensorarrayConfigAuthority_t *out)
{ *out = authority; return authorityKnown; }
esp_err_t sensorarraySerialIoQueue(const char *text, size_t length, bool priority)
{
    assert(length < sizeof(lastTerminal));
    if (priority) {
        memcpy(lastTerminal, text, length); lastTerminal[length] = '\0'; serialTerminals++;
        if (publishCount < sizeof(publishOrder)) { publishOrder[publishCount++] = 'U'; }
        return usbResult;
    }
    assert(otgEnabled && strlen(lastTerminal) == length && !memcmp(lastTerminal, text, length));
    debugMirrors++;
    if (publishCount < sizeof(publishOrder)) { publishOrder[publishCount++] = 'D'; }
    return debugResult;
}
esp_err_t sensorarrayTransportPublishLifecycleObserved(const char *text, size_t length, bool *queued)
{
    assert(text && length && strlen(lastTerminal) == length && !memcmp(lastTerminal, text, length));
    networkTerminals++; *queued = networkResult == ESP_OK;
    if (publishCount < sizeof(publishOrder)) { publishOrder[publishCount++] = 'N'; }
    return networkResult;
}

typedef esp_err_t (*formatter_t)(const char *, char *, size_t);
static size_t assertFormat(formatter_t format, const char *command, const char *required)
{
    struct { unsigned char left[16]; char text[512]; unsigned char right[16]; } guarded;
    memset(&guarded, 0xa5, sizeof(guarded));
    assert(format(command, guarded.text, sizeof(guarded.text)) == ESP_OK);
    size_t length = strlen(guarded.text);
    assert(length > 0u && length < 512u && guarded.text[length - 1u] == '\n');
    assert(strstr(guarded.text, required));
    for (unsigned i=0u; i<16u; i++) { assert(guarded.left[i] == 0xa5 && guarded.right[i] == 0xa5); }
    char exact[512];
    assert(format(command, exact, length + 1u) == ESP_OK && strcmp(exact, guarded.text) == 0);
    assert(format(command, exact, length) == ESP_ERR_INVALID_SIZE);
    assert(format(command, exact, 1u) == ESP_ERR_INVALID_SIZE && exact[0] == '\0');
    assert(format(command, exact, 0u) == ESP_ERR_INVALID_ARG);
    assert(format(NULL, exact, sizeof(exact)) == ESP_ERR_INVALID_ARG);
    assert(format(command, NULL, sizeof(exact)) == ESP_ERR_INVALID_ARG);
    printf("LINK_CONTROL_FORMAT,command=%s,bytes=%zu,capacity=512\n", command, length);
    return length;
}
static void reset(void)
{
    memset(&usbStats, 0, sizeof(usbStats)); memset(&serialStats, 0, sizeof(serialStats));
    memset(&link, 0, sizeof(link)); memset(&authority, 0, sizeof(authority));
    memset(&budget, 0, sizeof(budget));
    otgEnabled = romAvailable = authorityKnown = true;
    cdcEnabled = false;
    authority.identity.revision = 9u; authority.identity.appliedFrameBoundary = 100u;
    serialTerminals = networkTerminals = 0u; lastTerminal[0] = '\0';
    debugMirrors = publishCount = 0u; memset(publishOrder, 0, sizeof(publishOrder));
    usbResult = networkResult = debugResult = ESP_OK;
    sensorarrayRestartControlInit(0x123456789abcdef0ULL, 42u);
}
static void assertNotBusy(void)
{
    char query[512]; assert(sensorarrayRestartControlFormat(query, sizeof(query)) == ESP_OK);
    assert(strstr(query, "busy=0,"));
}
static uint32_t requestBoot(char *reply, size_t length)
{
    assert(sensorarrayUsbControlHandle("USBBOOT", reply, length) == ESP_OK);
    unsigned id = 0u;
    assert(sscanf(reply, "RACK,cmd=RESTART,id=%u,", &id) == 1 && id != 0u);
    assert(strstr(reply, "state=accepted,target=rom_download\n"));
    return id;
}
static void testUsbBoundaries(void)
{
    reset();
    usbStats.epoch = usbStats.mounts = usbStats.unmounts = usbStats.resets = UINT32_MAX;
    usbStats.rxDropped = usbStats.rxOversize = UINT32_MAX;
    usbStats.rxBytes = UINT64_MAX; usbStats.lastRxUs = INT64_MIN; usbStats.lastTxUs = INT64_MAX;
    usbStats.initialized = usbStats.mounted = usbStats.suspended = usbStats.dmaEnabled = usbStats.quiesced = true;
    usbStats.initError = ESP_FAIL;
    for (unsigned i=0u; i<3u; i++) { usbStats.accepted[i] = usbStats.callbackBytes[i] = UINT64_MAX; }
    serialStats.dataRejected = serialStats.priorityRejected = serialStats.dataSubmitted = UINT32_MAX;
    serialStats.prioritySubmitted = serialStats.normalExpired = serialStats.priorityExpired = UINT32_MAX;
    serialStats.partialWrite = serialStats.maxQueueAgeUs = serialStats.taskStackMinBytes = UINT32_MAX;
    serialStats.logRejected = serialStats.logSubmitted = serialStats.lineDropped = serialStats.lineOversize = UINT32_MAX;
    serialStats.discardBytes = UINT64_MAX;
    assertFormat(sensorarrayUsbControlHandle, "USB?", "backend=otg_bulk_v1");
    assertFormat(sensorarrayUsbControlHandle, "USBQ?", "callbackResult=unknown");
    assertFormat(sensorarrayUsbControlHandle, "SERIAL?", "scope=debug");
    assertFormat(sensorarrayUsbControlHandle, "USBSTREAM?", "control=bulk_ctrl");
    char reply[512]; assert(sensorarrayUsbControlHandle("SERIAL?", reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "rxCommands=unavailable,pcReceipt=unknown"));
    assert(sensorarrayUsbControlHandle("USBSTREAM=FULL", reply, sizeof(reply)) == ESP_ERR_NOT_SUPPORTED);
    assert(strstr(reply, "reason=legacy_usj_only,bulkData=always"));
    assert(sensorarrayUsbControlHandles("USBSTREAM=DEBUG") && sensorarrayUsbControlHandles("SERIAL?"));
    assert(strstr(sensorarrayUsbControlCapabilities(), "usb_vendor_bulk_v1"));
    assert(!sensorarrayUsbControlHandles(NULL) && !sensorarrayUsbControlHandles("BLELINK?"));
    cdcEnabled = true;
    assertFormat(sensorarrayUsbControlHandle, "USB?", "backend=otg_cdc_v1");
    assertFormat(sensorarrayUsbControlHandle, "USBQ?", "dCallbackBytes=unknown,cCallbackBytes=unknown");
    assertFormat(sensorarrayUsbControlHandle, "USBSTREAM?", "v=CDC,scope=otg_data,dataEvery=1,debug=cdc,control=shared_cdc");
    assert(sensorarrayUsbControlHandle("USBSTREAM=FULL", reply, sizeof(reply)) == ESP_ERR_NOT_SUPPORTED);
    assert(strstr(reply, "reason=legacy_usj_only,cdcData=always"));
    assert(strstr(sensorarrayUsbControlCapabilities(), "usb_cdc_data_v1"));
    assert(!strstr(sensorarrayUsbControlCapabilities(), "usb_vendor_bulk_v1"));
    otgEnabled = false;
    assertFormat(sensorarrayUsbControlHandle, "USB?", "backend=serial_jtag");
    assert(!sensorarrayUsbControlHandles("USBSTREAM?") && !sensorarrayUsbControlHandles("USBSTREAM=FULL"));
    assert(!sensorarrayUsbControlHandles("SERIAL?") && sensorarrayUsbControlHandles("USBBOOT"));
    assert(!strstr(sensorarrayUsbControlCapabilities(), "usb_vendor_bulk_v1"));
    assert(!strstr(sensorarrayUsbControlCapabilities(), "usb_cdc_data_v1"));
    assert(strstr(sensorarrayUsbControlCapabilities(), "serial_sink_v1"));
    assert(sensorarrayUsbControlHandle("SERIAL?", reply, sizeof(reply)) == ESP_ERR_NOT_SUPPORTED);
}
static void testBleBoundaries(void)
{
    reset();
    link.connectionGeneration = link.staleLinkEvents = UINT32_MAX;
    link.connected = link.phyKnown = link.connParamsKnown = link.dataLengthObserved = true;
    link.dataLengthConnectionKnown = false;
    link.mtu = link.connIntervalUnits = link.connLatency = link.supervisionTimeoutUnits = UINT16_MAX;
    link.txPhy = link.rxPhy = UINT8_MAX;
    link.phyRequestError = link.connRequestError = link.dataLengthRequestError = ESP_FAIL;
    link.phyUpdateStatus = link.connUpdateStatus = link.dataLengthStatus = UINT16_MAX;
    link.dataLengthTx = link.dataLengthRx = UINT16_MAX;
    link.dataDequeued = link.logDequeued = link.priorityDequeued = UINT32_MAX;
    for (unsigned i=0u; i<3u; i++) { link.queueMaxAgeUs[i] = UINT32_MAX; }
    link.sdkCallActive = true; link.sdkCallStartedUs = link.sdkCallCompletedUs = UINT64_MAX;
    link.sdkCallCount = link.sdkAccepted = link.sdkErrors = link.sdkMaxCallUs = UINT32_MAX;
    link.sdkLastError = ESP_FAIL; link.txTaskPriority = link.ctrlTaskPriority = UINT32_MAX;
    assertFormat(sensorarrayLinkDiagnosticsFormat, "BLELINK?", "dlePeerKnown=0");
    assertFormat(sensorarrayLinkDiagnosticsFormat, "BLETX?", "pcReceipt=unknown");
    link.sdkCompletedCalls = link.sdkIdleChanged = link.sdkIdleUnchanged = link.sdkIdleUnknown = UINT32_MAX;
    link.sdkWallUs = UINT64_MAX;
    link.confOk = link.confError = link.confMatched = link.confUnmatched = UINT32_MAX;
    assert(assertFormat(sensorarrayLinkDiagnosticsFormat, "BLESVC?",
        "confOk=4294967295,confError=4294967295,confMatched=4294967295,confUnmatched=4294967295,confMatch=waiter_predicate,confEpoch=unknown,pcReceipt=unknown\n") == 371u);
    budget.initError = ESP_FAIL; budget.registered = budget.idleKnown = true;
    budget.idleGeneration = UINT32_MAX;
    for (unsigned i = 0u; i < SENSORARRAY_WORK_BUDGET_WORKER_COUNT; ++i) {
        budget.workers[i].quantumDue = budget.workers[i].backstopDue = budget.workers[i].unitsDue = UINT32_MAX;
    }
    assertFormat(sensorarrayLinkDiagnosticsFormat, "SCHEDWHY?", "order=quantum/backstop/units,overlap=1");
    assert(sensorarrayLinkDiagnosticsHandles("BLELINK?") && sensorarrayLinkDiagnosticsHandles("BLETX?"));
    assert(sensorarrayLinkDiagnosticsHandles("BLESVC?") && sensorarrayLinkDiagnosticsHandles("SCHEDWHY?"));
    assert(!sensorarrayLinkDiagnosticsHandles(NULL) && !sensorarrayLinkDiagnosticsHandles("BLESVC=1"));
    assert(!sensorarrayLinkDiagnosticsHandles("SCHEDWHY=1") && !sensorarrayLinkDiagnosticsHandles("OTHER?"));
    memset(&link, 0, sizeof(link));
    link.phyRequestError = link.connRequestError = link.dataLengthRequestError = ESP_ERR_INVALID_STATE;
    link.phyUpdateStatus = link.connUpdateStatus = link.dataLengthStatus = UINT16_MAX;
    char reply[512]; assert(sensorarrayLinkDiagnosticsFormat("BLELINK?", reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "phyKnown=0,txPhy=0,rxPhy=0,connKnown=0"));
    assert(strstr(reply, "phyStatus=65535,connStatus=65535,dleStatus=65535,dleObserved=0,dlePeerKnown=0"));
    assert(sensorarrayLinkDiagnosticsFormat("BLESVC?", reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "active=0,started=0,completed=0,wallUs=0,idleChanged=0,idleSame=0,idleUnknown=0"));
    assert(strstr(reply, "confOk=0,confError=0,confMatched=0,confUnmatched=0,confMatch=waiter_predicate,confEpoch=unknown,pcReceipt=unknown\n"));
    memset(&budget, 0, sizeof(budget)); budget.initError = ESP_ERR_INVALID_STATE;
    assert(sensorarrayLinkDiagnosticsFormat("SCHEDWHY?", reply, sizeof(reply)) == ESP_OK);
    assert(strstr(reply, "init=0x103,registered=0,idleKnown=0,idleGen=0"));
    assert(sensorarrayLinkDiagnosticsFormat("OTHER?", reply, sizeof(reply)) == ESP_ERR_NOT_SUPPORTED);
}
static void testBootGuard(void)
{
    reset(); char reply[512]; uint32_t taken = 99u;
    romAvailable = false;
    assert(sensorarrayUsbControlHandle("USBBOOT", reply, sizeof(reply)) == ESP_ERR_NOT_SUPPORTED);
    assert(strstr(reply, "reason=rom_download_disabled")); assertNotBusy();
    romAvailable = true;
    for (size_t capacity=1u; capacity<sizeof("RACK,cmd=RESTART,id=4294967295,state=accepted,target=rom_download\n"); capacity++) {
        assert(sensorarrayUsbControlHandle("USBBOOT", reply, capacity) == ESP_ERR_INVALID_SIZE);
        assertNotBusy(); assert(!sensorarrayRestartControlTake(&taken));
    }
    uint32_t first = requestBoot(reply, sizeof(reply));
    assert(!sensorarrayRestartControlTake(&taken) && taken == 0u && !sensorarrayRestartControlIsRom(first));
    assert(!sensorarrayRestartControlAck(first+1u, ESP_OK));
    assert(sensorarrayUsbControlHandle("USBBOOT", reply, sizeof(reply)) == ESP_ERR_INVALID_STATE);
    assert(strstr(reply, "reason=restart_pending"));
    assert(sensorarrayRestartControlAck(first, ESP_ERR_TIMEOUT)); assertNotBusy();
    uint32_t second = requestBoot(reply, sizeof(reply)); assert(second != first);
    assert(!sensorarrayRestartControlAck(first, ESP_OK));
    assert(sensorarrayRestartControlAck(second, ESP_OK));
    assert(sensorarrayRestartControlTake(&taken) && taken == second && sensorarrayRestartControlIsRom(taken));
    assert(!sensorarrayRestartControlTake(NULL));
    assert(sensorarrayRestartControlPublish(taken, &authority, true) == ESP_OK);
    assert(serialTerminals == 1u && networkTerminals == 1u && strstr(lastTerminal, "target=rom_download"));
    assert(sensorarrayUsbControlHandle("USBBOOT", reply, sizeof(reply)) == ESP_ERR_INVALID_STATE);
    reset(); authorityKnown = false;
    first = requestBoot(reply, sizeof(reply));
    assert(sensorarrayRestartControlFormat(reply, sizeof(reply)) == ESP_OK && strstr(reply, "pendingKnown=0,pendingRev=0"));
    assert(sensorarrayRestartControlAck(first, ESP_FAIL)); assertNotBusy();
}
static void testCriticalText(void)
{
    reset(); debugResult = ESP_ERR_TIMEOUT;
    assert(sensorarrayTransportPublishRuntimeFault("MFAULT,seq=%lu,err=0x%X\n", 9ul, 0x108) == ESP_OK);
    assert(serialTerminals == 1u && networkTerminals == 1u && debugMirrors == 1u);
    assert(publishCount == 3u && !memcmp(publishOrder, "UND", 3u));
    assert(!strcmp(lastTerminal, "MFAULT,seq=9,err=0x108\n"));
    reset(); otgEnabled = false;
    assert(sensorarrayTransportPublishRuntimeFault("MFAULT,legacy=1\n") == ESP_OK);
    assert(serialTerminals == 1u && networkTerminals == 1u && debugMirrors == 0u);
    assert(publishCount == 2u && !memcmp(publishOrder, "UN", 2u));
    reset(); usbResult = ESP_ERR_TIMEOUT; networkResult = ESP_ERR_NO_MEM;
    assert(sensorarrayTransportPublishRuntimeFault("MFAULT,both_rejected=1\n") == ESP_ERR_TIMEOUT);
    assert(serialTerminals == 1u && networkTerminals == 1u && debugMirrors == 1u);
    reset(); networkResult = ESP_ERR_NO_MEM;
    assert(sensorarrayTransportPublishRuntimeFault("MFAULT,net_rejected=1\n") == ESP_ERR_NO_MEM);
    reset();
    char oversized[513]; memset(oversized, 'X', sizeof(oversized)); oversized[512] = '\0';
    assert(sensorarrayTransportPublishRuntimeFault("%s", oversized) == ESP_ERR_INVALID_SIZE);
    assert(sensorarrayTransportPublishRuntimeFault("") == ESP_ERR_INVALID_SIZE);
    assert(sensorarrayTransportPublishRuntimeFault(NULL) == ESP_ERR_INVALID_ARG);
    assert(serialTerminals == 0u && networkTerminals == 0u && debugMirrors == 0u);
    oversized[511] = '\0';
    assert(sensorarrayTransportPublishRuntimeFault("%s", oversized) == ESP_OK);
    assert(strlen(lastTerminal) == 511u && serialTerminals == 1u && networkTerminals == 1u && debugMirrors == 1u);
}
int main(void)
{
    testUsbBoundaries(); testBleBoundaries(); testBootGuard(); testCriticalText();
    puts("LINK_CONTROL_PASS,groups=4,actualModules=4,capacity=512,bootAckFence=1,backendCaps=1,criticalText=1,hardware=NOT_RUN");
    return 0;
}
