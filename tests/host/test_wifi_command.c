#include "sensorarrayTransportWifiControl.h"
#include "sensorarrayTransport.h"
#include "sensorarraySerialIo.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
static sensorarrayWifiOwnerSnapshot_t state;
static sensorarrayWifiStats_t stats;
static sensorarrayWifiRequestResult_t result;
static unsigned mutationCalls, serialCalls, networkCalls, terminalCalls;
static bool serialAccepted, networkAccepted;
static esp_err_t requestError;
static uint32_t exactQueried, notedId;
static bool notedQueued;
void sensorarrayWifiCopyOwnerSnapshot(sensorarrayWifiOwnerSnapshot_t *out) {*out=state;}
void sensorarrayWifiGetStats(sensorarrayWifiStats_t *out) {*out=stats;}
bool sensorarrayWifiCopyRequestResult(uint32_t id,sensorarrayWifiRequestResult_t *out) {exactQueried=id;*out=id==result.id?result:(sensorarrayWifiRequestResult_t){0};return out->known;}
bool sensorarrayWifiCopyPendingTerminal(sensorarrayWifiRequestResult_t *out) {*out=result;return result.known&&result.terminalPending;}
void sensorarrayWifiNoteTerminalAttempt(uint32_t id,bool queued) {terminalCalls++;notedId=id;notedQueued=queued;}
const char *sensorarrayWifiPhaseName(sensorarrayWifiPhase_t phase) {return "default_handlers";}
const char *sensorarrayWifiRequestOutcomeName(sensorarrayWifiRequestOutcome_t out) {static const char *n[]={"none","pending","running","applied","failed","superseded","quarantined"};return (unsigned)out<7?n[out]:"unknown";}
sensorarrayTransportWifiMode_t sensorarrayTransportGetWifiMode(void) {return state.requestedAp?SENSORARRAY_TRANSPORT_WIFI_AP:SENSORARRAY_TRANSPORT_WIFI_OFF;}
const char *sensorarrayTransportWifiModeName(sensorarrayTransportWifiMode_t m) {return m==SENSORARRAY_TRANSPORT_WIFI_AP?"AP":"OFF";}
esp_err_t sensorarrayTransportRequestWifiMode(sensorarrayTransportWifiMode_t m,uint32_t *id) {mutationCalls++;*id=requestError==ESP_OK?result.id:0;return requestError;}
esp_err_t sensorarraySerialIoQueue(const char *data,size_t length,bool priority) {assert(data&&length&&priority);serialCalls++;return serialAccepted?ESP_OK:ESP_ERR_NO_MEM;}
esp_err_t sensorarrayTransportPublishLifecycleObserved(const char *data,size_t length,bool *queued) {assert(data&&length);networkCalls++;*queued=networkAccepted;return ESP_OK;}
static size_t maximum;
static void query(const char *q)
{
    char buffer[513];unsigned mutations=mutationCalls;
    assert(sensorarrayTransportWifiCommandHandles(q));
    esp_err_t error=sensorarrayTransportWifiCommand(q,buffer,sizeof(buffer));
    if(error!=ESP_OK){fprintf(stderr,"query %s failed %x\n",q,error);}
    assert(error==ESP_OK);size_t length=strlen(buffer);if(length>maximum)maximum=length;assert(length<=512&&buffer[length-1]=='\n');
    assert(strstr(buffer,"peerReceipt=unknown")||!strcmp(q,"CAPS2?")||!strcmp(q,"WIFIGUARD?"));
    for(size_t size=1;size<=length;size++){char small[513];memset(small,0xa5,sizeof(small));assert(sensorarrayTransportWifiCommand(q,small,size)==ESP_ERR_INVALID_SIZE);assert(small[0]=='\0'&&(unsigned char)small[size]==0xa5);}
    assert(mutations==mutationCalls);
    printf("%s length=%zu\n",q,length);
}
int main(void)
{
    const char *queries[]={"CAPS2?","WIFI?","WIFIGUARD?","WIFIREQ?","WIFIREQ?4294967295","WIFIQ?D","WIFIQ?L","WIFIQ?C"};
    for(unsigned i=0;i<8;i++)query(queries[i]);
    char buffer[513];sensorarrayTransportWifiCommand("WIFIGUARD?",buffer,sizeof(buffer));assert(strstr(buffer,"startUs=unknown")&&strstr(buffer,"completionKnown=0"));
    sensorarrayTransportWifiCommand("WIFIQ?D",buffer,sizeof(buffer));assert(strstr(buffer,"stackKnown=0,stackMinBytes=unknown,stackScope=owner_after_job"));
    state=(sensorarrayWifiOwnerSnapshot_t){.sessionId=UINT64_MAX,.bootId=UINT32_MAX,.bootKnown=true,.startAllowed=true,
        .resourceAttempted=true,.resourceReady=true,.resourceError=INT_MIN,.outputEnabled=true,.backendReady=true,.stationKnown=true,
        .quarantined=true,.ownerBusy=true,.epoch=UINT32_MAX,.linkEpoch=UINT32_MAX,.requestId=UINT32_MAX,.requestedAp=true,
        .requestOutcome=SENSORARRAY_WIFI_REQUEST_QUARANTINED,.sdkActive=true,.phase=SENSORARRAY_WIFI_PHASE_DEFAULT_HANDLERS,
        .callId=UINT32_MAX,.startUs=UINT64_MAX,.deadlineUs=UINT64_MAX,.ageUs=UINT64_MAX,.detectedUs=UINT64_MAX,
        .completionKnown=true,.originalError=INT_MIN,.cleanupError=INT_MIN,.sdkError=INT_MIN,.sdkErrno=INT_MIN,.returnedBytes=INT_MIN,
        .timeouts=UINT32_MAX,.lateReturns=UINT32_MAX,.heldSlots=4,.writingSlots=4,.terminalNoSink=UINT32_MAX,.terminalEvicted=UINT32_MAX,.stackKnown=true,.stackMinimumBytes=UINT32_MAX};
    result=(sensorarrayWifiRequestResult_t){.known=true,.id=UINT32_MAX,.epoch=UINT32_MAX,.requestedAp=true,.outcome=SENSORARRAY_WIFI_REQUEST_QUARANTINED,
        .originalError=INT_MIN,.cleanupError=INT_MIN,.acceptedAtUs=UINT64_MAX,.completedAtUs=UINT64_MAX,.terminalAttempts=UINT8_MAX,.terminalQueued=true,.terminalPending=true};
    for(unsigned i=0;i<3;i++){stats.queued[i]=stats.admissionRejected[i]=stats.sent[i]=stats.dropped[i]=stats.expired[i]=stats.stale[i]=stats.partial[i]=UINT32_MAX;stats.driverAcceptedBytes[i]=UINT64_MAX;}
    for(unsigned i=0;i<8;i++)query(queries[i]);
    sensorarrayTransportWifiCommand("WIFIQ?L",buffer,sizeof(buffer));assert(strstr(buffer,"stackKnown=1,stackMinBytes=4294967295,stackScope=owner_after_job"));
    sensorarrayTransportWifiCommand("WIFIREQ?7",buffer,sizeof(buffer));assert(exactQueried==7&&strstr(buffer,"known=0")&&strstr(buffer,"id=7,"));
    const char *bad[]={"WIFIREQ?0","WIFIREQ?-1","WIFIREQ?4294967296","WIFIREQ?12x","WIFIQ?","WIFIQ?DD","WIFIQ?X"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++)assert(sensorarrayTransportWifiCommand(bad[i],buffer,sizeof(buffer))==ESP_ERR_INVALID_ARG);
    result.id=12;result.outcome=SENSORARRAY_WIFI_REQUEST_PENDING;result.terminalPending=false;requestError=ESP_OK;
    assert(sensorarrayTransportWifiCommand("WIFI=AP",buffer,sizeof(buffer))==ESP_OK);assert(strstr(buffer,"ok=0,")&&strstr(buffer,"accepted=1,pending=1")&&strstr(buffer,"id=12,"));
    result.outcome=SENSORARRAY_WIFI_REQUEST_APPLIED;
    assert(sensorarrayTransportWifiCommand("WIFI=AP",buffer,sizeof(buffer))==ESP_OK&&strstr(buffer,"ok=1,")&&strstr(buffer,"pending=0,"));
    result.requestedAp=false;
    assert(sensorarrayTransportWifiCommand("WIFI=OFF",buffer,sizeof(buffer))==ESP_OK&&strstr(buffer,"scope=output_gate_only"));
    requestError=ESP_ERR_NO_MEM;assert(sensorarrayTransportWifiCommand("WIFI=AP",buffer,sizeof(buffer))==ESP_ERR_NO_MEM&&strstr(buffer,"accepted=0")&&strstr(buffer,"id=0,"));
    result.terminalPending=true;serialAccepted=networkAccepted=false;sensorarrayTransportWifiDrainTerminal();assert(!notedQueued&&notedId==12&&terminalCalls==1);
    networkAccepted=true;sensorarrayTransportWifiDrainTerminal();assert(notedQueued&&terminalCalls==2);networkAccepted=false;serialAccepted=true;sensorarrayTransportWifiDrainTerminal();assert(notedQueued&&terminalCalls==3&&serialCalls==3&&networkCalls==3);
    assert(!sensorarrayTransportWifiCommandHandles("WIFIX?"));printf("wifi_command actual formatter max=%zu <=512; truncation/unknown/exact-ID/admission/terminal-noop PASS\n",maximum);return 0;
}
