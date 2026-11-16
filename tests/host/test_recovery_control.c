#define sensorarrayTransportPublishRuntimeFault printf /* Transport is synthetic in this main-logic harness. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include "sensorarrayRecoveryControl.h"
#include "sensorarrayRestartControl.h"
#include "sensorarraySerialIo.h"
#include "sensorarrayWorkBudget.h"
#include "sensorarrayTransportGuaranteedText.h"
#include "sensorarrayTransportPool.h"
/* PRODUCTION_MODULES */

#define SENSORARRAY_RECOVERY_LEVEL_NONE 255u
#define SENSORARRAY_RECOVERY_LEVEL_SOFT 0u
#define SENSORARRAY_RECOVERY_LEVEL_FULL 1u
#define SENSORARRAY_RECOVERY_LEVEL_RESTART 2u
#define SENSORARRAY_BOOT_RESTART_KIND_AUTO 1u
#define SENSORARRAY_BOOT_RESTART_KIND_MANUAL 2u
#define pdMS_TO_TICKS(ms) (ms)

typedef struct { bool safe, applied, known, verified, fdcSdHigh; uint32_t cleanupError, lastError; } sensorarrayRouteSnapshot_t;
typedef struct { unsigned mode; } Ads;
typedef struct {
    sensorarrayMeasurementModeContext_t measurementMode;
    sensorarrayRowModeProfile_t rowModeProfile;
    sensorarrayMeasurementRecovery_t adsRestoreRecovery;
    struct { Ads ads; int fdcPrimary, fdcSecondary; } state;
    sensorarrayRouteSnapshot_t routeController;
    Ads adsEngine;
    int fdcEngine, fdcRescue;
    struct { bool freshFrame; uint32_t sequence; } frame;
    uint32_t hostFrameSequence;
} sensorarrayAppContext_t;
typedef struct { uint32_t bootId; } sensorarrayBootBreadcrumb_t;
static uint64_t s_configSessionId = 0x123456789abcdef0ULL;
static uint32_t bootId = 7;
static sensorarrayFrameConfigSnapshot_t rows = {.rows=1,.cells=8,.rowMask=1,.generation=31,.requestId=19};
static bool autoRestartAllowed, held, quiescent = true, quarantined;
static unsigned safeCalls, modeStopCalls, restarts, modeApplyCalls, invalidFrames, ackCalls;
static esp_err_t safeError, stopError, applyError, rescueError;
static uint32_t applyCleanupError;
static sensorarraySerialIoStats_t serialStats;
static unsigned serialSnapshotCalls, schedulerSnapshotCalls, lifecycleCalls, serialQueueCalls;
static esp_err_t serialQueueError;
static char lifecycleText[384], serialText[384];
static bool netBleEnabled, netWifiEnabled, networkQueueAccept;
static unsigned bleTerminalCalls, wifiTerminalCalls, serialLogAccepted, serialLogRejected;
static sensorarrayTransportPayloadSlot_t transportSlot;
typedef unsigned QueueHandle_t;
typedef unsigned sensorarrayBleChannel_t;
#define SENSORARRAY_CFG_OUTPUT_BLE_CAP_TEXT 1
#define SENSORARRAY_BLE_CH_DATA 0u
#define SENSORARRAY_BLE_CH_LOG 1u
#define SENSORARRAY_WIFI_CH_LOG 1u
#define pdTRUE 1
static QueueHandle_t s_dataQueue=1,s_logQueue=2,s_lifecycleQueue=3;
static unsigned s_task=1;
static portMUX_TYPE g_sensorarrayTransportStatsMux=portMUX_INITIALIZER_UNLOCKED;
static struct {uint32_t queueDrop,queueDropData,queueDropLog,queueDropLifecycle,lifecycleDropped,lifecyclePublished;} g_sensorarrayTransportStats;
static bool sensorarrayTransportBleChannelEnabled(sensorarrayTransportChannel_t ch) {(void)ch;return netBleEnabled;}
static bool sensorarrayBleGetSendGeneration(unsigned ch,uint32_t *gen) {(void)ch;*gen=42;return netBleEnabled;}
static bool sensorarrayTransportWifiSinkEnabled(void) {return netWifiEnabled;}
static bool sensorarrayWifiIsReady(void) {return netWifiEnabled;}
static bool sensorarrayTransportAllocateDescriptor(sensorarrayTransportChannel_t ch,size_t length,sensorarrayTransportDescriptor_t *d) {
    *d=(sensorarrayTransportDescriptor_t){.channel=ch,.length=length};
    transportSlot=(sensorarrayTransportPayloadSlot_t){.channel=ch,.length=length};return true;
}
static sensorarrayTransportPayloadSlot_t *sensorarrayTransportDescriptorSlot(const sensorarrayTransportDescriptor_t *d) {(void)d;return &transportSlot;}
static bool sensorarrayTransportReleaseDescriptor(const sensorarrayTransportDescriptor_t *d) {(void)d;return true;}
static void sensorarrayTransportFillFrameMeta(sensorarrayTransportPayloadSlot_t *slot) {slot->hasFrameMeta=false;}
static bool sensorarrayTransportFindFieldU32(const char *data,const char *field,uint32_t *out) {(void)data;(void)field;(void)out;return false;}
static unsigned xQueueSend(QueueHandle_t queue,const void *data,unsigned timeout) {
    (void)data;assert(queue==s_lifecycleQueue&&timeout==0);lifecycleCalls++;
    assert(transportSlot.length<sizeof(lifecycleText));memcpy(lifecycleText,transportSlot.data,transportSlot.length+1);
    return networkQueueAccept?pdTRUE:0;
}
static void xTaskNotifyGive(unsigned task) {assert(task==s_task);}
esp_err_t sensorarraySerialIoQueue(const char *data,size_t length,bool priority) {
    assert(priority&&length<sizeof(serialText));serialQueueCalls++;
    memcpy(serialText,data,length);serialText[length]='\0';return serialQueueError;
}
static void sensorarrayTransportNoteSerialLog(bool accepted) {if(accepted)serialLogAccepted++;else serialLogRejected++;}
static esp_err_t sensorarrayBleNotifyLifecycleForGeneration(const uint8_t *data,size_t length,uint32_t generation) {
    assert(data&&length&&generation==42);bleTerminalCalls++;return ESP_OK;
}
static esp_err_t sensorarrayWifiSend(unsigned ch,const uint8_t *data,size_t length) {assert(ch==SENSORARRAY_WIFI_CH_LOG&&data&&length);wifiTerminalCalls++;return ESP_OK;}
static void sensorarrayTransportStatResult(sensorarrayTransportChannel_t ch,bool ble,esp_err_t err) {(void)ble;assert(ch==SENSORARRAY_TRANSPORT_CHANNEL_LOG&&err==ESP_OK);}
/* PRODUCTION_TRANSPORT */
static sensorarrayWorkBudgetSnapshot_t schedulerStats;
void sensorarrayWorkBudgetCopySnapshot(sensorarrayWorkBudgetSnapshot_t *out) { *out=schedulerStats; schedulerSnapshotCalls++; }
static bool sensorarrayBootBreadcrumbCopy(sensorarrayBootBreadcrumb_t *boot) { boot->bootId=bootId; return true; }
sensorarrayFrameConfigSnapshot_t sensorarrayScanConfigGetFrameSnapshot(void) { return rows; }
void sensorarrayScanConfigApplyPendingAtFrameBoundary(sensorarrayScanConfigApplyResult_t *r) { memset(r,0,sizeof(*r)); }
static void sensorarrayFrameBuilderInitInvalid(void *f) { (void)f; invalidFrames++; }
static bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteSnapshot_t *r,sensorarrayRouteSnapshot_t *out) { *out=*r; return true; }
static esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteSnapshot_t *r,const char *reason) {
    (void)reason; safeCalls++; r->safe=r->applied=r->known=r->verified=safeError==ESP_OK; r->lastError=(uint32_t)safeError; return safeError;
}
static esp_err_t sensorarrayAdsMatrixEngineSetMode(Ads *a,unsigned mode) { modeStopCalls++; if(stopError==ESP_OK)a->mode=mode; return stopError; }
static bool sensorarrayMeasureFdcWorkersQuiescent(void) { return quiescent; }
static bool sensorarrayMeasureFdcWorkersQuarantined(void) { return quarantined; }
static bool ads126xAdcIsQuarantined(const Ads *a) { (void)a; return quarantined; }
static void sensorarrayFrontendStartupRecheckFdcClock(int *f,const char *r) { (void)f;(void)r; }
static esp_err_t sensorarrayFdcMatrixEngineRunFullRescue(int *f,const char *r) { (void)f;(void)r;return rescueError; }
static void sensorarrayFdcRescueReset(int *f) { (void)f; }
static bool sensorarrayBootPrepareRestart(unsigned kind) { assert(kind==1||kind==2); return autoRestartAllowed; }
static void sensorarrayBootBreadcrumbSetStage(const char *r,esp_err_t e,void *c) { (void)r;(void)e;(void)c; }
static void sensorarrayBootBreadcrumbMarkStableProgress(bool healthy) { assert(!healthy); }
static bool sensorarrayFrontendsHeld(void) { return held; }
static void sensorarrayUsbOtgRestart(bool rom) { (void)rom; restarts++; }
static bool sensorarrayUsbControlHandles(const char *cmd) { (void)cmd; return false; }
static esp_err_t sensorarrayUsbControlHandle(const char *cmd,char *text,size_t size) { (void)cmd;(void)text;(void)size;return ESP_ERR_NOT_SUPPORTED; }
static esp_err_t sensorarrayLinkDiagnosticsFormat(const char *cmd,char *text,size_t size) { (void)cmd;(void)text;(void)size;return ESP_ERR_NOT_SUPPORTED; }
static bool sensorarrayLinkDiagnosticsHandles(const char *cmd) { return strcmp(cmd,"BLELINK?")==0 || strcmp(cmd,"BLETX?")==0 || strcmp(cmd,"BLESVC?")==0 || strcmp(cmd,"SCHEDWHY?")==0; }
static bool sensorarrayAdsTimingControlAck(uint32_t id,esp_err_t err) { (void)id;(void)err;return false; }
static void vTaskDelay(unsigned ticks) { assert(ticks<=21); }
static bool sensorarrayCommandMailboxCommitConfigAck(uint32_t id) { (void)id; ackCalls++; return true; }
static bool sensorarrayCommandMailboxCancelConfigAck(uint32_t id) { (void)id; ackCalls++; return true; }
void sensorarraySerialIoGetStats(sensorarraySerialIoStats_t *out) { *out=serialStats; serialSnapshotCalls++; }
/* PRODUCTION_PUBLISH */
static esp_err_t sensorarrayApplyMeasurementMode(sensorarrayAppContext_t *ctx, sensorarrayMeasurementMode_t m,uint32_t id,bool emit) {
    (void)emit; modeApplyCalls++;
    if(applyError!=ESP_OK) {
        ctx->routeController.safe=false;ctx->routeController.cleanupError=applyCleanupError;ctx->routeController.lastError=applyCleanupError;
        return applyError;
    }
    assert(sensorarrayMeasurementModeAccept(&ctx->measurementMode,m,id));
    assert(sensorarrayMeasurementModeBeginTransition(&ctx->measurementMode));
    assert(sensorarrayMeasurementModeCompleteTransition(&ctx->measurementMode,ctx->hostFrameSequence+1,0));
    ctx->routeController.applied=ctx->routeController.known=ctx->routeController.verified=true;
    ctx->routeController.safe=false;
    (void)sensorarrayPublishConfigAuthority(ctx,ctx->hostFrameSequence+1); return ESP_OK;
}
/* PRODUCTION_FUNCTIONS */

static size_t maxTerminalBytes;
static char lastTerminal[384];
static void drainTerminal(void) {
    char terminal[384];size_t n=0;
    if(sensorarrayTransportGuaranteedTextIsReserved()) {
        assert(sensorarrayTransportGuaranteedTextTakeDrain(terminal,sizeof(terminal),&n));
        assert(n<384&&(strstr(terminal,"RAPP,cmd=RECOVER,")||strstr(terminal,"RERR,cmd=RECOVER,"))&&strstr(terminal,"physicalSafe=unknown"));
        if(n>maxTerminalBytes)maxTerminalBytes=n;
        memcpy(lastTerminal,terminal,n+1);
    }
}
static void reset(sensorarrayAppContext_t *ctx) {
    (void)sensorarrayTransportGuaranteedTextAbort();
    s_configSessionId++;sensorarrayRecoveryControlInit(s_configSessionId,bootId);
    sensorarrayRestartControlInit(s_configSessionId,bootId);
    lifecycleCalls=serialQueueCalls=bleTerminalCalls=wifiTerminalCalls=serialLogAccepted=serialLogRejected=0;
    serialQueueError=ESP_OK;lifecycleText[0]=serialText[0]='\0';
    netBleEnabled=true;netWifiEnabled=false;networkQueueAccept=true;
    memset(ctx,0,sizeof(*ctx));
    sensorarrayMeasurementModeInit(&ctx->measurementMode);
    sensorarrayRowModeProfileInit(&ctx->rowModeProfile,SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE);
    assert(sensorarrayMeasurementModeAccept(&ctx->measurementMode,SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,7));
    assert(sensorarrayMeasurementModeBeginTransition(&ctx->measurementMode));
    assert(sensorarrayMeasurementModeCompleteTransition(&ctx->measurementMode,100,0));
    ctx->rowModeProfile.generation=17;ctx->rowModeProfile.appliedRequestId=13;
    ctx->hostFrameSequence=100;
    ctx->routeController.applied=ctx->routeController.known=ctx->routeController.verified=true;
    ctx->adsRestoreRecovery.active=false;
    (void)sensorarrayPublishConfigAuthority(ctx,101);
    autoRestartAllowed=held=quarantined=false;quiescent=true;
    safeCalls=modeStopCalls=modeApplyCalls=restarts=invalidFrames=ackCalls=0;
    safeError=stopError=applyError=rescueError=ESP_OK;applyCleanupError=0;
}
static uint32_t postAck(unsigned level) {
    uint32_t id=0;assert(sensorarrayRecoveryPost(level,&id));
    char ack[100];int n=snprintf(ack,sizeof(ack),"RACK,cmd=RECOVER,id=%u,level=%u,state=accepted\n",id,level);
    sensorarrayControlReplyPublished(ack,(size_t)n);assert(ackCalls==0);
    return id;
}
static void expectQuery(const char *a,const char *b) {
    char one[512],two[512];assert(sensorarrayRecoveryControlFormat(one,sizeof(one))==ESP_OK);
    assert(strlen(one)<512&&strstr(one,a)&&strstr(one,b));
    assert(sensorarrayRecoveryControlFormat(two,sizeof(two))==ESP_OK&&!strcmp(one,two));
}
static void testMain(void) {
    sensorarrayAppContext_t c; reset(&c);postAck(2);
    assert(sensorarrayRunRecoveryBoundary(&c));
    assert(c.measurementMode.snapshot.state==SENSORARRAY_MEASUREMENT_STATE_DEGRADED);
    assert(c.measurementMode.snapshot.activeMode==SENSORARRAY_MEASUREMENT_MODE_NONE&&modeStopCalls==1&&safeCalls==1);
    assert(sensorarrayTestActualFrameAdmission(&c)==ESP_ERR_INVALID_STATE&&invalidFrames==1);
    sensorarrayConfigAuthority_t a;assert(sensorarrayConfigIdentityCopy(&a));assert(a.identity.mode==SENSORARRAY_MEASUREMENT_MODE_NONE&&a.identity.state==SENSORARRAY_MEASUREMENT_STATE_DEGRADED);
    expectQuery("reason=restart_guard","originalError=0x103,cleanupError=0x0");drainTerminal();
    /* Dropped terminal does not erase its result; future legal MODE can run. */
    expectQuery("last=rejected","reason=restart_guard");
    assert(sensorarrayApplyMeasurementMode(&c,SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,99,false)==ESP_OK);
    assert(sensorarrayTestActualFrameAdmission(&c)==ESP_OK);
    postAck(0);assert(sensorarrayRunRecoveryBoundary(&c));expectQuery("last=applied","reason=recovered");drainTerminal();

    reset(&c);safeError=ESP_ERR_TIMEOUT;c.adsRestoreRecovery.active=true;postAck(2);
    assert(sensorarrayRunRecoveryBoundary(&c));assert(!c.adsRestoreRecovery.active&&c.adsRestoreRecovery.terminal);
    assert(sensorarrayTestActualFrameAdmission(&c)==ESP_ERR_INVALID_STATE);
    expectQuery("originalError=0x103,cleanupError=0x107","routeKnown=0,routeVerified=0");drainTerminal();

    reset(&c);applyError=ESP_ERR_INVALID_CRC;applyCleanupError=ESP_ERR_INVALID_RESPONSE;
    stopError=ESP_ERR_NO_MEM;safeError=ESP_ERR_TIMEOUT;postAck(0);assert(sensorarrayRunRecoveryBoundary(&c));
    expectQuery("originalError=0x109,cleanupError=0x108","last=rejected");drainTerminal();

    reset(&c);rescueError=ESP_ERR_INVALID_RESPONSE;postAck(1);assert(sensorarrayRunRecoveryBoundary(&c));
    assert(modeApplyCalls==0&&sensorarrayTestActualFrameAdmission(&c)==ESP_ERR_INVALID_STATE);
    expectQuery("originalError=0x108","last=rejected");drainTerminal();

    reset(&c);postAck(2);autoRestartAllowed=true;assert(sensorarrayRunRecoveryBoundary(&c));
    assert(restarts==1&&safeCalls==0);expectQuery("last=restarting","reason=auto_restart");drainTerminal();

    reset(&c);postAck(1);held=true;assert(sensorarrayRunRecoveryBoundary(&c));
    expectQuery("reason=wait_frontends","last=rejected");assert(safeCalls==0);drainTerminal();

    reset(&c);uint32_t id;assert(sensorarrayRecoveryPost(1,&id));
    assert(!sensorarrayRunRecoveryBoundary(&c)&&safeCalls==0); /* Await ACK. */
    char ack[90];snprintf(ack,sizeof(ack),"RACK,cmd=RECOVER,id=%u,level=1,state=accepted\n",id);
    sensorarrayControlReplyFailed(ESP_ERR_TIMEOUT,ack,strlen(ack));
    assert(!sensorarrayRunRecoveryBoundary(&c));expectQuery("reason=ack_publish_failed","terminalQueued=0");
    assert(!sensorarrayTransportGuaranteedTextIsReserved());

    reset(&c);postAck(1);
    sensorarrayMeasurementModeRecordRuntimeFault(&c.measurementMode,ESP_FAIL);
    sensorarrayPublishConfigAuthority(&c,101); /* Revision-only state change. */
    assert(!sensorarrayRunRecoveryBoundary(&c)&&safeCalls==0);
    expectQuery("reason=stale_config","last=rejected");drainTerminal();
}
static sensorarrayConfigAuthority_t threadAuthority;
static pthread_barrier_t barrier;
static esp_err_t posted[3];
static void *postThread(void *arg) {
    uintptr_t i=(uintptr_t)arg;pthread_barrier_wait(&barrier);
    uint32_t id=0;posted[i]=sensorarrayRecoveryControlPost(1,&threadAuthority,true,&id);return NULL;
}
static void testCache(void) {
    sensorarrayAppContext_t c;reset(&c);assert(sensorarrayConfigIdentityCopy(&threadAuthority));
    pthread_t threads[3];pthread_barrier_init(&barrier,NULL,3);
    for(uintptr_t i=0;i<3;i++)assert(!pthread_create(&threads[i],NULL,postThread,(void *)i));
    for(unsigned i=0;i<3;i++)pthread_join(threads[i],NULL);
    assert((posted[0]==ESP_OK)+(posted[1]==ESP_OK)+(posted[2]==ESP_OK)==1);pthread_barrier_destroy(&barrier);
    uint32_t id=s_recover.current.id;assert(!sensorarrayRecoveryControlAck(id+1,ESP_OK));
    assert(sensorarrayRecoveryControlAck(id,ESP_OK));sensorarrayRecoverRequest_t request;
    assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_APPLIED,"recovered",0,0,&threadAuthority,true,true,true,true)==ESP_ERR_INVALID_STATE);
    assert(sensorarrayRecoveryControlTake(&threadAuthority,true,&request)==SENSORARRAY_RECOVER_TAKE_READY);
    assert(sensorarrayRecoveryControlTake(&threadAuthority,true,&request)==SENSORARRAY_RECOVER_TAKE_NONE);
    assert(sensorarrayRecoveryControlComplete(id+1,SENSORARRAY_RECOVER_APPLIED,"recovered",0,0,&threadAuthority,true,true,true,true)==ESP_ERR_INVALID_STATE);
    assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_APPLIED,"recovered",0,0,&threadAuthority,true,true,true,true)==ESP_OK);drainTerminal();
    for(unsigned variant=0;variant<10;variant++) {
        reset(&c);sensorarrayConfigAuthority_t before;assert(sensorarrayConfigIdentityCopy(&before));
        assert(sensorarrayRecoveryControlPost(1,&before,true,&id)==ESP_OK);assert(sensorarrayRecoveryControlAck(id,0));
        sensorarrayConfigAuthority_t after=before;
        switch(variant){case 0:after.identity.sessionId++;break;case 1:after.identity.bootId++;break;case 2:after.identity.revision++;break;
        case 3:after.identity.modeGeneration++;break;case 4:after.identity.modeRequestId++;break;case 5:after.identity.rowsGeneration++;break;
        case 6:after.identity.rowsRequestId++;break;case 7:after.identity.profileGeneration++;break;case 8:after.identity.profileRequestId++;break;default:break;}
        assert(sensorarrayRecoveryControlTake(&after,variant!=9,&request)==SENSORARRAY_RECOVER_TAKE_STALE);
        assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_REJECTED,"stale_config",ESP_ERR_INVALID_STATE,0,&after,variant!=9,false,false,false)==ESP_OK);drainTerminal();
    }
    reset(&c);sensorarrayConfigAuthority_t a;assert(sensorarrayConfigIdentityCopy(&a));
    s_recover.nextId=UINT32_MAX-1;s_recover.bootId=UINT32_MAX;a.identity.bootId=UINT32_MAX;a.identity.revision=UINT32_MAX;a.identity.appliedFrameBoundary=UINT32_MAX;
    assert(sensorarrayRecoveryControlPost(2,&a,true,&id)==ESP_OK&&id==UINT32_MAX);
    assert(sensorarrayRecoveryControlAck(id,0));assert(sensorarrayRecoveryControlTake(&a,true,&request)==SENSORARRAY_RECOVER_TAKE_READY);
    assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_RESTARTING,"1234567890123456789012345678901",ESP_FAIL,ESP_FAIL,&a,true,false,false,false)==ESP_OK);drainTerminal();
    assert(sensorarrayRecoveryControlPost(2,&a,true,&id)==ESP_OK&&id==1);
    /* All simultaneously representable fields: both pending and last at max. */
    s_recover.current.id=UINT32_MAX; id=UINT32_MAX;
    char out[512];assert(sensorarrayRecoveryControlFormat(out,sizeof(out))==ESP_OK&&strlen(out)<512);
    printf("RECOVERY_QUERY_EXTREMA_PASS,bytes=%zu,capacity=512,terminal_bytes=%zu,terminal_capacity=384\n",strlen(out),maxTerminalBytes);
    char tiny[20];assert(sensorarrayRecoveryControlFormat(tiny,sizeof(tiny))==ESP_ERR_INVALID_SIZE&&tiny[0]=='\0');
    assert(sensorarrayRecoveryControlAck(id,ESP_FAIL));assert(!sensorarrayTransportGuaranteedTextIsReserved());
    const char *legacyReasons[]={"stale_config","wait_frontends","fdc_restart_required"};
    for(unsigned i=0;i<3;i++) {
        s_recover.nextId=UINT32_MAX-1;
        assert(sensorarrayRecoveryControlPost(2,&a,true,&id)==ESP_OK && id==UINT32_MAX);
        assert(sensorarrayRecoveryControlAck(id,0));
        assert(sensorarrayRecoveryControlTake(&a,true,&request)==SENSORARRAY_RECOVER_TAKE_READY);
        assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_REJECTED,legacyReasons[i],ESP_FAIL,ESP_FAIL,&a,true,false,false,false)==ESP_OK);
        drainTerminal(); assert(strncmp(lastTerminal,"RERR,cmd=RECOVER,",17)==0);
        assert(strstr(lastTerminal,i==0?"currentRev=4294967295":i==1?"route=UNKNOWN":"restartRequired=1"));
    }
    printf("RECOVERY_TERMINAL_EXTREMA_PASS,bytes=%zu,capacity=384,legacy_rerr=3\n",maxTerminalBytes);
    /* Unknown->unknown boot recovery is permitted; publication makes it stale. */
    reset(&c);assert(sensorarrayRecoveryControlPost(2,NULL,false,&id)==ESP_OK);assert(sensorarrayRecoveryControlAck(id,0));
    assert(sensorarrayRecoveryControlTake(NULL,false,&request)==SENSORARRAY_RECOVER_TAKE_READY);
    assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_RESTARTING,"auto_restart",0,0,NULL,false,false,false,false)==ESP_OK);drainTerminal();
    reset(&c);assert(sensorarrayRecoveryControlPost(2,NULL,false,&id)==ESP_OK);assert(sensorarrayRecoveryControlAck(id,0));
    assert(sensorarrayRecoveryControlTake(&a,true,&request)==SENSORARRAY_RECOVER_TAKE_STALE);
    assert(sensorarrayRecoveryControlComplete(id,SENSORARRAY_RECOVER_REJECTED,"stale_config",ESP_ERR_INVALID_STATE,0,&a,true,false,false,false)==ESP_OK);drainTerminal();
}
static void testSerial(void) {
    memset(&serialStats,0xff,sizeof(serialStats));serialSnapshotCalls=0;
    char out[512];assert(sensorarrayFormatSerialReply(out,sizeof(out))==ESP_OK);
    assert(strlen(out)<512&&strstr(out,"driverAcceptedBytes=18446744073709551615"));
    assert(strstr(out,"normalSubmitted=4294967295")&&strstr(out,"driverTxBufferBytes=4294967295")&&serialSnapshotCalls==1);
    assert(strstr(out,"dataSubmitted=4294967295,logSubmitted=4294967295") &&
           strstr(out,"dataRejected=4294967295,logRejected=4294967295"));
    printf("SERIAL_QUERY_EXTREMA_PASS,bytes=%zu,capacity=512,one_snapshot=1\n",strlen(out));
    assert(sensorarrayFormatSerialReply(out,20)==ESP_ERR_INVALID_SIZE&&out[0]=='\0');
}
static void testScheduler(void) {
    memset(&schedulerStats,0xff,sizeof(schedulerStats));
    schedulerStats.registered=schedulerStats.idleKnown=true;
    schedulerStats.idleAgeTicks=schedulerStats.maxIdleGapTicks=UINT32_MAX-1;
    schedulerSnapshotCalls=0;
    char out[512]; assert(sensorarrayFormatSchedulerReply(out,sizeof(out))==ESP_OK);
    assert(strlen(out)<512 && schedulerSnapshotCalls==1);
    assert(strstr(out,"initError=0xFFFFFFFF,registered=1,idleKnown=1") &&
           strstr(out,"graceUnits=4294967295,backstopTicks=4294967295") &&
           strstr(out,"quantumTicks=4294967295") &&
           strstr(out,"bt=4294967295/4294967295/4294967295") &&
           strstr(out,"idleAgeTicks=4294967294,maxIdleGapTicks=4294967294") &&
           strstr(out,"sc=4294967295/4294967295/4294967295") &&
           strstr(out,"wc=4294967295/4294967295/4294967295"));
    printf("SCHED_QUERY_EXTREMA_PASS,bytes=%zu,capacity=512,one_snapshot=1\n",strlen(out));
    schedulerStats.idleKnown=false;
    assert(sensorarrayFormatSchedulerReply(out,sizeof(out))==ESP_OK &&
           strstr(out,"idleKnown=0") && strstr(out,"idleAgeTicks=unknown,maxIdleGapTicks=unknown"));
    schedulerStats.idleKnown=true; schedulerStats.idleAgeTicks=schedulerStats.maxIdleGapTicks=UINT32_MAX;
    assert(sensorarrayFormatSchedulerReply(out,sizeof(out))==ESP_OK && strstr(out,"idleAgeTicks=unknown"));
    assert(sensorarrayFormatSchedulerReply(out,20)==ESP_ERR_INVALID_SIZE && out[0]=='\0');
}
static void expectRestartQuery(const char *one,const char *two) {
    char a[512],b[512];
    assert(sensorarrayRestartControlFormat(a,sizeof(a))==ESP_OK);
    assert(strstr(a,one)&&strstr(a,two));
    assert(sensorarrayRestartControlFormat(b,sizeof(b))==ESP_OK&&!strcmp(a,b));
}
static void restartAck(uint32_t id,esp_err_t error) {
    char text[90];int n=snprintf(text,sizeof(text),"RACK,cmd=RESTART,id=%u,state=accepted\n",id);
    if(error==ESP_OK)sensorarrayControlReplyPublished(text,(size_t)n);
    else sensorarrayControlReplyFailed(error,text,(size_t)n);
}
static void *restartPostThread(void *arg) {
    unsigned index=(unsigned)(uintptr_t)arg;pthread_barrier_wait(&barrier);
    posted[index]=sensorarrayRestartControlPost(&threadAuthority,true,NULL);return NULL;
}
static void testRestart(void) {
    sensorarrayAppContext_t c;reset(&c);uint32_t first,second=UINT32_MAX;
    char reply[512];
    assert(sensorarrayTestEarlyRestartCommand(reply,sizeof(reply))==ESP_OK && strstr(reply,"RACK,cmd=RESTART,id=1,"));
    assert(sensorarrayTestRestartCommand("RESTART",reply,sizeof(reply))==ESP_ERR_INVALID_STATE && strstr(reply,"ERR,cmd=RESTART,reason=pending"));
    assert(sensorarrayTestEarlyRestartCommand(reply,sizeof(reply))==ESP_ERR_INVALID_STATE && strstr(reply,"reason=pending"));
    restartAck(1,ESP_ERR_TIMEOUT);
    assert(sensorarrayTestRestartCommand("RESTART",reply,sizeof(reply))==ESP_OK && strstr(reply,"RACK,cmd=RESTART,id=2,"));
    restartAck(2,ESP_ERR_TIMEOUT);assert(restarts==0);
    reset(&c);
    assert(sensorarrayRestartPost(&first)&&first==1);
    assert(!sensorarrayRestartPost(&second)&&second==UINT32_MAX);
    assert(!sensorarrayRunRecoveryBoundary(&c)&&restarts==0); /* No ACK, no reset. */
    restartAck(first,ESP_ERR_TIMEOUT);
    assert(!sensorarrayRunRecoveryBoundary(&c)&&restarts==0);
    expectRestartQuery("busy=0","last=rejected,reason=ack_publish_failed");
    expectRestartQuery("originalError=0x107","terminalQueued=0");
    assert(sensorarrayRestartPost(&second)&&second==2);
    restartAck(first,ESP_OK);assert(!sensorarrayRunRecoveryBoundary(&c));
    restartAck(second,ESP_OK);assert(sensorarrayRunRecoveryBoundary(&c));
    assert(restarts==1&&lifecycleCalls==1&&safeCalls==0);
    assert(strstr(lifecycleText,"RAPP,cmd=RESTART,id=2,state=restarting,kind=manual,"));
    expectRestartQuery("last=restarting","terminalQueued=1");
    assert(!sensorarrayRunRecoveryBoundary(&c)&&restarts==1);
    assert(!sensorarrayRestartPost(&first)); /* Taken request remains busy until reset. */

    reset(&c);assert(sensorarrayRestartPost(&first));restartAck(first,ESP_OK);
    serialQueueError=ESP_ERR_NO_MEM;networkQueueAccept=false;assert(sensorarrayRunRecoveryBoundary(&c));
    assert(restarts==1);expectRestartQuery("publishError=0x101","terminalQueued=0");
    expectRestartQuery("last=restarting","originalError=0x0"); /* Lost terminal is queryable. */

    reset(&c);assert(sensorarrayTransportGuaranteedTextReserve()==ESP_OK);
    assert(sensorarrayRestartPost(&first));restartAck(first,ESP_OK);
    sensorarrayConfigAuthority_t accepted;assert(sensorarrayConfigIdentityCopy(&accepted));
    sensorarrayMeasurementModeRecordRuntimeFault(&c.measurementMode,ESP_FAIL);
    sensorarrayPublishConfigAuthority(&c,101);held=quarantined=true;quiescent=false;
    assert(sensorarrayRunRecoveryBoundary(&c)&&restarts==1&&safeCalls==0&&modeStopCalls==0);
    assert(sensorarrayTransportGuaranteedTextIsReserved()); /* Independent escape lane. */
    char result[512],expected[60];assert(sensorarrayRestartControlFormat(result,sizeof(result))==ESP_OK);
    snprintf(expected,sizeof(expected),"acceptedRev=%u",accepted.identity.revision);assert(strstr(result,expected));
    sensorarrayConfigAuthority_t now;assert(sensorarrayConfigIdentityCopy(&now));assert(now.identity.revision!=accepted.identity.revision);
    snprintf(expected,sizeof(expected),"rev=%u",now.identity.revision);assert(strstr(result,expected));

    reset(&c);assert(sensorarrayConfigIdentityCopy(&threadAuthority));
    assert(pthread_barrier_init(&barrier,NULL,3)==0);pthread_t threads[3];
    for(unsigned i=0;i<3;i++)assert(pthread_create(&threads[i],NULL,restartPostThread,(void*)(uintptr_t)i)==0);
    unsigned acceptedCount=0;for(unsigned i=0;i<3;i++){assert(pthread_join(threads[i],NULL)==0);acceptedCount+=posted[i]==ESP_OK;}
    assert(pthread_barrier_destroy(&barrier)==0&&acceptedCount==1);
    first=s_restart.current.id;restartAck(first,ESP_OK);assert(sensorarrayRunRecoveryBoundary(&c)&&restarts==1);

    reset(&c);s_restart.nextId=UINT32_MAX-1;s_restart.bootId=UINT32_MAX;
    threadAuthority.identity.revision=threadAuthority.identity.appliedFrameBoundary=UINT32_MAX;
    assert(sensorarrayRestartControlPost(&threadAuthority,true,&first)==ESP_OK&&first==UINT32_MAX);
    assert(sensorarrayRestartControlAck(first,ESP_FAIL));
    assert(sensorarrayRestartControlPost(&threadAuthority,true,&second)==ESP_OK&&second==1);
    s_restart.current.id=UINT32_MAX;second=UINT32_MAX;
    assert(sensorarrayRestartControlFormat(result,sizeof(result))==ESP_OK);size_t maxQuery=strlen(result);
    assert(sensorarrayRestartControlAck(second,ESP_OK));assert(sensorarrayRestartControlTake(&first));
    assert(sensorarrayRestartControlPublish(first,&threadAuthority,true)==ESP_OK);
    s_restart.last.originalError=s_restart.last.publishError=s_restart.last.serialError=s_restart.last.networkError=ESP_FAIL;
    assert(sensorarrayRestartControlFormat(result,sizeof(result))==ESP_OK);
    if(strlen(result)>maxQuery)maxQuery=strlen(result);
    assert(maxQuery<512&&strlen(lifecycleText)<384);
    printf("RESTART_QUERY_EXTREMA_PASS,bytes=%zu,capacity=512,terminal_bytes=%zu,terminal_capacity=384,state_bytes=%zu\n",maxQuery,strlen(lifecycleText),sizeof(s_restart));
    assert(sensorarrayRestartControlFormat(result,20)==ESP_ERR_INVALID_SIZE&&result[0]=='\0');
    sensorarrayRestartControlInit(s_configSessionId+1,bootId+1);
    expectRestartQuery("busy=0","lastId=0,last=none");assert(!sensorarrayRestartControlTake(&first));
    assert(!sensorarrayRestartControlAck(second,ESP_OK));
    sensorarrayRestartControlInit(5, 7);
    uint32_t romId = 0, taken = 0;
    sensorarrayConfigAuthority_t a = threadAuthority;
    assert(sensorarrayRestartControlPostRom(&a,true,&romId)==ESP_OK);
    assert(!sensorarrayRestartControlIsRom(romId));
    assert(!sensorarrayRestartControlTake(NULL));
    assert(sensorarrayRestartControlAck(romId,ESP_FAIL));
    assert(!sensorarrayRestartControlIsRom(romId));
    assert(sensorarrayRestartControlPostRom(&a,true,&romId)==ESP_OK);
    assert(sensorarrayRestartControlAck(romId,ESP_OK));
    assert(sensorarrayRestartControlTake(&taken) && taken==romId);
    assert(sensorarrayRestartControlIsRom(romId));
    assert(!sensorarrayRestartControlIsRom(romId+1));
    assert(sensorarrayRestartControlPublish(romId,&a,true)==ESP_OK);
    assert(strstr(lifecycleText,"target=rom_download"));
    puts("RESTART_ROM_TARGET_PASS,ack_fenced=1,failed_ack_cancels=1,request_id_bound=1");
    puts("RESTART_CONTROL_PASS,overwrite_blocked=1,ack_failure_cancels=1,before_ack_blocked=1,late_ack_fenced=1,lost_terminal_query=1,independent_escape=1,config_change_allowed=1,multitransport=3,wrap=1,new_boot_reset=1,synthetic_reset=1");
}
static void testTerminalRouting(void) {
    sensorarrayAppContext_t c;uint32_t id;
    /* Real Queue's no-network ESP_OK no-op must not claim a descriptor. */
    reset(&c);netBleEnabled=netWifiEnabled=false;
    assert(sensorarrayRestartPost(&id));restartAck(id,ESP_OK);assert(sensorarrayRunRecoveryBoundary(&c));
    assert(restarts==1&&serialQueueCalls==1&&lifecycleCalls==0);
    assert(strstr(serialText,"RAPP,cmd=RESTART,")&&strstr(serialText,"kind=manual,"));
    expectRestartQuery("serialQueued=1,serialError=0x0,netQueued=0,netError=0x0","terminalQueued=1");

    reset(&c);netBleEnabled=netWifiEnabled=false;serialQueueError=ESP_ERR_TIMEOUT;
    assert(sensorarrayRestartPost(&id));restartAck(id,ESP_OK);assert(sensorarrayRunRecoveryBoundary(&c));
    assert(restarts==1&&serialQueueCalls==1&&lifecycleCalls==0);
    expectRestartQuery("serialQueued=0,serialError=0x107,netQueued=0,netError=0x0","terminalQueued=0");

    reset(&c);networkQueueAccept=false;
    assert(sensorarrayRestartPost(&id));restartAck(id,ESP_OK);assert(sensorarrayRunRecoveryBoundary(&c));
    assert(serialQueueCalls==1&&lifecycleCalls==1);
    expectRestartQuery("serialQueued=1,serialError=0x0,netQueued=0,netError=0x107","publishError=0x107");

    reset(&c);serialQueueError=ESP_ERR_TIMEOUT;
    assert(sensorarrayRestartPost(&id));restartAck(id,ESP_OK);assert(sensorarrayRunRecoveryBoundary(&c));
    expectRestartQuery("serialQueued=0,serialError=0x107,netQueued=1,netError=0x0","publishError=0x107");

    /* Execute real GuaranteedText drain: RECOVER already has an explicit
     * Serial priority path, separately from optional BLE/Wi-Fi sinks. */
    reset(&c);netBleEnabled=netWifiEnabled=false;postAck(0);assert(sensorarrayRunRecoveryBoundary(&c));
    assert(serialQueueCalls==0);sensorarrayTransportDrainGuaranteedText();
    assert(serialQueueCalls==1&&serialLogAccepted==1&&bleTerminalCalls==0&&wifiTerminalCalls==0);
    assert(strstr(serialText,"RAPP,cmd=RECOVER,")&&!sensorarrayTransportGuaranteedTextIsReserved());
    expectQuery("last=applied","terminalQueued=1"); /* Historical slot admission. */
    reset(&c);serialQueueError=ESP_ERR_TIMEOUT;netWifiEnabled=true;postAck(0);assert(sensorarrayRunRecoveryBoundary(&c));
    sensorarrayTransportDrainGuaranteedText();
    assert(serialQueueCalls==1&&serialLogRejected==1&&bleTerminalCalls==1&&wifiTerminalCalls==1);
    expectQuery("last=applied","terminalQueued=1"); /* Not a sink/PC receipt flag. */
    puts("TERMINAL_SINK_ROUTING_PASS,production_queue=1,production_guaranteed_drain=1,restart_serial_only=1,usb_reject=1,network_reject=1,no_network_not_queued=1,recover_serial_preserved=1");
}
static void testQueryDispatch(void) {
    sensorarrayAppContext_t c; reset(&c);
    const char *commands[]={"RECOVERY?","RESTART?","SERIAL?","SCHED?"};
    for(unsigned i=0;i<4;i++) {
        char early[512],normal[512];
        assert(sensorarrayTestEarlyQuery(commands[i],early,sizeof(early),NULL)==ESP_OK);
        assert(sensorarrayTestRuntimeQuery(commands[i],normal,sizeof(normal),&c)==ESP_OK);
        assert(!strcmp(early,normal));
    }
}
int main(void) {
    testMain();testCache();testSerial();testScheduler();testRestart();testTerminalRouting();testQueryDispatch();
    printf("RECOVERY_STATIC_STATE,bytes=%zu\n",sizeof(s_recover));
    puts("RECOVERY_CONTROL_PASS,production_modules=6,main_functions=19,budget_stop=1,safe_failure=1,first_cleanup=1,rescue_error=1,lost_terminal_query=1,ack_fence=1,stale_identity=10,multitransport=3,wrap=1,future_mode=1,synthetic_hardware=1");
    return 0;
}
