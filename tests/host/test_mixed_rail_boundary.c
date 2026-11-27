#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayMixedRow.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayFrameBuilder.h"

/* Synthetic fault injection. Production MixedRow.c, ScanPlan.c and
 * FrameBuilder.c execute; only hardware/child engine boundaries are stubs. */
static sensorarrayState_t s_state;
static sensorarrayRouteController_t s_route;
static sensorarrayAdsMatrixEngine_t s_ads;
static sensorarrayFdcMatrixEngine_t s_fdc;
static sensorarrayFrame_t s_frame,s_segment;
static sensorarrayScanPlan_t s_plan;
static sensorarrayAdsTransactionEvidence_t s_railTransaction;
static bool s_railAvailable,s_isolated,s_unknownRoute;
static uint32_t s_railAge,s_minRequestedAge;
static unsigned s_refreshCalls,s_selectCalls,s_applyCalls,s_capCalls,s_adsCalls,s_sampledCells,s_safeCalls;
static unsigned s_failSelectCall;
static esp_err_t s_refreshError,s_gapCleanupError,s_restoreError,s_waitError;
static char s_order[512],s_output[768];
static size_t s_orderLength;
static void record(char what,unsigned row)
{ s_orderLength+=(size_t)snprintf(s_order+s_orderLength,sizeof(s_order)-s_orderLength,"%c%u,",what,row); }
int64_t esp_timer_get_time(void) { static int64_t time;return ++time; }
int sensorarrayAcqEventPrintf(const char *format,...)
{ va_list args;va_start(args,format);int n=vsnprintf(s_output,sizeof(s_output),format,args);va_end(args);return n; }
const char *sensorarrayAdsOwnerName(sensorarrayAdsOwner_t owner)
{ return owner==SENSORARRAY_ADS_OWNER_RAIL?"RAIL":"NONE"; }
sensorarrayFrameConfigSnapshot_t sensorarrayScanConfigGetFrameSnapshot(void)
{ return (sensorarrayFrameConfigSnapshot_t){.rows=8,.cells=64,.rowMask=0xFF,.generation=2,.requestId=2}; }
bool sensorarrayBoardMapHasIsolatedRailMonitor(void) { return s_isolated; }
bool sensorarrayAdsGapCopyRailSplit(uint32_t sequence,uint32_t maximumAge,sensorarrayAdsRailSplit_t *out)
{
    (void)sequence;
    if(maximumAge<s_minRequestedAge)s_minRequestedAge=maximumAge;
    *out=(sensorarrayAdsRailSplit_t){.valid=s_railAvailable && s_railAge<=maximumAge,
        .avddUv=3300000,.avssUv=-1800000,.ageFrames=s_railAge};
    return out->valid;
}
void sensorarrayAdsGapCopyTransactionEvidence(sensorarrayAdsTransactionEvidence_t *rail,
    sensorarrayAdsTransactionEvidence_t *battery,sensorarrayAdsTransactionEvidence_t *check)
{ if(rail)*rail=s_railTransaction;(void)battery;(void)check; }
esp_err_t sensorarrayAdsGapRefreshRailAtBoundary(sensorarrayState_t *state,uint32_t sequence)
{
    (void)state;(void)sequence;
    assert(s_sampledCells==0); /* Detect the original CAP-before-refresh defect. */
    assert(s_route.snapshot.mode==SENSORARRAY_MEASUREMENT_MODE_RESISTANCE || s_isolated);
    assert(s_route.snapshot.mode!=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE ||
           s_plan.rowModes[s_route.snapshot.row-1]==SENSORARRAY_MEASUREMENT_MODE_RESISTANCE);
    s_refreshCalls++;record('H',s_route.snapshot.row);
    s_railTransaction.id++;s_railTransaction.cleanupError=s_gapCleanupError;
    if(s_refreshError==ESP_OK){s_railAvailable=true;s_railAge=0;}
    return s_refreshError;
}
bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteController_t *route,sensorarrayRouteSnapshot_t *out)
{ *out=route->snapshot;return !s_unknownRoute; }
esp_err_t sensorarrayRouteControllerSelectRow(sensorarrayRouteController_t *route,uint8_t row)
{
    s_selectCalls++;record('S',row);
    if(s_selectCalls==s_failSelectCall)return ESP_ERR_TIMEOUT;
    route->snapshot.row=row;return ESP_OK;
}
esp_err_t sensorarrayRouteControllerApplyMode(sensorarrayRouteController_t *route,
    sensorarrayMeasurementMode_t mode,const sensorarrayAdsRailSplit_t *rail,uint64_t *duration)
{
    (void)rail;if(duration)*duration=1;s_applyCalls++;
    record(mode==SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE?'C':mode==SENSORARRAY_MEASUREMENT_MODE_VOLTAGE?'V':'R',route->snapshot.row);
    if(mode==SENSORARRAY_MEASUREMENT_MODE_RESISTANCE) {
        /* REF may only turn on after an intended RES row was selected. */
        assert(s_plan.rowModes[route->snapshot.row-1]==SENSORARRAY_MEASUREMENT_MODE_RESISTANCE);
    }
    if(mode==SENSORARRAY_MEASUREMENT_MODE_VOLTAGE) {
        assert(rail!=NULL && rail->valid);
    }
    if(s_refreshCalls && s_capCalls==0 && s_adsCalls==0 &&
       mode==SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE && s_restoreError!=ESP_OK)return s_restoreError;
    route->snapshot.mode=mode;return ESP_OK;
}
esp_err_t sensorarrayRouteControllerEnterSafeRailMonitor(sensorarrayRouteController_t *route,uint64_t *duration)
{ (void)route;(void)duration;assert(s_isolated);return ESP_OK; }
esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteController_t *route,const char *reason)
{ (void)reason;s_safeCalls++;route->snapshot.safe=true;return ESP_OK; }
esp_err_t ads126xAdcWaitReferenceReady(ads126xAdcHandle_t *ads)
{ (void)ads;record('W',0);return s_waitError; }
void sensorarrayAdsMatrixEngineInvalidateCaches(sensorarrayAdsMatrixEngine_t *engine)
{ (void)engine;record('I',0); }
esp_err_t sensorarrayAdsMatrixEngineSetMode(sensorarrayAdsMatrixEngine_t *engine,sensorarrayMeasurementMode_t mode)
{ engine->mode=mode;return ESP_OK; }
static void acquire(const sensorarrayScanPlan_t *plan,sensorarrayFrame_t *frame,bool cap)
{
    assert(!plan->auxiliaryTransactionsAllowed); /* Child cannot run a helper. */
    sensorarrayFrameBuilderInitInvalid(frame);
    uint64_t mask=sensorarrayFrameBuilderActiveCellMask(plan->configSnapshot.rowMask);
    frame->configSnapshot=plan->configSnapshot;frame->activeRows=plan->rowCount;
    frame->freshMask=frame->validMask=frame->acquiredMask=mask;
    frame->expectedMask=mask;frame->rowFreshMask=plan->configSnapshot.rowMask;frame->freshFrame=true;
    frame->errorMask=0;
    if(cap)frame->capValidMask=mask;
    else { frame->measurement.freshMask=frame->measurement.validMask=mask; }
    for(unsigned i=0;i<64;i++)if(mask&(UINT64_C(1)<<i))s_sampledCells++;
    frame->frameStartUs=(uint64_t)esp_timer_get_time();frame->frameEndUs=frame->frameStartUs+1;
}
esp_err_t sensorarrayFdcMatrixEngineReadFrame(sensorarrayFdcMatrixEngine_t *engine,
    const sensorarrayScanPlan_t *plan,sensorarrayFrame_t *frame)
{ (void)engine;s_capCalls++;record('F',0);acquire(plan,frame,true);return ESP_OK; }
esp_err_t sensorarrayAdsMatrixEngineReadFrame(sensorarrayAdsMatrixEngine_t *engine,
    const sensorarrayScanPlan_t *plan,sensorarrayFrame_t *frame)
{ (void)engine;s_adsCalls++;record('D',0);acquire(plan,frame,false);return ESP_OK; }
static void reset(const char *profile)
{
    memset(&s_state,0,sizeof(s_state));memset(&s_route,0,sizeof(s_route));
    memset(&s_ads,0,sizeof(s_ads));memset(&s_fdc,0,sizeof(s_fdc));
    memset(&s_railTransaction,0,sizeof(s_railTransaction));
    sensorarrayMeasurementMode_t modes[8];
    for(unsigned i=0;i<8;i++)modes[i]=profile[i]=='C'?SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE:
        profile[i]=='V'?SENSORARRAY_MEASUREMENT_MODE_VOLTAGE:SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    sensorarrayScanPlanBuildRowProfile(&s_plan,modes,3,3);
    s_ads.state=&s_state;s_ads.routeController=&s_route;s_ads.frameSequenceHint=101;s_fdc.state=&s_state;
    s_route.snapshot=(sensorarrayRouteSnapshot_t){.mode=SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,
        .row=2,.known=true,.verified=true,.applied=true};
    s_railAvailable=false;s_isolated=false;s_unknownRoute=false;s_railAge=101;s_minRequestedAge=UINT32_MAX;
    s_refreshCalls=s_selectCalls=s_applyCalls=s_capCalls=s_adsCalls=s_sampledCells=s_safeCalls=0;
    s_failSelectCall=0;s_refreshError=s_gapCleanupError=s_restoreError=s_waitError=ESP_OK;
    s_order[0]=s_output[0]='\0';s_orderLength=0;
}
static esp_err_t run(void)
{ return sensorarrayMixedRowEngineReadFrame(&s_fdc,&s_ads,&s_plan,&s_frame,&s_segment); }
static void unacquired(void)
{ assert(s_capCalls==0 && s_adsCalls==0 && s_frame.acquiredMask==0 && !s_frame.freshFrame); }
int main(void)
{
    reset("CCCCCCRR");assert(s_plan.auxiliaryTransactionsAllowed);
    assert(run()==ESP_OK);assert(s_refreshCalls==1 && s_capCalls==1 && s_adsCalls==1 && s_sampledCells==64);
    assert(s_frame.acquiredMask==UINT64_MAX && s_frame.freshFrame && s_minRequestedAge==99);
    assert(!strcmp(s_order,"S7,R7,H7,C7,S2,W0,I0,C2,F0,S7,R7,D0,"));
    reset("CCCCCCRR");s_railAvailable=true;s_railAge=1;
    esp_err_t cleanup=ESP_FAIL;
    assert(sensorarrayMixedRowPrepareRailAtBoundary(&s_ads,&s_plan,101,&cleanup)==ESP_OK);
    assert(s_applyCalls==0 && s_selectCalls==0 && s_refreshCalls==0 && cleanup==ESP_OK);
    assert(run()==ESP_OK && s_refreshCalls==0);
    reset("CCCCCCVV");assert(run()==ESP_ERR_NOT_SUPPORTED);unacquired();
    assert(s_refreshCalls==0 && s_applyCalls==0 && s_selectCalls==0);
    reset("CCCCCCVV");s_railAvailable=true;s_railAge=500;
    assert(run()==ESP_OK && s_refreshCalls==0 && s_capCalls==1 && s_adsCalls==1 && s_sampledCells==64);
    reset("CCCCCCRR");s_refreshError=ESP_ERR_INVALID_CRC;
    assert(run()==ESP_ERR_INVALID_CRC);unacquired();
    assert(s_route.snapshot.mode==SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE && s_route.snapshot.row==2);
    assert(strstr(s_output,"originalError=0x109,cleanupError=0x0,restored=1"));
    reset("CCCCCCRR");s_restoreError=ESP_ERR_INVALID_RESPONSE;
    assert(run()==ESP_ERR_INVALID_RESPONSE);unacquired();assert(s_safeCalls==1);
    assert(strstr(s_output,"originalError=0x0,cleanupError=0x108,restored=0"));
    reset("CCCCCCRR");s_refreshError=ESP_ERR_INVALID_CRC;s_restoreError=ESP_ERR_TIMEOUT;
    s_ads.faultContext=(sensorarrayAdsMatrixFaultContext_t){.valid=true,.sequence=100,
        .stage=SENSORARRAY_ADS_FAULT_STAGE_RESET_OBSERVED,.originalError=ESP_FAIL};
    assert(run()==ESP_ERR_INVALID_CRC);unacquired();
    assert(strstr(s_output,"originalError=0x109,cleanupError=0x107"));
    assert(s_ads.faultContext.valid && s_ads.faultContext.sequence==101 &&
        s_ads.faultContext.stage==SENSORARRAY_ADS_FAULT_STAGE_RAIL_MONITOR &&
        s_ads.faultContext.originalError==ESP_ERR_INVALID_CRC &&
        s_ads.faultContext.cleanupError==ESP_ERR_TIMEOUT &&
        !s_ads.faultContext.attempted && !s_ads.faultContext.acquired);
    sensorarrayAdsFaultEvent_t fault={.stage=SENSORARRAY_ADS_FAULT_STAGE_PROFILE_TRANSITION,
        .err=ESP_ERR_INVALID_STATE,.seq=101,.context=&s_ads.faultContext};
    char formatted[1537];
    assert(sensorarrayAdsFaultFormat(&fault,formatted,sizeof(formatted))>0);
    assert(strstr(formatted,"stage=RAIL_MONITOR,err=0x109") &&
        strstr(formatted,"originalError=0x109,cleanupError=0x107") &&
        strstr(formatted,"attempted=0,acquired=0"));
    sensorarrayAdsMatrixEngineRecordMixedBoundaryFault(&s_ads,101,ESP_FAIL,ESP_FAIL);
    assert(s_ads.faultContext.originalError==ESP_ERR_INVALID_CRC &&
        s_ads.faultContext.cleanupError==ESP_ERR_TIMEOUT);
    reset("CCCCCCRR");s_failSelectCall=1;
    assert(run()==ESP_ERR_TIMEOUT);unacquired();assert(s_refreshCalls==0);
    reset("CCCCCCRR");s_unknownRoute=true;
    assert(run()==ESP_ERR_INVALID_STATE);unacquired();assert(s_applyCalls==0 && s_refreshCalls==0);
    reset("CCCCCCRR");s_waitError=ESP_ERR_TIMEOUT;
    assert(run()==ESP_ERR_TIMEOUT);unacquired();assert(s_safeCalls==1);
    reset("CVRCCCCC");s_plan.rowCount=2;s_plan.configSnapshot.rowMask=3;
    assert(run()==ESP_ERR_NOT_SUPPORTED);unacquired();assert(s_refreshCalls==0);
    reset("CCCCCCRR");s_plan.auxiliaryTransactionsAllowed=false;
    assert(run()==ESP_ERR_INVALID_STATE);unacquired();assert(s_refreshCalls==0);
    reset("CCCCCCRR");s_railAvailable=true;s_railAge=100;
    assert(run()==ESP_OK && s_refreshCalls==1 && s_minRequestedAge==99);
    reset("CCCCCCRR");s_refreshError=ESP_ERR_INVALID_CRC;s_gapCleanupError=ESP_ERR_TIMEOUT;
    assert(run()==ESP_ERR_INVALID_CRC);unacquired();assert(s_safeCalls==1);
    assert(strstr(s_output,"originalError=0x109,cleanupError=0x107"));
    sensorarrayScanPlanBuildDefaultFdcMatrix(&s_plan);assert(s_plan.auxiliaryTransactionsAllowed);
    sensorarrayScanPlanBuildAdsMatrix(&s_plan,SENSORARRAY_MEASUREMENT_MODE_RESISTANCE);assert(s_plan.auxiliaryTransactionsAllowed);
    sensorarrayScanPlanBuildMixedExample(&s_plan);assert(s_plan.auxiliaryTransactionsAllowed);
    puts("MIXED_RAIL_BOUNDARY,passed=14,source=complete_production_mixed_engine,fixture=synthetic");
    return 0;
}
