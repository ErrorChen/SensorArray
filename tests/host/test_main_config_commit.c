#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayConfigIdentity.h"

#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL,line=%d,%s\n",__LINE__,#x);return 1; } } while(0)
#define SENSORARRAY_ROW_MODES_TERMINAL_TEXT_MAX 384u
#define SENSORARRAY_COMMAND_MEASUREMENT_MODE 10

typedef struct { bool fdcSdHigh; } sensorarrayRouteSnapshot_t;
typedef struct { uint32_t bootId; } sensorarrayBootBreadcrumb_t;
typedef struct { int type; uint32_t value; uint32_t requestId; } sensorarrayCommand_t;
typedef struct {
    sensorarrayMeasurementModeContext_t measurementMode;
    sensorarrayRowModeProfile_t rowModeProfile;
    int routeController;
    uint32_t hostFrameSequence;
    struct { uint64_t physicalSweepUs; } frame;
} sensorarrayAppContext_t;

static uint64_t s_configSessionId=UINT64_C(0x0123456789ABCDEF);
static bool s_routeKnown=true, s_acceptFail, s_applyFail;
static unsigned s_applyCalls, s_safeCalls, s_terminalCalls;
static char s_terminal[384];
static sensorarrayConfigAuthority_t s_terminalAuthority;

static bool sensorarrayBootBreadcrumbCopy(sensorarrayBootBreadcrumb_t *boot)
{ boot->bootId=17;return true; }
static bool sensorarrayRouteControllerCopySnapshot(const int *route,sensorarrayRouteSnapshot_t *snapshot)
{ (void)route; snapshot->fdcSdHigh=false;return s_routeKnown; }
static esp_err_t sensorarrayRouteControllerEnterSafe(int *route,const char *reason)
{ (void)route;(void)reason;s_safeCalls++;return ESP_OK; }
static esp_err_t sensorarrayCommandMailboxEmitConfigTerminal(uint32_t id,const char *text,size_t length)
{
    (void)id;
    if (length>=sizeof(s_terminal))return ESP_ERR_INVALID_SIZE;
    memcpy(s_terminal,text,length);s_terminal[length]='\0';s_terminalCalls++;
    (void)sensorarrayConfigIdentityCopy(&s_terminalAuthority);
    return ESP_OK;
}
static esp_err_t sensorarrayApplyMeasurementMode(sensorarrayAppContext_t *ctx,
    sensorarrayMeasurementMode_t mode,uint32_t id,bool emitEvent)
{
    (void)emitEvent;s_applyCalls++;
    if (!sensorarrayMeasurementModeAccept(&ctx->measurementMode,mode,id) ||
        !sensorarrayMeasurementModeBeginTransition(&ctx->measurementMode))return ESP_ERR_INVALID_STATE;
    if (s_applyFail) {
        sensorarrayMeasurementModeFailTransition(&ctx->measurementMode,ESP_ERR_INVALID_RESPONSE,2);
        return ESP_ERR_INVALID_RESPONSE;
    }
    return sensorarrayMeasurementModeCompleteTransition(&ctx->measurementMode,ctx->hostFrameSequence+1,2)?ESP_OK:ESP_ERR_INVALID_STATE;
}
static bool injectedAccept(sensorarrayRowModeProfile_t *profile,const sensorarrayMeasurementMode_t modes[8],uint32_t id)
{ return !s_acceptFail && sensorarrayRowModeProfileAccept(profile,modes,id); }

/* INSERT_PRODUCTION_PUBLISH */
/* INSERT_PRODUCTION_TERMINAL */

#define sensorarrayRowModeProfileAccept injectedAccept
static void runMode(sensorarrayAppContext_t *ctx,sensorarrayCommand_t command)
{
    for(unsigned once=0;once<1;++once) {
        switch(command.type) {
/* INSERT_PRODUCTION_MODE_CASE */
        default: break;
        }
    }
}
#undef sensorarrayRowModeProfileAccept

static void runMixedFailure(sensorarrayAppContext_t *ctx,
    sensorarrayRowModeProfile_t rowProfile,
    const sensorarrayMeasurementMode_t effectiveModes[8],
    uint32_t frameSequence,uint32_t mixedFailureErr)
/* INSERT_PRODUCTION_MIXED_FAILURE */

static void reset(sensorarrayAppContext_t *ctx)
{
    memset(ctx,0,sizeof(*ctx));sensorarrayMeasurementModeInit(&ctx->measurementMode);
    sensorarrayRowModeProfileInit(&ctx->rowModeProfile,SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE);
    (void)sensorarrayMeasurementModeAccept(&ctx->measurementMode,SENSORARRAY_MEASUREMENT_MODE_CAPACITANCE,7);
    (void)sensorarrayMeasurementModeBeginTransition(&ctx->measurementMode);
    (void)sensorarrayMeasurementModeCompleteTransition(&ctx->measurementMode,100,2);
    ctx->hostFrameSequence=100;
    s_routeKnown=true;s_acceptFail=false;s_applyFail=false;s_applyCalls=0;s_safeCalls=0;s_terminalCalls=0;s_terminal[0]='\0';
    s_configSessionId++;
    (void)sensorarrayPublishConfigAuthority(ctx,101);
}

int main(void)
{
    CHECK(sensorarrayScanConfigInit()==ESP_OK);
    sensorarrayAppContext_t ctx;
    sensorarrayCommand_t command={.type=SENSORARRAY_COMMAND_MEASUREMENT_MODE,.value=SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,.requestId=8};
    reset(&ctx);s_routeKnown=false;runMode(&ctx,command);
    CHECK(s_applyCalls==0 && s_terminalCalls==1 && strncmp(s_terminal,"MERR,",5)==0);
    CHECK(ctx.measurementMode.snapshot.appliedRequestId==7 && ctx.rowModeProfile.appliedRequestId==0);
    reset(&ctx);s_acceptFail=true;runMode(&ctx,command);
    CHECK(s_applyCalls==0 && s_terminalCalls==1 && strncmp(s_terminal,"MERR,",5)==0);
    CHECK(ctx.measurementMode.snapshot.appliedRequestId==7);
    reset(&ctx);ctx.measurementMode.snapshotVersion=1;runMode(&ctx,command);
    CHECK(s_applyCalls==0 && s_terminalCalls==1 && strncmp(s_terminal,"MERR,",5)==0);
    CHECK(strstr(s_terminal, "identityKnown=0")!=NULL && strstr(s_terminal, "session=0000000000000000")==NULL);
    reset(&ctx);ctx.rowModeProfile.snapshotVersion=1;runMode(&ctx,command);
    CHECK(s_applyCalls==0 && s_terminalCalls==1 && strncmp(s_terminal,"MERR,",5)==0);
    reset(&ctx);s_applyFail=true;runMode(&ctx,command);
    CHECK(s_applyCalls==1 && s_terminalCalls==1 && strncmp(s_terminal,"MERR,",5)==0);
    CHECK(ctx.measurementMode.snapshot.appliedRequestId==7 && ctx.rowModeProfile.appliedRequestId==0);
    CHECK(s_terminalAuthority.modeTerminalId==8 && s_terminalAuthority.profileTerminalId==8);
    CHECK(s_terminalAuthority.identity.state==SENSORARRAY_MEASUREMENT_STATE_SAFE);
    reset(&ctx);runMode(&ctx,command);
    CHECK(s_applyCalls==1 && s_terminalCalls==1 && strncmp(s_terminal,"MAPP,",5)==0);
    CHECK(s_terminalAuthority.identity.modeRequestId==8 && s_terminalAuthority.identity.profileRequestId==8);
    CHECK(s_terminalAuthority.identity.revision==2 && s_terminalAuthority.identity.appliedFrameBoundary==101);
    CHECK(strstr(s_terminal,",rev=2,boundary=101,identityKnown=1\n")!=NULL);
    CHECK(strcmp(s_terminalAuthority.identity.profile,"VVVVVVVV")==0);
    reset(&ctx);
    sensorarrayMeasurementMode_t mixed[8];CHECK(sensorarrayRowModeProfileParse("CVRCVRCV",8,mixed));
    CHECK(sensorarrayRowModeProfileAccept(&ctx.rowModeProfile,mixed,9));
    sensorarrayRowModeProfile_t pending;CHECK(sensorarrayRowModeProfileCopy(&ctx.rowModeProfile,&pending));
    runMixedFailure(&ctx,pending,mixed,101,ESP_ERR_INVALID_RESPONSE);
    CHECK(s_safeCalls==1 && s_terminalCalls==1 && strncmp(s_terminal,"RMERR,",6)==0);
    CHECK(s_terminalAuthority.identity.state==SENSORARRAY_MEASUREMENT_STATE_DEGRADED);
    CHECK(s_terminalAuthority.identity.mode==SENSORARRAY_MEASUREMENT_MODE_NONE);
    CHECK(s_terminalAuthority.profileTerminalId==9 && s_terminalAuthority.profileOutcome==SENSORARRAY_CONFIG_OUTCOME_REJECTED);
    CHECK(s_terminalAuthority.identity.profileRequestId==0);
    CHECK(strstr(s_terminal,",rev=2,boundary=101,identityKnown=1\n")!=NULL);
    puts("MAIN_CONFIG_COMMIT,passed=7,source=production_branches_with_hardware_stubs");
    return 0;
}
