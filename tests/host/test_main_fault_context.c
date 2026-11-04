#include <stdio.h>
#include <string.h>
#include "sensorarrayConfigIdentity.h"
#include "sensorarrayAdsFault.h"
#include "sensorarrayAdsCache.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayFrame.h"

/* Synthetic regression based on repaired-res8-01.rx.bin seq=4231: that
 * capture had ROWS gen/rid=2/2 but ADSFAULT incorrectly printed profile 3/3.
 * Keep all three generations different here to catch either domain swap. */
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL,line=%d,%s\n",__LINE__,#x);return 1; } } while (0)
typedef struct { bool safe; sensorarrayMeasurementMode_t mode; } sensorarrayRouteSnapshot_t;
typedef struct { sensorarrayRouteSnapshot_t snapshot; } sensorarrayRouteController_t;
typedef struct { uint32_t bootId,bootCount; } sensorarrayBootBreadcrumb_t;
typedef struct {
    sensorarrayAdsRegisterCache_t cache;
    sensorarrayAdsMatrixFaultContext_t captured;
    sensorarrayMeasurementMode_t mode;
} sensorarrayAdsMatrixEngine_t;
typedef struct {
    sensorarrayMeasurementModeContext_t measurementMode;
    sensorarrayRowModeProfile_t rowModeProfile;
    sensorarrayRouteController_t routeController;
    sensorarrayAdsMatrixEngine_t adsEngine;
    struct { ads126xAdcHandle_t ads; } state;
    sensorarrayFrame_t frame;
    uint32_t hostFrameSequence;
} sensorarrayAppContext_t;

static const uint64_t s_configSessionId=UINT64_C(0x78359E30AC76FFB2);
static char s_emitted[4096];
static size_t s_emittedLength;
static unsigned s_safeCalls,s_setModeCalls;
static bool sensorarrayBootBreadcrumbCopy(sensorarrayBootBreadcrumb_t *boot)
{ boot->bootId=5;boot->bootCount=5;return true; }
static bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteController_t *route,
    sensorarrayRouteSnapshot_t *snapshot)
{ *snapshot=route->snapshot;return true; }
static esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteController_t *route,const char *reason)
{ (void)reason;s_safeCalls++;route->snapshot.safe=true;route->snapshot.mode=SENSORARRAY_MEASUREMENT_MODE_NONE;return ESP_ERR_TIMEOUT; }
static sensorarrayAdsRegisterCache_t *sensorarrayAdsMatrixEngineRegisterCache(sensorarrayAdsMatrixEngine_t *engine)
{ return &engine->cache; }
static void sensorarrayAdsMatrixEngineInvalidateCaches(sensorarrayAdsMatrixEngine_t *engine)
{ engine->cache.generation+=100; }
static esp_err_t sensorarrayAdsMatrixEngineSetMode(sensorarrayAdsMatrixEngine_t *engine,sensorarrayMeasurementMode_t mode)
{ s_setModeCalls++;engine->mode=mode;engine->cache.generation+=100;return ESP_OK; }
static bool sensorarrayAdsMatrixEngineCopyFaultContext(const sensorarrayAdsMatrixEngine_t *engine,
    sensorarrayAdsMatrixFaultContext_t *out)
{ *out=engine->captured;return out->valid; }
uint32_t ads126xAdcGetDrdyGeneration(const ads126xAdcHandle_t *ads)
{ return ads->drdyGeneration; }
const char *sensorarrayAdsOwnerName(sensorarrayAdsOwner_t owner)
{ return owner==SENSORARRAY_ADS_OWNER_MATRIX?"MATRIX":"CHECK"; }
const char *sensorarrayAdsRailSourceName(sensorarrayAdsRailSource_t source)
{ (void)source;return "monitor"; }
static void sensorarrayPublishConfigAuthority(sensorarrayAppContext_t *ctx,uint32_t boundary)
{ (void)ctx;(void)boundary; }
static int64_t esp_timer_get_time(void)
{ static int64_t now;now+=1000001;return now; }
static void sensorarrayAdsFaultSink(const char *line,size_t length,void *context)
{
    (void)context;
    if (length<sizeof(s_emitted)-s_emittedLength) {
        memcpy(s_emitted+s_emittedLength,line,length);
        s_emittedLength+=length;s_emitted[s_emittedLength]='\0';
    }
}

/* INSERT_PRODUCTION_FILL */
/* INSERT_PRODUCTION_EMIT */

static void runHomogeneous(sensorarrayAppContext_t *ctx,esp_err_t err,uint32_t frameSequence,
    sensorarrayMeasurementModeSnapshot_t mode)
/* INSERT_PRODUCTION_HOMOGENEOUS */

static void runMixed(sensorarrayAppContext_t *ctx,esp_err_t err,uint32_t frameSequence)
/* INSERT_PRODUCTION_MIXED */

static void runCheckRestore(sensorarrayAppContext_t *ctx,esp_err_t checkErr,uint32_t requestId)
{
    struct { struct { uint8_t power; } registers; } result={.registers.power=1};
/* INSERT_PRODUCTION_CHECK */
}

static void reset(sensorarrayAppContext_t *ctx)
{
    memset(ctx,0,sizeof(*ctx));
    sensorarrayMeasurementModeInit(&ctx->measurementMode);
    sensorarrayRowModeProfileInit(&ctx->rowModeProfile,SENSORARRAY_MEASUREMENT_MODE_RESISTANCE);
    ctx->measurementMode.snapshot.activeMode=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    ctx->measurementMode.snapshot.state=SENSORARRAY_MEASUREMENT_STATE_RESISTANCE;
    ctx->measurementMode.snapshot.generation=3;
    ctx->measurementMode.snapshot.appliedRequestId=3;
    ctx->rowModeProfile.generation=7;
    ctx->rowModeProfile.appliedRequestId=17;
    ctx->routeController.snapshot.mode=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    ctx->adsEngine.mode=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;
    ctx->adsEngine.cache.generation=47;
    ctx->state.ads.drdyGeneration=34293;
    ctx->hostFrameSequence=ctx->frame.sequence=4231;
    ctx->frame.identity=(sensorarrayConfigIdentity_t){
        .sessionId=s_configSessionId,.bootId=5,.revision=4,.mode=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE,
        .modeGeneration=3,.modeRequestId=3,.rowsGeneration=2,.rowsRequestId=2,
        .profileGeneration=7,.profileRequestId=17,.profile="RRRRRRRR",.rows=8,
    };
    ctx->frame.measurement.railValid=true;ctx->frame.measurement.avddUv=3343697;
    ctx->frame.measurement.avssUv=-1823835;
    ctx->frame.measurement.referenceSource=SENSORARRAY_ADS_REFERENCE_INTERNAL;
    s_emitted[0]='\0';s_emittedLength=0;s_safeCalls=0;s_setModeCalls=0;
}

int main(void)
{
    CHECK(sensorarrayScanConfigInit()==ESP_OK);
    /* Two allocated IDs, but only one applied ROWS change -> gen=2/rid=2. */
    CHECK(sensorarrayScanConfigAllocateRequestId()==1);
    sensorarrayScanConfigRequestResult_t rows;
    CHECK(sensorarrayScanConfigRequestRows(8,&rows)==ESP_OK && rows.requestId==2);
    sensorarrayScanConfigApplyPendingAtFrameBoundary(NULL);
    sensorarrayAppContext_t ctx;
    reset(&ctx);runHomogeneous(&ctx,ESP_ERR_INVALID_CRC,4231,ctx.measurementMode.snapshot);
    CHECK(strstr(s_emitted,"mode=RES,modeGen=3,profileGen=7,rowGen=2,rowReq=2,profile=RRRRRRRR,route=RES"));
    CHECK(strstr(s_emitted,"drdyGen=34293,cfgGen=47"));
    CHECK(ctx.measurementMode.snapshot.activeMode==SENSORARRAY_MEASUREMENT_MODE_NONE && s_safeCalls==1 && s_setModeCalls==1);
    reset(&ctx);ctx.measurementMode.snapshot.activeMode=SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
    ctx.measurementMode.snapshot.generation=90;ctx.rowModeProfile.generation=99;
    runHomogeneous(&ctx,ESP_ERR_INVALID_CRC,4231,ctx.measurementMode.snapshot);
    CHECK(strstr(s_emitted,"mode=RES,modeGen=3,profileGen=7,rowGen=2,rowReq=2"));
    reset(&ctx);ctx.frame.mixedProfile=true;ctx.rowModeProfile.generation=99;
    memcpy(ctx.frame.identity.profile,"CVRCVRCV",9);runMixed(&ctx,ESP_ERR_INVALID_CRC,4231);
    CHECK(strstr(s_emitted,"mode=MIXED,modeGen=3,profileGen=7,rowGen=2,rowReq=2,profile=CVRCVRCV,route=RES"));
    CHECK(s_safeCalls==1);
    reset(&ctx);ctx.measurementMode.snapshot.generation=90;ctx.rowModeProfile.generation=99;
    sensorarrayAdsFaultEvent_t event={.stage=SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE,.seq=4232,.err=ESP_ERR_INVALID_CRC};
    sensorarrayAdsFaultFillContext(&ctx,&event);
    CHECK(event.modeGeneration==90 && event.profileGeneration==99 && event.rowGeneration==2 && event.rowRequestId==2 && event.profile==NULL);
    event.seq=4231;ctx.frame.identity.sessionId++;
    sensorarrayAdsFaultFillContext(&ctx,&event);
    CHECK(event.modeGeneration==90 && event.profileGeneration==99 && event.profile==NULL);
    reset(&ctx);ctx.measurementMode.snapshot.generation=90;ctx.rowModeProfile.generation=99;
    runCheckRestore(&ctx,ESP_ERR_INVALID_RESPONSE,1234);
    CHECK(strstr(s_emitted,"mode=RES,modeGen=90,profileGen=99,rowGen=2,rowReq=2,profile=unknown,route=RES"));
    CHECK(strstr(s_emitted,"cfgGen=47") && s_safeCalls==1);
    reset(&ctx);ctx.adsEngine.captured=(sensorarrayAdsMatrixFaultContext_t){
        .valid=true,.sequence=4231,.stage=SENSORARRAY_ADS_FAULT_STAGE_MATRIX_CRC,
        .mode="RES",.route="RES",.owner="MATRIX",.originalError=ESP_ERR_INVALID_CRC,
        .cleanupError=ESP_ERR_TIMEOUT,.attempted=true,
    };
    /* Engine already requested SAFE, but its first-error context is retained. */
    ctx.routeController.snapshot.safe=true;
    runHomogeneous(&ctx,ESP_FAIL,4231,ctx.measurementMode.snapshot);
    CHECK(strstr(s_emitted,"stage=MATRIX_CRC,err=0x109"));
    CHECK(strstr(s_emitted,"mode=RES,modeGen=3,profileGen=7,rowGen=2,rowReq=2,profile=RRRRRRRR,route=RES"));
    CHECK(strstr(s_emitted,"originalError=0x109,cleanupError=0x107"));
    reset(&ctx);ctx.adsEngine.captured=(sensorarrayAdsMatrixFaultContext_t){
        .valid=true,.sequence=4230,.stage=SENSORARRAY_ADS_FAULT_STAGE_RESET_OBSERVED,
        .mode="VOLT",.route="VOLT",.owner="MATRIX",.originalError=ESP_ERR_INVALID_RESPONSE,
    };
    runHomogeneous(&ctx,ESP_ERR_INVALID_CRC,4231,ctx.measurementMode.snapshot);
    CHECK(strstr(s_emitted,"stage=MATRIX_ROUTE,err=0x109") && !strstr(s_emitted,"ADSFAULTCTX"));
    reset(&ctx);event=(sensorarrayAdsFaultEvent_t){.stage=SENSORARRAY_ADS_FAULT_STAGE_MATRIX_ROUTE,.seq=4231};
    sensorarrayAdsFaultFillContext(&ctx,&event);
    CHECK(event.profile==ctx.frame.identity.profile);
    ctx.frame.identity.revision=0;ctx.rowModeProfile.generation=99;
    sensorarrayAdsFaultFillContext(&ctx,&event);
    CHECK(event.profile==NULL && event.profileGeneration==99);
    puts("MAIN_FAULT_CONTEXT,passed=10,source=production_functions_and_cleanup_branches");
    return 0;
}
