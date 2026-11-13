#include <assert.h>
#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayAdsMatrix.h"
#include "sensorarrayAdsGap.h"
#include "sensorarrayAdsMath.h"
#include "sensorarrayFrameBuilder.h"

#define CONFIG_SENSORARRAY_ADS_DRDY_TIMEOUT_US 4000u
#define CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_RAW_THRESHOLD 100u
#define CONFIG_SENSORARRAY_ADS_ADAPTIVE_NOISE_MULTIPLIER 3u
#define CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_STABLE_STREAK 3u
#define CONFIG_SENSORARRAY_ADS_MATRIX_MAX_SPREAD_UV 100u
#define CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT 1u
#define CONFIG_SENSORARRAY_ADS_PRECISION_FRAME_INTERVAL 64u
#define CONFIG_SENSORARRAY_ADS_REGISTER_HEALTH_INTERVAL 64u
#define CONFIG_SENSORARRAY_ADS_AVDD_TO_GND_UV 3300000
#define CONFIG_SENSORARRAY_ADS_AVSS_TO_GND_UV 1800000
#define CONFIG_SENSORARRAY_ADS_PROFILE_RAIL_INVALIDATE_UV 10000u
#define CONFIG_SENSORARRAY_ADS_PROFILE_RAIL_HYSTERESIS_UV 1000u
#define SENSORARRAY_ADS_MATRIX_ALL_CRC_REJECTED_LIMIT 3u

/* Synthetic boundary outcomes; real driver deadline/DMA/CRC parser has its
 * independent suite. Real ReadFreshAfter, ReadAdaptive, MeasureCell wrapper,
 * entire ReadFrame, FinishFrame and budget execute here, not copies. */
/* TYPES */
static sensorarrayState_t state;
static sensorarrayRouteController_t route;
static sensorarrayAdsMatrixEngine_t engine;
static sensorarrayFrame_t frame;
static sensorarrayScanPlan_t plan;
enum scenario { GOOD, CRC_FIRST, CRC_LATER, CRC_TIMEOUT, CRC_PHYSICAL_CHANGE,
    CRC_CACHE_CHANGE, CRC_QUARANTINE, RESET_SAMPLE, SPI_ERROR, OWNER_ERROR, OPEN_SAMPLE };
static enum scenario cells[64];
static unsigned reads[64], calls, rowsSelected;
static unsigned failRow;
static bool quarantined, healthFailure;
static unsigned passed;
static const sensorarrayAdsVoltageLimits_t s_sensorarrayAdsVoltageLimits = {
    .minimumUv=-10000000,.maximumUv=10000000};
static const sensorarrayAdsResistanceConfig_t s_sensorarrayAdsResistanceLimits = {
    .referenceResistorOhms=10000,.minimumOhms=0,.maximumOhms=100000000};
int64_t esp_timer_get_time(void) { static int64_t now=1000;return now+=100; }
bool ads126xAdcIsQuarantined(const ads126xAdcHandle_t *ads)
{ (void)ads;return quarantined; }
uint32_t ads126xAdcGetDrdyGeneration(const ads126xAdcHandle_t *ads)
{ (void)ads;return calls+1; }
int32_t ads126xAdcRawToMicrovolts(const ads126xAdcHandle_t *ads,int32_t raw)
{ (void)ads;return raw; }
esp_err_t ads126xAdcBeginTransaction(ads126xAdcHandle_t *ads,uint32_t budget)
{ (void)ads;assert(budget==100000u);return cells[engine.currentCell]==OWNER_ERROR?ESP_ERR_TIMEOUT:ESP_OK; }
void ads126xAdcEndTransaction(ads126xAdcHandle_t *ads) { (void)ads; }
esp_err_t ads126xAdcReadAdc1FreshChecked(ads126xAdcHandle_t *ads,uint32_t generation,
    uint32_t timeout,uint8_t *retry,int32_t *raw,uint8_t *status,ads126xAdcFreshReadReport_t *report)
{
    (void)timeout;assert(retry);unsigned cell=engine.currentCell;
    unsigned sample=++reads[cell];calls++;
    *report=(ads126xAdcFreshReadReport_t){.generationStart=generation,.generationEnd=generation+1};
    enum scenario s=cells[cell];
    bool bad=(s==CRC_LATER?sample==2:s>=CRC_FIRST && s<=CRC_QUARANTINE);
    if(bad) {
        report->firstError=ESP_ERR_INVALID_CRC;
        report->terminalError=s==CRC_TIMEOUT?ESP_ERR_TIMEOUT:ESP_ERR_INVALID_CRC;
        report->crcErrors=2;report->crcRetries=1;*retry=0;
        if(s==CRC_PHYSICAL_CHANGE)ads->configurationGeneration++;
        if(s==CRC_CACHE_CHANGE)engine.valueCache.generation++;
        if(s==CRC_QUARANTINE){quarantined=true;report->terminalError=ESP_ERR_TIMEOUT;}
        return ESP_ERR_INVALID_CRC;
    }
    if(s==SPI_ERROR){report->firstError=report->terminalError=ESP_FAIL;return ESP_FAIL;}
    *raw=1000;*status=s==RESET_SAMPLE?ADS126X_STATUS_RESET_OCCURRED:0;
    return ESP_OK;
}
/* CONFIG */
/* FRESH */
/* SPREAD */
/* ADAPTIVE */
static esp_err_t sensorarrayAdsMatrixMeasureCellImpl(sensorarrayAdsMatrixEngine_t *e,
    const sensorarrayAdsRailSplit_t *rail,uint8_t row,uint8_t column,bool precision,
    bool transition,sensorarrayMeasurementPayload_t *telemetry,sensorarrayAdsMatrixCellResult_t *out)
{
    (void)rail;(void)row;(void)column;memset(out,0,sizeof(*out));out->attempts=1;
    esp_err_t err=sensorarrayAdsMatrixReadAdaptive(e,1,NULL,false,true,precision,transition,telemetry,out);
    out->gain=1;out->nodeUv=out->differentialUv;out->pgaBypassed=true;
    if(cells[e->currentCell]==OPEN_SAMPLE && err==ESP_OK) {
        out->error=SENSORARRAY_CELL_ERROR_OPEN;return ESP_ERR_INVALID_RESPONSE;
    }
    if(err!=ESP_OK)out->error=SENSORARRAY_CELL_ERROR_SPI;
    if((out->status&ADS126X_STATUS_RESET_OCCURRED)!=0) {
        out->error=SENSORARRAY_CELL_ERROR_READBACK;return ESP_ERR_INVALID_RESPONSE;
    }
    return err;
}
static void sensorarrayAdsMatrixCaptureFault(sensorarrayAdsMatrixEngine_t *e,
    sensorarrayAdsFaultStage_t stage,esp_err_t err,uint8_t reg,uint8_t expected,uint8_t actual,uint8_t mask)
{ (void)reg;(void)expected;(void)actual;(void)mask;
  if(!e->faultContext.valid){e->faultContext.valid=true;e->faultContext.stage=stage;e->faultContext.originalError=err;} }
/* CELL */
/* INVALID */
/* VALID */
/* FINISH */
/* DISPOSITION */
/* MODE */
uint32_t sensorarrayAdsGapTransactionGeneration(void) { return 0; }
bool sensorarrayRouteControllerCopySnapshot(const sensorarrayRouteController_t *r,sensorarrayRouteSnapshot_t *out)
{ *out=r->snapshot;return true; }
esp_err_t sensorarrayRouteControllerSelectRow(sensorarrayRouteController_t *r,uint8_t row)
{ (void)r;rowsSelected++;return row==failRow?ESP_ERR_INVALID_RESPONSE:ESP_OK; }
esp_err_t sensorarrayRouteControllerEnterSafe(sensorarrayRouteController_t *r,const char *why)
{ (void)why;r->snapshot.safe=true;return ESP_OK; }
void sensorarrayRouteControllerUpdateRailSnapshot(sensorarrayRouteController_t *r,const sensorarrayAdsRailSplit_t *rail)
{ (void)r;(void)rail; }
void sensorarrayAdsGapNoteFirstValidFrame(uint64_t us) { (void)us; }
static esp_err_t sensorarrayAdsMatrixGetRail(sensorarrayAdsMatrixEngine_t *e,
    sensorarrayAdsRailSplit_t *rail,sensorarrayAdsGapSnapshot_t *gap,esp_err_t *cleanup,bool allow)
{
    (void)e;(void)allow;*cleanup=ESP_OK;
    *rail=(sensorarrayAdsRailSplit_t){.valid=true,.avddUv=3300000,.avssUv=-1800000,.aincomUv=750000};
    *gap=(sensorarrayAdsGapSnapshot_t){.railValid=true,.railUv=5100000,
        .railSource=SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION};
    return ESP_OK;
}
static esp_err_t sensorarrayAdsMatrixRefreshRegisterShadow(sensorarrayAdsMatrixEngine_t *e,
    sensorarrayMeasurementPayload_t *telemetry)
{ (void)e;(void)telemetry;return healthFailure?ESP_ERR_INVALID_RESPONSE:ESP_OK; }
int sensorarrayAcqEventPrintf(const char *format,...)
{ (void)format;return 0; }
static void sensorarrayAdsMatrixLogCrcEvidence(sensorarrayAdsMatrixEngine_t *e,uint32_t seq,esp_err_t error)
{ (void)e;(void)seq;(void)error; }
static sensorarrayCellError_t sensorarrayAdsMatrixIoError(esp_err_t err)
{ return err==ESP_ERR_TIMEOUT?SENSORARRAY_CELL_ERROR_DRDY_TIMEOUT:SENSORARRAY_CELL_ERROR_SPI; }
/* FRAME */
static void reset(uint8_t rows,uint8_t mask)
{
    memset(&state,0,sizeof(state));memset(&engine,0,sizeof(engine));memset(&route,0,sizeof(route));
    memset(&plan,0,sizeof(plan));memset(cells,0,sizeof(cells));memset(reads,0,sizeof(reads));
    calls=rowsSelected=failRow=0;quarantined=healthFailure=false;
    engine.state=&state;engine.routeController=&route;engine.mode=SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;
    engine.registerCache.registers[SENSORARRAY_ADS_REGISTER_MODE0].valid=true;
    engine.frameSequenceHint=100;
    engine.calibration.matrixReferenceSpanUv=2500000;engine.calibration.referenceResistorOhms=10000;
    route.snapshot=(sensorarrayRouteSnapshot_t){.mode=engine.mode,.gpioReadbackValid=true,.adsReadbackValid=true};
    plan.rowCount=rows;plan.configSnapshot.rows=rows;plan.configSnapshot.rowMask=mask;
}
static esp_err_t run(void)
{ memset(reads,0,sizeof(reads));return sensorarrayAdsMatrixEngineReadFrame(&engine,&plan,&frame); }
static void checkLocal(bool fresh)
{
    assert(frame.boundedCrcFailure && frame.freshFrame==fresh);
    assert(frame.measurement.valuesFixed[0]==SENSORARRAY_MEASUREMENT_INVALID_FIXED);
    assert(frame.measurement.errorReason[0]==SENSORARRAY_CELL_ERROR_SPI);
    assert((frame.measurement.validMask&1u)==0u);
    assert(((frame.acquiredMask&1u)!=0u)==fresh);
    assert(((frame.measurement.freshMask&1u)!=0u)==fresh);
}
int main(void)
{
    reset(1,1);cells[0]=CRC_FIRST;assert(run()==ESP_ERR_INVALID_CRC);checkLocal(false);assert(calls==22);passed++;
    reset(1,1);cells[0]=CRC_LATER;assert(run()==ESP_ERR_INVALID_CRC);checkLocal(true);passed++;
    for(enum scenario s=CRC_TIMEOUT;s<=CRC_QUARANTINE;s++) {
        reset(8,255);cells[0]=s;assert(run()==ESP_ERR_INVALID_CRC);
        assert(!frame.boundedCrcFailure && !frame.freshFrame && calls==1 && rowsSelected==1);
        assert(engine.cellHardFailure);passed++;
    }
    reset(8,255);cells[0]=CRC_FIRST;cells[1]=RESET_SAMPLE;assert(run()==ESP_ERR_INVALID_CRC);
    assert(!frame.boundedCrcFailure && !frame.freshFrame && calls==4 && rowsSelected==1);
    assert(engine.faultContext.originalError==ESP_ERR_INVALID_CRC);passed++;
    for(enum scenario s=SPI_ERROR;s<=OWNER_ERROR;s++) {
        reset(8,255);cells[0]=CRC_FIRST;cells[1]=s;assert(run()==ESP_ERR_INVALID_CRC);
        assert(!frame.boundedCrcFailure && !frame.freshFrame && rowsSelected==1);passed++;
    }
    reset(8,255);cells[0]=CRC_FIRST;failRow=2;assert(run()==ESP_ERR_INVALID_CRC);
    assert(!frame.boundedCrcFailure && !frame.freshFrame && rowsSelected==2);passed++;
    for(unsigned late=0;late<2;late++)for(unsigned sparse=0;sparse<3;sparse++) {
        reset(sparse?8:1,sparse==2?0x81:sparse?0xFF:1);
        for(unsigned i=0;i<64;i++)cells[i]=late?CRC_LATER:CRC_FIRST;
        assert(run()==ESP_ERR_INVALID_CRC && frame.boundedCrcFailure && engine.allCrcRejectedSweeps==1);
        assert(sensorarrayAdsMatrixEngineSetMode(&engine,SENSORARRAY_MEASUREMENT_MODE_NONE)==ESP_OK);
        assert(sensorarrayAdsMatrixEngineSetMode(&engine,SENSORARRAY_MEASUREMENT_MODE_VOLTAGE)==ESP_OK);
        assert(run()==ESP_ERR_INVALID_CRC && frame.boundedCrcFailure && engine.allCrcRejectedSweeps==2);
        assert(run()==ESP_ERR_INVALID_CRC && !frame.boundedCrcFailure && !frame.freshFrame);
        assert(engine.allCrcRejectedSweeps==3);passed++;
    }
    reset(1,1);for(unsigned i=0;i<8;i++)cells[i]=OPEN_SAMPLE;
    engine.allCrcRejectedSweeps=2;assert(run()==ESP_OK && frame.freshFrame && !frame.boundedCrcFailure);
    assert(engine.allCrcRejectedSweeps==0 && frame.measurement.validMask==0);passed++;
    reset(1,1);cells[0]=CRC_FIRST;engine.allCrcRejectedSweeps=2;
    assert(run()==ESP_ERR_INVALID_CRC && frame.boundedCrcFailure && engine.allCrcRejectedSweeps==0);passed++;
    reset(1,1);engine.frameCount=63;healthFailure=true;assert(run()==ESP_ERR_INVALID_RESPONSE);
    assert(!frame.boundedCrcFailure && !frame.freshFrame && calls==0);passed++;
    printf("ADS_CRC_DISPOSITION,passed=%u,source=production_readadaptive_cellwrapper_readframe_finish_budget,fixture=synthetic\n",passed);
    return 0;
}
