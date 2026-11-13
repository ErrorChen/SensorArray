/* Actual MeasureCellImpl, adaptive reader, cache, range and math execute here.
 * ADC wire conversions and profile writes are deterministic hardware stubs. */
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayAdsMatrix.h"
#include "sensorarrayAdsMath.h"
#include "sensorarrayBoardMap.h"
/* CONFIGURATION */
/* TYPES */
/* CONSTANTS */
static sensorarrayState_t state;
static sensorarrayAdsMatrixEngine_t engine;
static sensorarrayAdsRailSplit_t rail;
static int32_t samples[3];
static unsigned readCount, crcAt, passed;
static uint32_t drdy;
static const int32_t baseRaw = -700000000;
int64_t esp_timer_get_time(void) { static int64_t now=1000; return now+=10; }
bool ads126xAdcIsQuarantined(const ads126xAdcHandle_t *ads) { (void)ads; return false; }
uint32_t ads126xAdcGetDrdyGeneration(const ads126xAdcHandle_t *ads)
{ (void)ads; return drdy; }
/* RAW_UV */
esp_err_t ads126xAdcReadAdc1FreshChecked(ads126xAdcHandle_t *ads, uint32_t generation,
    uint32_t timeout, uint8_t *retry, int32_t *raw, uint8_t *status,
    ads126xAdcFreshReadReport_t *report)
{
    (void)ads; (void)timeout; assert(retry && readCount < 3u);
    *report=(ads126xAdcFreshReadReport_t){.generationStart=generation,.generationEnd=++drdy};
    readCount++;
    if(crcAt && readCount==crcAt) {
        report->firstError=report->terminalError=ESP_ERR_INVALID_CRC;
        report->crcErrors=2; report->crcRetries=1; *retry=0;
        return ESP_ERR_INVALID_CRC;
    }
    *raw=samples[readCount-1u]; *status=ADS126X_STATUS_ADC1_NEW_DATA;
    return ESP_OK;
}
bool sensorarrayBoardMapAdsMuxForDLine(uint8_t d, uint8_t *p, uint8_t *n)
{ assert(d==1u); *p=0u; *n=10u; return true; }
static esp_err_t sensorarrayAdsMatrixApplyInputProfile(sensorarrayAdsMatrixEngine_t *e,
    uint8_t p, uint8_t n, sensorarrayAdsInputMode_t mode, uint8_t gain,
    sensorarrayMeasurementPayload_t *telemetry, bool force, uint32_t *generation)
{
    (void)p; (void)n; (void)mode; (void)telemetry; (void)force;
    e->state->ads.pgaGain=gain; *generation=drdy; return ESP_OK;
}
static esp_err_t sensorarrayAdsMatrixConfirmHighZ(sensorarrayAdsMatrixEngine_t *e,
    const sensorarrayAdsRailSplit_t *r, uint8_t p, uint8_t n,
    sensorarrayAdsInputMode_t mode, uint8_t gain, sensorarrayMeasurementPayload_t *telemetry,
    sensorarrayAdsMatrixCellResult_t *sample)
{
    (void)e;(void)r;(void)p;(void)n;(void)mode;(void)gain;(void)telemetry;(void)sample;
    assert(!"fixture is well above the high-Z/open threshold"); return ESP_FAIL;
}
/* GENERATION */
/* FRESH */
/* SPREAD */
/* ADAPTIVE */
/* IO_ERROR */
/* INVALIDATE */
/* MEASURE_CELL_IMPL */

static sensorarrayAdsCellValueCache_t *cache(void)
{ return sensorarrayAdsValueCacheGet(&engine.valueCache,engine.mode,0u); }
static void reset(sensorarrayMeasurementMode_t mode, sensorarrayAdsInputMode_t input)
{
    memset(&state,0,sizeof(state)); memset(&engine,0,sizeof(engine));
    engine.state=&state; engine.mode=mode; engine.frameCount=16u;
    state.ads.configurationGeneration=1u; state.ads.pgaGain=1u; state.ads.vrefMicrovolts=5145000u;
    rail=(sensorarrayAdsRailSplit_t){.valid=true,.avddUv=3350000,.avssUv=-1795000,.aincomUv=777500};
    sensorarrayAdsProfileCacheInit(&engine.profileCache);
    sensorarrayAdsValueCacheInit(&engine.valueCache);
    assert(sensorarrayAdsProfileCacheStore(&engine.profileCache,mode,0u,input,1u,15u));
    for(unsigned i=0;i<CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_STABLE_STREAK;i++)
        sensorarrayAdsValueCacheObserve(cache(),baseRaw,0,0,1000u,15u,true);
    readCount=crcAt=0u; drdy=1u;
}
static esp_err_t measure(bool precision, bool unstable, bool medianFirst,
                         sensorarrayAdsMatrixCellResult_t *result)
{
    int32_t delta=unstable?2500000:1000;
    samples[0]=medianFirst?baseRaw:baseRaw-delta;
    samples[1]=medianFirst?baseRaw+delta:baseRaw;
    samples[2]=medianFirst?baseRaw-delta:baseRaw+delta;
    sensorarrayMeasurementPayload_t telemetry={0}; readCount=0u;
    engine.cellCrcRetryBudget=CONFIG_SENSORARRAY_ADS_MATRIX_IO_RETRY_COUNT;
    engine.frameCount++;
    return sensorarrayAdsMatrixMeasureCellImpl(&engine,&rail,1u,1u,precision,false,&telemetry,result);
}
static void unstableFollowup(sensorarrayMeasurementMode_t mode, sensorarrayAdsInputMode_t input)
{
    reset(mode,input);
    sensorarrayAdsMatrixCellResult_t result={0};
    assert(measure(true,true,false,&result)==ESP_ERR_INVALID_RESPONSE);
    assert(result.fresh && !result.stable && result.fullSample && result.error==SENSORARRAY_CELL_ERROR_UNSTABLE);
    assert(readCount==3u && result.gainChanges==0u);
    sensorarrayAdsCellProfile_t profile={0};
    assert(sensorarrayAdsProfileCacheGet(&engine.profileCache,mode,0u,&profile));
    assert(profile.failureStreak==1u);
    uint32_t priorStreak=cache()->stableStreak;
    esp_err_t followup=measure(false,true,true,&result);
#if defined(EXPECT_PGA_STABILITY_BUG)
    if(input==SENSORARRAY_ADS_INPUT_PGA) {
        assert(priorStreak>=CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_STABLE_STREAK);
        assert(followup==ESP_OK && result.stable && !result.fullSample && readCount==1u);
        printf("ADS_UNSTABLE_BASELINE_REPRODUCED,mode=%u,priorStableStreak=%lu,followupReads=1,falseFastAccept=1\n",
               (unsigned)mode,(unsigned long)priorStreak);
    } else
#endif
    {
        assert(priorStreak==0u);
        assert(followup==ESP_ERR_INVALID_RESPONSE && !result.stable && result.fullSample && readCount==3u);
        assert(result.error==SENSORARRAY_CELL_ERROR_UNSTABLE && cache()->stableStreak==0u);
    }
    passed++;
}
#if !defined(EXPECT_PGA_STABILITY_BUG)
static void recovery(sensorarrayMeasurementMode_t mode)
{
    reset(mode,SENSORARRAY_ADS_INPUT_PGA);
    sensorarrayAdsMatrixCellResult_t result={0};
    assert(measure(true,true,false,&result)==ESP_ERR_INVALID_RESPONSE);
    for(unsigned i=1;i<=CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_STABLE_STREAK;i++) {
        assert(measure(false,false,true,&result)==ESP_OK);
        assert(result.stable && result.fullSample && readCount==3u && cache()->stableStreak==i);
    }
    assert(measure(false,false,true,&result)==ESP_OK);
    assert(result.stable && !result.fullSample && readCount==1u);
    passed++;
}
#endif
int main(void)
{
    assert(CONFIG_SENSORARRAY_ADS_MATRIX_MAX_SPREAD_UV==5000u);
    assert(CONFIG_SENSORARRAY_ADS_PROFILE_FAILURE_INVALIDATE_COUNT==2u);
    assert(CONFIG_SENSORARRAY_ADS_ADAPTIVE_MIN_STABLE_STREAK==2u);
    for(unsigned mode=SENSORARRAY_MEASUREMENT_MODE_VOLTAGE;mode<=SENSORARRAY_MEASUREMENT_MODE_RESISTANCE;mode++) {
        unstableFollowup((sensorarrayMeasurementMode_t)mode,SENSORARRAY_ADS_INPUT_PGA);
        unstableFollowup((sensorarrayMeasurementMode_t)mode,SENSORARRAY_ADS_INPUT_BYPASS);
#if !defined(EXPECT_PGA_STABILITY_BUG)
        recovery((sensorarrayMeasurementMode_t)mode);
#endif
    }
    for(unsigned input=SENSORARRAY_ADS_INPUT_PGA;input<=SENSORARRAY_ADS_INPUT_BYPASS;input++) {
        for(unsigned bad=1;bad<=2;bad++) {
            reset(SENSORARRAY_MEASUREMENT_MODE_VOLTAGE,(sensorarrayAdsInputMode_t)input);
            sensorarrayAdsCellValueCache_t before=*cache();
            sensorarrayAdsMatrixCellResult_t result={0}; crcAt=bad;
            assert(measure(true,false,true,&result)==ESP_ERR_INVALID_CRC);
            assert(memcmp(&before,cache(),sizeof(before))==0 && readCount==bad);
            passed++;
        }
    }
    printf("ADS_CELL_STABILITY_PASS,scenarios=%u,production=MeasureCellImpl_ReadAdaptive_ReadFreshAfter_Cache_AutoRange_Math,hardware=STUB\n",passed);
    return 0;
}
