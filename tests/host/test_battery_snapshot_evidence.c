#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "sensorarrayAdsGap.h"

/* Synthetic branch tests motivated by real crc-fixed-res8-01.rx.bin seq413.
 * That capture cannot distinguish reset_observed from shadow mismatch. */
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL,line=%d,%s\n",__LINE__,#x);return 1; } } while (0)
/* INSERT_PRODUCTION_TYPES */
static ads126xAdc1RegisterSnapshot_t s_physical;
static esp_err_t s_readError;
static sensorarrayAdsGapSnapshot_t s_snapshot;
static struct { uint32_t id; } s_batteryTransaction;
static char s_output[4096];
static size_t s_outputLength,s_maxRecord;
static int capturePrintf(const char *format,...)
{
    va_list args;va_start(args,format);
    int written=vsnprintf(s_output+s_outputLength,sizeof(s_output)-s_outputLength,format,args);
    va_end(args);
    if (written>0 && (size_t)written<sizeof(s_output)-s_outputLength) {
        s_outputLength+=(size_t)written;
        if ((size_t)written>s_maxRecord)s_maxRecord=(size_t)written;
    }
    return written;
}
esp_err_t ads126xAdcReadAdc1RegisterSnapshot(ads126xAdcHandle_t *ads,ads126xAdc1RegisterSnapshot_t *out)
{ (void)ads;if(s_readError==ESP_OK)*out=s_physical;return s_readError; }
#define printf capturePrintf
/* INSERT_PRODUCTION_FUNCTIONS */
static void emitBatteryError(sensorarrayAdsBatteryDiag_t diag,esp_err_t err,uint32_t frameSequence)
{
/* INSERT_PRODUCTION_BATERR */
}
#undef printf

static sensorarrayAdsBatteryDiag_t reset(ads126xAdcHandle_t *ads,sensorarrayAdsRegisterCache_t *cache)
{
    memset(ads,0,sizeof(*ads));memset(cache,0,sizeof(*cache));
    s_physical=(ads126xAdc1RegisterSnapshot_t){.id=1,.power=3,.interface=6,.mode2=0x8F,
        .inpmux=0x7A,.fullScaleCal={0,0,0x40}};
    ads->idRegRaw=1;
    cache->vrefValid=true;cache->vrefUv=2500000;cache->adc1RunningValid=true;
    const uint8_t values[SENSORARRAY_ADS_REGISTER_COUNT]={
        [SENSORARRAY_ADS_REGISTER_POWER]=3,[SENSORARRAY_ADS_REGISTER_INTERFACE]=6,
        [SENSORARRAY_ADS_REGISTER_MODE2]=0x8F,[SENSORARRAY_ADS_REGISTER_INPMUX]=0x7A,
        [SENSORARRAY_ADS_REGISTER_FSCAL2]=0x40,
    };
    for(unsigned i=0;i<SENSORARRAY_ADS_REGISTER_COUNT;i++) {
        cache->registers[i].valid=true;cache->registers[i].value=values[i];
    }
    s_readError=ESP_OK;s_output[0]='\0';s_outputLength=0;s_maxRecord=0;
    s_batteryTransaction.id=UINT32_MAX;
    s_snapshot=(sensorarrayAdsGapSnapshot_t){.batteryLastGoodMv=3952,.batteryLastGoodValid=true};
    return (sensorarrayAdsBatteryDiag_t){.restoreOk=true,.reason="begin"};
}

int main(void)
{
    ads126xAdcHandle_t ads;sensorarrayAdsRegisterCache_t cache;
    ads126xAdc1RegisterSnapshot_t saved;
    sensorarrayAdsBatteryDiag_t diag=reset(&ads,&cache);
    CHECK(sensorarrayAdsBatterySaveVerifiedState(&ads,&cache,&saved,&diag)==ESP_OK);
    CHECK(diag.shadowKnown && diag.savedKnown && diag.restoreOk && !diag.firstCompare.valid);

    /* Reserved bits may differ; all documented config bits still matter. */
    diag=reset(&ads,&cache);s_physical.power=0xEF;s_physical.interface=0xF6;s_physical.refmux=0xC0;
    CHECK(sensorarrayAdsBatterySaveVerifiedState(&ads,&cache,&saved,&diag)==ESP_OK);
    CHECK(diag.saved.power==0xEF && !diag.firstCompare.valid);

    diag=reset(&ads,&cache);s_physical.refmux=1;
    CHECK(sensorarrayAdsBatterySaveVerifiedState(&ads,&cache,&saved,&diag)==ESP_ERR_INVALID_RESPONSE);
    CHECK(!strcmp(diag.reason,"snapshot_shadow_mismatch") && diag.savedKnown && diag.shadowKnown);
    CHECK(diag.saved.refmux==1 && diag.shadow.refmux==0 && !diag.restoreOk && diag.cleanupError==ESP_OK);
    CHECK(!diag.hardwareModified && !diag.restoreAttempted && diag.firstCompare.reg==0x0F &&
          diag.firstCompare.expected==0 && diag.firstCompare.actual==1 && diag.firstCompare.mask==0x3F);
    diag.readErr=ESP_ERR_INVALID_RESPONSE;
    sensorarrayAdsLogBatteryEvidence(413,&diag);
    sensorarrayAdsLogBatteryRegisters(413,"saved",&diag.saved,510095,diag.savedKnown);
    emitBatteryError(diag,diag.readErr,413);
    CHECK(strstr(s_output,"stage=snapshot_shadow_mismatch") && strstr(s_output,"modified=0,restoreAttempted=0,restore=unknown"));
    CHECK(strstr(s_output,"reg=0F,expected=00,actual=01,mask=3F") && strstr(s_output,"phase=saved,known=1"));
    CHECK(strstr(s_output,"restore=fail,action=isolate_ads,restoreState=unknown") && !strstr(s_output,"report_continue"));

    diag=reset(&ads,&cache);s_physical.power=0x13;s_physical.id=99;
    CHECK(sensorarrayAdsBatterySaveVerifiedState(&ads,&cache,&saved,&diag)==ESP_ERR_INVALID_RESPONSE);
    CHECK(!strcmp(diag.reason,"reset_observed") && diag.firstCompare.reg==1 && diag.firstCompare.mask==0x10);
    CHECK(diag.firstCompare.expected==3 && diag.firstCompare.actual==0x13 && diag.savedKnown && !diag.restoreAttempted);

    diag=reset(&ads,&cache);s_readError=ESP_ERR_TIMEOUT;ads.snapshotReadFailure=1;
    CHECK(sensorarrayAdsBatterySaveVerifiedState(&ads,&cache,&saved,&diag)==ESP_ERR_TIMEOUT);
    CHECK(!strcmp(diag.reason,"snapshot_read") && !diag.savedKnown && !diag.firstCompare.valid && diag.cleanupError==ESP_OK);

    diag=reset(&ads,&cache);s_readError=ESP_ERR_INVALID_RESPONSE;ads.snapshotReadFailure=2;
    ads.snapshotMismatchRegister=0x0F;ads.snapshotExpected=0;ads.snapshotActual=1;
    CHECK(sensorarrayAdsBatterySaveVerifiedState(&ads,&cache,&saved,&diag)==ESP_ERR_INVALID_RESPONSE);
    CHECK(!strcmp(diag.reason,"snapshot_inconsistent") && !diag.savedKnown && diag.firstCompare.mask==0x3F);

    /* Every non-reserved field fails, and a later comparison cannot erase the first. */
    diag=reset(&ads,&cache);ads126xAdc1RegisterSnapshot_t expected=s_physical;
    uint8_t *fields[]={&s_physical.id,&s_physical.power,&s_physical.interface,&s_physical.mode0,
        &s_physical.mode1,&s_physical.mode2,&s_physical.inpmux,&s_physical.offsetCal[0],
        &s_physical.offsetCal[1],&s_physical.offsetCal[2],&s_physical.fullScaleCal[0],
        &s_physical.fullScaleCal[1],&s_physical.fullScaleCal[2],&s_physical.refmux};
    const uint8_t regs[]={0,1,2,3,4,5,6,7,8,9,10,11,12,15};
    for(unsigned i=0;i<sizeof(fields)/sizeof(fields[0]);i++) {
        s_physical=expected;*fields[i]^=1;diag.firstCompare=(sensorarrayAdsBatteryCompare_t){0};
        CHECK(!sensorarrayAdsBatterySnapshotsMatch(&expected,&s_physical,&diag.firstCompare));
        CHECK(diag.firstCompare.reg==regs[i]);
        s_physical.mode0^=2;
        CHECK(!sensorarrayAdsBatterySnapshotsMatch(&expected,&s_physical,&diag.firstCompare));
        CHECK(diag.firstCompare.reg==regs[i]);
    }

    diag=reset(&ads,&cache);diag.restoreOk=false;diag.restoreAttempted=true;diag.hardwareModified=true;
    diag.reason="fresh_conversion";diag.cleanupReason="restore_snapshot_inconsistent";
    diag.readErr=ESP_ERR_INVALID_CRC;diag.cleanupError=ESP_ERR_INVALID_RESPONSE;
    diag.cleanupCompare=(sensorarrayAdsBatteryCompare_t){.valid=true,.reg=0x0F,.expected=0,.actual=1,.mask=0x3F};
    sensorarrayAdsLogBatteryEvidence(UINT32_MAX,&diag);emitBatteryError(diag,diag.readErr,UINT32_MAX);
    CHECK(strstr(s_output,"stage=fresh_conversion") && strstr(s_output,"originalError=0x109,cleanupError=0x108"));
    CHECK(strstr(s_output,"phase=restore,reg=0F,expected=00,actual=01,mask=3F") &&
          strstr(s_output,"restore=fail,action=isolate_ads,restoreState=failed") && s_maxRecord<384);

    diag=reset(&ads,&cache);diag.restoreOk=true;diag.restoreAttempted=true;diag.hardwareModified=true;
    emitBatteryError(diag,ESP_ERR_INVALID_RESPONSE,UINT32_MAX);
    CHECK(strstr(s_output,"restore=ok,action=report_continue,restoreState=verified"));
    diag.restoreAttempted=false;
    CHECK(!strcmp(sensorarrayAdsBatteryRestoreName(&diag),"unchanged"));
    puts("BATTERY_SNAPSHOT_EVIDENCE,passed=9,compared_fields=14,source=production_admission_comparison_logging");
    return 0;
}
