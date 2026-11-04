#include <stdio.h>
#include <string.h>
#include "sensorarrayConfig.h"
#include "sensorarrayAdsGap.h"

/* Synthetic regression of repaired-cap1-recovery-01.rx.bin: an expired rail
 * rejects admission without a conversion, even though the old rail snapshot
 * still contains valid/OK. This is not a fabricated raw capture. */
#define CHECK(x) do { if (!(x)) { fprintf(stderr,"FAIL,line=%d,%s\n",__LINE__,#x);return 1; } } while (0)
/* INSERT_PRODUCTION_TYPES */
static sensorarrayAdsGapSnapshot_t s_snapshot;
static sensorarrayAdsRailCalibration_t s_railCalibration;
static int64_t esp_timer_get_time(void) { return 999000; }
/* INSERT_PRODUCTION_AINCOM */
/* INSERT_PRODUCTION_REASON */
/* INSERT_PRODUCTION_VALIDITY */

static sensorarrayAdsBatteryDiag_t reset(void)
{
    s_snapshot = (sensorarrayAdsGapSnapshot_t){
        .bootCalibrationDone=true,.batteryRestoreOk=true,.vbiasEnabled=true,
        .railUsableForBattery=true,.railValid=true,.railStatus=SENSORARRAY_ADS_RAIL_STATUS_OK,
        .railUv=5100000,.ain8DiffUv=1250000,.batteryFresh=false,
        .batteryLastGoodValid=true,.batteryLastGoodMv=4055,
        .batteryLastGoodTimestampUs=123456,.batteryLastGoodFrame=42,
    };
    s_railCalibration = (sensorarrayAdsRailCalibration_t){
        .source=SENSORARRAY_ADS_RAIL_SOURCE_EXTERNAL_CALIBRATION,
        .externalAvddUv=3300000,.externalAvssUv=-1800000,
    };
    return (sensorarrayAdsBatteryDiag_t){.restoreOk=true,.reason="rail_invalid"};
}

static bool lastGoodUnchanged(void)
{
    return s_snapshot.batteryLastGoodValid && s_snapshot.batteryLastGoodMv==4055 &&
        s_snapshot.batteryLastGoodTimestampUs==123456 && s_snapshot.batteryLastGoodFrame==42;
}

int main(void)
{
    sensorarrayAdsBatteryDiag_t diag=reset();
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_STATE,&diag,4386);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_RAIL);
    CHECK(!s_snapshot.batteryValid && !s_snapshot.batteryFresh && s_snapshot.batteryMv==-1);
    CHECK(!s_snapshot.aincomGndValid && !s_snapshot.ain8GndValid && lastGoodUnchanged());

    diag=reset();diag.reason="snapshot_read";
    sensorarrayAdsUpdateBatteryValidity(ESP_FAIL,&diag,4387);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_SPI_ERROR && lastGoodUnchanged());

    diag=reset();
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_CRC,&diag,4388);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_SPI_ERROR && lastGoodUnchanged());

    diag=reset();diag.restoreOk=false;diag.cleanupError=ESP_ERR_TIMEOUT;
    s_snapshot.batteryRestoreOk=false;
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_STATE,&diag,4389);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_RESTORE_FAILED && lastGoodUnchanged());

    diag=reset();diag.cleanupError=ESP_ERR_INVALID_RESPONSE;
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_STATE,&diag,4390);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_SPI_ERROR && lastGoodUnchanged());

    diag=reset();
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_TIMEOUT,&diag,4391);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_ADC_TIMEOUT && lastGoodUnchanged());

    diag=reset();diag.reason="owner_busy";
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_STATE,&diag,4392);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_SPI_ERROR && lastGoodUnchanged());

    diag=reset();diag.reason=NULL;
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_STATE,&diag,4393);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_SPI_ERROR && lastGoodUnchanged());

    diag=reset();s_snapshot.bootCalibrationDone=false;
    sensorarrayAdsUpdateBatteryValidity(ESP_ERR_INVALID_STATE,&diag,4394);
    CHECK(s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_CAL && lastGoodUnchanged());

    diag=reset();diag.reason="ok";diag.fresh=true;diag.referenceValid=true;
    s_snapshot.batteryFresh=true;
    sensorarrayAdsUpdateBatteryValidity(ESP_OK,&diag,4395);
    CHECK(s_snapshot.batteryValid && s_snapshot.batteryInvalidReason==SENSORARRAY_BATTERY_INVALID_NONE);
    CHECK(s_snapshot.batteryLastGoodMv==4000 && s_snapshot.batteryLastGoodTimestampUs==999000 &&
          s_snapshot.batteryLastGoodFrame==4395);
    puts("BATTERY_VALIDITY,passed=10,source=production_classification_and_math");
    return 0;
}
