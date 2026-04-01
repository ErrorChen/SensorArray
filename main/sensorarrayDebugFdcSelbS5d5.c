#include "sensorarrayDebugFdcSelbS5d5.h"

#include <limits.h>
#include <stdio.h>

#include "boardSupport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

#define SENSORARRAY_FDC_DBG_MIN_SAMPLES 20u
#define SENSORARRAY_FDC_DBG_MAX_SAMPLES 100u
#define SENSORARRAY_FDC_DBG_SAMPLE_DELAY_MS 20u

#define SENSORARRAY_FDC_REG_DATA_MSB_BASE 0x00u
#define SENSORARRAY_FDC_REG_DATA_LSB_BASE 0x01u
#define SENSORARRAY_FDC_DATA_MSB_MASK 0x0FFFu
#define SENSORARRAY_FDC_DATA_ERR_WD_MASK (1u << 13)
#define SENSORARRAY_FDC_DATA_ERR_AW_MASK (1u << 12)

typedef struct {
    const sensorarrayRouteMap_t *route;
    const sensorarrayFdcDLineMap_t *fdcMap;
    sensorarrayFdcDeviceState_t *fdcState;
    BoardSupportI2cBusInfo_t busInfo;
    tmux1108Source_t swSource;
    uint8_t rowIndex;
    int selaWriteLevel;
    uint8_t i2cAddr;
    uint32_t sampleCount;
} sensorarrayFdcSelbS5d5Ctx_t;

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static const char *sensorarrayFdcChannelName(Fdc2214CapChannel_t ch)
{
    switch (ch) {
    case FDC2214_CH0:
        return "CH0";
    case FDC2214_CH1:
        return "CH1";
    case FDC2214_CH2:
        return "CH2";
    case FDC2214_CH3:
        return "CH3";
    default:
        return "INVALID";
    }
}

static uint8_t sensorarrayFdcRegDataMsb(Fdc2214CapChannel_t ch)
{
    return (uint8_t)(SENSORARRAY_FDC_REG_DATA_MSB_BASE + ((uint8_t)ch * 2u));
}

static uint8_t sensorarrayFdcRegDataLsb(Fdc2214CapChannel_t ch)
{
    return (uint8_t)(SENSORARRAY_FDC_REG_DATA_LSB_BASE + ((uint8_t)ch * 2u));
}

static uint32_t sensorarrayFdcSampleCountForBringup(void)
{
    uint32_t count = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_SAMPLE_COUNT;
    if (count < SENSORARRAY_FDC_DBG_MIN_SAMPLES) {
        count = SENSORARRAY_FDC_DBG_MIN_SAMPLES;
    }
    if (count > SENSORARRAY_FDC_DBG_MAX_SAMPLES) {
        count = SENSORARRAY_FDC_DBG_MAX_SAMPLES;
    }
    return count;
}

static void sensorarrayLogFinalDbg(const sensorarrayFdcSelbS5d5Ctx_t *ctx,
                                   bool rawValid,
                                   uint16_t rawMsb,
                                   uint16_t rawLsb,
                                   uint32_t raw28,
                                   bool errWd,
                                   bool errAw,
                                   const char *status)
{
    char rawMsbBuf[12];
    char rawLsbBuf[12];
    char rawBuf[20];
    char wdBuf[4];
    char awBuf[4];

    const char *mapLabel = (ctx && ctx->route && ctx->route->mapLabel) ? ctx->route->mapLabel : SENSORARRAY_NA;
    int port = (ctx && ctx->fdcState && ctx->fdcState->i2cCtx) ? (int)ctx->fdcState->i2cCtx->Port : -1;
    uint8_t addr = (ctx && ctx->fdcState) ? ctx->fdcState->i2cAddr : SENSORARRAY_FDC_I2C_ADDR_LOW;
    Fdc2214CapChannel_t channel = (ctx && ctx->fdcMap) ? ctx->fdcMap->channel : FDC2214_CH0;

    const char *rawMsbStr = SENSORARRAY_NA;
    const char *rawLsbStr = SENSORARRAY_NA;
    const char *rawStr = SENSORARRAY_NA;
    const char *wdStr = SENSORARRAY_NA;
    const char *awStr = SENSORARRAY_NA;
    if (rawValid) {
        snprintf(rawMsbBuf, sizeof(rawMsbBuf), "0x%04X", rawMsb);
        snprintf(rawLsbBuf, sizeof(rawLsbBuf), "0x%04X", rawLsb);
        snprintf(rawBuf, sizeof(rawBuf), "%lu", (unsigned long)raw28);
        snprintf(wdBuf, sizeof(wdBuf), "%u", errWd ? 1u : 0u);
        snprintf(awBuf, sizeof(awBuf), "%u", errAw ? 1u : 0u);
        rawMsbStr = rawMsbBuf;
        rawLsbStr = rawLsbBuf;
        rawStr = rawBuf;
        wdStr = wdBuf;
        awStr = awBuf;
    }

    printf("DBG,point=S5D5,kind=cap,mode=fdc,map=%s,fdcDev=SELB,i2cPort=%d,i2cAddr=0x%02X,channel=%s,"
           "rawMsb=%s,rawLsb=%s,raw=%s,freqHz=na(reason=missing_fref_clock),"
           "capPf=na(reason=missing_l_or_scale),wd=%s,amp=%s,status=%s\n",
           mapLabel,
           port,
           addr,
           sensorarrayFdcChannelName(channel),
           rawMsbStr,
           rawLsbStr,
           rawStr,
           wdStr,
           awStr,
           status ? status : SENSORARRAY_NA);
}

static esp_err_t sensorarrayReadFdcRawWords(Fdc2214CapDevice_t *dev,
                                            Fdc2214CapChannel_t channel,
                                            uint16_t *outRawMsb,
                                            uint16_t *outRawLsb)
{
    if (!dev || !outRawMsb || !outRawLsb) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = Fdc2214CapReadRawRegisters(dev, sensorarrayFdcRegDataMsb(channel), outRawMsb);
    if (err != ESP_OK) {
        return err;
    }
    return Fdc2214CapReadRawRegisters(dev, sensorarrayFdcRegDataLsb(channel), outRawLsb);
}

static bool sensorarrayVerifyRouteAndLog(const sensorarrayFdcSelbS5d5Ctx_t *ctx)
{
    if (!ctx) {
        return false;
    }

    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) != ESP_OK) {
        printf("DBGFDC,stage=row,point=S5D5,status=route_mismatch,detail=ctrl_read_failed\n");
        return false;
    }

    int expectedA0 = (int)(ctx->rowIndex & 0x1u);
    int expectedA1 = (int)((ctx->rowIndex >> 1u) & 0x1u);
    int expectedA2 = (int)((ctx->rowIndex >> 2u) & 0x1u);
    int expectedSelB = (ctx->route && ctx->route->selBLevel) ? 1 : 0;
    int expectedSw = (ctx->swSource == TMUX1108_SOURCE_REF) ? 1 : 0;

    bool rowOk = (ctrl.a0Level == expectedA0) && (ctrl.a1Level == expectedA1) && (ctrl.a2Level == expectedA2);
    bool selaOk = (ctrl.selaLevel == ctx->selaWriteLevel);
    bool selbOk = (ctrl.selbLevel == expectedSelB);
    bool swOk = (ctrl.source == ctx->swSource) || (ctrl.swLevel == expectedSw);
    bool routeOk = rowOk && selaOk && selbOk && swOk;

    printf("DBGFDC,stage=selB,point=S5D5,row=%u,expectedA0=%d,expectedA1=%d,expectedA2=%d,actualA0=%d,actualA1=%d,"
           "actualA2=%d,expectedSELA=%d,actualSELA=%d,expectedSELB=%d,actualSELB=%d,expectedSW=%d,actualSW=%d,"
           "status=%s\n",
           (unsigned)ctx->rowIndex,
           expectedA0,
           expectedA1,
           expectedA2,
           ctrl.a0Level,
           ctrl.a1Level,
           ctrl.a2Level,
           ctx->selaWriteLevel,
           ctrl.selaLevel,
           expectedSelB,
           ctrl.selbLevel,
           expectedSw,
           ctrl.swLevel,
           routeOk ? "read_ok" : "route_mismatch");

    sensorarrayLogControlGpio("route_verify", "S5D5");
    return routeOk;
}

static void sensorarrayHoldFailure(const char *reason)
{
    sensorarrayDebugIdleForever(reason ? reason : "fdc_selb_s5d5_failure");
}

void sensorarrayDebugRunTestFdc2214SelbS5D5(sensorarrayState_t *state)
{
    sensorarrayFdcSelbS5d5Ctx_t ctx = {
        .route = NULL,
        .fdcMap = NULL,
        .fdcState = NULL,
        .busInfo = {0},
        .swSource = TMUX1108_SOURCE_REF,
        // TMUX1108 uses 0-based row coding. S5 => row index 4 => A2/A1/A0 = 1/0/0.
        .rowIndex = (uint8_t)(SENSORARRAY_S5 - 1u),
        .selaWriteLevel = -1,
        .i2cAddr = SENSORARRAY_FDC_I2C_ADDR_LOW,
        .sampleCount = sensorarrayFdcSampleCountForBringup(),
    };

    printf("DBGFDC,stage=target,point=S5D5,kind=cap,mode=fdc,fdcDev=SELB,sColumn=%u,dLine=%u,i2cAddr=0x%02X,"
           "sda=%d,scl=%d,status=begin\n",
           (unsigned)SENSORARRAY_S5,
           (unsigned)SENSORARRAY_D5,
           ctx.i2cAddr,
           SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO,
           SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO);

    if (!state || !state->boardReady || !state->tmuxReady) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=route_mismatch,detail=state_not_ready\n");
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_state_not_ready");
        return;
    }

    ctx.route = sensorarrayBoardMapFindRoute(SENSORARRAY_S5, SENSORARRAY_D5, SENSORARRAY_PATH_CAPACITIVE);
    ctx.fdcMap = sensorarrayBoardMapFindFdcByDLine(SENSORARRAY_D5);
    ctx.fdcState = sensorarrayMeasureGetFdcState(state, SENSORARRAY_FDC_DEV_SECONDARY);
    if (!ctx.route || !ctx.fdcMap || !ctx.fdcState) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=route_mismatch,detail=route_or_fdc_state_missing\n");
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_map_missing");
        return;
    }

    if (ctx.fdcMap->devId != SENSORARRAY_FDC_DEV_SECONDARY) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=invalid_channel_map,detail=d5_not_secondary\n");
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "invalid_channel_map");
        sensorarrayHoldFailure("fdc_selb_s5d5_invalid_dline_owner");
        return;
    }
    if (ctx.fdcMap->channel != FDC2214_CH0) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=invalid_channel_map,detail=d5_expected_ch0_actual_%u\n",
               (unsigned)ctx.fdcMap->channel);
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "invalid_channel_map");
        sensorarrayHoldFailure("fdc_selb_s5d5_invalid_channel");
        return;
    }

    if (!sensorarrayBoardMapSelaRouteToGpioLevel(ctx.route->selaRoute, &ctx.selaWriteLevel)) {
        printf("DBGFDC,stage=selA,point=S5D5,status=route_mismatch,detail=sela_route_invalid\n");
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_sela_invalid");
        return;
    }
    if (ctx.route->selaRoute != SENSORARRAY_SELA_ROUTE_FDC2214 || !ctx.route->selBLevel) {
        printf("DBGFDC,stage=selA,point=S5D5,status=route_mismatch,detail=expected_cap_route_sela_fdc_selb_high\n");
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_route_semantic_mismatch");
        return;
    }

    ctx.swSource = sensorarrayBoardMapDefaultSwSource(ctx.route);
    if (ctx.swSource != TMUX1108_SOURCE_REF) {
        printf("DBGFDC,stage=sw,point=S5D5,status=route_mismatch,detail=cap_should_use_ref\n");
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_sw_invalid");
        return;
    }

    const BoardSupportI2cCtx_t *secondaryCtx = boardSupportGetI2c1Ctx();
    (void)boardSupportGetI2cBusInfo(true, &ctx.busInfo);
    ctx.fdcState->i2cCtx = secondaryCtx;
    ctx.fdcState->i2cAddr = ctx.i2cAddr;

    if (!ctx.busInfo.Enabled ||
        !ctx.fdcState->i2cCtx ||
        ctx.busInfo.SdaGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO ||
        ctx.busInfo.SclGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,fdcDev=SELB,i2cPort=%d,sda=%d,scl=%d,freqHz=%lu,timeoutMs=%lu,"
               "i2cAddr=0x%02X,status=no_i2c_ack\n",
               (int)ctx.busInfo.Port,
               ctx.busInfo.SdaGpio,
               ctx.busInfo.SclGpio,
               (unsigned long)ctx.busInfo.FrequencyHz,
               (unsigned long)(ctx.fdcState->i2cCtx ? ctx.fdcState->i2cCtx->TimeoutMs : 0u),
               ctx.i2cAddr);
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "no_i2c_ack");
        sensorarrayHoldFailure("fdc_selb_s5d5_i2c_bus_invalid");
        return;
    }

    esp_err_t adsErr = ESP_OK;
    const char *adsStatus = "ads_already_stopped";
    if (state->adsReady && state->adsAdc1Running) {
        adsErr = ads126xAdcStopAdc1(&state->ads);
        if (adsErr == ESP_OK) {
            state->adsAdc1Running = false;
            adsStatus = "ads_stopped";
        } else {
            adsStatus = "ads_stop_failed";
        }
    }
    printf("DBGFDC,stage=ads_isolation,point=S5D5,err=%ld,status=%s\n", (long)adsErr, adsStatus);
    if (adsErr != ESP_OK) {
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_ads_stop_fail");
        return;
    }

    sensorarrayFdcProbeDiag_t probe2A = {0};
    sensorarrayFdcProbeDiag_t probe2B = {0};
    (void)sensorarrayBringupProbeFdcCandidate(ctx.fdcState->i2cCtx, SENSORARRAY_FDC_I2C_ADDR_LOW, &probe2A);
    (void)sensorarrayBringupProbeFdcCandidate(ctx.fdcState->i2cCtx, SENSORARRAY_FDC_I2C_ADDR_HIGH, &probe2B);

    if (probe2A.status == SENSORARRAY_FDC_DISCOVERY_ID_OK && probe2B.status != SENSORARRAY_FDC_DISCOVERY_ID_OK) {
        ctx.i2cAddr = SENSORARRAY_FDC_I2C_ADDR_LOW;
    } else if (probe2B.status == SENSORARRAY_FDC_DISCOVERY_ID_OK &&
               probe2A.status != SENSORARRAY_FDC_DISCOVERY_ID_OK) {
        ctx.i2cAddr = SENSORARRAY_FDC_I2C_ADDR_HIGH;
    } else if (probe2A.ack && !probe2B.ack) {
        ctx.i2cAddr = SENSORARRAY_FDC_I2C_ADDR_LOW;
    } else if (probe2B.ack && !probe2A.ack) {
        ctx.i2cAddr = SENSORARRAY_FDC_I2C_ADDR_HIGH;
    }
    ctx.fdcState->i2cAddr = ctx.i2cAddr;

    char mfg2ABuf[12];
    char mfg2BBuf[12];
    char dev2ABuf[12];
    char dev2BBuf[12];
    const char *mfg2A = probe2A.haveManufacturerId ? (snprintf(mfg2ABuf, sizeof(mfg2ABuf), "0x%04X", probe2A.manufacturerId),
                                                      mfg2ABuf)
                                                    : SENSORARRAY_NA;
    const char *mfg2B = probe2B.haveManufacturerId ? (snprintf(mfg2BBuf, sizeof(mfg2BBuf), "0x%04X", probe2B.manufacturerId),
                                                      mfg2BBuf)
                                                    : SENSORARRAY_NA;
    const char *dev2A = probe2A.haveDeviceId ? (snprintf(dev2ABuf, sizeof(dev2ABuf), "0x%04X", probe2A.deviceId), dev2ABuf)
                                              : SENSORARRAY_NA;
    const char *dev2B = probe2B.haveDeviceId ? (snprintf(dev2BBuf, sizeof(dev2BBuf), "0x%04X", probe2B.deviceId), dev2BBuf)
                                              : SENSORARRAY_NA;

    bool anyAck = probe2A.ack || probe2B.ack;
    bool anyIdOk = (probe2A.status == SENSORARRAY_FDC_DISCOVERY_ID_OK) || (probe2B.status == SENSORARRAY_FDC_DISCOVERY_ID_OK);
    printf("DBGFDC,stage=fdc_probe,point=S5D5,fdcDev=SELB,i2cPort=%d,sda=%d,scl=%d,freqHz=%lu,timeoutMs=%lu,"
           "probe2A=%s,probe2B=%s,mfg2A=%s,mfg2B=%s,dev2A=%s,dev2B=%s,i2cAddr=0x%02X,status=%s\n",
           (int)ctx.fdcState->i2cCtx->Port,
           ctx.busInfo.SdaGpio,
           ctx.busInfo.SclGpio,
           (unsigned long)ctx.busInfo.FrequencyHz,
           (unsigned long)ctx.fdcState->i2cCtx->TimeoutMs,
           sensorarrayBringupFdcDiscoveryStatusName(probe2A.status),
           sensorarrayBringupFdcDiscoveryStatusName(probe2B.status),
           mfg2A,
           mfg2B,
           dev2A,
           dev2B,
           ctx.i2cAddr,
           anyIdOk ? "id_ok" : (anyAck ? "ack_detected" : "no_i2c_ack"));
    if (!anyAck) {
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "no_i2c_ack");
        sensorarrayHoldFailure("fdc_selb_s5d5_no_i2c_ack");
        return;
    }

    uint16_t idMfg = 0u;
    uint16_t idDev = 0u;
    esp_err_t idErr = sensorarrayBringupReadFdcIdsRaw(ctx.fdcState->i2cCtx, ctx.i2cAddr, &idMfg, &idDev);
    bool idOk = (idErr == ESP_OK) &&
                (idMfg == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID) &&
                (idDev == SENSORARRAY_FDC_EXPECTED_DEVICE_ID);
    const char *idStatus = (idErr != ESP_OK) ? "ack_but_read_failed" : (idOk ? "read_ok" : "bad_device_id");
    printf("DBGFDC,stage=fdc_probe,point=S5D5,fdcDev=SELB,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,err=%ld,status=%s\n",
           ctx.i2cAddr,
           idMfg,
           idDev,
           (long)idErr,
           idStatus);
    if (!idOk) {
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, idStatus);
        sensorarrayHoldFailure("fdc_selb_s5d5_bad_device_id");
        return;
    }

    const char *routeLabel = SENSORARRAY_NA;
    esp_err_t routeErr = sensorarrayMeasureApplyRoute(state,
                                                      SENSORARRAY_S5,
                                                      SENSORARRAY_D5,
                                                      SENSORARRAY_PATH_CAPACITIVE,
                                                      ctx.swSource,
                                                      &routeLabel);
    if (routeErr != ESP_OK) {
        printf("DBGFDC,stage=row,point=S5D5,map=%s,err=%ld,status=route_mismatch\n",
               routeLabel ? routeLabel : SENSORARRAY_NA,
               (long)routeErr);
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_route_apply_fail");
        return;
    }

    if (!sensorarrayVerifyRouteAndLog(&ctx)) {
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, "route_mismatch");
        sensorarrayHoldFailure("fdc_selb_s5d5_route_verify_fail");
        return;
    }

    if (ctx.fdcState->handle) {
        (void)Fdc2214CapDestroy(ctx.fdcState->handle);
        ctx.fdcState->handle = NULL;
        ctx.fdcState->ready = false;
    }

    sensorarrayFdcInitDiag_t initDiag = {0};
    esp_err_t initErr = sensorarrayBringupInitFdcSingleChannel(ctx.fdcState->i2cCtx,
                                                                ctx.i2cAddr,
                                                                ctx.fdcMap->channel,
                                                                &ctx.fdcState->handle,
                                                                &initDiag);
    ctx.fdcState->ready = (initErr == ESP_OK);
    ctx.fdcState->haveIds = initDiag.haveIds;
    ctx.fdcState->manufacturerId = initDiag.manufacturerId;
    ctx.fdcState->deviceId = initDiag.deviceId;

    const char *initStatus = "read_ok";
    if (initErr != ESP_OK) {
        initStatus = (initDiag.haveIds &&
                      (initDiag.manufacturerId != SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID ||
                       initDiag.deviceId != SENSORARRAY_FDC_EXPECTED_DEVICE_ID))
                         ? "bad_device_id"
                         : "no_i2c_ack";
    }
    printf("DBGFDC,stage=fdc_init,point=S5D5,fdcDev=SELB,i2cAddr=0x%02X,channel=%s,idMfg=0x%04X,idDev=0x%04X,"
           "detail=%ld,err=%ld,status=%s\n",
           ctx.i2cAddr,
           sensorarrayFdcChannelName(ctx.fdcMap->channel),
           initDiag.manufacturerId,
           initDiag.deviceId,
           (long)initDiag.detail,
           (long)initErr,
           initStatus);
    if (initErr != ESP_OK || !ctx.fdcState->ready || !ctx.fdcState->handle) {
        sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, initStatus);
        sensorarrayHoldFailure("fdc_selb_s5d5_init_fail");
        return;
    }

    uint32_t minRaw = UINT_MAX;
    uint32_t maxRaw = 0u;
    uint64_t sumRaw = 0u;
    uint32_t okCount = 0u;
    bool anyFlagged = false;
    const char *readStatus = "read_ok";

    for (uint32_t i = 0; i < ctx.sampleCount; ++i) {
        uint16_t rawMsb = 0u;
        uint16_t rawLsb = 0u;
        esp_err_t readErr = sensorarrayReadFdcRawWords(ctx.fdcState->handle, ctx.fdcMap->channel, &rawMsb, &rawLsb);
        if (readErr != ESP_OK) {
            readStatus = (readErr == ESP_ERR_TIMEOUT) ? "read_timeout" : "no_i2c_ack";
            printf("DBGFDC,stage=fdc_read,point=S5D5,index=%lu,fdcDev=SELB,i2cAddr=0x%02X,channel=%s,err=%ld,status=%s\n",
                   (unsigned long)i,
                   ctx.i2cAddr,
                   sensorarrayFdcChannelName(ctx.fdcMap->channel),
                   (long)readErr,
                   readStatus);
            sensorarrayLogFinalDbg(&ctx, false, 0, 0, 0, false, false, readStatus);
            break;
        }

        uint32_t raw28 = ((uint32_t)(rawMsb & SENSORARRAY_FDC_DATA_MSB_MASK) << 16) | (uint32_t)rawLsb;
        bool errWd = (rawMsb & SENSORARRAY_FDC_DATA_ERR_WD_MASK) != 0u;
        bool errAw = (rawMsb & SENSORARRAY_FDC_DATA_ERR_AW_MASK) != 0u;
        anyFlagged = anyFlagged || errWd || errAw;
        if (raw28 < minRaw) {
            minRaw = raw28;
        }
        if (raw28 > maxRaw) {
            maxRaw = raw28;
        }
        sumRaw += (uint64_t)raw28;
        okCount++;

        printf("DBGFDC,stage=fdc_read,point=S5D5,index=%lu,fdcDev=SELB,i2cPort=%d,i2cAddr=0x%02X,channel=%s,"
               "rawMsb=0x%04X,rawLsb=0x%04X,raw=%lu,wd=%u,amp=%u,status=read_ok\n",
               (unsigned long)i,
               (int)ctx.fdcState->i2cCtx->Port,
               ctx.i2cAddr,
               sensorarrayFdcChannelName(ctx.fdcMap->channel),
               rawMsb,
               rawLsb,
               (unsigned long)raw28,
               errWd ? 1u : 0u,
               errAw ? 1u : 0u);

        sensorarrayLogFinalDbg(&ctx, true, rawMsb, rawLsb, raw28, errWd, errAw, "read_ok");
        sensorarrayDelayMs(SENSORARRAY_FDC_DBG_SAMPLE_DELAY_MS);
    }

    if (okCount == 0u) {
        sensorarrayHoldFailure("fdc_selb_s5d5_read_zero");
        return;
    }

    uint32_t meanRaw = (uint32_t)(sumRaw / (uint64_t)okCount);
    uint32_t spanRaw = maxRaw - minRaw;
    uint32_t stableThreshold = (meanRaw / 20u) + 1u; // 5% window for bring-up stability check.
    bool readsComplete = (okCount == ctx.sampleCount);
    bool stable = readsComplete && (spanRaw <= stableThreshold) && !anyFlagged;
    bool canCalibrate = stable && idOk;

    printf("DBGFDC,stage=summary,point=S5D5,kind=cap,mode=fdc,fdcDev=SELB,i2cAddr=0x%02X,channel=%s,"
           "samples=%lu,ok=%lu,min=%lu,max=%lu,mean=%lu,span=%lu,stable=%u,calReady=%u,status=%s\n",
           ctx.i2cAddr,
           sensorarrayFdcChannelName(ctx.fdcMap->channel),
           (unsigned long)ctx.sampleCount,
           (unsigned long)okCount,
           (unsigned long)minRaw,
           (unsigned long)maxRaw,
           (unsigned long)meanRaw,
           (unsigned long)spanRaw,
           stable ? 1u : 0u,
           canCalibrate ? 1u : 0u,
           readsComplete ? "read_ok" : readStatus);

    sensorarrayDebugIdleForever(readsComplete ? "fdc_selb_s5d5_done" : "fdc_selb_s5d5_read_incomplete");
}
