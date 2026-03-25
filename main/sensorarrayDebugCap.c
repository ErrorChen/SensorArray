#include "sensorarrayDebugCap.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

typedef struct {
    const char *mode;
    uint8_t sColumn;
    uint8_t dLine;
    sensorarrayPath_t path;
    const sensorarrayRouteMap_t *route;
    const sensorarrayFdcDLineMap_t *fdcMap;
    sensorarrayFdcDeviceState_t *fdcState;
    BoardSupportI2cBusInfo_t busInfo;
    tmux1108Source_t swSource;
    int selaWriteLevel;
    int selaReadLevel;
} sensorarrayDebugCapContext_t;

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static const char *sensorarrayDebugCapDeviceName(sensorarrayFdcDeviceId_t devId)
{
    return (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? "secondary" : "primary";
}

static void sensorarrayDebugCapCaptureReadback(sensorarrayDebugCapContext_t *ctx)
{
    if (!ctx) {
        return;
    }

    ctx->selaReadLevel = -1;
    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) == ESP_OK) {
        ctx->selaReadLevel = ctrl.selaLevel;
    }
}

static void sensorarrayDebugCapLogCfg(const sensorarrayDebugCapContext_t *ctx, esp_err_t err, const char *status)
{
    printf("DBGCAPCFG,mode=%s,sColumn=%u,dLine=%u,path=%s,secondaryBusEnabled=%u,i2cPort=%d,sda=%d,scl=%d,"
           "i2cAddr=0x%02X,fdcDev=%s,channel=%u,discardFirst=%u,sampleCount=%u,loopDelayMs=%u,err=%ld,status=%s\n",
           ctx && ctx->mode ? ctx->mode : SENSORARRAY_NA,
           ctx ? (unsigned)ctx->sColumn : 0u,
           ctx ? (unsigned)ctx->dLine : 0u,
           ctx ? sensorarrayBoardMapPathName(ctx->path) : SENSORARRAY_NA,
           (ctx && ctx->busInfo.Enabled) ? 1u : 0u,
           ctx ? (int)ctx->busInfo.Port : -1,
           ctx ? ctx->busInfo.SdaGpio : -1,
           ctx ? ctx->busInfo.SclGpio : -1,
           ctx && ctx->fdcState ? ctx->fdcState->i2cAddr : 0u,
           (ctx && ctx->fdcMap) ? sensorarrayDebugCapDeviceName(ctx->fdcMap->devId) : SENSORARRAY_NA,
           (ctx && ctx->fdcMap) ? (unsigned)ctx->fdcMap->channel : 0u,
           (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_DISCARD_FIRST != 0) ? 1u : 0u,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_SAMPLE_COUNT,
           (unsigned)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOOP_DELAY_MS,
           (long)err,
           status ? status : SENSORARRAY_NA);
}

static void sensorarrayDebugCapLogRoute(const sensorarrayDebugCapContext_t *ctx, esp_err_t err, const char *status)
{
    printf("DBGCAPROUTE,mode=%s,sColumn=%u,dLine=%u,path=%s,selaRoute=%s,selaWriteLevel=%d,selaReadLevel=%d,"
           "selBLevel=%u,swSource=%s,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,channel=%u,err=%ld,status=%s\n",
           ctx && ctx->mode ? ctx->mode : SENSORARRAY_NA,
           ctx ? (unsigned)ctx->sColumn : 0u,
           ctx ? (unsigned)ctx->dLine : 0u,
           ctx ? sensorarrayBoardMapPathName(ctx->path) : SENSORARRAY_NA,
           (ctx && ctx->route) ? sensorarrayBoardMapSelaRouteName(ctx->route->selaRoute) : SENSORARRAY_NA,
           ctx ? ctx->selaWriteLevel : -1,
           ctx ? ctx->selaReadLevel : -1,
           (ctx && ctx->route && ctx->route->selBLevel) ? 1u : 0u,
           (ctx && ctx->route) ? sensorarrayLogSwSourceLogicalName(ctx->swSource) : SENSORARRAY_NA,
           (ctx && ctx->fdcMap) ? sensorarrayDebugCapDeviceName(ctx->fdcMap->devId) : SENSORARRAY_NA,
           ctx ? (int)ctx->busInfo.Port : -1,
           ctx && ctx->fdcState ? ctx->fdcState->i2cAddr : 0u,
           (ctx && ctx->fdcMap) ? (unsigned)ctx->fdcMap->channel : 0u,
           (long)err,
           status ? status : SENSORARRAY_NA);
}

static void sensorarrayDebugCapLogSample(const sensorarrayDebugCapContext_t *ctx,
                                         const Fdc2214CapSample_t *sample,
                                         bool discardFirst,
                                         esp_err_t err,
                                         const char *status)
{
    printf("DBGCAP,mode=%s,sColumn=%u,dLine=%u,path=%s,selaRoute=%s,selaWriteLevel=%d,selaReadLevel=%d,"
           "selBLevel=%u,swSource=%s,fdcDev=%s,i2cPort=%d,i2cAddr=0x%02X,channel=%u,discardFirst=%u,raw=%lu,"
           "watchdog=%u,amplitude=%u,err=%ld,status=%s\n",
           ctx && ctx->mode ? ctx->mode : SENSORARRAY_NA,
           ctx ? (unsigned)ctx->sColumn : 0u,
           ctx ? (unsigned)ctx->dLine : 0u,
           ctx ? sensorarrayBoardMapPathName(ctx->path) : SENSORARRAY_NA,
           (ctx && ctx->route) ? sensorarrayBoardMapSelaRouteName(ctx->route->selaRoute) : SENSORARRAY_NA,
           ctx ? ctx->selaWriteLevel : -1,
           ctx ? ctx->selaReadLevel : -1,
           (ctx && ctx->route && ctx->route->selBLevel) ? 1u : 0u,
           (ctx && ctx->route) ? sensorarrayLogSwSourceLogicalName(ctx->swSource) : SENSORARRAY_NA,
           (ctx && ctx->fdcMap) ? sensorarrayDebugCapDeviceName(ctx->fdcMap->devId) : SENSORARRAY_NA,
           ctx ? (int)ctx->busInfo.Port : -1,
           ctx && ctx->fdcState ? ctx->fdcState->i2cAddr : 0u,
           (ctx && ctx->fdcMap) ? (unsigned)ctx->fdcMap->channel : 0u,
           discardFirst ? 1u : 0u,
           (sample && err == ESP_OK) ? (unsigned long)sample->Raw28 : 0ul,
           (sample && err == ESP_OK && sample->ErrWatchdog) ? 1u : 0u,
           (sample && err == ESP_OK && sample->ErrAmplitude) ? 1u : 0u,
           (long)err,
           status ? status : SENSORARRAY_NA);
}

static void sensorarrayDebugCapHold(const char *reason)
{
    sensorarrayDebugIdleForever(reason);
}

void sensorarrayDebugRunS5D5CapFdcSecondaryMode(sensorarrayState_t *state)
{
    sensorarrayDebugCapContext_t ctx = {
        .mode = "S5D5_CAP_FDC_SECONDARY",
        .sColumn = (uint8_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_S_COLUMN,
        .dLine = (uint8_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_D_LINE,
        .path = SENSORARRAY_PATH_CAPACITIVE,
        .route = NULL,
        .fdcMap = NULL,
        .fdcState = NULL,
        .busInfo = {0},
        .swSource = TMUX1108_SOURCE_REF,
        .selaWriteLevel = -1,
        .selaReadLevel = -1,
    };

    (void)boardSupportGetI2cBusInfo(true, &ctx.busInfo);

    ctx.route = sensorarrayBoardMapFindRoute(ctx.sColumn, ctx.dLine, ctx.path);
    ctx.fdcMap = sensorarrayBoardMapFindFdcByDLine(ctx.dLine);
    if (ctx.fdcMap) {
        ctx.fdcState = sensorarrayMeasureGetFdcState(state, ctx.fdcMap->devId);
    }

    sensorarrayDebugCapLogCfg(&ctx, ESP_OK, "check_begin");

    if (!ctx.route) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_NOT_SUPPORTED, "route_map_missing");
        sensorarrayDebugCapHold("cap_route_map_missing");
        return;
    }
    if (!ctx.busInfo.Enabled) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_NOT_SUPPORTED, "secondary_bus_missing");
        sensorarrayDebugCapHold("cap_secondary_bus_missing");
        return;
    }
    if (ctx.busInfo.SdaGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO ||
        ctx.busInfo.SclGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_INVALID_STATE, "secondary_bus_pins_invalid");
        sensorarrayDebugCapHold("cap_secondary_bus_pins_invalid");
        return;
    }
    if (!ctx.fdcMap || ctx.fdcMap->devId != SENSORARRAY_FDC_DEV_SECONDARY) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_INVALID_STATE, "d5_not_mapped_to_secondary");
        sensorarrayDebugCapHold("cap_d5_not_secondary");
        return;
    }
    if (ctx.fdcMap->channel != FDC2214_CH0) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_INVALID_STATE, "d5_not_mapped_to_ch0");
        sensorarrayDebugCapHold("cap_d5_not_ch0");
        return;
    }
    if (!ctx.fdcState || !ctx.fdcState->i2cCtx) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_INVALID_STATE, "secondary_bus_missing");
        sensorarrayDebugCapHold("cap_secondary_bus_ctx_missing");
        return;
    }
    if (ctx.fdcState->i2cAddr != SENSORARRAY_FDC_I2C_ADDR_LOW) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_INVALID_STATE, "secondary_addr_invalid");
        sensorarrayDebugCapHold("cap_secondary_addr_invalid");
        return;
    }
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(ctx.route->selaRoute, &ctx.selaWriteLevel) ||
        ctx.route->selaRoute != SENSORARRAY_SELA_ROUTE_FDC2214) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_INVALID_STATE, "sela_route_invalid");
        sensorarrayDebugCapHold("cap_sela_route_invalid");
        return;
    }

    ctx.swSource = sensorarrayBoardMapDefaultSwSource(ctx.route);
    if (!ctx.fdcState->ready || !ctx.fdcState->handle) {
        sensorarrayDebugCapLogCfg(&ctx, ESP_ERR_INVALID_STATE, "fdc_handle_not_ready");
        sensorarrayDebugCapHold("cap_fdc_handle_not_ready");
        return;
    }

    sensorarrayDebugCapLogCfg(&ctx, ESP_OK, "ok");

    const char *routeLabel = SENSORARRAY_NA;
    esp_err_t err = sensorarrayMeasureApplyRoute(state, ctx.sColumn, ctx.dLine, ctx.path, ctx.swSource, &routeLabel);
    (void)routeLabel;
    sensorarrayDebugCapCaptureReadback(&ctx);
    if (err != ESP_OK) {
        sensorarrayDebugCapLogRoute(&ctx, err, "route_apply_failure");
        sensorarrayDebugCapHold("cap_route_apply_failure");
        return;
    }

    sensorarrayDebugCapLogRoute(&ctx, ESP_OK, "route_locked_static");

    while (true) {
        for (uint8_t sampleIndex = 0; sampleIndex < (uint8_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_SAMPLE_COUNT;
             ++sampleIndex) {
            const bool discardFirst =
                (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_DISCARD_FIRST != 0) && (sampleIndex == 0u);
            Fdc2214CapSample_t sample = {0};
            err = sensorarrayMeasureReadFdcSample(ctx.fdcState->handle, ctx.fdcMap->channel, discardFirst, &sample);
            sensorarrayDebugCapCaptureReadback(&ctx);
            if (err != ESP_OK) {
                sensorarrayDebugCapLogSample(&ctx, &sample, discardFirst, err, "fdc_read_error");
                sensorarrayDebugCapHold("cap_fdc_read_error");
                return;
            }

            const char *status =
                (sample.ErrWatchdog || sample.ErrAmplitude) ? "sample_flagged" : "ok";
            sensorarrayDebugCapLogSample(&ctx, &sample, discardFirst, ESP_OK, status);
        }

        sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOOP_DELAY_MS);
    }
}
