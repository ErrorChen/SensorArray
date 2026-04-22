#include "sensorarrayDebug.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebugFdcSelbS5d5.h"
#include "sensorarrayDebugFdcI2cDiscovery.h"
#include "sensorarrayDebugS1d1.h"
#include "sensorarrayDebugSelftest.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayRecovery.h"

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static const char *sensorarrayDebugPathKind(sensorarrayDebugPath_t path)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
        return "cap";
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return "volt";
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
    default:
        return "res";
    }
}

static bool sensorarrayDebugSelaRouteFromGpioLevel(bool selaGpioLevel, sensorarraySelaRoute_t *outRoute)
{
    return sensorarrayBoardMapSelaRouteFromGpioLevel(selaGpioLevel ? 1 : 0, outRoute);
}

static sensorarrayRecoveryStage_t sensorarrayDebugStageToRecoveryStage(const char *stage)
{
    if (!stage) {
        return SENSORARRAY_RECOVERY_STAGE_UNKNOWN;
    }
    if (strcmp(stage, "fdc_init") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_FDC_INIT_BEGIN;
    }
    if (strcmp(stage, "route_apply") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_ROUTE_ROW_ENTER;
    }
    if (strcmp(stage, "row") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_ROUTE_ROW_ENTER;
    }
    if (strcmp(stage, "selA") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_ROUTE_SELA_ENTER;
    }
    if (strcmp(stage, "selB") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_ROUTE_SELB_ENTER;
    }
    if (strcmp(stage, "sw") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_ROUTE_SW_ENTER;
    }
    if (strcmp(stage, "route_verify") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_ROUTE_VERIFY_ENTER;
    }
    if (strcmp(stage, "sweep") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_SWEEP_CANDIDATE_BEGIN;
    }
    if (strcmp(stage, "locked_sample") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_LOCKED_SAMPLE_BEGIN;
    }
    if (strcmp(stage, "recovery") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_RECOVERY_BEGIN;
    }
    if (strcmp(stage, "mode_enter") == 0) {
        return SENSORARRAY_RECOVERY_STAGE_APP_INIT;
    }
    return SENSORARRAY_RECOVERY_STAGE_UNKNOWN;
}

void sensorarrayDebugHandleFatal(const char *reason, esp_err_t err, const char *stage, uint8_t sColumn, uint8_t dLine)
{
    sensorarrayRecoveryStage_t stageId = sensorarrayDebugStageToRecoveryStage(stage);
    sensorarrayRecoveryHandleFatal(reason, err, stageId, sColumn, dLine);
}

void sensorarrayDebugIdleForever(const char *reason)
{
    sensorarrayLogControlGpio("idle", reason);
    sensorarrayDebugHandleFatal(reason ? reason : "debug_idle",
                                ESP_OK,
                                "idle",
                                0u,
                                0u);
}

static esp_err_t sensorarrayDebugApplyFixedRoute(sensorarrayState_t *state,
                                                 const sensorarrayAdsReadPolicy_t *adsPolicy,
                                                 const sensorarrayDebugFixedRoute_t *cfg)
{
    if (!state || !cfg) {
        return ESP_ERR_INVALID_ARG;
    }

    char columnBuf[6];
    char dLineBuf[6];
    char valueBuf[24];
    char uvBuf[24];
    char rawBuf[24];
    char wdBuf[8];
    char ampBuf[8];
    char mapBuf[96];

    snprintf(columnBuf, sizeof(columnBuf), "S%u", (unsigned)cfg->sColumn);
    snprintf(dLineBuf, sizeof(dLineBuf), "D%u", (unsigned)cfg->dLine);

    sensorarrayLogRouteStep("begin",
                            cfg->label,
                            cfg->sColumn,
                            cfg->dLine,
                            cfg->path,
                            cfg->swSource,
                            cfg->selaRoute,
                            cfg->selBLevel,
                            ESP_OK,
                            "apply_fixed_route");

    esp_err_t err = sensorarrayMeasureApplyRouteLevels(state,
                                                       cfg->sColumn,
                                                       cfg->dLine,
                                                       cfg->path,
                                                       cfg->swSource,
                                                       cfg->selaRoute,
                                                       cfg->selBLevel,
                                                       cfg->delayAfterRowMs,
                                                       cfg->delayAfterSelAMs,
                                                       cfg->delayAfterSelBMs,
                                                       cfg->delayAfterSwMs,
                                                       cfg->label);
    if (err != ESP_OK) {
        return err;
    }

    const char *kind = sensorarrayDebugPathKind(cfg->path);
    if (!cfg->skipAdsRead) {
        int32_t raw = 0;
        int32_t uv = 0;
        esp_err_t readErr = sensorarrayMeasureReadAdsUv(state, adsPolicy, cfg->dLine, false, &raw, &uv);
        sensorarrayLogDbg(cfg->label ? cfg->label : "FIXED",
                          kind,
                          columnBuf,
                          dLineBuf,
                          sensorarrayLogSwSourceName(cfg->swSource),
                          "ads_fixed",
                          sensorarrayLogFmtI32(valueBuf, sizeof(valueBuf), readErr == ESP_OK, uv),
                          sensorarrayLogFmtI32(uvBuf, sizeof(uvBuf), readErr == ESP_OK, uv),
                          SENSORARRAY_NA,
                          sensorarrayLogFmtI32(rawBuf, sizeof(rawBuf), readErr == ESP_OK, raw),
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          sensorarrayLogBuildMapLabel(mapBuf, sizeof(mapBuf), cfg->label, SENSORARRAY_NA),
                          readErr,
                          (readErr == ESP_OK) ? "ads_read_ok" : "ads_read_error");
    }

    if (!cfg->skipFdcRead) {
        const sensorarrayFdcDLineMap_t *fdcMap = NULL;
        sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcStateForDLine(state, cfg->dLine, &fdcMap);
        sensorarrayFdcReadDiag_t fdcDiag = {0};
        esp_err_t readErr = ESP_ERR_INVALID_STATE;
        if (fdcState && fdcState->ready && fdcState->handle && fdcMap) {
            readErr = sensorarrayMeasureReadFdcSampleDiag(fdcState->handle,
                                                          fdcMap->channel,
                                                          false,
                                                          fdcState->haveIds,
                                                          fdcState->configVerified,
                                                          &fdcDiag);
        } else if (!fdcMap || !fdcState) {
            readErr = ESP_ERR_INVALID_ARG;
        }

        const char *fdcStatus = (readErr == ESP_OK) ? sensorarrayMeasureFdcSampleStatusName(fdcDiag.statusCode)
                                                     : "i2c_read_error";
        sensorarrayLogDbgExtraCaptureCtrl();
        sensorarrayLogDbg(cfg->label ? cfg->label : "FIXED",
                          kind,
                          columnBuf,
                          dLineBuf,
                          sensorarrayLogSwSourceName(cfg->swSource),
                          "fdc_fixed",
                          sensorarrayLogFmtU32(valueBuf, sizeof(valueBuf), readErr == ESP_OK, fdcDiag.sample.Raw28),
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          sensorarrayLogFmtU32(rawBuf, sizeof(rawBuf), readErr == ESP_OK, fdcDiag.sample.Raw28),
                          sensorarrayLogFmtBool(wdBuf, sizeof(wdBuf), readErr == ESP_OK, fdcDiag.sample.ErrWatchdog),
                          sensorarrayLogFmtBool(ampBuf, sizeof(ampBuf), readErr == ESP_OK, fdcDiag.sample.ErrAmplitude),
                          fdcState ? fdcState->label : SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          sensorarrayLogBuildMapLabel(mapBuf,
                                                      sizeof(mapBuf),
                                                      cfg->label,
                                                      fdcMap ? fdcMap->mapLabel : SENSORARRAY_NA),
                          readErr,
                          fdcStatus);
    }

    if (cfg->holdForever) {
        sensorarrayLogRouteStep("hold_forever",
                                cfg->label,
                                cfg->sColumn,
                                cfg->dLine,
                                cfg->path,
                                cfg->swSource,
                                cfg->selaRoute,
                                cfg->selBLevel,
                                ESP_OK,
                                "fixed_state_latched");
        sensorarrayDebugIdleForever("fixed_state_latched");
        return ESP_OK;
    }

    if (cfg->holdMs > 0u) {
        sensorarrayLogRouteStep("hold_ms",
                                cfg->label,
                                cfg->sColumn,
                                cfg->dLine,
                                cfg->path,
                                cfg->swSource,
                                cfg->selaRoute,
                                cfg->selBLevel,
                                ESP_OK,
                                "holding_final_state");
        sensorarrayDelayMs(cfg->holdMs);
    }

    return ESP_OK;
}

static void sensorarrayDebugReadResistor(sensorarrayState_t *state,
                                         const sensorarrayAdsReadPolicy_t *adsPolicy,
                                         const char *pointLabel,
                                         uint8_t sColumn,
                                         uint8_t dLine)
{
    char valueBuf[24];
    char uvBuf[24];
    char mohmBuf[24];
    char rawBuf[24];
    char mapBuf[72];
    char columnBuf[6];
    char dLineBuf[6];

    esp_err_t err = ESP_OK;
    int32_t uv = 0;
    int32_t raw = 0;
    int32_t mohm = 0;
    bool haveUv = false;
    bool haveRaw = false;
    bool haveMohm = false;
    const char *status = "res_ok";
    const char *routeMap = SENSORARRAY_NA;

    if (!state->adsReady) {
        status = "skip_ads_unavailable";
    } else if (!state->adsRefReady) {
        status = "skip_ref_not_ready";
    } else {
        err = sensorarrayMeasureApplyRoute(state, sColumn, dLine, SENSORARRAY_PATH_RESISTIVE, TMUX1108_SOURCE_GND, &routeMap);
        if (err != ESP_OK) {
            status = (err == ESP_ERR_NOT_SUPPORTED) ? "route_map_missing" : "route_error";
        } else {
            err = sensorarrayMeasureReadAdsUv(state, adsPolicy, dLine, true, &raw, &uv);
            if (err != ESP_OK) {
                status = "ads_read_error";
            } else {
                haveRaw = true;
                haveUv = true;
                sensorarrayResConvertResult_t resResult = sensorarrayMeasureTryResistanceMohm(uv, &mohm);
                if (resResult == SENSORARRAY_RES_CONVERT_OK) {
                    haveMohm = true;
                } else if (resResult == SENSORARRAY_RES_CONVERT_SIGNED_INPUT) {
                    status = "adc_ok_signed_value";
                } else {
                    status = "adc_ok_but_res_model_invalid";
                }
            }
        }
    }

    snprintf(columnBuf, sizeof(columnBuf), "S%u", (unsigned)sColumn);
    snprintf(dLineBuf, sizeof(dLineBuf), "D%u", (unsigned)dLine);

    sensorarrayLogDbg(pointLabel,
                      "res",
                      columnBuf,
                      dLineBuf,
                      "LOW",
                      "ads",
                      sensorarrayLogFmtI32(valueBuf, sizeof(valueBuf), haveMohm, mohm),
                      sensorarrayLogFmtI32(uvBuf, sizeof(uvBuf), haveUv, uv),
                      sensorarrayLogFmtI32(mohmBuf, sizeof(mohmBuf), haveMohm, mohm),
                      sensorarrayLogFmtI32(rawBuf, sizeof(rawBuf), haveRaw, raw),
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      SENSORARRAY_NA,
                      sensorarrayLogBuildMapLabel(mapBuf, sizeof(mapBuf), routeMap, SENSORARRAY_NA),
                      err,
                      status);
}

static sensorarrayDebugPath_t sensorarrayConfiguredFixedPath(void)
{
#if CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_CAPACITIVE
    return SENSORARRAY_DEBUG_PATH_CAPACITIVE;
#elif CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_VOLTAGE
    return SENSORARRAY_DEBUG_PATH_VOLTAGE;
#else
    return SENSORARRAY_DEBUG_PATH_RESISTIVE;
#endif
}

static tmux1108Source_t sensorarrayConfiguredFixedSwSource(void)
{
#if CONFIG_SENSORARRAY_DEBUG_FIXED_SW_SOURCE_REF
    return TMUX1108_SOURCE_REF;
#else
    return TMUX1108_SOURCE_GND;
#endif
}

static void sensorarrayRunBringupLoop(sensorarrayState_t *state, const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    while (true) {
        sensorarrayDebugReadResistor(state, adsPolicy, "S1D1", SENSORARRAY_S1, SENSORARRAY_D1);
        sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_SCAN_LOOP_DELAY_MS);
    }
}

static void sensorarrayRunRouteIdleMode(void)
{
    sensorarrayLogStartup("debug_mode", ESP_OK, "route_idle", 0);
    sensorarrayDebugIdleForever("route_idle");
}

static void sensorarrayRunRouteFixedStateMode(sensorarrayState_t *state, const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    sensorarraySelaRoute_t fixedSelaRoute = SENSORARRAY_SELA_ROUTE_ADS1263;
    if (!sensorarrayDebugSelaRouteFromGpioLevel((CONFIG_SENSORARRAY_DEBUG_FIXED_SELA_LEVEL != 0), &fixedSelaRoute)) {
        sensorarrayLogStartup("route_fixed",
                              ESP_ERR_INVALID_ARG,
                              "fixed_sela_level_invalid",
                              (int32_t)CONFIG_SENSORARRAY_DEBUG_FIXED_SELA_LEVEL);
        sensorarrayDebugHandleFatal("route_fixed_invalid_sela_level",
                                    ESP_ERR_INVALID_ARG,
                                    "route_fixed",
                                    (uint8_t)CONFIG_SENSORARRAY_DEBUG_FIXED_S_COLUMN,
                                    (uint8_t)CONFIG_SENSORARRAY_DEBUG_FIXED_D_LINE);
        return;
    }

    sensorarrayDebugFixedRoute_t cfg = {
        .sColumn = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FIXED_S_COLUMN,
        .dLine = (uint8_t)CONFIG_SENSORARRAY_DEBUG_FIXED_D_LINE,
        .path = sensorarrayConfiguredFixedPath(),
        .swSource = sensorarrayConfiguredFixedSwSource(),
        .selaRoute = fixedSelaRoute,
        .selBLevel = (CONFIG_SENSORARRAY_DEBUG_FIXED_SELB_LEVEL != 0),
        .skipAdsRead = (CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_ADS_READ != 0),
        .skipFdcRead = (CONFIG_SENSORARRAY_DEBUG_FIXED_SKIP_FDC_READ != 0),
        .delayAfterRowMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .delayAfterSelAMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .delayAfterSelBMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .delayAfterSwMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
        .holdForever = (CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_FOREVER != 0),
        .holdMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_FIXED_HOLD_MS,
        .label = "ROUTE_FIXED",
    };

    esp_err_t err = sensorarrayDebugApplyFixedRoute(state, adsPolicy, &cfg);
    sensorarrayLogStartup("route_fixed", err, (err == ESP_OK) ? "done" : "apply_error", (int32_t)cfg.dLine);
    sensorarrayDebugIdleForever("route_fixed_done");
}

static void sensorarrayRunRouteStepOnceMode(sensorarrayState_t *state, const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    for (size_t i = 0; i < sensorarrayBoardMapRouteCount(); ++i) {
        const sensorarrayRouteMap_t *route = sensorarrayBoardMapRouteAt(i);
        if (!route) {
            continue;
        }

        tmux1108Source_t source = sensorarrayBoardMapDefaultSwSource(route);
        if (!sensorarrayBoardMapSelaRouteToGpioLevel(route->selaRoute, &(int){0})) {
            sensorarrayLogStartup("route_step_once", ESP_ERR_INVALID_STATE, "sela_route_invalid", (int32_t)i);
            break;
        }
        sensorarrayDebugFixedRoute_t cfg = {
            .sColumn = route->sColumn,
            .dLine = route->dLine,
            .path = sensorarrayBoardMapPathToDebugPath(route->path, source),
            .swSource = source,
            .selaRoute = route->selaRoute,
            .selBLevel = route->selBLevel,
            .skipAdsRead = (route->path == SENSORARRAY_PATH_CAPACITIVE),
            .skipFdcRead = (route->path != SENSORARRAY_PATH_CAPACITIVE),
            .delayAfterRowMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .delayAfterSelAMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .delayAfterSelBMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .delayAfterSwMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_ROUTE_STEP_DELAY_MS,
            .holdForever = false,
            .holdMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_STEP_ONCE_DELAY_MS,
            .label = route->mapLabel,
        };

        esp_err_t err = sensorarrayDebugApplyFixedRoute(state, adsPolicy, &cfg);
        if (err != ESP_OK) {
            sensorarrayLogStartup("route_step_once", err, "step_error", (int32_t)i);
            break;
        }
    }
    sensorarrayDebugIdleForever("route_step_once_done");
}

void sensorarrayDebugRunSelectedMode(sensorarrayState_t *state,
                                     const sensorarrayAdsReadPolicy_t *adsPolicy,
                                     sensorarrayDebugMode_t mode)
{
    sensorarrayLogStartup("debug_mode", ESP_OK, sensorarrayLogDebugModeName(mode), (int32_t)mode);

    switch (mode) {
    case SENSORARRAY_DEBUG_MODE_ROUTE_IDLE:
        sensorarrayRunRouteIdleMode();
        return;
    case SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE:
        sensorarrayRunRouteFixedStateMode(state, adsPolicy);
        return;
    case SENSORARRAY_DEBUG_MODE_ROUTE_STEP_ONCE:
        sensorarrayRunRouteStepOnceMode(state, adsPolicy);
        return;
    case SENSORARRAY_DEBUG_MODE_ROUTE_SCAN_LOOP:
        sensorarrayRunBringupLoop(state, adsPolicy);
        return;
    case SENSORARRAY_DEBUG_MODE_ADS_SELFTEST:
        sensorarrayDebugRunAdsSelftestModeImpl(state);
        return;
    case SENSORARRAY_DEBUG_MODE_FDC_SELFTEST:
        sensorarrayDebugRunFdcSelftestModeImpl(state);
        return;
    case SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR:
        sensorarrayDebugRunSingleResistorS1D1ModeImpl(state, adsPolicy);
        return;
    case SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY:
        sensorarrayDebugRunTestFdc2214SelbS5D5(state);
        return;
    case SENSORARRAY_DEBUG_MODE_FDC_I2C_DISCOVERY:
        sensorarrayDebugRunFdcI2cDiscoveryMode(state);
        return;
    default:
        sensorarrayRunBringupLoop(state, adsPolicy);
        return;
    }
}
