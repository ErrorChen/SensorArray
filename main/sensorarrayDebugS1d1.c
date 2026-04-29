#include "sensorarrayDebugS1d1.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

static const sensorarrayRouteMap_t *sensorarrayGetS1D1ResRoute(int *outSelaWriteLevel, tmux1108Source_t *outSwSource)
{
    const sensorarrayRouteMap_t *route =
        sensorarrayBoardMapFindRoute(SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE);
    if (!route) {
        return NULL;
    }

    if (outSelaWriteLevel) {
        *outSelaWriteLevel = -1;
        (void)sensorarrayBoardMapSelaRouteToGpioLevel(route->selaRoute, outSelaWriteLevel);
    }
    if (outSwSource) {
        *outSwSource = sensorarrayBoardMapDefaultSwSource(route);
    }
    return route;
}

static void sensorarrayLogS1D1RouteState(const char *mode,
                                         const sensorarrayRouteMap_t *route,
                                         tmux1108Source_t swSource,
                                         int selaWriteLevel,
                                         esp_err_t err,
                                         const char *status)
{
    tmuxSwitchControlState_t ctrl = {0};
    int selaCmdLevel = -1;
    int selaObsLevel = -1;
    if (tmuxSwitchGetControlState(&ctrl) == ESP_OK) {
        selaCmdLevel = ctrl.cmdSelaLevel;
        selaObsLevel = ctrl.obsSelaLevel;
    }

    printf("DBGS1D1CFG,mode=%s,sColumn=%u,dLine=%u,path=%s,selaRoute=%s,selaWriteLevel=%d,selaCmdLevel=%d,selaObsLevel=%d,"
           "selBLevel=%u,swSource=%s,label=%s,err=%ld,status=%s\n",
           mode ? mode : SENSORARRAY_NA,
           (unsigned)SENSORARRAY_S1,
           (unsigned)SENSORARRAY_D1,
           sensorarrayBoardMapPathName(SENSORARRAY_PATH_RESISTIVE),
           route ? sensorarrayBoardMapSelaRouteName(route->selaRoute) : SENSORARRAY_NA,
           selaWriteLevel,
           selaCmdLevel,
           selaObsLevel,
           route ? (route->selBLevel ? 1u : 0u) : 0u,
           sensorarrayLogSwSourceLogicalName(swSource),
           route && route->mapLabel ? route->mapLabel : SENSORARRAY_NA,
           (long)err,
           status ? status : SENSORARRAY_NA);
}

static esp_err_t sensorarrayApplyS1D1ResRoute(sensorarrayState_t *state,
                                              const sensorarrayRouteMap_t **outRoute,
                                              tmux1108Source_t *outSwSource)
{
    int selaWriteLevel = -1;
    tmux1108Source_t swSource = TMUX1108_SOURCE_REF;
    const sensorarrayRouteMap_t *route = sensorarrayGetS1D1ResRoute(&selaWriteLevel, &swSource);
    if (outRoute) {
        *outRoute = route;
    }
    if (outSwSource) {
        *outSwSource = swSource;
    }
    if (!route) {
        sensorarrayLogS1D1RouteState("S1D1_RESISTOR", NULL, swSource, selaWriteLevel, ESP_ERR_NOT_SUPPORTED, "route_map_missing");
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (swSource != TMUX1108_SOURCE_REF) {
        sensorarrayLogS1D1RouteState("S1D1_RESISTOR", route, swSource, selaWriteLevel, ESP_ERR_INVALID_STATE, "sw_source_invalid");
        return ESP_ERR_INVALID_STATE;
    }
    if (selaWriteLevel < 0) {
        sensorarrayLogS1D1RouteState("S1D1_RESISTOR", route, swSource, selaWriteLevel, ESP_ERR_INVALID_STATE, "sela_route_invalid");
        return ESP_ERR_INVALID_STATE;
    }

    const char *mapLabel = SENSORARRAY_NA;
    esp_err_t err = sensorarrayMeasureApplyResistanceRoute(state,
                                                           SENSORARRAY_S1,
                                                           SENSORARRAY_D1,
                                                           &mapLabel);
    (void)mapLabel;
    sensorarrayLogS1D1RouteState("S1D1_RESISTOR",
                                 route,
                                 swSource,
                                 selaWriteLevel,
                                 err,
                                 (err == ESP_OK) ? "route_locked" : "route_apply_failure");
    return err;
}

#if CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_AND_ADS_READ
static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static void sensorarrayRunS1D1AdsLoop(sensorarrayState_t *state, const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    while (true) {
        int32_t raw = 0;
        int32_t uv = 0;
        int32_t mohm = 0;
        char mohmBuf[24];
        bool haveMohm = false;
        esp_err_t err = sensorarrayMeasureReadAdsUv(state,
                                                    adsPolicy,
                                                    SENSORARRAY_D1,
                                                    (CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST != 0),
                                                    &raw,
                                                    &uv);
        const char *status = SENSORARRAY_NA;
        if (err == ESP_OK) {
            status = sensorarrayMeasureDividerModelStatus(uv, &mohm, &haveMohm);
        } else {
            status = "ads_read_error";
        }

        printf("DBGS1D1,mode=S1D1_RESISTOR,sColumn=%u,dLine=%u,path=%s,raw=%ld,uv=%ld,mohm=%s,err=%ld,status=%s\n",
               (unsigned)SENSORARRAY_S1,
               (unsigned)SENSORARRAY_D1,
               sensorarrayBoardMapPathName(SENSORARRAY_PATH_RESISTIVE),
               (long)raw,
               (long)uv,
               haveMohm ? sensorarrayLogFmtI32(mohmBuf, sizeof(mohmBuf), true, mohm) : SENSORARRAY_NA,
               (long)err,
               status);

        if (err != ESP_OK) {
            sensorarrayDebugIdleForever("s1d1_ads_read_error");
            return;
        }

        sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_INTERVAL_MS);
    }
}
#endif

void sensorarrayDebugRunSingleResistorS1D1ModeImpl(sensorarrayState_t *state,
                                                   const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state || !state->tmuxReady) {
        sensorarrayDebugIdleForever("s1d1_tmux_unavailable");
        return;
    }

    const sensorarrayRouteMap_t *route = NULL;
    tmux1108Source_t swSource = TMUX1108_SOURCE_REF;
    esp_err_t err = sensorarrayApplyS1D1ResRoute(state, &route, &swSource);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("s1d1_route_apply_failure");
        return;
    }

#if CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_AND_ADS_READ
    if (!state->adsReady || !state->adsRefReady) {
        sensorarrayLogS1D1RouteState("S1D1_RESISTOR",
                                     route,
                                     swSource,
                                     -1,
                                     ESP_ERR_INVALID_STATE,
                                     "ads_unavailable");
        sensorarrayDebugIdleForever("s1d1_ads_unavailable");
        return;
    }
    sensorarrayRunS1D1AdsLoop(state, adsPolicy);
#else
    (void)route;
    (void)swSource;
    sensorarrayDebugIdleForever("s1d1_route_locked");
#endif
}
