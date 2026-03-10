#include "sensorarrayDebugS1d1.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static esp_err_t sensorarrayLogAdsBootRegisterReadback(sensorarrayState_t *state, const char *step)
{
    sensorarrayAdsRegSnapshot_t regs = {0};
    esp_err_t err = sensorarrayMeasureReadAdsKeyRegisterSnapshot(state, &regs);

    printf("DBGADSBOOT,step=%s,id=0x%02X,power=0x%02X,interface=0x%02X,mode2=0x%02X,refmux=0x%02X,inpmux=0x%02X,"
           "err=%ld,status=%s\n",
           step ? step : "readback_regs",
           regs.id,
           regs.power,
           regs.iface,
           regs.mode2,
           regs.refmux,
           regs.inpmux,
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");
    return err;
}

static esp_err_t sensorarrayLogAdsSelfPair(sensorarrayState_t *state,
                                           const sensorarrayAdsReadPolicy_t *adsPolicy,
                                           const char *pairLabel,
                                           uint8_t muxp,
                                           uint8_t muxn,
                                           bool discardFirst)
{
    int32_t raw = 0;
    int32_t uv = 0;
    uint8_t statusByte = 0u;
    esp_err_t err =
        sensorarrayMeasureReadAdsPairUv(state, adsPolicy, muxp, muxn, discardFirst, &raw, &uv, &statusByte);

    printf("DBGADSSELF,pair=%s,muxp=%u(%s),muxn=%u(%s),raw=%ld,uv=%ld,statusByte=0x%02X,discardFirst=%u,err=%ld,status=%s\n",
           pairLabel ? pairLabel : SENSORARRAY_NA,
           (unsigned)(muxp & 0x0Fu),
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)(muxn & 0x0Fu),
           sensorarrayLogAdsMuxName(muxn),
           (long)raw,
           (long)uv,
           statusByte,
           discardFirst ? 1u : 0u,
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");
    return err;
}

static esp_err_t sensorarrayRunAdsS1D1BootSelftest(sensorarrayState_t *state,
                                                   const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ads126xAdcConfigure(&state->ads, true, true, ADS126X_CRC_OFF, 1, 0);
    printf("DBGADSBOOT,step=configure,enableStatusByte=1,enableInternalRef=1,err=%ld,status=%s\n",
           (long)err,
           (err == ESP_OK) ? "ok" : "configure_error");
    if (err != ESP_OK) {
        return err;
    }

    if (state->adsAdc1Running) {
        err = ads126xAdcStopAdc1(&state->ads);
        printf("DBGADSBOOT,step=stop1,err=%ld,status=%s\n", (long)err, (err == ESP_OK) ? "ok" : "stop_error");
        if (err != ESP_OK) {
            return err;
        }
        state->adsAdc1Running = false;
    }

    err = sensorarrayBringupAdsSetRefMux(state, 0x00u);
    printf("DBGADSBOOT,step=set_refmux,refmux=0x00,err=%ld,status=%s\n",
           (long)err,
           (err == ESP_OK) ? "ok" : "refmux_error");
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayDelayMs(SENSORARRAY_REF_SETTLE_MS);

    err = sensorarrayLogAdsBootRegisterReadback(state, "readback_regs");
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayLogAdsSelfPair(state, adsPolicy, "AINCOM_AINCOM", SENSORARRAY_ADS_MUX_AINCOM, SENSORARRAY_ADS_MUX_AINCOM, true);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayLogAdsSelfPair(state, adsPolicy, "AIN9_AINCOM", SENSORARRAY_ADS_MUX_AIN9, SENSORARRAY_ADS_MUX_AINCOM, true);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayLogAdsSelfPair(state, adsPolicy, "D1_AIN0_AINCOM", 0x00u, SENSORARRAY_ADS_MUX_AINCOM, true);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayLogAdsSelfPair(state, adsPolicy, "AIN8_AINCOM", SENSORARRAY_ADS_MUX_AIN8, SENSORARRAY_ADS_MUX_AINCOM, true);
    return err;
}

static esp_err_t sensorarrayApplyAdsS1D1RawState(sensorarrayState_t *state,
                                                 bool selALevel,
                                                 bool selBLevel,
                                                 tmuxSwitchControlState_t *outCtrl)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(SENSORARRAY_S1 - 1u));
    if (err != ESP_OK) {
        return err;
    }
    err = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
    if (err != ESP_OK) {
        return err;
    }
    err = tmux1134SetEnLogicalState(true);
    if (err != ESP_OK) {
        return err;
    }
    err = tmux1134SelectSelALevel(selALevel);
    if (err != ESP_OK) {
        return err;
    }
    err = tmux1134SelectSelBLevel(selBLevel);
    if (err != ESP_OK) {
        return err;
    }

    if (outCtrl) {
        return tmuxSwitchGetControlState(outCtrl);
    }
    return ESP_OK;
}

static void sensorarrayLogAdsS1D1SweepSample(uint8_t stateIndex,
                                             const char *stateLabel,
                                             bool desiredSelALevel,
                                             bool desiredSelBLevel,
                                             const tmuxSwitchControlState_t *ctrl,
                                             uint8_t muxp,
                                             uint8_t muxn,
                                             int32_t raw,
                                             int32_t uv,
                                             bool haveRead,
                                             uint8_t statusByte,
                                             int32_t mohm,
                                             bool haveMohm,
                                             const char *modelStatus,
                                             esp_err_t err)
{
    char mohmBuf[24];

    printf("DBGADSSWEEP,stateIndex=%u,state=%s,point=S1D1,mode=ads_only,routeLabel=s1d1_ads_only_raw_sweep,"
           "desiredRow=S1,desiredSwSource=GND,desiredSelALevel=%u,desiredSelBLevel=%u,row=%s,rowIndex=%u,"
           "swSource=%s,a0=%d,a1=%d,a2=%d,swLevel=%d,selaLevel=%d,selbLevel=%d,enLevel=%d,muxp=%u(%s),muxn=%u(%s),"
           "raw=%ld,uv=%ld,resMohm=%s,status=route_not_proven,modelStatus=%s,statusByte=0x%02X,err=%ld\n",
           (unsigned)stateIndex,
           stateLabel ? stateLabel : SENSORARRAY_NA,
           desiredSelALevel ? 1u : 0u,
           desiredSelBLevel ? 1u : 0u,
           (ctrl && ctrl->row == (SENSORARRAY_S1 - 1u)) ? "S1" : SENSORARRAY_NA,
           (unsigned)(ctrl ? ctrl->row : 0u),
           (ctrl != NULL) ? sensorarrayLogSwSourceLogicalName(ctrl->source) : SENSORARRAY_NA,
           ctrl ? ctrl->a0Level : -1,
           ctrl ? ctrl->a1Level : -1,
           ctrl ? ctrl->a2Level : -1,
           ctrl ? ctrl->swLevel : -1,
           ctrl ? ctrl->selaLevel : -1,
           ctrl ? ctrl->selbLevel : -1,
           ctrl ? ctrl->enLevel : -1,
           (unsigned)(muxp & 0x0Fu),
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)(muxn & 0x0Fu),
           sensorarrayLogAdsMuxName(muxn),
           haveRead ? (long)raw : 0L,
           haveRead ? (long)uv : 0L,
           sensorarrayLogFmtI32(mohmBuf, sizeof(mohmBuf), haveMohm, mohm),
           modelStatus ? modelStatus : SENSORARRAY_NA,
           statusByte,
           (long)err);
}

void sensorarrayDebugRunAdsS1D1OnlyModeImpl(sensorarrayState_t *state,
                                            const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state || !state->adsReady) {
        sensorarrayLogStartup("ads_s1d1_only", ESP_ERR_INVALID_STATE, "ads_unavailable", 0);
        sensorarrayDebugIdleForever("ads_s1d1_unavailable");
        return;
    }

    esp_err_t err = sensorarrayRunAdsS1D1BootSelftest(state, adsPolicy);
    sensorarrayLogStartup("ads_s1d1_only_selftest", err, (err == ESP_OK) ? "ok" : "selftest_error", 0);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_s1d1_selftest_error");
        return;
    }

    const struct {
        bool selALevel;
        bool selBLevel;
        const char *stateLabel;
    } sweepStates[] = {
        { false, false, "SELA0_SELB0" },
        { true, false, "SELA1_SELB0" },
        { false, true, "SELA0_SELB1" },
        { true, true, "SELA1_SELB1" },
    };

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (!sensorarrayBoardMapAdsMuxForDLine(SENSORARRAY_D1, &muxp, &muxn)) {
        sensorarrayDebugIdleForever("ads_s1d1_mux_invalid");
        return;
    }

    printf("DBGADSBOOT,step=divider_model,refOhms=%u,excitationUv=%u,expectedS1D1Ohms=%u\n",
           (unsigned)SENSORARRAY_RESIST_REF_OHMS,
           (unsigned)SENSORARRAY_RESIST_EXCITATION_UV,
           10000u);

    printf("DBGADSBOOT,step=start_s1d1_sweep,muxp=%u(%s),muxn=%u(%s),holdMs=%u,lockSingleState=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           (unsigned)SENSORARRAY_DEBUG_ADS_S1D1_SWEEP_HOLD_MS,
           (unsigned)(SENSORARRAY_DEBUG_ADS_S1D1_LOCK_TO_SINGLE_STATE != 0));

    while (true) {
        for (uint8_t stateIndex = 0; stateIndex < 4u; ++stateIndex) {
            bool desiredSelA = sweepStates[stateIndex].selALevel;
            bool desiredSelB = sweepStates[stateIndex].selBLevel;
            const char *stateLabel = sweepStates[stateIndex].stateLabel;

            if (SENSORARRAY_DEBUG_ADS_S1D1_LOCK_TO_SINGLE_STATE != 0) {
                desiredSelA = (SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0);
                desiredSelB = (SENSORARRAY_S1D1_DEBUG_SELB_LEVEL != 0);
                stateLabel = "LOCKED_STATE";
            }

            tmuxSwitchControlState_t ctrl = {0};
            err = sensorarrayApplyAdsS1D1RawState(state, desiredSelA, desiredSelB, &ctrl);
            if (err != ESP_OK) {
                sensorarrayLogAdsS1D1SweepSample(stateIndex,
                                                 stateLabel,
                                                 desiredSelA,
                                                 desiredSelB,
                                                 NULL,
                                                 muxp,
                                                 muxn,
                                                 0,
                                                 0,
                                                 false,
                                                 0u,
                                                 0,
                                                 false,
                                                 "route_apply_error",
                                                 err);
                sensorarrayDelayMs(SENSORARRAY_DEBUG_ADS_S1D1_SWEEP_HOLD_MS);
                if (SENSORARRAY_DEBUG_ADS_S1D1_LOCK_TO_SINGLE_STATE != 0) {
                    break;
                }
                continue;
            }

            sensorarrayDelayMs(25u);

            int32_t raw = 0;
            int32_t uv = 0;
            uint8_t statusByte = 0u;
            int32_t mohm = 0;
            bool haveMohm = false;
            esp_err_t readErr = sensorarrayMeasureReadAdsPairUv(state,
                                                                adsPolicy,
                                                                muxp,
                                                                muxn,
                                                                true,
                                                                &raw,
                                                                &uv,
                                                                &statusByte);
            const char *modelStatus = "ads_read_error";
            if (readErr == ESP_OK) {
                modelStatus = sensorarrayMeasureDividerModelStatus(uv, &mohm, &haveMohm);
            }

            sensorarrayLogAdsS1D1SweepSample(stateIndex,
                                             stateLabel,
                                             desiredSelA,
                                             desiredSelB,
                                             &ctrl,
                                             muxp,
                                             muxn,
                                             raw,
                                             uv,
                                             (readErr == ESP_OK),
                                             statusByte,
                                             mohm,
                                             haveMohm,
                                             modelStatus,
                                             readErr);

            sensorarrayDelayMs(SENSORARRAY_DEBUG_ADS_S1D1_SWEEP_HOLD_MS);
            if (SENSORARRAY_DEBUG_ADS_S1D1_LOCK_TO_SINGLE_STATE != 0) {
                break;
            }
        }
    }
}

static bool sensorarrayIsS1D1RouteOnlyBehavior(void)
{
#if CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_AND_ADS_READ
    return false;
#else
    return true;
#endif
}

static bool sensorarrayIsS1D1StaticRouteOnlyBehavior(void)
{
#if CONFIG_SENSORARRAY_S1D1_STATIC_ROUTE_AND_READ
    return false;
#else
    return true;
#endif
}

static const char *sensorarrayS1D1StaticBehaviorName(void)
{
    return sensorarrayIsS1D1StaticRouteOnlyBehavior() ? "ROUTE_ONLY" : "ROUTE_AND_READ";
}

static void sensorarrayLogS1D1ControlState(const char *mapLabel)
{
    char rowBuf[12];
    char a0Buf[8];
    char a1Buf[8];
    char a2Buf[8];
    char swBuf[8];
    char selABuf[8];
    char selBBuf[8];

    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);
    printf("DBGCTRL,point=S1D1,row=%s,a0=%s,a1=%s,a2=%s,sw=%s,selA=%s,selB=%s,map=%s\n",
           sensorarrayLogFmtU8(rowBuf, sizeof(rowBuf), haveCtrl, haveCtrl ? ctrl.row : 0u),
           sensorarrayLogFmtGpioLevel(a0Buf, sizeof(a0Buf), haveCtrl, haveCtrl ? ctrl.a0Level : -1),
           sensorarrayLogFmtGpioLevel(a1Buf, sizeof(a1Buf), haveCtrl, haveCtrl ? ctrl.a1Level : -1),
           sensorarrayLogFmtGpioLevel(a2Buf, sizeof(a2Buf), haveCtrl, haveCtrl ? ctrl.a2Level : -1),
           sensorarrayLogFmtGpioLevel(swBuf, sizeof(swBuf), haveCtrl, haveCtrl ? ctrl.swLevel : -1),
           sensorarrayLogFmtGpioLevel(selABuf, sizeof(selABuf), haveCtrl, haveCtrl ? ctrl.selaLevel : -1),
           sensorarrayLogFmtGpioLevel(selBBuf, sizeof(selBBuf), haveCtrl, haveCtrl ? ctrl.selbLevel : -1),
           mapLabel ? mapLabel : SENSORARRAY_NA);
}

static esp_err_t sensorarrayDebugReadAdsS1D1Once(sensorarrayState_t *state,
                                                 const sensorarrayAdsReadPolicy_t *adsPolicy,
                                                 bool discardFirst,
                                                 int32_t *outRaw,
                                                 int32_t *outUv)
{
    if (!state || !outRaw || !outUv) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    *outRaw = 0;
    *outUv = 0;

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (!sensorarrayBoardMapAdsMuxForDLine(SENSORARRAY_D1, &muxp, &muxn)) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("DBGADS1D1,point=S1D1,step=begin,dLine=1,muxp=%u(%s),muxn=%u(%s),discardFirst=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           discardFirst ? 1u : 0u);

    esp_err_t err = sensorarrayMeasureReadAdsUv(state, adsPolicy, SENSORARRAY_D1, discardFirst, outRaw, outUv);
    printf("DBGADS1D1,point=S1D1,step=done,dLine=1,muxp=%u(%s),muxn=%u(%s),raw=%ld,uv=%ld,discardFirst=%u,err=%ld,status=%s\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           (long)*outRaw,
           (long)*outUv,
           discardFirst ? 1u : 0u,
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");
    return err;
}

void sensorarrayDebugRunSingleResistorS1D1ModeImpl(sensorarrayState_t *state,
                                                   const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state) {
        sensorarrayDebugIdleForever("s1d1_state_null");
        return;
    }

    const sensorarrayRouteMap_t *routeMap =
        sensorarrayBoardMapFindRoute(SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE);
    const char *mapLabel = SENSORARRAY_NA;
    if (!routeMap) {
        sensorarrayLogStartup("s1d1_mode", ESP_ERR_NOT_SUPPORTED, "s1d1_route_missing", 0);
        sensorarrayDebugIdleForever("s1d1_route_missing");
        return;
    }

    esp_err_t err = sensorarrayMeasureApplyRoute(state,
                                                 SENSORARRAY_S1,
                                                 SENSORARRAY_D1,
                                                 SENSORARRAY_PATH_RESISTIVE,
                                                 TMUX1108_SOURCE_GND,
                                                 &mapLabel);
    printf("DBGROUTEFIX,point=S1D1,sColumn=1,row=0,dLine=1,path=res,map=%s,selAReq=%u,selBReq=%u,sw=%s,mode=%s,err=%ld,"
           "status=%s\n",
           mapLabel ? mapLabel : SENSORARRAY_NA,
           routeMap->selALevel ? 1u : 0u,
           routeMap->selBLevel ? 1u : 0u,
           sensorarrayLogSwSourceName(TMUX1108_SOURCE_GND),
           sensorarrayIsS1D1RouteOnlyBehavior() ? "ROUTE_ONLY" : "ROUTE_AND_ADS_READ",
           (long)err,
           (err == ESP_OK) ? "route_applied" : "route_error");
    sensorarrayLogS1D1ControlState(mapLabel);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("s1d1_route_error");
        return;
    }

    if (sensorarrayIsS1D1RouteOnlyBehavior()) {
        sensorarrayDebugIdleForever("s1d1_route_only");
        return;
    }

    while (true) {
        char valueBuf[24];
        char uvBuf[24];
        char rawBuf[24];
        int32_t raw = 0;
        int32_t uv = 0;

        err = sensorarrayDebugReadAdsS1D1Once(state,
                                              adsPolicy,
                                              (CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST != 0),
                                              &raw,
                                              &uv);
        sensorarrayLogDbg("S1D1",
                          "res",
                          "S1",
                          "D1",
                          sensorarrayLogSwSourceName(TMUX1108_SOURCE_GND),
                          "ads_s1d1",
                          sensorarrayLogFmtI32(valueBuf, sizeof(valueBuf), err == ESP_OK, uv),
                          sensorarrayLogFmtI32(uvBuf, sizeof(uvBuf), err == ESP_OK, uv),
                          SENSORARRAY_NA,
                          sensorarrayLogFmtI32(rawBuf, sizeof(rawBuf), err == ESP_OK, raw),
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          SENSORARRAY_NA,
                          mapLabel ? mapLabel : SENSORARRAY_NA,
                          err,
                          (err == ESP_OK) ? "ads_read_ok" : "ads_read_error");
        sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_INTERVAL_MS);
    }
}

static esp_err_t sensorarrayApplyS1D1StaticRoute(sensorarrayState_t *state,
                                                 const sensorarrayRouteMap_t *routeMap,
                                                 bool reapplyEachLoop,
                                                 const char **outMapLabel)
{
    const char *mapLabel = SENSORARRAY_NA;
    esp_err_t err = sensorarrayMeasureApplyRoute(state,
                                                 SENSORARRAY_S1,
                                                 SENSORARRAY_D1,
                                                 SENSORARRAY_PATH_RESISTIVE,
                                                 TMUX1108_SOURCE_GND,
                                                 &mapLabel);

    if (outMapLabel) {
        *outMapLabel = mapLabel ? mapLabel : SENSORARRAY_NA;
    }

    printf("DBGROUTEFIX,mode=s1d1_static,point=S1D1,kind=res,column=S1,dline=D1,pathIntent=resistive,"
           "swIntent=GND_LOW,map=%s,selARequested=%u,selBRequested=%u,reapplyEachLoop=%u,behaviour=%s,routeErr=%ld,"
           "status=%s\n",
           mapLabel ? mapLabel : SENSORARRAY_NA,
           (routeMap != NULL && routeMap->selALevel) ? 1u : 0u,
           (routeMap != NULL && routeMap->selBLevel) ? 1u : 0u,
           reapplyEachLoop ? 1u : 0u,
           sensorarrayS1D1StaticBehaviorName(),
           (long)err,
           (err == ESP_OK) ? "route_applied" : "route_error");
    sensorarrayLogS1D1ControlState(mapLabel);
    sensorarrayLogControlGpio("s1d1_static_route_applied", "S1D1");

    return err;
}

void sensorarrayDebugRunS1D1StaticResistorDebugImpl(sensorarrayState_t *state,
                                                    const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    (void)adsPolicy;
    if (!state) {
        sensorarrayDebugIdleForever("s1d1_static_state_null");
        return;
    }

    const sensorarrayRouteMap_t *routeMap =
        sensorarrayBoardMapFindRoute(SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE);
    const bool routeOnly = sensorarrayIsS1D1StaticRouteOnlyBehavior();
    const bool reapplyEachLoop = (CONFIG_SENSORARRAY_S1D1_STATIC_REAPPLY_ROUTE_EACH_LOOP != 0);
    const uint32_t loopDelayMs = (uint32_t)CONFIG_SENSORARRAY_S1D1_STATIC_LOOP_DELAY_MS;
    const char *mapLabel = SENSORARRAY_NA;
    uint32_t heartbeatCount = 0u;

    printf("=== S1D1 STATIC RESISTOR DEBUG MODE ACTIVE ===\n");
    printf("DBGSTATIC,mode=s1d1_resistor_static,status=active,point=S1D1,kind=res,column=S1,dline=D1,"
           "pathIntent=resistive,swIntent=GND_LOW,behaviour=%s,reapplyEachLoop=%u,loopDelayMs=%u\n",
           sensorarrayS1D1StaticBehaviorName(),
           reapplyEachLoop ? 1u : 0u,
           (unsigned)loopDelayMs);

    if (!routeMap) {
        sensorarrayLogStartup("s1d1_static", ESP_ERR_NOT_SUPPORTED, "s1d1_route_missing", 0);
        sensorarrayDebugIdleForever("s1d1_static_route_missing");
        return;
    }

    sensorarrayLogStartup("s1d1_static",
                          ESP_OK,
                          routeOnly ? "route_only" : "route_and_read",
                          (int32_t)loopDelayMs);

    if (!reapplyEachLoop) {
        esp_err_t routeErr = sensorarrayApplyS1D1StaticRoute(state, routeMap, false, &mapLabel);
        if (routeErr != ESP_OK) {
            sensorarrayDebugIdleForever("s1d1_static_route_error");
            return;
        }
    } else {
        mapLabel = routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA;
    }

    while (true) {
        esp_err_t routeErr = ESP_OK;
        char heartbeatBuf[24];

        if (reapplyEachLoop) {
            routeErr = sensorarrayApplyS1D1StaticRoute(state, routeMap, true, &mapLabel);
        }
        if (routeErr != ESP_OK) {
            sensorarrayLogDbg("S1D1",
                              "res",
                              "S1",
                              "D1",
                              sensorarrayLogSwSourceName(TMUX1108_SOURCE_GND),
                              routeOnly ? "s1d1_static_route_only" : "s1d1_static_route_and_read",
                              sensorarrayLogFmtU32(heartbeatBuf, sizeof(heartbeatBuf), true, heartbeatCount),
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              mapLabel ? mapLabel : SENSORARRAY_NA,
                              routeErr,
                              "route_error");
            sensorarrayDelayMs(loopDelayMs);
            ++heartbeatCount;
            continue;
        }

        if (routeOnly) {
            sensorarrayLogDbg("S1D1",
                              "res",
                              "S1",
                              "D1",
                              sensorarrayLogSwSourceName(TMUX1108_SOURCE_GND),
                              "s1d1_static_route_only",
                              sensorarrayLogFmtU32(heartbeatBuf, sizeof(heartbeatBuf), true, heartbeatCount),
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              SENSORARRAY_NA,
                              mapLabel ? mapLabel : SENSORARRAY_NA,
                              ESP_OK,
                              "heartbeat");
        } else {
            char valueBuf[24];
            char uvBuf[24];
            char rawBuf[24];
            int32_t raw = 0;
            int32_t uv = 0;

            esp_err_t readErr = sensorarrayDebugReadAdsS1D1Once(state,
                                                                adsPolicy,
                                                                (CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST != 0),
                                                                &raw,
                                                                &uv);
            sensorarrayLogDbg("S1D1",
                              "res",
                              "S1",
                              "D1",
                              sensorarrayLogSwSourceName(TMUX1108_SOURCE_GND),
                              "s1d1_static_route_and_read",
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
                              mapLabel ? mapLabel : SENSORARRAY_NA,
                              readErr,
                              (readErr == ESP_OK) ? "ads_read_ok" : "ads_read_error");
        }

        sensorarrayDelayMs(loopDelayMs);
        ++heartbeatCount;
    }
}
