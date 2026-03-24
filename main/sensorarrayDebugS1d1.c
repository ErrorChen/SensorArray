#include "sensorarrayDebugS1d1.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

typedef struct {
    bool readbackMatch;
    const char *mismatchReason;
    const char *routeConfidence;
} sensorarrayRouteReadbackEval_t;

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static sensorarrayRouteReadbackEval_t sensorarrayRouteEvalUnavailable(void)
{
    sensorarrayRouteReadbackEval_t eval = {
        .readbackMatch = false,
        .mismatchReason = "readback_gpio_unavailable",
        .routeConfidence = "unconfirmed",
    };
    return eval;
}

static const char *sensorarrayRowName(uint8_t rowIndex, char *buf, size_t bufSize)
{
    if (!buf || bufSize == 0u || rowIndex > 7u) {
        return SENSORARRAY_NA;
    }
    snprintf(buf, bufSize, "S%u", (unsigned)(rowIndex + 1u));
    return buf;
}

static const char *sensorarrayCtrlSourceName(const tmuxSwitchControlState_t *ctrl)
{
    if (!ctrl) {
        return SENSORARRAY_NA;
    }
    return sensorarrayLogSwSourceLogicalName(ctrl->source);
}

static const char *sensorarraySafePolicyModeName(const char *mode)
{
    return mode ? mode : SENSORARRAY_NA;
}

static const char *sensorarraySelaRouteNameFromGpioLevel(bool selaGpioLevel)
{
    sensorarraySelaRoute_t route = SENSORARRAY_SELA_ROUTE_ADS1263;
    if (sensorarrayBoardMapSelaRouteFromGpioLevel(selaGpioLevel ? 1 : 0, &route)) {
        return sensorarrayBoardMapSelaRouteName(route);
    }
    return "UNKNOWN";
}

static const char *sensorarrayResolvedSelaRouteName(int selaReadLevel)
{
    sensorarraySelaRoute_t route = SENSORARRAY_SELA_ROUTE_ADS1263;
    if (sensorarrayBoardMapSelaRouteFromGpioLevel(selaReadLevel, &route)) {
        return sensorarrayBoardMapSelaRouteName(route);
    }
    return "UNKNOWN";
}

static const sensorarrayAdsReadPolicy_t *sensorarrayReadPolicyOrDefault(const sensorarrayAdsReadPolicy_t *policy)
{
    static const sensorarrayAdsReadPolicy_t kDefaultPolicy = {
        .stopBeforeMuxChange = (CONFIG_SENSORARRAY_ADS_READ_STOP1_BEFORE_MUX != 0),
        .settleAfterMuxMs = (uint32_t)CONFIG_SENSORARRAY_ADS_READ_SETTLE_AFTER_MUX_MS,
        .startEveryRead = (CONFIG_SENSORARRAY_ADS_READ_START1_EVERY_READ != 0),
        .baseDiscardCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_BASE_DISCARD_COUNT,
        .readRetryCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_RETRY_COUNT,
    };
    return policy ? policy : &kDefaultPolicy;
}

static sensorarrayRouteReadbackEval_t sensorarrayEvaluateS1D1Readback(const tmuxSwitchControlState_t *ctrl,
                                                                      bool desiredSelaGpioLevel,
                                                                      bool desiredSelBLevel,
                                                                      bool desiredEn)
{
    sensorarrayRouteReadbackEval_t eval = sensorarrayRouteEvalUnavailable();
    if (!ctrl) {
        return eval;
    }

    if (ctrl->row != (SENSORARRAY_S1 - 1u)) {
        eval.mismatchReason = "row_mismatch";
        return eval;
    }
    if (ctrl->source != TMUX1108_SOURCE_GND) {
        eval.mismatchReason = "source_mismatch";
        return eval;
    }
    if (ctrl->selaLevel < 0 || ctrl->selbLevel < 0) {
        eval.mismatchReason = "readback_gpio_unavailable";
        return eval;
    }
    if (ctrl->selaLevel != (desiredSelaGpioLevel ? 1 : 0)) {
        eval.mismatchReason = "sela_mismatch";
        return eval;
    }
    if (ctrl->selbLevel != (desiredSelBLevel ? 1 : 0)) {
        eval.mismatchReason = "selb_mismatch";
        return eval;
    }

    if (!ctrl->tmux1134EnControllable) {
        if (ctrl->tmux1134EnLogicalOn != desiredEn) {
            eval.mismatchReason = "en_mismatch";
            return eval;
        }
        eval.readbackMatch = true;
        eval.mismatchReason = "en_not_controllable";
        eval.routeConfidence = "partially_confirmed";
        return eval;
    }

    if (ctrl->enLevel < 0) {
        eval.mismatchReason = "readback_gpio_unavailable";
        return eval;
    }
    if (ctrl->enLevel != (desiredEn ? 1 : 0)) {
        eval.mismatchReason = "en_mismatch";
        return eval;
    }

    eval.readbackMatch = true;
    eval.mismatchReason = "route_readback_ok";
    eval.routeConfidence = "confirmed";
    return eval;
}

static void sensorarrayLogRouteApply(const char *mode,
                                     bool lockedMode,
                                     bool desiredSelaGpioLevel,
                                     bool desiredSelBLevel,
                                     bool desiredEn,
                                     esp_err_t err)
{
    printf("tag=ROUTE_APPLY,mode=%s,locked=%u,lockedSelaGpioLevel=%u,lockedSelaRoute=%s,lockedSelBLevel=%u,"
           "row=S1,source=GND,desiredSelaGpioLevel=%u,desiredSelaRoute=%s,desiredSelBLevel=%u,desiredEn=%u,"
           "result=%s,err=%ld\n",
           sensorarraySafePolicyModeName(mode),
           lockedMode ? 1u : 0u,
           (unsigned)(SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0),
           sensorarraySelaRouteNameFromGpioLevel(SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0),
           (unsigned)(SENSORARRAY_S1D1_DEBUG_SELB_LEVEL != 0),
           desiredSelaGpioLevel ? 1u : 0u,
           sensorarraySelaRouteNameFromGpioLevel(desiredSelaGpioLevel),
           desiredSelBLevel ? 1u : 0u,
           desiredEn ? 1u : 0u,
           (err == ESP_OK) ? "ok" : "route_apply_error",
           (long)err);
}

static void sensorarrayLogRouteReadback(const tmuxSwitchControlState_t *ctrl,
                                        const sensorarrayRouteReadbackEval_t *eval,
                                        bool requestedSelaGpioLevel)
{
    char rowBuf[8];
    const sensorarrayRouteReadbackEval_t fallback = sensorarrayRouteEvalUnavailable();
    const sensorarrayRouteReadbackEval_t *report = eval ? eval : &fallback;

    printf("tag=ROUTE_READBACK,row=%s,source=%s,a0=%d,a1=%d,a2=%d,sw=%d,sel1=%d,sel2=%d,sel3=%d,sel4=%d,"
           "requestedSelaRoute=%s,requestedSelaGpioLevel=%u,resolvedSelaRoute=%s,selaLevel=%d,selbLevel=%d,"
           "enLevel=%d,readbackMatch=%u,mismatchReason=%s,routeConfidence=%s\n",
           ctrl ? sensorarrayRowName(ctrl->row, rowBuf, sizeof(rowBuf)) : SENSORARRAY_NA,
           sensorarrayCtrlSourceName(ctrl),
           ctrl ? ctrl->a0Level : -1,
           ctrl ? ctrl->a1Level : -1,
           ctrl ? ctrl->a2Level : -1,
           ctrl ? ctrl->swLevel : -1,
           ctrl ? ctrl->sel1Level : -1,
           ctrl ? ctrl->sel2Level : -1,
           ctrl ? ctrl->sel3Level : -1,
           ctrl ? ctrl->sel4Level : -1,
           sensorarraySelaRouteNameFromGpioLevel(requestedSelaGpioLevel),
           requestedSelaGpioLevel ? 1u : 0u,
           ctrl ? sensorarrayResolvedSelaRouteName(ctrl->selaLevel) : "UNKNOWN",
           ctrl ? ctrl->selaLevel : -1,
           ctrl ? ctrl->selbLevel : -1,
           ctrl ? ctrl->enLevel : -1,
           report->readbackMatch ? 1u : 0u,
           report->mismatchReason ? report->mismatchReason : SENSORARRAY_NA,
           report->routeConfidence ? report->routeConfidence : "unconfirmed");
}

static esp_err_t sensorarrayLogAdsCoreReadback(sensorarrayState_t *state, const char *stage)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t power = 0u;
    uint8_t iface = 0u;
    uint8_t mode2 = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    esp_err_t err = ads126xAdcReadCoreRegisters(&state->ads, &power, &iface, &mode2, &inpmux, &refmux);

    if (err == ESP_OK) {
        state->adsRefMux = refmux;
        state->adsRefMuxValid = true;
    }

    printf("tag=ADS_CORE_READBACK,stage=%s,power=0x%02X,interface=0x%02X,mode2=0x%02X,inpmux=0x%02X,refmux=0x%02X,"
           "intref=%u,vbias=%u,result=%s,err=%ld\n",
           stage ? stage : SENSORARRAY_NA,
           power,
           iface,
           mode2,
           inpmux,
           refmux,
           (unsigned)((power & ADS126X_POWER_INTREF) != 0u),
           (unsigned)((power & ADS126X_POWER_VBIAS) != 0u),
           (err == ESP_OK) ? "ok" : "read_error",
           (long)err);

    return err;
}

static esp_err_t sensorarrayVerifyAdsCorePath(sensorarrayState_t *state, const char *stage)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t power = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    esp_err_t err = ads126xAdcReadCoreRegisters(&state->ads, &power, NULL, NULL, &inpmux, &refmux);
    if (err != ESP_OK) {
        printf("tag=ADS_CORE_EXPECT,stage=%s,intrefExpected=1,intrefReadback=0,vbiasExpected=1,vbiasReadback=0,"
               "refmuxExpected=0x00,refmuxReadback=0x00,inpmuxReadback=0x00,result=read_error,err=%ld\n",
               stage ? stage : SENSORARRAY_NA,
               (long)err);
        return err;
    }

    const bool intrefOk = ((power & ADS126X_POWER_INTREF) != 0u);
    const bool vbiasOk = ((power & ADS126X_POWER_VBIAS) != 0u);
    const bool refmuxOk = (refmux == 0x00u);
    printf("tag=ADS_CORE_EXPECT,stage=%s,intrefExpected=1,intrefReadback=%u,vbiasExpected=1,vbiasReadback=%u,"
           "refmuxExpected=0x00,refmuxReadback=0x%02X,inpmuxReadback=0x%02X,result=%s,err=0\n",
           stage ? stage : SENSORARRAY_NA,
           intrefOk ? 1u : 0u,
           vbiasOk ? 1u : 0u,
           refmux,
           inpmux,
           (intrefOk && vbiasOk && refmuxOk) ? "ok" : "mismatch");

    return (intrefOk && vbiasOk && refmuxOk) ? ESP_OK : ESP_ERR_INVALID_STATE;
}

/*
 * Prepare ADS path for S1D1 debug:
 * - internal reference enabled
 * - VBIAS enabled for AINCOM level-shift
 * - REFMUX fixed to internal reference (0x00)
 */
static esp_err_t sensorarrayPrepareAdsBiasAndRef(sensorarrayState_t *state,
                                                 bool enableStatusByte,
                                                 const char *stage)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    if (state->adsAdc1Running) {
        esp_err_t stopErr = ads126xAdcStopAdc1(&state->ads);
        if (stopErr != ESP_OK) {
            return stopErr;
        }
        state->adsAdc1Running = false;
    }

    esp_err_t err = sensorarrayLogAdsCoreReadback(state, "prepare_before");
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcConfigure(&state->ads, true, enableStatusByte, ADS126X_CRC_OFF, 1, 0);
    printf("tag=ADS_PREP,stage=%s,step=configure,enableInternalRef=1,enableStatusByte=%u,result=%s,err=%ld\n",
           stage ? stage : SENSORARRAY_NA,
           enableStatusByte ? 1u : 0u,
           (err == ESP_OK) ? "ok" : "configure_error",
           (long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcSetVbiasEnabled(&state->ads, true);
    printf("tag=ADS_PREP,stage=%s,step=set_vbias,desiredVbias=1,result=%s,err=%ld\n",
           stage ? stage : SENSORARRAY_NA,
           (err == ESP_OK) ? "ok" : "vbias_error",
           (long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayBringupAdsSetRefMux(state, 0x00u);
    printf("tag=ADS_PREP,stage=%s,step=set_refmux,desiredRefmux=0x00,result=%s,err=%ld\n",
           stage ? stage : SENSORARRAY_NA,
           (err == ESP_OK) ? "ok" : "refmux_error",
           (long)err);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayDelayMs(SENSORARRAY_REF_SETTLE_MS);

    err = sensorarrayLogAdsCoreReadback(state, "prepare_after_settle");
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayVerifyAdsCorePath(state, "prepare_after_settle");
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcStartAdc1(&state->ads);
    printf("tag=ADS_PREP,stage=%s,step=start1,result=%s,err=%ld\n",
           stage ? stage : SENSORARRAY_NA,
           (err == ESP_OK) ? "ok" : "start_error",
           (long)err);
    if (err != ESP_OK) {
        return err;
    }
    state->adsAdc1Running = true;

    return sensorarrayLogAdsCoreReadback(state, "prepare_after_start");
}

static esp_err_t sensorarrayResolveS1D1AdsMux(uint8_t *outMuxp, uint8_t *outMuxn)
{
    if (!outMuxp || !outMuxn) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayBoardMapAdsMuxForDLine(SENSORARRAY_D1, outMuxp, outMuxn)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (*outMuxn != SENSORARRAY_ADS_MUX_AINCOM) {
        printf("tag=ADS_MUX_VALIDATE,dLine=D1,muxp=%u,muxn=%u,muxnName=%s,result=muxn_not_aincom\n",
               (unsigned)(*outMuxp),
               (unsigned)(*outMuxn),
               sensorarrayLogAdsMuxName(*outMuxn));
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t sensorarrayApplyS1D1Route(sensorarrayState_t *state,
                                           const char *mode,
                                           bool lockedMode,
                                           bool desiredSelaGpioLevel,
                                           bool desiredSelBLevel,
                                           sensorarrayRouteReadbackEval_t *outEval,
                                           tmuxSwitchControlState_t *outCtrl)
{
    if (outEval) {
        *outEval = sensorarrayRouteEvalUnavailable();
    }
    if (outCtrl) {
        *outCtrl = (tmuxSwitchControlState_t){0};
    }
    if (!state || !state->tmuxReady) {
        sensorarrayLogRouteApply(mode, lockedMode, desiredSelaGpioLevel, desiredSelBLevel, true, ESP_ERR_INVALID_STATE);
        sensorarrayLogRouteReadback(NULL, outEval, desiredSelaGpioLevel);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureApplyRouteLevels(state,
                                                       SENSORARRAY_S1,
                                                       SENSORARRAY_D1,
                                                       SENSORARRAY_DEBUG_PATH_RESISTIVE,
                                                       TMUX1108_SOURCE_GND,
                                                       desiredSelaGpioLevel,
                                                       desiredSelBLevel,
                                                       SENSORARRAY_SETTLE_AFTER_COLUMN_MS,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_SW_MS,
                                                       mode);

    sensorarrayLogRouteApply(mode, lockedMode, desiredSelaGpioLevel, desiredSelBLevel, true, err);

    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) != ESP_OK) {
        sensorarrayRouteReadbackEval_t eval = sensorarrayRouteEvalUnavailable();
        sensorarrayLogRouteReadback(NULL, &eval, desiredSelaGpioLevel);
        if (outEval) {
            *outEval = eval;
        }
        return err;
    }

    sensorarrayRouteReadbackEval_t eval =
        sensorarrayEvaluateS1D1Readback(&ctrl, desiredSelaGpioLevel, desiredSelBLevel, true);
    sensorarrayLogRouteReadback(&ctrl, &eval, desiredSelaGpioLevel);
    if (outCtrl) {
        *outCtrl = ctrl;
    }
    if (outEval) {
        *outEval = eval;
    }
    return err;
}

static void sensorarrayLogAdsSampleWithRegisters(sensorarrayState_t *state,
                                                 const char *mode,
                                                 const char *sampleLabel,
                                                 uint8_t muxp,
                                                 uint8_t muxn,
                                                 bool sampleValid,
                                                 int32_t raw,
                                                 int32_t uv,
                                                 uint8_t statusByte,
                                                 const char *modelStatus,
                                                 const sensorarrayRouteReadbackEval_t *routeEval,
                                                 esp_err_t sampleErr)
{
    char rawBuf[24];
    char uvBuf[24];
    char powerBuf[12];
    char refmuxBuf[12];
    char inpmuxBuf[12];

    uint8_t power = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    esp_err_t regsErr = ads126xAdcReadCoreRegisters(&state->ads, &power, NULL, NULL, &inpmux, &refmux);

    const sensorarrayRouteReadbackEval_t fallback = sensorarrayRouteEvalUnavailable();
    const sensorarrayRouteReadbackEval_t *eval = routeEval ? routeEval : &fallback;

    printf("tag=ADS_SAMPLE,mode=%s,sample=%s,muxp=%u,muxn=%u,muxpName=%s,muxnName=%s,raw=%s,uv=%s,statusByte=0x%02X,"
           "modelStatus=%s,power=%s,refmux=%s,inpmux=%s,routeConfidence=%s,result=%s,err=%ld,regsErr=%ld\n",
           sensorarraySafePolicyModeName(mode),
           sampleLabel ? sampleLabel : SENSORARRAY_NA,
           (unsigned)(muxp & 0x0Fu),
           (unsigned)(muxn & 0x0Fu),
           sensorarrayLogAdsMuxName(muxp),
           sensorarrayLogAdsMuxName(muxn),
           sensorarrayLogFmtI32(rawBuf, sizeof(rawBuf), sampleValid, raw),
           sensorarrayLogFmtI32(uvBuf, sizeof(uvBuf), sampleValid, uv),
           statusByte,
           modelStatus ? modelStatus : SENSORARRAY_NA,
           sensorarrayLogFmtHexU8(powerBuf, sizeof(powerBuf), regsErr == ESP_OK, power),
           sensorarrayLogFmtHexU8(refmuxBuf, sizeof(refmuxBuf), regsErr == ESP_OK, refmux),
           sensorarrayLogFmtHexU8(inpmuxBuf, sizeof(inpmuxBuf), regsErr == ESP_OK, inpmux),
           eval->routeConfidence ? eval->routeConfidence : "unconfirmed",
           (sampleErr == ESP_OK) ? "ok" : "ads_read_error",
           (long)sampleErr,
           (long)regsErr);
}

static esp_err_t sensorarrayReadAndLogAdsSample(sensorarrayState_t *state,
                                                const sensorarrayAdsReadPolicy_t *adsPolicy,
                                                const char *mode,
                                                const char *sampleLabel,
                                                uint8_t muxp,
                                                uint8_t muxn,
                                                bool discardFirst,
                                                const sensorarrayRouteReadbackEval_t *routeEval,
                                                const char *modelStatusOverride,
                                                int32_t *outRaw,
                                                int32_t *outUv,
                                                uint8_t *outStatusByte)
{
    int32_t raw = 0;
    int32_t uv = 0;
    uint8_t statusByte = 0u;

    esp_err_t readErr =
        sensorarrayMeasureReadAdsPairUv(state, adsPolicy, muxp, muxn, discardFirst, &raw, &uv, &statusByte);

    const char *modelStatus = modelStatusOverride;
    int32_t mohm = 0;
    bool haveMohm = false;
    if (!modelStatus) {
        modelStatus = (readErr == ESP_OK) ? sensorarrayMeasureDividerModelStatus(uv, &mohm, &haveMohm)
                                          : "ads_read_error";
    }

    sensorarrayLogAdsSampleWithRegisters(state,
                                         mode,
                                         sampleLabel,
                                         muxp,
                                         muxn,
                                         (readErr == ESP_OK),
                                         raw,
                                         uv,
                                         statusByte,
                                         modelStatus,
                                         routeEval,
                                         readErr);

    if (outRaw) {
        *outRaw = (readErr == ESP_OK) ? raw : 0;
    }
    if (outUv) {
        *outUv = (readErr == ESP_OK) ? uv : 0;
    }
    if (outStatusByte) {
        *outStatusByte = (readErr == ESP_OK) ? statusByte : 0u;
    }
    return readErr;
}

static esp_err_t sensorarrayRunAincomBiasDiagnostic(sensorarrayState_t *state,
                                                    const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t power = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    esp_err_t err = ads126xAdcReadCoreRegisters(&state->ads, &power, NULL, NULL, &inpmux, &refmux);
    if (err != ESP_OK) {
        return err;
    }

    const bool intrefEnabled = ((power & ADS126X_POWER_INTREF) != 0u);
    const bool vbiasEnabled = ((power & ADS126X_POWER_VBIAS) != 0u);

    sensorarrayRouteReadbackEval_t noRouteEval = sensorarrayRouteEvalUnavailable();
    int32_t aincomUv = 0;
    int32_t ain9Uv = 0;
    esp_err_t errAincom = sensorarrayReadAndLogAdsSample(state,
                                                         adsPolicy,
                                                         "aincom_bias_diag",
                                                         "AINCOM_AINCOM",
                                                         SENSORARRAY_ADS_MUX_AINCOM,
                                                         SENSORARRAY_ADS_MUX_AINCOM,
                                                         true,
                                                         &noRouteEval,
                                                         "aincom_diag_pair",
                                                         NULL,
                                                         &aincomUv,
                                                         NULL);
    esp_err_t errAin9 = sensorarrayReadAndLogAdsSample(state,
                                                       adsPolicy,
                                                       "aincom_bias_diag",
                                                       "AIN9_AINCOM",
                                                       SENSORARRAY_ADS_MUX_AIN9,
                                                       SENSORARRAY_ADS_MUX_AINCOM,
                                                       true,
                                                       &noRouteEval,
                                                       "aincom_diag_pair",
                                                       NULL,
                                                       &ain9Uv,
                                                       NULL);

    const char *biasStatus = vbiasEnabled ? "aincom_bias_enabled" : "aincom_bias_not_enabled";
    const bool selfConsistent = intrefEnabled && vbiasEnabled && (errAincom == ESP_OK) && (errAin9 == ESP_OK);
    const char *pathStatus = selfConsistent ? "aincom_path_self_consistent" : "aincom_path_suspicious";

    printf("tag=AINCOM_BIAS_DIAG,intref=%u,vbias=%u,refmux=0x%02X,inpmux=0x%02X,biasStatus=%s,pathStatus=%s,"
           "aincomUv=%ld,ain9Uv=%ld,result=%s\n",
           intrefEnabled ? 1u : 0u,
           vbiasEnabled ? 1u : 0u,
           refmux,
           inpmux,
           biasStatus,
           pathStatus,
           (long)aincomUv,
           (long)ain9Uv,
           selfConsistent ? "ok" : "suspicious");

    err = sensorarrayLogAdsCoreReadback(state, "aincom_bias_diag_after");
    if (err != ESP_OK) {
        return err;
    }
    return selfConsistent ? ESP_OK : ESP_ERR_INVALID_STATE;
}

static esp_err_t sensorarrayRunAdsS1D1BootSelftest(sensorarrayState_t *state,
                                                   const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayPrepareAdsBiasAndRef(state, true, "s1d1_boot");
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayRunAincomBiasDiagnostic(state, adsPolicy);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayRouteReadbackEval_t noRouteEval = sensorarrayRouteEvalUnavailable();
    uint8_t d1Muxp = 0u;
    uint8_t d1Muxn = 0u;
    err = sensorarrayResolveS1D1AdsMux(&d1Muxp, &d1Muxn);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayReadAndLogAdsSample(state,
                                         adsPolicy,
                                         "boot_selftest",
                                         "D1_AIN0_AINCOM",
                                         d1Muxp,
                                         d1Muxn,
                                         true,
                                         &noRouteEval,
                                         "boot_pair",
                                         NULL,
                                         NULL,
                                         NULL);
    if (err != ESP_OK) {
        return err;
    }

    return sensorarrayReadAndLogAdsSample(state,
                                          adsPolicy,
                                          "boot_selftest",
                                          "AIN8_AINCOM",
                                          SENSORARRAY_ADS_MUX_AIN8,
                                          SENSORARRAY_ADS_MUX_AINCOM,
                                          true,
                                          &noRouteEval,
                                          "boot_pair",
                                          NULL,
                                          NULL,
                                          NULL);
}

void sensorarrayDebugRunAdsS1D1OnlyModeImpl(sensorarrayState_t *state,
                                            const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state || !state->adsReady) {
        sensorarrayLogStartup("ads_s1d1_only", ESP_ERR_INVALID_STATE, "ads_unavailable", 0);
        sensorarrayDebugIdleForever("ads_s1d1_unavailable");
        return;
    }

    const sensorarrayAdsReadPolicy_t *readPolicy = sensorarrayReadPolicyOrDefault(adsPolicy);
    esp_err_t err = sensorarrayRunAdsS1D1BootSelftest(state, readPolicy);
    sensorarrayLogStartup("ads_s1d1_only_selftest", err, (err == ESP_OK) ? "ok" : "selftest_error", 0);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_s1d1_selftest_error");
        return;
    }

    uint8_t muxp = 0u;
    uint8_t muxn = 0u;
    err = sensorarrayResolveS1D1AdsMux(&muxp, &muxn);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("ads_s1d1_mux_invalid");
        return;
    }

    const bool lockedMode = (SENSORARRAY_DEBUG_ADS_S1D1_LOCK_TO_SINGLE_STATE != 0);
    const bool lockedSelaGpioLevel = (SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0);
    const bool lockedSelBLevel = (SENSORARRAY_S1D1_DEBUG_SELB_LEVEL != 0);
    printf("tag=ROUTE_MODE,mode=%s,locked=%u,lockedSelaGpioLevel=%u,lockedSelaRoute=%s,lockedSelBLevel=%u,muxp=%u,"
           "muxn=%u,muxpName=%s,muxnName=%s,holdMs=%u\n",
           lockedMode ? "locked_single_state" : "sweep_states",
           lockedMode ? 1u : 0u,
           lockedSelaGpioLevel ? 1u : 0u,
           sensorarraySelaRouteNameFromGpioLevel(lockedSelaGpioLevel),
           lockedSelBLevel ? 1u : 0u,
           (unsigned)muxp,
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxp),
           sensorarrayLogAdsMuxName(muxn),
           (unsigned)SENSORARRAY_DEBUG_ADS_S1D1_SWEEP_HOLD_MS);

    const struct {
        bool selaGpioLevel;
        bool selBLevel;
        const char *label;
    } sweepStates[] = {
        { false, false, "SELA_GPIO0_SELB0" },
        { true, false, "SELA_GPIO1_SELB0" },
        { false, true, "SELA_GPIO0_SELB1" },
        { true, true, "SELA_GPIO1_SELB1" },
    };

    while (true) {
        for (uint8_t i = 0u; i < 4u; ++i) {
            bool desiredSelaGpioLevel = lockedMode ? lockedSelaGpioLevel : sweepStates[i].selaGpioLevel;
            bool desiredSelBLevel = lockedMode ? lockedSelBLevel : sweepStates[i].selBLevel;
            const char *mode = lockedMode ? "locked_single_state" : "sweep_states";
            const char *sampleLabel = lockedMode ? "D1_AIN0_AINCOM_LOCKED" : sweepStates[i].label;

            sensorarrayRouteReadbackEval_t routeEval = sensorarrayRouteEvalUnavailable();
            err = sensorarrayApplyS1D1Route(state,
                                            mode,
                                            lockedMode,
                                            desiredSelaGpioLevel,
                                            desiredSelBLevel,
                                            &routeEval,
                                            NULL);
            if (err != ESP_OK) {
                sensorarrayDelayMs(SENSORARRAY_DEBUG_ADS_S1D1_SWEEP_HOLD_MS);
                if (lockedMode) {
                    break;
                }
                continue;
            }

            sensorarrayDelayMs(25u);
            (void)sensorarrayReadAndLogAdsSample(state,
                                                 readPolicy,
                                                 mode,
                                                 sampleLabel,
                                                 muxp,
                                                 muxn,
                                                 true,
                                                 &routeEval,
                                                 NULL,
                                                 NULL,
                                                 NULL,
                                                 NULL);

            sensorarrayDelayMs(SENSORARRAY_DEBUG_ADS_S1D1_SWEEP_HOLD_MS);
            if (lockedMode) {
                break;
            }
        }
    }
}

static void sensorarrayRunS1D1RouteOnlyLoop(sensorarrayState_t *state,
                                            const char *mode,
                                            bool lockedMode,
                                            bool desiredSelaGpioLevel,
                                            bool desiredSelBLevel,
                                            uint32_t loopDelayMs)
{
    while (true) {
        sensorarrayRouteReadbackEval_t routeEval = sensorarrayRouteEvalUnavailable();
        (void)sensorarrayApplyS1D1Route(state,
                                        mode,
                                        lockedMode,
                                        desiredSelaGpioLevel,
                                        desiredSelBLevel,
                                        &routeEval,
                                        NULL);
        sensorarrayDelayMs(loopDelayMs);
    }
}

void sensorarrayDebugRunS1D1ForceAdsHoldModeImpl(sensorarrayState_t *state,
                                                 const sensorarrayAdsReadPolicy_t *adsPolicy)
{
    if (!state || !state->adsReady) {
        sensorarrayLogStartup("s1d1_force_ads_hold", ESP_ERR_INVALID_STATE, "ads_unavailable", 0);
        sensorarrayDebugIdleForever("s1d1_force_ads_hold_ads_unavailable");
        return;
    }

    const sensorarrayAdsReadPolicy_t *readPolicy = sensorarrayReadPolicyOrDefault(adsPolicy);
    esp_err_t err = sensorarrayPrepareAdsBiasAndRef(state, true, "s1d1_force_hold");
    if (err != ESP_OK) {
        sensorarrayLogStartup("s1d1_force_ads_hold", err, "ads_prepare_error", 0);
        sensorarrayDebugIdleForever("s1d1_force_ads_hold_ads_prepare_error");
        return;
    }

    uint8_t muxp = 0u;
    uint8_t muxn = 0u;
    err = sensorarrayResolveS1D1AdsMux(&muxp, &muxn);
    if (err != ESP_OK) {
        sensorarrayDebugIdleForever("s1d1_force_ads_hold_mux_invalid");
        return;
    }

    const sensorarrayRouteMap_t *routeMap =
        sensorarrayBoardMapFindRoute(SENSORARRAY_S1, SENSORARRAY_D1, SENSORARRAY_PATH_RESISTIVE);
    int desiredSelaGpioLevelInt = 0;
    if (routeMap && !sensorarrayBoardMapSelaRouteToGpioLevel(routeMap->selaRoute, &desiredSelaGpioLevelInt)) {
        sensorarrayDebugIdleForever("s1d1_force_ads_hold_sela_route_invalid");
        return;
    }
    bool desiredSelaGpioLevel = (desiredSelaGpioLevelInt != 0);
    bool desiredSelBLevel = routeMap ? routeMap->selBLevel : false;

    const bool lockedMode = (SENSORARRAY_DEBUG_ADS_S1D1_LOCK_TO_SINGLE_STATE != 0);
    if (lockedMode) {
        desiredSelaGpioLevel = (SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0);
        desiredSelBLevel = (SENSORARRAY_S1D1_DEBUG_SELB_LEVEL != 0);
    }

    printf("tag=ROUTE_MODE,mode=%s,locked=%u,lockedSelaGpioLevel=%u,lockedSelaRoute=%s,lockedSelBLevel=%u,"
           "desiredSelaGpioLevel=%u,desiredSelaRoute=%s,desiredSelBLevel=%u,muxp=%u,muxn=%u,loopDelayMs=%u\n",
           lockedMode ? "locked_single_state" : "force_hold",
           lockedMode ? 1u : 0u,
           (unsigned)(SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0),
           sensorarraySelaRouteNameFromGpioLevel(SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0),
           (unsigned)(SENSORARRAY_S1D1_DEBUG_SELB_LEVEL != 0),
           desiredSelaGpioLevel ? 1u : 0u,
           sensorarraySelaRouteNameFromGpioLevel(desiredSelaGpioLevel),
           desiredSelBLevel ? 1u : 0u,
           (unsigned)muxp,
           (unsigned)muxn,
           (unsigned)CONFIG_SENSORARRAY_S1D1_FORCE_ADS_HOLD_LOOP_MS);

    while (true) {
        sensorarrayRouteReadbackEval_t routeEval = sensorarrayRouteEvalUnavailable();
        err = sensorarrayApplyS1D1Route(state,
                                        lockedMode ? "locked_single_state" : "force_hold",
                                        lockedMode,
                                        desiredSelaGpioLevel,
                                        desiredSelBLevel,
                                        &routeEval,
                                        NULL);
        if (err == ESP_OK) {
            (void)sensorarrayReadAndLogAdsSample(state,
                                                 readPolicy,
                                                 lockedMode ? "locked_single_state" : "force_hold",
                                                 "D1_AIN0_AINCOM",
                                                 muxp,
                                                 muxn,
                                                 (CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_DISCARD_FIRST != 0),
                                                 &routeEval,
                                                 NULL,
                                                 NULL,
                                                 NULL,
                                                 NULL);
        }
        sensorarrayDelayMs((uint32_t)CONFIG_SENSORARRAY_S1D1_FORCE_ADS_HOLD_LOOP_MS);
    }
}

void sensorarrayDebugRunSingleResistorS1D1ModeImpl(sensorarrayState_t *state,
                                                   const sensorarrayAdsReadPolicy_t *adsPolicy)
{
#if CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_AND_ADS_READ
    sensorarrayDebugRunS1D1ForceAdsHoldModeImpl(state, adsPolicy);
#else
    if (!state || !state->tmuxReady) {
        sensorarrayDebugIdleForever("s1d1_route_only_unavailable");
        return;
    }
    sensorarrayRunS1D1RouteOnlyLoop(state,
                                    "single_resistor_route_only",
                                    true,
                                    (SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0),
                                    (SENSORARRAY_S1D1_DEBUG_SELB_LEVEL != 0),
                                    (uint32_t)CONFIG_SENSORARRAY_DEBUG_S1D1_ADS_INTERVAL_MS);
#endif
}

void sensorarrayDebugRunS1D1StaticResistorDebugImpl(sensorarrayState_t *state,
                                                    const sensorarrayAdsReadPolicy_t *adsPolicy)
{
#if CONFIG_SENSORARRAY_S1D1_STATIC_ROUTE_AND_READ
    sensorarrayDebugRunS1D1ForceAdsHoldModeImpl(state, adsPolicy);
#else
    if (!state || !state->tmuxReady) {
        sensorarrayDebugIdleForever("s1d1_static_route_only_unavailable");
        return;
    }
    sensorarrayRunS1D1RouteOnlyLoop(state,
                                    "s1d1_static_route_only",
                                    true,
                                    (SENSORARRAY_S1D1_DEBUG_SELA_LEVEL != 0),
                                    (SENSORARRAY_S1D1_DEBUG_SELB_LEVEL != 0),
                                    (uint32_t)CONFIG_SENSORARRAY_S1D1_STATIC_LOOP_DELAY_MS);
#endif
}
