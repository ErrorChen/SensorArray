#include "sensorarrayMeasure.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayLog.h"

#define SENSORARRAY_FDC_STATUS_ERR_CHAN_SHIFT 14
#define SENSORARRAY_FDC_STATUS_ERR_WD_MASK (1U << 11)
#define SENSORARRAY_FDC_STATUS_ERR_AHW_MASK (1U << 10)
#define SENSORARRAY_FDC_STATUS_ERR_ALW_MASK (1U << 9)
#define SENSORARRAY_FDC_STATUS_DRDY_MASK (1U << 6)
#define SENSORARRAY_FDC_STATUS_UNREAD_CH0_MASK (1U << 3)
#define SENSORARRAY_FDC_STATUS_UNREAD_CH1_MASK (1U << 2)
#define SENSORARRAY_FDC_STATUS_UNREAD_CH2_MASK (1U << 1)
#define SENSORARRAY_FDC_STATUS_UNREAD_CH3_MASK (1U << 0)
#define SENSORARRAY_FDC_RAW_SCALE_2P28 268435456.0
#define SENSORARRAY_PI 3.14159265358979323846

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
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

static const char *sensorarrayMeasureDebugPathName(sensorarrayDebugPath_t path)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
        return "CAPACITIVE";
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return "PIEZO_VOLTAGE";
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
    default:
        return "RESISTIVE";
    }
}

static const char *sensorarrayMeasureRouteTag(sensorarrayDebugPath_t path)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
        return "DBGCAP_ROUTE";
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return "DBGPIEZO_ROUTE";
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
    default:
        return "DBGRES_ROUTE";
    }
}

static const char *sensorarrayMeasureRouteDeviceKey(sensorarrayDebugPath_t path)
{
    return (path == SENSORARRAY_DEBUG_PATH_CAPACITIVE) ? "fdc" : "adc";
}

static const char *sensorarrayMeasureRouteDeviceName(sensorarrayDebugPath_t path)
{
    return (path == SENSORARRAY_DEBUG_PATH_CAPACITIVE) ? "FDC2214" : "ADS126x";
}

static const char *sensorarrayMeasureRouteAssertStage(sensorarrayDebugPath_t path)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
        return "capacitive_route";
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return "piezo_route";
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
    default:
        return "resistance_route";
    }
}

static esp_err_t sensorarrayMeasureAssertAndLogRoute(uint8_t sColumn,
                                                     uint8_t dLine,
                                                     sensorarrayDebugPath_t path,
                                                     tmux1108Source_t swSource)
{
    const int expectedSwLevel = tmuxSwitch1108SourceToSwLevel(swSource);
    esp_err_t assertErr = tmuxSwitchAssert1108Source(swSource, sensorarrayMeasureRouteAssertStage(path));

    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);
    printf("%s,timestamp_us=%lld,s=S%u,d=D%u,path=%s,%s=%s,swSource=%s,expectedSwLevel=%d,"
           "cmdSwLevel=%d,obsSwLevel=%d,cmdA0=%d,cmdA1=%d,cmdA2=%d,obsA0=%d,obsA1=%d,obsA2=%d,"
           "cmdSELA=%d,cmdSELB=%d,obsSELA=%d,obsSELB=%d,status=%s\n",
           sensorarrayMeasureRouteTag(path),
           (long long)esp_timer_get_time(),
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayMeasureDebugPathName(path),
           sensorarrayMeasureRouteDeviceKey(path),
           sensorarrayMeasureRouteDeviceName(path),
           sensorarrayLogSwSourceLogicalName(swSource),
           expectedSwLevel,
           haveCtrl ? ctrl.cmdSwLevel : -1,
           haveCtrl ? ctrl.obsSwLevel : -1,
           haveCtrl ? ctrl.cmdA0Level : -1,
           haveCtrl ? ctrl.cmdA1Level : -1,
           haveCtrl ? ctrl.cmdA2Level : -1,
           haveCtrl ? ctrl.obsA0Level : -1,
           haveCtrl ? ctrl.obsA1Level : -1,
           haveCtrl ? ctrl.obsA2Level : -1,
           haveCtrl ? ctrl.cmdSelaLevel : -1,
           haveCtrl ? ctrl.cmdSelbLevel : -1,
           haveCtrl ? ctrl.obsSelaLevel : -1,
           haveCtrl ? ctrl.obsSelbLevel : -1,
           (assertErr == ESP_OK) ? "route_locked" : "sw_assert_failed");
    return assertErr;
}

static bool sensorarrayMeasureRouteSourceAllowed(sensorarrayPath_t path, tmux1108Source_t swSource)
{
    switch (path) {
    case SENSORARRAY_PATH_CAPACITIVE:
    case SENSORARRAY_PATH_PIEZO_VOLTAGE:
        return swSource == TMUX1108_SOURCE_GND;
    case SENSORARRAY_PATH_RESISTIVE:
        return swSource == TMUX1108_SOURCE_REF;
    default:
        return false;
    }
}

static bool sensorarrayMeasureDebugRouteSourceAllowed(sensorarrayDebugPath_t path, tmux1108Source_t swSource)
{
    switch (path) {
    case SENSORARRAY_DEBUG_PATH_CAPACITIVE:
    case SENSORARRAY_DEBUG_PATH_VOLTAGE:
        return swSource == TMUX1108_SOURCE_GND;
    case SENSORARRAY_DEBUG_PATH_RESISTIVE:
        return swSource == TMUX1108_SOURCE_REF;
    default:
        return false;
    }
}

static esp_err_t sensorarrayMeasureStopAdsBeforeRoute(sensorarrayState_t *state)
{
    if (!state || !state->adsReady || !state->adsAdc1Running) {
        return ESP_OK;
    }

    esp_err_t err = ads126xAdcStopAdc1(&state->ads);
    if (err == ESP_OK) {
        state->adsAdc1Running = false;
    }
    return err;
}

static esp_err_t sensorarrayMeasureWriteSela(sensorarrayState_t *state,
                                             sensorarraySelaRoute_t requestRoute,
                                             uint32_t settleDelayMs,
                                             const char *stage,
                                             const char *label)
{
    (void)state;

    int selaWriteLevel = -1;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(requestRoute, &selaWriteLevel)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmux1134SelectSelALevel(selaWriteLevel != 0);
    int selaCmdLevel = -1;
    int selaObsLevel = -1;
    bool obsResolvedValid = false;
    sensorarraySelaRoute_t obsResolvedRoute = SENSORARRAY_SELA_ROUTE_ADS1263;

    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) == ESP_OK) {
        selaCmdLevel = ctrl.cmdSelaLevel;
        selaObsLevel = ctrl.obsSelaLevel;
        obsResolvedValid = sensorarrayBoardMapSelaRouteFromGpioLevel(selaObsLevel, &obsResolvedRoute);
    }

    sensorarrayLogSelaRouteDecision(stage,
                                    label,
                                    requestRoute,
                                    selaWriteLevel,
                                    selaCmdLevel,
                                    selaObsLevel,
                                    obsResolvedValid,
                                    obsResolvedRoute);
    if (selaObsLevel >= 0 && selaObsLevel != selaWriteLevel) {
        sensorarrayLogSelaReadbackMismatch(stage, label, selaWriteLevel, selaObsLevel);
    }
    if (err != ESP_OK) {
        return err;
    }

    // SELA changes the TMUX1134 analog branch; allow a short conservative settle
    // window before the next ADS/FDC access touches the newly selected path.
    sensorarrayDelayMs(settleDelayMs);
    return ESP_OK;
}

sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcState(sensorarrayState_t *state,
                                                            sensorarrayFdcDeviceId_t devId)
{
    if (!state) {
        return NULL;
    }

    switch (devId) {
    case SENSORARRAY_FDC_DEV_PRIMARY:
        return &state->fdcPrimary;
    case SENSORARRAY_FDC_DEV_SECONDARY:
        return &state->fdcSecondary;
    default:
        return NULL;
    }
}

sensorarrayFdcDeviceState_t *sensorarrayMeasureGetFdcStateForDLine(sensorarrayState_t *state,
                                                                    uint8_t dLine,
                                                                    const sensorarrayFdcDLineMap_t **outMap)
{
    const sensorarrayFdcDLineMap_t *map = sensorarrayBoardMapFindFdcByDLine(dLine);
    if (outMap) {
        *outMap = map;
    }
    if (!map) {
        return NULL;
    }

    return sensorarrayMeasureGetFdcState(state, map->devId);
}

esp_err_t sensorarrayMeasureSetSelaPath(sensorarrayState_t *state,
                                        sensorarraySelaRoute_t selaRoute,
                                        uint32_t settleDelayMs,
                                        const char *stage,
                                        const char *label)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    int selaWriteLevel = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel)) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensorarrayMeasureWriteSela(state, selaRoute, settleDelayMs, stage, label);
}

esp_err_t sensorarrayMeasureApplyRouteLevels(sensorarrayState_t *state,
                                             uint8_t sColumn,
                                             uint8_t dLine,
                                             sensorarrayDebugPath_t path,
                                             tmux1108Source_t swSource,
                                             sensorarraySelaRoute_t selaRoute,
                                             bool selBLevel,
                                             uint32_t delayAfterRowMs,
                                             uint32_t delayAfterSelAMs,
                                             uint32_t delayAfterSelBMs,
                                             uint32_t delayAfterSwMs,
                                             const char *label)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayMeasureDebugRouteSourceAllowed(path, swSource)) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    sensorarrayLogRouteStep("ads_stop",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            err,
                            adsStopNeeded ? ((err == ESP_OK) ? "stop_ads_before_route" : "stop_ads_error")
                                          : "ads_already_stopped");
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_row");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterRowMs);

    err = sensorarrayMeasureSetSelaPath(state, selaRoute, delayAfterSelAMs, "selA", label);
    sensorarrayLogRouteStep("selA",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            err,
                            "set_sela_path");
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(selBLevel);
    sensorarrayLogRouteStep("selB", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSelBMs);

    err = tmuxSwitchSet1108Source(swSource);
    sensorarrayLogRouteStep("sw", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_sw");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSwMs);

    err = sensorarrayMeasureAssertAndLogRoute(sColumn, dLine, path, swSource);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayLogDbgExtraCaptureCtrl();
    return ESP_OK;
}

esp_err_t sensorarrayMeasureApplyRoute(sensorarrayState_t *state,
                                       uint8_t sColumn,
                                       uint8_t dLine,
                                       sensorarrayPath_t path,
                                       tmux1108Source_t swSource,
                                       const char **outMapLabel)
{
    if (outMapLabel) {
        *outMapLabel = SENSORARRAY_NA;
    }
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayMeasureRouteSourceAllowed(path, swSource)) {
        return ESP_ERR_INVALID_ARG;
    }

    const sensorarrayRouteMap_t *routeMap = sensorarrayBoardMapFindRoute(sColumn, dLine, path);
    if (!routeMap) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    sensorarrayDebugPath_t debugPath = sensorarrayBoardMapPathToDebugPath(path, swSource);
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(routeMap->selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_STATE;
    }

    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    sensorarrayLogRouteStep("ads_stop",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            debugPath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            adsStopNeeded ? ((err == ESP_OK) ? "stop_ads_before_route" : "stop_ads_error")
                                          : "ads_already_stopped");
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            debugPath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_row");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_COLUMN_MS);

    err = sensorarrayMeasureSetSelaPath(state,
                                        routeMap->selaRoute,
                                        SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                        "selA",
                                        routeMap->mapLabel);
    sensorarrayLogRouteStep("selA",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            debugPath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_sela_path");
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(routeMap->selBLevel);
    sensorarrayLogRouteStep("selB",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            debugPath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_PATH_MS);

    err = tmuxSwitchSet1108Source(swSource);
    sensorarrayLogRouteStep("sw",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            debugPath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_sw");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_SW_MS);

    err = sensorarrayMeasureAssertAndLogRoute(sColumn, dLine, debugPath, swSource);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayLogDbgExtraCaptureCtrl();

    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
    return ESP_OK;
}

esp_err_t sensorarrayMeasureApplyResistanceRoute(sensorarrayState_t *state,
                                                 uint8_t sColumn,
                                                 uint8_t dLine,
                                                 const char **outMapLabel)
{
    return sensorarrayMeasureApplyRoute(state,
                                        sColumn,
                                        dLine,
                                        SENSORARRAY_PATH_RESISTIVE,
                                        TMUX1108_SOURCE_REF,
                                        outMapLabel);
}

esp_err_t sensorarrayMeasureApplyPiezoVoltageRoute(sensorarrayState_t *state,
                                                   uint8_t sColumn,
                                                   uint8_t dLine,
                                                   const char **outMapLabel)
{
    if (outMapLabel) {
        *outMapLabel = "piezo_voltage_sela_ads126x_gnd";
    }
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }

    /*
     * Confirmed SensorArray board polarity:
     *   SW LOW  -> REF
     *   SW HIGH -> GND
     *
     * Piezo voltage reading uses the ADS126x voltage path and requires GND
     * at the selected TMUX1108 source, so this route always commands GND.
     */
    return sensorarrayMeasureApplyRouteLevels(state,
                                              sColumn,
                                              dLine,
                                              SENSORARRAY_DEBUG_PATH_VOLTAGE,
                                              TMUX1108_SOURCE_GND,
                                              SENSORARRAY_SELA_ROUTE_ADS1263,
                                              false,
                                              SENSORARRAY_SETTLE_AFTER_COLUMN_MS,
                                              SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                              SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                              SENSORARRAY_SETTLE_AFTER_SW_MS,
                                              "piezo_voltage_sela_ads126x_gnd");
}

esp_err_t sensorarrayMeasureApplyCapacitiveRoute(sensorarrayState_t *state,
                                                 uint8_t sColumn,
                                                 uint8_t dLine,
                                                 const char **outMapLabel)
{
    return sensorarrayMeasureApplyRoute(state,
                                        sColumn,
                                        dLine,
                                        SENSORARRAY_PATH_CAPACITIVE,
                                        TMUX1108_SOURCE_GND,
                                        outMapLabel);
}

esp_err_t sensorarrayMeasureReadAdsRawWithRetry(sensorarrayState_t *state,
                                                int32_t *outRaw,
                                                uint8_t retryCount,
                                                bool *outTimedOut,
                                                uint8_t *outStatusByte)
{
    if (!state || !outRaw) {
        return ESP_ERR_INVALID_ARG;
    }
    if (outTimedOut) {
        *outTimedOut = false;
    }
    if (outStatusByte) {
        *outStatusByte = 0u;
    }

    for (uint8_t attempt = 0; attempt <= retryCount; ++attempt) {
        esp_err_t err = ads126xAdcReadAdc1Raw(&state->ads, outRaw, outStatusByte);
        if (err == ESP_OK) {
            return ESP_OK;
        }

        if (err == ESP_ERR_TIMEOUT && outTimedOut) {
            *outTimedOut = true;
        }

        printf("DBGADSRETRY,attempt=%u,maxRetry=%u,err=%ld\n",
               (unsigned)attempt,
               (unsigned)retryCount,
               (long)err);

        if (attempt == retryCount) {
            return err;
        }
        sensorarrayDelayMs(1u);
    }

    return ESP_FAIL;
}

esp_err_t sensorarrayMeasureReadAdsPairUv(sensorarrayState_t *state,
                                          const sensorarrayAdsReadPolicy_t *policy,
                                          uint8_t muxp,
                                          uint8_t muxn,
                                          bool discardFirst,
                                          int32_t *outRaw,
                                          int32_t *outUv,
                                          uint8_t *outStatusByte)
{
    if (!state || !outRaw || !outUv || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    const sensorarrayAdsReadPolicy_t *readPolicy = sensorarrayReadPolicyOrDefault(policy);

    muxp &= 0x0Fu;
    muxn &= 0x0Fu;

    sensorarrayLogDbgExtraSetMux(muxp, muxn);
    if (state->adsRefMuxValid) {
        sensorarrayLogDbgExtraSetRefMux(state->adsRefMux);
    }

    uint16_t totalDiscard = (uint16_t)readPolicy->baseDiscardCount + (discardFirst ? 1u : 0u);
    if (totalDiscard > 255u) {
        totalDiscard = 255u;
    }
    sensorarrayLogDbgExtraSetDiscardCount((uint8_t)totalDiscard);

    printf("DBGADSSEQ,step=set_input_mux,muxp=%u(%s),muxn=%u(%s),discardFirst=%u,discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    if (readPolicy->stopBeforeMuxChange && state->adsAdc1Running) {
        esp_err_t stopErr = ads126xAdcStopAdc1(&state->ads);
        if (stopErr == ESP_OK) {
            state->adsAdc1Running = false;
        } else {
            printf("DBGADSSEQ,step=stop1,err=%ld,status=stop_error\n", (long)stopErr);
            return stopErr;
        }
    }

    printf("DBGADSSEQ,step=read,muxp=%u(%s),muxn=%u(%s),discardFirst=%u,discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);

    const bool startEachRead = readPolicy->startEveryRead || !state->adsAdc1Running;
    bool readTimedOut = false;
    uint8_t statusByte = 0u;
    esp_err_t err = ESP_FAIL;
    for (uint8_t attempt = 0u; attempt <= readPolicy->readRetryCount; ++attempt) {
        err = ads126xAdcReadSingleDiffUv(&state->ads,
                                         muxp,
                                         muxn,
                                         startEachRead,
                                         readPolicy->settleAfterMuxMs,
                                         (uint8_t)totalDiscard,
                                         outRaw,
                                         outUv,
                                         &statusByte);
        if (err == ESP_OK) {
            state->adsAdc1Running = true;
            break;
        }
        readTimedOut = (err == ESP_ERR_TIMEOUT);
        printf("DBGADSRETRY,attempt=%u,maxRetry=%u,muxp=%u,muxn=%u,err=%ld\n",
               (unsigned)attempt,
               (unsigned)readPolicy->readRetryCount,
               (unsigned)muxp,
               (unsigned)muxn,
               (long)err);
        if (attempt < readPolicy->readRetryCount) {
            sensorarrayDelayMs(1u);
        }
    }

    if (err != ESP_OK) {
        printf("DBGADSREAD,status=error,muxp=%u(%s),muxn=%u(%s),refmux=0x%02X,discardCount=%u,drdyTimeout=%u,err=%ld\n",
               (unsigned)muxp,
               sensorarrayLogAdsMuxName(muxp),
               (unsigned)muxn,
               sensorarrayLogAdsMuxName(muxn),
               state->adsRefMuxValid ? state->adsRefMux : 0u,
               (unsigned)totalDiscard,
               readTimedOut ? 1u : 0u,
               (long)err);
        return err;
    }

    if (outStatusByte) {
        *outStatusByte = statusByte;
    }

    printf("DBGADSSEQ,step=read_done,muxp=%u(%s),muxn=%u(%s),raw=%ld,uv=%ld,statusByte=0x%02X,discardFirst=%u,"
           "discardCount=%u\n",
           (unsigned)muxp,
           sensorarrayLogAdsMuxName(muxp),
           (unsigned)muxn,
           sensorarrayLogAdsMuxName(muxn),
           (long)*outRaw,
           (long)*outUv,
           statusByte,
           discardFirst ? 1u : 0u,
           (unsigned)totalDiscard);
    sensorarrayLogDbgExtraCaptureCtrl();
    return ESP_OK;
}

esp_err_t sensorarrayMeasureReadAdsUv(sensorarrayState_t *state,
                                      const sensorarrayAdsReadPolicy_t *policy,
                                      uint8_t dLine,
                                      bool discardFirst,
                                      int32_t *outRaw,
                                      int32_t *outUv)
{
    if (!state || !outRaw || !outUv || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t muxp = 0;
    uint8_t muxn = 0;
    if (!sensorarrayBoardMapAdsMuxForDLine(dLine, &muxp, &muxn)) {
        return ESP_ERR_INVALID_ARG;
    }

    return sensorarrayMeasureReadAdsPairUv(state, policy, muxp, muxn, discardFirst, outRaw, outUv, NULL);
}

sensorarrayResConvertResult_t sensorarrayMeasureTryResistanceMohm(int32_t uv, int32_t *outMohm)
{
    if (!outMohm) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }
    if (uv < 0) {
        return SENSORARRAY_RES_CONVERT_SIGNED_INPUT;
    }
    if (uv == 0 || (uint32_t)uv >= SENSORARRAY_RESIST_EXCITATION_UV) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    int64_t numerator = (int64_t)SENSORARRAY_RESIST_REF_OHMS * 1000 * (int64_t)uv;
    int64_t denominator = (int64_t)SENSORARRAY_RESIST_EXCITATION_UV - (int64_t)uv;
    if (denominator == 0) {
        return SENSORARRAY_RES_CONVERT_MODEL_INVALID;
    }

    *outMohm = (int32_t)(numerator / denominator);
    return SENSORARRAY_RES_CONVERT_OK;
}

const char *sensorarrayMeasureDividerModelStatus(int32_t uv, int32_t *outMohm, bool *outHaveMohm)
{
    if (outHaveMohm) {
        *outHaveMohm = false;
    }

    sensorarrayResConvertResult_t resResult = sensorarrayMeasureTryResistanceMohm(uv, outMohm);
    if (resResult == SENSORARRAY_RES_CONVERT_OK) {
        if (outHaveMohm) {
            *outHaveMohm = true;
        }
        return "divider_model_ok";
    }
    if (resResult == SENSORARRAY_RES_CONVERT_SIGNED_INPUT) {
        return "negative_uv";
    }
    return "divider_model_invalid";
}

static sensorarrayFdcSampleStatus_t sensorarrayMeasureMapFdcStatus(Fdc2214CapSampleStatus_t sampleStatus)
{
    switch (sampleStatus) {
    case FDC2214_SAMPLE_STATUS_SAMPLE_VALID:
        return SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID;
    case FDC2214_SAMPLE_STATUS_STILL_SLEEPING:
        return SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING;
    case FDC2214_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
        return SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING;
    case FDC2214_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
        return SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION;
    case FDC2214_SAMPLE_STATUS_ZERO_RAW_INVALID:
        return SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID;
    case FDC2214_SAMPLE_STATUS_WATCHDOG_FAULT:
        return SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT;
    case FDC2214_SAMPLE_STATUS_AMPLITUDE_FAULT:
        return SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT;
    case FDC2214_SAMPLE_STATUS_CONFIG_UNKNOWN:
    default:
        return SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN;
    }
}

const char *sensorarrayMeasureFdcSampleStatusName(sensorarrayFdcSampleStatus_t status)
{
    switch (status) {
    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR:
        return "i2c_read_error";
    case SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN:
        return "config_unknown";
    case SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING:
        return "still_sleeping";
    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
        return "i2c_read_ok_but_not_converting";
    case SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
        return "no_unread_conversion";
    case SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID:
        return "zero_raw_invalid";
    case SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT:
        return "watchdog_fault";
    case SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT:
        return "amplitude_fault";
    case SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID:
        return "sample_valid";
    default:
        return "config_unknown";
    }
}

typedef esp_err_t (*sensorarrayFdcReadSampleFn_t)(Fdc2214CapDevice_t *dev,
                                                   Fdc2214CapChannel_t ch,
                                                   Fdc2214CapSample_t *outSample);

static esp_err_t sensorarrayMeasureReadFdcSampleDiagWithReader(sensorarrayFdcReadSampleFn_t readFn,
                                                               Fdc2214CapDevice_t *dev,
                                                               Fdc2214CapChannel_t ch,
                                                               bool discardFirst,
                                                               bool idOk,
                                                               bool configOk,
                                                               bool relaxedMode,
                                                               sensorarrayFdcReadDiag_t *outDiag)
{
    if (!readFn || !dev || !outDiag) {
        return ESP_ERR_INVALID_ARG;
    }

    *outDiag = (sensorarrayFdcReadDiag_t){
        .err = ESP_FAIL,
        .i2cOk = false,
        .idOk = idOk,
        .configOk = configOk,
        .statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR,
        .sampleValid = false,
        .provisionalReadable = false,
        .qualityDegraded = false,
    };

    if (discardFirst) {
        Fdc2214CapSample_t throwaway = {0};
        esp_err_t discardErr = readFn(dev, ch, &throwaway);
        if (discardErr != ESP_OK) {
            outDiag->err = discardErr;
            return discardErr;
        }
    }

    esp_err_t err = readFn(dev, ch, &outDiag->sample);
    outDiag->err = err;
    outDiag->i2cOk = (err == ESP_OK);
    if (err != ESP_OK) {
        outDiag->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        return err;
    }

    outDiag->coreRegs.Status = outDiag->sample.StatusRaw;
    outDiag->coreRegs.Config = outDiag->sample.ConfigRaw;
    outDiag->coreRegs.MuxConfig = outDiag->sample.MuxRaw;
    (void)Fdc2214CapReadRawRegisters(dev, 0x19u, &outDiag->coreRegs.StatusConfig);

    uint16_t statusRaw = outDiag->sample.StatusRaw;
    outDiag->status = (Fdc2214CapStatus_t){
        .Raw = statusRaw,
        .ErrorChannel = (uint8_t)((statusRaw >> SENSORARRAY_FDC_STATUS_ERR_CHAN_SHIFT) & 0x3u),
        .ErrWatchdog = (statusRaw & SENSORARRAY_FDC_STATUS_ERR_WD_MASK) != 0u,
        .ErrAmplitudeHigh = (statusRaw & SENSORARRAY_FDC_STATUS_ERR_AHW_MASK) != 0u,
        .ErrAmplitudeLow = (statusRaw & SENSORARRAY_FDC_STATUS_ERR_ALW_MASK) != 0u,
        .DataReady = (statusRaw & SENSORARRAY_FDC_STATUS_DRDY_MASK) != 0u,
        .UnreadConversion = {
            (statusRaw & SENSORARRAY_FDC_STATUS_UNREAD_CH0_MASK) != 0u,
            (statusRaw & SENSORARRAY_FDC_STATUS_UNREAD_CH1_MASK) != 0u,
            (statusRaw & SENSORARRAY_FDC_STATUS_UNREAD_CH2_MASK) != 0u,
            (statusRaw & SENSORARRAY_FDC_STATUS_UNREAD_CH3_MASK) != 0u,
        },
    };

    outDiag->converting = outDiag->sample.Converting;
    outDiag->unreadConversionPresent = outDiag->sample.UnreadConversionPresent;

    sensorarrayFdcSampleStatus_t mappedStatus = sensorarrayMeasureMapFdcStatus(outDiag->sample.SampleStatus);
    if (!idOk || !configOk) {
        mappedStatus = SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN;
    }
    outDiag->statusCode = mappedStatus;
    outDiag->qualityDegraded = (!outDiag->sample.UnreadConversionPresent) ||
                               outDiag->sample.ErrWatchdog ||
                               outDiag->sample.ErrAmplitude;
    outDiag->provisionalReadable = idOk && configOk && outDiag->sample.Converting && (outDiag->sample.Raw28 != 0u);
    outDiag->sampleValid = relaxedMode ? outDiag->provisionalReadable
                                       : (mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID);
    if (!idOk || !configOk) {
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
    }
    return ESP_OK;
}

esp_err_t sensorarrayMeasureReadFdcSampleDiag(Fdc2214CapDevice_t *dev,
                                              Fdc2214CapChannel_t ch,
                                              bool discardFirst,
                                              bool idOk,
                                              bool configOk,
                                              sensorarrayFdcReadDiag_t *outDiag)
{
    return sensorarrayMeasureReadFdcSampleDiagWithReader(Fdc2214CapReadSample,
                                                         dev,
                                                         ch,
                                                         discardFirst,
                                                         idOk,
                                                         configOk,
                                                         false,
                                                         outDiag);
}

esp_err_t sensorarrayMeasureReadFdcSampleDiagRelaxed(Fdc2214CapDevice_t *dev,
                                                     Fdc2214CapChannel_t ch,
                                                     bool discardFirst,
                                                     bool idOk,
                                                     bool configOk,
                                                     sensorarrayFdcReadDiag_t *outDiag)
{
    return sensorarrayMeasureReadFdcSampleDiagWithReader(Fdc2214CapReadSampleRelaxed,
                                                         dev,
                                                         ch,
                                                         discardFirst,
                                                         idOk,
                                                         configOk,
                                                         true,
                                                         outDiag);
}

esp_err_t sensorarrayMeasureReadFdcSample(Fdc2214CapDevice_t *dev,
                                          Fdc2214CapChannel_t ch,
                                          bool discardFirst,
                                          Fdc2214CapSample_t *outSample)
{
    if (!dev || !outSample) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcReadDiag_t diag = {0};
    esp_err_t err = sensorarrayMeasureReadFdcSampleDiag(dev, ch, discardFirst, true, true, &diag);
    if (err != ESP_OK) {
        return err;
    }
    *outSample = diag.sample;
    return ESP_OK;
}

double sensorarrayMeasureFdcRawToFrequencyHz(uint32_t raw28, uint32_t refClockHz)
{
    if (raw28 == 0u || refClockHz == 0u) {
        return 0.0;
    }
    return ((double)raw28 * (double)refClockHz) / SENSORARRAY_FDC_RAW_SCALE_2P28;
}

bool sensorarrayMeasureFdcTryCapacitancePf(double frequencyHz, uint32_t inductorUh, double *outCapPf)
{
    return sensorarrayMeasureFdcComputeCapacitancePf(frequencyHz, (double)inductorUh, outCapPf);
}

bool sensorarrayMeasureFdcComputeCapacitancePf(double frequencyHz, double inductorValueUh, double *outCapPf)
{
    if (!outCapPf || frequencyHz <= 0.0 || inductorValueUh <= 0.0) {
        return false;
    }

    // Unit conversion: uH -> H.
    double inductorH = inductorValueUh * 1e-6;
    double omega = 2.0 * SENSORARRAY_PI * frequencyHz;
    double denom = omega * omega * inductorH;
    if (denom <= 0.0) {
        return false;
    }

    // Unit conversion: F -> pF.
    *outCapPf = (1.0 / denom) * 1e12;
    return true;
}

esp_err_t sensorarrayMeasureAdsReadRegister(sensorarrayState_t *state, uint8_t reg, uint8_t *outValue)
{
    if (!state || !outValue || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    return ads126xAdcReadRegisters(&state->ads, reg, outValue, 1);
}

esp_err_t sensorarrayMeasureReadAdsKeyRegisterSnapshot(sensorarrayState_t *state,
                                                       sensorarrayAdsRegSnapshot_t *outSnapshot)
{
    if (!state || !outSnapshot) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSnapshot = (sensorarrayAdsRegSnapshot_t){0};

    esp_err_t err = sensorarrayMeasureAdsReadRegister(state, SENSORARRAY_ADS_REG_ID, &outSnapshot->id);
    if (err == ESP_OK) {
        err = ads126xAdcReadCoreRegisters(&state->ads,
                                          &outSnapshot->power,
                                          &outSnapshot->iface,
                                          &outSnapshot->mode2,
                                          &outSnapshot->inpmux,
                                          &outSnapshot->refmux);
    }

    if (err == ESP_OK) {
        state->adsRefMux = outSnapshot->refmux;
        state->adsRefMuxValid = true;
    }

    return err;
}

esp_err_t sensorarrayMeasureDumpAdsKeyRegisters(sensorarrayState_t *state, const char *stage)
{
    sensorarrayAdsRegSnapshot_t regs = {0};
    esp_err_t err = sensorarrayMeasureReadAdsKeyRegisterSnapshot(state, &regs);

    printf("DBGADSREG,stage=%s,id=0x%02X,power=0x%02X,interface=0x%02X,mode2=0x%02X,inpmux=0x%02X,refmux=0x%02X,"
           "err=%ld,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           regs.id,
           regs.power,
           regs.iface,
           regs.mode2,
           regs.inpmux,
           regs.refmux,
           (long)err,
           (err == ESP_OK) ? "ok" : "read_error");
    return err;
}
