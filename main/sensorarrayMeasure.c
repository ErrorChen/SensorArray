#include "sensorarrayMeasure.h"

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayLog.h"
#include "sensorarrayRecovery.h"

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
#define SENSORARRAY_LEVEL_IGNORE (-2)
#define SENSORARRAY_ROUTE_GPIO_VERIFY_RETRY_MAX 1u

static const char *sensorarrayRouteSelBSemanticName(uint8_t dLine, int selBLevel);

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static uint32_t sensorarrayTickToMs(TickType_t tick)
{
    return (uint32_t)tick * (uint32_t)portTICK_PERIOD_MS;
}

static void sensorarrayLogRouteProgress(const char *stage,
                                        const char *phase,
                                        const char *label,
                                        uint8_t sColumn,
                                        uint8_t dLine,
                                        sensorarrayDebugPath_t path,
                                        tmux1108Source_t swSource,
                                        sensorarraySelaRoute_t selaRoute,
                                        bool selBLevel,
                                        TickType_t enterTick,
                                        esp_err_t err)
{
    TickType_t nowTick = xTaskGetTickCount();
    uint32_t elapsedMs = sensorarrayTickToMs(nowTick - enterTick);
    uint32_t uptimeMs = sensorarrayTickToMs(nowTick);
    int selaWriteLevel = -1;
    int swWriteLevel = sensorarrayBoardMapSwSourceToGpioLevel(swSource);
    bool haveSelaWrite = sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel);
    char selaWriteBuf[8] = {0};
    char swWriteBuf[8] = {0};

    printf("DBGROUTE,stage=%s_%s,label=%s,sColumn=%u,dLine=%u,path=%s,swSemantic=%s,swWrite=%s,selaRoute=%s,selaWrite=%s,"
           "selBLevel=%u,selBSemantic=%s,elapsedMs=%lu,err=%ld,tick=%lu,uptimeMs=%lu\n",
           stage ? stage : SENSORARRAY_NA,
           phase ? phase : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogDebugPathName(path),
           sensorarrayBoardMapSwSourceSemanticName(swSource),
           sensorarrayLogFmtGpioLevel(swWriteBuf, sizeof(swWriteBuf), true, swWriteLevel),
           sensorarrayBoardMapSelaRouteName(selaRoute),
           sensorarrayLogFmtGpioLevel(selaWriteBuf, sizeof(selaWriteBuf), haveSelaWrite, selaWriteLevel),
           selBLevel ? 1u : 0u,
           sensorarrayRouteSelBSemanticName(dLine, selBLevel ? 1 : 0),
           (unsigned long)elapsedMs,
           (long)err,
           (unsigned long)nowTick,
           (unsigned long)uptimeMs);

    if (sensorarrayRecoveryIsS5d5Mode()) {
        printf("DBGFDC_S5D5,stage=route_progress,phase=%s,routeStage=%s,label=%s,sColumn=%u,dLine=%u,path=%s,"
               "swSemantic=%s,swWrite=%d,selaRoute=%s,selBLevel=%u,selBSemantic=%s,elapsedMs=%lu,err=%ld,tick=%lu\n",
               phase ? phase : SENSORARRAY_NA,
               stage ? stage : SENSORARRAY_NA,
               label ? label : SENSORARRAY_NA,
               (unsigned)sColumn,
               (unsigned)dLine,
               sensorarrayLogDebugPathName(path),
               sensorarrayBoardMapSwSourceSemanticName(swSource),
               swWriteLevel,
               sensorarrayBoardMapSelaRouteName(selaRoute),
               selBLevel ? 1u : 0u,
               sensorarrayRouteSelBSemanticName(dLine, selBLevel ? 1 : 0),
               (unsigned long)elapsedMs,
               (long)err,
               (unsigned long)nowTick);
    }
}

static void sensorarrayRouteKickAndCheckpoint(sensorarrayRecoveryStage_t stage,
                                              uint8_t sColumn,
                                              uint8_t dLine,
                                              sensorarrayRecoveryCheckpointEvent_t checkpointEvent)
{
    sensorarrayRecoveryKick(stage, sColumn, dLine);
    sensorarrayRecoveryTaskWdtReset();
    sensorarrayRecoveryEmitCheckpoint(checkpointEvent);
}

static void sensorarrayRouteDelayAndWatchdog(uint32_t delayMs)
{
    if (delayMs == 0u) {
        return;
    }
    sensorarrayDelayMs(delayMs);
    sensorarrayRecoveryKickAlive();
    sensorarrayRecoveryTaskWdtReset();
}

typedef struct {
    bool ctrlStateReadOk;
    bool commandMatch;
    bool observedMatch;
    bool semanticMatch;
    bool consistent;
    tmuxSwitchControlState_t ctrl;
    int expectedA0;
    int expectedA1;
    int expectedA2;
    int expectedSw;
    int expectedSela;
    int expectedSelB;
    int expectedSel3;
    int expectedSel4;
    bool expectedSel3Valid;
    bool expectedSel4Valid;
    bool cmdSwSemanticValid;
    bool obsSwSemanticValid;
    tmux1108Source_t cmdSwSemantic;
    tmux1108Source_t obsSwSemantic;
    bool cmdSelaSemanticValid;
    bool obsSelaSemanticValid;
    sensorarraySelaRoute_t cmdSelaSemantic;
    sensorarraySelaRoute_t obsSelaSemantic;
} sensorarrayRouteConsistency_t;

static const char *sensorarrayRouteSelBSemanticName(uint8_t dLine, int selBLevel)
{
    if (dLine >= 5u && dLine <= 8u) {
        return (selBLevel != 0) ? "capacitive_D5_D8_side" : "resistive_D5_D8_side";
    }
    return (selBLevel != 0) ? "SxA" : "SxB";
}

static const char *sensorarrayRouteSemanticNameOrNa(bool valid, const char *name)
{
    return valid && name ? name : SENSORARRAY_NA;
}

static bool sensorarrayLevelMatchesOrIgnored(int expected, int value)
{
    return (expected == SENSORARRAY_LEVEL_IGNORE) || (expected == value);
}

static esp_err_t sensorarrayMeasureVerifyControlSnapshot(const char *stage,
                                                         const char *label,
                                                         int expectedA0,
                                                         int expectedA1,
                                                         int expectedA2,
                                                         int expectedSw,
                                                         int expectedSela,
                                                         int expectedSelb,
                                                         int expectedEn)
{
    tmuxSwitchControlState_t ctrl = {0};
    for (uint32_t attempt = 0u; attempt <= SENSORARRAY_ROUTE_GPIO_VERIFY_RETRY_MAX; ++attempt) {
        esp_err_t err = tmuxSwitchSnapshotControlState(&ctrl);
        if (err != ESP_OK) {
            printf("DBGROUTE_VERIFY,stage=%s,label=%s,attempt=%lu,status=ctrl_snapshot_failed,err=%ld\n",
                   stage ? stage : SENSORARRAY_NA,
                   label ? label : SENSORARRAY_NA,
                   (unsigned long)attempt,
                   (long)err);
            return err;
        }

        bool cmdMatch = sensorarrayLevelMatchesOrIgnored(expectedA0, ctrl.cmdA0Level) &&
                        sensorarrayLevelMatchesOrIgnored(expectedA1, ctrl.cmdA1Level) &&
                        sensorarrayLevelMatchesOrIgnored(expectedA2, ctrl.cmdA2Level) &&
                        sensorarrayLevelMatchesOrIgnored(expectedSw, ctrl.cmdSwLevel) &&
                        sensorarrayLevelMatchesOrIgnored(expectedSela, ctrl.cmdSelaLevel) &&
                        sensorarrayLevelMatchesOrIgnored(expectedSelb, ctrl.cmdSelbLevel) &&
                        sensorarrayLevelMatchesOrIgnored(expectedEn, ctrl.cmdEnLevel);
        bool obsMatch = sensorarrayLevelMatchesOrIgnored(expectedA0, ctrl.obsA0Level) &&
                        sensorarrayLevelMatchesOrIgnored(expectedA1, ctrl.obsA1Level) &&
                        sensorarrayLevelMatchesOrIgnored(expectedA2, ctrl.obsA2Level) &&
                        sensorarrayLevelMatchesOrIgnored(expectedSw, ctrl.obsSwLevel) &&
                        sensorarrayLevelMatchesOrIgnored(expectedSela, ctrl.obsSelaLevel) &&
                        sensorarrayLevelMatchesOrIgnored(expectedSelb, ctrl.obsSelbLevel) &&
                        sensorarrayLevelMatchesOrIgnored(expectedEn, ctrl.obsEnLevel);
        bool verifiedMatch = sensorarrayLevelMatchesOrIgnored(expectedA0, ctrl.verifiedA0Level) &&
                             sensorarrayLevelMatchesOrIgnored(expectedA1, ctrl.verifiedA1Level) &&
                             sensorarrayLevelMatchesOrIgnored(expectedA2, ctrl.verifiedA2Level) &&
                             sensorarrayLevelMatchesOrIgnored(expectedSw, ctrl.verifiedSwLevel) &&
                             sensorarrayLevelMatchesOrIgnored(expectedSela, ctrl.verifiedSelaLevel) &&
                             sensorarrayLevelMatchesOrIgnored(expectedSelb, ctrl.verifiedSelbLevel) &&
                             sensorarrayLevelMatchesOrIgnored(expectedEn, ctrl.verifiedEnLevel);
        if (cmdMatch && obsMatch && verifiedMatch) {
            return ESP_OK;
        }
        if (attempt < SENSORARRAY_ROUTE_GPIO_VERIFY_RETRY_MAX) {
            sensorarrayDelayMs(1u);
        }
    }

    printf("DBGROUTE_VERIFY,stage=%s,label=%s,status=cmd_obs_verified_mismatch,expA0=%d,expA1=%d,expA2=%d,"
           "expSW=%d,expSELA=%d,expSELB=%d,expEN=%d,cmdA0=%d,cmdA1=%d,cmdA2=%d,cmdSW=%d,cmdSELA=%d,cmdSELB=%d,"
           "cmdEN=%d,obsA0=%d,obsA1=%d,obsA2=%d,obsSW=%d,obsSELA=%d,obsSELB=%d,obsEN=%d,verifiedA0=%d,verifiedA1=%d,"
           "verifiedA2=%d,verifiedSW=%d,verifiedSELA=%d,verifiedSELB=%d,verifiedEN=%d\n",
           stage ? stage : SENSORARRAY_NA,
           label ? label : SENSORARRAY_NA,
           expectedA0,
           expectedA1,
           expectedA2,
           expectedSw,
           expectedSela,
           expectedSelb,
           expectedEn,
           ctrl.cmdA0Level,
           ctrl.cmdA1Level,
           ctrl.cmdA2Level,
           ctrl.cmdSwLevel,
           ctrl.cmdSelaLevel,
           ctrl.cmdSelbLevel,
           ctrl.cmdEnLevel,
           ctrl.obsA0Level,
           ctrl.obsA1Level,
           ctrl.obsA2Level,
           ctrl.obsSwLevel,
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsEnLevel,
           ctrl.verifiedA0Level,
           ctrl.verifiedA1Level,
           ctrl.verifiedA2Level,
           ctrl.verifiedSwLevel,
           ctrl.verifiedSelaLevel,
           ctrl.verifiedSelbLevel,
           ctrl.verifiedEnLevel);
    return ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t sensorarrayMeasureEvaluateRouteConsistency(uint8_t sColumn,
                                                            uint8_t dLine,
                                                            tmux1108Source_t swSource,
                                                            sensorarraySelaRoute_t selaRoute,
                                                            bool selBLevel,
                                                            sensorarrayRouteConsistency_t *outConsistency)
{
    if (!outConsistency) {
        return ESP_ERR_INVALID_ARG;
    }

    *outConsistency = (sensorarrayRouteConsistency_t){0};

    sensorarrayRouteConsistency_t *consistency = outConsistency;
    consistency->expectedA0 = (int)((sColumn - 1u) & 0x1u);
    consistency->expectedA1 = (int)(((sColumn - 1u) >> 1u) & 0x1u);
    consistency->expectedA2 = (int)(((sColumn - 1u) >> 2u) & 0x1u);
    consistency->expectedSw = sensorarrayBoardMapSwSourceToGpioLevel(swSource);
    consistency->expectedSelB = selBLevel ? 1 : 0;
    consistency->expectedSel3 = 0;
    consistency->expectedSel4 = 0;
    consistency->expectedSel3Valid = (CONFIG_TMUX1134_SEL3_GPIO >= 0);
    consistency->expectedSel4Valid = (CONFIG_TMUX1134_SEL4_GPIO >= 0);
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &consistency->expectedSela)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmuxSwitchSnapshotControlState(&consistency->ctrl);
    if (err != ESP_OK) {
        return err;
    }
    consistency->ctrlStateReadOk = true;

    bool cmdSel3Match = !consistency->expectedSel3Valid || (consistency->ctrl.cmdSel3Level == consistency->expectedSel3);
    bool cmdSel4Match = !consistency->expectedSel4Valid || (consistency->ctrl.cmdSel4Level == consistency->expectedSel4);
    bool obsSel3Match = !consistency->expectedSel3Valid || (consistency->ctrl.obsSel3Level == consistency->expectedSel3);
    bool obsSel4Match = !consistency->expectedSel4Valid || (consistency->ctrl.obsSel4Level == consistency->expectedSel4);

    consistency->commandMatch = (consistency->ctrl.cmdA0Level == consistency->expectedA0) &&
                                (consistency->ctrl.cmdA1Level == consistency->expectedA1) &&
                                (consistency->ctrl.cmdA2Level == consistency->expectedA2) &&
                                (consistency->ctrl.cmdSwLevel == consistency->expectedSw) &&
                                (consistency->ctrl.cmdSelaLevel == consistency->expectedSela) &&
                                (consistency->ctrl.cmdSelbLevel == consistency->expectedSelB) &&
                                cmdSel3Match &&
                                cmdSel4Match;

    consistency->observedMatch = (consistency->ctrl.obsA0Level == consistency->expectedA0) &&
                                 (consistency->ctrl.obsA1Level == consistency->expectedA1) &&
                                 (consistency->ctrl.obsA2Level == consistency->expectedA2) &&
                                 (consistency->ctrl.obsSwLevel == consistency->expectedSw) &&
                                 (consistency->ctrl.obsSelaLevel == consistency->expectedSela) &&
                                 (consistency->ctrl.obsSelbLevel == consistency->expectedSelB) &&
                                 obsSel3Match &&
                                 obsSel4Match;

    consistency->cmdSwSemanticValid =
        sensorarrayBoardMapSwSourceFromGpioLevel(consistency->ctrl.cmdSwLevel, &consistency->cmdSwSemantic);
    consistency->obsSwSemanticValid =
        sensorarrayBoardMapSwSourceFromGpioLevel(consistency->ctrl.obsSwLevel, &consistency->obsSwSemantic);
    consistency->cmdSelaSemanticValid =
        sensorarrayBoardMapSelaRouteFromGpioLevel(consistency->ctrl.cmdSelaLevel, &consistency->cmdSelaSemantic);
    consistency->obsSelaSemanticValid =
        sensorarrayBoardMapSelaRouteFromGpioLevel(consistency->ctrl.obsSelaLevel, &consistency->obsSelaSemantic);

    consistency->semanticMatch = consistency->cmdSwSemanticValid &&
                                 consistency->obsSwSemanticValid &&
                                 consistency->cmdSelaSemanticValid &&
                                 consistency->obsSelaSemanticValid &&
                                 (consistency->cmdSwSemantic == swSource) &&
                                 (consistency->obsSwSemantic == swSource) &&
                                 (consistency->cmdSelaSemantic == selaRoute) &&
                                 (consistency->obsSelaSemantic == selaRoute);
    consistency->consistent = consistency->commandMatch && consistency->observedMatch && consistency->semanticMatch;
    (void)dLine;
    return ESP_OK;
}

static void sensorarrayMeasureLogRouteSnapshot(const char *label,
                                               uint8_t sColumn,
                                               uint8_t dLine,
                                               sensorarrayDebugPath_t path,
                                               tmux1108Source_t swSource,
                                               sensorarraySelaRoute_t selaRoute,
                                               bool selBLevel,
                                               const sensorarrayRouteConsistency_t *consistency)
{
    if (!consistency) {
        return;
    }

    const tmuxSwitchControlState_t *ctrl = &consistency->ctrl;
    const char *cmdSwSemantic =
        sensorarrayRouteSemanticNameOrNa(consistency->cmdSwSemanticValid,
                                         sensorarrayBoardMapSwSourceSemanticName(consistency->cmdSwSemantic));
    const char *obsSwSemantic =
        sensorarrayRouteSemanticNameOrNa(consistency->obsSwSemanticValid,
                                         sensorarrayBoardMapSwSourceSemanticName(consistency->obsSwSemantic));
    const char *cmdSelaSemantic =
        sensorarrayRouteSemanticNameOrNa(consistency->cmdSelaSemanticValid,
                                         sensorarrayBoardMapSelaRouteName(consistency->cmdSelaSemantic));
    const char *obsSelaSemantic =
        sensorarrayRouteSemanticNameOrNa(consistency->obsSelaSemanticValid,
                                         sensorarrayBoardMapSelaRouteName(consistency->obsSelaSemantic));

    printf("DBGROUTE_SNAPSHOT,routeLabel=%s,targetPath=S%uD%u_%s,swSemanticExpected=%s,selaSemanticExpected=%s,"
           "selBSemanticExpected=%s,cmdA0=%d,cmdA1=%d,cmdA2=%d,cmdSW=%d,cmdSELA=%d,cmdSELB=%d,cmdSEL3=%d,cmdSEL4=%d,"
           "obsA0=%d,obsA1=%d,obsA2=%d,obsSW=%d,obsSELA=%d,obsSELB=%d,obsSEL3=%d,obsSEL4=%d,cmdSwSemantic=%s,"
           "obsSwSemantic=%s,cmdSelaSemantic=%s,obsSelaSemantic=%s,routeConsistency=%s\n",
           label ? label : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogDebugPathName(path),
           sensorarrayBoardMapSwSourceSemanticName(swSource),
           sensorarrayBoardMapSelaRouteName(selaRoute),
           sensorarrayRouteSelBSemanticName(dLine, selBLevel ? 1 : 0),
           ctrl->cmdA0Level,
           ctrl->cmdA1Level,
           ctrl->cmdA2Level,
           ctrl->cmdSwLevel,
           ctrl->cmdSelaLevel,
           ctrl->cmdSelbLevel,
           ctrl->cmdSel3Level,
           ctrl->cmdSel4Level,
           ctrl->obsA0Level,
           ctrl->obsA1Level,
           ctrl->obsA2Level,
           ctrl->obsSwLevel,
           ctrl->obsSelaLevel,
           ctrl->obsSelbLevel,
           ctrl->obsSel3Level,
           ctrl->obsSel4Level,
           cmdSwSemantic,
           obsSwSemantic,
           cmdSelaSemantic,
           obsSelaSemantic,
           consistency->consistent ? "pass" : "fail");
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
    TickType_t stageTick = xTaskGetTickCount();

    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sColumn < 1u || sColumn > 8u || dLine < 1u || dLine > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_ARG;
    }

    const int expectedA0 = (int)((sColumn - 1u) & 0x1u);
    const int expectedA1 = (int)(((sColumn - 1u) >> 1u) & 0x1u);
    const int expectedA2 = (int)(((sColumn - 1u) >> 2u) & 0x1u);
    const int expectedSw = sensorarrayBoardMapSwSourceToGpioLevel(swSource);
    int expectedSela = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &expectedSela)) {
        return ESP_ERR_INVALID_ARG;
    }
    const int expectedSelB = selBLevel ? 1 : 0;
    const int expectedEnOn = (CONFIG_TMUX1134_EN_GPIO >= 0)
                                 ? (CONFIG_TMUX1134_EN_OFF_LEVEL ? 0 : 1)
                                 : SENSORARRAY_LEVEL_IGNORE;

    sensorarrayLogRouteProgress("ads_stop",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    if (err != ESP_OK) {
        sensorarrayLogRouteProgress("ads_stop",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("ads_stop",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "stop_ads_error");
        return err;
    }
    sensorarrayLogRouteProgress("ads_stop",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    sensorarrayLogRouteStep("ads_stop",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            err,
                            adsStopNeeded ? "stop_ads_before_route" : "ads_already_stopped");

    stageTick = xTaskGetTickCount();
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_ROW_ENTER,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_ROW_ENTER);
    sensorarrayLogRouteProgress("row",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    if (err != ESP_OK) {
        sensorarrayLogRouteProgress("row",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("row", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_row_failed");
        return err;
    }
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_ROW_EXIT,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_ROW_EXIT);
    sensorarrayLogRouteProgress("row",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    sensorarrayLogRouteStep("row", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_row");
    err = sensorarrayMeasureVerifyControlSnapshot("row_verify",
                                                  label,
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE);
    if (err != ESP_OK) {
        sensorarrayLogRouteStep("row_verify",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "row_cmd_obs_verified_mismatch");
        return err;
    }
    sensorarrayRouteDelayAndWatchdog(delayAfterRowMs);

    stageTick = xTaskGetTickCount();
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_SELA_ENTER,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELA_ENTER);
    sensorarrayLogRouteProgress("selA",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    err = sensorarrayMeasureSetSelaPath(state, selaRoute, delayAfterSelAMs, "selA", label);
    if (err != ESP_OK) {
        sensorarrayLogRouteProgress("selA",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("selA",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "set_sela_path_failed");
        return err;
    }
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_SELA_EXIT,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELA_EXIT);
    sensorarrayLogRouteProgress("selA",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
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
    err = sensorarrayMeasureVerifyControlSnapshot("selA_verify",
                                                  label,
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  expectedSela,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE);
    if (err != ESP_OK) {
        sensorarrayLogRouteStep("selA_verify",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "sela_cmd_obs_verified_mismatch");
        return err;
    }

    stageTick = xTaskGetTickCount();
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_SELB_ENTER,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELB_ENTER);
    sensorarrayLogRouteProgress("selB",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    err = tmux1134SelectSelBLevel(selBLevel);
    if (err != ESP_OK) {
        sensorarrayLogRouteProgress("selB",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("selB", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_selB_failed");
        return err;
    }
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_SELB_EXIT,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SELB_EXIT);
    sensorarrayLogRouteProgress("selB",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    sensorarrayLogRouteStep("selB", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_selB");
    err = sensorarrayMeasureVerifyControlSnapshot("selB_verify",
                                                  label,
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  expectedSela,
                                                  expectedSelB,
                                                  SENSORARRAY_LEVEL_IGNORE);
    if (err != ESP_OK) {
        sensorarrayLogRouteStep("selB_verify",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "selb_cmd_obs_verified_mismatch");
        return err;
    }
    sensorarrayRouteDelayAndWatchdog(delayAfterSelBMs);

    stageTick = xTaskGetTickCount();
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_SW_ENTER,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SW_ENTER);
    sensorarrayLogRouteProgress("sw",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    err = tmuxSwitchSet1108Source(swSource);
    if (err != ESP_OK) {
        sensorarrayLogRouteProgress("sw",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("sw", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_sw_failed");
        return err;
    }
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_SW_EXIT,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_SW_EXIT);
    sensorarrayLogRouteProgress("sw",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    sensorarrayLogRouteStep("sw", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_sw");
    err = sensorarrayMeasureVerifyControlSnapshot("sw_verify",
                                                  label,
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  expectedSw,
                                                  expectedSela,
                                                  expectedSelB,
                                                  expectedEnOn);
    if (err != ESP_OK) {
        sensorarrayLogRouteStep("sw_verify",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "sw_cmd_obs_verified_mismatch");
        return err;
    }
    sensorarrayRouteDelayAndWatchdog(delayAfterSwMs);

    stageTick = xTaskGetTickCount();
    sensorarrayLogRouteProgress("sel3",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    err = tmux1134SelectSel3Level(false);
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        sensorarrayLogRouteProgress("sel3",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("sel3", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_sel3_failed");
        return err;
    }
    sensorarrayLogRouteProgress("sel3",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                err);
    sensorarrayLogRouteStep("sel3",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            (err == ESP_ERR_NOT_SUPPORTED) ? ESP_OK : err,
                            (err == ESP_ERR_NOT_SUPPORTED) ? "sel3_not_supported_skip" : "set_sel3_low");
    if (err == ESP_OK) {
        err = sensorarrayMeasureVerifyControlSnapshot("sel3_verify",
                                                      label,
                                                      expectedA0,
                                                      expectedA1,
                                                      expectedA2,
                                                      expectedSw,
                                                      expectedSela,
                                                      expectedSelB,
                                                      expectedEnOn);
        if (err != ESP_OK) {
            sensorarrayLogRouteStep("sel3_verify",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    err,
                                    "sel3_cmd_obs_verified_mismatch");
            return err;
        }
    }

    stageTick = xTaskGetTickCount();
    sensorarrayLogRouteProgress("sel4",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    err = tmux1134SelectSel4Level(false);
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        sensorarrayLogRouteProgress("sel4",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("sel4", label, sColumn, dLine, path, swSource, selaRoute, selBLevel, err, "set_sel4_failed");
        return err;
    }
    sensorarrayLogRouteProgress("sel4",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                err);
    sensorarrayLogRouteStep("sel4",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            (err == ESP_ERR_NOT_SUPPORTED) ? ESP_OK : err,
                            (err == ESP_ERR_NOT_SUPPORTED) ? "sel4_not_supported_skip" : "set_sel4_low");
    if (err == ESP_OK) {
        err = sensorarrayMeasureVerifyControlSnapshot("sel4_verify",
                                                      label,
                                                      expectedA0,
                                                      expectedA1,
                                                      expectedA2,
                                                      expectedSw,
                                                      expectedSela,
                                                      expectedSelB,
                                                      expectedEnOn);
        if (err != ESP_OK) {
            sensorarrayLogRouteStep("sel4_verify",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    err,
                                    "sel4_cmd_obs_verified_mismatch");
            return err;
        }
    }

    stageTick = xTaskGetTickCount();
    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_VERIFY_ENTER,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_VERIFY_ENTER);
    sensorarrayLogRouteProgress("route_verify",
                                "enter",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);

    char expectedSemantic[128] = {0};
    (void)snprintf(expectedSemantic,
                   sizeof(expectedSemantic),
                   "sw=%s,sela=%s,selB=%s",
                   sensorarrayBoardMapSwSourceSemanticName(swSource),
                   sensorarrayBoardMapSelaRouteName(selaRoute),
                   sensorarrayRouteSelBSemanticName(dLine, selBLevel ? 1 : 0));
    tmuxSwitchSnapshotContext_t snapshotContext = {
        .stage = "route_verify",
        .routeLabel = label ? label : SENSORARRAY_NA,
        .targetPath = sensorarrayLogDebugPathName(path),
        .expectedSemantic = expectedSemantic,
    };
    (void)tmuxSwitchLogControlSnapshot(&snapshotContext);

    sensorarrayRouteConsistency_t consistency = {0};
    err = sensorarrayMeasureEvaluateRouteConsistency(sColumn, dLine, swSource, selaRoute, selBLevel, &consistency);
    if (err != ESP_OK) {
        sensorarrayLogRouteProgress("route_verify",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("route_verify",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "raw_snapshot_read_failed");
        return err;
    }

    sensorarrayMeasureLogRouteSnapshot(label, sColumn, dLine, path, swSource, selaRoute, selBLevel, &consistency);
    sensorarrayLogDbgExtraCaptureCtrl();
    if (!consistency.consistent) {
        err = ESP_ERR_INVALID_STATE;
        sensorarrayLogRouteProgress("route_verify",
                                    "fail",
                                    label,
                                    sColumn,
                                    dLine,
                                    path,
                                    swSource,
                                    selaRoute,
                                    selBLevel,
                                    stageTick,
                                    err);
        sensorarrayLogRouteStep("route_verify",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                err,
                                "route_consistency_fail");
        printf("DBGROUTE,stage=route_verify,label=%s,sColumn=%u,dLine=%u,routeConsistency=fail,commandMatch=%u,"
               "observedMatch=%u,semanticMatch=%u,status=abort_followup\n",
               label ? label : SENSORARRAY_NA,
               (unsigned)sColumn,
               (unsigned)dLine,
               consistency.commandMatch ? 1u : 0u,
               consistency.observedMatch ? 1u : 0u,
               consistency.semanticMatch ? 1u : 0u);
        return err;
    }

    sensorarrayRouteKickAndCheckpoint(SENSORARRAY_RECOVERY_STAGE_ROUTE_VERIFY_EXIT,
                                      sColumn,
                                      dLine,
                                      SENSORARRAY_RECOVERY_CHECKPOINT_EVENT_ROUTE_VERIFY_EXIT);
    sensorarrayLogRouteProgress("route_verify",
                                "exit",
                                label,
                                sColumn,
                                dLine,
                                path,
                                swSource,
                                selaRoute,
                                selBLevel,
                                stageTick,
                                ESP_OK);
    sensorarrayLogRouteStep("route_verify",
                            label,
                            sColumn,
                            dLine,
                            path,
                            swSource,
                            selaRoute,
                            selBLevel,
                            ESP_OK,
                            "route_consistency_pass");
    printf("DBGROUTE,stage=route_verify,label=%s,sColumn=%u,dLine=%u,routeConsistency=pass,status=route_verified\n",
           label ? label : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine);
    return ESP_OK;
}

esp_err_t sensorarrayMeasureApplyS5d5SecondaryCapRouteExact(sensorarrayState_t *state, const char **outMapLabel)
{
    if (outMapLabel) {
        *outMapLabel = SENSORARRAY_NA;
    }
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t invariantErr = sensorarrayBoardMapAssertS5d5CapRouteInvariant();
    if (invariantErr != ESP_OK) {
        return invariantErr;
    }

    const sensorarrayRouteMap_t *routeMap = sensorarrayBoardMapFindRoute(SENSORARRAY_S5,
                                                                          SENSORARRAY_D5,
                                                                          SENSORARRAY_PATH_CAPACITIVE);
    if (!routeMap || routeMap->selaRoute != SENSORARRAY_SELA_ROUTE_FDC2214 || !routeMap->selBLevel) {
        printf("board_map_route_unexpected,scope=s5d5,sColumn=%u,dLine=%u,path=cap\n",
               (unsigned)SENSORARRAY_S5,
               (unsigned)SENSORARRAY_D5);
        return ESP_ERR_INVALID_STATE;
    }

    tmux1108Source_t swSource = sensorarrayBoardMapDefaultSwSource(routeMap);
    if (swSource != TMUX1108_SOURCE_GND) {
        printf("board_map_route_unexpected,scope=s5d5,reason=sw_not_gnd,swSource=%s\n",
               sensorarrayBoardMapSwSourceSemanticName(swSource));
        return ESP_ERR_INVALID_STATE;
    }

    const int expectedA0 = (int)((SENSORARRAY_S5 - 1u) & 0x1u);
    const int expectedA1 = (int)(((SENSORARRAY_S5 - 1u) >> 1u) & 0x1u);
    const int expectedA2 = (int)(((SENSORARRAY_S5 - 1u) >> 2u) & 0x1u);
    int expectedSela = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_FDC2214, &expectedSela)) {
        return ESP_ERR_INVALID_STATE;
    }
    const int expectedSelB = 1;
    const int expectedSw = sensorarrayBoardMapSwSourceToGpioLevel(swSource);
    const int expectedEnOn = (CONFIG_TMUX1134_EN_GPIO >= 0)
                                 ? (CONFIG_TMUX1134_EN_OFF_LEVEL ? 0 : 1)
                                 : SENSORARRAY_LEVEL_IGNORE;
    const int expectedEnOff = (CONFIG_TMUX1134_EN_GPIO >= 0)
                                  ? (CONFIG_TMUX1134_EN_OFF_LEVEL ? 1 : 0)
                                  : SENSORARRAY_LEVEL_IGNORE;

    printf("DBGS5D5_ROUTE,stage=apply_begin,sColumn=%u,dLine=%u,path=cap,sw=%s,sela=%s,selb=%u\n",
           (unsigned)SENSORARRAY_S5,
           (unsigned)SENSORARRAY_D5,
           sensorarrayBoardMapSwSourceSemanticName(swSource),
           sensorarrayBoardMapSelaRouteName(SENSORARRAY_SELA_ROUTE_FDC2214),
           (unsigned)expectedSelB);

    if (CONFIG_TMUX1134_EN_GPIO >= 0) {
        esp_err_t enOffErr = tmux1134SetEnLogicalState(false);
        if (enOffErr != ESP_OK) {
            printf("DBGS5D5_ROUTE,stage=en_set,status=fail,expected=%d,err=%ld\n", expectedEnOff, (long)enOffErr);
            return enOffErr;
        }
        enOffErr = sensorarrayMeasureVerifyControlSnapshot("en_set",
                                                           "S5D5_exact",
                                                           SENSORARRAY_LEVEL_IGNORE,
                                                           SENSORARRAY_LEVEL_IGNORE,
                                                           SENSORARRAY_LEVEL_IGNORE,
                                                           SENSORARRAY_LEVEL_IGNORE,
                                                           SENSORARRAY_LEVEL_IGNORE,
                                                           SENSORARRAY_LEVEL_IGNORE,
                                                           expectedEnOff);
        if (enOffErr != ESP_OK) {
            return enOffErr;
        }
    }

    esp_err_t err = tmuxSwitchSelectRow((uint8_t)(SENSORARRAY_S5 - 1u));
    printf("DBGS5D5_ROUTE,stage=row_set,status=%s,row=%u,err=%ld\n",
           (err == ESP_OK) ? "ok" : "fail",
           (unsigned)(SENSORARRAY_S5 - 1u),
           (long)err);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayMeasureVerifyControlSnapshot("row_set",
                                                  "S5D5_exact",
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetSelaPath(state,
                                        SENSORARRAY_SELA_ROUTE_FDC2214,
                                        SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                        "s5d5_exact",
                                        "S5D5_exact");
    printf("DBGS5D5_ROUTE,stage=sela_set,status=%s,expected=%d,err=%ld\n",
           (err == ESP_OK) ? "ok" : "fail",
           expectedSela,
           (long)err);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayMeasureVerifyControlSnapshot("sela_set",
                                                  "S5D5_exact",
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  expectedSela,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  SENSORARRAY_LEVEL_IGNORE);
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SelectSelBLevel(true);
    printf("DBGS5D5_ROUTE,stage=selb_set,status=%s,expected=%d,err=%ld\n",
           (err == ESP_OK) ? "ok" : "fail",
           expectedSelB,
           (long)err);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayMeasureVerifyControlSnapshot("selb_set",
                                                  "S5D5_exact",
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  SENSORARRAY_LEVEL_IGNORE,
                                                  expectedSela,
                                                  expectedSelB,
                                                  SENSORARRAY_LEVEL_IGNORE);
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchSet1108Source(swSource);
    printf("DBGS5D5_ROUTE,stage=sw_set,status=%s,expected=%d,err=%ld\n",
           (err == ESP_OK) ? "ok" : "fail",
           expectedSw,
           (long)err);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayMeasureVerifyControlSnapshot("sw_set",
                                                  "S5D5_exact",
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  expectedSw,
                                                  expectedSela,
                                                  expectedSelB,
                                                  SENSORARRAY_LEVEL_IGNORE);
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SetEnLogicalState(true);
    printf("DBGS5D5_ROUTE,stage=en_set,status=%s,expected=%d,err=%ld\n",
           (err == ESP_OK) ? "ok" : "fail",
           expectedEnOn,
           (long)err);
    if (err != ESP_OK) {
        return err;
    }
    err = sensorarrayMeasureVerifyControlSnapshot("en_set",
                                                  "S5D5_exact",
                                                  expectedA0,
                                                  expectedA1,
                                                  expectedA2,
                                                  expectedSw,
                                                  expectedSela,
                                                  expectedSelB,
                                                  expectedEnOn);
    if (err != ESP_OK) {
        return err;
    }

    sensorarrayRouteDelayAndWatchdog(SENSORARRAY_SETTLE_AFTER_PATH_MS);
    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
    printf("DBGS5D5_ROUTE,stage=apply_end,status=ok,label=%s\n", routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA);
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

    if (state->dedicatedSecondaryFdcMode) {
        bool isS5d5CapRequest = (sColumn == SENSORARRAY_S5) &&
                                (dLine == SENSORARRAY_D5) &&
                                (path == SENSORARRAY_PATH_CAPACITIVE);
        if (!isS5d5CapRequest) {
            printf("board_map_route_unexpected,scope=s5d5,reason=dedicated_mode_reject_non_s5d5,sColumn=%u,dLine=%u,path=%s\n",
                   (unsigned)sColumn,
                   (unsigned)dLine,
                   sensorarrayBoardMapPathName(path));
            return ESP_ERR_INVALID_ARG;
        }
        if (swSource != TMUX1108_SOURCE_GND) {
            printf("board_map_route_unexpected,scope=s5d5,reason=dedicated_mode_reject_sw_source,sw=%s\n",
                   sensorarrayBoardMapSwSourceSemanticName(swSource));
            return ESP_ERR_INVALID_ARG;
        }
        return sensorarrayMeasureApplyS5d5SecondaryCapRouteExact(state, outMapLabel);
    }

    const sensorarrayRouteMap_t *routeMap = sensorarrayBoardMapFindRoute(sColumn, dLine, path);
    if (!routeMap) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    sensorarrayDebugPath_t debugPath = sensorarrayBoardMapPathToDebugPath(path, swSource);
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(routeMap->selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureApplyRouteLevels(state,
                                                       sColumn,
                                                       dLine,
                                                       debugPath,
                                                       swSource,
                                                       routeMap->selaRoute,
                                                       routeMap->selBLevel,
                                                       SENSORARRAY_SETTLE_AFTER_COLUMN_MS,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_SW_MS,
                                                       routeMap->mapLabel);
    if (err != ESP_OK) {
        return err;
    }

    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
    return ESP_OK;
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
        .transportReadable = false,
        .statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR,
        .sampleValid = false,
        .provisionalReadable = false,
        .healthReadable = false,
        .shouldCountForSweep = false,
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
        outDiag->transportReadable = false;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        outDiag->healthReadable = false;
        outDiag->shouldCountForSweep = false;
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
    outDiag->transportReadable = outDiag->i2cOk && idOk && configOk;
    bool activeChannelMatch = ((uint8_t)outDiag->sample.ActiveChannel == (uint8_t)ch);
    outDiag->provisionalReadable =
        outDiag->transportReadable && outDiag->sample.Converting && (outDiag->sample.Raw28 != 0u);
    outDiag->healthReadable = outDiag->provisionalReadable &&
                              outDiag->sample.UnreadConversionPresent &&
                              !outDiag->sample.ErrWatchdog &&
                              !outDiag->sample.ErrAmplitude;
    outDiag->shouldCountForSweep = outDiag->healthReadable && activeChannelMatch;
    outDiag->qualityDegraded = outDiag->transportReadable && (!outDiag->healthReadable || !activeChannelMatch);
    outDiag->sampleValid = relaxedMode ? outDiag->healthReadable
                                       : (mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID);
    if (!outDiag->transportReadable) {
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        outDiag->healthReadable = false;
        outDiag->shouldCountForSweep = false;
        outDiag->qualityDegraded = false;
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

const char *sensorarrayMeasureFdcRefClockQualityName(sensorarrayFdcRefClockQuality_t quality)
{
    switch (quality) {
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_NOMINAL_INTERNAL:
        return "nominal_internal";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_ESTIMATED:
        return "estimated";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_CALIBRATED:
        return "calibrated";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_EXTERNAL_ASSUMED:
        return "external_assumed";
    case SENSORARRAY_FDC_REF_CLOCK_QUALITY_UNKNOWN:
    default:
        return "unknown";
    }
}

bool sensorarrayMeasureFdcRestoreFrequencyWithClockInfo(uint32_t raw28,
                                                        uint32_t refClockHz,
                                                        sensorarrayFdcRefClockQuality_t refClockQuality,
                                                        const Fdc2214CapClockDividerInfo_t *clockDividerInfo,
                                                        sensorarrayFdcFrequencyRestore_t *outRestore)
{
    if (!clockDividerInfo || !outRestore) {
        return false;
    }

    *outRestore = (sensorarrayFdcFrequencyRestore_t){
        .raw28 = raw28,
        .refClockHz = refClockHz,
        .refClockQuality = refClockQuality,
        .clockDividerInfo = *clockDividerInfo,
    };

    Fdc2214CapFrequencyInfo_t frequencyInfo = {0};
    if (!Fdc2214CapComputeSensorFrequencyHz(raw28, refClockHz, clockDividerInfo, &frequencyInfo)) {
        return false;
    }

    outRestore->fClkHz = frequencyInfo.FclkHz;
    outRestore->fRefHz = frequencyInfo.FrefHz;
    outRestore->fInHz = frequencyInfo.FinHz;
    outRestore->restoredSensorFrequencyHz = frequencyInfo.FsensorHz;
    return true;
}

bool sensorarrayMeasureFdcRestoreFrequency(const sensorarrayFdcDeviceState_t *fdcState,
                                           uint32_t raw28,
                                           sensorarrayFdcFrequencyRestore_t *outRestore)
{
    if (!outRestore) {
        return false;
    }

    uint32_t refClockHz = SENSORARRAY_FDC_REF_CLOCK_HZ;
    sensorarrayFdcRefClockQuality_t refClockQuality = SENSORARRAY_FDC_REF_CLOCK_QUALITY_UNKNOWN;
    Fdc2214CapClockDividerInfo_t clockDividerInfo = {0};
    bool haveClockDividerInfo = false;

    if (fdcState) {
        if (fdcState->refClockHz > 0u) {
            refClockHz = fdcState->refClockHz;
        }
        refClockQuality = fdcState->refClockQuality;
        if (fdcState->channel0ClockDividerValid) {
            clockDividerInfo = fdcState->channel0ClockDividerInfo;
            haveClockDividerInfo = true;
        } else if (fdcState->channel0ClockDividersRaw != 0u &&
                   Fdc2214CapDecodeClockDividers(fdcState->channel0ClockDividersRaw, &clockDividerInfo) == ESP_OK) {
            haveClockDividerInfo = true;
        }
    }

    if (!haveClockDividerInfo &&
        Fdc2214CapDecodeClockDividers(SENSORARRAY_FDC_DEBUG_CLOCK_DIVIDERS_CH0, &clockDividerInfo) == ESP_OK) {
        haveClockDividerInfo = true;
    }
    if (!haveClockDividerInfo) {
        return false;
    }

    return sensorarrayMeasureFdcRestoreFrequencyWithClockInfo(raw28,
                                                              refClockHz,
                                                              refClockQuality,
                                                              &clockDividerInfo,
                                                              outRestore);
}

double sensorarrayMeasureFdcRawToFrequencyHz(uint32_t raw28, uint32_t refClockHz)
{
    /*
     * Legacy path retained for compatibility with old logs/tests.
     * New code should use sensorarrayMeasureFdcRestoreFrequency* to include FIN_SEL/FREF_DIVIDER.
     */
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
