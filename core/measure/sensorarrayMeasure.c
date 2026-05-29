#include "sensorarrayMeasure.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayLog.h"

#ifndef CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE 0
#endif

#if CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define FDCLOW_TRACE(...) printf(__VA_ARGS__)
#else
#define FDCLOW_TRACE(...) do { } while (0)
#endif

#define SENSORARRAY_FDC_RAW_SCALE_2P28 268435456.0
#define SENSORARRAY_PI 3.14159265358979323846
#define SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE FDC2214_RR_SEQUENCE_CH0_CH1_CH2_CH3
#define SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK 0x0Fu
#define SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS CONFIG_SENSORARRAY_FDC_SWEEP_SAMPLE_TIMEOUT_MS
#define SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK 0x8000u
#define SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT 13u
#define SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK 0x6000u
#define SENSORARRAY_FDC_MUX_DEGLITCH_MASK 0x0007u
#define SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK 0x0040u
#define SENSORARRAY_FDC_REG_RCOUNT_BASE 0x08u
#define SENSORARRAY_FDC_REG_SETTLECOUNT_BASE 0x10u
#define SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE 0x14u
#define SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE 0x1Eu
#define SENSORARRAY_FDC_RAW28_SATURATED_THRESHOLD 0x0FFFFF00u

static SemaphoreHandle_t s_measureLock = NULL;
static portMUX_TYPE s_measureLockMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_fdcMatrixSequence = 0u;
static bool s_fastSpeedEnabled = false;
static uint32_t s_fdcMatrixAllInvalidSequence = 0u;

typedef struct {
    uint32_t raw28[4];
    bool valid[4];
    bool amplitudeWarning[4];
    bool watchdogFault[4];
    bool saturated[4];
    uint16_t statusRaw;
    uint8_t unreadMask;
} sensorarrayFdcAutoscanSamples_t;

typedef struct {
    bool validSeen[2][4];
    bool invalidSeen[2][4];
    bool amplitudeWarningSeen[2][4];
    bool watchdogSeen[2][4];
    bool saturatedSeen[2][4];
    bool zeroRawSeen[2][4];
    uint32_t lastRaw28[2][4];
} sensorarrayFdcFrameHealth_t;

static esp_err_t sensorarrayMeasureEnsureLock(void)
{
    if (s_measureLock) {
        return ESP_OK;
    }

    portENTER_CRITICAL(&s_measureLockMux);
    if (!s_measureLock) {
        s_measureLock = xSemaphoreCreateMutex();
    }
    portEXIT_CRITICAL(&s_measureLockMux);

    return s_measureLock ? ESP_OK : ESP_ERR_NO_MEM;
}

static esp_err_t sensorarrayMeasureTakeLock(void)
{
    esp_err_t err = sensorarrayMeasureEnsureLock();
    if (err != ESP_OK) {
        return err;
    }

    TickType_t ticks = pdMS_TO_TICKS((uint32_t)CONFIG_SENSORARRAY_MEASURE_LOCK_TIMEOUT_MS);
    if (ticks == 0) {
        ticks = 1;
    }
    return (xSemaphoreTake(s_measureLock, ticks) == pdTRUE) ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void sensorarrayMeasureGiveLock(void)
{
    if (s_measureLock) {
        xSemaphoreGive(s_measureLock);
    }
}

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

static const char *sensorarrayMatrixDSourceName(sensorarrayMatrixDSourcePolicy_t dSource)
{
    return (dSource == SENSORARRAY_MATRIX_D_SOURCE_REF) ? "REF" : "GND";
}

static const char *sensorarrayAdsIntRefPolicyName(sensorarrayAdsIntRefPolicy_t policy)
{
    switch (policy) {
    case SENSORARRAY_ADS_INTREF_ON:
        return "ON";
    case SENSORARRAY_ADS_INTREF_KEEP:
        return "KEEP";
    case SENSORARRAY_ADS_INTREF_OFF:
    default:
        return "OFF";
    }
}

static const char *sensorarrayAdsVbiasPolicyName(sensorarrayAdsVbiasPolicy_t policy)
{
    switch (policy) {
    case SENSORARRAY_ADS_VBIAS_ON:
        return "ON";
    case SENSORARRAY_ADS_VBIAS_KEEP:
        return "KEEP";
    case SENSORARRAY_ADS_VBIAS_OFF:
    default:
        return "OFF";
    }
}

static bool sensorarrayAdsIntRefPolicyUpdates(sensorarrayAdsIntRefPolicy_t policy)
{
    return policy != SENSORARRAY_ADS_INTREF_KEEP;
}

static bool sensorarrayAdsVbiasPolicyUpdates(sensorarrayAdsVbiasPolicy_t policy)
{
    return policy != SENSORARRAY_ADS_VBIAS_KEEP;
}

static int sensorarrayReadResetGpioLevel(void)
{
    if (CONFIG_BOARD_ADS126X_RESET_GPIO < 0) {
        return -1;
    }
    return gpio_get_level((gpio_num_t)CONFIG_BOARD_ADS126X_RESET_GPIO);
}

esp_err_t sensorarrayMeasureApplyRefPolicy(sensorarrayState_t *state,
                                           const char *stage,
                                           const char *mode,
                                           sensorarrayMatrixDSourcePolicy_t dSource,
                                           sensorarrayAdsIntRefPolicy_t intrefPolicy,
                                           sensorarrayAdsVbiasPolicy_t vbiasPolicy,
                                           const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    const bool updateIntref = sensorarrayAdsIntRefPolicyUpdates(intrefPolicy);
    const bool updateVbias = sensorarrayAdsVbiasPolicyUpdates(vbiasPolicy);
    const bool enableIntref = (intrefPolicy == SENSORARRAY_ADS_INTREF_ON);
    const bool enableVbias = (vbiasPolicy == SENSORARRAY_ADS_VBIAS_ON);

    uint8_t powerBefore = 0u;
    uint8_t powerAfter = 0u;
    bool havePower = false;
    esp_err_t err = ESP_OK;
    const char *status = "ok";

    if (!state->adsReady) {
        err = (updateIntref || updateVbias) ? ESP_ERR_INVALID_STATE : ESP_OK;
        status = (err == ESP_OK) ? "ads_unavailable_keep" : "ads_unavailable";
    } else if (updateIntref || updateVbias) {
        err = ads126xAdcApplyPowerPolicy(&state->ads,
                                         updateIntref,
                                         enableIntref,
                                         updateVbias,
                                         enableVbias,
                                         &powerBefore,
                                         &powerAfter);
        havePower = (err == ESP_OK);
        status = (err == ESP_OK) ? "ok" : "power_policy_error";
    } else {
        err = ads126xAdcReadPowerRegister(&state->ads, &powerBefore);
        if (err == ESP_OK) {
            powerAfter = powerBefore;
            havePower = true;
            status = "keep";
        } else {
            status = "read_power_error";
        }
    }

    if (err == ESP_OK) {
        if (intrefPolicy == SENSORARRAY_ADS_INTREF_OFF) {
            state->adsRefReady = false;
        } else if (intrefPolicy == SENSORARRAY_ADS_INTREF_ON) {
            state->adsRefReady = true;
        }
        sensorarrayLogSetAdsState(state->adsReady, state->adsRefReady);
    }

    char beforeBuf[8];
    char afterBuf[8];
    char resetBuf[8];
    printf("DBGREFPOLICY,stage=%s,mode=%s,dSource=%s,swReq=%s,intrefReq=%s,vbiasReq=%s,powerBefore=%s,"
           "powerAfter=%s,noDelay=1,adsReset=%s,reason=%s,err=%ld,status=%s\n",
           stage ? stage : SENSORARRAY_NA,
           mode ? mode : SENSORARRAY_NA,
           sensorarrayMatrixDSourceName(dSource),
           (dSource == SENSORARRAY_MATRIX_D_SOURCE_REF) ? sensorarrayLogSwSourceName(TMUX1108_SOURCE_REF)
                                                        : sensorarrayLogSwSourceName(TMUX1108_SOURCE_GND),
           sensorarrayAdsIntRefPolicyName(intrefPolicy),
           sensorarrayAdsVbiasPolicyName(vbiasPolicy),
           sensorarrayLogFmtHexU8(beforeBuf, sizeof(beforeBuf), havePower, powerBefore),
           sensorarrayLogFmtHexU8(afterBuf, sizeof(afterBuf), havePower, powerAfter),
           sensorarrayLogFmtGpioLevel(resetBuf, sizeof(resetBuf), true, sensorarrayReadResetGpioLevel()),
           reason ? reason : SENSORARRAY_NA,
           (long)err,
           status);
    return err;
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

static tmux1108Source_t sensorarrayMeasureSourceForSwPhysicalLevel(sensorarraySwPhysicalLevel_t level)
{
    const bool refSourceIsHigh = (CONFIG_TMUX1108_SW_REF_LEVEL != 0);
    if (level == SENSORARRAY_SW_PHYSICAL_HIGH) {
        return refSourceIsHigh ? TMUX1108_SOURCE_REF : TMUX1108_SOURCE_GND;
    }
    return refSourceIsHigh ? TMUX1108_SOURCE_GND : TMUX1108_SOURCE_REF;
}

esp_err_t sensorarrayMeasureSetSwPhysicalLevel(sensorarrayState_t *state,
                                               sensorarraySwPhysicalLevel_t level,
                                               const char *reason)
{
    (void)reason;
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (level != SENSORARRAY_SW_PHYSICAL_LOW && level != SENSORARRAY_SW_PHYSICAL_HIGH) {
        return ESP_ERR_INVALID_ARG;
    }

    return tmuxSwitchSet1108Source(sensorarrayMeasureSourceForSwPhysicalLevel(level));
}

static void sensorarrayMeasureDelayUs(uint32_t delayUs)
{
    if (delayUs > 0u) {
        esp_rom_delay_us(delayUs);
    }
}

static esp_err_t sensorarrayMeasureSetSelaPathQuiet(sensorarrayState_t *state,
                                                    sensorarraySelaRoute_t selaRoute,
                                                    uint32_t settleDelayUs)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    int selaWriteLevel = -1;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(selaRoute, &selaWriteLevel)) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmux1134SelectSelALevel(selaWriteLevel != 0);
    if (err == ESP_OK) {
        sensorarrayMeasureDelayUs(settleDelayUs);
    }
    return err;
}

static esp_err_t sensorarrayMeasureSetFdcSelBPathQuiet(sensorarrayState_t *state)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    bool selBLevel = false;
    if (!sensorarrayBoardMapFdcSelBLevel(&selBLevel)) {
        return ESP_ERR_INVALID_ARG;
    }
    return tmux1134SelectSelBLevel(selBLevel);
}

static esp_err_t sensorarrayMeasureForceAdsReferenceOff(sensorarrayState_t *state)
{
    if (!state || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ads126xAdcApplyPowerPolicy(&state->ads,
                                               true,
                                               false,
                                               true,
                                               false,
                                               NULL,
                                               NULL);
    if (err == ESP_OK) {
        state->adsRefReady = false;
        sensorarrayLogSetAdsState(state->adsReady, state->adsRefReady);
    }
    return err;
}

static int sensorarrayMeasureSwPhysicalReadbackFromControl(const tmuxSwitchControlState_t *ctrl)
{
    return ctrl ? ctrl->obsSwLevel : -1;
}

static void sensorarrayMeasureReadFdcPathControl(tmuxSwitchControlState_t *ctrl)
{
    if (!ctrl || tmuxSwitchGetControlState(ctrl) != ESP_OK) {
        if (ctrl) {
            *ctrl = (tmuxSwitchControlState_t){
                .cmdRow = 0xFFu,
                .cmdSwLevel = -1,
                .cmdSelaLevel = -1,
                .cmdSelbLevel = -1,
                .cmdEnLevel = -1,
                .obsSwLevel = -1,
                .obsSelaLevel = -1,
                .obsSelbLevel = -1,
                .obsEnLevel = -1,
            };
        }
    }
}

static bool sensorarrayMeasureFdcPathControlMatches(const tmuxSwitchControlState_t *ctrl)
{
    if (!ctrl) {
        return false;
    }

    int expectedSela = -1;
    bool expectedSelb = false;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_FDC2214, &expectedSela) ||
        !sensorarrayBoardMapFdcSelBLevel(&expectedSelb)) {
        return false;
    }

    int expectedSw = 1;
    bool swOk = ctrl->cmdSwLevel == expectedSw &&
                (ctrl->obsSwLevel < 0 || ctrl->obsSwLevel == expectedSw);
    bool selaOk = ctrl->cmdSelaLevel == expectedSela &&
                  (ctrl->obsSelaLevel < 0 || ctrl->obsSelaLevel == expectedSela);
    bool selbOk = ctrl->cmdSelbLevel == (expectedSelb ? 1 : 0) &&
                  (ctrl->obsSelbLevel < 0 || ctrl->obsSelbLevel == (expectedSelb ? 1 : 0));
    bool enOk = ctrl->cmdTmux1134EnLogicalOn &&
                (!ctrl->obsTmux1134EnLogicalOnValid || ctrl->obsTmux1134EnLogicalOn);
    return swOk && selaOk && selbOk && enOk;
}

esp_err_t sensorarrayMeasurePrepareFdcMatrixPath(sensorarrayState_t *state, const char *reason)
{
    const char *source = reason ? reason : SENSORARRAY_NA;
    if (!state || !state->tmuxReady || !state->adsReady) {
        printf("FDC_PATH,stage=prepare_done,reason=%s,ok=0,err=0x%lx,sw=-1,sela=-1,selb=-1,en=-1,adsRef=-1,adsVbias=-1\n",
               source,
               (unsigned long)ESP_ERR_INVALID_STATE);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    printf("FDC_PATH,stage=ads_stop,reason=%s,err=0x%lx\n", source, (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }
#if CONFIG_SENSORARRAY_ADS1263
    esp_err_t adc2Err = ads126xAdcStopAdc2(&state->ads);
    printf("FDC_PATH,stage=ads_stop2,reason=%s,err=0x%lx\n", source, (unsigned long)adc2Err);
    if (adc2Err != ESP_OK && adc2Err != ESP_ERR_NOT_SUPPORTED) {
        return adc2Err;
    }
#endif

    err = sensorarrayMeasureForceAdsReferenceOff(state);
    printf("FDC_PATH,stage=ads_ref_off,reason=%s,err=0x%lx,adsRefReady=%d\n",
           source,
           (unsigned long)err,
           state->adsRefReady ? 1 : 0);
    printf("FDC_PATH,stage=ads_vbias_off,reason=%s,err=0x%lx\n", source, (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetSelaPathQuiet(state,
                                             SENSORARRAY_SELA_ROUTE_FDC2214,
                                             (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US);
    tmuxSwitchControlState_t ctrl = {0};
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    int fdcSelaLevel = -1;
    (void)sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_FDC2214, &fdcSelaLevel);
    printf("FDC_PATH,stage=tmux1134_fdc,reason=%s,selaCmd=%d,selaReadback=%d,err=0x%lx\n",
           source,
           fdcSelaLevel,
           ctrl.obsSelaLevel,
           (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetFdcSelBPathQuiet(state);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    bool fdcSelbLevel = false;
    (void)sensorarrayBoardMapFdcSelBLevel(&fdcSelbLevel);
    printf("FDC_PATH,stage=selb_fdc,reason=%s,selbCmd=%d,selbReadback=%d,err=0x%lx\n",
           source,
           fdcSelbLevel ? 1 : 0,
           ctrl.obsSelbLevel,
           (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetSwPhysicalLevel(state,
                                               SENSORARRAY_SW_PHYSICAL_HIGH,
                                               "fdc_matrix_path");
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    printf("FDC_PATH,stage=sw_high,reason=%s,swCmd=HIGH,swReadback=%d,err=0x%lx\n",
           source,
           sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
           (unsigned long)err);
    if (err != ESP_OK) {
        return err;
    }

    err = tmux1134SetEnLogicalState(true);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    printf("FDC_PATH,stage=tmux1108_enable,reason=%s,enCmd=1,enReadback=%d,err=0x%lx\n",
           source,
           ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
           (unsigned long)err);

    bool ok = (err == ESP_OK) && sensorarrayMeasureFdcPathControlMatches(&ctrl);
    printf("FDC_PATH,stage=prepare_done,reason=%s,ok=%d,err=0x%lx,sw=%d,sela=%d,selb=%d,en=%d,adsRef=%d,adsVbias=0\n",
           source,
           ok ? 1 : 0,
           (unsigned long)(ok ? ESP_OK : ESP_ERR_INVALID_STATE),
           sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
           state->adsRefReady ? 1 : 0);
    return ok ? ESP_OK : ((err != ESP_OK) ? err : ESP_ERR_INVALID_STATE);
}

esp_err_t sensorarrayMeasureEnsureFdcMatrixPath(sensorarrayState_t *state, const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    tmuxSwitchControlState_t ctrl = {0};
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    if (!sensorarrayMeasureFdcPathControlMatches(&ctrl) || state->adsRefReady) {
        if (state->adsRefReady) {
            printf("FDC_PATH,stage=ensure_mismatch,reason=%s,field=adsRef,cmd=0,readback=%d\n",
                   reason ? reason : SENSORARRAY_NA,
                   state->adsRefReady ? 1 : 0);
        }
        if (!sensorarrayMeasureFdcPathControlMatches(&ctrl)) {
            printf("FDC_PATH,stage=ensure_mismatch,reason=%s,field=tmux_path,swCmd=%d,swReadback=%d,selaCmd=%d,selaReadback=%d,selbCmd=%d,selbReadback=%d,enCmd=%d,enReadback=%d\n",
                   reason ? reason : SENSORARRAY_NA,
                   ctrl.cmdSwLevel,
                   ctrl.obsSwLevel,
                   ctrl.cmdSelaLevel,
                   ctrl.obsSelaLevel,
                   ctrl.cmdSelbLevel,
                   ctrl.obsSelbLevel,
                   ctrl.cmdTmux1134EnLogicalOn ? 1 : 0,
                   ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : -1);
        }
        return sensorarrayMeasurePrepareFdcMatrixPath(state, reason);
    }

    printf("FDC_PATH,stage=ensure_ok,reason=%s,sw=%d,sela=%d,selb=%d,en=%d,adsRef=%d\n",
           reason ? reason : SENSORARRAY_NA,
           sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
           state->adsRefReady ? 1 : 0);
    return ESP_OK;
}

static esp_err_t sensorarrayMeasureSetSwForRoute(sensorarrayState_t *state,
                                                 const char *stage,
                                                 uint8_t sColumn,
                                                 uint8_t dLine,
                                                 sensorarrayRoutePathKind_t path,
                                                 tmux1108Source_t swSource,
                                                 sensorarraySelaRoute_t selaRoute,
                                                 bool selBLevel,
                                                 const char *label,
                                                 const char *status,
                                                 const char *reason)
{
    if (!state || !state->tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = tmuxSwitchSet1108Source(swSource);
    sensorarrayLogRouteStepEx(stage,
                              label,
                              sColumn,
                              dLine,
                              path,
                              swSource,
                              selaRoute,
                              selBLevel,
                              err,
                              status,
                              reason);
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

static const char *sensorarrayMeasureFdcDeviceName(sensorarrayFdcDeviceId_t devId)
{
    return (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? "secondary" : "primary";
}

static sensorarrayFdcDeviceId_t sensorarrayMeasureFdcDeviceIdFromState(const sensorarrayFdcDeviceState_t *fdcState)
{
    if (fdcState && fdcState->label && strstr(fdcState->label, "secondary")) {
        return SENSORARRAY_FDC_DEV_SECONDARY;
    }
    return SENSORARRAY_FDC_DEV_PRIMARY;
}

static uint8_t sensorarrayMeasureFdcUnreadMaskFromStatus(const Fdc2214CapStatus_t *status)
{
    if (!status) {
        return 0u;
    }

    uint8_t unreadMask = 0u;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if (status->UnreadConversion[ch]) {
            unreadMask |= (uint8_t)(1u << ch);
        }
    }
    return unreadMask;
}

static bool sensorarrayMeasureFdcDeglitchCodeValid(uint8_t deglitchCode)
{
    switch (deglitchCode) {
    case FDC2214_DEGLITCH_1MHZ:
    case FDC2214_DEGLITCH_3P3MHZ:
    case FDC2214_DEGLITCH_10MHZ:
    case FDC2214_DEGLITCH_33MHZ:
        return true;
    default:
        return false;
    }
}

static Fdc2214CapDeglitch_t sensorarrayMeasureSelectedFdcDeglitch(const sensorarrayFdcDeviceState_t *fdcState)
{
    uint8_t selectedCode = (uint8_t)FDC2214_DEGLITCH_10MHZ;
    uint32_t selectedBandwidthHz = 10000000u;

    if (fdcState) {
        uint8_t muxDeglitch = (uint8_t)(fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
        if (sensorarrayMeasureFdcDeglitchCodeValid(muxDeglitch)) {
            selectedCode = muxDeglitch;
        }

        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            const sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[ch];
            if (profile->valid &&
                sensorarrayMeasureFdcDeglitchCodeValid(profile->selectedDeglitchCode) &&
                profile->selectedDeglitchBandwidthHz >= selectedBandwidthHz) {
                selectedCode = profile->selectedDeglitchCode;
                selectedBandwidthHz = profile->selectedDeglitchBandwidthHz;
            }
        }
    }

    return (Fdc2214CapDeglitch_t)selectedCode;
}

static uint8_t sensorarrayMeasureFdcRegForChannel(uint8_t base, Fdc2214CapChannel_t channel)
{
    return (uint8_t)(base + (uint8_t)channel);
}

static esp_err_t sensorarrayMeasureVerifyFdcChannelConfigApplied(sensorarrayFdcDeviceState_t *fdcState)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t firstErr = ESP_OK;
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint16_t rcount = 0u;
        uint16_t settle = 0u;
        uint16_t clockDiv = 0u;
        uint16_t drive = 0u;
        esp_err_t err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                   sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_RCOUNT_BASE,
                                                                                      (Fdc2214CapChannel_t)ch),
                                                   &rcount);
        if (err == ESP_OK) {
            err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_SETTLECOUNT_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                             &settle);
        }
        if (err == ESP_OK) {
            err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                             &clockDiv);
        }
        if (err == ESP_OK) {
            err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                             sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE,
                                                                                (Fdc2214CapChannel_t)ch),
                                             &drive);
        }
        if (err == ESP_OK && (rcount == 0u || settle == 0u || clockDiv == 0u)) {
            err = ESP_ERR_INVALID_RESPONSE;
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
        if (err != ESP_OK) {
            printf("MATRIXFDC_DIAG,stage=autoscan_channel_config_invalid,device=%s,ch=%u,rCount=0x%04X,settle=0x%04X,clockDiv=0x%04X,drive=0x%04X,err=0x%lx\n",
                   fdcState->label ? fdcState->label : SENSORARRAY_NA,
                   (unsigned)ch,
                   rcount,
                   settle,
                   clockDiv,
                   drive,
                   (unsigned long)err);
        }
    }
    return firstErr;
}

static esp_err_t sensorarrayMeasureEnsureFdcAutoscan4ch(sensorarrayState_t *state,
                                                        sensorarrayFdcDeviceId_t devId)
{
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureVerifyFdcChannelConfigApplied(fdcState);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapDeglitch_t deglitch = sensorarrayMeasureSelectedFdcDeglitch(fdcState);
    err = Fdc2214CapSetAutoScanMode(fdcState->handle,
                                    SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE,
                                    deglitch);
    if (err != ESP_OK) {
        return err;
    }

    Fdc2214CapCoreRegs_t regs = {0};
    err = Fdc2214CapReadCoreRegs(fdcState->handle, &regs);
    if (err != ESP_OK) {
        return err;
    }

    fdcState->statusConfigReg = regs.StatusConfig;
    fdcState->configReg = regs.Config;
    fdcState->muxConfigReg = regs.MuxConfig;

    uint8_t rr = (uint8_t)((regs.MuxConfig & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                           SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
    uint8_t muxDeglitch = (uint8_t)(regs.MuxConfig & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
    bool autoscan = (regs.MuxConfig & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
    bool highCurrent = (regs.Config & SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;

    printf("FDC_AUTOSCAN_CONFIG,device=%s,mux=0x%04X,config=0x%04X,autoscan=%u,rr=%u,deglitch=0x%X,highCurrent=%u\n",
           sensorarrayMeasureFdcDeviceName(devId),
           regs.MuxConfig,
           regs.Config,
           autoscan ? 1u : 0u,
           (unsigned)rr,
           (unsigned)muxDeglitch,
           highCurrent ? 1u : 0u);

    if (!autoscan ||
        rr != SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE ||
        highCurrent ||
        regs.MuxConfig == 0x020Du) {
        printf("MATRIXFDC_DIAG,stage=autoscan_config_reject,device=%s,mux=0x%04X,config=0x%04X,autoscan=%u,rr=%u,highCurrent=%u,err=0x%lx\n",
               sensorarrayMeasureFdcDeviceName(devId),
               regs.MuxConfig,
               regs.Config,
               autoscan ? 1u : 0u,
               (unsigned)rr,
               highCurrent ? 1u : 0u,
               (unsigned long)ESP_ERR_INVALID_RESPONSE);
        return ESP_ERR_INVALID_RESPONSE;
    }

    return ESP_OK;
}

static esp_err_t sensorarrayMeasureWaitFdcAutoscanFrameReady(sensorarrayFdcDeviceState_t *fdcState,
                                                             uint8_t row,
                                                             uint32_t timeoutMs,
                                                             uint16_t *outStatus)
{
    if (!fdcState || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (timeoutMs == 0u) {
        timeoutMs = 1u;
    }

    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    Fdc2214CapStatus_t status = {0};
    esp_err_t lastErr = ESP_ERR_TIMEOUT;
    while (esp_timer_get_time() <= deadlineUs) {
        esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
        if (err == ESP_OK) {
            uint8_t unreadMask = sensorarrayMeasureFdcUnreadMaskFromStatus(&status);
            if (status.DataReady || unreadMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK) {
                if (outStatus) {
                    *outStatus = status.Raw;
                }
                printf("FDC_FRAME_READY,row=%u,device=%s,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
                       (unsigned)row,
                       sensorarrayMeasureFdcDeviceName(sensorarrayMeasureFdcDeviceIdFromState(fdcState)),
                       status.Raw,
                       (unsigned)unreadMask,
                       status.DataReady ? 1u : 0u,
                       (unsigned long)ESP_OK);
                return ESP_OK;
            }
        }
        lastErr = err;
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    if (outStatus) {
        *outStatus = status.Raw;
    }
    printf("FDC_FRAME_READY,row=%u,device=%s,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(sensorarrayMeasureFdcDeviceIdFromState(fdcState)),
           status.Raw,
           (unsigned)sensorarrayMeasureFdcUnreadMaskFromStatus(&status),
           status.DataReady ? 1u : 0u,
           (unsigned long)((lastErr == ESP_OK) ? ESP_ERR_TIMEOUT : lastErr));
    return (lastErr == ESP_OK) ? ESP_ERR_TIMEOUT : lastErr;
}

static esp_err_t sensorarrayMeasureReadFdcAutoscan4ch(sensorarrayFdcDeviceState_t *fdcState,
                                                      sensorarrayFdcAutoscanSamples_t *outSamples)
{
    if (!fdcState || !fdcState->handle || !outSamples) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSamples = (sensorarrayFdcAutoscanSamples_t){0};
    Fdc2214CapFastChannelSample_t fastSamples[4] = {0};
    esp_err_t firstErr = Fdc2214CapReadAutoscan4RawFast(fdcState->handle, fastSamples);
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        const Fdc2214CapFastChannelSample_t *sample = &fastSamples[ch];
        outSamples->raw28[ch] = sample->raw28;
        outSamples->statusRaw = sample->statusRaw;
        if (sample->unreadConversion) {
            outSamples->unreadMask |= (uint8_t)(1u << ch);
        }
        outSamples->amplitudeWarning[ch] =
            sample->errAmplitude ||
            ((sample->errorMask & FDC2214CAP_FAST_ERROR_AMPLITUDE) != 0u);
        outSamples->watchdogFault[ch] =
            sample->errWatchdog ||
            ((sample->errorMask & FDC2214CAP_FAST_ERROR_WATCHDOG) != 0u);
        outSamples->saturated[ch] = sample->raw28 >= SENSORARRAY_FDC_RAW28_SATURATED_THRESHOLD;

        bool i2cOk = (sample->errorMask & FDC2214CAP_FAST_ERROR_I2C) == 0u;
        bool readable = sample->unreadConversion || sample->dataReady;
        outSamples->valid[ch] = i2cOk &&
                                readable &&
                                sample->raw28 != 0u &&
                                !outSamples->watchdogFault[ch] &&
                                !outSamples->saturated[ch];
    }
    return firstErr;
}

static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  bool valid,
                                                  bool error);

static void sensorarrayMeasureFillFdcMatrixRow(sensorarrayFdcMatrixFrame_t *frame,
                                               uint8_t sIndex,
                                               const sensorarrayFdcAutoscanSamples_t *primary,
                                               const sensorarrayFdcAutoscanSamples_t *secondary,
                                               uint8_t *outValidMask8,
                                               uint8_t *outErrorMask8)
{
    uint8_t validMask8 = 0u;
    uint8_t errorMask8 = 0u;
    const sensorarrayFdcAutoscanSamples_t *samplesByHalf[2] = {primary, secondary};

    for (uint8_t half = 0u; half < 2u; ++half) {
        const sensorarrayFdcAutoscanSamples_t *samples = samplesByHalf[half];
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + (half * 4u));
            bool valid = samples && samples->valid[ch];
            bool error = !valid ||
                         (samples && (samples->amplitudeWarning[ch] ||
                                      samples->watchdogFault[ch] ||
                                      samples->saturated[ch]));
            uint32_t raw28 = samples ? samples->raw28[ch] : 0u;
            sensorarrayMeasureMarkFdcMatrixCellEx(frame, sIndex, dIndex, raw28, valid, error);
            if (valid) {
                validMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (error) {
                errorMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
        }
    }

    if (outValidMask8) {
        *outValidMask8 = validMask8;
    }
    if (outErrorMask8) {
        *outErrorMask8 = errorMask8;
    }
}

static void sensorarrayMeasureAccumulateFdcHealth(sensorarrayFdcFrameHealth_t *health,
                                                  sensorarrayFdcDeviceId_t devId,
                                                  const sensorarrayFdcAutoscanSamples_t *samples)
{
    if (!health || !samples) {
        return;
    }
    uint8_t devIndex = (uint8_t)devId;
    if (devIndex > (uint8_t)SENSORARRAY_FDC_DEV_SECONDARY) {
        return;
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if (samples->valid[ch]) {
            health->validSeen[devIndex][ch] = true;
            health->lastRaw28[devIndex][ch] = samples->raw28[ch];
        } else {
            health->invalidSeen[devIndex][ch] = true;
        }
        health->amplitudeWarningSeen[devIndex][ch] =
            health->amplitudeWarningSeen[devIndex][ch] || samples->amplitudeWarning[ch];
        health->watchdogSeen[devIndex][ch] =
            health->watchdogSeen[devIndex][ch] || samples->watchdogFault[ch];
        health->saturatedSeen[devIndex][ch] =
            health->saturatedSeen[devIndex][ch] || samples->saturated[ch];
        health->zeroRawSeen[devIndex][ch] =
            health->zeroRawSeen[devIndex][ch] || (samples->raw28[ch] == 0u);
    }
}

static void sensorarrayMeasureUpdateFdcRuntimeProfiles(sensorarrayState_t *state,
                                                       const sensorarrayFdcFrameHealth_t *health)
{
    if (!state || !health) {
        return;
    }

    uint32_t threshold = (uint32_t)CONFIG_SENSORARRAY_FDC_FAST_FAIL_THRESHOLD;
    if (threshold == 0u) {
        threshold = 1u;
    }

    for (uint8_t dev = 0u; dev < 2u; ++dev) {
        sensorarrayFdcDeviceState_t *fdcState =
            sensorarrayMeasureGetFdcState(state, (sensorarrayFdcDeviceId_t)dev);
        if (!fdcState) {
            continue;
        }
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[ch];
            if (health->validSeen[dev][ch]) {
                profile->valid = true;
                profile->lastRaw28 = health->lastRaw28[dev][ch];
                profile->lastValidTimestampUs = (uint64_t)esp_timer_get_time();
                profile->consecutiveInvalid = 0u;
                profile->consecutiveWatchdogFault = 0u;
                profile->consecutiveSaturated = 0u;
                profile->consecutiveZeroRaw = 0u;
                if (!health->amplitudeWarningSeen[dev][ch]) {
                    profile->consecutiveAmplitudeFault = 0u;
                }
            } else if (health->invalidSeen[dev][ch]) {
                if (profile->consecutiveInvalid < UINT32_MAX) {
                    profile->consecutiveInvalid++;
                }
                if (health->watchdogSeen[dev][ch] && profile->consecutiveWatchdogFault < UINT32_MAX) {
                    profile->consecutiveWatchdogFault++;
                }
                if (health->saturatedSeen[dev][ch] && profile->consecutiveSaturated < UINT32_MAX) {
                    profile->consecutiveSaturated++;
                }
                if (health->zeroRawSeen[dev][ch] && profile->consecutiveZeroRaw < UINT32_MAX) {
                    profile->consecutiveZeroRaw++;
                }
            }

            if (health->amplitudeWarningSeen[dev][ch] &&
                profile->consecutiveAmplitudeFault < UINT32_MAX) {
                profile->consecutiveAmplitudeFault++;
            }

            if ((profile->consecutiveInvalid >= threshold && health->invalidSeen[dev][ch]) ||
                profile->consecutiveAmplitudeFault >= threshold) {
                profile->quickSweepPending = true;
                profile->quickSweepReason =
                    (profile->consecutiveAmplitudeFault >= threshold) ? "amplitude_warning" :
                    health->watchdogSeen[dev][ch] ? "watchdog_fault" :
                    health->saturatedSeen[dev][ch] ? "saturated" :
                    health->zeroRawSeen[dev][ch] ? "zero_raw_no_oscillation" :
                    "invalid_streak";
                printf("FDC_RESCUE,stage=pending,scope=channel,device=%s,ch=%u,reason=%s,consecutiveInvalid=%lu,consecutiveAmplitude=%lu\n",
                       sensorarrayMeasureFdcDeviceName((sensorarrayFdcDeviceId_t)dev),
                       (unsigned)ch,
                       profile->quickSweepReason ? profile->quickSweepReason : SENSORARRAY_NA,
                       (unsigned long)profile->consecutiveInvalid,
                       (unsigned long)profile->consecutiveAmplitudeFault);
            }
        }
    }
}

static void sensorarrayMeasureInitFdcMatrixFrame(sensorarrayFdcMatrixFrame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->timestampUs = (uint64_t)esp_timer_get_time();
    frame->sequence = s_fdcMatrixSequence++;
}

static bool sensorarrayFdcMatrixFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return true;
    }
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        if (frame->raw28[i] != 0u) {
            return false;
        }
    }
    return true;
}

static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  bool valid,
                                                  bool error)
{
    if (!frame || !sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        return;
    }

    size_t index = sensorarrayMatrixIndex(sIndex, dIndex);
    uint64_t bit = 1ULL << index;
    frame->raw28[index] = raw28;
    if (valid) {
        frame->validMask |= bit;
    } else {
        frame->validMask &= ~bit;
    }
    if (error) {
        frame->errorMask |= bit;
    } else {
        frame->errorMask &= ~bit;
    }
}

static esp_err_t sensorarrayMeasureCheckFdcMatrixReady(sensorarrayState_t *state)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!state->boardReady || !state->tmuxReady || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!state->fdcPrimary.ready || !state->fdcPrimary.handle ||
        !state->fdcSecondary.ready || !state->fdcSecondary.handle) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!state->fdcPrimary.i2cCtx || !state->fdcSecondary.i2cCtx) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t sensorarrayMeasureReadFdcMatrixFrame(sensorarrayState_t *state,
                                               sensorarrayFdcMatrixFrame_t *outFrame)
{
    if (!outFrame) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayMeasureInitFdcMatrixFrame(outFrame);

    if (!state) {
        outFrame->errorMask = UINT64_MAX;
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = sensorarrayMeasureTakeLock();
    if (err != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        return err;
    }

    esp_err_t firstErr = sensorarrayMeasureCheckFdcMatrixReady(state);
    if (firstErr != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        printf("MATRIXFDC_DIAG,stage=read_abort,reason=matrix_not_ready,err=0x%lx\n",
               (unsigned long)firstErr);
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    firstErr = sensorarrayMeasureEnsureFdcMatrixPath(state, "fdc_matrix_frame");
    if (firstErr != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        printf("MATRIXFDC_DIAG,stage=read_abort,reason=path_prepare_failed,err=0x%lx\n",
               (unsigned long)firstErr);
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    sensorarrayFdcFrameHealth_t frameHealth = {0};
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        err = tmuxSwitchSelectRow((uint8_t)(s - 1u));
        sensorarrayMeasureDelayUs((uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US);
        tmuxSwitchControlState_t ctrl = {0};
        sensorarrayMeasureReadFdcPathControl(&ctrl);
        printf("FDC_ROW,stage=select,row=%u,selaCmd=%d,selaReadback=%d,err=0x%lx\n",
               (unsigned)s,
               ctrl.cmdSelaLevel,
               ctrl.obsSelaLevel,
               (unsigned long)err);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }

        err = sensorarrayMeasureEnsureFdcMatrixPath(state, "fdc_matrix_frame");
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }

        esp_err_t primaryErr = sensorarrayMeasureEnsureFdcAutoscan4ch(state, SENSORARRAY_FDC_DEV_PRIMARY);
        if (primaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = primaryErr;
        }
        esp_err_t secondaryErr = sensorarrayMeasureEnsureFdcAutoscan4ch(state, SENSORARRAY_FDC_DEV_SECONDARY);
        if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = secondaryErr;
        }

        uint16_t primaryStatus = 0u;
        uint16_t secondaryStatus = 0u;
        if (primaryErr == ESP_OK) {
            primaryErr = sensorarrayMeasureWaitFdcAutoscanFrameReady(&state->fdcPrimary,
                                                                     s,
                                                                     (uint32_t)SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS,
                                                                     &primaryStatus);
            if (primaryErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = primaryErr;
            }
        } else {
            printf("FDC_FRAME_READY,row=%u,device=primary,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
                   (unsigned)s,
                   primaryStatus,
                   0u,
                   0u,
                   (unsigned long)primaryErr);
        }
        if (secondaryErr == ESP_OK) {
            secondaryErr = sensorarrayMeasureWaitFdcAutoscanFrameReady(&state->fdcSecondary,
                                                                       s,
                                                                       (uint32_t)SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS,
                                                                       &secondaryStatus);
            if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = secondaryErr;
            }
        } else {
            printf("FDC_FRAME_READY,row=%u,device=secondary,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
                   (unsigned)s,
                   secondaryStatus,
                   0u,
                   0u,
                   (unsigned long)secondaryErr);
        }

        sensorarrayFdcAutoscanSamples_t primarySamples = {0};
        sensorarrayFdcAutoscanSamples_t secondarySamples = {0};
        if (primaryErr == ESP_OK) {
            primaryErr = sensorarrayMeasureReadFdcAutoscan4ch(&state->fdcPrimary, &primarySamples);
            if (primaryErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = primaryErr;
            }
        } else {
            primarySamples.statusRaw = primaryStatus;
        }
        if (secondaryErr == ESP_OK) {
            secondaryErr = sensorarrayMeasureReadFdcAutoscan4ch(&state->fdcSecondary, &secondarySamples);
            if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = secondaryErr;
            }
        } else {
            secondarySamples.statusRaw = secondaryStatus;
        }

        uint8_t rowValidMask8 = 0u;
        uint8_t rowErrorMask8 = 0u;
        sensorarrayMeasureFillFdcMatrixRow(outFrame,
                                           s,
                                           &primarySamples,
                                           &secondarySamples,
                                           &rowValidMask8,
                                           &rowErrorMask8);
        sensorarrayMeasureAccumulateFdcHealth(&frameHealth,
                                              SENSORARRAY_FDC_DEV_PRIMARY,
                                              &primarySamples);
        sensorarrayMeasureAccumulateFdcHealth(&frameHealth,
                                              SENSORARRAY_FDC_DEV_SECONDARY,
                                              &secondarySamples);

        printf("FDC_MATRIX_ROW,row=%u,d1=%lu,d2=%lu,d3=%lu,d4=%lu,d5=%lu,d6=%lu,d7=%lu,d8=%lu,validMask8=0x%02X,errorMask8=0x%02X\n",
               (unsigned)s,
               (unsigned long)primarySamples.raw28[0],
               (unsigned long)primarySamples.raw28[1],
               (unsigned long)primarySamples.raw28[2],
               (unsigned long)primarySamples.raw28[3],
               (unsigned long)secondarySamples.raw28[0],
               (unsigned long)secondarySamples.raw28[1],
               (unsigned long)secondarySamples.raw28[2],
               (unsigned long)secondarySamples.raw28[3],
               (unsigned)rowValidMask8,
               (unsigned)rowErrorMask8);
        taskYIELD();
    }
    sensorarrayMeasureUpdateFdcRuntimeProfiles(state, &frameHealth);

    sensorarrayMeasureGiveLock();
    if (outFrame->validMask == 0u) {
        bool allZero = sensorarrayFdcMatrixFrameRawAllZero(outFrame);
        uint32_t seq = s_fdcMatrixAllInvalidSequence++;
        if (allZero) {
            outFrame->errorMask = UINT64_MAX;
        }
        printf("MATRIXFDC_DIAG,stage=%s,seq=%lu,errorMask=0x%016llX,reason=%s\n",
               allZero ? "all_invalid" : "all_status_invalid",
               (unsigned long)seq,
               (unsigned long long)outFrame->errorMask,
               allZero ? "all_zero_raw" : "raw_nonzero_status_invalid");
        sensorarrayFdcSweepReportAllInvalidFrame(outFrame->validMask,
                                                 outFrame->errorMask,
                                                 allZero ? SENSORARRAY_MATRIX_CELL_COUNT : 0u);
        return (firstErr != ESP_OK) ? firstErr : ESP_ERR_INVALID_RESPONSE;
    }
    return firstErr;
}

bool sensorarrayFastSpeedIsEnabled(void)
{
    return s_fastSpeedEnabled;
}

void sensorarrayFastSpeedSetEnabled(bool enabled)
{
    s_fastSpeedEnabled = enabled;
}

static esp_err_t sensorarrayTransportSendFdcMatrixFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    (void)frame;
    return ESP_ERR_NOT_SUPPORTED;
}

static void sensorarrayFdcMatrixPrintFrame(const sensorarrayFdcMatrixFrame_t *frame, const char *tag)
{
    printf("%s,seq=%lu,timestampUs=%llu,validMask=0x%016llX,errorMask=0x%016llX,raw28=[",
           tag ? tag : "MATRIXFDC",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->errorMask);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%lu", (i == 0u) ? "" : ",", (unsigned long)frame->raw28[i]);
    }
    printf("]\n");
}

esp_err_t sensorarrayFdcMatrixEmitFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return ESP_ERR_INVALID_ARG;
    }

    if (sensorarrayFastSpeedIsEnabled()) {
        return sensorarrayTransportSendFdcMatrixFrame(frame);
    }

    sensorarrayFdcMatrixPrintFrame(frame, "MATRIXFDC");
    return ESP_OK;
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
                                             sensorarrayRoutePathKind_t path,
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

    if (swSource == TMUX1108_SOURCE_REF) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ref_guard",
                                              sColumn,
                                              dLine,
                                              path,
                                              TMUX1108_SOURCE_GND,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw_pre_ref_guard",
                                              "isolate_ref_before_route");
        if (err != ESP_OK) {
            return err;
        }
        err = sensorarrayMeasureApplyRefPolicy(state,
                                               "enter_ref_mode",
                                               "route_apply_levels",
                                               SENSORARRAY_MATRIX_D_SOURCE_REF,
                                               SENSORARRAY_ADS_INTREF_ON,
                                               SENSORARRAY_ADS_VBIAS_ON,
                                               "ref_d_before_connect");
        if (err != ESP_OK) {
            return err;
        }
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ground",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw_pre_ground",
                                              "ground_d_before_route");
        if (err != ESP_OK) {
            return err;
        }
        if (path == SENSORARRAY_ROUTE_PATH_CAPACITIVE) {
            sensorarrayAdsIntRefPolicy_t intrefPolicy =
                state->adsReady ? SENSORARRAY_ADS_INTREF_OFF : SENSORARRAY_ADS_INTREF_KEEP;
            sensorarrayAdsVbiasPolicy_t vbiasPolicy =
                state->adsReady ? SENSORARRAY_ADS_VBIAS_OFF : SENSORARRAY_ADS_VBIAS_KEEP;
            err = sensorarrayMeasureApplyRefPolicy(state,
                                                   "cap_fdc_ref_off",
                                                   "route_apply_levels",
                                                   SENSORARRAY_MATRIX_D_SOURCE_GND,
                                                   intrefPolicy,
                                                   vbiasPolicy,
                                                   "fdc_cap_path_no_ads_ref");
            if (err != ESP_OK) {
                return err;
            }
        }
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

    if (swSource == TMUX1108_SOURCE_GND) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_final_assert",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "ok",
                                              "final_ground_assert");
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw",
                                              sColumn,
                                              dLine,
                                              path,
                                              swSource,
                                              selaRoute,
                                              selBLevel,
                                              label,
                                              "set_sw",
                                              "connect_ref_after_route");
    }
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(delayAfterSwMs);

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

    const sensorarrayRouteMap_t *routeMap = sensorarrayBoardMapFindRoute(sColumn, dLine, path);
    if (!routeMap) {
        return ESP_ERR_NOT_SUPPORTED;
    }

    sensorarrayRoutePathKind_t routePath = sensorarrayBoardMapPathToRoutePath(path, swSource);
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(routeMap->selaRoute, &(int){0})) {
        return ESP_ERR_INVALID_STATE;
    }

    printf("ROUTEDEBUG,stage=apply_begin,label=%s,sColumn=%u,dLine=%u,path=%s,sw=%s,selaRequest=%s,"
           "selBLevel=%u,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n",
           routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogRoutePathName(routePath),
           sensorarrayLogSwSourceName(swSource),
           sensorarrayBoardMapSelaRouteName(routeMap->selaRoute),
           routeMap->selBLevel ? 1u : 0u);

    const bool adsStopNeeded = state->adsReady && state->adsAdc1Running;
    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    sensorarrayLogRouteStep("ads_stop",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                                           routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            adsStopNeeded ? ((err == ESP_OK) ? "stop_ads_before_route" : "stop_ads_error")
                                          : "ads_already_stopped");
    if (err != ESP_OK) {
        return err;
    }

    if (swSource == TMUX1108_SOURCE_REF) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ref_guard",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              TMUX1108_SOURCE_GND,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw_pre_ref_guard",
                                              "isolate_ref_before_route");
        if (err != ESP_OK) {
            return err;
        }
        err = sensorarrayMeasureApplyRefPolicy(state,
                                               "enter_ref_mode",
                                               "route_apply",
                                               SENSORARRAY_MATRIX_D_SOURCE_REF,
                                               SENSORARRAY_ADS_INTREF_ON,
                                               SENSORARRAY_ADS_VBIAS_ON,
                                               "ref_d_before_connect");
        if (err != ESP_OK) {
            return err;
        }
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_pre_ground",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw_pre_ground",
                                              "ground_d_before_route");
        if (err != ESP_OK) {
            return err;
        }
        if (path == SENSORARRAY_PATH_CAPACITIVE) {
            sensorarrayAdsIntRefPolicy_t intrefPolicy =
                state->adsReady ? SENSORARRAY_ADS_INTREF_OFF : SENSORARRAY_ADS_INTREF_KEEP;
            sensorarrayAdsVbiasPolicy_t vbiasPolicy =
                state->adsReady ? SENSORARRAY_ADS_VBIAS_OFF : SENSORARRAY_ADS_VBIAS_KEEP;
            err = sensorarrayMeasureApplyRefPolicy(state,
                                                   "cap_fdc_ref_off",
                                                   "route_apply",
                                                   SENSORARRAY_MATRIX_D_SOURCE_GND,
                                                   intrefPolicy,
                                                   vbiasPolicy,
                                                   "fdc_cap_path_no_ads_ref");
            if (err != ESP_OK) {
                return err;
            }
        }
    }

    err = tmuxSwitchSelectRow((uint8_t)(sColumn - 1u));
    sensorarrayLogRouteStep("row",
                            routeMap->mapLabel,
                            sColumn,
                            dLine,
                            routePath,
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
                            routePath,
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
                            routePath,
                            swSource,
                            routeMap->selaRoute,
                            routeMap->selBLevel,
                            err,
                            "set_selB");
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_PATH_MS);

    if (swSource == TMUX1108_SOURCE_GND) {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw_final_assert",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "ok",
                                              "final_ground_assert");
    } else {
        err = sensorarrayMeasureSetSwForRoute(state,
                                              "sw",
                                              sColumn,
                                              dLine,
                                              routePath,
                                              swSource,
                                              routeMap->selaRoute,
                                              routeMap->selBLevel,
                                              routeMap->mapLabel,
                                              "set_sw",
                                              "connect_ref_after_route");
    }
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayDelayMs(SENSORARRAY_SETTLE_AFTER_SW_MS);

    sensorarrayLogDbgExtraCaptureCtrl();

    if (outMapLabel) {
        *outMapLabel = routeMap->mapLabel;
    }
    printf("ROUTEDEBUG,stage=apply_done,label=%s,sColumn=%u,dLine=%u,path=%s,selaRequest=%s,"
           "selBLevel=%u,err=0,status=ok,note=route_verify_only_confirms_gpio_control_state_not_analog_conduction\n",
           routeMap->mapLabel ? routeMap->mapLabel : SENSORARRAY_NA,
           (unsigned)sColumn,
           (unsigned)dLine,
           sensorarrayLogRoutePathName(routePath),
           sensorarrayBoardMapSelaRouteName(routeMap->selaRoute),
           routeMap->selBLevel ? 1u : 0u);
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

    FDCLOW_TRACE("FDCLOW,stage=diag_enter,channel=%u,discardFirst=%u,relaxed=%u\n",
                    (unsigned)ch,
                    discardFirst ? 1u : 0u,
                    relaxedMode ? 1u : 0u);

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
        FDCLOW_TRACE("FDCLOW,stage=discard_sample_begin,channel=%u\n", (unsigned)ch);
        esp_err_t discardErr = readFn(dev, ch, &throwaway);
        FDCLOW_TRACE("FDCLOW,stage=discard_sample_done,channel=%u,err=%ld\n",
                        (unsigned)ch,
                        (long)discardErr);
        if (discardErr != ESP_OK) {
            outDiag->err = discardErr;
            FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                            (long)discardErr,
                            (unsigned)outDiag->statusCode);
            return discardErr;
        }
    }

    FDCLOW_TRACE("FDCLOW,stage=read_sample_begin,channel=%u\n", (unsigned)ch);
    esp_err_t err = readFn(dev, ch, &outDiag->sample);
    FDCLOW_TRACE("FDCLOW,stage=read_sample_done,channel=%u,err=%ld,raw=%lu,status=0x%04X,"
                    "config=0x%04X,mux=0x%04X,statusCode=%u\n",
                    (unsigned)ch,
                    (long)err,
                    (unsigned long)outDiag->sample.Raw28,
                    outDiag->sample.StatusRaw,
                    outDiag->sample.ConfigRaw,
                    outDiag->sample.MuxRaw,
                    (unsigned)outDiag->sample.SampleStatus);
    outDiag->err = err;
    outDiag->i2cOk = (err == ESP_OK);
    if (err != ESP_OK) {
        outDiag->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                        (long)err,
                        (unsigned)outDiag->statusCode);
        return err;
    }

    FDCLOW_TRACE("FDCLOW,stage=read_status_config_begin\n");
    esp_err_t statusCfgErr = Fdc2214CapReadCoreRegs(dev, &outDiag->coreRegs);
    FDCLOW_TRACE("FDCLOW,stage=read_status_config_done,err=%ld,statusConfig=0x%04X\n",
                    (long)statusCfgErr,
                    outDiag->coreRegs.StatusConfig);
    if (statusCfgErr != ESP_OK) {
        outDiag->err = statusCfgErr;
        outDiag->i2cOk = false;
        outDiag->statusCode = SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR;
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
        FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u\n",
                        (long)statusCfgErr,
                        (unsigned)outDiag->statusCode);
        return statusCfgErr;
    }

    (void)Fdc2214CapDecodeStatusRaw(outDiag->sample.StatusRaw, &outDiag->status);

    outDiag->converting = outDiag->sample.Converting;
    outDiag->unreadConversionPresent = Fdc2214CapStatusHasUnreadForChannel(&outDiag->status, ch);

    sensorarrayFdcSampleStatus_t mappedStatus = sensorarrayMeasureMapFdcStatus(outDiag->sample.SampleStatus);
    if (!idOk || !configOk) {
        mappedStatus = SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN;
    }
    bool statusFault = Fdc2214CapStatusHasWatchdogFault(&outDiag->status) ||
                       Fdc2214CapStatusHasAmplitudeFault(&outDiag->status);
    bool readable = outDiag->unreadConversionPresent || outDiag->status.DataReady || outDiag->sample.DataReady;
    outDiag->statusCode = mappedStatus;
    outDiag->qualityDegraded = (!readable) ||
                               outDiag->sample.ErrWatchdog ||
                               outDiag->sample.ErrAmplitude ||
                               statusFault;
    outDiag->provisionalReadable = idOk &&
                                   configOk &&
                                   outDiag->sample.Converting &&
                                   (outDiag->sample.Raw28 != 0u) &&
                                   !outDiag->sample.ErrWatchdog &&
                                   !outDiag->sample.ErrAmplitude &&
                                   !statusFault &&
                                   readable;
    outDiag->sampleValid = relaxedMode ? outDiag->provisionalReadable
                                       : (mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID);
    if (!idOk || !configOk) {
        outDiag->sampleValid = false;
        outDiag->provisionalReadable = false;
    }
    FDCLOW_TRACE("FDCLOW,stage=diag_done,err=%ld,statusCode=%u,sampleValid=%u,provisional=%u\n",
                    (long)outDiag->err,
                    (unsigned)outDiag->statusCode,
                    outDiag->sampleValid ? 1u : 0u,
                    outDiag->provisionalReadable ? 1u : 0u);
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

const char *sensorarrayMeasureFdcRefClockSourceName(sensorarrayFdcRefClockSource_t source)
{
    switch (source) {
    case SENSORARRAY_FDC_REF_CLOCK_SOURCE_INTERNAL:
        return "internal_oscillator";
    case SENSORARRAY_FDC_REF_CLOCK_SOURCE_EXTERNAL:
        return "external_clkin";
    default:
        return "unknown";
    }
}

sensorarrayFdcRefClockSource_t sensorarrayMeasureFdcEffectiveRefClockSource(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return SENSORARRAY_FDC_REF_CLOCK_SOURCE_EXTERNAL;
#else
    return SENSORARRAY_FDC_REF_CLOCK_SOURCE_INTERNAL;
#endif
}

uint32_t sensorarrayMeasureFdcEffectiveFclkHz(void)
{
#if SENSORARRAY_FDC_REF_CLOCK_USE_EXTERNAL
    return SENSORARRAY_FDC_EXTERNAL_CLOCK_HZ;
#else
#if SENSORARRAY_FDC_INTERNAL_CLOCK_CALIBRATED_HZ > 0u
    return SENSORARRAY_FDC_INTERNAL_CLOCK_CALIBRATED_HZ;
#else
    return SENSORARRAY_FDC_INTERNAL_CLOCK_NOMINAL_HZ;
#endif
#endif
}

bool sensorarrayMeasureFdcDecodeClockDividers(uint16_t clockDividers,
                                              uint8_t *outFinSelCode,
                                              uint8_t *outFinFactor,
                                              uint16_t *outFrefDivider,
                                              const char **outStatus)
{
    if (!outFinSelCode || !outFinFactor || !outFrefDivider || !outStatus) {
        if (outStatus) {
            *outStatus = "invalid_arg";
        }
        return false;
    }

    uint8_t finSelCode = (uint8_t)((clockDividers >> 12) & 0x03u);
    uint16_t frefDivider = (uint16_t)(clockDividers & 0x03FFu);

    *outFinSelCode = finSelCode;
    *outFinFactor = 0u;
    *outFrefDivider = frefDivider;
    *outStatus = "ok";

    if (frefDivider == 0u) {
        *outStatus = "zero_fref_divider";
        return false;
    }
    if (finSelCode == 0u) {
        *outStatus = "invalid_fin_sel_zero";
        return false;
    }
    if (finSelCode == 1u) {
        *outFinFactor = 1u;
        return true;
    }
    if (finSelCode == 2u) {
        *outFinFactor = 2u;
        return true;
    }

    *outStatus = "unsupported_fin_sel_3";
    return false;
}

bool sensorarrayMeasureFdcComputeFrequencyDiag(uint32_t raw28,
                                               uint16_t clockDividers,
                                               sensorarrayFdcFrequencyDiag_t *outDiag)
{
    if (!outDiag) {
        return false;
    }

    *outDiag = (sensorarrayFdcFrequencyDiag_t){
        .clockDividers = clockDividers,
        .refClockSource = sensorarrayMeasureFdcEffectiveRefClockSource(),
        .effectiveFclkHz = sensorarrayMeasureFdcEffectiveFclkHz(),
        .valid = false,
        .status = "unknown",
    };

    if (raw28 == 0u) {
        outDiag->status = "zero_raw";
        return false;
    }
    if (outDiag->effectiveFclkHz == 0u) {
        outDiag->status = "zero_effective_fclk";
        return false;
    }

    const char *decodeStatus = "unknown";
    bool decodeOk = sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                             &outDiag->finSelCode,
                                                             &outDiag->finFactor,
                                                             &outDiag->frefDivider,
                                                             &decodeStatus);
    outDiag->status = decodeStatus;
    if (!decodeOk) {
        return false;
    }

    outDiag->effectiveFrefHz = (double)outDiag->effectiveFclkHz / (double)outDiag->frefDivider;
    outDiag->freqHzBase = sensorarrayMeasureFdcRawToFrequencyHz(raw28, outDiag->effectiveFclkHz);
    outDiag->freqHzCorrected =
        ((double)raw28 * outDiag->effectiveFrefHz * (double)outDiag->finFactor) / SENSORARRAY_FDC_RAW_SCALE_2P28;
    outDiag->valid = true;
    outDiag->status = "ok";
    return true;
}

double sensorarrayMeasureFdcRawToSensorFrequencyHz(uint32_t raw28, uint16_t clockDividers)
{
    sensorarrayFdcFrequencyDiag_t diag = {0};
    if (!sensorarrayMeasureFdcComputeFrequencyDiag(raw28, clockDividers, &diag) || !diag.valid) {
        return 0.0;
    }
    return diag.freqHzCorrected;
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
