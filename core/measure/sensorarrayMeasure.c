#include "sensorarrayMeasure.h"

#include <math.h>
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
#ifndef CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS
#define CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS 20
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES
#ifdef CONFIG_SENSORARRAY_FDC_TIMING_LOG_EVERY_N_FRAMES
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES CONFIG_SENSORARRAY_FDC_TIMING_LOG_EVERY_N_FRAMES
#else
#define CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES 10
#endif
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PROFILE_ROW_DEFAULT
#define CONFIG_SENSORARRAY_FDC_PROFILE_ROW_DEFAULT 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_PROFILE_DEVICE_DEFAULT
#define CONFIG_SENSORARRAY_FDC_PROFILE_DEVICE_DEFAULT 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_EMIT_CAP_TOTAL_PF
#define CONFIG_SENSORARRAY_FDC_EMIT_CAP_TOTAL_PF 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH
#define CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH 18000
#endif
#ifndef CONFIG_SENSORARRAY_FDC_VERIFY_MODE_FULL
#define CONFIG_SENSORARRAY_FDC_VERIFY_MODE_FULL 0
#endif
#ifndef CONFIG_SENSORARRAY_FDC_VERIFY_MODE_STARTUP_ONLY
#define CONFIG_SENSORARRAY_FDC_VERIFY_MODE_STARTUP_ONLY 1
#endif
#ifndef CONFIG_SENSORARRAY_FDC_VERIFY_MODE_NONE
#define CONFIG_SENSORARRAY_FDC_VERIFY_MODE_NONE 0
#endif
#ifndef CONFIG_BOARD_I2C_FREQ_HZ
#define CONFIG_BOARD_I2C_FREQ_HZ 325000
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
#define SENSORARRAY_FDC_AMPLITUDE_RESCUE_THRESHOLD 4u
#define SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT 2u
#define SENSORARRAY_FDC_TARGET_FRAME_US (1000000u / CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS)
#define SENSORARRAY_FDC_TARGET_ROW_US (SENSORARRAY_FDC_TARGET_FRAME_US / SENSORARRAY_MATRIX_ROWS)

static SemaphoreHandle_t s_measureLock = NULL;
static portMUX_TYPE s_measureLockMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_fdcMatrixSequence = 0u;
static bool s_fastSpeedEnabled = false;
static uint32_t s_fdcMatrixAllInvalidSequence = 0u;
static bool s_fdcProfileSummaryEnabled = true;
static bool s_fdcProfileRowEnabled = (CONFIG_SENSORARRAY_FDC_PROFILE_ROW_DEFAULT != 0);
static bool s_fdcProfileDeviceEnabled = (CONFIG_SENSORARRAY_FDC_PROFILE_DEVICE_DEFAULT != 0);
static uint32_t s_fdcTimingSummaryEvery = CONFIG_SENSORARRAY_FDC_TIMING_SUMMARY_EVERY_N_FRAMES;
static uint8_t s_fdcDiscardFrames = (uint8_t)CONFIG_SENSORARRAY_FDC_DISCARD_FRAMES_AFTER_ROW_SWITCH;

static uint64_t sensorarrayMeasureElapsedUs(int64_t startUs);

typedef struct {
    uint32_t raw28[4];
    bool valid[4];
    bool amplitudeWarning[4];
    bool watchdogFault[4];
    bool saturated[4];
    bool i2cError[4];
    uint16_t statusRaw;
    uint8_t unreadMask;
} sensorarrayFdcAutoscanSamples_t;

typedef struct {
    bool validSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool invalidSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool amplitudeWarningSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool watchdogSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool saturatedSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool zeroRawSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    bool i2cErrorSeen[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint32_t lastRaw28[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    double lastFreqHz[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint16_t clockDividers[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint16_t driveCurrent[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint8_t deglitchCode[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
    uint32_t effectiveFclkHz[SENSORARRAY_MATRIX_ROWS][SENSORARRAY_MATRIX_COLS];
} sensorarrayFdcFrameHealth_t;

typedef struct {
    bool valid;
    uint16_t rCount;
    uint16_t settleCount;
    uint16_t clockDividers;
    uint16_t driveCurrent;
    uint8_t deglitchCode;
    uint32_t effectiveFclkHz;
} sensorarrayFdcRuntimeChannelConfig_t;

typedef struct {
    bool ready;
    bool dataReady;
    uint16_t statusRaw;
    uint8_t unreadMask;
    esp_err_t err;
    uint32_t pollCount;
    uint32_t timeoutCount;
} sensorarrayFdcReadyState_t;

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

    int expectedSw = CONFIG_TMUX1108_SW_REF_LEVEL ? 0 : 1;
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

    err = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
    sensorarrayMeasureReadFdcPathControl(&ctrl);
    printf("FDC_PATH,stage=sw_gnd,reason=%s,swCmd=GND,swReadback=%d,err=0x%lx\n",
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

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_PATH,stage=ensure_ok,reason=%s,sw=%d,sela=%d,selb=%d,en=%d,adsRef=%d\n",
           reason ? reason : SENSORARRAY_NA,
           sensorarrayMeasureSwPhysicalReadbackFromControl(&ctrl),
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsTmux1134EnLogicalOnValid ? (ctrl.obsTmux1134EnLogicalOn ? 1 : 0) : (ctrl.cmdTmux1134EnLogicalOn ? 1 : 0),
           state->adsRefReady ? 1 : 0);
#endif
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

bool sensorarrayMeasureDecodeMatrixIndex(uint8_t matrixIndex,
                                          uint8_t *outSColumn,
                                          uint8_t *outDLine)
{
    if (matrixIndex >= SENSORARRAY_MATRIX_CELL_COUNT || !outSColumn || !outDLine) {
        return false;
    }

    *outSColumn = (uint8_t)((matrixIndex / SENSORARRAY_MATRIX_COLS) + 1u);
    *outDLine = (uint8_t)((matrixIndex % SENSORARRAY_MATRIX_COLS) + 1u);
    return true;
}

bool sensorarrayMeasureMakeFdcCellTarget(sensorarrayState_t *state,
                                         uint8_t sColumn,
                                         uint8_t dLine,
                                         sensorarrayFdcCellTarget_t *outTarget)
{
    (void)state;
    if (!outTarget || !sensorarrayMatrixIndexIsValid(sColumn, dLine)) {
        return false;
    }

    const sensorarrayFdcDLineMap_t *map = sensorarrayBoardMapFindFdcByDLine(dLine);
    if (!map || map->channel > FDC2214_CH3) {
        return false;
    }

    *outTarget = (sensorarrayFdcCellTarget_t){
        .sColumn = sColumn,
        .dLine = dLine,
        .matrixIndex = (uint8_t)sensorarrayMatrixIndex(sColumn, dLine),
        .devId = map->devId,
        .fdcChannel = (uint8_t)map->channel,
        .mapLabel = map->mapLabel,
    };
    return true;
}

sensorarrayFdcCellConfigCache_t *sensorarrayMeasureGetFdcCellCache(sensorarrayState_t *state,
                                                                   const sensorarrayFdcCellTarget_t *target)
{
    if (!state || !target || !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine)) {
        return NULL;
    }
    return &state->fdcCellCache[target->sColumn - 1u][target->dLine - 1u];
}

static void sensorarrayMeasureMarkFdcAppliedCellDirty(sensorarrayState_t *state,
                                                      const sensorarrayFdcCellTarget_t *target);

static bool sensorarrayMeasureFdcReasonEquals(const char *reason, const char *expected)
{
    return reason && expected && strcmp(reason, expected) == 0;
}

static bool sensorarrayMeasureFdcRescueReasonIsManual(const char *reason)
{
    return sensorarrayMeasureFdcReasonEquals(reason, "manual_force_sweep") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_force_full_sweep_all") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_rescue") ||
           sensorarrayMeasureFdcReasonEquals(reason, "force_full_sweep") ||
           sensorarrayMeasureFdcReasonEquals(reason, "fdc_rescue");
}

static bool sensorarrayMeasureFdcRescueReasonIsHard(const char *reason)
{
    return sensorarrayMeasureFdcReasonEquals(reason, "cache_missing_and_hard_error") ||
           sensorarrayMeasureFdcReasonEquals(reason, "cache_apply_failed") ||
           sensorarrayMeasureFdcReasonEquals(reason, "no_unread_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "zero_raw_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "watchdog_fault_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "i2c_error_consecutive") ||
           sensorarrayMeasureFdcReasonEquals(reason, "device_config_lost") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_force_sweep") ||
           sensorarrayMeasureFdcReasonEquals(reason, "manual_rescue") ||
           sensorarrayMeasureFdcReasonEquals(reason, "i2c_read_error") ||
           sensorarrayMeasureFdcReasonEquals(reason, "watchdog_fault") ||
           sensorarrayMeasureFdcReasonEquals(reason, "zero_raw_no_oscillation") ||
           sensorarrayMeasureFdcReasonEquals(reason, "invalid_streak") ||
           sensorarrayMeasureFdcReasonEquals(reason, "all_invalid_frame");
}

esp_err_t sensorarrayMeasureRequestFdcCellRescue(sensorarrayState_t *state,
                                                uint8_t matrixIndex,
                                                const char *reason)
{
    if (!state) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t sColumn = 0u;
    uint8_t dLine = 0u;
    if (!sensorarrayMeasureDecodeMatrixIndex(matrixIndex, &sColumn, &dLine)) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, sColumn, dLine, &target)) {
        return ESP_ERR_INVALID_STATE;
    }

    sensorarrayFdcCellConfigCache_t *cache = sensorarrayMeasureGetFdcCellCache(state, &target);
    if (!cache) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *source = reason ? reason : "runtime_cell_rescue";
    if (sensorarrayMeasureFdcReasonEquals(source, "amplitude_warning") && cache->valid) {
        cache->reapplyPending = true;
        cache->lastWarningTimestampUs = esp_timer_get_time();
        snprintf(cache->lastWarningReason, sizeof(cache->lastWarningReason), "%s", source);
        sensorarrayMeasureMarkFdcAppliedCellDirty(state, &target);
        printf("FDC_CACHE,stage=reapply_pending,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=amplitude_warning,cacheValid=1\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel);
        return ESP_OK;
    }

    if (!sensorarrayMeasureFdcRescueReasonIsHard(source)) {
        printf("FDC_RESCUE_SUPPRESSED,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,policy=not_hard_error\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel,
               source);
        return ESP_OK;
    }

    if (!CONFIG_SENSORARRAY_FDC_RUNTIME_FAST_SWEEP_ENABLE &&
        !sensorarrayMeasureFdcRescueReasonIsManual(source)) {
        printf("FDC_RESCUE_SUPPRESSED,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,policy=runtime_fast_sweep_disabled\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel,
               source);
        return ESP_OK;
    }

    int64_t nowUs = esp_timer_get_time();
    int64_t cooldownUs = (int64_t)CONFIG_SENSORARRAY_FDC_FAST_SWEEP_MIN_COOLDOWN_MS * 1000LL;
    if (cache->lastRescueTimestampUs != 0 &&
        cooldownUs > 0 &&
        (nowUs - cache->lastRescueTimestampUs) < cooldownUs &&
        !sensorarrayMeasureFdcRescueReasonIsManual(source)) {
        printf("FDC_RESCUE_SUPPRESSED,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,policy=cooldown\n",
               (unsigned)target.sColumn,
               (unsigned)target.dLine,
               (unsigned)target.matrixIndex,
               sensorarrayMeasureFdcDeviceName(target.devId),
               (unsigned)target.fdcChannel,
               source);
        return ESP_OK;
    }

    cache->rescuePending = true;
    snprintf(cache->lastRescueReason, sizeof(cache->lastRescueReason), "%s", source);

    printf("FDC_RESCUE,stage=pending,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s,consecutiveWarnings=%u,consecutiveErrors=%u\n",
           (unsigned)target.sColumn,
           (unsigned)target.dLine,
           (unsigned)target.matrixIndex,
           sensorarrayMeasureFdcDeviceName(target.devId),
           (unsigned)target.fdcChannel,
           cache->lastRescueReason,
           (unsigned)cache->consecutiveAmplitudeWarnings,
           (unsigned)cache->consecutiveErrors);
    return ESP_OK;
}

esp_err_t sensorarrayMeasureFdcDiscardStaleSamples(sensorarrayState_t *state,
                                                   const sensorarrayFdcCellTarget_t *target,
                                                   uint8_t discardCount,
                                                   const char *reason)
{
    if (!state || !target || !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine) ||
        target->fdcChannel > (uint8_t)FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, target->devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    printf("FDC_DISCARD,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,count=%u,reason=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           (unsigned)discardCount,
           reason ? reason : SENSORARRAY_NA);

    esp_err_t firstErr = ESP_OK;
    for (uint8_t i = 0u; i < discardCount; ++i) {
        Fdc2214CapSample_t discard = {0};
        esp_err_t err = Fdc2214CapReadSampleRelaxed(fdcState->handle,
                                                    (Fdc2214CapChannel_t)target->fdcChannel,
                                                    &discard);
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    return firstErr;
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
    uint8_t selectedCode = (uint8_t)FDC2214_DEGLITCH_3P3MHZ;
    uint32_t selectedBandwidthHz = 3300000u;

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

static uint32_t sensorarrayMeasureFdcDeglitchBandwidthHz(uint8_t deglitchCode)
{
    switch (deglitchCode) {
    case FDC2214_DEGLITCH_1MHZ:
        return 1000000u;
    case FDC2214_DEGLITCH_3P3MHZ:
        return 3300000u;
    case FDC2214_DEGLITCH_10MHZ:
        return 10000000u;
    case FDC2214_DEGLITCH_33MHZ:
        return 33000000u;
    default:
        return 0u;
    }
}

static uint8_t sensorarrayMeasureFdcSafeDefaultDeglitch(void)
{
    return (uint8_t)FDC2214_DEGLITCH_10MHZ;
}

static void sensorarrayMeasureMarkFdcAppliedCellDirty(sensorarrayState_t *state,
                                                      const sensorarrayFdcCellTarget_t *target)
{
    if (!state || !target || target->devId > SENSORARRAY_FDC_DEV_SECONDARY || target->fdcChannel >= 4u) {
        return;
    }

    sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)target->devId];
    if (applied->valid &&
        applied->row == target->sColumn &&
        applied->deviceId == (uint8_t)target->devId) {
        applied->dirty = true;
    }
}

static bool sensorarrayMeasureAppliedRowConfigMatches(const sensorarrayFdcAppliedRowConfig_t *applied,
                                                      const sensorarrayFdcAppliedRowConfig_t *expected)
{
    if (!applied || !expected ||
        !applied->valid ||
        !applied->autoscanConfigured ||
        applied->dirty ||
        applied->row != expected->row ||
        applied->deviceId != expected->deviceId ||
        applied->selectedDeglitch != expected->selectedDeglitch) {
        return false;
    }

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        if (applied->rCount[ch] != expected->rCount[ch] ||
            applied->settleCount[ch] != expected->settleCount[ch] ||
            applied->clockDiv[ch] != expected->clockDiv[ch] ||
            applied->driveCurrent[ch] != expected->driveCurrent[ch] ||
            applied->cacheGeneration[ch] != expected->cacheGeneration[ch]) {
            return false;
        }
    }
    return true;
}

static sensorarrayFdcCellConfigCache_t *sensorarrayMeasureFdcRowDeviceCache(sensorarrayState_t *state,
                                                                            uint8_t row,
                                                                            sensorarrayFdcDeviceId_t devId,
                                                                            uint8_t ch,
                                                                            uint8_t *outDIndex)
{
    if (!state || row < 1u || row > SENSORARRAY_MATRIX_ROWS || ch >= 4u) {
        return NULL;
    }

    uint8_t dIndex = (uint8_t)(1u + ch + ((devId == SENSORARRAY_FDC_DEV_SECONDARY) ? 4u : 0u));
    if (outDIndex) {
        *outDIndex = dIndex;
    }
    sensorarrayFdcCellTarget_t target = {0};
    if (!sensorarrayMeasureMakeFdcCellTarget(state, row, dIndex, &target)) {
        return NULL;
    }
    return sensorarrayMeasureGetFdcCellCache(state, &target);
}

static uint8_t sensorarrayFdcMergeDeglitchForRowDevice(sensorarrayState_t *state,
                                                       uint8_t row,
                                                       sensorarrayFdcDeviceId_t devId,
                                                       uint8_t safeDefaultDeglitch)
{
    uint8_t selected = sensorarrayMeasureFdcDeglitchCodeValid(safeDefaultDeglitch) ?
        safeDefaultDeglitch :
        sensorarrayMeasureFdcSafeDefaultDeglitch();
    uint32_t selectedBandwidthHz = sensorarrayMeasureFdcDeglitchBandwidthHz(selected);
    bool anyValid = false;
    uint8_t sourceDeglitch[4] = {0};

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, NULL);
        sourceDeglitch[ch] = cache ? cache->deglitchCode : 0u;
        if (!cache || !cache->valid || !sensorarrayMeasureFdcDeglitchCodeValid(cache->deglitchCode)) {
            continue;
        }
        anyValid = true;
        uint32_t bandwidthHz = sensorarrayMeasureFdcDeglitchBandwidthHz(cache->deglitchCode);
        if (bandwidthHz >= selectedBandwidthHz) {
            selected = cache->deglitchCode;
            selectedBandwidthHz = bandwidthHz;
        }
    }

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_CACHE,stage=row_deglitch_merge,row=%u,device=%s,sourceDeglitch=[0x%X,0x%X,0x%X,0x%X],selected=0x%X,reason=%s\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)sourceDeglitch[0],
           (unsigned)sourceDeglitch[1],
           (unsigned)sourceDeglitch[2],
           (unsigned)sourceDeglitch[3],
           (unsigned)selected,
           anyValid ? "cover_highest_frequency" : "safe_default");
#else
    (void)anyValid;
    (void)sourceDeglitch;
#endif
    return selected;
}

static bool sensorarrayMeasureFdcAutoscanConfigLooksCurrent(const sensorarrayFdcDeviceState_t *fdcState,
                                                            uint8_t expectedDeglitch)
{
    if (!fdcState || !fdcState->configVerified) {
        return false;
    }

    uint8_t rr = (uint8_t)((fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                           SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
    uint8_t deglitch = (uint8_t)(fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
    bool autoscan = (fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
    bool highCurrent = (fdcState->configReg & SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
    bool sleeping = (fdcState->configReg & 0x2000u) != 0u;
    return autoscan &&
           rr == SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE &&
           deglitch == expectedDeglitch &&
           !highCurrent &&
           !sleeping;
}

static void sensorarrayMeasureRuntimeConfigsFromApplied(sensorarrayState_t *state,
                                                        sensorarrayFdcDeviceId_t devId,
                                                        sensorarrayFdcRuntimeChannelConfig_t configs[4])
{
    if (!state || !configs || devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return;
    }

    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    uint32_t effectiveFclkHz =
        (fdcState && fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
        fdcState->refClockHz :
        sensorarrayMeasureFdcEffectiveFclkHz();
    const sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        configs[ch] = (sensorarrayFdcRuntimeChannelConfig_t){
            .valid = applied->valid,
            .rCount = applied->rCount[ch],
            .settleCount = applied->settleCount[ch],
            .clockDividers = applied->clockDiv[ch],
            .driveCurrent = applied->driveCurrent[ch],
            .deglitchCode = applied->selectedDeglitch,
            .effectiveFclkHz = effectiveFclkHz,
        };
    }
}

static esp_err_t sensorarrayMeasureApplyFdcCachedRowConfig(sensorarrayState_t *state,
                                                           uint8_t row,
                                                           sensorarrayFdcDeviceId_t devId,
                                                           const char *reason,
                                                           bool forceWrite,
                                                           sensorarrayFdcDeviceTiming_t *timing)
{
    if (!state || row < 1u || row > SENSORARRAY_MATRIX_ROWS ||
        devId > SENSORARRAY_FDC_DEV_SECONDARY) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }
    int64_t applyStartUs = esp_timer_get_time();
    if (timing) {
        timing->row = row;
        timing->deviceId = devId;
    }

    int64_t buildStartUs = esp_timer_get_time();
    sensorarrayFdcAppliedRowConfig_t expected = {
        .valid = true,
        .autoscanConfigured = true,
        .row = row,
        .deviceId = (uint8_t)devId,
        .selectedDeglitch = sensorarrayMeasureFdcSafeDefaultDeglitch(),
    };
    uint8_t missingMask = 0u;
    bool reapplyRequested = false;

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        uint8_t dIndex = 0u;
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, &dIndex);
        bool useCache = cache && cache->valid;
        if (!useCache) {
            missingMask |= (uint8_t)(1u << ch);
        }
        if (cache && cache->reapplyPending) {
            reapplyRequested = true;
        }

        expected.rCount[ch] = (useCache && cache->rCount != 0u) ?
            cache->rCount :
            SENSORARRAY_FDC_RCOUNT;
        expected.settleCount[ch] = (useCache && cache->settleCount != 0u) ?
            cache->settleCount :
            SENSORARRAY_FDC_SETTLECOUNT;
        expected.clockDiv[ch] = (useCache && cache->clockDiv != 0u) ?
            cache->clockDiv :
            SENSORARRAY_FDC_CLOCK_DIVIDERS;
        expected.driveCurrent[ch] = (useCache && cache->driveCurrent != 0u) ?
            cache->driveCurrent :
            SENSORARRAY_FDC_DRIVE_CURRENT;
        expected.cacheGeneration[ch] = useCache ? cache->generation : 0u;
        (void)dIndex;
    }

    expected.selectedDeglitch =
        sensorarrayFdcMergeDeglitchForRowDevice(state,
                                                row,
                                                devId,
                                                sensorarrayMeasureFdcSafeDefaultDeglitch());
    if (timing) {
        timing->applyBuildConfigUs += sensorarrayMeasureElapsedUs(buildStartUs);
    }

    if (missingMask != 0u) {
        printf("FDC_CACHE,stage=miss,row=%u,device=%s,missingMask=0x%X,action=use_safe_default\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               (unsigned)missingMask);
    }

    sensorarrayFdcAppliedRowConfig_t *applied = &state->fdcAppliedRow[(uint8_t)devId];
    if (!forceWrite &&
        !reapplyRequested &&
        sensorarrayMeasureAppliedRowConfigMatches(applied, &expected)) {
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
        printf("FDC_CACHE,stage=hit,row=%u,device=%s,action=no_write\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId));
#endif
        if (timing) {
            timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
        }
        return ESP_OK;
    }

    esp_err_t firstErr = ESP_OK;
    const bool runtimeFullVerify = (CONFIG_SENSORARRAY_FDC_VERIFY_MODE_FULL != 0);
    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        Fdc2214CapChannelConfig_t cfg = {
            .Rcount = expected.rCount[ch],
            .SettleCount = expected.settleCount[ch],
            .Offset = SENSORARRAY_FDC_OFFSET,
            .ClockDividers = expected.clockDiv[ch],
            .DriveCurrent = expected.driveCurrent[ch],
        };
        int64_t channelWriteStartUs = esp_timer_get_time();
        esp_err_t err = Fdc2214CapConfigureChannelWriteOnly(fdcState->handle,
                                                            (Fdc2214CapChannel_t)ch,
                                                            &cfg);
        if (timing) {
            timing->channelConfigWriteUs += sensorarrayMeasureElapsedUs(channelWriteStartUs);
        }
        if (err == ESP_OK && runtimeFullVerify) {
            int64_t verifyStartUs = esp_timer_get_time();
            err = Fdc2214CapReadbackVerifyChannelConfig(fdcState->handle,
                                                        (Fdc2214CapChannel_t)ch,
                                                        &cfg);
            if (timing) {
                timing->verifyUs += sensorarrayMeasureElapsedUs(verifyStartUs);
            }
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }

    if (firstErr == ESP_OK) {
        bool globalCurrent =
            !forceWrite &&
            sensorarrayMeasureFdcAutoscanConfigLooksCurrent(fdcState, expected.selectedDeglitch);
        if (!globalCurrent) {
            uint16_t configWritten = 0u;
            uint16_t muxWritten = 0u;
            uint16_t configTemplate = fdcState->configReg;
            if (configTemplate == 0u) {
                Fdc2214CapConfigOptions_t fallbackOptions = {
                    .ActiveChannel = FDC2214_CH0,
                    .SleepModeEnabled = false,
                    .SensorActivateSelLowPower = true,
                    .RefClockSource = (fdcState->refClockSource == FDC2214_REF_CLOCK_EXTERNAL) ?
                        FDC2214_REF_CLOCK_EXTERNAL :
                        FDC2214_REF_CLOCK_INTERNAL,
                    .IntbDisabled = false,
                    .HighCurrentDrive = false,
                };
                configTemplate = Fdc2214CapBuildConfig(&fallbackOptions);
            }
            int64_t globalWriteStartUs = esp_timer_get_time();
            firstErr = Fdc2214CapSetAutoScanModeWriteOnly(fdcState->handle,
                                                          SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE,
                                                          (Fdc2214CapDeglitch_t)expected.selectedDeglitch,
                                                          configTemplate,
                                                          &configWritten,
                                                          &muxWritten);
            if (timing) {
                timing->globalConfigWriteUs += sensorarrayMeasureElapsedUs(globalWriteStartUs);
            }
            if (firstErr == ESP_OK) {
                fdcState->configReg = configWritten;
                fdcState->muxConfigReg = muxWritten;
                fdcState->configVerified = !runtimeFullVerify;
            }
            if (firstErr == ESP_OK && runtimeFullVerify) {
                Fdc2214CapCoreRegs_t regs = {0};
                int64_t verifyStartUs = esp_timer_get_time();
                esp_err_t verifyErr = Fdc2214CapReadCoreRegs(fdcState->handle, &regs);
                if (timing) {
                    timing->verifyUs += sensorarrayMeasureElapsedUs(verifyStartUs);
                }
                if (verifyErr == ESP_OK) {
                    fdcState->statusConfigReg = regs.StatusConfig;
                    fdcState->configReg = regs.Config;
                    fdcState->muxConfigReg = regs.MuxConfig;
                    fdcState->configVerified = true;
                    uint8_t rr = (uint8_t)((regs.MuxConfig & SENSORARRAY_FDC_MUX_RR_SEQUENCE_MASK) >>
                                           SENSORARRAY_FDC_MUX_RR_SEQUENCE_SHIFT);
                    uint8_t deglitch = (uint8_t)(regs.MuxConfig & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
                    bool autoscan = (regs.MuxConfig & SENSORARRAY_FDC_MUX_AUTOSCAN_EN_MASK) != 0u;
                    bool highCurrent = (regs.Config & SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
                    if (!autoscan ||
                        rr != SENSORARRAY_FDC_AUTOSCAN_RR_SEQUENCE ||
                        deglitch != expected.selectedDeglitch ||
                        highCurrent) {
                        verifyErr = ESP_ERR_INVALID_RESPONSE;
                    }
                }
                if (verifyErr != ESP_OK) {
                    firstErr = verifyErr;
                    fdcState->configVerified = false;
                }
            }
        }
    }

    if (firstErr != ESP_OK) {
        applied->dirty = true;
        printf("FDC_CACHE,stage=apply_row_failed,row=%u,device=%s,reason=%s,err=0x%lx\n",
               (unsigned)row,
               sensorarrayMeasureFdcDeviceName(devId),
               reason ? reason : SENSORARRAY_NA,
               (unsigned long)firstErr);
        if (timing) {
            timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
        }
        return firstErr;
    }

    uint32_t nextApplyCount = applied->applyCount + 1u;
    expected.applyCount = nextApplyCount;
    expected.lastAppliedTimestampUs = esp_timer_get_time();
    expected.dirty = false;
    *applied = expected;

    for (uint8_t ch = 0u; ch < 4u; ++ch) {
        sensorarrayFdcCellConfigCache_t *cache =
            sensorarrayMeasureFdcRowDeviceCache(state, row, devId, ch, NULL);
        if (cache) {
            cache->lastAppliedTimestampUs = applied->lastAppliedTimestampUs;
            cache->reapplyPending = false;
        }
    }

    printf("FDC_CACHE,stage=apply_row,row=%u,device=%s,reason=%s,drive=[0x%04X,0x%04X,0x%04X,0x%04X],rCount=[0x%04X,0x%04X,0x%04X,0x%04X],settle=[0x%04X,0x%04X,0x%04X,0x%04X],clockDiv=[0x%04X,0x%04X,0x%04X,0x%04X],deglitch=0x%X,force=%u\n",
           (unsigned)row,
           sensorarrayMeasureFdcDeviceName(devId),
           reason ? reason : SENSORARRAY_NA,
           applied->driveCurrent[0],
           applied->driveCurrent[1],
           applied->driveCurrent[2],
           applied->driveCurrent[3],
           applied->rCount[0],
           applied->rCount[1],
           applied->rCount[2],
           applied->rCount[3],
           applied->settleCount[0],
           applied->settleCount[1],
           applied->settleCount[2],
           applied->settleCount[3],
           applied->clockDiv[0],
           applied->clockDiv[1],
           applied->clockDiv[2],
           applied->clockDiv[3],
           (unsigned)applied->selectedDeglitch,
           forceWrite || reapplyRequested ? 1u : 0u);
    if (timing) {
        timing->applyUs += sensorarrayMeasureElapsedUs(applyStartUs);
    }
    return ESP_OK;
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

static esp_err_t __attribute__((unused)) sensorarrayMeasureEnsureFdcAutoscan4ch(sensorarrayState_t *state,
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

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_AUTOSCAN_CONFIG,device=%s,mux=0x%04X,config=0x%04X,autoscan=%u,rr=%u,deglitch=0x%X,highCurrent=%u\n",
           sensorarrayMeasureFdcDeviceName(devId),
           regs.MuxConfig,
           regs.Config,
           autoscan ? 1u : 0u,
           (unsigned)rr,
           (unsigned)muxDeglitch,
           highCurrent ? 1u : 0u);
#endif
    (void)muxDeglitch;

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
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
                printf("FDC_FRAME_READY,row=%u,device=%s,status=0x%04X,unreadMask=0x%X,drdy=%u,err=0x%lx\n",
                       (unsigned)row,
                       sensorarrayMeasureFdcDeviceName(sensorarrayMeasureFdcDeviceIdFromState(fdcState)),
                       status.Raw,
                       (unsigned)unreadMask,
                       status.DataReady ? 1u : 0u,
                       (unsigned long)ESP_OK);
#endif
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

static void sensorarrayMeasurePollFdcReady(sensorarrayFdcDeviceState_t *fdcState,
                                           sensorarrayFdcReadyState_t *ready)
{
    if (!ready) {
        return;
    }
    if (!fdcState || !fdcState->handle) {
        ready->err = ESP_ERR_INVALID_STATE;
        return;
    }

    ready->pollCount++;
    Fdc2214CapStatus_t status = {0};
    esp_err_t err = Fdc2214CapReadStatus(fdcState->handle, &status);
    ready->err = err;
    if (err != ESP_OK) {
        return;
    }
    ready->statusRaw = status.Raw;
    ready->dataReady = status.DataReady;
    ready->unreadMask = sensorarrayMeasureFdcUnreadMaskFromStatus(&status);
    ready->ready = ready->dataReady ||
                   ready->unreadMask == SENSORARRAY_FDC_AUTOSCAN_READY_UNREAD_MASK;
}

static esp_err_t sensorarrayMeasureWaitBothFdcAutoscanFrameReady(sensorarrayState_t *state,
                                                                 uint8_t row,
                                                                 uint32_t timeoutMs,
                                                                 sensorarrayFdcReadyState_t *primaryReady,
                                                                 sensorarrayFdcReadyState_t *secondaryReady)
{
    if (!state || !primaryReady || !secondaryReady) {
        return ESP_ERR_INVALID_ARG;
    }
    if (timeoutMs == 0u) {
        timeoutMs = 1u;
    }

    *primaryReady = (sensorarrayFdcReadyState_t){.err = ESP_ERR_TIMEOUT};
    *secondaryReady = (sensorarrayFdcReadyState_t){.err = ESP_ERR_TIMEOUT};
    int64_t deadlineUs = esp_timer_get_time() + ((int64_t)timeoutMs * 1000LL);
    while (esp_timer_get_time() <= deadlineUs) {
        if (!primaryReady->ready) {
            sensorarrayMeasurePollFdcReady(&state->fdcPrimary, primaryReady);
        }
        if (!secondaryReady->ready) {
            sensorarrayMeasurePollFdcReady(&state->fdcSecondary, secondaryReady);
        }
        if (primaryReady->ready && secondaryReady->ready) {
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
            printf("FDC_FRAME_READY,row=%u,device=both,primaryStatus=0x%04X,primaryUnread=0x%X,primaryDrdy=%u,secondaryStatus=0x%04X,secondaryUnread=0x%X,secondaryDrdy=%u,err=0x0\n",
                   (unsigned)row,
                   primaryReady->statusRaw,
                   (unsigned)primaryReady->unreadMask,
                   primaryReady->dataReady ? 1u : 0u,
                   secondaryReady->statusRaw,
                   (unsigned)secondaryReady->unreadMask,
                   secondaryReady->dataReady ? 1u : 0u);
#endif
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(1u));
    }

    esp_err_t err = ESP_ERR_TIMEOUT;
    if (!primaryReady->ready && primaryReady->err != ESP_OK && primaryReady->err != ESP_ERR_TIMEOUT) {
        err = primaryReady->err;
    } else if (!secondaryReady->ready &&
               secondaryReady->err != ESP_OK &&
               secondaryReady->err != ESP_ERR_TIMEOUT) {
        err = secondaryReady->err;
    }
    if (!primaryReady->ready) {
        primaryReady->timeoutCount++;
    }
    if (!secondaryReady->ready) {
        secondaryReady->timeoutCount++;
    }
    printf("FDC_FRAME_READY,row=%u,device=both,primaryReady=%u,primaryStatus=0x%04X,primaryUnread=0x%X,primaryDrdy=%u,primaryErr=0x%lx,secondaryReady=%u,secondaryStatus=0x%04X,secondaryUnread=0x%X,secondaryDrdy=%u,secondaryErr=0x%lx,err=0x%lx\n",
           (unsigned)row,
           primaryReady->ready ? 1u : 0u,
           primaryReady->statusRaw,
           (unsigned)primaryReady->unreadMask,
           primaryReady->dataReady ? 1u : 0u,
           (unsigned long)primaryReady->err,
           secondaryReady->ready ? 1u : 0u,
           secondaryReady->statusRaw,
           (unsigned)secondaryReady->unreadMask,
           secondaryReady->dataReady ? 1u : 0u,
           (unsigned long)secondaryReady->err,
           (unsigned long)err);
    return err;
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
        outSamples->i2cError[ch] = (sample->errorMask & FDC2214CAP_FAST_ERROR_I2C) != 0u;

        bool i2cOk = !outSamples->i2cError[ch];
        bool readable = sample->unreadConversion || sample->dataReady;
        outSamples->valid[ch] = i2cOk &&
                                readable &&
                                sample->raw28 != 0u &&
                                !outSamples->watchdogFault[ch] &&
                                !outSamples->saturated[ch];
    }
    return firstErr;
}

static esp_err_t __attribute__((unused)) sensorarrayMeasureReadFdcRuntimeChannelConfigs(sensorarrayState_t *state,
                                                                                        sensorarrayFdcRuntimeChannelConfig_t configs[2][4])
{
    if (!state || !configs) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(configs, 0, sizeof(sensorarrayFdcRuntimeChannelConfig_t) * 2u * 4u);
    esp_err_t firstErr = ESP_OK;
    for (uint8_t dev = 0u; dev < 2u; ++dev) {
        sensorarrayFdcDeviceState_t *fdcState =
            sensorarrayMeasureGetFdcState(state, (sensorarrayFdcDeviceId_t)dev);
        if (!fdcState || !fdcState->ready || !fdcState->handle) {
            if (firstErr == ESP_OK) {
                firstErr = ESP_ERR_INVALID_STATE;
            }
            continue;
        }

        Fdc2214CapCoreRegs_t coreRegs = {0};
        esp_err_t coreErr = Fdc2214CapReadCoreRegs(fdcState->handle, &coreRegs);
        uint8_t deglitchCode = (coreErr == ESP_OK) ?
            (uint8_t)(coreRegs.MuxConfig & SENSORARRAY_FDC_MUX_DEGLITCH_MASK) :
            (uint8_t)(fdcState->muxConfigReg & SENSORARRAY_FDC_MUX_DEGLITCH_MASK);
        uint32_t effectiveFclkHz =
            (fdcState->refClockKnown && fdcState->refClockHz != 0u) ?
            fdcState->refClockHz :
            sensorarrayMeasureFdcEffectiveFclkHz();

        if (coreErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = coreErr;
        }

        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            sensorarrayFdcRuntimeChannelConfig_t *cfg = &configs[dev][ch];
            cfg->deglitchCode = deglitchCode;
            cfg->effectiveFclkHz = effectiveFclkHz;

            esp_err_t err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                       sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_RCOUNT_BASE,
                                                                                          (Fdc2214CapChannel_t)ch),
                                                       &cfg->rCount);
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_SETTLECOUNT_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->settleCount);
            }
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->clockDividers);
            }
            if (err == ESP_OK) {
                err = Fdc2214CapReadRawRegisters(fdcState->handle,
                                                 sensorarrayMeasureFdcRegForChannel(SENSORARRAY_FDC_REG_DRIVE_CURRENT_BASE,
                                                                                    (Fdc2214CapChannel_t)ch),
                                                 &cfg->driveCurrent);
            }
            cfg->valid = err == ESP_OK && cfg->clockDividers != 0u && cfg->effectiveFclkHz != 0u;
            if (err != ESP_OK && firstErr == ESP_OK) {
                firstErr = err;
            }
        }
    }
    return firstErr;
}

static esp_err_t sensorarrayMeasureDiscardFdcAutoscanRow(sensorarrayState_t *state,
                                                         uint8_t sIndex,
                                                         sensorarrayFdcDeviceId_t devId,
                                                         uint8_t discardCount,
                                                         const char *reason)
{
    if (!state || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (discardCount == 0u) {
        return ESP_OK;
    }
    sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, devId);
    if (!fdcState || !fdcState->ready || !fdcState->handle) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t firstErr = ESP_OK;
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
    printf("FDC_DISCARD,scope=device,row=%u,device=%s,count=%u,reason=%s\n",
           (unsigned)sIndex,
           sensorarrayMeasureFdcDeviceName(devId),
           (unsigned)discardCount,
           reason ? reason : SENSORARRAY_NA);
#endif

    for (uint8_t i = 0u; i < discardCount; ++i) {
        uint16_t status = 0u;
        esp_err_t err = sensorarrayMeasureWaitFdcAutoscanFrameReady(fdcState,
                                                                    sIndex,
                                                                    (uint32_t)SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS,
                                                                    &status);
        if (err == ESP_OK) {
            sensorarrayFdcAutoscanSamples_t discard = {0};
            err = sensorarrayMeasureReadFdcAutoscan4ch(fdcState, &discard);
        }
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }
    }
    return firstErr;
}

static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  double freqHz,
                                                  const sensorarrayFdcRuntimeChannelConfig_t *config,
                                                  bool valid,
                                                  bool warning,
                                                  bool error);

static void sensorarrayMeasureFillFdcMatrixRow(sensorarrayFdcMatrixFrame_t *frame,
                                               uint8_t sIndex,
                                               const sensorarrayFdcAutoscanSamples_t *primary,
                                               const sensorarrayFdcAutoscanSamples_t *secondary,
                                               const sensorarrayFdcRuntimeChannelConfig_t configs[2][4],
                                               uint8_t *outValidMask8,
                                               uint8_t *outWarnMask8,
                                               uint8_t *outErrorMask8)
{
    uint8_t validMask8 = 0u;
    uint8_t warnMask8 = 0u;
    uint8_t errorMask8 = 0u;
    const sensorarrayFdcAutoscanSamples_t *samplesByHalf[2] = {primary, secondary};

    for (uint8_t half = 0u; half < 2u; ++half) {
        const sensorarrayFdcAutoscanSamples_t *samples = samplesByHalf[half];
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + (half * 4u));
            bool valid = samples && samples->valid[ch];
            bool warning = samples && samples->amplitudeWarning[ch];
            const sensorarrayFdcRuntimeChannelConfig_t *config = configs ? &configs[half][ch] : NULL;
            bool configOk = config && config->valid;
            bool severeFault = !valid ||
                               !configOk ||
                               (samples && (samples->i2cError[ch] ||
                                            samples->watchdogFault[ch] ||
                                            samples->saturated[ch] ||
                                            samples->raw28[ch] == 0u));
            bool error = severeFault;
            uint32_t raw28 = samples ? samples->raw28[ch] : 0u;
            double freqHz = (valid && configOk) ?
                sensorarrayMeasureFdcRaw28ToFreqHz(raw28, config->effectiveFclkHz, config->clockDividers) :
                0.0;
            bool frameValid = valid && freqHz > 0.0;
            sensorarrayMeasureMarkFdcMatrixCellEx(frame,
                                                  sIndex,
                                                  dIndex,
                                                  raw28,
                                                  freqHz,
                                                  config,
                                                  frameValid,
                                                  warning,
                                                  error);
            if (frameValid) {
                validMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (warning) {
                warnMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
            if (error) {
                errorMask8 |= (uint8_t)(1u << (dIndex - 1u));
            }
        }
    }

    if (outValidMask8) {
        *outValidMask8 = validMask8;
    }
    if (outWarnMask8) {
        *outWarnMask8 = warnMask8;
    }
    if (outErrorMask8) {
        *outErrorMask8 = errorMask8;
    }
}

static void sensorarrayMeasureAccumulateFdcHealth(sensorarrayFdcFrameHealth_t *health,
                                                  uint8_t sIndex,
                                                  const sensorarrayFdcAutoscanSamples_t *primary,
                                                  const sensorarrayFdcAutoscanSamples_t *secondary,
                                                  const sensorarrayFdcRuntimeChannelConfig_t configs[2][4],
                                                  const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!health || !sensorarrayMatrixIndexIsValid(sIndex, 1u)) {
        return;
    }

    const sensorarrayFdcAutoscanSamples_t *samplesByHalf[2] = {primary, secondary};
    for (uint8_t half = 0u; half < 2u; ++half) {
        const sensorarrayFdcAutoscanSamples_t *samples = samplesByHalf[half];
        for (uint8_t ch = 0u; ch < 4u; ++ch) {
            uint8_t dIndex = (uint8_t)(1u + ch + (half * 4u));
            size_t matrixIndex = sensorarrayMatrixIndex(sIndex, dIndex);
            uint8_t s0 = (uint8_t)(sIndex - 1u);
            uint8_t d0 = (uint8_t)(dIndex - 1u);
            bool valid = samples && samples->valid[ch] && frame &&
                         ((frame->validMask & (1ULL << matrixIndex)) != 0u);
            if (valid) {
                health->validSeen[s0][d0] = true;
                health->lastRaw28[s0][d0] = samples->raw28[ch];
                health->lastFreqHz[s0][d0] = frame ? frame->freqHz[matrixIndex] : 0.0;
            } else {
                health->invalidSeen[s0][d0] = true;
            }
            health->amplitudeWarningSeen[s0][d0] =
                health->amplitudeWarningSeen[s0][d0] || (samples && samples->amplitudeWarning[ch]);
            health->watchdogSeen[s0][d0] =
                health->watchdogSeen[s0][d0] || (samples && samples->watchdogFault[ch]);
            health->saturatedSeen[s0][d0] =
                health->saturatedSeen[s0][d0] || (samples && samples->saturated[ch]);
            health->zeroRawSeen[s0][d0] =
                health->zeroRawSeen[s0][d0] || (!samples || samples->raw28[ch] == 0u);
            health->i2cErrorSeen[s0][d0] =
                health->i2cErrorSeen[s0][d0] || (samples && samples->i2cError[ch]);
            if (configs) {
                const sensorarrayFdcRuntimeChannelConfig_t *config = &configs[half][ch];
                health->clockDividers[s0][d0] = config->clockDividers;
                health->driveCurrent[s0][d0] = config->driveCurrent;
                health->deglitchCode[s0][d0] = config->deglitchCode;
                health->effectiveFclkHz[s0][d0] = config->effectiveFclkHz;
            }
        }
    }
}

static void sensorarrayMeasureUpdateFdcRuntimeProfiles(sensorarrayState_t *state,
                                                       const sensorarrayFdcFrameHealth_t *health)
{
    if (!state || !health) {
        return;
    }

    uint32_t threshold = (uint32_t)CONFIG_SENSORARRAY_FDC_RESCUE_HARD_ERROR_THRESHOLD;
    if (threshold == 0u) {
        threshold = 1u;
    }

    uint32_t amplitudeThreshold = (uint32_t)CONFIG_SENSORARRAY_FDC_WARNING_REAPPLY_THRESHOLD;
    if (amplitudeThreshold == 0u) {
        amplitudeThreshold = 1u;
    }

    int64_t nowUs = esp_timer_get_time();
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
            uint8_t s0 = (uint8_t)(s - 1u);
            uint8_t d0 = (uint8_t)(d - 1u);
            uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(s, d);
            sensorarrayFdcCellTarget_t target = {0};
            if (!sensorarrayMeasureMakeFdcCellTarget(state, s, d, &target)) {
                continue;
            }

            sensorarrayFdcCellConfigCache_t *cache = sensorarrayMeasureGetFdcCellCache(state, &target);
            sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, target.devId);
            if (!cache || !fdcState || target.fdcChannel > (uint8_t)FDC2214_CH3) {
                continue;
            }

            sensorarrayFdcSweepProfile_t *profile = &fdcState->sweepProfile[target.fdcChannel];
            bool valid = health->validSeen[s0][d0];
            bool invalid = health->invalidSeen[s0][d0];
            bool amplitude = health->amplitudeWarningSeen[s0][d0];
            bool severeError = invalid ||
                               health->i2cErrorSeen[s0][d0] ||
                               health->watchdogSeen[s0][d0] ||
                               health->saturatedSeen[s0][d0] ||
                               health->zeroRawSeen[s0][d0];

            if (valid) {
                bool wasCacheValid = cache->valid;
                if (!wasCacheValid) {
                    uint32_t generation = cache->generation + 1u;
                    cache->source = SENSORARRAY_FDC_CACHE_SOURCE_LAST_GOOD;
                    cache->rCount = health->clockDividers[s0][d0] ? SENSORARRAY_FDC_RCOUNT : cache->rCount;
                    cache->settleCount = SENSORARRAY_FDC_SETTLECOUNT;
                    cache->clockDiv = health->clockDividers[s0][d0];
                    cache->driveCurrent = health->driveCurrent[s0][d0];
                    cache->deglitchCode = health->deglitchCode[s0][d0];
                    cache->effectiveFclkHz = health->effectiveFclkHz[s0][d0];
                    cache->generation = (generation == 0u) ? 1u : generation;
                    cache->storedTimestampUs = nowUs;
                    sensorarrayMeasureMarkFdcAppliedCellDirty(state, &target);
                }
                cache->valid = true;
                cache->lastRaw28 = health->lastRaw28[s0][d0];
                cache->lastFreqHz = health->lastFreqHz[s0][d0];
                cache->lastGoodTimestampUs = nowUs;
                cache->consecutiveErrors = 0u;
                cache->consecutiveNoUnread = 0u;
                cache->consecutiveZeroRaw = 0u;
                cache->consecutiveWatchdogFaults = 0u;
                cache->consecutiveI2cErrors = 0u;

                profile->valid = true;
                profile->lastRaw28 = health->lastRaw28[s0][d0];
                profile->lastFrequencyHz = health->lastFreqHz[s0][d0];
                profile->lastValidTimestampUs = (uint64_t)nowUs;
                profile->consecutiveInvalid = 0u;
                profile->consecutiveWatchdogFault = 0u;
                profile->consecutiveSaturated = 0u;
                profile->consecutiveZeroRaw = 0u;

                sensorarrayFdcCellCalibration_t *cal = sensorarrayFdcSweepGetCellCalibration(s, d);
                if (cal) {
                    cal->hasLastGood = true;
                    cal->lockValid = true;
                    cal->lastGoodDriveCurrent = cache->driveCurrent ? cache->driveCurrent : SENSORARRAY_FDC_DRIVE_CURRENT;
                    cal->lastGoodDeglitch = cache->deglitchCode ?
                        (Fdc2214CapDeglitch_t)cache->deglitchCode :
                        FDC2214_DEGLITCH_10MHZ;
                    cal->lastGoodHighCurrent = false;
                    cal->lastGoodRaw28 = cache->lastRaw28;
                    cal->lastGoodFreqHz = cache->lastFreqHz;
                    cal->lastGoodTimestampUs = (uint64_t)nowUs;
                    cal->consecutiveFailCount = 0u;
                    cal->consecutiveNoUnreadCount = 0u;
                    cal->consecutiveStatusFaultCount = 0u;
                    cal->consecutiveZeroRawCount = 0u;
                    cal->directFailCount = 0u;
                    cal->lastFailReason = "valid";
                }
            } else if (severeError) {
                if (cache->consecutiveErrors < UINT16_MAX) {
                    cache->consecutiveErrors++;
                }
                if (invalid &&
                    !health->i2cErrorSeen[s0][d0] &&
                    !health->watchdogSeen[s0][d0] &&
                    !health->zeroRawSeen[s0][d0] &&
                    !health->saturatedSeen[s0][d0] &&
                    cache->consecutiveNoUnread < UINT16_MAX) {
                    cache->consecutiveNoUnread++;
                }
                if (health->zeroRawSeen[s0][d0] && cache->consecutiveZeroRaw < UINT16_MAX) {
                    cache->consecutiveZeroRaw++;
                }
                if (health->watchdogSeen[s0][d0] && cache->consecutiveWatchdogFaults < UINT16_MAX) {
                    cache->consecutiveWatchdogFaults++;
                }
                if (health->i2cErrorSeen[s0][d0] && cache->consecutiveI2cErrors < UINT16_MAX) {
                    cache->consecutiveI2cErrors++;
                }
                if (profile->consecutiveInvalid < UINT32_MAX) {
                    profile->consecutiveInvalid++;
                }
                if (health->watchdogSeen[s0][d0] && profile->consecutiveWatchdogFault < UINT32_MAX) {
                    profile->consecutiveWatchdogFault++;
                }
                if (health->saturatedSeen[s0][d0] && profile->consecutiveSaturated < UINT32_MAX) {
                    profile->consecutiveSaturated++;
                }
                if (health->zeroRawSeen[s0][d0] && profile->consecutiveZeroRaw < UINT32_MAX) {
                    profile->consecutiveZeroRaw++;
                }
            }

            if (amplitude) {
                if (cache->consecutiveAmplitudeWarnings < UINT16_MAX) {
                    cache->consecutiveAmplitudeWarnings++;
                }
                cache->lastWarningTimestampUs = nowUs;
                snprintf(cache->lastWarningReason, sizeof(cache->lastWarningReason), "%s", "amplitude_warning");
                if (profile->consecutiveAmplitudeFault < UINT32_MAX) {
                    profile->consecutiveAmplitudeFault++;
                }
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
                printf("FDC_WARN,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=amplitude_warning,consecutive=%u\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       (unsigned)cache->consecutiveAmplitudeWarnings);
#endif
            } else {
                cache->consecutiveAmplitudeWarnings = 0u;
                profile->consecutiveAmplitudeFault = 0u;
            }

            if (amplitude &&
                cache->valid &&
                CONFIG_SENSORARRAY_FDC_REAPPLY_CACHE_ON_WARNING &&
                cache->consecutiveAmplitudeWarnings >= amplitudeThreshold) {
                if (!cache->reapplyPending) {
                    cache->reapplyPending = true;
                    sensorarrayMeasureMarkFdcAppliedCellDirty(state, &target);
                    printf("FDC_CACHE,stage=reapply_pending,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=amplitude_warning,cacheValid=1,consecutive=%u\n",
                           (unsigned)s,
                           (unsigned)d,
                           (unsigned)matrixIndex,
                           sensorarrayMeasureFdcDeviceName(target.devId),
                           (unsigned)target.fdcChannel,
                           (unsigned)cache->consecutiveAmplitudeWarnings);
                }
            } else if (amplitude) {
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
                printf("FDC_RESCUE_SUPPRESSED,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=amplitude_warning,consecutive=%u,threshold=%lu,policy=warning_only\n",
                       (unsigned)s,
                       (unsigned)d,
                       (unsigned)matrixIndex,
                       sensorarrayMeasureFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       (unsigned)cache->consecutiveAmplitudeWarnings,
                       (unsigned long)amplitudeThreshold);
#endif
            }

            const char *rescueReason =
                (severeError && cache->consecutiveI2cErrors >= threshold) ? "i2c_error_consecutive" :
                (severeError && cache->consecutiveWatchdogFaults >= threshold) ? "watchdog_fault_consecutive" :
                (severeError && cache->consecutiveZeroRaw >= threshold) ? "zero_raw_consecutive" :
                (severeError && cache->consecutiveNoUnread >= threshold) ? "no_unread_consecutive" :
                (severeError && cache->consecutiveErrors >= threshold) ? "invalid_streak" :
                NULL;
            if (rescueReason) {
                (void)sensorarrayMeasureRequestFdcCellRescue(state, matrixIndex, rescueReason);
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

static uint64_t sensorarrayMeasureComputeFdcFrameCapTotalPf(sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return 0u;
    }

    int64_t startUs = esp_timer_get_time();
    frame->capValidMask = 0u;
    const double inductorUh = (double)CONFIG_SENSORARRAY_FDC_TANK_INDUCTOR_NH / 1000.0;
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        frame->capTotalPf[i] = 0.0;
        if ((frame->validMask & (1ULL << i)) == 0u) {
            continue;
        }
        double capPf = 0.0;
        if (sensorarrayMeasureFdcComputeCapacitancePf(frame->freqHz[i], inductorUh, &capPf)) {
            frame->capTotalPf[i] = capPf;
            frame->capValidMask |= (1ULL << i);
        }
    }
    return sensorarrayMeasureElapsedUs(startUs);
}

static void sensorarrayMeasureMarkFdcMatrixCellEx(sensorarrayFdcMatrixFrame_t *frame,
                                                  uint8_t sIndex,
                                                  uint8_t dIndex,
                                                  uint32_t raw28,
                                                  double freqHz,
                                                  const sensorarrayFdcRuntimeChannelConfig_t *config,
                                                  bool valid,
                                                  bool warning,
                                                  bool error)
{
    if (!frame || !sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        return;
    }

    size_t index = sensorarrayMatrixIndex(sIndex, dIndex);
    uint64_t bit = 1ULL << index;
    frame->raw28[index] = raw28;
    frame->freqHz[index] = freqHz;
    if (config) {
        frame->clockDividers[index] = config->clockDividers;
        frame->driveCurrent[index] = config->driveCurrent;
        frame->deglitchCode[index] = config->deglitchCode;
        frame->effectiveFclkHz[index] = config->effectiveFclkHz;
    }
    if (valid) {
        frame->validMask |= bit;
    } else {
        frame->validMask &= ~bit;
    }
    if (warning) {
        frame->warnMask |= bit;
    } else {
        frame->warnMask &= ~bit;
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

static uint64_t sensorarrayMeasureElapsedUs(int64_t startUs)
{
    int64_t elapsedUs = esp_timer_get_time() - startUs;
    return (elapsedUs > 0) ? (uint64_t)elapsedUs : 0u;
}

static uint64_t sensorarrayMeasureEstimateI2cBits(uint32_t writeCount,
                                                  uint32_t readCount,
                                                  uint32_t writeBytes,
                                                  uint32_t readBytes)
{
    /*
     * Approximate START/address/ACK/STOP cost. This is intentionally an estimate
     * for profiling deltas, not a replacement for logic-analyzer timing.
     */
    uint64_t dataBytes = (uint64_t)writeBytes + (uint64_t)readBytes;
    uint64_t dataBitsWithAck = dataBytes * 9ull;
    uint64_t addressBitsWithAck = ((uint64_t)writeCount + ((uint64_t)readCount * 2ull)) * 9ull;
    uint64_t framingBits = ((uint64_t)writeCount + (uint64_t)readCount) * 2ull;
    return dataBitsWithAck + addressBitsWithAck + framingBits;
}

static void sensorarrayMeasureMergeFdcI2cStats(const Fdc2214CapI2cStats_t *primaryStats,
                                               const Fdc2214CapI2cStats_t *secondaryStats,
                                               const BoardSupportI2cBusInfo_t *primaryBus,
                                               const BoardSupportI2cBusInfo_t *secondaryBus,
                                               sensorarrayFdcTimingSummary_t *summary)
{
    if (!summary) {
        return;
    }

    Fdc2214CapI2cStats_t primary = primaryStats ? *primaryStats : (Fdc2214CapI2cStats_t){0};
    Fdc2214CapI2cStats_t secondary = secondaryStats ? *secondaryStats : (Fdc2214CapI2cStats_t){0};

    summary->i2cWriteCount = primary.writeCount + secondary.writeCount;
    summary->i2cReadCount = primary.readCount + secondary.readCount;
    summary->i2cVerifyReadCount = primary.verifyReadCount + secondary.verifyReadCount;
    summary->i2cRetryCount = primary.retryCount + secondary.retryCount;
    summary->i2cNackCount = primary.nackCount + secondary.nackCount;
    summary->i2cTimeoutCount = primary.timeoutCount + secondary.timeoutCount;
    summary->i2cRecoveryCount = primary.recoveryCount + secondary.recoveryCount;

    summary->i2cBus0WriteCount = primary.writeCount;
    summary->i2cBus0ReadCount = primary.readCount;
    summary->i2cBus0WriteBytes = primary.writeBytes;
    summary->i2cBus0ReadBytes = primary.readBytes;
    summary->i2cBus0TotalUs = primary.totalUs;
    summary->i2cBus0RetryCount = primary.retryCount;
    summary->i2cBus0NackCount = primary.nackCount;
    summary->i2cBus0TimeoutCount = primary.timeoutCount;

    summary->i2cBus1WriteCount = secondary.writeCount;
    summary->i2cBus1ReadCount = secondary.readCount;
    summary->i2cBus1WriteBytes = secondary.writeBytes;
    summary->i2cBus1ReadBytes = secondary.readBytes;
    summary->i2cBus1TotalUs = secondary.totalUs;
    summary->i2cBus1RetryCount = secondary.retryCount;
    summary->i2cBus1NackCount = secondary.nackCount;
    summary->i2cBus1TimeoutCount = secondary.timeoutCount;

    uint32_t primaryFreq = (primaryBus && primaryBus->FrequencyHz != 0u) ?
        primaryBus->FrequencyHz :
        (uint32_t)CONFIG_BOARD_I2C_FREQ_HZ;
    uint32_t secondaryFreq = (secondaryBus && secondaryBus->FrequencyHz != 0u) ?
        secondaryBus->FrequencyHz :
        primaryFreq;
    summary->i2cFreqHz = (primaryFreq == secondaryFreq) ? primaryFreq : 0u;

    uint64_t primaryBits = sensorarrayMeasureEstimateI2cBits(primary.writeCount,
                                                             primary.readCount,
                                                             primary.writeBytes,
                                                             primary.readBytes);
    uint64_t secondaryBits = sensorarrayMeasureEstimateI2cBits(secondary.writeCount,
                                                               secondary.readCount,
                                                               secondary.writeBytes,
                                                               secondary.readBytes);
    summary->i2cEstimatedBits = primaryBits + secondaryBits;
    uint64_t primaryEstimatedUs = primaryFreq ? ((primaryBits * 1000000ull) / primaryFreq) : 0ull;
    uint64_t secondaryEstimatedUs = secondaryFreq ? ((secondaryBits * 1000000ull) / secondaryFreq) : 0ull;
    summary->i2cEstimatedBusUs = primaryEstimatedUs + secondaryEstimatedUs;
    summary->i2cMeasuredUs = primary.totalUs + secondary.totalUs;
    summary->i2cOverheadUs = (int64_t)summary->i2cMeasuredUs - (int64_t)summary->i2cEstimatedBusUs;
}

static void sensorarrayMeasurePrintFdcTimingSummary(const sensorarrayFdcTimingSummary_t *summary,
                                                    uint32_t sequence)
{
    if (!summary) {
        return;
    }

    uint64_t targetFrameUs = SENSORARRAY_FDC_TARGET_FRAME_US;
    uint64_t overrunUs = (summary->frameUs > targetFrameUs) ? (summary->frameUs - targetFrameUs) : 0u;
    uint64_t fpsX100 = summary->frameUs ? (100000000ull / summary->frameUs) : 0ull;
    uint64_t budgetUsePct = targetFrameUs ? ((summary->frameUs * 100ull) / targetFrameUs) : 0ull;

    printf("SCAN_TIMING_SUMMARY,seq=%lu,targetFps=%lu,targetFrameUs=%llu,frameUs=%llu,fps=%llu.%02llu,budgetUsePct=%llu,overrun=%u,overrunUs=%llu,rowAvgUs=%llu,rowMaxUs=%llu,rowMinUs=%llu,slowRow=%u,pathEnsureUs=%llu,cacheApplyUs=%llu,applyBuildConfigUs=%llu,applyChannelConfigWriteUs=%llu,applyGlobalConfigWriteUs=%llu,applyVerifyUs=%llu,applyDelayUs=%llu,applyReadyWaitUs=%llu,applyMutexWaitUs=%llu,applyLogUs=%llu,discardUs=%llu,waitReadyUs=%llu,readUs=%llu,emitUs=%llu,capComputeUs=%llu,sweepUs=%llu,runtimeSweepCount=%lu,i2cWriteCount=%lu,i2cReadCount=%lu,i2cVerifyReadCount=%lu,i2cRetryCount=%lu,i2cNackCount=%lu,i2cTimeoutCount=%lu,i2cRecoveryCount=%lu,i2cFreqHz=%lu,i2cEstimatedBits=%llu,i2cEstimatedBusUs=%llu,i2cMeasuredUs=%llu,i2cOverheadUs=%lld,i2cBus0WriteCount=%lu,i2cBus0ReadCount=%lu,i2cBus0WriteBytes=%lu,i2cBus0ReadBytes=%lu,i2cBus0TotalUs=%llu,i2cBus0RetryCount=%lu,i2cBus0NackCount=%lu,i2cBus0TimeoutCount=%lu,i2cBus1WriteCount=%lu,i2cBus1ReadCount=%lu,i2cBus1WriteBytes=%lu,i2cBus1ReadBytes=%lu,i2cBus1TotalUs=%llu,i2cBus1RetryCount=%lu,i2cBus1NackCount=%lu,i2cBus1TimeoutCount=%lu\n",
           (unsigned long)sequence,
           (unsigned long)CONFIG_SENSORARRAY_FDC_MATRIX_TARGET_FPS,
           (unsigned long long)targetFrameUs,
           (unsigned long long)summary->frameUs,
           (unsigned long long)(fpsX100 / 100ull),
           (unsigned long long)(fpsX100 % 100ull),
           (unsigned long long)budgetUsePct,
           overrunUs ? 1u : 0u,
           (unsigned long long)overrunUs,
           (unsigned long long)summary->rowAvgUs,
           (unsigned long long)summary->rowMaxUs,
           (unsigned long long)summary->rowMinUs,
           (unsigned)summary->slowRow,
           (unsigned long long)summary->pathEnsureUs,
           (unsigned long long)summary->cacheApplyUs,
           (unsigned long long)summary->applyBuildConfigUs,
           (unsigned long long)summary->applyChannelConfigWriteUs,
           (unsigned long long)summary->applyGlobalConfigWriteUs,
           (unsigned long long)summary->applyVerifyUs,
           (unsigned long long)summary->applyDelayUs,
           (unsigned long long)summary->applyReadyWaitUs,
           (unsigned long long)summary->applyMutexWaitUs,
           (unsigned long long)summary->applyLogUs,
           (unsigned long long)summary->discardUs,
           (unsigned long long)summary->waitReadyUs,
           (unsigned long long)summary->readUs,
           (unsigned long long)summary->emitUs,
           (unsigned long long)summary->capComputeUs,
           (unsigned long long)summary->sweepUs,
           (unsigned long)summary->runtimeSweepCount,
           (unsigned long)summary->i2cWriteCount,
           (unsigned long)summary->i2cReadCount,
           (unsigned long)summary->i2cVerifyReadCount,
           (unsigned long)summary->i2cRetryCount,
           (unsigned long)summary->i2cNackCount,
           (unsigned long)summary->i2cTimeoutCount,
           (unsigned long)summary->i2cRecoveryCount,
           (unsigned long)summary->i2cFreqHz,
           (unsigned long long)summary->i2cEstimatedBits,
           (unsigned long long)summary->i2cEstimatedBusUs,
           (unsigned long long)summary->i2cMeasuredUs,
           (long long)summary->i2cOverheadUs,
           (unsigned long)summary->i2cBus0WriteCount,
           (unsigned long)summary->i2cBus0ReadCount,
           (unsigned long)summary->i2cBus0WriteBytes,
           (unsigned long)summary->i2cBus0ReadBytes,
           (unsigned long long)summary->i2cBus0TotalUs,
           (unsigned long)summary->i2cBus0RetryCount,
           (unsigned long)summary->i2cBus0NackCount,
           (unsigned long)summary->i2cBus0TimeoutCount,
           (unsigned long)summary->i2cBus1WriteCount,
           (unsigned long)summary->i2cBus1ReadCount,
           (unsigned long)summary->i2cBus1WriteBytes,
           (unsigned long)summary->i2cBus1ReadBytes,
           (unsigned long long)summary->i2cBus1TotalUs,
           (unsigned long)summary->i2cBus1RetryCount,
           (unsigned long)summary->i2cBus1NackCount,
           (unsigned long)summary->i2cBus1TimeoutCount);
}

static void sensorarrayMeasurePrintFdcRowTiming(uint32_t sequence,
                                                const sensorarrayFdcRowTiming_t *rowTiming)
{
    if (!rowTiming) {
        return;
    }

    uint64_t rowOverrunUs = (rowTiming->rowUs > SENSORARRAY_FDC_TARGET_ROW_US) ?
        (rowTiming->rowUs - SENSORARRAY_FDC_TARGET_ROW_US) :
        0u;
    uint64_t serialEquivalentUs = rowTiming->primaryTotalUs + rowTiming->secondaryTotalUs;
    uint64_t parallelActualUs = (rowTiming->primaryTotalUs > rowTiming->secondaryTotalUs) ?
        rowTiming->primaryTotalUs :
        rowTiming->secondaryTotalUs;
    parallelActualUs += rowTiming->parallelJoinWaitUs;
    uint64_t parallelEfficiencyPct = parallelActualUs ?
        ((serialEquivalentUs * 100ull) / parallelActualUs) :
        0u;

    printf("SCAN_ROW_TIMING,seq=%lu,row=%u,targetRowUs=%lu,rowUs=%llu,rowOverrun=%u,rowSelectUs=%llu,analogSettleUs=%llu,primaryTotalUs=%llu,secondaryTotalUs=%llu,parallelJoinWaitUs=%llu,discardUs=%llu,waitReadyUs=%llu,readUs=%llu,rowValidMask=0x%02X,rowWarnMask=0x%02X,rowErrorMask=0x%02X,parallelEnabled=0,serialEquivalentUs=%llu,parallelActualUs=%llu,parallelEfficiencyPct=%llu\n",
           (unsigned long)sequence,
           (unsigned)rowTiming->row,
           (unsigned long)SENSORARRAY_FDC_TARGET_ROW_US,
           (unsigned long long)rowTiming->rowUs,
           rowOverrunUs ? 1u : 0u,
           (unsigned long long)rowTiming->rowSelectUs,
           (unsigned long long)rowTiming->analogSettleUs,
           (unsigned long long)rowTiming->primaryTotalUs,
           (unsigned long long)rowTiming->secondaryTotalUs,
           (unsigned long long)rowTiming->parallelJoinWaitUs,
           (unsigned long long)rowTiming->discardUs,
           (unsigned long long)rowTiming->waitReadyUs,
           (unsigned long long)rowTiming->readUs,
           (unsigned)rowTiming->rowValidMask,
           (unsigned)rowTiming->rowWarnMask,
           (unsigned)rowTiming->rowErrorMask,
           (unsigned long long)serialEquivalentUs,
           (unsigned long long)parallelActualUs,
           (unsigned long long)parallelEfficiencyPct);
}

static void sensorarrayMeasurePrintFdcDeviceTiming(uint32_t sequence,
                                                   const sensorarrayFdcDeviceTiming_t *deviceTiming)
{
    if (!deviceTiming) {
        return;
    }

    printf("SCAN_DEVICE_TIMING,seq=%lu,row=%u,device=%s,deviceUs=%llu,applyUs=%llu,applyBuildConfigUs=%llu,channelConfigWriteUs=%llu,globalConfigWriteUs=%llu,verifyUs=%llu,discardUs=%llu,waitReadyUs=%llu,readRawUs=%llu,readyPollCount=%lu,regWriteCount=%lu,regReadCount=%lu,verifyReadCount=%lu,retryCount=%lu,nackCount=%lu,timeoutCount=%lu\n",
           (unsigned long)sequence,
           (unsigned)deviceTiming->row,
           sensorarrayMeasureFdcDeviceName(deviceTiming->deviceId),
           (unsigned long long)deviceTiming->deviceUs,
           (unsigned long long)deviceTiming->applyUs,
           (unsigned long long)deviceTiming->applyBuildConfigUs,
           (unsigned long long)deviceTiming->channelConfigWriteUs,
           (unsigned long long)deviceTiming->globalConfigWriteUs,
           (unsigned long long)deviceTiming->verifyUs,
           (unsigned long long)deviceTiming->discardUs,
           (unsigned long long)deviceTiming->waitReadyUs,
           (unsigned long long)deviceTiming->readRawUs,
           (unsigned long)deviceTiming->readyPollCount,
           (unsigned long)deviceTiming->regWriteCount,
           (unsigned long)deviceTiming->regReadCount,
           (unsigned long)deviceTiming->verifyReadCount,
           (unsigned long)deviceTiming->retryCount,
           (unsigned long)deviceTiming->nackCount,
           (unsigned long)deviceTiming->timeoutCount);
}

static void sensorarrayMeasureFillFdcDeviceI2cDelta(const Fdc2214CapI2cStats_t *before,
                                                    const Fdc2214CapI2cStats_t *after,
                                                    sensorarrayFdcDeviceTiming_t *timing)
{
    if (!before || !after || !timing) {
        return;
    }

    timing->regWriteCount = after->writeCount - before->writeCount;
    timing->regReadCount = after->readCount - before->readCount;
    timing->verifyReadCount = after->verifyReadCount - before->verifyReadCount;
    timing->retryCount = after->retryCount - before->retryCount;
    timing->nackCount = after->nackCount - before->nackCount;
    timing->timeoutCount = after->timeoutCount - before->timeoutCount;
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

    int64_t frameStartUs = esp_timer_get_time();
    sensorarrayFdcTimingSummary_t timing = {
        .rowMinUs = UINT64_MAX,
    };
    uint64_t rowTotalUs = 0u;

    esp_err_t firstErr = sensorarrayMeasureCheckFdcMatrixReady(state);
    if (firstErr != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        printf("MATRIXFDC_DIAG,stage=read_abort,reason=matrix_not_ready,err=0x%lx\n",
               (unsigned long)firstErr);
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    Fdc2214CapResetI2cStats(state->fdcPrimary.handle);
    Fdc2214CapResetI2cStats(state->fdcSecondary.handle);
    BoardSupportI2cBusInfo_t primaryBus = {0};
    BoardSupportI2cBusInfo_t secondaryBus = {0};
    (void)boardSupportGetI2cBusInfo(false, &primaryBus);
    (void)boardSupportGetI2cBusInfo(true, &secondaryBus);

    int64_t stageStartUs = esp_timer_get_time();
    firstErr = sensorarrayMeasureEnsureFdcMatrixPath(state, "fdc_matrix_frame");
    timing.pathEnsureUs += sensorarrayMeasureElapsedUs(stageStartUs);
    if (firstErr != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        printf("MATRIXFDC_DIAG,stage=read_abort,reason=path_prepare_failed,err=0x%lx\n",
               (unsigned long)firstErr);
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    sensorarrayFdcFrameHealth_t frameHealth = {0};
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        sensorarrayFdcRowTiming_t rowTiming = {
            .row = s,
        };
        sensorarrayFdcDeviceTiming_t primaryTiming = {
            .row = s,
            .deviceId = SENSORARRAY_FDC_DEV_PRIMARY,
        };
        sensorarrayFdcDeviceTiming_t secondaryTiming = {
            .row = s,
            .deviceId = SENSORARRAY_FDC_DEV_SECONDARY,
        };
        Fdc2214CapI2cStats_t primaryStatsBefore = {0};
        Fdc2214CapI2cStats_t secondaryStatsBefore = {0};
        Fdc2214CapGetI2cStats(state->fdcPrimary.handle, &primaryStatsBefore);
        Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStatsBefore);

        int64_t rowStartUs = esp_timer_get_time();
        int64_t rowSelectStartUs = esp_timer_get_time();
        err = tmuxSwitchSelectRow((uint8_t)(s - 1u));
        rowTiming.rowSelectUs = sensorarrayMeasureElapsedUs(rowSelectStartUs);
        int64_t analogSettleStartUs = esp_timer_get_time();
        sensorarrayMeasureDelayUs((uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US);
        rowTiming.analogSettleUs = sensorarrayMeasureElapsedUs(analogSettleStartUs);
#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
        tmuxSwitchControlState_t ctrl = {0};
        sensorarrayMeasureReadFdcPathControl(&ctrl);
        printf("FDC_ROW,stage=select,row=%u,selaCmd=%d,selaReadback=%d,err=0x%lx\n",
               (unsigned)s,
               ctrl.cmdSelaLevel,
               ctrl.obsSelaLevel,
               (unsigned long)err);
#endif
        if (err != ESP_OK && firstErr == ESP_OK) {
            firstErr = err;
        }

        stageStartUs = esp_timer_get_time();
        esp_err_t primaryErr = sensorarrayMeasureApplyFdcCachedRowConfig(state,
                                                                         s,
                                                                         SENSORARRAY_FDC_DEV_PRIMARY,
                                                                         "matrix_row",
                                                                         false,
                                                                         &primaryTiming);
        if (primaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = primaryErr;
        }
        esp_err_t secondaryErr = sensorarrayMeasureApplyFdcCachedRowConfig(state,
                                                                           s,
                                                                           SENSORARRAY_FDC_DEV_SECONDARY,
                                                                           "matrix_row",
                                                                           false,
                                                                           &secondaryTiming);
        if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = secondaryErr;
        }
        sensorarrayFdcRuntimeChannelConfig_t runtimeConfigs[2][4] = {0};
        sensorarrayMeasureRuntimeConfigsFromApplied(state,
                                                    SENSORARRAY_FDC_DEV_PRIMARY,
                                                    runtimeConfigs[SENSORARRAY_FDC_DEV_PRIMARY]);
        sensorarrayMeasureRuntimeConfigsFromApplied(state,
                                                    SENSORARRAY_FDC_DEV_SECONDARY,
                                                    runtimeConfigs[SENSORARRAY_FDC_DEV_SECONDARY]);
        uint64_t applyElapsedUs = sensorarrayMeasureElapsedUs(stageStartUs);
        timing.cacheApplyUs += applyElapsedUs;
        timing.applyBuildConfigUs += primaryTiming.applyBuildConfigUs + secondaryTiming.applyBuildConfigUs;
        timing.applyChannelConfigWriteUs += primaryTiming.channelConfigWriteUs + secondaryTiming.channelConfigWriteUs;
        timing.applyGlobalConfigWriteUs += primaryTiming.globalConfigWriteUs + secondaryTiming.globalConfigWriteUs;
        timing.applyVerifyUs += primaryTiming.verifyUs + secondaryTiming.verifyUs;

        uint8_t discardGroups = sensorarrayMeasureFdcDiscardFrames();
        if (primaryErr == ESP_OK) {
            stageStartUs = esp_timer_get_time();
            esp_err_t discardErr = sensorarrayMeasureDiscardFdcAutoscanRow(state,
                                                                           s,
                                                                           SENSORARRAY_FDC_DEV_PRIMARY,
                                                                           discardGroups,
                                                                           "row_switch");
            primaryTiming.discardUs += sensorarrayMeasureElapsedUs(stageStartUs);
            timing.discardUs += primaryTiming.discardUs;
            rowTiming.discardUs += primaryTiming.discardUs;
            if (discardErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = discardErr;
            }
        }
        if (secondaryErr == ESP_OK) {
            stageStartUs = esp_timer_get_time();
            esp_err_t discardErr = sensorarrayMeasureDiscardFdcAutoscanRow(state,
                                                                           s,
                                                                           SENSORARRAY_FDC_DEV_SECONDARY,
                                                                           discardGroups,
                                                                           "row_switch");
            secondaryTiming.discardUs += sensorarrayMeasureElapsedUs(stageStartUs);
            timing.discardUs += secondaryTiming.discardUs;
            rowTiming.discardUs += secondaryTiming.discardUs;
            if (discardErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = discardErr;
            }
        }

        sensorarrayFdcReadyState_t primaryReady = {0};
        sensorarrayFdcReadyState_t secondaryReady = {0};
        if (primaryErr == ESP_OK && secondaryErr == ESP_OK) {
            stageStartUs = esp_timer_get_time();
            esp_err_t readyErr = sensorarrayMeasureWaitBothFdcAutoscanFrameReady(state,
                                                                                 s,
                                                                                 (uint32_t)SENSORARRAY_FDC_AUTOSCAN_READY_TIMEOUT_MS,
                                                                                 &primaryReady,
                                                                                 &secondaryReady);
            uint64_t readyWaitUs = sensorarrayMeasureElapsedUs(stageStartUs);
            timing.waitReadyUs += readyWaitUs;
            rowTiming.waitReadyUs += readyWaitUs;
            primaryTiming.waitReadyUs += readyWaitUs;
            secondaryTiming.waitReadyUs += readyWaitUs;
            primaryTiming.readyPollCount += primaryReady.pollCount;
            secondaryTiming.readyPollCount += secondaryReady.pollCount;
            primaryTiming.timeoutCount += primaryReady.timeoutCount;
            secondaryTiming.timeoutCount += secondaryReady.timeoutCount;
            if (readyErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = readyErr;
            }
            if (!primaryReady.ready) {
                primaryErr = (primaryReady.err == ESP_OK) ? ESP_ERR_TIMEOUT : primaryReady.err;
            }
            if (!secondaryReady.ready) {
                secondaryErr = (secondaryReady.err == ESP_OK) ? ESP_ERR_TIMEOUT : secondaryReady.err;
            }
        }

        sensorarrayFdcAutoscanSamples_t primarySamples = {0};
        sensorarrayFdcAutoscanSamples_t secondarySamples = {0};
        if (primaryErr == ESP_OK) {
            stageStartUs = esp_timer_get_time();
            primaryErr = sensorarrayMeasureReadFdcAutoscan4ch(&state->fdcPrimary, &primarySamples);
            primaryTiming.readRawUs += sensorarrayMeasureElapsedUs(stageStartUs);
            timing.readUs += primaryTiming.readRawUs;
            rowTiming.readUs += primaryTiming.readRawUs;
            if (primaryErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = primaryErr;
            }
        } else {
            primarySamples.statusRaw = primaryReady.statusRaw;
        }
        if (secondaryErr == ESP_OK) {
            stageStartUs = esp_timer_get_time();
            secondaryErr = sensorarrayMeasureReadFdcAutoscan4ch(&state->fdcSecondary, &secondarySamples);
            secondaryTiming.readRawUs += sensorarrayMeasureElapsedUs(stageStartUs);
            timing.readUs += secondaryTiming.readRawUs;
            rowTiming.readUs += secondaryTiming.readRawUs;
            if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
                firstErr = secondaryErr;
            }
        } else {
            secondarySamples.statusRaw = secondaryReady.statusRaw;
        }

        uint8_t rowValidMask8 = 0u;
        uint8_t rowWarnMask8 = 0u;
        uint8_t rowErrorMask8 = 0u;
        sensorarrayMeasureFillFdcMatrixRow(outFrame,
                                           s,
                                           &primarySamples,
                                           &secondarySamples,
                                           runtimeConfigs,
                                           &rowValidMask8,
                                           &rowWarnMask8,
                                           &rowErrorMask8);
        sensorarrayMeasureAccumulateFdcHealth(&frameHealth,
                                              s,
                                              &primarySamples,
                                              &secondarySamples,
                                              runtimeConfigs,
                                              outFrame);

#if CONFIG_SENSORARRAY_FDC_VERBOSE_SCAN_LOG
        printf("FDC_MATRIX_ROW,row=%u,d1=%lu,d2=%lu,d3=%lu,d4=%lu,d5=%lu,d6=%lu,d7=%lu,d8=%lu,validMask8=0x%02X,warnMask8=0x%02X,errorMask8=0x%02X\n",
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
               (unsigned)rowWarnMask8,
               (unsigned)rowErrorMask8);
#endif
        rowTiming.rowValidMask = rowValidMask8;
        rowTiming.rowWarnMask = rowWarnMask8;
        rowTiming.rowErrorMask = rowErrorMask8;
        rowTiming.rowUs = sensorarrayMeasureElapsedUs(rowStartUs);
        rowTotalUs += rowTiming.rowUs;
        if (rowTiming.rowUs > timing.rowMaxUs) {
            timing.rowMaxUs = rowTiming.rowUs;
            timing.slowRow = s;
        }
        if (rowTiming.rowUs < timing.rowMinUs) {
            timing.rowMinUs = rowTiming.rowUs;
        }
        primaryTiming.deviceUs = primaryTiming.applyUs +
                                 primaryTiming.discardUs +
                                 primaryTiming.waitReadyUs +
                                 primaryTiming.readRawUs;
        secondaryTiming.deviceUs = secondaryTiming.applyUs +
                                   secondaryTiming.discardUs +
                                   secondaryTiming.waitReadyUs +
                                   secondaryTiming.readRawUs;
        rowTiming.primaryTotalUs = primaryTiming.deviceUs;
        rowTiming.secondaryTotalUs = secondaryTiming.deviceUs;

        Fdc2214CapI2cStats_t primaryStatsAfter = {0};
        Fdc2214CapI2cStats_t secondaryStatsAfter = {0};
        Fdc2214CapGetI2cStats(state->fdcPrimary.handle, &primaryStatsAfter);
        Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStatsAfter);
        sensorarrayMeasureFillFdcDeviceI2cDelta(&primaryStatsBefore, &primaryStatsAfter, &primaryTiming);
        sensorarrayMeasureFillFdcDeviceI2cDelta(&secondaryStatsBefore, &secondaryStatsAfter, &secondaryTiming);

        if (s_fdcProfileRowEnabled) {
            sensorarrayMeasurePrintFdcRowTiming(outFrame->sequence, &rowTiming);
        }
        if (s_fdcProfileDeviceEnabled) {
            sensorarrayMeasurePrintFdcDeviceTiming(outFrame->sequence, &primaryTiming);
            sensorarrayMeasurePrintFdcDeviceTiming(outFrame->sequence, &secondaryTiming);
        }
        taskYIELD();
    }
    sensorarrayMeasureUpdateFdcRuntimeProfiles(state, &frameHealth);

    timing.capComputeUs = sensorarrayMeasureComputeFdcFrameCapTotalPf(outFrame);
    timing.frameUs = sensorarrayMeasureElapsedUs(frameStartUs);
    timing.rowAvgUs = rowTotalUs / SENSORARRAY_MATRIX_ROWS;
    if (timing.rowMinUs == UINT64_MAX) {
        timing.rowMinUs = 0u;
    }

    Fdc2214CapI2cStats_t primaryStats = {0};
    Fdc2214CapI2cStats_t secondaryStats = {0};
    Fdc2214CapGetI2cStats(state->fdcPrimary.handle, &primaryStats);
    Fdc2214CapGetI2cStats(state->fdcSecondary.handle, &secondaryStats);
    sensorarrayMeasureMergeFdcI2cStats(&primaryStats, &secondaryStats, &primaryBus, &secondaryBus, &timing);

    uint32_t timingEvery = s_fdcTimingSummaryEvery;
    if (s_fdcProfileSummaryEnabled && timingEvery != 0u && (outFrame->sequence % timingEvery) == 0u) {
        sensorarrayMeasurePrintFdcTimingSummary(&timing, outFrame->sequence);
    }

    if (Fdc2214CapI2cTraceIsEnabled() &&
        (timing.frameUs > SENSORARRAY_FDC_TARGET_FRAME_US ||
         timing.i2cRetryCount > 0u ||
         timing.i2cTimeoutCount > 0u ||
         timing.i2cNackCount > 0u ||
         outFrame->errorMask != 0u)) {
        Fdc2214CapI2cTraceDump();
    }

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

void sensorarrayMeasureFdcProfileSetSummary(bool enabled)
{
    s_fdcProfileSummaryEnabled = enabled;
}

void sensorarrayMeasureFdcProfileSetRow(bool enabled)
{
    s_fdcProfileRowEnabled = enabled;
}

void sensorarrayMeasureFdcProfileSetDevice(bool enabled)
{
    s_fdcProfileDeviceEnabled = enabled;
}

void sensorarrayMeasureFdcProfileSetSummaryEvery(uint32_t everyNFrames)
{
    s_fdcTimingSummaryEvery = everyNFrames;
}

bool sensorarrayMeasureFdcProfileSummaryEnabled(void)
{
    return s_fdcProfileSummaryEnabled;
}

bool sensorarrayMeasureFdcProfileRowEnabled(void)
{
    return s_fdcProfileRowEnabled;
}

bool sensorarrayMeasureFdcProfileDeviceEnabled(void)
{
    return s_fdcProfileDeviceEnabled;
}

uint32_t sensorarrayMeasureFdcProfileSummaryEvery(void)
{
    return s_fdcTimingSummaryEvery;
}

esp_err_t sensorarrayMeasureFdcSetDiscardFrames(uint8_t discardFrames)
{
    if (discardFrames > 8u) {
        return ESP_ERR_INVALID_ARG;
    }
    s_fdcDiscardFrames = discardFrames;
    return ESP_OK;
}

uint8_t sensorarrayMeasureFdcDiscardFrames(void)
{
    return s_fdcDiscardFrames;
}

static esp_err_t sensorarrayTransportSendFdcMatrixFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    (void)frame;
    return ESP_ERR_NOT_SUPPORTED;
}

static void sensorarrayFdcMatrixPrintFrame(const sensorarrayFdcMatrixFrame_t *frame, const char *tag)
{
#if CONFIG_SENSORARRAY_FDC_EMIT_CAP_TOTAL_PF
    printf("%s,seq=%lu,timestampUs=%llu,unit=freqHz+capTotalPf,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,capValidMask=0x%016llX,freqHz=[",
           tag ? tag : "MATRIXFDC",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask,
           (unsigned long long)frame->capValidMask);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.1f", (i == 0u) ? "" : ",", frame->freqHz[i]);
    }
    printf("],capTotalPf=[");
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.6f", (i == 0u) ? "" : ",", frame->capTotalPf[i]);
    }
    printf("]\n");
#else
    printf("%s,seq=%lu,timestampUs=%llu,unit=freqHz,validMask=0x%016llX,warnMask=0x%016llX,errorMask=0x%016llX,freqHz=[",
           tag ? tag : "MATRIXFDC",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs,
           (unsigned long long)frame->validMask,
           (unsigned long long)frame->warnMask,
           (unsigned long long)frame->errorMask);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%.1f", (i == 0u) ? "" : ",", frame->freqHz[i]);
    }
    printf("]\n");
#endif

#if CONFIG_SENSORARRAY_FDC_RAW_DEBUG_LOG
    printf("DEBUGFDC_RAW,seq=%lu,timestampUs=%llu,raw28=[",
           (unsigned long)frame->sequence,
           (unsigned long long)frame->timestampUs);
    for (size_t i = 0u; i < SENSORARRAY_MATRIX_CELL_COUNT; ++i) {
        printf("%s%lu", (i == 0u) ? "" : ",", (unsigned long)frame->raw28[i]);
    }
    printf("]\n");
#endif
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

esp_err_t sensorarrayMeasureApplyFdcCellRoute(sensorarrayState_t *state,
                                              const sensorarrayFdcCellTarget_t *target,
                                              const char *reason)
{
    if (!state || !target ||
        !sensorarrayMatrixIndexIsValid(target->sColumn, target->dLine) ||
        target->fdcChannel > (uint8_t)FDC2214_CH3) {
        return ESP_ERR_INVALID_ARG;
    }

    bool selBLevel = false;
    if (!sensorarrayBoardMapFdcSelBLevel(&selBLevel)) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *source = reason ? reason : SENSORARRAY_NA;
    printf("FDC_CELL_ROUTE,stage=begin,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           source);

    uint32_t settleMs = ((uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US + 999u) / 1000u;
    esp_err_t err = sensorarrayMeasureApplyRouteLevels(state,
                                                       target->sColumn,
                                                       target->dLine,
                                                       SENSORARRAY_ROUTE_PATH_CAPACITIVE,
                                                       TMUX1108_SOURCE_GND,
                                                       SENSORARRAY_SELA_ROUTE_FDC2214,
                                                       selBLevel,
                                                       settleMs,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                       SENSORARRAY_SETTLE_AFTER_SW_MS,
                                                       target->mapLabel ? target->mapLabel : "fdc_cell_route");
    if (err == ESP_OK) {
        uint8_t discardCount = (uint8_t)CONFIG_SENSORARRAY_FDC_MATRIX_DISCARD_SAMPLES;
        if (discardCount < SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT) {
            discardCount = SENSORARRAY_FDC_CELL_ROUTE_DISCARD_COUNT;
        }
        err = sensorarrayMeasureFdcDiscardStaleSamples(state, target, discardCount, "row_switch");
    }

    printf("FDC_CELL_ROUTE,stage=done,s=%u,d=%u,index=%u,device=%s,ch=%u,err=0x%lx,status=%s\n",
           (unsigned)target->sColumn,
           (unsigned)target->dLine,
           (unsigned)target->matrixIndex,
           sensorarrayMeasureFdcDeviceName(target->devId),
           (unsigned)target->fdcChannel,
           (unsigned long)err,
           (err == ESP_OK) ? "ok" : "failed");
    return err;
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
    bool watchdogFault = Fdc2214CapStatusHasWatchdogFault(&outDiag->status);
    bool amplitudeFault = Fdc2214CapStatusHasAmplitudeFault(&outDiag->status) ||
                          outDiag->sample.ErrAmplitude;
    bool readable = outDiag->unreadConversionPresent || outDiag->status.DataReady || outDiag->sample.DataReady;
    outDiag->statusCode = mappedStatus;
    outDiag->qualityDegraded = (!readable) ||
                               outDiag->sample.ErrWatchdog ||
                               outDiag->sample.ErrAmplitude ||
                               watchdogFault ||
                               amplitudeFault;
    outDiag->provisionalReadable = idOk &&
                                   configOk &&
                                   outDiag->sample.Converting &&
                                   (outDiag->sample.Raw28 != 0u) &&
                                   !outDiag->sample.ErrWatchdog &&
                                   !watchdogFault &&
                                   readable;
    outDiag->sampleValid = relaxedMode ? outDiag->provisionalReadable
                                       : (outDiag->provisionalReadable &&
                                          (mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID ||
                                           mappedStatus == SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT));
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

double sensorarrayMeasureFdcFinFactorFromClockDiv(uint16_t clockDividers)
{
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *status = "unknown";
    if (!sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                  &finSelCode,
                                                  &finFactor,
                                                  &frefDivider,
                                                  &status)) {
        return 0.0;
    }
    (void)finSelCode;
    (void)frefDivider;
    (void)status;
    return (double)finFactor;
}

double sensorarrayMeasureFdcFrefDividerFromClockDiv(uint16_t clockDividers)
{
    uint8_t finSelCode = 0u;
    uint8_t finFactor = 0u;
    uint16_t frefDivider = 0u;
    const char *status = "unknown";
    if (!sensorarrayMeasureFdcDecodeClockDividers(clockDividers,
                                                  &finSelCode,
                                                  &finFactor,
                                                  &frefDivider,
                                                  &status)) {
        return 0.0;
    }
    (void)finSelCode;
    (void)finFactor;
    (void)status;
    return (double)frefDivider;
}

double sensorarrayMeasureFdcRaw28ToFreqHz(uint32_t raw28,
                                          uint32_t effectiveFclkHz,
                                          uint16_t clockDividers)
{
    if (raw28 == 0u || effectiveFclkHz == 0u) {
        return 0.0;
    }

    double finFactor = sensorarrayMeasureFdcFinFactorFromClockDiv(clockDividers);
    double frefDivider = sensorarrayMeasureFdcFrefDividerFromClockDiv(clockDividers);
    if (finFactor <= 0.0 || frefDivider <= 0.0) {
        return 0.0;
    }

    double effectiveFrefHz = (double)effectiveFclkHz / frefDivider;
    return ((double)raw28 * effectiveFrefHz * finFactor) / SENSORARRAY_FDC_RAW_SCALE_2P28;
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
        sensorarrayMeasureFdcRaw28ToFreqHz(raw28, outDiag->effectiveFclkHz, clockDividers);
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

    const double inductorH = inductorValueUh * 1e-6;
    const double omega = 2.0 * SENSORARRAY_PI * frequencyHz;
    const double denom = omega * omega * inductorH;
    if (!isfinite(denom) || denom <= 0.0) {
        return false;
    }

    const double cPf = (1.0 / denom) * 1e12;
    if (!isfinite(cPf) || cPf <= 0.0) {
        return false;
    }

    *outCapPf = cPf;
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
