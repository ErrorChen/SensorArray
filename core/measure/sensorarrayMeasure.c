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
#include "sensorarrayLog.h"

#ifndef CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE 0
#endif

#if CONFIG_FDC2214CAP_LOW_LEVEL_I2C_TRACE
#define FDCLOW_TRACE(...) printf(__VA_ARGS__)
#else
#define FDCLOW_TRACE(...) do { } while (0)
#endif

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
#define SENSORARRAY_FDC_CHANNEL_MASK_ALL 0x0Fu

static SemaphoreHandle_t s_measureLock = NULL;
static portMUX_TYPE s_measureLockMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t s_fdcMatrixSequence = 0u;
static bool s_fastSpeedEnabled = false;

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

static esp_err_t sensorarrayMeasurePrepareFdcMatrixPath(sensorarrayState_t *state, const char *reason)
{
    (void)reason;
    if (!state || !state->tmuxReady || !state->adsReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sensorarrayMeasureStopAdsBeforeRoute(state);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureForceAdsReferenceOff(state);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetSelaPathQuiet(state,
                                             SENSORARRAY_SELA_ROUTE_FDC2214,
                                             (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetFdcSelBPathQuiet(state);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMeasureSetSwPhysicalLevel(state,
                                               SENSORARRAY_SW_PHYSICAL_HIGH,
                                               "fdc_matrix_path");
    if (err != ESP_OK) {
        return err;
    }

    return tmux1134SetEnLogicalState(true);
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

static void sensorarrayMeasureInitFdcMatrixFrame(sensorarrayFdcMatrixFrame_t *frame)
{
    memset(frame, 0, sizeof(*frame));
    frame->timestampUs = (uint64_t)esp_timer_get_time();
    frame->sequence = s_fdcMatrixSequence++;
}

static void sensorarrayMeasureMarkFdcMatrixCell(sensorarrayFdcMatrixFrame_t *frame,
                                                uint8_t sIndex,
                                                uint8_t dIndex,
                                                uint32_t raw28,
                                                bool valid)
{
    if (!frame || !sensorarrayMatrixIndexIsValid(sIndex, dIndex)) {
        return;
    }

    size_t index = sensorarrayMatrixIndex(sIndex, dIndex);
    uint64_t bit = 1ULL << index;
    frame->raw28[index] = valid ? raw28 : 0u;
    if (valid) {
        frame->validMask |= bit;
        frame->errorMask &= ~bit;
    } else {
        frame->validMask &= ~bit;
        frame->errorMask |= bit;
    }
}

static void sensorarrayMeasureMarkFdcMatrixRowError(sensorarrayFdcMatrixFrame_t *frame, uint8_t sIndex)
{
    for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
        sensorarrayMeasureMarkFdcMatrixCell(frame, sIndex, d, 0u, false);
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

static void sensorarrayMeasureApplyFdcDeviceSamples(sensorarrayState_t *state,
                                                    sensorarrayFdcMatrixFrame_t *frame,
                                                    uint8_t sIndex,
                                                    sensorarrayFdcDeviceId_t devId,
                                                    const Fdc2214CapChannelSample_t samples[4])
{
    for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
        const sensorarrayFdcDLineMap_t *map = sensorarrayBoardMapFindFdcByDLine(d);
        if (!map || map->devId != devId) {
            continue;
        }

        sensorarrayFdcDeviceState_t *fdcState = sensorarrayMeasureGetFdcState(state, map->devId);
        bool valid = fdcState && fdcState->ready && (uint8_t)map->channel < 4u &&
                     samples[(uint8_t)map->channel].valid;
        uint32_t raw28 = valid ? samples[(uint8_t)map->channel].raw28 : 0u;
        sensorarrayMeasureMarkFdcMatrixCell(frame, sIndex, d, raw28, valid);
    }
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
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    firstErr = sensorarrayMeasurePrepareFdcMatrixPath(state, "fdc_matrix_frame");
    if (firstErr != ESP_OK) {
        outFrame->errorMask = UINT64_MAX;
        sensorarrayMeasureGiveLock();
        return firstErr;
    }

    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        err = sensorarrayMeasureSetSwPhysicalLevel(state, SENSORARRAY_SW_PHYSICAL_HIGH, "fdc_matrix_row_pre");
        if (err == ESP_OK) {
            err = tmuxSwitchSelectRow((uint8_t)(s - 1u));
        }
        if (err == ESP_OK) {
            err = sensorarrayMeasureSetSwPhysicalLevel(state, SENSORARRAY_SW_PHYSICAL_HIGH, "fdc_matrix_row_post");
        }
        if (err == ESP_OK) {
            err = sensorarrayMeasureSetSelaPathQuiet(state,
                                                     SENSORARRAY_SELA_ROUTE_FDC2214,
                                                     (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US);
        }
        if (err == ESP_OK) {
            err = sensorarrayMeasureSetFdcSelBPathQuiet(state);
        }
        if (err != ESP_OK) {
            if (firstErr == ESP_OK) {
                firstErr = err;
            }
            sensorarrayMeasureMarkFdcMatrixRowError(outFrame, s);
            taskYIELD();
            continue;
        }

        sensorarrayMeasureDelayUs((uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_SETTLE_US);

        for (uint8_t discard = 0u; discard < (uint8_t)CONFIG_SENSORARRAY_FDC_MATRIX_DISCARD_SAMPLES; ++discard) {
            Fdc2214CapChannelSample_t ignored[4] = {0};
            (void)Fdc2214CapReadChannelsRaw(state->fdcPrimary.handle,
                                            SENSORARRAY_FDC_CHANNEL_MASK_ALL,
                                            ignored,
                                            sizeof(ignored) / sizeof(ignored[0]));
            (void)Fdc2214CapReadChannelsRaw(state->fdcSecondary.handle,
                                            SENSORARRAY_FDC_CHANNEL_MASK_ALL,
                                            ignored,
                                            sizeof(ignored) / sizeof(ignored[0]));
        }

        Fdc2214CapChannelSample_t primarySamples[4] = {0};
        Fdc2214CapChannelSample_t secondarySamples[4] = {0};
        esp_err_t primaryErr = Fdc2214CapReadChannelsRaw(state->fdcPrimary.handle,
                                                         SENSORARRAY_FDC_CHANNEL_MASK_ALL,
                                                         primarySamples,
                                                         sizeof(primarySamples) / sizeof(primarySamples[0]));
        esp_err_t secondaryErr = Fdc2214CapReadChannelsRaw(state->fdcSecondary.handle,
                                                           SENSORARRAY_FDC_CHANNEL_MASK_ALL,
                                                           secondarySamples,
                                                           sizeof(secondarySamples) / sizeof(secondarySamples[0]));
        if (primaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = primaryErr;
        }
        if (secondaryErr != ESP_OK && firstErr == ESP_OK) {
            firstErr = secondaryErr;
        }

        sensorarrayMeasureApplyFdcDeviceSamples(state,
                                                outFrame,
                                                s,
                                                SENSORARRAY_FDC_DEV_PRIMARY,
                                                primarySamples);
        sensorarrayMeasureApplyFdcDeviceSamples(state,
                                                outFrame,
                                                s,
                                                SENSORARRAY_FDC_DEV_SECONDARY,
                                                secondarySamples);

        taskYIELD();
    }

    sensorarrayMeasureGiveLock();
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

static void sensorarrayFdcMatrixPrintFrame(const sensorarrayFdcMatrixFrame_t *frame)
{
    printf("MATRIXFDC,seq=%lu,timestampUs=%llu,validMask=0x%016llX,errorMask=0x%016llX,raw28=[",
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

    sensorarrayFdcMatrixPrintFrame(frame);
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
