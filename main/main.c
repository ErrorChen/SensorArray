#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ads126xAdc.h"
#include "boardSupport.h"
#include "sensorarrayApp.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayTypes.h"
#include "sensorarrayVoltageScan.h"
#include "sensorarrayVoltageStream.h"
#include "tmuxSwitch.h"

#ifndef CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST
#define CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST 0
#endif

static sensorarrayState_t s_state = {0};
static uint8_t s_gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
static bool s_voltageHeaderPrinted = false;
static const char *s_voltageFatalStage = "init";
static uint32_t s_legacyStatFrames = 0u;
static uint64_t s_legacyStatScanTotalUs = 0u;
static uint32_t s_legacyStatScanMaxUs = 0u;

#define SENSORARRAY_MAIN_TMUX1108_REF_LEVEL (CONFIG_TMUX1108_SW_REF_LEVEL ? 1 : 0)
#define SENSORARRAY_MAIN_TMUX1108_GND_LEVEL (CONFIG_TMUX1108_SW_REF_LEVEL ? 0 : 1)
#define SENSORARRAY_MAIN_TMUX1108_SOURCE_LEVEL(source) \
    (((source) == TMUX1108_SOURCE_REF) ? SENSORARRAY_MAIN_TMUX1108_REF_LEVEL : SENSORARRAY_MAIN_TMUX1108_GND_LEVEL)

static bool sensorarrayMainCanPrintText(void)
{
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY
    if (CONFIG_SENSORARRAY_BINARY_PURE_MODE && !CONFIG_SENSORARRAY_BINARY_ALLOW_STARTUP_TEXT) {
        return false;
    }
#endif
    return true;
}

static void sensorarrayMainConfigureLogLevel(void)
{
#if CONFIG_SENSORARRAY_OUTPUT_FORMAT_BINARY
    if (CONFIG_SENSORARRAY_BINARY_PURE_MODE) {
        esp_log_level_set("*", ESP_LOG_NONE);
        return;
    }
#endif
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_FAST || CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_MAX
    esp_log_level_set("*", ESP_LOG_WARN);
#endif
}

typedef struct {
    const char *modeName;
    const char *modeNameCn;
    tmux1108Source_t swSource;
    const char *swName;
    bool skipFdcInit;
    bool useAdsInternalRef;
    bool useAdsVbias;
    bool useAdsRefPrepPath;
    int expectedSwLevel;
    const char *adsRefPolicyName;
    bool checkAdsRefmux;
    uint8_t expectedAdsRefmux;
    uint32_t vrefMicrovolts;
} sensorarrayVoltageReadModeConfig_t;

static const sensorarrayVoltageReadModeConfig_t s_piezoReadMode = {
    .modeName = "PIEZO_READ",
    .modeNameCn = "压电读取",
    .swSource = TMUX1108_SOURCE_GND,
    .swName = "GND",
    .skipFdcInit = true,
    .useAdsInternalRef = false,
    .useAdsVbias = false,
    .useAdsRefPrepPath = false,
    .expectedSwLevel = 1,
    .adsRefPolicyName = "piezo_no_refout_no_vbias",
    .checkAdsRefmux = true,
    .expectedAdsRefmux = ADS126X_REFMUX_AVDD_AVSS,
    .vrefMicrovolts = (uint32_t)SENSORARRAY_REF_SUPPLY_SPAN_MV * 1000u,
};

static const sensorarrayVoltageReadModeConfig_t s_resistanceReadMode = {
    .modeName = "RESISTANCE_READ",
    .modeNameCn = "电阻读取",
    .swSource = TMUX1108_SOURCE_REF,
    .swName = "REF",
    .skipFdcInit = true,
    .useAdsInternalRef = true,
    .useAdsVbias = true,
    .useAdsRefPrepPath = true,
    .expectedSwLevel = SENSORARRAY_MAIN_TMUX1108_SOURCE_LEVEL(TMUX1108_SOURCE_REF),
    .adsRefPolicyName = "resistance_internal_ref_mid",
    .checkAdsRefmux = true,
    .expectedAdsRefmux = ADS126X_REFMUX_INTERNAL,
    .vrefMicrovolts = ADS126X_ADC_DEFAULT_VREF_UV,
};

static uint8_t sensorarrayMainNormalizeGain(int configuredGain)
{
    switch (configuredGain) {
    case ADS126X_GAIN_1:
    case ADS126X_GAIN_2:
    case ADS126X_GAIN_4:
    case ADS126X_GAIN_8:
    case ADS126X_GAIN_16:
    case ADS126X_GAIN_32:
        return (uint8_t)configuredGain;
    default:
        return ADS126X_GAIN_1;
    }
}

static const char *sensorarrayMainAutoGainModeName(void)
{
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_OFF
    return "off";
#elif CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_FRAME
    return "per_frame";
#elif CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_POINT
    return "per_point";
#else
    return "bootstrap_once";
#endif
}

static void sensorarrayMainSetFatalStage(const char *stage)
{
    s_voltageFatalStage = stage ? stage : "init";
}

static const char *sensorarrayMainOffOn(bool enabled)
{
    return enabled ? "on" : "off";
}

static void sensorarrayMainIdleAfterFatal(const char *stage, esp_err_t err)
{
    if (sensorarrayMainCanPrintText()) {
        printf("VOLTSCAN_FATAL,stage=%s,err=%ld\n", stage ? stage : "unknown", (long)err);
    }
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }
}

static void sensorarrayMainPrintResetReason(void)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("RESET_REASON,reason=%d,heapFree=%u,heapMinFree=%u\n",
           (int)esp_reset_reason(),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT));
}

static void sensorarrayMainPrintBootAnalogSafe(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!mode || !sensorarrayMainCanPrintText()) {
        return;
    }

    printf("BOOT_ANALOG_SAFE,mode=%s,sw=%s,ref=%s,adsIntRef=%u,adsVbias=%u\n",
           mode->modeName,
           mode->swName,
           mode->useAdsInternalRef ? "internal" : "avdd_avss",
           mode->useAdsInternalRef ? 1u : 0u,
           mode->useAdsVbias ? 1u : 0u);
}

static esp_err_t sensorarrayMainApplyVoltageTmuxDefaults(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tmuxSwitchSelectRow(0u);
    if (err != ESP_OK) {
        return err;
    }

    int selaLevel = 0;
    if (!sensorarrayBoardMapSelaRouteToGpioLevel(SENSORARRAY_SELA_ROUTE_ADS1263, &selaLevel)) {
        return ESP_ERR_INVALID_STATE;
    }

    err = tmux1134SelectSelALevel(selaLevel != 0);
    if (err == ESP_OK) {
        err = tmux1134SelectSelBLevel(false);
    }
    if (err == ESP_OK) {
        err = tmuxSwitchSet1108Source(mode->swSource);
    }
    if (err == ESP_OK) {
        err = tmux1134SetEnLogicalState(true);
    }
    if (err == ESP_OK) {
        esp_rom_delay_us((uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US);
    }
    return err;
}

static void __attribute__((unused)) sensorarrayMainPrintAppMode(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("APPMODE,active=%s,cnName=%s,skipAdsInit=0,skipFdcInit=%u,sw=%s,"
           "expectedSwLevel=%d,intrefExpected=%u,vbiasExpected=%u,expectedRefmux=0x%02X,"
           "vrefUv=%lu,adsRefPolicy=%s\n",
           mode->modeName,
           mode->modeNameCn,
           mode->skipFdcInit ? 1u : 0u,
           mode->swName,
           mode->expectedSwLevel,
           mode->useAdsInternalRef ? 1u : 0u,
           mode->useAdsVbias ? 1u : 0u,
           mode->expectedAdsRefmux,
           (unsigned long)mode->vrefMicrovolts,
           mode->adsRefPolicyName);
}

static void __attribute__((unused)) sensorarrayMainPrintRoutePolicy(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("DBGROUTEPOLICY,mode=%s,sw=%s,expectedSwLevel=%d,adsIntRef=%s,adsVbias=%s,"
           "adsRefmux=0x%02X,vrefUv=%lu,refPrepPath=%u,sela=ADS126X,fdcInitSkipped=%u\n",
           mode->modeName,
           mode->swName,
           mode->expectedSwLevel,
           sensorarrayMainOffOn(mode->useAdsInternalRef),
           sensorarrayMainOffOn(mode->useAdsVbias),
           mode->expectedAdsRefmux,
           (unsigned long)mode->vrefMicrovolts,
           mode->useAdsRefPrepPath ? 1u : 0u,
           mode->skipFdcInit ? 1u : 0u);
}

esp_err_t sensorarrayMainCheckTmuxPolicy(const sensorarrayVoltageReadModeConfig_t *mode,
                                         sensorarrayStatusCode_t *outCode)
{
    if (outCode) {
        *outCode = SENSORARRAY_STATUS_OK;
    }
    if (!mode) {
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_INTERNAL_ASSERT_FAIL;
        }
        return ESP_ERR_INVALID_ARG;
    }

    tmuxSwitchControlState_t control = {0};
    esp_err_t err = tmuxSwitchGetControlState(&control);
    if (err != ESP_OK) {
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_TMUX_SOURCE_FAIL;
        }
        return err;
    }

    const bool sourceOk = (control.cmdSource == mode->swSource);
    const bool cmdSwOk = (control.cmdSwLevel == mode->expectedSwLevel);
    const bool obsSwOk = (control.obsSwLevel == mode->expectedSwLevel);
    if (!sourceOk || !cmdSwOk || !obsSwOk) {
        if (!mode->useAdsInternalRef && mode->expectedSwLevel == 1) {
            sensorarrayMainSetFatalStage("piezo_sw_not_high");
        } else {
            sensorarrayMainSetFatalStage("tmux_sw_policy_mismatch");
        }
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_TMUX_SW_POLICY_MISMATCH;
        }
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t sensorarrayMainCheckAdsPowerPolicy(sensorarrayState_t *state,
                                             const sensorarrayVoltageReadModeConfig_t *mode,
                                             sensorarrayStatusCode_t *outCode)
{
    if (outCode) {
        *outCode = SENSORARRAY_STATUS_OK;
    }
    if (!state || !mode) {
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_INTERNAL_ASSERT_FAIL;
        }
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t power = 0u;
    uint8_t refmux = 0u;
    esp_err_t err = ads126xAdcReadCoreRegisters(&state->ads, &power, NULL, NULL, NULL, &refmux);
    if (err != ESP_OK) {
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_ADS_SPI_FAIL;
        }
        return err;
    }

    const bool intrefEnabled = ((power & ADS126X_POWER_INTREF) != 0u);
    const bool vbiasEnabled = ((power & ADS126X_POWER_VBIAS) != 0u);
    const bool intrefOk = (intrefEnabled == mode->useAdsInternalRef);
    const bool vbiasOk = (vbiasEnabled == mode->useAdsVbias);
    const bool refmuxOk = (!mode->checkAdsRefmux || refmux == mode->expectedAdsRefmux);

    if (!mode->useAdsInternalRef && intrefEnabled) {
        sensorarrayMainSetFatalStage("piezo_refout_not_off");
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH;
        }
        return ESP_ERR_INVALID_STATE;
    }
    if (!mode->useAdsVbias && vbiasEnabled) {
        sensorarrayMainSetFatalStage("piezo_vbias_not_off");
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH;
        }
        return ESP_ERR_INVALID_STATE;
    }
    if (!refmuxOk) {
        sensorarrayMainSetFatalStage("ads_refmux_policy_mismatch");
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH;
        }
        return ESP_ERR_INVALID_STATE;
    }
    if (!intrefOk || !vbiasOk) {
        sensorarrayMainSetFatalStage("ads_power_policy_mismatch");
        if (outCode) {
            *outCode = SENSORARRAY_STATUS_ADS_REF_POLICY_MISMATCH;
        }
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

esp_err_t sensorarrayMainCheckAdsRefPolicy(sensorarrayState_t *state,
                                           const sensorarrayVoltageReadModeConfig_t *mode,
                                           sensorarrayStatusCode_t *outCode)
{
    return sensorarrayMainCheckAdsPowerPolicy(state, mode, outCode);
}

static esp_err_t __attribute__((unused)) sensorarrayMainLogTmuxPolicyReadback(const sensorarrayVoltageReadModeConfig_t *mode,
                                                                              const char *stage)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    tmuxSwitchControlState_t control = {0};
    esp_err_t err = tmuxSwitchGetControlState(&control);
    const bool readOk = (err == ESP_OK);
    const bool sourceOk = readOk && (control.cmdSource == mode->swSource);
    const bool cmdSwOk = readOk && (control.cmdSwLevel == mode->expectedSwLevel);
    const bool obsSwOk = readOk && (control.obsSwLevel == mode->expectedSwLevel);
    const bool ok = sourceOk && cmdSwOk && obsSwOk;

    if (sensorarrayMainCanPrintText()) {
        printf("DBGTMUXPOLICY,mode=%s,stage=%s,swSource=%s,cmdSwLevel=%d,obsSwLevel=%d,"
               "expectedSwLevel=%d,sourceOk=%u,cmdSwOk=%u,obsSwOk=%u,result=%s,err=%ld\n",
               mode->modeName,
               stage ? stage : "unknown",
               mode->swName,
               readOk ? control.cmdSwLevel : -1,
               readOk ? control.obsSwLevel : -1,
               mode->expectedSwLevel,
               sourceOk ? 1u : 0u,
               cmdSwOk ? 1u : 0u,
               obsSwOk ? 1u : 0u,
               ok ? "ok" : "mismatch",
               (long)err);
    }

    if (!readOk) {
        return err;
    }
    if (!ok) {
        if (!mode->useAdsInternalRef && mode->expectedSwLevel == 1) {
            sensorarrayMainSetFatalStage("piezo_sw_not_high");
        } else {
            sensorarrayMainSetFatalStage("tmux_sw_policy_mismatch");
        }
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t __attribute__((unused)) sensorarrayMainLogAdsPowerPolicyReadback(sensorarrayState_t *state,
                                                                                  const sensorarrayVoltageReadModeConfig_t *mode,
                                                                                  const char *stage)
{
    if (!state || !mode) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t power = 0u;
    uint8_t iface = 0u;
    uint8_t mode2 = 0u;
    uint8_t inpmux = 0u;
    uint8_t refmux = 0u;
    esp_err_t err = ads126xAdcReadCoreRegisters(&state->ads, &power, &iface, &mode2, &inpmux, &refmux);
    const bool readOk = (err == ESP_OK);
    const bool intrefEnabled = readOk && ((power & ADS126X_POWER_INTREF) != 0u);
    const bool vbiasEnabled = readOk && ((power & ADS126X_POWER_VBIAS) != 0u);
    const bool intrefOk = readOk && (intrefEnabled == mode->useAdsInternalRef);
    const bool vbiasOk = readOk && (vbiasEnabled == mode->useAdsVbias);
    const bool refmuxOk = readOk && (!mode->checkAdsRefmux || refmux == mode->expectedAdsRefmux);
    const bool ok = intrefOk && vbiasOk && refmuxOk;

    if (sensorarrayMainCanPrintText()) {
        printf("DBGADSREFPOLICY,mode=%s,stage=%s,policy=%s,power=0x%02X,interface=0x%02X,"
               "mode2=0x%02X,inpmux=0x%02X,refmux=0x%02X,powerIntref=%u,powerVbias=%u,"
               "expectedIntref=%u,expectedVbias=%u,expectedRefmux=0x%02X,refmuxCheck=%u,refmuxOk=%u,"
               "vrefUv=%lu,refout=%s,result=%s,mismatch_read=%u,mismatch_intref=%u,mismatch_vbias=%u,"
               "mismatch_refmux=%u,err=%ld\n",
               mode->modeName,
               stage ? stage : "unknown",
               mode->adsRefPolicyName ? mode->adsRefPolicyName : "unknown",
               power,
               iface,
               mode2,
               inpmux,
               refmux,
               intrefEnabled ? 1u : 0u,
               vbiasEnabled ? 1u : 0u,
               mode->useAdsInternalRef ? 1u : 0u,
               mode->useAdsVbias ? 1u : 0u,
               mode->expectedAdsRefmux,
               mode->checkAdsRefmux ? 1u : 0u,
               refmuxOk ? 1u : 0u,
               (unsigned long)mode->vrefMicrovolts,
               intrefEnabled ? "on" : "off",
               ok ? "ok" : "mismatch",
               readOk ? 0u : 1u,
               intrefOk ? 0u : 1u,
               vbiasOk ? 0u : 1u,
               refmuxOk ? 0u : 1u,
               (long)err);
    }

    if (!readOk) {
        return err;
    }
    if (!mode->useAdsInternalRef && intrefEnabled) {
        sensorarrayMainSetFatalStage("piezo_refout_not_off");
        return ESP_ERR_INVALID_STATE;
    }
    if (!mode->useAdsVbias && vbiasEnabled) {
        sensorarrayMainSetFatalStage("piezo_vbias_not_off");
        return ESP_ERR_INVALID_STATE;
    }
    if (!refmuxOk) {
        sensorarrayMainSetFatalStage("ads_refmux_policy_mismatch");
        return ESP_ERR_INVALID_STATE;
    }
    if (!ok) {
        sensorarrayMainSetFatalStage("ads_power_policy_mismatch");
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

static esp_err_t sensorarrayMainPrepareAdsPiezoNoRef(sensorarrayState_t *state,
                                                     const sensorarrayVoltageReadModeConfig_t *mode,
                                                     uint8_t defaultGain,
                                                     uint8_t dataRateDr)
{
    if (!state || !mode) {
        return ESP_ERR_INVALID_ARG;
    }

    if (state->adsAdc1Running) {
        esp_err_t err = ads126xAdcStopAdc1(&state->ads);
        if (err != ESP_OK) {
            sensorarrayMainSetFatalStage("piezo_stop_adc1");
            return err;
        }
        state->adsAdc1Running = false;
    }

    esp_err_t err = ads126xAdcSetInternalRefEnabled(&state->ads, false);
    if (err != ESP_OK) {
        sensorarrayMainSetFatalStage("piezo_refout_disable");
        return err;
    }

    err = ads126xAdcSetVbiasEnabled(&state->ads, false);
    if (err != ESP_OK) {
        sensorarrayMainSetFatalStage("piezo_vbias_disable");
        return err;
    }

    err = sensorarrayBringupAdsSetRefMux(state, mode->expectedAdsRefmux);
    if (err != ESP_OK) {
        sensorarrayMainSetFatalStage("piezo_refmux_set");
        return err;
    }

    err = ads126xAdcSetVrefMicrovolts(&state->ads, mode->vrefMicrovolts);
    if (err != ESP_OK) {
        sensorarrayMainSetFatalStage("piezo_vref_set");
        return err;
    }

    err = ads126xAdcConfigureVoltageMode(&state->ads, defaultGain, dataRateDr, false, false);
    if (err != ESP_OK) {
        sensorarrayMainSetFatalStage("piezo_voltage_config");
        return err;
    }

    state->adsRefReady = false;
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_SAFE
    return sensorarrayMainLogAdsPowerPolicyReadback(state, mode, "piezo_no_ref_settled");
#else
    sensorarrayStatusCode_t code = SENSORARRAY_STATUS_OK;
    return sensorarrayMainCheckAdsRefPolicy(state, mode, &code);
#endif
}

static esp_err_t sensorarrayMainInitVoltageScan(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state = (sensorarrayState_t){0};
    s_voltageHeaderPrinted = false;
    s_legacyStatFrames = 0u;
    s_legacyStatScanTotalUs = 0u;
    s_legacyStatScanMaxUs = 0u;
    sensorarrayMainSetFatalStage("init");
    memset(s_gainTable, sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN), sizeof(s_gainTable));
    uint8_t defaultGain = sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN);
    uint8_t dataRateDr = (uint8_t)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F);

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_SAFE
    sensorarrayMainPrintAppMode(mode);
#endif

    esp_err_t err = boardSupportInit();
    s_state.boardReady = (err == ESP_OK);
    if (err != ESP_OK) {
        return err;
    }

    err = tmuxSwitchInit();
    s_state.tmuxReady = (err == ESP_OK);
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayBringupInitAds(&s_state);
    s_state.adsReady = (err == ESP_OK);
    if (err != ESP_OK) {
        return err;
    }

    if (mode->useAdsRefPrepPath) {
        err = sensorarrayBringupPrepareAdsRefPath(&s_state);
        s_state.adsRefReady = (err == ESP_OK);
        if (err != ESP_OK) {
            return err;
        }

        if (s_state.adsAdc1Running) {
            err = ads126xAdcStopAdc1(&s_state.ads);
            if (err != ESP_OK) {
                return err;
            }
            s_state.adsAdc1Running = false;
        }

        err = ads126xAdcSetVrefMicrovolts(&s_state.ads, mode->vrefMicrovolts);
        if (err != ESP_OK) {
            return err;
        }

        err = ads126xAdcConfigureVoltageMode(&s_state.ads, defaultGain, dataRateDr, false, false);
    } else {
        err = sensorarrayMainPrepareAdsPiezoNoRef(&s_state, mode, defaultGain, dataRateDr);
    }
    if (err != ESP_OK) {
        return err;
    }

    err = sensorarrayMainApplyVoltageTmuxDefaults(mode);
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayStatusCode_t policyCode = SENSORARRAY_STATUS_OK;
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_SAFE
    sensorarrayMainPrintRoutePolicy(mode);
    err = sensorarrayMainLogTmuxPolicyReadback(mode, "voltage_scan_init");
#else
    err = sensorarrayMainCheckTmuxPolicy(mode, &policyCode);
#endif
    if (err != ESP_OK) {
        return err;
    }

#if !CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_SAFE
    err = sensorarrayMainCheckAdsRefPolicy(&s_state, mode, &policyCode);
    if (err != ESP_OK) {
        return err;
    }
#endif
    (void)policyCode;

    err = ads126xAdcStartAdc1(&s_state.ads);
    if (err != ESP_OK) {
        return err;
    }
    s_state.adsAdc1Running = true;
    return ESP_OK;
}

static ads126xAdcVoltageReadConfig_t sensorarrayMainAutoGainReadConfig(uint8_t muxp, uint8_t muxn)
{
    return (ads126xAdcVoltageReadConfig_t){
        .muxp = muxp,
        .muxn = muxn,
        .stopBeforeMuxChange = false,
        .startAfterMuxChange = false,
        .settleAfterMuxUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US,
        .discardFirst = (CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST != 0),
        .oversampleCount = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE,
        .drdyTimeoutUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_DRDY_TIMEOUT_US,
    };
}

static esp_err_t sensorarrayMainBootstrapAutoGain(const sensorarrayVoltageReadModeConfig_t *mode, bool printSummary)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t defaultGain = sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN);

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_OFF
    memset(s_gainTable, defaultGain, sizeof(s_gainTable));
    if (printSummary && sensorarrayMainCanPrintText()) {
        printf("VOLTSCAN_GAIN,mode=off,gain=%u\n", (unsigned)defaultGain);
    }
    return ESP_OK;
#else
    uint32_t errCount = 0u;
    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        esp_err_t routeErr = sensorarrayVoltageScanApplyRouteFastWithSource(
            s,
            1u,
            mode->swSource,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_ROW_SETTLE_US,
            (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US);
        if (routeErr != ESP_OK) {
            errCount += SENSORARRAY_VOLTAGE_SCAN_COLS;
            continue;
        }

        for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            uint8_t muxp = 0u;
            uint8_t muxn = 0u;
            if (!sensorarrayBoardMapAdsMuxForDLine(d, &muxp, &muxn)) {
                s_gainTable[s - 1u][d - 1u] = defaultGain;
                errCount++;
                continue;
            }

            ads126xAdcVoltageReadConfig_t readCfg = sensorarrayMainAutoGainReadConfig(muxp, muxn);
            ads126xAdcAutoGainConfig_t gainCfg = {
                .minGain = ADS126X_GAIN_1,
                .maxGain = ADS126X_GAIN_32,
                .initialGain = defaultGain,
                .headroomPercent = (uint8_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT,
                .maxIterations = 4u,
                .settleAfterGainChangeUs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_MUX_SETTLE_US,
                .keepConfiguredOnSuccess = true,
            };
            ads126xAdcAutoGainResult_t gainResult = {0};
            esp_err_t gainErr = ads126xAdcSelectAutoGainForVoltage(&s_state.ads, &readCfg, &gainCfg, &gainResult);
            if (gainErr == ESP_OK) {
                s_gainTable[s - 1u][d - 1u] = gainResult.selectedGain;
            } else {
                s_gainTable[s - 1u][d - 1u] = defaultGain;
                errCount++;
            }
        }
    }

    if ((printSummary || CONFIG_SENSORARRAY_VOLTAGE_SCAN_VERBOSE_LOG) && sensorarrayMainCanPrintText()) {
        printf("VOLTSCAN_GAIN,mode=%s,points=64,errCount=%lu,headroomPercent=%u\n",
               sensorarrayMainAutoGainModeName(),
               (unsigned long)errCount,
               (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT);
    }
    return (errCount == 0u) ? ESP_OK : ESP_FAIL;
#endif
}

static void sensorarrayMainPrintVoltageHeader(void)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("MATV_HEADER,seq,timestamp_us,duration_us,unit");
    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",S%uD%u", (unsigned)s, (unsigned)d);
        }
    }
    printf("\n");
}

static void sensorarrayMainMaybePrintVoltageHeader(uint32_t sequence)
{
    int headerEvery = CONFIG_SENSORARRAY_VOLTAGE_SCAN_PRINT_HEADER_EVERY_N_FRAMES;
    if (!s_voltageHeaderPrinted || (headerEvery > 0 && (sequence % (uint32_t)headerEvery) == 0u)) {
        sensorarrayMainPrintVoltageHeader();
        s_voltageHeaderPrinted = true;
    }
}

static bool sensorarrayMainFrameHasError(const sensorarrayVoltageFrame_t *frame)
{
    if (!frame) {
        return false;
    }
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            if (frame->err[s][d] != ESP_OK) {
                return true;
            }
        }
    }
    return false;
}

static void sensorarrayMainPrintVoltageFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("MATV,%" PRIu32 ",%" PRIu64 ",%" PRIu32 ",uV",
           frame->sequence,
           frame->timestampUs,
           frame->scanDurationUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%" PRIi32, frame->microvolts[s][d]);
        }
    }
    printf("\n");
}

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_RAW
static void sensorarrayMainPrintRawFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("MATV_RAW,%" PRIu32 ",%" PRIu64, frame->sequence, frame->timestampUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%" PRIi32, frame->raw[s][d]);
        }
    }
    printf("\n");
}
#endif

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_GAIN
static void sensorarrayMainPrintGainFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("MATV_GAIN,%" PRIu32 ",%" PRIu64, frame->sequence, frame->timestampUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%u", (unsigned)frame->gain[s][d]);
        }
    }
    printf("\n");
}
#endif

static void sensorarrayMainPrintErrFrameCsv(const sensorarrayVoltageFrame_t *frame)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("MATV_ERR,%" PRIu32 ",%" PRIu64, frame->sequence, frame->timestampUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%ld", (long)frame->err[s][d]);
        }
    }
    printf("\n");
}

static void sensorarrayMainPrintLegacyStat(const sensorarrayVoltageFrame_t *frame)
{
    if (!frame || !sensorarrayMainCanPrintText()) {
        return;
    }

    s_legacyStatFrames++;
    s_legacyStatScanTotalUs += frame->scanDurationUs;
    if (frame->scanDurationUs > s_legacyStatScanMaxUs) {
        s_legacyStatScanMaxUs = frame->scanDurationUs;
    }

    if (CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES == 0 ||
        (frame->sequence % (uint32_t)CONFIG_SENSORARRAY_STATUS_PERIOD_N_FRAMES) != 0u) {
        return;
    }

    uint32_t scanAvgUs = s_legacyStatFrames == 0u
                             ? 0u
                             : (uint32_t)(s_legacyStatScanTotalUs / (uint64_t)s_legacyStatFrames);
    uint32_t fps = scanAvgUs == 0u ? 0u : (1000000u / scanAvgUs);
    uint32_t pps = fps * SENSORARRAY_VOLTAGE_SCAN_ROWS * SENSORARRAY_VOLTAGE_SCAN_COLS;
    uint8_t adsDr = (uint8_t)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F);

    printf("STAT,seq=%" PRIu32 ",fps=%" PRIu32 ",pps=%" PRIu32
           ",scanAvgUs=%" PRIu32 ",scanMaxUs=%" PRIu32
           ",routeAvgUs=0,inpmuxAvgUs=0,drdyAvgUs=0,adcReadAvgUs=0,spiAvgUs=0,"
           "queueAvgUs=0,usbAvgUs=0,drop=0,decimated=0,qFull=0,drdyTimeout=0,spiFail=0,"
           "adsDr=%u,adsSps=%" PRIu32 ",outputDiv=1,scanPeriodUs=%u,status=0x00000000,code=0x0000\n",
           frame->sequence,
           fps,
           pps,
           scanAvgUs,
           s_legacyStatScanMaxUs,
           (unsigned)adsDr,
           ads126xAdcDataRateCodeToSps(adsDr),
           (unsigned)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_FRAME_PERIOD_MS * 1000u));
}

static void sensorarrayMainPrintInitSummary(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!sensorarrayMainCanPrintText()) {
        return;
    }
    printf("VOLTSCAN_INIT,mode=%s,cnName=%s,sw=%s,expectedSwLevel=%d,adsIntRef=%s,adsVbias=%s,"
           "adsRefmux=0x%02X,vrefUv=%lu,fdcInitSkipped=%u,rows=8,cols=8,unit=uV,dr=%u,gainDefault=%u,autoGain=%s,"
           "discardFirst=%u,oversample=%u\n",
           mode->modeName,
           mode->modeNameCn,
           mode->swName,
           mode->expectedSwLevel,
           sensorarrayMainOffOn(mode->useAdsInternalRef),
           sensorarrayMainOffOn(mode->useAdsVbias),
           mode->expectedAdsRefmux,
           (unsigned long)mode->vrefMicrovolts,
           mode->skipFdcInit ? 1u : 0u,
           (unsigned)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F),
           (unsigned)sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN),
           sensorarrayMainAutoGainModeName(),
           (CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST != 0) ? 1u : 0u,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE);
}

static void sensorarrayMainDelayFramePeriod(const sensorarrayVoltageFrame_t *frame)
{
    uint32_t framePeriodMs = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_FRAME_PERIOD_MS;
    if (framePeriodMs == 0u || !frame) {
        return;
    }

    uint64_t targetUs = (uint64_t)framePeriodMs * 1000u;
    uint64_t nowUs = (uint64_t)esp_timer_get_time();
    uint64_t elapsedUs = nowUs - frame->timestampUs;
    if (elapsedUs >= targetUs) {
        return;
    }

    uint64_t remainingUs = targetUs - elapsedUs;
    if (remainingUs >= 1000u) {
        vTaskDelay(pdMS_TO_TICKS((uint32_t)((remainingUs + 999u) / 1000u)));
    } else {
        esp_rom_delay_us((uint32_t)remainingUs);
    }
}

static void sensorarrayRunVoltageReadMode(const sensorarrayVoltageReadModeConfig_t *mode)
{
    sensorarrayMainConfigureLogLevel();

    sensorarrayMainPrintBootAnalogSafe(mode);

    esp_err_t err = sensorarrayMainInitVoltageScan(mode);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal(s_voltageFatalStage, err);
    }

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_FAST || CONFIG_SENSORARRAY_VOLTAGE_SCAN_PROFILE_MAX
    memset(s_gainTable, sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN), sizeof(s_gainTable));
    sensorarrayVoltageStreamConfig_t streamCfg = {
        .ads = &s_state.ads,
        .swSource = mode->swSource,
        .modeName = mode->modeName,
        .swName = mode->swName,
        .expectedAdsRefmux = mode->expectedAdsRefmux,
        .useAdsInternalRef = mode->useAdsInternalRef,
        .useAdsVbias = mode->useAdsVbias,
        .routePolicyOk = true,
        .adsPolicyOk = true,
    };
    err = sensorarrayVoltageStreamStart(&streamCfg);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("voltage_stream_start", err);
    }
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }
#endif

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_POINT
    memset(s_gainTable, sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN), sizeof(s_gainTable));
    if (sensorarrayMainCanPrintText()) {
        printf("VOLTSCAN_GAIN,mode=per_point,points=64,headroomPercent=%u\n",
               (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT);
    }
#else
    (void)sensorarrayMainBootstrapAutoGain(mode, true);
#endif
    sensorarrayMainPrintInitSummary(mode);

    while (true) {
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_FRAME
        (void)sensorarrayMainBootstrapAutoGain(mode, false);
#endif

        sensorarrayVoltageFrame_t frame = {0};
        (void)sensorarrayVoltageScanOneFrameWithSource(&s_state.ads, s_gainTable, mode->swSource, &frame);

        sensorarrayMainMaybePrintVoltageHeader(frame.sequence);
        sensorarrayMainPrintVoltageFrameCsv(&frame);
        sensorarrayMainPrintLegacyStat(&frame);
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_RAW
        sensorarrayMainPrintRawFrameCsv(&frame);
#endif
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_GAIN
        sensorarrayMainPrintGainFrameCsv(&frame);
#endif
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_ERR
        if (sensorarrayMainFrameHasError(&frame)) {
            sensorarrayMainPrintErrFrameCsv(&frame);
        }
#endif

        sensorarrayMainDelayFramePeriod(&frame);
        if ((frame.sequence & 0x0Fu) == 0u) {
            taskYIELD();
        }
    }
}

void sensorarrayRunPiezoRead(void)
{
    sensorarrayRunVoltageReadMode(&s_piezoReadMode);
}

void sensorarrayRunResistanceRead(void)
{
    sensorarrayRunVoltageReadMode(&s_resistanceReadMode);
}

void sensorarrayRunFastBinaryFake(void)
{
    sensorarrayMainConfigureLogLevel();

    sensorarrayVoltageStreamFakeConfig_t fakeCfg = {
        .modeName = "FAST_BINARY_FAKE",
        .adsDr = (uint8_t)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F),
        .fps = (uint32_t)CONFIG_SENSORARRAY_FAST_BINARY_FAKE_FPS,
    };
    esp_err_t err = sensorarrayVoltageStreamStartFake(&fakeCfg);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("fast_binary_fake_start", err);
    }
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }
}

// The main application entry point.
void app_main(void)
{
    sensorarrayMainConfigureLogLevel();
    sensorarrayMainPrintResetReason();

#if CONFIG_SENSORARRAY_APP_MODE_FAST_BINARY_FAKE
    sensorarrayRunFastBinaryFake();
#elif CONFIG_SENSORARRAY_APP_MODE_DEBUG
    if (sensorarrayMainCanPrintText()) {
        printf("APPMODE,active=DEBUG,cnName=Debug,skipAdsInit=1,skipFdcInit=0,sw=DEBUG\n"); //debug mode, no ADS126x initialization, direct GPIO control for routing
    }
    sensorarrayAppRun();
#elif CONFIG_SENSORARRAY_APP_MODE_RESISTANCE_READ
    sensorarrayRunResistanceRead();
#else
    sensorarrayRunPiezoRead();
#endif
}
