#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ads126xAdc.h"
#include "boardSupport.h"
#include "sensorarrayApp.h"
#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayLog.h"
#include "sensorarrayTypes.h"
#include "sensorarrayVoltageScan.h"
#include "tmuxSwitch.h"

static sensorarrayState_t s_state = {0};
static uint8_t s_gainTable[SENSORARRAY_VOLTAGE_SCAN_ROWS][SENSORARRAY_VOLTAGE_SCAN_COLS];
static bool s_voltageHeaderPrinted = false;

static esp_err_t sensorarrayMainConfigureAndStartVoltageAdc(void);

typedef enum {
    ROUTE_SOURCE_REF = 0,
    ROUTE_SOURCE_GND = 1,
} route_source_t;

typedef struct {
    sensorarrayAppMode_t appMode;
    const char *modeName;
    const char *modeNameCn;
    sensorarrayPath_t path;
    const char *routeModeName;
    bool skipFdcInit;
} sensorarrayVoltageReadModeConfig_t;

static const  sensorarrayVoltageReadModeConfig_t s_resistanceReadMode= {
    .appMode = SENSORARRAY_APP_MODE_RESISTANCE_READ,
    .modeName = "RESISTANCE_READ",
    .modeNameCn = "电阻读取",
    .path = SENSORARRAY_PATH_RESISTIVE,
    .routeModeName = "RESISTIVE",
    .skipFdcInit = true,
};

static const  sensorarrayVoltageReadModeConfig_t s_piezoReadMode= {
    .appMode = SENSORARRAY_APP_MODE_PIEZO_READ,
    .modeName = "PIEZO_READ",
    .modeNameCn = "压电读取",
    .path = SENSORARRAY_PATH_PIEZO_VOLTAGE,
    .routeModeName = "PIEZO_VOLTAGE",
    .skipFdcInit = true,
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

static void sensorarrayMainIdleAfterFatal(const char *stage, esp_err_t err)
{
    printf("VOLTSCAN_FATAL,stage=%s,err=%ld\n", stage ? stage : "unknown", (long)err);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000u));
    }
}

static tmux1108Source_t sensorarrayMainRequiredSource(const sensorarrayVoltageReadModeConfig_t *mode)
{
    return sensorarrayAppModeRequiredSource(mode ? mode->appMode : sensorarrayAppCompiledMode());
}

static const char *sensorarrayMainRequiredSourceName(const sensorarrayVoltageReadModeConfig_t *mode)
{
    return sensorarrayAppSwSourceName(sensorarrayMainRequiredSource(mode));
}

static int sensorarrayMainExpectedSwLevel(const sensorarrayVoltageReadModeConfig_t *mode)
{
    return tmuxSwitch1108SourceToSwLevel(sensorarrayMainRequiredSource(mode));
}

static route_source_t sensorarrayMainRouteSourceFromTmux(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? ROUTE_SOURCE_REF : ROUTE_SOURCE_GND;
}

static route_source_t sensorarrayMainRouteSourceForMode(const sensorarrayVoltageReadModeConfig_t *mode)
{
    return sensorarrayMainRouteSourceFromTmux(sensorarrayMainRequiredSource(mode));
}

static const char *sensorarrayMainRouteSourceName(route_source_t source)
{
    switch (source) {
    case ROUTE_SOURCE_REF:
        return "REF";
    case ROUTE_SOURCE_GND:
        return "GND";
    default:
        return "UNKNOWN";
    }
}

static esp_err_t sensorarrayMainReadRefPolicyState(int *outSwLevel, bool *outIntref, bool *outVbias)
{
    if (!outSwLevel || !outIntref || !outVbias) {
        return ESP_ERR_INVALID_ARG;
    }

    tmuxSwitchControlState_t ctrl = {0};
    esp_err_t err = tmuxSwitchGetControlState(&ctrl);
    if (err != ESP_OK) {
        return err;
    }
    *outSwLevel = (ctrl.obsSwLevel >= 0) ? ctrl.obsSwLevel : ctrl.cmdSwLevel;

    uint8_t power = 0u;
    err = ads126xAdcReadCoreRegisters(&s_state.ads, &power, NULL, NULL, NULL, NULL);
    if (err != ESP_OK) {
        return err;
    }
    *outIntref = ((power & ADS126X_POWER_INTREF) != 0u);
    *outVbias = ((power & ADS126X_POWER_VBIAS) != 0u);
    return ESP_OK;
}

esp_err_t route_apply_source_policy(route_source_t source)
{
    if (source != ROUTE_SOURCE_REF && source != ROUTE_SOURCE_GND) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_state.adsReady || !s_state.tmuxReady) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ads126xAdcStopAdc1(&s_state.ads);
    if (err != ESP_OK) {
        return err;
    }
    s_state.adsAdc1Running = false;

    err = tmux1134SetEnLogicalState(false);
    if (err != ESP_OK) {
        return err;
    }

    if (source == ROUTE_SOURCE_REF) {
        err = tmuxSwitchSet1108Source(TMUX1108_SOURCE_REF);
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(5u));

        err = ads126x_enable_ref_for_resistance_mode(&s_state);
        if (err != ESP_OK) {
            return err;
        }
        printf("DBGREFPOLICY,source=REF,sw=0,intref=1,vbias=1,expected_ref_mv=700\n");
        return ESP_OK;
    }

    err = ads126x_disable_ref_for_ground_mode(&s_state);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(20u));

    err = tmuxSwitchSet1108Source(TMUX1108_SOURCE_GND);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(5u));
    printf("DBGREFPOLICY,source=GND,sw=1,intref=0,vbias=0,expected_ref_mv=0\n");
    return ESP_OK;
}

static esp_err_t sensorarrayMainCheckRefSwConflict(route_source_t source)
{
    int swLevel = -1;
    bool intref = false;
    bool vbias = false;
    esp_err_t err = sensorarrayMainReadRefPolicyState(&swLevel, &intref, &vbias);
    if (err != ESP_OK) {
        printf("DBGREFCONFLICT,reason=policy_state_unavailable,source=%s,sw=-1,intref=-1,vbias=-1,err=%ld\n",
               sensorarrayMainRouteSourceName(source),
               (long)err);
        printf("MATV_ABORT,reason=ref_sw_conflict\n");
        return err;
    }

    const char *reason = NULL;
    if (swLevel == 1 && intref) {
        reason = "sw_high_but_intref_on";
    } else if (source == ROUTE_SOURCE_GND && intref) {
        reason = "gnd_mode_but_intref_on";
    } else if (source == ROUTE_SOURCE_GND && vbias) {
        reason = "gnd_mode_but_vbias_on";
    } else if (source == ROUTE_SOURCE_REF && swLevel != 0) {
        reason = "ref_mode_but_sw_high";
    } else if (source == ROUTE_SOURCE_GND && swLevel != 1) {
        reason = "gnd_mode_but_sw_low";
    } else if (source == ROUTE_SOURCE_REF && !intref) {
        reason = "ref_mode_but_intref_off";
    } else if (source == ROUTE_SOURCE_REF && !vbias) {
        reason = "ref_mode_but_vbias_off";
    }

    if (reason) {
        printf("DBGREFCONFLICT,reason=%s,sw=%d,intref=%u,vbias=%u,source=%s\n",
               reason,
               swLevel,
               intref ? 1u : 0u,
               vbias ? 1u : 0u,
               sensorarrayMainRouteSourceName(source));
        printf("MATV_ABORT,reason=ref_sw_conflict\n");
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

static esp_err_t sensorarrayMainApplyVoltageTmuxDefaults(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    tmux1108Source_t requiredSource = sensorarrayMainRequiredSource(mode);
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
        err = tmuxSwitchSet1108Source(requiredSource);
    }
    if (err == ESP_OK) {
        err = tmux1134SetEnLogicalState(true);
    }
    if (err == ESP_OK) {
        esp_rom_delay_us((uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_PATH_SETTLE_US);
    }
    if (err == ESP_OK) {
        err = tmuxSwitchAssert1108Source(requiredSource, mode->routeModeName);
    }
    return err;
}

static void sensorarrayMainPrintModeRequirement(const sensorarrayVoltageReadModeConfig_t *mode)
{
    printf("APPPOLICY,appMode=%s,isDebugAppMode=0,mode=%s,requiredSwSource=%s,requiredSwLevel=%d,reason=%s\n",
           mode->modeName,
           mode->routeModeName,
           sensorarrayMainRequiredSourceName(mode),
           sensorarrayMainExpectedSwLevel(mode),
           sensorarrayAppModeSourceReason(mode->appMode));
}

static void sensorarrayMainPrintRoutePolicy(const sensorarrayVoltageReadModeConfig_t *mode)
{
    tmuxSwitchControlState_t ctrl = {0};
    bool haveCtrl = (tmuxSwitchGetControlState(&ctrl) == ESP_OK);
    printf("ROUTEPOLICY,mode=%s,swSource=%s,expectedSwLevel=%d,cmdSwLevel=%d,obsSwLevel=%d,"
           "sela=ADS126X,fdcInitSkipped=%u\n",
           mode->routeModeName,
           sensorarrayMainRequiredSourceName(mode),
           sensorarrayMainExpectedSwLevel(mode),
           haveCtrl ? ctrl.cmdSwLevel : -1,
           haveCtrl ? ctrl.obsSwLevel : -1,
           mode->skipFdcInit ? 1u : 0u);
}

static esp_err_t sensorarrayMainInitVoltageScan(const sensorarrayVoltageReadModeConfig_t *mode)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state = (sensorarrayState_t){0};
    s_voltageHeaderPrinted = false;
    memset(s_gainTable, sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN), sizeof(s_gainTable));

    sensorarrayMainPrintModeRequirement(mode);

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

    route_source_t routeSource = sensorarrayMainRouteSourceForMode(mode);
    err = route_apply_source_policy(routeSource);
    if (err != ESP_OK) {
        s_state.adsRefReady = false;
        return err;
    }

    if (routeSource == ROUTE_SOURCE_REF) {
        err = sensorarrayBringupVerifyAdsRefAnalog(&s_state);
        s_state.adsRefReady = (err == ESP_OK);
        if (err != ESP_OK) {
            sensorarrayMainIdleAfterFatal("ref_analog_verify", err);
        }
    } else {
        s_state.adsRefReady = true;
    }

    err = sensorarrayMainApplyVoltageTmuxDefaults(mode);
    if (err != ESP_OK) {
        return err;
    }
    sensorarrayMainPrintRoutePolicy(mode);

    err = sensorarrayMainCheckRefSwConflict(routeSource);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("ref_sw_conflict", err);
    }

    if (s_state.adsAdc1Running) {
        err = ads126xAdcStopAdc1(&s_state.ads);
        if (err != ESP_OK) {
            return err;
        }
        s_state.adsAdc1Running = false;
    }

    return sensorarrayMainConfigureAndStartVoltageAdc();
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
    if (printSummary) {
        printf("VOLTSCAN_GAIN,mode=off,gain=%u\n", (unsigned)defaultGain);
    }
    return ESP_OK;
#else
    tmux1108Source_t requiredSource = sensorarrayMainRequiredSource(mode);
    uint32_t errCount = 0u;
    for (uint8_t s = 1u; s <= SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        esp_err_t routeErr = sensorarrayVoltageScanApplyRouteFastWithSource(
            s,
            1u,
            requiredSource,
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

    if (printSummary || CONFIG_SENSORARRAY_VOLTAGE_SCAN_VERBOSE_LOG) {
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
    printf("MATV_ERR,%" PRIu32 ",%" PRIu64, frame->sequence, frame->timestampUs);
    for (uint8_t s = 0u; s < SENSORARRAY_VOLTAGE_SCAN_ROWS; ++s) {
        for (uint8_t d = 0u; d < SENSORARRAY_VOLTAGE_SCAN_COLS; ++d) {
            printf(",%ld", (long)frame->err[s][d]);
        }
    }
    printf("\n");
}

static void sensorarrayMainPrintInitSummary(const sensorarrayVoltageReadModeConfig_t *mode)
{
    printf("VOLTSCAN_INIT,mode=%s,appMode=%s,cnName=%s,swSource=%s,expectedSwLevel=%d,"
           "fdcInitSkipped=%u,rows=8,cols=8,unit=uV,"
           "dr=%u,gainDefault=%u,autoGain=%s,discardFirst=%u,oversample=%u\n",
           mode->routeModeName,
           mode->modeName,
           mode->modeNameCn,
           sensorarrayMainRequiredSourceName(mode),
           sensorarrayMainExpectedSwLevel(mode),
           mode->skipFdcInit ? 1u : 0u,
           (unsigned)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F),
           (unsigned)sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN),
           sensorarrayMainAutoGainModeName(),
           (CONFIG_SENSORARRAY_VOLTAGE_SCAN_DISCARD_FIRST != 0) ? 1u : 0u,
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_OVERSAMPLE);
}

static esp_err_t sensorarrayMainConfigureAndStartVoltageAdc(void)
{
    uint8_t defaultGain = sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN);
    uint8_t dataRateDr = (uint8_t)(CONFIG_SENSORARRAY_VOLTAGE_SCAN_ADS_DATA_RATE & 0x0F);
    esp_err_t err = ads126xAdcConfigureVoltageMode(&s_state.ads, defaultGain, dataRateDr, false, false);
    if (err != ESP_OK) {
        return err;
    }

    err = ads126xAdcStartAdc1(&s_state.ads);
    if (err != ESP_OK) {
        return err;
    }
    s_state.adsAdc1Running = true;
    return ESP_OK;
}

static esp_err_t sensorarrayMainRecoverAds(const sensorarrayVoltageReadModeConfig_t *mode, uint32_t failStreak)
{
    if (!mode) {
        return ESP_ERR_INVALID_ARG;
    }

    printf("VOLTSCAN_RECOVERY,stage=begin,appMode=%s,failStreak=%lu\n",
           mode->modeName,
           (unsigned long)failStreak);

    esp_err_t stopErr = ads126xAdcStopAdc1(&s_state.ads);
    if (stopErr == ESP_OK) {
        s_state.adsAdc1Running = false;
    }

    esp_err_t resetErr = ads126xAdcHardwareReset(&s_state.ads);
    const char *resetStatus = "hardware_reset_ok";
    if (resetErr == ESP_ERR_NOT_SUPPORTED) {
        resetStatus = "hardware_reset_not_available";
        resetErr = ESP_OK;
    } else if (resetErr != ESP_OK) {
        resetStatus = "hardware_reset_failed";
    }
    printf("VOLTSCAN_RECOVERY,stage=reset,stopErr=%ld,resetErr=%ld,status=%s\n",
           (long)stopErr,
           (long)resetErr,
           resetStatus);
    if (resetErr != ESP_OK) {
        return resetErr;
    }

    s_state.adsRefReady = false;
    route_source_t routeSource = sensorarrayMainRouteSourceForMode(mode);
    esp_err_t err = route_apply_source_policy(routeSource);
    if (err == ESP_OK && routeSource == ROUTE_SOURCE_REF) {
        err = sensorarrayBringupVerifyAdsRefAnalog(&s_state);
    }
    s_state.adsRefReady = (err == ESP_OK);
    if (err != ESP_OK) {
        printf("VOLTSCAN_RECOVERY,stage=ref_analog_verify,err=%ld,status=ref_analog_verify_failed\n",
               (long)err);
        return err;
    }

    err = sensorarrayMainApplyVoltageTmuxDefaults(mode);
    if (err != ESP_OK) {
        printf("VOLTSCAN_RECOVERY,stage=route_defaults,err=%ld,status=route_defaults_failed\n", (long)err);
        return err;
    }

    err = sensorarrayMainCheckRefSwConflict(routeSource);
    if (err != ESP_OK) {
        printf("VOLTSCAN_RECOVERY,stage=ref_sw_conflict,err=%ld,status=conflict_after_route_defaults\n",
               (long)err);
        return err;
    }

    if (s_state.adsAdc1Running) {
        esp_err_t stopBeforeCfgErr = ads126xAdcStopAdc1(&s_state.ads);
        if (stopBeforeCfgErr != ESP_OK) {
            printf("VOLTSCAN_RECOVERY,stage=stop_before_config,err=%ld,status=stop_failed\n",
                   (long)stopBeforeCfgErr);
            return stopBeforeCfgErr;
        }
        s_state.adsAdc1Running = false;
    }

    err = sensorarrayMainConfigureAndStartVoltageAdc();
    printf("VOLTSCAN_RECOVERY,stage=complete,err=%ld,status=%s\n",
           (long)err,
           (err == ESP_OK) ? "ok" : "configure_start_failed");
    return err;
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
    esp_err_t err = sensorarrayMainInitVoltageScan(mode);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("init", err);
    }

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_POINT
    memset(s_gainTable, sensorarrayMainNormalizeGain(CONFIG_SENSORARRAY_VOLTAGE_SCAN_DEFAULT_GAIN), sizeof(s_gainTable));
    printf("VOLTSCAN_GAIN,mode=per_point,points=64,headroomPercent=%u\n",
           (unsigned)CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_HEADROOM_PERCENT);
#else
    (void)sensorarrayMainBootstrapAutoGain(mode, true);
#endif
    sensorarrayMainPrintInitSummary(mode);

    uint32_t consecutiveFrameFailures = 0u;
    uint32_t recoveryThreshold = (uint32_t)CONFIG_SENSORARRAY_VOLTAGE_SCAN_RECOVERY_ERR_THRESHOLD;
    if (recoveryThreshold == 0u) {
        recoveryThreshold = 3u;
    }
    tmux1108Source_t requiredSource = sensorarrayMainRequiredSource(mode);
    route_source_t routeSource = sensorarrayMainRouteSourceForMode(mode);

    while (true) {
        esp_err_t conflictErr = sensorarrayMainCheckRefSwConflict(routeSource);
        if (conflictErr != ESP_OK) {
            sensorarrayMainIdleAfterFatal("ref_sw_conflict", conflictErr);
        }

#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_AUTO_GAIN_PER_FRAME
        (void)sensorarrayMainBootstrapAutoGain(mode, false);
#endif

        sensorarrayVoltageFrame_t frame = {0};
        esp_err_t readErr =
            sensorarrayVoltageScanOneFrameWithSource(&s_state.ads, s_gainTable, requiredSource, &frame);
        bool frameHasError = (readErr != ESP_OK) || sensorarrayMainFrameHasError(&frame);

        sensorarrayMainMaybePrintVoltageHeader(frame.sequence);
        sensorarrayMainPrintVoltageFrameCsv(&frame);
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_RAW
        sensorarrayMainPrintRawFrameCsv(&frame);
#endif
#if CONFIG_SENSORARRAY_VOLTAGE_SCAN_OUTPUT_GAIN
        sensorarrayMainPrintGainFrameCsv(&frame);
#endif
        if (frameHasError) {
            sensorarrayMainPrintErrFrameCsv(&frame);
            consecutiveFrameFailures++;
            vTaskDelay(pdMS_TO_TICKS(1u));
        } else {
            consecutiveFrameFailures = 0u;
        }

        sensorarrayMainDelayFramePeriod(&frame);
        taskYIELD();

        if (consecutiveFrameFailures >= recoveryThreshold) {
            vTaskDelay(pdMS_TO_TICKS(1u));
            esp_err_t recoveryErr = sensorarrayMainRecoverAds(mode, consecutiveFrameFailures);
            if (recoveryErr != ESP_OK) {
                sensorarrayMainIdleAfterFatal("recovery", recoveryErr);
            }
            requiredSource = sensorarrayMainRequiredSource(mode);
            consecutiveFrameFailures = 0u;
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

#if SENSOR_ARRAY_DIAG_REF_SW_CONFLICT
static void sensorarrayMainDiagPrintRefSw(const char *phase)
{
    int swLevel = -1;
    bool intref = false;
    bool vbias = false;
    esp_err_t err = sensorarrayMainReadRefPolicyState(&swLevel, &intref, &vbias);
    printf("DBGDIAG_REF_SW,phase=%s,sw=%d,intref=%u,vbias=%u,err=%ld\n",
           phase ? phase : "UNKNOWN",
           swLevel,
           intref ? 1u : 0u,
           vbias ? 1u : 0u,
           (long)err);
}

static void sensorarrayRunRefSwConflictDiag(void)
{
    s_state = (sensorarrayState_t){0};

    esp_err_t err = boardSupportInit();
    s_state.boardReady = (err == ESP_OK);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("diag_board", err);
    }

    err = tmuxSwitchInit();
    s_state.tmuxReady = (err == ESP_OK);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("diag_tmux", err);
    }

    err = sensorarrayBringupInitAds(&s_state);
    s_state.adsReady = (err == ESP_OK);
    if (err != ESP_OK) {
        sensorarrayMainIdleAfterFatal("diag_ads", err);
    }

    printf("APPMODE,compiled=DIAG_REF_SW_CONFLICT,entry=ref_sw_conflict_diag,swSource=REF|GND,expectedSwLevel=0|1\n");
    while (true) {
        err = route_apply_source_policy(ROUTE_SOURCE_REF);
        if (err != ESP_OK) {
            sensorarrayMainIdleAfterFatal("diag_ref_policy", err);
        }
        for (uint8_t i = 0u; i < 10u; ++i) {
            sensorarrayMainDiagPrintRefSw("REF");
            vTaskDelay(pdMS_TO_TICKS(500u));
        }

        err = route_apply_source_policy(ROUTE_SOURCE_GND);
        if (err != ESP_OK) {
            sensorarrayMainIdleAfterFatal("diag_gnd_policy", err);
        }
        for (uint8_t i = 0u; i < 10u; ++i) {
            sensorarrayMainDiagPrintRefSw("GND");
            vTaskDelay(pdMS_TO_TICKS(500u));
        }
    }
}
#endif

static void sensorarrayMainPrintStartupAppMode(void)
{
    sensorarrayAppMode_t appMode = sensorarrayAppCompiledMode();
    tmux1108Source_t source = sensorarrayAppModeRequiredSource(appMode);

#if CONFIG_SENSORARRAY_APP_MODE_DEBUG
    sensorarrayDebugMode_t debugMode = (sensorarrayDebugMode_t)SENSORARRAY_ACTIVE_DEBUG_MODE;
    const char *debugRouteMode = NULL;
    const char *debugReason = NULL;
    bool debugSourceResolved =
        sensorarrayAppDebugModeRequiredSource(debugMode, &source, &debugRouteMode, &debugReason);
    printf("APPMODE,compiled=%s,entry=%s,swSource=%s,expectedSwLevel=%d,"
           "debugMode=%s,debugRouteMode=%s,debugSourceResolved=%u,reason=%s\n",
           sensorarrayAppModeName(appMode),
           sensorarrayAppModeEntryName(appMode),
           sensorarrayAppSwSourceName(source),
           tmuxSwitch1108SourceToSwLevel(source),
           sensorarrayLogDebugModeName(debugMode),
           debugRouteMode ? debugRouteMode : "NONE",
           debugSourceResolved ? 1u : 0u,
           debugReason ? debugReason : "debug_submode_selects_source");
#else
    printf("APPMODE,compiled=%s,entry=%s,swSource=%s,expectedSwLevel=%d\n",
           sensorarrayAppModeName(appMode),
           sensorarrayAppModeEntryName(appMode),
           sensorarrayAppSwSourceName(source),
           tmuxSwitch1108SourceToSwLevel(source));
#endif
}

// The main application entry point.
void app_main(void)
{
#if SENSOR_ARRAY_DIAG_REF_SW_CONFLICT
    sensorarrayRunRefSwConflictDiag();
#else
    sensorarrayMainPrintStartupAppMode();
#if CONFIG_SENSORARRAY_APP_MODE_DEBUG
    sensorarrayAppRun();
#elif CONFIG_SENSORARRAY_APP_MODE_RESISTANCE_READ
    sensorarrayRunResistanceRead();
#else
    sensorarrayRunPiezoRead();
#endif
#endif
}
