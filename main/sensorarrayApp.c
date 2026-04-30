#include "sensorarrayApp.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "boardSupport.h"
#include "tmuxSwitch.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayTypes.h"

static sensorarrayState_t s_state = {0};

static const sensorarrayAdsReadPolicy_t s_adsReadPolicy = {
    .stopBeforeMuxChange = (CONFIG_SENSORARRAY_ADS_READ_STOP1_BEFORE_MUX != 0),
    .settleAfterMuxMs = (uint32_t)CONFIG_SENSORARRAY_ADS_READ_SETTLE_AFTER_MUX_MS,
    .startEveryRead = (CONFIG_SENSORARRAY_ADS_READ_START1_EVERY_READ != 0),
    .baseDiscardCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_BASE_DISCARD_COUNT,
    .readRetryCount = (uint8_t)CONFIG_SENSORARRAY_ADS_READ_RETRY_COUNT,
};

sensorarrayAppMode_t sensorarrayAppCompiledMode(void)
{
#if CONFIG_SENSORARRAY_APP_MODE_DEBUG
    return SENSORARRAY_APP_MODE_DEBUG;
#elif CONFIG_SENSORARRAY_APP_MODE_RESISTANCE_READ
    return SENSORARRAY_APP_MODE_RESISTANCE_READ;
#else
    return SENSORARRAY_APP_MODE_PIEZO_READ;
#endif
}

const char *sensorarrayAppModeName(sensorarrayAppMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_APP_MODE_RESISTANCE_READ:
        return "RESISTANCE_READ";
    case SENSORARRAY_APP_MODE_DEBUG:
        return "DEBUG";
    case SENSORARRAY_APP_MODE_PIEZO_READ:
    default:
        return "PIEZO_READ";
    }
}

const char *sensorarrayAppModeEntryName(sensorarrayAppMode_t mode)
{
    return (mode == SENSORARRAY_APP_MODE_DEBUG) ? "debug_dispatcher" : "main_fast_scan";
}

const char *sensorarrayAppSwSourceName(tmux1108Source_t source)
{
    return (source == TMUX1108_SOURCE_REF) ? "REF" : "GND";
}

tmux1108Source_t sensorarrayAppModeRequiredSource(sensorarrayAppMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_APP_MODE_RESISTANCE_READ:
        return TMUX1108_SOURCE_REF;
    case SENSORARRAY_APP_MODE_DEBUG:
        /*
         * DEBUG app mode has no single production source. GND is the safe
         * fallback for startup logging; each debug submode resolves its own
         * source through sensorarrayAppDebugModeRequiredSource().
         */
        return TMUX1108_SOURCE_GND;
    case SENSORARRAY_APP_MODE_PIEZO_READ:
    default:
        return TMUX1108_SOURCE_GND;
    }
}

const char *sensorarrayAppModeSourceReason(sensorarrayAppMode_t mode)
{
    switch (mode) {
    case SENSORARRAY_APP_MODE_RESISTANCE_READ:
        return "resistive_requires_reference_source";
    case SENSORARRAY_APP_MODE_DEBUG:
        return "debug_submode_selects_source";
    case SENSORARRAY_APP_MODE_PIEZO_READ:
    default:
        return "piezo_requires_ground_source";
    }
}

bool sensorarrayAppDebugModeRequiredSource(sensorarrayDebugMode_t debugMode,
                                           tmux1108Source_t *outSource,
                                           const char **outRouteMode,
                                           const char **outReason)
{
    const char *routeMode = NULL;
    tmux1108Source_t source = TMUX1108_SOURCE_GND;
    const char *reason = NULL;

    switch (debugMode) {
    case SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR:
        routeMode = "RESISTIVE";
        source = TMUX1108_SOURCE_REF;
        reason = "resistive_requires_reference_source";
        break;
    case SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY:
    case SENSORARRAY_DEBUG_MODE_FDC_SELFTEST:
        routeMode = "CAPACITIVE";
        source = TMUX1108_SOURCE_GND;
        reason = "fdc_capacitive_requires_ground_source";
        break;
    case SENSORARRAY_DEBUG_MODE_ROUTE_FIXED_STATE:
#if CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_CAPACITIVE
        routeMode = "CAPACITIVE";
        source = TMUX1108_SOURCE_GND;
        reason = "fdc_capacitive_requires_ground_source";
#elif CONFIG_SENSORARRAY_DEBUG_FIXED_PATH_VOLTAGE
        routeMode = "PIEZO_VOLTAGE";
        source = TMUX1108_SOURCE_GND;
        reason = "piezo_requires_ground_source";
#else
        routeMode = "RESISTIVE";
        source = TMUX1108_SOURCE_REF;
        reason = "resistive_requires_reference_source";
#endif
        break;
    default:
        return false;
    }

    if (outSource) {
        *outSource = source;
    }
    if (outRouteMode) {
        *outRouteMode = routeMode;
    }
    if (outReason) {
        *outReason = reason;
    }
    return true;
}

static void sensorarrayApplyTmuxDefaults(sensorarrayDebugMode_t activeMode)
{
    if (!s_state.tmuxReady) {
        return;
    }

    tmux1108Source_t defaultSource = sensorarrayAppModeRequiredSource(SENSORARRAY_APP_MODE_DEBUG);
    (void)sensorarrayAppDebugModeRequiredSource(activeMode, &defaultSource, NULL, NULL);

    esp_err_t tmuxErr = tmuxSwitchSelectRow(0);
    if (tmuxErr == ESP_OK) {
        tmuxErr = sensorarrayMeasureSetSelaPath(&s_state,
                                                SENSORARRAY_SELA_ROUTE_ADS1263,
                                                SENSORARRAY_SETTLE_AFTER_PATH_MS,
                                                "init_default",
                                                "tmux_defaults");
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmux1134SelectSelBLevel(false);
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmuxSwitchSet1108Source(defaultSource);
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmux1134SetEnLogicalState(true);
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmuxSwitchAssert1108Source(defaultSource, "tmux_defaults");
    }

    sensorarrayLogStartup("tmux_defaults",
                          tmuxErr,
                          (tmuxErr == ESP_OK) ? "ok" : "set_failed",
                          (int32_t)(tmuxErr == ESP_OK));
    sensorarrayLogControlGpio("tmux_defaults", "INIT");
}

static void sensorarrayAppLogModeRequirement(sensorarrayDebugMode_t activeMode)
{
    const char *mode = "NONE";
    tmux1108Source_t source = sensorarrayAppModeRequiredSource(SENSORARRAY_APP_MODE_DEBUG);
    const char *reason = "no_fixed_source_for_debug_submode";
    bool resolved = sensorarrayAppDebugModeRequiredSource(activeMode, &source, &mode, &reason);

    if (!resolved) {
        printf("DBGMODE,appMode=DEBUG,isDebugAppMode=1,debugMode=%s,mode=%s,"
               "requiredSwSource=DEBUG_SUBMODE_UNSPECIFIED,requiredSwLevel=-1,reason=%s\n",
               sensorarrayLogDebugModeName(activeMode),
               mode,
               reason);
        return;
    }

    printf("DBGMODE,appMode=DEBUG,isDebugAppMode=1,debugMode=%s,mode=%s,"
           "requiredSwSource=%s,requiredSwLevel=%d,reason=%s\n",
           sensorarrayLogDebugModeName(activeMode),
           mode,
           sensorarrayAppSwSourceName(source),
           tmuxSwitch1108SourceToSwLevel(source),
           reason);
}

void sensorarrayAppRun(void)
{
    sensorarrayLogDbgExtraReset();
    s_state = (sensorarrayState_t){0};
    sensorarrayLogSetAdsState(false, false);

    sensorarrayDebugMode_t activeMode = (sensorarrayDebugMode_t)SENSORARRAY_ACTIVE_DEBUG_MODE;
    sensorarrayAppLogModeRequirement(activeMode);
    bool s1d1ResMode = (activeMode == SENSORARRAY_DEBUG_MODE_S1D1_RESISTOR);
    bool s1d1RouteOnly = s1d1ResMode && (CONFIG_SENSORARRAY_DEBUG_S1D1_ROUTE_ONLY != 0);
    bool singleCapFdcMode = (activeMode == SENSORARRAY_DEBUG_MODE_S5D5_CAP_FDC_SECONDARY);
    bool fdcI2cDiscoveryMode = (activeMode == SENSORARRAY_DEBUG_MODE_FDC_I2C_DISCOVERY);
    bool skipAdsInit = s1d1RouteOnly || singleCapFdcMode || fdcI2cDiscoveryMode;
    bool skipFdcInit = s1d1ResMode || fdcI2cDiscoveryMode;

    uint8_t requestedChannels = sensorarrayBringupNormalizeFdcChannels((uint8_t)CONFIG_FDC2214CAP_CHANNELS);
    if (requestedChannels < SENSORARRAY_FDC_REQUIRED_CHANNELS) {
        requestedChannels = SENSORARRAY_FDC_REQUIRED_CHANNELS;
    }
    s_state.fdcConfiguredChannels = requestedChannels;

    sensorarrayBringupResetFdcState(&s_state.fdcPrimary,
                                    "primary_sela_side",
                                    (uint8_t)(CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR & 0xFFu));
    sensorarrayBringupResetFdcState(&s_state.fdcSecondary,
                                    "secondary_selb_side",
                                    (uint8_t)(CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR & 0xFFu));

    bool primaryAddrValid = sensorarrayBringupParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                                                              &s_state.fdcPrimary.i2cAddr);
    bool secondaryAddrValid = sensorarrayBringupParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
                                                                &s_state.fdcSecondary.i2cAddr);
    if (singleCapFdcMode) {
        // Dedicated SELB bring-up is fixed to ADDR-low secondary FDC2214.
        s_state.fdcSecondary.i2cAddr = SENSORARRAY_FDC_I2C_ADDR_LOW;
        secondaryAddrValid = true;
    }

    sensorarrayLogStartup("app", ESP_OK, "start", 0);
    sensorarrayLogStartup("fdc_channels", ESP_OK, "policy_applied", (int32_t)requestedChannels);
    sensorarrayLogStartupFdc("fdc_cfg",
                             &s_state.fdcPrimary,
                             primaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             primaryAddrValid ? "configured" : "invalid_addr_config",
                             (int32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                             false,
                             0,
                             0,
                             "D1..D4_primary_sela_side");
    sensorarrayLogStartupFdc("fdc_cfg",
                             &s_state.fdcSecondary,
                             secondaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             secondaryAddrValid
                                 ? (singleCapFdcMode ? "forced_addr_low_for_selb_mode" : "configured")
                                 : "invalid_addr_config",
                             (int32_t)s_state.fdcSecondary.i2cAddr,
                             false,
                             0,
                             0,
                             "D5..D8_secondary_selb_side");
    if (!singleCapFdcMode && !fdcI2cDiscoveryMode) {
        sensorarrayBoardMapAudit();
    }

    esp_err_t err = boardSupportInit();
    s_state.boardReady = (err == ESP_OK);
    sensorarrayLogStartup("board", err, s_state.boardReady ? "ok" : "init_failed", (int32_t)s_state.boardReady);

    err = tmuxSwitchInit();
    s_state.tmuxReady = (err == ESP_OK);
    sensorarrayLogStartup("tmux", err, s_state.tmuxReady ? "ok" : "init_failed", (int32_t)s_state.tmuxReady);

    sensorarrayApplyTmuxDefaults(activeMode);

    if (skipAdsInit) {
        const char *adsSkipStatus = singleCapFdcMode
                                        ? "skip_cap_fdc_secondary_mode"
                                        : (fdcI2cDiscoveryMode ? "skip_fdc_i2c_discovery_mode" : "skip_route_only_mode");
        s_state.adsReady = false;
        s_state.adsRefReady = false;
        s_state.adsAdc1Running = false;
        s_state.adsRefMuxValid = false;
        sensorarrayLogStartup("ads", ESP_ERR_INVALID_STATE, adsSkipStatus, 0);
        sensorarrayLogStartup("ads_ref", ESP_ERR_INVALID_STATE, adsSkipStatus, 0);
    } else {
        err = sensorarrayBringupInitAds(&s_state);
        s_state.adsReady = (err == ESP_OK);
        sensorarrayLogStartup("ads", err, s_state.adsReady ? "ok" : "init_failed", (int32_t)s_state.adsReady);

        if (s_state.adsReady) {
            err = sensorarrayBringupPrepareAdsRefPath(&s_state);
            s_state.adsRefReady = (err == ESP_OK);
            sensorarrayLogStartup("ads_ref", err, s_state.adsRefReady ? "ready" : "not_ready", (int32_t)s_state.adsRefReady);
        } else {
            s_state.adsRefReady = false;
            s_state.adsAdc1Running = false;
            s_state.adsRefMuxValid = false;
            sensorarrayLogStartup("ads_ref", ESP_ERR_INVALID_STATE, "skip_ads_unavailable", 0);
        }
    }
    sensorarrayLogSetAdsState(s_state.adsReady, s_state.adsRefReady);

    if (skipFdcInit) {
        const char *fdcSkipStatus = s1d1ResMode ? "skip_s1d1_resistor_mode" : "skip_fdc_i2c_discovery_mode";
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcPrimary,
                                 ESP_ERR_NOT_SUPPORTED,
                                 fdcSkipStatus,
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D1..D4_primary_sela_side");
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcSecondary,
                                 ESP_ERR_NOT_SUPPORTED,
                                 fdcSkipStatus,
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D5..D8_secondary_selb_side");
    } else if (s_state.boardReady) {
        s_state.fdcPrimary.i2cCtx = boardSupportGetI2cCtx();
        s_state.fdcSecondary.i2cCtx = boardSupportGetI2c1Ctx();

        if (singleCapFdcMode) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcPrimary,
                                     ESP_ERR_NOT_SUPPORTED,
                                     "skip_selb_single_point_mode",
                                     0,
                                     false,
                                     0,
                                     0,
                                     "D1..D4_primary_sela_side");
        } else if (!primaryAddrValid) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcPrimary,
                                     ESP_ERR_INVALID_ARG,
                                     "skip_invalid_addr_config",
                                     (int32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                                     false,
                                     0,
                                     0,
                                     "D1..D4_primary_sela_side");
        } else if (!s_state.fdcPrimary.i2cCtx) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcPrimary,
                                     ESP_ERR_NOT_SUPPORTED,
                                     "skip_i2c0_unavailable",
                                     0,
                                     false,
                                     0,
                                     0,
                                     "D1..D4_primary_sela_side");
        } else {
            sensorarrayBringupProbeFdcBus(&s_state.fdcPrimary);

            sensorarrayFdcInitDiag_t diag = {0};
            err = sensorarrayBringupInitFdcDevice(s_state.fdcPrimary.i2cCtx,
                                                  s_state.fdcPrimary.i2cAddr,
                                                  requestedChannels,
                                                  &s_state.fdcPrimary.handle,
                                                  &diag);
            s_state.fdcPrimary.ready = (err == ESP_OK);
            s_state.fdcPrimary.haveIds = diag.haveIds;
            s_state.fdcPrimary.manufacturerId = diag.manufacturerId;
            s_state.fdcPrimary.deviceId = diag.deviceId;
            s_state.fdcPrimary.configVerified = diag.configVerified;
            s_state.fdcPrimary.refClockKnown = diag.refClockKnown;
            s_state.fdcPrimary.refClockSource = diag.refClockSource;
            s_state.fdcPrimary.refClockHz = diag.refClockHz;
            s_state.fdcPrimary.statusConfigReg = diag.statusConfigReg;
            s_state.fdcPrimary.configReg = diag.configReg;
            s_state.fdcPrimary.muxConfigReg = diag.muxConfigReg;
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcPrimary,
                                     err,
                                     diag.status,
                                     (err == ESP_OK) ? (int32_t)requestedChannels : diag.detail,
                                     diag.haveIds,
                                     diag.manufacturerId,
                                     diag.deviceId,
                                     "D1..D4_primary_sela_side");
        }

        if (!secondaryAddrValid) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcSecondary,
                                     ESP_ERR_INVALID_ARG,
                                     "skip_invalid_addr_config",
                                     (int32_t)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
                                     false,
                                     0,
                                     0,
                                     "D5..D8_secondary_selb_side");
        } else if (!s_state.fdcSecondary.i2cCtx) {
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcSecondary,
                                     ESP_ERR_NOT_SUPPORTED,
                                     "skip_i2c1_unavailable",
                                     0,
                                     false,
                                     0,
                                     0,
                                     "D5..D8_secondary_selb_side");
        } else {
            sensorarrayBringupProbeFdcBus(&s_state.fdcSecondary);

            sensorarrayFdcInitDiag_t diag = {0};
            err = sensorarrayBringupInitFdcDevice(s_state.fdcSecondary.i2cCtx,
                                                  s_state.fdcSecondary.i2cAddr,
                                                  singleCapFdcMode ? 1u : requestedChannels,
                                                  &s_state.fdcSecondary.handle,
                                                  &diag);
            s_state.fdcSecondary.ready = (err == ESP_OK);
            s_state.fdcSecondary.haveIds = diag.haveIds;
            s_state.fdcSecondary.manufacturerId = diag.manufacturerId;
            s_state.fdcSecondary.deviceId = diag.deviceId;
            s_state.fdcSecondary.configVerified = diag.configVerified;
            s_state.fdcSecondary.refClockKnown = diag.refClockKnown;
            s_state.fdcSecondary.refClockSource = diag.refClockSource;
            s_state.fdcSecondary.refClockHz = diag.refClockHz;
            s_state.fdcSecondary.statusConfigReg = diag.statusConfigReg;
            s_state.fdcSecondary.configReg = diag.configReg;
            s_state.fdcSecondary.muxConfigReg = diag.muxConfigReg;
            sensorarrayLogStartupFdc("fdc_init",
                                     &s_state.fdcSecondary,
                                     err,
                                     diag.status,
                                     (err == ESP_OK) ? (int32_t)(singleCapFdcMode ? 1u : requestedChannels) : diag.detail,
                                     diag.haveIds,
                                     diag.manufacturerId,
                                     diag.deviceId,
                                     "D5..D8_secondary_selb_side");
        }
    } else {
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcPrimary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D1..D4_primary_sela_side");
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcSecondary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D5..D8_secondary_selb_side");
    }

    sensorarrayDebugRunSelectedMode(&s_state, &s_adsReadPolicy, activeMode);
}
