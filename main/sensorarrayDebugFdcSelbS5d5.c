#include "sensorarrayDebugFdcSelbS5d5.h"

#include <limits.h>
#include <stdio.h>

#include "boardSupport.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayDebug.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"

#define SENSORARRAY_FDC_DBG_MIN_SAMPLES 4u
#define SENSORARRAY_FDC_DBG_MAX_SAMPLES 32u
#define SENSORARRAY_FDC_DBG_SAMPLE_DELAY_MS 20u
#define SENSORARRAY_FDC_DBG_MIN_LOOP_DELAY_MS 50u
#define SENSORARRAY_FDC_DBG_STEP_SETTLE_MS 350u
#define SENSORARRAY_FDC_DBG_STEP_DISCARD_COUNT 4u

#define SENSORARRAY_FDC_REG_RCOUNT_CH0 0x08u
#define SENSORARRAY_FDC_REG_SETTLECOUNT_CH0 0x10u
#define SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_CH0 0x14u
#define SENSORARRAY_FDC_REG_STATUS 0x18u
#define SENSORARRAY_FDC_REG_STATUS_CONFIG 0x19u
#define SENSORARRAY_FDC_REG_CONFIG 0x1Au
#define SENSORARRAY_FDC_REG_MUX_CONFIG 0x1Bu
#define SENSORARRAY_FDC_REG_DRIVE_CURRENT_CH0 0x1Eu

#define SENSORARRAY_FDC_CONFIG_ACTIVE_CHAN_SHIFT 14u
#define SENSORARRAY_FDC_CONFIG_ACTIVE_CHAN_MASK 0xC000u
#define SENSORARRAY_FDC_CONFIG_SLEEP_MODE_MASK 0x2000u
#define SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK 0x0040u
#define SENSORARRAY_FDC_DRIVE_CURRENT_MASK 0xF800u
#define SENSORARRAY_FDC_CLOCK_DIVIDER_FREF_MASK 0x03FFu

/*
 * Dedicated weak-drive diagnosis table for S5D5/SELB/CH0:
 * sweep DRIVE_CURRENT_CH0 and HIGH_CURRENT_DRV to validate whether
 * low oscillation amplitude is the dominant cause of invalid samples.
 */
static const uint16_t SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[] = {
    0xA000u,
    0xB800u,
    0xC000u,
    0xD000u,
    0xE000u,
    0xF800u,
};

static const bool SENSORARRAY_FDC_HIGH_CURRENT_SWEEP_TABLE[] = {
    false,
    true,
};

typedef struct {
    const sensorarrayRouteMap_t *route;
    const sensorarrayFdcDLineMap_t *fdcMap;
    sensorarrayFdcDeviceState_t *fdcState;
    BoardSupportI2cBusInfo_t busInfo;
    tmux1108Source_t swSource;
    uint8_t rowIndex;
    int selaWriteLevel;
    uint8_t i2cAddr;
    uint32_t sampleCount;
    uint32_t loopDelayMs;
    bool discardFirst;
} sensorarrayFdcSelbS5d5Ctx_t;

typedef struct {
    uint16_t status;
    uint16_t statusConfig;
    uint16_t config;
    uint16_t muxConfig;
    uint16_t rcountCh0;
    uint16_t settleCountCh0;
    uint16_t clockDividersCh0;
    uint16_t driveCurrentCh0;
} sensorarrayFdcStepRegs_t;

typedef struct {
    bool initialized;
    uint32_t totalSamples;
    uint32_t validSamples;
    uint32_t invalidSamples;
    uint32_t i2cErrorCount;
    uint32_t configUnknownCount;
    uint32_t stillSleepingCount;
    uint32_t notConvertingCount;
    uint32_t noUnreadCount;
    uint32_t zeroRawCount;
    uint32_t watchdogCount;
    uint32_t amplitudeFaultCount;
    uint32_t amplitudeFlagAnyCount;
    uint32_t minRaw;
    uint32_t maxRaw;
    uint64_t sumRaw;
    uint32_t minFreqHz;
    uint32_t maxFreqHz;
    uint64_t sumFreqHz;
    uint32_t freqCount;
} sensorarrayFdcStepSummary_t;

typedef enum {
    ROUTE_STATUS_COMMAND_ONLY = 0,
    ROUTE_STATUS_GPIO_CONFIRMED,
    ROUTE_STATUS_GPIO_MISMATCH,
    ROUTE_STATUS_FUNCTIONALLY_CONFIRMED,
} sensorarrayRouteStatus_t;

typedef struct {
    bool ctrlStateReadOk;
    bool commandMatch;
    bool gpioObservedMatch;
    sensorarrayRouteStatus_t routeStatus;
} sensorarrayRouteCheck_t;

static void sensorarrayDelayMs(uint32_t delayMs)
{
    if (delayMs > 0u) {
        vTaskDelay(pdMS_TO_TICKS(delayMs));
    }
}

static const char *sensorarrayFdcChannelName(Fdc2214CapChannel_t ch)
{
    switch (ch) {
    case FDC2214_CH0:
        return "CH0";
    case FDC2214_CH1:
        return "CH1";
    case FDC2214_CH2:
        return "CH2";
    case FDC2214_CH3:
        return "CH3";
    default:
        return "INVALID";
    }
}

static uint32_t sensorarrayFdcSampleCountForBringup(void)
{
    uint32_t count = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_SAMPLE_COUNT;
    if (count < SENSORARRAY_FDC_DBG_MIN_SAMPLES) {
        count = SENSORARRAY_FDC_DBG_MIN_SAMPLES;
    }
    if (count > SENSORARRAY_FDC_DBG_MAX_SAMPLES) {
        count = SENSORARRAY_FDC_DBG_MAX_SAMPLES;
    }
    return count;
}

static uint32_t sensorarrayFdcLoopDelayMsForBringup(void)
{
    uint32_t delayMs = (uint32_t)CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_LOOP_DELAY_MS;
    if (delayMs < SENSORARRAY_FDC_DBG_MIN_LOOP_DELAY_MS) {
        delayMs = SENSORARRAY_FDC_DBG_MIN_LOOP_DELAY_MS;
    }
    return delayMs;
}

static const char *sensorarrayFdcRefClockSourceName(Fdc2214CapRefClockSource_t source)
{
    return (source == FDC2214_REF_CLOCK_EXTERNAL) ? "external_clkin" : "internal_oscillator";
}

static bool sensorarrayFdcEstimateSensorFreqHz(uint32_t raw28,
                                               uint32_t refClockHz,
                                               uint16_t clockDividersCh0,
                                               uint32_t *outHz)
{
    if (!outHz || refClockHz == 0u) {
        return false;
    }

    uint32_t frefDivider = (uint32_t)(clockDividersCh0 & SENSORARRAY_FDC_CLOCK_DIVIDER_FREF_MASK);
    if (frefDivider == 0u) {
        return false;
    }

    uint64_t effectiveRefHz = (uint64_t)refClockHz / (uint64_t)frefDivider;
    uint64_t sensorHz = (((uint64_t)raw28 * effectiveRefHz) + (1ULL << 27)) >> 28;
    if (sensorHz > UINT_MAX) {
        sensorHz = UINT_MAX;
    }
    *outHz = (uint32_t)sensorHz;
    return true;
}

static esp_err_t sensorarrayFdcReadStepRegs(Fdc2214CapDevice_t *dev, sensorarrayFdcStepRegs_t *outRegs)
{
    if (!dev || !outRegs) {
        return ESP_ERR_INVALID_ARG;
    }

    sensorarrayFdcStepRegs_t regs = {0};
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_STATUS, &regs.status);
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_STATUS_CONFIG, &regs.statusConfig);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_CONFIG, &regs.config);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_MUX_CONFIG, &regs.muxConfig);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_RCOUNT_CH0, &regs.rcountCh0);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_SETTLECOUNT_CH0, &regs.settleCountCh0);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_CLOCK_DIVIDERS_CH0, &regs.clockDividersCh0);
    }
    if (err == ESP_OK) {
        err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_DRIVE_CURRENT_CH0, &regs.driveCurrentCh0);
    }
    if (err != ESP_OK) {
        return err;
    }

    *outRegs = regs;
    return ESP_OK;
}

static esp_err_t sensorarrayFdcApplyDriveStep(Fdc2214CapDevice_t *dev,
                                              bool highCurrentDrv,
                                              uint16_t driveCurrentCh0,
                                              sensorarrayFdcStepRegs_t *outRegs)
{
    if (!dev) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t configReg = 0u;
    esp_err_t err = Fdc2214CapReadRawRegisters(dev, SENSORARRAY_FDC_REG_CONFIG, &configReg);
    if (err != ESP_OK) {
        return err;
    }

    configReg &= (uint16_t)~SENSORARRAY_FDC_CONFIG_ACTIVE_CHAN_MASK;
    configReg |= (uint16_t)((uint16_t)FDC2214_CH0 << SENSORARRAY_FDC_CONFIG_ACTIVE_CHAN_SHIFT);
    configReg &= (uint16_t)~SENSORARRAY_FDC_CONFIG_SLEEP_MODE_MASK;
    if (highCurrentDrv) {
        configReg |= SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK;
    } else {
        configReg &= (uint16_t)~SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK;
    }

    err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_FDC_REG_CONFIG, configReg);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t driveMasked = (uint16_t)(driveCurrentCh0 & SENSORARRAY_FDC_DRIVE_CURRENT_MASK);
    err = Fdc2214CapWriteRawRegisters(dev, SENSORARRAY_FDC_REG_DRIVE_CURRENT_CH0, driveMasked);
    if (err != ESP_OK) {
        return err;
    }

    if (!outRegs) {
        return ESP_OK;
    }
    return sensorarrayFdcReadStepRegs(dev, outRegs);
}

static void sensorarrayFdcInitStepSummary(sensorarrayFdcStepSummary_t *summary)
{
    if (!summary) {
        return;
    }

    *summary = (sensorarrayFdcStepSummary_t){
        .initialized = true,
        .minRaw = UINT_MAX,
        .minFreqHz = UINT_MAX,
    };
}

static int sensorarrayFdcCompareStepQuality(const sensorarrayFdcStepSummary_t *current,
                                            const sensorarrayFdcStepSummary_t *previous)
{
    if (!current || !previous || !previous->initialized) {
        return 0;
    }

    if (current->validSamples != previous->validSamples) {
        return (current->validSamples > previous->validSamples) ? 1 : -1;
    }
    if (current->invalidSamples != previous->invalidSamples) {
        return (current->invalidSamples < previous->invalidSamples) ? 1 : -1;
    }
    if (current->amplitudeFaultCount != previous->amplitudeFaultCount) {
        return (current->amplitudeFaultCount < previous->amplitudeFaultCount) ? 1 : -1;
    }
    if (current->noUnreadCount != previous->noUnreadCount) {
        return (current->noUnreadCount < previous->noUnreadCount) ? 1 : -1;
    }
    return 0;
}

static const char *sensorarrayFdcComparisonName(int cmp, bool havePrevious)
{
    if (!havePrevious) {
        return "unchanged";
    }
    if (cmp > 0) {
        return "better";
    }
    if (cmp < 0) {
        return "worse";
    }
    return "unchanged";
}

static const char *sensorarrayRouteStatusName(sensorarrayRouteStatus_t status)
{
    switch (status) {
    case ROUTE_STATUS_COMMAND_ONLY:
        return "ROUTE_STATUS_COMMAND_ONLY";
    case ROUTE_STATUS_GPIO_CONFIRMED:
        return "ROUTE_STATUS_GPIO_CONFIRMED";
    case ROUTE_STATUS_GPIO_MISMATCH:
        return "ROUTE_STATUS_GPIO_MISMATCH";
    case ROUTE_STATUS_FUNCTIONALLY_CONFIRMED:
        return "ROUTE_STATUS_FUNCTIONALLY_CONFIRMED";
    default:
        return "ROUTE_STATUS_UNKNOWN";
    }
}

static int sensorarrayExpectedSwLevel(tmux1108Source_t source)
{
    int refLevel = CONFIG_TMUX1108_SW_REF_LEVEL ? 1 : 0;
    return (source == TMUX1108_SOURCE_REF) ? refLevel : (refLevel ? 0 : 1);
}

static void sensorarrayPromoteRouteStatusWithFunctionalEvidence(sensorarrayRouteCheck_t *routeCheck, bool fdcCommOk)
{
    if (!routeCheck || !fdcCommOk) {
        return;
    }
    if (routeCheck->routeStatus == ROUTE_STATUS_GPIO_MISMATCH) {
        routeCheck->routeStatus = ROUTE_STATUS_FUNCTIONALLY_CONFIRMED;
    }
}

static void sensorarrayLogRouteEvidenceSummary(const sensorarrayFdcSelbS5d5Ctx_t *ctx,
                                               bool routeCommandIssued,
                                               const sensorarrayRouteCheck_t *routeCheck,
                                               bool fdcCommOk,
                                               const char *status)
{
    const char *mapLabel = (ctx && ctx->route && ctx->route->mapLabel) ? ctx->route->mapLabel : SENSORARRAY_NA;
    int ctrlStateRead = routeCheck ? (routeCheck->ctrlStateReadOk ? 1 : 0) : -1;
    int commandMatch = routeCheck ? (routeCheck->commandMatch ? 1 : 0) : -1;
    int gpioObservedMatch = routeCheck ? (routeCheck->gpioObservedMatch ? 1 : 0) : -1;
    const char *routeStatus = routeCheck ? sensorarrayRouteStatusName(routeCheck->routeStatus) : "ROUTE_STATUS_COMMAND_ONLY";

    printf("DBGFDC,stage=diagnosis,point=S5D5,map=%s,routeCommandIssued=%u,ctrlStateRead=%d,commandMatch=%d,"
           "gpioObservedMatch=%d,routeStatus=%s,fdcComm=%u,status=%s,"
           "note=GPIO_observation_is_MCU_side_only_and_not_analog_switch_conduction_proof\n",
           mapLabel,
           routeCommandIssued ? 1u : 0u,
           ctrlStateRead,
           commandMatch,
           gpioObservedMatch,
           routeStatus,
           fdcCommOk ? 1u : 0u,
           status ? status : SENSORARRAY_NA);
}

static void sensorarrayLogFinalDbg(const sensorarrayFdcSelbS5d5Ctx_t *ctx,
                                   const sensorarrayFdcReadDiag_t *diag,
                                   const char *status)
{
    char rawMsbBuf[12];
    char rawLsbBuf[12];
    char rawBuf[20];
    char wdBuf[4];
    char awBuf[4];
    char freqBuf[20];

    const char *mapLabel = (ctx && ctx->route && ctx->route->mapLabel) ? ctx->route->mapLabel : SENSORARRAY_NA;
    int port = (ctx && ctx->fdcState && ctx->fdcState->i2cCtx) ? (int)ctx->fdcState->i2cCtx->Port : -1;
    uint8_t addr = (ctx && ctx->fdcState) ? ctx->fdcState->i2cAddr : SENSORARRAY_FDC_I2C_ADDR_LOW;
    Fdc2214CapChannel_t channel = (ctx && ctx->fdcMap) ? ctx->fdcMap->channel : FDC2214_CH0;

    const char *rawMsbStr = SENSORARRAY_NA;
    const char *rawLsbStr = SENSORARRAY_NA;
    const char *rawStr = SENSORARRAY_NA;
    const char *wdStr = SENSORARRAY_NA;
    const char *awStr = SENSORARRAY_NA;
    const char *freqStr = SENSORARRAY_NA;
    const char *refClockSource = SENSORARRAY_NA;
    bool refClockSourceKnown = false;
    const char *sleepStr = SENSORARRAY_NA;
    const char *autoScanStr = SENSORARRAY_NA;
    const char *convertingStr = SENSORARRAY_NA;
    const char *unreadStr = SENSORARRAY_NA;
    const char *statusRegStr = SENSORARRAY_NA;
    const char *configRegStr = SENSORARRAY_NA;
    const char *muxRegStr = SENSORARRAY_NA;

    char sleepBuf[4];
    char autoScanBuf[4];
    char convertingBuf[4];
    char unreadBuf[4];
    char statusRegBuf[12];
    char configRegBuf[12];
    char muxRegBuf[12];

    if (diag && diag->i2cOk) {
        uint16_t dataMsb = (uint16_t)((diag->sample.Raw28 >> 16) & 0x0FFFu);
        if (diag->sample.ErrWatchdog) {
            dataMsb |= 0x2000u;
        }
        if (diag->sample.ErrAmplitude) {
            dataMsb |= 0x1000u;
        }

        snprintf(rawMsbBuf, sizeof(rawMsbBuf), "0x%04X", dataMsb);
        snprintf(rawLsbBuf, sizeof(rawLsbBuf), "0x%04X", (uint16_t)(diag->sample.Raw28 & 0xFFFFu));
        snprintf(rawBuf, sizeof(rawBuf), "%lu", (unsigned long)diag->sample.Raw28);
        snprintf(wdBuf, sizeof(wdBuf), "%u", diag->sample.ErrWatchdog ? 1u : 0u);
        snprintf(awBuf, sizeof(awBuf), "%u", diag->sample.ErrAmplitude ? 1u : 0u);
        snprintf(sleepBuf, sizeof(sleepBuf), "%u", diag->sample.SleepModeEnabled ? 1u : 0u);
        snprintf(autoScanBuf, sizeof(autoScanBuf), "%u", diag->sample.AutoScanEnabled ? 1u : 0u);
        snprintf(convertingBuf, sizeof(convertingBuf), "%u", diag->sample.Converting ? 1u : 0u);
        snprintf(unreadBuf, sizeof(unreadBuf), "%u", diag->sample.UnreadConversionPresent ? 1u : 0u);
        snprintf(statusRegBuf, sizeof(statusRegBuf), "0x%04X", diag->sample.StatusRaw);
        snprintf(configRegBuf, sizeof(configRegBuf), "0x%04X", diag->sample.ConfigRaw);
        snprintf(muxRegBuf, sizeof(muxRegBuf), "0x%04X", diag->sample.MuxRaw);

        rawMsbStr = rawMsbBuf;
        rawLsbStr = rawLsbBuf;
        rawStr = rawBuf;
        wdStr = wdBuf;
        awStr = awBuf;
        sleepStr = sleepBuf;
        autoScanStr = autoScanBuf;
        convertingStr = convertingBuf;
        unreadStr = unreadBuf;
        statusRegStr = statusRegBuf;
        configRegStr = configRegBuf;
        muxRegStr = muxRegBuf;
        refClockSource = (diag->sample.RefClockSource == FDC2214_REF_CLOCK_EXTERNAL) ? "external_clkin" : "internal_oscillator";
        refClockSourceKnown = true;
    }

    if (ctx && ctx->fdcState && ctx->fdcState->refClockKnown) {
        snprintf(freqBuf, sizeof(freqBuf), "%lu", (unsigned long)ctx->fdcState->refClockHz);
        freqStr = freqBuf;
        if (!refClockSourceKnown) {
            refClockSource = (ctx->fdcState->refClockSource == FDC2214_REF_CLOCK_EXTERNAL)
                                 ? "external_clkin"
                                 : "internal_oscillator";
            refClockSourceKnown = true;
        }
    }

    printf("DBG,point=S5D5,kind=cap,mode=fdc,map=%s,fdcDev=SELB,i2cPort=%d,i2cAddr=0x%02X,channel=%s,"
           "rawMsb=%s,rawLsb=%s,raw=%s,freqHz=%s,refClockSource=%s,capPf=na(reason=missing_l_or_scale),wd=%s,"
           "amp=%s,sleep=%s,autoscan=%s,converting=%s,unread=%s,statusReg=%s,configReg=%s,muxReg=%s,status=%s\n",
           mapLabel,
           port,
           addr,
           sensorarrayFdcChannelName(channel),
           rawMsbStr,
           rawLsbStr,
           rawStr,
           freqStr,
           refClockSource,
           wdStr,
           awStr,
           sleepStr,
           autoScanStr,
           convertingStr,
           unreadStr,
           statusRegStr,
           configRegStr,
           muxRegStr,
           status ? status : SENSORARRAY_NA);
}

static sensorarrayRouteCheck_t sensorarrayVerifyRouteAndLog(const sensorarrayFdcSelbS5d5Ctx_t *ctx)
{
    sensorarrayRouteCheck_t routeCheck = {
        .ctrlStateReadOk = false,
        .commandMatch = false,
        .gpioObservedMatch = false,
        .routeStatus = ROUTE_STATUS_COMMAND_ONLY,
    };
    if (!ctx) {
        return routeCheck;
    }

    tmuxSwitchControlState_t ctrl = {0};
    if (tmuxSwitchGetControlState(&ctrl) != ESP_OK) {
        printf("DBGFDC,stage=route_verify,point=S5D5,commandMatch=0,gpioObservedMatch=0,routeStatus=%s,"
               "status=ctrl_state_unavailable\n",
               sensorarrayRouteStatusName(routeCheck.routeStatus));
        sensorarrayLogControlGpio("route_verify", "S5D5");
        return routeCheck;
    }

    routeCheck.ctrlStateReadOk = true;
    int expectedA0 = (int)(ctx->rowIndex & 0x1u);
    int expectedA1 = (int)((ctx->rowIndex >> 1u) & 0x1u);
    int expectedA2 = (int)((ctx->rowIndex >> 2u) & 0x1u);
    int expectedSelB = (ctx->route && ctx->route->selBLevel) ? 1 : 0;
    int expectedSw = sensorarrayExpectedSwLevel(ctx->swSource);

    /*
     * cmd* fields are software-command metadata; obs* fields are MCU GPIO samples.
     * Neither is definitive analog-path proof, so functional FDC communication is
     * treated as stronger evidence when route status is ambiguous.
     */
    bool rowCommandMatch = (ctrl.cmdRow == ctx->rowIndex) &&
                           (ctrl.cmdA0Level == expectedA0) &&
                           (ctrl.cmdA1Level == expectedA1) &&
                           (ctrl.cmdA2Level == expectedA2);
    bool selaCommandMatch = (ctrl.cmdSelaLevel == ctx->selaWriteLevel);
    bool selbCommandMatch = (ctrl.cmdSelbLevel == expectedSelB);
    bool swCommandMatch = (ctrl.cmdSource == ctx->swSource) && (ctrl.cmdSwLevel == expectedSw);
    routeCheck.commandMatch = rowCommandMatch && selaCommandMatch && selbCommandMatch && swCommandMatch;

    bool rowObservedMatch = (ctrl.obsA0Level == expectedA0) &&
                            (ctrl.obsA1Level == expectedA1) &&
                            (ctrl.obsA2Level == expectedA2);
    bool selaObservedMatch = (ctrl.obsSelaLevel == ctx->selaWriteLevel);
    bool selbObservedMatch = (ctrl.obsSelbLevel == expectedSelB);
    bool swObservedMatch = (ctrl.obsSwLevel == expectedSw);
    routeCheck.gpioObservedMatch = rowObservedMatch && selaObservedMatch && selbObservedMatch && swObservedMatch;

    if (routeCheck.commandMatch && routeCheck.gpioObservedMatch) {
        routeCheck.routeStatus = ROUTE_STATUS_GPIO_CONFIRMED;
    } else if (routeCheck.commandMatch) {
        routeCheck.routeStatus = ROUTE_STATUS_GPIO_MISMATCH;
    } else {
        routeCheck.routeStatus = ROUTE_STATUS_COMMAND_ONLY;
    }

    const char *status = routeCheck.commandMatch
                             ? (routeCheck.gpioObservedMatch ? "route_state_ok" : "warn_gpio_mismatch")
                             : "warn_command_mismatch";
    printf("DBGFDC,stage=route_verify,point=S5D5,row=%u,expectedA0=%d,expectedA1=%d,expectedA2=%d,expectedSELA=%d,"
           "expectedSELB=%d,expectedSW=%d,cmdA0=%d,cmdA1=%d,cmdA2=%d,cmdSELA=%d,cmdSELB=%d,cmdSW=%d,obsA0=%d,"
           "obsA1=%d,obsA2=%d,obsSELA=%d,obsSELB=%d,obsSW=%d,commandMatch=%u,gpioObservedMatch=%u,routeStatus=%s,"
           "status=%s\n",
           (unsigned)ctx->rowIndex,
           expectedA0,
           expectedA1,
           expectedA2,
           ctx->selaWriteLevel,
           expectedSelB,
           expectedSw,
           ctrl.cmdA0Level,
           ctrl.cmdA1Level,
           ctrl.cmdA2Level,
           ctrl.cmdSelaLevel,
           ctrl.cmdSelbLevel,
           ctrl.cmdSwLevel,
           ctrl.obsA0Level,
           ctrl.obsA1Level,
           ctrl.obsA2Level,
           ctrl.obsSelaLevel,
           ctrl.obsSelbLevel,
           ctrl.obsSwLevel,
           routeCheck.commandMatch ? 1u : 0u,
           routeCheck.gpioObservedMatch ? 1u : 0u,
           sensorarrayRouteStatusName(routeCheck.routeStatus),
           status);

    if (!routeCheck.gpioObservedMatch) {
        printf("DBGFDC,stage=route_verify_warn,point=S5D5,detail=gpio_observed_mismatch_continuing_with_functional_probe\n");
    }
    if (!routeCheck.commandMatch) {
        printf("DBGFDC,stage=route_verify_warn,point=S5D5,detail=command_state_mismatch_continuing_with_functional_probe\n");
    }

    sensorarrayLogControlGpio("route_verify", "S5D5");
    return routeCheck;
}

static void sensorarrayHoldFailure(const char *reason)
{
    sensorarrayDebugIdleForever(reason ? reason : "fdc_selb_s5d5_failure");
}

static void sensorarrayAbortDebugRun(const sensorarrayFdcSelbS5d5Ctx_t *ctx,
                                     bool routeCommandIssued,
                                     const sensorarrayRouteCheck_t *routeCheck,
                                     bool fdcCommOk,
                                     const char *status,
                                     const char *holdReason)
{
    sensorarrayLogRouteEvidenceSummary(ctx, routeCommandIssued, routeCheck, fdcCommOk, status);
    sensorarrayLogFinalDbg(ctx, NULL, status);
    sensorarrayHoldFailure(holdReason);
}

void sensorarrayDebugRunTestFdc2214SelbS5D5(sensorarrayState_t *state)
{
    sensorarrayFdcSelbS5d5Ctx_t ctx = {
        .route = NULL,
        .fdcMap = NULL,
        .fdcState = NULL,
        .busInfo = {0},
        .swSource = TMUX1108_SOURCE_REF,
        // TMUX1108 uses 0-based row coding. S5 => row index 4 => A2/A1/A0 = 1/0/0.
        .rowIndex = (uint8_t)(SENSORARRAY_S5 - 1u),
        .selaWriteLevel = -1,
        .i2cAddr = SENSORARRAY_FDC_I2C_ADDR_LOW,
        .sampleCount = sensorarrayFdcSampleCountForBringup(),
        .loopDelayMs = sensorarrayFdcLoopDelayMsForBringup(),
        .discardFirst = (CONFIG_SENSORARRAY_DEBUG_CAP_FDC_SECONDARY_DISCARD_FIRST != 0),
    };
    sensorarrayRouteCheck_t routeCheck = {
        .ctrlStateReadOk = false,
        .commandMatch = false,
        .gpioObservedMatch = false,
        .routeStatus = ROUTE_STATUS_COMMAND_ONLY,
    };
    bool routeCommandIssued = false;
    bool fdcCommOk = false;

    printf("DBGFDC,stage=target,point=S5D5,kind=cap,mode=S5D5_CAP_FDC_SECONDARY,fdcDev=SELB,sColumn=%u,dLine=%u,"
           "i2cAddr=0x%02X,sda=%d,scl=%d,samplesPerLoop=%lu,loopDelayMs=%lu,discardFirst=%u,status=begin\n",
           (unsigned)SENSORARRAY_S5,
           (unsigned)SENSORARRAY_D5,
           ctx.i2cAddr,
           SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO,
           SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO,
           (unsigned long)ctx.sampleCount,
           (unsigned long)ctx.loopDelayMs,
           ctx.discardFirst ? 1u : 0u);
    printf("DBGFDC_SWEEP,stage=plan,point=S5D5,highCurrentStates=0|1,driveCurrentList=0xA000|0xB800|0xC000|0xD000|"
           "0xE000|0xF800,settleMs=%u,discardPerStep=%u,samplesPerStep=%lu,goal=stabilize_valid_raw_not_cap_scale\n",
           (unsigned)SENSORARRAY_FDC_DBG_STEP_SETTLE_MS,
           (unsigned)(SENSORARRAY_FDC_DBG_STEP_DISCARD_COUNT + (ctx.discardFirst ? 1u : 0u)),
           (unsigned long)ctx.sampleCount);

    if (!state || !state->boardReady || !state->tmuxReady) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=state_not_ready,detail=state_not_ready\n");
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "state_not_ready",
                                 "fdc_selb_s5d5_state_not_ready");
        return;
    }

    ctx.route = sensorarrayBoardMapFindRoute(SENSORARRAY_S5, SENSORARRAY_D5, SENSORARRAY_PATH_CAPACITIVE);
    ctx.fdcMap = sensorarrayBoardMapFindFdcByDLine(SENSORARRAY_D5);
    ctx.fdcState = sensorarrayMeasureGetFdcState(state, SENSORARRAY_FDC_DEV_SECONDARY);
    if (!ctx.route || !ctx.fdcMap || !ctx.fdcState) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=route_or_fdc_state_missing,detail=route_or_fdc_state_missing\n");
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "route_or_fdc_state_missing",
                                 "fdc_selb_s5d5_map_missing");
        return;
    }

    if (ctx.fdcMap->devId != SENSORARRAY_FDC_DEV_SECONDARY) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=invalid_channel_map,detail=d5_not_secondary\n");
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "invalid_channel_map",
                                 "fdc_selb_s5d5_invalid_dline_owner");
        return;
    }
    if (ctx.fdcMap->channel != FDC2214_CH0) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,status=invalid_channel_map,detail=d5_expected_ch0_actual_%u\n",
               (unsigned)ctx.fdcMap->channel);
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "invalid_channel_map",
                                 "fdc_selb_s5d5_invalid_channel");
        return;
    }

    if (!sensorarrayBoardMapSelaRouteToGpioLevel(ctx.route->selaRoute, &ctx.selaWriteLevel)) {
        printf("DBGFDC,stage=selA,point=S5D5,status=sela_route_invalid,detail=sela_route_invalid\n");
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "sela_route_invalid",
                                 "fdc_selb_s5d5_sela_invalid");
        return;
    }
    if (ctx.route->selaRoute != SENSORARRAY_SELA_ROUTE_FDC2214 || !ctx.route->selBLevel) {
        printf("DBGFDC,stage=selA,point=S5D5,status=route_semantic_mismatch,detail=expected_cap_route_sela_fdc_selb_high\n");
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "route_semantic_mismatch",
                                 "fdc_selb_s5d5_route_semantic_mismatch");
        return;
    }

    ctx.swSource = sensorarrayBoardMapDefaultSwSource(ctx.route);
    if (ctx.swSource != TMUX1108_SOURCE_REF) {
        printf("DBGFDC,stage=sw,point=S5D5,status=sw_source_invalid,detail=cap_should_use_ref\n");
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "sw_source_invalid",
                                 "fdc_selb_s5d5_sw_invalid");
        return;
    }

    const BoardSupportI2cCtx_t *secondaryCtx = boardSupportGetI2c1Ctx();
    (void)boardSupportGetI2cBusInfo(true, &ctx.busInfo);
    ctx.fdcState->i2cCtx = secondaryCtx;
    ctx.fdcState->i2cAddr = ctx.i2cAddr;

    if (!ctx.busInfo.Enabled ||
        !ctx.fdcState->i2cCtx ||
        ctx.busInfo.SdaGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SDA_GPIO ||
        ctx.busInfo.SclGpio != SENSORARRAY_SECONDARY_I2C_EXPECTED_SCL_GPIO) {
        printf("DBGFDC,stage=fdc_probe,point=S5D5,fdcDev=SELB,i2cPort=%d,sda=%d,scl=%d,freqHz=%lu,timeoutMs=%lu,"
               "i2cAddr=0x%02X,status=no_i2c_ack\n",
               (int)ctx.busInfo.Port,
               ctx.busInfo.SdaGpio,
               ctx.busInfo.SclGpio,
               (unsigned long)ctx.busInfo.FrequencyHz,
               (unsigned long)(ctx.fdcState->i2cCtx ? ctx.fdcState->i2cCtx->TimeoutMs : 0u),
               ctx.i2cAddr);
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "no_i2c_ack",
                                 "fdc_selb_s5d5_i2c_bus_invalid");
        return;
    }

    esp_err_t adsErr = ESP_OK;
    const char *adsStatus = "ads_already_stopped";
    if (state->adsReady && state->adsAdc1Running) {
        adsErr = ads126xAdcStopAdc1(&state->ads);
        if (adsErr == ESP_OK) {
            state->adsAdc1Running = false;
            adsStatus = "ads_stopped";
        } else {
            adsStatus = "ads_stop_failed";
        }
    }
    printf("DBGFDC,stage=ads_isolation,point=S5D5,err=%ld,status=%s\n", (long)adsErr, adsStatus);
    if (adsErr != ESP_OK) {
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 NULL,
                                 fdcCommOk,
                                 "ads_stop_failed",
                                 "fdc_selb_s5d5_ads_stop_fail");
        return;
    }

    const char *routeLabel = SENSORARRAY_NA;
    esp_err_t routeErr = sensorarrayMeasureApplyRoute(state,
                                                      SENSORARRAY_S5,
                                                      SENSORARRAY_D5,
                                                      SENSORARRAY_PATH_CAPACITIVE,
                                                      ctx.swSource,
                                                      &routeLabel);
    routeCommandIssued = (routeErr == ESP_OK);
    if (routeErr != ESP_OK) {
        printf("DBGFDC,stage=row,point=S5D5,map=%s,err=%ld,status=route_apply_failed\n",
               routeLabel ? routeLabel : SENSORARRAY_NA,
               (long)routeErr);
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 &routeCheck,
                                 fdcCommOk,
                                 "route_apply_failed",
                                 "fdc_selb_s5d5_route_apply_fail");
        return;
    }

    routeCheck = sensorarrayVerifyRouteAndLog(&ctx);
    if (!routeCheck.commandMatch || !routeCheck.gpioObservedMatch) {
        printf("DBGFDC,stage=route_verify,point=S5D5,commandMatch=%u,gpioObservedMatch=%u,status=warning_continue_to_fdc_probe\n",
               routeCheck.commandMatch ? 1u : 0u,
               routeCheck.gpioObservedMatch ? 1u : 0u);
    }

    printf("DBGFDC,stage=fdc_probe,point=S5D5,fdcDev=SELB,i2cPort=%d,sda=%d,scl=%d,freqHz=%lu,timeoutMs=%lu,"
           "i2cAddr=0x%02X,policy=force_0x2A_no_auto_switch,status=begin\n",
           (int)ctx.fdcState->i2cCtx->Port,
           ctx.busInfo.SdaGpio,
           ctx.busInfo.SclGpio,
           (unsigned long)ctx.busInfo.FrequencyHz,
           (unsigned long)ctx.fdcState->i2cCtx->TimeoutMs,
           ctx.i2cAddr);

    esp_err_t ackErr = boardSupportI2cProbeAddress(ctx.fdcState->i2cCtx, ctx.i2cAddr);
    printf("DBGFDC,stage=fdc_ack,point=S5D5,fdcDev=SELB,i2cPort=%d,i2cAddr=0x%02X,err=%ld,status=%s\n",
           (int)ctx.fdcState->i2cCtx->Port,
           ctx.i2cAddr,
           (long)ackErr,
           (ackErr == ESP_OK) ? "ack_ok" : "no_i2c_ack");
    if (ackErr != ESP_OK) {
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 &routeCheck,
                                 fdcCommOk,
                                 "no_i2c_ack",
                                 "fdc_selb_s5d5_no_i2c_ack");
        return;
    }
    fdcCommOk = true;

    uint16_t idMfg = 0u;
    uint16_t idDev = 0u;
    esp_err_t idErr = sensorarrayBringupReadFdcIdsRaw(ctx.fdcState->i2cCtx, ctx.i2cAddr, &idMfg, &idDev);
    bool idMatch = (idMfg == SENSORARRAY_FDC_EXPECTED_MANUFACTURER_ID) &&
                   (idDev == SENSORARRAY_FDC_EXPECTED_DEVICE_ID);
    const char *idStatus = ((idErr == ESP_OK) && idMatch) ? "id_ok" : "bad_device_id";
    const char *idDetail = (idErr != ESP_OK) ? "id_read_failed" : (idMatch ? "id_match" : "id_mismatch");
    printf("DBGFDC,stage=fdc_id,point=S5D5,fdcDev=SELB,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,err=%ld,detail=%s,"
           "status=%s\n",
           ctx.i2cAddr,
           idMfg,
           idDev,
           (long)idErr,
           idDetail,
           idStatus);
    if (idErr != ESP_OK || !idMatch) {
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 &routeCheck,
                                 fdcCommOk,
                                 "bad_device_id",
                                 "fdc_selb_s5d5_bad_device_id");
        return;
    }
    sensorarrayPromoteRouteStatusWithFunctionalEvidence(&routeCheck, fdcCommOk);

    if (ctx.fdcState->handle) {
        (void)Fdc2214CapDestroy(ctx.fdcState->handle);
        ctx.fdcState->handle = NULL;
        ctx.fdcState->ready = false;
    }

    sensorarrayFdcInitDiag_t initDiag = {0};
    esp_err_t initErr = sensorarrayBringupInitFdcSingleChannel(ctx.fdcState->i2cCtx,
                                                                ctx.i2cAddr,
                                                                ctx.fdcMap->channel,
                                                                &ctx.fdcState->handle,
                                                                &initDiag);
    ctx.fdcState->ready = (initErr == ESP_OK);
    ctx.fdcState->haveIds = initDiag.haveIds;
    ctx.fdcState->manufacturerId = initDiag.manufacturerId;
    ctx.fdcState->deviceId = initDiag.deviceId;
    ctx.fdcState->configVerified = initDiag.configVerified;
    ctx.fdcState->refClockKnown = initDiag.refClockKnown;
    ctx.fdcState->refClockSource = initDiag.refClockSource;
    ctx.fdcState->refClockHz = initDiag.refClockHz;
    ctx.fdcState->statusConfigReg = initDiag.statusConfigReg;
    ctx.fdcState->configReg = initDiag.configReg;
    ctx.fdcState->muxConfigReg = initDiag.muxConfigReg;

    const char *initStatus = (initErr == ESP_OK) ? "init_ok" : (initDiag.status ? initDiag.status : "init_failed");
    printf("DBGFDC,stage=fdc_init,point=S5D5,fdcDev=SELB,i2cAddr=0x%02X,channel=%s,idMfg=0x%04X,idDev=0x%04X,"
           "detail=%ld,err=%ld,status=%s\n",
           ctx.i2cAddr,
           sensorarrayFdcChannelName(ctx.fdcMap->channel),
           initDiag.manufacturerId,
           initDiag.deviceId,
           (long)initDiag.detail,
           (long)initErr,
           initStatus);
    if (initErr != ESP_OK || !ctx.fdcState->ready || !ctx.fdcState->handle) {
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 &routeCheck,
                                 fdcCommOk,
                                 initStatus,
                                 "fdc_selb_s5d5_init_fail");
        return;
    }

    fdcCommOk = true;
    sensorarrayPromoteRouteStatusWithFunctionalEvidence(&routeCheck, fdcCommOk);
    sensorarrayLogRouteEvidenceSummary(&ctx, routeCommandIssued, &routeCheck, fdcCommOk, "fdc_init_ok");

    Fdc2214CapCoreRegs_t coreRegs = {0};
    esp_err_t coreErr = Fdc2214CapReadCoreRegs(ctx.fdcState->handle, &coreRegs);
    printf("DBGFDC,stage=core_dump,point=S5D5,fdcDev=SELB,i2cPort=%d,i2cAddr=0x%02X,idMfg=0x%04X,idDev=0x%04X,"
           "status=0x%04X,statusConfig=0x%04X,config=0x%04X,muxConfig=0x%04X,refClockKnown=%u,refClock=%s,"
           "refClockHz=%lu,err=%ld,status=%s\n",
           (int)ctx.fdcState->i2cCtx->Port,
           ctx.i2cAddr,
           ctx.fdcState->manufacturerId,
           ctx.fdcState->deviceId,
           coreRegs.Status,
           coreRegs.StatusConfig,
           coreRegs.Config,
           coreRegs.MuxConfig,
           ctx.fdcState->refClockKnown ? 1u : 0u,
           (ctx.fdcState->refClockSource == FDC2214_REF_CLOCK_EXTERNAL) ? "external_clkin" : "internal_oscillator",
           (unsigned long)ctx.fdcState->refClockHz,
           (long)coreErr,
           (coreErr == ESP_OK) ? "core_regs_ok" : "core_regs_read_error");
    if (coreErr != ESP_OK) {
        sensorarrayAbortDebugRun(&ctx,
                                 routeCommandIssued,
                                 &routeCheck,
                                 fdcCommOk,
                                 "core_regs_read_error",
                                 "fdc_selb_s5d5_core_regs_read_error");
        return;
    }

    /*
     * Drive-strength debug sweep:
     * keep S5D5 route fixed and only vary HIGH_CURRENT_DRV + DRIVE_CURRENT_CH0
     * to diagnose weak oscillation amplitude vs conversion validity.
     */
    uint32_t sweepCycle = 0u;
    uint32_t stepGlobalIndex = 0u;
    sensorarrayFdcStepSummary_t previousSummary = {0};
    bool havePreviousSummary = false;

    while (true) {
        for (size_t highIndex = 0u;
             highIndex < (sizeof(SENSORARRAY_FDC_HIGH_CURRENT_SWEEP_TABLE) / sizeof(SENSORARRAY_FDC_HIGH_CURRENT_SWEEP_TABLE[0]));
             ++highIndex) {
            bool highCurrentDrv = SENSORARRAY_FDC_HIGH_CURRENT_SWEEP_TABLE[highIndex];
            for (size_t driveIndex = 0u;
                 driveIndex < (sizeof(SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE) / sizeof(SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[0]));
                 ++driveIndex) {
                uint16_t driveCurrentReq = SENSORARRAY_FDC_DRIVE_CURRENT_SWEEP_TABLE[driveIndex];
                uint16_t driveCurrentMasked = (uint16_t)(driveCurrentReq & SENSORARRAY_FDC_DRIVE_CURRENT_MASK);
                sensorarrayFdcStepRegs_t regs = {0};
                esp_err_t stepErr = sensorarrayFdcApplyDriveStep(ctx.fdcState->handle,
                                                                  highCurrentDrv,
                                                                  driveCurrentReq,
                                                                  &regs);
                if (stepErr != ESP_OK) {
                    printf("DBGFDC,stage=drive_step,point=S5D5,stepGlobal=%lu,sweepCycle=%lu,highCurrentDrv=%u,"
                           "driveCurrentReq=0x%04X,err=%ld,status=drive_setting_write_failed\n",
                           (unsigned long)stepGlobalIndex,
                           (unsigned long)sweepCycle,
                           highCurrentDrv ? 1u : 0u,
                           driveCurrentReq,
                           (long)stepErr);
                    sensorarrayAbortDebugRun(&ctx,
                                             routeCommandIssued,
                                             &routeCheck,
                                             fdcCommOk,
                                             "drive_setting_write_failed",
                                             "fdc_selb_s5d5_drive_write_failed");
                    return;
                }

                bool highCurrentReadback = (regs.config & SENSORARRAY_FDC_CONFIG_HIGH_CURRENT_DRV_MASK) != 0u;
                uint8_t activeChannelReadback =
                    (uint8_t)((regs.config & SENSORARRAY_FDC_CONFIG_ACTIVE_CHAN_MASK) >> SENSORARRAY_FDC_CONFIG_ACTIVE_CHAN_SHIFT);
                bool readbackMismatch = (highCurrentReadback != highCurrentDrv) ||
                                        (regs.driveCurrentCh0 != driveCurrentMasked) ||
                                        (activeChannelReadback != (uint8_t)ctx.fdcMap->channel);

                printf("DBGFDC_SWEEP,stage=step_header,point=S5D5,sweepCycle=%lu,stepGlobal=%lu,highIndex=%lu,"
                       "driveIndex=%lu,bus=secondary_i2c,port=%d,sda=%d,scl=%d,i2cFreqHz=%lu,i2cAddr=0x%02X,"
                       "channel=%s,highCurrentDrvReq=%u,highCurrentDrvReadback=%u,driveCurrentReq=0x%04X,"
                       "driveCurrentReg=0x%04X,configReg=0x%04X,muxConfig=0x%04X,statusConfig=0x%04X,"
                       "statusReg=0x%04X,rcountCh0=0x%04X,settleCountCh0=0x%04X,clockDividersCh0=0x%04X,"
                       "refClockHz=%lu,refClockSource=%s,readbackMatch=%u,status=%s\n",
                       (unsigned long)sweepCycle,
                       (unsigned long)stepGlobalIndex,
                       (unsigned long)highIndex,
                       (unsigned long)driveIndex,
                       (int)ctx.fdcState->i2cCtx->Port,
                       ctx.busInfo.SdaGpio,
                       ctx.busInfo.SclGpio,
                       (unsigned long)ctx.busInfo.FrequencyHz,
                       ctx.i2cAddr,
                       sensorarrayFdcChannelName(ctx.fdcMap->channel),
                       highCurrentDrv ? 1u : 0u,
                       highCurrentReadback ? 1u : 0u,
                       driveCurrentReq,
                       regs.driveCurrentCh0,
                       regs.config,
                       regs.muxConfig,
                       regs.statusConfig,
                       regs.status,
                       regs.rcountCh0,
                       regs.settleCountCh0,
                       regs.clockDividersCh0,
                       (unsigned long)ctx.fdcState->refClockHz,
                       sensorarrayFdcRefClockSourceName(ctx.fdcState->refClockSource),
                       readbackMismatch ? 0u : 1u,
                       readbackMismatch ? "drive_setting_readback_mismatch" : "step_configured");
                if (readbackMismatch) {
                    sensorarrayAbortDebugRun(&ctx,
                                             routeCommandIssued,
                                             &routeCheck,
                                             fdcCommOk,
                                             "drive_setting_readback_mismatch",
                                             "fdc_selb_s5d5_drive_readback_mismatch");
                    return;
                }

                sensorarrayDelayMs(SENSORARRAY_FDC_DBG_STEP_SETTLE_MS);

                uint32_t discardCount = SENSORARRAY_FDC_DBG_STEP_DISCARD_COUNT + (ctx.discardFirst ? 1u : 0u);
                for (uint32_t discardIndex = 0u; discardIndex < discardCount; ++discardIndex) {
                    sensorarrayFdcReadDiag_t discardDiag = {0};
                    esp_err_t discardErr = sensorarrayMeasureReadFdcSampleDiag(ctx.fdcState->handle,
                                                                               ctx.fdcMap->channel,
                                                                               false,
                                                                               ctx.fdcState->haveIds,
                                                                               ctx.fdcState->configVerified,
                                                                               &discardDiag);
                    const char *discardStatus = sensorarrayMeasureFdcSampleStatusName(discardDiag.statusCode);
                    printf("DBGFDC_SWEEP,stage=discard,point=S5D5,sweepCycle=%lu,stepGlobal=%lu,discardIndex=%lu,"
                           "discardCount=%lu,statusReg=0x%04X,converting=%u,unread=%u,raw=%lu,err=%ld,status=%s\n",
                           (unsigned long)sweepCycle,
                           (unsigned long)stepGlobalIndex,
                           (unsigned long)discardIndex,
                           (unsigned long)discardCount,
                           discardDiag.sample.StatusRaw,
                           discardDiag.converting ? 1u : 0u,
                           discardDiag.unreadConversionPresent ? 1u : 0u,
                           (unsigned long)discardDiag.sample.Raw28,
                           (long)discardErr,
                           discardStatus);
                    sensorarrayDelayMs(SENSORARRAY_FDC_DBG_SAMPLE_DELAY_MS);
                }

                sensorarrayFdcStepSummary_t summary = {0};
                sensorarrayFdcInitStepSummary(&summary);

                for (uint32_t sampleIndex = 0u; sampleIndex < ctx.sampleCount; ++sampleIndex) {
                    sensorarrayFdcReadDiag_t diag = {0};
                    esp_err_t readErr = sensorarrayMeasureReadFdcSampleDiag(ctx.fdcState->handle,
                                                                            ctx.fdcMap->channel,
                                                                            false,
                                                                            ctx.fdcState->haveIds,
                                                                            ctx.fdcState->configVerified,
                                                                            &diag);
                    const char *sampleStatus = sensorarrayMeasureFdcSampleStatusName(diag.statusCode);

                    bool watchdogFlag = diag.sample.ErrWatchdog || diag.status.ErrWatchdog;
                    bool amplitudeFlag = diag.sample.ErrAmplitude ||
                                         diag.status.ErrAmplitudeHigh ||
                                         diag.status.ErrAmplitudeLow;
                    if (watchdogFlag) {
                        summary.watchdogCount++;
                    }
                    if (amplitudeFlag) {
                        summary.amplitudeFlagAnyCount++;
                    }

                    summary.totalSamples++;
                    switch (diag.statusCode) {
                    case SENSORARRAY_FDC_SAMPLE_STATUS_SAMPLE_VALID:
                        summary.validSamples++;
                        if (diag.sample.Raw28 < summary.minRaw) {
                            summary.minRaw = diag.sample.Raw28;
                        }
                        if (diag.sample.Raw28 > summary.maxRaw) {
                            summary.maxRaw = diag.sample.Raw28;
                        }
                        summary.sumRaw += (uint64_t)diag.sample.Raw28;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_ERROR:
                        summary.i2cErrorCount++;
                        summary.invalidSamples++;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_CONFIG_UNKNOWN:
                        summary.configUnknownCount++;
                        summary.invalidSamples++;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_STILL_SLEEPING:
                        summary.stillSleepingCount++;
                        summary.invalidSamples++;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_I2C_READ_OK_BUT_NOT_CONVERTING:
                        summary.notConvertingCount++;
                        summary.invalidSamples++;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_NO_UNREAD_CONVERSION:
                        summary.noUnreadCount++;
                        summary.invalidSamples++;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_ZERO_RAW_INVALID:
                        summary.zeroRawCount++;
                        summary.invalidSamples++;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_WATCHDOG_FAULT:
                        summary.invalidSamples++;
                        break;
                    case SENSORARRAY_FDC_SAMPLE_STATUS_AMPLITUDE_FAULT:
                        summary.amplitudeFaultCount++;
                        summary.invalidSamples++;
                        break;
                    default:
                        summary.configUnknownCount++;
                        summary.invalidSamples++;
                        break;
                    }

                    uint16_t rawMsb = (uint16_t)((diag.sample.Raw28 >> 16) & 0x0FFFu);
                    uint16_t rawLsb = (uint16_t)(diag.sample.Raw28 & 0xFFFFu);
                    char freqBuf[20];
                    const char *freqStr = SENSORARRAY_NA;
                    uint32_t sensorFreqHz = 0u;
                    bool haveSensorFreq = sensorarrayFdcEstimateSensorFreqHz(diag.sample.Raw28,
                                                                             ctx.fdcState->refClockHz,
                                                                             regs.clockDividersCh0,
                                                                             &sensorFreqHz);
                    if (haveSensorFreq) {
                        snprintf(freqBuf, sizeof(freqBuf), "%lu", (unsigned long)sensorFreqHz);
                        freqStr = freqBuf;
                        if (sensorFreqHz < summary.minFreqHz) {
                            summary.minFreqHz = sensorFreqHz;
                        }
                        if (sensorFreqHz > summary.maxFreqHz) {
                            summary.maxFreqHz = sensorFreqHz;
                        }
                        summary.sumFreqHz += sensorFreqHz;
                        summary.freqCount++;
                    }

                    printf("DBGFDC_SWEEP,stage=sample,point=S5D5,sweepCycle=%lu,stepGlobal=%lu,sampleIndex=%lu,"
                           "highCurrentDrv=%u,driveCurrent=0x%04X,statusReg=0x%04X,converting=%u,unread=%u,"
                           "drdy=%u,errChan=%u,ampHigh=%u,ampLow=%u,amplitudeFault=%u,watchdogFault=%u,"
                           "raw=%lu,rawMsb=0x%03X,rawLsb=0x%04X,sensorFreqHz=%s,configReg=0x%04X,muxReg=0x%04X,"
                           "i2cOk=%u,idOk=%u,configOk=%u,valid=%u,err=%ld,status=%s\n",
                           (unsigned long)sweepCycle,
                           (unsigned long)stepGlobalIndex,
                           (unsigned long)sampleIndex,
                           highCurrentDrv ? 1u : 0u,
                           regs.driveCurrentCh0,
                           diag.sample.StatusRaw,
                           diag.converting ? 1u : 0u,
                           diag.unreadConversionPresent ? 1u : 0u,
                           diag.status.DataReady ? 1u : 0u,
                           (unsigned)diag.status.ErrorChannel,
                           diag.status.ErrAmplitudeHigh ? 1u : 0u,
                           diag.status.ErrAmplitudeLow ? 1u : 0u,
                           amplitudeFlag ? 1u : 0u,
                           watchdogFlag ? 1u : 0u,
                           (unsigned long)diag.sample.Raw28,
                           rawMsb,
                           rawLsb,
                           freqStr,
                           diag.sample.ConfigRaw,
                           diag.sample.MuxRaw,
                           diag.i2cOk ? 1u : 0u,
                           diag.idOk ? 1u : 0u,
                           diag.configOk ? 1u : 0u,
                           diag.sampleValid ? 1u : 0u,
                           (long)readErr,
                           sampleStatus);
                    sensorarrayDelayMs(SENSORARRAY_FDC_DBG_SAMPLE_DELAY_MS);
                }

                int cmp = sensorarrayFdcCompareStepQuality(&summary, &previousSummary);
                const char *comparison = sensorarrayFdcComparisonName(cmp, havePreviousSummary);
                const char *comparisonNote = havePreviousSummary ? "compared_to_previous_setting" : "first_setting_baseline";

                char minRawBuf[20];
                char maxRawBuf[20];
                char meanRawBuf[20];
                const char *minRawStr = SENSORARRAY_NA;
                const char *maxRawStr = SENSORARRAY_NA;
                const char *meanRawStr = SENSORARRAY_NA;
                if (summary.validSamples > 0u) {
                    uint32_t meanRaw = (uint32_t)(summary.sumRaw / (uint64_t)summary.validSamples);
                    snprintf(minRawBuf, sizeof(minRawBuf), "%lu", (unsigned long)summary.minRaw);
                    snprintf(maxRawBuf, sizeof(maxRawBuf), "%lu", (unsigned long)summary.maxRaw);
                    snprintf(meanRawBuf, sizeof(meanRawBuf), "%lu", (unsigned long)meanRaw);
                    minRawStr = minRawBuf;
                    maxRawStr = maxRawBuf;
                    meanRawStr = meanRawBuf;
                }

                char meanFreqBuf[20];
                const char *meanFreqStr = SENSORARRAY_NA;
                if (summary.freqCount > 0u) {
                    uint32_t meanFreqHz = (uint32_t)(summary.sumFreqHz / (uint64_t)summary.freqCount);
                    snprintf(meanFreqBuf, sizeof(meanFreqBuf), "%lu", (unsigned long)meanFreqHz);
                    meanFreqStr = meanFreqBuf;
                }

                printf("DBGFDC_SWEEP,stage=summary,point=S5D5,sweepCycle=%lu,stepGlobal=%lu,bus=secondary_i2c,port=%d,"
                       "i2cAddr=0x%02X,channel=%s,highCurrentDrv=%u,driveCurrent=0x%04X,totalSamples=%lu,"
                       "validSamples=%lu,invalidSamples=%lu,noUnreadConversionCount=%lu,amplitudeFaultCount=%lu,"
                       "amplitudeFaultAnyFlagCount=%lu,watchdogCount=%lu,i2cErrorCount=%lu,configUnknownCount=%lu,"
                       "stillSleepingCount=%lu,notConvertingCount=%lu,zeroRawCount=%lu,minRawValid=%s,maxRawValid=%s,"
                       "meanRawValid=%s,meanSensorFreqHz=%s,comparisonVsPrevious=%s,comparisonNote=%s\n",
                       (unsigned long)sweepCycle,
                       (unsigned long)stepGlobalIndex,
                       (int)ctx.fdcState->i2cCtx->Port,
                       ctx.i2cAddr,
                       sensorarrayFdcChannelName(ctx.fdcMap->channel),
                       highCurrentDrv ? 1u : 0u,
                       regs.driveCurrentCh0,
                       (unsigned long)summary.totalSamples,
                       (unsigned long)summary.validSamples,
                       (unsigned long)summary.invalidSamples,
                       (unsigned long)summary.noUnreadCount,
                       (unsigned long)summary.amplitudeFaultCount,
                       (unsigned long)summary.amplitudeFlagAnyCount,
                       (unsigned long)summary.watchdogCount,
                       (unsigned long)summary.i2cErrorCount,
                       (unsigned long)summary.configUnknownCount,
                       (unsigned long)summary.stillSleepingCount,
                       (unsigned long)summary.notConvertingCount,
                       (unsigned long)summary.zeroRawCount,
                       minRawStr,
                       maxRawStr,
                       meanRawStr,
                       meanFreqStr,
                       comparison,
                       comparisonNote);

                previousSummary = summary;
                previousSummary.initialized = true;
                havePreviousSummary = true;
                stepGlobalIndex++;
                sensorarrayDelayMs(ctx.loopDelayMs);
            }
        }
        sweepCycle++;
    }
}
