#include "sensorarrayApp.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "boardSupport.h"
#include "tmuxSwitch.h"

#include "sensorarrayBoardMap.h"
#include "sensorarrayBringup.h"
#include "sensorarrayConfig.h"
#include "sensorarrayFdcSweep.h"
#include "sensorarrayLog.h"
#include "sensorarrayMeasure.h"
#include "sensorarrayTypes.h"

static sensorarrayState_t s_state = {0};
static bool s_fdcBootSweepOk = false;
static bool s_fdcDiagnosticMode = false;
static uint32_t s_fdcMatrixPeriodOverrideMs = 0u;

static void sensorarrayAppLogStackHighWater(const char *stage)
{
    printf("APP_STACK,stage=%s,freeWords=%lu\n",
           stage ? stage : "na",
           (unsigned long)uxTaskGetStackHighWaterMark(NULL));
}

static uint32_t sensorarrayAppFdcFramePeriodMs(void)
{
    uint32_t periodMs = s_fdcMatrixPeriodOverrideMs ?
                        s_fdcMatrixPeriodOverrideMs :
                        (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS;
    if (periodMs == 0u) {
        periodMs = 1u;
    }
    return periodMs;
}

static void sensorarrayAppDelayFramePeriodSince(int64_t frameStartUs, uint32_t sequence)
{
    uint32_t periodMs = sensorarrayAppFdcFramePeriodMs();
    int64_t periodUs = (int64_t)periodMs * 1000LL;
    int64_t elapsedUs = esp_timer_get_time() - frameStartUs;
    int64_t remainingUs = periodUs - elapsedUs;
    if (remainingUs > 0) {
        uint32_t delayMs = (uint32_t)((remainingUs + 999LL) / 1000LL);
        if (delayMs == 0u) {
            delayMs = 1u;
        }
        vTaskDelay(pdMS_TO_TICKS(delayMs));
        return;
    }

    printf("SCAN_TIMING_OVERRUN,seq=%lu,frameUs=%lld,periodUs=%lld,overrun=1\n",
           (unsigned long)sequence,
           (long long)elapsedUs,
           (long long)periodUs);
}

static bool sensorarrayAppFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame)
{
    if (!frame) {
        return true;
    }
    return frame->freshCount == SENSORARRAY_MATRIX_CELL_COUNT &&
           frame->hardwareZeroRawCount == SENSORARRAY_MATRIX_CELL_COUNT;
}

static void sensorarrayAppDelayDiagnosticPeriod(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000u));
}

static const char *sensorarrayAppFdcDeviceName(sensorarrayFdcDeviceId_t devId)
{
    return (devId == SENSORARRAY_FDC_DEV_SECONDARY) ? "secondary" : "primary";
}

static const char *sensorarrayAppFdcCacheSourceName(sensorarrayFdcCacheSource_t source)
{
    switch (source) {
    case SENSORARRAY_FDC_CACHE_SOURCE_BOOT_FULL:
        return "boot_full";
    case SENSORARRAY_FDC_CACHE_SOURCE_MANUAL_FULL:
        return "manual_full";
    case SENSORARRAY_FDC_CACHE_SOURCE_FAST_RESCUE:
        return "fast_rescue";
    case SENSORARRAY_FDC_CACHE_SOURCE_LAST_GOOD:
        return "last_good";
    case SENSORARRAY_FDC_CACHE_SOURCE_NONE:
    default:
        return "none";
    }
}

static void sensorarrayAppMarkAllFdcCellsPending(const char *reason)
{
    const char *source = reason ? reason : "runtime_rescue";
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
            uint8_t matrixIndex = (uint8_t)sensorarrayMatrixIndex(s, d);
            (void)sensorarrayMeasureRequestFdcCellRescue(&s_state, matrixIndex, source);
        }
    }
}

static bool sensorarrayAppRunPendingFdcCellRescue(bool *rescueRunning, uint32_t *rescueEpoch)
{
    if (!rescueRunning || !rescueEpoch || *rescueRunning) {
        return false;
    }

    int64_t nowUs = esp_timer_get_time();
    int64_t cooldownUs = (int64_t)CONFIG_SENSORARRAY_FDC_FAST_SWEEP_MIN_COOLDOWN_MS * 1000LL;
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
            sensorarrayFdcCellTarget_t target = {0};
            if (!sensorarrayMeasureMakeFdcCellTarget(&s_state, s, d, &target)) {
                continue;
            }
            sensorarrayFdcCellConfigCache_t *cache =
                sensorarrayMeasureGetFdcCellCache(&s_state, &target);
            if (!cache || !cache->rescuePending) {
                continue;
            }

            if (cache->lastRescueTimestampUs != 0u &&
                cooldownUs > 0 &&
                (nowUs - (int64_t)cache->lastRescueTimestampUs) < cooldownUs) {
                printf("FDC_RESCUE,stage=defer,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,reason=cooldown,pendingReason=%s\n",
                       (unsigned)target.sColumn,
                       (unsigned)target.dLine,
                       (unsigned)target.matrixIndex,
                       sensorarrayAppFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       cache->lastRescueReason[0] ? cache->lastRescueReason : SENSORARRAY_NA);
                return false;
            }

            uint32_t epoch = ++(*rescueEpoch);
            const char *reason = cache->lastRescueReason[0] ?
                cache->lastRescueReason :
                "runtime_cell_rescue";
            int64_t startUs = esp_timer_get_time();
            *rescueRunning = true;
            printf("FDC_RESCUE,stage=begin,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,mode=fast,reason=%s,epoch=%lu\n",
                   (unsigned)target.sColumn,
                   (unsigned)target.dLine,
                   (unsigned)target.matrixIndex,
                   sensorarrayAppFdcDeviceName(target.devId),
                   (unsigned)target.fdcChannel,
                   reason,
                   (unsigned long)epoch);

            esp_err_t err = sensorarrayFdcSweepRunCellFastRescue(&s_state, &target, reason);
            esp_err_t restoreErr = sensorarrayFdcSweepRestoreAutoscan(&s_state, target.devId, reason);
            if (err == ESP_OK && restoreErr == ESP_OK) {
                cache->fastRescueFailCount = 0u;
            } else if (cache->fastRescueFailCount < UINT8_MAX) {
                cache->fastRescueFailCount++;
            }

            uint8_t fullThreshold = (uint8_t)CONFIG_SENSORARRAY_FDC_FAST_FAIL_THRESHOLD;
            if (fullThreshold == 0u) {
                fullThreshold = 1u;
            }
            if ((err != ESP_OK || restoreErr != ESP_OK) &&
                cache->fastRescueFailCount >= fullThreshold) {
                printf("FDC_RESCUE,stage=begin,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,mode=full,reason=fast_failed,epoch=%lu\n",
                       (unsigned)target.sColumn,
                       (unsigned)target.dLine,
                       (unsigned)target.matrixIndex,
                       sensorarrayAppFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       (unsigned long)epoch);
                esp_err_t fullErr = sensorarrayFdcSweepRunFullRescueCell(&s_state,
                                                                         target.sColumn,
                                                                         target.dLine,
                                                                         "runtime_cell_full");
                esp_err_t fullRestoreErr = sensorarrayFdcSweepRestoreAutoscan(&s_state,
                                                                              target.devId,
                                                                              "runtime_cell_full");
                if (fullErr == ESP_OK && fullRestoreErr == ESP_OK) {
                    cache->fastRescueFailCount = 0u;
                    err = ESP_OK;
                    restoreErr = ESP_OK;
                } else {
                    err = (fullErr != ESP_OK) ? fullErr : fullRestoreErr;
                    restoreErr = fullRestoreErr;
                }
                printf("FDC_RESCUE,stage=end,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,mode=full,reason=fast_failed,epoch=%lu,err=0x%lx,restoreErr=0x%lx\n",
                       (unsigned)target.sColumn,
                       (unsigned)target.dLine,
                       (unsigned)target.matrixIndex,
                       sensorarrayAppFdcDeviceName(target.devId),
                       (unsigned)target.fdcChannel,
                       (unsigned long)epoch,
                       (unsigned long)err,
                       (unsigned long)fullRestoreErr);
            }

            int64_t endUs = esp_timer_get_time();
            cache->lastRescueTimestampUs = endUs;
            cache->rescuePending = false;
            cache->lastRescueReason[0] = '\0';
            printf("FDC_RESCUE_HEARTBEAT,scope=cell,s=%u,d=%u,index=%u,elapsedMs=%lu\n",
                   (unsigned)target.sColumn,
                   (unsigned)target.dLine,
                   (unsigned)target.matrixIndex,
                   (unsigned long)((endUs - startUs) / 1000LL));
            *rescueRunning = false;
            printf("FDC_RESCUE,stage=end,scope=cell,s=%u,d=%u,index=%u,device=%s,ch=%u,mode=fast,reason=%s,epoch=%lu,err=0x%lx,restoreErr=0x%lx\n",
                   (unsigned)target.sColumn,
                   (unsigned)target.dLine,
                   (unsigned)target.matrixIndex,
                   sensorarrayAppFdcDeviceName(target.devId),
                   (unsigned)target.fdcChannel,
                   reason,
                   (unsigned long)epoch,
                   (unsigned long)err,
                   (unsigned long)restoreErr);
            return true;
        }
    }
    return false;
}

static void sensorarrayApplyTmuxDefaults(void)
{
    if (!s_state.tmuxReady) {
        return;
    }

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
        tmuxErr = sensorarrayMeasureSetSwPhysicalLevel(&s_state,
                                                       SENSORARRAY_SW_PHYSICAL_LOW,
                                                       "init_default");
    }
    if (tmuxErr == ESP_OK) {
        tmuxErr = tmux1134SetEnLogicalState(true);
    }

    sensorarrayLogStartup("tmux_defaults",
                          tmuxErr,
                          (tmuxErr == ESP_OK) ? "ok" : "set_failed",
                          (int32_t)(tmuxErr == ESP_OK));
}

static void sensorarrayAppInitFdcDevice(sensorarrayFdcDeviceState_t *fdcState,
                                        bool addressValid,
                                        uint8_t requestedChannels,
                                        const char *mapLabel)
{
    if (!fdcState) {
        return;
    }

    esp_err_t err = ESP_OK;
    if (!addressValid) {
        sensorarrayLogStartupFdc("fdc_init",
                                 fdcState,
                                 ESP_ERR_INVALID_ARG,
                                 "skip_invalid_addr_config",
                                 (int32_t)fdcState->i2cAddr,
                                 false,
                                 0,
                                 0,
                                 mapLabel);
        return;
    }
    if (!fdcState->i2cCtx) {
        sensorarrayLogStartupFdc("fdc_init",
                                 fdcState,
                                 ESP_ERR_NOT_SUPPORTED,
                                 "skip_i2c_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 mapLabel);
        return;
    }

    sensorarrayBringupProbeFdcBus(fdcState);

    sensorarrayFdcInitDiag_t diag = {0};
    err = sensorarrayBringupInitFdcDevice(fdcState->i2cCtx,
                                          fdcState->i2cAddr,
                                          requestedChannels,
                                          &fdcState->handle,
                                          &diag);
    sensorarrayBringupApplyFdcInitResult(fdcState, fdcState->handle, err, &diag);
    sensorarrayLogStartupFdc("fdc_init",
                             fdcState,
                             err,
                             diag.status,
                             (err == ESP_OK) ? (int32_t)requestedChannels : diag.detail,
                             diag.haveIds,
                             diag.manufacturerId,
                             diag.deviceId,
                             mapLabel);
}

static void sensorarrayAppLogFdcParallelCfg(void)
{
    BoardSupportI2cBusInfo_t primaryBus = {0};
    BoardSupportI2cBusInfo_t secondaryBus = {0};
    (void)boardSupportGetI2cBusInfo(false, &primaryBus);
    (void)boardSupportGetI2cBusInfo(true, &secondaryBus);
    bool sameBus = primaryBus.Enabled &&
                   secondaryBus.Enabled &&
                   primaryBus.Port == secondaryBus.Port;
    bool configEnabled = CONFIG_SENSORARRAY_FDC_PARALLEL_DUAL_BUS_READ != 0;
    bool enabled = configEnabled &&
                   primaryBus.Enabled &&
                   secondaryBus.Enabled &&
                   !sameBus;
    const char *reason = !configEnabled ? "config_disabled" :
        (!primaryBus.Enabled || !secondaryBus.Enabled) ? "bus_unavailable" :
        sameBus ? "same_bus" :
        "dual_bus_enabled";
    printf("FDC_PARALLEL_CFG,enabled=%u,primaryBus=%d,secondaryBus=%d,sameBus=%u,primaryCore=%d,secondaryCore=%d,workerMode=%s,reason=%s\n",
           enabled ? 1u : 0u,
           primaryBus.Enabled ? (int)primaryBus.Port : -1,
           secondaryBus.Enabled ? (int)secondaryBus.Port : -1,
           sameBus ? 1u : 0u,
           CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
           CONFIG_SENSORARRAY_FDC_WORKER_TASK_CORE,
           enabled ? "parallel" : "serial",
           reason);
}

static bool sensorarrayParseForceFullSweepCommand(const char *line,
                                                  bool *outHasCell,
                                                  uint8_t *outS,
                                                  uint8_t *outD)
{
    if (!line || !outHasCell || !outS || !outD) {
        return false;
    }
    if (strncmp(line, "force_full_sweep", strlen("force_full_sweep")) != 0) {
        return false;
    }

    bool haveS = false;
    bool haveD = false;
    uint8_t s = 1u;
    uint8_t d = 1u;
    const char *sArg = strstr(line, "s=");
    const char *dArg = strstr(line, "d=");
    if (sArg) {
        long parsed = strtol(sArg + 2, NULL, 10);
        if (parsed >= 1 && parsed <= 8) {
            s = (uint8_t)parsed;
            haveS = true;
        }
    }
    if (dArg) {
        long parsed = strtol(dArg + 2, NULL, 10);
        if (parsed >= 1 && parsed <= 8) {
            d = (uint8_t)parsed;
            haveD = true;
        }
    }

    *outHasCell = haveS && haveD;
    *outS = s;
    *outD = d;
    return true;
}

static bool sensorarrayParseOptionalCacheDiagArgs(const char *line,
                                                  bool *outHasS,
                                                  bool *outHasD,
                                                  uint8_t *outS,
                                                  uint8_t *outD)
{
    if (!line || !outHasS || !outHasD || !outS || !outD) {
        return false;
    }
    if (strncmp(line, "fdc_cache_diag", strlen("fdc_cache_diag")) != 0) {
        return false;
    }

    bool haveS = false;
    bool haveD = false;
    uint8_t s = 1u;
    uint8_t d = 1u;
    const char *sArg = strstr(line, "s=");
    const char *dArg = strstr(line, "d=");
    if (sArg) {
        long parsed = strtol(sArg + 2, NULL, 10);
        if (parsed >= 1 && parsed <= 8) {
            s = (uint8_t)parsed;
            haveS = true;
        }
    }
    if (dArg) {
        long parsed = strtol(dArg + 2, NULL, 10);
        if (parsed >= 1 && parsed <= 8) {
            d = (uint8_t)parsed;
            haveD = true;
        }
    }

    *outHasS = haveS;
    *outHasD = haveS && haveD;
    *outS = s;
    *outD = d;
    return true;
}

static esp_err_t sensorarrayPrintFdcCacheDiag(bool hasS, bool hasD, uint8_t sFilter, uint8_t dFilter)
{
    for (uint8_t s = 1u; s <= SENSORARRAY_MATRIX_ROWS; ++s) {
        if (hasS && s != sFilter) {
            continue;
        }
        for (uint8_t d = 1u; d <= SENSORARRAY_MATRIX_COLS; ++d) {
            if (hasD && d != dFilter) {
                continue;
            }
            sensorarrayFdcCellTarget_t target = {0};
            if (!sensorarrayMeasureMakeFdcCellTarget(&s_state, s, d, &target)) {
                continue;
            }
            sensorarrayFdcCellConfigCache_t *cache =
                sensorarrayMeasureGetFdcCellCache(&s_state, &target);
            if (!cache) {
                continue;
            }
            printf("FDC_CACHE_DIAG,s=%u,d=%u,index=%u,device=%s,ch=%u,valid=%u,source=%s,drive=0x%04X,rCount=0x%04X,settle=0x%04X,clockDiv=0x%04X,deglitch=0x%X,lastFreqHz=%.3f,lastRaw28=%lu,generation=%lu,quality=%lu,reapplyPending=%u,rescuePending=%u,warnCount=%u,errorCount=%u,lastAppliedUs=%lld,lastGoodUs=%lld\n",
                   (unsigned)s,
                   (unsigned)d,
                   (unsigned)target.matrixIndex,
                   sensorarrayAppFdcDeviceName(target.devId),
                   (unsigned)target.fdcChannel,
                   cache->valid ? 1u : 0u,
                   sensorarrayAppFdcCacheSourceName(cache->source),
                   cache->driveCurrent,
                   cache->rCount,
                   cache->settleCount,
                   cache->clockDiv,
                   (unsigned)cache->deglitchCode,
                   cache->lastFreqHz,
                   (unsigned long)cache->lastRaw28,
                   (unsigned long)cache->generation,
                   (unsigned long)cache->qualityScore,
                   cache->reapplyPending ? 1u : 0u,
                   cache->rescuePending ? 1u : 0u,
                   (unsigned)cache->consecutiveAmplitudeWarnings,
                   (unsigned)cache->consecutiveErrors,
                   (long long)cache->lastAppliedTimestampUs,
                   (long long)cache->lastGoodTimestampUs);
        }
    }
    return ESP_OK;
}

static bool sensorarrayParseOnOff(const char *arg, bool *outEnabled)
{
    if (!arg || !outEnabled) {
        return false;
    }
    if (strcmp(arg, "on") == 0) {
        *outEnabled = true;
        return true;
    }
    if (strcmp(arg, "off") == 0) {
        *outEnabled = false;
        return true;
    }
    return false;
}

static esp_err_t sensorarrayHandleCommandLine(const char *line)
{
    if (!line || line[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    bool hasCell = false;
    uint8_t s = 1u;
    uint8_t d = 1u;
    if (sensorarrayParseForceFullSweepCommand(line, &hasCell, &s, &d)) {
        esp_err_t err = hasCell ? sensorarrayFdcSweepRequestForceFullSweepCell(s, d)
                                : sensorarrayFdcSweepRequestForceFullSweepAll();
        printf("FDC_COMMAND,command=force_full_sweep,status=%s,err=0x%lx,s=%u,d=%u,scope=%s\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err,
               (unsigned)s,
               (unsigned)d,
               hasCell ? "cell" : "all");
        return err;
    }

    bool hasS = false;
    bool hasD = false;
    if (sensorarrayParseOptionalCacheDiagArgs(line, &hasS, &hasD, &s, &d)) {
        esp_err_t err = sensorarrayPrintFdcCacheDiag(hasS, hasD, s, d);
        printf("FDC_COMMAND,command=fdc_cache_diag,status=%s,err=0x%lx,s=%u,d=%u,scope=%s\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err,
               (unsigned)s,
               (unsigned)d,
               hasD ? "cell" : (hasS ? "row" : "all"));
        return err;
    }

    if (strncmp(line, "fdc_profile ", strlen("fdc_profile ")) == 0) {
        const char *arg = line + strlen("fdc_profile ");
        const char *space = strchr(arg, ' ');
        esp_err_t err = ESP_ERR_INVALID_ARG;
        char scope[16] = {0};
        bool enabled = false;
        if (space && (size_t)(space - arg) < sizeof(scope)) {
            memcpy(scope, arg, (size_t)(space - arg));
            const char *value = space + 1;
            if (sensorarrayParseOnOff(value, &enabled)) {
                if (strcmp(scope, "summary") == 0) {
                    sensorarrayMeasureFdcProfileSetSummary(enabled);
                    err = ESP_OK;
                } else if (strcmp(scope, "row") == 0) {
                    sensorarrayMeasureFdcProfileSetRow(enabled);
                    err = ESP_OK;
                } else if (strcmp(scope, "device") == 0) {
                    sensorarrayMeasureFdcProfileSetDevice(enabled);
                    err = ESP_OK;
                }
            }
        }
        printf("FDC_COMMAND,command=fdc_profile,status=%s,err=0x%lx,scope=%s,enabled=%u,summary=%u,row=%u,device=%u,every=%lu\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err,
               scope[0] ? scope : "invalid",
               enabled ? 1u : 0u,
               sensorarrayMeasureFdcProfileSummaryEnabled() ? 1u : 0u,
               sensorarrayMeasureFdcProfileRowEnabled() ? 1u : 0u,
               sensorarrayMeasureFdcProfileDeviceEnabled() ? 1u : 0u,
               (unsigned long)sensorarrayMeasureFdcProfileSummaryEvery());
        return err;
    }

    if (strncmp(line, "fdc_profile_every", strlen("fdc_profile_every")) == 0) {
        const char *arg = line + strlen("fdc_profile_every");
        while (*arg == ' ') {
            ++arg;
        }
        char *end = NULL;
        unsigned long parsed = strtoul(arg, &end, 10);
        esp_err_t err = (arg != end && parsed <= 10000ul) ? ESP_OK : ESP_ERR_INVALID_ARG;
        if (err == ESP_OK) {
            sensorarrayMeasureFdcProfileSetSummaryEvery((uint32_t)parsed);
        }
        printf("FDC_COMMAND,command=fdc_profile_every,status=%s,err=0x%lx,every=%lu\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err,
               (unsigned long)sensorarrayMeasureFdcProfileSummaryEvery());
        return err;
    }

    if (strncmp(line, "fdc_i2c_trace", strlen("fdc_i2c_trace")) == 0) {
        const char *arg = line + strlen("fdc_i2c_trace");
        while (*arg == ' ') {
            ++arg;
        }
        esp_err_t err = ESP_OK;
        bool enabled = Fdc2214CapI2cTraceIsEnabled();
        if (strcmp(arg, "on") == 0) {
            Fdc2214CapI2cTraceSetEnabled(true);
            enabled = true;
        } else if (strcmp(arg, "off") == 0) {
            Fdc2214CapI2cTraceSetEnabled(false);
            enabled = false;
        } else if (strcmp(arg, "dump") == 0) {
            Fdc2214CapI2cTraceDump();
        } else if (strcmp(arg, "clear") == 0) {
            Fdc2214CapI2cTraceClear();
        } else {
            err = ESP_ERR_INVALID_ARG;
        }
        printf("FDC_COMMAND,command=fdc_i2c_trace,status=%s,err=0x%lx,enabled=%u,action=%s\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err,
               enabled ? 1u : 0u,
               arg[0] ? arg : "invalid");
        return err;
    }

    if (strncmp(line, "fdc_discard_frames", strlen("fdc_discard_frames")) == 0) {
        const char *arg = line + strlen("fdc_discard_frames");
        while (*arg == ' ') {
            ++arg;
        }
        char *end = NULL;
        unsigned long parsed = strtoul(arg, &end, 10);
        esp_err_t err = (arg != end && parsed <= 8ul) ?
            sensorarrayMeasureFdcSetDiscardFrames((uint8_t)parsed) :
            ESP_ERR_INVALID_ARG;
        printf("FDC_COMMAND,command=fdc_discard_frames,status=%s,err=0x%lx,discardFrames=%u\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err,
               (unsigned)sensorarrayMeasureFdcDiscardFrames());
        return err;
    }

    if (strcmp(line, "fdc_diag") == 0) {
        esp_err_t err = sensorarrayFdcSweepDumpAllDeviceRegs(&s_state, "command_diag", "fdc_diag");
        printf("FDC_COMMAND,command=fdc_diag,status=%s,err=0x%lx\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err);
        return err;
    }

    if (strcmp(line, "fdc_boot_sweep") == 0) {
        printf("APP_FDC,stage=boot_sweep_begin,primaryReady=%d,secondaryReady=%d\n",
               s_state.fdcPrimary.ready ? 1 : 0,
               s_state.fdcSecondary.ready ? 1 : 0);
        esp_err_t err = sensorarrayFdcSweepRunBoot(&s_state);
        s_fdcBootSweepOk = (err == ESP_OK);
        s_fdcDiagnosticMode = (err != ESP_OK);
        printf("APP_FDC,stage=boot_sweep_return,err=0x%lx,ok=%d\n",
               (unsigned long)err,
               s_fdcBootSweepOk ? 1 : 0);
        printf("FDC_COMMAND,command=fdc_boot_sweep,status=%s,err=0x%lx\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err);
        return err;
    }

    if (strcmp(line, "fdc_rescue") == 0) {
        esp_err_t err = sensorarrayFdcSweepRequestForceFullSweepAll();
        printf("FDC_COMMAND,command=fdc_rescue,status=%s,err=0x%lx\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err);
        return err;
    }

    if (strncmp(line, "fdc_period_ms", strlen("fdc_period_ms")) == 0) {
        const char *arg = line + strlen("fdc_period_ms");
        while (*arg == ' ') {
            ++arg;
        }
        char *end = NULL;
        unsigned long parsed = strtoul(arg, &end, 10);
        esp_err_t err = (arg != end && parsed >= 1ul && parsed <= 10000ul) ? ESP_OK : ESP_ERR_INVALID_ARG;
        if (err == ESP_OK) {
            s_fdcMatrixPeriodOverrideMs = (uint32_t)parsed;
        }
        printf("FDC_COMMAND,command=fdc_period_ms,status=%s,err=0x%lx,periodMs=%lu\n",
               (err == ESP_OK) ? "accepted" : "failed",
               (unsigned long)err,
               (unsigned long)(err == ESP_OK ? s_fdcMatrixPeriodOverrideMs : 0u));
        return err;
    }

    printf("FDC_COMMAND,command=%s,status=failed,err=0x%lx\n",
           line,
           (unsigned long)ESP_ERR_NOT_SUPPORTED);
    return ESP_ERR_NOT_SUPPORTED;
}

static void sensorarrayCommandTask(void *arg)
{
    (void)arg;
    char line[96] = {0};
    size_t len = 0u;

    while (true) {
        int ch = getchar();
        if (ch < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            line[len] = '\0';
            if (len > 0u) {
                (void)sensorarrayHandleCommandLine(line);
            }
            len = 0u;
            continue;
        }
        if (len + 1u < sizeof(line)) {
            line[len++] = (char)ch;
        } else {
            len = 0u;
            printf("FDC_COMMAND,status=line_too_long\n");
        }
    }
}

static void sensorarrayStartCommandTask(void)
{
#if CONFIG_SENSORARRAY_ENABLE_WIRED
    BaseType_t ok = xTaskCreatePinnedToCore(sensorarrayCommandTask,
                                            "sa_cmd",
                                            (uint32_t)CONFIG_SENSORARRAY_COMM_TASK_STACK,
                                            NULL,
                                            (UBaseType_t)CONFIG_SENSORARRAY_COMM_TASK_PRIO,
                                            NULL,
                                            (BaseType_t)CONFIG_SENSORARRAY_COMM_TASK_CORE);
    sensorarrayLogStartup("command",
                          (ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM,
                          (ok == pdPASS) ? "console_ready" : "task_create_failed",
                          (int32_t)(ok == pdPASS));
#endif
}

static void sensorarrayRunFdcMatrixLoop(void)
{
    static uint32_t consecutiveAllInvalidFrames = 0u;
    static uint32_t consecutiveReadErrors = 0u;
    static uint32_t rescueEpoch = 0u;
    static uint32_t failedRescueCount = 0u;
    static int64_t lastFullRescueTimeUs = 0;
    static bool rescueRunning = false;
    uint32_t frameCounter = 0u;

    while (true) {
        int64_t nowUs = esp_timer_get_time();
        int64_t cooldownUs = (int64_t)CONFIG_SENSORARRAY_FDC_FULL_RESCUE_COOLDOWN_MS * 1000LL;
        bool cooldownElapsed = lastFullRescueTimeUs == 0 ||
                               cooldownUs <= 0 ||
                               (nowUs - lastFullRescueTimeUs) >= cooldownUs;

        if (!s_fdcDiagnosticMode && (frameCounter % 100u) == 0u) {
            sensorarrayAppLogStackHighWater("fdc_matrix_loop");
        }

        if (sensorarrayFdcSweepConsumeForceFullSweepAll()) {
            if (!rescueRunning && cooldownElapsed) {
                uint32_t epoch = ++rescueEpoch;
                sensorarrayAppLogStackHighWater("manual_full_rescue_before");
                rescueRunning = true;
                printf("FDC_RESCUE,stage=begin,reason=manual_force_full_sweep_all,epoch=%lu\n",
                       (unsigned long)epoch);
                esp_err_t rescueErr = sensorarrayFdcSweepRunFullRescueAll(&s_state,
                                                                          "manual_force_full_sweep_all");
                lastFullRescueTimeUs = esp_timer_get_time();
                rescueRunning = false;
                sensorarrayAppLogStackHighWater("manual_full_rescue_after");
                printf("FDC_RESCUE,stage=end,reason=manual_force_full_sweep_all,epoch=%lu,err=0x%lx\n",
                       (unsigned long)epoch,
                       (unsigned long)rescueErr);
                if (rescueErr == ESP_OK) {
                    failedRescueCount = 0u;
                } else {
                    failedRescueCount++;
                }
            } else {
                printf("FDC_RESCUE,stage=skip,reason=manual_cooldown\n");
            }
        }

        (void)sensorarrayAppRunPendingFdcCellRescue(&rescueRunning, &rescueEpoch);

        if (s_fdcDiagnosticMode) {
            printf("MATRIXFDC_DIAG,stage=diagnostic_mode,bootOk=%d,failedRescue=%lu,primaryReady=%d,secondaryReady=%d\n",
                   s_fdcBootSweepOk ? 1 : 0,
                   (unsigned long)failedRescueCount,
                   s_state.fdcPrimary.ready ? 1 : 0,
                   s_state.fdcSecondary.ready ? 1 : 0);
            sensorarrayAppMarkAllFdcCellsPending("diagnostic_retry");
            (void)sensorarrayFdcSweepDumpAllDeviceRegs(&s_state, "diagnostic_mode", "diagnostic_retry");
            sensorarrayAppDelayDiagnosticPeriod();
            continue;
        }

        int64_t frameStartUs = esp_timer_get_time();
        sensorarrayFdcMatrixFrame_t frame = {0};
        esp_err_t readErr = sensorarrayMeasureReadFdcMatrixFrame(&s_state, &frame);
        frameCounter++;
        bool hardwareRawAllZero = sensorarrayAppFrameRawAllZero(&frame);
        bool allInvalidFrame = (frame.validMask == 0u);
        if (readErr != ESP_OK) {
            consecutiveReadErrors++;
        } else {
            consecutiveReadErrors = 0u;
        }

        if (allInvalidFrame) {
            consecutiveAllInvalidFrames++;
            sensorarrayAppLogStackHighWater("all_invalid_frame_before");
            printf("MATRIXFDC_DIAG,stage=all_invalid_frame,seq=%lu,consecutive=%lu,errorMask=0x%016llX,readErr=0x%lx,bootOk=%u,freshCount=%u,hardwareZeroRawCount=%u,placeholderZeroCount=%u\n",
                   (unsigned long)frame.sequence,
                   (unsigned long)consecutiveAllInvalidFrames,
                   (unsigned long long)frame.errorMask,
                   (unsigned long)readErr,
                   s_fdcBootSweepOk ? 1u : 0u,
                   (unsigned)frame.freshCount,
                   (unsigned)frame.hardwareZeroRawCount,
                   (unsigned)frame.placeholderZeroCount);
            (void)sensorarrayFdcMatrixEmitFrame(&frame);
            nowUs = esp_timer_get_time();
            cooldownElapsed = lastFullRescueTimeUs == 0 ||
                              cooldownUs <= 0 ||
                              (nowUs - lastFullRescueTimeUs) >= cooldownUs;
            bool rescuePendingAllowed =
                consecutiveAllInvalidFrames >= (uint32_t)CONFIG_SENSORARRAY_FDC_ALL_INVALID_RESCUE_THRESHOLD &&
                hardwareRawAllZero &&
                frame.validMask == 0u &&
                cooldownElapsed &&
                !rescueRunning;
            if (rescuePendingAllowed) {
                sensorarrayAppLogStackHighWater("all_invalid_diag_before");
                printf("FDC_RESCUE,stage=pending,scope=cells,reason=persistent_all_rows_hardware_zero,consecutive=%lu\n",
                       (unsigned long)consecutiveAllInvalidFrames);
                printf("MATRIXFDC_DIAG,stage=all_invalid_register_dump,seq=%lu\n",
                       (unsigned long)frame.sequence);
                (void)sensorarrayFdcSweepDumpAllDeviceRegs(&s_state,
                                                           "persistent_all_rows_hardware_zero",
                                                           "persistent_all_rows_hardware_zero");
                (void)sensorarrayFdcSweepRequestForceFullSweepAll();
                sensorarrayAppLogStackHighWater("all_invalid_diag_after");
            } else {
                const char *deferReason = hardwareRawAllZero ?
                    "all_invalid_threshold_not_met" :
                    (frame.freshCount == 0u ? "intb_sync_suspected" : "normal_path_invalid_after_boot_ok");
                printf("FDC_RESCUE,stage=defer,reason=%s,consecutive=%lu,bootOk=%u,freshCount=%u,placeholderZeroCount=%u\n",
                       deferReason,
                       (unsigned long)consecutiveAllInvalidFrames,
                       s_fdcBootSweepOk ? 1u : 0u,
                       (unsigned)frame.freshCount,
                       (unsigned)frame.placeholderZeroCount);
            }
            sensorarrayAppLogStackHighWater("all_invalid_frame_after");
            sensorarrayAppDelayFramePeriodSince(frameStartUs, frame.sequence);
            continue;
        }

        if (frame.validMask != 0u) {
            consecutiveAllInvalidFrames = 0u;
            if (readErr != ESP_OK) {
                printf("MATRIXFDC_DIAG,stage=partial_frame_read_error,readErr=0x%lx,validMask=0x%016llX,errorMask=0x%016llX,consecutiveReadErrors=%lu\n",
                       (unsigned long)readErr,
                       (unsigned long long)frame.validMask,
                       (unsigned long long)frame.errorMask,
                       (unsigned long)consecutiveReadErrors);
            }
        }
        (void)sensorarrayFdcMatrixEmitFrame(&frame);
        sensorarrayAppDelayFramePeriodSince(frameStartUs, frame.sequence);
    }
}

void sensorarrayAppRun(void)
{
    sensorarrayLogDbgExtraReset();
    s_state = (sensorarrayState_t){0};
    sensorarrayLogSetAdsState(false, false);
    sensorarrayFastSpeedSetEnabled(false);

    uint8_t requestedChannels = sensorarrayBringupNormalizeFdcChannels((uint8_t)CONFIG_FDC2214CAP_CHANNELS);
    if (requestedChannels < SENSORARRAY_FDC_REQUIRED_CHANNELS) {
        requestedChannels = SENSORARRAY_FDC_REQUIRED_CHANNELS;
    }
    s_state.fdcConfiguredChannels = requestedChannels;

    sensorarrayBringupResetFdcState(&s_state.fdcPrimary,
                                    "primary_fdc2214",
                                    (uint8_t)(CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR & 0xFFu));
    sensorarrayBringupResetFdcState(&s_state.fdcSecondary,
                                    "secondary_fdc2214",
                                    (uint8_t)(CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR & 0xFFu));

    bool primaryAddrValid = sensorarrayBringupParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                                                              &s_state.fdcPrimary.i2cAddr);
    bool secondaryAddrValid = sensorarrayBringupParseI2cAddress((uint32_t)CONFIG_SENSORARRAY_FDC_SECONDARY_I2C_ADDR,
                                                                &s_state.fdcSecondary.i2cAddr);

    sensorarrayLogStartup("app", ESP_OK, "fdc_matrix_start", 0);
    sensorarrayLogStartup("fdc_channels", ESP_OK, "autoscan_ch0_ch3", (int32_t)requestedChannels);
    sensorarrayLogStartupFdc("fdc_cfg",
                             &s_state.fdcPrimary,
                             primaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             primaryAddrValid ? "configured" : "invalid_addr_config",
                             (int32_t)CONFIG_SENSORARRAY_FDC_PRIMARY_I2C_ADDR,
                             false,
                             0,
                             0,
                             "D1..D4_primary_ch0..ch3");
    sensorarrayLogStartupFdc("fdc_cfg",
                             &s_state.fdcSecondary,
                             secondaryAddrValid ? ESP_OK : ESP_ERR_INVALID_ARG,
                             secondaryAddrValid ? "configured" : "invalid_addr_config",
                             (int32_t)s_state.fdcSecondary.i2cAddr,
                             false,
                             0,
                             0,
                             "D5..D8_secondary_ch0..ch3");

    esp_err_t err = boardSupportInit();
    s_state.boardReady = (err == ESP_OK);
    sensorarrayLogStartup("board", err, s_state.boardReady ? "ok" : "init_failed", (int32_t)s_state.boardReady);

    err = tmuxSwitchInit();
    s_state.tmuxReady = (err == ESP_OK);
    sensorarrayLogStartup("tmux", err, s_state.tmuxReady ? "ok" : "init_failed", (int32_t)s_state.tmuxReady);

    sensorarrayApplyTmuxDefaults();

    err = sensorarrayBringupInitAds(&s_state);
    s_state.adsReady = (err == ESP_OK);
    s_state.adsRefReady = false;
    sensorarrayLogStartup("ads", err, s_state.adsReady ? "ok_ref_off" : "init_failed", (int32_t)s_state.adsReady);
    sensorarrayLogSetAdsState(s_state.adsReady, s_state.adsRefReady);

    if (s_state.boardReady) {
        s_state.fdcPrimary.i2cCtx = boardSupportGetI2cCtx();
        s_state.fdcSecondary.i2cCtx = boardSupportGetI2c1Ctx();
        sensorarrayAppInitFdcDevice(&s_state.fdcPrimary,
                                    primaryAddrValid,
                                    requestedChannels,
                                    "D1..D4_primary_ch0..ch3");
        sensorarrayAppInitFdcDevice(&s_state.fdcSecondary,
                                    secondaryAddrValid,
                                    requestedChannels,
                                    "D5..D8_secondary_ch0..ch3");
        sensorarrayAppLogFdcParallelCfg();
    } else {
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcPrimary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D1..D4_primary_ch0..ch3");
        sensorarrayLogStartupFdc("fdc_init",
                                 &s_state.fdcSecondary,
                                 ESP_ERR_INVALID_STATE,
                                 "skip_board_unavailable",
                                 0,
                                 false,
                                 0,
                                 0,
                                 "D5..D8_secondary_ch0..ch3");
    }

    printf("APP_FDC,stage=boot_sweep_begin,primaryReady=%d,secondaryReady=%d\n",
           s_state.fdcPrimary.ready ? 1 : 0,
           s_state.fdcSecondary.ready ? 1 : 0);
    sensorarrayAppLogStackHighWater("boot_sweep_before");
    if (s_state.fdcPrimary.ready && s_state.fdcSecondary.ready) {
        esp_err_t bootErr = sensorarrayFdcSweepRunBoot(&s_state);
        sensorarrayAppLogStackHighWater("boot_sweep_after");
        s_fdcBootSweepOk = (bootErr == ESP_OK);
        s_fdcDiagnosticMode = (bootErr != ESP_OK);
        printf("APP_FDC,stage=boot_sweep_return,err=0x%lx,ok=%d\n",
               (unsigned long)bootErr,
               s_fdcBootSweepOk ? 1 : 0);
        sensorarrayLogStartup("fdc_boot_sweep",
                              bootErr,
                              (bootErr == ESP_OK) ? "ok" : "failed",
                              (int32_t)bootErr);
        if (bootErr != ESP_OK && CONFIG_SENSORARRAY_FDC_BOOT_SWEEP_REQUIRED) {
            printf("FDC_FATAL,stage=boot,reason=boot_sweep_failed,err=0x%lx\n",
                   (unsigned long)bootErr);
        } else if (bootErr != ESP_OK) {
            s_fdcDiagnosticMode = false;
            printf("FDC_BOOT,stage=warning,reason=boot_sweep_failed_not_required,err=0x%lx\n",
                   (unsigned long)bootErr);
        }
    } else {
        sensorarrayAppLogStackHighWater("boot_sweep_after");
        s_fdcBootSweepOk = false;
        s_fdcDiagnosticMode = true;
        printf("APP_FDC,stage=boot_sweep_return,err=0x%lx,ok=0\n",
               (unsigned long)ESP_ERR_INVALID_STATE);
        printf("FDC_FATAL,stage=boot,reason=fdc_init_not_ready,err=0x%lx\n",
               (unsigned long)ESP_ERR_INVALID_STATE);
    }

    sensorarrayStartCommandTask();
    sensorarrayRunFdcMatrixLoop();
}
