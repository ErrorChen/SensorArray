#include "sensorarrayApp.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static void sensorarrayAppDelayFramePeriod(void)
{
    uint32_t periodMs = s_fdcMatrixPeriodOverrideMs ?
                        s_fdcMatrixPeriodOverrideMs :
                        (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS;
    if (periodMs == 0u) {
        periodMs = 1u;
    }
    vTaskDelay(pdMS_TO_TICKS(periodMs));
}

static bool sensorarrayAppFrameRawAllZero(const sensorarrayFdcMatrixFrame_t *frame)
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

static void sensorarrayAppDelayDiagnosticPeriod(void)
{
    vTaskDelay(pdMS_TO_TICKS(1000u));
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
        esp_err_t err = sensorarrayFdcSweepRunFullRescueAll(&s_state, "manual_command");
        s_fdcDiagnosticMode = (err != ESP_OK);
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

    while (true) {
        if (s_fdcDiagnosticMode) {
            printf("MATRIXFDC_DIAG,stage=diagnostic_mode,bootOk=%d,failedRescue=%lu,primaryReady=%d,secondaryReady=%d\n",
                   s_fdcBootSweepOk ? 1 : 0,
                   (unsigned long)failedRescueCount,
                   s_state.fdcPrimary.ready ? 1 : 0,
                   s_state.fdcSecondary.ready ? 1 : 0);
            uint32_t epoch = ++rescueEpoch;
            printf("FDC_RESCUE,stage=begin,reason=diagnostic_retry,epoch=%lu\n",
                   (unsigned long)epoch);
            esp_err_t rescueErr = sensorarrayFdcSweepRunFullRescueAll(&s_state, "diagnostic_retry");
            printf("FDC_RESCUE,stage=end,reason=diagnostic_retry,epoch=%lu,err=0x%lx\n",
                   (unsigned long)epoch,
                   (unsigned long)rescueErr);
            if (rescueErr == ESP_OK) {
                s_fdcDiagnosticMode = false;
                s_fdcBootSweepOk = true;
                consecutiveAllInvalidFrames = 0u;
                consecutiveReadErrors = 0u;
                failedRescueCount = 0u;
            } else {
                sensorarrayAppDelayDiagnosticPeriod();
            }
            continue;
        }

        sensorarrayFdcMatrixFrame_t frame = {0};
        esp_err_t readErr = sensorarrayMeasureReadFdcMatrixFrame(&s_state, &frame);
        bool rawAllZero = sensorarrayAppFrameRawAllZero(&frame);
        bool allInvalidZero = (frame.validMask == 0u && rawAllZero);
        if (readErr != ESP_OK) {
            consecutiveReadErrors++;
        } else {
            consecutiveReadErrors = 0u;
        }

        if (allInvalidZero) {
            consecutiveAllInvalidFrames++;
            printf("MATRIXFDC_DIAG,stage=all_invalid_frame,seq=%lu,consecutive=%lu,errorMask=0x%016llX,readErr=0x%lx\n",
                   (unsigned long)frame.sequence,
                   (unsigned long)consecutiveAllInvalidFrames,
                   (unsigned long long)frame.errorMask,
                   (unsigned long)readErr);
            (void)sensorarrayFdcMatrixEmitFrame(&frame);

            uint32_t epoch = ++rescueEpoch;
            printf("FDC_RESCUE,stage=begin,reason=all_invalid_frame,epoch=%lu\n",
                   (unsigned long)epoch);
            esp_err_t rescueErr = sensorarrayFdcSweepRunFullRescueAll(&s_state, "all_invalid_frame");
            printf("FDC_RESCUE,stage=end,reason=all_invalid_frame,epoch=%lu,err=0x%lx\n",
                   (unsigned long)epoch,
                   (unsigned long)rescueErr);
            if (rescueErr == ESP_OK) {
                consecutiveAllInvalidFrames = 0u;
                consecutiveReadErrors = 0u;
                failedRescueCount = 0u;
            } else {
                failedRescueCount++;
                printf("FDC_FATAL,stage=matrix_loop,reason=no_oscillation_after_full_rescue,consecutive=%lu,failedRescue=%lu\n",
                       (unsigned long)consecutiveAllInvalidFrames,
                       (unsigned long)failedRescueCount);
                if (failedRescueCount >= (uint32_t)CONFIG_SENSORARRAY_FDC_ALL_INVALID_RESTART_THRESHOLD) {
                    s_fdcDiagnosticMode = true;
                }
                sensorarrayAppDelayDiagnosticPeriod();
            }
            continue;
        }

        if (frame.validMask != 0u) {
            consecutiveAllInvalidFrames = 0u;
            failedRescueCount = 0u;
        }
        (void)sensorarrayFdcMatrixEmitFrame(&frame);
        sensorarrayAppDelayFramePeriod();
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
    if (s_state.fdcPrimary.ready && s_state.fdcSecondary.ready) {
        esp_err_t bootErr = sensorarrayFdcSweepRunBoot(&s_state);
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
