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

static void sensorarrayAppDelayFramePeriod(void)
{
    uint32_t periodMs = (uint32_t)CONFIG_SENSORARRAY_FDC_MATRIX_PERIOD_MS;
    if (periodMs == 0u) {
        periodMs = 1u;
    }
    vTaskDelay(pdMS_TO_TICKS(periodMs));
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
                bool hasCell = false;
                uint8_t s = 1u;
                uint8_t d = 1u;
                if (sensorarrayParseForceFullSweepCommand(line, &hasCell, &s, &d)) {
                    if (hasCell) {
                        sensorarrayFdcSweepRequestForceFullSweepCell(s, d);
                    } else {
                        sensorarrayFdcSweepRequestForceFullSweepAll();
                    }
                } else {
                    printf("FDC_COMMAND,command=%s,status=ignored\n", line);
                }
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
    while (true) {
        sensorarrayFdcMatrixFrame_t frame = {0};
        (void)sensorarrayMeasureReadFdcMatrixFrame(&s_state, &frame);
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

    if (s_state.fdcPrimary.ready && s_state.fdcSecondary.ready) {
        err = sensorarrayFdcSweepRunBoot(&s_state);
        sensorarrayLogStartup("fdc_boot_sweep",
                              err,
                              (err == ESP_OK) ? "ok" : "warning_or_failed",
                              (int32_t)err);
    }

    sensorarrayStartCommandTask();
    sensorarrayRunFdcMatrixLoop();
}
