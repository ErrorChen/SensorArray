#include "boardSupport.h"

#include <stdbool.h>
#include <string.h>

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "boardSupport";

#ifndef CONFIG_BOARD_I2C_FREQ_HZ
#define CONFIG_BOARD_I2C_FREQ_HZ 325000
#endif
#ifndef CONFIG_BOARD_I2C0_FREQ_HZ
#define CONFIG_BOARD_I2C0_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif
#ifndef CONFIG_BOARD_I2C1_FREQ_HZ
#define CONFIG_BOARD_I2C1_FREQ_HZ CONFIG_BOARD_I2C_FREQ_HZ
#endif

#define BOARD_SUPPORT_I2C_TIMEOUT_MS 100u
#define BOARD_SUPPORT_I2C_RECOVERY_PULSE_COUNT 9u
#define BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US 5u
#define BOARD_SUPPORT_I2C_FAILURE_STREAK_TRIGGER 2u
#define BOARD_SUPPORT_I2C_TRANSACTION_RETRY_MAX 1u
#define BOARD_SUPPORT_I2C_MUTEX_TIMEOUT_MS 200u
#define BOARD_SUPPORT_I2C_WARN_LOG_STRIDE 8u
#define BOARD_SUPPORT_I2C_POST_RECOVER_STABILIZE_MS 2u

static bool s_inited = false;
static bool s_i2c0_inited = false;
static bool s_i2c1_inited = false;
static uint32_t s_i2c0_configured_freq_hz = CONFIG_BOARD_I2C0_FREQ_HZ;
static uint32_t s_i2c1_configured_freq_hz = CONFIG_BOARD_I2C1_FREQ_HZ;
static uint32_t s_i2c0_failure_streak = 0u;
static uint32_t s_i2c1_failure_streak = 0u;
static uint32_t s_i2c0_warn_log_counter = 0u;
static uint32_t s_i2c1_warn_log_counter = 0u;
static uint32_t s_i2c0_bus_generation = 1u;
static uint32_t s_i2c1_bus_generation = 1u;
static bool s_i2c0_recovering = false;
static bool s_i2c1_recovering = false;
static SemaphoreHandle_t s_i2c0_mutex = NULL;
static SemaphoreHandle_t s_i2c1_mutex = NULL;
static BoardSupportI2cRecoveryStatus_t s_i2c0_recovery_status = {
    .Valid = false,
    .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
    .Reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE,
    .RecoveryLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE,
    .LastTransferErr = ESP_OK,
    .LastRecoverErr = ESP_OK,
    .LastReinitErr = ESP_OK,
    .FailureStreak = 0u,
    .RecoverAttempted = false,
    .RecoverFailed = false,
    .ReinitAttempted = false,
    .ReinitSucceeded = false,
    .DeleteAttempted = false,
    .DeleteErr = ESP_OK,
    .InitAttempted = false,
    .InitErr = ESP_OK,
    .PulsesIssued = 0u,
    .SclHigh = -1,
    .SdaHigh = -1,
    .BusGeneration = 1u,
};
static BoardSupportI2cRecoveryStatus_t s_i2c1_recovery_status = {
    .Valid = false,
    .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
    .Reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE,
    .RecoveryLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE,
    .LastTransferErr = ESP_OK,
    .LastRecoverErr = ESP_OK,
    .LastReinitErr = ESP_OK,
    .FailureStreak = 0u,
    .RecoverAttempted = false,
    .RecoverFailed = false,
    .ReinitAttempted = false,
    .ReinitSucceeded = false,
    .DeleteAttempted = false,
    .DeleteErr = ESP_OK,
    .InitAttempted = false,
    .InitErr = ESP_OK,
    .PulsesIssued = 0u,
    .SclHigh = -1,
    .SdaHigh = -1,
    .BusGeneration = 1u,
};

static BoardSupportI2cCtx_t s_i2c0_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
};

static BoardSupportI2cCtx_t s_i2c1_ctx = {
    .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
    .TimeoutMs = BOARD_SUPPORT_I2C_TIMEOUT_MS,
};

typedef struct {
    bool valid;
    bool enabled;
    i2c_port_t port;
    int sdaGpio;
    int sclGpio;
    uint32_t requestedFreqHz;
    bool *initedFlag;
    uint32_t *configuredFreqHz;
    uint32_t *failureStreak;
    uint32_t *warnLogCounter;
    uint32_t *busGeneration;
    bool *recoveringFlag;
    SemaphoreHandle_t *busMutex;
    const char *name;
} boardSupportI2cPortConfig_t;

typedef esp_err_t (*boardSupportI2cTransferFn_t)(const BoardSupportI2cCtx_t *ctx,
                                                 uint8_t addr7,
                                                 const uint8_t *tx,
                                                 size_t txLen,
                                                 uint8_t *rx,
                                                 size_t rxLen);

const char *boardSupportI2cRecoverReasonToString(BoardSupportI2cRecoveryReason_t reason)
{
    switch (reason) {
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_STARTUP_SOFT_REINIT:
        return "startup_soft_reinit";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_SINGLE_NACK:
        return "single_nack";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_REPEATED_NACK:
        return "repeated_nack";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_TIMEOUT:
        return "timeout";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW:
        return "line_stuck_low";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_CONTROLLER_STATE:
        return "controller_state";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST:
        return "manual_user_request";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_PREOP_LINE_STUCK:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_DURING_TRANSFER:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_SCL_HELD_LOW:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_SCL_HELD_LOW_AFTER_PULSE:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_GPIO_CONFIG_FAILED:
        return "line_stuck_low";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_TRANSFER_TIMEOUT:
        return "timeout";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_TRANSFER_ERROR_STREAK:
        return "repeated_nack";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_FAIL_DRIVER_REINIT:
        return "controller_state";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_REINIT:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_RECOVER:
        return "manual_user_request";
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE:
    default:
        return "none";
    }
}

const char *boardSupportI2cRecoveryReasonName(BoardSupportI2cRecoveryReason_t reason)
{
    return boardSupportI2cRecoverReasonToString(reason);
}

static const char *boardSupportI2cRecoveryLevelName(BoardSupportI2cRecoveryLevel_t level)
{
    switch (level) {
    case BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE:
        return "none";
    case BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY:
        return "controller_only";
    case BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY:
        return "line_recovery";
    case BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL:
        return "driver_reinstall";
    default:
        return "unknown";
    }
}

static BoardSupportI2cRecoveryReason_t boardSupportI2cRecoveryReasonFromLabel(const char *reason,
                                                                               BoardSupportI2cRecoveryReason_t fallback)
{
    if (!reason || reason[0] == '\0') {
        return fallback;
    }
    if (strcmp(reason, "startup_soft_reinit") == 0 || strcmp(reason, "s5d5_secondary_reinit") == 0) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_STARTUP_SOFT_REINIT;
    }
    if (strcmp(reason, "single_nack") == 0) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_SINGLE_NACK;
    }
    if (strcmp(reason, "repeated_nack") == 0 || strcmp(reason, "transfer_error_streak") == 0) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_REPEATED_NACK;
    }
    if (strcmp(reason, "timeout") == 0 || strcmp(reason, "transfer_timeout") == 0) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_TIMEOUT;
    }
    if (strcmp(reason, "line_stuck_low") == 0 ||
        strcmp(reason, "line_stuck_during_transfer") == 0 ||
        strcmp(reason, "preop_line_stuck") == 0 ||
        strcmp(reason, "recover_scl_held_low") == 0 ||
        strcmp(reason, "recover_scl_held_low_after_pulse") == 0 ||
        strcmp(reason, "recover_gpio_config_failed") == 0) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW;
    }
    if (strcmp(reason, "controller_state") == 0 || strcmp(reason, "recover_fail_driver_reinit") == 0) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_CONTROLLER_STATE;
    }
    if (strcmp(reason, "manual_user_request") == 0 ||
        strcmp(reason, "manual_reinit") == 0 ||
        strcmp(reason, "manual_recover") == 0) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST;
    }
    if (strstr(reason, "stuck") != NULL || strstr(reason, "scl_held_low") != NULL || strstr(reason, "sda_held_low") != NULL) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW;
    }
    if (strstr(reason, "timeout") != NULL) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_TIMEOUT;
    }
    if (strstr(reason, "nack") != NULL) {
        if (strstr(reason, "repeated") != NULL || strstr(reason, "streak") != NULL) {
            return BOARD_SUPPORT_I2C_RECOVERY_REASON_REPEATED_NACK;
        }
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_SINGLE_NACK;
    }
    if (strstr(reason, "controller") != NULL) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_CONTROLLER_STATE;
    }
    if (strstr(reason, "reinit") != NULL) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_STARTUP_SOFT_REINIT;
    }
    if (strstr(reason, "manual") != NULL || strstr(reason, "user") != NULL || strstr(reason, "recover") != NULL) {
        return BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST;
    }
    return fallback;
}

static bool boardSupportI2cPinsValid(int sda_gpio, int scl_gpio)
{
    return (sda_gpio >= 0) && (scl_gpio >= 0);
}

static bool boardSupportShouldEmitRateLimited(uint32_t *counter, uint32_t period)
{
    if (!counter) {
        return true;
    }

    (*counter)++;
    if (*counter == 1u) {
        return true;
    }
    if (period <= 1u) {
        return true;
    }
    return ((*counter % period) == 0u);
}

static const char *boardSupportI2cErrClassName(esp_err_t err)
{
    if (err == ESP_FAIL) {
        return "nack";
    }
    if (err == ESP_ERR_TIMEOUT) {
        return "timeout";
    }
    return "error";
}

static bool boardSupportI2c1Configured(void)
{
    return boardSupportI2cPinsValid(CONFIG_BOARD_I2C1_SDA_GPIO, CONFIG_BOARD_I2C1_SCL_GPIO) &&
           (CONFIG_BOARD_I2C1_PORT >= 0);
}

static esp_err_t boardSupportInitI2c(i2c_port_t port,
                                     int sda_gpio,
                                     int scl_gpio,
                                     uint32_t requested_freq_hz,
                                     uint32_t *out_configured_freq_hz)
{
    if (!boardSupportI2cPinsValid(sda_gpio, scl_gpio)) {
        return ESP_ERR_INVALID_ARG;
    }

    i2c_config_t cfg = {0};
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = sda_gpio;
    cfg.scl_io_num = scl_gpio;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = requested_freq_hz;
#ifdef SOC_I2C_SUPPORT_CLK_SRC
    cfg.clk_flags = 0;
#endif

    esp_err_t err = i2c_param_config(port, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config port %d failed: %d", (int)port, err);
        return err;
    }

    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install port %d failed: %d", (int)port, err);
        return err;
    }

    if (out_configured_freq_hz) {
        *out_configured_freq_hz = cfg.master.clk_speed;
    }
    return ESP_OK;
}

static bool boardSupportResolveI2cPort(const BoardSupportI2cCtx_t *i2cCtx, boardSupportI2cPortConfig_t *outCfg)
{
    if (!i2cCtx || !outCfg) {
        return false;
    }

    *outCfg = (boardSupportI2cPortConfig_t){0};
    if (i2cCtx->Port == (i2c_port_t)CONFIG_BOARD_I2C_PORT) {
        *outCfg = (boardSupportI2cPortConfig_t){
            .valid = true,
            .enabled = boardSupportI2cPinsValid(CONFIG_BOARD_I2C_SDA_GPIO, CONFIG_BOARD_I2C_SCL_GPIO) &&
                       (CONFIG_BOARD_I2C_PORT >= 0),
            .port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
            .sdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
            .sclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
            .requestedFreqHz = CONFIG_BOARD_I2C0_FREQ_HZ,
            .initedFlag = &s_i2c0_inited,
            .configuredFreqHz = &s_i2c0_configured_freq_hz,
            .failureStreak = &s_i2c0_failure_streak,
            .warnLogCounter = &s_i2c0_warn_log_counter,
            .busGeneration = &s_i2c0_bus_generation,
            .recoveringFlag = &s_i2c0_recovering,
            .busMutex = &s_i2c0_mutex,
            .name = "I2C0",
        };
        return true;
    }

    if (i2cCtx->Port == (i2c_port_t)CONFIG_BOARD_I2C1_PORT) {
        *outCfg = (boardSupportI2cPortConfig_t){
            .valid = true,
            .enabled = boardSupportI2c1Configured(),
            .port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
            .sdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
            .sclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
            .requestedFreqHz = CONFIG_BOARD_I2C1_FREQ_HZ,
            .initedFlag = &s_i2c1_inited,
            .configuredFreqHz = &s_i2c1_configured_freq_hz,
            .failureStreak = &s_i2c1_failure_streak,
            .warnLogCounter = &s_i2c1_warn_log_counter,
            .busGeneration = &s_i2c1_bus_generation,
            .recoveringFlag = &s_i2c1_recovering,
            .busMutex = &s_i2c1_mutex,
            .name = "I2C1",
        };
        return true;
    }

    return false;
}

static BoardSupportI2cRecoveryStatus_t *boardSupportRecoveryStatusForCfg(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg) {
        return NULL;
    }
    if (cfg->port == (i2c_port_t)CONFIG_BOARD_I2C_PORT) {
        return &s_i2c0_recovery_status;
    }
    if (cfg->port == (i2c_port_t)CONFIG_BOARD_I2C1_PORT) {
        return &s_i2c1_recovery_status;
    }
    return NULL;
}

static void boardSupportRecoveryStatusReset(BoardSupportI2cRecoveryStatus_t *status, i2c_port_t port)
{
    if (!status) {
        return;
    }
    uint32_t generation = 0u;
    if (port == (i2c_port_t)CONFIG_BOARD_I2C_PORT) {
        generation = s_i2c0_bus_generation;
    } else if (port == (i2c_port_t)CONFIG_BOARD_I2C1_PORT) {
        generation = s_i2c1_bus_generation;
    }
    *status = (BoardSupportI2cRecoveryStatus_t){
        .Valid = true,
        .Port = port,
        .Reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE,
        .RecoveryLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE,
        .LastTransferErr = ESP_OK,
        .LastRecoverErr = ESP_OK,
        .LastReinitErr = ESP_OK,
        .FailureStreak = 0u,
        .RecoverAttempted = false,
        .RecoverFailed = false,
        .ReinitAttempted = false,
        .ReinitSucceeded = false,
        .DeleteAttempted = false,
        .DeleteErr = ESP_OK,
        .InitAttempted = false,
        .InitErr = ESP_OK,
        .PulsesIssued = 0u,
        .SclHigh = -1,
        .SdaHigh = -1,
        .BusGeneration = generation,
    };
}

static void boardSupportRecoveryStatusUpdateLines(BoardSupportI2cRecoveryStatus_t *status,
                                                  bool haveLines,
                                                  bool sclHigh,
                                                  bool sdaHigh)
{
    if (!status) {
        return;
    }
    status->SclHigh = haveLines ? (sclHigh ? 1 : 0) : -1;
    status->SdaHigh = haveLines ? (sdaHigh ? 1 : 0) : -1;
}

static esp_err_t boardSupportI2cLockPort(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg || !cfg->busMutex || !(*cfg->busMutex)) {
        return ESP_ERR_INVALID_STATE;
    }

    TickType_t timeoutTicks = pdMS_TO_TICKS(BOARD_SUPPORT_I2C_MUTEX_TIMEOUT_MS);
    if (timeoutTicks == 0) {
        timeoutTicks = 1;
    }

    if (xSemaphoreTake(*cfg->busMutex, timeoutTicks) != pdTRUE) {
        ESP_LOGE(TAG,
                 "%s lock timeout after %u ms",
                 cfg->name ? cfg->name : "I2C",
                 (unsigned)BOARD_SUPPORT_I2C_MUTEX_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void boardSupportI2cUnlockPort(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg || !cfg->busMutex || !(*cfg->busMutex)) {
        return;
    }
    xSemaphoreGive(*cfg->busMutex);
}

static esp_err_t boardSupportI2cCheckLinesLocked(const boardSupportI2cPortConfig_t *cfg,
                                                 bool *outSclHigh,
                                                 bool *outSdaHigh)
{
    if (!cfg || !outSclHigh || !outSdaHigh || !cfg->valid || !cfg->enabled) {
        return ESP_ERR_INVALID_ARG;
    }

    *outSclHigh = gpio_get_level((gpio_num_t)cfg->sclGpio) != 0;
    *outSdaHigh = gpio_get_level((gpio_num_t)cfg->sdaGpio) != 0;
    return ESP_OK;
}

static esp_err_t boardSupportConfigureRecoveryPins(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg || !cfg->valid || !cfg->enabled) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->sdaGpio < 0 || cfg->sdaGpio >= (int)GPIO_NUM_MAX ||
        cfg->sclGpio < 0 || cfg->sclGpio >= (int)GPIO_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    uint64_t mask = (1ULL << (uint32_t)cfg->sdaGpio) | (1ULL << (uint32_t)cfg->sclGpio);
    gpio_config_t ioCfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&ioCfg);
    if (err != ESP_OK) {
        return err;
    }

    gpio_set_level((gpio_num_t)cfg->sclGpio, 1);
    gpio_set_level((gpio_num_t)cfg->sdaGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
    return ESP_OK;
}

static void boardSupportI2cRecoveryResultInit(BoardSupportI2cRecoveryResult_t *result,
                                              const boardSupportI2cPortConfig_t *cfg,
                                              BoardSupportI2cRecoveryReason_t reason,
                                              BoardSupportI2cRecoveryLevel_t requestedLevel)
{
    if (!result || !cfg) {
        return;
    }
    *result = (BoardSupportI2cRecoveryResult_t){
        .Reason = reason,
        .Port = cfg->port,
        .RequestedLevel = requestedLevel,
        .AttemptedLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE,
        .PreSclHigh = -1,
        .PreSdaHigh = -1,
        .PulsesIssued = 0u,
        .DeleteAttempted = false,
        .DeleteErr = ESP_OK,
        .InitAttempted = false,
        .InitErr = ESP_OK,
        .PostSclHigh = -1,
        .PostSdaHigh = -1,
        .Success = false,
        .BusGenerationAfter = cfg->busGeneration ? *cfg->busGeneration : 0u,
    };
}

static void boardSupportI2cUpdateStatusFromResult(BoardSupportI2cRecoveryStatus_t *status,
                                                  const BoardSupportI2cRecoveryResult_t *result,
                                                  esp_err_t recoverErr)
{
    if (!status || !result) {
        return;
    }
    status->Valid = true;
    status->Port = result->Port;
    status->Reason = result->Reason;
    status->RecoveryLevel = result->AttemptedLevel;
    status->RecoverAttempted = (result->AttemptedLevel != BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE);
    status->RecoverFailed = !result->Success;
    status->LastRecoverErr = recoverErr;
    status->ReinitAttempted = result->AttemptedLevel == BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL;
    status->ReinitSucceeded = status->ReinitAttempted && result->Success;
    status->DeleteAttempted = result->DeleteAttempted;
    status->DeleteErr = result->DeleteErr;
    status->InitAttempted = result->InitAttempted;
    status->InitErr = result->InitErr;
    status->LastReinitErr = result->InitAttempted ? result->InitErr : ESP_OK;
    status->PulsesIssued = result->PulsesIssued;
    status->SclHigh = result->PostSclHigh;
    status->SdaHigh = result->PostSdaHigh;
    status->BusGeneration = result->BusGenerationAfter;
}

static BoardSupportI2cRecoveryLevel_t boardSupportI2cDefaultLevelForReason(BoardSupportI2cRecoveryReason_t reason)
{
    switch (reason) {
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_STARTUP_SOFT_REINIT:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_SINGLE_NACK:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_REPEATED_NACK:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_TIMEOUT:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_CONTROLLER_STATE:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_PREOP_LINE_STUCK:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_DURING_TRANSFER:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_SCL_HELD_LOW:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_SCL_HELD_LOW_AFTER_PULSE:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_GPIO_CONFIG_FAILED:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_TRANSFER_TIMEOUT:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_TRANSFER_ERROR_STREAK:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_RECOVER_FAIL_DRIVER_REINIT:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_REINIT:
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_RECOVER:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL;
    case BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE:
    default:
        return BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
    }
}

static esp_err_t boardSupportI2cControllerRecoverLocked(const boardSupportI2cPortConfig_t *cfg)
{
    if (!cfg || !cfg->valid) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t txErr = i2c_reset_tx_fifo(cfg->port);
    esp_err_t rxErr = i2c_reset_rx_fifo(cfg->port);
    if (txErr != ESP_OK) {
        return txErr;
    }
    if (rxErr != ESP_OK) {
        return rxErr;
    }
    return ESP_OK;
}

static esp_err_t boardSupportI2cLineRecoverLocked(const boardSupportI2cPortConfig_t *cfg,
                                                  BoardSupportI2cRecoveryResult_t *result)
{
    if (!cfg || !result) {
        return ESP_ERR_INVALID_ARG;
    }
    bool preHighHigh = (result->PreSclHigh == 1) && (result->PreSdaHigh == 1);
    if (preHighHigh) {
        return ESP_OK;
    }

    esp_err_t pinErr = boardSupportConfigureRecoveryPins(cfg);
    if (pinErr != ESP_OK) {
        return pinErr;
    }

    bool sclAfterRelease = gpio_get_level((gpio_num_t)cfg->sclGpio) != 0;
    bool sdaAfterRelease = gpio_get_level((gpio_num_t)cfg->sdaGpio) != 0;
    if (!sclAfterRelease) {
        return ESP_ERR_TIMEOUT;
    }

    if (!sdaAfterRelease) {
        for (uint32_t i = 0u; i < BOARD_SUPPORT_I2C_RECOVERY_PULSE_COUNT; ++i) {
            gpio_set_level((gpio_num_t)cfg->sclGpio, 0);
            esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
            gpio_set_level((gpio_num_t)cfg->sclGpio, 1);
            esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
            result->PulsesIssued = i + 1u;
            if (gpio_get_level((gpio_num_t)cfg->sdaGpio) != 0) {
                break;
            }
        }
    }

    gpio_set_level((gpio_num_t)cfg->sdaGpio, 0);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
    gpio_set_level((gpio_num_t)cfg->sclGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);
    gpio_set_level((gpio_num_t)cfg->sdaGpio, 1);
    esp_rom_delay_us(BOARD_SUPPORT_I2C_RECOVERY_HALF_PERIOD_US);

    esp_err_t pinRestoreErr = i2c_set_pin(cfg->port,
                                          cfg->sdaGpio,
                                          cfg->sclGpio,
                                          GPIO_PULLUP_ENABLE,
                                          GPIO_PULLUP_ENABLE,
                                          I2C_MODE_MASTER);
    if (pinRestoreErr != ESP_OK) {
        return pinRestoreErr;
    }
    return ESP_OK;
}

static esp_err_t boardSupportI2cDriverReinstallLocked(const boardSupportI2cPortConfig_t *cfg,
                                                      BoardSupportI2cRecoveryResult_t *result)
{
    if (!cfg || !result || !cfg->initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }
    if (cfg->busGeneration) {
        (*cfg->busGeneration)++;
        result->BusGenerationAfter = *cfg->busGeneration;
    }

    result->DeleteAttempted = true;
    result->DeleteErr = i2c_driver_delete(cfg->port);
    if (result->DeleteErr != ESP_OK && result->DeleteErr != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG,
                 "%s driver_reinstall delete_state=unexpected_failure deleteErr=%d",
                 cfg->name ? cfg->name : "I2C",
                 result->DeleteErr);
        *cfg->initedFlag = false;
        return result->DeleteErr;
    }
    ESP_LOGW(TAG,
             "%s driver_reinstall delete_state=%s deleteErr=%d",
             cfg->name ? cfg->name : "I2C",
             (result->DeleteErr == ESP_ERR_INVALID_STATE) ? "driver_absent" : "delete_ok",
             result->DeleteErr);
    *cfg->initedFlag = false;

    result->InitAttempted = true;
    result->InitErr = boardSupportInitI2c(cfg->port,
                                          cfg->sdaGpio,
                                          cfg->sclGpio,
                                          cfg->requestedFreqHz,
                                          cfg->configuredFreqHz);
    if (result->InitErr != ESP_OK) {
        ESP_LOGE(TAG,
                 "%s driver_reinstall init_state=failed initErr=%d deleteErr=%d",
                 cfg->name ? cfg->name : "I2C",
                 result->InitErr,
                 result->DeleteErr);
        return result->InitErr;
    }
    ESP_LOGW(TAG,
             "%s driver_reinstall init_state=ok initErr=%d deleteErr=%d",
             cfg->name ? cfg->name : "I2C",
             result->InitErr,
             result->DeleteErr);
    *cfg->initedFlag = true;
    if (cfg->failureStreak) {
        *cfg->failureStreak = 0u;
    }
    if (cfg->warnLogCounter) {
        *cfg->warnLogCounter = 0u;
    }
    return ESP_OK;
}

static void boardSupportI2cLogRecoverSummary(const boardSupportI2cPortConfig_t *cfg,
                                             const BoardSupportI2cRecoveryResult_t *result,
                                             esp_err_t err)
{
    if (!cfg || !result) {
        return;
    }
    printf("BSI2C_RECOVER_SUMMARY,port=%d,reason=%s,level=%s,preScl=%d,preSda=%d,pulses=%lu,"
           "deleteAttempted=%u,deleteErr=%ld,initAttempted=%u,initErr=%ld,postScl=%d,postSda=%d,"
           "success=%u,busGeneration=%lu,err=%ld\n",
           (int)cfg->port,
           boardSupportI2cRecoverReasonToString(result->Reason),
           boardSupportI2cRecoveryLevelName(result->AttemptedLevel),
           result->PreSclHigh,
           result->PreSdaHigh,
           (unsigned long)result->PulsesIssued,
           result->DeleteAttempted ? 1u : 0u,
           (long)result->DeleteErr,
           result->InitAttempted ? 1u : 0u,
           (long)result->InitErr,
           result->PostSclHigh,
           result->PostSdaHigh,
           result->Success ? 1u : 0u,
           (unsigned long)result->BusGenerationAfter,
           (long)err);
}

static esp_err_t boardSupportI2cRecoverLocked(const boardSupportI2cPortConfig_t *cfg,
                                              BoardSupportI2cRecoveryReason_t reason,
                                              BoardSupportI2cRecoveryLevel_t requestedLevel,
                                              BoardSupportI2cRecoveryResult_t *outResult)
{
    if (!cfg || !cfg->valid || !cfg->enabled || !cfg->initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    BoardSupportI2cRecoveryResult_t localResult = {0};
    boardSupportI2cRecoveryResultInit(&localResult, cfg, reason, requestedLevel);
    if (outResult) {
        *outResult = localResult;
    }

    if (cfg->recoveringFlag && *cfg->recoveringFlag) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cfg->recoveringFlag) {
        *cfg->recoveringFlag = true;
    }

    esp_err_t err = ESP_OK;
    bool sclHigh = true;
    bool sdaHigh = true;
    if (boardSupportI2cCheckLinesLocked(cfg, &sclHigh, &sdaHigh) == ESP_OK) {
        localResult.PreSclHigh = sclHigh ? 1 : 0;
        localResult.PreSdaHigh = sdaHigh ? 1 : 0;
    }

    bool preHighHigh = (localResult.PreSclHigh == 1) && (localResult.PreSdaHigh == 1);
    BoardSupportI2cRecoveryLevel_t effectiveLevel = requestedLevel;
    bool forceDriverReinstall =
        (reason == BOARD_SUPPORT_I2C_RECOVERY_REASON_CONTROLLER_STATE) &&
        (requestedLevel >= BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL);
    if (effectiveLevel == BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE) {
        effectiveLevel = boardSupportI2cDefaultLevelForReason(reason);
    }

    if (preHighHigh && reason == BOARD_SUPPORT_I2C_RECOVERY_REASON_STARTUP_SOFT_REINIT) {
        effectiveLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
    }
    if (preHighHigh &&
        effectiveLevel >= BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY &&
        reason != BOARD_SUPPORT_I2C_RECOVERY_REASON_CONTROLLER_STATE &&
        reason != BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST) {
        /*
         * Idle-high means the bus is electrically free. Do not force GPIO line
         * pulsing or immediate driver reinstall in this state.
         */
        effectiveLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
    }

    if (effectiveLevel == BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE) {
        localResult.AttemptedLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
        err = ESP_OK;
    } else if (!preHighHigh &&
               reason == BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW &&
               effectiveLevel >= BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY) {
        localResult.AttemptedLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY;
        err = boardSupportI2cLineRecoverLocked(cfg, &localResult);
    } else {
        localResult.AttemptedLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
        err = boardSupportI2cControllerRecoverLocked(cfg);
        if (err != ESP_OK &&
            effectiveLevel >= BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY &&
            !preHighHigh) {
            localResult.AttemptedLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY;
            err = boardSupportI2cLineRecoverLocked(cfg, &localResult);
        }
    }

    if ((forceDriverReinstall || err != ESP_OK) &&
        requestedLevel >= BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL) {
        localResult.AttemptedLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL;
        err = boardSupportI2cDriverReinstallLocked(cfg, &localResult);
    }

    bool sclAfter = true;
    bool sdaAfter = true;
    if (boardSupportI2cCheckLinesLocked(cfg, &sclAfter, &sdaAfter) == ESP_OK) {
        localResult.PostSclHigh = sclAfter ? 1 : 0;
        localResult.PostSdaHigh = sdaAfter ? 1 : 0;
    } else {
        localResult.PostSclHigh = -1;
        localResult.PostSdaHigh = -1;
    }
    if (cfg->busGeneration) {
        localResult.BusGenerationAfter = *cfg->busGeneration;
    }
    localResult.Success = (err == ESP_OK);
    if (err == ESP_OK && cfg->failureStreak) {
        *cfg->failureStreak = 0u;
    }

    BoardSupportI2cRecoveryStatus_t *status = boardSupportRecoveryStatusForCfg(cfg);
    boardSupportI2cUpdateStatusFromResult(status, &localResult, err);
    if (status) {
        status->FailureStreak = cfg->failureStreak ? *cfg->failureStreak : 0u;
    }
    boardSupportI2cLogRecoverSummary(cfg, &localResult, err);

    if (cfg->recoveringFlag) {
        *cfg->recoveringFlag = false;
    }
    if (outResult) {
        *outResult = localResult;
    }
    return err;
}

static esp_err_t boardSupportI2cReinitLocked(const boardSupportI2cPortConfig_t *cfg, const char *reason)
{
    BoardSupportI2cRecoveryReason_t reasonEnum =
        boardSupportI2cRecoveryReasonFromLabel(reason, BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST);
    BoardSupportI2cRecoveryResult_t result = {0};
    esp_err_t err = boardSupportI2cRecoverLocked(cfg,
                                                 reasonEnum,
                                                 BOARD_SUPPORT_I2C_RECOVERY_LEVEL_DRIVER_REINSTALL,
                                                 &result);
    ESP_LOGW(TAG,
             "%s reinit reason=%s deleteAttempted=%u deleteErr=%d initAttempted=%u initErr=%d freqHz=%u status=%s",
             cfg ? cfg->name : "I2C",
             boardSupportI2cRecoverReasonToString(reasonEnum),
             result.DeleteAttempted ? 1 : 0,
             result.DeleteErr,
             result.InitAttempted ? 1 : 0,
             result.InitErr,
             (unsigned)(cfg && cfg->configuredFreqHz ? *cfg->configuredFreqHz : 0u),
             (err == ESP_OK) ? "reinit_ok" : "reinit_failed");
    return err;
}

static esp_err_t boardSupportI2cRecoverBusLocked(const boardSupportI2cPortConfig_t *cfg,
                                                 const char *reason,
                                                 uint32_t *outPulsesIssued)
{
    BoardSupportI2cRecoveryReason_t reasonEnum =
        boardSupportI2cRecoveryReasonFromLabel(reason, BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW);
    BoardSupportI2cRecoveryResult_t result = {0};
    esp_err_t err = boardSupportI2cRecoverLocked(cfg,
                                                 reasonEnum,
                                                 BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY,
                                                 &result);
    if (outPulsesIssued) {
        *outPulsesIssued = result.PulsesIssued;
    }
    return err;
}

static esp_err_t boardSupportI2cTransferWriteRead(const BoardSupportI2cCtx_t *ctx,
                                                  uint8_t addr7,
                                                  const uint8_t *tx,
                                                  size_t txLen,
                                                  uint8_t *rx,
                                                  size_t rxLen)
{
    return i2c_master_write_read_device(ctx->Port,
                                        addr7,
                                        tx,
                                        txLen,
                                        rx,
                                        rxLen,
                                        pdMS_TO_TICKS(ctx->TimeoutMs));
}

static esp_err_t boardSupportI2cTransferWrite(const BoardSupportI2cCtx_t *ctx,
                                              uint8_t addr7,
                                              const uint8_t *tx,
                                              size_t txLen,
                                              uint8_t *rx,
                                              size_t rxLen)
{
    (void)rx;
    (void)rxLen;
    return i2c_master_write_to_device(ctx->Port,
                                      addr7,
                                      tx,
                                      txLen,
                                      pdMS_TO_TICKS(ctx->TimeoutMs));
}

static esp_err_t boardSupportI2cTransferProbe(const BoardSupportI2cCtx_t *ctx,
                                              uint8_t addr7,
                                              const uint8_t *tx,
                                              size_t txLen,
                                              uint8_t *rx,
                                              size_t rxLen)
{
    (void)tx;
    (void)txLen;
    (void)rx;
    (void)rxLen;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = i2c_master_start(cmd);
    if (err == ESP_OK) {
        err = i2c_master_write_byte(cmd, (uint8_t)((addr7 << 1u) | I2C_MASTER_WRITE), true);
    }
    if (err == ESP_OK) {
        err = i2c_master_stop(cmd);
    }
    if (err == ESP_OK) {
        err = i2c_master_cmd_begin(ctx->Port, cmd, pdMS_TO_TICKS(ctx->TimeoutMs));
    }

    i2c_cmd_link_delete(cmd);
    return err;
}

static esp_err_t boardSupportI2cRunWithRecoveryLocked(const BoardSupportI2cCtx_t *ctx,
                                                      const boardSupportI2cPortConfig_t *cfg,
                                                      uint8_t addr7,
                                                      const uint8_t *tx,
                                                      size_t txLen,
                                                      uint8_t *rx,
                                                      size_t rxLen,
                                                      boardSupportI2cTransferFn_t transferFn,
                                                      const char *opName,
                                                      bool allowStreakRecover)
{
    if (!ctx || !cfg || !transferFn) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!cfg->valid || !cfg->enabled || !cfg->initedFlag || !(*cfg->initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    BoardSupportI2cRecoveryStatus_t *status = boardSupportRecoveryStatusForCfg(cfg);
    if (status) {
        status->Valid = true;
        status->Port = cfg->port;
        status->Reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE;
        status->RecoveryLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
        status->LastTransferErr = ESP_OK;
        status->LastRecoverErr = ESP_OK;
        status->LastReinitErr = ESP_OK;
        status->RecoverAttempted = false;
        status->RecoverFailed = false;
        status->ReinitAttempted = false;
        status->ReinitSucceeded = false;
        status->DeleteAttempted = false;
        status->DeleteErr = ESP_OK;
        status->InitAttempted = false;
        status->InitErr = ESP_OK;
        status->PulsesIssued = 0u;
        status->BusGeneration = cfg->busGeneration ? *cfg->busGeneration : 0u;
    }

    bool sclHigh = true;
    bool sdaHigh = true;
    bool havePreopLines = (boardSupportI2cCheckLinesLocked(cfg, &sclHigh, &sdaHigh) == ESP_OK);
    boardSupportRecoveryStatusUpdateLines(status, havePreopLines, sclHigh, sdaHigh);
    if (havePreopLines && (!sclHigh || !sdaHigh)) {
        esp_err_t preRecoverErr = boardSupportI2cRecoverLocked(cfg,
                                                               BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW,
                                                               BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY,
                                                               NULL);
        if (preRecoverErr != ESP_OK) {
            if (status) {
                status->LastRecoverErr = preRecoverErr;
                status->RecoverFailed = true;
            }
            ESP_LOGE(TAG, "%s %s pre-op recover failed: %d", cfg->name, opName ? opName : "i2c", preRecoverErr);
            return preRecoverErr;
        }
    }

    esp_err_t err = ESP_FAIL;
    for (uint32_t attempt = 0u; attempt <= BOARD_SUPPORT_I2C_TRANSACTION_RETRY_MAX; ++attempt) {
        err = transferFn(ctx, addr7, tx, txLen, rx, rxLen);
        if (err == ESP_OK) {
            if (cfg->failureStreak) {
                *cfg->failureStreak = 0u;
            }
            if (cfg->warnLogCounter) {
                *cfg->warnLogCounter = 0u;
            }
            if (status) {
                status->Reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_NONE;
                status->RecoveryLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
                status->LastTransferErr = ESP_OK;
                status->FailureStreak = 0u;
                status->RecoverFailed = false;
                status->SclHigh = 1;
                status->SdaHigh = 1;
                status->BusGeneration = cfg->busGeneration ? *cfg->busGeneration : 0u;
            }
            return ESP_OK;
        }

        if (cfg->failureStreak) {
            (*cfg->failureStreak)++;
        }
        bool haveLines = (boardSupportI2cCheckLinesLocked(cfg, &sclHigh, &sdaHigh) == ESP_OK);
        boardSupportRecoveryStatusUpdateLines(status, haveLines, sclHigh, sdaHigh);
        uint32_t failureStreak = cfg->failureStreak ? *cfg->failureStreak : 1u;
        BoardSupportI2cRecoveryReason_t reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_SINGLE_NACK;
        BoardSupportI2cRecoveryLevel_t level = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
        if (!haveLines || !sclHigh || !sdaHigh) {
            reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_LINE_STUCK_LOW;
            level = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY;
        } else if (err == ESP_ERR_TIMEOUT) {
            reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_TIMEOUT;
            level = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
        } else if (allowStreakRecover && failureStreak >= BOARD_SUPPORT_I2C_FAILURE_STREAK_TRIGGER) {
            reason = BOARD_SUPPORT_I2C_RECOVERY_REASON_REPEATED_NACK;
            level = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_CONTROLLER_ONLY;
        }
        if (status) {
            status->LastTransferErr = err;
            status->FailureStreak = failureStreak;
            status->Reason = reason;
            status->RecoveryLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE;
        }

        if (boardSupportShouldEmitRateLimited(cfg->warnLogCounter, BOARD_SUPPORT_I2C_WARN_LOG_STRIDE)) {
            ESP_LOGW(TAG,
                     "%s %s failed: addr=0x%02X attempt=%lu err=%d class=%s streak=%lu "
                     "sclHigh=%d sdaHigh=%d",
                     cfg->name,
                     opName ? opName : "i2c",
                     addr7,
                     (unsigned long)attempt,
                     err,
                     boardSupportI2cErrClassName(err),
                     (unsigned long)(cfg->failureStreak ? *cfg->failureStreak : 0u),
                     haveLines ? (sclHigh ? 1 : 0) : -1,
                     haveLines ? (sdaHigh ? 1 : 0) : -1);
        }

        bool shouldRecover = (level != BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE);
        if (!shouldRecover || attempt >= BOARD_SUPPORT_I2C_TRANSACTION_RETRY_MAX) {
            break;
        }

        esp_err_t recoverErr = boardSupportI2cRecoverLocked(cfg, reason, level, NULL);
        if (recoverErr != ESP_OK) {
            if (status) {
                status->LastRecoverErr = recoverErr;
                status->RecoverFailed = true;
            }
            ESP_LOGE(TAG, "%s %s recover failed: %d", cfg->name, opName ? opName : "i2c", recoverErr);
            return recoverErr;
        }
        if (status) {
            status->LastRecoverErr = ESP_OK;
            status->RecoverFailed = false;
            status->FailureStreak = 0u;
        }
    }

    return err;
}

esp_err_t boardSupportInit(void)
{
    if (s_inited) {
        return ESP_OK;
    }

    if (!s_i2c0_mutex) {
        s_i2c0_mutex = xSemaphoreCreateMutex();
        if (!s_i2c0_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }
    if (boardSupportIsI2c1Enabled() && !s_i2c1_mutex) {
        s_i2c1_mutex = xSemaphoreCreateMutex();
        if (!s_i2c1_mutex) {
            return ESP_ERR_NO_MEM;
        }
    }

    esp_err_t err = boardSupportInitI2c((i2c_port_t)CONFIG_BOARD_I2C_PORT,
                                        CONFIG_BOARD_I2C_SDA_GPIO,
                                        CONFIG_BOARD_I2C_SCL_GPIO,
                                        CONFIG_BOARD_I2C0_FREQ_HZ,
                                        &s_i2c0_configured_freq_hz);
    if (err != ESP_OK) {
        return err;
    }
    s_i2c0_inited = true;
    s_i2c0_failure_streak = 0u;
    s_i2c0_warn_log_counter = 0u;
    s_i2c0_recovering = false;
    if (s_i2c0_bus_generation == 0u) {
        s_i2c0_bus_generation = 1u;
    }
    boardSupportRecoveryStatusReset(&s_i2c0_recovery_status, (i2c_port_t)CONFIG_BOARD_I2C_PORT);
    ESP_LOGI(TAG,
             "I2C0: port=%d SDA=%d SCL=%d requestedFreqHz=%u configuredFreqHz=%u note=configured_not_measured",
             CONFIG_BOARD_I2C_PORT,
             CONFIG_BOARD_I2C_SDA_GPIO,
             CONFIG_BOARD_I2C_SCL_GPIO,
             (unsigned)CONFIG_BOARD_I2C0_FREQ_HZ,
             (unsigned)s_i2c0_configured_freq_hz);

    if (boardSupportIsI2c1Enabled()) {
        if (CONFIG_BOARD_I2C1_PORT == CONFIG_BOARD_I2C_PORT) {
            ESP_LOGE(TAG, "I2C1 port must differ from I2C0 port");
            i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C_PORT);
            s_i2c0_inited = false;
            return ESP_ERR_INVALID_ARG;
        }

        err = boardSupportInitI2c((i2c_port_t)CONFIG_BOARD_I2C1_PORT,
                                  CONFIG_BOARD_I2C1_SDA_GPIO,
                                  CONFIG_BOARD_I2C1_SCL_GPIO,
                                  CONFIG_BOARD_I2C1_FREQ_HZ,
                                  &s_i2c1_configured_freq_hz);
        if (err != ESP_OK) {
            i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C_PORT);
            s_i2c0_inited = false;
            return err;
        }
        s_i2c1_inited = true;
        s_i2c1_failure_streak = 0u;
        s_i2c1_warn_log_counter = 0u;
        s_i2c1_recovering = false;
        if (s_i2c1_bus_generation == 0u) {
            s_i2c1_bus_generation = 1u;
        }
        boardSupportRecoveryStatusReset(&s_i2c1_recovery_status, (i2c_port_t)CONFIG_BOARD_I2C1_PORT);
        ESP_LOGI(TAG,
                 "I2C1: port=%d SDA=%d SCL=%d requestedFreqHz=%u configuredFreqHz=%u note=configured_not_measured",
                 CONFIG_BOARD_I2C1_PORT,
                 CONFIG_BOARD_I2C1_SDA_GPIO,
                 CONFIG_BOARD_I2C1_SCL_GPIO,
                 (unsigned)CONFIG_BOARD_I2C1_FREQ_HZ,
                 (unsigned)s_i2c1_configured_freq_hz);
    }

    s_inited = true;
    return ESP_OK;
}

esp_err_t boardSupportDeinit(void)
{
    if (!s_inited) {
        return ESP_OK;
    }

    esp_err_t err = ESP_OK;
    if (s_i2c1_inited) {
        esp_err_t deinit_err = i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C1_PORT);
        if (deinit_err != ESP_OK) {
            err = deinit_err;
        }
        s_i2c1_inited = false;
    }

    if (s_i2c0_inited) {
        esp_err_t deinit_err = i2c_driver_delete((i2c_port_t)CONFIG_BOARD_I2C_PORT);
        if (deinit_err != ESP_OK) {
            err = deinit_err;
        }
        s_i2c0_inited = false;
    }

    s_i2c0_failure_streak = 0u;
    s_i2c1_failure_streak = 0u;
    s_i2c0_warn_log_counter = 0u;
    s_i2c1_warn_log_counter = 0u;
    s_i2c0_recovering = false;
    s_i2c1_recovering = false;
    s_i2c0_recovery_status.Valid = false;
    s_i2c1_recovery_status.Valid = false;
    if (s_i2c1_mutex) {
        vSemaphoreDelete(s_i2c1_mutex);
        s_i2c1_mutex = NULL;
    }
    if (s_i2c0_mutex) {
        vSemaphoreDelete(s_i2c0_mutex);
        s_i2c0_mutex = NULL;
    }
    s_inited = false;
    return err;
}

bool boardSupportIsI2c1Enabled(void)
{
    return boardSupportI2c1Configured();
}

const BoardSupportI2cCtx_t* boardSupportGetI2cCtx(void)
{
    return &s_i2c0_ctx;
}

const BoardSupportI2cCtx_t* boardSupportGetI2c1Ctx(void)
{
    if (!boardSupportIsI2c1Enabled()) {
        return NULL;
    }
    return &s_i2c1_ctx;
}

bool boardSupportGetI2cBusInfo(bool secondary, BoardSupportI2cBusInfo_t *outInfo)
{
    if (!outInfo) {
        return false;
    }

    if (secondary) {
        *outInfo = (BoardSupportI2cBusInfo_t){
            .Enabled = boardSupportIsI2c1Enabled(),
            .Port = (i2c_port_t)CONFIG_BOARD_I2C1_PORT,
            .SdaGpio = CONFIG_BOARD_I2C1_SDA_GPIO,
            .SclGpio = CONFIG_BOARD_I2C1_SCL_GPIO,
            .FrequencyHz = s_i2c1_inited ? s_i2c1_configured_freq_hz : CONFIG_BOARD_I2C1_FREQ_HZ,
        };
        return true;
    }

    *outInfo = (BoardSupportI2cBusInfo_t){
        .Enabled = boardSupportI2cPinsValid(CONFIG_BOARD_I2C_SDA_GPIO, CONFIG_BOARD_I2C_SCL_GPIO) &&
                   (CONFIG_BOARD_I2C_PORT >= 0),
        .Port = (i2c_port_t)CONFIG_BOARD_I2C_PORT,
        .SdaGpio = CONFIG_BOARD_I2C_SDA_GPIO,
        .SclGpio = CONFIG_BOARD_I2C_SCL_GPIO,
        .FrequencyHz = s_i2c0_inited ? s_i2c0_configured_freq_hz : CONFIG_BOARD_I2C0_FREQ_HZ,
    };
    return true;
}

esp_err_t boardSupportI2cCheckLines(const BoardSupportI2cCtx_t *i2cCtx, bool *outSclHigh, bool *outSdaHigh)
{
    if (!i2cCtx || !outSclHigh || !outSdaHigh) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cCheckLinesLocked(&cfg, outSclHigh, outSdaHigh);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cReinit(const BoardSupportI2cCtx_t *i2cCtx, const char *reason)
{
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cReinitLocked(&cfg, reason);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cRecoverBus(const BoardSupportI2cCtx_t *i2cCtx, const char *reason)
{
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cRecoverBusLocked(&cfg, reason, NULL);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cRecover(const BoardSupportI2cCtx_t *i2cCtx,
                                 BoardSupportI2cRecoveryReason_t reason,
                                 BoardSupportI2cRecoveryLevel_t requestedLevel,
                                 uint8_t addr7,
                                 uint32_t failureStreakHint,
                                 BoardSupportI2cRecoveryResult_t *outResult)
{
    (void)addr7;
    (void)failureStreakHint;
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cRecoverLocked(&cfg, reason, requestedLevel, outResult);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cGetPortGeneration(const BoardSupportI2cCtx_t *i2cCtx, uint32_t *outGeneration)
{
    if (!i2cCtx || !outGeneration) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.busGeneration) {
        return ESP_ERR_INVALID_ARG;
    }
    *outGeneration = *cfg.busGeneration;
    return ESP_OK;
}

esp_err_t boardSupportI2cGetBusGeneration(void *userCtx, uint32_t *outGeneration)
{
    return boardSupportI2cGetPortGeneration((const BoardSupportI2cCtx_t *)userCtx, outGeneration);
}

esp_err_t boardSupportI2cManualRecover(const BoardSupportI2cCtx_t *i2cCtx,
                                       uint8_t addr7,
                                       const char *reason,
                                       uint32_t failureStreakHint)
{
    if (!i2cCtx) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }

    bool sclHigh = true;
    bool sdaHigh = true;
    esp_err_t precheckErr = boardSupportI2cCheckLinesLocked(&cfg, &sclHigh, &sdaHigh);
    uint32_t failureStreak = failureStreakHint;
    if (cfg.failureStreak && *cfg.failureStreak > failureStreak) {
        failureStreak = *cfg.failureStreak;
    }

    printf("DBGS5D5_I2C,stage=precheck,port=%d,sda=%d,scl=%d,addr=0x%02X,reason=%s,failureStreak=%lu,sclHigh=%d,sdaHigh=%d,err=%ld\n",
           (int)cfg.port,
           cfg.sdaGpio,
           cfg.sclGpio,
           addr7,
           reason ? reason : "manual_recover",
           (unsigned long)failureStreak,
           (precheckErr == ESP_OK) ? (sclHigh ? 1 : 0) : -1,
           (precheckErr == ESP_OK) ? (sdaHigh ? 1 : 0) : -1,
           (long)precheckErr);

    BoardSupportI2cRecoveryReason_t reasonEnum = boardSupportI2cRecoveryReasonFromLabel(
        reason,
        BOARD_SUPPORT_I2C_RECOVERY_REASON_MANUAL_USER_REQUEST);
    BoardSupportI2cRecoveryLevel_t requestedLevel = boardSupportI2cDefaultLevelForReason(reasonEnum);
    if (precheckErr == ESP_OK && (!sclHigh || !sdaHigh) &&
        requestedLevel < BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY) {
        requestedLevel = BOARD_SUPPORT_I2C_RECOVERY_LEVEL_LINE_RECOVERY;
    }

    printf("DBGS5D5_I2C,stage=manual_recover_begin,port=%d,sda=%d,scl=%d,addr=0x%02X,reason=%s,reasonEnum=%s,"
           "requestedLevel=%s,failureStreak=%lu\n",
           (int)cfg.port,
           cfg.sdaGpio,
           cfg.sclGpio,
           addr7,
           reason ? reason : "manual_recover",
           boardSupportI2cRecoverReasonToString(reasonEnum),
           boardSupportI2cRecoveryLevelName(requestedLevel),
           (unsigned long)failureStreak);

    BoardSupportI2cRecoveryResult_t recoverResult = {0};
    esp_err_t recoverErr = boardSupportI2cRecoverLocked(&cfg, reasonEnum, requestedLevel, &recoverResult);
    BoardSupportI2cRecoveryStatus_t *status = boardSupportRecoveryStatusForCfg(&cfg);
    int reinitSucceeded = (status && status->ReinitSucceeded) ? 1 : 0;
    esp_err_t reinitErr = status ? status->LastReinitErr : ESP_OK;

    if (recoverErr == ESP_OK &&
        recoverResult.AttemptedLevel != BOARD_SUPPORT_I2C_RECOVERY_LEVEL_NONE &&
        BOARD_SUPPORT_I2C_POST_RECOVER_STABILIZE_MS > 0u) {
        printf("DBGS5D5_I2C,stage=post_recover_stabilize,port=%d,delayMs=%u\n",
               (int)cfg.port,
               (unsigned)BOARD_SUPPORT_I2C_POST_RECOVER_STABILIZE_MS);
        vTaskDelay(pdMS_TO_TICKS(BOARD_SUPPORT_I2C_POST_RECOVER_STABILIZE_MS));
    }

    printf("DBGS5D5_I2C,stage=manual_recover_end,port=%d,sda=%d,scl=%d,addr=0x%02X,reason=%s,failureStreak=%lu,"
           "level=%s,pulsesIssued=%lu,deleteAttempted=%u,deleteErr=%ld,driverReinitResult=%d,reinitErr=%ld,"
           "busGeneration=%lu,err=%ld\n",
           (int)cfg.port,
           cfg.sdaGpio,
           cfg.sclGpio,
           addr7,
           reason ? reason : "manual_recover",
           (unsigned long)failureStreak,
           boardSupportI2cRecoveryLevelName(recoverResult.AttemptedLevel),
           (unsigned long)recoverResult.PulsesIssued,
           recoverResult.DeleteAttempted ? 1u : 0u,
           (long)recoverResult.DeleteErr,
           reinitSucceeded,
           (long)reinitErr,
           (unsigned long)recoverResult.BusGenerationAfter,
           (long)recoverErr);

    boardSupportI2cUnlockPort(&cfg);
    return recoverErr;
}

bool boardSupportI2cGetLastRecoveryStatus(const BoardSupportI2cCtx_t *i2cCtx,
                                          BoardSupportI2cRecoveryStatus_t *outStatus)
{
    if (!i2cCtx || !outStatus) {
        return false;
    }
    if (!s_inited) {
        return false;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled) {
        return false;
    }

    BoardSupportI2cRecoveryStatus_t *status = boardSupportRecoveryStatusForCfg(&cfg);
    if (!status || !status->Valid) {
        return false;
    }
    *outStatus = *status;
    return true;
}

esp_err_t boardSupportI2cWriteRead(void* userCtx,
                                   uint8_t addr7,
                                   const uint8_t* tx,
                                   size_t txLen,
                                   uint8_t* rx,
                                   size_t rxLen)
{
    if (!userCtx || !tx || txLen == 0 || !rx || rxLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    const BoardSupportI2cCtx_t* ctx = (const BoardSupportI2cCtx_t*)userCtx;
    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(ctx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag || !(*cfg.initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cRunWithRecoveryLocked(ctx,
                                                         &cfg,
                                                         addr7,
                                                         tx,
                                                         txLen,
                                                         rx,
                                                         rxLen,
                                                         boardSupportI2cTransferWriteRead,
                                                         "write_read",
                                                         true);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cWrite(void* userCtx,
                               uint8_t addr7,
                               const uint8_t* tx,
                               size_t txLen)
{
    if (!userCtx || !tx || txLen == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    const BoardSupportI2cCtx_t* ctx = (const BoardSupportI2cCtx_t*)userCtx;
    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(ctx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag || !(*cfg.initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }
    esp_err_t err = boardSupportI2cRunWithRecoveryLocked(ctx,
                                                         &cfg,
                                                         addr7,
                                                         tx,
                                                         txLen,
                                                         NULL,
                                                         0u,
                                                         boardSupportI2cTransferWrite,
                                                         "write",
                                                         true);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

esp_err_t boardSupportI2cProbeAddress(const BoardSupportI2cCtx_t *i2cCtx, uint8_t addr7)
{
    if (!i2cCtx || addr7 > 0x7Fu) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_inited) {
        return ESP_ERR_INVALID_STATE;
    }

    boardSupportI2cPortConfig_t cfg = {0};
    if (!boardSupportResolveI2cPort(i2cCtx, &cfg) || !cfg.valid || !cfg.enabled || !cfg.initedFlag || !(*cfg.initedFlag)) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t lockErr = boardSupportI2cLockPort(&cfg);
    if (lockErr != ESP_OK) {
        return lockErr;
    }

    /*
     * Probe NACKs are expected for absent addresses during diagnostics.
     * Keep pre-op stuck-line recovery enabled, but disable streak-based
     * recovery escalation for plain probe failures.
     */
    esp_err_t err = boardSupportI2cRunWithRecoveryLocked(i2cCtx,
                                                         &cfg,
                                                         addr7,
                                                         NULL,
                                                         0u,
                                                         NULL,
                                                         0u,
                                                         boardSupportI2cTransferProbe,
                                                         "probe",
                                                         false);
    boardSupportI2cUnlockPort(&cfg);
    return err;
}

// IMPORTANT: Do not define app_main() here.
